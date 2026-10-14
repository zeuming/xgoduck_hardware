"""X5-local passive calibration using the original four-step and joint wizards.

Import and construction are CPU-only. An explicit purpose-bound recovery opens
one guarded local reader; this module never sends motor commands. Offline servo
fault bits remain visible in IMU-only feedback and never grant joint access.
"""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import re
import secrets
import stat
import threading
import time

import feedback_calibration_server as host

ALL_MASK = 32767
DIAGNOSTIC_STARTUP_MESSAGE = '已明确选择仅启动诊断网页；尚未执行完整硬件连接检查，尚未启动舵机操作和采集。'
PATHS = frozenset(('/api/imu-wizard', '/api/joint-calibration', '/api/joint-zero',
                   '/api/end-feedback', '/api/recover-feedback', '/api/recheck-feedback',
                   '/api/check-connection', '/api/servo-maintenance'))


def _imu_display_body(value):
    """Project a fresh IMU-only frame for display; grant no hardware capability."""
    session, feedback, body = value['session'], value['feedback'], value['body_imu']
    imu = value['calibration']['imu']
    if (session.get('status') != 'active' or session.get('collection_scope') != 'imu'
            or type(feedback) is not dict or feedback.get('scope') != 'imu'
            or feedback.get('imu_ok') is not True or type(body) is not dict
            or imu.get('axes_confirmed') is not True or imu.get('calibrated') is not True
            or not any(axes == imu.get('axes') for axes, _ in host.imu_wizard.wizard_rotations())):
        return None
    for name in ('accel_ms2', 'gyro_dps'):
        vector = body.get(name)
        if (type(vector) is not list or len(vector) != 3 or any(
                type(component) not in (int, float) or not math.isfinite(component) for component in vector)):
            return None
    for name in ('sample_us', 'imu_us'):
        if type(feedback.get(name)) is not int or not 0 <= feedback[name] <= 0xffffffff:
            return None
    age = session.get('received_age_ms')
    if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
        return None
    delta = (feedback['sample_us'] - feedback['imu_us']) & 0xffffffff
    if delta >= 0x80000000:
        delta -= 0x100000000
    if abs(delta) > 500000:
        return None
    imu_age_ms = max(0, delta) / 1000.
    if age + imu_age_ms >= host.FRESH * 1000:
        return None
    ax, ay, az = body['accel_ms2']
    norm = math.hypot(ax, ay, az)
    if not math.isfinite(norm) or norm <= 1e-6:
        return None
    # Specific force is positive +Z upright. Motion uses world gravity in
    # body axes, -a/|a|; preserve legacy top-level specific-force angles.
    gx, gy, gz = -ax / norm, -ay / norm, -az / norm
    attitude = dict(tilt_deg=math.degrees(math.acos(max(-1., min(1., -gz)))),
                    pitch_deg=math.degrees(math.atan2(gx, -gz)),
                    roll_deg=math.degrees(math.atan2(-gy, -gz)), protocol=1)
    return dict(body, attitude=attitude, attitude_protocol=1,
                source='calibration-feedback', collection_scope='imu', imu_age_ms=imu_age_ms,
                sample_us=feedback['sample_us'], imu_us=feedback['imu_us'])


class NativeCalibrationSession(host.CalibrationSession):
    """Shared calibration algorithms with an independent IMU-only gate."""

    def __init__(self, *args, native_host='192.168.1.11', **kwargs):
        self.native_host = native_host
        super().__init__(*args, **kwargs)
        self.serial_discovery_allowed = True
        self.transport_kind = 'local'

    def start_offline(self, actual_reason):
        """Publish a never-started fault page without sending END or opening I/O."""
        if (type(actual_reason) is not str or not actual_reason.strip()
                or len(actual_reason.encode('utf-8')) > 16384):
            raise ValueError('X5 离线启动需要有界的实际检查失败原因')
        with self.lock:
            if (self.status != 'waiting' or self.process is not None
                    or self.connection_done.is_set() or self._paused_without_attempt
                    or self._reader_preflight_failure_generation is not None
                    or self.recovery_pending or self.source_transmitting
                    or self.feedback is not None or self.ready is not None
                    or self.closed_proof is not None or self.end_request_attempted):
                raise ValueError('离线启动仅可用于从未启动的校准会话')
            # The original paused initializer is the sole provenance for
            # process=None meaning that this session never attempted hardware.
            self.start_initial(paused=True)
            self.status = 'fault'
            self.error, self.error_details = host.fault_text(actual_reason)
            self.recovery_recheck = host.recovery_recheck_error(actual_reason) or dict(
                category='trust', technical_reason=actual_reason,
                message='设备连接未通过启动检查，需要重新核对板端状态；已保存校准保留。')
            self.message = '网页已就绪；设备连接未通过检查，采集尚未启动。可重新检查连接，已保存校准保留。'
            self.connection_check.update(phase='failed', error=self.error,
                technical_reason=actual_reason, message=self.message)

    def _launch_feedback_attempt(self, purpose=None, **kwargs):
        # A general reader cannot turn absent servo replies into safe feedback.
        return super()._launch_feedback_attempt(purpose or 'joints', **kwargs)

    def _write_calibration(self, config):
        zero_save = self.joint_zero['stage'] == 'ready' and self.zero_collection is not None
        if zero_save or getattr(self, '_native_joint_save', False):
            # Both callers have validated the current torque-off sample and
            # proposed configuration. Drain the reader before persistent I/O;
            # END alone is insufficient while its consumer waits on our lock.
            if self.status != 'active' or self.end_reason is not None:
                raise OSError('关节保存前反馈会话已变化，未写入校准')
            draft = self.joint_zero if zero_save else self.joint_wizard.value
            stage, busy = draft['stage'], draft['busy']
            draft.update(stage='saving', busy=True)
            self.status, self.feedback = 'stopping', None
            self.end_reason, self.end_requested_at = 'manual', self.clock()
            self.message = '正在结束只读反馈；正常关闭后保存关节，保存后请复查关节反馈继续采集。'
            try:
                self._send_end_once()
                if not self.end_request_sent:
                    raise OSError('关节保存前结束反馈请求未送达，校准未写入：' + str(self.end_delivery_error))
                self._await_joint_save_reader_closed()
            except OSError as error:
                draft.update(stage=stage, busy=busy)
                self.fault(str(error))
                if zero_save:
                    self._fail_joint_zero(str(error))
                else:
                    self.joint_wizard.fail(str(error))
                raise
            draft.update(stage=stage, busy=busy)
        return super()._write_calibration(config)

    def _await_joint_save_reader_closed(self):
        # Condition.wait releases every recursion level of this RLock, so the
        # consumer can accept the real close record and process-exit proof.
        # Keep the existing two-second shutdown deadline; never fabricate proof
        # or write the proposed calibration after a failed shutdown.
        condition = threading.Condition(self.lock)
        wall_deadline = time.monotonic() + 2.
        while True:
            proof = self.closed_proof or {}
            if (self.status == 'closed' and self.connection_done.is_set()
                    and self.process is not None and self.process.poll() == 0
                    and proof.get('kind') == 'closed' and proof.get('generation') == self.generation
                    and proof.get('normal') is True and proof.get('serial_closed') is True):
                return
            remaining = min(wall_deadline - time.monotonic(),
                            2. - (self.clock() - self.end_requested_at))
            if self.status == 'fault' or remaining <= 0:
                raise OSError(self.error or '关节保存前未收到正常关闭证明，校准未写入')
            condition.wait(min(.02, remaining))

    def joint_action(self, body):
        with self.lock:
            if getattr(self, '_native_joint_save', False):
                raise ValueError('关节正在保存，请等待本次结果')
            self._native_joint_save = type(body) is dict and body.get('action') in ('save', 'save-direction')
            try:
                return super().joint_action(body)
            finally:
                self._native_joint_save = False

    def _scoped_online_mask(self, purpose):
        # IMU scope validates disarmed, torque-off feedback in _validate_state;
        # servo availability cannot authorize joints or prevent IMU-only reads.
        return None if purpose == 'imu' else super()._scoped_online_mask(purpose)

    def motion_reserved(self):
        # A status page asks this gate several times. Ownership is a lifecycle
        # fact; each gate must not request another full controller snapshot.
        reserved = getattr(self.motion, 'connection_reserved', None)
        if callable(reserved):
            result = reserved()
            if type(result) is not bool:
                raise ValueError('运控连接占用状态无效')
            return result
        return super().motion_reserved()

    def imu_read_blocked_reason(self):
        """Read-only IMU eligibility; servo holds retain their own authority."""
        if self.motion_reserved():
            return '运控正在使用连接；先停止并断开运控，再读取 IMU。'
        if self.maintenance is not None and self.maintenance.owns_connection:
            return '舵机维护仍占用连接，请先退出维护。'
        if self.connection_checker is None or self.recovery_launcher is None:
            return '当前服务尚未配置独立 IMU 读取入口。'
        if self.recovery_pending or self.connection_check['phase'] == 'checking':
            return '正在检查连接，请等待本次结果。'
        if self.status not in ('fault', 'closed') or not self.connection_done.is_set() or self.source_transmitting:
            return '当前采集尚未关闭，请结束采集并等待关闭确认。'
        if self.collection is not None or self.joint_wizard.active() or self.joint_zero['busy']:
            return '请先完成或取消当前未保存的采样，再读取 IMU。'
        if self.process is None:
            if not (self._paused_without_attempt or self._reader_preflight_failure_generation == self.generation):
                return '上次本地连接退出尚未得到确认，不能读取 IMU。'
        else:
            proof = self.closed_proof or {}
            if (self.process.poll() is None or proof.get('kind') != 'closed'
                    or type(proof.get('generation')) is not int or proof['generation'] != self.generation
                    or proof.get('serial_closed') is not True):
                return '原连接缺少匹配的串口关闭与进程退出证明，不能读取 IMU。'
        # No old fault is cleared here. The fresh metadata gate still checks
        # board/USB identity, complete kernel history and exclusive UART, then
        # IMU scope requires disarmed, torque-off, advancing sensor feedback.
        return None

    def _maintenance_can_reconcile_unopened_trust(self):
        hold=self.recovery_recheck
        return (type(hold) is dict and hold.get('category')=='trust'
            and self.maintenance_fault_hold and not self.maintenance_mapping_changed
            and not self.motion_reserved() and not self.recovery_pending
            and self.status in ('closed','fault') and not self.source_transmitting
            and self.connection_done.is_set() and self.process is None
            and (self._paused_without_attempt or self._reader_preflight_failure_generation==self.generation)
            and self.collection is None and not self.joint_wizard.active() and not self.joint_zero['busy'])

    def maintenance_review_blocked_reason(self):
        with self.lock:
            if self._maintenance_can_reconcile_unopened_trust():
                return None
            return super().maintenance_review_blocked_reason()

    def _maintenance_reviewed(self, rows, proof):
        with self.lock:
            reconcile=self._maintenance_can_reconcile_unopened_trust()
            previous=copy.deepcopy(self.recovery_recheck)
            # The transport's fresh identity/kernel gate precedes all reads;
            # the original callback still requires all 15 healthy torque-off
            # rows AND actual UART closure and process exit before unlocking.
            super()._maintenance_reviewed(rows,proof)
            if reconcile:
                host.private_new(self.directory/('maintenance-trust-reviewed-'+secrets.token_hex(8)+'.json'),
                    json.dumps(dict(previous_hold=previous,rows=rows,close_proof=proof,
                        calibration_preserved=True,motion_resumed=False),ensure_ascii=False)+'\n')
                self.recovery_recheck=None
                self.status,self.error,self.error_details='closed',None,None
                self.message='连接与 15 个舵机复查通过，扭矩全部关闭；可以重新连接运控。'
                self.connection_check.update(phase='passed',error=None,technical_reason=None,message=self.message)

    def _validate_state(self, state):
        if self.feedback_scope != 'imu':
            return super()._validate_state(state)
        if type(state) is not dict:
            raise ValueError('IMU 反馈字段无效')
        for key in ('sample_us', 'imu_us', 'arm_cookie'):
            if type(state.get(key)) is not int or not 0 <= state[key] <= 0xffffffff:
                raise ValueError('IMU 反馈计时无效')
        if (any(type(state.get(key)) is not int or not 0 <= state[key] <= ALL_MASK
                for key in ('online_mask', 'fault_mask'))
                or type(state.get('torque_mask')) is not int or state['torque_mask'] != 0
                or state.get('armed') is not False or state['arm_cookie'] != 0
                or state.get('imu_ok') is not True or type(state.get('last_error')) is not int
                or state['last_error'] != 0):
            raise ValueError('IMU 专用采集要求反馈字段有效、未使能、扭矩标志关闭且 IMU 有效')
        for key in ('accel_ms2', 'gyro_dps'):
            values = state.get(key)
            if type(values) is not list or len(values) != 3 or any(
                    type(value) not in (int, float) or not math.isfinite(value) for value in values):
                raise ValueError('IMU 向量无效')
        positions = state.get('positions')
        if (type(positions) is not list or len(positions) != 15 or any(
                type(value) is not int or not 0 <= value <= 4095 for value in positions)):
            raise ValueError('编码器字段无效；IMU 采集不会将离线值当作关节反馈')

    def accept(self, message):
        if message.get('kind') == 'state' and self.feedback_scope == 'imu':
            if message.get('collection_scope') != 'imu':
                raise ValueError('反馈未绑定 IMU 专用采集用途')
        with self.lock:
            result = super().accept(message)
            state = message.get('state')
            if (message.get('kind') == 'state' and self.status == 'active'
                    and self.feedback is not None and self.feedback == state):
                point = (self.generation, state['sample_us'], state['imu_us'])
                previous = getattr(self, '_cadence_last', None)
                if previous is None or previous[0] != point[0]:
                    self._reader_cadence = dict(frames=1, sample_span_ms=0., imu_span_ms=0.,
                        max_sample_gap_ms=0., max_imu_gap_ms=0., gaps_at_least_100ms=0)
                else:
                    gaps = [((point[i] - previous[i]) & 0xffffffff) / 1000 for i in (1, 2)]
                    if all(0 < gap < 0x80000000 / 1000 for gap in gaps):
                        c = self._reader_cadence
                        c['frames'] += 1
                        for label, gap in zip(('sample', 'imu'), gaps):
                            c[label + '_span_ms'] += gap
                            c['max_' + label + '_gap_ms'] = max(c['max_' + label + '_gap_ms'], gap)
                        c['gaps_at_least_100ms'] += int(max(gaps) >= 100)
                self._cadence_last = point
            return result

    def _require_imu(self, generation):
        if type(generation) is not int or generation != self.generation:
            raise ValueError('校准会话已变化，请刷新页面')
        if self.feedback_scope != 'imu':
            raise ValueError('请先结束关节采集，再进入 IMU 专用采集')
        if (self.status != 'active' or self.feedback is None or self.received_at is None
                or not 0 <= self.clock() - self.received_at < host.FRESH):
            raise ValueError('真机 IMU 反馈不可用或已过期，采样已停止')
        self._validate_state(self.feedback)
        age = (self.feedback['sample_us'] - self.feedback['imu_us']) & 0xffffffff
        if 500000 < age < 0x100000000 - 500000:
            raise ValueError('IMU 采样已过期或计时不一致')
        return self.feedback

    def _require_joint_feedback(self):
        if self.feedback_scope != 'joints':
            raise ValueError('IMU 专用采集不提供在线关节反馈，请结束后复查关节')
        return super()._require_joint_feedback()

    @staticmethod
    def _validate_local_quiescence(proof, generation):
        path = proof.get('serial_path') if type(proof) is dict else None
        if (type(proof) is not dict or proof.get('kind') != 'quiescent'
                or type(proof.get('generation')) is not int or proof['generation'] != generation
                or type(path) is not str or not re.fullmatch(r'/dev/ttyUSB(0|[1-9][0-9]{0,2})', path)
                or int(path.removeprefix('/dev/ttyUSB')) > 255
                or proof.get('serial_holders') != [] or proof.get('identity_checked') is not True
                or proof.get('guards_passed') is not True
                or type(proof.get('observation_count')) is not int or proof['observation_count'] != 2
                or type(proof.get('serial_open_attempts')) is not int or proof['serial_open_attempts'] != 0
                or type(proof.get('serial_bytes_written')) is not int or proof['serial_bytes_written'] != 0
                or proof.get('board_files_written') is not False or proof.get('automatic_retry') is not False
                or proof.get('transport') != 'local' or proof.get('ssh_started') is not False
                or type(proof.get('process_exit_code')) is not int or proof['process_exit_code'] != 0
                or proof.get('process_exited') is not True):
            raise ValueError('X5 本地连接检查证明无效，采集保持停止')

    def connection_check_blocked_reason(self):
        """Native preflight failures need no fictional SSH closure record."""
        if self.motion_reserved():
            return '运控正在使用连接；先在「运控模型」停止并断开。'
        if self.maintenance_mapping_changed:
            return '舵机编号映射待核对，连接检查不能解除编号锁。'
        if self.maintenance_fault_hold:
            return '维护故障尚未核对，连接检查不能解除维护锁。'
        if self.maintenance is not None and self.maintenance.owns_connection:
            return '舵机维护仍占用连接，请先退出维护。'
        if self.connection_checker is None or self.recovery_launcher is None:
            return '当前服务尚未配置 X5 本地连接检查入口。'
        hold = self.recovery_recheck
        if hold is None or hold.get('category') not in ('closure', 'trust'):
            return hold['message'] if hold else '当前无需检查旧连接，请使用采集入口。'
        if self.recovery_pending or self.connection_check['phase'] == 'checking':
            return '正在检查连接，请等待本次结果。'
        if self.status not in ('fault', 'closed') or not self.connection_done.is_set() or self.source_transmitting:
            return '上次采集进程尚在收尾，请稍候再检查连接。'
        if self.process is not None:
            if self.process.poll() is None:
                return '上次 X5 本地采集进程仍在运行，不能检查新连接。'
        elif not (self._paused_without_attempt
                  or self._reader_preflight_failure_generation == self.generation):
            return '上次本地连接退出尚未得到确认，不能检查新连接。'
        return None

    def check_connection_action(self, body):
        """Reconcile locally after a fault, retaining actual UART-close evidence."""
        with self.lock:
            if (type(body) is not dict or set(body) != {'generation', 'purpose'}
                    or type(body.get('generation')) is not int or body['generation'] != self.generation
                    or body.get('purpose') not in ('imu', 'joints')):
                raise ValueError('连接检查请求无效或页面会话已变化，请刷新')
            blocked = (self.imu_read_blocked_reason() if body['purpose'] == 'imu'
                       else self.connection_check_blocked_reason())
            if blocked is not None:
                raise ValueError(blocked)
            generation, purpose = self.generation, body['purpose']
            directory = self.directory / ('connection-check-' + secrets.token_hex(8))
            directory.mkdir(mode=0o700, exist_ok=False)
            host.private_new(directory / 'request.json', json.dumps(dict(
                generation=generation, purpose=purpose, previous_hold=self.recovery_recheck,
                previous_closed_proof=self.closed_proof, calibration_preserved=True,
                transport='local', explicit_user_request=True), ensure_ascii=False) + '\n')
            self.connection_check.update(phase='checking', error=None, technical_reason=None,
                message='正在检查 X5 身份、日志及串口占用；检查不会开启扭矩。')
            self.recovery_pending = True

            def check():
                try:
                    proof = self.connection_checker(generation, directory)
                    self._validate_local_quiescence(proof, generation)
                    with self.lock:
                        if self.generation != generation:
                            raise ValueError('检查过程中页面会话已变化，不能继续采集')
                        self.recovery_pending = False
                        self.connection_check['phase'] = 'passed'
                        blocked = (self.imu_read_blocked_reason() if purpose == 'imu'
                                   else self.connection_check_blocked_reason())
                        if blocked is not None:
                            raise ValueError(blocked)
                        host.private_new(directory / 'reconciled.json', json.dumps(dict(
                            generation=generation, purpose=purpose, proof=proof,
                            previous_hold=self.recovery_recheck, uart_closed_report_fabricated=False,
                            calibration_preserved=True), ensure_ascii=False) + '\n')
                        self.connection_check.update(error=None, technical_reason=None,
                            message='X5 本地串口已释放，正在复查实时反馈。')
                        self._launch_feedback_attempt(purpose, connection_reconciled=proof)
                except BaseException as error:
                    with self.lock:
                        if self.generation == generation:
                            self.recovery_pending = False
                            newer = host.recovery_recheck_error(error)
                            if newer is not None and newer.get('category') == 'trust':
                                self.recovery_recheck = newer
                            self.connection_check.update(phase='failed', error=str(error),
                                technical_reason=str(error), message='检查未通过，采集保持停止：' + str(error))
                            try:
                                host.private_new(directory / 'failed.json', json.dumps(dict(
                                    generation=generation, error=str(error), calibration_preserved=True,
                                    transport='local'), ensure_ascii=False) + '\n')
                            except OSError:
                                self.connection_check['message'] = '检查证据保存失败，采集保持停止。'
            try:
                threading.Thread(target=check, daemon=True).start()
            except RuntimeError as error:
                self.recovery_pending = False
                self.connection_check.update(phase='failed', error=str(error),
                    message='X5 本地连接检查未启动。')
            return self.snapshot()

    def snapshot(self, *, compact=True, include_motion=False):
        with self.lock:
            # Host action methods return self.snapshot() while still holding
            # the feedback lock. Their native caller discards that return and
            # builds the response itself. Never copy full kernel/motion history
            # here and stall the live consumer during a capture. The service's
            # explicit full-status projection still requests both below.
            # NativeWorkbench injects the single motion projection. Building
            # another one here repeatedly takes the live controller lock and
            # immediately discards the result in the outer workbench.
            value = super().snapshot(include_motion=include_motion, compact=compact)
            value['session']['reader_cadence'] = copy.deepcopy(getattr(self, '_reader_cadence', None))
            scope = self.feedback_scope
            imu_blocked = self.imu_read_blocked_reason()
        value['connection_target'] = dict(kind='wifi', host=self.native_host,
            page_url='http://' + self.native_host + ':8767/', native=True)
        value['session']['collection_scope'] = scope
        value['session']['transport'] = 'local'
        value['session'].update(imu_read_allowed=imu_blocked is None,
            imu_read_blocked_reason=imu_blocked, imu_read_requires_connection_check=True)
        value['capabilities'] = dict(imu_calibration=True, joint_calibration=True,
            mechanical_zero=True, imu_only=scope == 'imu',
            joint_feedback=scope == 'joints' and value['feedback'] is not None)
        if value['feedback'] is not None:
            value['feedback']['scope'] = scope
            value['feedback']['joint_feedback_valid'] = scope == 'joints'
        if scope == 'imu':
            value['body_imu'] = _imu_display_body(value)
        # The original algorithms share messages with the Mac application.
        # Native saves persist on X5 through the injected commit owner.
        def localize(item):
            if type(item) is dict:
                return {key: localize(content) for key, content in item.items()}
            if type(item) is list:
                return [localize(content) for content in item]
            if type(item) is str:
                return item.replace('笔记本', 'X5')
            return item
        return localize(value)


class NativeCalibrationService:
    """Adapter for original HTTP schemas and the shared native UART owner gate."""

    def __init__(self, config, directory, generation, *, reader_launcher, connection_checker,
                 before_acquire=None, calibration_writer=None, zero_provenance=None,
                 host='192.168.1.11', clock=time.monotonic, startup_fault=None):
        if (not callable(reader_launcher) or not callable(connection_checker)
                or before_acquire is not None and not callable(before_acquire)
                or calibration_writer is not None and not callable(calibration_writer)):
            raise ValueError('X5 本地采集接口无效')
        self.before_acquire = before_acquire or (lambda purpose: None)
        directory = Path(directory)
        info = directory.lstat()
        if (not directory.is_absolute() or directory.resolve() != directory
                or not stat.S_ISDIR(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o700
                or info.st_uid != os.getuid()):
            raise ValueError('X5 校准存储目录不可用')
        self.session = NativeCalibrationSession(config, directory, generation, clock,
            recovery_launcher=reader_launcher, connection_checker=connection_checker,
            calibration_writer=calibration_writer, zero_provenance=zero_provenance,
            native_host=host)
        if startup_fault is None or startup_fault == DIAGNOSTIC_STARTUP_MESSAGE:
            self.session.start_initial(paused=True)
        else:
            self.session.start_offline(startup_fault)

    def snapshot(self, *, compact=False, include_motion=True):
        return self.session.snapshot(compact=compact, include_motion=include_motion)

    @property
    def owns_connection(self):
        with self.session.lock:
            process = self.session.process
            return (self.session.recovery_pending or self.session.status in ('waiting', 'active', 'stopping')
                    or not self.session.connection_done.is_set()
                    or process is not None and process.poll() is None)

    def connection_closed(self):
        with self.session.lock:
            process = self.session.process
            if self.owns_connection:
                return False
            if process is None:
                return self.session._paused_without_attempt or (
                    self.session._reader_preflight_failure_generation == self.session.generation)
            proof = self.session.closed_proof or {}
            return (proof.get('generation') == self.session.generation
                    and proof.get('serial_closed') is True and process.poll() is not None)

    def end_for_motion(self, timeout=3):
        with self.session.lock:
            if self.session.connection_check['phase'] == 'checking':
                raise ValueError('正在复查连接，请等待本次检查完成再连接运控')
            if self.session.collection is not None or self.session.joint_wizard.active() or self.session._joint_zero_pending():
                raise ValueError('请先完成或取消未保存的采样，再连接运控')
            if self.session.status in ('waiting', 'active'):
                self.session.request_end('manual')
        if not self.session.connection_done.wait(timeout):
            raise ValueError('校准采集关闭尚未确认，运控未打开串口')
        with self.session.lock:
            process = self.session.process
            proof = self.session.closed_proof or {}
            # A rejected reader preflight never created a child or opened its
            # UART. Keep that fault visible, but do not demand a fictional
            # close record before the independent motion connection checks.
            unopened_preflight_failure = (self.session.status == 'fault'
                and process is None
                and self.session._reader_preflight_failure_generation == self.session.generation)
            if (self.session.recovery_pending or self.session.source_transmitting
                    or (self.session.status != 'closed' and not unopened_preflight_failure)
                    or (process is not None and (
                    process.poll() != 0 or proof.get('generation') != self.session.generation
                    or proof.get('normal') is not True or proof.get('serial_closed') is not True))):
                raise ValueError('校准连接缺少正常关闭证明，运控未打开串口')
            return copy.deepcopy(self.session.config)

    def dispatch(self, path, body):
        if path not in PATHS or type(body) is not dict:
            raise ValueError('未知的 X5 校准操作')
        if type(body.get('generation')) is not int or body['generation'] != self.session.generation:
            raise ValueError('校准会话已变化，请刷新页面')
        if path == '/api/recover-feedback':
            if set(body) not in ({'generation'}, {'generation', 'purpose'}):
                raise ValueError('恢复采集请求字段无效')
            purpose = body.get('purpose', 'joints')
            if purpose not in ('imu', 'joints'):
                raise ValueError('采集用途无效')
            self.before_acquire(purpose)
            # Purpose-bound recovery always uses the existing scoped recheck
            # and therefore waits for a continuously advancing 0.2 s window.
            return self.session.recheck_feedback_action(dict(generation=body.get('generation'), purpose=purpose))
        if path in ('/api/recheck-feedback', '/api/check-connection'):
            self.before_acquire(body.get('purpose'))
        action = {'/api/imu-wizard': self.session.action,
                  '/api/joint-calibration': self.session.joint_action,
                  '/api/joint-zero': self.session.joint_zero_action,
                  '/api/end-feedback': self.session.end_action,
                  '/api/recheck-feedback': self.session.recheck_feedback_action,
                  '/api/check-connection': self.session.check_connection_action,
                  '/api/servo-maintenance': self.session.maintenance_action}[path]
        return action(body)

    def close(self):
        with self.session.lock:
            if self.session.status in ('waiting', 'active'):
                self.session.request_end('manual')
        if not self.session.connection_done.wait(3):
            raise ValueError('X5 校准读串口关闭尚未确认')
        with self.session.lock:
            process = self.session.process
            if process is not None and (process.poll() is None
                    or (self.session.closed_proof or {}).get('serial_closed') is not True):
                raise ValueError('X5 校准缺少实际串口关闭与进程退出证明')
        return self.snapshot()
