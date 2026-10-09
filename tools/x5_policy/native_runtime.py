"""Independent RAM motion runtime using the reviewed one-shot controller.

Construction loads CPU models but never opens a UART. Every connection requires
an explicit call, owns a single reviewed UART, and retains STOP/closure evidence.
The caller supplies the reviewed board/RAM guard; there is no SSH parent lease.
"""
from __future__ import annotations

import ast
from collections import deque
import copy
import fcntl
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
import termios
import threading
import time


MODEL_NAMES = ('walk', 'getup', 'pick', 'roulade')
ACTION_EVIDENCE_LIMIT = 32
# Display publication only; controller sampling, safety gates, evidence drain
# and health monitoring retain their independent existing periods.
DISPLAY_PUBLISH_SECONDS = .2
# The live projection deliberately excludes complete historical STOP packets.
# Full status and on-disk evidence keep the original objects unchanged.
LIVE_HARDWARE_FIELDS = frozenset('''connected port protocol mode generation calibrated
mechanical_reference_confirmed hold_ready motor_output armed configured online_ids
fault_ids torque_ids age_ms imu_age_ms rawdata joints imu voltage_supported
motor_voltages voltage_range_v voltage_scale_hardware_verified command forward_trial last_action
inference_ms last_policy_target_deg last_policy_target_rejections last_executed_target_deg
target_rejection active_model behavior_protocol target_validation_protocol
policy_metadata_protocol stand_target_deg model_stand_target_deg hold_hip_pitch_bias_deg head_clearance_protocol
head_clearance_required fixed_head_walk_protocol fixed_head_walk_allowed
fixed_head_walk_ready fixed_head_walk_adapted fixed_head_walk_zero_velocity_only
hold_head_deg walk_preview_head_protocol walk_preview_head_command_rad policy_profiles
behavior_result behavior_seconds_remaining error pending_stop firmware_error sample_us
imu_us state_request_sequence timing rates background_hold hold_control_protocol
page_heartbeat_required heartbeat_remaining_ms start_blocked_reason model_start_blocked_reasons
release_confirmed imu_ok request_counts'''.split())
CONTROL_SUMMARY_FIELDS = frozenset('''connected mode generation release_confirmed armed
pending_stop online_ids fault_ids torque_ids age_ms imu_age_ms sample_us imu_us
hold_ready hold_head_deg serial_closed no_uart_opened reason error'''.split())
GUARD_NAMES = ('EXPECTED', 'ram_health', 'serial_proof', 'serial_path', '_holders', 'ProbeError',
               'connection_guard_factory')


def controller_classes(source):
    """Extract the two existing classes without executing the worker bootstrap."""
    if not isinstance(source, str) or len(source.encode()) > 512 * 1024:
        raise ValueError('Invalid reviewed controller source')
    tree = ast.parse(source)
    names = ('ReviewedTransport', 'OneShotController')
    found = {name: [node for node in ast.walk(tree)
                    if isinstance(node, ast.ClassDef) and node.name == name]
             for name in names}
    if any(len(nodes) != 1 for nodes in found.values()):
        raise ValueError('Reviewed source must contain each controller class exactly once')
    nodes = [found[name][0] for name in names]
    for node, base in zip(nodes, ('SerialTransport', 'MotionController')):
        if (node.decorator_list or len(node.bases) != 1 or node.keywords
                or not isinstance(node.bases[0], ast.Attribute)
                or not isinstance(node.bases[0].value, ast.Name)
                or node.bases[0].value.id != 'motion' or node.bases[0].attr != base
                or any(not isinstance(part, ast.FunctionDef) for part in node.body)):
            raise ValueError('Reviewed controller class structure changed')
    return compile(ast.Module(body=nodes, type_ignores=[]), '<reviewed-native-controller>', 'exec')


def _calibration_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()


def _control_summary(value):
    if not isinstance(value, dict):
        return None
    return copy.deepcopy({key: item for key, item in value.items() if key in CONTROL_SUMMARY_FIELDS})


def _compact_diagnostic(value):
    if not isinstance(value, dict):
        return None
    result = {key: value[key] for key in ('sequence', 'reason', 'source', 'recorded_monotonic',
              'details_deferred_until_UART_closed')
              if key in value}
    result.update(before=_control_summary(value.get('before')), details_available=True)
    return result


def _compact_operation(value):
    if not isinstance(value, dict):
        return None
    result = {key: value[key] for key in ('action', 'state', 'mode', 'error', 'generation',
              'stop_source', 'recorded_monotonic', 'connection_sequence', 'reason') if key in value}
    for key in ('before', 'after'):
        if key in value:
            result[key] = _control_summary(value[key])
    return result


def _compact_proof(value):
    if not isinstance(value, dict):
        return None
    result = {key: item for key, item in value.items() if key not in ('before', 'after')}
    result = copy.deepcopy(result)
    for key in ('before', 'after'):
        if key in value:
            result[key] = _control_summary(value[key])
    return result


def _hardware_projection(value, compact):
    # Select BEFORE copying, otherwise a compact GET still traverses all audit
    # history. Each result owns its lists; callers cannot mutate publication.
    if compact:
        result = copy.deepcopy({key: item for key, item in value.items() if key in LIVE_HARDWARE_FIELDS})
        result['last_stop_diagnostic'] = _compact_diagnostic(value.get('last_stop_diagnostic'))
        return result
    return copy.deepcopy(value)


def _own_controller_status(value):
    """Adopt a fresh reviewed status result after detaching its one borrowed list.

    MotionController.status constructs rawdata/IMU/voltage/timing/profile lists
    and copies configuration/targets; native OneShotController.status selects
    only a small STOP summary. Only base status's last_action borrows a mutable
    controller list. The root returned by status/start/stop/capture is new and
    stays private to this runtime; none of it is returned in a receipt.
    """
    value['last_action'] = copy.deepcopy(value.get('last_action'))
    return value


def _release_unconfirmed_reason(state, motor_ids, fresh_ms):
    """Explain an existing failed check without querying or accepting hardware."""
    reasons = []
    missing = [sid for sid in motor_ids if sid not in state.get('online_ids', [])]
    if missing:
        reasons.append('位置反馈无效或缺失的舵机 ID：' + ', '.join(map(str, missing)))
    for key, label in (('fault_ids', '报告故障的舵机 ID：'),
                       ('torque_ids', '扭矩反馈为开启的舵机 ID：')):
        values = state.get(key) or []
        if values:
            reasons.append(label + ', '.join(map(str, values)))
    age = state.get('age_ms')
    if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
        reasons.append('尚未收到有效状态反馈')
    elif age >= fresh_ms:
        reasons.append('状态反馈已过期（距采样 %.1f 毫秒）' % age)
    if state.get('armed'):
        reasons.append('控制器仍处于使能状态')
    if state.get('pending_stop'):
        reasons.append('停止指令尚未完成')
    if state.get('firmware_error'):
        reasons.append('下位机错误代码：' + str(state['firmware_error']))
    return '连接检查未通过：' + '；'.join(reasons or ['未取得完整的舵机扭矩关闭确认'])


class NativeRuntime:
    def __init__(self, guard_namespace, ram_directory, deployment, calibration, classes_source,
                 *, startup_fault=None, require_head_clearance=True, allow_fixed_head_walk=False,
                 hold_hip_pitch_bias_deg=0):
        if (type(hold_hip_pitch_bias_deg) not in (int, float)
                or not -3 <= hold_hip_pitch_bias_deg <= 3
                or not math.isfinite(hold_hip_pitch_bias_deg)):
            raise ValueError('固定 HOLD 髋偏置必须为 -3 至 3° 的有限数值')
        self._hold_hip_pitch_bias_deg = float(hold_hip_pitch_bias_deg)
        if startup_fault is not None and (type(startup_fault) is not str
                or not startup_fault.strip() or len(startup_fault.encode('utf-8')) > 16384):
            raise ValueError('A bounded actual hardware startup fault is required')
        if not isinstance(guard_namespace, dict) or any(name not in guard_namespace for name in GUARD_NAMES):
            raise ValueError('The reviewed native board guard is incomplete')
        if any(not callable(guard_namespace[name]) for name in GUARD_NAMES if name != 'EXPECTED'):
            raise ValueError('Invalid reviewed native board guard')
        self.guard = dict(guard_namespace)
        self.directory = Path(ram_directory)
        if (not self.directory.is_absolute() or self.directory.is_symlink()
                or self.directory.resolve(strict=True) != self.directory or not self.directory.is_dir()):
            raise ValueError('A canonical private RAM directory is required')
        self.deployment = copy.deepcopy(deployment)
        self.calibration = copy.deepcopy(calibration)
        self.require_head_clearance = require_head_clearance is True
        self.allow_fixed_head_walk = allow_fixed_head_walk is True
        if (not isinstance(self.deployment, dict)
                or set(self.deployment.get('model_hashes', {})) != set(MODEL_NAMES)):
            raise ValueError('All four reviewed models are required')
        calibration_path = self.directory / 'policy/calibration.json'
        node = calibration_path.lstat()
        if not stat.S_ISREG(node.st_mode) or node.st_mode & 0o077:
            raise ValueError('A private regular RAM calibration file is required')
        raw = calibration_path.read_bytes()
        if (raw != _calibration_bytes(self.calibration)
                or hashlib.sha256(raw).hexdigest() != self.deployment.get('calibration_sha256')):
            raise ValueError('RAM calibration differs from the reviewed deployment')
        self._classes = controller_classes(classes_source)
        self.classes_sha256 = hashlib.sha256(classes_source.encode()).hexdigest()
        self.lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._controller = None
        self._namespace = None
        self._monitor = None
        self._publisher = None
        self._action_evidence = deque()
        self._action_evidence_reserved = 0
        self._evidence_flush_lock = threading.RLock()
        self._latency_lock = threading.Lock()
        self._latency = {}
        self._generation_floor = 0
        self._connection_sequence = 0
        self._closed = False
        self.startup_fault = startup_fault
        self.phase = 'fault' if startup_fault is not None else 'idle'
        self.error = startup_fault
        self.closed_proof = (None if startup_fault is None else dict(
            serial_closed=True, release_confirmed=False, serial_open_attempts=0,
            request_counts={}, before=None, after=None, reason=startup_fault,
            no_uart_opened=True, scope='metadata_only_fault_page'))
        self.last_operation = None
        self._last_hardware = None
        self._hardware_snapshot = None
        self._hardware_snapshot_at = None
        self._hardware_snapshot_sequence = None
        self._hardware_version = 0
        self._hardware_generation_floor = 0
        self._evidence = self.directory / 'native-evidence'
        self._evidence.mkdir(mode=0o700, exist_ok=False)
        # An explicitly requested fault page may load its immutable CPU assets
        # while hardware is unavailable. This gives no UART permission: every
        # later connection still calls the original strict guard factory.
        if startup_fault is None:
            self.guard['ram_health']()
        self.motion = importlib.import_module('motion')
        infer = importlib.import_module('infer')
        self.sessions = {}
        for name in MODEL_NAMES:
            model_path = self.directory / 'policy/models' / ('xgoduck_' + name + '.onnx')
            if model_path.is_symlink() or hashlib.sha256(model_path.read_bytes()).hexdigest() != self.deployment['model_hashes'][name]:
                raise ValueError('Reviewed model digest differs: ' + name)
            session = infer.session(name)
            if session.get_providers() != ['CPUExecutionProvider']:
                raise ValueError('Native models must use the CPU provider')
            observation = infer.np.zeros((1, 61), infer.np.float32)
            observation[0, 5] = -1
            infer.infer(session, observation)
            self.sessions[name] = session
        # Metadata/calibration validation is CPU-only and does not call start().
        validation = self.motion.MotionController(self.sessions, config_path=calibration_path,
            hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg)
        # Explicit zero saves may invalidate one joint's direction/range.
        # The full workbench must still open to complete that calibration;
        # connect() retains the complete-calibration gate before opening UART.
        self._record('deployment.json', dict(deployment=self.deployment,
            controller_classes_sha256=self.classes_sha256, host='X5', runtime='CPUExecutionProvider',
            hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg,
            models=list(self.sessions), motor_start_attempted=False, serial_open_attempts=0))
        self._persist()

    @property
    def hold_hip_pitch_bias_deg(self):
        """Immutable connection profile chosen before any UART is opened."""
        return self._hold_hip_pitch_bias_deg

    def _record(self, name, value):
        # Compact encoding uses the C encoder and avoids pretty-printing large
        # histories in Python while sharing the control process's GIL.
        raw = (json.dumps(value, ensure_ascii=False, allow_nan=False,
                          separators=(',', ':')) + '\n').encode()
        temporary = self._evidence / ('.' + secrets.token_hex(16))
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(raw)
            os.replace(temporary, self._evidence / name)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    def _persist(self):
        # The live file is a display projection, not a historical export.
        # Complete STOP and trial diagnostics are saved after UART closure.
        self._record('live-state.json', self.snapshot(compact=True))

    def _reserve_action_evidence(self):
        with self.lock:
            if len(self._action_evidence) + self._action_evidence_reserved >= ACTION_EVIDENCE_LIMIT:
                raise ValueError('Native action evidence queue is full')
            self._action_evidence_reserved += 1

    def _remember_timing(self, name, value):
        # Fixed operation names, one small numeric record each; no history or
        # acquisition of the lifecycle/controller locks from instrumentation.
        with self._latency_lock:
            self._latency[name] = {key: round(item, 3) if type(item) is float else item
                                   for key, item in value.items()}

    def _timing_snapshot(self):
        with self._latency_lock:
            return dict(protocol=1, scope='latest_call_per_operation',
                        operations={key: dict(value) for key, value in self._latency.items()})

    def _queue_action_evidence(self, operation, *, owned=False):
        # Every accepted receipt reserves capacity before its physical action.
        # Capture the operation once; no JSON, file IO or sensor sampling here.
        began, cpu = time.perf_counter(), time.thread_time()
        transferred = operation if owned else copy.deepcopy(operation)
        copied, copy_cpu = time.perf_counter(), time.thread_time()
        with self.lock:
            acquired = time.perf_counter()
            self._action_evidence.append(('operation-' + secrets.token_hex(8) + '.json', transferred))
            self._action_evidence_reserved -= 1
            self.last_operation = transferred
        ended = time.perf_counter()
        self._remember_timing('evidence_enqueue', dict(wall_ms=(ended-began)*1000,
            cpu_ms=(time.thread_time()-cpu)*1000, copy_ms=(copied-began)*1000,
            copy_cpu_ms=(copy_cpu-cpu)*1000, lock_wait_ms=(acquired-copied)*1000,
            lock_hold_ms=(ended-acquired)*1000, ownership_transferred=owned))

    def _flush_action_evidence(self):
        # Peek until the write succeeds: a disk/audit failure retains the exact
        # pending operation in memory and faults the connection, never drops it.
        with self._evidence_flush_lock:
            for _ in range(ACTION_EVIDENCE_LIMIT):
                with self.lock:
                    if not self._action_evidence:
                        return
                    item = self._action_evidence[0]
                self._record(*item)
                with self.lock:
                    self._action_evidence.popleft()

    def reload_calibration(self, calibration, zero_provenance=None):
        """Accept a validated explicit save while the motion UART is closed.

        The bootstrap publisher atomically changes only the RAM calibration
        and its authenticated authority. No saved change reconnects hardware.
        """
        import calibration_restore
        checked = copy.deepcopy(calibration_restore.validate_calibration_snapshot(calibration))
        raw = _calibration_bytes(checked)
        digest = hashlib.sha256(raw).hexdigest()
        with self._lifecycle:
            with self.lock:
                if self._closed or self.phase == 'deploying' or self._owns_locked():
                    raise ValueError('保存校准前须停止并断开运控，确认运动串口已关闭')
                if self._controller is not None and (
                        self._namespace is None or not self.closed_proof
                        or self.closed_proof.get('serial_closed') is not True
                        or not self._serial_closed(self._controller, self._namespace)):
                    raise ValueError('运动串口关闭证明不完整，校准未载入运控')
                publish = self.guard.get('publish_calibration')
                if not callable(publish):
                    raise ValueError('当前后台未配置经过验证的校准保存入口')
                if publish(raw) != digest:
                    raise ValueError('已保存校准摘要与原子更新回执不一致')
                if (self.directory / 'policy/calibration.json').read_bytes() != raw:
                    raise ValueError('已保存校准与运控文件不一致')
                self.calibration = checked
                self.deployment['calibration_sha256'] = digest
                if self._controller is not None:
                    self._generation_floor = max(self._generation_floor,
                                                 self._controller.status()['generation'])
                self._controller, self._namespace = None, None
                self._last_hardware = None
            self.guard['ram_health']()
            self._record('calibration-reload.json', dict(calibration_sha256=digest,
                explicit_save=True, serial_open_attempts=0, motor_start_attempted=False,
                zero_provenance_present=zero_provenance is not None))
            self._persist()
        return digest

    def prepare_calibration_reload(self, calibration):
        """Validate a user save before its persistent commit; never open UART."""
        import calibration_restore
        checked = calibration_restore.validate_calibration_snapshot(copy.deepcopy(calibration))
        with self._lifecycle, self.lock:
            if self._closed or self.phase == 'deploying' or self._owns_locked():
                raise ValueError('保存校准前须停止并断开运控，确认运动串口已关闭')
            if self._controller is not None and (
                    self._namespace is None or not self.closed_proof
                    or self.closed_proof.get('serial_closed') is not True
                    or not self._serial_closed(self._controller, self._namespace)):
                raise ValueError('运动串口关闭证明不完整，校准未载入运控')
            if not callable(self.guard.get('publish_calibration')):
                raise ValueError('当前后台未配置经过验证的校准保存入口')
            return hashlib.sha256(_calibration_bytes(checked)).hexdigest()

    @staticmethod
    def _serial_closed(controller, namespace):
        thread = controller._thread
        return (controller._transport is None and controller._connected is False
                and (thread is None or not thread.is_alive())
                and all(getattr(transport, 'serial', None) is None
                        for transport in namespace['opened_transports']))

    def _owns_locked(self):
        return bool(self._controller is not None and self._namespace is not None
                    and not self._serial_closed(self._controller, self._namespace))

    def connection_reserved(self):
        """Check lifecycle ownership without acquiring sensor/controller state."""
        with self.lock:
            return self.phase == 'deploying' or self._owns_locked()

    def snapshot(self, *, compact=False):
        """Project one immutable publication, without copying under control locks.

        Published packets are private owned values, replaced rather than
        modified. A snapshot binds its packet, ages and metadata to the captured
        connection. A concurrent lifecycle/generation transition invalidates
        live feedback; it never relabels an older packet with a new connection.
        """
        began, cpu = time.perf_counter(), time.thread_time()
        with self.lock:
            acquired = time.perf_counter()
            controller, sequence = self._controller, self._connection_sequence
            proof = self.closed_proof
            closed = bool(proof and proof.get('serial_closed') is True and controller is not None
                          and self._serial_closed(controller, self._namespace))
            raw = self._hardware_snapshot if controller is not None and not closed else None
            published_at, publication_sequence = self._hardware_snapshot_at, self._hardware_snapshot_sequence
            version, generation_floor = self._hardware_version, self._hardware_generation_floor
            historical, operation = self._last_hardware, self.last_operation
            # Only this small root mapping is mutable during calibration saves.
            deployment = dict(self.deployment)
            value = dict(phase=self.phase, error=self.error, startup_fault=self.startup_fault,
                host='X5', ram_only=True, owns_connection=self._owns_locked(),
                connection_sequence=sequence,
                pending_action_evidence=len(self._action_evidence) + self._action_evidence_reserved,
                exit_reconciled=bool(proof and proof.get('serial_closed')),
                transport_exit=None, models_loaded=list(self.sessions),
                hardware_publication_version=version, snapshot_compact=compact)
        released = time.perf_counter()
        hardware, last_hardware, sample_age = None, None, None
        if raw is not None and publication_sequence == sequence and raw.get('generation', -1) >= generation_floor:
            current = _hardware_projection(raw, compact)
            elapsed_ms = max(0., time.monotonic() - published_at) * 1000
            for key in ('age_ms', 'imu_age_ms'):
                if type(current.get(key)) in (int, float):
                    current[key] = round(current[key] + elapsed_ms, 1)
            for row in current.get('motor_voltages', []):
                if type(row.get('age_ms')) in (int, float):
                    row['age_ms'] = round(row['age_ms'] + elapsed_ms, 1)
                    row['fresh'] = bool(row.get('fresh') and row['age_ms'] <= self.motion.VOLTAGE_FRESH_MS)
                    if not row['fresh']:
                        row['condition'] = 'unknown'
            joint_age, imu_age = current.get('age_ms'), current.get('imu_age_ms')
            if all(type(age) in (int, float) and math.isfinite(age) and age >= 0
                   for age in (joint_age, imu_age)):
                sample_age = max(joint_age, imu_age)
            if (current.get('connected') is True and type(joint_age) in (int, float)
                    and math.isfinite(joint_age) and 0 <= joint_age < 500):
                hardware = current
            # Compact live feedback needs no second copy of the same packet.
            # Full snapshots retain independently owned historical/live values.
            last_hardware = (None if compact and hardware is not None else
                             current if hardware is None else copy.deepcopy(current))
        elif historical is not None:
            last_hardware = _hardware_projection(historical, compact)
        value.update(hardware=hardware, last_hardware=last_hardware,
            feedback_age_ms=0. if hardware is not None else None, sample_age_ms=sample_age,
            deployment=copy.deepcopy(deployment),
            closed_proof=_compact_proof(proof) if compact else copy.deepcopy(proof),
            last_operation=_compact_operation(operation) if compact else copy.deepcopy(operation))
        projected = time.perf_counter()
        with self.lock:
            validated = time.perf_counter()
            latest = self._hardware_snapshot
            superseded = (controller is not self._controller or sequence != self._connection_sequence
                or proof is not self.closed_proof or value['phase'] != self.phase
                or raw is not None and self._hardware_generation_floor > raw.get('generation', -1)
                or raw is not None and latest is not None
                   and latest.get('generation', -1) > raw.get('generation', -1))
        if superseded:
            # Keep the captured sequence; discard its packets rather than mixing
            # a late old response with the new lifecycle's identity or freshness.
            value.update(hardware=None, last_hardware=None, sample_age_ms=None,
                         feedback_age_ms=None, owns_connection=False, snapshot_superseded=True)
        ended = time.perf_counter()
        self._remember_timing('snapshot_compact' if compact else 'snapshot_full',
            dict(wall_ms=(ended-began)*1000, cpu_ms=(time.thread_time()-cpu)*1000,
                 lock_wait_ms=(acquired-began)*1000, lock_hold_ms=(released-acquired)*1000,
                 projection_ms=(projected-released)*1000,
                 validate_lock_wait_ms=(validated-projected)*1000, superseded=superseded))
        value['runtime_timing'] = self._timing_snapshot()
        return value

    def control_context(self):
        """Ownership context for page leases, never a sensor freshness proof."""
        began, cpu = time.perf_counter(), time.thread_time()
        with self.lock:
            acquired = time.perf_counter()
            controller, sequence = self._controller, self._connection_sequence
        hardware = {}
        controller_wait = 0.
        if controller is not None:
            # A published display projection may predate a START/STOP. Read
            # only atomic controller ownership fields, without status gates,
            # UART reads, NumPy, sensor timestamps or heartbeat renewal.
            waiting = time.perf_counter()
            with controller._lock:
                controller_wait = time.perf_counter() - waiting
                state = controller._state or {}
                released = bool(controller._connected and state and
                    state.get('online_mask') == self.motion.ALL_MASK and not state.get('fault_mask') and
                    not state.get('armed') and not state.get('torque_mask') and
                    not state.get('cookie') and not controller._pending_stop)
                hardware = dict(connected=controller._connected, generation=controller._generation,
                                mode=controller._mode, release_confirmed=released,
                                hold_control_protocol=2)
        waiting = time.perf_counter()
        with self.lock:
            validated = time.perf_counter()
            if controller is not self._controller or sequence != self._connection_sequence:
                hardware = {}
            value = dict(phase=self.phase, owns_connection=self._owns_locked(),
                         connection_sequence=self._connection_sequence, hardware=hardware)
        self._remember_timing('control_context', dict(wall_ms=(time.perf_counter()-began)*1000,
            cpu_ms=(time.thread_time()-cpu)*1000, runtime_lock_wait_ms=(acquired-began)*1000,
            controller_lock_wait_ms=controller_wait*1000,
            validate_lock_wait_ms=(validated-waiting)*1000))
        return value

    def _publish_hardware(self, controller, current=None):
        """Publish outside HTTP paths and outside the runtime lifecycle lock."""
        # Starting the age baseline before status is conservative: status's
        # later gate/diagnostic work can never make a measurement younger.
        captured = time.monotonic()
        if current is None:
            current = _own_controller_status(controller.status())
        else:
            # Explicit callers may retain their result. The publisher's own
            # freshly allocated status result has the ownership contract above.
            current = copy.deepcopy(current)
        self._publish_owned_hardware(controller, current, captured)

    def _publish_owned_hardware(self, controller, current, captured, sequence=None):
        """Transfer a private immutable status; never accept a caller-owned root."""
        with self.lock:
            if (self._controller is not controller or
                    sequence is not None and sequence != self._connection_sequence or
                    self.closed_proof and self.closed_proof.get('serial_closed')):
                return
            previous = self._hardware_snapshot
            if (current['generation'] < self._hardware_generation_floor or
                    previous is not None and (current['generation'] < previous['generation'] or
                    current['generation'] == previous['generation'] and captured < self._hardware_snapshot_at)):
                return
            self._hardware_snapshot, self._hardware_snapshot_at = current, captured
            self._hardware_snapshot_sequence = self._connection_sequence
            self._hardware_generation_floor = max(self._hardware_generation_floor, current['generation'])
            self._hardware_version += 1
            self._last_hardware = current

    def _new_namespace(self):
        # The factory is called only by explicit connect after the old UART
        # and its control thread are proved closed. A shallow copy of guard
        # functions would retain their old globals/_SERIAL_BINDINGS forever.
        factory = self.guard.get('usb_recovery_guard_factory', self.guard['connection_guard_factory'])
        fresh = factory()
        if (not isinstance(fresh, dict) or any(name not in fresh for name in GUARD_NAMES)
                or any(not callable(fresh[name]) for name in GUARD_NAMES if name != 'EXPECTED')):
            raise ValueError('The explicit connection guard is incomplete')
        namespace = dict(fresh)
        namespace.update(motion=self.motion, fcntl=fcntl, termios=termios, time=time, os=os,
                         threading=threading, request_counts={}, serial_open_attempts=0,
                         opened_transports=[], health_error=None, exit_event=threading.Event(),
                         native_compact_stop_diagnostic=_compact_diagnostic)
        exec(self._classes, namespace)
        return namespace

    def connect(self):
        with self._lifecycle:
            with self.lock:
                if self._closed:
                    raise ValueError('Native service has closed')
                if self.phase == 'deploying' or self._owns_locked():
                    raise ValueError('请先确认当前运控连接已关闭')
                if self._action_evidence or self._action_evidence_reserved:
                    raise ValueError('前一动作审计尚未完成；先断开并等待证据写入')
                if self._controller is not None:
                    if (not self.closed_proof or not self.closed_proof.get('serial_closed')
                            or self._namespace is None
                            or not self._serial_closed(self._controller, self._namespace)):
                        raise ValueError('旧串口关闭尚未获得证明')
                    self._generation_floor = max(self._generation_floor, self._controller.status()['generation'])
                self.phase, self.error = 'deploying', None
                self._connection_sequence += 1
                self.closed_proof = None
                self._hardware_snapshot = self._hardware_snapshot_at = None
                self._hardware_snapshot_sequence = None
                self._last_hardware = None
                self._hardware_generation_floor = 0
                self._hardware_version += 1
                self.last_operation = dict(action='connect', state='running',
                                           connection_sequence=self._connection_sequence)
            # A factory refusal occurs before any transport exists, but still
            # needs a precise fault page and a truthful zero-open close proof.
            namespace = dict(serial_open_attempts=0, request_counts={})
            controller = None
            try:
                namespace = self._new_namespace()
                if callable(namespace.get('bind_serial')):
                    namespace['bind_serial'](namespace['EXPECTED'])
                namespace['ram_health']()
                node, _ = namespace['serial_proof'](namespace['EXPECTED'])
                if namespace['_holders'](node.st_rdev, time.monotonic() + .2):
                    raise ValueError('UART already has an owner')
                controller = namespace['OneShotController'](self.sessions,
                    port=namespace['serial_path'](namespace['EXPECTED']),
                    config_path=self.directory / 'policy/calibration.json',
                    transport_factory=namespace['ReviewedTransport'],
                    require_head_clearance=self.require_head_clearance,
                    allow_fixed_head_walk=self.allow_fixed_head_walk,
                    hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg)
                self._generation_floor += 1
                controller._generation = self._generation_floor
                with self.lock:
                    self._controller, self._namespace = controller, namespace
                if not controller._calibrated_locked():
                    raise ValueError('Saved calibration is incomplete')
                controller.start()
                deadline = time.monotonic() + 5
                while True:
                    state = controller.status()
                    if state['connected']:
                        break
                    if not controller._thread.is_alive() or time.monotonic() >= deadline:
                        raise ValueError(state.get('error') or 'Motion connection deadline exceeded')
                    time.sleep(.02)
                if state.get('release_confirmed') is not True:
                    raise ValueError(_release_unconfirmed_reason(
                        state, self.motion.MOTOR_IDS, self.motion.FRESH_SECONDS * 1000))
                completed = dict(self.last_operation, state='complete', after=copy.deepcopy(state))
                with self.lock:
                    self.phase, self.error = 'ready', None
                    self.startup_fault = None
                    self.last_operation = completed
                self._publish_hardware(controller, state)
                self._record('connection-%d-ready.json' % self._connection_sequence,
                    dict(hardware=state, serial_open_attempts=namespace['serial_open_attempts'],
                         request_counts=namespace['request_counts'], motor_start_attempted=False))
                self._monitor = threading.Thread(target=self._watch,
                    args=(controller, namespace, self._connection_sequence),
                    name='native-motion-health', daemon=True)
                self._publisher = threading.Thread(target=self._publish_loop,
                    args=(controller, namespace, self._connection_sequence),
                    name='native-motion-status', daemon=True)
                self._publisher.start()
                self._monitor.start()
                self._persist()
                return self.snapshot()
            except BaseException as error:
                if controller is not None:
                    self._finish(controller, namespace, str(error) or type(error).__name__, fault=True)
                else:
                    with self.lock:
                        self.phase, self.error = 'fault', str(error) or type(error).__name__
                        self.closed_proof = dict(serial_closed=True, release_confirmed=False,
                            serial_open_attempts=namespace['serial_open_attempts'],
                            request_counts=dict(namespace['request_counts']), before=None,
                            after=None, reason=self.error, no_uart_opened=True)
                        self.last_operation = dict(self.last_operation, state='failed', error=self.error)
                    self._persist()
                raise ValueError(self.error) from error

    def _finish(self, controller, namespace, reason, *, fault):
        with self._lifecycle:
            with self.lock:
                if self._controller is not controller:
                    return
                if (self.closed_proof and self.closed_proof.get('serial_closed') is True
                        and self._serial_closed(controller, namespace)):
                    return
            before = _own_controller_status(controller.status())
            release = False
            stop_error = None
            try:
                stopped = controller.stop(source='native-disconnect')
                release = stopped.get('release_confirmed') is True
            except BaseException as error:
                stop_error = str(error) or type(error).__name__
            namespace['exit_event'].set()
            try:
                controller.close()
            except BaseException as error:
                stop_error = stop_error or str(error) or type(error).__name__
            after = _own_controller_status(controller.status())
            if self._serial_closed(controller, namespace):
                # The transport and control thread are now closed: preserve
                # full evidence once, without competing with active control.
                after['last_stop_diagnostic'] = copy.deepcopy(
                    getattr(controller, '_last_stop_diagnostic', None))
            proof = dict(serial_closed=self._serial_closed(controller, namespace),
                release_confirmed=release, serial_open_attempts=namespace['serial_open_attempts'],
                request_counts=dict(namespace['request_counts']), before=before, after=after,
                reason=reason, stop_error=stop_error, no_uart_opened=namespace['serial_open_attempts'] == 0)
            # All roots published to snapshot are immutable after assignment.
            # Keep heavy copies outside the short publication lock.
            historical = copy.deepcopy(after)
            operation_proof = copy.deepcopy(proof)
            with self.lock:
                self.closed_proof = proof
                self._last_hardware = historical
                self._hardware_snapshot = self._hardware_snapshot_at = None
                self._hardware_snapshot_sequence = None
                self._hardware_generation_floor = max(self._hardware_generation_floor, after['generation'])
                self._hardware_version += 1
                self._generation_floor = max(self._generation_floor, after['generation'])
                self.phase = 'fault' if fault or not proof['serial_closed'] else 'closed'
                self.error = reason if fault else stop_error
                if not proof['serial_closed']:
                    self.error = self.error or '旧串口关闭尚未获得证明'
                self.last_operation = dict(action='disconnect',
                    state='complete' if proof['serial_closed'] and not stop_error else 'failed',
                    reason=reason, error=self.error, after=operation_proof)
            try:
                self._flush_action_evidence()
                self._record('connection-%d-closed.json' % self._connection_sequence, proof)
                self._persist()
            except BaseException as error:
                # A failed RAM evidence write must not remove the in-memory
                # fault page or replace actual UART closure with a guess.
                with self.lock:
                    self.phase = 'fault'
                    self.error = self.error or 'Native status evidence failed: ' + str(error)
                    self.closed_proof = dict(self.closed_proof,
                        evidence_error=str(error) or type(error).__name__)
                    self.last_operation = dict(self.last_operation, state='failed', error=self.error)

    def _publish_loop(self, controller, namespace, sequence):
        # Metadata inventories and audit writes can take longer than the
        # display's freshness window. They must not pause sensor publication.
        # This thread only publishes measured state; it never sends commands,
        # refreshes a measurement timestamp, or replaces the health monitor.
        while not namespace['exit_event'].wait(DISPLAY_PUBLISH_SECONDS):
            with self.lock:
                if self._controller is not controller or self._connection_sequence != sequence:
                    return
            try:
                self._publish_hardware(controller)
            except BaseException as error:
                namespace['health_error'] = 'Native status publication failed: ' + str(error)
                self._finish(controller, namespace, namespace['health_error'], fault=True)
                return

    def _watch(self, controller, namespace, sequence):
        last_health, last_persist = time.monotonic(), 0.
        while not namespace['exit_event'].wait(.05):
            with self.lock:
                if self._controller is not controller or self._connection_sequence != sequence:
                    return
            thread = controller._thread
            if thread is None or not thread.is_alive():
                namespace['exit_event'].set()
                break
            now = time.monotonic()
            try:
                self._flush_action_evidence()
            except BaseException as error:
                namespace['health_error'] = 'Native status evidence failed: ' + str(error)
                self._finish(controller, namespace, namespace['health_error'], fault=True)
                return
            if now - last_health >= 1:
                try:
                    namespace['ram_health']()
                except BaseException as error:
                    namespace['health_error'] = str(error) or type(error).__name__
                    self._finish(controller, namespace, namespace['health_error'], fault=True)
                    return
                last_health = now
            if now - last_persist >= .5:
                try:
                    self._persist()
                except BaseException as error:
                    namespace['health_error'] = 'Native status evidence failed: ' + str(error)
                    self._finish(controller, namespace, namespace['health_error'], fault=True)
                    return
                last_persist = now
        # _run sets this connection's event on terminal transport failure.
        with self.lock:
            already_closed = bool(self.closed_proof and self.closed_proof.get('serial_closed'))
        if not already_closed:
            reason = getattr(controller, '_terminal_error', None) or namespace['health_error'] or '运控连接已结束'
            self._finish(controller, namespace, reason, fault=True)

    def disconnect(self):
        with self._lifecycle:
            with self.lock:
                controller, namespace = self._controller, self._namespace
                already_closed = (controller is not None and namespace is not None and self.closed_proof
                        and self.closed_proof.get('serial_closed') is True
                        and self._serial_closed(controller, namespace))
            if already_closed:
                # Retry retained audit records after an earlier disk failure;
                # this can never reopen UART or resume a physical action.
                try:
                    self._flush_action_evidence()
                except BaseException as error:
                    with self.lock:
                        self.phase = 'fault'
                        self.error = 'Native status evidence failed: ' + str(error)
                        self.closed_proof = dict(self.closed_proof,
                            evidence_error=str(error) or type(error).__name__)
            elif controller is not None and namespace is not None:
                self._finish(controller, namespace, '操作员断开运控', fault=False)
            return self.snapshot()

    def action(self, body):
        if not isinstance(body, dict):
            raise ValueError('运控请求必须是对象')
        action = body.get('action')
        if action == 'connect':
            return self.connect()
        if action == 'disconnect':
            return self.disconnect()
        if action not in ('start', 'heartbeat', 'command', 'stop', 'capture-head-clearance', 'forward-trial'):
            raise ValueError('未知运控操作')
        with self.lock:
            controller, namespace = self._controller, self._namespace
            if controller is None or namespace is None:
                raise ValueError('请先连接并载入模型')
            if action != 'stop' and (self.phase != 'ready' or namespace['health_error']):
                raise ValueError(self.error or namespace['health_error'] or '运控连接未就绪')
        if action == 'heartbeat':
            if type(body.get('generation')) is not int:
                raise ValueError('Motion state changed; refresh before starting')
            if 'background' in body and type(body['background']) is not bool:
                raise ValueError('Invalid background heartbeat')
            result = controller.heartbeat(background=body.get('background') is True,
                expected_generation=body['generation'], minimal=True)
            if not result['connected'] or result['mode'] == 'off':
                raise ValueError('当前动作已停止，控制心跳未续期')
            # The independent publisher and monitor already publish measured
            # state and persist it. Lease responses must not wait for a full
            # status serialization or evidence write.
            return dict(heartbeat_accepted=True, generation=result['generation'])
        if action == 'stop':
            current = controller.status()
            if not current['connected']:
                raise ValueError(current.get('error') or '运控串口已关闭，请重新连接')
            result = controller.stop(source=body.get('stop_source', 'button'))
            if result.get('release_confirmed') is not True:
                raise ValueError(result.get('error') or '停止已发送，但未回读确认全部扭矩关闭')
        else:
            current = controller.status()
            if type(body.get('generation')) is not int or body['generation'] != current['generation']:
                raise ValueError('Motion state changed; refresh before starting')
            if action == 'capture-head-clearance':
                result = controller.capture_head_clearance(
                    clearance_confirmed=body.get('clearance_confirmed') is True,
                    expected_generation=body['generation'])
            elif action == 'forward-trial':
                result = controller.forward_trial(expected_generation=body['generation'],
                    support_confirmed=body.get('support_confirmed') is True)
            elif action == 'start':
                result = controller.start_mode(body.get('mode'),
                    support_confirmed=body.get('support_confirmed') is True,
                    expected_generation=body['generation'], preview_model=body.get('preview_model', 'walk'),
                    behavior_confirmed=body.get('behavior_confirmed') is True)
            else:
                result = controller.command(body.get('forward'), body.get('side'), body.get('yaw'))
        try:
            self._publish_hardware(controller, result)
            if action not in ('heartbeat', 'command'):
                operation = dict(action=action, state='complete', after=copy.deepcopy(result))
                with self.lock:
                    self.last_operation = operation
                self._record('operation-' + secrets.token_hex(8) + '.json', operation)
            if action != 'command':
                self._persist()
        except BaseException as error:
            namespace['health_error'] = 'Native status evidence failed: ' + str(error)
            self._finish(controller, namespace, namespace['health_error'], fault=True)
            raise ValueError(namespace['health_error']) from error
        return self.snapshot()

    def action_receipt(self, body):
        """START/STOP/head capture acknowledgement; no display or evidence IO.

        Controller feedback/target/head/tilt/heartbeat gates are unchanged.
        STOP still performs the reviewed STOP/STATE release verification.
        The watcher writes the bounded, owned audit event; failures fault-stop.
        This context is never a claim of fresh sensor data or a renewed lease.
        """
        if not isinstance(body, dict) or body.get('action') not in ('start', 'stop', 'capture-head-clearance', 'forward-trial'):
            raise ValueError('Control receipts support only start, stop and head capture')
        action = body['action']
        with self.lock:
            controller, namespace = self._controller, self._namespace
            sequence = self._connection_sequence
            if controller is None or namespace is None:
                raise ValueError('请先连接并载入模型')
            if action != 'stop' and (self.phase != 'ready' or namespace['health_error']):
                raise ValueError(self.error or namespace['health_error'] or '运控连接未就绪')
        if action != 'stop' and type(body.get('generation')) is not int:
            raise ValueError('Motion state changed; refresh before starting')
        try:
            self._reserve_action_evidence()
        except BaseException as error:
            namespace['health_error'] = 'Native status evidence failed: ' + str(error)
            self._finish(controller, namespace, namespace['health_error'], fault=True)
            raise ValueError(namespace['health_error']) from error
        operation = dict(action=action, state='pending', generation=body.get('generation'),
                         connection_sequence=sequence, mode=body.get('mode'),
                         recorded_monotonic=time.monotonic())
        queued = False
        try:
            began, action_cpu = time.monotonic(), time.thread_time()
            if action == 'start':
                # start_mode checks this generation atomically, so a duplicate
                # pre-action full status cannot improve its safety proof.
                result = controller.start_mode(body.get('mode'),
                    support_confirmed=body.get('support_confirmed') is True,
                    expected_generation=body['generation'],
                    preview_model=body.get('preview_model', 'walk'),
                    behavior_confirmed=body.get('behavior_confirmed') is True)
            elif action == 'forward-trial':
                forward = getattr(controller, 'forward_trial', None)
                if not callable(forward): raise ValueError('当前模型版本不支持受限试步')
                result = forward(expected_generation=body['generation'],
                    support_confirmed=body.get('support_confirmed') is True)
            elif action == 'capture-head-clearance':
                # The original controller checks actual generation, OFF, fresh
                # feedback, complete calibration, all motors, and head ranges.
                # Captured angles are configuration, never a new sensor packet.
                result = controller.capture_head_clearance(
                    clearance_confirmed=body.get('clearance_confirmed') is True,
                    expected_generation=body['generation'])
            else:
                context = self.control_context()
                if not context['hardware'].get('connected'):
                    raise ValueError('运控串口已关闭，请重新连接')
                result = controller.stop(source=body.get('stop_source', 'button'))
                if result.get('release_confirmed') is not True:
                    raise ValueError(result.get('error') or '停止已发送，但未回读确认全部扭矩关闭')
            action_ms = (time.monotonic() - began) * 1000
            action_cpu_ms = (time.thread_time() - action_cpu) * 1000
            # Controller transitions return a new status root with only
            # last_action borrowed. Adopt once; publication and the audit queue
            # share immutable private references, never exposed to the caller.
            result = _own_controller_status(result)
            # Share this now-owned immutable result with the display and audit
            # event. The baseline precedes the transition, conservatively
            # counting lock/STOP/STATE work rather than refreshing its age.
            self._publish_owned_hardware(controller, result, began, sequence)
            operation.update(state='complete', after=result)
            enqueue_at = time.monotonic()
            self._queue_action_evidence(operation, owned=True)
            queued = True
            enqueue_ms = (time.monotonic() - enqueue_at) * 1000
            context_at = time.monotonic()
            context = self.control_context()
            current = context['hardware']
            if (context['connection_sequence'] != sequence or
                    current.get('generation') != result.get('generation') or
                    current.get('mode') != result.get('mode') or not current.get('connected') or
                    action == 'start' and current.get('mode') != body.get('mode') or
                    action in ('stop', 'capture-head-clearance') and
                    (current.get('mode') != 'off' or current.get('release_confirmed') is not True)):
                raise ValueError('Motion state changed; refresh before starting')
            receipt = dict(action_receipt_protocol=1, action=action,
                        connection_sequence=sequence, generation=current['generation'],
                        mode=current['mode'], connected=current['connected'],
                        release_confirmed=current['release_confirmed'],
                        transition_timing_ms=dict(controller_action=round(action_ms, 3),
                            context=round((time.monotonic()-context_at)*1000, 3),
                            evidence_enqueue=round(enqueue_ms, 3)))
            if action == 'capture-head-clearance':
                head = result.get('hold_head_deg')
                if (not isinstance(head, dict) or set(head) != {30, 31, 32, 33} or
                        any(type(angle) not in (int, float) or not math.isfinite(angle)
                            for angle in head.values())):
                    raise ValueError('Captured head configuration is incomplete')
                receipt['captured_head_deg'] = {str(sid): angle for sid, angle in head.items()}
            self._remember_timing('action_receipt', dict(controller_action_ms=action_ms,
                controller_action_cpu_ms=action_cpu_ms, context_ms=receipt['transition_timing_ms']['context'],
                evidence_enqueue_ms=enqueue_ms))
            receipt['runtime_timing'] = self._timing_snapshot()
            return receipt
        except BaseException as error:
            if not queued:
                operation.update(state='failed', error=str(error) or type(error).__name__)
                self._queue_action_evidence(operation, owned=True)
            # A concurrent disconnect can stop the watcher before this action
            # exits preflight. Flush its late audit event after physical STOP.
            if namespace['exit_event'].is_set():
                try:
                    self._flush_action_evidence()
                except BaseException as evidence_error:
                    namespace['health_error'] = 'Native status evidence failed: ' + str(evidence_error)
                    with self.lock:
                        self.phase, self.error = 'fault', namespace['health_error']
                    self._finish(controller, namespace, namespace['health_error'], fault=True)
            raise

    def close(self):
        with self._lifecycle:
            with self.lock:
                self._closed = True
            return self.disconnect()
