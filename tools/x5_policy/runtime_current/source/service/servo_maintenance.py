"""Explicit binary-firmware maintenance owner; importing never opens hardware.

The injected transport owns XD framing, sequence validation, SSH and UART. One
worker serializes commands and heartbeats. A missing closure proof retains the
owner, so callers cannot start a calibration reader over an uncertain session.
No operation retries a write, changes saved calibration or clears safety holds.
"""
from __future__ import annotations

import copy
import re
import secrets
import struct
import threading
import time

from joint_calibration import DEFAULT_DEG, IDS
from maintenance_diagnostics import describe_center_failure

CAPABILITY = b'XGODUCK_MAINT_1'
CAPS, ENTER, READ, RENAME, CENTER, POLL, RELEASE, EXIT = range(8, 16)
ACK = 0x80
ROW = struct.Struct('<IBBBHB')
VOLTAGE_CAPS, VOLTAGE_READ = 17, 18
VOLTAGE_CAPABILITY = b'XGODUCK_VOLTAGE_1'
VOLTAGE_ROW = struct.Struct('<IBBBB')  # cookie, ID, outcome, device error, raw 0.1 V
HEARTBEAT_SECONDS = .15
FRESH_SECONDS = .5
CENTER_SECONDS = 10.0
CENTER_FAILURES = {
    16: '动作前的位置或扭矩反馈未通过检查',
    17: '写入当前位置作为保持目标时，发送或舵机应答未通过检查',
    18: '保持目标位置读回不一致或回复无效',
    19: '写入保持目标后的扭矩模式或反馈未通过检查',
    20: '开启当前舵机扭矩时，发送或舵机应答未通过检查',
    21: '未读回确认当前舵机扭矩已开启',
}


class MaintenanceError(ValueError):
    pass


def integer(value, low, high, label):
    if type(value) is not int or not low <= value <= high:
        raise MaintenanceError(f'{label}须为 {low}–{high} 的整数')
    return value


class MaintenanceController:
    """Local-only until an explicit enter or one-click scan action.

    gate() returns a blocking message or None. It must allow the already-owned
    maintenance session to read/exit after identity_changed(), while blocking
    calibration separately. The mapping callback runs BEFORE attempting an ID
    write. All externally shared state changes use the supplied reentrant lock.
    """

    def __init__(self, *, lock, transport_factory, gate, calibration,
                 identity_changed, fault_callback=lambda error: None, clock=time.monotonic,
                 review_gate=None, reviewed_callback=None):
        self.lock = lock
        self._condition = threading.Condition(lock)
        self._factory, self._gate = transport_factory, gate
        self._calibration, self._identity_changed = calibration, identity_changed
        self._fault_callback = fault_callback
        self._review_gate, self._reviewed_callback = review_gate, reviewed_callback
        self._reviewing = self._review_cancelled = self._exit_confirmed = False
        self._review_rows = None
        self._hardware_fault = self._fault_reported = False
        self.clock = clock
        self._transport = self._thread = None
        self._factory_attempted = False
        self._factory_failure_proof = None
        self._pending = None
        self._start_with_enter = False
        self._stop = False
        self._entered = False
        self._exit_attempted = False
        self._committed_write = None
        self._cookie = 0
        self._last_poll = self._received = None
        self._voltage_received = {}
        self._state = dict(phase='closed', owns_connection=False, connected=False,
                           compatible=None, busy=False, action='', error=None,
                           message='维护尚未连接。', servo=None, results=[], progress=0,
                           identity_changed=False, received_age_ms=None, close_proof=None,
                           voltage_supported=None, voltage_results=[])

    @property
    def owns_connection(self):
        with self.lock:
            return self._state['owns_connection']

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy(self._state)
            value['received_age_ms'] = (None if self._received is None else
                                       max(0., self.clock() - self._received) * 1000.)
            for row in value['voltage_results']:
                row['age_ms'] = max(0., self.clock() - self._voltage_received[row['id']]) * 1000.
            return value

    def _allowed(self):
        with self.lock:
            reason = (self._review_gate() if self._reviewing and self._review_gate is not None
                      else self._gate())
        if reason:
            raise MaintenanceError(str(reason))

    def _update(self, **values):
        with self.lock:
            self._state.update(values)

    def _validated(self, body):
        if type(body) is not dict or type(body.get('action')) is not str:
            raise MaintenanceError('维护请求格式无效')
        data = copy.deepcopy(body)
        action = data['action']
        fields = {'enter': set(), 'review': set(), 'read': {'id'}, 'scan': set(), 'voltages': set(), 'exit': set(),
                  'release': {'id'}, 'rename': {'id', 'new_id', 'isolated_confirmed', 'mapping_acknowledged'},
                  'center': {'id', 'support_confirmed'}}
        expected = fields.get(action, set()) | {'action'}
        if action not in fields or (set(data) != expected and not (
                action == 'center' and set(data) == expected | {'target'})):
            raise MaintenanceError('维护操作或参数无效')
        if 'id' in data:
            integer(data['id'], 0, 253, '舵机编号')
        if action == 'rename':
            integer(data['new_id'], 0, 253, '新编号')
            if data['id'] == data['new_id']:
                raise MaintenanceError('新编号与原编号相同')
            if data['isolated_confirmed'] is not True or data['mapping_acknowledged'] is not True:
                raise MaintenanceError('改编号须确认只连接选中舵机，并确认原编号映射将失效')
        if action == 'center' and data['support_confirmed'] is not True:
            raise MaintenanceError('回中前须确认机身已稳固支撑')
        if action == 'center' and 'target' in data:
            integer(data['target'], 0, 4095, '目标位置')
        return data

    def action(self, body):
        data = self._validated(body)
        action = data['action']
        with self._condition:
            if action == 'review':
                if self._review_gate is None or self._reviewed_callback is None:
                    raise MaintenanceError('当前服务未配置维护故障复查')
                reason = self._review_gate()
                if reason: raise MaintenanceError(reason)
            elif action not in ('exit', 'release'):
                self._allowed()
            start_for_scan = (action in ('scan', 'voltages') and not self.owns_connection
                              and not self._state['connected'])
            if action in ('enter', 'review') or start_for_scan:
                if self.owns_connection or self._thread is not None and self._thread.is_alive():
                    raise MaintenanceError('维护连接仍在占用或收尾，请先结束')
                self._state.update(phase='connecting', owns_connection=True, connected=False,
                                   compatible=None, busy=True, action=action, error=None,
                                   message='正在检查维护协议能力。', servo=None, results=[],
                                   progress=0, close_proof=None)
                self._state.update(voltage_supported=None, voltage_results=[])
                self._voltage_received.clear()
                self._received, self._entered, self._stop = None, False, False
                self._exit_attempted = False
                self._hardware_fault = self._fault_reported = False
                self._reviewing = action == 'review'
                self._review_cancelled = self._exit_confirmed = False
                self._review_rows = None
                self._factory_attempted, self._factory_failure_proof = False, None
                self._pending = data
                self._start_with_enter = start_for_scan or self._reviewing
                self._thread = threading.Thread(target=self._worker, daemon=True,
                                                name='servo-maintenance-owner')
                try:
                    self._thread.start()
                except BaseException:
                    self._state.update(phase='closed', owns_connection=False, busy=False, action='')
                    self._pending = None
                    raise
            elif action == 'exit':
                if not self.owns_connection:
                    return self.snapshot()
                self._stop = True
                if self._reviewing: self._review_cancelled = True
                self._state.update(busy=True, action='exit', phase='closing', message='正在结束维护连接。')
                if self._thread is None or not self._thread.is_alive():
                    self._thread = threading.Thread(target=self._close_transport, daemon=True,
                                                    name='servo-maintenance-close')
                    self._thread.start()
                self._condition.notify_all()
            else:
                if self._reviewing:
                    raise MaintenanceError('正在只读复查舵机反馈，请等待结果；复查期间不能执行维护动作')
                if not self._state['connected'] or self._state['compatible'] is not True:
                    raise MaintenanceError('请先建立兼容的维护连接')
                if self._state['busy'] or self._pending is not None or self._stop:
                    raise MaintenanceError('当前维护操作尚未完成')
                if action == 'center':
                    self._center_target(data)
                self._pending = data
                self._state.update(busy=True, action=action, error=None, progress=0,
                                   message='正在执行维护操作。')
                self._condition.notify_all()
            return self.snapshot()

    def close(self, timeout=2.):
        """Request cleanup; a timeout never fabricates a transport closure."""
        self.action({'action': 'exit'})
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout)
        return self.snapshot()

    def _request(self, kind, payload=b'', reply=None, timeout=.2):
        try:
            result = self._transport.request(kind, payload, kind | 0x80 if reply is None else reply, timeout)
        except BaseException as error:
            self._hardware_fault = True
            match = re.fullmatch(r'Maintenance request rejected: ACK error ([0-9]+)', str(error))
            if kind == CENTER and match and int(match.group(1)) in CENTER_FAILURES:
                code = int(match.group(1))
                detail = getattr(self._transport, 'center_diagnostic', None)
                evidence = (' ' + describe_center_failure(detail) + '；原始应答已保存。'
                            if detail is not None and detail['stage'] == code else '')
                raise MaintenanceError(f'回中未完成：{CENTER_FAILURES[code]}（步骤码 {code}）。{evidence}') from error
            raise
        if type(result) is not bytes:
            self._bad_feedback('维护响应格式无效')
        return result

    def _bad_feedback(self, reason):
        self._hardware_fault = True
        raise MaintenanceError(reason)

    def _ack(self, kind, payload, timeout=.3):
        result = self._request(kind, payload, ACK, timeout)
        if len(result) != 2 or result[0] != kind:
            self._bad_feedback('维护确认响应不匹配')
        if result[1] != 0:
            if kind == CENTER and result[1] in CENTER_FAILURES:
                self._bad_feedback(f'回中未完成：{CENTER_FAILURES[result[1]]}（步骤码 {result[1]}）。')
            self._bad_feedback(f'固件拒绝维护操作 {kind}，错误码 {result[1]}')

    def _submit_mutation(self, kind, payload, *, timeout=.3, before=None):
        """Linearize a one-shot write against exit, without locking network I/O.

        If exit acquires the shared lock first, the mutation is never submitted.
        Once reserved here, it is already committed to the transport; exit can
        still set _stop immediately and the worker sends EXIT after this reply.
        """
        with self.lock:
            if self._stop:
                return False
            self._allowed()
            if before is not None:
                before()
            if self._stop:
                return False
            self._committed_write = kind
        try:
            self._ack(kind, payload, timeout=timeout)
        finally:
            with self.lock:
                self._committed_write = None
        return True

    def _row(self, payload, sid=None, *, poll=False):
        if len(payload) != ROW.size:
            self._bad_feedback('维护反馈长度无效')
        cookie, actual, status, error, position, torque = ROW.unpack(payload)
        if cookie != self._cookie or actual > 253 and (not poll or actual != 255):
            self._bad_feedback('维护反馈会话或编号不匹配')
        if sid is not None and actual != sid:
            self._bad_feedback('维护反馈编号不匹配')
        if status > 3:
            self._bad_feedback('维护反馈状态无效')
        good = (not poll and status == 0) or (poll and status != 3)
        if poll and actual == 255:
            if status != 0 or error != 0 or position != 0xffff or torque != 0xff:
                self._bad_feedback('维护空闲反馈无效')
            good = False
        if good and (error != 0 or position > 4095 or torque > 1):
            self._bad_feedback('维护反馈数值无效')
        if not poll and status == 2 and error == 0:
            self._bad_feedback('舵机故障反馈缺少错误码')
        return dict(id=actual, state=status, error=error,
                    position=position if good else None, torque=torque if good else None,
                    online=good and actual != 255)

    def _remember(self, row):
        with self.lock:
            self._state['servo'] = copy.deepcopy(row)
            self._received = self.clock()

    def _read(self, sid):
        self._allowed()
        row = self._row(self._request(READ, struct.pack('<IB', self._cookie, sid)), sid)
        self._remember(row)
        if row['state'] in (2, 3):
            self._bad_feedback(f'舵机 ID {sid} 反馈异常，错误码 0x{row["error"]:02X}')
        return row

    def _healthy(self, sid, *, allow_torque=False):
        row = self._read(sid)
        if not row['online']:
            self._bad_feedback(f'舵机 ID {sid} 未响应')
        if not allow_torque and row['torque'] != 0:
            self._bad_feedback('实际扭矩尚未关闭，未执行写操作')
        return row

    def _poll(self, *, force=False):
        if not force and self._last_poll is not None and self.clock() - self._last_poll < HEARTBEAT_SECONDS:
            return None
        self._allowed()
        row = self._row(self._request(POLL, struct.pack('<I', self._cookie)), poll=True)
        self._last_poll = self.clock()
        if row['id'] != 255:
            self._remember(row)
        if row['state'] == 3:
            self._bad_feedback(f'维护固件报告故障，错误码 0x{row["error"]:02X}')
        return row

    def _scan(self):
        results = []
        self._update(results=[])
        for sid in range(254):
            if self._stop:
                return None
            self._poll()
            row = self._read(sid)
            if row['online']:
                results.append(row)
            self._update(results=copy.deepcopy(results), progress=round((sid + 1) * 100 / 254))
        return results

    def _voltages(self):
        """One explicit batch, using the existing exclusive, disarmed owner.

        This diagnostic never supplies torque/position proof, clears a hold,
        changes calibration, or retries an unresponsive servo.
        """
        self._update(voltage_results=[], message='正在检查舵机电压读取能力。')
        self._voltage_received.clear()
        self._allowed()
        capability = self._request(VOLTAGE_CAPS, timeout=.3)
        if capability == bytes((VOLTAGE_CAPS, 1)):
            self._update(voltage_supported=False,
                         message='当前 ESP32 固件尚不支持舵机电压采集，需要更新固件。')
            return
        if capability != VOLTAGE_CAPABILITY:
            self._bad_feedback('舵机电压能力响应无效')
        self._update(voltage_supported=True)
        rows = []
        for sid in IDS:
            if self._stop:
                return
            self._poll()
            self._allowed()
            payload = self._request(VOLTAGE_READ, struct.pack('<IB', self._cookie, sid))
            if len(payload) != VOLTAGE_ROW.size:
                self._bad_feedback('舵机电压反馈长度无效')
            cookie, actual, status, error, raw = VOLTAGE_ROW.unpack(payload)
            if cookie != self._cookie or actual != sid or status > 3:
                self._bad_feedback('舵机电压反馈会话、编号或状态无效')
            if ((status == 0 and error != 0) or (status == 1 and error != 0)
                    or (status == 2 and error == 0) or (status != 0 and raw != 0)):
                self._bad_feedback('舵机电压反馈数值无效')
            row = dict(id=sid, state=status, error=error,
                       raw=raw if status == 0 else None,
                       voltage_v=raw / 10. if status == 0 else None)
            rows.append(row)
            with self.lock:
                self._voltage_received[sid] = self.clock()
                self._state.update(voltage_results=copy.deepcopy(rows),
                                   progress=round(len(rows) * 100 / len(IDS)))
            if status in (2, 3):
                self._bad_feedback(f'舵机 ID {sid} 电压应答异常，错误码 0x{error:02X}；采集已停止')
        count = sum(row['state'] == 0 for row in rows)
        self._update(message=f'电压采集完成：{count} / {len(IDS)} 个有效读数。未响应不代表电压为 0 V。')

    def _center_config(self, sid):
        if self._state['identity_changed']:
            raise MaintenanceError('编号映射已变化，核对映射前禁止按原校准回中')
        config = self._calibration()
        joints = config.get('joints', []) if type(config) is dict else []
        matches = [j for j in joints if type(j) is dict and j.get('id') == sid]
        if sid not in IDS or len(matches) != 1:
            raise MaintenanceError('所选舵机没有唯一的已保存关节校准')
        joint = matches[0]
        low = integer(joint.get('min'), 0, 4095, '安全下限')
        high = integer(joint.get('max'), 0, 4095, '安全上限')
        zero = integer(joint.get('zero'), 0, 4095, '机械零点')
        sign = joint.get('sign')
        if joint.get('direction_confirmed') is not True or type(sign) is not int or sign not in (-1, 1):
            raise MaintenanceError('关节方向尚未确认')
        stand = round(zero + sign * DEFAULT_DEG[IDS.index(sid)] * 4095. / 360.)
        if not low < high or not low <= zero <= high or not low <= stand <= high:
            raise MaintenanceError('已保存安全范围未包含机械零点和默认站姿')
        return zero, low, high

    def _center_target(self, data):
        zero, low, high = self._center_config(data['id'])
        target = data.get('target', zero)
        integer(target, 0, 4095, '目标位置')
        if not low <= target <= high:
            raise MaintenanceError(f'目标位置须在该关节已保存的安全范围 {low}–{high} 内')
        return target, low, high

    def _enter(self):
        self._allowed()
        self._factory_attempted = True
        try:
            self._transport = self._factory()
        except BaseException as error:
            # A failed factory may already have opened SSH/UART. Retain its
            # partial transport and demand actual closure, never infer it.
            self._transport = getattr(error, 'transport', None)
            self._factory_failure_proof = getattr(error, 'close_report', None)
            raise
        if self._stop:
            return
        self._allowed()
        try:
            capability = self._request(CAPS, timeout=.3)
        except Exception as error:
            unsupported = re.fullmatch(
                r'Maintenance unsupported by connected firmware: ACK error ([1-9][0-9]{0,2})',
                str(error))
            if unsupported is not None and int(unsupported.group(1)) <= 255:
                self._update(compatible=False)
                raise MaintenanceError('当前 ESP32 固件不支持统一维护协议，需要先更新固件') from error
            raise
        if capability != CAPABILITY:
            self._update(compatible=False)
            raise MaintenanceError('当前 ESP32 固件不支持统一维护协议，需要先更新固件')
        self._update(compatible=True)
        self._cookie = secrets.randbelow(0xffffffff) + 1
        if self._stop:
            return
        self._allowed()
        self._ack(ENTER, struct.pack('<I', self._cookie))
        self._entered = True
        if self._stop:
            return
        self._poll(force=True)
        self._update(phase='idle', connected=True, message='维护已连接，等待选择操作。')

    def _perform(self, data):
        action = data['action']
        if action == 'enter':
            return self._enter()
        if action == 'review':
            rows = []
            for sid in IDS:
                if self._stop: return
                self._poll()
                rows.append(self._healthy(sid))
                self._update(progress=round(len(rows) * 100 / len(IDS)),
                             message=f'正在复查舵机反馈：{len(rows)} / {len(IDS)}。')
            self._review_rows = rows
            self._stop = True  # EXIT/closure must succeed before clearing the hold.
            return
        if action == 'read':
            row = self._read(data['id'])
            self._update(message=('读取完成。' if row['online'] else '所选编号未响应。'))
        elif action == 'scan':
            rows = self._scan()
            if rows is not None:
                self._update(message=f'扫描完成，发现 {len(rows)} 个舵机。')
        elif action == 'voltages':
            self._voltages()
        elif action == 'release':
            self._ack(RELEASE, struct.pack('<IB', self._cookie, data['id']))
            # Reading must still pass the caller's health gate. If it now fails,
            # the release was attempted once; cleanup reports no false success.
            self._healthy(data['id'])
            self._update(message='已读回确认所选舵机扭矩关闭。')
        elif action == 'rename':
            self._healthy(data['id'])
            rows = self._scan()
            if rows is None:
                return
            if len(rows) != 1 or rows[0]['id'] != data['id']:
                raise MaintenanceError('改编号要求总线只检测到选中的一只舵机，未写入编号')
            self._healthy(data['id'])
            def invalidate_mapping():
                self._state['identity_changed'] = True
                self._identity_changed(data['id'], data['new_id'])
            if not self._submit_mutation(
                    RENAME, struct.pack('<IBB', self._cookie, data['id'], data['new_id']),
                    timeout=1.8, before=invalidate_mapping):
                return
            if self._stop:
                return
            self._healthy(data['new_id'])
            self._update(message='新编号读回成功；旧校准映射已标记待核对，断电保存尚未验证。')
        elif action == 'center':
            with self.lock:
                target, low, high = self._center_target(data)
            initial = self._healthy(data['id'])
            if not low <= initial['position'] <= high:
                raise MaintenanceError('当前位置不在已保存安全范围内，未执行回中')
            if not self._submit_mutation(
                    CENTER, struct.pack('<IBHHH', self._cookie, data['id'], target, low, high)):
                return
            self._update(phase='moving', message=f'正在缓慢移动到目标位置 {target}。')
            deadline = self.clock() + CENTER_SECONDS
            while not self._stop:
                row = self._poll(force=True)
                if row['id'] != data['id'] or row['state'] not in (1, 2):
                    self._bad_feedback('回中反馈的目标舵机或阶段不匹配')
                if row['state'] == 2:
                    if abs(row['position'] - target) > 12 or row['torque'] != 0:
                        self._bad_feedback('回中终点或释放扭矩未通过验证')
                    self._update(message=f'已到达目标位置 {target}，并读回确认扭矩关闭。', progress=100)
                    break
                if self.clock() >= deadline:
                    self._bad_feedback('回中超时，维护连接正在关闭')
                with self._condition:
                    self._condition.wait(HEARTBEAT_SECONDS)

    def _worker(self):
        try:
            while True:
                with self._condition:
                    if self._stop:
                        break
                    data, self._pending = self._pending, None
                if data is not None:
                    if self._start_with_enter:
                        self._start_with_enter = False
                        self._enter()
                        if self._stop:
                            continue
                    self._perform(data)
                    with self.lock:
                        if not self._stop:
                            self._state.update(busy=False, action='', phase='idle')
                else:
                    self._poll()
                    with self._condition:
                        if self._pending is None and not self._stop:
                            self._condition.wait(HEARTBEAT_SECONDS)
            self._exit_once()
        except BaseException as error:
            self._update(error=str(error), message=str(error), phase='fault')
            self._report_fault_once(error)
            # A host validation failure can leave the existing transport live.
            # Request the firmware's torque-off exit once rather than waiting
            # for its watchdog. Never reopen or retry a failed EXIT.
            try:
                self._exit_once()
            except BaseException as exit_error:
                with self.lock:
                    self._state['message'] += f'；退出维护未确认：{exit_error}'
                self._report_fault_once(exit_error)
        finally:
            self._close_transport()
            if (self._reviewing and not self._review_cancelled and self._review_rows is not None
                    and self._exit_confirmed and not self.owns_connection and self._state['error'] is None):
                try:
                    self._reviewed_callback(copy.deepcopy(self._review_rows), copy.deepcopy(self._state['close_proof']))
                    self._update(message='15 个舵机反馈正常且扭矩关闭，维护故障复查已通过。可重新连接舵机。')
                except BaseException as error:
                    self._update(error=str(error), message=f'复查未完成：{error}')
            self._reviewing = False

    def _exit_once(self):
        if (not self._entered or self._exit_attempted or self._transport is None
                or getattr(self._transport, 'close_report', None) is not None
                or getattr(self._transport, 'closed_record', None) is not None):
            return
        self._exit_attempted = True
        self._ack(EXIT, struct.pack('<I', self._cookie))
        self._entered = False
        self._exit_confirmed = True

    def _report_fault_once(self, error):
        if self._hardware_fault and self._state['compatible'] is True and not self._fault_reported:
            self._fault_reported = True
            try:
                with self.lock:
                    self._fault_callback(str(error))
            except BaseException as callback_error:
                with self.lock:
                    self._state['message'] += f'；故障记录失败：{callback_error}'

    def _close_transport(self):
        self._update(phase='closing', connected=False, busy=True, action='exit')
        try:
            if self._transport is not None:
                proof = self._transport.close()
            elif not self._factory_attempted:
                proof = dict(serial_closed=True, process_exited=True, never_opened=True)
            else:
                proof = self._factory_failure_proof or dict(
                    serial_closed=None, process_exited=False, factory_failure_without_closure=True)
        except BaseException as error:
            proof = dict(serial_closed=None, process_exited=False, close_error=str(error))
        complete = (type(proof) is dict and proof.get('serial_closed') is True
                    and proof.get('process_exited') is True)
        with self.lock:
            self._state.update(close_proof=copy.deepcopy(proof), owns_connection=not complete,
                               phase='closed' if complete else 'closing', connected=False,
                               busy=False, action='')
            if not complete:
                self._state['message'] = '维护连接关闭尚未确认，继续保留占用，不能启动校准采集。'
            elif self._state['error'] is None:
                self._state['message'] = '维护连接已结束。'
            if complete:
                self._transport = None
                self._entered = False
