"""Bounded, fail-closed X5 motion transport and calibration runtime.

Construction never opens hardware. ``start()`` enables feedback only; every
connection begins with STOP, and torque requires an explicit ``start_mode``.
Calibration is local JSON; no servo ID, EEPROM or persistent gains are written.
"""
from __future__ import annotations

import collections
import copy
import json
import math
import os
from pathlib import Path
import secrets
import struct
import tempfile
import threading
import time

import numpy as np

from rl_core import (ACTION_ALPHA, ACTION_ALPHA_HEAD, DEFAULT_DEG, FIXED_STANCE_OVERRIDES, ImuFilter,
                     MOTOR_IDS, State, action_to_deg, build_obs, policy_profile,
                     stance_from_home)

HEADER = struct.Struct('<2sBBHI')
STATE_PACKET = struct.Struct('<IIHHHBB15H15h6fIBB')
CONFIG_PACKET = struct.Struct('<15H15b15H15H')
MAGIC, VERSION, MAX_PAYLOAD = b'XD', 1, 256
HELLO, STATE, CONFIG, ARM, TARGET, STOP = range(1, 7)
ACK, HELLO_REPLY, STATE_REPLY = 0x80, 0x81, 0x82
VOLTAGE_STATE, TELEMETRY_CAPS = 19, 20
VOLTAGE_STATE_REPLY, TELEMETRY_CAPS_REPLY = 0x93, 0x94
VOLTAGE_EXTENSION = struct.Struct('<BHH15B15I')
VOLTAGE_FRESH_MS = 1000.
ALL_MASK = (1 << 15) - 1
PERIOD, FRESH_SECONDS, HEARTBEAT_SECONDS = .02, .1, .4
TIMING_WINDOW_SECONDS, TIMING_HISTORY_LIMIT = 1., 150
# ARM performs complete gain/torque readbacks and an interleaved position
# refresh. This is only its bounded ACK wait, not a feedback or motion lease.
ARM_RESPONSE_SECONDS = .20
# Only an explicitly renewed background fixed-pose lease tolerates slower
# browser timers. UART freshness and the firmware target watchdog stay strict.
BACKGROUND_HOLD_HEARTBEAT_SECONDS = 3.
MODEL_MODES = ('walk', 'getup', 'pick', 'roulade')
EPISODE_SECONDS = {'getup': 8.0, 'pick': 4.0, 'roulade': 1.9}
MODEL_LABELS = {'walk': '行走', 'getup': '起身', 'pick': '俯身', 'roulade': '翻滚'}
SPEED_COUNTS_PER_UNIT = 50.0  # Official HD1910 present-speed conversion.
ERRORS = {1: '数据包错误', 2: '下位机状态不允许操作', 3: '下位机校准配置无效',
          4: '下位机反馈超时', 5: '目标超出关节范围'}
ARM_STAGES = {0: '请求校验', 1: '启用前检查', 2: '增益写入', 3: '增益读取',
              4: '准备后扭矩读取', 5: '准备后实际位置与 IMU 检查', 6: '当前目标写入',
              7: '使能前反馈检查', 8: '使能写入', 9: '使能后扭矩检查'}
SERVO_READ_OUTCOMES = {0: '未读取', 1: '正常应答', 2: '舵机错误应答',
                       3: '总线尚未空闲', 4: '串口写入不完整', 5: '串口硬件错误',
                       6: '截止时间内没有有效应答', 7: '寄存器应答无效'}
TARGET_STOP_PACKET = struct.Struct('<6B3H5I4BI')
STOP_CAUSES = {1: '舵机报告故障', 2: '舵机反馈或扭矩读取掉线', 3: 'IMU 读取失败',
               4: '关节采样超过 50 毫秒', 5: 'IMU 采样超过 50 毫秒',
               6: 'X5 目标指令间隔超过 250 毫秒', 7: '舵机扭矩意外关闭',
               8: '舵机目标写入失败', 9: '控制器主动保护停机'}


def target_stop_detail(payload):
    """Decode a cookie-bound first-stop record; never infer a battery fault."""
    if len(payload) != TARGET_STOP_PACKET.size:
        raise MotionError('TARGET 停机诊断字段不匹配')
    (kind, error, version, cause, first_error, imu_ok, online, fault, torque,
     at_us, position_age, imu_age, target_age, uart_errors, read_id, address,
     outcome, device_error, read_elapsed) = TARGET_STOP_PACKET.unpack(payload)
    if (kind != TARGET or error != 2 or version != 1 or cause not in STOP_CAUSES
            or first_error not in ERRORS or imu_ok not in (0, 1)
            or (online | fault | torque) & ~ALL_MASK
            or read_id not in (*MOTOR_IDS, 255) or address not in (0, 40, 50, 56, 62)
            or outcome not in SERVO_READ_OUTCOMES):
        raise MotionError('TARGET 停机诊断字段不匹配')
    detail = dict(version=version, cause=cause, reason=STOP_CAUSES[cause],
        first_error=first_error, imu_ok=bool(imu_ok), online_mask=online,
        fault_mask=fault, torque_mask=torque, stopped_at_us=at_us,
        position_age_us=position_age, imu_age_us=imu_age, target_age_us=target_age,
        uart_errors=uart_errors, read_id=None if read_id == 255 else read_id,
        read_address=address, read_outcome=outcome, device_error=device_error,
        read_elapsed_us=read_elapsed)
    text = (f'最先停机原因：{detail["reason"]}；关节采样 {position_age / 1000:.1f} ms，'
            f'IMU {imu_age / 1000:.1f} ms，目标指令间隔 {target_age / 1000:.1f} ms')
    missing = ALL_MASK & ~online
    lost_torque = ALL_MASK & ~torque
    mask = fault if cause == 1 else missing if cause == 2 else lost_torque if cause == 7 else 0
    if mask:
        text += '；涉及舵机 ' + ', '.join(str(sid) for i, sid in enumerate(MOTOR_IDS) if mask & (1 << i))
    if read_id != 255:
        read_result = SERVO_READ_OUTCOMES[outcome] if outcome else '读取尚未完成'
        text += f'；当时读取 ID {read_id} 寄存器 {address}：{read_result}'
        if device_error:
            text += f'，舵机错误 {device_error}'
    return detail, text


def arm_failure_detail(payload):
    if len(payload) != 22 or payload[0] != ARM or not payload[1] or payload[2] != 1 or payload[3] not in ARM_STAGES:
        raise MotionError('ARM 失败诊断字段不匹配')
    stage, sid, address, found, error, size, outcome, kp, actual_kp, kd, actual_kd = payload[3:14]
    if found not in (0, 1) or size > 6 or outcome not in SERVO_READ_OUTCOMES:
        raise MotionError('ARM 失败诊断字段不匹配')
    sample_age, imu_age = struct.unpack_from('<II', payload, 14)
    text = '阶段：' + ARM_STAGES[stage]
    if sid:
        text += f'，ID {sid}，寄存器 {address}，' + ('收到回复' if found else '未收到有效回复')
        if stage == 3 and found:
            text += f'，增益 {actual_kp}/{actual_kd}（期望 {kp}/{kd}），舵机错误 {error}'
        elif stage in (4, 9) and address == 40 and found and size == 1:
            text += f'，扭矩开关 {actual_kp}（期望 {kp}），舵机错误 {error}'
        elif error:
            text += f'，舵机错误 {error}'
    if stage == 3 or (stage in (4, 9) and sid):
        text += f'；读取结果：{SERVO_READ_OUTCOMES[outcome]}（代码 {outcome}）'
    return f'{text}；关节反馈 {sample_age / 1000:.1f} ms，IMU {imu_age / 1000:.1f} ms'


class MotionError(ValueError):
    pass


def crc16(data: bytes) -> int:
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def encode_frame(kind: int, seq: int, payload: bytes = b'') -> bytes:
    if not 0 <= kind <= 255 or not 0 <= seq <= 0xffffffff or len(payload) > MAX_PAYLOAD:
        raise MotionError('数据包字段无效')
    body = HEADER.pack(MAGIC, VERSION, kind, len(payload), seq) + payload
    return body + struct.pack('<H', crc16(body))


class FrameParser:
    """Streaming parser; reject bad CRC/version/length and recover from noise."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while True:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b''
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, size, seq = HEADER.unpack_from(self.buffer)
            if version != VERSION or size > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            total = HEADER.size + size + 2
            if len(self.buffer) < total:
                break
            body = bytes(self.buffer[:total - 2])
            if crc16(body) != struct.unpack_from('<H', self.buffer, total - 2)[0]:
                del self.buffer[0]
                continue
            frames.append((kind, seq, bytes(self.buffer[HEADER.size:total - 2])))
            del self.buffer[:total]
        return frames


class SerialTransport:
    """One persistent exclusive serial connection, with no uncertain retries."""
    def __init__(self, port='/dev/ttyUSB0', timeout=.16, serial_factory=None):
        self.port, self.timeout = port, timeout
        self.serial_factory = serial_factory
        self.serial = None
        self._posix_write = False
        self.seq = 0
        self.parser = FrameParser()
        self.request_timings = collections.deque(maxlen=64)
        self.last_firmware_stop = None
        self._last_target_attempt = None

    def open(self):
        factory = self.serial_factory
        native = factory is None
        if factory is None:
            import serial
            factory = serial.Serial
        # Apply DTR/RTS before opening. Connecting must not pulse reset lines.
        # Each small protocol frame gets one nonblocking write. pySerial's
        # finite write timeout can raise *after* os.write accepted the whole
        # frame if this thread is descheduled before its readiness check.
        # A short/blocked write still fails below, without replaying any bytes;
        # only the matching MCU reply confirms the request completed.
        stream = factory(port=None, baudrate=921600, timeout=.01,
                         write_timeout=0, exclusive=True, xonxoff=False,
                         rtscts=False, dsrdtr=False)
        stream.dtr = False
        stream.rts = False
        stream.port = self.port
        try:
            stream.open()
            if native and os.get_blocking(stream.fileno()):
                raise MotionError('运动串口必须以非阻塞方式打开')
        except Exception:
            stream.close()
            raise
        self.serial = stream
        self._posix_write = native

    def request(self, kind, payload=b''):
        began = time.monotonic()
        gap = (began - self._last_target_attempt) * 1000 if kind == TARGET and self._last_target_attempt is not None else None
        if kind == TARGET:
            self._last_target_attempt = began
        error = None
        try:
            return self._request_once(kind, payload)
        except BaseException as exc:
            error = str(exc) or type(exc).__name__
            raise
        finally:
            self.request_timings.append(dict(kind=kind, sequence=self.seq,
                began_monotonic=began, elapsed_ms=round((time.monotonic()-began)*1000, 3),
                target_attempt_gap_ms=round(gap, 3) if gap is not None else None, error=error))

    def timing_diagnostics(self):
        return dict(requests=copy.deepcopy(list(self.request_timings)),
                    firmware_first_stop=copy.deepcopy(self.last_firmware_stop))

    def _request_once(self, kind, payload=b''):
        if self.serial is None:
            raise MotionError('串口未连接')
        if self.seq == 0xffffffff:
            raise MotionError('串口序号已耗尽，需要重新连接')
        self.seq += 1
        frame = encode_frame(kind, self.seq, payload)
        # Use exactly one nonblocking syscall on the real POSIX UART. Even
        # pySerial's write_timeout=0 loops on EAGAIN/EINTR; a blocked UART must
        # instead fail immediately so that STOP/closure can run. Injected test
        # streams retain their explicit write interface.
        written = (os.write(self.serial.fileno(), frame) if self._posix_write
                   else self.serial.write(frame))
        if written != len(frame):
            raise MotionError('串口发送不完整，操作不会自动重试')
        response_timeout = max(self.timeout, ARM_RESPONSE_SECONDS) if kind == ARM else self.timeout
        deadline = time.monotonic() + response_timeout
        expected = {HELLO: HELLO_REPLY, STATE: STATE_REPLY, VOLTAGE_STATE: VOLTAGE_STATE_REPLY,
                    TELEMETRY_CAPS: TELEMETRY_CAPS_REPLY}.get(kind, ACK)
        while time.monotonic() < deadline:
            data = self.serial.read(min(512, max(1, self.serial.in_waiting)))
            for response_kind, seq, response in self.parser.feed(data):
                if seq != self.seq:
                    continue
                if response_kind == ACK:
                    if (len(response) not in (2, 22, TARGET_STOP_PACKET.size) or response[0] != kind
                            or (len(response) == 22 and (kind != ARM or not response[1]))
                            or (len(response) == TARGET_STOP_PACKET.size and (kind != TARGET or response[1] != 2))):
                        raise MotionError('下位机应答字段不匹配')
                    if response[1]:
                        # Only this exact legacy refusal means unsupported.
                        # Timeouts/corruption/other errors never downgrade.
                        if kind == TELEMETRY_CAPS and response == bytes([TELEMETRY_CAPS, 1]):
                            return None
                        name = {HELLO: 'HELLO', STATE: 'STATE', CONFIG: 'CONFIG',
                                ARM: 'ARM', TARGET: 'TARGET', STOP: 'STOP'}.get(kind, str(kind))
                        if len(response) == TARGET_STOP_PACKET.size:
                            self.last_firmware_stop, stop_text = target_stop_detail(response)
                            detail = '；' + stop_text
                        else:
                            detail = '；' + arm_failure_detail(response) if len(response) == 22 else ''
                        raise MotionError(f"{ERRORS.get(response[1], '下位机拒绝操作')}（指令 {name}，错误码 {response[1]}）{detail}")
                    if expected != ACK:
                        raise MotionError('下位机应答类型不匹配')
                    if kind == ARM:
                        self.last_firmware_stop = None
                        self._last_target_attempt = None
                    return response
                if response_kind != expected:
                    raise MotionError('下位机应答类型不匹配')
                if kind == HELLO and response != b'XGODUCK_MOTION_1':
                    raise MotionError('串口不是运动控制固件')
                return response
        raise MotionError('运动固件响应超时；检查 ESP32 固件与串口占用')

    def negotiate_voltage(self):
        payload = self.request(TELEMETRY_CAPS)
        if payload == b'XGODUCK_TELEMETRY_1':
            return True
        if payload is None:
            # Legacy unknown commands latch BadPacket; clear only this probe
            # with a safe STOP before reading the real connection state.
            self.request(STOP)
            return False
        raise MotionError('实时电压能力应答无效')

    def close(self):
        self._posix_write = False
        if self.serial is not None:
            self.serial.close()
            self.serial = None


def decode_state(payload):
    if len(payload) not in (STATE_PACKET.size, STATE_PACKET.size + VOLTAGE_EXTENSION.size):
        raise MotionError('下位机状态长度不匹配')
    values = STATE_PACKET.unpack(payload[:STATE_PACKET.size])
    sample_us, imu_us, online, fault, torque, armed, imu_ok = values[:7]
    pos, velocity = list(values[7:22]), list(values[22:37])
    acc, gyro = list(values[37:40]), list(values[40:43])
    cookie, error, configured = values[43:]
    if ((online | fault | torque) & ~ALL_MASK or armed not in (0, 1) or
            imu_ok not in (0, 1) or configured not in (0, 1) or
            any(not math.isfinite(value) for value in acc + gyro) or
            any(online & (1 << i) and not 0 <= p <= 4095 for i, p in enumerate(pos))):
        raise MotionError('下位机状态字段无效')
    voltage = None
    if len(payload) != STATE_PACKET.size:
        version, seen, valid, *rest = VOLTAGE_EXTENSION.unpack(payload[STATE_PACKET.size:])
        raw, ages = rest[:15], rest[15:]
        if (version != 1 or (seen | valid) & ~ALL_MASK or valid & ~seen or
                any((seen & (1 << i) and (raw[i] == 0 or ages[i] == 0xffffffff)) or
                    (not seen & (1 << i) and (raw[i] != 0 or ages[i] != 0xffffffff)) or
                    (valid & (1 << i) and ages[i] > VOLTAGE_FRESH_MS * 1000) for i in range(15))):
            raise MotionError('实时电压状态字段无效')
        voltage = dict(seen_mask=seen, valid_mask=valid, raw=raw, ages_us=ages)
    return dict(sample_us=sample_us, imu_us=imu_us, online_mask=online,
                fault_mask=fault, torque_mask=torque, armed=bool(armed), imu_ok=bool(imu_ok),
                positions=pos, velocities=velocity, acc=acc, gyro=gyro, cookie=cookie,
                last_error=error, configured=bool(configured), voltage=voltage)


def _axis_matrix(axes):
    if not isinstance(axes, list) or len(axes) != 3:
        raise MotionError('IMU 需要三个机身轴映射')
    matrix = np.zeros((3, 3), dtype=np.float64)
    canonical = []
    for row, axis in enumerate(axes):
        if not isinstance(axis, str) or axis not in ('x', 'y', 'z', '+x', '+y', '+z', '-x', '-y', '-z'):
            raise MotionError('IMU 轴只能为 x、y、z 或其负方向')
        matrix[row, 'xyz'.index(axis[-1])] = -1 if axis.startswith('-') else 1
        canonical.append(axis.lstrip('+'))
    if not np.array_equal(matrix @ matrix.T, np.eye(3)) or round(np.linalg.det(matrix)) != 1:
        raise MotionError('IMU 映射必须使用三个不同轴，并组成右手坐标系')
    return canonical, matrix


def _default_config():
    return dict(version=1, mechanical_reference_confirmed=False,
                joints=[dict(id=sid, zero=None, sign=-1, min=None, max=None,
                             direction_confirmed=False) for sid in MOTOR_IDS],
                imu=dict(axes=['x', 'y', 'z'], axes_confirmed=False,
                         calibrated=False, bias_dps=[0., 0., 0.]))


def _joint_ready(joint):
    vals = [joint.get(key) for key in ('min', 'zero', 'max')]
    return (all(isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 4095 for v in vals)
            and vals[0] < vals[2] and vals[0] <= vals[1] <= vals[2] and joint.get('sign') in (-1, 1)
            and joint.get('direction_confirmed') is True)


class MotionController:
    def __init__(self, sessions, config_path=None, port='/dev/ttyUSB0', *,
                 transport_factory=None, clock=time.monotonic, require_head_clearance=False,
                 allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=0):
        if (type(hold_hip_pitch_bias_deg) not in (int, float)
                or not -3 <= hold_hip_pitch_bias_deg <= 3
                or not math.isfinite(hold_hip_pitch_bias_deg)):
            raise MotionError('固定 HOLD 髋偏置必须为 -3 至 3° 的有限数值')
        self._hold_hip_pitch_bias_deg = float(hold_hip_pitch_bias_deg)
        self.sessions = sessions
        try:
            self._policy_profiles = {name: policy_profile(session) for name, session in sessions.items()}
        except (TypeError, ValueError, AttributeError) as error:
            raise MotionError('模型元数据校验未通过：' + str(error)) from error
        self._stand_deg = stance_from_home(self._policy_profiles['walk']['home_rad']) if 'walk' in self._policy_profiles else None
        self._require_head_clearance = require_head_clearance is True
        # An explicit, separately reviewed actuator adaptation. Merely capturing
        # a clear head pose never authorizes it; native callers default to off.
        self._allow_fixed_head_walk = allow_fixed_head_walk is True
        self._hold_head_deg = None
        self.port = port
        self.config_path = Path(config_path or Path(__file__).with_name('calibration.json'))
        self._factory = transport_factory or SerialTransport
        self._clock = clock
        self._lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._closed = threading.Event()
        self._thread = None
        self._transport = None
        self._connected = False
        self._state = None
        self._state_request = STATE
        self._state_request_sequence = None
        self._voltage_received = None
        self._last_sample = None
        self._sample_received = None
        self._filter = ImuFilter()
        self._filtered = None
        self._config = _default_config()
        self._config_dirty = True
        self._generation = 0
        self._mode = 'off'
        self._error = None
        self._pending_stop = False
        self._cookie = 0
        self._heartbeat = None
        self._background_hold = False
        self._hold_begin = None
        self._hold_initial = None
        self._command = np.zeros(3, dtype=np.float32)
        self._previous_action = np.zeros(14, dtype=np.float32)
        self._smooth_action = np.zeros(14, dtype=np.float32)
        self._last_action = None
        self._last_policy_target = None
        self._last_policy_target_rejections = []
        self._last_executed_target = None
        self._last_inference_ms = None
        self._target_rejection = None
        self._preview_model = 'walk'
        self._episode_started = None
        self._episode_upright_since = None
        self._episode_result = None
        self._imu_collection = None
        self._state_times = collections.deque(maxlen=150)
        self._target_times = collections.deque(maxlen=150)
        self._state_receive_times = collections.deque(maxlen=TIMING_HISTORY_LIMIT)
        self._state_receive_count = 0
        self._state_sample_count = 0
        self._timing_records = {name: collections.deque(maxlen=TIMING_HISTORY_LIMIT)
                                for name in ('state_request', 'target_request', 'cycle')}
        self._load_config()

    def _load_config(self):
        if not self.config_path.exists():
            return
        try:
            cfg = json.loads(self.config_path.read_text())
            if cfg.get('version') != 1 or not isinstance(cfg.get('mechanical_reference_confirmed'), bool):
                raise ValueError('校准文件版本无效')
            if [j['id'] for j in cfg['joints']] != MOTOR_IDS:
                raise ValueError('校准文件关节顺序无效')
            for j in cfg['joints']:
                if j.get('sign') not in (-1, 1) or isinstance(j.get('sign'), bool):
                    raise ValueError('校准文件方向无效')
                if not isinstance(j.get('direction_confirmed'), bool):
                    raise ValueError('校准文件确认字段无效')
                for key in ('min', 'zero', 'max'):
                    v = j.get(key)
                    if v is not None and (isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 4095):
                        raise ValueError('校准文件编码器值无效')
            imu = cfg['imu']
            imu['axes'], _ = _axis_matrix(imu['axes'])
            if any(not isinstance(imu.get(k), bool) for k in ('axes_confirmed', 'calibrated')):
                raise ValueError('校准文件 IMU 确认字段无效')
            bias = imu['bias_dps']
            if len(bias) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                                    not math.isfinite(v) or abs(v) > 5 for v in bias):
                raise ValueError('校准文件 IMU 偏置无效')
            if imu['calibrated'] and not imu['axes_confirmed']:
                raise ValueError('IMU 轴向尚未确认')
            self._config = cfg
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self._error = '校准文件未载入：' + str(exc)

    def _save_locked(self):
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.calibration-', suffix='.json', dir=self.config_path.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(self._config, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.config_path)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def _joints_ready_locked(self):
        return all(_joint_ready(j) for j in self._config['joints'])

    def _calibrated_locked(self):
        imu = self._config['imu']
        return (self._config['mechanical_reference_confirmed'] and self._joints_ready_locked()
                and imu['axes_confirmed'] and imu['calibrated'])

    def start(self):
        with self._lock:
            if self._closed.is_set():
                raise MotionError('控制器已关闭')
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name='duck-motion', daemon=True)
                self._thread.start()
        return self.status()

    def _request(self, kind, payload=b''):
        with self._io_lock:
            if self._transport is None:
                raise MotionError('运动串口未连接')
            return self._request_transport(self._transport, kind, payload)

    def _record_timing(self, name, began, failed):
        # Each history has one serialized writer: requests use the existing
        # UART lock and cycles use the sole control thread. Immutable records
        # let status copy this bounded evidence without adding a control lock.
        records = self._timing_records[name]
        previous = records[-1] if records else None
        ended = self._clock()
        records.append((ended, max(0., ended - began) * 1000, bool(failed),
                        (previous[3] if previous else 0) + 1,
                        (previous[4] if previous else 0) + int(bool(failed))))

    def _request_transport(self, transport, kind, payload=b''):
        name = ('state_request' if kind in (STATE, VOLTAGE_STATE) else
                'target_request' if kind == TARGET else None)
        if name is None:
            return transport.request(kind, payload)
        began, failed = self._clock(), True
        try:
            response = transport.request(kind, payload)
            if kind in (STATE, VOLTAGE_STATE):
                sequence = getattr(transport, 'seq', None)
                self._state_request_sequence = sequence if type(sequence) is int else None
            failed = False
            return response
        finally:
            self._record_timing(name, began, failed)

    def _read_state(self):
        payload = self._request(self._state_request)
        expected = STATE_PACKET.size + (VOLTAGE_EXTENSION.size if self._state_request == VOLTAGE_STATE else 0)
        if len(payload) != expected:
            raise MotionError('协商后的实时状态长度不匹配')
        return decode_state(payload)

    def _connect_once(self):
        transport = self._factory(self.port)
        try:
            transport.open()
            transport.request(STOP)
            if transport.request(HELLO) != b'XGODUCK_MOTION_1':
                raise MotionError('串口固件协议不匹配')
            voltage_supported = transport.negotiate_voltage() if hasattr(transport, 'negotiate_voltage') else False
            state_request = VOLTAGE_STATE if voltage_supported else STATE
            payload = self._request_transport(transport, state_request)
            if len(payload) != STATE_PACKET.size + (VOLTAGE_EXTENSION.size if voltage_supported else 0):
                raise MotionError('协商后的实时状态长度不匹配')
            state = decode_state(payload)
            with self._lock:
                if self._closed.is_set():
                    transport.close()
                    return
                self._transport = transport
                self._state_request = state_request
                self._connected = True
                self._mode = 'off'
                self._hold_head_deg = None
                self._last_policy_target = None
                self._last_policy_target_rejections = []
                self._last_executed_target = None
                self._cookie = 0
                self._generation += 1
                self._config_dirty = True
                self._pending_stop = False
                self._filter = ImuFilter()
                self._last_sample = None
                self._error = None
                self._accept_state_locked(state)
                self._apply_config_locked()
        except Exception:
            transport.close()
            raise

    def _disconnect(self, exc):
        with self._lock:
            self._generation += 1
            self._mode = 'off'
            self._connected = False
            self._cookie = 0
            self._last_action = None
            self._hold_head_deg = None
            self._heartbeat = None
            self._background_hold = False
            self._imu_collection = None
            self._error = str(exc)
            transport, self._transport = self._transport, None
        if transport is not None:
            with self._io_lock:
                transport.close()

    def _apply_config_locked(self):
        if (not self._connected or not self._joints_ready_locked() or self._mode != 'off'
                or self._state['armed'] or self._state['torque_mask']):
            return
        if not self._config_dirty and self._state['configured']:
            return
        joints = self._config['joints']
        payload = CONFIG_PACKET.pack(*[j['zero'] for j in joints], *[j['sign'] for j in joints],
                                     *[j['min'] for j in joints], *[j['max'] for j in joints])
        self._request(CONFIG, payload)
        self._config_dirty = False
        self._accept_state_locked(self._read_state())

    def _run(self):
        retry_at = 0.
        while not self._closed.is_set():
            started = self._clock()
            try:
                with self._lock:
                    connected = self._connected
                if not connected:
                    if started >= retry_at:
                        self._connect_once()
                else:
                    self._cycle()
            except Exception as exc:
                # A rejected target/model output must remove torque immediately;
                # a broken transport is additionally covered by the MCU watchdog.
                with self._lock:
                    self._stop_locked(str(exc))
                    try:
                        self._service_stop_locked()
                    except Exception:
                        pass
                self._disconnect(exc)
                retry_at = self._clock() + 1.
            self._closed.wait(max(.001, PERIOD - (self._clock() - started)))
        self._disconnect('控制器已关闭')

    def _age_locked(self):
        return math.inf if self._sample_received is None else max(0., self._clock() - self._sample_received)

    def _imu_age_locked(self):
        if self._state is None:
            return math.inf
        delta = (self._state['sample_us'] - self._state['imu_us']) & 0xffffffff
        if delta >= 0x80000000:
            delta -= 0x100000000
        # The IMU can be sampled just after the latest completed servo cycle.
        # Compare modulo uint32 without interpreting that as a 71-minute gap.
        return self._age_locked() + max(0, delta) / 1e6

    def _accept_state_locked(self, state):
        now = self._clock()
        self._state_receive_times.append(now)
        self._state_receive_count += 1
        self._state = state
        self._voltage_received = now
        if state['sample_us'] == self._last_sample:
            return
        dt = PERIOD if self._last_sample is None else ((state['sample_us'] - self._last_sample) & 0xffffffff) / 1e6
        self._last_sample = state['sample_us']
        self._sample_received = now
        self._state_times.append(now)
        self._state_sample_count += 1
        self._filtered = None
        joints = self._config['joints']
        if (all(j['zero'] is not None for j in joints) and state['online_mask'] == ALL_MASK
                and state['imu_ok'] and self._imu_age_locked() < FRESH_SECONDS):
            _, matrix = _axis_matrix(self._config['imu']['axes'])
            acc = matrix @ np.asarray(state['acc'])
            gyro = matrix @ (np.asarray(state['gyro']) - self._config['imu']['bias_dps'])
            angles = [(p - j['zero']) * 360. / 4095 * j['sign']
                      for p, j in zip(state['positions'], joints)]
            vels = [v * SPEED_COUNTS_PER_UNIT * 360. / 4095 * j['sign']
                    for v, j in zip(state['velocities'], joints)]
            # ImuFilter itself maps [z,-x,-y]. Feed its inverse because the
            # calibrated matrix already supplies the policy's body axes.
            # LSM6DSR reports specific force: level is +9.8 along body +Z.
            # The policy instead consumes the world gravity vector in body axes.
            sensor_acc, sensor_gyro = [acc[1], acc[2], -acc[0]], [-gyro[1], -gyro[2], gyro[0]]
            self._filtered = self._filter.update(State(sensor_acc, sensor_gyro, angles, vels), dt)
        self._collect_imu_locked()

    def _require_off_locked(self):
        if self._mode != 'off' or self._state and (self._state['armed'] or self._state['torque_mask']):
            raise MotionError('先停止运动并确认全部舵机扭矩关闭')

    def _require_feedback_locked(self):
        if not self._connected or self._state is None:
            raise MotionError('ESP32 运动固件尚未连接')
        if self._age_locked() >= FRESH_SECONDS:
            raise MotionError('舵机反馈已过期')

    def capture(self, sid, field):
        if field not in ('zero', 'min', 'max'):
            raise MotionError('只能记录 zero、min 或 max')
        if isinstance(sid, bool) or sid not in MOTOR_IDS:
            raise MotionError('未知关节 ID')
        with self._lock:
            self._require_off_locked()
            self._require_feedback_locked()
            i = MOTOR_IDS.index(sid)
            if not self._state['online_mask'] & (1 << i) or self._state['fault_mask'] & (1 << i):
                raise MotionError('所选舵机离线或报告故障')
            self._config['joints'][i][field] = int(self._state['positions'][i])
            self._config_dirty = True
            self._save_locked()
            self._filter = ImuFilter()
            self._filtered = None
        return self.status()

    def configure_joint(self, sid, sign, direction_confirmed):
        if isinstance(sid, bool) or sid not in MOTOR_IDS:
            raise MotionError('未知关节 ID')
        if isinstance(sign, bool) or sign not in (-1, 1) or not isinstance(direction_confirmed, bool):
            raise MotionError('方向必须为 -1 或 1，并明确确认转向')
        with self._lock:
            self._require_off_locked()
            j = self._config['joints'][MOTOR_IDS.index(sid)]
            j['sign'], j['direction_confirmed'] = int(sign), direction_confirmed
            self._config_dirty = True
            self._save_locked()
            self._filter = ImuFilter()
            self._filtered = None
        return self.status()

    def configure_reference(self, confirmed):
        if not isinstance(confirmed, bool):
            raise MotionError('机械零姿态需要明确确认')
        with self._lock:
            self._require_off_locked()
            self._config['mechanical_reference_confirmed'] = confirmed
            self._save_locked()
        return self.status()

    def set_imu_axes(self, axes, axes_confirmed=False):
        axes, _ = _axis_matrix(axes)
        if not isinstance(axes_confirmed, bool):
            raise MotionError('IMU 轴向需要明确确认')
        with self._lock:
            self._require_off_locked()
            self._config['imu'] = dict(axes=axes, axes_confirmed=axes_confirmed,
                                       calibrated=False, bias_dps=[0., 0., 0.])
            self._imu_collection = None
            self._save_locked()
            self._filter = ImuFilter()
            self._filtered = None
        return self.status()

    def calibrate_imu(self):
        with self._lock:
            self._require_off_locked()
            self._require_feedback_locked()
            if not self._config['imu']['axes_confirmed']:
                raise MotionError('先确认 IMU 各轴相对于机身的安装方向')
            if not self._state['imu_ok'] or self._imu_age_locked() >= FRESH_SECONDS:
                raise MotionError('IMU 采样尚未就绪')
            self._config['imu']['calibrated'] = False
            self._save_locked()
            self._imu_collection = dict(begin=self._clock(), samples=[], last_us=None)
            self._error = None
        return self.status()

    def _collect_imu_locked(self):
        collection = self._imu_collection
        if collection is None:
            return
        state = self._state
        if state['armed'] or state['torque_mask'] or not state['imu_ok'] or self._imu_age_locked() >= FRESH_SECONDS:
            self._imu_collection = None
            self._error = 'IMU 校准中断：需要全部扭矩关闭且采样连续'
            return
        elapsed = self._clock() - collection['begin']
        if elapsed > 6:
            self._imu_collection = None
            self._error = 'IMU 校准超时：没有足够的独立采样'
            return
        if state['imu_us'] != collection['last_us']:
            collection['last_us'] = state['imu_us']
            collection['samples'].append(state['acc'] + state['gyro'])
        if len(collection['samples']) < 100 or elapsed < 1.8:
            return
        samples = np.asarray(collection['samples'])
        _, matrix = _axis_matrix(self._config['imu']['axes'])
        acc_mean = matrix @ samples[:, :3].mean(axis=0)
        gyro_mean = samples[:, 3:].mean(axis=0)
        norm = float(np.linalg.norm(acc_mean))
        level = norm > 0 and math.degrees(math.acos(float(np.clip(acc_mean[2] / norm, -1, 1)))) < 10
        stationary = (np.max(samples[:, :3].std(axis=0)) < .3 and
                      np.max(samples[:, 3:].std(axis=0)) < .5 and
                      np.linalg.norm(gyro_mean) < 3)
        if not 9.0 <= norm <= 10.6 or not level or not stationary:
            self._error = 'IMU 校准失败：请保持机身水平静止，并核对轴向（水平加速度应为机身 +Z 约 9.8）'
        else:
            self._config['imu']['bias_dps'] = gyro_mean.tolist()
            self._config['imu']['calibrated'] = True
            self._save_locked()
            self._filter = ImuFilter()
            self._error = None
        self._imu_collection = None

    def _tilt_locked(self):
        if self._filtered is None:
            return None
        return math.degrees(math.acos(float(np.clip(-self._filtered.grav[2], -1, 1))))

    @property
    def hold_hip_pitch_bias_deg(self):
        """Constructor-only HOLD profile; never a calibration or policy HOME."""
        return self._hold_hip_pitch_bias_deg

    def _hold_ready_locked(self):
        if (self._mode != 'hold' or self._hold_begin is None or
                self._clock() - self._hold_begin < 3 or self._filtered is None or
                self._stand_deg is None):
            return False
        tilt = self._tilt_locked()
        return (tilt is not None and math.isfinite(tilt) and tilt < 15 and
                max(abs(a - b) for a, b in zip(self._filtered.angles, self._fixed_stance_locked())) < 5)

    def _fixed_stance_locked(self):
        target = copy.deepcopy(self._stand_deg)
        if target is not None:
            for sid, angle in FIXED_STANCE_OVERRIDES.items():
                target[MOTOR_IDS.index(sid)] = angle
            # An additional fixed HOLD experiment, relative to the existing
            # paired hip/ankle stance. Ankles, model HOME and calibration stay
            # unchanged; nonzero bias blocks every live model handoff below.
            target[MOTOR_IDS.index(12)] += self._hold_hip_pitch_bias_deg
            target[MOTOR_IDS.index(22)] -= self._hold_hip_pitch_bias_deg
        if target is not None and self._hold_head_deg is not None:
            for sid, angle in self._hold_head_deg.items():
                target[MOTOR_IDS.index(sid)] = angle
        return target

    def _walk_head_command_locked(self):
        # The trained input is a four-joint delta from policy HOME, in radians.
        # Preview and any separately authorized live walk must use the same
        # command. This never changes HOME, overwrites outputs or grants motion.
        if self._hold_head_deg is None or 'walk' not in self._policy_profiles:
            return None
        angles = [self._hold_head_deg[sid] for sid in (30, 31, 32, 33)]
        return np.asarray(np.deg2rad(angles) - self._policy_profiles['walk']['home_rad'][5:9],
                          dtype=np.float32)

    def _fixed_head_walk_ready_locked(self):
        return (self._allow_fixed_head_walk and self._hold_hip_pitch_bias_deg == 0 and self._connected and
                self._hold_head_deg is not None)

    def _execution_target_locked(self, mode, policy_target):
        """Keep policy observations/history intact; adapt only physical head targets."""
        target = list(policy_target)
        if mode == 'walk' and self._fixed_head_walk_ready_locked():
            for sid, angle in self._hold_head_deg.items():
                target[MOTOR_IDS.index(sid)] = angle
        return target

    def _record_policy_target_locked(self, target):
        # Raw, complete policy output remains visible even when an unexecuted
        # head target would exceed its range. Leg targets are never clamped.
        self._last_policy_target = [float(v) for v in target]
        self._last_policy_target_rejections = []
        for angle, joint in zip(target, self._config['joints']):
            value = int(round(joint['zero'] + angle / 360 * 4095 * joint['sign']))
            if not joint['min'] <= value <= joint['max']:
                self._last_policy_target_rejections.append(dict(id=joint['id'],
                    target_angle_deg=float(angle), target_position=value,
                    min=joint['min'], max=joint['max']))

    def capture_head_clearance(self, *, clearance_confirmed=False, expected_generation=None):
        """Read four unpowered head joints; never change zeros or send a target.

        This physical confirmation is valid only for this connection. Native
        policy actions remain blocked unless a separately reviewed fixed-head
        walk actuator adaptation was explicitly enabled by the constructor.
        """
        with self._lock:
            if type(expected_generation) is not int or expected_generation != self._generation:
                raise MotionError('状态已变更，请重新检查后记录头颈位置')
            if clearance_confirmed is not True:
                raise MotionError('请先确认头颈与板卡、线束有活动间隙')
            self._require_feedback_locked()
            self._require_off_locked()
            if self._state['armed'] or self._state['torque_mask']:
                raise MotionError('记录头颈避让位置需要全部扭矩关闭')
            if self._pending_stop or self._state['last_error']:
                raise MotionError('请先完成停止确认并处理下位机故障')
            if not self._calibrated_locked() or self._filtered is None or self._stand_deg is None:
                raise MotionError('需要完整校准和有效的关节反馈')
            if self._state['online_mask'] != ALL_MASK or self._state['fault_mask']:
                raise MotionError('记录头颈位置需要 15 个舵机全部在线且无故障')
            candidate = dict(zip((30, 31, 32, 33), self._filtered.angles[10:14]))
            target = list(self._stand_deg)
            for sid, angle in candidate.items():
                target[MOTOR_IDS.index(sid)] = angle
            self._raw_targets_locked(target, mode='hold')
            self._hold_head_deg = candidate
            self._require_head_clearance = True
            self._error = None
            self._generation += 1
        return self.status()

    def _raw_targets_locked(self, angles, *, mode=None):
        if len(angles) != 15 or not all(math.isfinite(v) for v in angles):
            raise MotionError('目标角度无效')
        raw = []
        for angle, joint in zip(angles, self._config['joints']):
            value = int(round(joint['zero'] + angle / 360 * 4095 * joint['sign']))
            if not joint['min'] <= value <= joint['max']:
                self._target_rejection = dict(id=joint['id'], mode=mode or self._mode,
                    target_angle_deg=float(angle), target_position=value,
                    min=joint['min'], max=joint['max'])
                raise MotionError(f"ID {joint['id']} 目标超过已记录的机械范围"
                    f"（目标 {value} ticks / {angle:.2f}°；允许 {joint['min']}–{joint['max']} ticks）")
            raw.append(value)
        return raw

    def _motion_gate_locked(self, mode=None):
        self._require_feedback_locked()
        if not self._calibrated_locked():
            raise MotionError('先完成机械零姿态确认、15 个关节零点/范围/转向和 IMU 校准')
        state = self._state
        if state['online_mask'] != ALL_MASK or state['fault_mask']:
            raise MotionError('需要 15 个舵机全部在线且无故障')
        if not state['imu_ok'] or self._imu_age_locked() >= FRESH_SECONDS or self._filtered is None:
            raise MotionError('需要新鲜有效的 IMU 与关节反馈')
        selected = self._mode if mode is None else mode
        if selected in MODEL_MODES and self._hold_hip_pitch_bias_deg != 0:
            raise MotionError('当前为偏置固定 HOLD 试验，模型接管尚未审查')
        if selected == 'hold' and self._require_head_clearance and self._hold_head_deg is None:
            raise MotionError('先在扭矩关闭时记录头颈避让位置')
        if selected in MODEL_MODES and self._require_head_clearance:
            if selected != 'walk' or not self._fixed_head_walk_ready_locked():
                raise MotionError('当前头颈避让仅用于固定站姿；模型动作的机械间隙尚未验证')
        if selected == 'walk' and self._fixed_head_walk_ready_locked():
            if max(abs(self._filtered.angles[MOTOR_IDS.index(sid)] - angle)
                   for sid, angle in self._hold_head_deg.items()) >= 2:
                raise MotionError('固定头颈适配：实际头颈偏离记录位置达到 2°，已停止')
        # Only explicitly started, bounded getup/roulade may invert the body.
        # Feedback, torque, per-joint actual/target limits and STOP remain the
        # same. Normal stand/walk never inherit these behavior tilt limits.
        limit = (15 if selected == 'hold' else 180 if selected in ('getup', 'roulade')
                 else 55 if selected == 'pick' else 40)
        if self._tilt_locked() >= limit and limit != 180:
            raise MotionError(f'机身倾角超过 {limit}°，请先支撑并扶正')
        if self._stand_deg is None:
            raise MotionError('站姿需要已验证默认姿态的行走模型')
        target = self._fixed_stance_locked() if selected == 'hold' else self._execution_target_locked(selected, self._stand_deg)
        self._raw_targets_locked(target)
        for p, j in zip(state['positions'], self._config['joints']):
            if not j['min'] <= p <= j['max']:
                raise MotionError(f"ID {j['id']} 实际位置超出记录范围")

    def _model_gate_locked(self, mode):
        self._require_feedback_locked()
        self._motion_gate_locked(mode)
        if mode not in self.sessions:
            raise MotionError(MODEL_LABELS[mode] + '模型未加载')
        if mode in ('walk', 'pick', 'roulade'):
            if not self._hold_ready_locked():
                raise MotionError('先启动站姿，并等待实际关节接近默认姿态')
            if self._tilt_locked() >= 15:
                raise MotionError('动作开始前机身需要保持水平（倾角小于 15°）')
        elif self._mode not in ('off', 'hold') or self._mode == 'hold' and not self._hold_ready_locked():
            raise MotionError('先停止当前动作，或等待站姿到位，再启动起身模型')

    def start_mode(self, mode, support_confirmed=False, *, expected_generation=None,
                   preview_model='walk', behavior_confirmed=False):
        if mode not in ('shadow', 'hold') + MODEL_MODES:
            raise MotionError('未知动作模式；停止请使用停止按钮')
        preflight_generation, preflight_began, preflight_target = None, None, None
        if mode in MODEL_MODES:
            with self._lock:
                if expected_generation is not None and (type(expected_generation) is not int or
                                                        expected_generation != self._generation):
                    raise MotionError('状态已变更；旧的启动请求已取消，请重新检查后启动')
                if support_confirmed is not True:
                    raise MotionError('启动动作前需要明确确认鸭子已获得安全支撑')
                if mode in ('getup', 'roulade') and behavior_confirmed is not True:
                    raise MotionError('起身或翻滚需要确认动作区域清空并已有防跌保护')
                self._model_gate_locked(mode)
                st = copy.deepcopy(self._filtered)
                head_command = self._walk_head_command_locked() if mode == 'walk' else None
                preflight_generation, preflight_began = self._generation, self._clock()
            # Never hold the controller lock during inference: a STOP arriving
            # here must invalidate this start before any ARM or target write.
            preflight_target, _, _, _ = self._infer_policy_target(mode, st,
                np.zeros(14, np.float32), np.zeros(3, np.float32), np.zeros(14, np.float32), 0.,
                head_command=head_command)
        with self._lock:
            if preflight_generation is not None and preflight_generation != self._generation:
                raise MotionError('状态已变更；旧的启动请求已取消，请重新检查后启动')
            if preflight_began is not None and self._clock() - preflight_began >= FRESH_SECONDS:
                raise MotionError('启动前模型检查耗时过长，请重新检查状态后启动')
            if expected_generation is not None and (type(expected_generation) is not int or
                                                    expected_generation != self._generation):
                raise MotionError('状态已变更；旧的启动请求已取消，请重新检查后启动')
            self._require_feedback_locked()
            if mode == 'hold' and self._hold_head_deg is not None:
                if max(abs(self._filtered.angles[MOTOR_IDS.index(sid)] - angle)
                       for sid, angle in self._hold_head_deg.items()) >= 2:
                    raise MotionError('头颈已离开记录位置；请停止后重新确认间隙并记录')
            if mode == 'shadow':
                self._require_off_locked()
                if preview_model not in MODEL_MODES or preview_model not in self.sessions:
                    raise MotionError('所选预览模型未加载')
            else:
                if support_confirmed is not True:
                    raise MotionError('启动动作前需要明确确认鸭子已获得安全支撑')
                if mode in ('getup', 'roulade') and behavior_confirmed is not True:
                    raise MotionError('起身或翻滚需要确认动作区域清空并已有防跌保护')
                self._motion_gate_locked(mode)
                if mode in MODEL_MODES:
                    self._model_gate_locked(mode)
                    # Evaluate the first policy frame against this robot's
                    # saved limits before ARM or leaving a held stance. This
                    # consumes no motor command or policy history; every later
                    # frame is still checked against fresh state and limits.
                    self._record_policy_target_locked(preflight_target)
                    preflight_execution = self._execution_target_locked(mode, preflight_target)
                    try:
                        self._raw_targets_locked(preflight_execution, mode=mode)
                    except MotionError as error:
                        raise MotionError(MODEL_LABELS[mode] + '启动前检查未通过：' + str(error)) from error
                elif self._mode not in ('off', 'walk', 'hold'):
                    raise MotionError('先退出真实反馈测试')
                if self._mode == 'off':
                    self._apply_config_locked()
                    if not self._state['configured']:
                        raise MotionError('下位机尚未接收有效关节配置')
                    if self._state['armed'] or self._state['torque_mask']:
                        raise MotionError('下位机必须处于停止状态')
                    cookie = secrets.randbits(32) or 1
                    try:
                        self._request(ARM, struct.pack('<I', cookie))
                        self._accept_state_locked(self._read_state())
                        if (not self._state['armed'] or self._state['cookie'] != cookie or
                                self._state['torque_mask'] != ALL_MASK):
                            raise MotionError('下位机未确认启用，已停止')
                    except Exception as error:
                        self._stop_locked('启用失败：' + str(error))
                        try:
                            self._service_stop_locked()
                        except Exception as stop_error:
                            self._disconnect('启用失败：' + str(error) + '；停止确认失败：' + str(stop_error))
                        raise
                    self._cookie = cookie
            self._generation += 1
            self._mode = mode
            self._heartbeat = self._clock()
            self._background_hold = False
            self._command.fill(0)
            self._previous_action.fill(0)
            self._smooth_action.fill(0)
            self._last_action = None
            if mode not in MODEL_MODES:
                self._last_policy_target = None
                self._last_policy_target_rejections = []
            self._last_executed_target = None
            self._last_inference_ms = None
            self._target_rejection = None
            self._error = None
            self._preview_model = preview_model if mode == 'shadow' else 'walk'
            self._episode_started = self._clock() if mode in EPISODE_SECONDS else None
            self._episode_upright_since = None
            self._episode_result = None
            if mode == 'hold':
                self._hold_begin = self._clock()
                self._hold_initial = list(self._filtered.angles)
        return self.status()

    def command(self, forward, side, yaw):
        values = [forward, side, yaw]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise MotionError('速度指令需要有限数字')
        with self._lock:
            if self._mode not in ('walk', 'shadow'):
                raise MotionError('先进入行走或真实反馈测试模式')
            if (self._mode == 'walk' and self._fixed_head_walk_ready_locked()
                    and any(v != 0 for v in values)):
                raise MotionError('固定头颈实验目前仅允许零速度站立平衡；行走尚未验证')
            self._command[:] = np.clip(values, [-.2, -.1, -.5], [.2, .1, .5])
        return self.status()

    def _heartbeat_timeout_locked(self):
        return BACKGROUND_HOLD_HEARTBEAT_SECONDS if self._mode == 'hold' and self._background_hold else HEARTBEAT_SECONDS

    def _heartbeat_expired_locked(self):
        return self._mode != 'hold' and (self._heartbeat is None or
            self._clock() - self._heartbeat >= self._heartbeat_timeout_locked())

    def heartbeat(self, *, background=False, expected_generation=None, minimal=False):
        with self._lock:
            if expected_generation is not None and (type(expected_generation) is not int or
                                                    expected_generation != self._generation):
                raise MotionError('Motion state changed; refresh before starting')
            if self._mode != 'off':
                self._heartbeat = self._clock()
                self._background_hold = background is True and self._mode == 'hold'
            if minimal is True:
                return dict(generation=self._generation, mode=self._mode, connected=self._connected)
        return self.status()

    def _stop_locked(self, reason=None):
        self._generation += 1
        self._mode = 'off'
        self._cookie = 0
        self._heartbeat = None
        self._background_hold = False
        self._command.fill(0)
        self._last_action = None
        self._last_inference_ms = None
        self._episode_started = None
        self._episode_upright_since = None
        self._imu_collection = None
        self._pending_stop = True
        self._error = reason

    def _service_stop_locked(self):
        if self._pending_stop and self._connected:
            self._request(STOP)
            self._pending_stop = False

    def stop(self):
        with self._lock:
            self._stop_locked()
            try:
                self._service_stop_locked()
            except Exception as exc:
                self._disconnect(exc)
        return self.status()

    def _cycle(self):
        began, failed = self._clock(), True
        try:
            self._cycle_once()
            failed = False
        finally:
            self._record_timing('cycle', began, failed)

    def _cycle_once(self):
        with self._lock:
            self._service_stop_locked()
            # Read and acceptance are one state phase. A concurrent ARM must
            # never be followed by accepting an older, pre-ARM STATE reply.
            # Release this lock before _control_once performs inference.
            state = self._read_state()
            self._accept_state_locked(state)
            if self._mode == 'off':
                self._apply_config_locked()
                return
            # Fixed stance is owned by X5, including the slow transition into
            # it. Browser timers/lifecycle cannot release it. Real feedback,
            # interlocks, explicit STOP and the MCU watchdog remain mandatory.
            if self._heartbeat_expired_locked():
                self._stop_locked('网页控制心跳超时，已停止')
            elif self._age_locked() >= FRESH_SECONDS:
                self._stop_locked('真实反馈超时，已停止')
            elif self._mode in ('hold',) + MODEL_MODES:
                try:
                    self._motion_gate_locked()
                    if (not state['armed'] or state['cookie'] != self._cookie or not state['configured']
                            or state['torque_mask'] != ALL_MASK):
                        raise MotionError('下位机已停止或会话失效')
                except MotionError as exc:
                    self._stop_locked(str(exc))
            elif state['armed'] or state['torque_mask']:
                self._stop_locked('真实反馈测试要求全部扭矩关闭')
            self._service_stop_locked()
            if self._mode == 'hold':
                # HOLD has no model inference. Keep its measured-state and
                # TARGET phase together so a display reader cannot acquire
                # the lock between them and age the accepted feedback.
                self._control_once()
                return
        self._control_once()

    def _infer_policy_target(self, selected, st, previous, command, smooth, elapsed, *,
                             head_command=None):
        session = self.sessions.get(selected)
        if session is None:
            raise MotionError(MODEL_LABELS[selected] + '模型未加载')
        profile = self._policy_profiles.get(selected)
        if profile is None:
            raise MotionError(MODEL_LABELS[selected] + '模型元数据尚未验证')
        mouth = 0.
        if selected == 'pick':
            phase = elapsed / EPISODE_SECONDS['pick']
            command = np.array([math.cos(2*math.pi*phase), math.sin(2*math.pi*phase), 0.], np.float32)
            mouth = 30. if phase < .4 else 0.
        elif selected != 'walk':
            command = np.zeros(3, np.float32)
        head = np.zeros(4, np.float32) if head_command is None else np.asarray(head_command, dtype=np.float32)
        if head.shape != (4,) or not np.isfinite(head).all():
            raise MotionError('头部指令需要四个有限弧度值')
        obs = build_obs(st, previous, command, head,
                        home_rad=profile['home_rad']).reshape(1, 61)
        began = time.perf_counter()
        action = np.asarray(session.run(None, {session.get_inputs()[0].name: obs})[0])
        inference_ms = (time.perf_counter() - began) * 1000
        if action.shape != (1, 14) or not np.isfinite(action).all():
            raise MotionError(MODEL_LABELS[selected] + '策略输出无效，已停止')
        action = action.reshape(14)
        smooth = smooth.copy()
        # POLICY_IDS contains four head joints at indices5..8. Index9 is
        # ID20, the right hip yaw, and belongs to the leg smoothing group.
        for sl, alpha in ((slice(0, 5), ACTION_ALPHA), (slice(5, 9), ACTION_ALPHA_HEAD),
                          (slice(9, 14), ACTION_ALPHA)):
            if selected == 'roulade': alpha = .15
            smooth[sl] = smooth[sl] * alpha + action[sl] * (1 - alpha)
        return action_to_deg(smooth, mouth_deg=mouth, home_rad=profile['home_rad'],
                             action_scale=profile['scale']), action, smooth, inference_ms

    def _control_once(self):
        with self._lock:
            generation, mode = self._generation, self._mode
            if mode == 'off':
                return
            if mode == 'shadow' and (not self._calibrated_locked() or self._filtered is None):
                self._last_action = None
                self._error = '当前只读取真实状态；完成全部校准后才计算模型输出'
                return
            if self._filtered is None:
                return
            elapsed = self._clock() - self._episode_started if self._episode_started is not None else 0.
            if mode in EPISODE_SECONDS:
                finished = elapsed >= EPISODE_SECONDS[mode]
                if mode == 'getup' and elapsed >= 1.:
                    upright = self._tilt_locked() < 15 and max(
                        abs(a-b) for a,b in zip(self._filtered.angles, self._stand_deg)) < 5
                    if not upright:
                        self._episode_upright_since = None
                    elif self._episode_upright_since is None:
                        self._episode_upright_since = self._clock()
                    elif self._clock() - self._episode_upright_since >= 1.:
                        finished = True
                if finished:
                    timeout = mode == 'getup' and elapsed >= EPISODE_SECONDS[mode]
                    self._episode_result = MODEL_LABELS[mode] + ('超时，已停止' if timeout else '本轮模型运行结束，已停止')
                    self._stop_locked(self._episode_result if timeout else None)
                    self._service_stop_locked()
                    return
            st = copy.deepcopy(self._filtered)
            selected = self._preview_model if mode == 'shadow' else mode
            head_command = self._walk_head_command_locked() if selected == 'walk' else None
            previous, command = self._previous_action.copy(), self._command.copy()
            smooth = self._smooth_action.copy()
            if mode == 'hold':
                fraction = min(1., max(0., (self._clock() - self._hold_begin) / 3.))
                target = [(1 - fraction) * a + fraction * b for a, b in zip(self._hold_initial, self._fixed_stance_locked())]
            elif mode == 'getup' and elapsed < 1.:
                # Match upstream recovery's one-second default-pose stage.
                target = list(self._stand_deg)
            else:
                target = None
        inference_ms, action = None, None
        if selected in MODEL_MODES and target is None:
            session = self.sessions.get(selected)
            if session is None:
                with self._lock:
                    if generation == self._generation:
                        self._stop_locked(MODEL_LABELS[selected] + '模型未加载')
                        self._service_stop_locked()
                return
            target, action, smooth, inference_ms = self._infer_policy_target(
                selected, st, previous, command, smooth, elapsed if mode != 'shadow' else 0.,
                head_command=head_command)
        with self._lock:
            # STOP or any mode change arriving during inference discards the
            # obsolete result before obtaining the serial transmit lock.
            if generation != self._generation or mode != self._mode:
                return
            if self._heartbeat_expired_locked():
                self._stop_locked('网页控制心跳超时，已停止')
                self._service_stop_locked()
                return
            self._require_feedback_locked()
            preview_error = None
            if action is not None:
                self._record_policy_target_locked(target)
            if mode != 'shadow':
                self._motion_gate_locked()
                execution_target = self._execution_target_locked(selected, target)
                raw = self._raw_targets_locked(execution_target)
                self._request(TARGET, struct.pack('<I15H', self._cookie, *raw))
                self._last_executed_target = [float(v) for v in execution_target]
                self._target_times.append(self._clock())
            else:
                try:
                    self._raw_targets_locked(target, mode=selected)
                except MotionError as error:
                    preview_error = '不动作预览：' + str(error) + '；未发送舵机目标'
            if action is not None:
                self._previous_action[:] = action
                self._smooth_action[:] = smooth
            self._last_action = [float(v) for v in target]
            self._last_inference_ms = inference_ms
            self._error = preview_error

    @staticmethod
    def _cadence_summary(times, total, now):
        times = list(times)
        recent = [value for value in times if 0 <= now - value < TIMING_WINDOW_SECONDS]
        intervals = [(second - first) * 1000 for first, second in zip(times, times[1:])
                     if 0 <= now - second < TIMING_WINDOW_SECONDS]
        return dict(total_count=total, window_count=len(recent), interval_count=len(intervals),
                    mean_interval_ms=round(sum(intervals) / len(intervals), 3) if intervals else None,
                    max_interval_ms=round(max(intervals), 3) if intervals else None,
                    last_interval_ms=round((times[-1] - times[-2]) * 1000, 3) if len(times) > 1 else None,
                    last_received_age_ms=round(max(0., now - times[-1]) * 1000, 3) if times else None,
                    window_truncated=len(times) == TIMING_HISTORY_LIMIT and
                        now - times[0] < TIMING_WINDOW_SECONDS)

    @staticmethod
    def _duration_summary(records, now):
        records = list(records)
        recent = [row for row in records if 0 <= now - row[0] < TIMING_WINDOW_SECONDS]
        durations = [row[1] for row in recent]
        return dict(total_count=records[-1][3] if records else 0,
                    total_failed=records[-1][4] if records else 0,
                    window_count=len(recent), window_failed=sum(row[2] for row in recent),
                    mean_ms=round(sum(durations) / len(durations), 3) if durations else None,
                    max_ms=round(max(durations), 3) if durations else None,
                    last_ms=round(records[-1][1], 3) if records else None,
                    last_completed_age_ms=round(max(0., now - records[-1][0]) * 1000, 3) if records else None,
                    window_truncated=len(records) == TIMING_HISTORY_LIMIT and
                        now - records[0][0] < TIMING_WINDOW_SECONDS)

    def _timing_summary_locked(self):
        now = self._clock()
        result = dict(protocol=1, window_seconds=TIMING_WINDOW_SECONDS,
                      history_limit=TIMING_HISTORY_LIMIT, totals_scope='controller_lifetime',
                      state_receive=self._cadence_summary(self._state_receive_times, self._state_receive_count, now),
                      state_samples=self._cadence_summary(self._state_times, self._state_sample_count, now))
        result.update({name: self._duration_summary(records, now)
                       for name, records in self._timing_records.items()})
        return result

    def status(self):
        with self._lock:
            state = self._state
            joints = [dict(copy.deepcopy(j), ready=_joint_ready(j)) for j in self._config['joints']]
            imu = copy.deepcopy(self._config['imu'])
            imu.update(calibrating=self._imu_collection is not None, progress=0., acc_body=None,
                       gyro_body=None, gravity=None, pitch_deg=None, roll_deg=None)
            if self._imu_collection:
                imu['progress'] = min(1., len(self._imu_collection['samples']) / 100.)
            if state:
                _, matrix = _axis_matrix(imu['axes'])
                imu['acc_body'] = (matrix @ np.asarray(state['acc'])).tolist()
                imu['gyro_body'] = (matrix @ (np.asarray(state['gyro']) - imu['bias_dps'])).tolist()
            if self._filtered:
                gx, gy, gz = self._filtered.grav
                imu.update(gravity=list(self._filtered.grav), pitch_deg=math.degrees(math.atan2(gx, -gz)),
                           roll_deg=math.degrees(math.atan2(-gy, -gz)))
            def ids(mask):
                return [sid for i, sid in enumerate(MOTOR_IDS) if mask & (1 << i)]
            rawdata = []
            if state:
                for i, j in enumerate(joints):
                    online = bool(state['online_mask'] & (1 << i))
                    rawdata.append(dict(id=j['id'], position=state['positions'][i] if online else None,
                                        velocity_raw=state['velocities'][i] if online else None,
                                        angle_deg=(state['positions'][i] - j['zero']) * 360. / 4095 * j['sign']
                                        if online and j['zero'] is not None else None,
                                        velocity_deg_s=state['velocities'][i] * SPEED_COUNTS_PER_UNIT *
                                        360. / 4095 * j['sign'] if online else None))
            def rate(times):
                recent = [t for t in times if self._clock() - t < 1.]
                span = recent[-1] - recent[0] if len(recent) > 1 else 0.
                return round((len(recent) - 1) / span, 1) if span > 0 else 0.
            age, imu_age = self._age_locked(), self._imu_age_locked()
            preview_head = self._walk_head_command_locked()
            voltage = state.get('voltage') if state else None
            voltage_rows = []
            elapsed_ms = max(0., self._clock() - self._voltage_received) * 1000 if self._voltage_received is not None else math.inf
            for i, sid in enumerate(MOTOR_IDS):
                seen = bool(voltage and voltage['seen_mask'] & (1 << i))
                voltage_age = voltage['ages_us'][i] / 1000 + elapsed_ms if seen else None
                fresh = bool(seen and self._connected and voltage['valid_mask'] & (1 << i)
                             and voltage_age <= VOLTAGE_FRESH_MS)
                volts = voltage['raw'][i] / 10. if seen else None
                voltage_rows.append(dict(id=sid, volts=volts, age_ms=round(voltage_age, 1) if seen else None,
                                         fresh=fresh, condition=('low' if volts < 4. else 'high' if volts > 8.4 else 'normal')
                                         if fresh else 'unknown'))
            return dict(connected=self._connected, port=self.port, protocol=1, mode=self._mode,
                        generation=self._generation,
                        calibrated=self._calibrated_locked(), mechanical_reference_confirmed=
                        self._config['mechanical_reference_confirmed'], hold_ready=self._hold_ready_locked(),
                        motor_output=self._mode in ('hold',) + MODEL_MODES and bool(state and state['armed']),
                        armed=bool(state and state['armed']), configured=bool(state and state['configured']),
                        online_ids=ids(state['online_mask']) if state else [],
                        fault_ids=ids(state['fault_mask']) if state else [],
                        torque_ids=ids(state['torque_mask']) if state else [],
                        age_ms=round(age * 1000, 1) if math.isfinite(age) else None,
                        imu_age_ms=round(imu_age * 1000, 1) if math.isfinite(imu_age) else None,
                        rawdata=rawdata, joints=joints, imu=imu,
                        voltage_supported=voltage is not None, motor_voltages=voltage_rows,
                        voltage_range_v=[4., 8.4], voltage_scale_hardware_verified=False,
                        command=dict(zip(('forward', 'side', 'yaw'), map(float, self._command))),
                        last_action=self._last_action, inference_ms=self._last_inference_ms,
                        last_policy_target_deg=copy.deepcopy(self._last_policy_target),
                        last_policy_target_rejections=copy.deepcopy(self._last_policy_target_rejections),
                        last_executed_target_deg=copy.deepcopy(self._last_executed_target),
                        target_rejection=copy.deepcopy(self._target_rejection),
                        active_model=self._preview_model if self._mode == 'shadow' else self._mode if self._mode in MODEL_MODES else None,
                        behavior_protocol=1, target_validation_protocol=1,
                        policy_metadata_protocol=1, stand_target_deg=self._fixed_stance_locked(),
                        model_stand_target_deg=copy.deepcopy(self._stand_deg),
                        hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg,
                        head_clearance_protocol=1, head_clearance_required=self._require_head_clearance,
                        fixed_head_walk_protocol=1,
                        fixed_head_walk_allowed=self._allow_fixed_head_walk and self._hold_hip_pitch_bias_deg == 0,
                        fixed_head_walk_ready=self._fixed_head_walk_ready_locked(),
                        fixed_head_walk_adapted=self._mode == 'walk' and self._fixed_head_walk_ready_locked(),
                        fixed_head_walk_zero_velocity_only=True,
                        hold_head_deg=copy.deepcopy(self._hold_head_deg),
                        walk_preview_head_protocol=1,
                        walk_preview_head_command_rad=preview_head.tolist() if preview_head is not None else None,
                        policy_profiles={name:dict(home_rad=profile['home_rad'].tolist(),scale=profile['scale'])
                                         for name,profile in self._policy_profiles.items()},
                        behavior_result=self._episode_result,
                        behavior_seconds_remaining=max(0.,EPISODE_SECONDS[self._mode]-(self._clock()-self._episode_started))
                            if self._mode in EPISODE_SECONDS and self._episode_started is not None else None,
                        error=self._error, pending_stop=self._pending_stop,
                        firmware_error=state['last_error'] if state else None,
                        sample_us=state['sample_us'] if state else None,
                        imu_us=state['imu_us'] if state else None,
                        state_request_sequence=self._state_request_sequence,
                        timing=self._timing_summary_locked(),
                        rates=dict(state_hz=rate(self._state_times), control_hz=rate(self._target_times)),
                        background_hold=self._mode == 'hold' and self._background_hold,
                        hold_control_protocol=2, page_heartbeat_required=self._mode != 'hold',
                        heartbeat_remaining_ms=None if self._mode == 'hold' else
                            max(0., round((self._heartbeat_timeout_locked() - (self._clock() - self._heartbeat))
                                         * 1000, 1)) if self._heartbeat is not None else 0.)

    def close(self):
        self.stop()
        self._closed.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.)
        if thread is None:
            self._disconnect('控制器已关闭')
