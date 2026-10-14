"""Hardware-free safety and protocol tests; never import/open a serial device."""
import json
import copy
import math
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest import mock

import numpy as np

from motion import (ACK, ALL_MASK, ARM, CONFIG, CONFIG_PACKET, FrameParser, HEADER,
                    HEARTBEAT_SECONDS, HELLO, HELLO_REPLY, MAGIC, MotionController,
                    MotionError, SerialTransport, STATE, STATE_PACKET, STATE_REPLY,
                    STOP, TARGET, _axis_matrix, _default_config, crc16, decode_state,
                    encode_frame, arm_failure_detail)
from rl_core import (DEFAULT_DEG, FIXED_STANCE_OVERRIDES, MOTOR_IDS,
                     POLICY_JOINT_NAMES, POLICY_OBSERVATION_NAMES)


class FakeClock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now

    def advance(self, seconds=.02):
        self.now += seconds


class FakeTransport:
    def __init__(self, port):
        self.port = port
        self.requests = []
        self.opened = self.closed = False
        self.sample_us = 100000
        self.online = ALL_MASK
        self.fault = 0
        self.torque = 0
        self.armed = False
        self.imu_ok = True
        self.configured = False
        self.cookie = 0
        self.positions = [2047] * 15
        self.velocity = [0] * 15
        self.acc = [0., 0., 9.8]
        self.gyro = [.1, -.2, .3]
        self.imu_age_us = 0
        self.freeze_sample = False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def request(self, kind, payload=b''):
        self.requests.append((kind, payload))
        if kind == HELLO:
            return b'XGODUCK_MOTION_1'
        if kind == STATE:
            if not self.freeze_sample:
                self.sample_us = (self.sample_us + 20000) & 0xffffffff
            return STATE_PACKET.pack(self.sample_us, (self.sample_us - self.imu_age_us) & 0xffffffff,
                                     self.online, self.fault, self.torque, self.armed, self.imu_ok,
                                     *self.positions, *self.velocity, *self.acc, *self.gyro,
                                     self.cookie, 0, self.configured)
        if kind == STOP:
            self.torque = 0
            self.armed = False
            self.cookie = 0
        elif kind == CONFIG:
            CONFIG_PACKET.unpack(payload)
            if self.armed:
                raise MotionError('fake config while armed')
            self.configured = True
        elif kind == ARM:
            if not self.configured:
                raise MotionError('fake arm before config')
            self.cookie = struct.unpack('<I', payload)[0]
            self.armed = True
            self.torque = ALL_MASK
        elif kind == TARGET:
            cookie, *positions = struct.unpack('<I15H', payload)
            if not self.armed or cookie != self.cookie:
                raise MotionError('fake target when not armed')
            self.positions = positions
        return bytes([kind, 0])


class FakePolicy:
    def __init__(self, name='walk'):
        self.observation = None
        self.calls = 0
        self.entered = self.release = None
        self.output = np.zeros((1, 14), dtype=np.float32)
        q = '.419' if name == 'roulade' else '.349'
        self.metadata = dict(joint_names=','.join(POLICY_JOINT_NAMES),
                             observation_names=','.join(POLICY_OBSERVATION_NAMES),
                             default_joint_pos=','.join(['0','-.087','-'+q,'-.005',q,'.349','.349','0','0','0','.087',q,'.005','-'+q]),
                             action_scale='1.0')

    def get_inputs(self):
        return [type('Input', (), {'name': 'obs', 'shape': [1,61], 'type': 'tensor(float)'})()]

    def get_outputs(self):
        return [type('Output', (), {'name': 'actions', 'shape': [1,14], 'type': 'tensor(float)'})()]

    def get_modelmeta(self):
        return type('Metadata', (), {'custom_metadata_map': self.metadata})()

    def run(self, outputs, feed):
        self.calls += 1
        self.observation = feed['obs'].copy()
        if self.entered is not None:
            self.entered.set()
            self.release.wait(2.)
        return [self.output.copy()]


def ready_config():
    config = _default_config()
    config['mechanical_reference_confirmed'] = True
    for j in config['joints']:
        j.update(zero=2047, min=100, max=3990, sign=-1, direction_confirmed=True)
    config['imu'].update(axes_confirmed=True, calibrated=True, bias_dps=[.1, -.2, .3])
    return config


class ProtocolTests(unittest.TestCase):
    def target_stop_packet(self, **overrides):
        values = dict(kind=TARGET,error=2,version=1,cause=6,first_error=4,imu_ok=1,
            online=ALL_MASK,fault=0,torque=ALL_MASK,at_us=25,
            position_age=12000,imu_age=3000,target_age=250001,uart_errors=0,
            read_id=21,address=40,outcome=1,device_error=0,read_elapsed=600)
        values.update(overrides)
        from motion import TARGET_STOP_PACKET
        return TARGET_STOP_PACKET.pack(*values.values())

    def test_first_stop_target_ack_reports_watchdog_and_preserves_timings_without_retry(self):
        from motion import target_stop_detail
        packet=self.target_stop_packet()
        diagnostic,text=target_stop_detail(packet)
        self.assertEqual(diagnostic['target_age_us'],250001)
        self.assertEqual(diagnostic['stopped_at_us'],25,'MCU uptime may cross uint32 wrap')
        self.assertIn('目标指令间隔超过 250 毫秒',text)
        self.assertIn('250.0 ms',text)
        writes=[]
        class Stream:
            in_waiting=0
            def write(self,data):
                writes.append(data)
                *_,seq=HEADER.unpack_from(data)
                self.rx=encode_frame(ACK,seq,packet);self.in_waiting=len(self.rx)
                return len(data)
            def read(self,size):
                value,self.rx=self.rx[:size],self.rx[size:]
                self.in_waiting=len(self.rx);return value
        transport=SerialTransport('fake');transport.serial=Stream()
        with self.assertRaisesRegex(MotionError,'最先停机原因'):
            transport.request(TARGET,struct.pack('<I15H',123,*([2047]*15)))
        self.assertEqual(len(writes),1)
        evidence=transport.timing_diagnostics()
        self.assertEqual(evidence['firmware_first_stop'],diagnostic)
        self.assertEqual(evidence['requests'][0]['kind'],TARGET)
        self.assertIn('指令 TARGET',evidence['requests'][0]['error'])
        evidence['firmware_first_stop']['cause']=1
        self.assertEqual(transport.last_firmware_stop['cause'],6)

    def test_first_stop_ack_validates_fields_and_distinguishes_servo_from_battery(self):
        from motion import target_stop_detail
        detail,text=target_stop_detail(self.target_stop_packet(cause=2,online=ALL_MASK&~(1<<6),outcome=6))
        self.assertIn('涉及舵机 21',text)
        self.assertIn('截止时间内没有有效应答',text)
        self.assertNotIn('电压',text)
        for invalid in [self.target_stop_packet()[:-1], self.target_stop_packet()+b'\0']:
            with self.assertRaises(MotionError):target_stop_detail(invalid)
        for changes in [dict(version=2),dict(cause=0),dict(cause=10),dict(imu_ok=2),
                        dict(error=4),dict(first_error=0),dict(kind=ARM),dict(online=0x8000),
                        dict(read_id=254),dict(outcome=8),dict(address=41)]:
            with self.subTest(changes=changes),self.assertRaises(MotionError):
                target_stop_detail(self.target_stop_packet(**changes))

    def test_transport_timing_history_is_bounded_and_records_target_attempt_gaps(self):
        clock=FakeClock()
        class Stream:
            in_waiting=0
            def write(self,data):
                _,_,kind,_,seq=HEADER.unpack_from(data)
                self.rx=encode_frame(ACK,seq,bytes([kind,0]));self.in_waiting=len(self.rx)
                clock.advance(.004);return len(data)
            def read(self,size):
                value,self.rx=self.rx[:size],self.rx[size:]
                self.in_waiting=len(self.rx);return value
        transport=SerialTransport('fake');transport.serial=Stream()
        with mock.patch('motion.time.monotonic',clock):
            for _ in range(80):
                transport.request(TARGET);clock.advance(.016)
        evidence=transport.timing_diagnostics()
        self.assertEqual(len(evidence['requests']),64)
        self.assertEqual(evidence['requests'][0]['sequence'],17)
        self.assertEqual(evidence['requests'][-1]['elapsed_ms'],4.)
        self.assertEqual(evidence['requests'][-1]['target_attempt_gap_ms'],20.)
        self.assertIsNone(evidence['firmware_first_stop'])

    def test_crc_known_vector_and_fragmented_frame(self):
        self.assertEqual(crc16(b'123456789'), 0x29b1)
        parser = FrameParser()
        encoded = encode_frame(STATE_REPLY, 19, b'abc')
        self.assertEqual(parser.feed(b'boot noise' + encoded[:5]), [])
        self.assertEqual(parser.feed(encoded[5:]), [(STATE_REPLY, 19, b'abc')])

    def test_crc_c_implementation_matches_firmware_for_all_packet_lengths(self):
        # Independent bitwise reference retained from the firmware contract.
        def firmware_crc(data):
            crc = 0xffff
            for byte in data:
                crc ^= byte << 8
                for _ in range(8):
                    crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
            return crc
        rng = np.random.default_rng(1971)
        for length in range(HEADER.size + 257):
            data = rng.bytes(length)
            self.assertEqual(crc16(data), firmware_crc(data))
        frame = encode_frame(STATE_REPLY, 18, rng.bytes(220))
        for position in range(len(frame)):
            damaged = bytearray(frame)
            damaged[position] ^= 1
            self.assertEqual(FrameParser().feed(damaged), [])

    def test_invalid_crc_version_and_length_are_not_delivered(self):
        valid = encode_frame(ACK, 9, bytes([STOP, 0]))
        corrupt = bytearray(valid)
        corrupt[-1] ^= 1
        wrong_version = HEADER.pack(MAGIC, 2, ACK, 0, 2)
        oversized = HEADER.pack(MAGIC, 1, ACK, 257, 3)
        self.assertEqual(FrameParser().feed(bytes(corrupt) + wrong_version + oversized + valid),
                         [(ACK, 9, bytes([STOP, 0]))])

    def test_state_finite_fields_masks_and_position_validation(self):
        transport = FakeTransport('fake')
        state = decode_state(transport.request(STATE))
        self.assertEqual(state['positions'], [2047] * 15)
        transport.acc[0] = float('nan')
        with self.assertRaises(MotionError):
            decode_state(transport.request(STATE))
        transport.acc[0] = 0
        transport.positions[0] = 4096
        with self.assertRaises(MotionError):
            decode_state(transport.request(STATE))

    def test_serial_open_sets_reset_lines_before_open_and_checks_seq(self):
        events = []

        class Stream:
            def __init__(self, **kwargs):
                events.append(('construct', kwargs))
                self.rx = bytearray()
            def __setattr__(self, key, value):
                if key in ('dtr', 'rts', 'port'):
                    events.append((key, value))
                object.__setattr__(self, key, value)
            def open(self):
                events.append(('open',))
            def close(self):
                events.append(('close',))
            @property
            def in_waiting(self):
                return len(self.rx)
            def write(self, data):
                _, _, kind, size, seq = HEADER.unpack_from(data)
                self.rx.extend(encode_frame(ACK, seq - 1, bytes([kind, 0])))
                self.rx.extend(encode_frame(ACK, seq, bytes([kind, 0])))
                return len(data)
            def read(self, size):
                data = bytes(self.rx[:size])
                del self.rx[:size]
                return data

        transport = SerialTransport('fake', serial_factory=Stream)
        self.assertEqual(events, [])
        transport.open()
        self.assertLess(events.index(('dtr', False)), events.index(('open',)))
        self.assertLess(events.index(('rts', False)), events.index(('open',)))
        self.assertTrue(events[0][1]['exclusive'])
        self.assertEqual(events[0][1]['write_timeout'], 0)
        self.assertEqual(transport.request(STOP), bytes([STOP, 0]))
        transport.close()

    def test_short_or_blocked_write_never_replays_or_reads_an_ack(self):
        for kind in (ARM, TARGET, STOP, STATE):
            for short in (0, 1):
                with self.subTest(kind=kind, written=short):
                    stream = mock.Mock()
                    stream.write.return_value = short
                    transport = SerialTransport('fake')
                    transport.serial = stream
                    with self.assertRaisesRegex(MotionError, '发送不完整'):
                        transport.request(kind, b'payload')
                    stream.write.assert_called_once_with(encode_frame(kind, 1, b'payload'))
                    stream.read.assert_not_called()
                    self.assertEqual(transport.seq, 1)
                    self.assertIn('发送不完整', transport.request_timings[-1]['error'])

    def test_write_exception_is_not_replayed_or_treated_as_acknowledged(self):
        stream = mock.Mock()
        stream.write.side_effect = OSError('actual disconnected UART')
        transport = SerialTransport('fake')
        transport.serial = stream
        with self.assertRaisesRegex(OSError, 'actual disconnected UART'):
            transport.request(TARGET, b'payload')
        stream.write.assert_called_once()
        stream.read.assert_not_called()
        self.assertEqual(transport.request_timings[-1]['error'], 'actual disconnected UART')

    def test_real_posix_path_uses_one_syscall_and_fails_on_blocked_or_interrupted_write(self):
        frame = encode_frame(STOP, 1)
        for outcome in (0, 1, BlockingIOError('busy UART'), InterruptedError('interrupted UART'), len(frame)):
            with self.subTest(outcome=outcome):
                stream, module = mock.Mock(), mock.Mock()
                stream.fileno.return_value = 42
                stream.in_waiting = len(encode_frame(ACK, 1, bytes([STOP, 0])))
                stream.read.return_value = encode_frame(ACK, 1, bytes([STOP, 0]))
                module.Serial.return_value = stream
                syscall = mock.Mock(side_effect=outcome) if isinstance(outcome, Exception) else mock.Mock(return_value=outcome)
                with mock.patch.dict('sys.modules', {'serial': module}), \
                        mock.patch('motion.os.get_blocking', return_value=False), \
                        mock.patch('motion.os.write', syscall):
                    transport = SerialTransport('fake-posix')
                    transport.open()
                    try:
                        if isinstance(outcome, Exception):
                            with self.assertRaises(type(outcome)):
                                transport.request(STOP)
                        elif outcome != len(frame):
                            with self.assertRaisesRegex(MotionError, '发送不完整'):
                                transport.request(STOP)
                        else:
                            self.assertEqual(transport.request(STOP), bytes([STOP, 0]))
                        syscall.assert_called_once_with(42, frame)
                        stream.write.assert_not_called()
                        if outcome != len(frame):
                            stream.read.assert_not_called()
                    finally:
                        transport.close()
                    self.assertFalse(transport._posix_write)

    def test_blocking_real_uart_is_closed_before_any_command(self):
        stream, module = mock.Mock(), mock.Mock()
        module.Serial.return_value = stream
        with mock.patch.dict('sys.modules', {'serial': module}), \
                mock.patch('motion.os.get_blocking', return_value=True), \
                mock.patch('motion.os.write') as write:
            transport = SerialTransport('fake-posix')
            with self.assertRaisesRegex(MotionError, '非阻塞方式'):
                transport.open()
            stream.close.assert_called_once()
            write.assert_not_called()
            self.assertIsNone(transport.serial)

    def test_axes_are_explicit_signed_right_handed(self):
        _, matrix = _axis_matrix(['z', '-x', '-y'])
        self.assertEqual(round(np.linalg.det(matrix)), 1)
        for bad in (['x', 'x', 'z'], ['x', 'y', '-z'], ['bad', 'y', 'z']):
            with self.assertRaises(MotionError):
                _axis_matrix(bad)

    def test_rejected_arm_ack_identifies_command_and_exact_error_code(self):
        class Stream:
            in_waiting = 0
            def write(self, data):
                _, _, kind, _, seq = HEADER.unpack_from(data)
                self.rx = encode_frame(ACK, seq, bytes([kind, 4]))
                self.in_waiting = len(self.rx)
                return len(data)
            def read(self, _size):
                value, self.rx = self.rx, b''
                self.in_waiting = 0
                return value
        transport = SerialTransport('fake')
        transport.serial = Stream()
        with self.assertRaisesRegex(MotionError, '指令 ARM，错误码 4'):
            transport.request(ARM, struct.pack('<I',123))

    def test_arm_preparation_ack_budget_is_bounded_and_never_replayed(self):
        # Slow valid preparation can need 130 ms on the MCU plus USB latency.
        # A longer ARM ACK wait must not extend STATE/TARGET waits or retry ARM.
        for kind, delay, succeeds in ((ARM, .18, True), (ARM, .25, False),
                                      (STATE, .18, False), (TARGET, .18, False)):
            with self.subTest(kind=kind, delay=delay):
                clock = FakeClock()
                writes = []
                class Stream:
                    @property
                    def in_waiting(self):
                        return len(self.rx) if clock() >= self.ready else 0
                    def write(self, data):
                        writes.append(data)
                        _, _, request, _, seq = HEADER.unpack_from(data)
                        self.rx = encode_frame(ACK, seq, bytes([request, 0]))
                        self.ready = clock() + delay
                        return len(data)
                    def read(self, size):
                        if clock() < self.ready:
                            clock.advance(.01)
                            return b''
                        data, self.rx = self.rx[:size], self.rx[size:]
                        return data
                transport = SerialTransport('fake')
                transport.serial = Stream()
                began = clock()
                with mock.patch('motion.time.monotonic', clock):
                    if succeeds:
                        self.assertEqual(transport.request(kind), bytes([kind, 0]))
                    else:
                        with self.assertRaises(MotionError): transport.request(kind)
                self.assertEqual(len(writes), 1)
                self.assertLess(clock() - began, .22 if kind == ARM else .18)

    def test_extended_arm_detail_keeps_stage_register_and_sample_ages(self):
        detail = bytes([ARM,4,1,3,31,50,0,0,0,5,6,0,20,0])+struct.pack('<II',64000,1200)
        text = arm_failure_detail(detail)
        for expected in ['增益读取','ID 31','寄存器 50','未收到有效回复','64.0 ms','1.2 ms']:
            self.assertIn(expected,text)
        for invalid in [detail[:-1],detail+b'\0',detail[:2]+b'\2'+detail[3:],detail[:3]+b'\x0a'+detail[4:]]:
            with self.assertRaises(MotionError): arm_failure_detail(invalid)

    def test_arm_read_outcome_distinguishes_busy_bus_from_missing_reply(self):
        detail = bytearray(bytes([ARM,4,1,3,10,50,0,0,0,3,6,0,20,0])+struct.pack('<II',5600,7900))
        self.assertIn('总线尚未空闲（代码 3）', arm_failure_detail(detail))
        detail[9] = 6
        self.assertIn('截止时间内没有有效应答（代码 6）', arm_failure_detail(detail))
        detail[9] = 8
        with self.assertRaises(MotionError): arm_failure_detail(detail)

    def test_arm_torque_failure_retains_first_servo_and_read_result(self):
        detail = bytearray(bytes([ARM,4,1,9,21,40,0,0,0,6,1,0,0,0])+struct.pack('<II',9300,5100))
        for expected in ['使能后扭矩检查', 'ID 21', '寄存器 40', '未收到有效回复',
                         '截止时间内没有有效应答（代码 6）', '9.3 ms', '5.1 ms']:
            self.assertIn(expected, arm_failure_detail(detail))
        detail[1] = 2; detail[6] = 1; detail[8] = 1; detail[9] = 1
        self.assertIn('扭矩开关 0（期望 1）', arm_failure_detail(detail))
        detail[3] = 4; detail[10] = 0; detail[11] = 1
        self.assertIn('准备后扭矩读取', arm_failure_detail(detail))
        self.assertIn('扭矩开关 1（期望 0）', arm_failure_detail(detail))
        # Legacy version-1 stage-9 ACKs lack a failed ID; do not invent one.
        detail[3] = 9; detail[4] = 0
        self.assertNotIn('ID ', arm_failure_detail(detail))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.config_path = Path(self.tmp.name) / 'calibration.json'
        self.clock = FakeClock()
        self.policy = FakePolicy()
        self.transports = []

    def tearDown(self):
        self.tmp.cleanup()

    def controller(self, ready=False, sessions=None, require_head_clearance=False,
                   allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=0):
        if ready:
            self.config_path.write_text(json.dumps(ready_config()))
        def factory(port):
            transport = FakeTransport(port)
            self.transports.append(transport)
            return transport
        return MotionController({'walk': self.policy} if sessions is None else sessions, self.config_path,
                                transport_factory=factory, clock=self.clock,
                                require_head_clearance=require_head_clearance,
                                allow_fixed_head_walk=allow_fixed_head_walk,
                                hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg)

    def test_hold_hip_bias_is_bounded_finite_numeric_and_constructor_only(self):
        for value in (True, False, None, '-2', float('nan'), float('inf'),
                      -float('inf'), -3.0001, 3.0001, 10 ** 400):
            with self.subTest(value=value), self.assertRaisesRegex(MotionError, '髋偏置'):
                self.controller(hold_hip_pitch_bias_deg=value)
        self.assertEqual(self.transports, [])
        for value in (-3, -2., 0, 3.):
            c = self.controller(hold_hip_pitch_bias_deg=value)
            self.assertIs(type(c.hold_hip_pitch_bias_deg), float)
            self.assertEqual(c.hold_hip_pitch_bias_deg, value)
            with self.assertRaises(AttributeError):
                c.hold_hip_pitch_bias_deg = 0

    def test_hold_hip_bias_changes_only_hips_and_retains_slow_hold_and_head_targets(self):
        c = self.controller(ready=True, require_head_clearance=True,
                            hold_hip_pitch_bias_deg=-2)
        c._connect_once()
        t = self.transports[-1]
        t.positions[10:14] = [2180, 1900, 2055, 2040]
        self.cycle(c)
        original = self.config_path.read_bytes()
        c.capture_head_clearance(clearance_confirmed=True,
                                 expected_generation=c.status()['generation'])
        baseline = list(DEFAULT_DEG)
        for sid, angle in FIXED_STANCE_OVERRIDES.items():
            baseline[MOTOR_IDS.index(sid)] = angle
        expected = baseline.copy()
        expected[2] -= 2
        expected[7] += 2
        expected[10:14] = [c._hold_head_deg[sid] for sid in (30, 31, 32, 33)]
        np.testing.assert_allclose(c.status()['stand_target_deg'], expected)
        np.testing.assert_allclose(c.status()['model_stand_target_deg'], DEFAULT_DEG)
        np.testing.assert_allclose(c._stand_deg, DEFAULT_DEG)
        initial = list(c._filtered.angles)
        self.hold(c)
        self.cycle(c, dt=1.5)
        np.testing.assert_allclose(c.status()['last_executed_target_deg'],
                                   (np.asarray(initial) + expected) / 2)
        self.assertFalse(c.status()['hold_ready'])
        self.cycle(c, dt=1.6)
        np.testing.assert_allclose(c.status()['last_executed_target_deg'], expected)
        self.cycle(c)
        self.assertTrue(c.status()['hold_ready'])
        # Readiness compares against the new HOLD target, with the original <5° gate.
        c._filtered.angles[2] = baseline[2] + 4
        self.assertFalse(c.status()['hold_ready'])
        c._filtered.angles[2] = expected[2]
        self.assertTrue(c.status()['hold_ready'])
        self.assertEqual(self.config_path.read_bytes(), original)
        self.assertEqual(self.policy.calls, 0)

    def test_hold_hip_bias_target_range_is_checked_before_arm_without_clamping(self):
        config = ready_config()
        config['joints'][2].update(zero=1982, sign=-1, min=1617, max=2210)
        self.config_path.write_text(json.dumps(config))
        original = self.config_path.read_bytes()
        c = self.controller(hold_hip_pitch_bias_deg=-2)
        c._connect_once()
        self.transports[-1].positions[2] = 1982
        self.cycle(c)
        with self.assertRaisesRegex(MotionError, 'ID 12 目标超过'):
            self.hold(c)
        self.assertEqual(c.status()['target_rejection']['target_position'], 2221)
        self.assertFalse(any(kind in (ARM, TARGET) for kind, _ in self.transports[-1].requests))
        self.assertEqual(c.status()['mode'], 'off')
        self.assertEqual(self.config_path.read_bytes(), original)

    def test_head_clearance_capture_is_read_only_and_hold_preserves_legs(self):
        c = self.controller(ready=True, require_head_clearance=True)
        c._connect_once()
        with self.assertRaisesRegex(MotionError, '记录头颈'):
            self.hold(c)
        t = self.transports[0]
        self.assertNotIn(ARM, [kind for kind, _ in t.requests])
        t.positions[10:14] = [2180, 1900, 2055, 2040]
        self.cycle(c)
        generation = c.status()['generation']
        original = self.config_path.read_bytes()
        requests = list(t.requests)
        result = c.capture_head_clearance(clearance_confirmed=True, expected_generation=generation)
        self.assertEqual(t.requests, requests, 'capture must not send UART requests or enable motors')
        self.assertEqual(self.config_path.read_bytes(), original, 'capture must not change calibration')
        self.assertEqual(result['generation'], generation + 1)
        expected = [FIXED_STANCE_OVERRIDES.get(sid, angle)
                    for sid, angle in zip(MOTOR_IDS, DEFAULT_DEG)]
        np.testing.assert_allclose(result['stand_target_deg'][:10], expected[:10])
        self.assertEqual(result['stand_target_deg'][10:14], [r['angle_deg'] for r in result['rawdata'][10:14]])
        self.hold(c)
        for _ in range(155):
            self.cycle(c)
        result = c.status()
        self.assertTrue(result['hold_ready'])
        self.assertEqual(t.positions[10:14], [2180, 1900, 2055, 2040])
        for mode in ('walk', 'getup', 'pick', 'roulade'):
            with self.assertRaisesRegex(MotionError, '机械间隙尚未验证'):
                c.start_mode(mode, support_confirmed=True, behavior_confirmed=True)
        self.assertEqual(self.policy.calls, 0)
        c.stop()
        # A new connection/controller cannot inherit an old physical approval.
        other = self.controller(ready=True, require_head_clearance=True)
        other._connect_once()
        self.assertIsNone(other.status()['hold_head_deg'])
        with self.assertRaisesRegex(MotionError, '记录头颈'):
            self.hold(other)

    def test_neck_hold_uses_saved_zero_and_keeps_unsafe_model_home_blocked(self):
        config = ready_config()
        config['joints'][10].update(zero=1977, min=1814, max=2770)
        self.config_path.write_text(json.dumps(config))
        original = self.config_path.read_bytes()
        c = self.controller()
        c._connect_once()
        target = c.status()['stand_target_deg']
        self.assertEqual(target[10], 0.)
        expected = [FIXED_STANCE_OVERRIDES.get(sid, angle)
                    for sid, angle in zip(MOTOR_IDS, DEFAULT_DEG)]
        np.testing.assert_allclose(target[:10], expected[:10])
        np.testing.assert_allclose(target[11:], DEFAULT_DEG[11:])
        self.assertEqual(c._raw_targets_locked(target)[10], 1977)
        self.assertEqual(round(1977 - c._stand_deg[10] * 4095 / 360), 1750)
        with self.assertRaisesRegex(MotionError, '目标超过已记录的机械范围'):
            c._raw_targets_locked(c._stand_deg, mode='walk')
        self.hold(c)
        t = self.transports[0]
        for _ in range(155):
            self.cycle(c)
        self.assertTrue(c.status()['hold_ready'])
        self.assertEqual(t.positions[10], 1977)
        frames = [struct.unpack('<I15H', payload)[1:] for kind, payload in t.requests if kind == TARGET]
        self.assertTrue(frames)
        self.assertTrue(all(1814 <= frame[10] <= 2770 for frame in frames))
        requests = list(t.requests)
        with self.assertRaisesRegex(MotionError, '目标超过已记录的机械范围'):
            c.start_mode('walk', support_confirmed=True)
        self.assertEqual(t.requests, requests)
        self.assertEqual(self.policy.calls, 0)
        self.assertEqual(self.config_path.read_bytes(), original)

    def test_captured_head_uses_trained_walk_input_only_in_non_actuating_preview(self):
        policies = {name: FakePolicy(name) for name in ('walk', 'getup')}
        c = self.controller(ready=True, sessions=policies, require_head_clearance=True)
        c._connect_once()
        t = self.transports[0]
        t.positions[10:14] = [2180, 1900, 2055, 2040]
        self.cycle(c)
        original = self.config_path.read_bytes()
        c.capture_head_clearance(clearance_confirmed=True, expected_generation=c.status()['generation'])
        captured = c.status()['stand_target_deg'][10:14]
        expected = np.deg2rad(captured) - c._policy_profiles['walk']['home_rad'][5:9]
        for name in ('walk', 'getup'):
            policy = policies[name]
            # Distinct outputs prove that captured angles do not replace targets.
            policy.output[0, 5:9] = [.1, -.2, .03, -.04]
            c.start_mode('shadow', preview_model=name)
            self.cycle(c)
            np.testing.assert_allclose(policy.observation[0, 51:55], expected if name == 'walk' else 0., atol=1e-7)
            np.testing.assert_allclose(policy.observation[0, 6:20],
                np.deg2rad(c._filtered.angles)[[MOTOR_IDS.index(sid) for sid in
                    (10,11,12,13,14,30,31,32,33,20,21,22,23,24)]] - c._policy_profiles[name]['home_rad'], atol=1e-7)
            expected_targets = np.rad2deg(c._policy_profiles[name]['home_rad'][5:9] + .55 * policy.output[0, 5:9])
            np.testing.assert_allclose(c.status()['last_action'][10:14], expected_targets, atol=1e-5)
            self.assertFalse(c.status()['motor_output'])
            self.assertFalse(t.armed)
            c.stop()
        self.assertFalse(any(kind in (ARM, TARGET) for kind, _ in t.requests))
        for mode in ('walk', 'getup'):
            with self.assertRaisesRegex(MotionError, '机械间隙尚未验证'):
                c.start_mode(mode, support_confirmed=True, behavior_confirmed=True)
        self.assertEqual(self.config_path.read_bytes(), original)
        other = self.controller(ready=True, require_head_clearance=True)
        self.assertIsNone(other.status()['walk_preview_head_command_rad'])

    def test_authorized_fake_walk_preflight_and_running_policy_share_head_command(self):
        # A CPU-only controller without the native clearance restriction models
        # a separately authorized robot. Native capture still cannot lift it.
        c = self.controller(ready=True)
        c._connect_once()
        t = self.transports[0]
        c._hold_head_deg = {30: 12., 31: 15., 32: 1., 33: 2.}
        t.positions = c._raw_targets_locked(c._fixed_stance_locked())
        self.cycle(c)
        original = self.config_path.read_bytes()
        c.start_mode('hold', support_confirmed=True)
        self.clock.advance(3.1)
        self.cycle(c)
        self.assertTrue(c.status()['hold_ready'])
        self.policy.output[0, 5:9] = [.03, .02, .01, .04]
        expected = np.deg2rad([12.,15.,1.,2.]) - c._policy_profiles['walk']['home_rad'][5:9]
        c.start_mode('walk', support_confirmed=True)
        np.testing.assert_allclose(self.policy.observation[0,51:55], expected, atol=1e-7)
        self.cycle(c)
        np.testing.assert_allclose(self.policy.observation[0,51:55], expected, atol=1e-7)
        policy_targets = np.rad2deg(c._policy_profiles['walk']['home_rad'][5:9] + .55*self.policy.output[0,5:9])
        np.testing.assert_allclose(c.status()['last_action'][10:14], policy_targets, atol=1e-5)
        self.assertEqual(self.config_path.read_bytes(), original)

    def fixed_head_walk_controller(self, hold_hip_pitch_bias_deg=0):
        policies = {name: FakePolicy(name) for name in ('walk', 'getup', 'pick', 'roulade')}
        config = ready_config()
        # The model's unadapted neck HOME is unsafe here, while the captured
        # physical pose and mechanical zero remain within the saved range.
        config['joints'][10].update(min=2000, max=2250)
        self.config_path.write_text(json.dumps(config))
        c = self.controller(sessions=policies, require_head_clearance=True,
                            allow_fixed_head_walk=True, hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg)
        c._connect_once()
        t = self.transports[-1]
        t.positions[10:14] = [2180, 1900, 2055, 2040]
        self.cycle(c)
        c.capture_head_clearance(clearance_confirmed=True, expected_generation=c.status()['generation'])
        captured = copy.deepcopy(c.status()['hold_head_deg'])
        self.hold(c)
        for _ in range(155):
            self.cycle(c)
        self.assertTrue(c.status()['hold_ready'])
        return c, policies, t, captured

    def test_hold_hip_bias_blocks_all_live_models_before_inference_and_arm(self):
        c, policies, t, captured = self.fixed_head_walk_controller(hold_hip_pitch_bias_deg=-2)
        self.assertTrue(c.status()['hold_ready'])
        self.assertEqual(c.status()['hold_head_deg'], captured)
        self.assertFalse(c.status()['fixed_head_walk_allowed'])
        self.assertFalse(c.status()['fixed_head_walk_ready'])
        self.assertEqual(c.status()['hold_hip_pitch_bias_deg'], -2.)
        before = list(t.requests)
        for mode in ('walk', 'getup', 'pick', 'roulade'):
            with self.subTest(mode=mode), self.assertRaisesRegex(MotionError, '模型接管尚未审查'):
                c.start_mode(mode, support_confirmed=True, behavior_confirmed=True)
            self.assertEqual(policies[mode].calls, 0)
        self.assertEqual(t.requests, before)
        # The same gate protects the ongoing control boundary, even if a mode
        # were internally changed instead of using the public start method.
        c._mode = 'walk'
        c._heartbeat = self.clock()
        self.cycle(c)
        self.assertEqual(c.status()['mode'], 'off')
        self.assertIn('模型接管尚未审查', c.status()['error'])
        self.assertFalse(t.armed)
        self.assertFalse(any(kind in (ARM, TARGET) for kind, _ in t.requests[len(before):]))
        self.assertTrue(any(kind == STOP for kind, _ in t.requests[len(before):]))
        self.assertTrue(all(policy.calls == 0 for policy in policies.values()))
        # Offline preview remains available and keeps the unmodified model HOME.
        self.cycle(c)  # Observe the real OFF reply after the protection STOP.
        c.start_mode('shadow', preview_model='walk')
        self.cycle(c)
        np.testing.assert_allclose(c.status()['last_action'][:10], DEFAULT_DEG[:10])
        self.assertFalse(t.armed)

    def test_fixed_head_walk_preflight_and_live_targets_preserve_raw_policy_contract(self):
        c, policies, t, captured = self.fixed_head_walk_controller()
        original = self.config_path.read_bytes()
        policy = policies['walk']
        policy.output[0] = [.01, .02, -.03, .04, -.05, 2., -1.5, .6, -.8,
                            -.01, -.02, .03, -.04, .05]
        before = len(t.requests)
        first = c.start_mode('walk', support_confirmed=True)
        self.assertTrue(first['fixed_head_walk_allowed'])
        self.assertTrue(first['fixed_head_walk_ready'])
        self.assertTrue(first['fixed_head_walk_adapted'])
        self.assertEqual(first['fixed_head_walk_protocol'], 1)
        self.assertTrue(first['fixed_head_walk_zero_velocity_only'])
        self.assertEqual(first['command'], dict(forward=0., side=0., yaw=0.))
        self.assertIsNone(first['last_executed_target_deg'])
        self.assertEqual([r['id'] for r in first['last_policy_target_rejections']], [30])
        self.assertLess(first['last_policy_target_rejections'][0]['target_position'], 2000)
        self.assertFalse(any(kind in (ARM, TARGET) for kind, _ in t.requests[before:]),
                         'switching from held stance only checks the first frame')
        self.cycle(c)
        status = c.status()
        np.testing.assert_allclose(status['last_policy_target_deg'], first['last_policy_target_deg'])
        np.testing.assert_allclose(status['last_action'], status['last_policy_target_deg'])
        executed = status['last_executed_target_deg']
        for i, sid in enumerate(MOTOR_IDS):
            self.assertAlmostEqual(executed[i], captured[sid] if sid in captured else
                                   status['last_policy_target_deg'][i])
        wire = [struct.unpack('<I15H', payload)[1:] for kind, payload in t.requests[before:]
                if kind == TARGET]
        self.assertEqual(len(wire), 1)
        self.assertEqual(list(wire[0]), c._raw_targets_locked(executed))
        np.testing.assert_allclose(c._previous_action, policy.output[0], atol=1e-7)
        self.cycle(c)
        expected_head = np.deg2rad([captured[sid] for sid in (30,31,32,33)]) - c._policy_profiles['walk']['home_rad'][5:9]
        np.testing.assert_allclose(policy.observation[0,51:55], expected_head, atol=1e-7)
        np.testing.assert_allclose(policy.observation[0,34:48], policy.output[0], atol=1e-7)
        np.testing.assert_allclose(policy.observation[0,11:15], expected_head, atol=1e-7)
        self.assertEqual(self.config_path.read_bytes(), original)

    def test_fixed_head_walk_requires_explicit_flag_and_current_connection_capture(self):
        for allowed in (False, 1, True):
            c = self.controller(ready=True, require_head_clearance=True,
                                allow_fixed_head_walk=allowed)
            c._connect_once()
            self.assertEqual(c.status()['fixed_head_walk_allowed'], allowed is True)
            self.assertFalse(c.status()['fixed_head_walk_ready'])
            with self.assertRaisesRegex(MotionError, '机械间隙尚未验证'):
                c.start_mode('walk', support_confirmed=True)
            c.capture_head_clearance(clearance_confirmed=True, expected_generation=c.status()['generation'])
            self.assertEqual(c.status()['fixed_head_walk_ready'], allowed is True)
            self.assertFalse(c.status()['fixed_head_walk_adapted'])
            if allowed is not True:
                with self.assertRaisesRegex(MotionError, '机械间隙尚未验证'):
                    c.start_mode('walk', support_confirmed=True)
            c._disconnect('fake connection ended')
            self.assertIsNone(c.status()['hold_head_deg'])
            self.assertFalse(c.status()['fixed_head_walk_ready'])
            c._connect_once()
            self.assertIsNone(c.status()['hold_head_deg'])
            self.assertFalse(c.status()['fixed_head_walk_ready'])
            self.assertFalse(any(kind in (ARM,TARGET) for kind, _ in self.transports[-1].requests))

    def test_fixed_head_walk_does_not_unlock_other_models(self):
        c, policies, t, _ = self.fixed_head_walk_controller()
        before = len(t.requests)
        for mode in ('getup', 'pick', 'roulade'):
            with self.assertRaisesRegex(MotionError, '机械间隙尚未验证'):
                c.start_mode(mode, support_confirmed=True, behavior_confirmed=True)
            self.assertEqual(policies[mode].calls, 0)
        self.assertFalse(any(kind in (ARM,TARGET) for kind, _ in t.requests[before:]))

    def test_fixed_head_walk_head_drift_stops_before_another_target(self):
        c, _, t, captured = self.fixed_head_walk_controller()
        c.start_mode('walk', support_confirmed=True)
        self.cycle(c)
        for sid in (30,31,32,33):
            i = MOTOR_IDS.index(sid)
            actual = c._filtered.angles[i]
            c._filtered.angles[i] = captured[sid] + 2.
            with self.assertRaisesRegex(MotionError, '偏离记录位置达到 2°'):
                c._motion_gate_locked('walk')
            c._filtered.angles[i] = actual
        t.positions[MOTOR_IDS.index(30)] += 23  # 2.02°, still inside the saved range.
        before = len(t.requests)
        self.cycle(c)
        status = c.status()
        self.assertEqual(status['mode'], 'off')
        self.assertIn('偏离记录位置达到 2°', status['error'])
        self.assertFalse(status['fixed_head_walk_adapted'])
        self.assertFalse(t.armed)
        self.assertEqual(t.torque, 0)
        self.assertTrue(any(kind == STOP for kind, _ in t.requests[before:]))
        self.assertFalse(any(kind == TARGET for kind, _ in t.requests[before:]))

    def test_fixed_head_walk_rejects_unsafe_leg_preflight_without_clamping(self):
        c, policies, t, _ = self.fixed_head_walk_controller()
        policies['walk'].output[0,4] = 5.
        before = len(t.requests)
        with self.assertRaisesRegex(MotionError, '启动前检查未通过：ID 14'):
            c.start_mode('walk', support_confirmed=True)
        status = c.status()
        self.assertEqual(status['mode'], 'hold')
        self.assertEqual(status['target_rejection']['id'], 14)
        self.assertLess(status['target_rejection']['target_position'], 100)
        self.assertIn(14, [r['id'] for r in status['last_policy_target_rejections']])
        self.assertFalse(any(kind in (ARM,TARGET) for kind, _ in t.requests[before:]))

    def test_fixed_head_walk_stop_during_preflight_discards_adapted_start(self):
        c, policies, t, _ = self.fixed_head_walk_controller()
        policy = policies['walk']
        policy.entered, policy.release = threading.Event(), threading.Event()
        errors = []
        def start():
            try:
                c.start_mode('walk', support_confirmed=True)
            except MotionError as error:
                errors.append(str(error))
        before = len(t.requests)
        thread = threading.Thread(target=start)
        thread.start()
        self.assertTrue(policy.entered.wait(1))
        c.stop()
        policy.release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIn('状态已变更', errors[0])
        self.assertEqual(c.status()['mode'], 'off')
        self.assertFalse(any(kind in (ARM,TARGET) for kind, _ in t.requests[before:]))

    def test_fixed_head_walk_stop_during_live_inference_discards_adapted_target(self):
        c, policies, t, _ = self.fixed_head_walk_controller()
        c.start_mode('walk', support_confirmed=True)
        policy = policies['walk']
        policy.entered, policy.release = threading.Event(), threading.Event()
        before = len(t.requests)
        thread = threading.Thread(target=c._control_once)
        thread.start()
        self.assertTrue(policy.entered.wait(1))
        c.stop()
        policy.release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(c.status()['mode'], 'off')
        self.assertFalse(t.armed)
        self.assertFalse(any(kind in (ARM,TARGET) for kind, _ in t.requests[before:]))

    def test_fixed_head_walk_rejects_walking_commands_but_shadow_and_original_walk_do_not(self):
        c, _, t, _ = self.fixed_head_walk_controller()
        c.start_mode('walk', support_confirmed=True)
        self.assertEqual(c.command(0, 0, 0)['command'], dict(forward=0., side=0., yaw=0.))
        before = len(t.requests)
        for command in ((.01,0,0), (0,-.01,0), (0,0,.01), (1e-12,0,0)):
            with self.assertRaisesRegex(MotionError, '仅允许零速度站立平衡；行走尚未验证'):
                c.command(*command)
            self.assertEqual(c.status()['command'], dict(forward=0., side=0., yaw=0.))
        self.assertEqual(len(t.requests), before)
        c.stop()
        self.cycle(c)
        c.start_mode('shadow', preview_model='walk')
        self.assertEqual(c.command(.1, -.05, .2)['command'],
                         dict(forward=float(np.float32(.1)), side=float(np.float32(-.05)), yaw=float(np.float32(.2))))
        original = self.controller(ready=True)
        original._connect_once()
        self.walk(original)
        self.assertFalse(original.status()['fixed_head_walk_adapted'])
        self.assertEqual(original.command(.1, -.05, .2)['command'],
                         dict(forward=float(np.float32(.1)), side=float(np.float32(-.05)), yaw=float(np.float32(.2))))

    def test_minimal_heartbeat_checks_generation_and_renews_without_status(self):
        c = self.controller(ready=True)
        c._connect_once()
        self.hold(c)
        generation = c.status()['generation']
        previous = c._heartbeat
        self.clock.advance(.1)
        for stale in (generation - 1, True):
            with self.assertRaisesRegex(MotionError, 'Motion state changed'):
                c.heartbeat(expected_generation=stale, minimal=True)
            self.assertEqual(c._heartbeat, previous)
        with mock.patch.object(c, 'status', side_effect=AssertionError('minimal heartbeat called status')):
            result = c.heartbeat(expected_generation=generation, minimal=True)
        self.assertEqual(result, dict(generation=generation, mode='hold', connected=True))
        self.assertEqual(c._heartbeat, self.clock())
        full = c.heartbeat()
        self.assertIn('joints', full)
        self.assertEqual(full['generation'], generation)

    def test_adapted_head_preview_still_reports_unclamped_out_of_range_output(self):
        config = ready_config()
        config['joints'][10].update(zero=1977, min=1846, max=3146)
        self.config_path.write_text(json.dumps(config))
        c = self.controller(require_head_clearance=True)
        c._connect_once()
        t = self.transports[0]
        t.positions[10] = 2033
        self.cycle(c)
        c.capture_head_clearance(clearance_confirmed=True, expected_generation=c.status()['generation'])
        c.start_mode('shadow', preview_model='walk')
        self.cycle(c)
        result = c.status()
        self.assertIn('ID 30', result['error'])
        self.assertIn('未发送舵机目标', result['error'])
        self.assertEqual(result['target_rejection']['target_position'], 1750)
        self.assertAlmostEqual(result['last_action'][10], DEFAULT_DEG[10], places=5)
        self.assertFalse(any(kind in (ARM, TARGET) for kind, _ in t.requests))

    def test_forward_hold_shift_preserves_foot_pitch_policy_home_and_recorded_limits(self):
        config = ready_config()
        rows = {12: (1982, -1, 1617, 2882), 14: (2371, -1, 1983, 3212),
                22: (3192, -1, 2296, 3606), 24: (2093, -1, 1264, 2769)}
        for joint in config['joints']:
            if joint['id'] in rows:
                zero, sign, low, high = rows[joint['id']]
                joint.update(zero=zero, sign=sign, min=low, max=high)
        self.config_path.write_text(json.dumps(config))
        original = self.config_path.read_bytes()
        c = self.controller()
        c._connect_once()
        target = c.status()['stand_target_deg']
        np.testing.assert_allclose(c._stand_deg, DEFAULT_DEG)
        # Mirrored hip/ankle rotations cancel: no new pitch error in either foot.
        self.assertAlmostEqual(target[2] + target[4], DEFAULT_DEG[2] + DEFAULT_DEG[4])
        self.assertAlmostEqual(target[7] + target[9], DEFAULT_DEG[7] + DEFAULT_DEG[9])
        raw = c._raw_targets_locked(target)
        self.assertEqual({sid: raw[MOTOR_IDS.index(sid)] for sid in rows},
                         {12: 2198, 14: 2155, 22: 2976, 24: 2309})
        self.transports[0].positions = [joint['zero'] for joint in config['joints']]
        self.cycle(c)
        self.hold(c)
        t = self.transports[0]
        for _ in range(155):
            self.cycle(c)
        self.assertTrue(c.status()['hold_ready'])
        frames = [struct.unpack('<I15H', payload)[1:] for kind, payload in t.requests if kind == TARGET]
        self.assertTrue(frames)
        for sid, (_, _, low, high) in rows.items():
            index = MOTOR_IDS.index(sid)
            self.assertTrue(all(low <= frame[index] <= high for frame in frames))
            self.assertEqual(t.positions[index], raw[index])
        self.assertEqual(self.config_path.read_bytes(), original)
        self.assertEqual(self.policy.calls, 0)

    def test_head_clearance_rejects_unconfirmed_stale_faulty_and_moved_pose(self):
        c = self.controller(ready=True, require_head_clearance=True)
        c._connect_once()
        generation = c.status()['generation']
        for confirmation, expected in ((False, generation), (True, generation - 1), (True, True)):
            with self.assertRaises(MotionError):
                c.capture_head_clearance(clearance_confirmed=confirmation, expected_generation=expected)
        self.clock.advance(.101)
        with self.assertRaisesRegex(MotionError, '过期'):
            c.capture_head_clearance(clearance_confirmed=True, expected_generation=generation)
        self.cycle(c)
        for key, value in (('fault_mask', 1), ('online_mask', 1), ('torque_mask', 1), ('armed', True), ('last_error', 2)):
            previous = c._state[key]
            c._state[key] = value
            with self.assertRaises(MotionError):
                c.capture_head_clearance(clearance_confirmed=True, expected_generation=generation)
            c._state[key] = previous
        c.capture_head_clearance(clearance_confirmed=True, expected_generation=generation)
        self.transports[0].positions[10] += 30
        self.cycle(c)
        with self.assertRaisesRegex(MotionError, '头颈已离开'):
            self.hold(c)
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[0].requests])

    def cycle(self, controller, dt=.02):
        self.clock.advance(dt)
        controller._cycle()

    def hold(self, controller):
        controller.start_mode('hold', support_confirmed=True)

    def walk(self, controller):
        self.hold(controller)
        for _ in range(155):
            controller.heartbeat()
            self.cycle(controller)
        self.assertTrue(controller.status()['hold_ready'])
        controller.start_mode('walk', support_confirmed=True)

    def all_models(self, standing=False):
        policies = {name: FakePolicy(name) for name in ('walk','getup','pick','roulade')}
        controller = self.controller(ready=True, sessions=policies)
        controller._connect_once()
        if standing:
            self.hold(controller)
            for _ in range(155):
                controller.heartbeat()
                self.cycle(controller)
            self.assertTrue(controller.status()['hold_ready'])
        return controller, policies, self.transports[-1]

    def test_all_four_preview_models_use_correct_session_without_motor_commands(self):
        controller, policies, transport = self.all_models()
        before = list(transport.requests)
        for name, policy in policies.items():
            controller.start_mode('shadow', preview_model=name)
            self.cycle(controller)
            self.assertIsNotNone(policy.observation)
            self.assertEqual(controller.status()['active_model'], name)
            self.assertFalse(controller.status()['motor_output'])
            controller.stop()
        self.assertFalse(any(k in (ARM,TARGET) for k,_ in transport.requests[len(before):]))

    def test_model_start_requires_loaded_policy_stand_and_behavior_confirmation(self):
        controller, policies, transport = self.all_models()
        for name in ('pick','roulade'):
            with self.assertRaises(MotionError):
                controller.start_mode(name, support_confirmed=True, behavior_confirmed=True)
        for name in ('getup','roulade'):
            with self.assertRaisesRegex(MotionError, '防跌保护'):
                controller.start_mode(name, support_confirmed=True)
        del controller.sessions['getup']
        with self.assertRaisesRegex(MotionError, '模型未加载'):
            controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
        self.assertNotIn(ARM, [k for k,_ in transport.requests])

    def test_hold_cycle_allows_below_15_and_stops_at_boundary_without_later_targets(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        transport = self.transports[-1]
        # Keep STATE acceptance/filtering and the full cycle intact; supply
        # exact gate angles independently of acos representation rounding.
        with mock.patch.object(controller, '_tilt_locked', return_value=14.9):
            self.hold(controller)
            before = len(transport.requests)
            self.cycle(controller)
            self.assertEqual([kind for kind, _ in transport.requests[before:]], [STATE, TARGET])
            self.assertEqual(controller.status()['mode'], 'hold')
        generation = controller.status()['generation']
        target_count = sum(kind == TARGET for kind, _ in transport.requests)
        arm_count = sum(kind == ARM for kind, _ in transport.requests)
        with mock.patch.object(controller, '_tilt_locked', return_value=15.):
            before = len(transport.requests)
            self.cycle(controller)
            self.assertEqual([kind for kind, _ in transport.requests[before:]], [STATE, STOP])
            self.assertEqual(controller.status()['mode'], 'off')
            self.assertEqual(controller.status()['generation'], generation + 1)
            self.assertIn('超过 15', controller.status()['error'])
            self.assertFalse(controller.status()['hold_ready'])
            for _ in range(3):
                self.cycle(controller)
        self.assertFalse(transport.armed)
        self.assertEqual(transport.torque, 0)
        self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), target_count)
        self.assertEqual(sum(kind == ARM for kind, _ in transport.requests), arm_count)

    def test_hold_ready_requires_finite_total_tilt_below_15_even_when_joints_arrive(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        self.hold(controller)
        for _ in range(155):
            self.cycle(controller)
        self.assertTrue(controller.status()['hold_ready'])
        self.assertLess(max(abs(a - b) for a, b in zip(
            controller._filtered.angles, controller._fixed_stance_locked())), .1)
        before = list(self.transports[-1].requests)
        controller._filtered.grav = np.asarray([math.sin(math.radians(33)), 0.,
                                               -math.cos(math.radians(33))])
        self.assertAlmostEqual(controller._tilt_locked(), 33.)
        self.assertFalse(controller.status()['hold_ready'])
        for tilt, ready in ((14.9, True), (15., False), (float('nan'), False),
                            (float('inf'), False), (None, False)):
            with self.subTest(tilt=tilt), mock.patch.object(controller, '_tilt_locked', return_value=tilt):
                self.assertEqual(controller.status()['hold_ready'], ready)
        self.assertEqual(self.transports[-1].requests, before)

    def test_hold_cycle_uses_received_imu_tilt_before_any_target(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        transport = self.transports[-1]
        transport.acc = [9.8 * math.sin(math.radians(14.9)), 0.,
                         9.8 * math.cos(math.radians(14.9))]
        controller._filter._gravity = None  # Start the actual filter from this sample.
        self.cycle(controller)
        self.assertAlmostEqual(controller._tilt_locked(), 14.9, places=4)
        self.hold(controller)
        self.cycle(controller)
        before = len(transport.requests)
        target_count = sum(kind == TARGET for kind, _ in transport.requests)
        transport.acc = [9.8 * math.sin(math.radians(15.1)), 0.,
                         9.8 * math.cos(math.radians(15.1))]
        controller._filter._gravity = None
        self.cycle(controller)
        self.assertAlmostEqual(controller._tilt_locked(), 15.1, places=4)
        self.assertEqual([kind for kind, _ in transport.requests[before:]], [STATE, STOP])
        self.assertEqual(controller.status()['mode'], 'off')
        self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), target_count)

    def test_hold_limit_does_not_change_walk_pick_or_inverted_behavior_cycle_limits(self):
        for mode, below, boundary in (('walk', 39.9, 40.), ('pick', 54.9, 55.)):
            with self.subTest(mode=mode):
                controller, _, transport = self.all_models(standing=True)
                controller.start_mode(mode, support_confirmed=True)
                before = sum(kind == TARGET for kind, _ in transport.requests)
                with mock.patch.object(controller, '_tilt_locked', return_value=below):
                    self.cycle(controller)
                self.assertEqual(controller.status()['mode'], mode)
                self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), before + 1)
                with mock.patch.object(controller, '_tilt_locked', return_value=boundary):
                    self.cycle(controller)
                self.assertEqual(controller.status()['mode'], 'off')
                self.assertIn('超过 ' + str(int(boundary)), controller.status()['error'])
                self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), before + 1)
        for mode in ('getup', 'roulade'):
            with self.subTest(mode=mode):
                controller, _, transport = self.all_models(standing=mode == 'roulade')
                controller.start_mode(mode, support_confirmed=True, behavior_confirmed=True)
                before = sum(kind == TARGET for kind, _ in transport.requests)
                with mock.patch.object(controller, '_tilt_locked', return_value=180.):
                    self.cycle(controller)
                self.assertEqual(controller.status()['mode'], mode)
                self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), before + 1)

    def test_pick_phase_and_mouth_finish_once_without_automatic_walk(self):
        controller, policies, transport = self.all_models(standing=True)
        calibration_before = self.config_path.read_bytes()
        arms_before = sum(k==ARM for k,_ in transport.requests)
        controller.start_mode('pick', support_confirmed=True)
        self.cycle(controller)
        np.testing.assert_allclose(policies['pick'].observation[0,48:51],
                                   [np.cos(.02/4*2*np.pi),np.sin(.02/4*2*np.pi),0],atol=1e-6)
        self.assertEqual(controller.status()['last_action'][-1], 30)
        self.assertIsNone(policies['walk'].observation)
        for _ in range(81):
            controller.heartbeat(); self.cycle(controller)
        self.assertEqual(controller.status()['last_action'][-1], 0)
        for _ in range(120):
            controller.heartbeat(); self.cycle(controller)
        self.assertEqual(controller.status()['mode'],'off')
        self.assertIn('俯身本轮模型运行结束', controller.status()['behavior_result'])
        self.assertFalse(transport.armed)
        self.assertEqual(transport.torque,0)
        self.assertEqual(sum(k==ARM for k,_ in transport.requests),arms_before)
        self.assertEqual(calibration_before,self.config_path.read_bytes())

    def test_roulade_uses_own_policy_and_bounded_episode_with_tilt_gate_isolation(self):
        controller, policies, transport = self.all_models(standing=True)
        policies['roulade'].output.fill(.1)
        controller.start_mode('roulade', support_confirmed=True, behavior_confirmed=True)
        controller._filtered.grav = [1.,0.,0.]
        controller._control_once()
        np.testing.assert_allclose(controller._smooth_action, .085, atol=1e-6)
        np.testing.assert_allclose(policies['roulade'].observation[0,48:55], np.zeros(7))
        with self.assertRaisesRegex(MotionError,'超过 40'):
            controller._motion_gate_locked('walk')
        with self.assertRaisesRegex(MotionError,'超过 15'):
            controller._motion_gate_locked('hold')
        for _ in range(97):
            controller.heartbeat(); self.cycle(controller)
        self.assertEqual(controller.status()['mode'],'off')
        self.assertFalse(transport.armed)
        self.assertIsNone(policies['walk'].observation)

    def test_getup_default_pose_then_recovery_policy_stops_after_actual_upright(self):
        controller, policies, transport = self.all_models()
        controller._filtered.grav = [1.,0.,0.]
        controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
        self.assertEqual(policies['getup'].calls, 1)  # motor-free preflight
        self.cycle(controller)
        self.assertEqual(policies['getup'].calls, 1)
        np.testing.assert_allclose(controller.status()['last_action'],DEFAULT_DEG)
        for _ in range(52):
            controller.heartbeat(); self.cycle(controller)
        self.assertIsNotNone(policies['getup'].observation)
        np.testing.assert_allclose(policies['getup'].observation[0,48:55], np.zeros(7))
        for _ in range(55):
            controller.heartbeat(); self.cycle(controller)
        self.assertEqual(controller.status()['mode'],'off')
        self.assertIn('起身本轮模型运行结束',controller.status()['behavior_result'])
        self.assertFalse(transport.armed)

    def test_getup_outside_saved_neck_limit_is_rejected_before_arm_or_default_pose(self):
        controller, policies, transport = self.all_models()
        neck = controller._config['joints'][MOTOR_IDS.index(30)]
        neck.update(zero=1995, sign=-1, min=1626, max=2990)
        policies['getup'].output[0, 5] = 1.6767
        before = list(transport.requests)
        config = copy.deepcopy(controller._config)
        with self.assertRaisesRegex(MotionError, '起身启动前检查未通过：ID 30.*允许 1626–2990'):
            controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
        self.assertFalse(any(k in (ARM, TARGET) for k, _ in transport.requests[len(before):]))
        self.assertEqual(controller.status()['mode'], 'off')
        self.assertFalse(transport.armed)
        self.assertEqual(controller._config, config)
        rejection = controller.status()['target_rejection']
        self.assertEqual(rejection['id'], 30)
        self.assertEqual(rejection['mode'], 'getup')
        self.assertEqual(rejection['target_position'], 1167)
        self.assertAlmostEqual(rejection['target_angle_deg'], math.degrees(.349 + 1.6767 * .55), places=4)
        np.testing.assert_array_equal(controller._previous_action, np.zeros(14))
        np.testing.assert_array_equal(controller._smooth_action, np.zeros(14))

    def test_preview_reports_model_target_limit_without_arm_target_or_clamping(self):
        controller, policies, transport = self.all_models()
        neck = controller._config['joints'][MOTOR_IDS.index(30)]
        neck.update(zero=1995, sign=-1, min=1626, max=2990)
        policies['getup'].output[0, 5] = 1.6767
        before = list(transport.requests)
        controller.start_mode('shadow', preview_model='getup')
        self.cycle(controller)
        status = controller.status()
        self.assertEqual(status['mode'], 'shadow')
        self.assertIn('不动作预览：ID 30', status['error'])
        self.assertIn('未发送舵机目标', status['error'])
        self.assertEqual(status['target_rejection']['target_position'], 1167)
        self.assertGreater(status['last_action'][MOTOR_IDS.index(30)], 72)
        self.assertFalse(any(k in (ARM, TARGET) for k, _ in transport.requests[len(before):]))

    def test_stop_during_start_preflight_invalidates_start_without_arm(self):
        controller, policies, transport = self.all_models()
        policy = policies['getup']
        policy.entered, policy.release = threading.Event(), threading.Event()
        errors = []
        def start():
            try:
                controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
            except MotionError as error:
                errors.append(str(error))
        before = list(transport.requests)
        thread = threading.Thread(target=start)
        thread.start()
        self.assertTrue(policy.entered.wait(1))
        stopped = controller.stop()
        self.assertEqual(stopped['mode'], 'off')
        policy.release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIn('状态已变更', errors[0])
        self.assertFalse(any(k in (ARM, TARGET) for k, _ in transport.requests[len(before):]))

    def test_slow_start_preflight_cannot_arm_using_stale_input(self):
        controller, policies, transport = self.all_models()
        policy = policies['getup']
        original_run = policy.run
        def slow_run(*args):
            result = original_run(*args)
            self.clock.advance(.2)
            return result
        policy.run = slow_run
        before = list(transport.requests)
        with self.assertRaisesRegex(MotionError, '检查耗时过长'):
            controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
        self.assertFalse(any(k in (ARM, TARGET) for k, _ in transport.requests[len(before):]))
        self.assertEqual(controller.status()['mode'], 'off')

    def test_safe_first_frame_does_not_allow_a_later_unsafe_policy_target(self):
        controller, policies, transport = self.all_models()
        neck = controller._config['joints'][MOTOR_IDS.index(30)]
        neck.update(zero=1995, sign=-1, min=1626, max=2990)
        controller.start_mode('getup', support_confirmed=True, behavior_confirmed=True)
        self.cycle(controller)
        policies['getup'].output[0, 5] = 1.6767
        self.clock.advance(1.01)
        controller._accept_state_locked(decode_state(transport.request(STATE)))
        controller.heartbeat()
        before = sum(k == TARGET for k, _ in transport.requests)
        with self.assertRaisesRegex(MotionError, 'ID 30.*目标 1167 ticks'):
            controller._control_once()
        self.assertEqual(sum(k == TARGET for k, _ in transport.requests), before)
        self.assertEqual(controller.status()['target_rejection']['target_position'], 1167)

    def test_getup_timeout_cannot_claim_success_or_start_walk(self):
        controller, policies, transport = self.all_models()
        controller.start_mode('getup',support_confirmed=True,behavior_confirmed=True)
        for _ in range(401):
            controller.heartbeat()
            self.clock.advance(.02)
            controller._accept_state_locked(decode_state(transport.request(STATE)))
            controller._filtered.grav = [1.,0.,0.]
            controller._control_once()
        self.assertEqual(controller.status()['mode'],'off')
        self.assertIn('超时', controller.status()['error'])
        self.assertFalse(transport.armed)
        self.assertIsNone(policies['walk'].observation)

    def test_each_episode_stops_for_fault_and_discards_inference_after_stop(self):
        for mode in ('getup','pick','roulade'):
            with self.subTest(mode=mode):
                controller, policies, transport = self.all_models(standing=mode!='getup')
                controller.start_mode(mode,support_confirmed=True,behavior_confirmed=True)
                if mode=='getup':
                    for _ in range(51): controller.heartbeat(); self.cycle(controller)
                policy=policies[mode]
                policy.entered,policy.release=threading.Event(),threading.Event()
                before=sum(k==TARGET for k,_ in transport.requests)
                thread=threading.Thread(target=controller._control_once);thread.start()
                self.assertTrue(policy.entered.wait(1))
                controller.stop();policy.release.set();thread.join(1)
                self.assertFalse(thread.is_alive())
                self.assertEqual(sum(k==TARGET for k,_ in transport.requests),before)
                self.assertFalse(transport.armed)
                controller,_,transport=self.all_models(standing=mode!='getup')
                controller.start_mode(mode,support_confirmed=True,behavior_confirmed=True)
                transport.fault=1
                controller.heartbeat();self.cycle(controller)
                self.assertEqual(controller.status()['mode'],'off')
                self.assertFalse(transport.armed)

    def test_construction_is_inert_and_reconnect_never_arms(self):
        controller = self.controller(ready=True)
        self.assertEqual(self.transports, [])
        controller._connect_once()
        transport = self.transports[-1]
        self.assertEqual([kind for kind, _ in transport.requests[:3]], [STOP, HELLO, STATE])
        self.assertIn(CONFIG, [kind for kind, _ in transport.requests])
        self.assertNotIn(ARM, [kind for kind, _ in transport.requests])
        self.hold(controller)
        controller._disconnect('lost link')
        controller._connect_once()
        self.assertEqual(controller.status()['mode'], 'off')
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[-1].requests])

    def test_incomplete_calibration_cannot_arm_and_capture_is_read_only(self):
        controller = self.controller()
        controller._connect_once()
        controller.capture(10, 'zero')
        self.assertEqual(controller.status()['joints'][0]['zero'], 2047)
        with self.assertRaises(MotionError):
            self.hold(controller)
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[-1].requests])
        self.assertNotIn(TARGET, [kind for kind, _ in self.transports[-1].requests])
        self.assertIsNotNone(json.loads(self.config_path.read_text())['joints'][0]['zero'])
        controller.start_mode('shadow')
        self.cycle(controller)
        self.assertEqual(controller.status()['mode'], 'shadow')
        self.assertIsNone(controller.status()['last_action'])

    def test_support_reference_direction_and_default_pose_gates(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        with self.assertRaises(MotionError):
            controller.start_mode('hold')
        controller.configure_reference(False)
        with self.assertRaises(MotionError):
            self.hold(controller)
        controller.configure_reference(True)
        controller.configure_joint(10, -1, False)
        with self.assertRaises(MotionError):
            self.hold(controller)
        controller.configure_joint(10, -1, True)
        self.cycle(controller)
        # ID 12's model home near -20 deg moves raw positive; a narrow upper limit must fail.
        controller._config['joints'][2]['max'] = 2050
        with self.assertRaises(MotionError):
            self.hold(controller)
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[-1].requests])

    def test_corrupt_saved_calibration_cannot_arm(self):
        controller = self.controller(ready=True)
        invalid = ready_config()
        invalid['imu']['bias_dps'][0] = float('nan')
        self.config_path.write_text(json.dumps(invalid))
        controller = self.controller()
        self.assertIn('校准文件', controller.status()['error'])
        self.assertFalse(controller.status()['calibrated'])
        controller._connect_once()
        with self.assertRaises(MotionError):
            self.hold(controller)
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[-1].requests])

    def test_imu_newer_than_servo_timestamp_and_uint32_rollover(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        transport = self.transports[-1]
        transport.imu_age_us = -5000  # IMU read completes after the servo sample.
        self.cycle(controller)
        self.assertLess(controller.status()['imu_age_ms'], 1.)
        transport.sample_us = 0xffffffff - 15000
        transport.imu_age_us = 1000
        self.cycle(controller)
        self.assertAlmostEqual(controller.status()['imu_age_ms'], 1., places=2)
        self.hold(controller)

    def test_mouth_closed_zero_can_be_at_mechanical_endpoint(self):
        config = ready_config()
        config['joints'][-1].update(zero=2047, min=100, max=2047)
        self.config_path.write_text(json.dumps(config))
        controller = self.controller()
        controller._connect_once()
        self.assertTrue(controller.status()['joints'][-1]['ready'])
        self.hold(controller)
        self.assertTrue(controller.status()['motor_output'])

    def test_stop_invalidates_an_already_queued_start_generation(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        old_generation = controller.status()['generation']
        controller.stop()
        with self.assertRaises(MotionError):
            controller.start_mode('hold', support_confirmed=True, expected_generation=old_generation)
        self.assertNotIn(ARM, [kind for kind, _ in self.transports[-1].requests])
        controller.start_mode('hold', support_confirmed=True,
                              expected_generation=controller.status()['generation'])
        self.assertTrue(controller.status()['motor_output'])

    def test_initial_arm_preserves_measurement_hold_ramps_then_walks(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        self.hold(controller)
        transport = self.transports[-1]
        self.assertEqual(transport.positions, [2047] * 15)
        self.assertNotIn(TARGET, [kind for kind, _ in transport.requests])
        self.cycle(controller)
        self.assertLess(abs(controller.status()['last_action'][2]), 1)
        with self.assertRaises(MotionError):
            controller.start_mode('walk', support_confirmed=True)
        for _ in range(155):
            controller.heartbeat()
            self.cycle(controller)
        controller.start_mode('walk', support_confirmed=True)
        self.cycle(controller)
        self.assertEqual(self.policy.observation.shape, (1, 61))
        np.testing.assert_allclose(self.policy.observation[0, 3:6], [0, 0, -1], atol=1e-6)
        np.testing.assert_allclose(self.policy.observation[0, :3], [0, 0, 0], atol=1e-6)

    def test_status_polling_does_not_feed_watchdog(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        self.walk(controller)
        for _ in range(10):
            controller.status()
        self.cycle(controller, HEARTBEAT_SECONDS + .01)
        self.assertEqual(controller.status()['mode'], 'off')
        self.assertFalse(self.transports[-1].armed)
        self.assertIn('心跳', controller.status()['error'])

    def test_stale_sample_missing_servo_or_fault_stops(self):
        for issue in ('frozen', 'offline', 'fault', 'imu_stale'):
            with self.subTest(issue=issue):
                controller = self.controller(ready=True)
                controller._connect_once()
                self.hold(controller)
                transport = self.transports[-1]
                if issue == 'frozen':
                    transport.freeze_sample = True
                    dt = .11
                else:
                    dt = .02
                    if issue == 'offline': transport.online &= ~1
                    elif issue == 'fault': transport.fault = 1
                    else: transport.imu_age_us = 110000
                controller.heartbeat()
                self.cycle(controller, dt)
                self.assertEqual(controller.status()['mode'], 'off')
                self.assertFalse(transport.armed)

    def test_imu_calibration_distinct_samples_correct_specific_force(self):
        controller = self.controller()
        controller._connect_once()
        controller.set_imu_axes(['x', 'y', 'z'], axes_confirmed=True)
        controller.calibrate_imu()
        for _ in range(101):
            self.cycle(controller)
        self.assertTrue(controller.status()['imu']['calibrated'])
        np.testing.assert_allclose(controller.status()['imu']['bias_dps'], [.1, -.2, .3], atol=1e-6)
        self.assertFalse(controller.status()['calibrated'])  # Joint reference remains unset.
        controller.set_imu_axes(['x', 'y', 'z'], axes_confirmed=True)
        self.transports[-1].acc = [0, 0, -9.8]
        controller.calibrate_imu()
        for _ in range(101): self.cycle(controller)
        self.assertFalse(controller.status()['imu']['calibrated'])
        self.assertIn('轴向', controller.status()['error'])

    def test_stop_during_policy_inference_discards_pending_target(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        self.walk(controller)
        self.policy.entered, self.policy.release = threading.Event(), threading.Event()
        transport = self.transports[-1]
        before = sum(kind == TARGET for kind, _ in transport.requests)
        thread = threading.Thread(target=controller._control_once)
        thread.start()
        self.assertTrue(self.policy.entered.wait(1))
        controller.stop()
        self.policy.release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(sum(kind == TARGET for kind, _ in transport.requests), before)
        self.assertEqual(controller.status()['mode'], 'off')
        self.assertFalse(transport.armed)

    def test_commands_clamped_and_unsupported_actions_rejected(self):
        controller = self.controller(ready=True)
        controller._connect_once()
        controller.start_mode('shadow')
        command = controller.command(100, -100, 100)['command']
        self.assertAlmostEqual(command['forward'], .2)
        self.assertAlmostEqual(command['side'], -.1)
        self.assertAlmostEqual(command['yaw'], .5)
        for mode in ('getup', 'pick', 'roulade'):
            with self.assertRaises(MotionError):
                controller.start_mode(mode, support_confirmed=True)
        with self.assertRaises(MotionError):
            controller.command(float('nan'), 0, 0)


if __name__ == '__main__':
    unittest.main()
