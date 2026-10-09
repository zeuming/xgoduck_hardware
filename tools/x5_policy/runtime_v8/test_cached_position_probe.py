"""CPU-only cache probe tests: every UART fd, clock and I/O call is fake."""
import ast
import copy
import json
from pathlib import Path
import struct
import termios
import types
import unittest
from unittest import mock

import cached_position_probe as probe

D = Path(__file__).resolve().parent
V5 = D.parent / 'bounded-forward-v5'
SEAL = '6bcd0d1b2ec6b7fe2c492146bc9bafff8cb608cab3e38c03177b2a1e787785f1'
BOOT = 'ad81aebd-5571-4093-a128-fb4222934bee'
CID = '035344534b33324785d55cb687017c00'
CAL = '96134fd339f0d71ae7af1fc2f80718b2a8e442cd0ebcc8bef34b6f581f42c1d7'


def cached_packet(sid, position, tick, rejected=False):
    return bytes([1, sid, 56, 6, 11 if rejected else 3, 6, 0, 7 if rejected else 1]) + \
        struct.pack('<2I3H', tick, tick, position, 32769, 123)


class CachedProbeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.tick = 1000000
        self.calls = []
        self.ns = dict(bind_serial=lambda expected: None,
            serial_proof=lambda expected, fd=None: (types.SimpleNamespace(st_rdev=12), {}),
            serial_path=lambda expected: '/dev/ttyUSB0', _holders=lambda *args: [],
            _modem_bits=lambda fd: dict(dtr=True, rts=True),
            _raw_settings=lambda fd: dict(baud=921600, hupcl=False),
            _decode_state=lambda payload: payload,
            _hello_feedback=lambda fd, metadata: self.calls.append((1, 1)),
            _exchange=self.exchange)
        self.guard = mock.Mock(return_value=dict(boot_id='same'))

    def state(self):
        self.tick += 20000
        return dict(sample_us=self.tick, imu_us=self.tick, online_mask=32639,
                    fault_mask=0, torque_mask=0, armed=False, arm_cookie=0,
                    last_error=0, imu_ok=True, accel_ms2=[0., 0., 9.8], gyro_dps=[0., 0., 0.])

    def exchange(self, fd, kind, sequence, deadline, metadata, **options):
        self.calls.append((kind, sequence))
        self.assertLessEqual(deadline - self.now, .400001)
        if kind == 21:
            sid = options['payload'][0]
            self.assertIn(sid, (22, 24))
            return cached_packet(sid, 65535 if sid == 22 else 2301, self.tick, sid == 22)
        self.assertEqual(kind, 2)
        self.assertEqual(options, {})
        return self.state()

    def sleep(self, seconds):
        self.now += seconds

    def execute(self, hupcl=False, close_error=None):
        with (mock.patch.object(probe.os, 'open', return_value=91) as opened,
              mock.patch.object(probe.os, 'close', side_effect=close_error) as closed,
              mock.patch('fcntl.flock') as flock, mock.patch('fcntl.ioctl') as ioctl,
              mock.patch('termios.tcgetattr', return_value=[0, 0, termios.HUPCL if hupcl else 0, 0, 0, 0, []])):
            result = probe.run_probe(self.ns, {}, self.guard,
                                     clock=lambda: self.now, sleeper=self.sleep)
        return result, opened, closed, flock, ioctl

    def test_partial_online_off_collects_rejected_raw_and_closes_once(self):
        result, opened, closed, flock, ioctl = self.execute()
        self.assertTrue(result['success'])
        self.assertTrue(result['serial_closed'])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['new_servo_read_requested'])
        self.assertFalse(result['correlated_with_walking_failure'])
        self.assertEqual(self.calls, [(1, 1), (2, 2), (2, 3), (21, 4), (21, 5), (2, 6)])
        self.assertEqual(result['diagnostic_attempts'], 2)
        diagnostic = result['diagnostics'][0]['decoded']
        self.assertTrue(diagnostic['register_rejected'])
        self.assertEqual(diagnostic['position_raw'], 65535)
        self.assertEqual(diagnostic['cached_data_hex'], 'ffff01807b00')
        self.assertFalse(diagnostic['position_inside_12bit_register'])
        opened.assert_called_once()
        closed.assert_called_once_with(91)
        flock.assert_called_once()
        ioctl.assert_called_once_with(91, termios.TIOCEXCL)

    def test_invalid_readiness_never_queries_cache_and_always_closes(self):
        for key, value in (('armed', True), ('arm_cookie', 1), ('torque_mask', 1),
                           ('fault_mask', 1), ('last_error', 4), ('imu_ok', False)):
            with self.subTest(key=key):
                self.calls.clear()
                self.ns['_decode_state'] = lambda data: dict(data, **{key: value})
                result, _, closed, *_ = self.execute()
                self.assertFalse(result['success'])
                self.assertEqual(result['diagnostic_attempts'], 0)
                self.assertNotIn(21, [kind for kind, _ in self.calls])
                closed.assert_called_once()

    def test_unadvanced_or_reset_state_fails_without_cache(self):
        self.ns['_decode_state'] = lambda data: dict(data, sample_us=5, imu_us=5)
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertLessEqual(result['elapsed_seconds'], 2.000001)
        self.assertEqual(result['diagnostic_attempts'], 0)
        closed.assert_called_once()
        self.calls.clear()
        self.ns['_decode_state'] = lambda data: dict(data, sample_us=2000000 - data['sample_us'])
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertIn('reversed/reset', result['error'])

    def test_hupcl_or_modem_change_prevents_protocol_requests(self):
        result, _, closed, *_ = self.execute(hupcl=True)
        self.assertFalse(result['success'])
        self.assertEqual(self.calls, [])
        closed.assert_called_once()
        self.ns['_modem_bits'] = mock.Mock(side_effect=[dict(dtr=True), dict(dtr=False)])
        result, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(self.calls, [])

    def test_owner_or_guard_failure_never_opens_uart(self):
        self.ns['_holders'] = lambda *args: [dict(pid=123, fd='4')]
        result, opened, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        opened.assert_not_called()
        closed.assert_not_called()
        self.ns['_holders'] = lambda *args: []
        self.guard.side_effect = ValueError('generation changed')
        result, opened, *_ = self.execute()
        opened.assert_not_called()
        self.assertIn('generation changed', result['error'])

    def test_cache_request_timeout_is_not_retried(self):
        original = self.ns['_exchange']
        def fail(fd, kind, sequence, *args, **kwargs):
            if kind == 21:
                self.calls.append((kind, sequence))
                raise TimeoutError('original timeout')
            return original(fd, kind, sequence, *args, **kwargs)
        self.ns['_exchange'] = fail
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['diagnostic_attempts'], 1)
        self.assertEqual([c for c in self.calls if c[0] == 21], [(21, 4)])
        closed.assert_called_once()

    def test_bad_cache_layout_stops_after_preserving_raw(self):
        original = self.ns['_exchange']
        def bad(fd, kind, *args, **kwargs):
            value = original(fd, kind, *args, **kwargs)
            return bytes([2]) + value[1:] if kind == 21 else value
        self.ns['_exchange'] = bad
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['diagnostic_attempts'], 1)
        self.assertTrue(result['diagnostics'][0]['raw_payload_hex'].startswith('02'))
        closed.assert_called_once()

    def test_close_failure_cannot_be_success(self):
        result, _, closed, *_ = self.execute(close_error=OSError('close failed'))
        self.assertFalse(result['serial_closed'])
        self.assertFalse(result['success'])
        self.assertEqual(result['close_error'], 'close failed')
        closed.assert_called_once()

    def test_closure_uses_independent_off_keeps_release_unconfirmed(self):
        value = json.loads((D / 'post-failure-joint-check/final-closed.json').read_text())
        self.assertFalse(value['motion']['closed_proof']['release_confirmed'])
        generations = probe.validate_closed_snapshot(value)
        probe.validate_closed_snapshot(value, generations)
        self.assertFalse(value['motion']['closed_proof']['release_confirmed'])
        mutations = (
            lambda d: d['motion'].update(owns_connection=True),
            lambda d: d['motion']['closed_proof'].update(serial_closed=False),
            lambda d: d['session']['closed_proof']['query']['last_feedback'].update(torque_mask=1),
            lambda d: d['session']['closed_proof']['query']['last_feedback'].update(armed=True),
            lambda d: d['session']['closed_proof']['query'].update(request_types_attempted=[1, 2, 4]),
            lambda d: d['session']['closed_proof'].update(serial_closed=False),
            lambda d: d['session'].update(recovery_pending=True),
            lambda d: d['maintenance'].update(owns_connection=True),
            lambda d: d['joint_zero'].update(busy=True),
        )
        for change in mutations:
            broken = copy.deepcopy(value)
            change(broken)
            with self.assertRaises(ValueError):
                probe.validate_closed_snapshot(broken)
        broken = copy.deepcopy(value)
        broken['session']['generation'] += 1
        with self.assertRaisesRegex(ValueError, 'generation'):
            probe.validate_closed_snapshot(broken, generations)

    def test_generated_current_v5_source_compiles_and_frame_allowlist_is_narrow(self):
        import tarfile
        import io
        source = probe.build_remote_source(V5 / 'installation.tar.gz',
            D / 'post-failure-joint-check/final-closed.json', boot_id=BOOT, cid=CID,
            persistent_calibration_sha256=CAL, expected_seal=SEAL)
        compile(source, '<test-cached-source>', 'exec')
        assign = next(n for n in ast.parse(source).body if isinstance(n, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'PINS' for t in n.targets))
        pins = ast.literal_eval(assign.value)
        self.assertFalse(pins['original_motion_release_confirmed'])
        self.assertEqual(pins['installation_seal'], SEAL)
        self.assertEqual(pins['independent_OFF_observation']['online_mask'], 32639)
        self.assertEqual(pins['independent_OFF_observation']['torque_mask'], 0)
        self.assertIn("validate_closed_snapshot(current,PINS['closure_generations'])", source)
        with tarfile.open(V5 / 'installation.tar.gz') as archive:
            with tarfile.open(fileobj=io.BytesIO(archive.extractfile('native-bundle.tar.gz').read())) as bundle:
                guard = bundle.extractfile('service/guard-runtime.py').read().decode()
        namespace = {}
        exec(guard, namespace)
        probe.adapt_helpers(guard, namespace)
        for sid in (22, 24):
            raw = namespace['_frame'](21, 3, bytes([sid]))
            self.assertEqual(raw[10:-2], bytes([sid]))
        for kind, payload in ((3, b''), (4, b''), (5, b''), (6, b''), (7, b''),
                              (21, b''), (21, b'\x17'), (22, b'\x16'), (1, b'\x16')):
            with self.subTest(kind=kind, payload=payload), self.assertRaises(namespace['ProbeError']):
                namespace['_frame'](kind, 3, payload)
        for change in ('none', 'sequence', 'type', 'crc'):
            body = cached_packet(22, 65535, 1000000, True)
            header = namespace['HEADER'].pack(b'XD', 1, 0x82 if change == 'type' else 0x95,
                22, 2 if change == 'sequence' else 3)
            tail = body + struct.pack('<H', namespace['_crc16'](header + body) ^ (change == 'crc'))
            namespace['_read_exact'] = mock.Mock(side_effect=[header, tail])
            namespace['_write_once'] = mock.Mock()
            with mock.patch.object(namespace['select'], 'select', return_value=([], [], [])):
                call = lambda: namespace['_exchange'](91, 21, 3, probe.time.monotonic() + .4,
                    dict(request_types_attempted=[], reply_types_received=[], request_bytes_written=0), payload=b'\x16')
                if change == 'none':
                    self.assertEqual(call(), body)
                else:
                    with self.assertRaises(namespace['ProbeError']):
                        call()

    def test_generated_pin_rejects_wrong_archive_seal(self):
        with self.assertRaisesRegex(ValueError, 'v5 seal'):
            probe.build_remote_source(V5 / 'installation.tar.gz',
                D / 'post-failure-joint-check/final-closed.json', boot_id=BOOT, cid=CID,
                persistent_calibration_sha256=CAL, expected_seal='0' * 64)

    def test_snapshot_prefers_full_and_only_retries_publication_race(self):
        packet = dict(snapshot=dict(selected='compact'), full_snapshot=dict(selected='full'))
        with mock.patch.object(probe, 'read_private', create=True,
                side_effect=[ValueError('Reviewed file changed during read: web-state.json'),
                             json.dumps(packet).encode()]) as reader, mock.patch.object(probe.time, 'sleep') as slept:
            self.assertEqual(probe.read_current_snapshot(Path('/fake')), dict(selected='full'))
            self.assertEqual(reader.call_count, 2)
            slept.assert_called_once_with(.01)
        with mock.patch.object(probe, 'read_private', create=True,
                side_effect=ValueError('Private reviewed file metadata differs')) as reader:
            with self.assertRaisesRegex(ValueError, 'metadata'):
                probe.read_current_snapshot(Path('/fake'))
            reader.assert_called_once()

    def test_public_result_removes_cookies_tokens_and_session_generation(self):
        self.assertEqual(probe.public_result(dict(states=[dict(arm_cookie=0, armed=False)],
            session_generation=33, token='hidden', control_owner=dict(id='hidden'), success=True)),
            dict(states=[dict(armed=False)], success=True))


if __name__ == '__main__':
    unittest.main()
