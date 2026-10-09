"""CPU-only probe tests; no actual file descriptor, network or UART is opened."""
import copy
import json
from pathlib import Path
import struct
import tempfile
import termios
import types
import unittest
import sys
from unittest import mock

import servo_goal_probe as probe
from test_servo_goal_diagnostics import packet


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.calls = []
        self.tick = 1000000
        self.ns = dict(bind_serial=lambda expected: None,
                       serial_proof=lambda expected, fd=None: (types.SimpleNamespace(st_rdev=12), {}),
                       serial_path=lambda expected: '/dev/ttyUSB0', _holders=lambda *args: [],
                       _modem_bits=lambda fd: dict(dtr=False, rts=False),
                       _raw_settings=lambda fd: dict(baud=921600, hupcl=False),
                       _decode_state=lambda data: data,
                       _hello_feedback=lambda fd, metadata: self.calls.append((1, 1)),
                       _exchange=self.exchange)
        self.guard = mock.Mock(return_value=dict(boot_id='same'))

    def state(self):
        self.tick += 20000
        return dict(sample_us=self.tick, imu_us=self.tick, online_mask=0x7fff,
                    fault_mask=0, torque_mask=0, armed=False, arm_cookie=0,
                    last_error=0, imu_ok=True, accel_ms2=[0, 0, 9.8], gyro_dps=[0, 0, 0])

    def exchange(self, fd, kind, sequence, deadline, metadata, **options):
        self.calls.append((kind, sequence))
        self.assertLessEqual(deadline - self.now, .400001)
        if kind == 22:
            self.assertEqual(options, dict(payload=b'\x0b'))
            raw = bytearray(packet())
            struct.pack_into('<4I', raw, 18, self.tick, self.tick, self.tick, self.tick)
            return bytes(raw)
        self.assertEqual(kind, 2)
        self.assertEqual(options, {})
        return self.state()

    def sleep(self, seconds):
        self.now += seconds

    def execute(self, *, hupcl=False, close_error=None):
        attributes = [0, 0, termios.HUPCL if hupcl else 0, 0, 0, 0, []]
        with (mock.patch.object(probe.os, 'open', return_value=91) as opened,
              mock.patch.object(probe.os, 'close', side_effect=close_error) as closed,
              mock.patch('fcntl.flock') as flock, mock.patch('fcntl.ioctl') as ioctl,
              mock.patch('termios.tcgetattr', return_value=attributes)):
            result = probe.run_probe(self.ns, {}, self.guard,
                                     clock=lambda: self.now, sleeper=self.sleep)
        return result, opened, closed, flock, ioctl

    def test_one_diagnostic_between_real_off_states_and_closes_without_motion(self):
        result, opened, closed, flock, ioctl = self.execute()
        self.assertTrue(result['success'])
        self.assertTrue(result['serial_closed'])
        self.assertFalse(result['correlated_with_previous_hold'])
        self.assertFalse(result['sensor_freshness_established'])
        self.assertFalse(result['motion_authorized'])
        self.assertEqual(self.calls, [(1, 1), (2, 2), (2, 3), (22, 4), (2, 5)])
        opened.assert_called_once()
        closed.assert_called_once_with(91)
        flock.assert_called_once()
        ioctl.assert_called_once_with(91, termios.TIOCEXCL)
        self.assertEqual(result['diagnostic_attempts'], 1)
        self.assertEqual(self.guard.call_count, 5)

    def test_hupcl_and_modem_changes_refuse_before_any_protocol_write(self):
        self.ns['_raw_settings'] = mock.Mock()
        result, _, closed, _, _ = self.execute(hupcl=True)
        self.assertFalse(result['success'])
        self.assertIn('HUPCL', result['error'])
        self.ns['_raw_settings'].assert_not_called()
        self.assertEqual(self.calls, [])
        closed.assert_called_once_with(91)
        self.ns['_modem_bits'] = mock.Mock(side_effect=[dict(dtr=False), dict(dtr=True)])
        result, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(self.calls, [])

    def test_existing_owner_and_health_failure_prevent_open(self):
        self.ns['_holders'] = lambda *args: [dict(pid=123, fd='4')]
        result, opened, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertTrue(result['serial_closed'])
        opened.assert_not_called()
        closed.assert_not_called()
        self.ns['_holders'] = lambda *args: []
        self.guard.side_effect = ValueError('kernel changed')
        result, opened, *_ = self.execute()
        opened.assert_not_called()
        self.assertIn('kernel changed', result['error'])

    def test_timeout_and_unsafe_state_never_send_diagnostic(self):
        self.ns['_decode_state'] = lambda data: dict(data, online_mask=0)
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertLessEqual(result['elapsed_seconds'], 2.000001)
        self.assertEqual(result['diagnostic_attempts'], 0)
        self.assertNotIn(22, [kind for kind, _ in self.calls])
        closed.assert_called_once()
        self.calls.clear()
        self.ns['_decode_state'] = lambda data: dict(data, torque_mask=1)
        result, *_ = self.execute()
        self.assertIn('healthy OFF', result['error'])
        self.assertEqual(result['diagnostic_attempts'], 0)

    def test_failed_diagnostic_is_not_retried_and_closes(self):
        original = self.ns['_exchange']
        def fail(fd, kind, *args, **kwargs):
            if kind == 22:
                self.calls.append((22, args[0]))
                raise TimeoutError('original diagnostic timeout')
            return original(fd, kind, *args, **kwargs)
        self.ns['_exchange'] = fail
        result, _, closed, *_ = self.execute()
        self.assertEqual(result['error_type'], 'TimeoutError')
        self.assertEqual(result['error'], 'original diagnostic timeout')
        self.assertEqual(sum(kind == 22 for kind, _ in self.calls), 1)
        closed.assert_called_once()

    def test_bad_raw_diagnostic_and_post_state_failure_preserve_evidence(self):
        original = self.ns['_exchange']
        def bad(fd, kind, *args, **kwargs):
            value = original(fd, kind, *args, **kwargs)
            if kind == 22:
                raw = bytearray(value)
                raw[0] = 2
                return bytes(raw)
            return value
        self.ns['_exchange'] = bad
        result, _, closed, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['rejected_raw_payload_hex'], result['diagnostic_raw_payload_hex'])
        self.assertEqual(result['diagnostic_attempts'], 1)
        closed.assert_called_once()
        self.calls.clear()
        self.ns['_exchange'] = original
        self.ns['_decode_state'] = lambda data: dict(data, fault_mask=1 if len(self.calls) == 5 else 0)
        result, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertIn('diagnostic', result)

    def test_close_failure_cannot_be_success(self):
        result, _, closed, *_ = self.execute(close_error=OSError('close failed'))
        self.assertFalse(result['serial_closed'])
        self.assertFalse(result['success'])
        self.assertEqual(result['close_error'], 'close failed')
        closed.assert_called_once()

    def test_reviewed_exchange_adds_only_id11_request_and_validates_sequence_crc(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'x5_recovery'))
        import feedback_stream
        namespace = {}
        source = feedback_stream.runtime_source()
        exec(source, namespace)
        probe.adapt_helpers(source, namespace)
        for kind, data in ((3, b''), (22, b'\x0a'), (22, b''), (1, b'\x0b')):
            with self.subTest(kind=kind, data=data), self.assertRaises(namespace['ProbeError']):
                namespace['_frame'](kind, 5, data)
        packet_data = namespace['_frame'](22, 5, b'\x0b')
        self.assertEqual(namespace['HEADER'].unpack(packet_data[:10]), (b'XD', 1, 22, 1, 5))
        raw = packet()
        for change in ('none', 'seq', 'type', 'crc'):
            header = namespace['HEADER'].pack(b'XD', 1, 0x82 if change == 'type' else 0x96, 78, 4 if change == 'seq' else 5)
            tail = raw + struct.pack('<H', namespace['_crc16'](header + raw) ^ (1 if change == 'crc' else 0))
            namespace['_read_exact'] = mock.Mock(side_effect=[header, tail])
            namespace['_write_once'] = mock.Mock()
            with mock.patch.object(namespace['select'], 'select', return_value=([], [], [])):
                call = lambda: namespace['_exchange'](91, 22, 5, probe.time.monotonic() + .4,
                    dict(request_types_attempted=[], reply_types_received=[], request_bytes_written=0), payload=b'\x0b')
                if change == 'none':
                    self.assertEqual(call(), raw)
                else:
                    with self.assertRaises(namespace['ProbeError']):call()

    def test_current_real_archive_and_closure_generate_compilable_pinned_source(self):
        import ast
        import hashlib
        import io
        import tarfile
        root = Path(__file__).resolve().parents[2]
        archive_path = root / 'output/x5_recovery/replacement-board-20261004T011959+0800/native-service-install-4s_mwp7d/installation.tar.gz'
        closure = root / 'tmp/standing-audit-20261008/off-goal-probe-initial/after-update-disconnect.json'
        source = probe.build_remote_source(archive_path, closure, 'b4610e59482bd35adeccbcd5c574177f71d7cf007ec72abd8d82b0c5baca6321',
            boot_id='91ac84d9-a758-454d-9567-ef53dac9b29e', cid='035344534b33324785d55cb687017c00')
        compile(source, '<test-generated-probe>', 'exec')
        pins_node = next(n for n in ast.parse(source).body if isinstance(n, ast.Assign)
                         and any(isinstance(t, ast.Name) and t.id == 'PINS' for t in n.targets))
        pins = ast.literal_eval(pins_node.value)
        self.assertEqual(pins['installation_seal'], 'bfcee8584c091bd555b2546a1b8ff44b8f6859a17fa369252e32621d20664cf8')
        with tarfile.open(archive_path) as archive:
            with tarfile.open(fileobj=io.BytesIO(archive.extractfile('native-bundle.tar.gz').read())) as bundle:
                guard = bundle.extractfile('service/guard-runtime.py').read() + bytes([10]) + archive.extractfile('persistent_boot.py').read()
        self.assertEqual(pins['guard_sha'], hashlib.sha256(guard).hexdigest())
        namespace = {}
        exec(guard, namespace)
        probe.adapt_helpers(guard.decode(), namespace)
        packet_data = namespace['_frame'](22, 3, bytes([11]))
        self.assertEqual(namespace['HEADER'].unpack(packet_data[:10]), (b'XD', 1, 22, 1, 3))
        self.assertEqual(packet_data[10:-2], bytes([11]))
        self.assertEqual(struct.unpack('<H', packet_data[-2:])[0], namespace['_crc16'](packet_data[:-2]))

    def test_selected_head_id_has_one_fixed_read_allowlist_and_no_motion(self):
        import feedback_stream
        namespace={};source=feedback_stream.runtime_source();exec(source,namespace)
        probe.adapt_helpers(source,namespace,probe_id=33)
        self.assertEqual(namespace['_frame'](22,5,b'!')[10:-2],b'!')
        for kind,payload in ((22,b'\x0b'),(22,b''),(3,b'!'),(4,b'!'),(5,b'!')):
            with self.subTest(kind=kind,payload=payload),self.assertRaises(namespace['ProbeError']):
                namespace['_frame'](kind,5,payload)
        for value in (True,0,16,255):
            with self.subTest(value=value),self.assertRaises(ValueError):
                probe.adapt_helpers(source,{},probe_id=value)

    def test_head_probe_decodes_matching_mapped_id_once(self):
        original=self.ns['_exchange']
        def head_exchange(fd,kind,sequence,deadline,metadata,**options):
            if kind!=22:return original(fd,kind,sequence,deadline,metadata,**options)
            self.calls.append((kind,sequence))
            self.assertEqual(options,dict(payload=b'!'))
            raw=bytearray(packet());raw[1]=33;raw[2]=13
            struct.pack_into('<4I',raw,18,self.tick,self.tick,self.tick,self.tick)
            return bytes(raw)
        self.ns['_exchange']=head_exchange
        with (mock.patch.object(probe.os,'open',return_value=91),
              mock.patch.object(probe.os,'close')as closed,
              mock.patch('fcntl.flock'),mock.patch('fcntl.ioctl'),
              mock.patch('termios.tcgetattr',return_value=[0,0,0,0,0,0,[]])):
            result=probe.run_probe(self.ns,{},self.guard,probe_id=33,
                clock=lambda:self.now,sleeper=self.sleep)
        self.assertTrue(result['success'])
        self.assertEqual(result['diagnostic']['id'],33)
        self.assertEqual(sum(kind==22 for kind,_ in self.calls),1)
        self.assertFalse(any(kind in (3,4,5,6) for kind,_ in self.calls))
        closed.assert_called_once_with(91)


if __name__ == '__main__':
    unittest.main()
