"""Pure single-launch tests: processes/routes/pipe I/O are mocked."""
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import run_feedback_fault_details as launch
import feedback_stream
from test_feedback_fault_probe import payload


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.profile_path = feedback_stream.EVIDENCE_DIR / 'feedback-reboot-reviewed-20261004-01.json'
        self.profile_sha = feedback_stream.REBOOT_PROFILE_SHA256
        self.expected_path = feedback_stream.EVIDENCE_DIR / 'feedback-calibration-20261004-joints-held-01/expectations.json'
        self.expected, self.source = launch.prepare_source(self.expected_path, self.profile_path, self.profile_sha, 61273)
        self.collector = launch.Collector(61273, self.expected)
        self.ns = self.collector.namespace

    def query(self, *, closed=False, ready=False):
        return dict(servo_motion_authorized=False, board_files_written_by_reader=False,
            state_records=0, serial_open_attempts=1, serial_closed=closed,
            request_types_attempted=[1] if ready else [1, 7],
            reply_types_received=[0x81] if ready else [0x81, 0x87])

    def ready(self):
        identity = {k: copy.deepcopy(self.expected[k]) for k in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial')}
        identity['kernel_log'] = self.expected['kernel_prefix']
        return dict(kind='ready', generation=61273, identity=identity, query=self.query(ready=True))

    def diagnostic(self, **changes):
        value = self.ns['_decode_fault_details'](payload(self.ns, **changes))
        return dict(kind='diagnostic', generation=61273, sequence=2, diagnostic=value, query=self.query())

    def closed(self, *, normal=True):
        return dict(kind='closed', generation=61273, normal=normal, serial_closed=True,
                    query=self.query(closed=True))

    def test_source_preparation_helpers_and_pins_are_pure(self):
        compile(self.source, '<prepared-diagnostic>', 'exec')
        self.assertIn(b'raise SystemExit(run_fault_probe(EXPECTED))', self.source)
        self.assertEqual(self.expected['reviewed_reconnect_proof']['profile_sha256'], self.profile_sha)
        helpers = launch.helpers()
        self.assertIn('reader_bootstrap', helpers)
        for sha in ('0' * 64, None):
            with self.assertRaises(ValueError): launch.prepare_source(self.expected_path, self.profile_path, sha, 61273)
        for generation in (0, True, 0x80000000):
            with self.assertRaises(ValueError): launch.prepare_source(self.expected_path, self.profile_path, self.profile_sha, generation)

    def test_diagnostic_roundtrip_all_flags_raw_errors_no_motion_permission(self):
        for changes in ({'armed': 1}, {'cookie': 1}, {'torque': 32767},
                        {'fault': 32767, 'torque_faults': 32767}):
            value = self.diagnostic(**changes)['diagnostic']
            self.assertEqual(launch.validated_diagnostic(value, self.ns), value)
            self.assertFalse(value['motion_authorized'])
        value = self.diagnostic()['diagnostic']
        for change in ({'motion_authorized': True}, {'unsafe_flags_present': 0},
                       {'snapshot_us': True}, {'unknown': 3}, {'motor_ids': list(range(15))}):
            with self.assertRaises((ValueError, self.ns['ProbeError'])):
                launch.validated_diagnostic({**value, **change}, self.ns)

    def test_collector_preserves_unsafe_diagnostic_and_real_closure(self):
        self.collector.accept(self.ready())
        value = self.diagnostic(armed=1, cookie=12, torque=32767)
        self.collector.accept(value)
        value['diagnostic']['torque_mask'] = 0
        self.assertEqual(self.collector.diagnostic['diagnostic']['torque_mask'], 32767)
        self.assertIsNone(self.collector.closed)
        self.collector.accept(self.closed())
        self.assertTrue(self.collector.closed['serial_closed'])
        with self.assertRaises(ValueError): self.collector.accept(self.closed())

    def test_state_unknown_generation_bad_scope_and_out_of_order_rejected(self):
        for message in (dict(kind='state', generation=61273), {**self.ready(), 'generation': True},
                        {**self.ready(), 'generation': 61274}, self.diagnostic()):
            with self.assertRaises(ValueError): self.collector.accept(message)
        for change in ({'servo_motion_authorized': True}, {'board_files_written_by_reader': True},
                       {'serial_open_attempts': 2}, {'request_types_attempted': [1, 5]}, {'state_records': False}):
            message = self.ready()
            message['query'].update(change)
            with self.assertRaises(ValueError): self.collector.accept(message)

    def test_ready_changed_identity_and_new_kernel_fault_rejected(self):
        for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
            message = self.ready()
            message['identity'][key] = 'wrong'
            with self.assertRaises(ValueError): self.collector.accept(message)
        message = self.ready()
        message['identity']['kernel_log'] += '[ 10000.0] mmc1: error -5\n'
        with self.assertRaises(self.ns['ProbeError']): self.collector.accept(message)

    def test_explicit_discovery_accepts_bound_ch340_receipt_and_rejects_other_adapters(self):
        expected = {**self.expected, 'discover_serial': True}
        message = self.ready()
        actual = {**expected['serial'], 'path': '/dev/ttyUSB2', 'sysfs_name': '1-1.4'}
        message['identity']['serial'] = actual
        collector = launch.Collector(61273, expected)
        collector.accept(message)
        self.assertEqual(collector.ready['identity']['serial'], actual)
        with self.assertRaisesRegex(ValueError, 'serial'):
            self.collector.accept(message)
        for change in ({'vid': 'ffff'}, {'pid': 'ffff'}, {'serial_number': 'other'},
                       {'path': '/dev/ttyS0'}, {'path': '/dev/ttyUSB256'}, {'extra': 'untrusted'}):
            altered = copy.deepcopy(message)
            altered['identity']['serial'].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'serial'):
                launch.Collector(61273, expected).accept(altered)

    def test_fault_before_ready_and_absent_close_are_not_success_proofs(self):
        query = self.query()
        query.update(serial_open_attempts=0, request_types_attempted=[], reply_types_received=[])
        message = dict(kind='fault', generation=61273, reason='New kernel fault', query=query)
        self.collector.accept(message)
        self.assertIsNone(self.collector.closed)
        self.assertIsNone(self.collector.diagnostic)
        query['serial_closed'] = True
        self.collector.accept(dict(kind='closed', generation=61273, normal=False, serial_closed=True, query=query))
        self.assertFalse(self.collector.closed['normal'])

    def test_missing_or_wrong_source_digest_refuses_before_any_route_or_process(self):
        for digest in (None, '0' * 64):
            with mock.patch.object(launch.subprocess, 'run', side_effect=AssertionError('route called')), \
                 mock.patch.object(launch.subprocess, 'Popen', side_effect=AssertionError('SSH called')):
                with self.assertRaises(ValueError):
                    launch.execute(self.source, self.expected, Path('/unused'), 'battery-off',
                                   Path('/unused/known_hosts'), {}, reviewed_source_sha=digest)

    def test_one_ssh_stdin_kept_open_through_real_exit_records_saved(self):
        messages = [self.ready(), self.diagnostic(fault=32767, torque_faults=32767), self.closed()]
        stdout = b''.join((json.dumps(value) + '\n').encode() for value in messages)
        reads = {92: [stdout, b''], 93: [b'']}
        events = []
        class Pipe(io.BytesIO):
            def __init__(self, fd): super().__init__(); self.fd = fd
            def fileno(self): return self.fd
            def close(self):
                events.append(('close', self.fd, child.poll()))
                super().close()
        class Child:
            def __init__(self):
                self.stdin, self.stdout, self.stderr = Pipe(91), Pipe(92), Pipe(93)
                self.code = None
            def poll(self): return self.code
            def wait(self, timeout=None):
                self.assert_stdin_open()
                self.code = 0
                return 0
            def assert_stdin_open(self):
                if self.stdin.closed: raise AssertionError('stdin EOF before remote exit')
            def terminate(self): raise AssertionError('unnecessary terminate')
            def kill(self): raise AssertionError('unnecessary kill')
        child = Child()
        helper = launch.helpers()
        def write(fd, data, seconds):
            self.assertEqual((fd, data), (91, self.source))
            events.append(('source', len(data)))
        helper['write_pipe_once'] = write
        with tempfile.TemporaryDirectory() as tmp, \
             mock.patch.object(launch.subprocess, 'run', return_value=types.SimpleNamespace(returncode=0, stdout='interface: en5\n')), \
             mock.patch.object(launch.subprocess, 'Popen', return_value=child) as popen, \
             mock.patch.object(launch.os, 'set_blocking'), \
             mock.patch.object(launch.select, 'select', side_effect=lambda active, *args: (active, [], [])), \
             mock.patch.object(launch.os, 'read', side_effect=lambda fd, size: reads[fd].pop(0)):
            directory = Path(tmp)
            report = launch.execute(self.source, self.expected, directory, 'battery-off',
                launch.BASE / 'known_hosts', helper, reviewed_source_sha=hashlib.sha256(self.source).hexdigest())
            self.assertEqual(popen.call_count, 1)
            command = popen.call_args.args[0]
            self.assertIn('ConnectionAttempts=1', command)
            self.assertIn('root@192.168.128.10', command)
            self.assertTrue(report['query_completed'])
            self.assertTrue(report['serial_closed'])
            self.assertEqual(report['ssh_exit_code'], 0)
            self.assertEqual(report['diagnostic_report']['diagnostic']['fault_mask'], 32767)
            self.assertFalse(report['servo_motion_authorized'])
            self.assertTrue((directory / 'record-03.json').exists())
            self.assertTrue((directory / 'report.json').exists())
            self.assertFalse((directory / 'askpass.sh').exists())
            # Local SSH success without the remote closure record must retain
            # unknown UART closure rather than inventing a successful proof.
            child = Child()
            popen.return_value = child
            popen.reset_mock()
            reads = {92: [b''.join((json.dumps(v) + '\n').encode() for v in messages[:2]), b''], 93: [b'']}
            second = directory / 'missing-close'
            second.mkdir(mode=0o700)
            incomplete = launch.execute(self.source, self.expected, second, 'battery-off',
                launch.BASE / 'known_hosts', helper, reviewed_source_sha=hashlib.sha256(self.source).hexdigest())
            self.assertEqual(popen.call_count, 1)
            self.assertEqual(incomplete['ssh_exit_code'], 0)
            self.assertIsNone(incomplete['serial_closed'])
            self.assertIsNone(incomplete['remote_closed_report'])
            self.assertFalse(incomplete['query_completed'])
        self.assertTrue(all(event[2] == 0 for event in events if event[0] == 'close'))


if __name__ == '__main__':
    unittest.main()
