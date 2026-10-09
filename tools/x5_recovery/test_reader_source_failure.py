"""Early reader transport diagnostics, using fake SSH children and local pipes only."""
import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

import feedback_calibration_server as host
from test_feedback_calibration_server import Clock, config


class PipeChild:
    """Popen interface whose output is entirely supplied by this test."""
    def __init__(self, stderr=b'', stdout=b'', exit_code=255):
        self.returncode = exit_code
        self.terminate_count = 0
        self.stdin_read, stdin_write = os.pipe()
        self.stdin = os.fdopen(stdin_write, 'wb', buffering=0)
        self.writers = []
        for name, data in (('stdout', stdout), ('stderr', stderr)):
            read_fd, write_fd = os.pipe()
            setattr(self, name, os.fdopen(read_fd, 'rb', buffering=0))
            def write_output(fd=write_fd, payload=data):
                try:
                    offset = 0
                    while offset < len(payload):
                        offset += os.write(fd, payload[offset:offset + 4096])
                except BrokenPipeError:
                    pass
                finally:
                    os.close(fd)
            worker = threading.Thread(target=write_output, daemon=True)
            worker.start()
            self.writers.append(worker)

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminate_count += 1
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        return self.returncode

    def close(self):
        for stream in (self.stdin, self.stdout, self.stderr):
            if not stream.closed:
                stream.close()
        os.close(self.stdin_read)
        for worker in self.writers:
            worker.join(timeout=2)


class ReaderSourceFailureTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.helper_directory = self.directory / 'helper'
        self.helper_directory.mkdir()
        (self.helper_directory / 'run_postflash_passive.py').write_text(
            "EXPECTED_FINGERPRINT = 'test-pinned'\n"
            "def known_hosts_fingerprint(value): return 'test-pinned'\n"
            "def parse_route(value): return 'en5'\n")
        self.known_hosts = self.directory / 'known_hosts'
        self.known_hosts.write_text('test pinned key\n')
        self.clock = Clock()
        self.session = host.CalibrationSession(config(), self.directory, 7, self.clock)
        self.original = copy.deepcopy(self.session.config)
        self.trace = self.directory / 'attempt'
        self.trace.mkdir()
        self.children = []

    def tearDown(self):
        for child in self.children:
            child.close()
        self.temporary.cleanup()

    def launch_failure(self, *, stderr=b'', stdout=b'', fully_sent=False,
                       exit_code=255, writer_effect=None):
        child = PipeChild(stderr, stdout, exit_code)
        self.children.append(child)
        route = mock.Mock(returncode=0, stdout='test route')
        effect = writer_effect or BrokenPipeError(32, 'Broken pipe')
        with mock.patch.object(host, 'RECOVERY', self.helper_directory), \
                mock.patch.object(host.subprocess, 'run', return_value=route), \
                mock.patch.object(host.subprocess, 'Popen', return_value=child), \
                mock.patch.object(host, 'write_pipe_once',
                                  side_effect=None if fully_sent else effect) as writer:
            if fully_sent:
                with mock.patch.object(self.session, 'source_finished',
                                       side_effect=OSError('post-transfer failure')):
                    with self.assertRaises(ValueError) as caught:
                        host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
            else:
                with self.assertRaises(ValueError) as caught:
                    host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        return child, str(caught.exception), writer.call_count

    def record(self, name):
        return json.loads((self.trace / name).read_text())

    def test_initial_reader_uses_background_single_attempt_lifecycle(self):
        self.session.recovery_launcher = mock.Mock()
        with mock.patch.object(host.threading, 'Thread') as thread:
            self.session.start_initial()
        thread.return_value.start.assert_called_once()
        self.session.recovery_launcher.assert_not_called()
        self.assertEqual(self.session.status, 'waiting')
        self.assertNotEqual(self.session.generation, 7)
        self.assertFalse(self.session._paused_without_attempt)
        self.assertTrue(self.session.recovery_pending)
        self.assertIsNone(self.session.process)
        self.assertEqual(self.session.config, self.original)

    def test_route_failure_before_ssh_can_be_rechecked_without_remote_closure(self):
        self.session.recovery_launcher = mock.Mock()
        route = mock.Mock(returncode=1, stdout='')
        with mock.patch.object(host, 'RECOVERY', self.helper_directory), \
                mock.patch.object(host.subprocess, 'run', return_value=route), \
                mock.patch.object(host.subprocess, 'Popen') as popen:
            with self.assertRaisesRegex(ValueError, 'USB'):
                host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        popen.assert_not_called()
        self.assertEqual(self.session._reader_preflight_failure_generation, 7)
        self.assertTrue(self.session.connection_done.is_set())
        self.assertIsNone(self.session.process)
        self.assertIsNone(self.session.closed_proof)
        self.assertFalse(self.session._paused_without_attempt)
        self.assertIsNone(self.session.recheck_blocked_reason())
        self.assertEqual(self.session.config, self.original)
        proof = self.record('local-preflight-failure.json')
        self.assertFalse(proof['ssh_start_attempted'])
        self.assertEqual(proof['source_bytes_sent'], 0)
        self.assertFalse(proof['automatic_retry'])
        self.assertIn('USB 网络连接未就绪', self.session.error)
        self.session.recovery_launcher.assert_not_called()
        # Only a new explicit request may retry; its generation cannot
        # inherit the previous local-only exception.
        with mock.patch.object(host.threading, 'Thread') as thread:
            self.session.recheck_feedback_action(dict(generation=7, purpose='joints'))
        thread.return_value.start.assert_called_once()
        self.assertIsNone(self.session._reader_preflight_failure_generation)
        self.session.recovery_launcher.assert_not_called()

    def test_local_route_failure_preserves_existing_servo_hold(self):
        self.session.recovery_launcher = mock.Mock()
        self.session.start_initial(paused=True, hold_unsafe=True)
        self.session._paused_without_attempt = False
        self.session.status = 'waiting'
        held = copy.deepcopy(self.session.pending_servo_hold)
        with mock.patch.object(host, 'reader_launch_preflight',
                               side_effect=ValueError('Route is not the verified en5 USB flash link')):
            with self.assertRaises(ValueError):
                host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        self.assertEqual(self.session.pending_servo_hold, held)
        self.assertEqual(self.session.recovery_recheck['category'], 'servo')
        self.assertIsNone(self.session.recheck_blocked_reason())
        self.assertIsNotNone(self.session.recovery_blocked_reason())
        self.assertEqual(self.session.config, self.original)

    def test_preflight_host_identity_failure_keeps_trust_lock(self):
        self.session.recovery_launcher = mock.Mock()
        with mock.patch.object(host, 'reader_launch_preflight',
                               side_effect=ValueError('X5 的 SSH 身份已变化')):
            with self.assertRaises(ValueError):
                host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.assertIsNone(self.session.closed_proof)

    def test_popen_failure_does_not_grant_local_preflight_exception(self):
        self.session.recovery_launcher = mock.Mock()
        route = mock.Mock(returncode=0, stdout='test route')
        with mock.patch.object(host, 'RECOVERY', self.helper_directory), \
                mock.patch.object(host.subprocess, 'run', return_value=route), \
                mock.patch.object(host.subprocess, 'Popen', side_effect=OSError('spawn failed')):
            with self.assertRaisesRegex(ValueError, 'spawn failed'):
                host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        self.assertIsNone(self.session._reader_preflight_failure_generation)
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.assertFalse((self.trace / 'local-preflight-failure.json').exists())

    def test_obsolete_preflight_failure_cannot_change_new_generation(self):
        def failure(*args):
            self.session.generation = 8
            raise ValueError('X5 USB 闪联路由不可用')
        with mock.patch.object(host, 'reader_launch_preflight', side_effect=failure):
            with self.assertRaises(ValueError):
                host.run_reader(self.session, b'pass\n', self.known_hosts, self.trace)
        self.assertEqual(self.session.generation, 8)
        self.assertIsNone(self.session._reader_preflight_failure_generation)
        self.assertEqual(self.session.status, 'waiting')
        self.assertFalse((self.trace / 'local-preflight-failure.json').exists())

    def assert_finished_without_closure(self):
        finished = self.record('connection-finished.json')
        self.assertEqual(finished['generation'], 7)
        self.assertIsNone(finished['closed_proof'])
        self.assertEqual(self.session.recovery_recheck['category'], 'closure')
        self.assertTrue(self.session.connection_done.is_set())
        self.assertFalse(self.session.source_transmitting)
        self.assertEqual(self.session.config, self.original)
        self.assertFalse((self.trace / 'askpass.sh').exists())

    def test_auth_error_and_exit_are_retained_instead_of_only_broken_pipe(self):
        child, diagnostic, calls = self.launch_failure(
            stderr=b'root@192.168.128.10: Permission denied (password).\n')
        self.assertIn('Permission denied', diagnostic)
        self.assertIn('exit 255', diagnostic)
        failure = self.record('source-transmission-failure.json')
        self.assertFalse(failure['source_fully_sent'])
        self.assertEqual(failure['ssh_exit_code'], 255)
        self.assertIn('Broken pipe', failure['error'])
        self.assertIn('Permission denied', failure['stderr'])
        self.assertTrue(failure['closure_not_inferred'])
        self.assertEqual(bytes(self.session.stderr).decode(), failure['stderr'])
        self.assertEqual(calls, 1)
        self.assertEqual(child.terminate_count, 0)
        self.assert_finished_without_closure()

    def test_host_down_real_stderr_is_not_hidden_by_epipe(self):
        _, diagnostic, _ = self.launch_failure(
            stderr=b'ssh: connect to host 192.168.128.10 port 22: Host is down\n')
        self.assertIn('Host is down', diagnostic)
        self.assertIn('Host is down', self.record('connection-finished.json')['stderr'])
        self.assert_finished_without_closure()

    def test_host_key_failure_is_a_trust_hold_without_fabricated_proof(self):
        _, diagnostic, _ = self.launch_failure(
            stderr=b'WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!\nHost key verification failed.\n')
        self.assertTrue(diagnostic.startswith('Pinned SSH host identity changed:'))
        self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assertIsNone(self.session.closed_proof)
        self.assertEqual(self.record('connection-finished.json')['recovery_recheck']['category'], 'trust')
        self.assertTrue(self.session.connection_done.is_set())
        self.assertFalse(self.session.source_transmitting)

    def test_unfinished_source_cannot_grant_uart_closure_from_stdout(self):
        record = dict(kind='closed', generation=7, normal=True, serial_closed=True)
        self.launch_failure(stdout=(json.dumps(record) + '\n').encode())
        self.assertEqual(self.record('source-transmission-failure.json')['stdout'],
                         json.dumps(record) + '\n')
        self.assertFalse(list(self.trace.glob('closed-*.json')))
        self.assert_finished_without_closure()

    def test_fully_sent_guard_fault_and_actual_close_are_preserved(self):
        reason = 'Reviewed boot/root/card/model identity changed'
        records = [dict(kind='fault', generation=7, error=reason),
                   dict(kind='closed', generation=7, normal=False, serial_closed=True,
                        reason=reason)]
        self.launch_failure(fully_sent=True,
            stdout=b''.join((json.dumps(record) + '\n').encode() for record in records))
        failure = self.record('source-transmission-failure.json')
        self.assertTrue(failure['source_fully_sent'])
        self.assertEqual(failure['closed_proof'], records[-1])
        self.assertEqual(self.session.error_details, reason)
        self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assertEqual(self.session.recovery_recheck['technical_reason'], reason)
        self.assertEqual(len(list(self.trace.glob('fault-*.json'))), 2)  # raw + evidence
        self.assertEqual(len(list(self.trace.glob('closed-*.json'))), 1)
        self.assertTrue(self.session.connection_done.is_set())

    def test_wrong_generation_closed_keeps_raw_without_attaching_proof(self):
        record = dict(kind='closed', generation=6, normal=True, serial_closed=True)
        self.launch_failure(fully_sent=True, stdout=(json.dumps(record) + '\n').encode())
        self.assertIsNone(self.session.closed_proof)
        self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assertEqual(self.session.recovery_recheck['technical_reason'], '反馈会话标识不匹配')
        self.assertEqual(json.loads(next(self.trace.glob('closed-*.json')).read_text()), record)

    def test_output_is_bounded_and_only_this_live_child_is_stopped(self):
        child, _, _ = self.launch_failure(stderr=b'x' * 10000 + b'Host is down\n',
                                         stdout=b'y' * 300000, exit_code=None)
        failure = self.record('source-transmission-failure.json')
        self.assertEqual(len(failure['stderr'].encode()), 4096)
        self.assertEqual(len(failure['stdout'].encode()), 262144)
        self.assertTrue(failure['stdout_truncated'])
        self.assertEqual(child.terminate_count, 1)
        self.assertEqual(failure['ssh_exit_code'], -15)
        self.assert_finished_without_closure()

    def test_failed_output_drain_still_stops_this_live_child(self):
        with mock.patch.object(host.select, 'select', side_effect=OSError('test drain failed')):
            child, _, calls = self.launch_failure(exit_code=None)
        failure = self.record('source-transmission-failure.json')
        self.assertEqual(child.terminate_count, 1)
        self.assertEqual(failure['ssh_exit_code'], -15)
        self.assertIn('first_drain: test drain failed', failure['cleanup_error'])
        self.assertIn('final_drain: test drain failed', failure['cleanup_error'])
        self.assertEqual(calls, 1)
        self.assert_finished_without_closure()

    def test_late_old_attempt_cannot_fault_new_generation(self):
        def change_owner(*args):
            self.session.generation = 8
            self.session.process = None
            raise BrokenPipeError(32, 'Broken pipe')
        self.launch_failure(stderr=b'Host is down\n', writer_effect=change_owner)
        failure = self.record('source-transmission-failure.json')
        self.assertEqual(failure['generation'], 7)
        self.assertIsNone(failure['closed_proof'])
        self.assertEqual(self.session.generation, 8)
        self.assertEqual(self.session.status, 'waiting')
        self.assertIsNone(self.session.error)
        self.assertFalse((self.trace / 'connection-finished.json').exists())


if __name__ == '__main__':
    unittest.main()
