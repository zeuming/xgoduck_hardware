"""Explicit metadata reconciliation: synthetic host records only, no hardware."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import feedback_calibration_server as host
from test_feedback_calibration_server import FakeProcess
import test_feedback_recheck as recheck_tests


class ConnectionReconciliationTests(unittest.TestCase):
    launch = recheck_tests.FeedbackRecheckTests.launch
    feed = recheck_tests.FeedbackRecheckTests.feed
    assert_preserved = recheck_tests.FeedbackRecheckTests.assert_preserved

    def setUp(self):
        recheck_tests.FeedbackRecheckTests.setUp(self)
        self.session.connection_checker = self.checker
        self.checks = []
        self.failure = None
        self.proof_changes = {}
        self.session._paused_without_attempt = False
        self.session.process = FakeProcess()
        self.session.process.returncode = 255
        self.session.connection_done.set()
        self.session.recovery_recheck = host.recovery_recheck_error(host.CLOSURE_REASONS[0])
        self.session.pending_servo_hold = None
        self.session.fault_record = {'generation': 7, 'error': 'first missing feedback'}
        self.original_fault = copy.deepcopy(self.session.fault_record)

    def checker(self, generation, directory):
        self.checks.append((generation, directory))
        if self.failure:
            raise ValueError(self.failure)
        proof = dict(kind='quiescent', generation=generation, serial_path='/dev/ttyUSB0',
                     serial_holders=[], identity_checked=True, guards_passed=True,
                     observation_count=2, serial_open_attempts=0, serial_bytes_written=0,
                     board_files_written=False, automatic_retry=False,
                     ssh_exit_code=0, ssh_process_exited=True)
        proof.update(self.proof_changes)
        return proof

    def check(self, purpose='joints'):
        return self.session.check_connection_action(dict(generation=self.session.generation, purpose=purpose))

    def test_status_does_not_probe_and_original_rechecks_stay_blocked(self):
        for _ in range(3):
            snap = self.session.snapshot()['session']
            self.assertTrue(snap['connection_check']['allowed'])
            self.assertFalse(snap['recheck_allowed'])
            self.assertEqual(snap['recovery_recheck']['category'], 'closure')
        for request in (self.session.recheck_feedback_action, self.session.recover_action):
            body = {'generation': 7}
            if request == self.session.recheck_feedback_action:
                body['purpose'] = 'joints'
            with self.assertRaises(ValueError):
                request(body)
        self.assertEqual(self.checks, [])
        self.assertEqual(self.launches, [])

    def test_one_click_probes_then_existing_guarded_window_required(self):
        result = self.check()
        self.assertEqual(result['session']['connection_check']['phase'], 'checking')
        self.assertEqual(result['session']['status'], 'fault')
        with self.assertRaises(ValueError):
            self.check()
        self.workers.pop(0)()
        self.assertEqual(len(self.checks), 1)
        self.assertEqual(self.launches, [])
        self.assertEqual(self.session.status, 'waiting')
        self.assertEqual(self.session.recovery_recheck['category'], 'closure')
        self.assertIsNone(self.session.closed_proof)
        request = json.loads((self.session.trace_directory / 'feedback-recheck-request.json').read_text())
        self.assertEqual(request['previous_fault'], self.original_fault)
        self.assertEqual(request['connection_reconciled']['generation'], 7)
        self.workers.pop(0)()
        self.feed(.01)
        self.assertEqual(self.session.status, 'waiting')
        self.feed(.21)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.recovery_recheck)
        passed = json.loads((self.session.trace_directory / 'feedback-recheck-passed.json').read_text())
        self.assertTrue(passed['connection_hold_cleared'])
        self.assertFalse(passed['servo_hold_cleared'])
        self.assert_preserved()

    def test_imu_window_can_resolve_only_the_missing_closure(self):
        self.check('imu')
        self.workers.pop(0)()
        self.workers.pop(0)()
        self.feed(.01, online=0)
        self.feed(.21, online=0)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.recovery_recheck)
        self.assertEqual(self.session.feedback_scope, 'imu')
        with self.assertRaises(ValueError):
            self.session._require_joint_feedback()
        self.assert_preserved()

    def test_unsafe_then_missing_close_then_imu_keeps_independent_servo_hold(self):
        self.session.recovery_recheck = None
        self.session.fault(host.UNSAFE_FEEDBACK_REASON)
        self.session.fault(host.CLOSURE_REASONS[0])
        self.assertEqual(self.session.recovery_recheck['category'], 'closure')
        self.assertEqual(self.session.pending_servo_hold['category'], 'servo')
        self.check('imu')
        self.workers.pop(0)()
        self.workers.pop(0)()
        self.feed(.01, online=0)
        self.feed(.21, online=0)
        self.assertEqual(self.session.status, 'active')
        self.assertEqual(self.session.recovery_recheck['category'], 'servo')
        self.assertEqual(self.session.pending_servo_hold['category'], 'servo')
        passed = json.loads((self.session.trace_directory / 'feedback-recheck-passed.json').read_text())
        self.assertTrue(passed['connection_hold_cleared'])
        self.assertFalse(passed['servo_hold_cleared'])
        self.assert_preserved()

    def test_transient_holder_failure_preserves_fault_and_can_retry_explicitly(self):
        self.failure = 'UART already has an open holder: [123]'
        self.check()
        self.workers.pop(0)()
        snap = self.session.snapshot()['session']
        self.assertEqual(snap['connection_check']['phase'], 'failed')
        self.assertIn('旧串口仍被占用', snap['connection_check']['message'])
        self.assertTrue(snap['connection_check']['allowed'])
        self.assertEqual(self.session.fault_record, self.original_fault)
        self.assertEqual(self.launches, [])
        self.assert_preserved()

    def test_ssh_transport_failure_remains_explicitly_retryable_with_clear_chinese_reason(self):
        self.failure = 'Connection-check SSH transport failed: 255'
        self.check()
        self.workers.pop(0)()
        value = self.session.snapshot()['session']
        self.assertTrue(value['connection_check']['allowed'])
        self.assertIn('检查 X5 供电和 USB 数据连接', value['connection_check']['error'])
        self.assertEqual(value['connection_check']['technical_reason'], self.failure)
        self.assertEqual(value['recovery_recheck']['category'], 'closure')
        self.assertEqual(self.session.fault_record, self.original_fault)
        self.assertEqual(self.launches, [])
        self.assert_preserved()

    def test_host_down_and_authentication_are_clear_without_hiding_identity_faults(self):
        for raw, expected in (
            ('Connection-check SSH transport failed: ssh: Host is down; exit 255', '192.168.128.10 没有响应'),
            ('Connection-check SSH transport failed: Permission denied (password); exit 255', 'SSH 登录没有通过'),
            ('Connection-check SSH transport failed: Connection refused; exit 255', 'SSH 服务拒绝连接'),
            ('[Errno 32] Broken pipe', '发送检查程序时提前中断')):
            with self.subTest(raw=raw):
                self.assertIn(expected, host.connection_check_error_text(raw))
        self.assertIn('身份或证据不匹配', host.connection_check_error_text(
            'Pinned SSH host identity changed: Host key verification failed'))

    def test_new_kernel_identity_or_proof_failure_keeps_acquisition_locked_but_metadata_retry_allowed(self):
        errors = ('New kernel fault; classifier result requires source review: card',
                  'Reviewed UART node changed during connection check',
                  'Connection-check identity differs: cid',
                  'Invalid completed connection-check proof schema')
        for error in errors:
            with self.subTest(error=error):
                self.session.recovery_recheck = host.recovery_recheck_error(host.CLOSURE_REASONS[0])
                self.failure = error
                self.check()
                self.workers.pop(0)()
                self.assertEqual(self.session.recovery_recheck['category'], 'trust')
                self.assertTrue(self.session.snapshot()['session']['connection_check']['allowed'])
                self.assertFalse(self.session.snapshot()['session']['recheck_allowed'])
                self.check()
                self.workers.pop(0)()
                self.assertEqual(self.session.recovery_recheck['category'], 'trust')
                self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.launches, [])

    def test_repeated_metadata_failure_never_changes_baseline_or_clears_servo_hold(self):
        self.session.pending_servo_hold = host.recovery_recheck_error(host.UNSAFE_FEEDBACK_REASON)
        before = copy.deepcopy(self.session.pending_servo_hold)
        self.failure = 'New kernel fault; classifier result requires source review: card'
        for _ in range(3):
            self.check()
            self.workers.pop(0)()
            self.assertEqual(self.session.status, 'fault')
            self.assertEqual(self.session.pending_servo_hold, before)
            self.assertIsNone(self.session.closed_proof)
        self.assertEqual(len(self.checks), 3)
        self.assertEqual(self.launches, [])
        self.assert_preserved()

    def test_trust_retry_needs_both_new_passed_metadata_and_fresh_powered_feedback(self):
        self.session.recovery_recheck = host.recovery_recheck_error('Reviewed UART node changed during connection check')
        original = copy.deepcopy(self.session.recovery_recheck)
        self.check()
        self.workers.pop(0)()
        self.assertEqual(self.session.recovery_recheck, original)
        self.workers.pop(0)()
        self.feed(.01)
        self.assertEqual(self.session.status, 'waiting')
        self.assertEqual(self.session.recovery_recheck, original)
        self.feed(.21)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.recovery_recheck)
        passed = json.loads((self.session.trace_directory / 'feedback-recheck-passed.json').read_text())
        self.assertTrue(passed['trust_hold_cleared'])
        self.assertFalse(passed['connection_hold_cleared'])
        self.assert_preserved()

    def test_malformed_receipt_never_launches_or_certifies_closed(self):
        for changes in ({'generation': True}, {'generation': 8}, {'serial_holders': [123]},
                        {'serial_open_attempts': 1}, {'board_files_written': True},
                        {'ssh_exit_code': 255}, {'observation_count': 1}):
            with self.subTest(changes=changes):
                self.session.recovery_recheck = host.recovery_recheck_error(host.CLOSURE_REASONS[0])
                self.proof_changes = changes
                self.check()
                self.workers.pop(0)()
                self.assertEqual(self.session.status, 'fault')
                self.assertIsNone(self.session.closed_proof)
                self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assertEqual(self.launches, [])
        self.assert_preserved()

    def test_running_local_process_and_unfinished_source_cannot_probe(self):
        self.session.process.returncode = None
        with self.assertRaises(ValueError):
            self.check()
        self.session.process.returncode = 255
        self.session.connection_done.clear()
        with self.assertRaises(ValueError):
            self.check()
        self.session.connection_done.set()
        self.session.source_transmitting = True
        with self.assertRaises(ValueError):
            self.check()
        self.assertEqual(self.checks, [])

    def test_no_process_without_audited_resume_is_not_a_finished_transport(self):
        self.session.process = None
        self.session._paused_without_attempt = True
        with self.assertRaises(ValueError):
            self.check()
        self.assertEqual(self.checks, [])

    def test_maintenance_holds_and_owner_block_probe(self):
        for field in ('maintenance_mapping_changed', 'maintenance_fault_hold'):
            setattr(self.session, field, True)
            with self.assertRaises(ValueError):
                self.check()
            setattr(self.session, field, False)
        self.session.maintenance = mock.Mock(owns_connection=True)
        with self.assertRaises(ValueError):
            self.check()
        self.assertEqual(self.checks, [])

    def test_requests_are_exact_and_generation_bound(self):
        for body in ({}, {'generation': True, 'purpose': 'imu'}, {'generation': 8, 'purpose': 'joints'},
                     {'generation': 7, 'purpose': 'all'},
                     {'generation': 7, 'purpose': 'imu', 'skip_guard': True}):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.session.check_connection_action(body)
        self.assertEqual(self.checks, [])

    def test_new_current_servo_fault_survives_old_quiescence_receipt(self):
        self.check()
        self.workers.pop(0)()
        self.workers.pop(0)()
        self.session.fault(host.UNSAFE_FEEDBACK_REASON)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.recovery_recheck['category'], 'servo')
        self.assertFalse(self.session.snapshot()['session']['connection_check']['allowed'])
        self.assert_preserved()

    def test_trust_fault_overrides_closure_and_late_good_feedback_cannot_clear(self):
        self.check()
        self.workers.pop(0)()
        self.workers.pop(0)()
        self.session.fault('Reviewed boot/root/card/model identity changed')
        self.feed(.01)
        self.feed(.21)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.recovery_recheck['category'], 'trust')
        self.assert_preserved()


class StoppedSessionReloadTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.run = self.base / 'feedback-calibration-test'
        self.run.mkdir(mode=0o700)
        self.attempt = self.run / 'feedback-attempt-test'
        self.attempt.mkdir(mode=0o700)
        self.expected = dict(generation=99, boot_id='pinned-boot', cid='pinned-card')
        self.error = '连续 0.6 秒未收到新反馈，采集已暂停；已保存校准保持不变。'
        self.finished = dict(generation=7, ssh_exit_code=255, session_status='fault',
                             closed_proof=None, error=self.error, error_details=self.error)
        self.evidence = copy.deepcopy(self.finished)
        self.write('connection-finished.json', self.finished)
        self.write('fault-evidence.json', self.evidence)
        self.write('expectations.json', dict(self.expected, generation=7))
        self.write('feedback-recheck-passed.json', dict(generation=7, purpose='joints', frame_count=4, span_ms=212))
        self.patch = mock.patch.object(host, 'RECOVERY', self.base.resolve())
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def write(self, name, value):
        path = self.attempt / name
        path.unlink(missing_ok=True)
        host.private_new(path, json.dumps(value))

    def load(self):
        return host.load_stopped_session(self.run, self.expected)

    def test_reload_retains_fault_and_never_grants_never_attempted_exception(self):
        record = self.load()
        from test_feedback_calibration_server import config
        session = host.CalibrationSession(config(), self.base, 7, recovery_launcher=mock.Mock(),
                                          connection_checker=mock.Mock())
        session.restore_stopped(record)
        self.assertEqual(session.status, 'fault')
        self.assertEqual(session.error, self.error)
        self.assertIsNone(session.process)
        self.assertIsNone(session.closed_proof)
        self.assertFalse(session._paused_without_attempt)
        self.assertFalse(session.snapshot()['session']['recheck_allowed'])
        self.assertTrue(session.snapshot()['session']['connection_check']['allowed'])
        session.recovery_launcher.assert_not_called()
        session.connection_checker.assert_not_called()

    def test_missing_exit_or_changed_identity_cannot_reload(self):
        self.write('connection-finished.json', dict(self.finished, ssh_exit_code=None))
        with self.assertRaises(ValueError):
            self.load()
        self.write('connection-finished.json', self.finished)
        self.write('expectations.json', dict(self.expected, generation=7, cid='new-card'))
        with self.assertRaises(ValueError):
            self.load()

    def test_late_raw_fault_report_cannot_be_erased_by_first_timeout(self):
        self.write('fault-abcd.json', dict(kind='fault', generation=7,
            reason='Reviewed boot/root/card/model identity changed'))
        with self.assertRaises(ValueError):
            self.load()

    def test_unfinished_additional_attempt_blocks_reload(self):
        (self.run / 'feedback-attempt-unfinished').mkdir(mode=0o700)
        with self.assertRaises(OSError):
            self.load()

    def test_nonprivate_source_rejected(self):
        (self.attempt / 'connection-finished.json').chmod(0o644)
        with self.assertRaises(ValueError):
            self.load()


if __name__ == '__main__':
    unittest.main()
