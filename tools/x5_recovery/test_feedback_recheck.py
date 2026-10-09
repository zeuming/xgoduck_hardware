"""Purpose-bound, explicit feedback rechecks: synthetic records, no hardware."""
import copy
from email.message import Message
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import feedback_calibration_server as host
from test_feedback_calibration_server import Clock, FakeProcess, config


class FeedbackRecheckTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.clock = Clock()
        self.config = config()
        self.config['imu'].update(axes_confirmed=True, calibrated=True, bias_dps=[.1, .2, .3])
        host.atomic_calibration(self.directory, self.config)
        self.saved = (self.directory / 'calibration-latest.json').read_bytes()
        self.workers, self.launches, self.controls = [], [], []
        test = self
        class DeferredThread:
            def __init__(self, *, target, daemon):
                self.target = target
            def start(self):
                test.workers.append(self.target)
        for patch in (mock.patch.object(host.threading, 'Thread', DeferredThread),
                      mock.patch.object(host.subprocess, 'Popen', side_effect=AssertionError('No hardware')),
                      mock.patch.object(host, 'write_pipe_once', side_effect=lambda *a: self.controls.append(a))):
            patch.start()
            self.addCleanup(patch.stop)
        self.session = host.CalibrationSession(self.config, self.directory, 7, self.clock,
                                               recovery_launcher=self.launch)
        self.session.start_initial(paused=True, hold_unsafe=True)
        self.sequence = 1
        self.sample = 1000000

    def launch(self, session, generation, directory):
        self.launches.append((generation, directory))
        session.process = FakeProcess()
        session.accept(dict(kind='ready', generation=generation, emitted_monotonic=self.clock()))

    def begin(self, purpose='joints'):
        self.session.recheck_feedback_action(dict(generation=self.session.generation, purpose=purpose))
        self.workers.pop(0)()

    def feed(self, advance=.01, online=32767, changes=None, message_changes=None, sample_advance=None):
        self.clock.now += advance
        self.sequence += 1
        self.sample = (self.sample + (int(advance * 1e6) if sample_advance is None else sample_advance)) & 0xffffffff
        state = dict(sample_us=self.sample, imu_us=(self.sample - 1000) & 0xffffffff,
                     armed=False, arm_cookie=0, online_mask=online, torque_mask=0, fault_mask=0,
                     imu_ok=True, last_error=0, positions=[1000] * 15,
                     accel_ms2=[0., 0., 9.81], gyro_dps=[0., 0., 0.])
        state.update(changes or {})
        message = dict(kind='state', generation=self.session.generation, sequence=self.sequence,
                       received_monotonic=self.clock(), state=state)
        message.update(message_changes or {})
        try:
            self.session.accept(message)
        except (ValueError, KeyError, OSError) as error:
            # Same application boundary as run_reader's consume; no reader runs.
            self.session.fault(error)
        return message

    def closed(self, normal=False):
        self.session.accept(dict(kind='closed', generation=self.session.generation,
                                 serial_closed=True, normal=normal))
        self.session.process.returncode = 0 if normal else 20
        if normal:
            self.session.confirm_closed(0)
        self.session.connection_done.set()

    def assert_preserved(self):
        self.assertEqual(self.session.config, self.config)
        self.assertEqual((self.directory / 'calibration-latest.json').read_bytes(), self.saved)

    def test_status_is_pure_and_unsafe_regular_recovery_stays_forbidden(self):
        for _ in range(3):
            snap = self.session.snapshot()['session']
            self.assertTrue(snap['recheck_allowed'])
            self.assertFalse(snap['recovery_allowed'])
            self.assertIsNone(snap['feedback_recheck'])
        with self.assertRaises(ValueError):
            self.session.recover_action(dict(generation=7))
        self.assertEqual(self.workers, [])
        self.assertEqual(self.launches, [])

    def test_request_body_is_exact_and_rejections_never_launch(self):
        for body in ({}, {'generation': True, 'purpose': 'joints'}, {'generation': 8, 'purpose': 'imu'},
                     {'generation': 7}, {'generation': 7, 'purpose': 'all'},
                     {'generation': 7, 'purpose': ['joints']},
                     {'generation': 7, 'purpose': 'joints', 'skip_guard': True}):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.session.recheck_feedback_action(body)
        self.assertEqual(self.workers, [])
        self.assertEqual(self.launches, [])

    def test_claim_rotates_identity_preserves_evidence_and_calls_original_launcher_once(self):
        old_token, old_hold = self.session.token, copy.deepcopy(self.session.recovery_recheck)
        self.session.fault_record = {'generation': 7, 'error': 'original diagnostic'}
        result = self.session.recheck_feedback_action(dict(generation=7, purpose='joints'))
        generation = self.session.generation
        self.assertNotEqual(generation, 7)
        self.assertNotEqual(self.session.token, old_token)
        self.assertEqual(result['session']['status'], 'waiting')
        self.assertEqual(result['session']['feedback_recheck']['phase'], 'checking')
        self.assertIsNone(result['feedback'])
        self.assertEqual(self.launches, [])
        self.assertFalse(self.session.connection_done.is_set())
        evidence = json.loads((self.session.trace_directory / 'feedback-recheck-request.json').read_text())
        self.assertEqual(evidence['previous_hold'], old_hold)
        self.assertEqual(evidence['previous_fault']['error'], 'original diagnostic')
        for stale in (7, generation):
            with self.assertRaises(ValueError):
                self.session.recheck_feedback_action(dict(generation=stale, purpose='joints'))
        self.workers.pop(0)()
        self.assertEqual(self.launches, [(generation, self.session.trace_directory)])
        self.assert_preserved()

    def test_not_even_wizard_tick_can_run_before_complete_advancing_window(self):
        self.begin()
        with mock.patch.object(self.session, '_tick_wizard') as imu, \
                mock.patch.object(self.session, '_tick_joint') as joint:
            self.feed(.01)
            self.feed(.10)
            self.assertEqual(self.session.status, 'waiting')
            self.assertIsNone(self.session.feedback)
            self.assertIsNotNone(self.session.recovery_recheck)
            imu.assert_not_called()
            joint.assert_not_called()
            self.feed(.10)
            self.assertEqual(self.session.status, 'active')
            self.assertEqual(self.session.feedback_recheck['phase'], 'passed')
            self.assertIsNone(self.session.recovery_recheck)
            imu.assert_called_once()
            joint.assert_called_once()
        self.assertTrue((self.session.trace_directory / 'feedback-recheck-passed.json').is_file())
        self.assert_preserved()

    def test_sensor_window_must_advance_point_two_seconds_not_just_host_time(self):
        self.begin()
        self.feed(.01)
        self.feed(.25, sample_advance=1)
        self.assertEqual(self.session.status, 'waiting')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assertLess(self.session.feedback_recheck['span_ms'], .01)

    def test_sensor_timestamp_wrap_is_accepted_when_forward(self):
        self.sample = 0xffff0000
        self.begin()
        self.feed(.01)
        self.feed(.21)
        self.assertEqual(self.session.status, 'active')
        self.assert_preserved()

    def test_missing_last_error_is_not_proof_of_zero_fault(self):
        self.begin()
        record = self.feed(.01)
        state = copy.deepcopy(record['state'])
        del state['last_error']
        self.feed(.21, message_changes={'state': state})
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assertEqual(self.session.feedback_recheck['phase'], 'failed')
        self.assert_preserved()

    def test_failed_purpose_attempt_cannot_bypass_close_gate_through_regular_recover(self):
        self.session.recovery_recheck = None
        self.session.recovery_launcher = mock.Mock(side_effect=OSError('Popen outcome unknown'))
        self.begin('imu')
        self.assertIsNone(self.session.process)
        self.assertTrue(self.session.connection_done.is_set())
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        with self.assertRaises(ValueError):
            self.session.recover_action(dict(generation=self.session.generation))
        self.session.recovery_launcher.assert_called_once()

    def test_wrong_scope_is_retryable_only_after_actual_closure(self):
        self.begin('imu')
        self.feed(.01, online=32767)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.closed()
        self.assertIsNone(self.session.recheck_blocked_reason())
        self.begin('joints')
        self.feed(.01)
        self.feed(.21)
        self.assertEqual(self.session.status, 'active')
        self.assert_preserved()

    def test_imu_pass_is_not_permission_for_joints_or_maintenance(self):
        self.begin('imu')
        self.feed(.01, online=0)
        self.feed(.21, online=0)
        self.assertEqual(self.session.feedback_recheck['phase'], 'passed')
        self.assertIsNotNone(self.session.recovery_recheck)
        with self.assertRaises(ValueError):
            self.session._require_joint_feedback()
        self.assertIsNotNone(self.session.maintenance_blocked_reason())
        self.feed(.01, online=32767)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.feedback_recheck['phase'], 'failed')
        self.assert_preserved()

    def test_scope_switch_after_clean_manual_end_needs_new_request_and_window(self):
        self.begin('joints')
        self.feed(.01)
        self.feed(.21)
        self.session.end_action(dict(generation=self.session.generation))
        self.closed(normal=True)
        self.assertIsNone(self.session.recheck_blocked_reason())
        self.begin('imu')
        self.feed(.01, online=0)
        self.assertEqual(self.session.status, 'waiting')
        self.feed(.21, online=0)
        self.assertEqual(self.session.status, 'active')
        self.assertEqual(self.session.feedback_scope, 'imu')
        self.assertEqual(len(self.launches), 2)
        self.assert_preserved()

    def test_pause_provenance_is_not_inferred_from_missing_process(self):
        self.session._paused_without_attempt = False
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        with self.assertRaises(ValueError):
            self.session.recheck_feedback_action(dict(generation=7, purpose='imu'))
        self.assertEqual(self.workers, [])

    def test_clean_paused_without_hold_and_no_previous_reader_can_start(self):
        self.session = host.CalibrationSession(self.config, self.directory, 9, self.clock,
                                               recovery_launcher=self.launch)
        self.session.start_initial(paused=True)
        self.begin('imu')
        self.feed(.01, online=0)
        self.feed(.21, online=0)
        self.assertEqual(self.session.status, 'active')
        self.assert_preserved()

    def test_a_failed_scope_without_old_servo_hold_may_be_corrected_after_closure(self):
        self.session.recovery_recheck = None
        self.begin('joints')
        self.feed(.01, online=0)
        self.closed()
        self.assertIsNone(self.session.recheck_blocked_reason())
        self.begin('imu')
        self.assertEqual(len(self.launches), 2)
        self.assert_preserved()

    def test_new_hard_fault_does_not_hide_behind_previous_servo_hold(self):
        self.begin()
        self.session.fault(host.UNSAFE_FEEDBACK_REASON)
        first = copy.deepcopy(self.session.fault_record)
        self.session.fault('Serial USB VID/PID/serial identity changed')
        self.assertEqual(self.session.fault_record, first)
        self.assertEqual(self.session.recovery_recheck['technical_reason'],
                         'Serial USB VID/PID/serial identity changed')
        self.closed()
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.assert_preserved()

    def test_negative_close_proof_promotes_to_hard_hold(self):
        self.begin()
        self.session.fault(host.UNSAFE_FEEDBACK_REASON)
        self.session.accept(dict(kind='closed', generation=self.session.generation,
                                 serial_closed=False, normal=False))
        self.session.process.returncode = 20
        self.session.connection_done.set()
        self.assertFalse(host.is_unsafe_feedback(self.session.recovery_recheck['technical_reason']))
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.assert_preserved()

    def test_stale_queued_feedback_never_unlocks_or_ticks(self):
        self.begin()
        self.feed(.01)
        self.feed(.3, message_changes={'received_monotonic': self.clock() - 1.})
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assert_preserved()

    def test_gate_requires_workers_done_source_finished_and_strict_matching_closed_kind(self):
        self.session.process = FakeProcess()
        self.session.process.returncode = 20
        self.session.closed_proof = dict(kind='closed', generation=7, serial_closed=True)
        self.session.connection_done.clear()
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.session.connection_done.set()
        self.session.source_transmitting = True
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.session.source_transmitting = False
        for proof in ({'kind': 'ready', 'generation': 7, 'serial_closed': True},
                      {'kind': 'closed', 'generation': True, 'serial_closed': True},
                      {'kind': 'closed', 'generation': 7, 'serial_closed': 1}):
            self.session.closed_proof = proof
            self.assertIsNotNone(self.session.recheck_blocked_reason())
        self.assertEqual(self.launches, [])

    def test_pure_handler_uses_existing_csrf_boundary_and_new_exact_route(self):
        Handler = host.handler_for(self.session, 8767)
        instance = Handler.__new__(Handler)
        instance.path = '/api/recheck-feedback'
        instance.headers = Message()
        body = b'{"generation":7,"purpose":"imu"}'
        for key, value in {'Host': '127.0.0.1:8767', 'Origin': 'http://127.0.0.1:8767',
                           'X-CSRF-Token': self.session.token, 'Content-Type': 'application/json',
                           'Content-Length': str(len(body))}.items():
            instance.headers[key] = value
        instance.rfile = io.BytesIO(body)
        replies = []
        instance.reply = lambda code, value: replies.append((code, value))
        instance.do_POST()
        self.assertEqual(replies[0][0], 200)
        self.assertEqual(replies[0][1]['session']['feedback_recheck']['purpose'], 'imu')
        self.assertEqual(self.launches, [])
        self.assertEqual(len(self.workers), 1)
        self.assert_preserved()


if __name__ == '__main__':
    unittest.main()
