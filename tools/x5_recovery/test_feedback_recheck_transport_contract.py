"""Independent recheck boundary tests using only synthetic reader records."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import feedback_calibration_server as host
import feedback_stream
from test_feedback_calibration_server import Clock, FakeProcess, config


class RecheckTransportContractTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.clock = Clock()
        self.original = config()
        self.original['imu'].update(calibrated=True, axes_confirmed=True)
        host.atomic_calibration(self.directory, self.original)
        self.saved = (self.directory/'calibration-latest.json').read_bytes()
        self.launch_calls, self.workers = [], []
        owner = self
        class DeferredThread:
            def __init__(self, target, **kwargs): self.target = target
            def start(self): owner.workers.append(self.target)
            def is_alive(self): return False
            def join(self, **kwargs): pass
        self.thread_patch = mock.patch.object(host.threading, 'Thread', DeferredThread)
        self.process_patch = mock.patch.object(host.subprocess, 'Popen',
            side_effect=AssertionError('No hardware process in recheck contract tests'))
        self.thread_patch.start()
        self.process_patch.start()
        self.session = self.new_session()
        self.sequence, self.sample_us = 1, 1000000

    def tearDown(self):
        self.process_patch.stop()
        self.thread_patch.stop()
        self.temporary.cleanup()

    def new_session(self):
        session = host.CalibrationSession(self.original, self.directory, 7, self.clock,
                                          recovery_launcher=self.launch)
        session.start_initial(paused=True, hold_unsafe=True)
        return session

    def launch(self, session, generation, path):
        self.launch_calls.append((generation, path))
        session.process = FakeProcess()

    def begin(self, purpose):
        self.session.recheck_feedback_action(dict(generation=self.session.generation, purpose=purpose))
        self.assertEqual(len(self.workers), 1)
        self.workers.pop(0)()
        self.assertEqual(len(self.launch_calls), 1)
        self.session.accept(dict(kind='ready', generation=self.session.generation,
                                 emitted_monotonic=self.clock(), reader_elapsed_seconds=0.))

    def feed(self, online=32767, *, advance=.11, sequence=None, sample_advance=None):
        self.clock.now += advance
        self.sequence += 1
        self.sample_us += int(advance*1000000) if sample_advance is None else sample_advance
        state = dict(sample_us=self.sample_us, imu_us=self.sample_us-1000,
            online_mask=online, fault_mask=0, torque_mask=0, armed=False, arm_cookie=0,
            imu_ok=True, last_error=0, positions=[1000]*15,
            accel_ms2=[0.,0.,9.81], gyro_dps=[0.,0.,0.])
        record = dict(kind='state', generation=self.session.generation,
                      sequence=self.sequence if sequence is None else sequence, state=state,
                      received_monotonic=self.clock(), reader_elapsed_seconds=self.clock()-20.)
        try:
            self.session.accept(record)
        except (ValueError, KeyError) as error:
            # This is the same fail-closed boundary used by run_reader's
            # consume() around persist_reader_record; no real reader is run.
            self.session.fault(error)
        return record

    def assert_saved(self):
        self.assertEqual(self.session.config, self.original)
        self.assertEqual((self.directory/'calibration-latest.json').read_bytes(), self.saved)

    def test_same_inflight_generation_cannot_launch_twice(self):
        generation = self.session.generation
        self.session.recheck_feedback_action(dict(generation=generation, purpose='joints'))
        for candidate in (generation, self.session.generation):
            with self.assertRaises(ValueError):
                self.session.recheck_feedback_action(dict(generation=candidate, purpose='joints'))
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(self.launch_calls, [])
        self.assert_saved()

    def test_unsafe_then_new_hard_errors_are_not_hidden_by_original_reason(self):
        reasons = ('New kernel fault; classifier result requires source review: SD error',
                   'Serial USB physical topology changed',
                   'Serial USB VID/PID/serial identity changed',
                   'Canonical root block-node identity changed',
                   'Swap is active or its inventory is malformed',
                   'X5 的 SSH 身份已变化')
        for reason in reasons:
            with self.subTest(reason=reason):
                session = self.new_session()
                session.fault(reason)
                with self.assertRaises(ValueError):
                    session.recheck_feedback_action(dict(generation=session.generation, purpose='joints'))
        self.assertEqual(self.workers, [])
        self.assertEqual(self.launch_calls, [])
        self.assert_saved()

    def test_real_previous_process_requires_generation_bound_serial_closure(self):
        for proof, exit_code in ((None,20), ({'generation':7,'serial_closed':False},20),
                ({'generation':6,'serial_closed':True},20),
                ({'generation':7,'serial_closed':True},None)):
            with self.subTest(proof=proof, exit_code=exit_code):
                session = self.new_session()
                session.process = FakeProcess()
                session.process.returncode = exit_code
                session.closed_proof = copy.deepcopy(proof)
                session.connection_done.set()
                with self.assertRaises(ValueError):
                    session.recheck_feedback_action(dict(generation=session.generation,purpose='joints'))
        self.assertEqual(self.workers, [])
        self.assertEqual(self.launch_calls, [])

    def test_failed_unsafe_reader_exit_20_can_be_rechecked_only_with_actual_close(self):
        self.session.process = FakeProcess()
        self.session.process.returncode = 20
        self.session.closed_proof = dict(kind='closed', generation=7, normal=False, serial_closed=True)
        self.session.connection_done.set()
        self.session.recheck_feedback_action(dict(generation=7,purpose='joints'))
        self.assertEqual(len(self.workers), 1)
        self.assertEqual(self.launch_calls, [])
        self.assert_saved()

    def test_failed_launch_with_no_process_object_does_not_regain_paused_provenance(self):
        self.session.recovery_launcher = mock.Mock(side_effect=OSError('Popen outcome unknown'))
        self.session.recheck_feedback_action(dict(generation=7,purpose='joints'))
        self.workers.pop(0)()
        self.assertIsNone(self.session.process)
        self.assertTrue(self.session.connection_done.is_set())
        self.assertIsNotNone(self.session.recheck_blocked_reason())
        with self.assertRaises(ValueError):
            self.session.recheck_feedback_action(dict(generation=self.session.generation,purpose='joints'))
        self.session.recovery_launcher.assert_called_once()
        self.assert_saved()

    def test_maintenance_owner_and_persisted_holds_cannot_be_bypassed_by_recheck(self):
        for field in ('maintenance_mapping_changed', 'maintenance_fault_hold'):
            with self.subTest(field=field):
                session = self.new_session()
                setattr(session, field, True)
                with self.assertRaises(ValueError):
                    session.recheck_feedback_action(dict(generation=7,purpose='joints'))
        session = self.new_session()
        session.maintenance = mock.Mock(owns_connection=True)
        with self.assertRaises(ValueError):
            session.recheck_feedback_action(dict(generation=7,purpose='joints'))
        self.assertEqual(self.workers, [])
        self.assertEqual(self.launch_calls, [])

    def test_joints_need_a_point_two_second_advancing_window_not_one_good_frame(self):
        self.begin('joints')
        self.feed(advance=.01)
        self.assertNotEqual(self.session.status, 'active')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.feed(advance=.10)
        self.assertNotEqual(self.session.status, 'active')
        self.feed(advance=.11)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.recovery_recheck)
        self.assert_saved()

    def test_repeated_timestamps_cannot_unlock_even_after_wall_clock_window(self):
        self.begin('joints')
        self.feed(advance=.01)
        self.feed(advance=.21, sequence=self.sequence, sample_advance=0)
        self.assertNotEqual(self.session.status, 'active')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assert_saved()

    def test_zero_online_imu_success_retains_servo_hold_and_blocks_later_online_change(self):
        self.begin('imu')
        self.feed(0, advance=.01)
        self.feed(0, advance=.11)
        self.feed(0, advance=.11)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assertIsNotNone(self.session.maintenance_blocked_reason())
        with self.assertRaises(ValueError): self.session._require_joint_feedback()
        self.feed(32767, advance=.02)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.feedback)
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assert_saved()

    def test_joints_recheck_cannot_treat_all_offline_as_servo_healthy(self):
        self.begin('joints')
        self.feed(0, advance=.01)
        self.feed(0, advance=.25)
        self.assertNotEqual(self.session.status, 'active')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assert_saved()

    def test_powered_scope_does_not_silently_become_imu_after_success(self):
        self.begin('joints')
        for delta in (.01, .11, .11): self.feed(advance=delta)
        self.assertEqual(self.session.status, 'active')
        self.feed(0, advance=.02)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.feedback)
        self.assert_saved()

    def test_hard_fault_during_checking_cannot_be_cleared_by_late_good_frames(self):
        self.begin('joints')
        self.feed(advance=.01)
        reason = 'New kernel fault; classifier result requires source review: mmc I/O error'
        self.session.fault(reason)
        self.feed(advance=.25)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.feedback)
        self.assertEqual(self.session.recovery_recheck['technical_reason'], reason)
        self.assertEqual(self.session.feedback_recheck['phase'], 'failed')
        self.assert_saved()

    def test_existing_reader_frame_builder_cannot_send_maintenance_or_motion(self):
        namespace = {}
        exec(compile(feedback_stream.runtime_source(), '<readonly-reader-contract>', 'exec'), namespace)
        for kind in range(16):
            if kind in (1, 2):
                frame = namespace['_frame'](kind, 2)
                self.assertEqual(frame[:4], b'XD\x01' + bytes([kind]))
            else:
                with self.subTest(kind=kind), self.assertRaises(namespace['ProbeError']):
                    namespace['_frame'](kind, 2)

    def test_ending_during_checking_cannot_be_overridden_by_late_healthy_frame(self):
        self.begin('joints')
        self.feed(advance=.01)
        with mock.patch.object(host, 'write_pipe_once'):
            self.session.end_action(dict(generation=self.session.generation))
        self.feed(advance=.25)
        self.assertNotEqual(self.session.status, 'active')
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assert_saved()


if __name__ == '__main__':
    unittest.main()
