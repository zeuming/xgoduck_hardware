"""CPU/fake-UART tests. No real hardware, sockets, SSH or device opens."""
from pathlib import Path
import copy, json, threading, time, unittest
from unittest import mock
import test_native_zero_adapter as fixture
import motion, production_motion as production

H = Path(__file__).resolve().parent
# Test the exact sealed worker classes, rather than the older tool fixture.
fixture.fixture.motion_workbench.WORKER = (H/'source/service/worker-classes.py').read_text()

class PhaseTimingTests(unittest.TestCase):
    setUp = fixture.NativeZeroAdapterTests.setUp
    tearDown = fixture.NativeZeroAdapterTests.tearDown
    prepare = fixture.NativeZeroAdapterTests.prepare
    create = fixture.NativeZeroAdapterTests.create
    controller = fixture.NativeZeroAdapterTests.controller
    ready_walk = fixture.NativeZeroAdapterTests.ready_walk
    def test_preinference_pause_is_recorded_without_TARGET_or_refresh(self):
        c = self.ready_walk()
        try:
            c._thread = threading.current_thread()
            original = production._copy_policy_state
            def delayed(value):
                if value is c._filtered:
                    time.sleep(.12)
                    c._clock.advance(.12)
                return original(value)
            before = sum(kind == motion.TARGET for kind, _ in c._transport.requests)
            c.heartbeat(expected_generation=c._generation, minimal=True)
            with mock.patch.object(production, '_copy_policy_state', delayed):
                with self.assertRaisesRegex(motion.MotionError, '舵机反馈已过期'):
                    c._cycle()
            timing = c._last_control_attempt
            phases = {name:(wall,cpu) for name,wall,cpu in timing['production_phases']}
            pause = phases['filtered_copy_end'][0]-phases['filtered_copy_begin'][0]
            self.assertGreaterEqual(pause, .11)
            self.assertEqual(timing['phase_timing_protocol'],1)
            self.assertEqual(timing['sample_us'],timing['current_sample_us'])
            self.assertGreaterEqual(timing['feedback_age_after_ms'],119.9)
            self.assertNotIn('TARGET_request_begin',phases)
            self.assertIn('feedback_gate_begin',phases)
            self.assertNotIn('feedback_gate_end',phases)
            self.assertEqual(sum(kind == motion.TARGET for kind, _ in c._transport.requests),before)
            self.assertLessEqual(len(phases),18)
            c.stop()
            saved = c.completed_trial_traces()[-1]['latency_at_stop']['control']
            self.assertEqual(saved['production_phases'],timing['production_phases'])
            json.dumps(saved,allow_nan=False)
        finally:
            c._thread=None
            c.close()

    def test_actual_worker_status_lock_wait_and_hold_are_bounded_and_readonly(self):
        runtime = self.create()
        namespace = runtime._new_namespace()
        c = namespace['OneShotController'](
            {n:fixture.FakePolicy(n) for n in fixture.app.MODEL_NAMES},
            self.calibration_path, clock=fixture.FakeClock(),allow_fixed_head_walk=True)
        self.assertTrue(c._validation_only)
        entered, release = threading.Event(), threading.Event()
        original = production.MotionController.status
        def slow(value):
            if threading.current_thread().name == 'fake-status-delay':
                entered.set()
                if not release.wait(2):raise AssertionError('test status timed out')
            return original(value)
        results = []
        with mock.patch.object(production.MotionController,'status',slow):
            t=threading.Thread(name='fake-status-delay',target=lambda:results.append(c.status()))
            t.start()
            self.assertTrue(entered.wait(1))
            waiting=threading.Event()
            def reader():
                waiting.set();results.append(c.status())
            other=threading.Thread(target=reader)
            other.start();self.assertTrue(waiting.wait(1))
            time.sleep(.06);release.set();t.join(2);other.join(2)
            self.assertFalse(t.is_alive());self.assertFalse(other.is_alive())
        rows=c._status_lock_timings
        self.assertLess(rows[-2][2]-rows[-2][1],.05)
        self.assertLess(rows[-1][1]-rows[-1][0],.05)
        self.assertTrue(results[0]['status_lock_diagnostics']['formatting_outside_control_lock'])
        for _ in range(20):result=c.status()
        self.assertEqual(len(c._status_lock_timings),8)
        self.assertEqual(result['status_lock_diagnostics']['recent'],c._status_lock_timings)
        self.assertIsNone(c._transport);self.assertIsNone(c._thread)
        self.assertEqual(namespace['request_counts'],{})
        json.dumps(result,allow_nan=False)

if __name__ == '__main__':unittest.main()
