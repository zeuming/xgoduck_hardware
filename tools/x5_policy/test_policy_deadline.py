"""Late-result regression with a fake MCU watchdog; never opens robot hardware."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
import motion
from test_motion import FakeClock, FakePolicy, FakeTransport, ready_config


class PolicyDeadlineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = Path(self.tmp.name) / 'calibration.json'
        path.write_text(json.dumps(ready_config()))
        self.clock = FakeClock()
        self.policy = FakePolicy()
        clock = self.clock

        class WatchdogTransport(FakeTransport):
            last_target = None
            watchdog_stops = 0

            def request(self, kind, payload=b''):
                if self.armed and self.last_target is not None and clock()-self.last_target >= .250:
                    self.armed, self.torque, self.cookie = False, 0, 0
                    self.watchdog_stops += 1
                reply = super().request(kind, payload)
                if kind in (motion.ARM, motion.TARGET):
                    self.last_target = clock()
                return reply

        self.transport = WatchdogTransport('fake')
        self.c = motion.MotionController({'walk': self.policy}, path,
            transport_factory=lambda port: self.transport, clock=self.clock)
        self.c._connect_once()
        self.c.start_mode('hold', support_confirmed=True)
        for _ in range(155):
            self.clock.advance(.02)
            self.c._cycle()
        self.c.start_mode('walk', support_confirmed=True)
        self.c._cycle()

    def slow_once(self, delay=.11):
        original = self.policy.run
        def slow(*args):
            self.clock.advance(delay)
            return original(*args)
        return mock.patch.object(self.policy, 'run', side_effect=slow)

    def test_late_result_sends_nothing_and_does_not_refresh_feedback_or_action_state(self):
        c = self.c
        before = list(self.transport.requests)
        received, heartbeat = c._sample_received, c._heartbeat
        previous, smooth = c._previous_action.copy(), c._smooth_action.copy()
        self.policy.output[:] = .01
        with self.slow_once():
            c._control_once()
        self.assertEqual(self.transport.requests, before)
        self.assertEqual((c._sample_received, c._heartbeat), (received, heartbeat))
        np.testing.assert_array_equal(c._previous_action, previous)
        np.testing.assert_array_equal(c._smooth_action, smooth)
        self.assertEqual(c._mode, 'walk')
        self.assertTrue(self.transport.armed)
        self.assertEqual(c.status()['policy_deadline']['discarded_frames'], 1)
        self.assertGreaterEqual(c.status()['policy_deadline']['last_discard']['observation_age_ms'], 100)
        c._cycle()
        self.assertEqual([k for k, _ in self.transport.requests[len(before):]], [motion.STATE, motion.TARGET])
        np.testing.assert_array_equal(c._previous_action, self.policy.output[0])

    def test_frozen_feedback_after_discard_still_stops(self):
        with self.slow_once():
            self.c._control_once()
        self.transport.freeze_sample = True
        self.c._cycle()
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)

    def test_new_device_fault_after_discard_still_stops(self):
        with self.slow_once():
            self.c._control_once()
        self.transport.fault = 1
        self.c._cycle()
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)

    def test_repeated_slow_inference_cannot_renew_mcu_watchdog_or_rearm(self):
        arms = sum(k == motion.ARM for k, _ in self.transport.requests)
        with self.slow_once():
            for _ in range(4):
                self.c.heartbeat()
                self.c._cycle()
        self.assertEqual(self.transport.watchdog_stops, 1)
        self.assertEqual(self.c._mode, 'off')
        self.assertEqual(sum(k == motion.ARM for k, _ in self.transport.requests), arms)

    def test_already_stale_input_does_not_enter_inference(self):
        calls = self.policy.calls
        self.clock.advance(.11)
        with self.assertRaises(motion.FeedbackExpired):
            self.c._control_once()
        self.assertEqual(self.policy.calls, calls)
        self.assertEqual(self.c._late_policy_frames, 0)

    def test_stop_during_slow_inference_still_invalidates_result(self):
        original = self.policy.run
        def stop(*args):
            self.clock.advance(.11)
            self.c.stop()
            return original(*args)
        before = sum(k == motion.TARGET for k, _ in self.transport.requests)
        with mock.patch.object(self.policy, 'run', side_effect=stop):
            self.c._control_once()
        self.assertEqual(sum(k == motion.TARGET for k, _ in self.transport.requests), before)
        self.assertEqual(self.c._mode, 'off')

    def test_delay_inside_gate_never_sends_expired_target(self):
        original = self.c._motion_gate_locked
        def delayed(*args):
            self.clock.advance(.11)
            return original(*args)
        before = list(self.transport.requests)
        with mock.patch.object(self.c, '_motion_gate_locked', side_effect=delayed):
            self.c._control_once()
        self.assertEqual(self.transport.requests, before)
        self.assertEqual(self.c._late_policy_frames, 1)

    def test_range_fault_is_not_treated_as_scheduling_delay(self):
        self.policy.output[:] = 100
        with self.assertRaisesRegex(motion.MotionError, '机械范围'):
            self.c._control_once()
        self.assertEqual(self.c._late_policy_frames, 0)


if __name__ == '__main__':
    unittest.main()
