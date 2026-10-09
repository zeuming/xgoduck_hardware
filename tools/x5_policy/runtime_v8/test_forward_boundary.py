"""CPU fake-UART regression: normal pulse expiry is not a release command."""
import unittest
import numpy as np
import motion
from bounded_forward import ForwardPulse, ForwardTargetExpired
from test_forward_trial import ForwardTrialTests as _Fixture


class ForwardBoundaryTests(unittest.TestCase):
    setUp = _Fixture.setUp
    tearDown = _Fixture.tearDown
    prepare = _Fixture.prepare
    controller = _Fixture.controller
    ready_walk = _Fixture.ready_walk
    advance = _Fixture.advance
    warmed = _Fixture.warmed
    trigger = _Fixture.trigger

    def test_inference_crossing_two_seconds_discards_then_reinfers_zero_without_release(self):
        c = self.warmed()
        try:
            self.trigger(c)
            self.advance(c, 1.96)
            generation, epoch = c._generation, c._forward.epoch
            requests = list(c._transport.requests)
            previous, smooth = c._previous_action.copy(), c._smooth_action.copy()
            policy, original = c.sessions['walk'], c.sessions['walk'].run

            def delayed(*args, **kwargs):
                value = original(*args, **kwargs)
                c._clock.advance(.06)
                # Emulate a concurrently accepted fresh sample, not a bypass
                # in production. The actual-expired-feedback case is below.
                c._sample_received = c._clock()
                return value

            policy.run = delayed
            self.advance(c, .02)
            self.assertEqual(c._transport.requests[len(requests):][0][0], motion.STATE)
            self.assertFalse(any(kind in (motion.TARGET, motion.STOP, motion.ARM)
                                 for kind, _ in c._transport.requests[len(requests):]))
            self.assertEqual(c._generation, generation)
            self.assertEqual(c._forward.epoch, epoch)
            self.assertEqual(c._mode, 'walk')
            self.assertTrue(c._transport.armed)
            self.assertEqual(list(c._command), [0., 0., 0.])
            self.assertEqual(c._forward.phase, 'returned-to-zero')
            np.testing.assert_array_equal(c._previous_action, previous)
            np.testing.assert_array_equal(c._smooth_action, smooth)

            policy.run = original
            before = sum(kind == motion.TARGET for kind, _ in c._transport.requests)
            self.advance(c, .02)
            self.assertEqual(sum(kind == motion.TARGET for kind, _ in c._transport.requests), before + 1)
            self.assertGreater(c._forward.zero_targets_after_positive, 0)
            self.assertEqual(c._mode, 'walk')
            self.assertTrue(c._transport.armed)
            self.assertEqual(c._generation, generation)
        finally:
            c.close()

    def test_real_faults_crossing_same_boundary_still_STOP_and_release(self):
        for fault in ('heartbeat', 'feedback', 'tilt', 'actualrange'):
            with self.subTest(fault=fault):
                c = self.warmed()
                try:
                    self.trigger(c)
                    self.advance(c, 1.96)
                    before = sum(kind == motion.TARGET for kind, _ in c._transport.requests)
                    uart, policy = c._transport, c.sessions['walk']
                    original = policy.run

                    def delayed(*args, **kwargs):
                        value = original(*args, **kwargs)
                        c._clock.advance(.401 if fault == 'heartbeat' else .12 if fault == 'feedback' else .06)
                        if fault not in ('heartbeat', 'feedback'):
                            c._sample_received = c._clock()
                        if fault == 'tilt':
                            c._tilt_locked = lambda: 8.01
                        elif fault == 'actualrange':
                            c._state['positions'][1] = self.calibration['joints'][1]['min'] - 1
                        return value

                    class OneCycle:
                        done = False
                        def is_set(self): return self.done
                        def set(self): self.done = True
                        def wait(self, seconds): self.done = True

                    policy.run = delayed
                    c._closed = OneCycle()
                    c._clock.advance(.02)
                    c.heartbeat(expected_generation=c._generation, minimal=True)
                    c._run()
                    self.assertEqual(sum(kind == motion.TARGET for kind, _ in uart.requests), before)
                    self.assertEqual(c._mode, 'off')
                    self.assertFalse(uart.armed)
                    self.assertTrue(any(kind == motion.STOP for kind, _ in uart.requests))
                finally:
                    c.close()

    def test_only_current_normal_deadline_has_expiry_type(self):
        pulse = ForwardPulse()
        pulse.begin(10., 7)
        epoch = pulse.epoch
        with self.assertRaises(ForwardTargetExpired) as found:
            pulse.check_inflight(12., 7, epoch)
        self.assertEqual((found.exception.generation, found.exception.epoch), (7, epoch))
        with self.assertRaises(ValueError):
            pulse.check_inflight(9., 7, epoch)
        with self.assertRaises(ValueError):
            pulse.check_inflight(12., 8, epoch)
        pulse.cancel()
        with self.assertRaises(ValueError):
            pulse.check_inflight(12., 7, epoch)


del _Fixture

if __name__ == '__main__':
    unittest.main()
