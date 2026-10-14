"""HOLD -> WALK continuity regression; actual controller, memory transport only."""
import json
import threading
import unittest
import numpy as np
import test_official_head_command as head

base, motion, args = head.base, head.motion, head.args
PROFILE = [-0.48095456322770314, -0.40883986006837697,
           -0.10126745550033033, -0.05216808313653381]


class HeadHandoffTests(base.RuntimeTests):
    profiled = head.HeadCommandTests.profiled

    def held(self, profile=PROFILE, send=True):
        c, t, sessions = self.profiled(profile)
        c.start_mode('hold', expected_generation=c._generation)
        if send:
            self.clock.advance(.02)
            c._cycle()
        return c, t, sessions

    def test_matching_head_actor_does_not_jump_toward_HOME_on_takeover(self):
        c, t, sessions = self.held()
        held = np.asarray(c._last_executed_target).copy()
        sessions['walk'].output[0, 5:9] = PROFILE
        c.start_mode('walk', expected_generation=c._generation)
        self.clock.advance(.02)
        c._cycle()
        np.testing.assert_allclose(c._last_executed_target[10:14], held[10:14], atol=3e-6, rtol=0)
        self.assertTrue(t.armed)

    def test_preflight_and_first_target_use_same_seed(self):
        c, t, sessions = self.held()
        sessions['walk'].output[:] = .01
        seen = []
        validate = c._raw_targets_locked
        def recorded(target, *a, **kw):
            seen.append(np.asarray(target).copy())
            return validate(target, *a, **kw)
        c._raw_targets_locked = recorded
        c.start_mode('walk', expected_generation=c._generation)
        first_check = seen[-1]
        self.clock.advance(.02)
        c._cycle()
        np.testing.assert_array_equal(first_check, c._last_executed_target)

    def test_only_head_filter_changes_raw_history_and_leg_output_stay_original(self):
        c, t, sessions = self.held()
        sessions['walk'].output[:] = .02
        c.start_mode('walk', expected_generation=c._generation)
        np.testing.assert_array_equal(c._previous_action, np.zeros(14, np.float32))
        legs = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13]
        np.testing.assert_array_equal(c._smooth_action[legs], np.zeros(10, np.float32))
        original = c._decode_policy_action('walk', sessions['walk'].output,
                                           np.zeros(14, np.float32), np.zeros(14, np.float32), 0., 0.)[0]
        self.clock.advance(.02)
        c._cycle()
        np.testing.assert_array_equal(c._last_executed_target[:10], original[:10])
        np.testing.assert_array_equal(sessions['walk'].observation[0, 34:48], np.zeros(14, np.float32))

    def test_direct_off_start_and_other_models_keep_original_zero_seed(self):
        c, t, sessions = self.profiled(PROFILE)
        c.start_mode('walk', expected_generation=c._generation)
        np.testing.assert_array_equal(c._smooth_action, np.zeros(14, np.float32))
        for mode in ('pick', 'getup', 'roulade'):
            c, t, sessions = self.held()
            c.start_mode(mode, expected_generation=c._generation, behavior_confirmed=True)
            np.testing.assert_array_equal(c._smooth_action, np.zeros(14, np.float32))

    def test_zero_profile_keeps_original_bitwise_reset(self):
        c, t, sessions = self.held([0., 0., 0., 0.])
        c.start_mode('walk', expected_generation=c._generation)
        np.testing.assert_array_equal(c._smooth_action, np.zeros(14, np.float32))

    def test_unsent_hold_does_not_invent_a_prior_target(self):
        c, t, sessions = self.held(send=False)
        self.assertIsNone(c._last_executed_target)
        c.start_mode('walk', expected_generation=c._generation)
        np.testing.assert_array_equal(c._smooth_action, np.zeros(14, np.float32))

    def test_repeated_walk_start_does_not_reuse_prior_hold_seed(self):
        c, t, sessions = self.held()
        c.start_mode('walk', expected_generation=c._generation)
        self.clock.advance(.02)
        c._cycle()
        c.start_mode('walk', expected_generation=c._generation)
        np.testing.assert_array_equal(c._smooth_action, np.zeros(14, np.float32))

    def test_stop_during_seeded_preflight_cancels_start(self):
        c, t, sessions = self.held()
        entered, release = threading.Event(), threading.Event()
        original = sessions['walk'].run
        def blocked(*a, **kw):
            entered.set()
            assert release.wait(3)
            return original(*a, **kw)
        sessions['walk'].run = blocked
        errors = []
        def start():
            try:
                c.start_mode('walk', expected_generation=c._generation)
            except motion.MotionError as e:
                errors.append(str(e))
        th = threading.Thread(target=start)
        th.start()
        try:
            self.assertTrue(entered.wait(3))
            c.stop()
            count = len([1 for k, p in t.requests if k in (motion.ARM, motion.TARGET, motion.SOURCE_TARGET)])
        finally:
            release.set()
            th.join(3)
        self.assertFalse(th.is_alive())
        self.assertTrue(errors)
        self.assertEqual(c._mode, 'off')
        self.assertFalse(t.armed)
        self.assertEqual(count, len([1 for k, p in t.requests if k in (motion.ARM, motion.TARGET, motion.SOURCE_TARGET)]))


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(HeadHandoffTests))
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(), tests=result.testsRun,
        failures=[str(v) for v in result.errors + result.failures], hardware_access=False), indent=2) + '\n')
    raise SystemExit(not result.wasSuccessful())
