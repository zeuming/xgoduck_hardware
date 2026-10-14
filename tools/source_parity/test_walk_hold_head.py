"""Actual controller regression; simulated UART only, no device connection."""
import copy
import json
import unittest
import numpy as np
import test_source_host as base
import test_head_handoff as handoff

motion, args = base.motion, base.args
PROFILE = handoff.PROFILE


class WalkHoldHeadTests(unittest.TestCase):
    setUp = base.RuntimeTests.setUp
    profiled = handoff.HeadHandoffTests.profiled
    held = handoff.HeadHandoffTests.held
    test_stop_during_preflight_discards_pending_start = (
        handoff.HeadHandoffTests.test_stop_during_seeded_preflight_cancels_start)

    def test_zero_forward_backward_side_and_turn_retain_same_hold_head(self):
        c, t, sessions = self.held()
        held = np.asarray(c._last_executed_target).copy()
        c.start_mode('walk', expected_generation=c._generation)
        for command in ((0.,0.,0.),(.3,0.,0.),(-.2,0.,0.),(0.,.2,0.),(0.,0.,.5)):
            c.command(*command)
            for value in (.05, -.04, .07):
                # Deliberately disagree with the HOLD head. The old input-only
                # implementation follows these outputs and fails this test.
                sessions['walk'].output[:] = value
                self.clock.advance(.02)
                c.heartbeat(expected_generation=c._generation)
                c._cycle()
                np.testing.assert_array_equal(c._last_executed_target[10:14], held[10:14])
                np.testing.assert_array_equal(c._last_executed_target[:10], c._last_action[:10])
                self.assertEqual(c._last_executed_target[14], c._last_action[14])
                np.testing.assert_array_equal(c._previous_action, sessions['walk'].output[0])
                np.testing.assert_allclose(sessions['walk'].observation[0,48:51], command, atol=1e-7)
                np.testing.assert_array_equal(sessions['walk'].observation[0,51:55],
                                              np.asarray(PROFILE,np.float32))
                wire = [p for k,p in t.requests if k in (motion.TARGET,motion.SOURCE_TARGET)][-1]
                self.assertEqual(wire, c._target_payload_locked(c._raw_targets_locked(c._last_executed_target)))
        self.assertTrue(t.armed)
        self.assertTrue(c.status()['official_runtime']['head_fixed'])

    def test_head_goal_is_replacement_not_double_offset(self):
        c,t,s = self.profiled(PROFILE)
        held = c._fixed_stance_locked()
        for actor_head in ([0.,0.,0.,0.], held[10:14], [5.,6.,7.,8.]):
            policy = list(c._stand_deg); policy[10:14] = actor_head
            untouched = policy.copy()
            actual = c._execution_target_locked('walk', policy)
            np.testing.assert_array_equal(actual[10:14], held[10:14])
            np.testing.assert_array_equal(policy, untouched)
            rows = c.status()['walk_head_execution']['last']
            self.assertEqual([r['policy_deg'] for r in rows], list(actor_head))
            self.assertEqual([r['hold_deg'] for r in rows], held[10:14])

    def test_other_models_and_default_recovery_keep_their_own_targets(self):
        c,t,s = self.profiled(PROFILE)
        goal = list(c._stand_deg); goal[10:14] = [4.,3.,2.,1.]
        for selected in ('hold','getup','pick','roulade'):
            np.testing.assert_array_equal(c._execution_target_locked(selected,goal), goal)
        c._mode = 'walk'
        c._recovery.phase = 'default_pose'
        self.assertEqual(c._select_policy_locked('walk'), 'getup')
        recovery = c._default_recovery_target_locked()
        np.testing.assert_array_equal(c._execution_target_locked('getup', recovery), recovery)

    def test_saved_bounds_still_apply_after_head_replacement(self):
        c,t,s = self.profiled(PROFILE)
        held = c._fixed_stance_locked()
        joint = c._config['joints'][10]
        raw = int(round(joint['zero'] + held[10]/360*4095*joint['sign']))
        joint['min'], joint['max'] = raw+5, raw+100
        result = c._execution_target_locked('walk',c._stand_deg)
        encoded = int(round(joint['zero']+result[10]/360*4095*joint['sign']))
        self.assertEqual(encoded,joint['min']+1)
        self.assertEqual(c._target_projection[0]['id'],30)
        self.assertEqual(c._config['joints'][10]['min'],raw+5)

    def test_zero_profile_preserves_actor_output(self):
        c,t,s = self.profiled([0.,0.,0.,0.])
        target = list(c._stand_deg); target[10:14] = [4.,3.,2.,1.]
        np.testing.assert_array_equal(c._execution_target_locked('walk',target),target)

    def test_invalid_actor_output_cannot_be_hidden_by_head_replacement(self):
        c,t,s = self.profiled(PROFILE)
        target = list(c._stand_deg); target[10] = float('nan')
        with self.assertRaises(motion.MotionError): c._execution_target_locked('walk',target)


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(WalkHoldHeadTests))
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        failures=[str(v) for v in result.errors+result.failures],
        hardware_access=False, physical_balance_tested=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
