"""Test the actual packaged controller with an in-memory transport, never UART."""
import copy
import json
import struct
import numpy as np
import test_source_host as base

motion, args = base.motion, base.args


class TargetClampTests(base.RuntimeTests):
    def angles_for(self, ticks):
        return [(v-j['zero'])*360/4095*j['sign']
                for v, j in zip(ticks, self.c._config['joints'])]

    def raw(self, target, mode='walk'):
        return self.c._raw_targets_locked(self.c._execution_target_locked(mode, target))

    def test_reported_joint13_and_joint22_failures_now_bound_inward(self):
        cfg = self.c._config['joints']
        cfg[3].update(min=1300, max=3000)
        cfg[7].update(min=1230, max=3837)
        ticks = [j['zero'] for j in cfg]
        ticks[3], ticks[7] = 1212, 3949
        actual = self.raw(self.angles_for(ticks))
        self.assertEqual(actual[3], 1301)
        self.assertEqual(actual[7], 3836)
        self.assertEqual(len(self.c._target_projection), 2)
        for i in set(range(15)) - {3, 7}:
            self.assertEqual(actual[i], ticks[i])

    def test_all_joints_both_signs_and_outside_encoder_domain(self):
        for sign in (-1, 1):
            for j in self.c._config['joints']:
                j.update(sign=sign, min=1000, max=3000)
            for requested, expected in [(-100000,1001), (999,1001),
                                        (3001,2999), (100000,2999)]:
                self.assertEqual(self.raw(self.angles_for([requested]*15)), [expected]*15)

    def test_endpoints_and_in_range_angles_are_not_trimmed_or_quantized(self):
        cfg = self.c._config['joints']
        for ticks in ([j['min'] for j in cfg], [j['max'] for j in cfg],
                      [j['zero']+.123 for j in cfg]):
            target = self.angles_for(ticks)
            executed = self.c._execution_target_locked('walk', target)
            self.assertEqual(executed, target)
            self.assertEqual(self.c._target_projection, [])

    def test_one_tick_wide_valid_range_never_produces_outside_value(self):
        for j in self.c._config['joints']:
            j.update(min=2000, max=2001)
        self.assertEqual(self.raw(self.angles_for([1999]*15)), [2001]*15)
        self.assertEqual(self.raw(self.angles_for([2002]*15)), [2000]*15)

    def test_scope_includes_hold_and_recovery_and_mouth(self):
        cfg = self.c._config['joints']
        target = self.angles_for([j['max']+100 for j in cfg])
        expected = [j['max']-1 for j in cfg]
        for mode in ('hold','walk','getup','pick','roulade'):
            self.assertEqual(self.raw(target, mode), expected)

    def test_saturation_does_not_modify_config_actor_ema_or_raw_history(self):
        config, previous, smooth = copy.deepcopy(self.c._config), self.c._previous_action.copy(), self.c._smooth_action.copy()
        target = self.angles_for([j['max']+100 for j in config['joints']])
        original = list(target)
        self.raw(target)
        self.assertEqual(target, original)
        self.assertEqual(self.c._config, config)
        np.testing.assert_array_equal(self.c._previous_action, previous)
        np.testing.assert_array_equal(self.c._smooth_action, smooth)

    def test_start_and_continuous_outside_requests_send_only_bounded_wire_goals(self):
        self.sessions['walk'].output[:] = 4.0
        self.start_walk()  # preflight formerly stopped before ARM
        for _ in range(6):
            self.step()
        goals = [p for k,p in self.transport.requests if k in (motion.TARGET,motion.SOURCE_TARGET)]
        self.assertEqual(len(goals), 6)
        for payload in goals:
            ticks = struct.unpack_from('<I15H', payload)[1:]
            for raw,j in zip(ticks,self.c._config['joints']):
                self.assertLessEqual(j['min'],raw)
                self.assertLessEqual(raw,j['max'])
        self.assertEqual(self.c._mode,'walk')
        self.assertTrue(self.transport.armed)
        self.assertIsNone(self.c._error)
        np.testing.assert_array_equal(self.c._previous_action, np.full(14,4.0,np.float32))
        self.assertNotEqual(self.c._last_policy_target,self.c._last_executed_target)
        self.assertEqual(self.c._trace[-1]['target_range_projection'], self.c._target_projection)
        self.assertEqual(self.c._policy_trace[-1]['transmitted_ticks'], list(struct.unpack_from('<I15H',goals[-1])[1:]))
        self.assertTrue(self.c._policy_trace[-1]['target_write_completed'])
        self.assertTrue(self.c._policy_trace[-1]['target_range_projection'])

    def test_projection_clears_on_next_in_range_target(self):
        self.raw([999.]*15)
        self.assertTrue(self.c._target_projection)
        self.raw(self.angles_for([j['zero'] for j in self.c._config['joints']]))
        self.assertEqual(self.c._target_projection, [])
        self.assertEqual(self.c._head_projection, [])

    def test_invalid_policy_values_do_not_become_actuation(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            before = len(self.transport.requests)
            with self.assertRaises(motion.MotionError):
                self.c._execution_target_locked('walk',[value]*15)
            self.assertEqual(before,len(self.transport.requests))
        with self.assertRaises(motion.MotionError):
            self.c._execution_target_locked('walk',[0.]*14)

    def test_actual_position_gate_still_rejects_physical_outside_range(self):
        self.start_walk()
        self.c._state['positions'][0] = self.c._config['joints'][0]['min']-1
        with self.assertRaisesRegex(motion.MotionError, '实际位置'):
            self.c._motion_gate_locked('walk')

    def test_in_range_packet_matches_original_target_exactly(self):
        self.sessions['walk'].output[:] = .05
        self.start_walk();self.step()
        target = self.c._last_policy_target
        expected = [int(round(j['zero']+a/360*4095*j['sign']))
                    for a,j in zip(target,self.c._config['joints'])]
        sent = [p for k,p in self.transport.requests if k in (motion.TARGET,motion.SOURCE_TARGET)][-1]
        self.assertEqual(list(struct.unpack_from('<I15H',sent)[1:]),expected)
        self.assertEqual(self.c._last_executed_target,target)


if __name__ == '__main__':
    result = base.unittest.TextTestRunner(verbosity=2).run(base.unittest.defaultTestLoader.loadTestsFromTestCase(TargetClampTests))
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        failures=[str(v) for v in result.errors+result.failures],hardware_access=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
