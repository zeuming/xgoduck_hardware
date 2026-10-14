"""MuJoCo observation timing regressions; CPU only, no hardware modules."""
import unittest
import ast
from pathlib import Path

import numpy as np

from audit_policy_sim import (assess_result, refresh_observation_data, scheduled_command,
                             execution_action, command_distance, FootMotionTrace,
                             smooth_action, apply_official_contact_contract,
                             OFFICIAL_FRICTION_SOLREF, OFFICIAL_FRICTION_SOLIMP, REFERENCE)


class AdaptationScheduleTests(unittest.TestCase):
    def test_distance_accounts_for_warmup_ramps_and_early_stop(self):
        self.assertAlmostEqual(command_distance(10., .05, 5., 2., .5), .075)
        self.assertAlmostEqual(command_distance(5.25, .2, 5., 2., .5), .0125)
        self.assertAlmostEqual(command_distance(10., -.2, 5., 2., 0.), -.4)
        self.assertAlmostEqual(command_distance(10., .2, 5., None, .5), .95)
        self.assertEqual(command_distance(4., .2, 5., 2., .5), 0.)

    def test_pulse_has_zero_warmup_bounded_ramp_and_zero_recovery(self):
        times = [0., 4.99, 5., 5.25, 5.5, 6., 6.5, 6.75, 7., 9.]
        expected = [0., 0., 0., .1, .2, .2, .2, .1, 0., 0.]
        np.testing.assert_allclose([scheduled_command(t,.2,5.,2.,.5) for t in times], expected)

    def test_metadata_action_is_unmodified_and_independent(self):
        raw = np.arange(14,dtype=np.float32)
        output = execution_action(raw, {}, {}, 'metadata')
        np.testing.assert_array_equal(output, raw)
        output[0] = -10
        self.assertEqual(raw[0], 0.)

    def test_filter_keeps_raw_action_and_right_yaw_out_of_head_slice(self):
        raw = np.ones(14, dtype=np.float32)
        before = np.zeros(14, dtype=np.float32)
        filtered = smooth_action(before, raw, .2, .8)
        np.testing.assert_allclose(filtered[[0, 4, 9, 10, 13]], .8)
        np.testing.assert_allclose(filtered[5:9], .2)
        np.testing.assert_array_equal(raw, np.ones(14))
        np.testing.assert_array_equal(before, np.zeros(14))
        np.testing.assert_array_equal(smooth_action(before, raw, 0., 0.), raw)

    def test_frozen_or_invalid_filters_are_rejected(self):
        for value in (1., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                smooth_action(np.zeros(14), np.ones(14), value, .45)

try:
    import mujoco as mj
except ImportError:
    mj = None


class ResultAssessmentTests(unittest.TestCase):
    def walking_result(self):
        return dict(ended=dict(reason='duration'), max_tilt_deg=4., speed_m_s=.2,
            command_progress=dict(origin_at_s=5., direction_progress_m=.3),
            displacement_m=[.6, .01], outputs={stage: dict(violating_samples=0)
                for stage in ('requested_execution_target', 'mcu_applied_goal', 'actual_joint')},
            actual_foot_motion=dict(feet={side: dict(contact_fraction=.5,
                qualifying_lift_intervals=[dict(completed=True, duration_s=.1,
                    opposite_contact_s=.1) for _ in range(2)]) for side in ('left', 'right')}))

    def test_complete_stepping_is_only_an_offline_screen(self):
        result = assess_result(self.walking_result())
        self.assertTrue(result['offline_screen_passed'])
        for key in ('physical_standing_verified', 'physical_walking_verified', 'deployment_authorized'):
            self.assertFalse(result[key])

    def test_short_slow_pulse_is_not_compared_to_15_cm(self):
        report = self.walking_result()
        report['speed_m_s'] = .05
        report['command_progress']['direction_progress_m'] = .075
        self.assertTrue(assess_result(report)['offline_screen_passed'])
        report['command_progress']['direction_progress_m'] = -.01
        self.assertFalse(assess_result(report)['offline_screen_passed'])
        del report['command_progress']
        self.assertIn('command_window_progress_not_measured', assess_result(report)['failures'])

    def test_lift_finishes_only_after_contact_and_needs_full_clearance_duration(self):
        trace = object.__new__(FootMotionTrace)
        trace.dt = .01
        trace.rows = {side: dict(samples=0, contact_samples=0, minimum=None, maximum=None,
            min_clearance_m=None, max_clearance_m=None, last_contact=None,
            contact_changes=[], active_lift=None, lift_intervals=[]) for side in ('left','right')}
        def add(at, height, contact):
            trace.add(at, dict(left=dict(site_position_m=[0,0,height], mesh_min_height_m=height,
                contact=contact, lifted=height >= .003 and not contact),
                right=dict(site_position_m=[0,0,0], mesh_min_height_m=0, contact=True, lifted=False)))
        for i in range(5): add(i*.01, .004, False)
        add(.05, .002, False)
        self.assertIsNotNone(trace.rows['left']['active_lift'])
        self.assertEqual(trace.rows['left']['lift_intervals'], [])
        add(.06, 0., True)
        lift = trace.report(.06)['feet']['left']['qualifying_lift_intervals'][0]
        self.assertTrue(lift['completed'])
        self.assertAlmostEqual(lift['clearance_s'], .05)

    def test_survival_and_forward_fall_do_not_count_as_walking(self):
        report = self.walking_result()
        report['actual_foot_motion']['feet']['left']['qualifying_lift_intervals'] = []
        self.assertFalse(assess_result(report)['offline_screen_passed'])
        report = self.walking_result()
        report['max_tilt_deg'] = 15.
        self.assertFalse(assess_result(report)['offline_screen_passed'])

    def test_unfinished_or_unsupported_lifts_do_not_count(self):
        report = self.walking_result()
        report['actual_foot_motion']['feet']['left']['qualifying_lift_intervals'][0]['completed'] = False
        report['actual_foot_motion']['feet']['right']['qualifying_lift_intervals'][0]['opposite_contact_s'] = .049
        self.assertEqual(assess_result(report)['completed_supported_lifts'], dict(left=1, right=1))
        self.assertFalse(assess_result(report)['offline_screen_passed'])

    def test_counterfactual_violations_and_early_termination_fail(self):
        for stage in ('requested_execution_target', 'mcu_applied_goal', 'actual_joint'):
            report = self.walking_result()
            report['outputs'][stage]['violating_samples'] = 1
            self.assertFalse(assess_result(report)['offline_screen_passed'])
        report = self.walking_result()
        report['ended']['reason'] = 'fall_criterion'
        self.assertFalse(assess_result(report)['offline_screen_passed'])
        report = self.walking_result()
        report['range_ignoring_counterfactual'] = True
        self.assertFalse(assess_result(report)['offline_screen_passed'])

    def test_standing_needs_contact_and_small_drift(self):
        report = self.walking_result()
        report.update(speed_m_s=0., displacement_m=[.001, .001])
        for foot in report['actual_foot_motion']['feet'].values():
            foot.update(contact_fraction=1., qualifying_lift_intervals=[])
        self.assertTrue(assess_result(report)['offline_screen_passed'])
        report['actual_foot_motion']['feet']['left']['contact_fraction'] = .899
        self.assertFalse(assess_result(report)['offline_screen_passed'])


@unittest.skipIf(mj is None, 'CPU MuJoCo is required')
class ObservationTimingTests(unittest.TestCase):
    def setUp(self):
        self.model = mj.MjModel.from_xml_string('''<mujoco>
          <option timestep="0.005" gravity="0 0 0"/>
          <worldbody><body name="trunk" pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1"/>
            <site name="foot" pos="0.1 0 0"/>
          </body></worldbody></mujoco>''')
        self.data = mj.MjData(self.model)
        self.observed = mj.MjData(self.model)

    def test_integrated_attitude_and_foot_pose_without_changing_solver_state(self):
        data, model = self.data, self.model
        data.qvel[4] = 1.0
        mj.mj_forward(model, data)
        mj.mj_step(model, data)
        expected_rotation = np.empty(9)
        mj.mju_quat2Mat(expected_rotation, data.qpos[3:7])
        self.assertGreater(np.max(abs(expected_rotation - data.xmat[1])), .004)
        saved = {name: getattr(data, name).copy() for name in
                 ('qpos', 'qvel', 'xmat', 'site_xpos', 'qfrc_bias',
                  'qfrc_constraint', 'qfrc_actuator', 'qacc_warmstart')}
        refresh_observation_data(mj, model, data, self.observed)
        np.testing.assert_allclose(self.observed.xmat[1], expected_rotation, atol=1e-14)
        expected_foot = data.qpos[:3] + expected_rotation.reshape(3, 3) @ [.1, 0, 0]
        np.testing.assert_allclose(self.observed.site_xpos[0], expected_foot, atol=1e-14)
        self.assertEqual(self.observed.time, data.time)
        for name, old in saved.items():
            np.testing.assert_array_equal(getattr(data, name), old)

    def test_body_angular_velocity_uses_new_qvel(self):
        data, model = self.data, self.model
        data.qfrc_applied[4] = .004
        mj.mj_forward(model, data)
        mj.mj_step(model, data)
        velocity = np.zeros(6)
        mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, 1, velocity, 1)
        np.testing.assert_array_equal(velocity[:3], [0, 0, 0])
        self.assertAlmostEqual(data.qvel[4], .005)
        refresh_observation_data(mj, model, data, self.observed)
        mj.mj_objectVelocity(model, self.observed, mj.mjtObj.mjOBJ_BODY, 1, velocity, 1)
        np.testing.assert_allclose(velocity[:3], data.qvel[3:6], atol=1e-14)

    def test_collision_contract_does_not_change_joint_limits_or_visual_meshes(self):
        spec = mj.MjSpec.from_string('''<mujoco><worldbody>
          <body><joint name="hip" range="-20 30"/>
            <geom name="left_foot_collision" type="sphere" size="0.1"/>
            <geom name="trunk_collision" type="sphere" size="0.1"/>
            <geom name="named_visual" type="sphere" size="0.1" contype="0" conaffinity="0"/>
            <geom type="sphere" size="0.1"/>
          </body></worldbody></mujoco>''')
        limits = spec.joints[0].range.copy()
        apply_official_contact_contract(spec)
        model = spec.compile()
        foot = model.geom('left_foot_collision')
        self.assertEqual(foot.priority, 1)
        self.assertEqual(foot.condim, 3)
        self.assertEqual(foot.contype, 1)
        self.assertEqual(foot.conaffinity, 0)
        self.assertEqual(model.geom('trunk_collision').conaffinity, 2)
        self.assertEqual(model.geom('named_visual').contype, 0)
        self.assertEqual(spec.geoms[-1].contype, 0)
        np.testing.assert_array_equal(spec.joints[0].range, limits)

    @unittest.skipUnless((REFERENCE/'vendor/bam/bam/mjlab.py').exists(), 'Audited source unavailable')
    def test_friction_contract_constants_still_match_official_source(self):
        source = ast.parse((REFERENCE/'vendor/bam/bam/mjlab.py').read_text())
        literals = {}
        for node in ast.walk(source):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in (
                            '_STIFF_SOLREF_FRICTION', '_STIFF_SOLIMP_FRICTION'):
                        literals[target.id] = ast.literal_eval(node.value)
        self.assertEqual(literals['_STIFF_SOLREF_FRICTION'], OFFICIAL_FRICTION_SOLREF)
        self.assertEqual(literals['_STIFF_SOLIMP_FRICTION'], OFFICIAL_FRICTION_SOLIMP)


if __name__ == '__main__':
    unittest.main()
