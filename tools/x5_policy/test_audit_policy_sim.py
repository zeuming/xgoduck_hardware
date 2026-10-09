"""MuJoCo observation timing regressions; CPU only, no hardware modules."""
import unittest

import numpy as np

from audit_policy_sim import assess_result, refresh_observation_data

try:
    import mujoco as mj
except ImportError:
    mj = None


class ResultAssessmentTests(unittest.TestCase):
    def walking_result(self):
        return dict(ended=dict(reason='duration'), max_tilt_deg=4., speed_m_s=.2,
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


if __name__ == '__main__':
    unittest.main()
