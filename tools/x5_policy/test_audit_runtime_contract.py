import copy
import math
import unittest
from audit_runtime_contract import assess_fixed_head_runtime_contract


def result():
    return dict(executor='fixed-head', speed_m_s=0., simulated_s=10.,
        ended=dict(reason='duration'), max_tilt_deg=1., displacement_m=[0., 0., 0.],
        outputs={stage: dict(first_range_violation=None, violating_samples=0) for stage in
            ('requested_execution_target', 'mcu_applied_goal', 'actual_joint')},
        actual_foot_motion=dict(feet={side: dict(contact_fraction=1.,
            qualifying_lift_intervals=[]) for side in ('left', 'right')}),
        head_max_deviation_deg=[0., 0., 0., 0.])


class FixedHeadContractTests(unittest.TestCase):
    def test_exact_boundary_rejects_a_base_screen_pass(self):
        value = result()
        value['head_max_deviation_deg'][0] = 2.
        assessment = assess_fixed_head_runtime_contract(value)
        self.assertTrue(assessment['base_assessment']['offline_screen_passed'])
        self.assertFalse(assessment['offline_contract_screen_passed'])
        self.assertIn('encoder_head_deviation_reached_2_deg', assessment['failures'])
        value['head_max_deviation_deg'][0] = 1.97
        self.assertTrue(assess_fixed_head_runtime_contract(value)['offline_contract_screen_passed'])
        value['head_max_deviation_deg'][0] = 22.5 * 360. / 4095
        assessment = assess_fixed_head_runtime_contract(value)
        self.assertFalse(assessment['offline_contract_screen_passed'])
        self.assertEqual(assessment['head_gate_trip_ticks'], 23)

    def test_missing_nonfinite_and_malformed_evidence_reject(self):
        for deviations in (None, [], [0.] * 3, [0., 0., 0., math.nan],
                           [0., 0., 0., -1.], [0., 0., 0., True]):
            value = result()
            value['head_max_deviation_deg'] = deviations
            self.assertFalse(assess_fixed_head_runtime_contract(value)['offline_contract_screen_passed'])
        value = result()
        value['executor'] = 'policy'
        self.assertFalse(assess_fixed_head_runtime_contract(value)['offline_contract_screen_passed'])

    def test_pass_never_authorizes_hardware_and_does_not_mutate_result(self):
        value = result()
        before = copy.deepcopy(value)
        assessment = assess_fixed_head_runtime_contract(value)
        self.assertTrue(assessment['offline_contract_screen_passed'])
        for name in ('physical_standing_verified', 'physical_walking_verified', 'deployment_authorized'):
            self.assertFalse(assessment[name])
        self.assertEqual(value, before)


if __name__ == '__main__':
    unittest.main()
