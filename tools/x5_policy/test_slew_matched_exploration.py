import copy
import math
from statistics import NormalDist
import unittest
from slew_matched_exploration import exploration_spec


class ExplorationScaleTests(unittest.TestCase):
    def setUp(self):
        self.candidate=dict(policy_joint_ids=[10,11,12,13,14,30,31,32,33,20,21,22,23,24],action_scale=1.,
            action=dict(goal_slew_ticks_s=600.,control_dt_s=.02,ticks_per_turn=4095))
    def test_derived_probability_and_original_parameters(self):
        before=copy.deepcopy(self.candidate);s=exploration_spec(self.candidate)
        self.assertEqual(self.candidate,before)
        z=s['original_goal_step_rad']/s['initial_latent_std']
        self.assertAlmostEqual((2*NormalDist().cdf(z)-1)**10,.99,places=12)
        self.assertAlmostEqual(s['original_goal_step_deg'],12/4095*360)
    def test_scaling_is_inverse_to_model_action_scale(self):
        a=exploration_spec(self.candidate)['initial_latent_std']
        self.candidate['action_scale']=2
        self.assertEqual(exploration_spec(self.candidate)['initial_latent_std'],a/2)
    def test_rejects_invalid_source(self):
        for v in [float('nan'),0.,-1.]:
            c=copy.deepcopy(self.candidate);c['action']['control_dt_s']=v
            with self.assertRaises(ValueError):exploration_spec(c)


if __name__=='__main__':unittest.main()
