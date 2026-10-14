"""Transparent initial exploration scale derived from the existing goal slew.

This is an opt-in learning initialization, never a servo limit or proof of
balance. The 99% joint-vector probability is an explicitly chosen diagnostic
coverage target, not a measured hardware safety margin. Arctan has slope <=1,
so pre-EMA leg perturbations are bounded probabilistically by latent noise.
"""
import math
from statistics import NormalDist


def exploration_spec(candidate, joint_vector_probability=.99):
    action=candidate['action'];scale=candidate['action_scale']
    count=sum(sid not in (30,31,32,33) for sid in candidate['policy_joint_ids'])
    if count!=10 or not 0<joint_vector_probability<1:raise ValueError('Expected ten leg coordinates and finite probability')
    values=[scale,action['goal_slew_ticks_s'],action['control_dt_s'],action['ticks_per_turn']]
    if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('Invalid source scale/slew')
    step_rad=action['goal_slew_ticks_s']*action['control_dt_s']*2*math.pi/action['ticks_per_turn']
    z=NormalDist().inv_cdf((1+joint_vector_probability**(1/count))/2)
    sigma=step_rad/(scale*z)
    return dict(name='one_period_slew_initialization_v1',initial_latent_std=sigma,
        leg_count=count,joint_vector_probability=joint_vector_probability,normal_quantile=z,
        original_goal_step_rad=step_rad,original_goal_step_deg=math.degrees(step_rad),
        derivation='sigma = existing per-period goal slew radians / action_scale / simultaneous Normal quantile',
        scope='pre-EMA perturbation relative to deterministic bounded actor on the same observation',
        unchanged=['physical_goal_slew','plant_ranges','rewards','gains','termination_conditions'],
        hardware_safe=False,balance_proven=False)
