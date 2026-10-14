"""Model-derived position lead for an OFFLINE geometric teacher.

BAM voltage P law: V = vin * kp * error_gain * ratio * (q_goal-q).
Motor torque: tau = kt/R * (V - kt*kd_scale*qdot).
At q=q_reference, the offset below supplies back-EMF and optionally the
current simulated friction budget. It is not gravity/contact compensation.
Physical limits and the plant remain outside this helper, unchanged.
"""
import numpy as np


def position_lead(velocity, *, mode, kt, resistance, vin, kp, error_gain,
                  error_gain_ratio, kd_scale, friction_budget, viscous, max_pwm):
    if mode not in ('none', 'back-emf', 'back-emf-friction'):
        raise ValueError('Unknown compensation experiment')
    velocity = np.asarray(velocity, dtype=np.float64)
    gain = np.asarray(vin)*np.asarray(kp)*error_gain*error_gain_ratio
    if not np.isfinite(gain).all() or np.any(gain <= 0) or kt <= 0 or resistance <= 0:
        raise ValueError('Invalid original motor parameters')
    back_emf = kt*np.asarray(kd_scale)*velocity
    friction_torque = (np.asarray(friction_budget)*np.sign(velocity)+viscous*velocity
                       if mode == 'back-emf-friction' else np.zeros_like(velocity))
    voltage = back_emf + resistance/kt*friction_torque
    if mode == 'none':
        voltage = np.zeros_like(velocity)
    bounded = np.clip(voltage, -np.asarray(vin)*max_pwm, np.asarray(vin)*max_pwm)
    return bounded/gain, dict(voltage_requested=voltage, voltage_bounded=bounded,
        desired_friction_torque=friction_torque, pwm_clipped=voltage != bounded)
