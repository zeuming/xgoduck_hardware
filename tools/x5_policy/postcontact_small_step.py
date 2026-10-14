"""Experimental gait rewards from post-forward contact, without extra physics.

The original environment still steps/resets/forwards exactly once as before.
Only the returned training reward is corrected, for nonterminal worlds. All
physical constraints and the independent walking acceptance gate stay fixed.
"""
import hashlib
import inspect
import torch
import small_step_experiment as base
from slow_gait_experiment import sole_heights, outside_observation_boundary

REMOVED = ('air_time', 'small_whole_sole_height', 'excess_whole_sole_height',
           'completed_alternating_landing')
SPEC = dict(name='post_forward_supported_small_step_v1',
    source='same post-forward CAD sole and original contact sensor as independent gate',
    qualified_air_height_m=.003, swing_max_load_n=.001, support_min_load_n=1.,
    air_span_min_s=.125, air_span_max_s=.65, clearance_target_m=.005,
    landing_span_s=.06, terminal_policy='no post-reset reward; reset temporal buffers',
    zero_command_reward_unchanged=True, extra_forward_calls=False,
    hardware_eligible=False, optimizer_continuation=False)


class PostContactReward:
    def __init__(self, env):
        self.dt = env.step_dt
        self.air_count = torch.zeros((env.num_envs, 2), device=env.device, dtype=torch.long)
        self.landing = base.AlternatingLanding(None, env)
        self.last_terms = {}

    def reset(self, ids=None):
        ids = slice(None) if ids is None else ids
        self.air_count[ids] = 0
        self.landing.reset(ids)

    def compute(self, height, load, active, episode_step, done):
        active = active & ~done
        qualified = ((height >= .003) & (load < .001) &
                     (load.flip(-1) >= 1.) & active[:, None])
        self.air_count = torch.where(qualified, self.air_count + 1,
                                     torch.zeros_like(self.air_count))
        span = (self.air_count - 1).clamp_min(0) * self.dt
        air = ((span > .125) & (span < .65) & qualified).float().sum(-1)
        height_value, excess = base.height_values(height, load)
        landing = self.landing.update(height, load, active, episode_step)
        terms = dict(post_supported_height=3. * height_value,
                     post_excess_height=-.5 * excess,
                     post_qualified_air=3. * air,
                     post_alternating_landing=.1 * landing)
        self.last_terms = {k: torch.where(active, v, torch.zeros_like(v))
                           for k, v in terms.items()}
        correction = sum(self.last_terms.values()) * self.dt
        if done.any():
            self.reset(done.nonzero(as_tuple=False).flatten())
        return correction


def configure(candidate, *, small_step=True, **kwargs):
    # Here the baseline is the already repaired SMALL-STEP reward, not the old
    # source reward that collapsed to standing. The trainer reports this arm.
    cfg, rl, spec = base.configure(candidate, small_step=True, **kwargs)
    spec['postcontact_reward'] = dict(**SPEC, enabled=small_step)
    if kwargs.get('training', True) and small_step:
        for name in REMOVED:
            cfg.rewards.pop(name)
    return cfg, rl, spec


def install_correction(env):
    """Return correction diagnostics; never write a simulated body/joint state."""
    if hasattr(env, '_postcontact_correction'):
        raise ValueError('Post-contact correction already installed')
    state = PostContactReward(env)
    original_step, original_reset = env.step, env.reset
    env._postcontact_correction = state
    env._postcontact_original_step_sha256 = hashlib.sha256(
        inspect.getsource(original_step).encode()).hexdigest()

    def reset(*args, **kwargs):
        result = original_reset(*args, **kwargs)
        state.reset(kwargs.get('env_ids'))
        return result

    def step(actions):
        # Commands advance inside env.step; the returned reward belongs to the
        # command that produced this transition, not the next command.
        command = env.command_manager.get_command('twist')[:, 0].clone()
        obs, reward, terminated, truncated, extras = original_step(actions)
        done = terminated | truncated
        load = env.scene['feet_ground_contact']._extract_sensor_data().force[:, :, 2].abs()
        active = (command > 0) & ~outside_observation_boundary(env)
        correction = state.compute(sole_heights(env), load, active,
                                   env.episode_length_buf, done)
        return obs, reward + correction, terminated, truncated, extras

    env.reset, env.step = reset, step
    return state
