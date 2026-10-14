"""Declared offline forward-learning ablation, never hardware configuration.

Both arms use the same frozen, verified actor with a zero-initialized linear
residual. The treatment changes only the scale of the linear tracking reward.
This is actor initialization for a NEW experiment, not checkpoint continuation:
critic, Adam, KL schedule and simulator start afresh, explicitly.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import torch
from torch import nn


PEAK_SPEED = .2
MIN_SCALE_SPEED = .05
RELATIVE_TOLERANCE = .5


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative_tracking(command, actual, original_std):
    """Same xyz squared error as upstream; 50% relative tolerance in motion.

At >= .05 m/s, staying still earns exp(-4), rather than 94--98% at
.05--.1 m/s. The .05 floor is the lowest diagnostic command, avoiding a
singularity during ramp-up. Exactly-zero commands retain the upstream reward.
These are declared experimental objective choices, not proven optimal values.
"""
    speed = torch.linalg.vector_norm(command[:, :2], dim=1)
    scale = RELATIVE_TOLERANCE * speed.clamp_min(MIN_SCALE_SPEED)
    scale = torch.where(speed == 0, torch.full_like(scale, original_std), scale)
    error = ((command[:, :2] - actual[:, :2])**2).sum(dim=1) + actual[:, 2]**2
    return torch.exp(-error / scale**2)


def task_scaled_linear_velocity(env, std, command_name, asset_cfg):
    command = env.command_manager.get_command(command_name)
    if command is None:
        raise ValueError('The forward experiment has no command')
    return relative_tracking(command, env.scene[asset_cfg.name].data.root_link_lin_vel_b, std)


def apply_reward_experiment(cfg, kind):
    if kind not in ('original', 'relative-v1'):
        raise ValueError('Unknown forward reward experiment')
    from mjlab.tasks.velocity.mdp.rewards import track_linear_velocity
    term = cfg.rewards['track_linear_velocity']
    if term.func is not track_linear_velocity or term.weight != 2.:
        raise ValueError('Unexpected original linear tracking reward')
    before = copy.deepcopy(term.params)
    # The upstream default is a SceneEntityCfg argument, not always explicit.
    from mjlab.managers.scene_entity_config import SceneEntityCfg
    if kind == 'relative-v1':
        term.func = task_scaled_linear_velocity
        term.params.setdefault('asset_cfg', SceneEntityCfg('robot'))
    return dict(name=kind, scope='new_offline_objective_ablation',
        modified_terms=['track_linear_velocity'] if kind != 'original' else [],
        original_std=float(before['std']), original_weight=term.weight,
        relative_tolerance=RELATIVE_TOLERANCE, minimum_scale_speed_m_s=MIN_SCALE_SPEED,
        zero_command_reward_unchanged=True, physical_or_termination_changes=False,
        parameter_basis='50 percent relative error gives exp(-1); stationary motion-command reward exp(-4); floor is lowest already-diagnosed command .05 m/s',
        status='experimental design, not an identified optimal reward')


class ZeroPreservingResidual(nn.Module):
    """Frozen latent actor plus command-gated linear correction to ten legs.

Use the normalized representation of exact zero, computed with the identical
arithmetic as the frozen normalizer. Thus gate(0) is exactly 0, without a
tolerance/deadband that would silently change the command contract. No history
or hidden clock is added; observation and action orders remain unchanged.
"""
    def __init__(self, base, mean, denominator, leg_mask):
        super().__init__()
        self.base = base
        self.base.requires_grad_(False)
        self.correction = nn.Linear(61, 14, device=mean.device, dtype=mean.dtype)
        nn.init.zeros_(self.correction.weight)
        nn.init.zeros_(self.correction.bias)
        self.register_buffer('zero_forward', ((torch.zeros_like(mean) - mean) / denominator)[:, 48:49].clone())
        self.register_buffer('forward_denominator', denominator[:, 48:49].clone())
        self.register_buffer('leg_mask', torch.as_tensor(leg_mask, device=mean.device, dtype=mean.dtype).reshape(1, 14))

    def forward(self, normalized):
        gate = ((normalized[:, 48:49] - self.zero_forward) * self.forward_denominator / PEAK_SPEED).clamp(0., 1.)
        return self.base(normalized) + gate * self.leg_mask * self.correction(normalized)


def install_residual(runner, candidate):
    actor = runner.alg.actor
    if hasattr(actor.mlp, 'correction'):
        raise ValueError('Cannot install the residual twice')
    normalizer = actor.obs_normalizer
    actor.mlp = ZeroPreservingResidual(actor.mlp, normalizer.onnx_mean,
        normalizer.onnx_denominator, [sid < 30 for sid in candidate['policy_joint_ids']])
    # Keep the source's learned exploration exactly fixed in both arms. This
    # isolates reward learning from the already-rejected sigma sweep.
    actor.distribution.requires_grad_(False)
    optimizer = runner.alg.optimizer
    if optimizer.state or len(optimizer.param_groups) != 1:
        raise ValueError('Residual must be installed before any optimizer work')
    optimizer.param_groups[0]['params'] = list(actor.parameters()) + list(runner.alg.critic.parameters())
    return dict(kind='frozen_base_plus_linear_61_to_14_forward_residual_v1',
        gate='clamp(vx / .2, 0, 1), computed relative to exact normalized zero',
        held_head_unchanged=True, base_parameters_frozen=True, source_sigma_frozen=True,
        trainable_actor_parameters=sum(p.numel() for p in actor.parameters() if p.requires_grad),
        zero_guarantee='same deterministic mapping at exactly zero vx, not a guarantee of recovery from a different state')


def initialize_from_actor_run(path, runner, current, environment_config):
    """Load only actor after checking source/physics/transform provenance."""
    from train_constrained_policy import assert_state_identical, tensor_state_sha
    path = Path(path).resolve()
    previous = json.loads((path / 'run.json').read_text())
    stages = {'offline_training_run_completed_unaccepted', 'offline_training_wall_budget_completed_unaccepted',
              'offline_training_candidate_rejected'}
    if previous.get('stage') not in stages or previous.get('forward_experiment'):
        raise ValueError('Need a completed original-architecture actor run')
    for key in ('candidate_sha256', 'original_model_sha256', 'task_profile_sha256', 'dependencies',
                'api_sources', 'distribution_source_sha256', 'bounded_distribution',
                'warmstart_importer_sha256', 'validation_baseline'):
        if previous[key] != current[key]:
            raise ValueError('Actor seed contract differs: ' + key)
    for file, key in [('runner-source.py', 'runner_source_sha256'), ('distribution-source.py', 'distribution_source_sha256'),
                      ('warmstart-importer-source.py', 'warmstart_importer_sha256')]:
        if sha(path / file) != previous[key]:
            raise ValueError('Actor seed source archive differs: ' + file)
    old_env = json.loads((path / 'effective-environment-config.json').read_text())
    new_env = json.loads(Path(environment_config).read_text())
    if current['forward_experiment']['reward']['name'] == 'relative-v1':
        old_term = old_env['rewards']['track_linear_velocity']
        new_term = new_env['rewards']['track_linear_velocity']
        if new_term['weight'] != old_term['weight'] or new_term['params']['std'] != old_term['params']['std']:
            raise ValueError('Unexpected additional objective change')
        # This exact, recorded term replacement is the entire treatment.
        new_env['rewards']['track_linear_velocity'] = copy.deepcopy(old_term)
    for config in (old_env, new_env):
        config.pop('seed', None)
        config['scene'].pop('num_envs', None)
    if old_env != new_env:
        raise ValueError('Actor seed physics/command/other rewards differ')
    checkpoint = path / 'checkpoint.pt'
    if sha(checkpoint) != previous['checkpoint']['sha256']:
        raise ValueError('Actor seed checkpoint hash differs')
    saved = torch.load(checkpoint, map_location=runner.device, weights_only=True)
    infos = saved['infos']
    if (infos['run_id'] != previous['run_id'] or infos['candidate_sha256'] != current['candidate_sha256']
            or infos['task_profile_sha256'] != current['task_profile_sha256']
            or tensor_state_sha(saved['actor_state_dict']) != previous['actor_state_after_sha256']):
        raise ValueError('Actor seed state binding differs')
    before_buffers = {k: v.detach().clone() for k, v in runner.alg.actor.distribution.named_buffers()}
    runner.alg.actor.load_state_dict(saved['actor_state_dict'], strict=True)
    assert_state_identical(saved['actor_state_dict'], runner.alg.actor.state_dict())
    assert_state_identical(before_buffers, dict(runner.alg.actor.distribution.named_buffers()))
    return dict(parent_run_id=previous['run_id'], parent_run_json_sha256=sha(path / 'run.json'),
        checkpoint_sha256=sha(checkpoint), actor_sha256=previous['actor_state_after_sha256'],
        checkpoint_resume=False, critic_restored=False, optimizer_restored=False, kl_schedule_restored=False,
        initialization='source actor and learned sigma only; new residual, critic, Adam, KL schedule, simulator and RNG')


def audit_zero_mapping(actor, source_state, observations):
    """After learning check exact base/sigma freeze and zero latent equality."""
    from train_constrained_policy import assert_state_identical
    from tensordict import TensorDict
    assert_state_identical({k.removeprefix('mlp.'): v for k, v in source_state.items() if k.startswith('mlp.')}, actor.mlp.base.state_dict())
    assert_state_identical({k.removeprefix('distribution.'): v for k, v in source_state.items() if k.startswith('distribution.')}, actor.distribution.state_dict())
    raw = observations.detach().clone()
    raw[:, 48:51] = 0.
    with torch.inference_mode():
        normalized = actor.obs_normalizer(raw)
        expected = actor.mlp.base(normalized)
        actual = actor.mlp(normalized)
        assert_state_identical(expected, actual)
        actions = actor(TensorDict({'actor': raw}, batch_size=[len(raw)]))
        if not torch.isfinite(actions).all():
            raise ValueError('Nonfinite zero-command residual policy')
    return dict(samples=len(raw), exact_latent_equality=True, base_parameters_unchanged=True,
        source_distribution_unchanged=True, recovery_stability_proven=False)
