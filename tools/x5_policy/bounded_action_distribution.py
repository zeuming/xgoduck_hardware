"""A new bounded exploration policy, not a clipped version of an old model.

Ten leg actions use a fixed HOME-anchored arctan bijection; the four physically
ignored head actions stay unchanged, including in action history. The Gaussian
is in latent coordinates. PPO evaluates the transformed action density, its
Jacobian and exact (invariant) latent-normal KL. Entropy is an unbiased,
reparameterized Monte Carlo estimate of the transformed differential entropy.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import torch
from rsl_rl.modules.distribution import GaussianDistribution


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def distribution_config(candidate):
    ids = candidate['policy_joint_ids']
    if len(ids) != 14 or len(set(ids)) != 14 or not {30, 31, 32, 33}.issubset(ids):
        raise ValueError('Expected 14 unique policy joints including the four fixed-head joints')
    scale = candidate['action_scale']
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid action scale')
    low = [(a - h) / scale for a, h in zip(candidate['lower_rad'], candidate['home_rad'])]
    high = [(b - h) / scale for b, h in zip(candidate['upper_rad'], candidate['home_rad'])]
    mask = [sid not in (30, 31, 32, 33) for sid in ids]
    if len(low) != 14 or len(high) != 14:
        raise ValueError('Expected 14 saved ranges')
    if any(not math.isfinite(v) for v in low + high):
        raise ValueError('Nonfinite saved range')
    if any(mask[i] and not low[i] < 0 < high[i] for i in range(14)):
        raise ValueError('Every leg HOME must be strictly inside its original saved range')
    return dict(schema_version=1, kind='home_anchored_piecewise_arctan_gaussian',
                policy_joint_ids=list(ids), lower_action=low, upper_action=high,
                transformed_mask=mask, head_action_transform='identity; physical head remains held by the plant',
                formula='r * (2/pi) * atan(pi*z/(2*r)); r=upper if z>=0 else -lower',
                home_zero_preserved=True, derivative_at_zero=1.,
                entropy='Normal analytic entropy + rsample log-Jacobian; unbiased pathwise Monte Carlo estimate',
                kl='exact latent-Normal KL only for identical fixed transform signatures',
                std_reporting='latent Normal sigma, not transformed action standard deviation',
                deterministic_reporting='transform(latent mean), a median/location; not transformed expected mean',
                output_history='transformed 14D output; same output emitted by deterministic ONNX',
                original_onnx_function_equivalent=False)


class ArctanActionTransform(torch.nn.Module):
    def __init__(self, config, *, device='cpu'):
        super().__init__()
        self.register_buffer('lower', torch.tensor(config['lower_action'], dtype=torch.float32, device=device))
        self.register_buffer('upper', torch.tensor(config['upper_action'], dtype=torch.float32, device=device))
        self.register_buffer('mask', torch.tensor(config['transformed_mask'], dtype=torch.bool, device=device))
        signature = bytes.fromhex(_digest(config))
        # RSL RolloutStorage defaults to float32. Bytes remain exactly representable.
        self.register_buffer('contract_signature', torch.tensor(list(signature), dtype=torch.float32, device=device))

    def _radius(self, values):
        radius = torch.where(values >= 0, self.upper, -self.lower)
        return torch.where(self.mask, radius, torch.ones_like(radius))

    def forward(self, latent):
        radius = self._radius(latent)
        mapped = (2. / math.pi) * radius * torch.atan((math.pi / 2.) * latent / radius)
        result = torch.where(self.mask, mapped, latent)
        # atan(inf) is finite: explicitly propagate faults to existing plant gates.
        return torch.where(torch.isfinite(latent), result, latent)

    def inverse_and_support(self, outputs):
        inside = (~self.mask) | ((outputs > self.lower) & (outputs < self.upper))
        # Evaluate inverse only where its mathematical support holds. This is not
        # action clipping: outside support is separately returned as density zero.
        safe = torch.where(inside & torch.isfinite(outputs), outputs, torch.zeros_like(outputs))
        radius = self._radius(safe)
        latent = (2. / math.pi) * radius * torch.tan((math.pi / 2.) * safe / radius)
        return torch.where(self.mask, latent, safe), inside.all(dim=-1)

    def log_abs_det_jacobian(self, latent):
        radius = self._radius(latent)
        log_jac = -torch.log1p(((math.pi / 2.) * latent / radius).square())
        return torch.where(self.mask, log_jac, torch.zeros_like(log_jac))

    def _load_from_state_dict(self, state_dict, prefix, local_metadata, strict,
                              missing_keys, unexpected_keys, error_msgs):
        # Strict PyTorch loading otherwise overwrites buffers, silently changing
        # the transform even if the receiver was built from different ranges.
        for name in ('lower', 'upper', 'mask', 'contract_signature'):
            key = prefix + name
            if key in state_dict and not torch.equal(state_dict[key].to(getattr(self, name).device), getattr(self, name)):
                error_msgs.append(f'{key}: fixed action transform differs from checkpoint')
                return
        super()._load_from_state_dict(state_dict, prefix, local_metadata, strict,
                                      missing_keys, unexpected_keys, error_msgs)


class HomeAnchoredArctanGaussian(GaussianDistribution):
    """Same 14 latent MLP outputs, with a new, declared output distribution."""
    def __init__(self, config, previous):
        super().__init__(14, init_std=1., std_type=previous.std_type)
        # Preserve the parameter already referenced by PPO's optimizer.
        if previous.std_type == 'scalar':
            self.std_param = previous.std_param
        else:
            self.log_std_param = previous.log_std_param
        self.transform = ArctanActionTransform(config, device=next(previous.parameters()).device)

    def update(self, mlp_output):
        sigma = self.std_param if self.std_type == 'scalar' else self.log_std_param.exp()
        if (not bool(torch.isfinite(mlp_output).all()) or not bool(torch.isfinite(sigma).all())
                or not bool((sigma > 0).all())):
            raise ValueError('Nonfinite latent mean or invalid Gaussian sigma')
        super().update(mlp_output)

    def sample(self):
        output = self.transform(self._distribution.sample())
        valid = (~self.transform.mask) | ((output > self.transform.lower) & (output < self.transform.upper))
        if not bool(torch.isfinite(output).all()) or not bool(valid.all()):
            raise FloatingPointError('Bounded sampling reached a nonfinite or rounded support endpoint; no clipping is permitted')
        return output

    def deterministic_output(self, mlp_output):
        return self.transform(mlp_output)

    def as_deterministic_output_module(self):
        return copy.deepcopy(self.transform)

    @property
    def mean(self):
        # Compatibility location statistic. Params below use the latent mean.
        return self.transform(self._distribution.mean)

    @property
    def std(self):
        # RSL logger calls this action_std; report records that it is latent sigma.
        return self._distribution.stddev

    @property
    def params(self):
        mean, sigma = self._distribution.mean, self._distribution.stddev
        signature = self.transform.contract_signature.expand(*mean.shape[:-1], -1)
        return mean, sigma, signature

    def log_prob(self, outputs):
        latent, supported = self.transform.inverse_and_support(outputs)
        log_prob = (self._distribution.log_prob(latent) - self.transform.log_abs_det_jacobian(latent)).sum(dim=-1)
        log_prob = torch.where(supported & torch.isfinite(outputs).all(dim=-1), log_prob, torch.full_like(log_prob, -torch.inf))
        return torch.where(torch.isnan(outputs).any(dim=-1), torch.full_like(log_prob, torch.nan), log_prob)

    @property
    def entropy(self):
        latent = self._distribution.rsample()
        return (self._distribution.entropy() + self.transform.log_abs_det_jacobian(latent)).sum(dim=-1)

    def kl_divergence(self, old_params, new_params):
        if len(old_params) != 3 or len(new_params) != 3:
            raise ValueError('Transformed KL requires latent params and fixed-transform signatures')
        old_mean, old_std, old_signature = old_params
        new_mean, new_std, new_signature = new_params
        signature = self.transform.contract_signature
        if (not bool((old_signature == signature).all()) or not bool((new_signature == signature).all())):
            raise ValueError('Cannot compare KL across different fixed action transforms')
        old = torch.distributions.Normal(old_mean, old_std)
        new = torch.distributions.Normal(new_mean, new_std)
        return torch.distributions.kl_divergence(old, new).sum(dim=-1)


def install_bounded_action_distribution(actor, candidate):
    """Install after vanilla ONNX warm-start verification, before any PPO step.

    No MLP weights or normalization change. The resulting deterministic action
    function DOES change, so it is a new policy and must not claim old-ONNX
    equivalence. Use the same config before strict checkpoint reload.
    """
    previous = actor.distribution
    if type(previous) is not GaussianDistribution or previous.output_dim != 14:
        raise ValueError('Install exactly once over a genuine 14D Gaussian actor')
    config = distribution_config(candidate)
    before = {id(p) for p in actor.parameters()}
    actor.distribution = HomeAnchoredArctanGaussian(config, previous)
    if {id(p) for p in actor.parameters()} != before:
        raise AssertionError('Distribution installation changed optimizer parameter identities')
    return dict(config=config, config_sha256=_digest(config),
                source=dict(path=str(Path(__file__).resolve()), sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
                optimizer_parameters_preserved=True, original_onnx_function_equivalent=False,
                deployment_approved=False, plant_ranges_modified=False,
                checkpoint_load='same importer and distribution config before strict runner.load')
