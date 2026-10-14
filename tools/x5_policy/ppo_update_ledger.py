"""Observe post-optimizer KL without drawing samples or changing PPO state."""
from __future__ import annotations

import torch


def distribution_on_rollout(algorithm):
    actor = algorithm.actor
    if actor.is_recurrent:
        raise ValueError('This diagnostic only supports the pinned feedforward actor')
    storage = algorithm.storage
    if storage.step != storage.num_transitions_per_env:
        raise ValueError('Expected a complete rollout before optimizer updates')
    old = tuple(p.flatten(0, 1) for p in storage.distribution_params)
    observations = storage.observations.flatten(0, 1)
    with torch.no_grad():
        latent = actor.mlp(actor.get_latent(observations))
        distribution = actor.distribution
        sigma = distribution.std_param if distribution.std_type == 'scalar' else distribution.log_std_param.exp()
        sigma = sigma.expand_as(latent)
        if not torch.isfinite(sigma).all() or not (sigma > 0).all():
            raise ValueError('Optimizer produced invalid exploration sigma')
        current = (latent, sigma)
        if len(old) == 3:
            current += (distribution.transform.contract_signature.expand(len(latent), -1),)
        kl = actor.get_kl_divergence(old, current)
        old_action = distribution.deterministic_output(old[0])
        new_action = distribution.deterministic_output(latent)
        return dict(mean_kl=float(kl.mean()), maximum_kl=float(kl.max()),
                    maximum_action_delta=float((new_action-old_action).abs().max()),
                    minimum_sigma=float(sigma.min()), maximum_sigma=float(sigma.max()),
                    observations=len(latent))


class UpdateLedger:
    def __init__(self, algorithm, max_updates=20):
        self.algorithm = algorithm
        self.max_updates = max_updates
        self.rows = []
        self.pending = None
        self.pre_handle = algorithm.optimizer.register_step_pre_hook(self.before)
        self.post_handle = algorithm.optimizer.register_step_post_hook(self.after)

    def before(self, optimizer, args, kwargs):
        if len(self.rows) < self.max_updates:
            self.pending = dict(update=len(self.rows)+1,
                                learning_rates=[group['lr'] for group in optimizer.param_groups],
                                before=distribution_on_rollout(self.algorithm))

    def after(self, optimizer, args, kwargs):
        if self.pending is not None:
            self.pending['after'] = distribution_on_rollout(self.algorithm)
            self.rows.append(self.pending)
            self.pending = None

    def close(self):
        self.pre_handle.remove()
        self.post_handle.remove()
