"""Passive, pre-autoreset command-phase coverage for the pinned training task.

Uses the original cached reward rates and original sensor/termination results.
No extra reward evaluation, RNG draws, physics forwards or action writes.
Contact absence alone is NOT evidence of a qualified step or foot clearance.
"""
import torch


class PhaseTrainingLedger:
    def __init__(self, env):
        self.env = env
        if abs(env.step_dt - .02) > 1e-12:
            raise ValueError('Ledger requires the pinned 20ms stand-button profile')
        self.names = list(env.reward_manager.active_terms)
        if 'air_time' not in self.names:
            raise ValueError('Original air-time reward missing')
        self.crossed = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.rows = []
        self.original_compute = env.reward_manager.compute
        env.reward_manager.compute = self.compute

    def compute(self, dt):
        rewards = self.original_compute(dt)
        env = self.env
        # The manager increments age before computing the transition reward.
        # This transition was generated with the command at age-1.
        age = env.episode_length_buf - 1
        phase = torch.where(age < 250, 0, torch.where(age < 350, 1, 2))
        gravity = env.scene['robot'].data.projected_gravity_b
        tilt = torch.rad2deg(torch.acos((-gravity[:, 2] / gravity.norm(dim=1)).clamp(-1, 1)))
        crossing = (tilt >= 15) & ~self.crossed
        self.crossed |= tilt >= 15
        eligible = ~self.crossed
        # RewardManager._step_reward is the original weighted rate, before dt.
        rates = env.reward_manager._step_reward
        scale = dt if env.cfg.scale_rewards_by_dt else 1.
        torch.testing.assert_close(rates.sum(dim=1) * scale, rewards, atol=2e-6, rtol=1e-6)
        found = env.scene['feet_ground_contact'].data.found
        if tuple(found.shape) != (env.num_envs, 2):
            raise ValueError('Unexpected original two-foot contact sensor shape')
        reasons = {name: env.termination_manager.get_term(name) for name in env.termination_manager.active_terms}
        entry = {}
        for index, name in enumerate(('zero', 'forward', 'recovery')):
            mask = phase == index
            count = int(mask.sum())
            if not count:
                continue
            before = mask & eligible
            entry[name] = dict(frames=count, frames_before_first_15deg=int(before.sum()),
                first_15deg_crossings=int((mask & crossing).sum()),
                reward_rate_sums=dict(zip(self.names, rates[mask].sum(dim=0).tolist())),
                before_15deg_reward_rate_sums=dict(zip(self.names, rates[before].sum(dim=0).tolist())),
                air_time_positive_frames=int((mask & (rates[:, self.names.index('air_time')] > 0)).sum()),
                before_15deg_air_time_positive_frames=int((before & (rates[:, self.names.index('air_time')] > 0)).sum()),
                before_15deg_no_contact_frames={side: int((before & (found[:, j] == 0)).sum())
                    for j, side in enumerate(('left', 'right'))},
                termination_counts={key: int((mask & value).sum()) for key, value in reasons.items()})
        self.rows.append(entry)
        # Clear only completed environments after recording their final data.
        self.crossed[env.reset_buf] = False
        return rewards

    def summary(self):
        result = {}
        for name in ('zero', 'forward', 'recovery'):
            rows = [r[name] for r in self.rows if name in r]
            if not rows:
                continue
            def add(values):
                if isinstance(values[0], dict):
                    return {key: add([v[key] for v in values]) for key in values[0]}
                return sum(values)
            result[name] = add(rows)
            result[name]['mean_reward_rates'] = {k: v / result[name]['frames']
                for k, v in result[name]['reward_rate_sums'].items()}
        return dict(phases=result, vector_steps=len(self.rows),
            sample_time='original reward computation before automatic reset; derived fields may lag one physics substep as in original training',
            qualification=False, scope='coverage of sampled experiences, not gait validation',
            boundary='existing 15deg observation boundary only; no termination limit changed')

    def close(self):
        self.env.reward_manager.compute = self.original_compute
