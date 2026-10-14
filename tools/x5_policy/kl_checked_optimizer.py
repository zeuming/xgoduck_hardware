"""Explicit PPO variant: admit Adam actor steps only after full-rollout KL.

This changes the learning algorithm, not robot guards or the simulation plant.
Each trial restores parameters AND Adam moments before halving only the actor
learning rate. If the actor trial cannot be admitted, only the critic updates.
The KL bound is the existing configured desired_kl; it is not a hardware limit.
"""
from __future__ import annotations

import copy
import math
import torch


class KlCheckedOptimizer:
    def __init__(self, optimizer, actor_parameters, critic_parameters, measure,
                 maximum_kl, max_trials=16):
        if not math.isfinite(maximum_kl) or maximum_kl <= 0 or max_trials < 1:
            raise ValueError('Need positive finite KL bound and trial count')
        if optimizer.state or len(optimizer.param_groups) != 1:
            raise ValueError('Install on the fresh, single-group optimizer before checkpoint load')
        self.optimizer = optimizer
        self.actor_parameters = list(actor_parameters)
        self.critic_parameters = list(critic_parameters)
        self.parameters = self.actor_parameters+self.critic_parameters
        ids = [id(p) for p in self.parameters]
        if len(set(ids)) != len(ids) or set(ids) != {id(p) for p in optimizer.param_groups[0]['params']}:
            raise ValueError('Actor and critic must partition the existing optimizer parameters')
        initial = dict(optimizer.param_groups[0])
        optimizer.param_groups.clear()
        optimizer.add_param_group(dict(initial,params=self.actor_parameters,training_role='actor'))
        optimizer.add_param_group(dict(initial,params=self.critic_parameters,training_role='critic'))
        self.measure = measure
        self.maximum_kl = maximum_kl
        self.max_trials = max_trials
        self.original_step = optimizer.step
        self.committed_steps = self.actor_steps = self.critic_only_steps = self.trial_steps = 0
        self.actor_numerical_noops = self.rollouts = 0
        self.nominal_actor_lr = self.preferred_actor_lr = initial['lr']
        self.rollout_closed = False
        self.first_step_in_rollout = True
        self.maximum_committed_kl = 0.
        self.records = []
        self.on_commit = None
        optimizer.step = self.step

    def begin_rollout(self):
        # Recover the first useful admitted rate, not the vanishing rate of a
        # minibatch already on the previous rollout's KL boundary. Every new
        # trial is still checked and can be rejected. No actor weight resets.
        growth = 1. if self.rollouts == 0 else 1.5
        self.optimizer.param_groups[0]['lr'] = min(self.nominal_actor_lr,self.preferred_actor_lr*growth)
        self.rollouts += 1
        self.rollout_closed = False
        self.first_step_in_rollout = True

    def snapshot(self):
        return [p.detach().clone() for p in self.parameters], copy.deepcopy(self.optimizer.state_dict())

    def restore(self, snapshot):
        parameters, state = snapshot
        with torch.no_grad():
            for parameter, saved in zip(self.parameters, parameters):
                parameter.copy_(saved)
        self.optimizer.load_state_dict(state)

    def step(self, closure=None):
        if closure is not None:
            raise ValueError('The pinned PPO does not use optimizer closures')
        before = self.measure()
        if before['mean_kl'] > self.maximum_kl or not math.isfinite(before['mean_kl']):
            raise ValueError('The pre-step actor already violates the admitted rollout KL')
        saved = self.snapshot()
        actor_lr = self.optimizer.param_groups[0]['lr']
        trials = []
        accepted = False
        for attempt in range(0 if self.rollout_closed else self.max_trials):
            if attempt:
                self.restore(saved)
            self.optimizer.param_groups[0]['lr'] = actor_lr
            self.original_step()
            self.trial_steps += 1
            try:
                after = self.measure()
            except (ValueError,FloatingPointError) as error:
                after = dict(mean_kl=None,error=str(error))
            kl = after.get('mean_kl')
            accepted = kl is not None and math.isfinite(kl) and kl <= self.maximum_kl
            trials.append(dict(actor_lr=actor_lr,after=after,accepted=accepted))
            if accepted:
                self.actor_steps += 1
                changed = any(not torch.equal(p.detach(),value) for p,value in
                              zip(self.actor_parameters,saved[0][:len(self.actor_parameters)]))
                if not changed:
                    self.actor_numerical_noops += 1
                    self.rollout_closed = True
                if self.first_step_in_rollout:
                    self.preferred_actor_lr = actor_lr
                break
            actor_lr *= .5
        if not accepted:
            # Retain critic learning without changing actor parameters/moments.
            self.restore(saved)
            self.rollout_closed = True
            gradients = [p.grad for p in self.actor_parameters]
            try:
                for p in self.actor_parameters:p.grad=None
                self.original_step()
                self.trial_steps += 1
            finally:
                for p,g in zip(self.actor_parameters,gradients):p.grad=g
            after = self.measure()
            if after['mean_kl'] != before['mean_kl']:
                raise ValueError('Critic-only fallback changed actor KL')
            self.critic_only_steps += 1
        self.first_step_in_rollout = False
        self.committed_steps += 1
        self.maximum_committed_kl = max(self.maximum_committed_kl,after['mean_kl'])
        if len(self.records) < 100:
            self.records.append(dict(step=self.committed_steps,before=before,trials=trials,
                                     actor_updated=accepted,committed=after))
        if self.on_commit is not None:self.on_commit(self.optimizer)

    def summary(self):
        return dict(kind='post_step_full_rollout_kl_backtracking_v2',
                    maximum_kl=self.maximum_kl,maximum_trials_per_actor_step=self.max_trials,
                    committed_optimizer_steps=self.committed_steps,actor_steps=self.actor_steps,
                    actor_numerical_noops=self.actor_numerical_noops,
                    effective_actor_steps=self.actor_steps-self.actor_numerical_noops,
                    rollouts=self.rollouts,preferred_actor_lr=self.preferred_actor_lr,
                    critic_only_steps=self.critic_only_steps,trial_optimizer_calls=self.trial_steps,
                    maximum_committed_kl=self.maximum_committed_kl,first_100_steps=self.records,
                    rollback='all parameters and complete Adam state before each retry',
                    learning_rate_recovery='each rollout starts at up to 1.5 times the first admitted rate of the previous rollout; all attempts remain KL-checked',
                    plant_or_hardware_limits_changed=False)

    def continuation_state(self):
        """State needed at a COMPLETED rollout boundary, before begin_rollout.

        Adam tensors belong to optimizer.state_dict(). No environment/RNG or
        partially consumed rollout is represented, so this is not exact replay.
        """
        return dict(schema_version=1, boundary='before_next_begin_rollout',
            maximum_kl=self.maximum_kl, max_trials=self.max_trials,
            nominal_actor_lr=self.nominal_actor_lr,
            **{name: copy.deepcopy(getattr(self,name)) for name in
               ('preferred_actor_lr','rollouts','committed_steps','actor_steps',
                'critic_only_steps','trial_steps','actor_numerical_noops',
                'maximum_committed_kl','records')})

    def restore_continuation_state(self, state):
        expected=set(self.continuation_state())
        if set(state)!=expected or state['schema_version']!=1 or state['boundary']!='before_next_begin_rollout':
            raise ValueError('Unsupported KL continuation state')
        for name in ('maximum_kl','max_trials','nominal_actor_lr'):
            if state[name]!=getattr(self,name):raise ValueError('KL continuation configuration differs: '+name)
        counts=('rollouts','committed_steps','actor_steps','critic_only_steps','trial_steps','actor_numerical_noops')
        if any(type(state[k]) is not int or state[k]<0 for k in counts):
            raise ValueError('Invalid KL continuation counters')
        if (state['actor_steps']+state['critic_only_steps']!=state['committed_steps']
                or state['actor_numerical_noops']>state['actor_steps']
                or state['trial_steps']<state['committed_steps']):
            raise ValueError('Inconsistent KL continuation counters')
        if (not 0<state['preferred_actor_lr']<=self.nominal_actor_lr
                or not math.isfinite(state['maximum_committed_kl'])
                or not 0<=state['maximum_committed_kl']<=self.maximum_kl
                or not isinstance(state['records'],list) or len(state['records'])>100):
            raise ValueError('Invalid KL continuation rates or records')
        for name in expected-{'schema_version','boundary','maximum_kl','max_trials','nominal_actor_lr'}:
            setattr(self,name,copy.deepcopy(state[name]))
        # The first update must explicitly begin a fresh rollout. Closed actor
        # state cannot accidentally consume a half-restored previous rollout.
        self.rollout_closed=True
        self.first_step_in_rollout=False
