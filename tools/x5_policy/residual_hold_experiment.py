"""Experimental two-rate policy. Original constrained plant still runs at 50Hz.

A new 5Hz Gaussian policy selects TEN leg residuals, held for ten control
periods. The verified standing network continues to read feedback every period.
This is a new semi-MDP training/ONNX contract, not a production configuration.
"""
from __future__ import annotations
import copy
import math
import torch
from torch import nn

HOLD_STEPS=10
MICRO_GAMMA=.99
MICRO_LAMBDA=.95
INITIAL_SIGMA_RAD=math.radians(4.9)


def spec(candidate):
    return dict(name='feedback_base_50hz_residual_5hz_v1',control_hz=50,decision_hz=5,
        hold_steps=HOLD_STEPS,actor_observations=73,actor_outputs=10,
        actor_fields='original61, phase sin/cos2, previous held leg residual10',
        gamma_micro=MICRO_GAMMA,gamma_decision=MICRO_GAMMA**HOLD_STEPS,
        lambda_micro=MICRO_LAMBDA,lambda_decision=MICRO_LAMBDA**HOLD_STEPS,
        reward='sum_{k=0..9} gamma_micro**k * micro_reward through FIRST terminal inclusive',
        horizon='finite20s command task; no timeout bootstrap',
        reset_bridge='after first terminal residual is zero and rewards ignored until next decision; original env autoreset and zero-command standing feedback continue',
        initial_latent_sigma=INITIAL_SIGMA_RAD/candidate['action_scale'],
        sigma_basis='prior within-range kinematic3mm endpoint needed up to4.9deg; initial experiment scale, not a safety margin',
        lower_actor='frozen standing actor with command inputs zeroed, recomputed every20ms',
        action_processing='same arctan coordinates then original plant head hold/EMA/slew/range gates',
        original_action_and_physics_limits_changed=False,checkpoint_resume=False,
        hardware_eligible=False,production_loader_compatible=False)


class FrozenStandingController(nn.Module):
    def __init__(self,candidate,seed_state,device='cpu'):
        super().__init__()
        from bounded_action_distribution import ArctanActionTransform,distribution_config
        self.base=nn.Sequential(nn.Linear(61,512),nn.ELU(),nn.Linear(512,256),nn.ELU(),nn.Linear(256,128),nn.ELU(),nn.Linear(128,14)).to(device)
        self.base.load_state_dict({k.removeprefix('mlp.'):v.to(device) for k,v in seed_state.items() if k.startswith('mlp.')},strict=True)
        self.register_buffer('mean',seed_state['obs_normalizer.onnx_mean'].to(device).clone())
        self.register_buffer('denominator',seed_state['obs_normalizer.onnx_denominator'].to(device).clone())
        self.transform=ArctanActionTransform(distribution_config(candidate),device=device)
        self.transform.load_state_dict({k.removeprefix('distribution.transform.'):v.to(device) for k,v in seed_state.items() if k.startswith('distribution.transform.')},strict=True)
        indices=[i for i,sid in enumerate(candidate['policy_joint_ids']) if sid<30]
        if len(indices)!=10:raise ValueError('Expected10leg residuals')
        embedding=torch.zeros(10,14,device=device);embedding[torch.arange(10,device=device),indices]=1.
        self.register_buffer('embedding',embedding)
        self.requires_grad_(False)
    def latent_standing(self,obs):
        raw=torch.cat((obs[:,:48],torch.zeros_like(obs[:,48:51]),obs[:,51:]),dim=-1)
        return self.base((raw-self.mean)/self.denominator)
    def forward(self,obs,residual):
        gate=(obs[:,48:49]/.03).clamp(0.,1.)
        return self.transform(self.latent_standing(obs)+gate*(residual@self.embedding))


class ResidualNormalizer(nn.Module):
    def __init__(self,lower,sigma):
        super().__init__()
        self.register_buffer('mean',lower.mean.clone());self.register_buffer('denominator',lower.denominator.clone())
        self.register_buffer('residual_scale',torch.tensor(sigma,device=lower.mean.device))
    def forward(self,x):
        return torch.cat(((x[:,:61]-self.mean)/self.denominator,x[:,61:63],x[:,63:]/self.residual_scale),dim=-1)
    def update(self,*args,**kwargs):pass


class HybridExport(nn.Module):
    """Explicit stateful-controller I/O; no implicit Python clock in ONNX."""
    def __init__(self,lower,actor):
        super().__init__();self.lower=copy.deepcopy(lower)
        self.upper=nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp))
    def forward(self,obs61,phase2,held10,update1):
        proposed=self.upper(torch.cat((obs61,phase2,held10),dim=-1))
        held=torch.where(update1>.5,proposed,held10)
        return self.lower(obs61,held),held


def discounted_prefix(rewards,dones,gamma=MICRO_GAMMA):
    """Testable mathematical macro-return; include terminal reward exactly once."""
    alive=torch.ones_like(dones[0],dtype=torch.bool);total=torch.zeros_like(rewards[0]);ended=~alive
    for k,(reward,done) in enumerate(zip(rewards,dones)):
        total=total+(gamma**k)*reward*alive
        ended=ended|done.bool();alive=alive&~done.bool()
    return total,ended


def wrapper_class():
    from mjlab.rl import RslRlVecEnvWrapper
    from slow_gait_experiment import phase_features,sole_heights,outside_observation_boundary
    class ResidualHoldWrapper(RslRlVecEnvWrapper):
        def __init__(self,env,lower):
            super().__init__(env,clip_actions=None)
            self.num_actions=10;self.max_episode_length=math.ceil(env.max_episode_length/HOLD_STEPS)
            self.lower=lower;self.held=torch.zeros(env.num_envs,10,device=env.device)
            self.micro_obs=env.obs_buf
            self.micro_frames=0;self.active_reward_transitions=0;self.decision_frames=0
            self.counters={};self.rewards=[];self.export_samples=[]
            self.coverage=torch.zeros(4,device=self.device,dtype=torch.long)
            self.reward_terms=torch.zeros(len(env.reward_manager.active_terms),device=self.device)
        def augment(self,obs):
            obs=obs.clone(False);phase=phase_features(self.env)
            actor=torch.cat((obs['actor'],phase,self.held),dim=-1)
            term=self.env.action_manager.get_term('joint_pos')
            remaining=(1.-self.env.episode_length_buf.to(torch.float32)/self.env.max_episode_length).clamp(0.,1.)[:,None]
            critic=torch.cat((obs['critic'],phase,self.held,term._goal-term._home,term._ema-term._home,remaining),dim=-1)
            obs['actor']=actor;obs['critic']=critic
            return obs
        def get_observations(self):
            obs=super().get_observations();self.micro_obs=obs
            return self.augment(obs)
        def reset(self):
            self.held.zero_();obs,extra=super().reset();self.micro_obs=obs
            return self.augment(obs),extra
        def step(self,residual):
            if residual.shape!=(self.num_envs,10) or not torch.isfinite(residual).all():raise ValueError('Invalid10Dresidual')
            self.held=residual.to(self.device).clone()
            alive=torch.ones(self.num_envs,device=self.device,dtype=torch.bool)
            ended=torch.zeros_like(alive);total=torch.zeros(self.num_envs,device=self.device)
            for k in range(HOLD_STEPS):
                active_before=(self.micro_obs['actor'][:,48]>0)&alive
                joint_action=self.lower(self.micro_obs['actor'],self.held)
                obs,reward,done,extra=super().step(joint_action)
                active=active_before&~done.bool()
                # Diagnostic samples only; independent acceptance uses raw CAD
                # contacts, continuous durations and an unreset deterministic run.
                height=sole_heights(self.env)
                load=self.env.scene['feet_ground_contact']._extract_sensor_data().force[:,:,2].abs()
                qualified=(height>=.003)&(load<.001)&(load.flip(-1)>=1.)&active[:,None]&~outside_observation_boundary(self.env)[:,None]
                self.coverage+=torch.cat((active.sum().reshape(1),qualified.sum(0),done.bool().sum().reshape(1)))
                self.reward_terms+=(self.env.reward_manager._step_reward*active_before[:,None]).sum(0)
                total+=(MICRO_GAMMA**k)*reward*alive
                self.active_reward_transitions+=int(alive.sum())
                ended|=done.bool();alive&=~done.bool()
                self.held=torch.where(done[:,None].bool(),torch.zeros_like(self.held),self.held)
                self.micro_obs=obs;self.micro_frames+=1
                for name in self.env.termination_manager.active_terms:
                    value=self.env.termination_manager.get_term(name).sum().detach()
                    self.counters[name]=self.counters.get(name,0)+value
                if len(self.export_samples)<128:
                    self.export_samples.append((obs['actor'][:8].detach().cpu(),phase_features(self.env)[:8].detach().cpu(),self.held[:8].detach().cpu()))
            if not self.env.cfg.is_finite_horizon:raise ValueError('Macro task must explicitly use finite horizon')
            extra=dict(extra);extra.pop('time_outs',None)
            self.decision_frames+=1;self.rewards.append(total.mean().detach())
            return self.augment(obs),total,ended.to(torch.long),extra
    return ResidualHoldWrapper


def install_upper(runner,lower,candidate):
    """Fresh 10D actor/critic/Adam. Frozen sigma isolates temporal coherence."""
    actor=runner.alg.actor
    actor.obs_normalizer=ResidualNormalizer(lower,INITIAL_SIGMA_RAD/candidate['action_scale'])
    nn.init.zeros_(actor.mlp[-1].weight);nn.init.zeros_(actor.mlp[-1].bias)
    actor.distribution.requires_grad_(False)
    if runner.alg.optimizer.state:raise ValueError('Requires a fresh optimizer')
    runner.alg.optimizer.param_groups[0]['params']=list(actor.parameters())+list(runner.alg.critic.parameters())
    return dict(fresh_upper_actor_critic_adam_kl=True,frozen_lower=True,frozen_sigma=True,
        old_checkpoint_resume=False)
