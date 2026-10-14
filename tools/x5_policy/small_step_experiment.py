"""New offline small-step objective; original plant and 61D inputs are retained.

This is an actor-only task transfer, not continuation of the old critic/Adam.
Rewards are shaping signals. The separate CAD/contact walking gate is decisive.
"""
import math
import torch
from mjlab.managers import RewardTermCfg
from slow_gait_experiment import sole_heights, outside_observation_boundary
import source_adaptation_experiment as source

BONUSES = ('track_angular_velocity', 'upright', 'pose', 'head_pose_tracking')
REWARD_SPEC = dict(name='positive_actor_small_step_v1', observation_size=61,
    clearance_target_m=.005, acceptance_clearance_m=.003,
    airborne_max_load_n=.001, support_min_load_n=1., qualified_span_s=.060,
    stationary_bonus_fraction=.2, progress_bonus_fraction=.8,
    height_reward_weight=3., excess_height_penalty_weight=.5,
    landing_reward_integral=.1, air_time_max_s=.65,
    rationale='5mm whole-sole target leaves2mm above independent3mm gate; forward bonuses require progress; .65s air window permits slower steps than source .3s; values are experimental, not physical measurements',
    hardware_eligible=False, checkpoint_resume=False)


def progress_gate(command, forward_velocity):
    """Zero command is unchanged; standing under forward command earns20%."""
    fraction=(forward_velocity/command.clamp_min(.001)).clamp(0.,1.)
    return torch.where(command>0, .2+.8*fraction, torch.ones_like(fraction))


def forward_bonus_rebate(env):
    # Pinned mjlab1.3 computes _step_reward in insertion order, in reward/sec.
    # This term is deliberately appended after every referenced positive term.
    manager=env.reward_manager; names=manager.active_terms
    current=names.index('forward_bonus_rebate')
    ids=[names.index(n) for n in BONUSES]
    if any(i>=current for i in ids):raise ValueError('Bonus rebate order changed')
    command=env.command_manager.get_command('twist')[:,0]
    velocity=env.scene['robot'].data.root_link_lin_vel_b[:,0]
    bonus=manager._step_reward[:,ids].clamp_min(0.).sum(-1)
    return (1.-progress_gate(command,velocity))*bonus


def height_values(height, load):
    support=(load.flip(-1)/REWARD_SPEC['support_min_load_n']).clamp(0.,1.)
    # Continuous shaping before strict contact release; not an acceptance gate.
    unload=(1.-load/.5).clamp(0.,1.)
    clearance=(height/REWARD_SPEC['clearance_target_m']).clamp(0.,1.)
    good=(clearance*unload*support).sum(-1)
    excess=((height-REWARD_SPEC['clearance_target_m']).clamp_min(0.)/.02).square().sum(-1)
    return good,excess


def small_height(env, excess=False):
    height=sole_heights(env);load=env.scene['feet_ground_contact'].data.force[:,:,2].abs()
    good,cost=height_values(height,load)
    active=(env.command_manager.get_command('twist')[:,0]>0)&~outside_observation_boundary(env)
    return torch.where(active,cost if excess else good,torch.zeros_like(good))


class AlternatingLanding:
    """Completed supported steps; no reward for bouncing one foot or hovering."""
    def __init__(self,cfg,env):
        self.dt=env.step_dt; self.device=env.device
        self.span_samples=math.ceil(.060/self.dt-1e-9)+1
        self.air=torch.zeros((env.num_envs,2),device=env.device,dtype=torch.long)
        self.land=self.air.clone();self.armed=self.air.bool();self.last=torch.full((env.num_envs,),-1,device=env.device,dtype=torch.long)
        self.last_step=torch.full_like(self.last,-1)
        self.value=torch.zeros(env.num_envs,device=env.device)
    def reset(self,env_ids=None):
        ids=slice(None) if env_ids is None else env_ids
        self.air[ids]=0;self.land[ids]=0;self.armed[ids]=False;self.last[ids]=-1;self.last_step[ids]=-1;self.value[ids]=0
    def update(self,height,load,active,episode_step):
        if torch.any(self.last_step==episode_step):raise ValueError('Landing reward evaluated twice per step')
        self.last_step.copy_(episode_step)
        good=(height>=.003)&(load<.001)&(load.flip(-1)>=1.)&active[:,None]
        self.air=torch.where(good,self.air+1,torch.zeros_like(self.air))
        self.armed|=self.air>=self.span_samples
        contact=(load>=1.)&(height<.001)&active[:,None]
        self.land=torch.where(contact,self.land+1,torch.zeros_like(self.land))
        landed=self.armed&(self.land>=self.span_samples)
        unique=landed.sum(-1)==1
        side=landed.long().argmax(-1)
        accepted=unique&(side!=self.last)&active
        self.last=torch.where(accepted,side,self.last)
        self.armed&=~landed
        self.air[~active]=0;self.land[~active]=0;self.armed[~active]=False;self.last[~active]=-1
        self.value=accepted.float()/self.dt
        return self.value
    def __call__(self,env):
        active=(env.command_manager.get_command('twist')[:,0]>0)&~outside_observation_boundary(env)
        return self.update(sole_heights(env),env.scene['feet_ground_contact'].data.force[:,:,2].abs(),active,env.episode_length_buf)


def configure(candidate, *, small_step=True, **kwargs):
    cfg,rl,spec=source.configure(candidate,**kwargs)
    spec['new_task']=dict(**REWARD_SPEC,enabled=small_step)
    if kwargs.get('training',True) and small_step:
        cfg.rewards.pop('foot_clearance');cfg.rewards.pop('foot_swing_height')
        cfg.rewards['air_time'].params['threshold_max']=.65
        cfg.rewards['small_whole_sole_height']=RewardTermCfg(func=small_height,weight=3.)
        cfg.rewards['excess_whole_sole_height']=RewardTermCfg(func=small_height,params={'excess':True},weight=-.5)
        cfg.rewards['completed_alternating_landing']=RewardTermCfg(func=AlternatingLanding,weight=.1)
        cfg.rewards['forward_bonus_rebate']=RewardTermCfg(func=forward_bonus_rebate,weight=-1.)
    return cfg,rl,spec
