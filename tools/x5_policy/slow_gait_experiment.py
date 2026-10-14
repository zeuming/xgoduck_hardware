"""Offline phase-conditioned slow-gait experiment; not a production controller.

The actor input is explicitly 63D: the existing 61D contract followed by sine
and cosine of the commanded gait phase. Plant limits and action processing are
unchanged. This is a new actor-only experiment, never a checkpoint continuation.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, fields
import math

import torch
from torch import nn

SPEC = dict(name='standing_base_slow_phase_v1', observation_size=63,
    phase_features=['sin(2*pi*phase)', 'cos(2*pi*phase)'],
    warmup_s=2., walk_s=16., recovery_s=2., cycle_s=2., swing_fraction=.30,
    command_m_s=.03, ramp_s=.5, sole_height_m=.004,
    parameter_basis='4mm target gives 1mm above the independent 3mm clearance gate; 2s cycle and 0.03m/s are declared initial slow-task choices, not identified optima',
    hardware_eligible=False, checkpoint_resume=False)


def schedule(steps, dt):
    t=steps.to(torch.float64)*dt-SPEC['warmup_s']
    fraction=torch.minimum(t/SPEC['ramp_s'], (SPEC['walk_s']-t)/SPEC['ramp_s']).clamp(0.,1.)
    command=(SPEC['command_m_s']*fraction).to(torch.float32)
    phase=(t.clamp_min(0.)/SPEC['cycle_s']).remainder(1.).to(torch.float32)
    phase=torch.where(command>0,phase,torch.zeros_like(phase))
    return command,phase


def phase_features(env):
    _,phase=schedule(env.episode_length_buf,env.step_dt)
    return torch.stack((torch.sin(2*math.pi*phase),torch.cos(2*math.pi*phase)),dim=-1)


def swing_shape(phase):
    local=torch.stack((phase,(phase-.5).remainder(1.)),dim=-1)
    u=local/SPEC['swing_fraction']
    return torch.where(u<1.,torch.sin(math.pi*u).square(),torch.zeros_like(u))


def sole_heights(env):
    """Minimum of every actual CAD foot collision-mesh vertex, not body Z."""
    if not hasattr(env,'_slow_sole_geometry'):
        import mujoco
        model=env.sim.mj_model
        items=[]
        for side in ('left','right'):
            ids=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==side+'_foot_collision']
            if len(ids)!=1:raise ValueError('Ambiguous sole geometry')
            gid=ids[0];mid=int(model.geom_dataid[gid])
            start,count=int(model.mesh_vertadr[mid]),int(model.mesh_vertnum[mid])
            vertices=torch.tensor(model.mesh_vert[start:start+count].copy(),device=env.device,dtype=torch.float32)
            items.append((gid,vertices))
        env._slow_sole_geometry=items
    heights=[]
    for gid,vertices in env._slow_sole_geometry:
        rotation=env.sim.data.geom_xmat[:,gid].reshape(-1,3,3)
        z=vertices @ rotation[:,2,:].unsqueeze(-1)
        heights.append(z.squeeze(-1).amin(-1)+env.sim.data.geom_xpos[:,gid,2])
    return torch.stack(heights,dim=-1)


def dense_gait_values(height,load,phase,weight_n,tilt_rad):
    """Shaping only. Airborne/fallen states cannot earn the gait term.

    Full-foot height, swing unloading and the OTHER foot's support must occur
    together. This continuous signal is deliberately not an acceptance gate.
    """
    shape=swing_shape(phase)
    support=load.flip(-1)/(weight_n[:,None]+1e-6)
    support_gate=support.clamp(0.,1.)
    upright=(tilt_rad<math.radians(15)).to(height.dtype)[:,None]
    normalized_load=load/(weight_n[:,None]+1e-6)
    # Partial unloading supplies gradient information before any foot is airborne.
    unload=(1.-normalized_load).clamp(0.,1.)
    clearance=(height/SPEC['sole_height_m']).clamp(0.,1.)
    desired=SPEC['sole_height_m']*shape
    tracking=torch.exp(-((height-desired)/SPEC['sole_height_m']).square())
    swing=(shape*(.4*unload+.6*clearance)*support_gate*upright).sum(-1)
    excessive=((height-SPEC['sole_height_m']).clamp_min(0.)/.01).square().sum(-1)
    return swing + .2*tracking.mean(-1)*support_gate.max(-1).values*upright[:,0]-.1*excessive


def gait_reward(env):
    command=env.command_manager.get_command('twist')[:,0]
    _,phase=schedule(env.episode_length_buf-1,env.step_dt)
    # Reward is computed before the command manager advances its next observation.
    sensor=env.scene['feet_ground_contact'].data
    load=sensor.force[:,:,2].abs()
    mass=env.sim.wp_model.body_mass
    import warp as wp
    masses=wp.to_torch(mass)
    if masses.ndim==1:weight=masses.sum().expand(env.num_envs)*9.81
    else:weight=masses.sum(-1)*9.81
    gravity=env.scene['robot'].data.projected_gravity_b
    tilt=torch.acos((-gravity[:,2]/gravity.norm(dim=-1)).clamp(-1.,1.))
    result=dense_gait_values(sole_heights(env),load,phase,weight,tilt)
    return torch.where(command>0,result,torch.zeros_like(result))


def slow_velocity_reward(env):
    command=env.command_manager.get_command('twist')
    velocity=env.scene['robot'].data.root_link_lin_vel_b
    scale=torch.where(command[:,0]>0,torch.full_like(command[:,0],.015),torch.full_like(command[:,0],math.sqrt(.1)))
    return torch.exp(-((velocity-command)**2).sum(-1)/scale.square())


def lateral_transfer_values(com_lateral,feet_lateral,phase):
    """Continuous geometric shaping toward the opposite support foot.

    Normalize by current foot separation; unlike contact force, COM position
    changes continuously while both soles are still touching. This is a learning
    objective, not a proof the COM lies in an actual contact support polygon.
    """
    separation=(feet_lateral[:,0]-feet_lateral[:,1]).abs().clamp_min(.01)
    distance=(com_lateral[:,None]-feet_lateral.flip(-1)).abs()
    proximity=(1.-distance/separation[:,None]).clamp(0.,1.)
    return (swing_shape(phase)*proximity).sum(-1)


def lateral_transfer_reward(env):
    import warp as wp
    # The actual randomized body masses, not nominal CAD mass or a root proxy.
    masses=wp.to_torch(env.sim.wp_model.body_mass)
    if masses.ndim==1:masses=masses[None].expand(env.num_envs,-1)
    com=(masses[:,:,None]*env.sim.data.xipos).sum(1)/masses.sum(-1)[:,None]
    sole_heights(env)  # Initialize geometry IDs without writing the plant.
    feet=torch.stack([env.sim.data.geom_xpos[:,gid] for gid,_ in env._slow_sole_geometry],dim=1)
    q=env.scene['robot'].data.root_link_quat_w
    yaw=torch.atan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2].square()+q[:,3].square()))
    lateral=torch.stack((-torch.sin(yaw),torch.cos(yaw),torch.zeros_like(yaw)),dim=-1)
    com_y=(com*lateral).sum(-1);feet_y=(feet*lateral[:,None]).sum(-1)
    _,phase=schedule(env.episode_length_buf-1,env.step_dt)
    result=lateral_transfer_values(com_y,feet_y,phase)
    command=env.command_manager.get_command('twist')[:,0]
    return torch.where((command>0)&~outside_observation_boundary(env),result,torch.zeros_like(result))


def outside_observation_boundary(env):
    gravity=env.scene['robot'].data.projected_gravity_b
    return -gravity[:,2]/gravity.norm(dim=-1)<math.cos(math.radians(15))


def configure(cfg,com_transfer=False):
    from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand,UniformVelocityCommandCfg
    from mjlab.managers import RewardTermCfg,TerminationTermCfg
    class SlowCommand(UniformVelocityCommand):
        def _resample(self,env_ids):
            self.time_left[env_ids]=float('inf');self._resample_command(env_ids)
            self.command_counter[env_ids]+=1
        def _resample_command(self,env_ids):
            self.vel_command_b[env_ids]=0.;self.vel_command_w[env_ids]=0.
            self.is_standing_env[env_ids]=True
            self.is_heading_env[env_ids]=False;self.is_world_env[env_ids]=False;self.is_forward_env[env_ids]=False
        def compute(self,dt):
            vx,_=schedule(self._env.episode_length_buf,self._env.step_dt)
            self.vel_command_b.zero_();self.vel_command_b[:,0]=vx
            self.vel_command_w.copy_(self.vel_command_b);self.is_standing_env.copy_(vx==0)
    @dataclass(kw_only=True)
    class SlowCommandCfg(UniformVelocityCommandCfg):
        def build(self,env):return SlowCommand(self,env)
    original=cfg.commands['twist']
    args={f.name:copy.deepcopy(getattr(original,f.name)) for f in fields(UniformVelocityCommandCfg)}
    args.update(heading_command=False,rel_heading_envs=0.,rel_world_envs=0.,rel_forward_envs=0.,
        rel_standing_envs=0.,init_velocity_prob=0.,resampling_time_range=(1e9,1e9))
    args['ranges']=UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.,SPEC['command_m_s']),lin_vel_y=(0.,0.),ang_vel_z=(0.,0.))
    cfg.commands['twist']=SlowCommandCfg(**args)
    cfg.episode_length_s=sum(SPEC[k] for k in ('warmup_s','walk_s','recovery_s'))
    # Record an explicit new objective; retain physics, randomization and guards.
    removed_rewards={k:cfg.rewards.pop(k) for k in ('air_time','foot_clearance','foot_swing_height') if k in cfg.rewards}
    cfg.rewards['track_linear_velocity']=RewardTermCfg(func=slow_velocity_reward,weight=2.)
    cfg.rewards['dense_slow_gait']=RewardTermCfg(func=gait_reward,weight=4.)
    if com_transfer:
        cfg.rewards['geometric_support_transfer']=RewardTermCfg(func=lateral_transfer_reward,weight=4.)
    cfg.terminations['learning_15deg_boundary']=TerminationTermCfg(func=outside_observation_boundary,time_out=False)
    # The fixed objective must not be overwritten mid-experiment by curricula.
    curricula=list(cfg.curriculum);cfg.curriculum.clear()
    return dict(**SPEC,geometric_transfer_reward=com_transfer,
        transfer_basis='actual randomized whole-body COM lateral distance to opposite foot, normalized by foot separation; weight4 same scale as clearance shaping' if com_transfer else None,
        removed_rewards=list(removed_rewards),removed_curricula=curricula,
        additional_offline_termination='same 15 degree acceptance boundary; all earlier original terminations retained',
        physical_changes=False,actor_contract='existing 61 values plus explicit two phase values; production 61D loader incompatible')


class PhaseNormalizer(nn.Module):
    def __init__(self,mean,denominator):
        super().__init__();self.register_buffer('mean',mean.clone());self.register_buffer('denominator',denominator.clone())
    def forward(self,x):return torch.cat(((x[:,:61]-self.mean)/self.denominator,x[:,61:]),dim=-1)
    def update(self,*args,**kwargs):pass


class StandingResidual(nn.Module):
    def __init__(self,base,mean,denominator,mask):
        super().__init__();self.base=base;self.base.requires_grad_(False)
        self.correction=nn.Sequential(nn.Linear(63,128),nn.ELU(),nn.Linear(128,128),nn.ELU(),nn.Linear(128,14)).to(mean.device)
        nn.init.zeros_(self.correction[-1].weight);nn.init.zeros_(self.correction[-1].bias)
        self.register_buffer('zero_command',(-mean[:,48:51]/denominator[:,48:51]).clone())
        self.register_buffer('command_denominator',denominator[:,48:49].clone())
        self.register_buffer('leg_mask',torch.tensor(mask,device=mean.device,dtype=mean.dtype)[None])
    def forward(self,x):
        base_input=torch.cat((x[:,:48],self.zero_command.expand(x.shape[0],-1),x[:,51:61]),dim=-1)
        gate=((x[:,48:49]-self.zero_command[:,:1])*self.command_denominator/SPEC['command_m_s']).clamp(0.,1.)
        return self.base(base_input)+gate*self.leg_mask*self.correction(x)


def install_actor(runner,candidate,seed_state):
    from bounded_action_distribution import install_bounded_action_distribution
    from train_constrained_policy import assert_state_identical
    actor=runner.alg.actor
    device=runner.device
    state={k:v.to(device) for k,v in seed_state.items()}
    mean=state['obs_normalizer.onnx_mean'];denominator=state['obs_normalizer.onnx_denominator']
    base=nn.Sequential(nn.Linear(61,512),nn.ELU(),nn.Linear(512,256),nn.ELU(),nn.Linear(256,128),nn.ELU(),nn.Linear(128,14)).to(device)
    base.load_state_dict({k.removeprefix('mlp.'):v for k,v in state.items() if k.startswith('mlp.')},strict=True)
    actor.obs_normalizer=PhaseNormalizer(mean,denominator)
    actor.mlp=StandingResidual(base,mean,denominator,[sid<30 for sid in candidate['policy_joint_ids']])
    install_bounded_action_distribution(actor,candidate)
    expected={k.removeprefix('distribution.'):v for k,v in state.items() if k.startswith('distribution.')}
    actor.distribution.load_state_dict(expected,strict=True)
    assert_state_identical(expected,actor.distribution.state_dict())
    # Exploration is frozen to the standing parent's learned scale; the new
    # experiment changes the deterministic controller/objective, not a sigma sweep.
    actor.distribution.requires_grad_(False)
    if runner.alg.optimizer.state:raise ValueError('New experiment needs fresh Adam state')
    runner.alg.optimizer.param_groups[0]['params']=list(actor.parameters())+list(runner.alg.critic.parameters())
    return dict(initialization='frozen standing actor only; new phase residual, critic, Adam, KL, environment and RNG',
        checkpoint_resume=False,base_sha_preserved=True,distribution_frozen=True)
