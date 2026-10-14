"""Separate source-domain curriculum, never a production or calibrated candidate.

The positive source walking policy is adapted towards the immutable saved range
and target-slew contract. Intermediate SOURCE domains may use an explicitly
reported ankle training boundary above the saved boundary and a higher/no slew
limit. Only a final evaluation at the original candidate can be accepted. No
candidate/calibration/production files are edited. Original CAD/BAM and15deg stay.
"""
import copy, math
from dataclasses import dataclass, fields
import torch
from mjlab.envs.mdp.actions import JointPositionAction,JointPositionActionCfg
from mjlab.managers import EventTermCfg,TerminationTermCfg,RewardTermCfg
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand,UniformVelocityCommandCfg
from constrained_training_candidate import HEAD_INDICES,_reset_candidate_joints,_set_source_gain_vector

@dataclass(kw_only=True)
class ActionCfg(JointPositionActionCfg):
    candidate:dict
    source_ankle_upper_deg:float
    source_slew_ticks_s:float
    def build(self,env):return Action(self,env)

class Action(JointPositionAction):
    def __init__(self,cfg,env):
        super().__init__(cfg,env);c=cfg.candidate
        tensor=lambda v:torch.tensor(v,device=self.device,dtype=torch.float32)
        self.home=tensor(c['home_rad']);self.saved_low=tensor(c['lower_rad']);self.saved_high=tensor(c['upper_rad'])
        self.low=self.saved_low.clone();self.high=self.saved_high.clone();self.high[4]=math.radians(cfg.source_ankle_upper_deg)
        fraction=(50-cfg.source_ankle_upper_deg)/(50-math.degrees(c['upper_rad'][4]))
        self.low[1]=(1-fraction)*math.radians(-25.)+fraction*self.saved_low[1]
        self.high[10]=(1-fraction)*math.radians(25.)+fraction*self.saved_high[10]
        self.held=self.home.clone();self.held[list(HEAD_INDICES)]=tensor(c['held_head_rad'])
        self.ema=self.held.repeat(self.num_envs,1);self.goal=self.ema.clone();self.requested=self.goal.clone()
        self.failed=torch.zeros(self.num_envs,device=self.device,dtype=torch.bool)
        self.delta=cfg.source_slew_ticks_s*2*math.pi/c['action']['ticks_per_turn']*env.step_dt
    def process_actions(self,actions):
        self._raw_actions[:]=actions
        requested=self.home+self.cfg.candidate['action_scale']*actions
        requested[:,list(HEAD_INDICES)]=self.held[list(HEAD_INDICES)]
        self.requested=requested
        self.failed|=(~torch.isfinite(requested)).any(-1)|((requested<self.low)|(requested>self.high)).any(-1)
        a=self.cfg.candidate['action']['alpha'];ema=a*self.ema+(1-a)*requested
        ema[:,list(HEAD_INDICES)]=self.held[list(HEAD_INDICES)]
        goal=self.goal+torch.clamp(ema-self.goal,-self.delta,self.delta) if self.delta>0 else ema
        self.ema=torch.where(self.failed[:,None],self.ema,ema);self.goal=torch.where(self.failed[:,None],self.goal,goal)
        self._processed_actions=self.goal
    def apply_actions(self):
        actual=self._entity.data.joint_pos[:,self.target_ids]
        self.failed|=(~torch.isfinite(actual)).any(-1)|((actual<self.low)|(actual>self.high)).any(-1)
        if torch.any(self._entity.data.encoder_bias[:,self.target_ids]!=0):raise ValueError('Unexpected encoder bias')
        super().apply_actions()
    def reset(self,env_ids=None):
        super().reset(env_ids);ids=slice(None) if env_ids is None else env_ids
        self.ema[ids]=self.held;self.goal[ids]=self.held;self._processed_actions[ids]=self.held;self.failed[ids]=False

def bad_range(env):
    term=env.action_manager.get_term('joint_pos');q=term._entity.data.joint_pos[:,term.target_ids]
    return term.failed|((q<term.low)|(q>term.high)).any(-1)

def target_range_cost(env):
    term=env.action_manager.get_term('joint_pos')
    # An explicit 5deg shaping scale. Penalize toward the FINAL saved range,
    # without disguising the intermediate source-domain boundary as compliant.
    excess=(term.requested-term.saved_high).clamp_min(0)+(term.saved_low-term.requested).clamp_min(0)
    return (excess/math.radians(5)).square().sum(-1)

def velocity_reward(env):
    command=env.command_manager.get_command('twist');v=env.scene['robot'].data.root_link_lin_vel_b
    std=torch.where(command[:,0]>0,(.5*command[:,0]).clamp_min(.025),torch.full_like(command[:,0],math.sqrt(.1)))
    return torch.exp(-((v-command).square()).sum(-1)/std.square())

def schedule(steps,dt,spec):
    t=steps.to(torch.float64)*dt-spec['warmup_s']
    return (spec['speed_m_s']*torch.minimum(t/.5,(spec['walk_s']-t)/.5).clamp(0,1)).to(torch.float32)

class Command(UniformVelocityCommand):
    def _resample(self,env_ids):
        self.time_left[env_ids]=float('inf');self.vel_command_b[env_ids]=0;self.vel_command_w[env_ids]=0
        self.is_standing_env[env_ids]=True;self.is_heading_env[env_ids]=False;self.is_world_env[env_ids]=False;self.is_forward_env[env_ids]=False
        self.command_counter[env_ids]+=1
    def compute(self,dt):
        vx=schedule(self._env.episode_length_buf,self._env.step_dt,self.cfg.spec)
        self.vel_command_b.zero_();self.vel_command_b[:,0]=vx;self.vel_command_w.copy_(self.vel_command_b);self.is_standing_env[:]=vx==0
@dataclass(kw_only=True)
class CommandCfg(UniformVelocityCommandCfg):
    spec:dict
    def build(self,env):return Command(self,env)

def configure(c,*,ankle_deg,slew,speed,warmup=2.,walk=8.,recovery=2.,penalty=1.,training=True):
    from mjlab_microduck.tasks import _xgoduck_velocity_cfg,XgoduckRlCfg
    from slow_gait_experiment import outside_observation_boundary
    from simulation_evidence import audit_interval_push
    saved=math.degrees(c['upper_rad'][4])
    if not saved-1e-9<=ankle_deg<=50 or not (slew==0 or slew>=c['action']['goal_slew_ticks_s']):raise ValueError('Invalid declared source curriculum domain')
    cfg=copy.deepcopy(_xgoduck_velocity_cfg(play=False));rl=copy.deepcopy(XgoduckRlCfg)
    robot=cfg.scene.entities['robot'];robot.init_state.joint_pos=dict(zip(c['joint_names'],c['home_rad']))
    robot.init_state.pos=(*robot.init_state.pos[:2],c['reset_placement']['root_z_m']);cfg.events['reset_base'].params['pose_range']['z']=(0.,0.)
    cfg.events['source_gains']=EventTermCfg(func=_set_source_gain_vector,mode='startup',params={'candidate':c})
    for name,event in list(cfg.events.items()):
        if getattr(event.func,'__name__','')=='reset_joints_by_offset':cfg.events[name]=EventTermCfg(func=_reset_candidate_joints,mode='reset',params={'candidate':c})
    cfg.events.pop('encoder_bias',None)
    for group in ('actor','critic'):cfg.observations[group].terms['joint_pos'].params['biased']=False
    cfg.commands['head_pose'].ranges=tuple((float(v),float(v)) for v in c['head_command_rad']);cfg.commands['body_pose'].ranges=((0.,0.),)*6
    old=cfg.actions['joint_pos'];cfg.actions['joint_pos']=ActionCfg(**{f.name:copy.deepcopy(getattr(old,f.name)) for f in fields(JointPositionActionCfg)},candidate=c,source_ankle_upper_deg=ankle_deg,source_slew_ticks_s=slew)
    cfg.actions['joint_pos'].scale=c['action_scale']
    spec=dict(kind='source_domain_curriculum_not_calibrated_candidate',ankle_upper_deg=ankle_deg,slew_ticks_s=slew,speed_m_s=speed,
        warmup_s=warmup,walk_s=walk,recovery_s=recovery,range_penalty_weight=penalty,
        source_roll_bound_basis='successful source requested leftroll minimum-22.9435deg/rightroll maximum22.34395deg; round beyond2deg margin to25deg; linearly return to saved bounds as ankle50deg returns to34.10989deg',
        final_saved_ankle_upper_deg=saved,final_slew_ticks_s=c['action']['goal_slew_ticks_s'],
        fully_constrained_domain=abs(ankle_deg-saved)<1e-9 and slew==c['action']['goal_slew_ticks_s'],hardware_eligible=False)
    old=cfg.commands['twist'];kw={f.name:copy.deepcopy(getattr(old,f.name)) for f in fields(UniformVelocityCommandCfg)}
    kw.update(heading_command=False,rel_heading_envs=0.,rel_world_envs=0.,rel_forward_envs=0.,rel_standing_envs=0.,init_velocity_prob=0.)
    kw['ranges']=UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.,speed),lin_vel_y=(0.,0.),ang_vel_z=(0.,0.))
    cfg.commands['twist']=CommandCfg(**kw,spec=spec)
    cfg.curriculum.clear();cfg.episode_length_s=warmup+walk+recovery;cfg.is_finite_horizon=True
    cfg.terminations['declared_source_range']=TerminationTermCfg(func=bad_range)
    cfg.terminations['observation_15deg']=TerminationTermCfg(func=outside_observation_boundary)
    if training:
        cfg.rewards['target_saved_range_cost']=RewardTermCfg(func=target_range_cost,weight=-penalty)
        cfg.rewards['track_linear_velocity']=RewardTermCfg(func=velocity_reward,weight=2.)
    ledger=[];spec['push']=audit_interval_push(cfg,ledger,scale=0.)
    rl.upload_model=False;rl.algorithm.symmetry_cfg=None;rl.logger='tensorboard'
    return cfg,rl,spec
