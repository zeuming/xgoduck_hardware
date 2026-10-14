#!/usr/bin/env python3
"""Offline upstream-policy reference, never a hardware-eligible candidate.

Use the source XgoDuck BAM task without the local fixed-head/EMA/600 adapter.
Align HOME/action scale to the supplied ONNX metadata and correct the already
identified absolute-vs-offset reset Z mismatch. The task is a scripted forward
command with zero head/body commands and original startup randomization. This
is a declared source comparison, NOT proof this ONNX was trained by this exact
source revision. Original deployed calibration and safeguards are untouched.
"""
import argparse,copy,hashlib,importlib.util,inspect,json,random,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def initial_fingerprint(env):
 """Passive physical and random-state identity; expected gain changes stay visible."""
 import torch,warp as wp,dataclasses
 def digest(value):
  if isinstance(value,torch.Tensor):value=value.detach().cpu().numpy()
  elif isinstance(value,wp.array):value=value.numpy()
  arr=np.asarray(value)
  return dict(shape=list(arr.shape),dtype=str(arr.dtype),sha256=hashlib.sha256(arr.tobytes()).hexdigest())
 result={}
 # Hash every exposed tensor, rather than checking nominal mass alone.
 for prefix,obj in [('model',env.sim.model),('data',env.sim.data),
                    ('motor',env.scene['robot'].actuators[0])]:
  for key in sorted(dir(obj)):
   if key.startswith('__'):continue
   try:value=getattr(obj,key)
   except (AttributeError,RuntimeError):continue
   if isinstance(value,torch.Tensor):result[prefix+'.'+key]=digest(value)
 result['torch_rng']=digest(torch.get_rng_state())
 for field in dataclasses.fields(env.sim.wp_model):
  value=getattr(env.sim.wp_model,field.name)
  if isinstance(value,wp.array):result['warp_model.'+field.name]=digest(value)
 # Delay/BAM histories are distinct from the joint-state and physical model.
 motor=env.scene['robot'].actuators[0]
 for prefix,obj in [('delay',motor._delay_buffer),('bam',motor._bam_model),
                    ('bam_actuator',motor._bam_model.actuator),('robot_data',env.scene['robot'].data)]:
  for key,value in vars(obj).items():
   if isinstance(value,(torch.Tensor,wp.array,np.ndarray)):result[prefix+'.'+key]=digest(value)
 return result

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--speed',type=float,default=.3)
 p.add_argument('--factors',default='',help='comma-separated: gains,ema,slew,head,bias,reset,ranges,home,roll,targetclip,anklesoft')
 p.add_argument('--slew-ticks-s',type=float,default=600.,help='Offline counterfactual only; never changes firmware')
 p.add_argument('--left-ankle-margin-ticks',type=int,default=3,help='Offline targetclip upper-bound inset; not a calibration update')
 p.add_argument('--ankle-map-stage',choices=('post-ema','pre-ema'),default='post-ema',help='Paired order-of-operations hypothesis; same retargeting function')
 p.add_argument('--quantize-goals',action='store_true',help='Apply the actual validated integer encoder goals to the simulation')
 p.add_argument('--official-runtime',action='store_true',help='Declared KP6 and action-space EMA profile; no experimental transforms')
 p.add_argument('--head-range-projection',action='store_true',help='Offline hypothesis: project only head goals into saved ranges; does not edit limits')
 p.add_argument('--coordinate-knee',action='store_true',help='Offline CAD-derived ankle/knee pitch coordination; requires anklesoft')
 p.add_argument('--saved-calibration',type=Path,help='Offline range-only snapshot; rejects changed zero, direction or IMU')
 p.add_argument('--installed-action-candidate',type=Path,help='Offline actual deployed output path; requires explicitly declared current calibration')
 p.add_argument('--installed-action-calibration',type=Path,help='Complete current coordinate map for output-path comparison; physical zero alignment remains an assumption')
 p.add_argument('--installed-feedback-path',action='store_true',help='Use actual deployed feedback conversion/filter with CAD IMU sensor values, rather than ideal actor observations')
 p.add_argument('--control-hz',type=int,choices=(40,50),default=50,help='Offline cadence counterfactual; no runtime timing change')
 p.add_argument('--sensor-lag-steps',type=int,choices=(0,1),default=0,help='Offline async pipeline hypothesis: delay sensor observation, preserve current raw action history and commands')
 p.add_argument('--lag-prediction',action='store_true',help='Explicit actor-only kinematic timestamp extrapolation of the simulated sensor lag; physical checks remain original')
 p.add_argument('--runtime-home',choices=('metadata','upstream-python'),default='metadata',help='Offline one-factor HOME contrast; initial physical state remains metadata pose in BOTH cases')
 p.add_argument('--mass-distribution-scenario',type=Path,help='Explicit file-only inertial hypothesis; constant total mass with optional trunk point-mass relocation')
 p.add_argument('--physical-mass-distribution-scenario',type=Path,help='Declared robustness counterfactual only: keep stance-map design binding but change physical load; requires body-frame-retarget')
 p.add_argument('--controller',choices=('actor','hold'),default='actor',help='Offline fixed-HOME HOLD contrast; zero actions do not run the actor or balance the body')
 p.add_argument('--initial-hold-seconds',type=float,default=0.,help='Offline HOLD-to-original-actor transition; no root-state injection')
 p.add_argument('--stance-reference',type=Path,help='Verified geometric stance hypothesis for reset/HOLD only; actor inputs and outputs remain unchanged')
 p.add_argument('--stance-retarget',action='store_true',help='Offline paired constant pose-map hypothesis: subtract solved joint offset from position observation and add it after original EMA; genuine gravity and joint velocity unchanged')
 p.add_argument('--body-frame-retarget',action='store_true',help='Explicit offline virtual-body map bound to solved rotation; rotate gyro/gravity for actor only, preserve real plant/safety measurements')
 p.add_argument('--runtime-head-profile',type=Path,help='Explicit installed head command for local HOLD/actor handoff contrast; no root or sensor rewrite')
 p.add_argument('--retain-head-filter-at-handoff',action='store_true',help='Seed only four head EMA entries from the preceding HOLD goal; paired with original zero-seed baseline')
 p.add_argument('--head-command-from-stance',action='store_true',help='Use the official four-dimensional head command from a solved HEAD-ONLY stance; no sensor remapping or model output offset')
 p.add_argument('--uncompensated-entry',action='store_true',help='Start from original randomized HOME; compensation reaches the plant only through actuator goals, without pre-positioning the simulated joints/root')
 p.add_argument('--recorded-entry-trial',type=Path,help='Conditional reset from first recorded zero-policy sample; no running state injection or forced actor observations')
 p.add_argument('--range-gate',choices=('legacy-request','transmitted'),default='transmitted',
  help='transmitted uses the real host validator after EMA and before MCU slew; legacy-request reproduces the archived, stricter diagnostic')
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 if bool(a.installed_action_candidate)!=bool(a.installed_action_calibration):raise ValueError('Actual output path needs both candidate and calibration')
 if a.installed_feedback_path and (not a.installed_action_candidate or a.sensor_lag_steps):raise ValueError('Feedback comparison needs actual output path and no second synthetic lag')
 if a.installed_action_candidate and (not a.official_runtime or not a.quantize_goals or not a.runtime_head_profile or a.saved_calibration or a.stance_reference or a.runtime_home!='metadata' or a.lag_prediction):
  raise ValueError('Actual output path must use metadata HOME, explicit head profile, exact ticks and its own declared calibration')
 if a.installed_action_candidate:
  import sys
  sys.path.insert(0,str(a.installed_action_candidate.resolve()/'policy'))
 if a.runtime_head_profile and (a.stance_reference or 'head' in a.factors):raise ValueError('Head profile contrast cannot combine with another head override')
 if a.retain_head_filter_at_handoff and not (a.runtime_head_profile and a.official_runtime and a.initial_hold_seconds>0):raise ValueError('Head handoff seed requires explicit profile, original EMA and prior HOLD')
 if a.controller=='hold' and a.speed!=0:raise ValueError('Fixed HOLD cannot be graded as commanded walking')
 if not np.isfinite(a.initial_hold_seconds) or a.initial_hold_seconds<0:raise ValueError('Invalid initial HOLD duration')
 if a.stance_retarget and not a.stance_reference:raise ValueError('Pose map requires a bound geometric reference')
 if a.body_frame_retarget and not a.stance_retarget:raise ValueError('Body frame requires stance retargeting')
 if a.head_command_from_stance and (not a.stance_reference or a.stance_retarget):raise ValueError('Source head command requires its own explicit stance and cannot combine with output/input retargeting')
 if a.uncompensated_entry and not a.stance_reference:raise ValueError('Uncompensated entry requires an explicit stance')
 if a.physical_mass_distribution_scenario and not a.stance_reference:raise ValueError('Physical load contrast requires a frozen declared stance design')
 # A head command may accompany a recorded physical entry only when the
 # solved pose is NOT used to initialize joint/root state. Keep other maps
 # separate: the paired comparison must start from the same recorded pose.
 if a.recorded_entry_trial and a.stance_reference and not (a.head_command_from_stance and a.uncompensated_entry):
  raise ValueError('Recorded entry cannot combine with solved stance initialization or sensor remapping')
 factors=set(filter(None,a.factors.split(',')));assert factors<={'gains','ema','slew','head','bias','reset','ranges','home','roll','targetclip','anklesoft'}
 if a.coordinate_knee and 'anklesoft' not in factors:raise ValueError('Knee coordination requires the explicit ankle map')
 assert np.isfinite(a.slew_ticks_s) and a.slew_ticks_s>0
 from train_constrained_policy import install_offline_guards
 install_offline_guards()
 import torch,onnxruntime as ort,mujoco
 from dataclasses import dataclass,fields
 from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand,UniformVelocityCommandCfg
 from mjlab.envs import ManagerBasedRlEnv
 from mjlab.managers import EventTermCfg
 from mjlab.envs.mdp.actions import JointPositionAction,JointPositionActionCfg
 from constrained_training_candidate import _set_source_gain_vector,_reset_candidate_joints,HEAD_INDICES
 from mjlab_microduck.tasks import _xgoduck_velocity_cfg
 from simulation_evidence import audit_interval_push,effective_body_properties
 from audit_transmitted_range_gate import validator
 cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
 c=json.loads(cp.read_text())['candidate'];model=Path(c['source_model']['path']);assert sha(model)==c['source_model']['sha256']
 calibration_overlay=None
 if a.saved_calibration:
  from offline_calibration_overlay import range_snapshot
  c,calibration_overlay=range_snapshot(c,a.saved_calibration)
 if a.official_runtime:
  if factors!={'gains','ema','ranges'}:raise ValueError('Official profile requires exactly gains,ema,ranges')
  c['source_expected_gains']['kp_by_id']={str(sid):6 for sid in c['policy_joint_ids']}
 stance_reference=None;stance_delta=np.zeros(14);body_frame_map=None
 if a.stance_reference:
  stance_reference=json.loads(a.stance_reference.read_text())
  if not stance_reference['geometry_verified']:raise ValueError('Unverified static stance')
  if stance_reference['actor_sha256']!=sha(model):raise ValueError('Stance actor binding differs')
  np.testing.assert_array_equal(stance_reference['home_rad'],c['home_rad'])
  if stance_reference['calibration_overlay']['canonical_sha256']!=c['calibration']['sha256']:raise ValueError('Stance calibration binding differs')
  if not a.mass_distribution_scenario or stance_reference['loaded']['scenario_sha256']!=sha(a.mass_distribution_scenario):raise ValueError('Stance load hypothesis differs')
  stance_delta=np.asarray(stance_reference['stance_delta_rad'],dtype=float)
  if stance_delta.shape!=(14,) or not np.isfinite(stance_delta).all():raise ValueError('Invalid stance delta')
  if a.body_frame_retarget:
   from body_frame_pose_map import BodyFramePoseMap
   body_frame_map=BodyFramePoseMap(stance_reference['body_frame_rotation'],stance_delta)
 validate_wire,validator_sha=validator(c)
 from rl_core import MOTOR_IDS
 joint_cal={j['id']:j for j in c['calibration']['joints']}
 class WireRangeRejected(Exception):pass
 home_shift=np.zeros(14)
 if 'home' in factors:
  for sid,degrees in {12:-1.,14:-1.,22:1.,24:1.}.items():home_shift[c['policy_joint_ids'].index(sid)]=np.deg2rad(degrees)
 runtime_home_source=None
 if a.runtime_home=='upstream-python':
  if 'home' in factors:raise ValueError('Do not combine upstream HOME with experimental HOME offsets')
  runtime_home_source=ROOT/'output/xgoduck_runtime_upstream/python/rl_core.py'
  spec=importlib.util.spec_from_file_location('causal_pinned_upstream_core',runtime_home_source)
  source_core=importlib.util.module_from_spec(spec);import sys
  sys.modules[spec.name]=source_core;spec.loader.exec_module(source_core)
  assert source_core.POLICY_IDS==c['policy_joint_ids']
  home_shift=np.deg2rad(source_core.HOME_DEG.astype(np.float64))-np.array(c['home_rad'])
  factors.add('home')
 roll_maps={}
 if 'roll' in factors:
  from model_adaptation_contract import legacy_candidate_contract
  legacy=legacy_candidate_contract()
  math_path=Path(legacy['source_files']['candidate_math.py']['path'])
  spec=importlib.util.spec_from_file_location('frozen_candidate_roll_math',math_path)
  roll_math=importlib.util.module_from_spec(spec);spec.loader.exec_module(roll_math)
  for j in c['calibration']['joints']:
   if j['id'] in (11,21):
    lo,hi=sorted((tick-j['zero_tick'])*360/4095*j['sign'] for tick in (j['min_tick']+3,j['max_tick']-3))
    roll_maps[j['policy_index']]=roll_math.ExponentialRollMap(lo,hi,16.)
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
 actor=ort.InferenceSession(str(model),opts,providers=['CPUExecutionProvider'])
 installed_path=None
 if a.installed_action_candidate:
  from installed_action_path import InstalledActionPath
  installed_path=InstalledActionPath(a.installed_action_candidate,a.installed_action_calibration,
    json.loads(a.runtime_head_profile.read_text())['walk_head_command_rad'],actor)
  c=installed_path.overlay(c)
  joint_cal={j['id']:j for j in c['calibration']['joints']}
  validate_wire,validator_sha=validator(c)
 cfg=copy.deepcopy(_xgoduck_velocity_cfg(play=False));cfg.scene.num_envs=1;cfg.seed=a.seed;cfg.auto_reset=False
 cfg.decimation=round(1/a.control_hz/cfg.sim.mujoco.timestep)
 robot=cfg.scene.entities['robot'];robot.init_state.joint_pos=dict(zip(c['joint_names'],c['home_rad']))
 mass_distribution=None
 if a.mass_distribution_scenario:
  from offline_mass_distribution import scenario_spec
  physical_mass_path=a.physical_mass_distribution_scenario or a.mass_distribution_scenario
  robot.spec_fn,mass_distribution=scenario_spec(robot.spec_fn,physical_mass_path)
  mass_distribution['implementation_sha256']=sha(ROOT/'tools/x5_policy/offline_mass_distribution.py')
 robot.init_state.pos=(*robot.init_state.pos[:2],c['reset_placement']['root_z_m'])
 cfg.events['reset_base'].params['pose_range']['z']=(0.,0.)
 cfg.actions['joint_pos'].scale=c['action_scale']
 cfg.commands['head_pose'].ranges=((0.,0.),)*4;cfg.commands['body_pose'].ranges=((0.,0.),)*6
 cfg.curriculum.clear()
 head_command=np.array(c['held_head_rad'])-np.array(c['home_rad'])[list(HEAD_INDICES)] if 'head' in factors else np.zeros(4)
 if a.head_command_from_stance:
  legs=[i for i in range(14) if i not in HEAD_INDICES]
  if np.any(stance_delta[legs]) or stance_reference['body_pitch_rad']!=0.:
   raise ValueError('Official head command comparison accepts a HEAD-ONLY stance')
  head_command=stance_delta[list(HEAD_INDICES)].copy()
  # Pinned official controller.command accepts abs(head)<=1rad per component.
  if np.any(abs(head_command)>1.):raise ValueError('Exceeds original head-command contract')
 if a.runtime_head_profile:
  head_command=np.asarray(json.loads(a.runtime_head_profile.read_text())['walk_head_command_rad'],dtype=float)
  if head_command.shape!=(4,) or not np.isfinite(head_command).all() or np.any(abs(head_command)>1.):raise ValueError('Invalid original four-head-command profile')
  stance_delta[list(HEAD_INDICES)]=head_command
 cfg.commands['head_pose'].ranges=tuple((float(v),float(v)) for v in head_command)
 if 'gains' in factors:cfg.events['factor_gains']=EventTermCfg(func=_set_source_gain_vector,mode='startup',params={'candidate':c})
 if 'bias' in factors:
  cfg.events.pop('encoder_bias',None)
  for group in ('actor','critic'):cfg.observations[group].terms['joint_pos'].params['biased']=False
 if 'reset' in factors:
  for name,event in list(cfg.events.items()):
   if getattr(event.func,'__name__','')=='reset_joints_by_offset':cfg.events[name]=EventTermCfg(func=_reset_candidate_joints,mode='reset',params={'candidate':c})
 if stance_reference and not a.uncompensated_entry:
  def apply_geometric_stance_reset(env,env_ids):
   if env.common_step_counter!=0:raise RuntimeError('Geometric stance cannot overwrite a running plant')
   asset=env.scene['robot']
   ids=torch.arange(env.num_envs,device=env.device) if env_ids is None else env_ids
   joints=torch.tensor([list(asset.joint_names).index(n) for n in c['joint_names']],device=env.device)
   position=asset.data.joint_pos[ids][:,joints].clone()+torch.tensor(stance_delta,device=env.device,dtype=torch.float32)
   velocity=asset.data.joint_vel[ids][:,joints].clone()
   # Keep default_joint_pos/actor offset and random draws intact. Only the
   # declared initial joint pose changes; never move a running root state.
   asset.write_joint_state_to_sim(position,velocity,joint_ids=joints,env_ids=ids)
   if body_frame_map is not None:
    from mjlab.utils.lab_api.math import quat_mul
    import mujoco
    quaternion=np.empty(4)
    mujoco.mju_mat2Quat(quaternion,np.ascontiguousarray(body_frame_map.rotation).ravel())
    # Other reset events have written qpos/qvel; derived link poses must be
    # refreshed before composing the declared initial frame change.
    env.sim.forward()
    state=torch.cat((asset.data.root_link_pose_w[ids],asset.data.root_link_vel_w[ids]),dim=-1).clone()
    state[:,3:7]=quat_mul(state[:,3:7],torch.tensor(quaternion,device=env.device,dtype=torch.float32).repeat(len(ids),1))
    state[:,0]+=stance_reference['trunk_forward_translation_m']
    state[:,1]+=stance_reference.get('trunk_lateral_translation_m',0.)
    state[:,2]+=stance_reference['trunk_height_change_m']
    asset.write_root_state_to_sim(state,env_ids=ids)
  cfg.events['geometric_stance_reset']=EventTermCfg(func=apply_geometric_stance_reset,mode='reset')
 recorded_entry_provenance=None
 if a.recorded_entry_trial:
  from offline_recorded_entry import recorded_entry
  initialize,recorded_entry_provenance=recorded_entry(a.recorded_entry_trial,c,
   ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml')
  cfg.events['recorded_entry_reset']=EventTermCfg(func=initialize,mode='reset')
 class FactorAction(JointPositionAction):
  def __init__(self,cfg,env):
   super().__init__(cfg,env)
   self.home=torch.tensor(np.array(c['home_rad'])+home_shift,dtype=torch.float32,device=self.device);self.low=torch.tensor(c['lower_rad'],device=self.device);self.high=torch.tensor(c['upper_rad'],device=self.device)
   self.held=self.home.clone();self.held[list(HEAD_INDICES)]=torch.tensor(c['held_head_rad'],device=self.device)
   self.ema=self.home.repeat(self.num_envs,1);self.goal=self.ema.clone();self.range_failed=False
   self.action_ema=torch.zeros_like(self.ema)
   self.holding=a.controller=='hold' or a.initial_hold_seconds>0
   self.was_holding=False
   self.fixed_hold_target=self.home+torch.tensor(stance_delta,dtype=torch.float32,device=self.device)
   self.delta=a.slew_ticks_s*2*np.pi/c['action']['ticks_per_turn']*env.step_dt
  def soft_ankle(self,target):
   index=c['policy_joint_ids'].index(14)
   pivot=self.home[index]
   span=self.high[index]-3*2*np.pi/c['action']['ticks_per_turn']-pivot
   if span <= 0:raise ValueError('No ankle retargeting interval above HOME')
   q=target[:,index];target=target.clone()
   target[:,index]=torch.where(q>pivot,pivot-span*torch.expm1(-torch.clamp(q-pivot,min=0)/span),q)
   if a.coordinate_knee:
    # robot_walk.xml's ankle-body quaternion reverses its Z hinge relative to
    # the knee. Measured world-axis dot=-0.999999999973; equal joint deltas
    # preserve sole pitch to <0.000074deg for a10deg ankle change. This does
    # not preserve foot translation or establish dynamic feasibility.
    target[:,c['policy_joint_ids'].index(13)]+=target[:,index]-q
   return target
  def process_actions(self,actions):
   super().process_actions(actions)
   if installed_path is not None:
    goal=installed_path.step(actions.detach().cpu().numpy(),self.holding)
    self._processed_actions=torch.tensor([goal.tolist()],dtype=actions.dtype,device=self.device)
    self.goal=self._processed_actions.clone()
    self.requested=torch.tensor([installed_path.last['requested_policy_rad']],dtype=actions.dtype,device=self.device)
    self.unbounded_transmitted=self.requested.clone();self.transmitted=self.goal.clone()
    self.wire_validation=dict(accepted=True,ticks=installed_path.last['ticks'],projection=installed_path.last['projection'])
    self.was_holding=self.holding;self.range_failed=False
    return
   target=self._processed_actions
   if 'home' in factors:target=target+torch.tensor(home_shift,dtype=target.dtype,device=self.device)
   for index,fn in roll_maps.items():
    home=float(c['home_rad'][index]+home_shift[index])
    q=np.rad2deg(home+c['action_scale']*.95*float(actions[0,index]))
    adapted=np.float32((np.deg2rad(fn(q))-home)/c['action_scale'])
    target[0,index]=home+c['action_scale']*float(adapted)
   if 'head' in factors:target[:,list(HEAD_INDICES)]=self.held[list(HEAD_INDICES)]
   self.requested=target.clone()
   if a.stance_retarget:self.requested+=torch.tensor(stance_delta,dtype=target.dtype,device=self.device)
   if 'anklesoft' in factors and a.ankle_map_stage=='pre-ema':target=self.soft_ankle(target)
   if a.official_runtime:
    # Match the official float32 multiply-then-add; previous observation action remains raw.
    # Isolated handoff hypothesis: no new gain, EMA coefficient, or interpolation.
    # Prior goal is an actuator command, never a write to the running plant state.
    if a.retain_head_filter_at_handoff and self.was_holding and not self.holding:
     self.action_ema[:,list(HEAD_INDICES)]=(self.goal[:,list(HEAD_INDICES)]-self.home[list(HEAD_INDICES)])/c['action_scale']
    self.action_ema *= .45
    self.action_ema += actions * .55
    target=self.home+c['action_scale']*self.action_ema
   elif 'ema' in factors:self.ema=c['action']['alpha']*self.ema+(1-c['action']['alpha'])*target;target=self.ema
   if a.stance_retarget:target=target+torch.tensor(stance_delta,dtype=target.dtype,device=self.device)
   if self.holding:
    target=self.fixed_hold_target.repeat(self.num_envs,1)
    self.requested=target.clone()
   self.unbounded_transmitted=target.clone()
   if a.head_range_projection:
    target=target.clone()
    for index in HEAD_INDICES:
     # Explicit saved bounds, no new shrink factor or hidden fixed head pose.
     target[:,index]=torch.clamp(target[:,index],min=self.low[index],max=self.high[index])
   if 'anklesoft' in factors and a.ankle_map_stage=='post-ema':
    # New isolated hypothesis: C1 upper retargeting avoids a hard-stop corner.
    # The pivot is the original policy HOME, not an invented angle threshold.
    # Preserve HOME and the entire lower branch. Do not alter action history.
    target=self.soft_ankle(target)
   if 'targetclip' in factors:
    # Isolated decoder hypothesis, not a replacement for host/MCU validation.
    # Preserve saved bounds and the prior roll adapter's three-count margin.
    margin=3*2*np.pi/c['action']['ticks_per_turn']
    target=torch.clamp(target,min=self.low+margin,max=self.high-margin)
    if a.left_ankle_margin_ticks < 3:raise ValueError('Cannot reduce the existing inset')
    index=c['policy_joint_ids'].index(14)
    ankle_high=self.high[index]-a.left_ankle_margin_ticks*2*np.pi/c['action']['ticks_per_turn']
    if ankle_high <= self.low[index]+margin:raise ValueError('Inset consumes entire ankle range')
    target[:,index]=torch.minimum(target[:,index],ankle_high)
   self.transmitted=target.clone()
   self.wire_validation=validate_wire(target[0].detach().cpu().numpy())
   if 'ranges' in factors and a.range_gate=='transmitted' and not self.wire_validation['accepted']:
    raise WireRangeRejected()
   if a.quantize_goals:
    if not self.wire_validation['accepted']:raise WireRangeRejected()
    ticks=self.wire_validation['ticks']
    quantized=[(ticks[MOTOR_IDS.index(sid)]-joint_cal[sid]['zero_tick'])*2*np.pi/4095*joint_cal[sid]['sign'] for sid in c['policy_joint_ids']]
    target=torch.tensor([quantized],dtype=target.dtype,device=self.device)
   if 'slew' in factors:target=self.goal+torch.clamp(target-self.goal,-self.delta,self.delta)
   self.goal=target.clone();self._processed_actions=target
   self.was_holding=self.holding
   self.range_failed=bool(((self.requested<self.low)|(self.requested>self.high)).any())
  def reset(self,env_ids=None):
   super().reset(env_ids)
   ids=slice(None) if env_ids is None else env_ids
   self.ema[ids]=self.held if 'head' in factors else self.home;self.goal[ids]=self.ema[ids];self.range_failed=False
   self.action_ema[ids]=0
   self.was_holding=False
 @dataclass(kw_only=True)
 class FactorActionCfg(JointPositionActionCfg):
  def build(self,env):return FactorAction(self,env)
 oldaction=cfg.actions['joint_pos'];cfg.actions['joint_pos']=FactorActionCfg(**{f.name:copy.deepcopy(getattr(oldaction,f.name)) for f in fields(JointPositionActionCfg)})
 def speed_at(at):return a.speed*min(max((at-5.)/.5,0.),1.,max((11.-at)/.5,0.))
 class ReferenceCommand(UniformVelocityCommand):
  def _resample(self,env_ids):
   self.time_left[env_ids]=float('inf');self.vel_command_b[env_ids]=0.;self.vel_command_w[env_ids]=0.
   self.is_standing_env[env_ids]=True;self.is_heading_env[env_ids]=False
   self.is_world_env[env_ids]=False;self.is_forward_env[env_ids]=False;self.command_counter[env_ids]+=1
  def compute(self,dt):
   vx=speed_at(self._env.common_step_counter*self._env.step_dt)
   self.vel_command_b.zero_();self.vel_command_b[:,0]=vx;self.vel_command_w.copy_(self.vel_command_b)
   self.is_standing_env[:]=vx==0
 @dataclass(kw_only=True)
 class ReferenceCommandCfg(UniformVelocityCommandCfg):
  def build(self,env):return ReferenceCommand(self,env)
 old=cfg.commands['twist'];args={f.name:copy.deepcopy(getattr(old,f.name)) for f in fields(UniformVelocityCommandCfg)}
 args.update(heading_command=False,rel_heading_envs=0.,rel_world_envs=0.,rel_forward_envs=0.,rel_standing_envs=0.,init_velocity_prob=0.)
 args['ranges']=UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.,a.speed),lin_vel_y=(0.,0.),ang_vel_z=(0.,0.))
 cfg.commands['twist']=ReferenceCommandCfg(**args)
 ledger=[];push=audit_interval_push(cfg,ledger,scale=0.)
 evaluator=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'
 spec=importlib.util.spec_from_file_location('source_reference_measurement',evaluator)
 measurement=importlib.util.module_from_spec(spec);spec.loader.exec_module(measurement)
 report=dict(scope=__doc__,hardware_access=False,hardware_eligible=False,walking_accepted=False,seed=a.seed,
  source_compatibility_assumption=True,factors=sorted(factors),model_sha256=sha(model),source_task_sha256=sha(inspect.getfile(_xgoduck_velocity_cfg)),
  source_evaluator_sha256=sha(evaluator),source_sha256=sha(__file__),speed_m_s=a.speed,push=push,
  original_saved_limits_unchanged=True,original_source_bam_cfg=repr(robot.articulation.actuators[0]),
  local_action_adapter_present=True,head_outputs_follow_original_actor='head' not in factors,maximum_wall_seconds=90,frames=0)
 report.update(range_gate=a.range_gate,wire_validator_sha256=validator_sha,left_ankle_margin_ticks=a.left_ankle_margin_ticks,ankle_map_stage=a.ankle_map_stage,integer_wire_goals_applied=a.quantize_goals,knee_pitch_coordinated=a.coordinate_knee,
  official_runtime_profile=a.official_runtime,effective_kp_by_id=c['source_expected_gains']['kp_by_id'] if 'gains' in factors else None,
  runtime_head_profile=str(a.runtime_head_profile) if a.runtime_head_profile else None,
  retain_head_filter_at_handoff=a.retain_head_filter_at_handoff,
  head_range_projection=a.head_range_projection,
  calibration_overlay=calibration_overlay,calibration_sha256=c['calibration']['sha256'],
  range_gate_correction='Saved raw-actor excursions remain diagnostics; transmitted validation occurs after EMA and head selection, before MCU slew.')
 report.update(home_shift_deg=np.rad2deg(home_shift).tolist(),offline_slew_ticks_s=a.slew_ticks_s,
  installed_action_path=None if installed_path is None else installed_path.provenance,
  installed_feedback_path=a.installed_feedback_path,
  mass_distribution=mass_distribution,
  load_map_design_scenario=str(a.mass_distribution_scenario) if a.mass_distribution_scenario else None,
  load_map_robustness_counterfactual=str(a.physical_mass_distribution_scenario) if a.physical_mass_distribution_scenario else None,
  runtime_home=a.runtime_home,runtime_home_source_sha256=sha(runtime_home_source) if runtime_home_source else None,
  initial_physical_pose_reference='metadata HOME retained independently of runtime HOME',
  control_hz=a.control_hz,sensor_lag_steps=a.sensor_lag_steps,
  lag_prediction=a.lag_prediction,
  lag_prediction_sha256=sha(ROOT/'tools/x5_policy/predict_policy_sensors.py') if a.lag_prediction else None,
  controller=a.controller,actor_controls_plant=a.controller=='actor',
  initial_hold_seconds=a.initial_hold_seconds,stance_reference=stance_reference,
  uncompensated_entry=a.uncompensated_entry,
  initial_compensated_pose=bool(stance_reference and not a.uncompensated_entry),
  recorded_entry=recorded_entry_provenance,
  stance_retarget=a.stance_retarget,
  head_command_from_stance=a.head_command_from_stance,
  official_head_command_rad=head_command.tolist(),
  body_frame_retarget=a.body_frame_retarget,
  body_frame_map_scope='Offline only: Rphysical=Rvirtual@solvedR; rotate physical gyro/gravity to virtual body, constant joint offset; all physical safety/range checks unchanged' if body_frame_map is not None else None,
  stance_reference_scope=('Explicit virtual-body/constant-joint map; physical IMU untouched, actor-only basis rotation; original ONNX/EMA/history retained' if body_frame_map is not None else 'Explicit constant pose map; physical q=virtual q+solved offset, velocity Jacobian identity, gravity unchanged; original ONNX/EMA/action history retained' if a.stance_retarget else 'Reset and fixed HOLD only; genuine gravity/gyro/joint sensors and original actor decoding retained; not a runtime adapter'),
  body_frame_map_sha256=sha(ROOT/'tools/x5_policy/body_frame_pose_map.py') if body_frame_map is not None else None,
  timing_hypothesis='Sensor-only sample delay; current previous raw action and command preserved. Not a replay of current real hardware timing.',
  legacy_roll_source=legacy if 'roll' in factors else None,
  transform_scope='Existing HOME and roll transformations applied in isolated simulation; physical reset/randomization unchanged, raw previous action preserved.')
 (a.output/'source.py').write_bytes(Path(__file__).read_bytes())
 env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);rows=[];started=time.monotonic()
 try:
  obs,_=env.reset(seed=a.seed)
  _,_,report['actual_body_properties']=effective_body_properties(env)
  term=env.action_manager.get_term('joint_pos')
  assert list(term.target_names)==c['joint_names']
  np.testing.assert_allclose(term._offset.numpy()[0],c['home_rad'],atol=1e-7,rtol=0)
  initial=measurement.actual_plant_measurements(env);report['initial']=initial
  if recorded_entry_provenance:
   asset=env.scene['robot'];joints=term.target_ids
   np.testing.assert_allclose(asset.data.joint_pos[0,joints].numpy(),recorded_entry_provenance['joint_rad'],atol=1e-7,rtol=0)
   np.testing.assert_allclose(asset.data.joint_vel[0,joints].numpy(),recorded_entry_provenance['joint_velocity_rad_s'],atol=1e-7,rtol=0)
   np.testing.assert_allclose(asset.data.projected_gravity_b[0].numpy(),np.asarray(recorded_entry_provenance['body_gravity'])/np.linalg.norm(recorded_entry_provenance['body_gravity']),atol=2e-7,rtol=0)
  report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  report['initial_qvel_sha256']=hashlib.sha256(env.sim.data.qvel.numpy().tobytes()).hexdigest()
  report['initial_fingerprint']=initial_fingerprint(env)
  report['first_request_outside_saved_ranges']=None
  report['control_dt_s']=env.step_dt
  assert abs(env.step_dt-1/a.control_hz)<1e-12
  sensor_history=[]
  for k in range(round(14/env.step_dt)):
   if time.monotonic()-started>90:raise TimeoutError('Reference wall budget')
   raw=obs['actor'].numpy().copy();assert raw.shape==(1,61)
   physical_observation=raw[0].copy()
   if a.installed_feedback_path:
    def sensor(name):
     matches=[i for i in range(env.sim.mj_model.nsensor) if
      (mujoco.mj_id2name(env.sim.mj_model,mujoco.mjtObj.mjOBJ_SENSOR,i) or '').split('/')[-1]==name]
     if len(matches)!=1:raise ValueError('Missing or ambiguous CAD sensor '+name)
     sid=matches[0]
     start=int(env.sim.mj_model.sensor_adr[sid]);size=int(env.sim.mj_model.sensor_dim[sid])
     return env.sim.data.sensordata[0,start:start+size].numpy().copy()
    raw=installed_path.observation_from_sensors(
      term._entity.data.joint_pos[0,term.target_ids].numpy(),
      term._entity.data.joint_vel[0,term.target_ids].numpy(),
      sensor('imu_accel'),sensor('imu_ang_vel'),(k+1)*env.step_dt,
      physical_observation[48:51])
   raw[0,6:20]-=home_shift.astype(np.float32)
   if body_frame_map is not None:raw=body_frame_map.observations(raw)
   elif a.stance_retarget:raw[0,6:20]-=stance_delta.astype(np.float32)
   np.testing.assert_allclose(raw[0,51:61],np.r_[head_command,np.zeros(6)],atol=1e-7,rtol=0)
   np.testing.assert_allclose(raw[0,48:51],[speed_at(k*env.step_dt),0.,0.],atol=1e-7,rtol=0)
   current_sensors=raw[0,:34].copy()
   sensor_history.append(current_sensors)
   if len(sensor_history)>a.sensor_lag_steps+1:sensor_history.pop(0)
   raw[0,:34]=sensor_history[0]
   if a.lag_prediction:
    from predict_policy_sensors import extrapolate
    observed_age=(len(sensor_history)-1)*env.step_dt
    raw=extrapolate(raw,observed_age,observed_age)
   term.holding=a.controller=='hold' or k*env.step_dt<a.initial_hold_seconds
   action=(actor.run(None,{actor.get_inputs()[0].name:raw})[0] if not term.holding
           else np.zeros((1,14),dtype=np.float32))
   target=np.array(c['home_rad'])+c['action_scale']*action[0]
   if a.runtime_home=='upstream-python':target=target+home_shift
   if a.stance_retarget:target=target+stance_delta
   bad=np.flatnonzero((target<np.array(c['lower_rad']))|(target>np.array(c['upper_rad'])))
   if len(bad) and report['first_request_outside_saved_ranges'] is None:
    report['first_request_outside_saved_ranges']=dict(frame=k,ids=[c['policy_joint_ids'][i] for i in bad],degrees=np.rad2deg(target[bad]).tolist())
   try:
    obs,reward,terminated,truncated,_=env.step(torch.from_numpy(action))
   except WireRangeRejected:
    report['transmitted_range_rejection']=dict(frame=k,time_s=k*env.step_dt,
      physics_advanced=False,**term.wire_validation['rejection'])
    if rows:rows[-1]['failed_terms'].append('transmitted_saved_range')
    break
   row=measurement.actual_plant_measurements(env);row.update(frame=k,time_s=(k+1)*env.step_dt,
    input_command=raw[0,48:51].tolist(),action=action[0].tolist(),requested_joint_rad=target.tolist(),
    controller_phase='hold' if term.holding else 'official_actor',
    terminated=bool(terminated[0]),truncated=bool(truncated[0]),
    actual_joint_rad=term._entity.data.joint_pos[:,term.target_ids].numpy()[0].tolist(),
    actual_joint_velocity_rad_s=term._entity.data.joint_vel[:,term.target_ids].numpy()[0].tolist(),
    actor_observation=raw[0].tolist(),processed_joint_rad=term._processed_actions.numpy()[0].tolist(),adapter_requested_joint_rad=term.requested.numpy()[0].tolist(),
    physical_actor_observation=physical_observation.tolist(),
    transmitted_joint_rad=term.transmitted.numpy()[0].tolist(),wire_validation=term.wire_validation,
    unbounded_transmitted_joint_rad=term.unbounded_transmitted.numpy()[0].tolist(),
    current_sensor_observation=current_sensors.tolist(),
    command_m_s=float(raw[0,48]),failed_terms=['terminated'] if terminated[0] else [],task_complete=k==round(14/env.step_dt)-1)
   row['saved_range_conflicts']={}
   for key in ('requested_joint_rad','adapter_requested_joint_rad','processed_joint_rad','actual_joint_rad'):
    values=np.asarray(row[key]);bad=np.flatnonzero((values<np.array(c['lower_rad']))|(values>np.array(c['upper_rad'])))
    row['saved_range_conflicts'][key]=[c['policy_joint_ids'][i] for i in bad]
   if 'ranges' in factors and a.range_gate=='legacy-request' and term.range_failed:row['failed_terms'].append('saved_range')
   if 'ranges' in factors and row['saved_range_conflicts']['actual_joint_rad']:row['failed_terms'].append('actual_saved_range')
   rows.append(row)
   if row['tilt_deg']>=15 or terminated[0] or truncated[0] or row['failed_terms']:break
  from evaluate_slow_gait import grade
  report['gate']=grade(rows,'zero' if a.speed==0 else 'forward',sample_period=env.step_dt);report['gate']['scope']='offline factor reference; not a hardware candidate'
  report['all_saved_ranges']=bool(rows) and all(not ids for row in rows for key,ids in row['saved_range_conflicts'].items() if key!='requested_joint_rad')
  report['all_transmitted_ranges']=bool(rows) and all(row['wire_validation']['accepted'] for row in rows) and 'transmitted_range_rejection' not in report
  report['all_actual_ranges']=bool(rows) and all(not row['saved_range_conflicts']['actual_joint_rad'] for row in rows)
  applicable_range_check=report['all_saved_ranges'] if a.range_gate=='legacy-request' else report['all_transmitted_ranges'] and report['all_actual_ranges']
  report['fully_constrained_simulation_accepted']=bool(report['gate']['accepted'] and applicable_range_check and a.slew_ticks_s==600 and {'head','gains','ema','slew','ranges'}<=factors)
  report['fixed_hold_simulation_accepted']=bool(a.controller=='hold' and report['gate']['accepted'] and applicable_range_check)
  report['installed_action_path_simulation_accepted']=bool(installed_path is not None and a.controller=='actor' and report['gate']['accepted'] and applicable_range_check)
  report['official_runtime_simulation_accepted']=bool(installed_path is None and a.controller=='actor' and a.official_runtime and not a.stance_retarget and a.quantize_goals and report['gate']['accepted'] and applicable_range_check)
  report['official_onnx_pose_map_simulation_accepted']=bool(a.official_runtime and a.stance_retarget and a.quantize_goals and report['gate']['accepted'] and applicable_range_check)
  report.update(frames=len(rows),max_tilt_deg=max((x['tilt_deg'] for x in rows),default=None),
   stopped_at_existing_15deg_observation_boundary=bool(rows) and rows[-1]['tilt_deg']>=15,
   qualified_lift_samples={s:sum(x['feet'][s]['lifted'] for x in rows) for s in ('left','right')},
   wall_seconds=time.monotonic()-started,complete=True)
 finally:
  env.close();(a.output/'trace.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows))
  (a.output/'push-ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
  (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:report[k] for k in ('frames','max_tilt_deg','qualified_lift_samples','first_request_outside_saved_ranges','wall_seconds')}))

if __name__=='__main__':main()
