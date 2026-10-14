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
 p.add_argument('--factors',default='',help='comma-separated: gains,ema,slew,head,bias,reset,ranges')
 p.add_argument('--pulse-duration',type=float,default=1.2)
 p.add_argument('--pulse-ramp',type=float,default=.4)
 p.add_argument('--servo-period',type=float,default=.01,choices=(.01,.02))
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 factors=set(filter(None,a.factors.split(',')));assert factors<={'gains','ema','slew','head','bias','reset','ranges'}
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
 cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
 c=json.loads(cp.read_text())['candidate'];model=Path(c['source_model']['path']);assert sha(model)==c['source_model']['sha256']
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
 actor=ort.InferenceSession(str(model),opts,providers=['CPUExecutionProvider'])
 cfg=copy.deepcopy(_xgoduck_velocity_cfg(play=False));cfg.scene.num_envs=1;cfg.seed=a.seed;cfg.auto_reset=False
 robot=cfg.scene.entities['robot'];robot.init_state.joint_pos=dict(zip(c['joint_names'],c['home_rad']))
 robot.init_state.pos=(*robot.init_state.pos[:2],c['reset_placement']['root_z_m'])
 cfg.events['reset_base'].params['pose_range']['z']=(0.,0.)
 cfg.actions['joint_pos'].scale=c['action_scale']
 cfg.commands['head_pose'].ranges=((0.,0.),)*4;cfg.commands['body_pose'].ranges=((0.,0.),)*6
 cfg.curriculum.clear()
 head_command=np.array(c['held_head_rad'])-np.array(c['home_rad'])[list(HEAD_INDICES)] if 'head' in factors else np.zeros(4)
 cfg.commands['head_pose'].ranges=tuple((float(v),float(v)) for v in head_command)
 if 'gains' in factors:cfg.events['factor_gains']=EventTermCfg(func=_set_source_gain_vector,mode='startup',params={'candidate':c})
 if 'bias' in factors:
  cfg.events.pop('encoder_bias',None)
  for group in ('actor','critic'):cfg.observations[group].terms['joint_pos'].params['biased']=False
 if 'reset' in factors:
  for name,event in list(cfg.events.items()):
   if getattr(event.func,'__name__','')=='reset_joints_by_offset':cfg.events[name]=EventTermCfg(func=_reset_candidate_joints,mode='reset',params={'candidate':c})
 class FactorAction(JointPositionAction):
  def __init__(self,cfg,env):
   super().__init__(cfg,env)
   self.home=torch.tensor(c['home_rad'],device=self.device);self.low=torch.tensor(c['lower_rad'],device=self.device);self.high=torch.tensor(c['upper_rad'],device=self.device)
   self.held=self.home.clone();self.held[list(HEAD_INDICES)]=torch.tensor(c['held_head_rad'],device=self.device)
   self.ema=self.home.repeat(self.num_envs,1);self.goal=self.ema.clone();self.range_failed=False
   self.delta=c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']*env.step_dt
   by_id={j['id']:j for j in c['calibration']['joints']}
   self.zeros=torch.tensor([by_id[i]['zero_tick'] for i in c['policy_joint_ids']],device=self.device)
   self.signs=torch.tensor([by_id[i]['sign'] for i in c['policy_joint_ids']],device=self.device)
   self.counts_per_rad=c['action']['ticks_per_turn']/(2*np.pi)
   self.continuous=self.zeros+self.signs*self.goal*self.counts_per_rad
   self.desired=torch.floor(self.continuous+.5)
   self.servo_steps=round(a.servo_period/env.physics_dt)
   assert abs(self.servo_steps*env.physics_dt-a.servo_period)<1e-9
   self.substep=0
  def process_actions(self,actions):
   super().process_actions(actions)
   target=self._processed_actions
   if 'head' in factors:target[:,list(HEAD_INDICES)]=self.held[list(HEAD_INDICES)]
   self.requested=target.clone()
   if 'ema' in factors:self.ema=c['action']['alpha']*self.ema+(1-c['action']['alpha'])*target;target=self.ema
   self.desired=torch.floor(self.zeros+self.signs*target*self.counts_per_rad+.5)
   self.range_failed=bool(((self.requested<self.low)|(self.requested>self.high)).any())
  def apply_actions(self):
   if self.substep % self.servo_steps==0:
    if 'slew' in factors:
     delta=c['action']['goal_slew_ticks_s']*a.servo_period
     self.continuous+=torch.clamp(self.desired-self.continuous,-delta,delta)
    else:self.continuous=self.desired.clone()
    self.goal=(torch.floor(self.continuous+.5)-self.zeros)*self.signs/self.counts_per_rad
   self._processed_actions=self.goal
   super().apply_actions()
   self.substep+=1
  def reset(self,env_ids=None):
   super().reset(env_ids)
   ids=slice(None) if env_ids is None else env_ids
   self.ema[ids]=self.held if 'head' in factors else self.home;self.goal[ids]=self.ema[ids];self.range_failed=False
   self.continuous[ids]=self.zeros+self.signs*self.goal[ids]*self.counts_per_rad
   self.desired[ids]=torch.floor(self.continuous[ids]+.5);self.substep=0
 @dataclass(kw_only=True)
 class FactorActionCfg(JointPositionActionCfg):
  def build(self,env):return FactorAction(self,env)
 oldaction=cfg.actions['joint_pos'];cfg.actions['joint_pos']=FactorActionCfg(**{f.name:copy.deepcopy(getattr(oldaction,f.name)) for f in fields(JointPositionActionCfg)})
 duration=5.+a.pulse_duration+3.
 def speed_at(at):return a.speed*min(max((at-5.)/a.pulse_ramp,0.),1.,max((5.+a.pulse_duration-at)/a.pulse_ramp,0.))
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
 (a.output/'source.py').write_bytes(Path(__file__).read_bytes())
 env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);rows=[];started=time.monotonic()
 try:
  obs,_=env.reset(seed=a.seed)
  _,_,report['actual_body_properties']=effective_body_properties(env)
  term=env.action_manager.get_term('joint_pos')
  # Firmware ARM starts its interpolator at the latest actual encoder reading.
  actual=term._entity.data.joint_pos[:,term.target_ids]
  term.continuous=torch.floor(term.zeros+term.signs*actual*term.counts_per_rad+.5)
  term.goal=(term.continuous-term.zeros)*term.signs/term.counts_per_rad
  report['firmware_approximation']=dict(servo_period_s=a.servo_period,integer_encoder_targets=True,
   limiter_ticks_per_s=600 if 'slew' in factors else None,
   ideal_schedule_only=True,real_bus_jitter_verified=False,hardware_eligible=False)
  assert list(term.target_names)==c['joint_names']
  np.testing.assert_allclose(term._offset.numpy()[0],c['home_rad'],atol=1e-7,rtol=0)
  initial=measurement.actual_plant_measurements(env);report['initial']=initial
  report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  report['initial_qvel_sha256']=hashlib.sha256(env.sim.data.qvel.numpy().tobytes()).hexdigest()
  report['initial_fingerprint']=initial_fingerprint(env)
  report['first_request_outside_saved_ranges']=None
  report['control_dt_s']=env.step_dt
  for k in range(round(duration/env.step_dt)):
   if time.monotonic()-started>90:raise TimeoutError('Reference wall budget')
   raw=obs['actor'].numpy().copy();assert raw.shape==(1,61)
   np.testing.assert_allclose(raw[0,51:61],np.r_[head_command,np.zeros(6)],atol=1e-7,rtol=0)
   np.testing.assert_allclose(raw[0,48:51],[speed_at(k*env.step_dt),0.,0.],atol=1e-7,rtol=0)
   action=actor.run(None,{actor.get_inputs()[0].name:raw})[0]
   target=np.array(c['home_rad'])+c['action_scale']*action[0]
   bad=np.flatnonzero((target<np.array(c['lower_rad']))|(target>np.array(c['upper_rad'])))
   if len(bad) and report['first_request_outside_saved_ranges'] is None:
    report['first_request_outside_saved_ranges']=dict(frame=k,ids=[c['policy_joint_ids'][i] for i in bad],degrees=np.rad2deg(target[bad]).tolist())
   obs,reward,terminated,truncated,_=env.step(torch.from_numpy(action))
   row=measurement.actual_plant_measurements(env);row.update(frame=k,time_s=(k+1)*env.step_dt,
    input_command=raw[0,48:51].tolist(),action=action[0].tolist(),requested_joint_rad=target.tolist(),
    terminated=bool(terminated[0]),truncated=bool(truncated[0]),
    actual_joint_rad=term._entity.data.joint_pos[:,term.target_ids].numpy()[0].tolist(),
    actual_joint_velocity_rad_s=term._entity.data.joint_vel[:,term.target_ids].numpy()[0].tolist(),
    actor_observation=raw[0].tolist(),processed_joint_rad=term._processed_actions.numpy()[0].tolist(),adapter_requested_joint_rad=term.requested.numpy()[0].tolist(),
    command_m_s=float(raw[0,48]),failed_terms=['terminated'] if terminated[0] else [],task_complete=k==round(duration/env.step_dt)-1)
   row['saved_range_conflicts']={}
   for key in ('requested_joint_rad','adapter_requested_joint_rad','processed_joint_rad','actual_joint_rad'):
    values=np.asarray(row[key]);bad=np.flatnonzero((values<np.array(c['lower_rad']))|(values>np.array(c['upper_rad'])))
    row['saved_range_conflicts'][key]=[c['policy_joint_ids'][i] for i in bad]
   if 'ranges' in factors and term.range_failed:row['failed_terms'].append('saved_range')
   if 'ranges' in factors and row['saved_range_conflicts']['actual_joint_rad']:row['failed_terms'].append('actual_saved_range')
   rows.append(row)
   if row['tilt_deg']>=15 or terminated[0] or truncated[0] or row['failed_terms']:break
  from native_pulse_grade import grade_native
  report['pulse']=dict(peak=a.speed,duration=a.pulse_duration,ramp=a.pulse_ramp)
  report['gate']=grade_native(rows,50);report['gate']['scope']='offline factor reference; not a hardware candidate'
  report['all_saved_ranges']=all(not ids for row in rows for key,ids in row['saved_range_conflicts'].items() if key!='requested_joint_rad')
  report['fully_constrained_simulation_accepted']=bool(report['gate']['accepted'] and report['all_saved_ranges'] and {'head','gains','ema','slew','ranges'}<=factors)
  report.update(frames=len(rows),max_tilt_deg=max(x['tilt_deg'] for x in rows),
   stopped_at_existing_15deg_observation_boundary=rows[-1]['tilt_deg']>=15,
   qualified_lift_samples={s:sum(x['feet'][s]['lifted'] for x in rows) for s in ('left','right')},
   wall_seconds=time.monotonic()-started,complete=True)
 finally:
  env.close();(a.output/'trace.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows))
  (a.output/'push-ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
  (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:report[k] for k in ('frames','max_tilt_deg','qualified_lift_samples','first_request_outside_saved_ranges','wall_seconds')}))

if __name__=='__main__':main()
