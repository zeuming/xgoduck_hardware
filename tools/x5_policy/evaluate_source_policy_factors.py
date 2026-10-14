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

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--speed',type=float,default=.3)
 p.add_argument('--factors',default='',help='comma-separated: gains,ema,slew,head,bias,reset,ranges')
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
  def process_actions(self,actions):
   super().process_actions(actions)
   target=self._processed_actions
   if 'head' in factors:target[:,list(HEAD_INDICES)]=self.held[list(HEAD_INDICES)]
   self.requested=target.clone()
   if 'ema' in factors:self.ema=c['action']['alpha']*self.ema+(1-c['action']['alpha'])*target;target=self.ema
   if 'slew' in factors:target=self.goal+torch.clamp(target-self.goal,-self.delta,self.delta)
   self.goal=target.clone();self._processed_actions=target
   self.range_failed=bool(((self.requested<self.low)|(self.requested>self.high)).any())
  def reset(self,env_ids=None):
   super().reset(env_ids)
   ids=slice(None) if env_ids is None else env_ids
   self.ema[ids]=self.held if 'head' in factors else self.home;self.goal[ids]=self.ema[ids];self.range_failed=False
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
 (a.output/'source.py').write_bytes(Path(__file__).read_bytes())
 env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);rows=[];started=time.monotonic()
 try:
  obs,_=env.reset(seed=a.seed)
  _,_,report['actual_body_properties']=effective_body_properties(env)
  term=env.action_manager.get_term('joint_pos')
  assert list(term.target_names)==c['joint_names']
  np.testing.assert_allclose(term._offset.numpy()[0],c['home_rad'],atol=1e-7,rtol=0)
  initial=measurement.actual_plant_measurements(env);report['initial']=initial
  report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  report['initial_qvel_sha256']=hashlib.sha256(env.sim.data.qvel.numpy().tobytes()).hexdigest()
  report['first_request_outside_saved_ranges']=None
  report['control_dt_s']=env.step_dt
  for k in range(round(14/env.step_dt)):
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
    command_m_s=float(raw[0,48]),failed_terms=['terminated'] if terminated[0] else [],task_complete=k==699)
   if 'ranges' in factors and term.range_failed:row['failed_terms'].append('saved_range')
   rows.append(row)
   if row['tilt_deg']>=15 or terminated[0] or truncated[0] or row['failed_terms']:break
  from evaluate_slow_gait import grade
  report['gate']=grade(rows,'forward');report['gate']['scope']='offline factor reference; not a hardware candidate'
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
