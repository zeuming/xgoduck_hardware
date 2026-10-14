"""Independent final-600 gate using the original pinned constrained action class."""
import argparse,hashlib,importlib.util,json,random,time
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--zero-seconds',type=float,default=0.)
 p.add_argument('--candidate',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--measurement-source',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--ankle-deg',type=float,default=50.);p.add_argument('--slew',type=float,default=0.);p.add_argument('--speed',type=float,default=.3)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 from train_constrained_policy import install_offline_guards
 install_offline_guards()
 import torch,numpy as np,onnxruntime as ort
 from mjlab.envs import ManagerBasedRlEnv
 from constrained_training_candidate import load_candidate,digest
 from evaluate_slow_gait import grade
 import source_adaptation_experiment as ex
 from constrained_training_candidate import make_mjlab_configs,_range_failure
 from mjlab.managers import TerminationTermCfg
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 c=load_candidate(a.candidate);cfg,_,spec=ex.configure(c,ankle_deg=a.ankle_deg,slew=a.slew,speed=a.speed,warmup=a.zero_seconds or 5.,walk=0. if a.zero_seconds else 6.,recovery=0. if a.zero_seconds else 3.,training=False)
 if a.zero_seconds and not 5<=a.zero_seconds<=60:raise ValueError('Bounded zero test must be5–60seconds')
 frames=round(cfg.episode_length_s/(cfg.sim.mujoco.timestep*cfg.decimation))
 if a.slew!=c['action']['goal_slew_ticks_s'] or abs(a.ankle_deg-spec['final_saved_ankle_upper_deg'])>1e-9:raise ValueError('Canonical acceptance only permits the original candidate range and600 cap')
 canonical,_=make_mjlab_configs(c)
 cfg.actions['joint_pos']=canonical.actions['joint_pos']
 cfg.terminations['declared_source_range']=TerminationTermCfg(func=_range_failure)
 spec['original_constrained_action_class']=True
 cfg.scene.num_envs=1;cfg.auto_reset=False;cfg.seed=a.seed
 opt=ort.SessionOptions();opt.intra_op_num_threads=opt.inter_op_num_threads=1
 actor=ort.InferenceSession(str(a.model),opt,providers=['CPUExecutionProvider'])
 module=importlib.util.spec_from_file_location('independent_source_measure',a.measurement_source);m=importlib.util.module_from_spec(module);module.loader.exec_module(m)
 env=None;rows=[];report=dict(experiment=spec,seed=a.seed,candidate_sha256=digest(c),model_sha256=sha(a.model),hardware_access=False,hardware_eligible=False,
   measurement_sha256=sha(a.measurement_source),sources={Path(x).name:sha(x) for x in (__file__,ex.__file__)})
 start=time.monotonic()
 try:
  env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);obs,_=env.reset(seed=a.seed);term=env.action_manager.get_term('joint_pos')
  report['initial']=m.actual_plant_measurements(env);report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  for k in range(frames):
   if time.monotonic()-start>120:raise TimeoutError('Independent gate exceeded120s')
   raw=obs['actor'].numpy().copy();command=float(raw[0,48]);action=actor.run(None,{actor.get_inputs()[0].name:raw})[0]
   if action.shape!=(1,14) or not np.isfinite(action).all():raise ValueError('Nonfinite or invalid actor output')
   obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
   failed=[name for name in env.termination_manager.active_terms if name!='time_out' and bool(env.termination_manager.get_term(name)[0])]
   measure=m.actual_plant_measurements(env)
   if measure['tilt_deg']>=15 and 'poststep15' not in failed:failed.append('poststep15')
   q=term._entity.data.joint_pos[:,term.target_ids].numpy()[0]
   saved_bad={n:bool(((v<np.array(c['lower_rad']))|(v>np.array(c['upper_rad']))).any()) for n,v in [('requested',term.requested_targets.numpy()[0]),('goal',term._goal.numpy()[0]),('actual',q)]}
   rows.append(dict(**measure,time_s=(k+1)*env.step_dt,frame=k,command_m_s=command,failed_terms=failed,task_complete=k==frames-1,
    action=action[0].tolist(),processed_joint_rad=term._goal.numpy()[0].tolist(),actual_joint_rad=q.tolist(),
    requested_joint_rad=term.requested_targets.numpy()[0].tolist(),original_saved_range_conflicts=saved_bad))
   if failed or terminated[0] or truncated[0]:break
  g=grade(rows,'zero' if a.zero_seconds else 'forward');g['source_domain_accepted']=g.pop('accepted')
  if a.zero_seconds:
   first=rows[0]['root_position'];g['zero_entire_run_drift_m']=max(float(np.hypot(r['root_position'][0]-first[0],r['root_position'][1]-first[1])) for r in rows)
   g['source_domain_accepted'] &= g['zero_entire_run_drift_m']<=.015
  g['all_original_saved_ranges']=all(not any(r['original_saved_range_conflicts'].values()) for r in rows)
  g['fully_constrained_simulation_accepted']=g['source_domain_accepted'] and g['all_original_saved_ranges'] and spec['fully_constrained_domain']
  report.update(gate=g,wall_seconds=time.monotonic()-start)
 finally:
  if env is not None:env.close()
  (a.output/'trace.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));(a.output/'summary.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report['gate']))
if __name__=='__main__':main()
