"""Offline replay of the CURRENT LOCAL native pulse; not a deployed-X5 verification."""
import argparse,hashlib,importlib.util,json,random,time
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--candidate',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--measurement-source',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--ankle-deg',type=float,default=50.);p.add_argument('--slew',type=float,default=0.);p.add_argument('--hz',type=int,choices=(40,50),default=50)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 from train_constrained_policy import install_offline_guards
 install_offline_guards()
 import torch,numpy as np,onnxruntime as ort
 from mjlab.envs import ManagerBasedRlEnv
 from constrained_training_candidate import load_candidate,digest
 from native_pulse_grade import grade_native
 from rl_core import policy_profile
 import bounded_forward as native
 import source_adaptation_experiment as ex
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 c=load_candidate(a.candidate)
 def schedule(steps,dt,spec):
  pulse=native.ForwardPulse();pulse.begin(native.WARMUP_S,0)
  values=[pulse.command_at(float(k)*dt,0)[0] if float(k)*dt>=native.WARMUP_S else 0. for k in steps]
  return torch.tensor(values,dtype=torch.float32,device=steps.device)
 ex.schedule=schedule
 cfg,_,spec=ex.configure(c,ankle_deg=a.ankle_deg,slew=a.slew,speed=native.PEAK_M_S,warmup=native.WARMUP_S,walk=native.DURATION_S,recovery=native.RECOVERY_S,training=False)
 cfg.decimation=round(1/a.hz/cfg.sim.mujoco.timestep)
 assert abs(cfg.sim.mujoco.timestep*cfg.decimation-1/a.hz)<1e-10
 frames=round((native.WARMUP_S+native.DURATION_S+native.RECOVERY_S)*a.hz)
 spec['native_local_pulse']=dict(source_sha256=sha(native.__file__),peak_m_s=native.PEAK_M_S,duration_s=native.DURATION_S,ramp_s=native.RAMP_S,warmup_s=native.WARMUP_S,recovery_s=native.RECOVERY_S,control_hz=a.hz,deployed_version_verified=False)

 cfg.scene.num_envs=1;cfg.auto_reset=False;cfg.seed=a.seed
 opt=ort.SessionOptions();opt.intra_op_num_threads=opt.inter_op_num_threads=1
 actor=ort.InferenceSession(str(a.model),opt,providers=['CPUExecutionProvider'])
 profile=policy_profile(actor);np.testing.assert_array_equal(profile['home_rad'],c['home_rad']);assert profile['scale']==c['action_scale']
 module=importlib.util.spec_from_file_location('independent_source_measure',a.measurement_source);m=importlib.util.module_from_spec(module);module.loader.exec_module(m)
 env=None;rows=[];report=dict(experiment=spec,seed=a.seed,candidate_sha256=digest(c),model_sha256=sha(a.model),hardware_access=False,hardware_eligible=False,
   measurement_sha256=sha(a.measurement_source),sources={Path(x).name:sha(x) for x in (__file__,ex.__file__,native.__file__,Path(__file__).with_name('native_pulse_grade.py'))})
 start=time.monotonic()
 try:
  env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);obs,_=env.reset(seed=a.seed);term=env.action_manager.get_term('joint_pos')
  report['initial']=m.actual_plant_measurements(env);report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  for k in range(frames):
   if time.monotonic()-start>120:raise TimeoutError('Independent gate exceeded120s')
   raw=obs['actor'].numpy().copy();command=float(raw[0,48]);action=actor.run(None,{actor.get_inputs()[0].name:raw})[0]
   obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
   failed=[name for name in env.termination_manager.active_terms if name!='time_out' and bool(env.termination_manager.get_term(name)[0])]
   measure=m.actual_plant_measurements(env)
   if measure['tilt_deg']>=15 and 'poststep15' not in failed:failed.append('poststep15')
   q=term._entity.data.joint_pos[:,term.target_ids].numpy()[0]
   saved_bad={n:bool(((v<np.array(c['lower_rad']))|(v>np.array(c['upper_rad']))).any()) for n,v in [('requested',term.requested.numpy()[0]),('goal',term.goal.numpy()[0]),('actual',q)]}
   rows.append(dict(**measure,time_s=(k+1)*env.step_dt,frame=k,command_m_s=command,failed_terms=failed,task_complete=k==frames-1,
    action=action[0].tolist(),processed_joint_rad=term.goal.numpy()[0].tolist(),actual_joint_rad=q.tolist(),
    requested_joint_rad=term.requested.numpy()[0].tolist(),original_saved_range_conflicts=saved_bad))
   if failed or terminated[0] or truncated[0]:break
  g=grade_native(rows,a.hz);g['native_pulse_accepted']=g.pop('accepted')
  g['all_original_saved_ranges']=all(not any(r['original_saved_range_conflicts'].values()) for r in rows)
  g['fully_constrained_native_pulse_accepted']=g['native_pulse_accepted'] and g['all_original_saved_ranges'] and spec['fully_constrained_domain']
  report.update(gate=g,wall_seconds=time.monotonic()-start)
 finally:
  if env is not None:env.close()
  (a.output/'trace.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));(a.output/'summary.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report['gate']))
if __name__=='__main__':main()
