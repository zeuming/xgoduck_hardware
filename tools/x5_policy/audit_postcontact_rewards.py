"""Paired small-step reward audit; the independent physical acceptance gate is unchanged."""
import argparse,hashlib,importlib.util,json,random,time
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--candidate',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--baseline-rewards',action='store_true')
 p.add_argument('--measurement-source',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--ankle-deg',type=float,default=50.);p.add_argument('--slew',type=float,default=0.);p.add_argument('--speed',type=float,default=.3)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 from train_constrained_policy import install_offline_guards
 install_offline_guards()
 import torch,numpy as np,onnxruntime as ort
 from mjlab.envs import ManagerBasedRlEnv
 from constrained_training_candidate import load_candidate,digest
 from evaluate_slow_gait import grade
 import postcontact_small_step as ex
 torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 c=load_candidate(a.candidate);cfg,_,spec=ex.configure(c,ankle_deg=a.ankle_deg,slew=a.slew,speed=a.speed,warmup=5.,walk=6.,recovery=3.,training=True,small_step=not a.baseline_rewards)
 cfg.scene.num_envs=1;cfg.auto_reset=False;cfg.seed=a.seed
 opt=ort.SessionOptions();opt.intra_op_num_threads=opt.inter_op_num_threads=1
 actor=ort.InferenceSession(str(a.model),opt,providers=['CPUExecutionProvider'])
 module=importlib.util.spec_from_file_location('independent_source_measure',a.measurement_source);m=importlib.util.module_from_spec(module);module.loader.exec_module(m)
 env=None;rows=[];report=dict(experiment=spec,seed=a.seed,candidate_sha256=digest(c),model_sha256=sha(a.model),hardware_access=False,hardware_eligible=False,
   measurement_sha256=sha(a.measurement_source),sources={Path(x).name:sha(x) for x in (__file__,ex.__file__)})
 start=time.monotonic()
 try:
  env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
  correction=ex.install_correction(env) if not a.baseline_rewards else None
  obs,_=env.reset(seed=a.seed);term=env.action_manager.get_term('joint_pos')
  report['initial']=m.actual_plant_measurements(env);report['initial_qpos_sha256']=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest()
  for k in range(700):
   if time.monotonic()-start>120:raise TimeoutError('Independent gate exceeded120s')
   raw=obs['actor'].numpy().copy();command=float(raw[0,48]);action=actor.run(None,{actor.get_inputs()[0].name:raw})[0]
   obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
   failed=[name for name in env.termination_manager.active_terms if name!='time_out' and bool(env.termination_manager.get_term(name)[0])]
   measure=m.actual_plant_measurements(env)
   if measure['tilt_deg']>=15 and 'poststep15' not in failed:failed.append('poststep15')
   q=term._entity.data.joint_pos[:,term.target_ids].numpy()[0]
   saved_bad={n:bool(((v<np.array(c['lower_rad']))|(v>np.array(c['upper_rad']))).any()) for n,v in [('requested',term.requested.numpy()[0]),('goal',term.goal.numpy()[0]),('actual',q)]}
   rows.append(dict(**measure,time_s=(k+1)*env.step_dt,frame=k,command_m_s=command,failed_terms=failed,task_complete=k==699,
    action=action[0].tolist(),processed_joint_rad=term.goal.numpy()[0].tolist(),actual_joint_rad=q.tolist(),
    requested_joint_rad=term.requested.numpy()[0].tolist(),original_saved_range_conflicts=saved_bad,
    reward_terms={**{n:float(env.reward_manager._step_reward[0,i]) for i,n in enumerate(env.reward_manager.active_terms)},**({n:float(v[0]) for n,v in correction.last_terms.items()} if correction is not None else {})}))
   if failed or terminated[0] or truncated[0]:break
  g=grade(rows,'forward');g['source_domain_accepted']=g.pop('accepted')
  g['all_original_saved_ranges']=all(not any(r['original_saved_range_conflicts'].values()) for r in rows)
  g['fully_constrained_simulation_accepted']=g['source_domain_accepted'] and g['all_original_saved_ranges'] and spec['fully_constrained_domain']
  forward=[r for r in rows if r['command_m_s']>0]
  report.update(gate=g,wall_seconds=time.monotonic()-start,reward_audit=dict(forward_frames=len(forward),term_mean={n:float(np.mean([r['reward_terms'][n] for r in forward])) for n in rows[0]['reward_terms']} if forward else {},total_forward_per_second=float(np.mean([sum(r['reward_terms'].values()) for r in forward])) if forward else None))
 finally:
  if env is not None:env.close()
  (a.output/'trace.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));(a.output/'summary.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report['gate']))
if __name__=='__main__':main()
