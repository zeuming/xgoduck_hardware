"""Bounded source walking-policy adaptation; no hardware or production writes."""
import argparse,copy,hashlib,json,random,time
from dataclasses import asdict
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu');p.add_argument('--num-envs',type=int,default=2)
 p.add_argument('--iterations',type=int,default=2);p.add_argument('--steps',type=int,default=16);p.add_argument('--epochs',type=int,default=2)
 p.add_argument('--wall-seconds',type=float,default=180);p.add_argument('--max-transitions',type=int,default=64)
 p.add_argument('--ankle-deg',type=float,default=50.);p.add_argument('--slew',type=float,default=0.);p.add_argument('--speed',type=float,default=.3)
 p.add_argument('--penalty',type=float,default=1.);p.add_argument('--seed',type=int,default=0);p.add_argument('--snapshot-every',type=int,default=32)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 from train_constrained_policy import install_offline_guards,require_budget,assert_state_identical,tensor_state_sha
 require_budget(a.iterations,a.steps,a.num_envs,a.epochs,1,a.max_transitions)
 if not 0<a.wall_seconds<=1800:raise ValueError('Bounded source experiment at most30min')
 install_offline_guards(accelerator=a.device=='cuda:0')
 import numpy as np,torch
 from tensordict import TensorDict
 from mjlab.envs import ManagerBasedRlEnv
 from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
 from constrained_training_candidate import load_candidate,digest
 from onnx_actor_warmstart import warmstart_actor_from_onnx
 from kl_checked_optimizer import KlCheckedOptimizer
 from ppo_update_ledger import distribution_on_rollout
 import source_adaptation_experiment as ex
 torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
 if a.device!='cpu' and not torch.cuda.is_available():raise RuntimeError('No CUDA fallback')
 c=load_candidate(a.candidate);cfg,rl,spec=ex.configure(c,ankle_deg=a.ankle_deg,slew=a.slew,speed=a.speed,penalty=a.penalty)
 cfg.scene.num_envs=a.num_envs;cfg.seed=a.seed;cfg.auto_reset=True
 rl.num_steps_per_env=a.steps;rl.max_iterations=a.iterations;rl.seed=a.seed
 rl.algorithm.num_learning_epochs=a.epochs;rl.algorithm.num_mini_batches=1 if a.num_envs<16 else 4
 rl.algorithm.schedule='fixed';rl.algorithm.learning_rate=1e-4;rl.algorithm.entropy_coef=0.
 rl.actor.distribution_cfg['init_std']=.02
 rcfg=asdict(rl);env=None;start=time.monotonic();progress=[]
 report=dict(scope=__doc__,experiment=spec,budgets={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
  candidate_sha256=digest(c),original_model_sha256=c['source_model']['sha256'],hardware_eligible=False,walking_accepted=False,
  initialization='original ONNX actor only; fresh critic, frozen0.02rad Gaussian, Adam and KL state',checkpoint_resume=False,
  sources={Path(f).name:sha(f) for f in (__file__,ex.__file__)})
 for f in (__file__,ex.__file__):(a.output/Path(f).name).write_bytes(Path(f).read_bytes())
 (a.output/'environment.json').write_text(json.dumps(asdict(cfg),indent=2,default=repr))
 (a.output/'runner-config.json').write_text(json.dumps(rcfg,indent=2))
 def save(stage):
  report.update(stage=stage,wall_seconds=time.monotonic()-start,iterations=len(progress),transitions=len(progress)*a.num_envs*a.steps)
  t=a.output/'run.tmp';t.write_text(json.dumps(report,indent=2));t.replace(a.output/'run.json')
 class BudgetEnd(Exception):pass
 try:
  save('constructing');env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
  class Wrapper(RslRlVecEnvWrapper):
   def __init__(self,e):self.samples=[];self.rewards=[];self.counts={};super().__init__(e,clip_actions=None)
   def step(self,actions):
    result=super().step(actions);self.rewards.append(result[1].mean().detach())
    for name in env.termination_manager.active_terms:self.counts[name]=self.counts.get(name,0)+env.termination_manager.get_term(name).sum().detach()
    if len(self.samples)<64:self.samples.append(result[0]['actor'].detach().cpu()[:4])
    return result
  wrapped=Wrapper(env);runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rcfg),log_dir=None,device=a.device)
  actor=runner.alg.actor;report['import']=warmstart_actor_from_onnx(actor,c['source_model']['path'],expected_sha256=c['source_model']['sha256'],parity_observations=wrapped.get_observations()['actor'].cpu().numpy()[:4])
  actor.distribution.requires_grad_(False)
  sigma={k:v.clone() for k,v in actor.distribution.state_dict().items()}
  checked=KlCheckedOptimizer(runner.alg.optimizer,actor.parameters(),runner.alg.critic.parameters(),lambda:distribution_on_rollout(runner.alg),runner.alg.desired_kl)
  old=runner.alg.update
  def update():checked.begin_rollout();return old()
  runner.alg.update=update
  def export(path):
   model=torch.nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp)).cpu().eval()
   sample=torch.cat(wrapped.samples)[:256] if wrapped.samples else wrapped.get_observations()['actor'].cpu()
   torch.onnx.export(model,sample[:1],str(path),input_names=['obs61'],output_names=['actions'],opset_version=17,dynamo=False)
   import onnxruntime as ort
   opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
   session=ort.InferenceSession(str(path),opts,providers=['CPUExecutionProvider'])
   with torch.no_grad():expected=model(sample).numpy()
   actual=np.concatenate([session.run(None,{'obs61':r[None].numpy()})[0] for r in sample])
   np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=1e-5)
   meta=dict(sha256=sha(path),candidate_sha256=digest(c),experiment=spec,iterations=len(progress),max_export_error=float(abs(actual-expected).max()),hardware_eligible=False)
   path.with_suffix('.json').write_text(json.dumps(meta,indent=2));return meta
  def log(**kw):
   row=dict(iteration=int(kw['it']),reward=float(torch.stack(wrapped.rewards).mean()),loss={k:float(v) for k,v in kw['loss_dict'].items()},
    terminations={k:int(v) for k,v in wrapped.counts.items()},collect_s=kw['collect_time'],learn_s=kw['learn_time'],actor_lr=runner.alg.optimizer.param_groups[0]['lr'],max_kl=checked.maximum_committed_kl)
   progress.append(row);wrapped.rewards.clear();wrapped.counts.clear()
   with (a.output/'iterations.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
   if len(progress)%a.snapshot_every==0:export(a.output/f'snapshot-{len(progress):05d}.onnx')
   if len(progress)%8==0 or len(progress)==1:print('PROGRESS '+json.dumps(row),flush=True);save('training')
   if time.monotonic()-start>a.wall_seconds:raise BudgetEnd()
  runner.logger.log=log
  try:runner.learn(a.iterations,init_at_random_ep_len=False)
  except BudgetEnd:report['wall_budget_reached']=True
  assert_state_identical(sigma,actor.distribution.state_dict())
  runner.save(str(a.output/'checkpoint.pt'),infos=dict(experiment=spec,candidate_sha256=digest(c),kl=checked.continuation_state()))
  again=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rcfg),log_dir=None,device=a.device)
  warmstart_actor_from_onnx(again.alg.actor,c['source_model']['path'],expected_sha256=c['source_model']['sha256']);again.alg.actor.distribution.requires_grad_(False)
  ck=KlCheckedOptimizer(again.alg.optimizer,again.alg.actor.parameters(),again.alg.critic.parameters(),lambda:distribution_on_rollout(again.alg),again.alg.desired_kl)
  again.load(str(a.output/'checkpoint.pt'),map_location='cpu')
  saved=torch.load(a.output/'checkpoint.pt',map_location='cpu',weights_only=True);ck.restore_continuation_state(saved['infos']['kl'])
  assert_state_identical(actor.state_dict(),again.alg.actor.state_dict());assert_state_identical(runner.alg.critic.state_dict(),again.alg.critic.state_dict())
  assert_state_identical(runner.alg.optimizer.state_dict(),again.alg.optimizer.state_dict());assert_state_identical(checked.continuation_state(),ck.continuation_state())
  report.update(full_checkpoint_reload=True,checkpoint_sha256=sha(a.output/'checkpoint.pt'),actor_tensor_sha256=tensor_state_sha(actor.state_dict()),export=export(a.output/'actor.onnx'),kl=checked.summary())
  save('complete_requires_independent_walking_gate')
 except Exception as e:report['error']=repr(e);save('failed');raise
 finally:
  if env is not None:env.close()
if __name__=='__main__':main()
