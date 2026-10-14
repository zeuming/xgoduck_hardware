#!/usr/bin/env python3
"""Bounded offline training for an explicitly new 63D slow-gait controller."""
from __future__ import annotations
import argparse
import copy
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import random
import time


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def semantic_plant(candidate):
    # Path relocation/reset-placement rounding is recorded separately. All
    # action coordinates, ranges, HOME, head, gains and source bytes stay bound.
    return {k:candidate[k] for k in ('policy_joint_ids','joint_names','observation_names',
        'action_scale','home_rad','lower_rad','upper_rad','held_head_rad','head_command_rad','action','training_scope')}


def prepare_seed(parent,output):
    import torch
    from train_constrained_policy import tensor_state_sha
    parent=Path(parent);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    from constrained_training_candidate import load_candidate,digest
    run=json.loads((parent/'run.json').read_text());candidate=load_candidate(parent/'candidate.json')
    if digest(candidate)!=run['candidate_sha256']:raise ValueError('Parent candidate binding changed')
    if sha(parent/'checkpoint.pt')!=run['checkpoint']['sha256']:raise ValueError('Parent checkpoint changed')
    state=torch.load(parent/'checkpoint.pt',weights_only=True,map_location='cpu')['actor_state_dict']
    if tensor_state_sha(state)!=run['actor_state_after_sha256']:raise ValueError('Parent actor binding changed')
    if sha(parent/'unvalidated_actor.onnx')!=run['export']['sha256']:raise ValueError('Parent ONNX changed')
    torch.save(state,output/'standing_actor.pt')
    (output/'standing_actor.onnx').write_bytes((parent/'unvalidated_actor.onnx').read_bytes())
    manifest=dict(schema_version=1,scope='actor only seed for NEW task/architecture; no optimizer/critic/KL/environment resume',
        parent_run_id=run['run_id'],parent_run_sha256=sha(parent/'run.json'),parent_checkpoint_sha256=sha(parent/'checkpoint.pt'),
        actor_tensor_sha256=tensor_state_sha(state),files={n:sha(output/n) for n in ('standing_actor.pt','standing_actor.onnx')},
        plant=semantic_plant(candidate),source_files={k:v['sha256'] for k,v in candidate['source_files'].items()},
        source_model_sha256=candidate['source_model']['sha256'],adapter_sha256=candidate['adapter_source']['sha256'],
        original_candidate_sha256=run['candidate_sha256'])
    (output/'seed.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def load_seed(path,candidate):
    import torch
    from train_constrained_policy import tensor_state_sha
    path=Path(path);manifest=json.loads((path/'seed.json').read_text())
    for name,value in manifest['files'].items():
        if name not in ('standing_actor.pt','standing_actor.onnx') or sha(path/name)!=value:raise ValueError('Seed file mismatch')
    if manifest['plant']!=semantic_plant(candidate):raise ValueError('Actor-only seed physical/coordinate contract differs')
    if manifest['source_files']!={k:v['sha256'] for k,v in candidate['source_files'].items()}:raise ValueError('Physics source bytes differ')
    if manifest['adapter_sha256']!=candidate['adapter_source']['sha256']:raise ValueError('Adapter bytes differ')
    state=torch.load(path/'standing_actor.pt',weights_only=True,map_location='cpu')
    if tensor_state_sha(state)!=manifest['actor_tensor_sha256']:raise ValueError('Seed tensor mismatch')
    return state,manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare-seed',type=Path);p.add_argument('--seed-dir',type=Path,required=True)
    p.add_argument('--candidate',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--num-envs',type=int,default=2);p.add_argument('--iterations',type=int,default=4)
    p.add_argument('--steps',type=int,default=32);p.add_argument('--epochs',type=int,default=3)
    p.add_argument('--mini-batches',type=int,default=4);p.add_argument('--max-transitions',type=int,default=256)
    p.add_argument('--wall-seconds',type=float,default=120);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--snapshot-every',type=int,default=64)
    p.add_argument('--com-transfer',action='store_true',help='Declared single reward addition for geometric support transfer')
    a=p.parse_args()
    if a.prepare_seed:
        print(json.dumps(prepare_seed(a.prepare_seed,a.seed_dir),indent=2));return
    from train_constrained_policy import install_offline_guards,require_budget,assert_state_identical,tensor_state_sha
    require_budget(a.iterations,a.steps,a.num_envs,a.epochs,a.mini_batches,a.max_transitions)
    if not 0<a.wall_seconds<=7200:raise ValueError('Explicit wall budget must be at most two hours')
    a.output.mkdir(parents=True,exist_ok=False)
    install_offline_guards(accelerator=a.device=='cuda:0')
    import numpy as np
    import torch
    from tensordict import TensorDict
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest,save_candidate
    import slow_gait_experiment as experiment
    from kl_checked_optimizer import KlCheckedOptimizer
    from ppo_update_ledger import distribution_on_rollout
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
    if a.device=='cuda:0' and not torch.cuda.is_available():raise ValueError('Requested CUDA unavailable')
    candidate=load_candidate(a.candidate);seed_state,seed_manifest=load_seed(a.seed_dir,candidate)
    cfg,rl=make_mjlab_configs(candidate);task=experiment.configure(cfg,com_transfer=a.com_transfer)
    cfg.scene.num_envs=a.num_envs;cfg.seed=a.seed;cfg.auto_reset=True
    rl.num_steps_per_env=a.steps;rl.max_iterations=a.iterations;rl.seed=a.seed
    rl.algorithm.num_learning_epochs=a.epochs;rl.algorithm.num_mini_batches=a.mini_batches
    rl.algorithm.schedule='fixed';rl.logger='tensorboard';rl.upload_model=False
    rl_cfg=asdict(rl)
    report=dict(task=task,seed=seed_manifest,candidate_sha256=digest(candidate),budgets=vars(a).copy(),
        sources={Path(__file__).name:sha(__file__),Path(experiment.__file__).name:sha(experiment.__file__)},
        walking_verified=False,hardware_eligible=False,hardware_access=False,checkpoint_resume=False)
    report['budgets']={k:str(v) if isinstance(v,Path) else v for k,v in report['budgets'].items()}
    save_candidate(candidate,a.output/'candidate.json')
    (a.output/'environment.json').write_text(json.dumps(asdict(cfg),indent=2,default=repr)+'\n')
    (a.output/'runner-config.json').write_text(json.dumps(rl_cfg,indent=2)+'\n')
    for source in (__file__,experiment.__file__):(a.output/Path(source).name).write_bytes(Path(source).read_bytes())
    start=time.monotonic();env=None;iterations=[]
    def save(stage):
        report.update(stage=stage,wall_seconds=time.monotonic()-start,iterations=len(iterations),transitions=len(iterations)*a.steps*a.num_envs)
        tmp=a.output/'run.tmp';tmp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');tmp.replace(a.output/'run.json')
    class BudgetEnd(Exception):pass
    try:
        save('constructing')
        env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
        class Wrapper(RslRlVecEnvWrapper):
            def __init__(self,e):
                self.counters={};self.observations=[];self.rewards=[]
                super().__init__(e,clip_actions=None)
            def augment(self,obs):
                obs=obs.clone(False);phase=experiment.phase_features(env)
                for key in ('actor','critic'):obs[key]=torch.cat((obs[key],phase),dim=-1)
                return obs
            def get_observations(self):return self.augment(super().get_observations())
            def step(self,actions):
                result=super().step(actions)
                for name in env.termination_manager.active_terms:
                    value=env.termination_manager.get_term(name).sum().detach()
                    self.counters[name]=self.counters.get(name,0)+value
                obs=self.augment(result[0]);self.rewards.append(result[1].mean().detach())
                if len(self.observations)<128:self.observations.append(obs['actor'].detach().cpu()[:8])
                return (obs,*result[1:])
        wrapped=Wrapper(env)
        runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=a.device)
        report['initialization']=experiment.install_actor(runner,candidate,seed_state)
        actor=runner.alg.actor
        frozen={k:v.detach().clone() for k,v in actor.mlp.base.state_dict().items()}
        sigma={k:v.detach().clone() for k,v in actor.distribution.state_dict().items()}
        checked=KlCheckedOptimizer(runner.alg.optimizer,actor.parameters(),runner.alg.critic.parameters(),
            lambda:distribution_on_rollout(runner.alg),runner.alg.desired_kl)
        old_update=runner.alg.update
        def update():checked.begin_rollout();return old_update()
        runner.alg.update=update
        def log(**kw):
            row=dict(iteration=int(kw['it']),loss={k:float(v) for k,v in kw['loss_dict'].items()},
                reward=float(torch.stack(wrapped.rewards).mean()),collect_s=kw['collect_time'],learn_s=kw['learn_time'],
                terminations={k:int(v) for k,v in wrapped.counters.items()},
                actor_lr=runner.alg.optimizer.param_groups[0]['lr'],committed_max_kl=checked.maximum_committed_kl)
            if not np.isfinite(list(row['loss'].values())).all():raise ValueError('Nonfinite losses')
            iterations.append(row);wrapped.rewards.clear();wrapped.counters.clear()
            with (a.output/'iterations.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            if len(iterations)%16==0:
                runner.save(str(a.output/'latest.pt'),infos=dict(task=task,seed=seed_manifest,kl=checked.continuation_state(),candidate_sha256=digest(candidate)))
            if a.snapshot_every>0 and len(iterations)%a.snapshot_every==0:
                snapshot=a.output/f'snapshot-{len(iterations):05d}.onnx'
                snapshot_model=torch.nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp),copy.deepcopy(actor.distribution.transform)).cpu().eval()
                example=wrapped.observations[0][:1].cpu()
                torch.onnx.export(snapshot_model,example,str(snapshot),input_names=['obs63'],output_names=['actions'],opset_version=17,dynamo=False)
                (snapshot.with_suffix('.json')).write_text(json.dumps(dict(iteration=len(iterations),sha256=sha(snapshot),candidate_sha256=digest(candidate),task=task,unvalidated=True))+'\n')
            if len(iterations)%8==0 or len(iterations)==1:
                print('PROGRESS '+json.dumps(row),flush=True);save('training')
            if time.monotonic()-start>a.wall_seconds:raise BudgetEnd()
        runner.logger.log=log
        try:runner.learn(a.iterations,init_at_random_ep_len=False)
        except BudgetEnd:report['wall_budget_reached']=True
        assert_state_identical(frozen,actor.mlp.base.state_dict());assert_state_identical(sigma,actor.distribution.state_dict())
        runner.save(str(a.output/'checkpoint.pt'),infos=dict(task=task,seed=seed_manifest,kl=checked.continuation_state(),candidate_sha256=digest(candidate)))
        saved=torch.load(a.output/'checkpoint.pt',map_location='cpu',weights_only=True)
        assert_state_identical({k:v.cpu() for k,v in actor.state_dict().items()},saved['actor_state_dict'])
        reloaded=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=a.device)
        experiment.install_actor(reloaded,candidate,seed_state)
        reload_check=KlCheckedOptimizer(reloaded.alg.optimizer,reloaded.alg.actor.parameters(),reloaded.alg.critic.parameters(),
            lambda:distribution_on_rollout(reloaded.alg),reloaded.alg.desired_kl)
        reloaded.load(str(a.output/'checkpoint.pt'),map_location='cpu')
        reload_check.restore_continuation_state(saved['infos']['kl'])
        assert_state_identical(actor.state_dict(),reloaded.alg.actor.state_dict())
        assert_state_identical(runner.alg.critic.state_dict(),reloaded.alg.critic.state_dict())
        assert_state_identical(runner.alg.optimizer.state_dict(),reloaded.alg.optimizer.state_dict())
        assert_state_identical(checked.continuation_state(),reload_check.continuation_state())
        report['full_checkpoint_reload']=True
        report['checkpoint_sha256']=sha(a.output/'checkpoint.pt');report['actor_tensor_sha256']=tensor_state_sha(actor.state_dict())
        model=torch.nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp),copy.deepcopy(actor.distribution.transform)).cpu().eval()
        rows=torch.cat(wrapped.observations).cpu();zero=rows.clone();zero[:,48:51]=0.
        with torch.no_grad():
            normalized=model[0](zero)
            expected=model[2](model[1].base(normalized[:,:61]));actual=model(zero)
            torch.testing.assert_close(expected,actual,atol=0,rtol=0)
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
        source_session=ort.InferenceSession(str(a.seed_dir/'standing_actor.onnx'),options,providers=['CPUExecutionProvider'])
        source=np.concatenate([source_session.run(None,{source_session.get_inputs()[0].name:r[None,:61].numpy()})[0] for r in zero])
        np.testing.assert_allclose(source,actual.numpy(),atol=1e-5,rtol=1e-5)
        report['zero_mapping']=dict(samples=len(zero),exact_pytorch_identity=True,parent_onnx_max_error=float(np.max(np.abs(source-actual.numpy()))),recovery_proven=False)
        torch.onnx.export(model,rows[:1],str(a.output/'candidate63.onnx'),input_names=['obs63'],output_names=['actions'],opset_version=17,dynamo=False)
        session=ort.InferenceSession(str(a.output/'candidate63.onnx'),options,providers=['CPUExecutionProvider'])
        output=np.concatenate([session.run(None,{'obs63':r[None].numpy()})[0] for r in rows])
        with torch.no_grad():expected=model(rows).numpy()
        np.testing.assert_allclose(output,expected,atol=1e-5,rtol=1e-5)
        report['export']=dict(sha256=sha(a.output/'candidate63.onnx'),observation_size=63,samples=len(rows),max_error=float(np.max(np.abs(output-expected))),production_loader_compatible=False)
        report['kl']=checked.summary();save('training_complete_requires_independent_gait_validation')
    except Exception as e:
        report['error']=repr(e);save('failed');raise
    finally:
        if env is not None:env.close()


if __name__=='__main__':main()
