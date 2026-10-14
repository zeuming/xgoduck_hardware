#!/usr/bin/env python3
"""Bounded, offline semi-MDP PPO with 50Hz standing feedback and 5Hz residuals."""
from __future__ import annotations
import argparse
import copy
from dataclasses import asdict
import json
from pathlib import Path
import random
import time
from train_slow_gait import load_seed,sha


def validate_export(lower,actor,samples,seed_dir,output):
    import numpy as np
    import torch
    import onnxruntime as ort
    from residual_hold_experiment import HybridExport
    model=HybridExport(lower,actor).cpu().eval()
    obs,phase,held=[torch.cat([r[i] for r in samples]).cpu() for i in range(3)]
    update=(torch.arange(len(obs))%2).float()[:,None]
    # Include both commands and both update paths, with previous held corrections.
    inputs=(obs,phase,held,update)
    names=['obs61','phase2','held10','update1'];outnames=['action14','nextheld10']
    torch.onnx.export(model,tuple(x[:1] for x in inputs),str(output/'candidate-hybrid.onnx'),
        input_names=names,output_names=outnames,opset_version=17,dynamo=False)
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=ort.InferenceSession(str(output/'candidate-hybrid.onnx'),options,providers=['CPUExecutionProvider'])
    actual=[session.run(None,{n:x[i:i+1].numpy() for n,x in zip(names,inputs)}) for i in range(len(obs))]
    with torch.no_grad():expected=model(*inputs)
    errors=[]
    for j in range(2):
        value=np.concatenate([row[j] for row in actual]);reference=expected[j].numpy()
        np.testing.assert_allclose(value,reference,atol=1e-5,rtol=1e-5)
        errors.append(float(np.max(np.abs(value-reference))))
    zero=obs.clone();zero[:,48:51]=0.
    with torch.no_grad():
        zero_actual=model(zero,phase,held,update)[0]
        zero_reference=model.lower(zero,torch.zeros_like(held))
        torch.testing.assert_close(zero_actual,zero_reference,atol=0,rtol=0)
    parent=ort.InferenceSession(str(seed_dir/'standing_actor.onnx'),options,providers=['CPUExecutionProvider'])
    values=np.concatenate([parent.run(None,{parent.get_inputs()[0].name:r[None].numpy()})[0] for r in zero])
    np.testing.assert_allclose(values,zero_actual.numpy(),atol=1e-5,rtol=1e-5)
    return dict(file='candidate-hybrid.onnx',sha256=sha(output/'candidate-hybrid.onnx'),
        input_names=names,input_shapes=[[1,61],[1,2],[1,10],[1,1]],outputs=outnames,
        samples=len(obs),onnx_max_errors=errors,zero_exact_torch_identity=True,
        zero_parent_onnx_max_error=float(np.max(np.abs(values-zero_actual.numpy()))),
        zero_recovery_proven=False,production_loader_compatible=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','seed-dir','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--num-envs',type=int,default=2);p.add_argument('--iterations',type=int,default=2)
    p.add_argument('--steps',type=int,default=8,help='5Hz decisions per rollout, each has10 physical control periods')
    p.add_argument('--epochs',type=int,default=3);p.add_argument('--mini-batches',type=int,default=4)
    p.add_argument('--max-micro-transitions',type=int,default=320)
    p.add_argument('--wall-seconds',type=float,default=180);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--snapshot-every',type=int,default=64)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards,require_budget,assert_state_identical,tensor_state_sha
    from residual_hold_experiment import HOLD_STEPS
    require_budget(a.iterations,a.steps,a.num_envs,a.epochs,a.mini_batches,a.max_micro_transitions//HOLD_STEPS)
    if not 0<a.wall_seconds<=7200:raise ValueError('Bounded wall time required')
    a.output.mkdir(parents=True,exist_ok=False);install_offline_guards(accelerator=a.device=='cuda:0')
    import numpy as np
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest,save_candidate
    import slow_gait_experiment as task_source
    import residual_hold_experiment as experiment
    from kl_checked_optimizer import KlCheckedOptimizer
    from ppo_update_ledger import distribution_on_rollout
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
    candidate=load_candidate(a.candidate);seed_state,seed_manifest=load_seed(a.seed_dir,candidate)
    cfg,rl=make_mjlab_configs(candidate);task=task_source.configure(cfg,com_transfer=True)
    cfg.scene.num_envs=a.num_envs;cfg.seed=a.seed;cfg.auto_reset=True;cfg.is_finite_horizon=True
    task.update(controller=experiment.spec(candidate),actor_contract='73D upper10 leg residuals;50Hz lower61D feedback',is_finite_horizon=True)
    rl.num_steps_per_env=a.steps;rl.max_iterations=a.iterations;rl.seed=a.seed
    rl.actor.hidden_dims=(128,128)
    rl.actor.distribution_cfg=dict(class_name='GaussianDistribution',init_std=experiment.INITIAL_SIGMA_RAD/candidate['action_scale'],std_type='log')
    rl.algorithm.gamma=experiment.MICRO_GAMMA**HOLD_STEPS;rl.algorithm.lam=experiment.MICRO_LAMBDA**HOLD_STEPS
    rl.algorithm.num_learning_epochs=a.epochs;rl.algorithm.num_mini_batches=a.mini_batches
    rl.algorithm.schedule='fixed';rl.logger='tensorboard';rl.upload_model=False
    rl_cfg=asdict(rl)
    sources=[__file__,experiment.__file__,task_source.__file__]
    report=dict(task=task,seed=seed_manifest,candidate_sha256=digest(candidate),
        budgets={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
        sources={Path(s).name:sha(s) for s in sources},walking_verified=False,hardware_eligible=False,hardware_access=False,checkpoint_resume=False)
    save_candidate(candidate,a.output/'candidate.json')
    (a.output/'environment.json').write_text(json.dumps(asdict(cfg),indent=2,default=repr)+'\n')
    (a.output/'runner-config.json').write_text(json.dumps(rl_cfg,indent=2)+'\n')
    for s in sources:(a.output/Path(s).name).write_bytes(Path(s).read_bytes())
    started=time.monotonic();env=None;wrapped=None;iterations=[]
    def save(stage):
        report.update(stage=stage,wall_seconds=time.monotonic()-started,iterations=len(iterations))
        if wrapped is not None:report.update(decision_transitions=wrapped.decision_frames*a.num_envs,
            micro_transitions=wrapped.micro_frames*a.num_envs,active_reward_transitions=wrapped.active_reward_transitions)
        temp=a.output/'run.tmp';temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');temp.replace(a.output/'run.json')
    class BudgetEnd(Exception):pass
    try:
        save('constructing');env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
        lower=experiment.FrozenStandingController(candidate,seed_state,a.device)
        frozen=copy.deepcopy(lower.state_dict())
        wrapped=experiment.wrapper_class()(env,lower)
        runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=a.device)
        report['initialization']=experiment.install_upper(runner,lower,candidate)
        actor=runner.alg.actor;sigma=copy.deepcopy(actor.distribution.state_dict())
        checked=KlCheckedOptimizer(runner.alg.optimizer,actor.parameters(),runner.alg.critic.parameters(),
            lambda:distribution_on_rollout(runner.alg),runner.alg.desired_kl)
        old_update=runner.alg.update
        def update():checked.begin_rollout();return old_update()
        runner.alg.update=update
        def checkpoint(path):
            runner.save(str(path),infos=dict(task=task,seed=seed_manifest,kl=checked.continuation_state(),
                lower_tensor_sha256=tensor_state_sha(frozen),sources=report['sources'],candidate_sha256=digest(candidate)))
        def log(**kw):
            row=dict(iteration=int(kw['it']),loss={k:float(v) for k,v in kw['loss_dict'].items()},
                macro_reward=float(torch.stack(wrapped.rewards).mean()),collect_s=kw['collect_time'],learn_s=kw['learn_time'],
                terminations={k:int(v) for k,v in wrapped.counters.items()},actor_lr=runner.alg.optimizer.param_groups[0]['lr'],
                committed_max_kl=checked.maximum_committed_kl,micro_transitions=wrapped.micro_frames*a.num_envs,
                active_training_sensor_samples=int(wrapped.coverage[0]),sensor_lift_samples=wrapped.coverage[1:3].cpu().tolist(),
                moving_reward_rates={name:float(value)/max(1,int(wrapped.coverage[0])) for name,value in zip(env.reward_manager.active_terms,wrapped.reward_terms)})
            if not np.isfinite(list(row['loss'].values())).all():raise ValueError('Nonfinite losses')
            iterations.append(row);wrapped.rewards.clear();wrapped.counters.clear();wrapped.coverage.zero_();wrapped.reward_terms.zero_()
            with (a.output/'iterations.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            if len(iterations)%16==0:checkpoint(a.output/'latest.pt')
            if a.snapshot_every>0 and len(iterations)%a.snapshot_every==0:
                model=experiment.HybridExport(lower,actor).cpu().eval();s=wrapped.export_samples[0]
                snapshot=a.output/f'snapshot-{len(iterations):05d}.onnx'
                torch.onnx.export(model,tuple(x[:1] for x in s)+(torch.ones(1,1),),str(snapshot),
                    input_names=['obs61','phase2','held10','update1'],output_names=['action14','nextheld10'],opset_version=17,dynamo=False)
                snapshot.with_suffix('.json').write_text(json.dumps(dict(iteration=len(iterations),sha256=sha(snapshot),
                    candidate_sha256=digest(candidate),lower_tensor_sha256=tensor_state_sha(frozen),task=task,unvalidated=True))+'\n')
            if len(iterations)%8==0 or len(iterations)==1:print('PROGRESS '+json.dumps(row),flush=True);save('training')
            if time.monotonic()-started>a.wall_seconds:raise BudgetEnd()
        runner.logger.log=log
        try:runner.learn(a.iterations,init_at_random_ep_len=False)
        except BudgetEnd:report['wall_budget_reached']=True
        assert_state_identical(frozen,lower.state_dict());assert_state_identical(sigma,actor.distribution.state_dict())
        checkpoint(a.output/'checkpoint.pt')
        saved=torch.load(a.output/'checkpoint.pt',map_location='cpu',weights_only=True)
        reloaded=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=a.device)
        experiment.install_upper(reloaded,lower,candidate)
        reload_check=KlCheckedOptimizer(reloaded.alg.optimizer,reloaded.alg.actor.parameters(),reloaded.alg.critic.parameters(),
            lambda:distribution_on_rollout(reloaded.alg),reloaded.alg.desired_kl)
        reloaded.load(str(a.output/'checkpoint.pt'),map_location='cpu');reload_check.restore_continuation_state(saved['infos']['kl'])
        for left,right in ((actor.state_dict(),reloaded.alg.actor.state_dict()),(runner.alg.critic.state_dict(),reloaded.alg.critic.state_dict()),
            (runner.alg.optimizer.state_dict(),reloaded.alg.optimizer.state_dict()),(checked.continuation_state(),reload_check.continuation_state())):
            assert_state_identical(left,right)
        report.update(full_checkpoint_reload=True,lower_preserved=True,sigma_preserved=True,
            lower_tensor_sha256=tensor_state_sha(frozen),checkpoint_sha256=sha(a.output/'checkpoint.pt'),actor_tensor_sha256=tensor_state_sha(actor.state_dict()))
        report['export']=validate_export(lower,actor,wrapped.export_samples,a.seed_dir,a.output)
        report['kl']=checked.summary();save('training_complete_requires_independent_gait_validation')
    except Exception as e:report['error']=repr(e);save('failed');raise
    finally:
        if env is not None:env.close()


if __name__=='__main__':main()
