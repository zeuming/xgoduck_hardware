"""Bounded four-task ONNX actor adaptation; no hardware or deployment access."""
from __future__ import annotations
import argparse
import copy
from dataclasses import asdict
import json
import importlib.metadata
from pathlib import Path
import random
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract', type=Path, required=True)
    p.add_argument('--mode', choices=('walk','getup','pick','roulade'), required=True)
    p.add_argument('--body', default='rear-mid')
    p.add_argument('--domains',help='Explicit balanced domain list, e.g. reference,rear-mid,rear-high; no new actor inputs')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--device', choices=('cpu','cuda:0'), default='cpu')
    p.add_argument('--num-envs', type=int, default=4)
    p.add_argument('--iterations', type=int, default=2)
    p.add_argument('--steps', type=int, default=24)
    p.add_argument('--epochs', type=int, default=5)
    p.add_argument('--wall-seconds', type=float, default=120)
    p.add_argument('--max-transitions', type=int, default=100000000)
    p.add_argument('--snapshot-every', type=int, default=64)
    p.add_argument('--initial-std', type=float, default=.02)
    p.add_argument('--learning-variant',choices=('kl-checked','source','source-warmstart'),default='kl-checked',
                   help='Legacy admission, source PPO, or one explicit warm-start distribution-initialization control.')
    p.add_argument('--source-observer',choices=('passive','off'),default='off',
                   help='Default off: CPU passive parity passed, GPU bitwise parity did not. Passive is diagnostic only.')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--resume', type=Path)
    a = p.parse_args()
    native_source = a.learning_variant in ('source', 'source-warmstart')
    domains=a.domains.split(',') if a.domains else [a.body]
    if len(set(domains))!=len(domains) or any(x not in ('reference','rear-mid','rear-high') for x in domains):
        raise ValueError('Select unique declared physical domains')
    total_envs=a.num_envs*len(domains)
    body_binding='+'.join(domains)
    from train_constrained_policy import install_offline_guards, require_budget, assert_state_identical
    require_budget(a.iterations,a.steps,total_envs,a.epochs,1,a.max_transitions)
    if not 0<a.wall_seconds<=3600 or not 0<a.initial_std<=1:
        raise ValueError('Need explicit bounded wall time and valid exploration initialization')
    a.output.mkdir(parents=True,exist_ok=False)
    install_offline_guards(accelerator=a.device=='cuda:0')
    import numpy as np
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
    from current_body_training import load_contract,configure,sha
    from onnx_actor_warmstart import warmstart_actor_from_onnx
    from kl_checked_optimizer import KlCheckedOptimizer
    from ppo_update_ledger import distribution_on_rollout
    from source_update_observer import PassiveSourceObserver
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
    if a.device!='cpu' and not torch.cuda.is_available():
        raise RuntimeError('Requested CUDA is unavailable; no CPU fallback')
    contract=load_contract(a.contract)
    configs=[];declarations={}
    for index,body in enumerate(domains):
        domain_cfg,domain_rl,decl=configure(contract,a.mode,body)
        domain_cfg.scene.num_envs=a.num_envs;domain_cfg.seed=a.seed+index
        configs.append((body,domain_cfg));declarations[body]=decl
    cfg,rl=configs[0][1],domain_rl
    declaration=declarations[domains[0]] if len(domains)==1 else dict(
        balanced_independent_domains=declarations,envs_per_domain=a.num_envs,
        new_actor_inputs=False,reward_weights_unchanged=True,
        reason='Fixed rear-high pilot regressed one mid-domain recovery. Preserve source/mid/high coverage within each PPO rollout.')
    rl.num_steps_per_env=a.steps;rl.max_iterations=a.iterations;rl.seed=a.seed
    original_ppo=asdict(rl.algorithm)
    rl.algorithm.num_learning_epochs=a.epochs
    if a.learning_variant=='kl-checked':
        rl.algorithm.num_mini_batches=1 if a.num_envs<16 else 4
        # Historical warm-start experiment; retain it as an explicit control.
        rl.actor.distribution_cfg['init_std']=a.initial_std
        rl.algorithm.schedule='fixed'
    elif a.learning_variant=='source':
        # Restore values from the pinned source config, not a sigma sweep.
        # Native std remains trainable; native adaptive scheduling/PPO update
        # is untouched. No post-step KL admission or parameter-group rewrite.
        a.initial_std=float(rl.actor.distribution_cfg['init_std'])
    else:
        # ONNX exports only the deterministic actor, not its learned sigma.
        # This control changes ONLY the initial sigma versus native source PPO;
        # it remains trainable. The selected value is an already-used legacy
        # control, NOT a recovered official training value or a real servo limit.
        # No KL admission, fixed schedule, new reward or network is introduced.
        rl.actor.distribution_cfg['init_std']=a.initial_std
    rcfg=asdict(rl)
    model=contract['models'][a.mode]
    model_path=contract['files'][a.mode]['resolved_path']
    report=dict(scope=__doc__,mode=a.mode,body_scenario=body_binding,contract_sha256=sha(a.contract),
        original_model_sha256=contract['files'][a.mode]['sha256'],
        body_identified=False,hardware_eligible=False,hardware_access=False,
        training_converged=False,simulation_accepted=False,
        initialization='official actor and frozen affine normalizer; fresh critic/Adam/exploration',
        checkpoint_resume=bool(a.resume),original_ppo=original_ppo,
        learning_changes={'variant':a.learning_variant,'exploration_initial_std':a.initial_std,
           'frozen_sigma':a.learning_variant=='kl-checked',
           'native_algorithm_parameters_preserved':asdict(rl.algorithm)==original_ppo,
           'actor_step_admission':('full-rollout KL backtracking' if a.learning_variant=='kl-checked'
               else 'none added; original PPO/adaptive schedule'),
           'source_observer':a.source_observer if native_source else None,
           'initialization_control':('only source Gaussian initial std differs; learned sigma absent from ONNX'
               if a.learning_variant=='source-warmstart' else None)},
        declaration=declaration,iterations=0,transitions=0,
        budgets={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
        sources={Path(f).name:sha(f) for f in (__file__,Path(__file__).with_name('current_body_training.py'),
                   Path(__file__).with_name('source_update_observer.py'))})
    report['versions']={n:importlib.metadata.version(n) for n in
        ('mjlab','rsl-rl-lib','torch','warp-lang','mujoco-warp','mujoco','onnx','onnxruntime','tensordict','numpy')}
    if a.device!='cpu':
        report['accelerator']={'name':torch.cuda.get_device_name(0),'cuda':torch.version.cuda,
                               'bytes':torch.cuda.get_device_properties(0).total_memory}
    def serializable(v):
        if callable(v):return v.__module__+'.'+v.__qualname__
        if isinstance(v,Path):return str(v)
        return repr(v)
    effective={name:asdict(config) for name,config in configs}
    environment_json=json.dumps(effective,sort_keys=True,default=serializable)
    (a.output/'environment.json').write_text(environment_json+'\n')
    (a.output/'runner-config.json').write_text(json.dumps(rcfg,indent=2))
    from mixed_body_training import MixedDomainEnv
    import hashlib
    context=dict(contract_sha256=report['contract_sha256'],mode=a.mode,domains=domains,
        learning_variant=a.learning_variant,source_observer=a.source_observer,
        observer_sha256=report['sources']['source_update_observer.py'],
        initial_std=a.initial_std,trainer_sha256=sha(__file__),adapter_sha256=report['sources']['current_body_training.py'],
        mixture_sha256=sha(Path(__file__).with_name('mixed_body_training.py')),
        environment_sha256=hashlib.sha256(environment_json.encode()).hexdigest(),
        runner_sha256=hashlib.sha256(json.dumps(rcfg,sort_keys=True).encode()).hexdigest(),versions=report['versions'])
    context_sha256=hashlib.sha256(json.dumps(context,sort_keys=True).encode()).hexdigest()
    report['checkpoint_binding']=context
    report['checkpoint_binding_sha256']=context_sha256
    start=time.monotonic();env=None;progress=[]
    def save(stage):
        report.update(stage=stage,wall_seconds=time.monotonic()-start,
                      iterations=len(progress),transitions=len(progress)*total_envs*a.steps)
        tmp=a.output/'run.tmp';tmp.write_text(json.dumps(report,indent=2,default=repr));tmp.replace(a.output/'run.json')
    class BudgetEnd(Exception):pass
    try:
        save('constructing_environment')
        env=(ManagerBasedRlEnv(cfg,device=a.device,render_mode=None) if len(configs)==1
             else MixedDomainEnv(configs,device=a.device,render_mode=None))
        class Wrapper(RslRlVecEnvWrapper):
            def __init__(self,e):
                self.samples=[];self.rewards=[];self.counts={};self.projections=0
                super().__init__(e,clip_actions=None)
            def step(self,actions):
                result=super().step(actions)
                self.rewards.append(result[1].mean().detach())
                for name in env.termination_manager.active_terms:
                    value=env.termination_manager.get_term(name).sum().detach()
                    self.counts[name]=self.counts.get(name,0)+value
                term=env.action_manager.get_term('joint_pos')
                self.projections+=int(term.decoder.projected.sum())
                if len(self.samples)<64:self.samples.append(result[0]['actor'].detach().cpu()[:4])
                return result
        wrapped=Wrapper(env)
        if wrapped.get_observations()['actor'].shape!=(total_envs,61):
            raise ValueError('Official 61D observation contract violated')
        runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rcfg),log_dir=None,device=a.device)
        actor=runner.alg.actor
        report['import']=warmstart_actor_from_onnx(actor,model_path,
            expected_sha256=contract['files'][a.mode]['sha256'],
            parity_observations=wrapped.get_observations()['actor'].cpu().numpy()[:4])
        def monitor(learner):
            if native_source:
                return PassiveSourceObserver(learner.alg.optimizer,
                    lambda:distribution_on_rollout(learner.alg),enabled=a.source_observer=='passive')
            learner.alg.actor.distribution.requires_grad_(False)
            return KlCheckedOptimizer(learner.alg.optimizer,learner.alg.actor.parameters(),learner.alg.critic.parameters(),
                lambda:distribution_on_rollout(learner.alg),learner.alg.desired_kl)
        checked=monitor(runner)
        report['distribution_parameters_trainable']={k:v.requires_grad for k,v in actor.distribution.named_parameters()}
        if a.resume:
            saved=torch.load(a.resume,map_location='cpu',weights_only=True)
            info=saved['infos']
            if info.get('checkpoint_binding_sha256')!=context_sha256:
                raise ValueError('Checkpoint configuration/source/version binding differs; start an explicit new experiment')
            # Load storage on CPU: load_state_dict moves parameter/moment
            # tensors to their CUDA parameters while preserving CPU Adam step
            # counters. Mapping every saved tensor to CUDA changes those.
            runner.load(str(a.resume),map_location='cpu')
            checked.restore_continuation_state(info['kl'])
            continuation=info.get('training_progress')
            if (not continuation or continuation.get('schema_version')!=1
                    or continuation.get('boundary')!='completed_rollout'):
                raise ValueError('Checkpoint lacks validated native iteration/LR/curriculum progress')
            for key in ('control_steps','next_learning_iteration'):
                if type(continuation[key]) is not int or continuation[key]<0:
                    raise ValueError('Invalid continuation progress')
            if not np.isfinite(continuation['algorithm_learning_rate']) or continuation['algorithm_learning_rate']<=0:
                raise ValueError('Invalid native adaptive learning-rate state')
            runner.alg.learning_rate=continuation['algorithm_learning_rate']
            runner.current_learning_iteration=continuation['next_learning_iteration']
            env.common_step_counter=continuation['control_steps']
            # This is explicitly a fresh physical episode. Apply source
            # step-based reward/DR curricula at the saved global clock BEFORE
            # the new reset events. Do not rewind source curricula to step0.
            env.reset()
            assert_state_identical(saved['actor_state_dict'],runner.alg.actor.state_dict())
            assert_state_identical(saved['critic_state_dict'],runner.alg.critic.state_dict())
            assert_state_identical(saved['optimizer_state_dict'],runner.alg.optimizer.state_dict())
            worlds=env.environments if hasattr(env,'environments') else [(domains[0],env)]
            assert all(e.common_step_counter==continuation['control_steps'] for _,e in worlds)
            report.update(resume_semantics='complete optimizer/monitor/actor/critic/native LR/iteration/curriculum clock; NEW physical episodes and RNG, not exact trajectory',
                restored_training_progress=copy.deepcopy(continuation),pre_update_restored_network_optimizer_exact=True,
                restored_world_control_steps={name:e.common_step_counter for name,e in worlds},
                restored_algorithm_learning_rate=runner.alg.learning_rate)
        old=runner.alg.update
        def update():checked.begin_rollout();return old()
        runner.alg.update=update
        def checkpoint(path):
            runner.save(str(path),infos=dict(contract_sha256=report['contract_sha256'],mode=a.mode,
                body_scenario=body_binding,initial_std=a.initial_std,checkpoint_binding_sha256=context_sha256,
                adapter_sha256=report['sources']['current_body_training.py'],kl=checked.continuation_state(),
                training_progress=dict(schema_version=1,boundary='completed_rollout',control_steps=env.common_step_counter,
                    next_learning_iteration=runner.current_learning_iteration+1,
                    algorithm_learning_rate=runner.alg.learning_rate)))
        def export(path):
            import onnx
            import onnxruntime as ort
            model_net=torch.nn.Sequential(copy.deepcopy(actor.obs_normalizer),copy.deepcopy(actor.mlp)).cpu().eval()
            sample=torch.cat(wrapped.samples)[:256] if wrapped.samples else wrapped.get_observations()['actor'].cpu()
            torch.onnx.export(model_net,sample[:1],str(path),input_names=['obs61'],output_names=['actions'],opset_version=17,dynamo=False)
            graph=onnx.load(str(path))
            metadata=dict(default_joint_pos=','.join(format(v,'.17g') for v in model['home_rad']),
                joint_names=','.join(model['joint_names']),observation_names=','.join(model['observation_names']),
                action_scale=str(model['action_scale']),training_contract_sha256=report['contract_sha256'],
                training_mode=a.mode,body_identified='false',hardware_eligible='false')
            onnx.helper.set_model_props(graph,metadata);onnx.save(graph,str(path))
            opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
            session=ort.InferenceSession(str(path),opts,providers=['CPUExecutionProvider'])
            with torch.no_grad():expected=model_net(sample).numpy()
            actual=np.concatenate([session.run(None,{'obs61':r[None].numpy()})[0] for r in sample])
            np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=1e-5)
            meta=dict(sha256=sha(path),iterations=len(progress),contract_sha256=report['contract_sha256'],
                      completed_learning_iterations=runner.current_learning_iteration+1,
                      export_max_abs_error=float(abs(actual-expected).max()),hardware_eligible=False)
            path.with_suffix('.json').write_text(json.dumps(meta,indent=2));return meta
        def log(**kw):
            row=dict(iteration=int(kw['it']),reward=float(torch.stack(wrapped.rewards).mean()),
                loss={k:float(v) for k,v in kw['loss_dict'].items()},
                terminations={k:int(v) for k,v in wrapped.counts.items()},
                projected_joint_goals=wrapped.projections,collect_s=kw['collect_time'],learn_s=kw['learn_time'],
                actor_lr=runner.alg.optimizer.param_groups[0]['lr'],
                max_kl=(None if native_source and a.source_observer=='off'
                        else checked.maximum_committed_kl))
            progress.append(row);wrapped.rewards.clear();wrapped.counts.clear();wrapped.projections=0
            with (a.output/'iterations.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            if len(progress)%a.snapshot_every==0:
                checkpoint(a.output/f'snapshot-{len(progress):05d}.pt')
                export(a.output/f'snapshot-{len(progress):05d}.onnx')
            if len(progress)%8==0 or len(progress)==1:
                print('PROGRESS '+json.dumps(row),flush=True);save('training')
            if time.monotonic()-start>a.wall_seconds:raise BudgetEnd()
        runner.logger.log=log
        save('training')
        try:runner.learn(a.iterations,init_at_random_ep_len=False)
        except BudgetEnd:report['wall_budget_reached']=True
        report['post_training_torch_rng_sha256']=hashlib.sha256(torch.get_rng_state().numpy().tobytes()).hexdigest()
        report['post_training_cuda_rng_sha256']=([hashlib.sha256(x.cpu().numpy().tobytes()).hexdigest()
            for x in torch.cuda.get_rng_state_all()] if a.device!='cpu' else [])
        report['post_training_distribution_state']={k:v.detach().cpu().tolist() for k,v in actor.distribution.state_dict().items()}
        report['completed_learning_iterations']=runner.current_learning_iteration+1
        report['completed_control_steps']=env.common_step_counter
        checkpoint(a.output/'checkpoint.pt')
        saved=torch.load(a.output/'checkpoint.pt',map_location='cpu',weights_only=True)
        # A fresh genuine runner also checks Adam and custom KL state. Copying
        # a live actor is invalid because its distribution caches graph tensors.
        again=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rcfg),log_dir=None,device=a.device)
        warmstart_actor_from_onnx(again.alg.actor,model_path,expected_sha256=contract['files'][a.mode]['sha256'])
        restored=monitor(again)
        again.load(str(a.output/'checkpoint.pt'),map_location='cpu')
        restored.restore_continuation_state(saved['infos']['kl'])
        assert_state_identical(actor.state_dict(),again.alg.actor.state_dict())
        assert_state_identical(runner.alg.critic.state_dict(),again.alg.critic.state_dict())
        assert_state_identical(runner.alg.optimizer.state_dict(),again.alg.optimizer.state_dict())
        assert_state_identical(checked.continuation_state(),restored.continuation_state())
        report.update(checkpoint_sha256=sha(a.output/'checkpoint.pt'),strict_network_reload=True,full_optimizer_kl_reload=True,
                      export=export(a.output/'actor.onnx'),kl=checked.summary())
        save('complete_requires_independent_evaluation')
    except Exception as e:
        report['error']=repr(e);save('failed');raise
    finally:
        if env is not None:env.close()


if __name__=='__main__':main()
