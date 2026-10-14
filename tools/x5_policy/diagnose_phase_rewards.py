"""Frozen exported-policy, paired-seed CPU reward/actuator-chain diagnostic.

Uses the training stand-button command producer, no autoreset or optimizer.
This is an additional diagnostic, not the fixed seed-zero regression gate.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import random
import time

ROOT=Path(__file__).resolve().parents[2]
EVALUATOR=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','run','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--seeds',type=int,nargs='+',default=[0,1,2,3])
    parser.add_argument('--speed-scale',type=float,choices=(.25,.5,1.),default=1.,
        help='Explicit command-speed diagnostic; original validation gate stays unchanged')
    parser.add_argument('--trained-only',action='store_true')
    parser.add_argument('--audit-pushes',action='store_true')
    parser.add_argument('--push-scale',type=float,choices=(0.,1.),default=1.)
    args=parser.parse_args()
    if not 1<=len(args.seeds)<=4 or len(set(args.seeds))!=len(args.seeds):
        raise ValueError('Use up to four predetermined unique seeds')
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ['MPLCONFIGDIR']=str(args.output/'mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import onnxruntime as ort
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest,check_model_compatibility
    from stand_button_training_profile import apply_task_profile,command_at_steps
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.managers.reward_manager import RewardManager
    from simulation_evidence import effective_body_properties,audit_interval_push
    import inspect
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    candidate=load_candidate(args.candidate)
    run=json.loads((args.run/'run.json').read_text())
    if run['candidate_sha256']!=digest(candidate):raise ValueError('Different candidate')
    models=dict(baseline=Path(run['validation_baseline']['path']),trained=args.run/'unvalidated_actor.onnx')
    if args.trained_only:models.pop('baseline')
    expected=dict(baseline=run['validation_baseline']['sha256'],trained=run['export']['sha256'])
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    sessions={}
    for label,path in models.items():
        if sha(path)!=expected[label]:raise ValueError('Different '+label+' model')
        check_model_compatibility(candidate,path)
        sessions[label]=ort.InferenceSession(str(path),options,providers=['CPUExecutionProvider'])
    spec=importlib.util.spec_from_file_location('phase_measurements',EVALUATOR)
    evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
    pins={str(p):sha(p) for p in (Path(__file__),EVALUATOR,args.candidate,args.run/'run.json',
        Path(inspect.getfile(RewardManager)),Path(inspect.getfile(apply_task_profile)),Path(inspect.getfile(audit_interval_push)),*models.values())}
    report=dict(sources=pins,hardware_access=False,network_access=False,optimizer_steps=0,
        weights_changed=False,automatic_reset=False,maximum_steps_per_case=500,
        maximum_wall_seconds_per_case=90,seeds=args.seeds,candidate_sha256=digest(candidate),
        scope='frozen deterministic exports in training command profile; not historical PPO rollout reconstruction or hardware qualification',
        command_speed_scale=args.speed_scale,command_peak_m_s=.2*args.speed_scale,
        original_validation_gate_unchanged=True,
        interval_push_scale=args.push_scale,
        no_extra_action_transform=True,reward_terms='cached original weighted reward rates, not recomputed functions',
        cases=[],completed=False)
    def save():
        (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'diagnostic-source.py').write_bytes(Path(__file__).read_bytes());save()
    pairs={}
    for seed in args.seeds:
        for label,session in sessions.items():
            env=None;started=time.monotonic();rows=[]
            case=dict(seed=seed,model=label,model_sha256=expected[label],stage='constructing')
            report['cases'].append(case);save()
            try:
                torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
                cfg,_=make_mjlab_configs(candidate)
                profile=apply_task_profile(cfg,'stand-button')
                cfg.scene.num_envs=1;cfg.seed=seed;cfg.auto_reset=False
                ledger=[]
                if args.audit_pushes or args.push_scale!=1.:
                    case['push_audit']=audit_interval_push(cfg,ledger,args.push_scale)
                def serialize(value):
                    return {'callable':value.__module__+'.'+value.__qualname__} if callable(value) else str(value)
                case['configuration_sha256']=hashlib.sha256(json.dumps(asdict(cfg),sort_keys=True,default=serialize).encode()).hexdigest()
                env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
                if args.speed_scale!=1.:
                    command=env.command_manager.get_term('twist')
                    original_update=command._update_command
                    def scaled_command():
                        original_update()
                        command.vel_command_b[:,0]*=args.speed_scale
                        command.vel_command_w[:,0]*=args.speed_scale
                    command._update_command=scaled_command
                obs,_=env.reset(seed=seed)
                _,_,case['effective_body_properties']=effective_body_properties(env)
                initial=evaluation.actual_plant_measurements(env)
                term=env.action_manager.get_term('joint_pos')
                fingerprint=hashlib.sha256(obs['actor'].numpy().tobytes()+env.sim.wp_data.qpos.numpy().tobytes()+env.sim.wp_data.qvel.numpy().tobytes()).hexdigest()
                case['initial_state_sha256']=fingerprint
                pair=(fingerprint,case['configuration_sha256'])
                if seed not in pairs:pairs[seed]=pair
                elif pairs[seed]!=pair:raise ValueError('Paired environment differs')
                case['reward_weights']={name:env.reward_manager.get_term_cfg(name).weight for name in env.reward_manager.active_terms}
                for index in range(500):
                    if time.monotonic()-started>90:raise TimeoutError('Bounded diagnostic wall time')
                    actor=obs['actor'].detach().cpu().numpy().copy()
                    speed=float(command_at_steps(torch.tensor([index]),env.step_dt)[0])*args.speed_scale
                    np.testing.assert_allclose(actor[0,48:51],[speed,0,0],atol=1e-7,rtol=0)
                    raw=session.run(None,{session.get_inputs()[0].name:actor})[0]
                    obs,reward,terminated,truncated,_=env.step(torch.from_numpy(raw))
                    rates={name:float(values[0]) for name,values in env.reward_manager.get_active_iterable_terms(0)}
                    scale=env.step_dt if cfg.scale_rewards_by_dt else 1.
                    if abs(sum(rates.values())*scale-float(reward[0]))>2e-6:
                        raise ValueError('Cached reward decomposition differs from emitted reward')
                    requested=term.requested_targets[0].detach().cpu().numpy().copy()
                    goal=term._goal[0].detach().cpu().numpy().copy()
                    ema=term._ema[0].detach().cpu().numpy().copy()
                    actual=term._entity.data.joint_pos[0,term._target_ids].detach().cpu().numpy().copy()
                    violations={stage:bool((~np.isfinite(values)).any() or (values<np.asarray(candidate['lower_rad'])).any() or (values>np.asarray(candidate['upper_rad'])).any())
                        for stage,values in [('requested',requested),('applied',goal),('actual',actual)]}
                    reasons={name:bool(env.termination_manager.get_term(name)[0]) for name in env.termination_manager.active_terms}
                    plant=evaluation.actual_plant_measurements(env)
                    row=dict(index=index,time_s=(index+1)*env.step_dt,speed_m_s=speed,
                        phase='zero' if index<250 else 'forward' if index<350 else 'recovery',
                        reward=float(reward[0]),weighted_reward_rates=rates,actor_observation=actor[0].tolist(),
                        action=raw[0].tolist(),requested_rad=requested.tolist(),ema_rad=ema.tolist(),
                        goal_rad=goal.tolist(),actual_rad=actual.tolist(),range_violations=violations,
                        termination_terms=reasons,**plant)
                    rows.append(row)
                    if bool(terminated[0]) or bool(truncated[0]) or any(violations.values()) or plant['tilt_deg']>=15:break
                phases={}
                for name in ('zero','forward','recovery'):
                    group=[r for r in rows if r['phase']==name]
                    if not group:continue
                    phases[name]=dict(frames=len(group),max_tilt_deg=max(r['tilt_deg'] for r in group),
                        mean_reward_rates={term:float(np.mean([r['weighted_reward_rates'][term] for r in group])) for term in case['reward_weights']},
                        nonzero_reward_frames={term:sum(r['weighted_reward_rates'][term]!=0 for r in group) for term in case['reward_weights']},
                        lift_frames={side:sum(r['feet'][side]['lifted'] for r in group) for side in ('left','right')},
                        both_contact_frames=sum(all(r['feet'][s]['contact_count']>0 for s in ('left','right')) for r in group))
                case.update(stage='complete',steps=len(rows),phases=phases,
                    interval_pushes=ledger,
                    zero_minimum_pass=len(rows)>=250 and all(r['tilt_deg']<15 and not any(r['range_violations'].values()) and not any(r['termination_terms'].values()) for r in rows[:250]),
                    final_tilt_deg=rows[-1]['tilt_deg'],final_termination_terms=rows[-1]['termination_terms'],
                    full_command_schedule_completed=len(rows)==500,walking_qualified=False)
                print(json.dumps({k:case[k] for k in ('seed','model','steps','zero_minimum_pass','final_tilt_deg')}),flush=True)
            except BaseException as error:
                case.update(stage='failed',error=type(error).__name__+': '+str(error));raise
            finally:
                if env is not None:env.close()
                trace=args.output/f'{label}-seed-{seed}.jsonl'
                trace.write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))
                case.update(trace_sha256=sha(trace),wall_seconds=time.monotonic()-started);save()
    if any(sha(path)!=digest for path,digest in pins.items()):raise ValueError('Diagnostic sources changed')
    report['completed']=True;save()


if __name__=='__main__':main()
