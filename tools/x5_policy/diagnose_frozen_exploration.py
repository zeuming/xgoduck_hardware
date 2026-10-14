#!/usr/bin/env python3
"""Paired, optimizer-free CPU diagnostic of the existing bounded initializer.

Compares fixed deterministic output and the *existing* std=1 exploration.
Independent noise generator does not consume the environment RNG. Original
physical ranges, rewards, gains and terminations are unchanged. This is not
training, a statistical convergence claim, or hardware qualification.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import time

from train_constrained_policy import install_offline_guards, file_sha

ROOT=Path(__file__).resolve().parents[2]
EVALUATOR=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seeds',type=int,nargs='+',default=[0,1,2,3])
    parser.add_argument('--slew-matched',action='store_true',help='Compare deterministic against source-derived smaller noise, not std=1')
    args=parser.parse_args()
    if len(args.seeds)>8 or len(set(args.seeds))!=len(args.seeds):raise ValueError('Use at most eight unique predetermined seeds')
    args.output.mkdir(parents=True,exist_ok=False)
    install_offline_guards()
    import numpy as np
    import torch
    import onnxruntime as ort
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from bounded_action_distribution import ArctanActionTransform,distribution_config
    from stand_button_training_profile import apply_task_profile,command_at_steps
    from mjlab.envs import ManagerBasedRlEnv
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    candidate=load_candidate(args.candidate)
    model=Path(candidate['source_model']['path'])
    assert file_sha(model)==candidate['source_model']['sha256']
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(model),sess_options=opts,providers=['CPUExecutionProvider'])
    transform=ArctanActionTransform(distribution_config(candidate))
    from slew_matched_exploration import exploration_spec
    spec_exploration=exploration_spec(candidate) if args.slew_matched else None
    conditions=(0.,spec_exploration['initial_latent_std'] if spec_exploration else 1.)
    spec=importlib.util.spec_from_file_location('frozen_exploration_plant_measurements',EVALUATOR)
    evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
    sources={str(p.relative_to(ROOT)):file_sha(p) for p in [Path(__file__),EVALUATOR,
        ROOT/'tools/x5_policy/bounded_action_distribution.py',ROOT/'tools/x5_policy/stand_button_training_profile.py',
        ROOT/'tools/x5_policy/slew_matched_exploration.py']}
    report=dict(network_allowed=False,hardware_access=False,optimizer_steps=0,weights_modified=False,
        candidate_sha256=digest(candidate),original_model_sha256=file_sha(model),seeds=args.seeds,
        conditions=list(conditions),exploration_spec=spec_exploration,sources=sources,maximum_steps_per_case=250,
        observation_boundary_deg=15,observation_boundary_is_not_validated_fall_threshold=True,
        pairing='fresh environment and identical seed per condition; noise uses a separate fixed numpy generator',
        stochastic_samples_not_claimed_identical_to_old_torch_run=True,walking_verified=False,hardware_eligible=False,cases=[])
    def save():
        (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'diagnostic-source.py').write_bytes(Path(__file__).read_bytes())
    save()
    initial_by_seed={}
    for seed in args.seeds:
        for sigma in conditions:
            started=time.monotonic();env=None;rows=[]
            case=dict(seed=seed,sigma=sigma,stage='constructing',standing_pass=False)
            report['cases'].append(case);save()
            try:
                torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
                cfg,_=make_mjlab_configs(candidate)
                profile=apply_task_profile(cfg,'stand-button')
                cfg.scene.num_envs=1;cfg.seed=seed;cfg.auto_reset=False
                if abs(cfg.decimation*cfg.sim.mujoco.timestep-.02)>1e-12:raise ValueError('Expected original 20ms plant')
                def serialize(v):
                    return {'callable':v.__module__+'.'+v.__qualname__} if callable(v) else str(v)
                case['config_sha256']=hashlib.sha256(json.dumps(asdict(cfg),sort_keys=True,default=serialize).encode()).hexdigest()
                env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
                obs,_=env.reset(seed=seed)
                term=env.action_manager.get_term('joint_pos')
                initial=evaluation.actual_plant_measurements(env)
                case['initial']=initial
                fingerprint=hashlib.sha256(obs['actor'].numpy().tobytes()+env.sim.wp_data.qpos.numpy().tobytes()+env.sim.wp_data.qvel.numpy().tobytes()).hexdigest()
                case['initial_state_sha256']=fingerprint
                if sigma==0:initial_by_seed[seed]=(fingerprint,case['config_sha256'])
                elif initial_by_seed[seed]!=(fingerprint,case['config_sha256']):raise ValueError('Paired initial environment differs')
                noise=np.random.default_rng(seed+1024).standard_normal((250,14)).astype(np.float32)
                for step in range(250):
                    actual_command=env.command_manager.get_term('twist').command[0].detach().cpu().numpy()
                    if not np.array_equal(actual_command,np.zeros(3)):raise ValueError('Zero diagnostic command mismatch')
                    input_obs=obs['actor'].detach().cpu().numpy().copy()
                    latent=session.run(None,{session.get_inputs()[0].name:input_obs})[0]
                    action=transform(torch.from_numpy(latent+sigma*noise[step:step+1]))
                    mean_action=transform(torch.from_numpy(latent))
                    perturb_deg=((action-mean_action)*candidate['action_scale']*180/np.pi)[0]
                    legs=torch.tensor([sid not in (30,31,32,33) for sid in candidate['policy_joint_ids']])
                    obs,reward,terminated,truncated,_=env.step(action)
                    plant=evaluation.actual_plant_measurements(env)
                    reasons={name:bool(env.termination_manager.get_term(name)[0]) for name in env.termination_manager.active_terms}
                    actual=term._entity.data.joint_pos[:,term._target_ids]
                    actual_bad=bool((~torch.isfinite(actual)).any() or ((actual<term._low)|(actual>term._high)).any())
                    row=dict(step=step,seconds=(step+1)*env.step_dt,
                        input_observation=input_obs[0].tolist(),raw_latent=latent[0].tolist(),output_action=action[0].tolist(),
                        leg_request_perturb_max_deg=float(perturb_deg[legs].abs().max()),
                        reward=float(reward[0]),terminated=bool(terminated[0]),truncated=bool(truncated[0]),
                        actual_range_violation=actual_bad,termination_terms=reasons,**plant)
                    rows.append(row)
                    # Independent diagnostic stops when the existing observed
                    # boundary fails; no environment rule is loosened.
                    if row['terminated'] or row['truncated'] or actual_bad or row['tilt_deg']>=15:break
                case.update(stage='complete',steps=len(rows),seconds=len(rows)*env.step_dt,
                    max_tilt_deg=max(row['tilt_deg'] for row in rows),
                    max_leg_request_perturb_deg=max(row['leg_request_perturb_max_deg'] for row in rows),
                    mean_leg_request_perturb_deg=float(np.mean([row['leg_request_perturb_max_deg'] for row in rows])),
                    drift=evaluation.projected_displacement(initial,rows[-1]),last_reasons=rows[-1]['termination_terms'],
                    standing_pass=len(rows)==250 and not any(row['terminated'] or row['truncated'] or row['actual_range_violation'] or row['tilt_deg']>=15 for row in rows))
                print(json.dumps({k:case[k] for k in ['seed','sigma','steps','max_tilt_deg','standing_pass']}) ,flush=True)
            except BaseException as exc:
                case.update(stage='failed',error=type(exc).__name__+': '+str(exc));raise
            finally:
                if env is not None:env.close()
                case['wall_seconds']=time.monotonic()-started
                trace=args.output/f'seed-{seed}-sigma-{sigma:g}.json'
                trace.write_text(json.dumps(rows,allow_nan=False)+'\n')
                case['trace_sha256']=file_sha(trace);save()
    assert file_sha(model)==report['original_model_sha256']
    report['completed']=True;save()


if __name__=='__main__':main()
