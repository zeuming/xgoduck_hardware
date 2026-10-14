#!/usr/bin/env python3
"""Offline PPO entry point for the pinned constrained XgoDuck candidate.

The default 32 environment steps verify training/checkpoint/export plumbing.
They do not establish convergence, walking, or eligibility for hardware.
No network logging, downloads, remote jobs, serial access or deployment exists.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict
import hashlib
import importlib.metadata
import inspect
import json
import os
from pathlib import Path
import random
import re
import socket
import sys
import time
import traceback
import uuid


class TrainingTimeBudgetReached(Exception):
    """Graceful stop after a complete PPO iteration, before checkpoint/export."""


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def allowed_training_device(path, accelerator=False):
    if path in ('/dev/null','/dev/urandom','/dev/random'):
        return True
    return bool(accelerator and (re.fullmatch(r'/dev/nvidia[0-9]+',path)
        or path in ('/dev/nvidiactl','/dev/nvidia-uvm','/dev/nvidia-uvm-tools')
        or re.fullmatch(r'/dev/nvidia-caps/nvidia-cap[0-9]+',path)))


def install_offline_guards(*, accelerator=False):
    os.environ.update(WANDB_MODE='disabled',WANDB_DISABLED='true',HF_HUB_OFFLINE='1',
                      TRANSFORMERS_OFFLINE='1',MPLBACKEND='Agg',WORLD_SIZE='1')
    def deny_network(*args,**kwargs):
        raise RuntimeError('Network connections are disabled for this offline training run')
    socket.create_connection=deny_network
    socket.socket.connect=deny_network
    socket.socket.connect_ex=deny_network
    socket.socket.sendto=deny_network
    def audit(event,values):
        if event in ('socket.connect','socket.sendto'):
            deny_network()
        if event=='open' and values and isinstance(values[0],(str,bytes)):
            path=os.path.realpath(os.fsdecode(values[0]))
            if path.startswith('/dev/') and not allowed_training_device(path,accelerator):
                raise PermissionError('Training cannot open robot or unselected device files')
    sys.addaudithook(audit)


def tensor_state_sha(state):
    result=hashlib.sha256()
    for name,value in sorted(state.items()):
        value=value.detach().cpu().contiguous()
        result.update(name.encode());result.update(str(value.dtype).encode())
        result.update(str(tuple(value.shape)).encode());result.update(value.numpy().tobytes())
    return result.hexdigest()


def optimizer_steps(optimizer):
    return sorted({int(value['step'].item() if hasattr(value['step'],'item') else value['step'])
                   for value in optimizer.state.values() if 'step' in value})


def assert_state_identical(expected,actual):
    """Check all checkpoint tensors and scalar settings, not only step count."""
    import torch
    if torch.is_tensor(expected):
        if not torch.is_tensor(actual):raise ValueError('Reloaded checkpoint value type differs')
        torch.testing.assert_close(expected,actual,atol=0,rtol=0)
    elif isinstance(expected,dict):
        if not isinstance(actual,dict) or expected.keys()!=actual.keys():
            raise ValueError('Reloaded checkpoint dictionary keys differ')
        for key,value in expected.items():assert_state_identical(value,actual[key])
    elif isinstance(expected,(tuple,list)):
        if type(expected) is not type(actual) or len(expected)!=len(actual):
            raise ValueError('Reloaded checkpoint sequence differs')
        for before,after in zip(expected,actual):assert_state_identical(before,after)
    elif type(expected) is not type(actual) or expected!=actual:
        raise ValueError('Reloaded checkpoint scalar differs')


def require_budget(iterations,steps,num_envs,epochs,mini_batches,max_env_steps):
    values=(iterations,steps,num_envs,epochs,mini_batches,max_env_steps)
    if any(type(x) is not int or x<=0 for x in values):
        raise ValueError('Iteration, rollout, environment, optimizer and budget counts must be positive integers')
    total=iterations*steps*num_envs
    if total>max_env_steps:
        raise ValueError(f'Requested {total} environment steps exceeds explicit budget {max_env_steps}')
    if steps*num_envs<2 or steps*num_envs%mini_batches:
        raise ValueError('Rollout must have at least two samples and divide evenly into mini-batches')
    return total


def attach_bound_export(runner,candidate,checkpoint,output,run_id,step_count,observations,distribution_report=None,task_profile=None,experiment_report=None):
    """Export actual post-update actor weights, bind provenance and verify ORT."""
    import numpy as np
    import onnxruntime as ort
    import torch
    from tensordict import TensorDict
    from mjlab.rl.exporter_utils import get_base_metadata,attach_metadata_to_onnx
    from constrained_training_candidate import training_export_metadata,check_model_compatibility,digest
    actor=runner.get_inference_policy(device='cpu')
    runner.export_policy_to_onnx(str(output.parent),output.name)
    metadata=get_base_metadata(runner.env.unwrapped,run_path=str(checkpoint))
    if metadata['joint_names']!=candidate['joint_names'] or metadata['observation_names']!=candidate['observation_names']:
        raise ValueError('Actual export joint/observation order differs from candidate')
    np.testing.assert_allclose(metadata['default_joint_pos'],candidate['home_rad'],atol=1e-6,rtol=0)
    if not np.all(np.asarray(metadata['action_scale'])==candidate['action_scale']):
        raise ValueError('Actual export action scale differs from candidate')
    # Upstream defaults to rounding lists to three decimals. Preserve exact
    # metadata HOME instead of silently changing the decoder contract here.
    metadata.update(joint_names=','.join(candidate['joint_names']),
        observation_names=','.join(candidate['observation_names']),
        default_joint_pos=','.join(format(v,'.17g') for v in candidate['home_rad']),
        action_scale=format(candidate['action_scale'],'.17g'),
        **training_export_metadata(candidate),
        constrained_training_run_id=run_id,constrained_training_checkpoint_sha256=file_sha(checkpoint),
        constrained_training_optimizer_steps=str(step_count),training_converged='false',
        walking_verified='false',hardware_eligible='false')
    if distribution_report is not None:
        metadata['constrained_training_distribution']=json.dumps(distribution_report,sort_keys=True,separators=(',',':'))
        metadata['constrained_training_distribution_sha256']=digest(distribution_report)
    if task_profile is not None:
        metadata['constrained_training_task_profile']=json.dumps(task_profile,sort_keys=True,separators=(',',':'))
        metadata['constrained_training_task_profile_sha256']=digest(task_profile)
    if experiment_report is not None:
        metadata['constrained_forward_experiment']=json.dumps(experiment_report,sort_keys=True,separators=(',',':'))
        metadata['constrained_forward_experiment_sha256']=digest(experiment_report)
    attach_metadata_to_onnx(str(output),metadata)
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=ort.InferenceSession(str(output),options,providers=['CPUExecutionProvider'])
    sample=np.asarray(observations,dtype=np.float32).reshape(-1,61)
    if experiment_report is not None:
        # Compact telemetry retains an early prefix, potentially all zero-speed.
        # Validate the exported residual's command path too, not only its gate=0.
        command_samples=[]
        for speed in (0.,.05,.1,.2):
            rows=sample.copy();rows[:,48:51]=0.;rows[:,48]=speed
            command_samples.append(rows)
        sample=np.concatenate(command_samples)
    with torch.inference_mode():
        expected=actor(TensorDict({'actor':torch.from_numpy(sample)},batch_size=[len(sample)])).numpy()
    actual=np.concatenate([session.run(None,{session.get_inputs()[0].name:row[None]})[0] for row in sample])
    np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=1e-5)
    return dict(path=str(output),sha256=file_sha(output),checkpoint_sha256=file_sha(checkpoint),
        candidate_sha256=digest(candidate),source='actual_post_optimizer_checkpoint_actor',
        observations_compared=len(sample),torch_onnx_max_absolute_error=float(np.max(np.abs(actual-expected))),
        metadata=check_model_compatibility(candidate,output),hardware_eligible=False,walking_verified=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--iterations',type=int,default=2)
    parser.add_argument('--steps-per-env',type=int,default=16)
    parser.add_argument('--num-envs',type=int,default=1)
    parser.add_argument('--epochs',type=int,default=1)
    parser.add_argument('--mini-batches',type=int,default=1)
    parser.add_argument('--max-env-steps',type=int,default=64)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--device',choices=('cpu','cuda:0'),default='cpu',help='Explicit accelerator; never silently falls back from CUDA to CPU')
    parser.add_argument('--compact-telemetry',action='store_true',help='Reduce per-step counters on device; preserve full counts without storing every observation')
    parser.add_argument('--max-export-observations',type=int,help='Optional deterministic prefix for export parity; does not change training observations')
    parser.add_argument('--max-training-seconds',type=float,
        help='Optional wall budget checked after each complete PPO iteration; checkpoints and export are still verified')
    parser.add_argument('--warm-start-onnx',type=Path,
        help='Optional actor-only import; critic/std/optimizer remain newly initialized, never checkpoint resume')
    parser.add_argument('--bounded-exploration',action='store_true',
        help='Use an explicit new policy with HOME-anchored arctan bounds on leg actions; preserve physical range guards')
    parser.add_argument('--exploration-initialization',choices=('upstream','goal-slew'),default='upstream',
        help='Opt-in initial sigma from existing per-period target slew; never changes physical limits')
    parser.add_argument('--task-profile',choices=['upstream-velocity','stand-button'],default='upstream-velocity')
    parser.add_argument('--validation-baseline-onnx',type=Path,help='Required for stand-button; independently re-evaluated zero-speed baseline')
    parser.add_argument('--regression-output',type=Path,help='Optional new directory for independent stand-button baseline/candidate gate')
    parser.add_argument('--update-ledger',action='store_true',help='Observe the first 20 optimizer updates on the complete rollout without sampling or changing updates')
    parser.add_argument('--kl-checked-updates',action='store_true',help='New PPO variant: rollback and backtrack actor Adam steps against the existing desired KL on the entire rollout')
    parser.add_argument('--resume-run',type=Path,help='Verified completed run: restore actor/critic/Adam/KL state in a fresh environment, not an exact trajectory replay')
    parser.add_argument('--phase-ledger',action='store_true',help='Record original reward and termination coverage per command phase before automatic reset')
    parser.add_argument('--forward-experiment',choices=('original','relative-v1'),
        help='New experiment: frozen source actor plus forward residual, with original or task-scaled reward')
    parser.add_argument('--actor-seed-run',type=Path,
        help='Only for forward experiment: verified source actor and sigma, explicitly fresh critic/Adam/KL state')
    args=parser.parse_args(argv)
    if bool(args.forward_experiment)!=bool(args.actor_seed_run):
        raise ValueError('Forward experiment and actor seed run must be supplied together')
    if args.forward_experiment and (args.resume_run or not args.kl_checked_updates or not args.warm_start_onnx or args.task_profile!='stand-button'):
        raise ValueError('New forward experiment requires stand-button, original normalizer and KL checks; never matching-config resume')
    if args.resume_run and (not args.kl_checked_updates or not args.warm_start_onnx or args.task_profile!='stand-button'):
        raise ValueError('Continuation requires KL checks, original normalizer initialization and stand-button profile')
    if args.phase_ledger and args.task_profile!='stand-button':
        raise ValueError('Phase ledger requires the pinned stand-button task')
    if args.kl_checked_updates and (not args.bounded_exploration or args.update_ledger):
        raise ValueError('KL-checked updates require bounded exploration and provide their own update ledger')
    if args.max_export_observations is not None and args.max_export_observations<1:
        raise ValueError('Export observation cap must be positive')
    if args.device!='cpu' and (not args.compact_telemetry or args.max_export_observations is None):
        raise ValueError('CUDA requires explicit compact telemetry and an export observation cap')
    if args.task_profile=='stand-button' and args.validation_baseline_onnx is None:
        raise ValueError('stand-button requires an explicit independent validation baseline ONNX')
    if args.exploration_initialization=='goal-slew' and not args.bounded_exploration:
        raise ValueError('Goal-slew initialization requires the verified bounded action distribution')
    if args.max_training_seconds is not None and (not 0<args.max_training_seconds<float('inf')):
        raise ValueError('Training wall budget must be finite and positive')
    total=require_budget(args.iterations,args.steps_per_env,args.num_envs,args.epochs,args.mini_batches,args.max_env_steps)
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'runner-source.py').write_bytes(Path(__file__).read_bytes())
    install_offline_guards(accelerator=args.device!='cpu')
    started=time.monotonic();env=None
    report=dict(schema_version=1,run_id=str(uuid.uuid4()),candidate_path=str(args.candidate.resolve()),
        runner_source_sha256=file_sha(__file__),network_allowed=False,hardware_access=False,
        model_uploaded=False,training_started=False,optimizer_steps=0,environment_steps=0,
        training_converged=False,walking_verified=False,hardware_eligible=False,
        artifact_scope='bounded_training_pipeline_smoke' if total<=64 else 'unaccepted_offline_training',
        checkpoint_resume=False,initialization='actor_only_onnx_warmstart' if args.warm_start_onnx else 'explicit_random_initialization',
        bounded_exploration=args.bounded_exploration,
        budget=dict(iterations=args.iterations,steps_per_env=args.steps_per_env,num_envs=args.num_envs,
            environment_steps=total,max_environment_steps=args.max_env_steps,
            learning_epochs=args.epochs,mini_batches=args.mini_batches,max_training_seconds=args.max_training_seconds))
    def save(stage):
        report.update(stage=stage,total_wall_seconds=time.monotonic()-started)
        (args.output/'run.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('TRAINING_STAGE '+stage,flush=True)
    try:
        save('load_pinned_candidate')
        import numpy as np
        import torch
        if args.device!='cpu' and not torch.cuda.is_available():
            raise RuntimeError('CUDA was requested but is unavailable; no CPU fallback is permitted')
        from constrained_training_candidate import load_candidate,make_mjlab_configs,digest,save_candidate
        from mjlab.envs import ManagerBasedRlEnv
        from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
        candidate=load_candidate(args.candidate)
        report['candidate_sha256']=digest(candidate)
        save_candidate(candidate,args.output/'candidate.json')
        report['original_model_sha256']=file_sha(candidate['source_model']['path'])
        report['candidate_file_sha256']=file_sha(args.candidate)
        report['dependencies']={name:importlib.metadata.version(name) for name in
            ('mjlab','rsl-rl-lib','torch','warp-lang','mujoco-warp','mujoco','onnx','onnxruntime')}
        report['api_sources']={name:dict(path=inspect.getfile(value),sha256=file_sha(inspect.getfile(value)))
            for name,value in [('runner',MjlabOnPolicyRunner),('wrapper',RslRlVecEnvWrapper),('environment',ManagerBasedRlEnv)]}
        if args.warm_start_onnx and file_sha(args.warm_start_onnx)!=candidate['source_model']['sha256']:
            raise ValueError('Warm-start model differs from the candidate source model')
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        torch.manual_seed(args.seed);np.random.seed(args.seed);random.seed(args.seed)
        cfg,rl=make_mjlab_configs(candidate)
        if args.exploration_initialization=='goal-slew':
            from slew_matched_exploration import exploration_spec
            exploration_source=Path(inspect.getfile(exploration_spec))
            report['exploration_initialization']=dict(**exploration_spec(candidate),
                source_sha256=file_sha(exploration_source))
            (args.output/'exploration-initialization-source.py').write_bytes(exploration_source.read_bytes())
            rl.actor.distribution_cfg['init_std']=report['exploration_initialization']['initial_latent_std']
        from stand_button_training_profile import apply_task_profile,PULSE_SOURCE
        report['task_profile']=apply_task_profile(cfg,args.task_profile)
        report['task_profile_sha256']=digest(report['task_profile'])
        if args.forward_experiment:
            from forward_learning_experiment import apply_reward_experiment,initialize_from_actor_run,install_residual,audit_zero_mapping
            experiment_source=Path(inspect.getfile(apply_reward_experiment))
            report['forward_experiment']=dict(source_sha256=file_sha(experiment_source),
                reward=apply_reward_experiment(cfg,args.forward_experiment),checkpoint_resume=False)
            (args.output/'forward-experiment-source.py').write_bytes(experiment_source.read_bytes())
            # The command profile still makes no reward changes. This separate
            # experiment declaration binds the explicit override afterwards.
            report['forward_experiment_sha256']=digest(report['forward_experiment'])
        if args.task_profile=='stand-button':
            profile_path=Path(inspect.getfile(apply_task_profile))
            (args.output/'task-profile-source.py').write_bytes(profile_path.read_bytes())
            (args.output/'task-button-source.py').write_bytes(PULSE_SOURCE.read_bytes())
            report['validation_baseline']=dict(path=str(args.validation_baseline_onnx.resolve()),sha256=file_sha(args.validation_baseline_onnx))
            from validate_training_regression import EVALUATOR
            gate_source=Path(__file__).with_name('validate_training_regression.py')
            report['validation_sources']={str(path):file_sha(path) for path in (gate_source,EVALUATOR)}
            (args.output/'regression-gate-source.py').write_bytes(gate_source.read_bytes())
            (args.output/'independent-evaluator-source.py').write_bytes(EVALUATOR.read_bytes())
        report['upstream_ppo_defaults']=asdict(rl.algorithm)
        cfg.scene.num_envs=args.num_envs;cfg.seed=args.seed;cfg.auto_reset=True
        rl.num_steps_per_env=args.steps_per_env;rl.max_iterations=args.iterations;rl.seed=args.seed
        rl.algorithm.num_learning_epochs=args.epochs;rl.algorithm.num_mini_batches=args.mini_batches
        if args.kl_checked_updates:
            # Post-step admission replaces the upstream pre-step-only heuristic.
            # Keep its configured desired KL and the original critic learning rate.
            rl.algorithm.schedule='fixed'
        rl.logger='tensorboard';rl.upload_model=False
        if rl.clip_actions is not None:raise ValueError('Action clipping would violate the constrained adapter contract')
        rl_cfg=asdict(rl)
        def config_value(value):
            if callable(value):return {'callable':value.__module__+'.'+value.__qualname__}
            if isinstance(value,Path):return str(value)
            return repr(value)
        (args.output/'effective-environment-config.json').write_text(json.dumps(asdict(cfg),indent=2,default=config_value,allow_nan=False)+'\n')
        (args.output/'runner-config.json').write_text(json.dumps(rl_cfg,indent=2,allow_nan=False)+'\n')
        report['configuration']=dict(device=args.device,seed=args.seed,control_dt_s=cfg.decimation*cfg.sim.mujoco.timestep,
            physics_dt_s=cfg.sim.mujoco.timestep,automatic_episode_reset=True,
            external_logger=None,upload_model=False,critic_std_optimizer='newly_initialized',
            compact_telemetry=args.compact_telemetry,max_export_observations=args.max_export_observations)
        if args.device!='cpu':
            report['accelerator']=dict(name=torch.cuda.get_device_name(0),cuda_version=torch.version.cuda,
                                      total_memory_bytes=torch.cuda.get_device_properties(0).total_memory)
        save('construct_actual_'+args.device.replace(':','_')+'_environment')
        env=ManagerBasedRlEnv(cfg,device=args.device,render_mode=None)
        phase_ledger=None
        if args.phase_ledger:
            from phase_training_ledger import PhaseTrainingLedger
            phase_source=Path(inspect.getfile(PhaseTrainingLedger))
            report['phase_ledger_source_sha256']=file_sha(phase_source)
            (args.output/'phase-ledger-source.py').write_bytes(phase_source.read_bytes())
            phase_ledger=PhaseTrainingLedger(env)
        class CountedWrapper(RslRlVecEnvWrapper):
            def __init__(self,environment):
                self.transitions=[];self.actor_observations=[]
                super().__init__(environment,clip_actions=None)
                self._inside_counted_step=False
                term=environment.action_manager.get_term('joint_pos')
                original_apply=term.apply_actions
                original_reset=environment._reset_idx
                def observe_actual():
                    if self._inside_counted_step:
                        actual=term._entity.data.joint_pos[:,term._target_ids]
                        self._actual_invalid |= (~torch.isfinite(actual)).any(dim=-1) | ((actual<term._low)|(actual>term._high)).any(dim=-1)
                def observed_apply():
                    observe_actual()
                    if self._inside_counted_step:
                        goal=term._processed_actions
                        self._applied_invalid |= (~torch.isfinite(goal)).any(dim=-1) | ((goal<term._low)|(goal>term._high)).any(dim=-1)
                    return original_apply()
                def observed_reset(env_ids=None):
                    # Preserve the final integrated position before automatic
                    # reset can erase evidence of a physical joint excursion.
                    observe_actual()
                    return original_reset(env_ids)
                term.apply_actions=observed_apply
                environment._reset_idx=observed_reset
                self._observe_actual=observe_actual
            def remember_observations(self,obs):
                remaining=(len(obs) if args.max_export_observations is None else
                           max(0,args.max_export_observations-len(self.actor_observations)))
                if remaining:
                    self.actor_observations.extend(obs[:remaining].detach().cpu().numpy().copy())
            def values(self,value):
                if args.compact_telemetry:return [int(value.sum())]
                return value.detach().cpu().tolist()
            def step(self,actions):
                # Observe the exact effective request before the plant can hold
                # or reset it. This does not modify actions or any guard.
                term=env.action_manager.get_term('joint_pos')
                requested=torch.where(term._mask,term._held,term._home+candidate['action_scale']*actions)
                request_invalid=(~torch.isfinite(requested)).any(dim=-1) | ((requested<term._low)|(requested>term._high)).any(dim=-1)
                self._actual_invalid=torch.zeros(self.num_envs,device=env.device,dtype=torch.bool)
                self._applied_invalid=torch.zeros_like(self._actual_invalid)
                self._inside_counted_step=True
                try:
                    result=super().step(actions)
                    self._observe_actual()
                finally:
                    self._inside_counted_step=False
                obs,reward,done,_=result
                self.remember_observations(obs['actor'])
                # The pinned manager retains each computed term through reset;
                # the action term's range_failed itself is cleared on reset.
                # Read retained termination terms so autoreset cannot hide a
                # rejected rollout behind fresh next-episode observations.
                reasons={name:self.values(env.termination_manager.get_term(name))
                    for name in env.termination_manager.active_terms}
                self.transitions.append(dict(step=len(self.transitions),
                    reward=[float(reward.mean())] if args.compact_telemetry else reward.detach().cpu().tolist(),
                    done=self.values(done),done_count=int(done.bool().sum()),finite_actions=bool(torch.isfinite(actions).all()),
                    requested_target_invalid=self.values(request_invalid),
                    applied_target_invalid=self.values(self._applied_invalid),
                    actual_joint_invalid=self.values(self._actual_invalid),
                    range_failure_without_bad_request=int((env.termination_manager.get_term('saved_range_violation') & ~request_invalid).sum()),
                    termination_terms=reasons,
                    timed_out=self.values(env.termination_manager.time_outs)))
                report['environment_steps']+=self.num_envs
                return result
        wrapped=CountedWrapper(env)
        obs=wrapped.get_observations()
        if list(obs['actor'].shape)!=[args.num_envs,61]:raise ValueError('Expected the genuine 61-dimensional actor observation')
        if list(env.observation_manager.active_terms['actor'])!=candidate['observation_names']:
            raise ValueError('Actor observation terms differ from candidate')
        wrapped.remember_observations(obs['actor'])
        runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=args.device)
        if args.warm_start_onnx:
            from onnx_actor_warmstart import warmstart_actor_from_onnx
            report['warmstart_importer_sha256']=file_sha(inspect.getfile(warmstart_actor_from_onnx))
            (args.output/'warmstart-importer-source.py').write_bytes(Path(inspect.getfile(warmstart_actor_from_onnx)).read_bytes())
            report['warmstart']=warmstart_actor_from_onnx(runner.alg.get_policy(),args.warm_start_onnx,
                expected_sha256=candidate['source_model']['sha256'],parity_observations=obs['actor'].detach().cpu().numpy())
        actor=runner.alg.get_policy()
        if args.bounded_exploration:
            from bounded_action_distribution import install_bounded_action_distribution
            distribution_path=Path(inspect.getfile(install_bounded_action_distribution))
            report['distribution_source_sha256']=file_sha(distribution_path)
            (args.output/'distribution-source.py').write_bytes(distribution_path.read_bytes())
            with torch.inference_mode():original_mean=actor(obs).detach().clone()
            old_parameters={id(parameter) for group in runner.alg.optimizer.param_groups for parameter in group['params']}
            report['bounded_distribution']=install_bounded_action_distribution(actor,candidate)
            new_parameters={id(parameter) for model in (runner.alg.actor,runner.alg.critic) for parameter in model.parameters()}
            if old_parameters!=new_parameters:
                raise ValueError('Installing the bounded distribution orphaned optimizer parameter references')
            with torch.inference_mode():new_mean=actor(obs).detach()
            delta=new_mean-original_mean
            report['bounded_initialization']=dict(kind='new_bounded_policy_initialized_from_original_actor_network',
                original_network_parity_is_before_output_transform=True,original_final_actions_preserved=False,
                initial_real_observations=len(delta),maximum_action_change=float(delta.abs().max()),
                rms_action_change=float(torch.sqrt((delta*delta).mean())),optimizer_parameters_preserved=True)
        if args.forward_experiment:
            report['forward_experiment']['actor_initialization']=initialize_from_actor_run(args.actor_seed_run,runner,
                report,args.output/'effective-environment-config.json')
            source_actor_state={name:value.detach().clone() for name,value in actor.state_dict().items()}
            with torch.inference_mode():seed_output=actor(obs).detach().clone()
            report['forward_experiment']['architecture']=install_residual(runner,candidate)
            with torch.inference_mode():assert_state_identical(seed_output,actor(obs))
            report['forward_experiment']['initial_output_exact_identity']=True
            report['forward_experiment_sha256']=digest(report['forward_experiment'])
            report['initialization']='new_forward_residual_experiment_from_verified_actor_only'
            report['configuration']['critic_std_optimizer']='critic/Adam/KL fresh; source actor/sigma frozen; residual trainable'
        before={name:value.detach().clone() for name,value in actor.named_parameters() if name.startswith('mlp.')}
        report['actor_state_before_sha256']=tensor_state_sha(actor.state_dict())
        if runner.alg.optimizer.state:raise ValueError('Optimizer unexpectedly contains imported training state')
        checked_optimizer=None
        if args.kl_checked_updates:
            from kl_checked_optimizer import KlCheckedOptimizer
            from ppo_update_ledger import distribution_on_rollout
            checked_source=Path(inspect.getfile(KlCheckedOptimizer))
            measure_source=Path(inspect.getfile(distribution_on_rollout))
            report['checked_optimizer_sources']={str(p):file_sha(p) for p in (checked_source,measure_source)}
            (args.output/'checked-optimizer-source.py').write_bytes(checked_source.read_bytes())
            (args.output/'update-measure-source.py').write_bytes(measure_source.read_bytes())
            checked_optimizer=KlCheckedOptimizer(runner.alg.optimizer,runner.alg.actor.parameters(),
                runner.alg.critic.parameters(),lambda:distribution_on_rollout(runner.alg),runner.alg.desired_kl)
            original_algorithm_update=runner.alg.update
            def checked_algorithm_update():
                checked_optimizer.begin_rollout()
                return original_algorithm_update()
            runner.alg.update=checked_algorithm_update
        previous_updates=previous_iterations=0
        if args.resume_run:
            from checkpoint_continuation import restore_run
            continuation_source=Path(inspect.getfile(restore_run))
            report['continuation_source_sha256']=file_sha(continuation_source)
            (args.output/'continuation-source.py').write_bytes(continuation_source.read_bytes())
            report['continuation']=restore_run(args.resume_run,runner,checked_optimizer,report,rl_cfg,
                args.output/'effective-environment-config.json')
            previous_updates=report['continuation']['parent_optimizer_steps']
            previous_iterations=report['continuation']['parent_iterations_completed']
            report['checkpoint_resume']=True
            report['initialization']='full_training_state_continuation_in_fresh_environment'
            report['configuration']['critic_std_optimizer']='restored_from_verified_checkpoint'
            report['actor_state_before_sha256']=tensor_state_sha(actor.state_dict())
            before={name:value.detach().clone() for name,value in actor.named_parameters() if name.startswith('mlp.')}
        updates=[]
        def record_update(optimizer,*unused):
            report['optimizer_steps']+=1
            updates.append(dict(index=report['optimizer_steps'],parameter_step_counters=optimizer_steps(optimizer)))
        handle=None
        if checked_optimizer is None:
            handle=runner.alg.optimizer.register_step_post_hook(record_update)
        else:
            checked_optimizer.on_commit=record_update
        update_ledger=None
        if args.update_ledger:
            from ppo_update_ledger import UpdateLedger
            update_ledger=UpdateLedger(runner.alg)
            ledger_source=Path(inspect.getfile(UpdateLedger))
            report['update_ledger_source_sha256']=file_sha(ledger_source)
            (args.output/'update-ledger-source.py').write_bytes(ledger_source.read_bytes())
        losses=[]
        def checkpoint_infos():
            return dict(run_id=report['run_id'],candidate_sha256=digest(candidate),
                optimizer_steps=report['optimizer_steps'],iterations_completed=len(losses),
                cumulative_optimizer_steps=previous_updates+report['optimizer_steps'],
                cumulative_iterations_completed=previous_iterations+len(losses),
                task_profile_sha256=report['task_profile_sha256'],
                forward_experiment_sha256=report.get('forward_experiment_sha256'),
                checked_optimizer_continuation=checked_optimizer.continuation_state() if checked_optimizer else None,
                training_converged=False,walking_verified=False,hardware_eligible=False)
        original_log=runner.logger.log
        def local_log(**values):
            row={name:float(value) for name,value in values['loss_dict'].items()}
            if not all(np.isfinite(list(row.values()))):raise ValueError('Nonfinite PPO losses')
            transitions=wrapped.transitions[-args.steps_per_env:]
            losses.append(dict(iteration=values['it'],losses=row,collect_seconds=values['collect_time'],
                update_seconds=values['learn_time'],learning_rate=values['learning_rate'],
                parameter_group_learning_rates=[group['lr'] for group in runner.alg.optimizer.param_groups],
                mean_reward=float(np.mean([item['reward'] for item in transitions])),
                mean_action_std=float(values['action_std'].detach().mean()),
                requested_target_invalid=sum(sum(item['requested_target_invalid']) for item in transitions),
                applied_target_invalid=sum(sum(item['applied_target_invalid']) for item in transitions),
                actual_joint_invalid=sum(sum(item['actual_joint_invalid']) for item in transitions),
                termination_counts={name:sum(sum(item['termination_terms'][name]) for item in transitions)
                    for name in env.termination_manager.active_terms}))
            result=original_log(**values)
            # This callback runs after alg.update() and after the runner sets its
            # current iteration. It cannot leave a half-filled PPO rollout.
            if len(losses)%8==0:
                runner.save(str(args.output/'latest-checkpoint.pt'),infos=checkpoint_infos())
                save('actual_ppo_updates_in_progress')
            if (args.max_training_seconds is not None and len(losses)<args.iterations
                    and time.monotonic()-learn_started>=args.max_training_seconds):
                raise TrainingTimeBudgetReached()
            return result
        runner.logger.log=local_log
        save('begin_actual_ppo_updates')
        report['training_started']=True
        learn_started=time.monotonic()
        report['stopped_at_training_wall_budget']=False
        try:
            runner.learn(num_learning_iterations=args.iterations,init_at_random_ep_len=False)
        except TrainingTimeBudgetReached:
            report['stopped_at_training_wall_budget']=True
        report['training_wall_seconds']=time.monotonic()-learn_started
        if update_ledger is not None:
            update_ledger.close()
            report['optimizer_update_ledger']=update_ledger.rows
            if file_sha(ledger_source)!=report['update_ledger_source_sha256']:
                raise ValueError('Update diagnostic changed during training')
        if handle is not None:handle.remove()
        if checked_optimizer is not None:
            report['checked_optimizer']=checked_optimizer.summary()
            if any(file_sha(p)!=s for p,s in report['checked_optimizer_sources'].items()):
                raise ValueError('Checked optimizer source changed during training')
        report['environment_steps_per_second']=report['environment_steps']/report['training_wall_seconds']
        report['iterations_completed']=len(losses);report['losses']=losses;report['optimizer_updates']=updates
        report['cumulative_optimizer_steps']=previous_updates+report['optimizer_steps']
        report['cumulative_iterations_completed']=previous_iterations+len(losses)
        if checked_optimizer is not None and checked_optimizer.committed_steps!=report['cumulative_optimizer_steps']:
            raise ValueError('Custom KL optimizer continuation counter differs')
        if phase_ledger is not None:
            report['phase_coverage']=phase_ledger.summary()
            (args.output/'phase-steps.json').write_text(json.dumps(phase_ledger.rows,allow_nan=False)+'\n')
            phase_ledger.close()
            if file_sha(phase_source)!=report['phase_ledger_source_sha256']:
                raise ValueError('Phase diagnostics changed during training')
        expected_steps=len(losses)*args.steps_per_env*args.num_envs
        if (report['environment_steps']!=expected_steps or expected_steps>total
                or (not report['stopped_at_training_wall_budget'] and expected_steps!=total)
                or report['optimizer_steps']!=len(losses)*args.epochs*args.mini_batches):
            raise ValueError('Observed environment/optimizer work differs from configured budget')
        report['actor_mlp_parameter_l1_delta']=sum(float((value.detach()-before[name]).abs().sum())
            for name,value in actor.named_parameters() if name in before)
        if report['actor_mlp_parameter_l1_delta']<=0:raise ValueError('Optimizer did not change actor MLP weights')
        report['actor_state_after_sha256']=tensor_state_sha(actor.state_dict())
        if args.forward_experiment:
            report['forward_zero_mapping_audit']=audit_zero_mapping(actor,source_actor_state,
                torch.as_tensor(np.asarray(wrapped.actor_observations,dtype=np.float32),device=args.device).reshape(-1,61))
        report['optimizer_state_steps']=optimizer_steps(runner.alg.optimizer)
        report['termination_counts']={name:sum(sum(row['termination_terms'][name]) for row in wrapped.transitions)
            for name in env.termination_manager.active_terms}
        report['requested_target_invalid_count']=sum(sum(row['requested_target_invalid']) for row in wrapped.transitions)
        report['applied_target_invalid_count']=sum(sum(row['applied_target_invalid']) for row in wrapped.transitions)
        report['actual_joint_invalid_count']=sum(sum(row['actual_joint_invalid']) for row in wrapped.transitions)
        report['range_diagnostic_scope']='Request checked after policy transform/head replacement; applied goal and actual joint positions observed before each physics substep and before automatic reset. Latent Gaussian values are not physical requests.'
        report['saved_range_termination_without_bad_current_request']=sum(row['range_failure_without_bad_request'] for row in wrapped.transitions)
        report['completed_or_terminated_episodes']=sum(row['done_count'] for row in wrapped.transitions)
        report['telemetry_scope']='per-step device-reduced event counts and mean reward' if args.compact_telemetry else 'per-step per-environment records'
        report['export_parity_observations_retained']=len(wrapped.actor_observations)
        report['transition_observation_scope']='Post-step observations include automatic reset; termination_terms preserve the pre-reset computed cause'
        (args.output/'transitions.json').write_text(json.dumps(wrapped.transitions,indent=2,allow_nan=False)+'\n')
        np.save(args.output/'actor-observations.npy',np.asarray(wrapped.actor_observations,dtype=np.float32))
        save('save_and_reload_real_checkpoint')
        checkpoint=args.output/'checkpoint.pt'
        infos=checkpoint_infos()
        runner.save(str(checkpoint),infos=infos)
        reloaded=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl_cfg),log_dir=None,device=args.device)
        if args.warm_start_onnx:
            # Install the same frozen normalizer structure before strict load.
            warmstart_actor_from_onnx(reloaded.alg.get_policy(),args.warm_start_onnx,
                expected_sha256=candidate['source_model']['sha256'])
        if args.bounded_exploration:
            reloaded_distribution=install_bounded_action_distribution(reloaded.alg.get_policy(),candidate)
            if reloaded_distribution!=report['bounded_distribution']:
                raise ValueError('Reloaded bounded distribution specification differs')
            expected_distribution_buffers={name:value.detach().clone() for name,value in
                reloaded.alg.get_policy().distribution.named_buffers()}
        if args.forward_experiment:
            reloaded_architecture=install_residual(reloaded,candidate)
            if reloaded_architecture!=report['forward_experiment']['architecture']:
                raise ValueError('Reloaded residual architecture differs')
        if args.kl_checked_updates:
            reloaded_checked_optimizer=KlCheckedOptimizer(reloaded.alg.optimizer,reloaded.alg.actor.parameters(),
                reloaded.alg.critic.parameters(),lambda:distribution_on_rollout(reloaded.alg),reloaded.alg.desired_kl)
        # Non-capturable Adam deliberately keeps its scalar step on CPU, even
        # when moments/parameters are CUDA tensors. Mapping every saved tensor
        # to CUDA changes that state and fails exact reload verification. Load
        # storage on CPU; model/optimizer load_state_dict route parameters and
        # moments to their existing device and preserve CPU step counters.
        loaded_info=reloaded.load(str(checkpoint),strict=True,map_location='cpu')
        if checked_optimizer is not None:
            reloaded_checked_optimizer.restore_continuation_state(loaded_info['checked_optimizer_continuation'])
            assert_state_identical(checked_optimizer.continuation_state(),reloaded_checked_optimizer.continuation_state())
        if loaded_info['candidate_sha256']!=digest(candidate):raise ValueError('Checkpoint candidate binding differs')
        if loaded_info['task_profile_sha256']!=report['task_profile_sha256']:raise ValueError('Checkpoint task binding differs')
        if loaded_info.get('forward_experiment_sha256')!=report.get('forward_experiment_sha256'):
            raise ValueError('Checkpoint forward experiment binding differs')
        if args.bounded_exploration:
            # Compare the loaded transform to freshly candidate-derived buffers,
            # not merely to the old checkpoint's own state/provenance claim.
            assert_state_identical(expected_distribution_buffers,
                dict(reloaded.alg.get_policy().distribution.named_buffers()))
        for side in ('actor','critic'):
            original_state=getattr(runner.alg,side).state_dict();reloaded_state=getattr(reloaded.alg,side).state_dict()
            if original_state.keys()!=reloaded_state.keys():raise ValueError('Reload model keys differ')
            for name,value in original_state.items():torch.testing.assert_close(value,reloaded_state[name],atol=0,rtol=0)
        if optimizer_steps(reloaded.alg.optimizer)!=report['optimizer_state_steps']:
            raise ValueError('Reloaded optimizer step counters differ')
        assert_state_identical(runner.alg.optimizer.state_dict(),reloaded.alg.optimizer.state_dict())
        report['checkpoint']=dict(path=str(checkpoint),sha256=file_sha(checkpoint),
            candidate_binding_verified=True,actor_critic_reload_identical=True,
            optimizer_step_counters_reload_identical=True,optimizer_full_state_reload_identical=True,
            custom_kl_continuation_state_reload_identical=checked_optimizer is not None,
            loaded_transform_matches_current_candidate=bool(args.bounded_exploration),
            iteration_index=reloaded.current_learning_iteration,
            iterations_completed=len(losses))
        save('export_checkpoint_actor_and_check_onnx')
        report['export']=attach_bound_export(reloaded,candidate,checkpoint,args.output/'unvalidated_actor.onnx',
            report['run_id'],report['cumulative_optimizer_steps'],wrapped.actor_observations,report.get('bounded_distribution'),report['task_profile'],report.get('forward_experiment'))
        if file_sha(candidate['source_model']['path'])!=report['original_model_sha256']:
            raise ValueError('Original source ONNX changed during offline work')
        if digest(load_candidate(args.candidate))!=report['candidate_sha256']:
            raise ValueError('Candidate sources changed during training')
        if args.warm_start_onnx and file_sha(inspect.getfile(warmstart_actor_from_onnx))!=report['warmstart_importer_sha256']:
            raise ValueError('Warm-start importer changed during training')
        if args.bounded_exploration and file_sha(distribution_path)!=report['distribution_source_sha256']:
            raise ValueError('Bounded action distribution changed during training')
        if args.resume_run and file_sha(continuation_source)!=report['continuation_source_sha256']:
            raise ValueError('Checkpoint continuation loader changed during training')
        if args.forward_experiment and file_sha(experiment_source)!=report['forward_experiment']['source_sha256']:
            raise ValueError('Forward experiment implementation changed during training')
        if args.exploration_initialization=='goal-slew' and file_sha(exploration_source)!=report['exploration_initialization']['source_sha256']:
            raise ValueError('Exploration initializer changed during training')
        if args.task_profile=='stand-button':
            if file_sha(profile_path)!=report['task_profile']['source_sha256']:
                raise ValueError('Task profile source changed during training')
            if file_sha(args.validation_baseline_onnx)!=report['validation_baseline']['sha256']:
                raise ValueError('Validation baseline changed during training')
            if any(file_sha(path)!=value for path,value in report['validation_sources'].items()):
                raise ValueError('Independent validation implementation changed during training')
            # Independent fresh environments, with no training autoreset or optimizer.
            env.close();env=None
            from validate_training_regression import run_gate
            report['regression_gate']=run_gate(args.candidate,args.validation_baseline_onnx,
                args.output/'unvalidated_actor.onnx',args.regression_output or args.output/'regression-gate')
            if not report['regression_gate']['new_zero_regression_pass']:
                save('offline_training_candidate_rejected')
                return 2
        report['throughput_scope']='32/64-step CPU smoke or explicitly budgeted run; not a prediction of time to convergence'
        save('training_pipeline_smoke_completed' if total<=64 else
            'offline_training_wall_budget_completed_unaccepted' if report['stopped_at_training_wall_budget'] else
            'offline_training_run_completed_unaccepted')
        return 0
    except Exception as error:
        report['failure']=dict(type=type(error).__name__,message=str(error),traceback=traceback.format_exc())
        save('failed')
        return 1
    finally:
        if env is not None:env.close()


if __name__=='__main__':
    raise SystemExit(main())
