"""Bounded frozen stochastic-policy coverage, never optimizer continuation."""
import argparse
import copy
from dataclasses import asdict
import hashlib
import inspect
import json
import os
from pathlib import Path
import random
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('candidate', 'run', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    parser.add_argument('--steps', type=int, default=700)
    parser.add_argument('--num-envs', type=int, default=32)
    parser.add_argument('--verify-passive-parity', action='store_true')
    parser.add_argument('--trained-only', action='store_true')
    parser.add_argument('--std-multiplier', type=float, choices=(1.,2.,4.,8.), default=1.,
        help='Explicit frozen exploration ablation only; no training or actuator-limit change')
    args = parser.parse_args()
    if not 1 <= args.steps <= 1000 or not 1 <= args.num_envs <= 32:
        raise ValueError('Explicit diagnostic budget exceeded')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['MPLCONFIGDIR'] = str(args.output / 'mpl-cache')
    from train_constrained_policy import install_offline_guards, file_sha, tensor_state_sha, assert_state_identical
    install_offline_guards()
    import numpy as np
    import torch
    from constrained_training_candidate import load_candidate, make_mjlab_configs, digest
    from stand_button_training_profile import apply_task_profile
    from onnx_actor_warmstart import warmstart_actor_from_onnx
    from bounded_action_distribution import install_bounded_action_distribution
    from phase_training_ledger import PhaseTrainingLedger
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
    torch.set_num_threads(1); torch.set_num_interop_threads(1)
    candidate = load_candidate(args.candidate)
    report = json.loads((args.run / 'run.json').read_text())
    if report['candidate_sha256'] != digest(candidate):
        raise ValueError('Candidate differs')
    checkpoint = args.run / 'checkpoint.pt'
    if file_sha(checkpoint) != report['checkpoint']['sha256']:
        raise ValueError('Checkpoint differs')
    for function, key in ((warmstart_actor_from_onnx, 'warmstart_importer_sha256'),
                          (install_bounded_action_distribution, 'distribution_source_sha256')):
        if file_sha(inspect.getfile(function)) != report[key]:
            raise ValueError('Actor construction source differs')
    loaded = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if loaded['infos']['candidate_sha256'] != digest(candidate) or loaded['infos']['run_id'] != report['run_id']:
        raise ValueError('Checkpoint binding differs')
    saved_actor = loaded['actor_state_dict']
    if tensor_state_sha(saved_actor) != report['actor_state_after_sha256']:
        raise ValueError('Checkpoint actor tensors differ')
    rl_cfg = json.loads((args.run / 'runner-config.json').read_text())
    sources = {str(path): file_sha(path) for path in (Path(__file__), Path(inspect.getfile(PhaseTrainingLedger)),
        Path(inspect.getfile(make_mjlab_configs)), Path(inspect.getfile(apply_task_profile)),
        args.run / 'run.json', args.run / 'runner-config.json', checkpoint)}
    result = dict(hardware_access=False, network_access=False, optimizer_steps=0, network_weights_changed=False,
        parameters_changed_during_rollout=False, exploration_sigma_multiplier=args.std_multiplier,
        exploration_parameters_changed_before_rollout=args.std_multiplier!=1.,
        checkpoint_resume=False, scope='fresh-environment frozen stochastic sampling, not historical PPO reconstruction',
        maximum_wall_seconds_per_case=180, steps=args.steps, num_envs=args.num_envs,
        candidate_sha256=digest(candidate), sources=sources, completed=False, cases=[])
    def save():
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (args.output / 'diagnostic-source.py').write_bytes(Path(__file__).read_bytes())
    (args.output / 'ledger-source.py').write_bytes(Path(inspect.getfile(PhaseTrainingLedger)).read_bytes())
    save()
    conditions = [('initial', True), ('trained', True)]
    if args.trained_only: conditions=[('trained',True)]
    if args.verify_passive_parity:
        conditions = [('trained', False), ('trained', True)]
    initial = None; parity = None
    for label, enabled in conditions:
        env = None; ledger = None; start = time.monotonic()
        case = dict(model=label, ledger_enabled=enabled, completed=False)
        result['cases'].append(case); save()
        try:
            torch.manual_seed(0); np.random.seed(0); random.seed(0)
            cfg, _ = make_mjlab_configs(candidate)
            profile = apply_task_profile(cfg, 'stand-button')
            if digest(profile) != report['task_profile_sha256']:
                raise ValueError('Task profile differs')
            cfg.scene.num_envs=args.num_envs; cfg.seed=0; cfg.auto_reset=True
            env = ManagerBasedRlEnv(cfg, device='cpu', render_mode=None)
            wrapped = RslRlVecEnvWrapper(env, clip_actions=None)
            obs = wrapped.get_observations()
            runner = MjlabOnPolicyRunner(wrapped, copy.deepcopy(rl_cfg), log_dir=None, device='cpu')
            actor = runner.alg.get_policy()
            warmstart_actor_from_onnx(actor, candidate['source_model']['path'],
                expected_sha256=candidate['source_model']['sha256'])
            distribution = install_bounded_action_distribution(actor, candidate)
            if distribution != report['bounded_distribution']:
                raise ValueError('Action transform differs')
            if label == 'trained':
                actor.load_state_dict(saved_actor, strict=True)
                assert_state_identical(saved_actor, actor.state_dict())
            case['original_latent_std']=actor.distribution.std_param.detach().tolist()
            with torch.no_grad():actor.distribution.std_param.mul_(args.std_multiplier)
            actor.eval()
            before = tensor_state_sha(actor.state_dict())
            sigma = actor.distribution.std_param.detach().clone()
            case['latent_std'] = sigma.tolist()
            fingerprint = hashlib.sha256(obs['actor'].numpy().tobytes() + env.sim.wp_data.qpos.numpy().tobytes() + env.sim.wp_data.qvel.numpy().tobytes()).hexdigest()
            if initial is None: initial = fingerprint
            elif initial != fingerprint: raise ValueError('Paired initial state differs')
            case['initial_state_sha256'] = fingerprint
            if enabled: ledger = PhaseTrainingLedger(env)
            trace = hashlib.sha256()
            # Identical stochastic draws at start; no claim of old rollout replay.
            torch.manual_seed(1024)
            with torch.inference_mode():
                for step in range(args.steps):
                    if time.monotonic() - start > 180: raise TimeoutError('Diagnostic wall budget')
                    actions = actor(obs, stochastic_output=True)
                    obs, reward, done, _ = wrapped.step(actions)
                    for value in (actions, obs['actor'], reward, done):
                        trace.update(value.detach().cpu().contiguous().numpy().tobytes())
                    if (step+1) % 250 == 0:
                        print(label, 'ledger', enabled, 'steps', step+1, flush=True)
            after = tensor_state_sha(actor.state_dict())
            if before != after: raise ValueError('Frozen actor changed')
            if runner.alg.optimizer.state: raise ValueError('Unexpected optimizer update')
            case.update(completed=True, actor_sha256=after, trajectory_sha256=trace.hexdigest(),
                coverage=ledger.summary() if ledger is not None else None)
            if args.verify_passive_parity:
                if parity is None: parity=case['trajectory_sha256']
                elif parity != case['trajectory_sha256']: raise ValueError('Passive diagnostics changed trajectory')
        except BaseException as error:
            case['error']=type(error).__name__ + ': ' + str(error)
            raise
        finally:
            if ledger is not None:
                (args.output / (label+'-phase-steps.json')).write_text(json.dumps(ledger.rows,allow_nan=False)+'\n')
                ledger.close()
            if env is not None: env.close()
            case['wall_seconds']=time.monotonic()-start; save()
    if any(file_sha(p) != s for p,s in sources.items()): raise ValueError('Diagnostic source changed')
    result.update(completed=True, passive_trajectory_parity_passed=args.verify_passive_parity); save()


if __name__ == '__main__':
    main()
