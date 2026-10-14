"""Verify a local completed run before a fresh-environment continuation.

Restores complete actor, critic, Adam and KL admission schedule state. It does
not restore simulator state, random generators or a partly collected rollout.
Legacy KL v2 reports contain sufficient next-rollout state; migration is only
accepted when all six algorithm methods are AST-identical to current methods.
"""
import ast
import copy
import hashlib
import json
from pathlib import Path


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def optimizer_core(path):
    tree=ast.parse(Path(path).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='KlCheckedOptimizer')
    names={'__init__','begin_rollout','snapshot','restore','step','summary'}
    core={node.name:ast.dump(node,include_attributes=False) for node in cls.body
          if isinstance(node,ast.FunctionDef) and node.name in names}
    if set(core)!=names:raise ValueError('Unsupported optimizer implementation')
    return core


def legacy_guard_state(report, nominal_lr):
    old=report['checked_optimizer']
    if old['kind']!='post_step_full_rollout_kl_backtracking_v2':
        raise ValueError('Legacy KL version cannot be migrated')
    return dict(schema_version=1,boundary='before_next_begin_rollout',
        maximum_kl=old['maximum_kl'],max_trials=old['maximum_trials_per_actor_step'],
        nominal_actor_lr=nominal_lr,preferred_actor_lr=old['preferred_actor_lr'],
        rollouts=old['rollouts'],committed_steps=old['committed_optimizer_steps'],
        actor_steps=old['actor_steps'],critic_only_steps=old['critic_only_steps'],
        trial_steps=old['trial_optimizer_calls'],actor_numerical_noops=old['actor_numerical_noops'],
        maximum_committed_kl=old['maximum_committed_kl'],records=copy.deepcopy(old['first_100_steps']))


def restore_run(path, runner, guard, current, rl_cfg, environment_config):
    import torch
    from train_constrained_policy import assert_state_identical,tensor_state_sha
    path=Path(path).resolve()
    previous=json.loads((path/'run.json').read_text())
    accepted_stages={'offline_training_run_completed_unaccepted','offline_training_wall_budget_completed_unaccepted',
                     'offline_training_candidate_rejected','training_pipeline_smoke_completed'}
    if previous['stage'] not in accepted_stages or 'checkpoint' not in previous:
        raise ValueError('Continuation requires a completed run with verified checkpoint')
    if previous.get('forward_experiment') or current.get('forward_experiment'):
        raise ValueError('Forward residual experiments require a separate architecture-aware continuation contract')
    for key in ('candidate_sha256','original_model_sha256','task_profile_sha256','dependencies',
                'api_sources','distribution_source_sha256','bounded_distribution','warmstart_importer_sha256',
                'validation_baseline'):
        if previous[key]!=current[key]:raise ValueError('Continuation contract differs: '+key)
    # Actual archived source must match its original report, not only a claimed
    # version number. Core algorithm AST comparison permits serialization only.
    archived=path/'checked-optimizer-source.py'
    old_sources=previous['checked_optimizer_sources']
    guard_source=next(k for k in current['checked_optimizer_sources'] if Path(k).name=='kl_checked_optimizer.py')
    old_guard_source=next(k for k in old_sources if Path(k).name=='kl_checked_optimizer.py')
    if sha(archived)!=old_sources[old_guard_source] or optimizer_core(archived)!=optimizer_core(guard_source):
        raise ValueError('Continuation KL algorithm differs')
    for key,value in old_sources.items():
        if Path(key).name!='kl_checked_optimizer.py' and current['checked_optimizer_sources'].get(key)!=value:
            raise ValueError('Continuation KL measurement differs')
    for file,key in [('runner-source.py','runner_source_sha256'),('distribution-source.py','distribution_source_sha256'),
                     ('warmstart-importer-source.py','warmstart_importer_sha256')]:
        if sha(path/file)!=previous[key]:raise ValueError('Archived source hash differs: '+file)
    old_cfg=json.loads((path/'runner-config.json').read_text())
    # Dataclass tuples were serialized as JSON arrays in the saved artifact.
    # Compare the same canonical representation, without dropping any values.
    rl_cfg=json.loads(json.dumps(rl_cfg,allow_nan=False))
    old_env=json.loads((path/'effective-environment-config.json').read_text())
    new_env=json.loads(Path(environment_config).read_text())
    for config in (old_env,new_env):
        config.pop('seed',None)
        config['scene'].pop('num_envs',None)
    if old_env!=new_env:raise ValueError('Continuation physics/reward/environment configuration differs')
    # Learning rule, network, normalization, minibatches and rollout length must
    # match. New iteration budget/seed/log name do not change those contracts.
    for key in ('actor','critic','algorithm','obs_groups','clip_actions','num_steps_per_env'):
        if old_cfg.get(key)!=rl_cfg.get(key):raise ValueError('Continuation runner config differs: '+key)
    checkpoint=path/'checkpoint.pt'
    if sha(checkpoint)!=previous['checkpoint']['sha256']:raise ValueError('Continuation checkpoint hash differs')
    saved=torch.load(checkpoint,map_location=runner.device,weights_only=True)
    infos=saved['infos']
    if (infos['run_id']!=previous['run_id'] or infos['candidate_sha256']!=current['candidate_sha256']
            or infos['task_profile_sha256']!=current['task_profile_sha256']):
        raise ValueError('Continuation checkpoint binding differs')
    if tensor_state_sha(saved['actor_state_dict'])!=previous['actor_state_after_sha256']:
        raise ValueError('Continuation actor hash differs')
    state=infos.get('checked_optimizer_continuation')
    migrated=state is None
    if migrated:
        state=legacy_guard_state(previous,old_cfg['algorithm']['learning_rate'])
    # Validate custom state before model/optimizer load; only detached diagnostic
    # continuation objects are involved, never hardware or a running training job.
    guard.restore_continuation_state(state)
    previous_updates=previous.get('cumulative_optimizer_steps',previous['optimizer_steps'])
    previous_iterations=previous.get('cumulative_iterations_completed',previous['iterations_completed'])
    if state['committed_steps']!=previous_updates or state['rollouts']!=previous_iterations:
        raise ValueError('Continuation optimizer counters differ')
    if saved['iter']+1!=previous_iterations:raise ValueError('Continuation iteration boundary differs')
    runner.alg.load(saved,load_cfg=None,strict=True)
    for key,model in [('actor_state_dict',runner.alg.actor),('critic_state_dict',runner.alg.critic)]:
        assert_state_identical(saved[key],model.state_dict())
    assert_state_identical(saved['optimizer_state_dict'],runner.alg.optimizer.state_dict())
    # Upstream stores the last completed index, learn() begins at current index.
    runner.current_learning_iteration=saved['iter']+1
    return dict(parent_run_id=previous['run_id'],parent_run_json_sha256=sha(path/'run.json'),
        checkpoint_path=str(checkpoint),checkpoint_sha256=sha(checkpoint),
        parent_optimizer_steps=previous_updates,parent_iterations_completed=previous_iterations,
        custom_kl_state_migrated_from_completed_v2_report=migrated,
        actor_critic_optimizer_reload_identical=True,custom_kl_state=guard.continuation_state(),
        scope='fresh environment and RNG seed; complete training parameters and Adam/KL schedule continued',
        exact_trajectory_resume=False,simulator_or_rng_state_restored=False)
