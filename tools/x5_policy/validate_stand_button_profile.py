#!/usr/bin/env python3
"""Real-environment command coverage fixture; zero optimizer updates.

The supplied model is only a deterministic fixture controller. Surviving this
fixture does not establish gait quality or physical robot eligibility.
"""
from pathlib import Path
import argparse
import hashlib
import json

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','fixture-policy','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards,file_sha
    install_offline_guards()
    import numpy as np
    import torch
    import onnxruntime as ort
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest,check_model_compatibility
    from stand_button_training_profile import apply_task_profile,command_at_steps,source_pulse,PULSE_SOURCE
    from mjlab.envs import ManagerBasedRlEnv
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    candidate=load_candidate(args.candidate)
    check_model_compatibility(candidate,args.fixture_policy,require_training_provenance=True)
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=ort.InferenceSession(str(args.fixture_policy),options,providers=['CPUExecutionProvider'])
    def infer(actor):
        return np.concatenate([session.run(None,{session.get_inputs()[0].name:row[None]})[0] for row in actor])
    report=dict(candidate_sha256=digest(candidate),fixture_policy_sha256=file_sha(args.fixture_policy),
        fixture_source_sha256=file_sha(__file__),optimizer_steps=0,training_started=False,
        walking_verified=False,hardware_eligible=False,network_allowed=False)
    pulse=source_pulse();trace=[];env=None
    try:
        cfg,_=make_mjlab_configs(candidate)
        # Compare untouched plant/reward/guard repr before/after the explicit task.
        protected=lambda:repr((cfg.scene,cfg.actions,cfg.rewards,cfg.terminations,cfg.sim,cfg.decimation,cfg.episode_length_s))
        protected_before=protected()
        report['profile']=apply_task_profile(cfg,'stand-button')
        assert protected()==protected_before
        report['plant_reward_guard_configuration_identical']=True
        cfg.scene.num_envs=1;cfg.seed=0;cfg.auto_reset=False
        env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
        observations,_=env.reset(seed=0)
        term=env.command_manager.get_term('twist')
        reward_commands=[];original_reward=env.reward_manager.compute
        def record_reward(*a,**kw):
            reward_commands.append(term.command.detach().numpy().copy())
            return original_reward(*a,**kw)
        env.reward_manager.compute=record_reward
        for index in range(500):
            expected=pulse.command_at(index*env.step_dt,0)[0] if index>=250 else 0.
            actor=observations['actor'].numpy()
            np.testing.assert_allclose(actor[0,48:51],[expected,0,0],atol=1e-7,rtol=0)
            raw=infer(actor)
            observations,reward,terminated,truncated,_=env.step(torch.from_numpy(raw))
            np.testing.assert_allclose(reward_commands[-1][0],[expected,0,0],atol=1e-7,rtol=0)
            actual=env.scene['robot'].data.joint_pos[:,env.action_manager.get_term('joint_pos').target_ids]
            t=env.action_manager.get_term('joint_pos')
            invalid=bool(((actual<t._low)|(actual>t._high)|~torch.isfinite(actual)).any())
            trace.append(dict(index=index,command=actor[0,48:51].tolist(),reward_command=reward_commands[-1][0].tolist(),
                next_observation_command=observations['actor'][0,48:51].tolist(),terminated=terminated.tolist(),
                truncated=truncated.tolist(),actual_range_invalid=invalid))
            if bool(terminated.any()|truncated.any()) or invalid:
                raise ValueError('Fixture controller failed; timing proof is incomplete')
        report.update(full_profile_completed=True,real_physics_steps=500,zero_input_frames=250,
            button_input_frames=100,recovery_input_frames=150,actor_reward_same_command_all_frames=True,
            frozen_button_match_all_frames=True)
        env.close();env=None
        cfg,_=make_mjlab_configs(candidate);apply_task_profile(cfg,'stand-button')
        cfg.scene.num_envs=2;cfg.seed=0;cfg.auto_reset=True
        env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
        observations,_=env.reset(seed=0)
        # Genuine parallel physics establishes independent episode clocks.
        # Full forward timing is covered above; this fixture tests reset semantics.
        for index in range(30):
            raw=infer(observations['actor'].numpy())
            observations,_,terminated,truncated,_=env.step(torch.from_numpy(raw))
            if bool(terminated.any()|truncated.any()):raise ValueError('Parallel fixture unexpectedly reset before planned selective reset')
        # Controlled clock integration case, NOT 300 physics steps of stability:
        # put env1 into forward phase to catch accidental all-env command clearing.
        env.episode_length_buf[1]=300
        env.command_manager.compute(dt=0.)
        before=env.episode_length_buf.clone();before_cmd=env.command_manager.get_command('twist').clone()
        observations,_=env.reset(env_ids=torch.tensor([0],device='cpu'))
        after=env.episode_length_buf.clone();after_cmd=env.command_manager.get_command('twist').clone()
        assert before.tolist()==[30,300] and after.tolist()==[0,300]
        torch.testing.assert_close(after_cmd[0],torch.zeros(3),atol=0,rtol=0)
        torch.testing.assert_close(after_cmd[1],before_cmd[1],atol=0,rtol=0)
        # Deliberately send one invalid request to env0. The unchanged saved-range
        # guard must hold its goal, terminate that env, and actually auto-reset it.
        raw=infer(observations['actor'].numpy())
        raw[0,4]=100.
        observations,_,terminated,truncated,_=env.step(torch.from_numpy(raw))
        assert terminated.tolist()==[True,False] and env.episode_length_buf.tolist()==[0,301]
        assert env.termination_manager.get_term('saved_range_violation').tolist()==[True,False]
        torch.testing.assert_close(observations['actor'][0,48:51],torch.zeros(3),atol=0,rtol=0)
        np.testing.assert_allclose(observations['actor'][1,48:51].numpy(),[.2,0,0],atol=1e-7,rtol=0)
        report['independent_reset']=dict(real_parallel_steps=31,before=before.tolist(),after_manual=after.tolist(),
            before_commands=before_cmd.tolist(),after_commands=after_cmd.tolist(),
            after_actual_saved_range_failure_autoreset=env.episode_length_buf.tolist(),
            clock_fixture='env1 age explicitly set to 300 after 30 real steps to test forward command isolation; not 300 steps of physical stability',
            failure_fixture='only env0 ID14 raw request=100; original saved-range guard rejects and automatically resets; no threshold changes',
            selected_command_zero=True,other_environment_clock_and_command_preserved=True)
        report['status']='real_command_coverage_fixture_passed_not_gait_acceptance'
    except Exception as error:
        report.update(status='failed',error=str(error))
        raise
    finally:
        if env is not None:env.close()
        (args.output/'fixture.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        (args.output/'trace.json').write_text(json.dumps(trace,indent=2,allow_nan=False)+'\n')
        for name,path in [('fixture-source.py',Path(__file__)),('profile-source.py',Path(__file__).with_name('stand_button_training_profile.py')),('button-source.py',PULSE_SOURCE)]:
            (args.output/name).write_bytes(path.read_bytes())
    print(json.dumps({key:report[key] for key in ('status','real_physics_steps','optimizer_steps')}))

if __name__=='__main__':main()
