#!/usr/bin/env python3
"""Read-only policy sampling: correlate actual lift coverage with PPO advantages.

No optimization and no acceptance claim. Compare frozen deterministic and
stochastic controllers; infer whether rare lift samples get a positive learning
signal. Post-reset observations cannot count as a lift or a landing.
"""
import argparse
import copy
import json
from pathlib import Path
import random
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('candidate','run','seed-dir','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--num-envs',type=int,default=64);p.add_argument('--decisions',type=int,default=160)
    p.add_argument('--wall-seconds',type=float,default=180)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards,tensor_state_sha
    install_offline_guards(accelerator=a.device=='cuda:0')
    import numpy as np
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from train_slow_gait import load_seed,sha
    import slow_gait_experiment as task
    import residual_hold_experiment as core
    torch.set_num_threads(1)
    run=json.loads((a.run/'run.json').read_text())
    if run['stage']!='training_complete_requires_independent_gait_validation':raise ValueError('Requires completed frozen checkpoint')
    if sha(a.run/'checkpoint.pt')!=run['checkpoint_sha256']:raise ValueError('Checkpoint changed')
    candidate=load_candidate(a.candidate)
    if digest(candidate)!=run['candidate_sha256']:raise ValueError('Candidate changed')
    seed,manifest=load_seed(a.seed_dir,candidate)
    if manifest!=run['seed']:raise ValueError('Seed changed')
    for module in (core,task):
        if sha(module.__file__)!=run['sources'][Path(module.__file__).name]:raise ValueError('Control/task source changed')
    saved=torch.load(a.run/'checkpoint.pt',weights_only=True,map_location='cpu')
    if tensor_state_sha(saved['actor_state_dict'])!=run['actor_tensor_sha256']:raise ValueError('Actor changed')
    report=dict(scope=__doc__,checkpoint_sha256=sha(a.run/'checkpoint.pt'),source_sha256=sha(__file__),
        hardware_access=False,optimizer_steps=0,arms=[])
    start=time.monotonic();initial_hash=None
    for arm_name in ('deterministic','stochastic','zero_upper_stochastic'):
        stochastic=arm_name!='deterministic'
        for f in (torch.manual_seed,np.random.seed,random.seed):f(19)
        cfg,_=make_mjlab_configs(candidate);task.configure(cfg,com_transfer=True)
        cfg.scene.num_envs=a.num_envs;cfg.seed=19;cfg.auto_reset=True;cfg.is_finite_horizon=True
        env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
        lower=core.FrozenStandingController(candidate,seed,a.device)
        wrapped=core.wrapper_class()(env,lower)
        rl=json.loads((a.run/'runner-config.json').read_text())
        runner=MjlabOnPolicyRunner(wrapped,copy.deepcopy(rl),log_dir=None,device=a.device)
        core.install_upper(runner,lower,candidate)
        actor=runner.alg.actor;critic=runner.alg.critic
        actor.load_state_dict(saved['actor_state_dict'],strict=True);critic.load_state_dict(saved['critic_state_dict'],strict=True)
        actor.eval();critic.eval()
        exploration_rng=torch.Generator(device=a.device);exploration_rng.manual_seed(190)
        obs=wrapped.get_observations()
        import warp as wp
        initial=tensor_state_sha({n:wp.to_torch(getattr(obj,n)).cpu().clone() for obj,n in
            ((env.sim.wp_data,'qpos'),(env.sim.wp_data,'qvel'),(env.sim.wp_model,'body_mass'))})
        if initial_hash is None:initial_hash=initial
        elif initial!=initial_hash:raise ValueError('Arm initial qpos/qvel/mass differ')
        frames=[];rewards=[];dones=[];values=[];means=[];macro_lifts=[];macro_phase=[]
        original=RslRlVecEnvWrapper.step;alive=torch.ones(a.num_envs,device=a.device,dtype=torch.bool)
        def observe(self,action):
            nonlocal alive
            active=(self.micro_obs['actor'][:,48]>0)&alive
            result=original(self,action);done=result[2].bool();valid=active&~done
            height=task.sole_heights(env)
            load=env.scene['feet_ground_contact']._extract_sensor_data().force[:,:,2].abs()
            tilt=task.outside_observation_boundary(env)
            lift=(height>=.003)&(load<.001)&(load.flip(-1)>=1.)&valid[:,None]&~tilt[:,None]
            frames.append(dict(height=height.detach().cpu(),load=load.detach().cpu(),lift=lift.cpu(),
                done=done.cpu(),active=valid.cpu(),rates=env.reward_manager._step_reward.detach().cpu().clone()))
            alive&=~done
            return result
        RslRlVecEnvWrapper.step=observe
        try:
            with torch.inference_mode():
                for decision in range(a.decisions):
                    if time.monotonic()-start>a.wall_seconds:raise TimeoutError('Diagnostic wall limit')
                    alive.fill_(True)
                    values.append(critic(obs).flatten().cpu());macro_phase.append(task.phase_features(env).cpu())
                    mean=actor.mlp(actor.get_latent(obs));sigma=actor.distribution.log_std_param.exp()
                    if arm_name=='zero_upper_stochastic':mean=torch.zeros_like(mean)
                    residual=mean+torch.randn(mean.shape,device=a.device,generator=exploration_rng)*sigma if stochastic else mean
                    means.append(mean.cpu());before=len(frames)
                    obs,reward,done,_=wrapped.step(residual)
                    rewards.append(reward.cpu());dones.append(done.cpu())
                    macro_lifts.append(torch.stack([r['lift'] for r in frames[before:]]).any(0).any(-1))
                bootstrap=critic(obs).flatten().cpu()
            r=torch.stack(rewards);d=torch.stack(dones).bool();v=torch.stack(values)
            advantage=torch.zeros(a.num_envs);adv=torch.zeros_like(r);nextvalue=bootstrap
            gamma=core.MICRO_GAMMA**core.HOLD_STEPS;lam=core.MICRO_LAMBDA**core.HOLD_STEPS
            for k in reversed(range(len(r))):
                delta=r[k]+gamma*nextvalue*~d[k]-v[k]
                advantage=delta+gamma*lam*~d[k]*advantage;adv[k]=advantage;nextvalue=v[k]
            normalized=(adv-adv.mean())/(adv.std()+1e-8);lifts=torch.stack(macro_lifts)
            arrays={k:torch.stack([f[k] for f in frames]).numpy() for k in frames[0]}
            arrays.update(rewards=r.numpy(),dones=d.numpy(),values=v.numpy(),advantages=adv.numpy(),
                normalized_advantages=normalized.numpy(),mean_residual=torch.stack(means).numpy(),phase=torch.stack(macro_phase).numpy())
            np.savez_compressed(a.output/(arm_name+'.npz'),**arrays)
            continuous=np.zeros((a.num_envs,2),int);maxrun=continuous.copy();landrun=continuous.copy();air=np.zeros_like(continuous,bool);cycles=continuous.copy()
            for f in frames:
                lift=f['lift'].numpy();done=f['done'].numpy()
                continuous=np.where(lift,continuous+1,0);maxrun=np.maximum(maxrun,continuous)
                air|=continuous>=3
                landing=(f['height'].numpy()<.001)&(f['load'].numpy()>=1)&f['active'].numpy()[:,None]
                landrun=np.where(landing,landrun+1,0);finished=air&(landrun>=3)&~done[:,None]
                cycles+=finished;air&=~finished;air[done]=False;landrun[done]=0;continuous[done]=0
            arm=dict(arm=arm_name,stochastic=stochastic,decisions=a.decisions,micro_transitions=len(frames)*a.num_envs,
                initial_qpos_qvel_mass_sha256=initial,qualified_sensor_frames=arrays['lift'].sum((0,1)).tolist(),
                environments_with_60ms_sensor_lift=(maxrun>=3).sum(0).tolist(),completed_sensor_lift_land_cycles=cycles.sum(0).tolist(),
                reset_count=int(d.sum()),lift_decision_count=int(lifts.sum()),mean_macro_reward=float(r.mean()),
                mean_gae=float(adv.mean()),lift_mean_gae=float(adv[lifts].mean()) if lifts.any() else None,
                lift_mean_normalized_gae=float(normalized[lifts].mean()) if lifts.any() else None,
                lift_positive_normalized_gae_fraction=float((normalized[lifts]>0).float().mean()) if lifts.any() else None,
                learning_signal_note='new rollout with final trained critic; zero-upper arm has an off-policy critic; not historical minibatches or causal attribution',
                rate_names=list(env.reward_manager.active_terms))
            report['arms'].append(arm)
            (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(arm),flush=True)
        finally:
            RslRlVecEnvWrapper.step=original;env.close()
    report['wall_seconds']=time.monotonic()-start
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
