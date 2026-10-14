#!/usr/bin/env python3
"""Paired offline probe of independent versus 200ms coherent leg exploration.

Same marginal latent standard deviation and unchanged 50Hz constrained plant.
The larger scale is explicitly a NEW diagnostic, not source sigma continuation.
No policy is trained and no result is a deployable gait or a proof of feasibility.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--seed-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--num-envs',type=int,default=64);p.add_argument('--steps',type=int,default=800)
    p.add_argument('--wall-seconds',type=float,default=180)
    p.add_argument('--trace-env',type=int)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards,tensor_state_sha
    install_offline_guards(accelerator=a.device=='cuda:0')
    import numpy as np
    import torch
    from torch import nn
    import warp as wp
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from bounded_action_distribution import ArctanActionTransform,distribution_config
    from train_slow_gait import load_seed,sha
    import slow_gait_experiment as experiment
    from mjlab.envs import ManagerBasedRlEnv
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    candidate=load_candidate(a.candidate);state,manifest=load_seed(a.seed_dir,candidate)
    scale_rad=math.radians(4.9)/candidate['action_scale']
    if a.trace_env is not None and not 0<=a.trace_env<a.num_envs:raise ValueError('Trace environment outside batch')
    report=dict(source_sha256=sha(__file__),candidate_sha256=digest(candidate),seed_manifest_sha256=sha(a.seed_dir/'seed.json'),
        diagnostic='same marginal noise, IID versus 10-control-period hold',
        latent_sigma=scale_rad,scale_basis='prior within-range 3mm kinematic endpoint required up to4.9deg; not proof of dynamic support',
        cadence_hz=50,plant_changes=False,trained=False,walking_verified=False,hardware_eligible=False,arms=[])
    start=time.monotonic();initials=[]
    for hold in (1,10):
        torch.manual_seed(0);np.random.seed(0)
        base=nn.Sequential(nn.Linear(61,512),nn.ELU(),nn.Linear(512,256),nn.ELU(),nn.Linear(256,128),nn.ELU(),nn.Linear(128,14)).to(a.device).eval()
        base.load_state_dict({k.removeprefix('mlp.'):v.to(a.device) for k,v in state.items() if k.startswith('mlp.')})
        mean=state['obs_normalizer.onnx_mean'].to(a.device);den=state['obs_normalizer.onnx_denominator'].to(a.device)
        transform=ArctanActionTransform(distribution_config(candidate),device=a.device)
        cfg,_=make_mjlab_configs(candidate);experiment.configure(cfg)
        cfg.scene.num_envs=a.num_envs;cfg.seed=0;cfg.auto_reset=True
        env=None
        try:
            env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
            obs,_=env.reset(seed=0)
            initial={k:v.detach().cpu().clone() for k,v in [('qpos',env.sim.data.qpos),('qvel',env.sim.data.qvel),('mass',wp.to_torch(env.sim.wp_model.body_mass))]}
            initials.append(initial)
            if len(initials)>1:
                for k,v in initial.items():torch.testing.assert_close(v,initials[0][k],atol=0,rtol=0)
            rng=torch.Generator(device=a.device).manual_seed(219)
            mask=torch.tensor([sid<30 for sid in candidate['policy_joint_ids']],device=a.device)
            counts=torch.zeros((a.num_envs,2),device=a.device,dtype=torch.long)
            streak=torch.zeros_like(counts);max_streak=torch.zeros_like(counts)
            peaks=torch.full((a.num_envs,2),-float('inf'),device=a.device)
            max_tilt=torch.zeros(a.num_envs,device=a.device)
            active_count=torch.zeros(a.num_envs,device=a.device,dtype=torch.long)
            deaths={k:0 for k in env.termination_manager.active_terms};noise=None;frames=0;trace=[]
            with torch.inference_mode():
                for step in range(a.steps):
                    if step%hold==0:noise=torch.randn(a.num_envs,14,device=a.device,generator=rng)*scale_rad*mask
                    raw=obs['actor'].clone();raw[:,48:51]=0.
                    gate=(env.command_manager.get_command('twist')[:,0]/experiment.SPEC['command_m_s']).clamp(0.,1.)
                    action=transform(base((raw-mean)/den)+noise*gate[:,None])
                    obs,_,terminated,truncated,_=env.step(action)
                    active=(gate>0)&~terminated&~truncated
                    heights=experiment.sole_heights(env)
                    load=env.scene['feet_ground_contact']._extract_sensor_data().force[:,:,2].abs()
                    gravity=env.scene['robot'].data.projected_gravity_b
                    tilt=torch.acos((-gravity[:,2]/gravity.norm(dim=-1)).clamp(-1.,1.))*180/math.pi
                    valid=(heights>=.003)&(load<.001)&(load.flip(-1)>=1.)&(tilt[:,None]<15)&active[:,None]
                    counts+=valid;streak=torch.where(valid,streak+1,torch.zeros_like(streak));max_streak=torch.maximum(max_streak,streak)
                    peaks=torch.maximum(peaks,torch.where(active[:,None],heights,torch.full_like(heights,-float('inf'))))
                    max_tilt=torch.maximum(max_tilt,torch.where(active,tilt,torch.zeros_like(tilt)));active_count+=active
                    for k in deaths:deaths[k]+=int(env.termination_manager.get_term(k).sum())
                    if a.trace_env is not None:
                        i=a.trace_env;term=env.action_manager.get_term('joint_pos')
                        trace.append(dict(step=step,gate=float(gate[i]),episode_step_after=int(env.episode_length_buf[i]),
                            done=bool(terminated[i]|truncated[i]),qualified=valid[i].cpu().tolist(),
                            height_m=heights[i].cpu().tolist(),load_n=load[i].cpu().tolist(),tilt_deg=float(tilt[i]),
                            noise=noise[i].cpu().tolist(),actions=action[i].cpu().tolist(),
                            goal_rad=term._goal[i].cpu().tolist(),actual_joint_rad=term._entity.data.joint_pos[i,term._target_ids].cpu().tolist(),
                            root_position=env.scene['robot'].data.root_link_pos_w[i].cpu().tolist(),
                            sample_after_automatic_reset_if_done=True))
                    frames+=1
                    if time.monotonic()-start>a.wall_seconds:raise TimeoutError('Bounded exploration diagnostic exceeded its wall limit')
            finite_peaks=torch.where(torch.isfinite(peaks),peaks,torch.zeros_like(peaks))
            arm=dict(hold_steps=hold,hold_seconds=hold*.02,frames=frames,transitions=frames*a.num_envs,
                initial_state_sha256=tensor_state_sha(initial),qualified_lift_frames_per_env=counts.cpu().tolist(),
                max_lift_streak_per_env=max_streak.cpu().tolist(),qualified_lift_frames=counts.sum(0).cpu().tolist(),
                environments_with_60ms_lift=(max_streak>=3).sum(0).cpu().tolist(),
                max_whole_sole_clearance_m=finite_peaks.amax(0).cpu().tolist(),
                active_samples=int(active_count.sum()),termination_counts=deaths,
                maximum_surviving_tilt_deg=float(max_tilt.max()),acceptance_scope='exploration coverage only; no landing/alternation/forward success claimed')
            report['arms'].append(arm)
            if a.trace_env is not None:
                (a.output/f'hold-{hold}-env-{a.trace_env}.jsonl').write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in trace))
            (a.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            print(json.dumps({k:v for k,v in arm.items() if not k.endswith('_per_env')}),flush=True)
        finally:
            if env is not None:env.close()
    report.update(initial_states_exactly_matched=True,wall_seconds=time.monotonic()-start)
    (a.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
