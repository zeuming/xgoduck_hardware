"""Replay a verified partial IK path through the unchanged free-floating plant.

Only uses geometrically accepted stages from a saved+CAD bounded diagnostic.
Retimes joint waypoints to <= half the existing 600tick/s command slew limit,
then reverses the same path. Partial transfer is not single support or walking.
"""
import argparse
import hashlib
import importlib.util
import json
import inspect
import os
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[2]
EVALUATOR=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('candidate','run','reference','output'):p.add_argument('--'+n,type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    os.environ['MPLCONFIGDIR']=str(args.output/'mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import mujoco
    import onnxruntime as ort
    from scipy.spatial import ConvexHull
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from stand_button_training_profile import apply_task_profile
    from mjlab.envs import ManagerBasedRlEnv
    from simulation_evidence import effective_body_properties,world_com,verify_com_at_current_state,audit_interval_push
    c=load_candidate(args.candidate);run=json.loads((args.run/'run.json').read_text())
    reference=json.loads(args.reference.read_text());model_path=args.run/'unvalidated_actor.onnx'
    if not reference['completed'] or reference['candidate_sha256']!=digest(c) or digest(c)!=run['candidate_sha256'] or sha(model_path)!=run['export']['sha256']:
        raise ValueError('Reference/candidate/model binding differs')
    spec=importlib.util.spec_from_file_location('support_replay_measurement',EVALUATOR)
    evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(model_path),opts,providers=['CPUExecutionProvider'])
    sources={str(x):sha(x) for x in (Path(__file__),EVALUATOR,args.candidate,args.reference,args.run/'run.json',model_path,Path(inspect.getfile(world_com)))}
    report=dict(sources=sources,hardware_access=False,reference_controller_root_state_writes_after_reset=0,
        upstream_interval_pushes_retained_and_logged=True,external_support_forces_added=False,
        walking_qualified=False,scope=__doc__,completed=False,cases=[],original_limits_unchanged=True)
    def save():(args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'source.py').write_bytes(Path(__file__).read_bytes());save()
    # Two matched geometric amplitudes, not an unbounded search. The .4 stage
    # has remaining tilt reserve; the final valid stage tests the local boundary.
    for source in reference['cases']:
        valid=[s for s in source['stages'] if s['geometric_witness'] and s['stage']=='transfer']
        if not valid or 'ik_lower_rad' not in source:raise ValueError('Require CAD-aware source')
        for fraction in (.4,valid[-1]['fraction']):
            env=None;rows=[];started=time.monotonic()
            case=dict(swing=source['swing'],transfer_fraction=fraction);report['cases'].append(case);save()
            try:
                torch.manual_seed(0);np.random.seed(0)
                cfg,_=make_mjlab_configs(c);apply_task_profile(cfg,'stand-button')
                cfg.seed=0;cfg.scene.num_envs=1;cfg.auto_reset=False
                ledger=[];case['push_audit']=audit_interval_push(cfg,ledger)
                env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);obs,_=env.reset(seed=0)
                term=env.action_manager.get_term('joint_pos')
                for i in range(250):
                    if time.monotonic()-started>120:raise TimeoutError('Bounded reference replay')
                    if torch.any(obs['actor'][:,48:51]!=0):raise ValueError('Nonzero settling command')
                    action=session.run(None,{session.get_inputs()[0].name:obs['actor'].numpy()})[0]
                    obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
                    if bool(terminated[0]) or bool(truncated[0]) or evaluation.actual_plant_measurements(env)['tilt_deg']>=15:
                        raise ValueError('Source settling failed')
                model=env.sim.mj_model;data=mujoco.MjData(model)
                qpos=env.sim.wp_data.qpos.numpy()[0].copy()
                np.testing.assert_allclose(qpos,source['settled_qpos'],atol=2e-7,rtol=0)
                legs=[c['policy_joint_ids'].index(j) for j in source['joint_ids']]
                actual=term._entity.data.joint_pos[0,term._target_ids].numpy().copy()
                q0=actual[legs].astype(float);points=[q0]
                for stage in valid:
                    if stage['fraction']>fraction+1e-10:break
                    q=np.asarray(stage['joint_rad'])
                    if np.any(q<source['ik_lower_rad']) or np.any(q>source['ik_upper_rad']):raise ValueError('Invalid IK witness range')
                    points.append(q)
                vmax=c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']*.5
                times=[0.]
                for a,b in zip(points[:-1],points[1:]):times.append(times[-1]+max(.2,float(np.max(np.abs(b-a)))/vmax))
                end=times[-1];forward_times=times.copy();forward_points=list(points)
                times.append(end+.25);points.append(points[-1])
                for t,q in zip(reversed(forward_times[:-1]),reversed(forward_points[:-1])):
                    times.append(end+.25+end-t);points.append(q)
                times.append(times[-1]+.5);points.append(q0)
                if 5+times[-1]>=cfg.episode_length_s:raise ValueError('Reference exceeds original episode duration')
                points=np.asarray(points);times=np.asarray(times)
                case.update(reference_duration_s=float(times[-1]),one_way_duration_s=end,
                    maximum_requested_rate_deg_s=float(np.degrees(np.abs(np.diff(points,axis=0))/np.diff(times)[:,None]).max()),
                    original_goal_slew_deg_s=float(np.degrees(vmax)*2),
                    waypoints=[dict(time_s=float(t),joint_rad=q.tolist()) for t,q in zip(times,points)])
                mass,ipos,properties=effective_body_properties(env)
                case['effective_body_properties']=properties
                if properties!=source['effective_body_properties']:raise ValueError('Reference/replay effective mass/COM differs')
                footprints={}
                for side in ('left','right'):
                    gid=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==side+'_foot_collision'][0]
                    mid=int(model.geom_dataid[gid]);s=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid])
                    footprints[side]=(gid,np.asarray(model.mesh_vert[s:s+n],dtype=float))
                for i in range(int(np.ceil(times[-1]/env.step_dt))):
                    if time.monotonic()-started>120:raise TimeoutError('Bounded reference replay')
                    t=(i+1)*env.step_dt;desired=actual.astype(float).copy()
                    desired[legs]=[np.interp(t,times,points[:,j]) for j in range(len(legs))]
                    raw=((desired-np.asarray(c['home_rad']))/c['action_scale']).astype(np.float32)[None,:]
                    obs,_,terminated,truncated,_=env.step(torch.from_numpy(raw));plant=evaluation.actual_plant_measurements(env)
                    data.qpos[:]=env.sim.wp_data.qpos.numpy()[0];mujoco.mj_forward(model,data)
                    com=world_com(data,mass,ipos);verify_com_at_current_state(env,data,mass,ipos);margins={}
                    for side,(gid,vertices) in footprints.items():
                        world=vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
                        hull=ConvexHull(world[:,:2]);margins[side]=float(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2]).max())
                    values={name:value[0].detach().cpu().numpy().copy() for name,value in (
                        ('requested',term.requested_targets),('ema',term._ema),('goal',term._goal),('actual',term._entity.data.joint_pos[:,term._target_ids]))}
                    violations={name:bool((~np.isfinite(v)).any() or (v<np.asarray(c['lower_rad'])).any() or (v>np.asarray(c['upper_rad'])).any()) for name,v in values.items()}
                    reasons=[n for n in env.termination_manager.active_terms if bool(env.termination_manager.get_term(n)[0])]
                    rows.append(dict(index=i,time_s=t,**plant,com_m=com.tolist(),full_foot_projection_margin_m=margins,
                        terminated=bool(terminated[0]),truncated=bool(truncated[0]),termination_terms=reasons,range_violations=violations,
                        **{k+'_rad':v.tolist() for k,v in values.items()}))
                    if bool(terminated[0]) or bool(truncated[0]) or any(violations.values()) or plant['tilt_deg']>=15:break
                final=rows[-1];case.update(completed=True,steps=len(rows),reference_completed=bool(final['time_s']>=times[-1]),
                    interval_pushes=ledger,
                    max_tilt_deg=max(r['tilt_deg'] for r in rows),stop_time_s=final['time_s'],
                    stop_reasons=final['termination_terms'],final_range_violations=final['range_violations'],
                    lift_frames={s:sum(r['feet'][s]['lifted'] for r in rows) for s in ('left','right')},
                    max_goal_actual_error_deg=float(max(np.degrees(np.abs(np.array(r['goal_rad'])-r['actual_rad'])).max() for r in rows)),
                    max_requested_goal_error_deg=float(max(np.degrees(np.abs(np.array(r['requested_rad'])-r['goal_rad'])).max() for r in rows)))
                print(json.dumps({k:case[k] for k in ('swing','transfer_fraction','reference_completed','max_tilt_deg','lift_frames','max_goal_actual_error_deg')}),flush=True)
            finally:
                if env is not None:env.close()
                trace=args.output/f"{case['swing']}-{fraction:.1f}.jsonl"
                trace.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows));case['trace_sha256']=sha(trace);save()
    if any(sha(p)!=s for p,s in sources.items()):raise ValueError('Sources changed')
    report['completed']=True;save()


if __name__=='__main__':main()
