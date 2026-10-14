"""Offline support-aware IK reference; separate from the learned walking policy.

Stance sole pose determines floating root only during geometric optimization.
The subsequent dynamics uses normal free-floating physics and joint commands;
it never writes root state or adds support forces. No hardware access.
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
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','run','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--wall-seconds',type=float,default=240.)
    parser.add_argument('--swing-mode',choices=('fixed-pose','free-pose'),default='fixed-pose')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    os.environ['MPLCONFIGDIR']=str(args.output/'mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import mujoco
    import onnxruntime as ort
    from scipy.optimize import minimize
    from scipy.spatial import ConvexHull
    from scipy.spatial.transform import Rotation
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from stand_button_training_profile import apply_task_profile
    from mjlab.envs import ManagerBasedRlEnv
    from simulation_evidence import effective_body_properties,world_com,verify_com_at_current_state
    candidate=load_candidate(args.candidate)
    run=json.loads((args.run/'run.json').read_text());model_path=args.run/'unvalidated_actor.onnx'
    if digest(candidate)!=run['candidate_sha256'] or sha(model_path)!=run['export']['sha256']:
        raise ValueError('Candidate/source model binding differs')
    spec=importlib.util.spec_from_file_location('support_measurements',EVALUATOR)
    evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
    sources={str(p):sha(p) for p in (Path(__file__),args.candidate,args.run/'run.json',model_path,EVALUATOR,Path(inspect.getfile(world_com)))}
    report=dict(sources=sources,candidate_sha256=digest(candidate),hardware_access=False,
        experiment='support-aware geometric reference, not trained policy or validation-gate change',
        static_reserve_m=.002,sole_vertex_band_m=.001,lift_center_delta_m=.004,
        geometric_tilt_bound_deg=14.9999,dynamic_stop_tilt_deg=15.,
        original_limits_unchanged=True,dynamic_root_constraint=False,swing_mode=args.swing_mode,
        completed=False,cases=[])
    def save():
        (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'source.py').write_bytes(Path(__file__).read_bytes());save()
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(model_path),opts,providers=['CPUExecutionProvider'])
    started=time.monotonic()
    def budget():
        if time.monotonic()-started>args.wall_seconds:raise TimeoutError('Bounded support diagnostic')
    for swing in ('left','right'):
        env=None;rows=[];case=dict(swing=swing,stages=[]);report['cases'].append(case);save()
        try:
            torch.manual_seed(0);np.random.seed(0)
            cfg,_=make_mjlab_configs(candidate);apply_task_profile(cfg,'stand-button')
            cfg.seed=0;cfg.scene.num_envs=1;cfg.auto_reset=False
            env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);obs,_=env.reset(seed=0)
            for index in range(250):
                budget()
                if torch.any(obs['actor'][:,48:51]!=0):raise ValueError('Settling command not zero')
                action=session.run(None,{session.get_inputs()[0].name:obs['actor'].numpy()})[0]
                obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
                plant=evaluation.actual_plant_measurements(env)
                if bool(terminated[0]) or bool(truncated[0]) or plant['tilt_deg']>=15:
                    raise ValueError('Source zero settling failed')
            model=env.sim.mj_model;data=mujoco.MjData(model)
            settled=np.asarray(env.sim.wp_data.qpos.numpy()[0],dtype=float).copy()
            data.qpos[:]=settled;mujoco.mj_forward(model,data)
            addresses=[];jointids=[]
            for name in candidate['joint_names']:
                joints=[j for j in range(model.njnt) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j) or '').split('/')[-1]==name]
                if len(joints)!=1:raise ValueError('Ambiguous joint '+name)
                addresses.append(int(model.jnt_qposadr[joints[0]]))
                jointids.append(joints[0])
            legs=[i for i,sid in enumerate(candidate['policy_joint_ids']) if sid<30]
            legadr=np.asarray(addresses)[legs];q0=settled[legadr].copy()
            low=np.asarray(candidate['lower_rad'])[legs];high=np.asarray(candidate['upper_rad'])[legs]
            # IK must respect both independent bound sources. CAD hard stops
            # remain in the dynamics; saved operating ranges do not replace them.
            cad_limited=np.asarray(model.jnt_limited)[np.asarray(jointids)[legs]].astype(bool)
            cad_ranges=np.asarray(model.jnt_range)[np.asarray(jointids)[legs]]
            low=np.maximum(low,np.where(cad_limited,cad_ranges[:,0],-np.inf))
            high=np.minimum(high,np.where(cad_limited,cad_ranges[:,1],np.inf))
            # Inward numerical margin avoids float32 rounding a request beyond
            # the exact saved/CAD bound; it does not widen either bound.
            low+=1e-6;high-=1e-6
            if np.any(low>=high) or np.any(q0<low) or np.any(q0>high):raise ValueError('Settled pose or bound intersection invalid')
            mass,ipos,properties=effective_body_properties(env)
            case['effective_body_properties']=properties
            case['independent_com_error_m']=verify_com_at_current_state(env,data,mass,ipos)
            case.update(settled_qpos=settled.tolist(),model_total_mass_kg=float(mass.sum()),
                joint_ids=[candidate['policy_joint_ids'][i] for i in legs],ik_lower_rad=low.tolist(),ik_upper_rad=high.tolist(),
                cad_limited=cad_limited.tolist(),cad_ranges_rad=cad_ranges.tolist())
            geom={};vertices={};basepos={};baserot={};basepoints={}
            for side in ('left','right'):
                gids=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==side+'_foot_collision']
                if len(gids)!=1:raise ValueError('Ambiguous sole')
                gid=gids[0];geom[side]=gid;mid=int(model.geom_dataid[gid])
                start=int(model.mesh_vertadr[mid]);count=int(model.mesh_vertnum[mid])
                vertices[side]=np.asarray(model.mesh_vert[start:start+count],dtype=float)
                basepos[side]=data.geom_xpos[gid].copy();baserot[side]=data.geom_xmat[gid].reshape(3,3).copy()
                basepoints[side]=vertices[side]@baserot[side].T+basepos[side]
                np.testing.assert_allclose(basepos[side],env.sim.data.geom_xpos[0,gid].numpy(),atol=2e-7,rtol=0)
            stance='right' if swing=='left' else 'left';sg=geom[stance];wg=geom[swing]
            sole=basepoints[stance];bottom=sole[sole[:,2]<=sole[:,2].min()+.001]
            hull=ConvexHull(bottom[:,:2]);planes=hull.equations.copy()
            rootrot=Rotation.from_quat(settled[3:7],scalar_first=True).as_matrix()
            cache={}
            def geometry(q):
                budget()
                key=np.asarray(q).tobytes()
                if cache.get('key')==key:return cache['value']
                data.qpos[:]=settled;data.qpos[legadr]=q;mujoco.mj_forward(model,data)
                correction=baserot[stance]@data.geom_xmat[sg].reshape(3,3).T
                position=basepos[stance]+correction@(settled[:3]-data.geom_xpos[sg])
                rotation=correction@rootrot
                data.qpos[:3]=position;data.qpos[3:7]=Rotation.from_matrix(rotation).as_quat(scalar_first=True)
                mujoco.mj_forward(model,data)
                com=world_com(data,mass,ipos)
                points=vertices[swing]@data.geom_xmat[wg].reshape(3,3).T+data.geom_xpos[wg]
                value=dict(com=com,margin=float(-(planes[:,:2]@com[:2]+planes[:,2]).max()),
                    cosine=float(rotation[2,2]),tilt=float(np.degrees(np.arccos(np.clip(rotation[2,2],-1,1)))),
                    swing_position=data.geom_xpos[wg].copy(),swing_rotation=data.geom_xmat[wg].reshape(3,3).copy(),
                    swing_min_z=float(points[:,2].min()),qpos=data.qpos.copy(),
                    stance_position_error=float(np.linalg.norm(data.geom_xpos[sg]-basepos[stance])),
                    stance_rotation_error=float(np.linalg.norm(data.geom_xmat[sg].reshape(3,3)-baserot[stance])))
                cache.update(key=key,value=value);return value
            initial=geometry(q0);case['initial_support_margin_m']=initial['margin']
            case['support_polygon_xy_m']=bottom[hull.vertices,:2].tolist()
            case['support_scope']='bottom 1mm mesh projection at settled pose; necessary geometric condition, not contact-force proof'
            # Initial double support can have COM outside either individual sole.
            # Progressively move its half-plane constraint to 2mm within stance.
            q=q0.copy();waypoints=[(0.,q0.copy())]
            stages=[('transfer',f,0.,2.*f) for f in np.linspace(.1,1.,10)]
            stages += [('lift',1.,.004*f,2.+f) for f in np.linspace(.1,1.,10)]
            feasible=True
            for label,f,height,t in stages:
                targetpos=basepos[swing]+np.array([0,0,height])
                margin=initial['margin']+( .002-initial['margin'])*f
                def equality(v):
                    g=geometry(v)
                    rot=Rotation.from_matrix(g['swing_rotation']@baserot[swing].T).as_rotvec()
                    return np.r_[(g['swing_position']-targetpos)/.1,rot]
                def inequality(v):
                    g=geometry(v)
                    return np.r_[(-(planes[:,:2]@g['com'][:2]+planes[:,2])-margin)/.1,
                        g['cosine']-np.cos(np.radians(14.9999)),
                        (g['swing_min_z']-basepoints[swing][:,2].min()-height)/.1]
                previous=q.copy()
                constraints=[dict(type='ineq',fun=inequality)]
                if args.swing_mode=='fixed-pose':constraints.append(dict(type='eq',fun=equality))
                def objective(v):
                    cost=np.sum((v-q0)**2)+.05*np.sum((v-previous)**2)
                    # A soft displacement cost is NOT a hidden fixed sole.
                    if args.swing_mode=='free-pose':cost+=np.sum(equality(v)**2)
                    return float(cost)
                result=minimize(objective,q,
                    method='SLSQP',bounds=list(zip(low,high)),constraints=constraints,
                    options=dict(maxiter=250,ftol=1e-10))
                q=result.x;g=geometry(q)
                good=bool(result.success and (args.swing_mode!='fixed-pose' or np.max(np.abs(equality(q)))<1e-6) and inequality(q).min()>=-1e-7
                    and np.all(q>=low) and np.all(q<=high))
                item=dict(stage=label,fraction=float(f),height_m=float(height),time_s=float(t),solver_success=bool(result.success),
                    message=result.message,geometric_witness=good,iterations=int(result.nit),joint_rad=q.tolist(),
                    max_delta_deg=float(np.degrees(np.abs(q-q0).max())),tilt_deg=g['tilt'],com_m=g['com'].tolist(),
                    support_margin_m=g['margin'],swing_min_height_m=g['swing_min_z'],
                    equality_max_error=float(np.abs(equality(q)).max()),inequality_min=float(inequality(q).min()),
                    qpos=g['qpos'].tolist(),stance_position_error_m=g['stance_position_error'],stance_rotation_error=g['stance_rotation_error'])
                case['stages'].append(item);save();print(json.dumps({k:item[k] for k in ('stage','fraction','geometric_witness','max_delta_deg','tilt_deg','support_margin_m')}),flush=True)
                if not good:feasible=False;break
                waypoints.append((t,q.copy()))
            case['geometric_path_found']=feasible
            case['walking_qualified']=False
            if not feasible:
                # A failed local homotopy is not an infeasibility proof. Try a
                # declared three-start endpoint solve, still within all limits.
                # Even a valid endpoint would not establish an executable path.
                margin=.002;height=.004;targetpos=basepos[swing]+np.array([0,0,height])
                accepted=[s for s in case['stages'] if s['geometric_witness']]
                last=np.asarray(accepted[-1]['joint_rad']) if accepted else q0
                seeds=[('settled',q0),('last_valid',last),('bound_midpoint',(low+high)/2)]
                case['endpoint_multistart']=[]
                for seed_name,seed in seeds:
                    result=minimize(lambda v:float(np.sum((v-q0)**2)),seed,method='SLSQP',
                        bounds=list(zip(low,high)),constraints=[dict(type='ineq',fun=inequality)],
                        options=dict(maxiter=400,ftol=1e-10))
                    g=geometry(result.x)
                    good=bool(result.success and inequality(result.x).min()>=-1e-7
                        and np.all(result.x>=low) and np.all(result.x<=high))
                    case['endpoint_multistart'].append(dict(seed=seed_name,solver_success=bool(result.success),
                        message=result.message,geometric_witness=good,iterations=int(result.nit),
                        joint_rad=result.x.tolist(),tilt_deg=g['tilt'],support_margin_m=g['margin'],
                        swing_min_height_m=g['swing_min_z'],qpos=g['qpos'].tolist(),
                        path_validated=False,torque_validated=False,collision_path_validated=False))
                    save()
            if feasible:
                # Free-floating dynamical test: no root or foot constraints,
                # no plant/gain/EMA/slew/range changes and no external forces.
                term=env.action_manager.get_term('joint_pos');home=np.asarray(candidate['home_rad'])
                transfer=waypoints[10][1].copy();peak=q.copy()
                waypoints.extend([(3.5,peak),(4.5,transfer),(6.5,q0),(7.0,q0)])
                times=np.array([t for t,_ in waypoints]);qs=np.array([q for _,q in waypoints])
                complete=True
                for index in range(round(7./env.step_dt)):
                    budget();t=(index+1)*env.step_dt
                    desired=settled[addresses].copy()
                    desired[legs]=[np.interp(t,times,qs[:,j]) for j in range(len(legs))]
                    action=((desired-home)/candidate['action_scale']).astype(np.float32)[None,:]
                    obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
                    plant=evaluation.actual_plant_measurements(env)
                    values={name:value[0].detach().cpu().numpy().copy() for name,value in (
                        ('requested',term.requested_targets),('goal',term._goal),('actual',term._entity.data.joint_pos[:,term._target_ids]))}
                    violations={name:bool((~np.isfinite(v)).any() or (v<np.asarray(candidate['lower_rad'])).any() or (v>np.asarray(candidate['upper_rad'])).any()) for name,v in values.items()}
                    reasons=[n for n in env.termination_manager.active_terms if bool(env.termination_manager.get_term(n)[0])]
                    rows.append(dict(index=index,reference_time_s=t,**plant,range_violations=violations,termination_terms=reasons,
                        **{k+'_rad':v.tolist() for k,v in values.items()}))
                    if bool(terminated[0]) or bool(truncated[0]) or any(violations.values()) or plant['tilt_deg']>=15:
                        complete=False;break
                case['dynamic']=dict(completed_reference=complete,steps=len(rows),max_tilt_deg=max(r['tilt_deg'] for r in rows),
                    qualified_lift_frames={s:sum(r['feet'][s]['lifted'] for r in rows) for s in ('left','right')},
                    maximum_goal_actual_error_deg=float(max(np.degrees(np.abs(np.array(r['goal_rad'])-r['actual_rad'])).max() for r in rows)),
                    final_termination_terms=rows[-1]['termination_terms'],final_time_s=rows[-1]['reference_time_s'],
                    root_never_constrained=True,no_policy_training=True)
                case['joint_reference_waypoints']=[dict(time_s=t,joint_rad=v.tolist()) for t,v in waypoints]
            case['completed']=True
        finally:
            if env is not None:env.close()
            trace=args.output/(swing+'-dynamic.jsonl');trace.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
            case['trace_sha256']=sha(trace);save()
    if any(sha(p)!=s for p,s in sources.items()):raise ValueError('Source changed while running')
    report.update(completed=True,wall_seconds=time.monotonic()-started);save()


if __name__=='__main__':main()
