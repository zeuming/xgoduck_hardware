"""Offline fixed-root foot-clearance feasibility; NOT a dynamically stable gait.

Freeze the settled root and other leg, seek a close swing-leg configuration
inside saved ranges with every collision-mesh vertex >=3mm above the ground.
SLSQP is local: success gives a geometric witness, failure is not infeasibility.
No actuation, contact balance, collision-free path or hardware claim follows.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','run','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ['MPLCONFIGDIR']=str(args.output/'mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import mujoco
    import onnxruntime as ort
    from scipy.optimize import minimize
    from scipy.spatial import ConvexHull
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from stand_button_training_profile import apply_task_profile
    from mjlab.envs import ManagerBasedRlEnv
    candidate=load_candidate(args.candidate)
    run=json.loads((args.run/'run.json').read_text())
    model_path=args.run/'unvalidated_actor.onnx'
    if digest(candidate)!=run['candidate_sha256'] or sha(model_path)!=run['export']['sha256']:
        raise ValueError('Candidate/source model binding differs')
    torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(0);np.random.seed(0)
    cfg,_=make_mjlab_configs(candidate);apply_task_profile(cfg,'stand-button')
    cfg.seed=0;cfg.scene.num_envs=1;cfg.auto_reset=False
    report=dict(source_sha256={str(p):sha(p) for p in (Path(__file__),args.candidate,args.run/'run.json',model_path)},
        hardware_access=False,optimizer_steps=0,scope=__doc__,completed=False,cases=[])
    def save():
        (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.output/'source.py').write_bytes(Path(__file__).read_bytes());save()
    env=None;started=time.monotonic()
    try:
        env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
        obs,_=env.reset(seed=0)
        opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
        session=ort.InferenceSession(str(model_path),opts,providers=['CPUExecutionProvider'])
        for i in range(250):
            if time.monotonic()-started>120:raise TimeoutError('Kinematic diagnostic time budget')
            if torch.any(obs['actor'][:,48:51]!=0):raise ValueError('Settling command is not zero')
            action=session.run(None,{session.get_inputs()[0].name:obs['actor'].numpy()})[0]
            obs,_,terminated,truncated,_=env.step(torch.from_numpy(action))
            gravity=env.scene['robot'].data.projected_gravity_b[0]
            tilt=float(torch.rad2deg(torch.acos((-gravity[2]/gravity.norm()).clamp(-1,1))))
            if bool(terminated[0]) or bool(truncated[0]) or tilt>=15:
                raise ValueError('Zero settling did not pass')
        model=env.sim.mj_model;data=mujoco.MjData(model)
        settled=np.asarray(env.sim.wp_data.qpos.numpy()[0],dtype=float).copy()
        data.qpos[:]=settled;mujoco.mj_forward(model,data)
        report['settled_qpos']=settled.tolist();report['settled_tilt_deg']=tilt
        mass=np.asarray(model.body_mass,dtype=float)
        report['model_total_mass_kg']=float(mass.sum())
        dt=float(env.step_dt);limit_deg_s=600*360/4095
        for side,servo_ids in [('left',list(range(10,15))),('right',list(range(20,25)))]:
            indices=[candidate['policy_joint_ids'].index(sid) for sid in servo_ids]
            names=[candidate['joint_names'][i] for i in indices]
            addresses=[]
            for name in names:
                joint=[j for j in range(model.njnt) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j) or '').split('/')[-1]==name]
                if len(joint)!=1:raise ValueError('Ambiguous model joint '+name)
                addresses.append(int(model.jnt_qposadr[joint[0]]))
            gid=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==side+'_foot_collision']
            if len(gid)!=1:raise ValueError('Ambiguous foot collision mesh')
            gid=gid[0];mid=int(model.geom_dataid[gid]);start=int(model.mesh_vertadr[mid]);count=int(model.mesh_vertnum[mid])
            vertices=np.asarray(model.mesh_vert[start:start+count],dtype=float)
            q0=settled[addresses].copy()
            lower=np.asarray(candidate['lower_rad'])[indices];upper=np.asarray(candidate['upper_rad'])[indices]
            def vertices_at(q):
                if time.monotonic()-started>120:raise TimeoutError('Kinematic diagnostic time budget')
                data.qpos[:]=settled;data.qpos[addresses]=q;mujoco.mj_forward(model,data)
                return vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
            baseline=vertices_at(q0)
            warp_position=env.sim.data.geom_xpos[0,gid].numpy()
            warp_rotation=env.sim.data.geom_xmat[0,gid].numpy().reshape(3,3)
            warp_vertices=vertices@warp_rotation.T+warp_position
            np.testing.assert_allclose(baseline,warp_vertices,atol=2e-7,rtol=0)
            def constraints(x):
                delta=x[:5]-q0
                return np.r_[x[5]-delta,x[5]+delta,vertices_at(x[:5])[:,2]-.003]
            # Auxiliary t minimizes the largest angular displacement. Bounds
            # are the exact saved ranges, no widening or target clipping.
            result=minimize(lambda x:x[5],np.r_[q0,0.],method='SLSQP',
                bounds=list(zip(lower,upper))+[(0.,np.pi)],
                constraints=[dict(type='ineq',fun=constraints)],options=dict(maxiter=200,ftol=1e-10))
            target=result.x[:5];points=vertices_at(target)
            # This uses the entire projected opposite-foot mesh, an optimistic
            # superset of actual contact support. Negative margin therefore
            # rules out static single support for this fixed-root endpoint;
            # positive margin alone would NOT prove contact/torque feasibility.
            stance='right' if side=='left' else 'left'
            stance_gid=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==stance+'_foot_collision']
            if len(stance_gid)!=1:raise ValueError('Ambiguous stance foot')
            sg=stance_gid[0];sm=int(model.geom_dataid[sg]);ss=int(model.mesh_vertadr[sm]);sc=int(model.mesh_vertnum[sm])
            sv=np.asarray(model.mesh_vert[ss:ss+sc],dtype=float)@data.geom_xmat[sg].reshape(3,3).T+data.geom_xpos[sg]
            hull=ConvexHull(sv[:,:2])
            com=(mass[:,None]*data.xipos).sum(axis=0)/mass.sum()
            margin=float(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2]).max())
            clearance=float(points[:,2].min());in_range=bool(np.all(target>=lower) and np.all(target<=upper))
            witnesses=bool(result.success and clearance>=.003-1e-8 and in_range)
            maximum=float(np.abs(target-q0).max()*180/np.pi)
            case=dict(side=side,ids=servo_ids,solver_success=bool(result.success),solver_message=result.message,
                geometric_witness_found=witnesses,initial_joint_rad=q0.tolist(),target_joint_rad=target.tolist(),
                delta_deg=((target-q0)*180/np.pi).tolist(),maximum_delta_deg=maximum,
                saved_range_valid=in_range,initial_min_height_m=float(baseline[:,2].min()),target_min_height_m=clearance,
                reference_goal_speed_deg_s=limit_deg_s,
                travel_time_lower_bound_for_this_target_s=maximum/limit_deg_s,
                whole_robot_com_m=com.tolist(),opposite_foot_projection_margin_m=margin,
                support_projection_necessary_condition=margin>=0,
                support_scope='entire opposite-foot XY mesh hull, optimistic versus true contact footprint; not a contact force or actuator torque check',
                semantics='local fixed-root endpoint only; no claim of global minimum, collision-free path, support/torque feasibility or walking')
            report['cases'].append(case);save();print(json.dumps(case),flush=True)
        report['completed']=True
    finally:
        if env is not None:env.close()
        report['wall_seconds']=time.monotonic()-started;save()


if __name__=='__main__':main()
