"""Saved-trajectory centroidal contact-wrench analysis, not a controller.

The stored post-step qfrc_constraint is refreshed by mjlab's final forward().
Root generalized forces are transformed by the root Jacobian, rather than
assuming the rotational DOFs use world coordinates. Ground-plane COP and
conditional force/moment changes do not prove actuator or trajectory feasibility.
"""
import hashlib, json, time
from pathlib import Path
import numpy as np
import mujoco
from simulation_evidence import world_com
from diagnose_support_load_bound import hull_xy, minimum_swing_load
from diagnose_capture_balance import closest_on_hull

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def root_wrench(model,data,generalized):
    jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
    body=int(model.jnt_bodyid[0]);origin=data.xpos[body].copy()
    mujoco.mj_jac(model,data,jp,jr,origin,body)
    jac=np.vstack((jp[:,:6],jr[:,:6]))
    force_moment=np.linalg.solve(jac.T,np.asarray(generalized)[:6])
    return force_moment[:3],force_moment[3:],origin


def main():
    started=time.monotonic();cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c=json.loads(cp.read_text())['candidate'];xml=Path(c['reset_placement']['xml_path'])
    assert hashlib.sha256(xml.read_bytes()).hexdigest()==c['reset_placement']['xml_sha256']
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
    joints=np.asarray([model.joint(n).id for n in c['joint_names']]);adr=model.jnt_qposadr[joints]
    assert model.nv==20 and model.jnt_type[0]==mujoco.mjtJoint.mjJNT_FREE
    feet=[]
    for side in ('left','right'):
        gid=model.geom(side+'_foot_collision').id;mid=int(model.geom_dataid[gid])
        a,n=model.mesh_vertadr[mid],model.mesh_vertnum[mid]
        feet.append((gid,model.mesh_vert[a:a+n].astype(float)))
    out=BASE/'centroidal-wrench-v1';out.mkdir(exist_ok=False)
    report=dict(completed=False,hardware_access=False,forward_simulation=False,maximum_wall_seconds=90,
        scope=__doc__,original_limits_unchanged=True,runs=[],sources={})
    paths=[Path(__file__),cp,xml,Path(__file__).with_name('simulation_evidence.py'),
        Path(__file__).with_name('diagnose_support_load_bound.py'),Path(__file__).with_name('diagnose_capture_balance.py'),
        ROOT/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/lib/python3.12/site-packages/mjlab/envs/manager_based_rl_env.py']
    for p in paths:report['sources'][str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/'source.py').write_bytes(Path(__file__).read_bytes())
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    # Independent force application verifies frame, point and virtual-work signs.
    rng=np.random.default_rng(20261011);errors=[]
    for _ in range(16):
        quat=rng.normal(size=4);quat/=np.linalg.norm(quat)
        data.qpos[:7]=np.r_[rng.normal(size=3),quat];mujoco.mj_forward(model,data)
        force=rng.normal(size=3);moment=rng.normal(size=3);point=rng.normal(size=3)
        qforce=np.zeros(model.nv);body=int(model.jnt_bodyid[0])
        mujoco.mj_applyFT(model,data,force,moment,point,body,qforce)
        f,t,origin=root_wrench(model,data,qforce)
        err=max(abs(f-force).max(),abs(t-moment-np.cross(point-origin,force)).max())
        errors.append(float(err));assert err<1e-12
    report['virtual_work_checks']=dict(count=len(errors),maximum_error=max(errors));save()
    dirs=[BASE/'bam-tracking-control-tap-v2']+[BASE/f'transfer-preservation-ablation-v1/preserve-seed-{s}' for s in range(4)]
    try:
        for directory in dirs:
            source=directory/'search.json';s=json.loads(source.read_text());g=s['generations'][0]
            trace=directory/'generation-00.npz';assert hashlib.sha256(trace.read_bytes()).hexdigest()==g['trace_sha256']
            z=np.load(trace);bp=g['env0_effective_body_properties'];mass=np.asarray(bp['body_mass_kg']);ipos=np.asarray(bp['body_ipos_m'])
            assert mass[0]==mass[1]==0 and len(mass)==model.nbody+1
            mass=np.r_[mass[:1],mass[2:]];ipos=np.concatenate((ipos[:1],ipos[2:]))
            records=[];load_errors=[]
            for i in range(len(z['root'])):
                if time.monotonic()-started>90:raise TimeoutError('Bounded wrench analysis')
                if not z['active_before'][i,0]:break
                data.qpos[:7]=z['root'][i,0];data.qpos[adr]=z['actual'][i,0];mujoco.mj_forward(model,data)
                com=world_com(data,mass,ipos);hulls=[]
                for gid,verts in feet:
                    points=verts@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid];hulls.append(hull_xy(points))
                f,t,origin=root_wrench(model,data,z['qfrc_constraint'][i,0])
                # Motors, joint friction and joint stops exert no net external root wrench.
                assert np.max(abs(z['qfrc_actuator'][i,0,:6]))==0
                load_error=float(abs(f[2]-z['load'][i,0].sum()));load_errors.append(load_error)
                assert load_error<1e-4 and f[2]>1.
                hdot=t+np.cross(origin-com,f)
                ground_moment=t+np.cross(origin,f)
                cop=np.array([-ground_moment[1],ground_moment[0]])/f[2]
                decomposition=com[:2]-com[2]*f[:2]/f[2]+np.array([-hdot[1],hdot[0]])/f[2]
                assert abs(cop-decomposition).max()<1e-12
                closest=closest_on_hull(cop,hulls[1]);dp=closest-cop
                force_change=-f[2]/com[2]*dp
                moment_change=f[2]*np.array([dp[1],-dp[0]])
                bound=minimum_swing_load(hulls[0],hulls[1],cop)
                records.append(dict(step=i,time_s=(i+1)*.02,tilt_deg=float(z['tilt'][i,0]),
                    com_m=com.tolist(),contact_force_world_n=f.tolist(),contact_moment_about_com_nm=hdot.tolist(),
                    contact_cop_on_ground_m=cop.tolist(),cop_distance_to_optimistic_stance_m=float(np.linalg.norm(dp)),
                    com_to_cop_translation_acceleration_term_m=(-com[2]*f[:2]/f[2]).tolist(),
                    com_to_cop_angular_momentum_rate_term_m=(np.array([-hdot[1],hdot[0]])/f[2]).tolist(),
                    required_acceleration_change_if_vertical_force_and_hdot_held_m_s2=(force_change/mass.sum()).tolist(),
                    required_hdot_change_if_force_held_nm=moment_change.tolist(),
                    conditional_normal_load_bound=bound,actual_swing_load_fraction=float(z['load'][i,0,0]/f[2]),
                    actual_swing_clearance_m=float(z['height'][i,0,0])))
            finite=[r['conditional_normal_load_bound']['minimum_swing_load_fraction'] for r in records if r['conditional_normal_load_bound']['feasible']]
            run=dict(source=str(directory.relative_to(BASE)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                trace_sha256=g['trace_sha256'],effective_mass_kg=float(mass.sum()),frames=len(records),
                vertical_force_sum_max_error_n=max(load_errors),minimum_dynamic_cop_distance_to_stance_m=min(r['cop_distance_to_optimistic_stance_m'] for r in records),
                dynamic_cop_inside_stance_frames=sum(r['cop_distance_to_optimistic_stance_m']<1e-7 for r in records),
                minimum_conditional_swing_load_fraction=min(finite,default=None),
                ground_plane_lp_infeasible_frames=len(records)-len(finite),
                prepare_end=records[min(34,len(records)-1)],lift_peak=records[min(55,len(records)-1)],records=records)
            report['runs'].append(run);save();print(json.dumps({k:v for k,v in run.items() if k not in ('records','prepare_end','lift_peak')}),flush=True)
        report['completed']=True
    except Exception as e:report['error']=str(e);raise
    finally:report['wall_seconds']=time.monotonic()-started;save()


if __name__=='__main__':main()
