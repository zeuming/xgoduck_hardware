"""Kinematic COM velocity and conditional LIPM capture-point diagnosis.

The constant-height, zero-centroidal-angular-momentum-rate inverted pendulum
is a reduced diagnostic model, not this robot's full dynamics or acceptance
gate. Its velocity corrections are NOT commanded to any simulator or robot.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from simulation_evidence import world_com
from diagnose_support_load_bound import minimum_swing_load

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def closest_on_hull(point,points):
    hull=ConvexHull(points);poly=points[hull.vertices]
    if np.max(hull.equations[:,:2]@point+hull.equations[:,2])<=0:return point.copy()
    candidates=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        d=b-a;t=np.clip(np.dot(point-a,d)/np.dot(d,d),0.,1.);candidates.append(a+t*d)
    return min(candidates,key=lambda p:np.linalg.norm(p-point))


def com_jacobian(model,data,mass,ipos):
    jac=np.zeros((3,model.nv));point_jac=np.zeros_like(jac)
    for i,m in enumerate(mass):
        if m==0:continue
        point=data.xpos[i]+data.xmat[i].reshape(3,3)@ipos[i]
        mujoco.mj_jac(model,data,point_jac,None,point,i)
        jac+=m*point_jac
    return jac/mass.sum()


def main():
    started=time.monotonic();cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c=json.loads(cp.read_text())['candidate'];xml=Path(c['reset_placement']['xml_path'])
    assert hashlib.sha256(xml.read_bytes()).hexdigest()==c['reset_placement']['xml_sha256']
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model);check=mujoco.MjData(model)
    joints=np.asarray([model.joint(n).id for n in c['joint_names']]);adr=model.jnt_qposadr[joints];vadr=model.jnt_dofadr[joints]
    assert model.nv==20 and model.jnt_type[0]==mujoco.mjtJoint.mjJNT_FREE
    np.testing.assert_array_equal(model.opt.gravity[:2],0.);gravity=-model.opt.gravity[2]
    gid=model.geom('right_foot_collision').id;mesh=model.geom_dataid[gid]
    a,n=model.mesh_vertadr[mesh],model.mesh_vertnum[mesh];verts=model.mesh_vert[a:a+n].astype(float)
    load_path=BASE/'support-load-bound-v1/summary.json';loads=json.loads(load_path.read_text());assert loads['completed']
    out=BASE/'capture-balance-v1';out.mkdir(exist_ok=False)
    report=dict(completed=False,hardware_access=False,maximum_wall_seconds=60,scope=__doc__,runs=[],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path(__file__),cp,xml,load_path,Path(__file__).with_name('simulation_evidence.py'),Path(__file__).with_name('diagnose_support_load_bound.py'))})
    (out/'source.py').write_bytes(Path(__file__).read_bytes())
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    # Analytic load-bound checks, independent of the robot model.
    rectangle=np.array([[-.01,-.01],[.01,-.01],[.01,.01],[-.01,.01]])
    left,right=rectangle+[0,.03],rectangle+[0,-.03]
    assert abs(minimum_swing_load(left,right,np.array([0.,0.]))['minimum_swing_load_fraction']-1/3)<1e-10
    assert minimum_swing_load(left,right,np.array([0.,-.025]))['minimum_swing_load_fraction']==0
    assert not minimum_swing_load(left,right,np.array([0.,.1]))['feasible']
    report['analytic_load_checks_passed']=3;save()
    try:
        for lr in loads['runs']:
            if time.monotonic()-started>60:raise TimeoutError('Bounded capture diagnosis')
            directory=BASE/lr['source'];trace=directory/'generation-00.npz';assert hashlib.sha256(trace.read_bytes()).hexdigest()==lr['trace_sha256']
            src=directory/'search.json';s=json.loads(src.read_text());g=s['generations'][0]
            bp=g['env0_effective_body_properties'];mass=np.asarray(bp['body_mass_kg']);ipos=np.asarray(bp['body_ipos_m'])
            assert len(mass)==model.nbody+1 and mass[0]==mass[1]==0
            mass=np.r_[mass[:1],mass[2:]];ipos=np.concatenate((ipos[:1],ipos[2:]))
            z=np.load(trace);rows=[];jac_errors=[]
            for i,previous in enumerate(lr['records']):
                data.qpos[:7]=z['root'][i,0];data.qpos[adr]=z['actual'][i,0]
                # Saved environment actuator layout is [six root, policy14].
                data.qvel[:6]=z['qvel'][i,0,:6];data.qvel[vadr]=z['qvel'][i,0,6:]
                mujoco.mj_forward(model,data);com=world_com(data,mass,ipos)
                np.testing.assert_allclose(com,previous['com_m'],atol=1e-12,rtol=0)
                velocity=com_jacobian(model,data,mass,ipos)@data.qvel
                if i in (0,34,49,55,len(lr['records'])-1):
                    derivatives=[]
                    for sign in (-1,1):
                        check.qpos[:]=data.qpos;mujoco.mj_integratePos(model,check.qpos,data.qvel,sign*1e-6)
                        mujoco.mj_forward(model,check);derivatives.append(world_com(check,mass,ipos))
                    fd=(derivatives[1]-derivatives[0])/(2e-6);error=float(abs(fd-velocity).max());jac_errors.append(error)
                    assert error<1e-6
                points=verts@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
                hull=ConvexHull(points[:,:2]);poly=points[hull.vertices,:2]
                assert com[2]>.01
                omega=float(np.sqrt(gravity/com[2]));capture=com[:2]+velocity[:2]/omega
                closest=closest_on_hull(capture,poly);correction=omega*(closest-capture)
                row=dict(step=i,time_s=(i+1)*.02,com_m=com.tolist(),com_velocity_m_s=velocity.tolist(),
                    stance_projection_xy_m=poly.tolist(),capture_point_m=capture.tolist(),
                    capture_distance_from_stance_m=float(np.linalg.norm(closest-capture)),
                    minimum_lipm_velocity_correction_m_s=correction.tolist(),lipm_omega_s=omega,
                    actual_swing_load_fraction=previous['actual_swing_load_fraction'],tilt_deg=previous['tilt_deg'])
                rows.append(row)
            run=dict(source=lr['source'],trace_sha256=lr['trace_sha256'],jacobian_finite_difference_checks=len(jac_errors),
                maximum_jacobian_velocity_error_m_s=max(jac_errors),frames=len(rows),
                capture_inside_stance_frames=sum(r['capture_distance_from_stance_m']<1e-7 for r in rows),
                minimum_capture_distance_from_stance_m=min(r['capture_distance_from_stance_m'] for r in rows),
                prepare_end=rows[min(34,len(rows)-1)],lift_peak=rows[min(55,len(rows)-1)],records=rows)
            report['runs'].append(run);save();print(json.dumps({k:v for k,v in run.items() if k not in ('records','prepare_end','lift_peak')}),flush=True)
        report['completed']=True
    except Exception as e:report['error']=str(e);raise
    finally:report['wall_seconds']=time.monotonic()-started;save()


if __name__=='__main__':main()
