"""Static counterfactual: does fixed head pose constrain single-foot support?

No actor/action-mask/plant/hardware change. Free-head endpoints are expressly
ineligible for the current fixed-head controller, even if geometry succeeds.
Local optimization is never an impossibility or dynamic stability proof.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from simulation_evidence import world_com

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def main():
    start=time.monotonic()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    sp=BASE/'weight-transfer-multistart-v5/summary.json'
    c=json.loads(cp.read_text())['candidate'];source=json.loads(sp.read_text())
    xml=Path(c['reset_placement']['xml_path'])
    assert hashlib.sha256(xml.read_bytes()).hexdigest()==c['reset_placement']['xml_sha256']
    m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
    joints=np.array([m.joint(n).id for n in c['joint_names']]);adr=m.jnt_qposadr[joints]
    lo=np.maximum(c['lower_rad'],m.jnt_range[joints,0])+1e-6
    hi=np.minimum(c['upper_rad'],m.jnt_range[joints,1])-1e-6
    out=BASE/'head-support-geometry-v1';out.mkdir(exist_ok=False)
    report=dict(completed=False,hardware_access=False,dynamic_validation=False,actor_head_hold_unchanged=True,
        deployment_eligible=False,original_physical_limits_unchanged=True,maximum_wall_seconds=75,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (cp,sp,xml,Path(__file__),Path(__file__).with_name('simulation_evidence.py'))},cases=[])
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'source.py').write_bytes(Path(__file__).read_bytes());save()
    def budget():
        if time.monotonic()-start>75:raise TimeoutError('Bounded geometry diagnostic')
    try:
        for src in source['cases']:
            qpos=np.asarray(src['settled_qpos'],dtype=float);q0=qpos[adr].copy()
            # The saved mjlab model contains one massless namespace body before
            # the CAD trunk. Prove this mapping against its saved COM/sole data.
            mass=np.asarray(src['effective_body_properties']['body_mass_kg'])
            ipos=np.asarray(src['effective_body_properties']['body_ipos_m'])
            assert len(mass)==m.nbody+1 and mass[0]==0 and mass[1]==0
            mass=np.r_[mass[:1],mass[2:]];ipos=np.concatenate((ipos[:1],ipos[2:]))
            check=next(s for s in src['stages'] if s['geometric_witness'])
            d.qpos[:]=check['qpos'];mujoco.mj_forward(m,d)
            np.testing.assert_allclose(world_com(d,mass,ipos),check['com_m'],atol=2e-7,rtol=0)
            swing=src['swing'];stance='right' if swing=='left' else 'left'
            gids={s:m.geom(s+'_foot_collision').id for s in (swing,stance)}
            vertices={}
            for s,gid in gids.items():
                mid=m.geom_dataid[gid];a=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid]
                vertices[s]=m.mesh_vert[a:a+n].astype(float)
            d.qpos[:]=qpos;mujoco.mj_forward(m,d)
            poses={s:(d.geom_xpos[g].copy(),d.geom_xmat[g].reshape(3,3).copy()) for s,g in gids.items()}
            points={s:vertices[s]@poses[s][1].T+poses[s][0] for s in gids}
            hull=ConvexHull(np.asarray(src['support_polygon_xy_m']));planes=hull.equations
            rootrot=Rotation.from_quat(qpos[3:7],scalar_first=True).as_matrix()
            last=next(s for s in reversed(src['stages']) if s['geometric_witness'])
            starts=[('settled',q0.copy()),('last_valid',np.asarray(last['qpos'])[adr])]
            case=dict(swing=swing,cases=[],effective_mass_kg=float(mass.sum()));report['cases'].append(case)
            for mode in ('fixed','free'):
                idx=np.array([i for i,sid in enumerate(c['policy_joint_ids']) if mode=='free' or sid<30])
                for name,initial in starts:
                    budget();cache={}
                    def geometry(v):
                        budget();key=v.tobytes()
                        if cache.get('key')==key:return cache['value']
                        q=q0.copy();q[idx]=v
                        d.qpos[:]=qpos;d.qpos[adr]=q;mujoco.mj_forward(m,d)
                        sg=gids[stance];correction=poses[stance][1]@d.geom_xmat[sg].reshape(3,3).T
                        d.qpos[:3]=poses[stance][0]+correction@(qpos[:3]-d.geom_xpos[sg])
                        rot=correction@rootrot;d.qpos[3:7]=Rotation.from_matrix(rot).as_quat(scalar_first=True)
                        mujoco.mj_forward(m,d);com=world_com(d,mass,ipos);wg=gids[swing]
                        sole=vertices[swing]@d.geom_xmat[wg].reshape(3,3).T+d.geom_xpos[wg]
                        constraints=np.r_[(-(planes[:,:2]@com[:2]+planes[:,2])-.002)/.1,
                            rot[2,2]-np.cos(np.deg2rad(14.9999)),
                            (sole[:,2].min()-points[swing][:,2].min()-.004)/.1]
                        value=(constraints,q,d.qpos.copy(),com,rot,float(sole[:,2].min()))
                        cache.update(key=key,value=value);return value
                    fit=minimize(lambda v:float(np.sum((v-q0[idx])**2)),np.clip(initial[idx],lo[idx],hi[idx]),
                        bounds=list(zip(lo[idx],hi[idx])),method='SLSQP',constraints=[dict(type='ineq',fun=lambda v:geometry(v)[0])],
                        options=dict(maxiter=350,ftol=1e-10))
                    con,q,pose,com,rot,z=geometry(fit.x)
                    valid=bool(fit.success and con.min()>=-1e-7 and np.all(q>=lo) and np.all(q<=hi))
                    rec=dict(mode=mode,start=name,solver_success=bool(fit.success),message=str(fit.message),
                        geometric_witness=valid,minimum_constraint=float(con.min()),iterations=int(fit.nit),
                        tilt_deg=float(np.rad2deg(np.arccos(np.clip(rot[2,2],-1,1)))),
                        support_margin_m=float(-(planes[:,:2]@com[:2]+planes[:,2]).max()),sole_minimum_m=z,
                        joint_rad=q.tolist(),head_change_deg=np.rad2deg(q[5:9]-q0[5:9]).tolist(),qpos=pose.tolist(),
                        current_fixed_head_controller_compatible=mode=='fixed',path_and_torque_validated=False)
                    case['cases'].append(rec);save();print(json.dumps(dict(side=swing,**{k:rec[k] for k in ('mode','start','geometric_witness','tilt_deg','support_margin_m','head_change_deg')})),flush=True)
        report['completed']=True
    except Exception as e:report['error']=str(e);raise
    finally:report['wall_seconds']=time.monotonic()-start;save()


if __name__=='__main__':main()
