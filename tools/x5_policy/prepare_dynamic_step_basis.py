"""Construct bounded joint-space bases for an OFFLINE dynamic step experiment.

Uses a previously verified settled pose and small partial transfer. Foot lift
IK fixes root only to define a geometric direction; it is NOT a stable pose.
The eventual free-floating simulator must validate every trajectory.
"""
import argparse
import hashlib
import json
from pathlib import Path


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','reference','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import mujoco
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    from constrained_training_candidate import load_candidate,digest
    c=load_candidate(a.candidate);r=json.loads(a.reference.read_text())
    if not r['completed'] or r['candidate_sha256']!=digest(c):raise ValueError('Reference mismatch')
    xml=Path(c['reset_placement']['xml_path'])
    if sha(xml)!=c['reset_placement']['xml_sha256']:raise ValueError('XML changed')
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
    joints=[model.joint(n).id for n in c['joint_names']]
    adr=np.asarray(model.jnt_qposadr)[joints]
    low=np.maximum(c['lower_rad'],np.asarray(model.jnt_range)[joints,0])+1e-6
    high=np.minimum(c['upper_rad'],np.asarray(model.jnt_range)[joints,1])-1e-6
    report=dict(scope=__doc__,hardware_access=False,candidate_sha256=digest(c),
        sources={str(p):sha(p) for p in (Path(__file__),a.candidate,a.reference,xml)},
        completed=False,stable_gait_claim=False,cases=[])
    (a.output/'source.py').write_bytes(Path(__file__).read_bytes())
    for source in r['cases']:
        side=source['swing'];settled=np.asarray(source['settled_qpos'])
        if model.nq!=len(settled):raise ValueError('Geometry layout mismatch')
        data.qpos[:]=settled;mujoco.mj_forward(model,data)
        gid=model.geom(side+'_foot_collision').id;pos=data.geom_xpos[gid].copy();rot=data.geom_xmat[gid].reshape(3,3).copy()
        mid=int(model.geom_dataid[gid]);start=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid])
        vertices=np.asarray(model.mesh_vert[start:start+n],dtype=float)
        idx=[i for i,sid in enumerate(c['policy_joint_ids']) if sid in range(10 if side=='left' else 20,15 if side=='left' else 25)]
        q0=settled[adr].copy()
        def geometry(q):
            data.qpos[:]=settled;data.qpos[adr[idx]]=q;mujoco.mj_forward(model,data)
            return data.geom_xpos[gid].copy(),data.geom_xmat[gid].reshape(3,3).copy()
        def residual(q):
            xyz,rotation=geometry(q)
            error=Rotation.from_matrix(rotation@rot.T).as_rotvec()
            return np.r_[(xyz-pos-np.array([0,0,.004]))/.1,error[:2]]
        fit=least_squares(residual,q0[idx],bounds=(low[idx],high[idx]),max_nfev=500,ftol=1e-12,xtol=1e-12,gtol=1e-12)
        xyz,rotation=geometry(fit.x);height=float((vertices@rotation.T+xyz)[:,2].min())
        if np.max(np.abs(residual(fit.x)))>1e-5 or height<.003 or not fit.success:
            raise ValueError(f'No flat foot lift witness for {side}: {fit.message}, error={residual(fit.x)}, height={height}')
        lift=np.zeros(14);lift[idx]=fit.x-q0[idx]
        stage=[s for s in source['stages'] if s['geometric_witness'] and abs(s['fraction']-.4)<1e-9][0]
        transfer=np.zeros(14)
        for sid,q in zip(source['joint_ids'],stage['joint_rad']):
            i=c['policy_joint_ids'].index(sid);transfer[i]=q-q0[i]
        case=dict(side=side,settled_policy_joint_rad=q0.tolist(),lift_basis_rad=lift.tolist(),transfer_basis_rad=transfer.tolist(),
            foot_lift_target_m=.004,minimum_foot_height_m=height,residual=residual(fit.x).tolist(),
            maximum_lift_delta_deg=float(np.degrees(np.abs(lift)).max()),
            root_fixed_only_during_basis_construction=True,contact_dynamics_validated=False)
        report['cases'].append(case);print(json.dumps(case),flush=True)
    report.update(completed=True,lower_intersection_rad=low.tolist(),upper_intersection_rad=high.tolist())
    (a.output/'basis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
