"""File-only gait-aware constant frame map, with hard HOME geometry audit.

Keep the official actor and real safety measurements. Choose among rear-load
HOME compensation solutions using their foot/COM distortion on a qualified
source trajectory, instead of minimizing joint angles alone. This is a local
geometric optimizer, not training, a globally optimal map, or real-plant proof.
"""
import argparse
import copy
import json
import time
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import minimize
from scipy.optimize._numdiff import approx_derivative
from scipy.linalg import qr
from scipy.spatial.transform import Rotation

from audit_pose import set_pose, forward_kinematics
from offline_calibration_overlay import range_snapshot
from offline_mass_distribution import scenario_spec
from solve_loaded_stance import ROOT, foot_vertices, sha


def solve(candidate, previous, trace, mjcf, reference_path, loaded_path):
    source=lambda:mj.MjSpec.from_file(str(Path(mjcf).resolve()))
    before_fn, reference_evidence=scenario_spec(source,reference_path)
    after_fn, loaded_evidence=scenario_spec(source,loaded_path)
    before, after=before_fn().compile(),after_fn().compile()
    d0,d1=mj.MjData(before),mj.MjData(after)
    names=candidate['joint_names'];home=np.asarray(candidate['home_rad'])
    root=after.body('trunk_base').id
    feet=[after.body('ankle_'+s).id for s in ('left','right')]
    joints=[after.joint(n).id for n in names]
    qadr=after.jnt_qposadr[joints]
    rows=[json.loads(line) for line in Path(trace).read_text().splitlines()]
    active=[row for row in rows if row['command_m_s']>.299]
    if len(active)<16:raise ValueError('Qualified forward reference required')
    from evaluate_slow_gait import grade
    if not grade(rows,'forward')['accepted']:raise ValueError('Source trace fails independent gait gate')
    selected=np.unique(np.linspace(0,len(active)-1,16).astype(int))
    poses=[home]+[np.asarray(active[i][key]) for i in selected
                  for key in ('actual_joint_rad','processed_joint_rad')]
    desired=[]
    for q in poses:
        set_pose(before,d0,dict(zip(names,q)),np.eye(3))
        desired.append((d0.xpos[feet].copy(),d0.xmat[feet].reshape(2,3,3).copy(),d0.subtree_com[root,:2].copy()))
    baseline=d0.qpos[:3].copy()
    set_pose(before,d0,dict(zip(names,home)),np.eye(3))
    extent=max(np.ptp(foot_vertices(before,d0,s)[:,0]) for s in ('left','right'))
    low=np.maximum(np.asarray(candidate['lower_rad']),after.jnt_range[joints,0])
    high=np.minimum(np.asarray(candidate['upper_rad']),after.jnt_range[joints,1])
    # These are intersections of the existing physical limits on the chosen
    # reference poses, never an expanded real range or output clip.
    offset_low=np.max(low-np.asarray(poses),axis=0)
    offset_high=np.min(high-np.asarray(poses),axis=0)
    if np.any(offset_low>=offset_high):raise ValueError('No map domain on source poses')
    reach=float(after.stat.extent)
    numerical_units=np.r_[np.full(6,1e-6),np.full(6,extent*1e-4),np.full(2,1e-7)]
    began=time.monotonic();last=[]

    def geometry(x):
        if time.monotonic()-began>120:raise TimeoutError('120s file-only geometry budget')
        rotation=Rotation.from_rotvec([0.,x[3],0.]).as_matrix()
        quaternion=np.empty(4);mj.mju_mat2Quat(quaternion,np.ascontiguousarray(rotation).ravel())
        residuals=[]
        for q,(p0,r0,com0) in zip(poses,desired):
            d1.qpos[:]=after.qpos0
            d1.qpos[:3]=baseline+x[:3];d1.qpos[3:7]=quaternion
            d1.qpos[qadr]=q+x[4:];mj.mj_forward(after,d1)
            angular=np.asarray([Rotation.from_matrix(old.T@now).as_rotvec() for old,now in
                                zip(r0,d1.xmat[feet].reshape(2,3,3))]).ravel()
            residuals.append(np.r_[(d1.xpos[feet]-p0).ravel(),extent*angular,d1.subtree_com[root,:2]-com0])
        return np.asarray(residuals)

    def objective(x):
        errors=geometry(x)
        # HOME must retain the same numerical foot/COM audit. Secondary
        # trajectory errors are expressed as angular-equivalent displacement
        # using the actual sole length, rather than tuning runtime gains.
        return np.r_[errors[0]/numerical_units,(errors[1:]/extent).ravel()]

    initial=np.r_[[previous['trunk_forward_translation_m'],0.,previous['trunk_height_change_m']],
                  previous['body_pitch_rad'],previous['stance_delta_rad']]
    lower=np.r_[np.full(3,-reach),-np.deg2rad(15),offset_low]
    upper=np.r_[np.full(3,reach),np.deg2rad(15),offset_high]
    # Bound only the solver's starting point; outputs are never clipped.
    initial=np.minimum(np.maximum(initial,lower+1e-12),upper-1e-12)
    # Solve the constrained problem directly. The older augmented-residual
    # attempt stalled after300evaluations despite meeting HOME constraints;
    # keep that failure. Eliminate only numerically redundant equality rows,
    # then independently audit ALL original rows after optimization.
    jacobian=approx_derivative(lambda x:geometry(x)[0]/extent,initial,method='3-point')
    _,triangular,pivots=qr(jacobian.T,mode='economic',pivoting=True)
    singular=np.linalg.svd(jacobian,compute_uv=False)
    rank=int(np.count_nonzero(singular>singular[0]*np.sqrt(np.finfo(float).eps)))
    independent=np.sort(pivots[:rank])

    def trajectory_cost(x):
        errors=geometry(x)[1:]/extent
        return .5*float(np.sum(errors**2))

    fit=minimize(trajectory_cost,initial,method='SLSQP',bounds=list(zip(lower,upper)),
        constraints={'type':'eq','fun':lambda x:(geometry(x)[0]/extent)[independent]},
        options={'ftol':1e-12,'maxiter':200})
    errors=geometry(fit.x)
    # Independently audit HOME after geometry() last evaluated a gait pose.
    rotation=Rotation.from_rotvec([0.,fit.x[3],0.]).as_matrix()
    set_pose(after,d1,dict(zip(names,home+fit.x[4:])),rotation)
    d1.qpos[:3]+=fit.x[:3];mj.mj_forward(after,d1)
    rotations,positions=forward_kinematics(after,d1.qpos)
    fk=max(abs(positions-d1.xpos).max(),abs(rotations-d1.xmat.reshape(-1,3,3)).max())
    foot_error=abs(errors[0,:6]).max()
    angle_error=np.linalg.norm(errors[0,6:12].reshape(2,3)/extent,axis=1).max()
    com_error=abs(errors[0,-2:]).max()
    verified=bool(fit.success and foot_error<1e-6 and angle_error<1e-4 and com_error<1e-7 and fk<1e-10)
    report=copy.deepcopy(previous)
    report.update(kind='gait_aware_unmeasured_load_constant_body_frame_hypothesis',
        geometry_verified=verified,solver_success=bool(fit.success),evaluations=fit.nfev,
        optimizer='SLSQP with independent HOME equalities and complete post-audit',
        optimizer_message=str(fit.message),home_jacobian_singular_values=singular.tolist(),
        independent_constraint_indices=independent.tolist(),
        whole_foot_position_error_m=float(foot_error),whole_foot_orientation_error_rad=float(angle_error),
        max_independent_fk_error=float(fk),restored_whole_com_projection_error_m=float(com_error),
        trunk_forward_translation_m=float(fit.x[0]),trunk_lateral_translation_m=float(fit.x[1]),trunk_height_change_m=float(fit.x[2]),
        body_pitch_rad=float(fit.x[3]),body_pitch_deg=float(np.degrees(fit.x[3])),body_frame_rotation=rotation.tolist(),
        stance_delta_rad=fit.x[4:].tolist(),stance_delta_deg=np.degrees(fit.x[4:]).tolist(),
        stance_target_rad=(home+fit.x[4:]).tolist(),
        reference=reference_evidence,loaded=loaded_evidence,
        geometric_check_scope='Unchanged HOME foot/COM audit, now includes lateralCOM; bounded secondary source-gait foot/COM distortion objective, no runtime gain/limit change',
        reference_gait_sha256=sha(trace),reference_gait_samples=len(poses)-1,
        max_fit_foot_position_error_m=float(abs(errors[1:,:6]).max()),
        max_fit_foot_rotation_error_rad=float(np.linalg.norm(errors[1:,6:12].reshape(-1,2,3)/extent,axis=2).max()),
        max_fit_com_projection_error_m=float(abs(errors[1:,-2:]).max()),
        source_sha256=sha(__file__),wall_seconds=time.monotonic()-began,
        objective_scope='Local geometric fit only; independent dynamic evaluation and held-out poses still required')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('initial','trace','reference','loaded','calibration','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve previous experiment')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c,overlay=range_snapshot(json.loads(cp.read_text())['candidate'],a.calibration)
    initial=json.loads(a.initial.read_text())
    if initial['calibration_overlay']['canonical_sha256']!=c['calibration']['sha256']:raise ValueError('Calibration binding differs')
    if initial['loaded']['scenario_sha256']!=sha(a.loaded):raise ValueError('Load hypothesis differs')
    xml=ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'
    report=solve(c,initial,a.trace,xml,a.reference,a.loaded)
    report.update(calibration_overlay=overlay,initial_stance_sha256=sha(a.initial))
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('geometry_verified','body_pitch_deg','max_fit_foot_position_error_m','max_fit_foot_rotation_error_rad','wall_seconds')}))


if __name__=='__main__':main()
