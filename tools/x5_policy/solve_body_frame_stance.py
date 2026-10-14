"""File-only body-pitch and leg-coordinate stance hypothesis.

Solve the existing rear-load geometry while retaining both complete foot
poses and the reference COM projection. Minimize total squared joint/body
rotation away from the official HOME. The resulting pitch is
computed, never chosen as a trim parameter. This does not identify the real
load, certify balance, edit calibration, or authorize hardware commands.
"""
import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from audit_pose import forward_kinematics, set_pose
from offline_calibration_overlay import range_snapshot
from offline_mass_distribution import scenario_spec
from solve_loaded_stance import foot_vertices, sha, ROOT

HIP_IDS = (10, 11, 12, 20, 21, 22)
LEG_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24)


def solve(candidate, mjcf, reference_path, loaded_path):
    source = lambda: mj.MjSpec.from_file(str(Path(mjcf).resolve()))
    ref_fn, ref_evidence = scenario_spec(source, reference_path)
    load_fn, load_evidence = scenario_spec(source, loaded_path)
    ref, model = ref_fn().compile(), load_fn().compile()
    before, data = mj.MjData(ref), mj.MjData(model)
    home = np.asarray(candidate['home_rad'])
    names = candidate['joint_names']
    for m, d in ((ref, before), (model, data)):
        set_pose(m, d, dict(zip(names, home)), np.eye(3))
        d.qpos[2] -= min(foot_vertices(m, d, s)[:, 2].min() for s in ('left', 'right'))
        mj.mj_forward(m, d)
    baseline = data.qpos.copy()
    root = model.body('trunk_base').id
    feet = [model.body('ankle_'+s).id for s in ('left', 'right')]
    feet_positions, feet_rotations = before.xpos[feet].copy(), before.xmat[feet].reshape(2, 3, 3).copy()
    com_x = before.subtree_com[root, 0]
    initial_backshift = com_x-data.subtree_com[root, 0]
    indices = [candidate['policy_joint_ids'].index(s) for s in HIP_IDS]
    joints = [model.joint(names[i]).id for i in indices]
    addresses = model.jnt_qposadr[joints]
    low = np.maximum(np.asarray(candidate['lower_rad'])[indices], model.jnt_range[joints, 0])
    high = np.minimum(np.asarray(candidate['upper_rad'])[indices], model.jnt_range[joints, 1])
    extent = max(np.ptp(foot_vertices(ref, before, s)[:, 0]) for s in ('left', 'right'))

    def residual(x):
        data.qpos[:] = baseline
        data.qpos[[0, 2]] = x[:2]
        rotation = Rotation.from_rotvec([0., x[2], 0.]).as_matrix()
        mj.mju_mat2Quat(data.qpos[3:7], np.ascontiguousarray(rotation).ravel())
        data.qpos[addresses] = x[3:]
        mj.mj_forward(model, data)
        orientations = [Rotation.from_matrix(old.T @ now).as_rotvec() for old, now in
                        zip(feet_rotations, data.xmat[feet].reshape(2, 3, 3))]
        return np.r_[(data.xpos[feet]-feet_positions).ravel(),
                     extent*np.asarray(orientations).ravel(),
                     data.subtree_com[root, 0]-com_x]

    initial = np.r_[baseline[[0, 2]], 0., home[indices]]
    reach = float(model.stat.extent)
    #15deg is the existing offline observation boundary, not a new runtime
    # lean allowance. CAD and saved joint ranges are both retained.
    hip_fit = least_squares(residual, initial,
        bounds=(np.r_[-reach, .001, -np.deg2rad(15.), low],
                np.r_[reach, reach, np.deg2rad(15.), high]),
        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=500, diff_step=1e-5)
    # Rounded/asymmetric hinge frames leave a small position residual if the
    # distal joints are fixed exactly. Project this hip-based solution onto
    # the complete bilateral foot+COM constraints using all ten leg joints.
    # No constraint is relaxed and no candidate angle is hand selected.
    first_angles = home.copy()
    first_angles[indices] = hip_fit.x[3:]
    hip_residual = residual(hip_fit.x).copy()
    indices = [candidate['policy_joint_ids'].index(s) for s in LEG_IDS]
    joints = [model.joint(names[i]).id for i in indices]
    addresses = model.jnt_qposadr[joints]
    low = np.maximum(np.asarray(candidate['lower_rad'])[indices], model.jnt_range[joints, 0])
    high = np.minimum(np.asarray(candidate['upper_rad'])[indices], model.jnt_range[joints, 1])
    fit = least_squares(residual, np.r_[hip_fit.x[:3], first_angles[indices]],
        bounds=(np.r_[-reach, .001, -np.deg2rad(15.), low],
                np.r_[reach, reach, np.deg2rad(15.), high]),
        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=500, diff_step=1e-5)
    # The geometric constraints alone admit multiple postures. Select a
    # reproducible small-change posture, rather than exporting whichever
    # point an underdetermined solve happened to find. All14 policy joints
    # (including model-controlled head) are allowed inside the same bounds.
    projected = home.copy(); projected[indices] = fit.x[3:]
    indices = list(range(14))
    joints = [model.joint(names[i]).id for i in indices]
    addresses = model.jnt_qposadr[joints]
    low = np.maximum(np.asarray(candidate['lower_rad']), model.jnt_range[joints, 0])
    high = np.minimum(np.asarray(candidate['upper_rad']), model.jnt_range[joints, 1])
    numerical_units = np.r_[np.full(6, 1e-6), np.full(6, extent*1e-4), 1e-7]

    def constrained_small_change(x):
        return np.r_[residual(x)/numerical_units, x[2], x[3:]-home]

    fit = least_squares(constrained_small_change, np.r_[fit.x[:3], projected],
        bounds=(np.r_[-reach, .001, -np.deg2rad(15.), low],
                np.r_[reach, reach, np.deg2rad(15.), high]),
        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=1000, diff_step=1e-5)
    error = residual(fit.x)
    rotations, positions = forward_kinematics(model, data.qpos)
    fk_error = max(abs(positions-data.xpos).max(), abs(rotations-data.xmat.reshape(-1, 3, 3)).max())
    foot_error = abs(data.xpos[feet]-feet_positions).max()
    orientation_error = max(np.linalg.norm(Rotation.from_matrix(old.T @ now).as_rotvec())
        for old, now in zip(feet_rotations, data.xmat[feet].reshape(2, 3, 3)))
    offset = np.zeros(14); offset[indices] = fit.x[3:]-home[indices]
    rotation = Rotation.from_rotvec([0., fit.x[2], 0.]).as_matrix()
    valid = (fit.success and foot_error<1e-6 and orientation_error<1e-4 and
             abs(error[-1])<1e-7 and fk_error<1e-10)
    return dict(kind='unmeasured_load_body_frame_stance_hypothesis', hardware_access=False,
        hardware_eligible=False, dynamic_stability_verified=False, geometry_verified=bool(valid),
        solver_success=bool(fit.success), evaluations=fit.nfev, residual_m=error.tolist(),
        hip_only_residual_m=hip_residual.tolist(),
        whole_foot_position_error_m=float(foot_error), whole_foot_orientation_error_rad=float(orientation_error),
        max_independent_fk_error=float(fk_error),
        initial_whole_com_backshift_m=float(initial_backshift),
        restored_whole_com_projection_error_m=float(error[-1]),
        trunk_forward_translation_m=float(fit.x[0]-baseline[0]),
        trunk_height_change_m=float(fit.x[1]-baseline[2]), body_pitch_rad=float(fit.x[2]),
        body_pitch_deg=float(np.degrees(fit.x[2])), body_frame_rotation=rotation.tolist(),
        stance_delta_rad=offset.tolist(), stance_delta_deg=np.degrees(offset).tolist(),
        stance_target_rad=(home+offset).tolist(), home_rad=home.tolist(),
        policy_joint_ids=candidate['policy_joint_ids'], reference=ref_evidence, loaded=load_evidence,
        actor_sha256=candidate['source_model']['sha256'],
        geometric_check_scope='Feet1um/orientation0.0001rad/COM0.1um numerical audit; final objective minimum sum of squared body/joint angular changes; numerical residual scaling is audit units, not runtime parameters',
        root_state_usage='Initial reset only; no dynamic root writes',
        unknown=['Unmeasured rear mass/height/inertia', 'Real joint and IMU alignment', 'Loaded contact and actuator dynamics'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('reference', 'loaded', 'calibration', 'output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists(): raise FileExistsError('Preserve earlier experiments')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    candidate, overlay=range_snapshot(json.loads(cp.read_text())['candidate'],a.calibration)
    mjcf=ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'
    report=solve(candidate,mjcf,a.reference,a.loaded)
    report.update(calibration_overlay=overlay, source_sha256=sha(__file__), mjcf_sha256=sha(mjcf))
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('geometry_verified','body_pitch_deg','stance_delta_deg','whole_foot_position_error_m')}))


if __name__=='__main__': main()
