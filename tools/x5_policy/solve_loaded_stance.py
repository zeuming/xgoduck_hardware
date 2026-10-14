"""File-only COM-restoring stance calculation, never a hardware setpoint.

The loaded and reference robots share geometry and total nominal mass. Solve
six sagittal joints while keeping the torso level, torso height, both foot
positions and orientations, and the reference whole-body COM projection.
This is geometry, not a dynamic stability certificate or identified payload.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from audit_pose import forward_kinematics, set_pose
from offline_calibration_overlay import range_snapshot
from offline_mass_distribution import scenario_spec

ROOT = Path(__file__).resolve().parents[2]
SAGITTAL_IDS = (12, 13, 14, 22, 23, 24)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def foot_vertices(model, data, side):
    geom = model.geom(f'{side}_foot_collision').id
    mesh = model.geom_dataid[geom]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    points = model.mesh_vert[start:start+count].astype(float)
    return points @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]


def solve(candidate, mjcf, reference_scenario, loaded_scenario):
    source = lambda: mj.MjSpec.from_file(str(Path(mjcf).resolve()))
    reference_fn, reference_evidence = scenario_spec(source, reference_scenario)
    loaded_fn, loaded_evidence = scenario_spec(source, loaded_scenario)
    reference, loaded = reference_fn().compile(), loaded_fn().compile()
    before, data = mj.MjData(reference), mj.MjData(loaded)
    names, home = candidate['joint_names'], np.array(candidate['home_rad'])
    set_pose(reference, before, dict(zip(names, home)), np.eye(3))
    set_pose(loaded, data, dict(zip(names, home)), np.eye(3))
    root = loaded.body('trunk_base').id
    root_qadr = loaded.jnt_qposadr[loaded.body_jntadr[root]]
    # Place the initial soles on the same floor. This is a reset calculation,
    # not a write into a running simulation or the robot's root state.
    floor_z = min(foot_vertices(reference, before, s)[:, 2].min() for s in ('left', 'right'))
    before.qpos[root_qadr+2] -= floor_z
    data.qpos[root_qadr+2] -= floor_z
    mj.mj_forward(reference, before); mj.mj_forward(loaded, data)
    baseline = data.qpos.copy()
    target_com_x = before.subtree_com[root, 0]
    initial_com_x = data.subtree_com[root, 0]
    foot_ids = [loaded.body('ankle_'+s).id for s in ('left', 'right')]
    foot_positions = before.xpos[foot_ids].copy()
    foot_rotations = before.xmat[foot_ids].reshape(2, 3, 3).copy()
    policy_indices = [candidate['policy_joint_ids'].index(sid) for sid in SAGITTAL_IDS]
    joint_ids = [loaded.joint(names[i]).id for i in policy_indices]
    joint_qadr = [loaded.jnt_qposadr[j] for j in joint_ids]
    low = np.maximum(np.array(candidate['lower_rad'])[policy_indices], loaded.jnt_range[joint_ids, 0])
    high = np.minimum(np.array(candidate['upper_rad'])[policy_indices], loaded.jnt_range[joint_ids, 1])
    if np.any(low > high):
        raise ValueError('Saved and CAD limits have empty intersection')
    extent = max(np.ptp(foot_vertices(reference, before, s)[:, 0]) for s in ('left', 'right'))

    def evaluate(x):
        data.qpos[:] = baseline
        data.qpos[root_qadr:root_qadr+3:2] = x[:2]
        data.qpos[joint_qadr] = x[2:]
        mj.mj_forward(loaded, data)
        position = (data.xpos[foot_ids]-foot_positions).ravel()
        orientation = np.array([Rotation.from_matrix(old.T @ now).as_rotvec()
                                for old, now in zip(foot_rotations, data.xmat[foot_ids].reshape(2, 3, 3))]).ravel()
        return np.r_[position, extent*orientation,
                     data.qpos[root_qadr+2]-baseline[root_qadr+2],
                     data.subtree_com[root, 0]-target_com_x]

    initial = np.r_[baseline[root_qadr], baseline[root_qadr+2], home[policy_indices]]
    # Root search bounds enclose the complete robot reach; they are solver
    # domains, not a new controller range or a selected lean angle.
    reach = np.linalg.norm(loaded.stat.extent)
    result = least_squares(evaluate, initial,
        bounds=(np.r_[-reach, .001, low], np.r_[reach, reach, high]),
        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=500, diff_step=1e-5)
    residual = evaluate(result.x)
    rotations, positions = forward_kinematics(loaded, data.qpos)
    fk_error = max(float(abs(positions-data.xpos).max()),
                   float(abs(rotations-data.xmat.reshape(-1, 3, 3)).max()))
    foot_error = float(abs(data.xpos[foot_ids]-foot_positions).max())
    rotation_error = max(np.linalg.norm(Rotation.from_matrix(old.T @ now).as_rotvec())
                         for old, now in zip(foot_rotations, data.xmat[foot_ids].reshape(2, 3, 3)))
    delta = np.zeros(14); delta[policy_indices] = result.x[2:]-home[policy_indices]
    # Rounded CAD hinge frames leave submicrometre lateral inconsistency in
    # this sagittal-only solve. Judge each physical constraint in its own unit;
    # do not mix a redundant residual norm with the explicit whole-foot check.
    restored = (result.success and foot_error < 1e-6 and rotation_error < 1e-4
                and abs(residual[-1]) < 1e-7 and abs(residual[-2]) < 1e-7 and fk_error < 1e-10)
    # All geometric changes are retained even if the solve fails. Never export
    # an unverified stance as a usable reference.
    return dict(kind='unmeasured_load_geometric_stance_hypothesis', hardware_access=False,
        hardware_eligible=False, dynamic_stability_verified=False,
        solver_success=bool(result.success), geometry_verified=bool(restored), evaluations=result.nfev,
        residual_m=residual.tolist(), max_independent_fk_error=fk_error,
        geometric_check_scope='Numerical checks only: feet1um,orientation0.0001rad,COM/height0.1um; no physical protection or runtime threshold changed',
        whole_foot_position_error_m=foot_error, whole_foot_orientation_error_rad=float(rotation_error),
        reference=reference_evidence, loaded=loaded_evidence,
        nominal_total_mass_kg=float(loaded.body_mass.sum()),
        initial_whole_com_backshift_m=float(target_com_x-initial_com_x),
        restored_whole_com_projection_error_m=float(data.subtree_com[root,0]-target_com_x),
        trunk_forward_translation_m=float(result.x[0]-baseline[root_qadr]),
        trunk_height_change_m=float(result.x[1]-baseline[root_qadr+2]),
        policy_joint_ids=candidate['policy_joint_ids'], home_rad=home.tolist(),
        stance_delta_rad=delta.tolist(), stance_delta_deg=np.rad2deg(delta).tolist(),
        stance_target_rad=(home+delta).tolist(),
        root_state_usage='Only nominal static geometry; dynamic trials may change joint reset, never inject root motion',
        physical_unknowns=['Actual payload mass/height/inertia', 'Physical joint zeros and mounting', 'Loaded servo and contact behavior'],
        actor_sha256=candidate['source_model']['sha256'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--loaded',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    candidate,overlay=range_snapshot(json.loads(cp.read_text())['candidate'],a.calibration)
    mjcf=ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'
    report=solve(candidate,mjcf,a.reference,a.loaded)
    report.update(calibration_overlay=overlay,source_sha256=sha(__file__),mjcf_sha256=sha(mjcf))
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:report[key] for key in ('geometry_verified','initial_whole_com_backshift_m',
        'trunk_forward_translation_m','stance_delta_deg','whole_foot_position_error_m')}))


if __name__=='__main__':
    main()
