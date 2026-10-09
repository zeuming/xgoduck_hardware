"""Offline snapshot/CAD pose audit. Reads files only; never connects to hardware.

Example (run with a Python environment containing numpy and mujoco):
  python tools/x5_policy/audit_pose.py --snapshot status.json --mjcf robot_walk.xml

The snapshot is historical evidence, not a fresh hardware measurement. In
particular, flat physical soles and the snapshot must refer to the same pose.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from rl_core import POLICY_IDS, POLICY_JOINT_NAMES


def unit(value):
    vector = np.asarray(value, dtype=float)
    norm = np.linalg.norm(vector)
    if not np.isfinite(vector).all() or norm < 1e-12:
        raise ValueError('Expected a finite nonzero vector')
    return vector / norm


def angle_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1, 1))))


def quat_matrix(value):
    w, x, y, z = unit(value)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def axis_rotation(axis, theta):
    x, y, z = unit(axis)
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(theta)*skew + (1-np.cos(theta))*np.einsum('ij,jk->ik', skew, skew)


def gravity_rotation(gravity):
    """Body-to-world rotation mapping body gravity onto world down (yaw free)."""
    source, down = unit(gravity), np.array([0., 0., -1.])
    cross, cosine = np.cross(source, down), np.dot(source, down)
    if cosine < -1 + 1e-10:
        return axis_rotation([1, 0, 0], np.pi)
    skew = np.array([[0., -cross[2], cross[1]],
                     [cross[2], 0., -cross[0]], [-cross[1], cross[0], 0.]])
    return np.eye(3) + skew + np.einsum('ij,jk->ik', skew, skew)/(1+cosine)


def forward_kinematics(model, qpos):
    """Independent rigid FK, including arbitrary 3D hinge axes and pivots.

    Uses the compiled model's rigid transforms, but does not call mj_forward.
    MuJoCo applies a hinge/ball rotation about jnt_pos in the current body frame.
    A hinge coordinate is relative to qpos0 (the MJCF joint ref).
    """
    rotations = np.repeat(np.eye(3)[None], model.nbody, axis=0)
    positions = np.zeros((model.nbody, 3))
    for body in range(1, model.nbody):
        parent = model.body_parentid[body]
        rotation = np.einsum('ij,jk->ik', rotations[parent], quat_matrix(model.body_quat[body]))
        position = positions[parent] + np.einsum('ij,j->i', rotations[parent], model.body_pos[body])
        start = model.body_jntadr[body]
        for joint in range(start, start + model.body_jntnum[body]):
            address = model.jnt_qposadr[joint]
            # pybind enum membership and NumPy scalar equality differ; use int.
            kind = int(model.jnt_type[joint])
            if kind == mj.mjtJoint.mjJNT_FREE:
                position = np.array(qpos[address:address+3], copy=True)
                rotation = quat_matrix(qpos[address+3:address+7])
            elif kind == mj.mjtJoint.mjJNT_SLIDE:
                displacement = model.jnt_axis[joint] * (qpos[address]-model.qpos0[address])
                position += np.einsum('ij,j->i', rotation, displacement)
            elif kind in (mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_BALL):
                pivot = model.jnt_pos[joint]
                anchor = position + np.einsum('ij,j->i', rotation, pivot)
                delta = (axis_rotation(model.jnt_axis[joint], qpos[address]-model.qpos0[address])
                         if kind == mj.mjtJoint.mjJNT_HINGE else quat_matrix(qpos[address:address+4]))
                rotation = np.einsum('ij,jk->ik', rotation, delta)
                position = anchor - np.einsum('ij,j->i', rotation, pivot)
            else:
                raise ValueError(f'Unsupported joint type: {kind}')
        rotations[body], positions[body] = rotation, position
    return rotations, positions


def snapshot_hardware(snapshot):
    motion = snapshot.get('motion') or {}
    for label, value in [('motion.hardware', motion.get('hardware')),
                         ('motion.last_hardware', motion.get('last_hardware')),
                         ('hardware', snapshot.get('hardware')), ('root', snapshot)]:
        if isinstance(value, dict) and isinstance(value.get('rawdata'), list):
            return label, value
    raise ValueError('Snapshot has no hardware rawdata rows')


def snapshot_angles(snapshot, hardware):
    calibration = hardware.get('joints') or (snapshot.get('calibration') or {}).get('joints')
    if not isinstance(calibration, list):
        raise ValueError('Snapshot has no joint calibration')
    calibration = {row['id']: row for row in calibration}
    raw = {row['id']: row for row in hardware['rawdata']}
    angles, rows = {}, []
    for sid, name in zip(POLICY_IDS, POLICY_JOINT_NAMES):
        if sid not in raw or sid not in calibration:
            raise ValueError(f'Snapshot missing joint {sid}')
        row, cal = raw[sid], calibration[sid]
        if cal['sign'] not in (-1, 1):
            raise ValueError(f'Invalid calibration sign for joint {sid}')
        angle = (row['position']-cal['zero'])*360/4095*cal['sign']
        if not np.isfinite(angle):
            raise ValueError(f'Nonfinite angle for joint {sid}')
        if 'angle_deg' in row and not np.isclose(angle, row['angle_deg'], rtol=0, atol=1e-6):
            raise ValueError(f'Raw ticks and saved angle disagree for joint {sid}')
        angles[name] = np.radians(angle)
        rows.append(dict(id=sid, name=name, position=row['position'], zero=cal['zero'],
                         sign=cal['sign'], angle_deg=angle))
    return angles, rows


def set_pose(model, data, angles, rotation):
    data.qpos[:] = model.qpos0
    root = model.body('trunk_base').id
    root_joint = model.body_jntadr[root]
    if model.jnt_type[root_joint] != mj.mjtJoint.mjJNT_FREE:
        raise ValueError('trunk_base requires a free joint')
    address = model.jnt_qposadr[root_joint]
    quat = np.empty(4)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(rotation).reshape(9))
    data.qpos[address:address+7] = [0., 0., .14, *quat]
    for name, value in angles.items():
        joint = model.joint(name).id
        if model.jnt_type[joint] != mj.mjtJoint.mjJNT_HINGE:
            raise ValueError(f'Expected hinge joint: {name}')
        data.qpos[model.jnt_qposadr[joint]] = value
    mj.mj_forward(model, data)


def sole_reference(model, data):
    """Fit actual mesh sole planes; foot body +Z is not necessarily sole up."""
    soles = []
    for body_name, geom_name in [('ankle_left', 'left_foot_collision'),
                                 ('ankle_right', 'right_foot_collision')]:
        geom = model.geom(geom_name).id
        if model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH:
            raise ValueError(f'Expected collision mesh: {geom_name}')
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = np.asarray(model.mesh_vert[start:start+count], dtype=float)
        world = np.einsum('nj,ij->ni', vertices, data.geom_xmat[geom].reshape(3, 3)) + data.geom_xpos[geom]
        bottom = world[world[:, 2] < world[:, 2].min()+.0005]
        if len(bottom) < 3:
            raise ValueError(f'Insufficient bottom vertices: {geom_name}')
        _, singular, axes = np.linalg.svd(bottom-bottom.mean(0))
        normal = axes[-1] * (1 if axes[-1, 2] >= 0 else -1)
        body = model.body(body_name).id
        body_normal = np.einsum('ji,j->i', data.xmat[body].reshape(3, 3), normal)
        soles.append(dict(body=body, body_name=body_name, geom_name=geom_name,
                          body_normal=body_normal, vertex_count=len(bottom),
                          home_tilt_deg=angle_deg(normal, [0, 0, 1]),
                          plane_residual_rms_mm=float(singular[-1]/np.sqrt(len(bottom))*1000)))
    return soles


def audit(snapshot_path, mjcf_path):
    snapshot_path, mjcf_path = Path(snapshot_path), Path(mjcf_path)
    snapshot = json.loads(snapshot_path.read_text())
    section, hardware = snapshot_hardware(snapshot)
    angles, joint_rows = snapshot_angles(snapshot, hardware)
    profiles = hardware.get('policy_profiles') or {}
    home = (profiles.get('walk') or {}).get('home_rad')
    if home is None or len(home) != len(POLICY_IDS) or not np.isfinite(home).all():
        raise ValueError('Snapshot needs the walk policy metadata home_rad (14 finite radians)')
    gravity = unit((hardware.get('imu') or {})['gravity'])
    rotation = gravity_rotation(gravity)
    model = mj.MjModel.from_xml_path(str(mjcf_path.resolve()))
    data = mj.MjData(model)
    home_angles = dict(zip(POLICY_JOINT_NAMES, home))
    set_pose(model, data, home_angles, np.eye(3))
    soles = sole_reference(model, data)
    variants = [('saved_calibration', []), ('hypothesis_flip_11', [11]),
                ('hypothesis_flip_21', [21]), ('hypothesis_flip_11_and_21', [11, 21])]
    results = []
    for label, flipped in variants:
        selected = dict(angles)
        for sid in flipped:
            name = POLICY_JOINT_NAMES[POLICY_IDS.index(sid)]
            selected[name] *= -1
        set_pose(model, data, selected, rotation)
        independent_rotations, independent_positions = forward_kinematics(model, data.qpos)
        rotation_error = float(np.max(np.abs(independent_rotations-data.xmat.reshape(-1, 3, 3))))
        position_error = float(np.max(np.abs(independent_positions-data.xpos)))
        if max(rotation_error, position_error) > 1e-10:
            raise ValueError('Independent FK disagrees with MuJoCo')
        normals, feet = [], []
        for sole in soles:
            normal = np.einsum('ij,j->i', independent_rotations[sole['body']], sole['body_normal'])
            normals.append(normal)
            feet.append(dict(foot_body=sole['body_name'], predicted_sole_normal_world=normal.tolist(),
                             predicted_sole_tilt_deg=angle_deg(normal, [0, 0, 1])))
        difference = angle_deg(*normals)
        results.append(dict(case=label, hypothetical_sign_flips=flipped, feet=feet,
                            intersole_normal_angle_deg=difference,
                            minimum_possible_max_sole_tilt_deg=difference/2,
                            independent_fk_rotation_max_error=rotation_error,
                            independent_fk_position_max_error_m=position_error))
    positive_motion = []
    for sid, foot in [(11, 'left_foot'), (21, 'right_foot')]:
        name = POLICY_JOINT_NAMES[POLICY_IDS.index(sid)]
        set_pose(model, data, home_angles, np.eye(3))
        before = data.site(foot).xpos.copy()
        selected = dict(home_angles)
        selected[name] += np.radians(5)
        set_pose(model, data, selected, np.eye(3))
        delta = (data.site(foot).xpos-before)*1000
        positive_motion.append(dict(id=sid, angle_increment_deg=5,
                                    foot_displacement_trunk_mm=delta.tolist(),
                                    direction='duck left (+Y)' if delta[1] > 0 else 'duck right (-Y)'))
    return dict(kind='offline_snapshot_CAD_consistency_audit',
                snapshot=str(snapshot_path.resolve()), snapshot_sha256=hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
                hardware_section=section, mjcf=str(mjcf_path.resolve()),
                mjcf_sha256=hashlib.sha256(mjcf_path.read_bytes()).hexdigest(), mujoco_version=mj.__version__,
                snapshot_state={key: hardware.get(key) for key in
                                ('mode', 'armed', 'online_ids', 'fault_ids', 'torque_ids', 'age_ms', 'imu_age_ms')},
                gravity_body=gravity.tolist(),
                gravity_alignment_error=float(np.max(np.abs(np.einsum('ij,j->i', rotation, gravity)-[0, 0, -1]))),
                imu_pitch_deg=float(np.degrees(np.arcsin(gravity[0]))),
                imu_roll_deg=float(np.degrees(np.arctan2(-gravity[1], -gravity[2]))),
                joint_angles=joint_rows,
                sole_reference=[{key: value for key, value in sole.items()
                                 if key not in ('body', 'body_normal')} for sole in soles],
                results=results, model_positive_roll_motion=positive_motion,
                assumptions=['Physical full soles flat must refer to this exact snapshot pose.',
                             'Saved joint angles map to these CAD coordinates; gravity maps to CAD trunk.',
                             'Meshes and rigid links describe the physical assembly.'],
                interpretation='A common IMU/root rotation cannot change the intersole angle. Sign-flip cases are diagnostic hypotheses only: zero offsets, assembly/model differences and a mismatched physical observation can also explain errors. No correction is uniquely identified.',
                calibration_changes=0, hardware_connections=0, motor_commands=0,
                live_motion_readiness='not established by this offline audit')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--mjcf', type=Path, required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(audit(args.snapshot, args.mjcf), ensure_ascii=False, indent=2))
    except (ValueError, KeyError, OSError, json.JSONDecodeError) as error:
        parser.exit(2, f'Pose audit failed: {error}\n')


if __name__ == '__main__':
    main()
