"""File-only HOME contrast on saved policy inputs; no plant replay or hardware.

Each counterfactual retains that frame's RECORDED previous raw action and EMA.
Later frames therefore are independent one-step contrasts, never a rollout of
the changed policy. CAD results assume the model's zero/assembly and mass;
gravity supplies a conditional attitude, not verified physical foot contact.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
import onnxruntime as ort


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for seq, result in ((points, lower), (points[::-1], upper)):
        for point in seq:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
    return np.array(lower[:-1]+upper[:-1])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('candidate', 'upstream', 'trial', 'mjcf', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve prior evidence')
    source = load('home_audit_upstream_core', a.upstream/'python/rl_core.py')
    sys.path[:0] = [str(a.candidate/'policy'), str(Path(__file__).parents[1]/'x5_policy')]
    import rl_core as native
    from audit_pose import (angle_deg, forward_kinematics, gravity_rotation,
                            set_pose, sole_reference)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    model_file = a.upstream/'python/xgoduck_walk.onnx'
    session = ort.InferenceSession(str(model_file), options, providers=['CPUExecutionProvider'])
    home = native.policy_profile(session)['home_rad']
    trial = json.loads(a.trial.read_text())
    rows = []
    ids, names = native.POLICY_IDS, native.POLICY_JOINT_NAMES
    for index, row in enumerate(trial['policy_frames']):
        observed = np.array(row['submitted_observation'], dtype=np.float32)
        actual = session.run(None, {session.get_inputs()[0].name: observed})[0][0]
        old_ema = np.array(row['previous_EMA'], dtype=np.float32)
        ema = native.ACTION_ALPHA*old_ema+(1-native.ACTION_ALPHA)*actual
        target = native.action_to_deg(ema, home_rad=home)
        raw_error = float(np.max(np.abs(actual-np.array(row['raw_action']))))
        target_error = float(np.max(np.abs(np.array(target)-row['requested_target_deg'])))
        assert raw_error < 2e-6 and target_error < .0003, (index, raw_error, target_error)
        # Reconstruct the same physical angles from the native submitted delta.
        angles = np.zeros(15, dtype=np.float32)
        angles[native.POLICY_INDEX] = np.rad2deg(observed[0, 6:20].astype(float)+home)
        velocities = np.zeros(15, dtype=np.float32)
        velocities[native.POLICY_INDEX] = np.rad2deg(observed[0, 20:34])
        state = source.State([0, 0, 0], [0, 0, 0], angles.tolist(), velocities.tolist(),
                             observed[0, 3:6].tolist(), observed[0, :3].tolist())
        alternative = source.build_obs(state, observed[0, 34:48], observed[0, 48:51],
                                       observed[0, 51:55]).reshape(1, 61)
        # Degree/radian inversion introduces small velocity-rounding changes.
        # Keep every non-position input EXACTLY recorded in this HOME contrast.
        velocity_roundtrip = float(np.max(abs(alternative[0, 20:34]-observed[0, 20:34])))
        assert velocity_roundtrip < 2e-6
        alternative[0, :6] = observed[0, :6]
        alternative[0, 20:] = observed[0, 20:]
        changed_indices = np.flatnonzero(np.any(alternative != observed, axis=0)).tolist()
        assert set(changed_indices) <= set(range(6, 20))
        counter_raw = session.run(None, {session.get_inputs()[0].name: alternative})[0][0]
        counter_ema = source.ACTION_ALPHA*old_ema+(1-source.ACTION_ALPHA)*counter_raw
        counter_target = source.action_to_deg(counter_ema)
        rows.append(dict(index=index, sample_us=row['observation_sample_us'],
            raw_replay_max_error=raw_error, target_replay_max_error_deg=target_error,
            velocity_roundtrip_error_before_exclusion=velocity_roundtrip,
            upstream_input_changed_indices=changed_indices,
            upstream_target_deg=counter_target,
            upstream_minus_recorded_target_deg=(np.array(counter_target)-target).tolist(),
            upstream_raw_action=counter_raw.tolist(), recorded_pitch_deg=float(
                np.degrees(np.arcsin(np.clip(observed[0, 3], -1, 1))))))
    model = mj.MjModel.from_xml_path(str(a.mjcf.resolve()))
    data = mj.MjData(model)
    set_pose(model, data, dict(zip(names, home)), np.eye(3))
    soles = sole_reference(model, data)
    local_points = []
    for sole in soles:
        geom = model.geom(sole['geom_name']).id
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = np.array(model.mesh_vert[start:start+count], dtype=float)
        points = vertices@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
        local_points.append(vertices[points[:, 2] < points[:, 2].min()+.0005])
    def geometry(label, angles, rotation):
        set_pose(model, data, dict(zip(names, angles)), rotation)
        rotations, positions = forward_kinematics(model, data.qpos)
        fk_error = max(float(np.max(abs(rotations-data.xmat.reshape(-1, 3, 3)))),
                       float(np.max(abs(positions-data.xpos))))
        assert fk_error < 1e-10
        normals, points = [], []
        for sole, vertices in zip(soles, local_points):
            geom = model.geom(sole['geom_name']).id
            normals.append(rotations[sole['body']]@sole['body_normal'])
            points.append(vertices@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom])
        all_points = np.concatenate(points)
        polygon = hull(all_points[:, :2])
        root = model.body('trunk_base').id
        com = data.subtree_com[root].copy()
        margins = []
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge = end-start
            delta = com[:2]-start
            margins.append((edge[0]*delta[1]-edge[1]*delta[0])/np.linalg.norm(edge))
        return dict(case=label, joint_deg=np.rad2deg(angles).tolist(),
            sole_tilt_deg=[angle_deg(n, [0, 0, 1]) for n in normals],
            intersole_normal_angle_deg=angle_deg(*normals),
            lowest_sole_point_separation_mm=float(abs(points[0][:, 2].min()-points[1][:, 2].min())*1000),
            com_world_m=com.tolist(), support_polygon_xy_m=polygon.tolist(),
            projected_COM_margin_mm=float(min(margins)*1000),
            com_minus_mean_sole_x_mm=float((com[0]-all_points[:, 0].mean())*1000),
            independent_fk_max_error=fk_error)
    cases = [geometry('metadata_default_body_level', home, np.eye(3)),
             geometry('upstream_hardcoded_default_body_level', np.deg2rad(source.HOME_DEG), np.eye(3))]
    for index in (0, len(rows)-1):
        obs = np.asarray(trial['policy_frames'][index]['submitted_observation'])[0]
        cases.append(geometry('recorded_input_'+str(index), obs[6:20]+home,
                              gravity_rotation(obs[3:6])))
    paths = [a.trial, model_file, a.mjcf, a.candidate/'policy/rl_core.py',
             a.candidate/'policy/model_reference.py', a.upstream/'python/rl_core.py', Path(__file__)]
    xml = ET.parse(a.mjcf).getroot()
    asset_dir = a.mjcf.parent/xml.find('compiler').get('meshdir', '.')
    paths += [asset_dir/mesh.get('file') for mesh in xml.findall('./asset/mesh') if mesh.get('file')]
    result = dict(scope=__doc__, hardware_access=False, deployment=False,
        upstream_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',
        input_sha256={str(v):hashlib.sha256(v.read_bytes()).hexdigest() for v in paths},
        metadata_home_deg=np.rad2deg(home).tolist(), upstream_home_deg=source.HOME_DEG.tolist(),
        same_recorded_previous_action_and_EMA=True, counterfactual_is_not_closed_loop=True,
        changed_input_scope='Positions6:20 only; every other input is exactly recorded.',
        recorded_raw_history_consistent=all(
            trial['policy_frames'][i]['previous_raw_action']==trial['policy_frames'][i-1]['raw_action']
            and trial['policy_frames'][i]['previous_EMA']==trial['policy_frames'][i-1]['EMA']
            for i in range(1, len(rows))),
        number_of_policy_frames=len(rows), rows=rows, CAD_geometry=cases,
        model_mass_kg=float(model.body_mass.sum()),
        CAD_limitations=['Assumes current encoded angles equal CAD joint zero and physical assembly.',
                         'Projected polygon is geometric, not measured contact/COP or stability.',
                         'Nominal CAD mass800g; physical mass871g and distribution unverified.',
                         'Gravity alignment during motion is conditional; not verified ground/IMU mounting.',
                         'No dynamics stepping, no reset of real state, no calibration modifications.'])
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(frames=len(rows), raw_max_error=max(r['raw_replay_max_error'] for r in rows),
        geometry=[{k:v for k,v in c.items() if k in ('case','sole_tilt_deg','projected_COM_margin_mm','com_minus_mean_sole_x_mm')}
                  for c in cases]), ensure_ascii=False))


if __name__ == '__main__':
    main()
