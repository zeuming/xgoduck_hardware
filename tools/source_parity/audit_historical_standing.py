"""Recover an operator-confirmed standing baseline using files only.

No device imports, network, serial, deployment, or calibration mutation. CAD
comparisons are conditional geometry, not physical stability or gait tests.
"""
import hashlib
import json
from pathlib import Path
import sys

import mujoco as mj
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/x5_policy'))
from audit_pose import angle_deg, forward_kinematics, set_pose, sole_reference
from rl_core import MOTOR_IDS, POLICY_IDS, POLICY_INDEX, POLICY_JOINT_NAMES, policy_profile


def read(relative):
    return json.loads((ROOT / relative).read_text())


def sha(relative):
    return hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()


def main():
    old_dir = 'output/x5_gait_trial/20261008/leg-kp8-candidate/attended-trial-01/'
    new_dir = 'output/duck_sim/source-parity-20261012/post-execution-fall-20261012-1941/'
    out = ROOT / new_dir / 'historical-standing-audit.json'
    if out.exists():
        raise FileExistsError('Preserve completed evidence')
    inputs = [old_dir + name for name in ('outcome.json', 'hold-check.json', 'connection-1-trial-1.json')]
    inputs += [new_dir + 'retrieved/evidence/connection-1-trial-1.json',
               'output/duck_sim/source-parity-20261012/backward-causal-observation/calibration-operator-33-v2.json',
               'output/xgoduck_runtime_upstream/python/xgoduck_walk.onnx',
               'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml']
    outcome, old_status, trace, new_hold, calibration = [read(p) for p in inputs[:5]]
    assert outcome['operator_confirmed_basic_entire_episode_independent_body_support'] is True
    assert outcome['operator_answer'] == '是，全程独立承重，到时才接住'
    assert not trace['truncated']
    h = old_status['hardware']
    original_target = np.array(h['stand_target_deg'])
    old_cal = {j['id']: j for j in trace['calibration']['joints']}
    current_cal = {j['id']: j for j in calibration['joints']}
    assert MOTOR_IDS == trace['motor_ids']
    old_raw = [round(old_cal[sid]['zero'] + original_target[i]*4095/360*old_cal[sid]['sign'])
               for i, sid in enumerate(MOTOR_IDS)]
    def decode(ticks):
        return np.array([(t-current_cal[sid]['zero'])*360/4095*current_cal[sid]['sign']
                         for sid, t in zip(MOTOR_IDS, ticks)])
    opt = ort.SessionOptions()
    opt.intra_op_num_threads = opt.inter_op_num_threads = 1
    session = ort.InferenceSession(str(ROOT / inputs[5]), opt, providers=['CPUExecutionProvider'])
    home = policy_profile(session)['home_rad']
    current_target = np.zeros(15)
    current_target[POLICY_INDEX] = np.rad2deg(home)
    current_raw = [round(current_cal[sid]['zero'] + current_target[i]*4095/360*current_cal[sid]['sign'])
                   for i, sid in enumerate(MOTOR_IDS)]
    assert all(row['attempted_target_ticks'] == current_raw for row in new_hold['frames'])
    old_hold = [row for row in trace['frames'] if row['mode'] == 'hold' and row['attempted_target_ticks']]
    assert old_hold[-1]['attempted_target_ticks'] == old_raw
    old_walk = [row for row in trace['frames'] if row['mode'] == 'walk' and row['attempted_target_ticks']]
    assert len(old_walk) == outcome['control_frames'] == 977
    assert not any(row['request_error'] for row in old_walk)
    assert all(row['command'] == [0., 0., 0.] for row in old_walk)
    old_head_ticks = np.array([r['attempted_target_ticks'][10:14] for r in old_walk])
    assert np.all(old_head_ticks == np.array(old_raw)[10:14])
    changes = [{ 'id': sid, 'field': key, 'old': old_cal[sid][key], 'current': current_cal[sid][key]}
               for sid in MOTOR_IDS for key in ('zero', 'sign', 'min', 'max')
               if old_cal[sid][key] != current_cal[sid][key]]
    assert not any(row['field'] == 'zero' for row in changes)
    assert [(row['id'], row['old'], row['current']) for row in changes if row['field'] == 'sign'] == [(21, 1, -1)]
    head_only = current_target.copy()
    head_only[10:14] = original_target[10:14]
    leg_only = current_target.copy()
    leg_only[:10] = original_target[:10]
    cases = {'current_metadata': current_target, 'historical_head_only': head_only,
             'historical_leg_HOME_only_current_signs': leg_only,
             'historical_head_and_HOME_current_signs': original_target,
             'historical_exact_ticks_decoded_current_signs': decode(old_raw)}
    model = mj.MjModel.from_xml_path(str(ROOT / inputs[6]))
    data = mj.MjData(model)
    def pose(target):
        set_pose(model, data, dict(zip(POLICY_JOINT_NAMES, np.deg2rad(target[POLICY_INDEX]))), np.eye(3))
    pose(current_target)
    soles = sole_reference(model, data)
    geometry = []
    for name, target in cases.items():
        pose(target)
        rotations, positions = forward_kinematics(model, data.qpos)
        assert np.max(abs(positions-data.xpos)) < 1e-10
        assert np.max(abs(rotations-data.xmat.reshape(-1, 3, 3))) < 1e-10
        foot_x = np.mean([data.geom_xpos[model.geom(s['geom_name']).id, 0] for s in soles])
        com = data.subtree_com[model.body('trunk_base').id].copy()
        ticks = [round(current_cal[sid]['zero'] + target[i]*4095/360*current_cal[sid]['sign'])
                 for i, sid in enumerate(MOTOR_IDS)]
        normals = [rotations[s['body']]@s['body_normal'] for s in soles]
        geometry.append(dict(case=name, target_deg=target.tolist(), target_ticks=ticks,
            out_of_current_saved_range=[sid for sid, t in zip(MOTOR_IDS, ticks)
                if not current_cal[sid]['min'] <= t <= current_cal[sid]['max']],
            nominal_COM_x_relative_mean_foot_geom_mm=float((com[0]-foot_x)*1000),
            sole_tilt_deg=[angle_deg(n, [0, 0, 1]) for n in normals],
            intersole_normal_deg=angle_deg(*normals)))
    ref_com = geometry[0]['nominal_COM_x_relative_mean_foot_geom_mm']
    for g in geometry:
        g['COM_forward_change_mm'] = g['nominal_COM_x_relative_mean_foot_geom_mm']-ref_com
    result = dict(scope=__doc__, hardware_access=False, installed=False,
        input_sha256={p: sha(p) for p in inputs},
        success=outcome, matched_trace_targets=len(old_walk), frozen_head_target_frames=len(old_walk),
        calibration_changes=changes, calibration_write=False,
        current_model_home_deg=np.rad2deg(home).tolist(), historical_stance_deg=original_target.tolist(),
        historical_candidate_profile=h['candidate_profile'],
        nominal_CAD_mass_kg=float(model.body_mass.sum()), conditional_geometry=geometry,
        recovered_head_command_rad=(np.deg2rad(original_target[10:14])-home[5:9]).tolist(),
        gains=dict(historical_kp=8, current_known_source_kp=6, kd_both=20, mouth_kp_both=10,
            limitation='KP8 trial succeeded; not isolated causal proof or current firmware RAM readback.'),
        exclusions=['Do not restore old ID21 direction or old ranges.',
            'Historical 600ticks/s, 15deg stop, head overwrite and nonlinear roll transform are NOT reinstalled.',
            'This CAD has nominal 800g inertia, not measured current rear installation.',
            'Historical success is one zero-speed episode; no walking acceptance.',
            'Head-command input alone is not equivalent to historical frozen head outputs.'])
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(out), targets=len(old_walk), frozen_head_frames=len(old_walk),
        geometry=[{k:g[k] for k in ('case','COM_forward_change_mm','out_of_current_saved_range','intersole_normal_deg')}
                  for g in geometry]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
