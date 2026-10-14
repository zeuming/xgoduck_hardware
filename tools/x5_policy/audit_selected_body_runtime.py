"""Read-only replay of selected simulation traces through cached host code.

Never construct a transport or run a controller. This checks pure ONNX/codec
functions, and records workflow differences instead of claiming device parity.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


MATRICES = {
    'walk': ('pilot128-walk-matrix-v1', 'walk128-holdout-seeds5-7-v1'),
    'getup': ('native512-getup-matrix-v1', 'getup512-holdout-v1'),
    'pick': ('warmstart1024-pick-matrix-v1', 'warmstart1024-pick-holdout-v1'),
    'roulade': ('mixed512-roulade-matrix-v1', 'roll512-holdout-v1'),
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--production', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import onnxruntime as ort

    root = a.root.resolve()
    contract = json.loads((root/'current-body-v2/inputs/contract.json').read_text())
    selected = json.loads((root/'selected-models-v1.json').read_text())
    joints = json.loads((root/'current-body-v2/inputs/calibration-numerical.json').read_text())['joints']
    sys.path.insert(0, str(a.production.resolve()/'policy'))
    import motion
    import rl_core

    # Only populate data used by the pure decoding functions. No __init__,
    # serial factory, thread, controller loop, feedback or ARM is invoked.
    controller = object.__new__(motion.MotionController)
    controller._config = {'joints': joints}
    controller._policy_profiles = {}
    controller._walk_head_profile = tuple(contract['head_command_rad'])
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    report = dict(hardware_access=False, production_verified_current=False,
                  scope='Frozen-input ONNX and pure cached target codec replay; not physical or integrated-controller validation',
                  auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  models={}, workflow_differences=[])
    report['cached_source_sha256'] = {
        name: hashlib.sha256((a.production/'policy'/name).read_bytes()).hexdigest()
        for name in ('motion.py', 'production_motion.py', 'rl_core.py', 'official_recovery.py')
    }
    for mode, matrix_names in MATRICES.items():
        info = selected['models'][mode]
        model = root/info['model']
        assert hashlib.sha256(model.read_bytes()).hexdigest() == info['sha256']
        session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
        profile = rl_core.policy_profile(session)
        assert np.array_equal(profile['home_rad'], contract['models'][mode]['home_rad'])
        assert profile['scale'] == contract['models'][mode]['action_scale']
        controller._policy_profiles[mode] = profile
        seed = np.zeros(14, np.float32)
        if mode == 'walk':
            seed[5:9] = contract['head_command_rad']
        controller._mode = 'hold'
        controller._last_executed_target = rl_core.stance_from_home(profile['home_rad'])
        for sid, delta in zip((30, 31, 32, 33), controller._walk_head_profile):
            controller._last_executed_target[motion.MOTOR_IDS.index(sid)] += float(np.rad2deg(delta))
        runtime_seed = controller._policy_start_smooth_locked(mode)
        mode_report = dict(frames=0, cases=0, onnx_max_abs_error=0.,
                           declared_seed_goal_mismatches=0, runtime_handoff_goal_mismatches=0,
                           raw_history_mismatches=0, metadata_matches=True,
                           handoff_seed_max_abs_error=float(np.max(np.abs(seed-runtime_seed))),
                           cases_detail=[])
        for matrix_name in matrix_names:
            matrix = json.loads((root/matrix_name/'matrix.json').read_text())
            assert matrix['complete']
            for case in matrix['cases']:
                if case['mode'] != mode:
                    continue
                baseline_name = 'original-matrix-v1' if case['seed'] in (0, 3) else {
                    'walk': 'original-holdout-seeds5-7-v1',
                    'getup': 'original-getup-holdout-v1',
                    'pick': 'original-pick-holdout-v1',
                    'roulade': 'original-roll-holdout-v1',
                }[mode]
                gate = json.loads((root/matrix_name/case['key']/'gate.json').read_text())
                baseline = json.loads((root/baseline_name/case['key']/'gate.json').read_text())
                assert case['accepted'] and gate['initial_fingerprint']
                assert gate['initial_fingerprint'] == baseline['initial_fingerprint']
                trace_path = root/matrix_name/case['key']/'trace.jsonl'
                smooth, runtime_smooth = seed.copy(), runtime_seed.copy()
                previous = np.zeros(14, np.float32)
                case_result = dict(key=case['key'], frames=0, declared_mismatches=0,
                                   handoff_mismatches=0, max_tick_error=0)
                last_before_stop = None
                with trace_path.open() as stream:
                    for step, line in enumerate(stream):
                        row = json.loads(line)
                        obs = np.array(row['actor_observation'], np.float32).reshape(1, 61)
                        raw = np.array(row['raw_action'], np.float32).reshape(1, 14)
                        actual = session.run(None, {session.get_inputs()[0].name: obs})[0]
                        error = float(np.max(np.abs(actual-raw)))
                        mode_report['onnx_max_abs_error'] = max(mode_report['onnx_max_abs_error'], error)
                        mode_report['raw_history_mismatches'] += int(not np.array_equal(obs[0, 34:48], previous))
                        # The source getup reset preparation deliberately ignores
                        # actor outputs during its first 100 physical frames.
                        effective = np.zeros_like(raw) if mode == 'getup' and step < 100 else raw
                        expected = np.array(row['transmitted_ticks'], np.int64)
                        for branch in ('declared', 'handoff'):
                            state = smooth if branch == 'declared' else runtime_smooth
                            target, _, filtered, _ = controller._decode_policy_action(
                                mode, effective, previous, state, step*.02, 0.)
                            executed = controller._execution_target_locked(mode, target)
                            ticks = controller._raw_targets_locked(executed, mode=mode)
                            ticks = np.array([ticks[motion.MOTOR_IDS.index(s)] for s in rl_core.POLICY_IDS])
                            mismatch = int(not np.array_equal(expected, ticks))
                            case_result[branch+'_mismatches'] += mismatch
                            case_result['max_tick_error'] = max(case_result['max_tick_error'], int(np.max(np.abs(expected-ticks))))
                            if branch == 'declared':
                                smooth = filtered
                            else:
                                runtime_smooth = filtered
                        previous = effective[0].copy()
                        mode_report['frames'] += 1
                        case_result['frames'] += 1
                        if mode == 'roulade' and row['time_s'] < motion.EPISODE_SECONDS[mode]:
                            last_before_stop = dict(time_s=row['time_s'], tilt_deg=row['tilt_deg'],
                                feet={s: dict(vertical_load_n=row['feet'][s]['vertical_load_n'],
                                              mesh_min_height_m=row['feet'][s]['mesh_min_height_m'])
                                      for s in ('left', 'right')})
                if last_before_stop is not None:
                    case_result['existing_trace_last_frame_before_cached_button_stop'] = last_before_stop
                mode_report['declared_seed_goal_mismatches'] += case_result['declared_mismatches']
                mode_report['runtime_handoff_goal_mismatches'] += case_result['handoff_mismatches']
                mode_report['cases_detail'].append(case_result)
                mode_report['cases'] += 1
        report['models'][mode] = mode_report
    report['workflow_differences'] = [
        dict(mode='roulade', cached_host_stop_s=motion.EPISODE_SECONDS['roulade'],
             independent_evaluation_s=6., requires_integrated_simulation=True,
             scope='Existing trace prefix only; not a simulated STOP or a device measurement'),
        dict(mode='getup', cached_host_default_pose_s=1., source_reset_preparation_s=2.,
             requires_integrated_simulation=True,
             scope='Reset preparation is not the same as a runtime default-pose stage'),
    ]
    report['pure_codec_passed'] = all(
        row['onnx_max_abs_error'] == 0 and row['declared_seed_goal_mismatches'] == 0
        and row['runtime_handoff_goal_mismatches'] == 0 and row['raw_history_mismatches'] == 0
        for row in report['models'].values())
    report['integrated_controller_passed'] = False
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'models'}))
    print(json.dumps({m: {k: v for k, v in row.items() if k != 'cases_detail'} for m, row in report['models'].items()}))


if __name__ == '__main__':
    main()
