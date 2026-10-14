"""Recorded-input evidence, never a closed-loop simulation or hardware action."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import types
import numpy as np
import onnxruntime as ort

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--input', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(a.input / 'installed/policy'))
import motion
from rl_core import policy_profile

model = root / 'output/xgoduck_runtime_upstream/python/xgoduck_walk.onnx'
opt = ort.SessionOptions()
opt.intra_op_num_threads = opt.inter_op_num_threads = 1
actor = ort.InferenceSession(str(model), opt, providers=['CPUExecutionProvider'])
profile = policy_profile(actor)
ctx = types.SimpleNamespace(_policy_profiles={'walk': profile})
results = []
for label in ('tynwyp4m', '7vuq_63t', 'vv_en1vy'):
    ready = json.loads((a.input / label / 'evidence/connection-1-ready.json').read_text())['hardware']
    calibration = {j['id']: j for j in ready['joints']}
    for path in sorted((a.input / label / 'evidence').glob('connection-*-trial-*.json')):
        trial = json.loads(path.read_text())
        for (mode, generation), rows in itertools.groupby(trial['policy_frames'], lambda f: (f['mode'], f['generation'])):
            fs = list(rows)
            if mode != 'walk' or any(f['selected_model'] != 'walk' for f in fs):
                continue
            first, last = fs[0], fs[-1]
            holds = [f for f in trial['frames'] if f['mode'] == 'hold' and not f.get('error')
                     and f.get('attempted_target_ticks') and f['monotonic'] < first['submission_monotonic']]
            def pitch(f):
                g = f['submitted_observation'][0][3:6]
                return math.degrees(math.atan2(g[0], -g[2]))
            raw_errors, decode_errors, goal_tick_errors = [], [], []
            for f in fs:
                obs = np.asarray(f['submitted_observation'], np.float32)
                raw = actor.run(None, {actor.get_inputs()[0].name: obs})[0]
                raw_errors.append(float(np.max(abs(raw[0] - f['raw_action']))))
                target, _, _, _ = motion.MotionController._decode_policy_action(ctx, 'walk', raw,
                    np.asarray(f['previous_raw_action'], np.float32), np.asarray(f['previous_EMA'], np.float32), 0., 0.)
                decode_errors.append(float(np.max(abs(np.asarray(target) - f['requested_target_deg']))))
                # Quantization comparison is before existing head range projection.
                actual = np.asarray(f['requested_target_deg'])
                before = [round(calibration[sid]['zero'] + calibration[sid]['sign'] * float(v) * 4095/360)
                          for sid, v in zip(motion.MOTOR_IDS, actual)]
                replay = [round(calibration[sid]['zero'] + calibration[sid]['sign'] * float(v) * 4095/360)
                          for sid, v in zip(motion.MOTOR_IDS, target)]
                goal_tick_errors.append(max(abs(x-y) for x, y in zip(before, replay)))
            report = dict(source=str(path.relative_to(a.input)), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                generation=generation, policy_frames=len(fs), span_s=last['submission_monotonic']-first['submission_monotonic'],
                initial_pitch_deg=pitch(first), last_pitch_deg=pitch(last),
                min_pitch_deg=min(map(pitch, fs)), max_pitch_deg=max(map(pitch, fs)),
                trace_terminal_reason=trial['reason'], all_zero_velocity_command=all(f['submitted_observation'][0][48:51] == [0.,0.,0.] for f in fs),
                original_actor_max_abs_error=max(raw_errors), original_decoder_max_abs_deg_error=max(decode_errors),
                max_quantized_target_tick_error=max(goal_tick_errors),
                head_command=first['submitted_observation'][0][51:55],
                previous_head_EMA=first['previous_EMA'][5:9],
                first_requested_head_deg=first['requested_target_deg'][10:14],
                max_position_age_ms=max(max(f['input_feedback_provenance']['position_ages_us']) for f in fs)/1000)
            if holds:
                hold = holds[-1]
                # Preserve the actual preceding successful wire goal, with its quantization.
                held = np.asarray([(tick-calibration[sid]['zero']) * calibration[sid]['sign'] * 360/4095
                                   for sid,tick in zip(motion.MOTOR_IDS, hold['attempted_target_ticks'])], np.float32)
                seed = np.zeros(14, np.float32)
                seed[5:9] = (np.deg2rad(held[10:14])-profile['home_rad'][5:9])/profile['scale']
                fixed, _, _, _ = motion.MotionController._decode_policy_action(ctx, 'walk',
                    np.asarray([first['raw_action']], np.float32), np.zeros(14, np.float32), seed, 0., 0.)
                fixed = np.asarray(fixed, np.float32)
                np.testing.assert_array_equal(fixed[:10], np.asarray(first['requested_target_deg'], np.float32)[:10])
                report['handoff'] = dict(last_hold_target_ticks=hold['attempted_target_ticks'],
                    last_hold_head_deg=held[10:14].tolist(), first_head_target_change_deg=(np.asarray(first['requested_target_deg'])[10:14]-held[10:14]).tolist(),
                    counterfactual_head_seed=seed[5:9].tolist(), counterfactual_first_head_deg=fixed[10:14].tolist(),
                    counterfactual_first_head_change_deg=(fixed[10:14]-held[10:14]).tolist(),
                    counterfactual_leg_target_identical=True, physical_counterfactual_verified=False)
            results.append(report)
out = dict(scope=__doc__, installed=False, hardware_access=False,
    official_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(), episodes=results,
    conclusion='Original nonzero head input was present. Clearing head EMA on a preceding non-HOME HOLD creates a repeatable startup target jump. This proves a transition discontinuity, not the complete cause of later backward falls.',
    limits=['Trials include operator intervention and unknown contact/support.',
            'Preserving a head filter seed has not been physically tested.',
            'Some stops were heartbeat or command-deadline failures, separate from geometric balance.',
            'No calibration, protection, gain, firmware or live service was changed.'])
a.output.write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(episodes=len(results), frames=sum(r['policy_frames'] for r in results),
    raw_error=max(r['original_actor_max_abs_error'] for r in results),
    tick_error=max(r['max_quantized_target_tick_error'] for r in results),
    handoffs=[dict(source=r['source'],old=r['handoff']['first_head_target_change_deg'][:2],
        candidate=r['handoff']['counterfactual_first_head_change_deg'][:2]) for r in results if 'handoff' in r]),ensure_ascii=False))
