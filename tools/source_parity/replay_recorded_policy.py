"""Offline official-actor diagnostic; frozen measurements are not a plant replay.

Compare recorded tilted gravity with an explicitly counterfactual upright
gravity. No UART, calibration writes, range edits, or deployment.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import onnxruntime as ort

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--candidate', type=Path, required=True)
p.add_argument('--events', type=Path, required=True)
p.add_argument('--model', type=Path, required=True)
p.add_argument('--trial', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
sys.path.insert(0, str(a.candidate/'policy'))
import motion
from rl_core import State, build_obs, policy_profile

events = [json.loads(x) for x in a.events.read_text().splitlines()]
h = next(x['hardware'] for x in events if x.get('event') == 'display'
         and (x.get('hardware') or {}).get('mode') == 'walk')
options = ort.SessionOptions()
options.intra_op_num_threads = 1
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
session = ort.InferenceSession(str(a.model), sess_options=options,
                               providers=['CPUExecutionProvider'])
profile = policy_profile(session)
# Construction is validation-only. It cannot open hardware.
controller = motion.MotionController({'walk': session})
trace = json.loads(a.trial.read_text())['frames']
rows = h['rawdata']
st = State([0.]*3, [0.]*3, [x['angle_deg'] for x in rows],
           [x['velocity_deg_s'] for x in rows], h['imu']['gravity'],
           np.deg2rad(h['imu']['gyro_body']).tolist())
results = {}
for name, gravity in (('recorded_tilt_frozen', st.grav),
                       ('upright_gravity_only_counterfactual', [0., 0., -1.])):
    st.grav = list(gravity)
    previous = np.zeros(14, np.float32)
    smooth = previous.copy()
    frames = []
    for i in range(8):
        obs = build_obs(st, previous, np.zeros(3, np.float32),
                        np.zeros(4, np.float32), home_rad=profile['home_rad']).reshape(1, 61)
        raw = session.run(None, {session.get_inputs()[0].name: obs})[0]
        target, previous, smooth, _ = controller._decode_policy_action(
            'walk', raw, previous, smooth, 0., 0.)
        ticks = [round(j['zero'] + d*4095/360*j['sign'])
                 for j,d in zip(h['joints'], target)]
        rejections = [dict(id=j['id'], target_ticks=t, min=j['min'], max=j['max'])
                      for j,t in zip(h['joints'], ticks)
                      if not j['min'] <= t <= j['max']]
        frames.append(dict(index=i, observation=obs.tolist(), raw_action=raw.tolist(),
                           EMA=smooth.tolist(), target_deg=list(target), ticks=ticks,
                           rejections=rejections))
    results[name] = frames
measured = [x['attempted_target_ticks'] for x in trace]
predicted = [x['ticks'] for x in results['recorded_tilt_frozen'][:len(measured)]]
leg_indexes = [0,1,2,3,4,5,6,7,8,9]
comparison = [dict(frame=i, max_leg_tick_difference=max(abs(p[j]-m[j]) for j in leg_indexes))
              for i,(p,m) in enumerate(zip(predicted, measured))]
result = dict(model_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),
              measured_pitch_deg=h['imu']['pitch_deg'], measured_sample_us=h['sample_us'],
              distinction='Frozen published state; not exact submitted observation, plant replay, or physical success',
              recorded_target_comparison=comparison, cases=results,
              hardware_access=False, calibration_modified=False)
a.output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(pitch=result['measured_pitch_deg'], recorded_target_comparison=comparison,
    first_3_ID22_deg={k:[x['target_deg'][7] for x in v[:3]] for k,v in results.items()},
    first_3_leg_rejections={k:[x['rejections'] for x in v[:3]] for k,v in results.items()})))
