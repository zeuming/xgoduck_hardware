"""Offline input-contract and recorded cadence audit. Never opens hardware.

The synthetic filter comparison uses the installed acceptance method and pinned
official SignalFilter at identical body inputs and sample intervals. It does not
establish the physical mounting, calibration, or walking capability.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import re
from pathlib import Path
import sys

import numpy as np


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean=float(a.mean()), p95=float(np.percentile(a, 95)),
                maximum=float(a.max())) if len(a) else dict(count=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'Preserve existing audit evidence'
    source_core = load('rl_core', args.upstream/'python/rl_core.py')
    source_proc = load('audit_source_proc', args.upstream/'python/proc.py')
    del sys.modules['rl_core']
    sys.path.insert(0, str(args.candidate/'policy'))
    import production_motion as native
    text=(args.candidate/'policy/production_motion.py').read_text()
    match=re.search(r'v \* SPEED_COUNTS_PER_UNIT \* 360\. / (\d+) \*',text)
    assert match, 'Velocity denominator must be read from the actual candidate'
    velocity_denominator=int(match.group(1))

    event_path = args.evidence/'walk-fixed-1.jsonl'
    events = [json.loads(line) for line in event_path.read_text().splitlines()]
    hardware = next(x['hardware'] for x in events if x.get('hardware'))
    imu = hardware['imu']
    _, matrix = native._axis_matrix(imu['axes'])
    now = [1.]
    # Constructor performs only local validation. A missing config path is used
    # so the synthetic test cannot inadvertently depend on another calibration.
    controller = native.MotionController({}, config_path=args.output.parent/'absent-audit-config.json',
        transport_factory=lambda port: (_ for _ in ()).throw(AssertionError('No hardware')),
        clock=lambda: now[0])
    controller._config['joints'] = copy.deepcopy(hardware['joints'])
    controller._config['imu'].update(copy.deepcopy(imu))
    source_filter = source_proc.SignalFilter()
    rng = np.random.default_rng(12100515)
    errors = dict(gravity=[], gyro_rad=[], velocity_same_scale=[])
    sample = 1000000
    for index in range(512):
        delta = 10000 if index == 0 else int(rng.integers(1000, 65001))
        if index:
            sample += delta
            now[0] += delta/1e6
        force = rng.normal(size=3)
        force *= rng.uniform(8.5, 10.5)/np.linalg.norm(force)
        gyro = rng.uniform(-35., 35., 3)
        positions = [int(x['zero'])+int(rng.integers(-100,101)) for x in hardware['joints']]
        velocity = rng.integers(-100,101,15).tolist()
        state = dict(sample_us=sample, imu_us=sample, online_mask=native.ALL_MASK,
            imu_ok=True, positions=positions, velocities=velocity,
            acc=(matrix.T@force).tolist(),
            gyro=(matrix.T@gyro+imu['bias_dps']).tolist())
        controller._accept_state_locked(state)
        actual = controller._filtered
        assert actual is not None
        angles = [(p-j['zero'])*360/4095*j['sign']
                  for p,j in zip(positions,hardware['joints'])]
        velocities = [v*50*360/velocity_denominator*j['sign']
                      for v,j in zip(velocity,hardware['joints'])]
        expected = source_filter.update((-force).tolist(), gyro.tolist(), angles,
            velocities, timestamp=now[0])
        errors['gravity'].append(float(np.max(np.abs(np.array(actual.grav)-expected['grav']))))
        errors['gyro_rad'].append(float(np.max(np.abs(np.array(actual.gyro_rad)-expected['gyro_rad']))))
        errors['velocity_same_scale'].append(float(np.max(np.abs(np.array(actual.vels)-expected['vels_deg']))))
    # Timestamp floating-point subtraction may differ from exact integer MCU dt.
    assert max(errors['gravity']) < 1e-12
    assert max(errors['gyro_rad']) < 1e-12
    assert max(errors['velocity_same_scale']) < 1e-9
    measured = []
    for name in ('default-stance-4.jsonl','default-stance-5.jsonl',
                 'default-stance-verified.jsonl','walk-fixed-1.jsonl'):
        for row in map(json.loads,(args.evidence/name).read_text().splitlines()):
            h = row.get('hardware')
            if not h or not (h.get('imu') or {}).get('acc_body'):
                continue
            i = h['imu']; ax,ay,az = i['acc_body']
            pitch = math.degrees(math.atan2(-ax,az))
            roll = math.degrees(math.atan2(ay,az))
            measured.append(dict(file=name,mode=h['mode'],sample_us=h['sample_us'],
                force_norm=float(np.linalg.norm(i['acc_body'])), raw_pitch_deg=pitch,
                raw_roll_deg=roll, filtered_pitch_deg=i['pitch_deg'],
                filtered_roll_deg=i['roll_deg'], pitch_difference_deg=pitch-i['pitch_deg']))
    trial = args.evidence/'hold-device-records/duck-source-hold-verified-20261012/connection-4-trial-1.json'
    frames = json.loads(trial.read_text())['frames']
    intervals = np.diff([x['monotonic'] for x in frames])*1000
    samples = np.diff([x['sample_us'] for x in frames])/1000
    durations = [(x['request_ended_monotonic']-x['monotonic'])*1000 for x in frames]
    display = [json.loads(x)['hardware'] for x in (args.evidence/'default-stance-verified.jsonl').read_text().splitlines()
               if json.loads(x).get('hardware')]
    timings = [dict(sample_us=x['sample_us'],mode=x['mode'],timing=x['timing']) for x in display]
    inputs = [event_path,trial,args.candidate/'policy/production_motion.py',
              args.candidate/'policy/rl_core.py',args.upstream/'python/proc.py',
              args.upstream/'python/rl_core.py',args.upstream/'sketch/scs_bus.h']
    result = dict(hardware_access=False,deployment=False,calibration_modified=False,
        upstream_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',
        evidence_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs},
        filter_contract=dict(samples=512,seed=12100515,axes=imu['axes'],
            axis_determinant=float(np.linalg.det(matrix)),
            maximum_errors={k:max(v) for k,v in errors.items()},
            native_overall_acc='negative calibrated specific force',
            native_overall_gyro='calibrated bias-corrected body angular velocity',
            limitation='Same body inputs and dt only. Physical mounting not verified; source uses host arrival dt, native uses MCU sample dt.'),
        speed_conversion=dict(source_counts_per_unit=50,native_counts_per_unit=50,
            source_denominator=4096,native_denominator=velocity_denominator,
            native_relative_scale_excess=4096/velocity_denominator-1,
            source_deg_s_per_unit=50*360/4096,native_deg_s_per_unit=50*360/velocity_denominator,
            disposition='Matches pinned source' if velocity_denominator==4096 else 'Source discrepancy retained explicitly; not asserted to cause the tilted stationary failure.'),
        measured_attitudes=measured,
        cadence=dict(frames=len(frames),effective_hz=float(1000/intervals.mean()),
            target_interval_ms=stats(intervals),mcu_sample_interval_ms=stats(samples),
            target_request_duration_ms=stats(durations),gaps_at_least_40ms=int(sum(intervals>=40)),
            window_measurements=timings,
            limitation='Recorded fixed-stance control, not ONNX walking. Request elapsed combines firmware wait, USB and host scheduling; it cannot identify each component.'),
        conclusion='Settled raw acceleration supports the tilted observation. The inverse-axis/filter adapter matches pinned source math; fixture geometry vs physical sensor alignment remains unknown. No walking acceptance.')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(filter_maximum_errors=result['filter_contract']['maximum_errors'],
        speed_scale_excess_percent=(4096/velocity_denominator-1)*100,
        hold_effective_hz=result['cadence']['effective_hz'],measured_rows=len(measured),hardware_access=False)))


if __name__ == '__main__':
    main()
