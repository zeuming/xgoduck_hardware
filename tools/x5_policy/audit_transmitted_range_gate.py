"""Replay saved simulation targets through the actual host range validator.

No controller constructor, hardware, network, or simulation is run. Raw policy
actions and transmitted targets are different contracts: the host validates
after decoding, EMA, fixed-head selection, and encoder rounding.
"""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def validator(candidate):
    from motion import MotionController, MotionError
    from rl_core import MOTOR_IDS
    # This pure method only reads calibration; never call the constructor/start.
    host = object.__new__(MotionController)
    by_id = {j['id']: j for j in candidate['calibration']['joints']}
    host._config = {'joints': [dict(id=sid, zero=by_id[sid]['zero_tick'],
        sign=by_id[sid]['sign'], min=by_id[sid]['min_tick'], max=by_id[sid]['max_tick'])
        for sid in MOTOR_IDS]}

    def check(policy_rad):
        angles = [0.] * len(MOTOR_IDS)
        for sid, value in zip(candidate['policy_joint_ids'], policy_rad):
            angles[MOTOR_IDS.index(sid)] = math.degrees(float(value))
        try:
            ticks = host._raw_targets_locked(angles, mode='walk')
            return dict(accepted=True, ticks=ticks)
        except MotionError:
            return dict(accepted=False, rejection=dict(host._target_rejection))
    return check, hashlib.sha256(inspect.getsource(MotionController._raw_targets_locked).encode()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    root = ROOT/'output/duck_sim/slow-gait-20261011'
    candidate_path = ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    candidate = json.loads(candidate_path.read_text())['candidate']
    check, source_sha = validator(candidate)
    cases = []
    for seed in range(4):
        folder = root/f'official-causal-v3-gains-ema-head-seed{seed}'
        rows = [json.loads(x) for x in (folder/'trace.jsonl').read_text().splitlines()]
        first = {}
        for row in rows:
            for field in ('adapter_requested_joint_rad', 'processed_joint_rad', 'actual_joint_rad'):
                result = check(row[field])
                if not result['accepted'] and field not in first:
                    first[field] = dict(frame=row['frame'], time_s=row['time_s'], **result['rejection'])
        cases.append(dict(seed=seed, first_conflict=first,
            trace_sha256=hashlib.sha256((folder/'trace.jsonl').read_bytes()).hexdigest()))
    report = dict(hardware_access=False, production_changed=False,
        validator_method_sha256=source_sha, cases=cases,
        correction='Earlier ranges factor rejects pre-EMA requests. The host rejects transmitted rounded targets after EMA. These are not identical gates.',
        scope='Frozen trajectory replay locates differences; not a counterfactual continued run or hardware proof.')
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
