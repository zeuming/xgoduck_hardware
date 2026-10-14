"""Bounded sequential original/selected workflow pairs; offline CPU only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--production', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--modes', default='roulade,pick,getup')
    p.add_argument('--wall-seconds', type=float, default=5400.)
    p.add_argument('--selection', type=Path, help='Explicit candidate override; canonical selection remains unchanged')
    p.add_argument('--original-baseline', type=Path, help='Reuse completed original cases from the identical workflow protocol')
    p.add_argument('--bodies', default='reference,rear-mid,rear-high')
    p.add_argument('--seeds', default='0,3')
    p.add_argument('--control-hz', type=int, choices=(40, 50), default=50)
    p.add_argument('--sensor-lag-steps', type=int, choices=(0, 1), default=0)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    script = Path(__file__).with_name('evaluate_body_workflow.py').resolve()
    rows = []
    modes = a.modes.split(',')
    assert len(set(modes)) == len(modes) and all(m in ('roulade', 'pick', 'getup') for m in modes)
    bodies, seeds = a.bodies.split(','), [int(s) for s in a.seeds.split(',')]
    assert len(set(bodies)) == len(bodies) and all(b in ('reference', 'rear-mid', 'rear-high') for b in bodies)
    assert len(set(seeds)) == len(seeds) and all(s >= 0 for s in seeds)
    if a.original_baseline is not None and (a.control_hz != 50 or a.sensor_lag_steps != 0):
        raise ValueError('Do not reuse a50Hz no-extra-lag baseline for timing stress')
    expected = len(modes)*len(bodies)*len(seeds)*2
    def save():
        tmp = a.output/'matrix.tmp'
        tmp.write_text(json.dumps(dict(cases=rows, expected_cases=expected, complete=len(rows)==expected,
            wall_seconds=time.monotonic()-started, pid=os.getpid(), hardware_access=False,
            control_hz=a.control_hz, additional_actor_sensor_lag_steps=a.sensor_lag_steps,
            installed_runtime_tested=False, evaluator_sha256=hashlib.sha256(script.read_bytes()).hexdigest()), indent=2)+'\n')
        tmp.replace(a.output/'matrix.json')
    save()
    for mode in modes:
        for body in bodies:
            for seed in seeds:
                original = None
                for model_set in ('original', 'selected'):
                    if time.monotonic()-started > a.wall_seconds:
                        raise TimeoutError('workflow matrix total budget')
                    key = f'{mode}-{body}-seed{seed}-{model_set}'
                    output = a.output/key
                    if model_set == 'original' and a.original_baseline is not None:
                        old = a.original_baseline/key/'gate.json'
                        original = json.loads(old.read_text())
                        assert original['mode'] == mode and original['model_set'] == 'original'
                        assert original['body'] == body and original['seed'] == seed
                        assert original['initial_fingerprint'] and not original.get('error')
                        row = dict(key=key, mode=mode, body=body, seed=seed, model_set=model_set,
                            exit=0, accepted=original['simulation_accepted'], gate=original['gate'],
                            error=None, transitions=original['transitions'], reused_gate=str(old),
                            reused_gate_sha256=hashlib.sha256(old.read_bytes()).hexdigest())
                        rows.append(row)
                        save()
                        continue
                    cmd = [sys.executable, str(script), '--root', str(a.root),
                           '--production', str(a.production), '--mode', mode,
                           '--body', body, '--seed', str(seed), '--models', model_set,
                           '--output', str(output), '--wall-seconds', '240']
                    cmd += ['--control-hz', str(a.control_hz), '--sensor-lag-steps', str(a.sensor_lag_steps)]
                    if a.selection is not None:
                        cmd += ['--selection', str(a.selection)]
                    with (a.output/(key+'.log')).open('x') as log:
                        result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=260)
                    gate = json.loads((output/'gate.json').read_text()) if (output/'gate.json').exists() else {}
                    row = dict(key=key, mode=mode, body=body, seed=seed, model_set=model_set,
                        exit=result.returncode, accepted=gate.get('simulation_accepted', False),
                        gate=gate.get('gate'), error=gate.get('error'), transitions=gate.get('transitions'))
                    if model_set == 'original':
                        original = gate
                    else:
                        row['paired_initial_state_exact'] = bool(gate.get('initial_fingerprint')) and gate.get('initial_fingerprint') == original.get('initial_fingerprint')
                        row['original_accepted'] = original.get('simulation_accepted', False)
                        row['regressed'] = row['original_accepted'] and not row['accepted']
                    rows.append(row)
                    save()
                    print(json.dumps(row), flush=True)
                    if result.returncode or row['error']:
                        raise RuntimeError('workflow evaluator failed: '+key)
    save()


if __name__ == '__main__':
    main()
