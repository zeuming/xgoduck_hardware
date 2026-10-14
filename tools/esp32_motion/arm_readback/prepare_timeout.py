"""Reproduce the reviewed ARM receive-budget candidate offline; never open UART."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
BASELINE = {
    'firmware/main.cpp': 'e807e55b1dd5643a69584f4188c5c24b60ee6e343d6bdcbaa1d7f159d17f9f6e',
    'firmware/motion_protocol.h': 'ed09b1eb6b01a2399ac1785460ab5814b15f33347b181286900960b2fef305f0',
}


def prepare(baseline, output):
    baseline = baseline.resolve()
    output = output.resolve()
    if output.exists():
        raise ValueError('Output already exists; previous evidence is preserved')
    for name, expected in BASELINE.items():
        if hashlib.sha256((baseline / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Not the reviewed diagnostic-v2 baseline: ' + name)
    shutil.copytree(baseline, output,
                    ignore=shutil.ignore_patterns('.pio', '__pycache__'))
    patch = HERE / 'arm-timeout-reference.patch'
    command = ['patch', '--batch', '--forward', '-p1', '-i', str(patch)]
    subprocess.run(command + ['--dry-run'], cwd=output, check=True)
    subprocess.run(command, cwd=output, check=True)
    shutil.copy2(HERE / 'test_timeout.cpp', output / 'tests/test_arm_timeout.cpp')
    report = {
        'offline_only': True,
        'baseline_files': BASELINE,
        'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
        'source_files': {
            name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in BASELINE
        },
        'arm_torque_receive_us': 6000,
        'budget_starts_after_confirmed_tx': True,
        'normal_poll_budget_changed': False,
        'retries_added': False,
        'model_calibration_gains_limits_watchdog_changed': False,
        'deployed': False,
    }
    (output / 'arm-timeout-preparation.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.baseline, args.output), indent=2))
