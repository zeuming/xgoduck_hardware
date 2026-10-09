"""Restart the native RAM workbench within the currently reviewed X5 boot.

Run on the laptop. A different X5 boot or an active UART owner is refused.
The command never starts IMU acquisition, enables torque, or starts motion.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import http.cookiejar
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tools/x5_recovery')]
import maintenance_transport as transport

BASE = ROOT / 'output/x5_recovery/replacement-board-20261004T011959+0800'
SETUP = BASE / 'native-workbench-20261006-15'
TEMPLATE = BASE / 'native-workbench-20261006-14'
LATEST = BASE / 'native-workbench-restart-latest.json'


def load(path):
    return transport.strict_json(transport.private_read(path, 2097152))


def digest(path):
    return hashlib.sha256(transport.private_read(path, 2097152)).hexdigest()


def prepare(live_snapshot=None):
    previous = SETUP
    if LATEST.exists():
        reference = load(LATEST)
        previous = Path(reference['directory'])
        if previous.parent != BASE or not previous.name.startswith('native-workbench-restart-'):
            raise ValueError('Unexpected previous service directory')
        if set(reference['receipts']) != {'detached-result.json', 'deployment.json', 'independence-check/proof.json'}:
            raise ValueError('Complete previous service receipts are required')
        for name, expected in reference['receipts'].items():
            if name not in ('detached-result.json', 'deployment.json', 'independence-check/proof.json') or digest(previous / name) != expected:
                raise ValueError('Previously verified service receipt changed')
    exited = load(previous / 'ssh-exit.json')
    checked = load(previous / 'independence-check/proof.json')
    result = load(previous / 'detached-result.json')
    manifest = load(previous / 'deployment.json')
    if (exited.get('process_exited') is not True or exited.get('ssh_exit_code') != 0
            or checked.get('kind') != 'native_independence_verified'
            or checked.get('ssh_exit_code') != 0 or checked.get('ssh_process_exited') is not True
            or checked.get('all_native_members_verified') is not True
            or checked.get('pid') != result.get('pid') or checked.get('model_count') != 4):
        raise ValueError('Actual completed launch and independent verification are required')

    directory = Path(tempfile.mkdtemp(prefix='native-workbench-restart-', dir=BASE))
    expected = load(SETUP / 'expected-preparation.json')
    expected['generation'] = int(time.time())
    transport.private_new(directory / 'expected-preparation.json', json.dumps(expected, ensure_ascii=False, indent=2) + '\n')
    for name in ('startup-state.json', 'calibration-latest.json', 'joint-zero-provenance.json', 'verify-independence.py'):
        data = (json.dumps(live_snapshot,ensure_ascii=False,indent=2)+'\n'
                if name=='startup-state.json' and live_snapshot is not None
                else transport.private_read(SETUP / name, 2097152))
        transport.private_new(directory / name, data)

    source = transport.private_read(TEMPLATE / 'stop-previous-service/remote-source.py', 262144).decode()
    nodes = {node.targets[0].id: node for node in ast.parse(source).body
             if isinstance(node, ast.Assign) and len(node.targets) == 1
             and isinstance(node.targets[0], ast.Name)
             and node.targets[0].id in ('EXPECTED', 'OLD', 'MANIFEST', 'STARTUP_SHA')}
    if set(nodes) != {'EXPECTED', 'OLD', 'MANIFEST', 'STARTUP_SHA'}:
        raise ValueError('Exact service-stop template differs')
    replacements = {'EXPECTED': expected,
                    'OLD': {key: result[key] for key in ast.literal_eval(nodes['OLD'].value)},
                    'MANIFEST': {key: manifest[key] for key in ast.literal_eval(nodes['MANIFEST'].value)},
                    'STARTUP_SHA': result['startup_fault']['proof_sha256']}
    lines = source.splitlines(keepends=True)
    for name, node in sorted(nodes.items(), key=lambda pair: pair[1].lineno, reverse=True):
        lines[node.lineno - 1:node.end_lineno] = [name + ' = ' + repr(replacements[name]) + '\n']
    stop_source = ''.join(lines).encode()
    if len(stop_source) > 262144:
        raise ValueError('Exact stop source exceeds its fixed bound')
    compile(stop_source, '<exact-reviewed-service-stop>', 'exec')
    (directory / 'stop-previous-service').mkdir(mode=0o700)
    transport.private_new(directory / 'stop-previous-service/remote-source.py', stop_source)

    original = transport.private_read(TEMPLATE / 'launch-native.py', 262144).decode()
    stop_block = original[original.index('    stop_directory = DIRECTORY /'):original.index('    channel = None')]
    launch = transport.private_read(SETUP / 'launch-native.py', 262144).decode()
    if launch.count('    channel = None') != 1:
        raise ValueError('Reviewed launch template differs')
    launch = launch.replace('    channel = None', stop_block + '    channel = None', 1)
    compile(launch, '<reviewed-workbench-restart>', 'exec')
    transport.private_new(directory / 'launch-native.py', launch)
    return directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true', help='Prepare sources locally without contacting X5')
    args = parser.parse_args()
    snapshot=None
    if not args.prepare_only:
        origin='http://192.168.1.11:8767'
        opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        opener.open(origin+'/',timeout=6).read()
        snapshot=json.load(opener.open(origin+'/api/status',timeout=6))
        if (snapshot.get('native_access',{}).get('full_workbench') is not True
                or snapshot.get('calibration') != load(SETUP/'calibration-latest.json')):
            raise ValueError('Current native calibration must be preserved before restart')
    directory = prepare(snapshot)
    if args.prepare_only:
        print(json.dumps(dict(prepared_directory=str(directory), hardware_contacted=False)))
        return
    subprocess.run([sys.executable, '-B', str(directory / 'launch-native.py')], check=True)
    subprocess.run([sys.executable, '-B', str(directory / 'verify-independence.py'), '--execute'], check=True)
    reference = dict(directory=str(directory), receipts={name: digest(directory / name) for name in (
        'detached-result.json', 'deployment.json', 'independence-check/proof.json')})
    descriptor, temporary = tempfile.mkstemp(prefix='.native-workbench-latest-', dir=BASE)
    with os.fdopen(descriptor, 'wb') as stream:
        stream.write((json.dumps(reference, indent=2) + '\n').encode())
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, LATEST)
    print('服务已重启：http://192.168.1.11:8767/')


if __name__ == '__main__':
    main()
