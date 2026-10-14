"""Rebuild the HEAD-PROFILE-20261012 source overlay; files only, no deployment.

The baseline must be a fresh extraction of the installed X5 bundle. Parameters
come from the archived operator-confirmed standing trial, not mass guesses.
Calibration and firmware are not part of the overlay. A live installer still
needs its own exact deployment seal, current calibration hash and idle-UART gate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PATCH = Path(__file__).with_name('recovered-head-profile.patch')
FILES = {'policy/motion.py', 'service/bootstrap.py', 'service/native_runtime.py'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--profile', type=Path, required=True)
    a = p.parse_args()
    baseline, output = a.baseline.resolve(), a.output.resolve()
    if output.exists():
        raise FileExistsError('Do not overwrite a reviewed candidate')
    # Keep patch destinations inside this checkout; git apply neither stages nor
    # commits changes. A mismatched baseline fails instead of replacing files.
    destination = output.relative_to(ROOT).as_posix()
    shutil.copytree(baseline, output)
    cmd = ['git', 'apply', '--directory='+destination, str(PATCH)]
    subprocess.run(cmd[:2]+['--check']+cmd[2:], cwd=ROOT, check=True)
    subprocess.run(cmd, cwd=ROOT, check=True)
    source_hashes = {}
    changed = []
    for old in baseline.rglob('*.py'):
        name = old.relative_to(baseline).as_posix()
        new = (output/name).read_bytes()
        compile(new, name, 'exec')
        source_hashes[name] = hashlib.sha256(new).hexdigest()
        if new != old.read_bytes():
            changed.append(name)
    if set(changed) != FILES:
        raise ValueError('Unexpected source difference')
    profile = json.loads(a.profile.read_text())
    vector = profile['walk_head_command_rad']
    expected = [-0.48095456322770314, -0.40883986006837697,
                -0.10126745550033033, -0.05216808313653381]
    if vector != expected:
        raise ValueError('Profile differs from the recovered standing evidence')
    report = dict(changed_sources=sorted(changed), source_sha256=source_hashes,
                  patch_sha256=hashlib.sha256(PATCH.read_bytes()).hexdigest(),
                  profile_sha256=hashlib.sha256(a.profile.read_bytes()).hexdigest(),
                  installed=False, hardware_access=False)
    (output/'head-profile-preparation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(changed_sources=sorted(changed), output=str(output))))


if __name__ == '__main__':
    main()
