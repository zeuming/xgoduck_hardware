"""Rebuild the reviewed local source candidate. No SSH, UART or deployment.

The installed-source snapshot and v6 firmware are immutable input artifacts.
Every changed file is checked before and after applying exact line edits.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--firmware-baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--recipe', type=Path, default=Path(__file__).with_name('changes.json'))
    args = parser.parse_args()
    spec = json.loads(args.recipe.read_text())
    baseline = args.baseline.resolve()
    firmware = args.firmware_baseline.resolve()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit('Output must not exist; refusing to overwrite any source or installation.')
    if any(parent == output or output in parent.parents or parent in output.parents
           for parent in (baseline, firmware)):
        raise SystemExit('Output must be separate from both input trees.')
    snapshot = json.loads((baseline/'snapshot.json').read_text())
    if snapshot['installation_sha256'] != spec['baseline_installation_sha256']:
        raise SystemExit('Installed-source seal differs from the reviewed input.')
    for relative, expected in snapshot['all_archive_members'].items():
        file = baseline/relative
        if file.is_file() and sha(file.read_bytes()) != expected:
            raise SystemExit(f'Baseline hash mismatch: {relative}')
    # Check every modified input before creating output; new paths must be absent.
    for change in spec['changes']:
        relative = Path(change['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise SystemExit('Invalid patch path')
        original = (firmware/Path(*relative.parts[1:])
                    if relative.parts[0] == 'firmware' else baseline/relative)
        actual = sha(original.read_bytes()) if original.is_file() else None
        if actual != change['before_sha256']:
            raise SystemExit(f'Reviewed source mismatch: {relative}')
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', '.pio')
    shutil.copytree(baseline, output, ignore=ignore)
    shutil.copytree(firmware, output/'firmware', ignore=ignore)
    for change in spec['changes']:
        file = output/change['path']
        lines = file.read_text().splitlines(keepends=True) if file.exists() else []
        for edit in reversed(change['edits']):
            lines[edit['start']:edit['end']] = edit['lines']
        data = ''.join(lines).encode()
        if sha(data) != change['after_sha256']:
            raise SystemExit(f'Patched hash mismatch: {change["path"]}')
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(data)
    receipt = {'schema': 1, 'installed': False, 'hardware_access': False,
               'baseline_installation_sha256': spec['baseline_installation_sha256'],
               'upstream_commit': spec['upstream_commit'],
               'source_changes_sha256': sha(args.recipe.read_bytes()),
               'files': {c['path']: c['after_sha256'] for c in spec['changes']}}
    (output/'source-candidate.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps({'output': str(output), 'verified_changed_files': len(spec['changes']),
                      'installed': False, 'hardware_access': False}))


if __name__ == '__main__':
    main()
