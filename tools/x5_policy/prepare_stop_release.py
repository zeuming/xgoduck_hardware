"""Reproduce the reviewed STOP recovery patch without touching hardware.

Apply only exact unified-diff contexts. Do not fuzz-match a different installed
runtime; a changed baseline must be reviewed again before installation.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil


def prepare(source, output):
    patch = Path(__file__).with_name('stop_release_recovery.patch').read_text().splitlines(True)
    changes = {}
    index = 0
    while index < len(patch):
        assert patch[index].startswith('--- a/')
        name = patch[index][6:].rstrip('\n')
        assert patch[index+1] == '+++ b/' + name + '\n'
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        previous = (source/name).read_text().splitlines(True) if (source/name).exists() else []
        result, cursor = [], 0
        index += 2
        while index < len(patch) and not patch[index].startswith('--- a/'):
            match = re.fullmatch(r'@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@.*\n', patch[index])
            assert match, 'Malformed hunk'
            old_start, old_count, new_start, new_count = match.groups()
            start = max(0, int(old_start)-1)
            assert start >= cursor
            result.extend(previous[cursor:start])
            cursor = start
            index += 1
            old_seen = new_seen = 0
            while index < len(patch) and not patch[index].startswith(('@@ ', '--- a/')):
                line = patch[index]
                assert line[0] in ' +-'
                if line[0] in ' -':
                    assert cursor < len(previous) and previous[cursor] == line[1:], 'Baseline changed: '+name
                    cursor += 1
                    old_seen += 1
                if line[0] in ' +':
                    result.append(line[1:])
                    new_seen += 1
                index += 1
            assert old_seen == int(old_count or 1) and new_seen == int(new_count or 1)
        result.extend(previous[cursor:])
        changes[name] = ''.join(result).encode()
    assert set(changes) == {'service/native_runtime.py', 'service/native_workbench.py',
        'service/worker-classes.py', 'service/official_runtime_ui.js', 'policy/stop_release.py'}
    assert changes['policy/stop_release.py'] == Path(__file__).with_name('stop_release.py').read_bytes()
    for name, data in changes.items():
        if name.endswith('.py'):
            compile(data, name, 'exec')
    assert not output.exists(), 'Use a new output directory; do not overwrite evidence'
    shutil.copytree(source, output, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name, data in changes.items():
        target = output/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    return {name: hashlib.sha256(data).hexdigest() for name, data in changes.items()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output), indent=2))
