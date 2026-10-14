"""Prepare all-model controls from an extracted current bundle, offline."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def prepare(source, output):
    if output.exists():
        raise ValueError('Use a new output directory; preserve the baseline')
    here = Path(__file__).resolve().parent
    shutil.copytree(source, output, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name in ('native_gamepad.py', 'gamepad_panel.html', 'command_view_page.py', 'command_view_ui.js'):
        shutil.copyfile(here / name, output / 'service' / name)
    workbench = output / 'service/native_workbench.py'
    text = workbench.read_text()
    before = "body.get('mode') in ('hold', 'walk')"
    if text.count(before) != 1:
        raise ValueError('Shared model admission source changed')
    workbench.write_text(text.replace(before, "body.get('mode') in ('hold',) + MODEL_NAMES", 1))
    digest = lambda data: hashlib.sha256(data).hexdigest()
    changes = {}
    for file in output.rglob('*'):
        if not file.is_file():
            continue
        relative = file.relative_to(output)
        original = source / relative
        if file.read_bytes() != original.read_bytes():
            if file.suffix == '.py':
                compile(file.read_bytes(), str(file), 'exec')
            changes[str(relative)] = dict(before_sha256=digest(original.read_bytes()),
                                          after_sha256=digest(file.read_bytes()))
    expected = {'service/' + n for n in ('native_gamepad.py', 'native_workbench.py',
        'gamepad_panel.html', 'command_view_page.py', 'command_view_ui.js')}
    if set(changes) != expected:
        raise ValueError('Unexpected change set')
    return dict(changes=changes, model_policy_firmware_calibration_unchanged=True,
                automatic_motion_start=False, deployed=False, hardware_access=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output), indent=2))
