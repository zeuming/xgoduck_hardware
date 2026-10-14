"""Prepare a byte-equivalent control-math optimization; no install or device I/O.

Only the inspected controller baseline is accepted. Calibration, policy,
protection thresholds, recurrence and firmware remain untouched.
"""
import argparse
import hashlib
import json
from pathlib import Path

BASE = {
    'production_motion.py': 'c88d9eb8a0cf11e651f74d4471ead4668bfcc67cc8e005cff1217114d90a61ce',
    'candidate_controller.py': '7a748bc65adbdac6e9b88fffa4aa81a3f8a4cbd9c379045a477ce6f0b84529bd',
    'status_snapshot.py': '1969adfcee73a9dd8897d9cb5dc7b6a9a7c25829125f4bd1ca767e86102643ca',
    'current_pose_probe.py': 'f4fbe5d28e3c48a2eb395d5b8e724a2e48ea93c18e8ed346edc9d736337cc03c',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Inspected replacement anchor changed: ' + before[:60])
    return text.replace(before, after, 1)


def prepare(source, destination):
    source, destination = Path(source), Path(destination)
    for name, expected in BASE.items():
        if sha((source/'policy'/name).read_bytes()) != expected:
            raise ValueError('Baseline differs; inspect before preparing: ' + name)
    raw = (source/'policy/production_motion.py').read_bytes()
    production = once(raw.decode(), 'import collections\n', 'import collections\nimport binascii\n')
    begin, end = production.index('def crc16('), production.index('\n\ndef encode_frame(')
    production = production[:begin] + '''def crc16(data: bytes) -> int:
    # Identical CRC-16/CCITT-FALSE, implemented in C rather than per-bit Python.
    return binascii.crc_hqx(data, 0xffff)
''' + production[end:]
    candidate_raw = (source/'policy/candidate_controller.py').read_bytes()
    candidate = once(candidate_raw.decode(), sha(raw), sha(production.encode()))
    candidate = once(candidate, 'import production_motion\n',
                     'import production_motion\nfrom status_snapshot import clone_display_value\n')
    # Known bounded JSON trees. This produces fully detached copies without
    # deepcopy's memo traversal on each immutable scalar. No caching or aliasing.
    candidate = once(candidate, 'calibration=copy.deepcopy(self._config[\'joints\'])',
                     'calibration=clone_display_value(self._config[\'joints\'])')
    candidate = once(candidate, 'diagnostic=copy.deepcopy(target.diagnostics)',
                     'diagnostic=clone_display_value(target.diagnostics)')
    candidate = once(candidate, 'target=copy.deepcopy(self._stand_deg)',
                     'target=clone_display_value(self._stand_deg)')
    values = {'policy/production_motion.py': production.encode(),
              'policy/candidate_controller.py': candidate.encode(),
              'policy/current_pose_probe.py': Path(__file__).with_name('current_pose_probe.py').read_bytes()}
    destination.mkdir(parents=True, exist_ok=False)
    manifest = dict(kind='control_latency_repair', installed=False, device_access=False,
                    policy_math_changed=False, calibration_changed=False, thresholds_changed=False,
                    inactive_diagnostic_outer_lock_removed=True,
                    before={}, after={})
    for name, data in values.items():
        compile(data, name, 'exec')
        path = destination/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        manifest['before'][name] = sha((source/name).read_bytes())
        manifest['after'][name] = sha(data)
    (destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.destination), indent=2))
