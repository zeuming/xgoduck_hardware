"""Build an offline host candidate requiring sample-bound firmware v2.

This never installs, connects, changes calibration, or grants walking approval.
The complete inspected baseline is copied before a bounded source patch.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

PINS = {
    'production_motion.py': 'a1d9f8082d5c805aa1dea4737048667320759cbe96545c8922bfe1bb76ab5a98',
    'candidate_controller.py': '5019439aababd73923619fd87087c89fdd2f86c65ad43ab84721e5cb3a5ae424',
    'motion.py': '0e2f87fb0fc303f757f6e31a08463dce2a547439a897e1a00021dde88873e95e',
    'control_readiness.py': 'cfd91fbf4a8bca71bdd171c356d6a6e0fa78caa9699607c3f52b5ca18babe47a',
}
BUDGET_PATCH_SHA256 = '1339da3f1b1d5e1295f0a0ebef943754ef76967f7990a8c992179166f70f1228'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Reviewed patch anchor changed: ' + old[:80])
    return text.replace(old, new, 1)


def production_patch(source):
    text = once(source, 'import collections\n',
        'import collections\nfrom causal_feedback import PAIRED_EXTENSION, decode_paired_extension\n')
    text = once(text, "payload = self.request(MOTION_EVIDENCE_CAPS)\n        if payload == b'XGODUCK_MOTION_EVIDENCE_1':\n            return True",
        "payload = self.request(MOTION_EVIDENCE_CAPS, b'\\x02')\n        if payload == b'XGODUCK_MOTION_EVIDENCE_2':\n            return 2")
    text = once(text,
        'STATE_PACKET.size + VOLTAGE_EXTENSION.size + MOTION_EVIDENCE_EXTENSION.size):',
        'STATE_PACKET.size + VOLTAGE_EXTENSION.size + MOTION_EVIDENCE_EXTENSION.size,\n'
        '                            STATE_PACKET.size + VOLTAGE_EXTENSION.size + PAIRED_EXTENSION.size):')
    anchor = '    return dict(sample_us=sample_us, imu_us=imu_us, online_mask=online,'
    text = once(text, anchor, '''    if len(payload) == STATE_PACKET.size + VOLTAGE_EXTENSION.size + PAIRED_EXTENSION.size:
        try:
            motion_evidence = decode_paired_extension(
                payload[STATE_PACKET.size + VOLTAGE_EXTENSION.size:],
                dict(sample_us=sample_us, armed=bool(armed), cookie=cookie,
                     online_mask=online, fault_mask=fault))
        except ValueError as error:
            raise MotionError(str(error)) from error
''' + anchor)
    text = once(text,
        '+ (MOTION_EVIDENCE_EXTENSION.size if self._motion_evidence_supported else 0))',
        '+ (PAIRED_EXTENSION.size if self._motion_evidence_supported == 2 else\n'
        '               MOTION_EVIDENCE_EXTENSION.size if self._motion_evidence_supported else 0))')
    text = once(text, "            state_payload = b'\\x01' if evidence_supported else b''",
        "            if evidence_supported != 2:\n"
        "                raise MotionError('此修复版本需要 MCU 采样配对协议 v2；旧固件不会启动动作')\n"
        "            state_payload = b'\\x02'")
    text = once(text,
        '+ (MOTION_EVIDENCE_EXTENSION.size if evidence_supported else 0)):',
        '+ PAIRED_EXTENSION.size):')
    return text


def prepare(source, destination, budget_patch=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    for name, digest in PINS.items():
        if sha((source/'policy'/name).read_bytes()) != digest:
            raise ValueError('Inspected host baseline differs: ' + name)
    if destination.exists():
        raise ValueError('Candidate destination must be new')
    product = production_patch((source/'policy/production_motion.py').read_text())
    controller_source = (source/'policy/candidate_controller.py').read_text()
    if budget_patch is not None:
        patch_data = Path(budget_patch).read_bytes()
        if sha(patch_data) != BUDGET_PATCH_SHA256:
            raise ValueError('Reviewed diagnostic-cost patch differs')
        controller_source = patch_data.decode()
    controller = once(controller_source,
        PINS['production_motion.py'], sha(product.encode()))
    replacements = {
        'production_motion.py': product.encode(),
        'candidate_controller.py': controller.encode(),
        'motion.py': Path(__file__).with_name('native_trial_adapter.py').read_bytes(),
        'control_readiness.py': Path(__file__).with_name('control_readiness.py').read_bytes(),
        'causal_feedback.py': Path(__file__).with_name('causal_feedback.py').read_bytes(),
    }
    for name, data in replacements.items():
        compile(data, name, 'exec')
    shutil.copytree(source, destination, symlinks=True,
        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name, data in replacements.items():
        (destination/'policy'/name).write_bytes(data)
    manifest = dict(kind='sample_bound_feedback_host_candidate',
        required_firmware_capability='XGODUCK_MOTION_EVIDENCE_2',
        hardware_access=False, installed=False, walking_validated=False,
        calibration_changed=False, model_changed=False, thresholds_changed=False,
        budget_patch_sha256=BUDGET_PATCH_SHA256 if budget_patch is not None else None,
        host_goal_fallback=False, firmware_tx_is_servo_acceptance=False,
        before=PINS, after={name:sha(data) for name,data in replacements.items()})
    (destination/'causal-host-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path)
    parser.add_argument('--budget-patch',type=Path,
        help='Reviewed candidate_controller.py from prepare_control_budget_repair')
    args=parser.parse_args()
    print(json.dumps(prepare(args.source,args.destination,args.budget_patch),indent=2))
