"""Build a source-only native diagnostic candidate; never connects or deploys."""
import argparse
import hashlib
import json
from pathlib import Path

PINS = {
    'policy/motion.py': 'f7f4e2eb50f7532f22463b7d2c66e55ccbc04329ff77335a3dd7ba25784e8b26',
    'service/native_runtime.py': 'cf13117ea0f08aff771a796c96e103dcfd8c45b366e88f6f3bdeeb27bbfa2a00',
    'service/native_workbench.py': 'fbce9ad1d5b0975cc85b8caf516a08bf3156af67df4f95c57e98c2416bfd10f9',
}


def once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('Reviewed source anchor changed: '+old[:100])
    return source.replace(old, new)


def build(source, output):
    values = {}
    for name, digest in PINS.items():
        raw = (source/name).read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            raise ValueError('Source changed since review: '+name)
        values[name] = raw.decode()
    values['policy/current_pose_probe.py'] = Path(__file__).with_name('current_pose_probe.py').read_text()
    values['policy/motion.py'] += '''
# Explicit attended current-pose diagnostic; no automatic invocation.
from current_pose_probe import controller_class as current_pose_controller_class
MotionController = current_pose_controller_class(MotionController, MotionError, MOTOR_IDS, ALL_MASK, TARGET)
'''
    runtime = values['service/native_runtime.py']
    runtime = once(runtime, 'command forward_trial tracking_probe last_action',
                   'command forward_trial tracking_probe current_pose_probe last_action')
    runtime = once(runtime,
        "if action not in ('start', 'heartbeat', 'command', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe'):",
        "if action not in ('start', 'heartbeat', 'command', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe', 'current-pose-probe'):")
    runtime = once(runtime, "            elif action == 'start':\n                result = controller.start_mode",
        "            elif action == 'current-pose-probe':\n"
        "                result = controller.current_pose_probe(expected_generation=body['generation'],\n"
        "                    support_confirmed=body.get('support_confirmed') is True,\n"
        "                    clearance_confirmed=body.get('clearance_confirmed') is True)\n"
        "            elif action == 'start':\n                result = controller.start_mode")
    runtime = once(runtime,
        "body.get('action') not in ('start', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe'):",
        "body.get('action') not in ('start', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe', 'current-pose-probe'):")
    runtime = once(runtime, "            elif action == 'capture-head-clearance':\n                # The original controller",
        "            elif action == 'current-pose-probe':\n"
        "                result = controller.current_pose_probe(expected_generation=body['generation'],\n"
        "                    support_confirmed=body.get('support_confirmed') is True,\n"
        "                    clearance_confirmed=body.get('clearance_confirmed') is True)\n"
        "            elif action == 'capture-head-clearance':\n                # The original controller")
    values['service/native_runtime.py'] = runtime
    web = values['service/native_workbench.py']
    web = once(web, "'tracking-probe': {'support_confirmed', 'joint_id'}})",
        "'tracking-probe': {'support_confirmed', 'joint_id'}, 'current-pose-probe': {'support_confirmed', 'clearance_confirmed'}})")
    web = once(web, "        if action == 'command':\n            for key, bound",
        "        if action == 'current-pose-probe' and (body.get('support_confirmed') is not True\n"
        "                or body.get('clearance_confirmed') is not True):\n"
        "            raise ValueError('请确认本次托稳机身、双脚着地和头颈及腿部间隙')\n"
        "        if action == 'command':\n            for key, bound")
    web = once(web,
        "if action in ('start', 'command', 'heartbeat', 'capture-head-clearance', 'forward-trial', 'tracking-probe') and (",
        "if action in ('start', 'command', 'heartbeat', 'capture-head-clearance', 'forward-trial', 'tracking-probe', 'current-pose-probe') and (")
    web = once(web,
        "audited = action in ('start', 'stop', 'disconnect', 'connect', 'capture-head-clearance', 'forward-trial', 'tracking-probe')",
        "audited = action in ('start', 'stop', 'disconnect', 'connect', 'capture-head-clearance', 'forward-trial', 'tracking-probe', 'current-pose-probe')")
    web = once(web,
        "receipt_action = live and action in ('start', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe')",
        "receipt_action = live and action in ('start', 'stop', 'capture-head-clearance', 'forward-trial', 'tracking-probe', 'current-pose-probe')")
    web = once(web, "                            or action == 'start' and after.get('mode') != body['mode']",
        "                            or action == 'current-pose-probe' and after.get('mode') != 'hold'\n"
        "                            or action == 'start' and after.get('mode') != body['mode']")
    web = once(web, "            if action == 'start' and hardware.get('mode') != 'off' and (",
        "            if action == 'current-pose-probe' and hardware.get('mode') != 'off':\n"
        "                raise ValueError('当前位置检查需要先托稳并停止，不能中断现有动作')\n"
        "            if action == 'start' and hardware.get('mode') != 'off' and (")
    values['service/native_workbench.py'] = web
    hashes = {}
    for name, text in values.items():
        compile(text, name, 'exec')
        path = output/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output/'hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
    return hashes


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output), indent=2))
