"""Prepare additive native slider-control edits from a retrieved source tree.

Only writes a new review directory. Never connects, restarts or deploys. Source
anchors must match exactly; preserve all unrelated installed policy changes.
"""
import argparse
import hashlib
import json
from pathlib import Path


def once(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError('Source changed; review required: '+old[:100])
    return text.replace(old, new)


def patch_runtime(text):
    text = once(text, 'command forward_trial', 'command joint_follow forward_trial')
    text = once(text, 'hold_control_protocol=2)',
        "hold_control_protocol=1 if getattr(controller, '_joint_follow', None) and controller._joint_follow['active'] else 2,\n"
        "                                joint_follow=dict(active=bool(getattr(controller, '_joint_follow', None) and controller._joint_follow['active'])))")
    text = once(text, '        exec(self._classes, namespace)\n',
        '        exec(self._classes, namespace)\n'
        '        from joint_follow import controller_class\n'
        "        namespace['OneShotController'] = controller_class(namespace['OneShotController'],\n"
        '            self.motion.MotionError, self.motion.MOTOR_IDS, self.motion.ALL_MASK, self.motion.TARGET)\n')
    text = once(text, "        if action not in ('start', 'heartbeat', 'command', 'stop',",
        "        if action not in ('start', 'heartbeat', 'command', 'joint-target', 'stop',")
    marker = "        if action == 'heartbeat':\n            if type(body.get('generation')) is not int:"
    text = once(text, marker,
        "        if action == 'joint-target':\n"
        "            return controller.joint_target(expected_generation=body.get('generation'),\n"
        "                joint_id=body.get('joint_id'), angle_deg=body.get('angle_deg'), sequence=body.get('sequence'))\n"+marker)
    # Both full and receipt START dispatch retain existing audit/owner paths.
    text = once(text, "result = controller.start_mode(body.get('mode'),",
        "result = (controller.start_joint_follow(expected_generation=body['generation'],\n"
        "                    support_confirmed=body.get('support_confirmed') is True,\n"
        "                    clearance_confirmed=body.get('clearance_confirmed') is True)\n"
        "                    if body.get('joint_follow') is True else controller.start_mode(body.get('mode'),", count=2)
    text = once(text, "behavior_confirmed=body.get('behavior_confirmed') is True)",
        "behavior_confirmed=body.get('behavior_confirmed') is True))", count=2)
    return text


def patch_web(text):
    text = once(text, 'page_generation=self.generation, full_workbench=True)',
        'page_generation=self.generation, full_workbench=True, joint_follow_protocol=1)')
    text = once(text, "extras = dict(start={'mode', 'support_confirmed', 'behavior_confirmed', 'preview_model'},",
        "extras = dict(start={'mode', 'support_confirmed', 'behavior_confirmed', 'preview_model', 'joint_follow', 'clearance_confirmed'},")
    text = once(text, "command={'forward', 'side', 'yaw'}, heartbeat={'background'},",
        "command={'forward', 'side', 'yaw'}, heartbeat={'background'},\n"
        "                      **{'joint-target': {'joint_id', 'angle_deg', 'sequence'}},")
    marker = "        if action == 'heartbeat' and 'background' in body"
    text = once(text, marker,
        "        if action == 'start' and ('joint_follow' in body or 'clearance_confirmed' in body):\n"
        "            if body.get('joint_follow') is not True or body.get('mode') != 'hold' or body.get('clearance_confirmed') is not True:\n"
        "                raise ValueError('关节随动需要固定保持模式和活动间隙确认')\n"
        "        if action == 'joint-target':\n"
        "            from joint_follow import validate_target\n"
        "            validate_target(body.get('joint_id'), body.get('angle_deg'), body.get('sequence'),\n"
        "                            (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34))\n"+marker)
    text = once(text, "if action in ('start', 'command', 'heartbeat',",
        "if action in ('start', 'command', 'joint-target', 'heartbeat',")
    text = once(text, "if action in ('command', 'heartbeat', 'forward-trial',",
        "if action in ('command', 'joint-target', 'heartbeat', 'forward-trial',")
    text = once(text, "if action == 'stop' and hardware.get('mode') == 'hold' and (",
        "if action == 'stop' and hardware.get('mode') == 'hold' and not hardware.get('joint_follow', {}).get('active') and (")
    marker = "            if action == 'start' and hardware.get('mode') != 'off' and ("
    text = once(text, marker,
        "            if action == 'start' and body.get('joint_follow') is True and hardware.get('mode') != 'off':\n"
        "                raise ValueError('请先停止当前动作，再启用关节随动')\n"+marker)
    marker = "                if action == 'heartbeat':\n                    # Authentication"
    text = once(text, marker,
        "                if action == 'joint-target':\n"
        "                    return dict(result, page_generation=self.generation)\n"+marker)
    return text


def build(source, output):
    if output.exists():
        raise ValueError('Output must be a new directory')
    values = {}
    before = {}
    for name, patch in (('service/native_runtime.py', patch_runtime), ('service/native_workbench.py', patch_web)):
        raw = (source/name).read_bytes()
        before[name] = hashlib.sha256(raw).hexdigest()
        values[name] = patch(raw.decode()).encode()
    values['policy/joint_follow.py'] = Path(__file__).with_name('joint_follow.py').read_bytes()
    for name, raw in values.items():
        compile(raw, name, 'exec')
    for name, raw in values.items():
        path = output/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    manifest = dict(before=before, after={n:hashlib.sha256(v).hexdigest() for n,v in values.items()},
                    deployed=False, physical_following_verified=False)
    (output/'joint-follow-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output), indent=2))
