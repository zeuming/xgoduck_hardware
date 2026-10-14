"""Build the official-runtime restoration from an exact installed snapshot.

No network, device access or deployment. Archived experiments remain unchanged.
"""
import argparse
import ast
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
REFERENCE = ROOT / 'output/xgoduck_runtime_upstream'
FW = ROOT / 'output/duck_sim/official-execution-20261012/candidate-v2/firmware'


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Source anchor changed: ' + before[:90])
    return text.replace(before, after, 1)


def prepare(root):
    baseline = root / 'baseline'
    receipt = json.loads((root / 'baseline.json').read_text())
    for name, expected in receipt['files'].items():
        if sha(baseline / name) != expected: raise ValueError('Baseline changed: ' + name)
    out = root / 'candidate'
    if out.exists(): raise ValueError('Use an empty candidate directory')
    shutil.copytree(baseline, out / 'host')
    host = out / 'host'
    def put(name, text):
        if name.endswith('.py'): compile(text, name, 'exec')
        (host / name).write_text(text)
    put('policy/motion.py', (HERE / 'official_runtime_motion.py').read_text())
    shutil.copyfile(REFERENCE / 'python/recovery.py', host / 'policy/official_recovery.py')
    text = (host / 'policy/production_motion.py').read_text()
    text = once(text, 'PERIOD, FRESH_SECONDS, HEARTBEAT_SECONDS = .02, .1, .4',
                'PERIOD, FRESH_SECONDS, HEARTBEAT_SECONDS = .02, .15, 1.0')
    text = once(text, "selected = self._preview_model if mode == 'shadow' else mode",
                'selected = self._select_policy_locked(mode)')
    text = once(text, """            if mode == 'hold':
                fraction = min(1., max(0., (self._clock() - self._hold_begin) / 3.))
                target = [(1 - fraction) * a + fraction * b for a, b in zip(self._hold_initial, self._fixed_stance_locked())]
""", """            if mode == 'hold':
                target = self._fixed_stance_locked()
            elif mode == 'walk' and self._recovery.phase == 'default_pose':
                target = self._default_recovery_target_locked()
""")
    text = once(text, 'previous, command = self._previous_action.copy(), self._command.copy()',
        "previous, command = self._previous_action.copy(), self._command.copy()\n            if mode == 'walk' and self._recovery.phase != 'walk': command.fill(0)")
    put('policy/production_motion.py', text)
    text = (host / 'policy/status_snapshot.py').read_text()
    text = once(text, '    view._transport = None',
                '    view._transport = None\n    if hasattr(controller, "_recovery"): view._recovery = copy.deepcopy(controller._recovery)')
    put('policy/status_snapshot.py', text)
    text = (host / 'service/worker-classes.py').read_text()
    text = once(text, 'from official_goal_execution import POLICY_POSITION_CAPS',
                'from motion import POLICY_POSITION_CAPS, OFFICIAL_RUNTIME_CAPS')
    text = once(text, 'motion.MOTION_EVIDENCE_CAPS, POLICY_POSITION_CAPS):',
                'motion.MOTION_EVIDENCE_CAPS, POLICY_POSITION_CAPS, OFFICIAL_RUNTIME_CAPS):')
    text = once(text, "if kind == POLICY_POSITION_CAPS and payload != b'':",
                "if kind in (POLICY_POSITION_CAPS, OFFICIAL_RUNTIME_CAPS) and payload != b'':")
    put('service/worker-classes.py', text)
    text = (host / 'service/native_runtime.py').read_text()
    text = once(text, 'motor_voltages voltage_range_v voltage_scale_hardware_verified command ',
                'motor_voltages voltage_range_v voltage_scale_hardware_verified official_runtime policy_goal_execution command ')
    text = once(text, "hold_control_protocol=1 if getattr(controller, '_joint_follow', None)",
        "hold_control_protocol=1 if getattr(controller, '_official_firmware', False) or getattr(controller, '_joint_follow', None)")
    put('service/native_runtime.py', text)
    # The shared adapter now uses explicit single buttons and the deployed
    # continuous command API. Do not regenerate the superseded R1 combinations.
    put('service/native_gamepad.py', (HERE / 'native_gamepad.py').read_text())
    put('service/gamepad_panel.html', (HERE / 'gamepad_panel.html').read_text())
    put('service/official_runtime_ui.js', (HERE / 'official_runtime_ui.js').read_text())
    text = (host / 'service/native_workbench.py').read_text()
    text = once(text, "(('forward', .2), ('side', .1), ('yaw', .5))", "(('forward', .4), ('side', .3), ('yaw', 1.))")
    text = text.replace('零速试跑最长30秒，到期缓慢恢复固定站姿并保持扭矩。需要释放扭矩时先托稳机身，再点击「停止运动 · 释放扭矩」。',
        '行走模型持续运行，速度回零后继续平衡。需要释放扭矩时先托稳机身，再点击「停止运动 · 释放扭矩」。')
    text = once(text, "    page = page.replace('</script>', SOFT_BUTTON_UI + '\\n</script>', 1)",
        "    page = page.replace('</script>', SOFT_BUTTON_UI + '\\n' + Path(__file__).with_name('official_runtime_ui.js').read_text() + '\\n</script>', 1)")
    put('service/native_workbench.py', text)
    # Compile and validate exact paired telemetry firmware, not the generic port.
    fw = out / 'firmware'
    shutil.copytree(FW, fw, ignore=shutil.ignore_patterns('.pio', '__pycache__'))
    text = (fw / 'firmware/motion_protocol.h').read_text()
    text = once(text, 'kStateFreshUs = 50000;', 'kStateFreshUs = 100000;')
    text = once(text, 'if (!feedback.fresh(now)) return record(Stale);',
        'if (!feedback.fresh(now) || uint32_t(now - feedback.imuUs) >= 50000) return record(Stale);')
    text = once(text, 'PositionRetryDetails = 26', 'PositionRetryDetails = 26, OfficialRuntimeCaps = 27')
    (fw / 'firmware/motion_protocol.h').write_text(text)
    text = (fw / 'firmware/main.cpp').read_text()
    text = once(text, '  if (id == 33) return 12;\n  return (id == 23 || id == 34) ? 10 : 8;',
                '  return id == 34 ? 10 : 6;')
    text = once(text, 'const uint8_t kp = 8, kd = 20, mouthKp = 10;',
                'const uint8_t kp = 6, kd = 20, mouthKp = 10;')
    text = text.replace('// Attended ID33 tracking comparison; other gains and safety gates retained.',
                        '// Official runtime transient gains (duck_config.h).')
    text = text.replace('// Single-joint attended comparison: right knee KP10; all other gains retained.',
                        '// Explicit per-joint write/readback uses the same official gain.')
    text = once(text, '  if (frame.type == motion::PolicyPositionCaps) {',
        '  if (frame.type == motion::OfficialRuntimeCaps) {\n    ack(frame, frame.length ? motion::BadPacket : motion::Ok); return;\n  }\n  if (frame.type == motion::PolicyPositionCaps) {')
    (fw / 'firmware/main.cpp').write_text(text)
    text = (fw / 'tests/test_firmware.cpp').read_text()
    text = text.replace("(id == 0xfe && packet[5] == 50 && packet[6] == 8)", "(id == 0xfe && packet[5] == 50 && packet[6] == 6)")
    text = text.replace("((id == 23 || id == 34) && packet[5] == 50 && packet[6] == 10)", "(id == 34 && packet[5] == 50 && packet[6] == 10)")
    text = text.replace("(id == 33 && packet[5] == 50 && packet[6] == 12)", "((id == 23 || id == 33) && packet[5] == 50 && packet[6] == 6)")
    text = text.replace("(id >= 30 && id <= 32 && packet[5] == 50 && packet[6] == 8)", "(id >= 30 && id <= 32 && packet[5] == 50 && packet[6] == 6)")
    text = text.replace("(expectedIds[i] == 33 ? 12 : (expectedIds[i] == 23 || expectedIds[i] == 34) ? 10 : 8)", "(expectedIds[i] == 34 ? 10 : 6)")
    for sid in (23,24,33): text = text.replace(f'mockKp[{sid}] = 6', f'mockKp[{sid}] = 8')
    for old in (10,12): text = text.replace(f'expectedKp == {old} && armDiagnostic.actualKp == 6', 'expectedKp == 6 && armDiagnostic.actualKp == 8')
    text = text.replace('A head servo retaining KP6', 'A head servo retaining nonofficial KP8')
    text = text.replace('The single-knee override', 'The official knee gain')
    (fw / 'tests/test_firmware.cpp').write_text(text)
    changes = {}; patch = []
    for before, after, label in ((baseline,host,'host'),(FW,fw,'firmware')):
        for p in sorted(after.rglob('*')):
            if not p.is_file() or p.suffix not in ('.py','.js','.html','.h','.cpp'): continue
            old = before / p.relative_to(after)
            if old.exists() and old.read_bytes() == p.read_bytes(): continue
            changes[label+'/'+str(p.relative_to(after))] = sha(p)
            patch.extend(difflib.unified_diff(old.read_text().splitlines(True) if old.exists() else [],
                p.read_text().splitlines(True),fromfile='before/'+str(p.relative_to(after)),tofile='after/'+str(p.relative_to(after))))
    (out / 'review.patch').write_text(''.join(patch))
    report = dict(baseline_seal=receipt['seal'], changes=changes, firmware_capability=27,
        official_recovery_sha256=sha(host/'policy/official_recovery.py'),
        deployed=False, hardware_walking_verified=False, calibration_changed=False,
        automatic_connection=False, automatic_motion_start=False)
    (out / 'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    print(json.dumps(prepare(parser.parse_args().root), indent=2))
