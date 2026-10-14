"""Stage a reviewed stop-policy change; never install, connect, or actuate.

Official baseline: LuwuDynamics/xgoduck_runtime_arduino 8cdbbd84710d856581982c9eaf0d5e2970666232.
scs_bus.h drops error-status replies without a global torque-off. This port
keeps the error mask for diagnostics and startup, but restores that running
policy. Target clipping, protocol validation and original watchdogs stay.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def replace_once(path, before, after):
    source = path.read_text()
    if source.count(before) != 1:
        raise ValueError(f'Unexpected baseline at {path}: {source.count(before)} matches')
    path.write_text(source.replace(before, after))


def prepare(host, firmware, destination):
    if destination.exists():
        raise ValueError('Use a new staging directory; existing evidence is preserved')
    shutil.copytree(host, destination, ignore=shutil.ignore_patterns('__pycache__', '.pio'))
    fw = destination / 'firmware'
    if fw.exists():
        shutil.rmtree(fw)  # Only the fresh local copy, never the baseline.
    shutil.copytree(firmware, fw, ignore=shutil.ignore_patterns('__pycache__', '.pio'))
    changes = []
    def edit(relative, before, after):
        path = destination / relative
        old = hashlib.sha256(path.read_bytes()).hexdigest()
        replace_once(path, before, after)
        changes.append(dict(path=relative, before_sha256=old,
                            after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    edit('firmware/firmware/motion_protocol.h',
         '    if (feedback.faultMask) { disarm(BadState); return true; }',
         '''    // STOP-POLICY-20261013: upstream scs_bus.h ignores a servo error
    // reply; it does not broadcast OFF to the other joints. Keep faultMask
    // visible and reject a NEW ARM on faults, but do not add global release
    // to an already running source policy. Legacy maintenance is unchanged.
    if (!sourceRuntime && feedback.faultMask) { disarm(BadState); return true; }''')
    edit('firmware/firmware/main.cpp',
         'const StopCause cause = feedback.faultMask ? DeviceFaultStop :',
         'const StopCause cause = (!lock.sourceRuntime && feedback.faultMask) ? DeviceFaultStop :')
    edit('firmware/firmware/motion_protocol.h',
         '''    if (!config.contains(feedback.positions)) return record(Range);
    cookie = u32(frame.payload);
    for (size_t i = 0; i < kMotorCount; ++i) {
      desired[i] = goals[i] = feedback.positions[i]; continuousGoals[i] = feedback.positions[i];
    }''',
         '''    // SAVED-RANGE-20261013: official startup does not reject the
    // measured pose against user-recorded limits. The user requests bounded
    // GOALS, not an actual-position stop (including a new source ARM).
    // Keep raw feedback untouched; clamp the initial hold goal exactly like
    // the host target adapter, before enabling torque. No new motion command
    // or automatic ARM is introduced. Legacy maintenance is unchanged.
    if (!sourceRuntime && !config.contains(feedback.positions)) return record(Range);
    for (size_t i = 0; i < kMotorCount; ++i)
      if (feedback.positions[i] > 4095) return record(Range);
    cookie = u32(frame.payload);
    for (size_t i = 0; i < kMotorCount; ++i) {
      uint16_t goal = feedback.positions[i];
      if (sourceRuntime) {
        const uint16_t low = config.minimum[i], high = config.maximum[i];
        if (goal < low) goal = low < high ? low + 1 : low;
        else if (goal > high) goal = low < high ? high - 1 : high;
      }
      desired[i] = goals[i] = goal; continuousGoals[i] = goal;
    }''')
    edit('firmware/tests/test_source_firmware.cpp',
         'armNow(6);mockServoFault[11]=4;sampleServos();assert(!lock.armed&&lock.lastError==motion::BadState);mockServoFault[11]=0;stopNow();',
         'armNow(6);mockServoFault[11]=4;sampleServos();assert(lock.armed&&feedback.faultMask==2);mockServoFault[11]=0;stopNow();')
    edit('policy/motion.py',
         "        if state['fault_mask'] or ((self._mode == 'off' or getattr(self, '_joint_follow', None) and self._joint_follow['active']) and state['online_mask'] != ALL_MASK):",
         '''        # STOP-POLICY-20261013, upstream controller.step/scs_bus.h:
        # retain bad/missing reply diagnostics, not a new all-joint OFF rule.
        # New ARM and attended joint-follow still require a complete scan.
        strict_entry = self._mode == 'off' or bool(
            getattr(self, '_joint_follow', None) and self._joint_follow['active'])
        if strict_entry and (state['fault_mask'] or state['online_mask'] != ALL_MASK):''')
    edit('policy/motion.py',
         '''        for p, j in zip(state['positions'], self._config['joints']):
            if not j['min'] <= p <= j['max']:
                raise MotionError(f"ID {j['id']} 实际位置超出保存范围")''',
         '''        # Upstream has no saved-range actual-position stop. At
        # normal model entry and while running, record excursions and constrain
        # every outgoing goal using the user's min+1/max-1 adapter. Do not
        # interpret the hand-recorded interval as an all-joint OFF trigger.
        # Only the independent attended joint-follow workflow retains its
        # measured-pose gate; ordinary HOLD/WALK/model start does not.
        if getattr(self, '_joint_follow', None) and self._joint_follow['active']:
            for p, j in zip(state['positions'], self._config['joints']):
                if not j['min'] <= p <= j['max']:
                    raise MotionError(f"ID {j['id']} 实际位置超出保存范围")''')
    edit('policy/motion.py',
         "        result['position_feedback_ages_us']=state.get('position_ages_us')",
         '''        result['position_feedback_ages_us']=state.get('position_ages_us')
        result['non_stopping_feedback_warnings'] = dict(
            fault_ids=list(result.get('fault_ids', [])),
            outside_saved_range=[dict(id=j['id'], position=p, min=j['min'], max=j['max'])
                for p, j in zip(state.get('positions', []), self._config['joints'])
                if j['min'] is not None and j['max'] is not None
                and not j['min'] <= p <= j['max']],
            scope='source-entry-and-running; fault/online startup and maintenance checks retained')''')
    gamepad = destination / 'service/native_gamepad.py'
    before = hashlib.sha256(gamepad.read_bytes()).hexdigest()
    # The canonical gamepad source carries the reviewed ownership fix.
    shutil.copyfile(Path(__file__).resolve().parents[1] / 'x5_policy/native_gamepad.py', gamepad)
    changes.append(dict(path='service/native_gamepad.py', before_sha256=before,
                        after_sha256=hashlib.sha256(gamepad.read_bytes()).hexdigest()))
    (destination / 'stop-policy-repair.json').write_text(json.dumps(dict(
        source_commit='8cdbbd84710d856581982c9eaf0d5e2970666232', changes=changes,
        installed=False, hardware_access=False,
        unchanged=['ONNX', 'calibration', 'target clamp', 'gains', 'timeouts',
                   'emergency stop', 'invalid targets', 'IMU and command watchdogs']), indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=Path, required=True)
    parser.add_argument('--firmware', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.host, args.firmware, args.output)
