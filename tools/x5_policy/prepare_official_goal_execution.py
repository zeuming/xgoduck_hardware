"""Prepare an isolated execution fix; never connects, deploys or changes calibration."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT/'output/duck_sim/overnight-20261011/host-deferred-v4'
FIRMWARE = ROOT/'output/duck_sim/repair-20261011/firmware'
PINS = {
    'policy/production_motion.py': '66b214ff661861476e5a114f98bf6ac32ba0d6834bbfb28a6b769155ebd32367',
    'policy/motion.py': '273e5338e894584aa69e47178131b054421f643031e6e2d408ed63467084f353',
    'policy/serial_owner_controller.py': 'eca65e27dfe88157a2bfbfc83da90f9304189c31d2d0f1c4a38ccdaa368a3bf8',
    'policy/candidate_controller.py': '4cfff2c93aae5b3b2f1d4461d50ed996818600da7c461f20167941cfb8e907d2',
}


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Reviewed anchor changed: '+before[:100])
    return text.replace(before, after, 1)


def prepare(out, *, official_contract=False):
    if out.exists(): raise ValueError('Use a new output directory')
    for name, expected in PINS.items():
        if sha(HOST/name) != expected: raise ValueError('Host source changed: '+name)
    out.mkdir(parents=True)
    host = out/'host'
    shutil.copytree(HOST, host, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    def edit(name, value):
        compile(value, name, 'exec')
        (host/name).write_text(value)
    text=(host/'policy/production_motion.py').read_text()
    text=once(text, '    def _record_timing(self, name, began, failed):',
        "    def _target_payload_locked(self, raw):\n        return struct.pack('<I15H', self._cookie, *raw)\n\n"
        '    def _record_timing(self, name, began, failed):')
    for prefix in ('self._io_submit_target_locked(', 'self._request(TARGET, '):
        text=once(text, prefix+"struct.pack('<I15H', self._cookie, *raw)", prefix+'self._target_payload_locked(raw)')
    edit('policy/production_motion.py', text)
    text=(host/'policy/candidate_controller.py').read_text()
    edit('policy/candidate_controller.py',once(text,PINS['policy/production_motion.py'],sha(host/'policy/production_motion.py')))
    text=(host/'policy/motion.py').read_text()
    text=once(text, 'from candidate_controller import ResearchMotionController',
        'from candidate_controller import ResearchMotionController\nfrom official_goal_execution import OfficialGoalExecutionMixin')
    text=once(text, 'class MotionController(ControlReadinessMixin,',
        'class MotionController(OfficialGoalExecutionMixin, ControlReadinessMixin,')
    text=once(text, "struct.unpack('<I15H', payload)", "struct.unpack_from('<I15H', payload)")
    text=once(text, 'nominal_firmware_slew_ticks_s=600,',
        "nominal_firmware_slew_ticks_s=None, execution_profile='explicit-policy-position-v1',")
    text=once(text, 'firmware_slew_ticks_s=600,firmware_changed=False,',
        "firmware_slew_ticks_s=None,firmware_changed=True,execution_profile='explicit-policy-position-v1',")
    edit('policy/motion.py',text)
    text=(host/'policy/serial_owner_controller.py').read_text()
    edit('policy/serial_owner_controller.py',once(text,"struct.unpack('<I15H',payload)","struct.unpack_from('<I15H',payload)"))
    shutil.copyfile(Path(__file__).with_name('official_goal_execution.py'),host/'policy/official_goal_execution.py')
    if official_contract:
        shutil.copyfile(Path(__file__).with_name('official_policy_contract.py'),host/'policy/official_policy_contract.py')
        text=(host/'policy/motion.py').read_text()
        text=once(text, 'from official_goal_execution import OfficialGoalExecutionMixin',
            'from official_goal_execution import OfficialGoalExecutionMixin\nfrom official_policy_contract import OfficialPolicyContractMixin')
        text=once(text, 'class MotionController(OfficialGoalExecutionMixin, ControlReadinessMixin,',
            'class MotionController(OfficialPolicyContractMixin, OfficialGoalExecutionMixin, ControlReadinessMixin,')
        edit('policy/motion.py',text)
    # The real native service wraps SerialTransport with an explicit command
    # allowlist. Negotiate the new read-only capability through that wrapper.
    text=(host/'service/worker-classes.py').read_text()
    text=once(text, "    def request(self, kind, payload=b''):\n",
        "    def request(self, kind, payload=b''):\n        from official_goal_execution import POLICY_POSITION_CAPS\n")
    text=once(text, 'motion.TELEMETRY_CAPS, motion.MOTION_EVIDENCE_CAPS):',
        'motion.TELEMETRY_CAPS, motion.MOTION_EVIDENCE_CAPS, POLICY_POSITION_CAPS):')
    text=once(text, '        if health_error and kind not in (motion.STOP, motion.STATE, motion.VOLTAGE_STATE):',
        "        if kind == POLICY_POSITION_CAPS and payload != b'':\n"
        "            raise ProbeError('Policy position capability query must be empty')\n"
        '        if health_error and kind not in (motion.STOP, motion.STATE, motion.VOLTAGE_STATE):')
    edit('service/worker-classes.py', text)
    fw=out/'firmware'
    fw.mkdir()
    # Preserve the exact paired telemetry/retry firmware, not an older generic
    # build. Copy only source/build inputs and tests; never stale binaries.
    for name in ('platformio.ini','motion_partitions.csv'):
        shutil.copyfile(FIRMWARE/name,fw/name)
    for name in ('firmware','tests'):
        shutil.copytree(FIRMWARE/name,fw/name)
    generic=ROOT/'tools/esp32_motion/firmware'
    # Interlock is hardware independent and identical before this fix. Review
    # its complete before/after in the emitted patch, preserve all other fields.
    before=(FIRMWARE/'firmware/motion_protocol.h').read_text()
    reference=(ROOT/'output/duck_sim/position-read-retry-candidate/firmware-source/firmware/motion_protocol.h').read_text()
    if before[before.index('class Interlock {'):] != reference[reference.index('class Interlock {'):]:
        raise ValueError('Firmware interlock differs from reviewed baseline')
    current=(generic/'motion_protocol.h').read_text()
    text=before[:before.index('class Interlock {')]+current[current.index('class Interlock {'):]
    text=once(text,'constexpr float kSlewCountsPerSec = 600.0f;',
        'constexpr float kSlewCountsPerSec = 600.0f; // Legacy pose path only.\nconstexpr uint8_t kPolicyPositionProfile = 1;')
    text=once(text,'OffRegisterDetails = 23,','OffRegisterDetails = 23, PolicyPositionCaps = 24,')
    (fw/'firmware/motion_protocol.h').write_text(text)
    text=(FIRMWARE/'firmware/main.cpp').read_text()
    text=once(text,'frame.type == motion::Target && error == motion::BadState && frame.length == 34 &&',
        'frame.type == motion::Target && error == motion::BadState &&\n      (frame.length == 34 || (frame.length == 35 && frame.payload[34] == motion::kPolicyPositionProfile)) &&')
    text=once(text,'  if (frame.type == motion::FaultDetails) {',
        '  if (frame.type == motion::PolicyPositionCaps) {\n'
        '    ack(frame, frame.length ? motion::BadPacket : motion::Ok); return;\n  }\n'
        '  if (frame.type == motion::FaultDetails) {')
    text=once(text,'// Goal position + zero duration + zero speed; configured RAM slew limits\n  // the goal changes. No ID/EPROM writes exist in this firmware.',
        '// Upstream position goal, zero duration and speed; legacy pose slew\n  // or explicit policy-position execution is selected by Interlock.')
    (fw/'firmware/main.cpp').write_text(text)
    for name in ('test_policy_position.cpp','test_motion.cpp'):
        shutil.copyfile(ROOT/'tools/esp32_motion/tests'/name,fw/'tests'/name)
    patch=[]
    for base,dest,label in ((HOST,host,'host'),(FIRMWARE,fw,'firmware')):
        for path in sorted(dest.rglob('*')):
            if path.suffix not in ('.py','.cpp','.h') or not path.is_file():continue
            original=base/path.relative_to(dest)
            if not original.exists() or original.read_bytes()!=path.read_bytes():
                patch.extend(difflib.unified_diff(original.read_text().splitlines(True) if original.exists() else [],path.read_text().splitlines(True),
                    fromfile='before/'+label+'/'+str(path.relative_to(dest)),tofile='after/'+label+'/'+str(path.relative_to(dest))))
    (out/'review.patch').write_text(''.join(patch))
    manifest=dict(kind='official_policy_goal_execution_candidate',deployed=False,flashed=False,
        hardware_access=False,hardware_walking_verified=False,model_changed=False,calibration_changed=False,
        ranges_changed=False,watchdog_changed=False,tilt_limit_changed=False,
        original_policy_contract_restored=official_contract,
        host_baseline=str(HOST),firmware_baseline=str(FIRMWARE),host_pins=PINS,
        protocol=dict(capability=24,ack=[24,0],target_bytes=35,profile_byte=1,legacy_target_bytes=34),
        before_firmware={str(p.relative_to(FIRMWARE)):sha(p) for p in (FIRMWARE/'firmware').glob('*') if p.is_file()},
        after={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--official-contract',action='store_true',help='Also restore original metadata HOME and raw-action EMA, without research roll remapping')
    args=parser.parse_args()
    print(json.dumps(prepare(args.output,official_contract=args.official_contract),indent=2))
