#!/usr/bin/env python3
"""Prepare a pinned, offline-only sample-bound goal telemetry candidate.

No devices, network access, upload command, gain/range change or calibration
write exists in this tool. The ordinary firmware source is never modified.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'output/duck_sim/position-read-retry-candidate/firmware-source'
BASE_APP = '59033a26323411641e9a5f73466463087dc8f288f66478e5b4af76977925e677'
PINNED = {
    'firmware/main.cpp':'277c0bc0ea9ae45a1386300b23d6264fa131c4a7f98eaa870c560b03a9f70684',
    'firmware/motion_protocol.h':'553204f5fef38b10cb6aa8d0f712098d89637bfd904a4391c4904fd113778340',
    'firmware/maintenance_impl.h':'2b98c934de90360779fa08baf06624ead142b8f574eaf0c18c7b0f5175d354fe',
    'firmware/off_register_details.h':'1b02cedaa669fc9e13889eedae253de6b9fae99396f319bffb160e0acfa3c482',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Pinned source transformation must match exactly once: '+old[:100])
    return text.replace(old,new,1)


def transform_main(text):
    text=replace_once(text,'uint32_t lastWrittenGoalsUs = 0;', '''uint32_t lastWrittenGoalsUs = 0;
// Diagnostic-only sequence/epoch counters. They never renew a safety clock.
uint32_t writtenGoalsSequence = 0, causalArmEpoch = 0;
bool writtenGoalsSeen = false;
struct SampleBoundGoals {
  bool valid = false;
  uint32_t sampleSequence = 0, sampleStartUs = 0, armCookie = 0, armEpoch = 0;
  uint32_t goalWriteUs = 0, goalWriteSequence = 0;
  uint16_t goals[15] = {};
} sampleBoundGoals;''')
    text=replace_once(text,'    lastWrittenGoalsUs = micros();', '''    lastWrittenGoalsUs = micros();
    ++writtenGoalsSequence; writtenGoalsSeen = true;''')
    text=replace_once(text,'void state(uint32_t seq, bool withVoltage = false, bool withMotionEvidence = false) {',
        'void state(uint32_t seq, bool withVoltage = false, uint8_t evidenceVersion = 0) {')
    text=replace_once(text,'  if (withMotionEvidence) {', '''  if (evidenceVersion == 2) {
    // One complete feedback scan, paired with the TX cache present BEFORE
    // its first read. A later goal write must not replace this snapshot.
    uint8_t* evidence = data + motion::kVoltageStateSize;
    evidence[0] = 2;
    evidence[1] = sampleBoundGoals.valid && !maintenanceMode && lock.armed &&
      lock.cookie == sampleBoundGoals.armCookie && causalArmEpoch == sampleBoundGoals.armEpoch &&
      sampleBoundGoals.sampleSequence == servoSampleSequence;
    motion::put32(evidence + 2, sampleBoundGoals.sampleSequence);
    motion::put32(evidence + 6, sampleBoundGoals.sampleStartUs);
    motion::put32(evidence + 10, sampleBoundGoals.armCookie);
    motion::put32(evidence + 14, sampleBoundGoals.goalWriteUs);
    motion::put32(evidence + 18, sampleBoundGoals.goalWriteSequence);
    for (size_t i = 0; i < motion::kMotorCount; ++i)
      motion::put16(evidence + 22 + 2*i, sampleBoundGoals.goals[i]);
    respond(0x93, seq, data, motion::kSampleBoundEvidenceSize); return;
  }
  if (evidenceVersion == 1) {''')
    text=replace_once(text,'withMotionEvidence ? motion::kMotionEvidenceSize : motion::kVoltageStateSize',
        'evidenceVersion == 1 ? motion::kMotionEvidenceSize : motion::kVoltageStateSize')
    text=replace_once(text,'''    const bool evidence = frame.type == motion::VoltageState && frame.length == 1 && frame.payload[0] == 1;
    if (frame.length && !evidence) ack(frame, lock.record(motion::BadPacket));
    else state(frame.sequence, frame.type == motion::VoltageState, evidence);''', '''    const uint8_t evidenceVersion = frame.type == motion::VoltageState && frame.length == 1 &&
      (frame.payload[0] == 1 || frame.payload[0] == 2) ? frame.payload[0] : 0;
    if (frame.length && !evidenceVersion) ack(frame, lock.record(motion::BadPacket));
    else state(frame.sequence, frame.type == motion::VoltageState, evidenceVersion);''')
    text=replace_once(text,'''  if (frame.type == motion::MotionEvidenceCaps) {
    if (frame.length) ack(frame, lock.record(motion::BadPacket));
    else {''', '''  if (frame.type == motion::MotionEvidenceCaps) {
    if (frame.length == 1 && frame.payload[0] == 2) {
      constexpr char capability[] = "XGODUCK_MOTION_EVIDENCE_2";
      respond(0x99, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability)-1);
    } else if (frame.length) ack(frame, lock.record(motion::BadPacket));
    else {''')
    text=replace_once(text,'      lock.armed = true;\n      refreshTorque();',
        '      lock.armed = true;\n      ++causalArmEpoch;\n      refreshTorque();')
    text=replace_once(text,'''void sampleServos() {
  const uint32_t startingSequence = servoSampleSequence;''', '''void sampleServos() {
  const uint32_t startingSequence = servoSampleSequence;
  SampleBoundGoals candidate;
  candidate.sampleStartUs = micros(); candidate.armCookie = lock.cookie;
  candidate.armEpoch = causalArmEpoch;
  candidate.goalWriteUs = lastWrittenGoalsUs; candidate.goalWriteSequence = writtenGoalsSequence;
  memcpy(candidate.goals, lastWrittenGoals, sizeof(candidate.goals));
  candidate.valid = lock.armed && lock.cookie && writtenGoalsSeen && !maintenanceMode &&
    uint32_t(candidate.sampleStartUs - candidate.goalWriteUs) < 0x80000000u;''')
    text=replace_once(text,'''  ++servoSampleSequence;
  updateMasks(micros()); checkSafety(); flushDisable();''', '''  ++servoSampleSequence;
  candidate.sampleSequence = servoSampleSequence;
  candidate.valid = candidate.valid && lock.armed && lock.cookie == candidate.armCookie &&
    causalArmEpoch == candidate.armEpoch && writtenGoalsSequence == candidate.goalWriteSequence &&
    cycleOnline == motion::kAllMotors && cycleFaults == 0;
  sampleBoundGoals = candidate;
  updateMasks(micros()); checkSafety(); flushDisable();''')
    return text


def prepare(output):
    for name,digest in PINNED.items():
        if sha(BASE/name) != digest:
            raise ValueError('Base hash mismatch: '+name)
    if sha(BASE/'.pio/build/esp32s3_motion/firmware.bin') != BASE_APP:
        raise ValueError('Base firmware application does not match the recorded candidate')
    if output.exists():
        raise ValueError('Refusing to overwrite existing output')
    shutil.copytree(BASE,output,ignore=shutil.ignore_patterns('.pio','__pycache__'))
    main=output/'firmware/main.cpp';main.write_text(transform_main(main.read_text()))
    header=output/'firmware/motion_protocol.h'
    header.write_text(replace_once(header.read_text(),
        'static_assert(kMotionEvidenceSize <= kMaxPayload, "Motion evidence fits one frame");',
        '''static_assert(kMotionEvidenceSize <= kMaxPayload, "Motion evidence fits one frame");
// Explicit v2: sample-bound TX snapshot. Omits raw loads; v1 is unchanged.
constexpr size_t kSampleBoundEvidenceSize = kVoltageStateSize + 52;
static_assert(kSampleBoundEvidenceSize <= kMaxPayload, "Sample-bound evidence fits one frame");'''))
    test=output/'tests/test_motion_evidence.cpp'
    test.write_text(replace_once(test.read_text(),'q.payload[0]=2;command(q);',
        'q.payload[0]=3;command(q);'))
    shutil.copyfile(Path(__file__).with_name('test_causal_feedback.cpp'),output/'tests/test_causal_feedback.cpp')
    run=output/'tests/run.sh'
    run.write_text(run.read_text()+'''\nc++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \\
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_causal_feedback.cpp" -o "$task_build/causal-feedback"
XGODUCK_CAUSAL_FIXTURE="$task_build/causal-v2.bin" "$task_build/causal-feedback"
''')
    changed=[];patch=[]
    for name in PINNED:
        before=(BASE/name).read_text();after=(output/name).read_text()
        if before != after:
            changed.append(name)
            patch.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
                fromfile='a/'+name,tofile='b/'+name))
    (output/'candidate.patch').write_text(''.join(patch))
    manifest=dict(kind='offline_sample_bound_feedback_candidate',hardware_accessed=False,
        compiled=False,flashed=False,base_app_sha256=BASE_APP,base_source_hashes=PINNED,
        changed_firmware_files=changed,
        candidate_source_hashes={name:sha(output/name) for name in PINNED},
        wire_v2_payload_bytes=238,servo_register_acceptance_proven=False)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(prepare(args.output),indent=2))
