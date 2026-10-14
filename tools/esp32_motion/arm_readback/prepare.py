"""Prepare an isolated diagnostic candidate; never flash or access UART."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
FIRMWARE_SHA = '4bc4caa9a773f02a90fae7a36abcf1db6a3fd411dbec0606aa7710874d85a20e'
HOST_SHA = 'b3af847416758660a675cdedd381dc629292de6194cb413d32207c62e055544f'


def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError('Source changed; review required: ' + before[:80])
    return source.replace(before, after, 1)


def prepare(firmware, host, output):
    main = (firmware/'firmware/main.cpp').read_bytes()
    production = host.read_bytes()
    if hashlib.sha256(main).hexdigest() != FIRMWARE_SHA or hashlib.sha256(production).hexdigest() != HOST_SHA:
        raise ValueError('Not the reviewed installed baseline')
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(firmware, output/'firmware', ignore=shutil.ignore_patterns('.pio', '__pycache__'))
    text = main.decode()
    text = replace_once(text, 'uint32_t elapsedUs = 0, errorsBefore = 0, errorsAfter = 0;',
        'uint32_t elapsedUs = 0, errorsBefore = 0, errorsAfter = 0;\n'
        '  uint32_t startedUs = 0, writtenUs = 0, firstRxUs = 0, lastRxUs = 0;\n'
        '  uint16_t pendingCount = 0;\n  bool requestWritten = false;')
    text = replace_once(text, '} lastServoReadTrace;', '''} lastServoReadTrace;

// Copy the first failed ARM torque read BEFORE STOP or later reads replace it.
// Diagnostic only: no extra bus request, retry, delay, or freshness renewal.
ServoReadTrace armFailedReadTrace;
bool armFailedReadSeen = false;
uint32_t armReadSequence = 0, armReadCookie = 0, armEnableCompletedUs = 0;
constexpr size_t kArmReadTraceAckSize = 134;
void retainArmFailedRead() {
  if (armFailedReadSeen) return;
  armFailedReadTrace = lastServoReadTrace; armFailedReadSeen = true;
}
void encodeArmReadTrace(uint8_t* data) {
  const ServoReadTrace& trace = armFailedReadTrace;
  data[22] = (trace.reply.invalidReplySeen ? 1 : 0) |
    (trace.drainedCount > 16 ? 2 : 0) | (trace.receivedCount > 48 ? 4 : 0) |
    (trace.requestWritten ? 8 : 0) | (trace.receivedCount ? 16 : 0);
  data[23] = trace.requestedSize;
  motion::put16(data + 24, trace.drainedCount); motion::put16(data + 26, trace.receivedCount);
  motion::put16(data + 28, trace.pendingCount);
  motion::put32(data + 30, trace.startedUs); motion::put32(data + 34, trace.writtenUs);
  motion::put32(data + 38, trace.firstRxUs); motion::put32(data + 42, trace.lastRxUs);
  motion::put32(data + 46, trace.elapsedUs); motion::put32(data + 50, trace.errorsBefore);
  motion::put32(data + 54, trace.errorsAfter); motion::put32(data + 58, armEnableCompletedUs);
  motion::put32(data + 62, armReadSequence); motion::put32(data + 66, armReadCookie);
  memcpy(data + 70, trace.drained, 16); memcpy(data + 86, trace.received, 48);
}''')
    text = replace_once(text, 'lastServoReadTrace.elapsedUs = uint32_t(micros() - start);',
        'lastServoReadTrace.elapsedUs = uint32_t(micros() - start);\n'
        '    const int pending = Serial1.available();\n'
        '    lastServoReadTrace.pendingCount = pending > 65535 ? 65535 : pending > 0 ? uint16_t(pending) : 0;')
    text = replace_once(text, 'lastServoReadTrace.id = id; lastServoReadTrace.address = address;',
        'lastServoReadTrace.startedUs = start;\n  lastServoReadTrace.id = id; lastServoReadTrace.address = address;')
    text = replace_once(text, '  uint8_t received[16] = {}; size_t used = 0;',
        '  lastServoReadTrace.requestWritten = true; lastServoReadTrace.writtenUs = micros();\n'
        '  uint8_t received[16] = {}; size_t used = 0;')
    text = replace_once(text, '  if (!sendServo(id, 3, data, size + 1) || !txComplete()) return false;',
        '  if (!sendServo(id, 3, data, size + 1) || !txComplete()) return false;\n'
        '  if (id == 254 && address == 40 && size == 1 && value[0] == 1) armEnableCompletedUs = micros();')
    text = replace_once(text, '    lastServoReadTrace.byte(uint8_t(value), false);',
        '    const uint32_t receivedAt = micros();\n'
        '    if (!lastServoReadTrace.receivedCount) lastServoReadTrace.firstRxUs = receivedAt;\n'
        '    lastServoReadTrace.lastRxUs = receivedAt;\n'
        '    lastServoReadTrace.byte(uint8_t(value), false);')
    text = replace_once(text, '      armDiagnostic.id = motion::kMotorIds[i]; armDiagnostic.address = 40;',
        '      retainArmFailedRead();\n      armDiagnostic.id = motion::kMotorIds[i]; armDiagnostic.address = 40;')
    text = replace_once(text, '    uint8_t data[22] = {frame.type, uint8_t(error), 1, armDiagnostic.stage,',
        '    static_assert(kArmReadTraceAckSize <= motion::kMaxPayload, "ARM evidence fits frame");\n'
        '    const bool extended = armFailedReadSeen && (armDiagnostic.stage == 4 || armDiagnostic.stage == 9);\n'
        '    uint8_t data[kArmReadTraceAckSize] = {frame.type, uint8_t(error), uint8_t(extended ? 2 : 1), armDiagnostic.stage,')
    text = replace_once(text, '    motion::put32(data + 18, uint32_t(now - feedback.imuUs));\n    respond(0x80, frame.sequence, data, sizeof(data)); return;',
        '    motion::put32(data + 18, uint32_t(now - feedback.imuUs));\n'
        '    if (extended) encodeArmReadTrace(data);\n'
        '    respond(0x80, frame.sequence, data, extended ? sizeof(data) : 22); return;')
    text = replace_once(text, '  if (frame.type == motion::Arm) armDiagnostic = ArmDiagnostic{};',
        '  if (frame.type == motion::Arm) {\n    armDiagnostic = ArmDiagnostic{}; armFailedReadSeen = false;\n'
        '    armFailedReadTrace = ServoReadTrace{}; armEnableCompletedUs = 0;\n'
        '    armReadSequence = frame.sequence; armReadCookie = frame.length == 4 ? motion::u32(frame.payload) : 0;\n  }')
    (output/'firmware/firmware/main.cpp').write_text(text)
    tests = output/'firmware/tests/test_firmware.cpp'
    t = tests.read_text().replace('response.length == 22 && response.payload[3] == 9',
                                'response.length == kArmReadTraceAckSize && response.payload[2] == 2 && response.payload[3] == 9')
    tests.write_text(t)
    h = production.decode()
    h = replace_once(h, 'import numpy as np',
        'import numpy as np\nfrom arm_read_trace import ARM_TRACE_SIZE, decode_arm_read_trace, arm_read_trace_text')
    h = replace_once(h, 'def arm_failure_detail(payload):\n', '''def arm_failure_detail(payload):
    suffix = ''
    if len(payload) == ARM_TRACE_SIZE:
        try:
            suffix = '；' + arm_read_trace_text(decode_arm_read_trace(payload))
        except ValueError as error:
            raise MotionError(str(error)) from error
        payload = payload[:2] + b'\\x01' + payload[3:22]
''')
    h = replace_once(h, "return f'{text}；关节反馈 {sample_age / 1000:.1f} ms，IMU {imu_age / 1000:.1f} ms'",
        "return f'{text}；关节反馈 {sample_age / 1000:.1f} ms，IMU {imu_age / 1000:.1f} ms' + suffix")
    h = replace_once(h, 'len(response) not in (2, 22, TARGET_STOP_PACKET.size)',
        'len(response) not in (2, 22, ARM_TRACE_SIZE, TARGET_STOP_PACKET.size)')
    h = replace_once(h, '(len(response) == 22 and (kind != ARM or not response[1]))',
        '(len(response) in (22, ARM_TRACE_SIZE) and (kind != ARM or not response[1]))')
    h = replace_once(h, "detail = '；' + arm_failure_detail(response) if len(response) == 22 else ''", '''if len(response) == ARM_TRACE_SIZE:
                                try:
                                    self.last_arm_read_trace = decode_arm_read_trace(response,
                                        sequence=self.seq, cookie=struct.unpack_from('<I', payload)[0])
                                except (ValueError, struct.error) as error:
                                    raise MotionError('ARM 原始记录校验失败') from error
                            detail = '；' + arm_failure_detail(response) if len(response) in (22, ARM_TRACE_SIZE) else '' '''.rstrip())
    (output/'host').mkdir()
    (output/'host/production_motion.py').write_text(h)
    shutil.copy2(HERE/'arm_read_trace.py', output/'host/arm_read_trace.py')
    manifest = dict(baseline_firmware_source_sha256=FIRMWARE_SHA, baseline_host_sha256=HOST_SHA,
        original_app_sha256='201f4e7a10d10c3cdbaa71ffd75a1aa70dd61dbf45e91230b382966427b56b55',
        diagnostic_only=True, deployed=False, arm_timeout_root_cause_proven=False,
        arm_retry_added=False, timeout_or_protection_changed=False,
        changed_files={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [output/'firmware/firmware/main.cpp',output/'host/production_motion.py',output/'host/arm_read_trace.py']})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--firmware',type=Path,required=True)
    p.add_argument('--host',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    print(json.dumps(prepare(a.firmware,a.host,a.output),indent=2))
