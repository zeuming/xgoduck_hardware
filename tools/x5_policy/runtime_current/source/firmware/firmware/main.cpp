#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <driver/uart.h>
#include <esp_arduino_version.h>
#include "motion_protocol.h"
#include "source_cycle.h"

#if ESP_ARDUINO_VERSION_MAJOR < 3 && !ARDUINO_USB_CDC_ON_BOOT
static HardwareSerial& host = Serial;
#else
static HardwareSerial& host = Serial0;
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "ESP32-S3 only"
#endif

namespace {
motion::Parser parser;
motion::SequenceGuard sequence;
motion::Interlock lock;
motion::Feedback feedback;
constexpr uint32_t kServoDeadlineUs = 1500; // Legacy maintenance transactions only; not source model polling.
// Upstream sketch/duck_config.h SERVO_REPLY_US is 6000 after TX flush.
// Normal model polling is the grouped source_cycle path. Individual reads
// remain only for explicit legacy maintenance/diagnostic and OFF proof.

constexpr uint32_t kTorqueFreshUs = 500000;
constexpr uint32_t kSamplePeriodUs = 10000;
constexpr uint32_t kTorquePeriodUs = 250000;
constexpr size_t kBackgroundTorqueBatch = 3;
constexpr uint32_t kImuRecoveryUs = 1000000;
constexpr uint32_t kImuRecoveryResetUs = 20000;
std::atomic<uint32_t> servoErrors{0};
uint16_t presentMask = 0, presentFaults = 0, torqueFaults = 0, torqueKnown = 0;
uint32_t torqueReadUs[15] = {};
uint32_t positionReadUs[15] = {}; uint16_t positionSeen = 0;
uint32_t servoSampleSequence = 0;
bool goalReadDetailsSeen = false;
uint32_t lastGoalReadDetailsUs = 0;
uint32_t lastTorqueRefreshUs = 0;
size_t nextBackgroundTorque = motion::kMotorCount;
bool sourceVoltageEnabled = false;
// Deferred read-only OFF inspection is distinct from required STOP verification.
bool sourceOffVerifyPending = false;
size_t nextVoltage = 0;
uint8_t voltageRaw[15] = {};
uint32_t voltageReadUs[15] = {};
uint16_t voltageSeen = 0, voltageValid = 0;
int16_t velocities[15] = {};
uint16_t presentLoadRaw[15] = {}, lastWrittenGoals[15] = {};
uint32_t lastWrittenGoalsUs = 0;
// Diagnostic-only sequence/epoch counters. They never renew a safety clock.
uint32_t writtenGoalsSequence = 0, causalArmEpoch = 0;
bool writtenGoalsSeen = false;
struct SampleBoundGoals {
  bool valid = false;
  uint32_t sampleSequence = 0, sampleStartUs = 0, armCookie = 0, armEpoch = 0;
  uint32_t goalWriteUs = 0, goalWriteSequence = 0;
  uint16_t goals[15] = {};
} sampleBoundGoals;
float accel[3] = {}, gyro[3] = {};
bool disablePending = false, torqueVerifyPending = false, verifyingTorque = false;
bool maintenanceMode = false;
bool preparingArm = false;
struct ArmDiagnostic {
  uint8_t stage = 0, id = 0, address = 0, found = 0, error = 0, size = 0, outcome = 0;
  uint8_t expectedKp = 0, actualKp = 0, expectedKd = 0, actualKd = 0;
} armDiagnostic;
struct SourceArmFailure {
 bool seen=false; uint16_t online=0,fault=0,torque=0; bool imuOk=false;
 uint32_t positionAge=0,imuAge=0;
} sourceArmFailure;
bool i2cReady = false, imuConfigured = false, imuSettling = false;
uint8_t imuAddress = 0;
uint32_t imuSettleStartUs = 0, lastImuRecoveryUs = 0, lastDisableUs = 0, lastHostByteUs = 0;

// Preserve the first autonomous stop of this ARM session. A later TARGET
// refusal or STOP must not replace the event that actually removed torque.
// This records existing checks only: no additional UART read or grace period.
enum StopCause : uint8_t {
  DeviceFaultStop = 1, OfflineStop = 2, ImuInvalidStop = 3,
  PositionStaleStop = 4, ImuStaleStop = 5, TargetWatchdogStop = 6,
  TorqueLostStop = 7, GoalWriteStop = 8, InternalStop = 9
};
struct SafetyDiagnostic {
  bool seen = false;
  uint8_t cause = 0, error = 0, imuOk = 0;
  uint16_t online = 0, fault = 0, torque = 0;
  uint32_t cookie = 0, atUs = 0, positionAge = 0, imuAge = 0, targetAge = 0, uartErrors = 0;
  uint8_t readId = 255, address = 0, outcome = 0, deviceError = 0;
  uint32_t readElapsed = 0;
} safetyDiagnostic;
void captureSafetyStop(StopCause cause, motion::Error error, uint32_t now, uint32_t cookie);

void uartError(hardwareSerial_error_t) { ++servoErrors; }

void updateMasks(uint32_t now) {
  uint16_t freshTorque = torqueKnown;
  for (size_t i = 0; i < 15; ++i) {
    const uint16_t bit = uint16_t(1u << i);
    if (uint32_t(now - torqueReadUs[i]) > kTorqueFreshUs) {
      freshTorque &= ~bit; feedback.torqueMask &= ~bit;
    }
  }
  feedback.onlineMask = lock.sourceRuntime ? presentMask : presentMask & freshTorque;
  feedback.faultMask = presentFaults | torqueFaults;
}

void requestDisarm(motion::Error error, StopCause cause = InternalStop) {
  if (lock.armed && error != motion::Ok)
    captureSafetyStop(cause, error, micros(), lock.cookie);
  lock.disarm(error);
  disablePending = true; torqueVerifyPending = true;
}

void checkSafety() {
  // Maintenance has its own selected-axis feedback/watchdog. Motion masks
  // remain invalid throughout that mode and may not trigger background UART.
  if (maintenanceMode) return;
  const uint32_t now = micros();
  updateMasks(now);
  const uint32_t cookie = lock.cookie;
  if (lock.check(feedback, now)) {
    const StopCause cause = (!lock.sourceRuntime && feedback.faultMask) ? DeviceFaultStop :
      (!lock.sourceRuntime && feedback.onlineMask != motion::kAllMotors) ? OfflineStop :
      !feedback.imuOk ? ImuInvalidStop :
      (!lock.sourceRuntime && uint32_t(now - feedback.sampleUs) > motion::kStateFreshUs) ? PositionStaleStop :
      uint32_t(now - feedback.imuUs) > motion::kStateFreshUs ? ImuStaleStop : TargetWatchdogStop;
    captureSafetyStop(cause, lock.lastError, now, cookie);
    disablePending = true; torqueVerifyPending = true;
  }
  if (!lock.sourceRuntime && lock.armed && !verifyingTorque && feedback.torqueMask != motion::kAllMotors) {
    captureSafetyStop(TorqueLostStop, motion::BadState, now, lock.cookie);
    requestDisarm(motion::BadState);
  }
}

bool sendServo(uint8_t id, uint8_t instruction, const uint8_t* data, uint8_t count) {
  // Only internal fixed-register commands call this; no arbitrary bus packet API.
  uint8_t packet[128];
  if (size_t(count) + 6 > sizeof(packet)) return false;
  packet[0] = packet[1] = 0xff; packet[2] = id; packet[3] = count + 2; packet[4] = instruction;
  uint8_t sum = packet[2] + packet[3] + packet[4];
  for (uint8_t i = 0; i < count; ++i) { packet[i + 5] = data[i]; sum += data[i]; }
  packet[count + 5] = uint8_t(~sum);
  return Serial1.write(packet, count + 6) == size_t(count + 6);
}

bool txComplete() { return uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(5)) == ESP_OK; }
bool writeServoRegister(uint8_t id, uint8_t address, const uint8_t* value,
                        uint8_t size, uint8_t* deviceError = nullptr);
bool writeTorque(bool enabled) {
  // STOP/boot must always attempt the original TX-only disable even when an
  // unpowered bus has RX noise. Its success never substitutes for readback.
  if (!enabled || lock.sourceRuntime) {
    const uint8_t data[] = {40, uint8_t(enabled)};
    return sendServo(0xfe, 0x03, data, sizeof(data)) && txComplete();
  }
  // Own the enable broadcast's optional echo for the same bounded window as
  // other register writes. TX completion alone allows the following torque
  // READ to collide with a delayed echo. Silence is valid; readback is still
  // mandatory, and neither the broadcast nor ARM is retried.
  const uint8_t value = 1;
  return writeServoRegister(0xfe, 40, &value, 1);
}
bool writeGoals(const uint16_t* goals) {
  // SCS synchronous WRITE, goal-position register 42, six bytes per motor.
  // Upstream position goal, zero duration and speed; legacy pose slew
  // or explicit policy-position execution is selected by Interlock.
  uint8_t data[107]; data[0] = 42; data[1] = 6;
  for (size_t i = 0; i < 15; ++i) {
    const size_t offset = 2 + i * 7;
    data[offset] = motion::kMotorIds[i];
    motion::put16(data + offset + 1, goals[i]);
    motion::put16(data + offset + 3, 0); motion::put16(data + offset + 5, 0);
  }
  const bool sent = sendServo(0xfe, 0x83, data, sizeof(data)) && txComplete();
  if (sent) {
    memcpy(lastWrittenGoals, goals, sizeof(lastWrittenGoals));
    lastWrittenGoalsUs = micros();
    ++writtenGoalsSequence; writtenGoalsSeen = true;
  }
  return sent;
}

void flushDisable() {
  if (!disablePending) return;
  disablePending = false;
  if (!writeTorque(false)) { disablePending = true; lock.record(motion::BadState); }
  lastDisableUs = micros();
  // Do not claim that a broadcast write disabled torque. Retain last observed
  // mask until readback; its timestamps and validity are managed separately.
}

struct ServoReply {
  bool found = false;
  // Maintenance must distinguish silence from corrupt/wrong-ID traffic when
  // proving that a new ID is unoccupied. Existing motion diagnostics keep
  // their established outcome codes and masks.
  bool invalidReplySeen = false;
  uint8_t error = 0, size = 0, data[6] = {};
  motion::ReadOutcome outcome = motion::NotAttempted;
};

// Bounded evidence for a failed maintenance read. It never changes the read
// deadline, accepts a rejected reply, or issues another bus transaction.
struct ServoReadTrace {
  uint8_t id = 255, address = 0, requestedSize = 0;
  uint8_t drained[16] = {}, received[48] = {};
  uint16_t drainedCount = 0, receivedCount = 0;
  uint32_t elapsedUs = 0, errorsBefore = 0, errorsAfter = 0;
  uint32_t startedUs = 0, writtenUs = 0, firstRxUs = 0, lastRxUs = 0, transmittedUs = 0;
  uint16_t pendingCount = 0;
  bool requestWritten = false;
  ServoReply reply;
  void byte(uint8_t value, bool drain) {
    uint16_t& count = drain ? drainedCount : receivedCount;
    if (drain && count < sizeof(drained)) drained[count] = value;
    if (!drain && count < sizeof(received)) received[count] = value;
    if (count != 0xffff) ++count;
  }
} lastServoReadTrace;

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
void encodeArmReadTrace(uint8_t* data, const ServoReadTrace& trace = armFailedReadTrace) {
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
}

void captureSafetyStop(StopCause cause, motion::Error error, uint32_t now, uint32_t cookie) {
  if (safetyDiagnostic.seen) return;
  safetyDiagnostic.seen = true;
  safetyDiagnostic.cause = uint8_t(cause); safetyDiagnostic.error = uint8_t(error);
  safetyDiagnostic.imuOk = feedback.imuOk;
  safetyDiagnostic.online = feedback.onlineMask; safetyDiagnostic.fault = feedback.faultMask;
  safetyDiagnostic.torque = feedback.torqueMask; safetyDiagnostic.cookie = cookie;
  safetyDiagnostic.atUs = now; safetyDiagnostic.positionAge = uint32_t(now - feedback.sampleUs);
  safetyDiagnostic.imuAge = uint32_t(now - feedback.imuUs);
  safetyDiagnostic.targetAge = uint32_t(now - lock.lastCommandUs);
  safetyDiagnostic.uartErrors = servoErrors.load();
  safetyDiagnostic.readId = lastServoReadTrace.id; safetyDiagnostic.address = lastServoReadTrace.address;
  safetyDiagnostic.outcome = uint8_t(lastServoReadTrace.reply.outcome);
  safetyDiagnostic.deviceError = lastServoReadTrace.reply.error;
  safetyDiagnostic.readElapsed = lastServoReadTrace.elapsedUs;
}

// Passive ARM stage9 transaction history; fixed RAM, no new servo traffic.
struct ArmBusEntry { ServoReadTrace trace; uint32_t disabledUs = 0; bool armed = false; };
ArmBusEntry armBusEntries[32];
uint8_t armBusCount = 0;
uint8_t nextArmFirstIndex = 0, activeArmFirstIndex = 0;
struct FinishServoReadTrace {
  ServoReply& reply;
  uint32_t start;
  FinishServoReadTrace(ServoReply& value, uint32_t started) : reply(value), start(started) {}
  ~FinishServoReadTrace() {
    lastServoReadTrace.reply = reply;
    lastServoReadTrace.elapsedUs = uint32_t(micros() - start);
    const int pending = Serial1.available();
    lastServoReadTrace.pendingCount = pending > 65535 ? 65535 : pending > 0 ? uint16_t(pending) : 0;
    lastServoReadTrace.errorsAfter = servoErrors.load();
    if (preparingArm && armDiagnostic.stage == 9 && armBusCount < 32) {
      ArmBusEntry& entry = armBusEntries[armBusCount++];
      entry.trace = lastServoReadTrace; entry.disabledUs = lastDisableUs; entry.armed = lock.armed;
    }
  }
};

bool writeServoRegister(uint8_t id, uint8_t address, const uint8_t* value,
                        uint8_t size, uint8_t* deviceError) {
  // Fixed internal writes only. Own the optional echo/status reply before
  // another request uses the half-duplex bus. Silence is allowed for servos
  // configured not to ACK writes; every caller still requires readback.
  if (deviceError) *deviceError = 0;
  if (id > 254 || size > 6) return false;
  uint8_t data[7]; data[0] = address; memcpy(data + 1, value, size);
  const uint32_t beforeErrors = servoErrors.load();
  if (!sendServo(id, 3, data, size + 1) || !txComplete()) return false;
  if (id == 254 && address == 40 && size == 1 && value[0] == 1) armEnableCompletedUs = micros();
  uint8_t echo[13] = {255,255,id,uint8_t(size + 3),3,address};
  memcpy(echo + 6, value, size);
  uint8_t sum = 0; for (size_t i = 2; i < size_t(size) + 6; ++i) sum += echo[i];
  echo[size + 6] = uint8_t(~sum);
  uint8_t received[13] = {}; size_t used = 0;
  bool echoSeen = false, ackSeen = false;
  const uint32_t start = micros();
  while (uint32_t(micros() - start) < kServoDeadlineUs) {
    if (servoErrors.load() != beforeErrors) return false;
    if (!Serial1.available()) { delayMicroseconds(10); continue; }
    const int byte = Serial1.read();
    if (byte < 0) continue;
    if (used == sizeof(received)) return false;
    received[used++] = uint8_t(byte);
    if ((used >= 1 && received[0] != 255) || (used >= 2 && received[1] != 255)) return false;
    if (used < 4) continue;
    const size_t total = size_t(received[3]) + 4;
    if (total != 6 && total != size_t(size) + 7) return false;
    if (used < total) continue;
    uint8_t checksum = 0; for (size_t i = 2; i < total; ++i) checksum += received[i];
    if (checksum != 255) return false;
    if (total == size_t(size) + 7 && memcmp(received, echo, total) == 0 && !echoSeen) echoSeen = true;
    else if (id != 254 && total == 6 && !ackSeen && (received[2] == id ||
             (address == 5 && size == 1 && received[2] == value[0]))) {
      ackSeen = true;
      if (received[4]) {
        if (deviceError) *deviceError = received[4];
        return false;
      }
    } else return false;
    used = 0;
  }
  return !used && servoErrors.load() == beforeErrors;
}

// Diagnostic-only history of the existing fixed-register background reads.
// A missed reply clears only the latest-attempt fields; it cannot erase the
// last checksum/ID-valid status byte or change the existing safety masks.
struct ServoReadHistory {
  uint16_t attemptedMask = 0, latestFoundMask = 0, lastValidSeenMask = 0;
  uint8_t latestError[15] = {}, lastValidError[15] = {};
  uint8_t latestOutcome[15] = {};
  uint32_t lastValidUs[15] = {};
  void record(size_t index, const ServoReply& reply, bool registerValid) {
    const uint16_t bit = uint16_t(1u << index);
    attemptedMask |= bit;
    latestError[index] = reply.found ? reply.error : 0;
    latestOutcome[index] = uint8_t(reply.outcome);
    if (reply.outcome == motion::ValidNormalReply && !registerValid)
      latestOutcome[index] = uint8_t(motion::InvalidRegisterReply);
    if (!reply.found) { latestFoundMask &= ~bit; return; }
    latestFoundMask |= bit; lastValidSeenMask |= bit;
    lastValidError[index] = reply.error; lastValidUs[index] = micros();
  }
  void encode(uint8_t* data) const {
    motion::put16(data, attemptedMask); motion::put16(data + 2, latestFoundMask);
    motion::put16(data + 4, lastValidSeenMask);
    memcpy(data + 6, latestError, sizeof(latestError));
    memcpy(data + 21, lastValidError, sizeof(lastValidError));
    memcpy(data + 36, latestOutcome, sizeof(latestOutcome));
    for (size_t i = 0; i < 15; ++i) motion::put32(data + 51 + i * 4, lastValidUs[i]);
  }
};
ServoReadHistory presentHistory, torqueHistory;

// Retain the latest existing fixed-map position attempt, including bytes that
// the motion validity checks reject. Reading this cache never touches UART or
// promotes rejected position data into Feedback.
struct PositionReadEvidence {
  bool attempted = false, registerRejected = false;
  uint32_t attemptUs = 0;
  ServoReply reply;
  void record(const ServoReply& value, bool registerValid, uint32_t startedUs) {
    attempted = true; attemptUs = startedUs; reply = value;
    registerRejected = value.outcome == motion::ValidNormalReply && !registerValid;
  }
} positionReadEvidence[motion::kMotorCount];

// Candidate: one extra position READ per complete cycle, only after pure
// silence while already armed. All old freshness/watchdog limits still run
// inside readServo(). Never retry ARM, writes, torque reads, or hard errors.
struct PositionRetryDiagnostic {
  uint8_t id = 255, firstOutcome = 0, retryOutcome = 0;
  uint32_t attempts = 0, recovered = 0, failed = 0;
  uint32_t atUs = 0, firstElapsedUs = 0, retryElapsedUs = 0;
} positionRetryDiagnostic;
void incrementRetryCounter(uint32_t& value) { if (value != UINT32_MAX) ++value; }
bool canRetryPosition(const ServoReply& reply, bool used, uint32_t now) {
  return !used && lock.armed && !maintenanceMode && !preparingArm &&
    !verifyingTorque && !disablePending && !torqueVerifyPending &&
    !reply.found && !reply.invalidReplySeen && !reply.error &&
    reply.outcome == motion::NoValidReplyByDeadline &&
    lastServoReadTrace.receivedCount == 0 && lastServoReadTrace.drainedCount == 0 &&
    lastServoReadTrace.errorsBefore == lastServoReadTrace.errorsAfter &&
    feedback.fresh(now) && feedback.torqueMask == motion::kAllMotors &&
    uint32_t(now - feedback.sampleUs) <= motion::kStateFreshUs - kServoDeadlineUs &&
    uint32_t(now - feedback.imuUs) <= motion::kStateFreshUs - kServoDeadlineUs &&
    uint32_t(now - lock.lastCommandUs) <= motion::kWatchdogUs - kServoDeadlineUs;
}

ServoReply readServo(uint8_t id, uint8_t address, uint8_t size, bool sourceDiagnosticRead = false) {
  ServoReply reply;
  const uint32_t start = micros();
  const uint32_t beforeErrors = servoErrors.load();
  lastServoReadTrace = ServoReadTrace{};
  lastServoReadTrace.startedUs = start;
  lastServoReadTrace.id = id; lastServoReadTrace.address = address;
  lastServoReadTrace.requestedSize = size; lastServoReadTrace.errorsBefore = beforeErrors;
  FinishServoReadTrace finishTrace(reply, start);
  if (sourceDiagnosticRead) {
    // Explicit SYNC diagnostic only. Match the source polling transaction:
    // drain the bytes already queued, send once, wait for TX completion, then
    // use upstream SERVO_REPLY_US. The previous 200us/30us precondition could
    // reject before READ based on CPU buffer-drain cost, not wire activity.
    // Snapshot bounds work even under continuous incoming noise. The normal
    // strict ID/length/checksum/device-error/UART-error checks below remain.
    int buffered = Serial1.available();
    while (buffered-- > 0) {
      checkSafety();
      const int value = Serial1.read();
      if (value >= 0) lastServoReadTrace.byte(uint8_t(value), true);
    }
  } else {
    // Bound noise draining and require a quiet bus before a new request.
    uint32_t quietSince = start;
    while (uint32_t(micros() - start) < 200) {
      checkSafety();
      if (Serial1.available()) {
        const int drained = Serial1.read();
        if (drained >= 0) lastServoReadTrace.byte(uint8_t(drained), true);
        quietSince = micros();
      }
      else if (uint32_t(micros() - quietSince) >= 30) break;
      else delayMicroseconds(5);
    }
    if (Serial1.available() || uint32_t(micros() - quietSince) < 30) {
      reply.outcome = motion::BusNotQuiet; return reply;
    }
  }
  uint8_t request[8] = {0xff, 0xff, id, 4, 2, address, size, 0};
  uint8_t sum = 0;
  for (size_t i = 2; i < 7; ++i) sum += request[i];
  request[7] = uint8_t(~sum);
  if (Serial1.write(request, sizeof(request)) != sizeof(request)) {
    reply.outcome = motion::ShortUartWrite; return reply;
  }
  lastServoReadTrace.requestWritten = true; lastServoReadTrace.writtenUs = micros();
  const bool armTorqueRead = preparingArm && (armDiagnostic.stage == 4 || armDiagnostic.stage == 9) && ((address == 40 && size == 1) || (armDiagnostic.stage == 9 && activeArmFirstIndex == motion::kMotorCount && address == 56 && size == 6));
  const bool sourceOffRead = lock.sourceRuntime && !maintenanceMode;
  uint32_t replyStart = start;
  if (armTorqueRead || sourceOffRead || sourceDiagnosticRead) {
    // Enqueue is not wire completion. Own one request and validate its reply;
    // never resend enable/ARM/read and never renew stale sample timestamps.
    if (!txComplete()) { reply.outcome = motion::NoValidReplyByDeadline; return reply; }
    replyStart = micros(); lastServoReadTrace.transmittedUs = replyStart;
  }
  const uint32_t replyBudget = (armTorqueRead || sourceOffRead || sourceDiagnosticRead) ? source_cycle::replyUs : kServoDeadlineUs;
  uint8_t received[16] = {}; size_t used = 0;
  while (uint32_t(micros() - replyStart) < replyBudget) {
    checkSafety();
    if (servoErrors.load() != beforeErrors) { reply.outcome = motion::HardwareUartError; return reply; }
    if (!Serial1.available()) { delayMicroseconds(10); continue; }
    const int value = Serial1.read();
    if (value < 0) continue;
    const uint32_t receivedAt = micros();
    if (!lastServoReadTrace.receivedCount) lastServoReadTrace.firstRxUs = receivedAt;
    lastServoReadTrace.lastRxUs = receivedAt;
    lastServoReadTrace.byte(uint8_t(value), false);
    if (used == sizeof(received)) { --used; memmove(received, received + 1, used); }
    received[used++] = uint8_t(value);
    while (used >= 2) {
      if (received[0] != 0xff || received[1] != 0xff) {
        reply.invalidReplySeen = true;
        --used; memmove(received, received + 1, used); continue;
      }
      if (used < 4) break;
      const uint8_t length = received[3];
      if (length != size + 2 && length != 2 && length != 4) {
        reply.invalidReplySeen = true;
        --used; memmove(received, received + 1, used); continue;
      }
      const size_t total = length + 4;
      if (used < total) break;
      uint8_t checksum = 0;
      for (size_t j = 2; j < total; ++j) checksum += received[j];
      if (checksum != 0xff) { reply.invalidReplySeen = true; --used; memmove(received, received + 1, used); continue; }
      const bool echo = total == 8 && memcmp(received, request, 8) == 0;
      if (!echo && received[2] == id &&
          (length == size + 2 || (length == 2 && received[4] != 0))) {
        reply.found = true; reply.error = received[4]; reply.size = length - 2;
        reply.outcome = reply.error ? motion::ValidDeviceErrorReply : motion::ValidNormalReply;
        if (reply.size) memcpy(reply.data, received + 5, reply.size);
        return reply;
      }
      if (!echo) reply.invalidReplySeen = true;
      used -= total; memmove(received, received + total, used);
    }
  }
  if (used) reply.invalidReplySeen = true;
  reply.outcome = motion::NoValidReplyByDeadline; return reply;
}

bool readI2c(uint8_t address, uint8_t reg, uint8_t* data, size_t count) {
  if (!i2cReady) return false;
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, count, true) != count) return false;
  for (size_t i = 0; i < count; ++i) {
    if (!Wire.available()) return false;
    data[i] = uint8_t(Wire.read());
  }
  return true;
}
bool writeI2c(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}
bool initImu(uint32_t resetTimeoutUs = 100000) {
  feedback.imuOk = false; imuConfigured = false; imuSettling = false; imuAddress = 0;
  uint8_t value = 0;
  for (uint8_t address = 0x6a; address <= 0x6b; ++address) {
    if (readI2c(address, 0x0f, &value, 1) && value == 0x6b) { imuAddress = address; break; }
  }
  if (!imuAddress || !writeI2c(0x12, 0x01)) return false;
  const uint32_t resetStart = micros();
  do {
    if (!readI2c(imuAddress, 0x12, &value, 1)) return false;
    if (!(value & 1)) break;
    delay(1);
  } while (uint32_t(micros() - resetStart) < resetTimeoutUs);
  if (value & 1) return false;
  // ST DS11976 Rev 2: 104 Hz, +/-4g (0.122 mg/LSB), +/-500dps
  // (17.50 mdps/LSB), BDU + IF_INC, data-ready settling mask.
  if (!writeI2c(0x12, 0x44) || !writeI2c(0x10, 0x48) ||
      !writeI2c(0x11, 0x44) || !writeI2c(0x13, 0x08)) return false;
  uint8_t registers[4] = {};
  if (!readI2c(imuAddress, 0x10, registers, 4) || registers[0] != 0x48 ||
      registers[1] != 0x44 || registers[2] != 0x44 || registers[3] != 0x08) return false;
  imuConfigured = true; imuSettleStartUs = micros(); imuSettling = true;
  return true;
}

void maybeRecoverImu(uint32_t now) {
  // A sensor power interruption can clear its ODR registers while the host
  // still considers it configured. A successful old init is not proof that
  // sampling is still running. Recover only after a sustained sample loss.
  if (imuConfigured && (imuSettling || uint32_t(now - feedback.imuUs) <= kImuRecoveryUs)) return;
  if (uint32_t(now - lastImuRecoveryUs) < kImuRecoveryUs) return;
  // This runs only at the loop boundary, never inside ARM, maintenance or a
  // torque read. Unknown/expired torque bits may look zero: require all real
  // readbacks, with a fresh completed servo cycle, before rebuilding I2C.
  if (lock.armed || lock.cookie || maintenanceMode || preparingArm || verifyingTorque ||
      disablePending || torqueVerifyPending) return;
  updateMasks(now);
  if (torqueKnown != motion::kAllMotors || presentMask != motion::kAllMotors ||
      feedback.onlineMask != motion::kAllMotors || feedback.faultMask || feedback.torqueMask ||
      uint32_t(now - feedback.sampleUs) > motion::kStateFreshUs) return;
  for (size_t i = 0; i < motion::kMotorCount; ++i)
    if (uint32_t(now - torqueReadUs[i]) > motion::kStateFreshUs) return;

  feedback.imuOk = false; imuConfigured = false; imuSettling = false; i2cReady = false;
  lastImuRecoveryUs = now;
  // end() can fail; begin() must not mistake an old surviving driver for a
  // new bus. Keep invalid feedback on any failure and throttle later retries.
  if (Wire.end()) {
    Wire.setTimeOut(2); i2cReady = Wire.begin(8, 9, 400000);
    // Runtime recovery must leave time for host STATE/STOP handling even if
    // a responsive sensor never clears its reset bit. Boot keeps its budget.
    if (i2cReady) initImu(kImuRecoveryResetUs);
  }
  lastImuRecoveryUs = micros();
  // Preserve configuration and the fault latch. New data never rearms motors.
}

void sampleImu() {
  if (!imuConfigured) return;
  if (imuSettling) {
    if (uint32_t(micros() - imuSettleStartUs) < 100000) return;
    // This startup delay ends once. Comparing an old deadline forever would
    // reject sampling again after half of the 32-bit micros() range.
    imuSettling = false;
  }
  uint8_t status = 0;
  if (!readI2c(imuAddress, 0x1e, &status, 1)) { feedback.imuOk = false; checkSafety(); return; }
  if ((status & 3) != 3) return;  // Require both gyro and accelerometer new data.
  uint8_t raw[12] = {};
  if (!readI2c(imuAddress, 0x22, raw, sizeof(raw))) { feedback.imuOk = false; checkSafety(); return; }
  for (size_t i = 0; i < 3; ++i) {
    gyro[i] = float(int16_t(motion::u16(raw + i * 2))) * 0.0175f;
    accel[i] = float(int16_t(motion::u16(raw + 6 + i * 2))) * (0.000122f * 9.80665f);
  }
  feedback.imuUs = micros(); feedback.imuOk = true;
}

// Official runtime transient gains (duck_config.h).
uint8_t runKpForId(uint8_t id) {
  return id == 34 ? 10 : 6;
}

motion::Error prepareRunGains() {
  // Match the official runtime's setRunKpKd(): transient STS RAM registers
  // 50/51, not permanent register 21 or unlock/EPROM writes. This runs only
  // for an explicit valid ARM while confirmed torque is disabled.
  const uint8_t kp = 6, kd = 20, mouthKp = 10;
  armDiagnostic.stage = 2;
  if (!writeServoRegister(0xfe, 50, &kp, 1) ||
      !writeServoRegister(0xfe, 51, &kd, 1)) return motion::BadState;
  // Explicit per-joint write/readback uses the same official gain.
  const uint8_t rightKneeKp = runKpForId(23);
  armDiagnostic.id = 23; armDiagnostic.address = 50;
  if (!writeServoRegister(23, 50, &rightKneeKp, 1, &armDiagnostic.error)) return motion::BadState;
  // Temporary RAM only, torque off; every ID must read back before enable.
  for (uint8_t sid = 30; sid <= 33; ++sid) {
    const uint8_t headKp = runKpForId(sid);
    armDiagnostic.id = sid; armDiagnostic.address = 50;
    if (!writeServoRegister(sid, 50, &headKp, 1, &armDiagnostic.error)) return motion::BadState;
  }
  armDiagnostic.id = 34; armDiagnostic.address = 50;
  if (!writeServoRegister(34, 50, &mouthKp, 1, &armDiagnostic.error)) return motion::BadState;
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    armDiagnostic.stage = 3; armDiagnostic.id = motion::kMotorIds[i]; armDiagnostic.address = 50;
    const ServoReply reply = readServo(motion::kMotorIds[i], 50, 2);
    armDiagnostic.found = reply.found; armDiagnostic.error = reply.error;
    armDiagnostic.size = reply.size; armDiagnostic.outcome = uint8_t(reply.outcome);
    armDiagnostic.expectedKp = runKpForId(motion::kMotorIds[i]); armDiagnostic.expectedKd = 20;
    armDiagnostic.actualKp = reply.size > 0 ? reply.data[0] : 0;
    armDiagnostic.actualKd = reply.size > 1 ? reply.data[1] : 0;
    sampleImu(); checkSafety(); flushDisable();
    if (!reply.found) {
      presentMask &= ~uint16_t(1u << i); updateMasks(micros());
      return motion::Stale;
    }
    if (reply.error) {
      presentFaults |= uint16_t(1u << i); updateMasks(micros());
      return motion::BadState;
    }
    const uint8_t expectedKp = runKpForId(motion::kMotorIds[i]);
    if (reply.size != 2 || reply.data[0] != expectedKp || reply.data[1] != 20) return motion::BadState;
  }
  updateMasks(micros());
  // Gains do not refresh the cached position sample. The ARM handler must
  // obtain and validate a complete new snapshot before writing goals/torque.
  return feedback.torqueMask == 0 ? motion::Ok : motion::BadState;
}

void sampleServos();
void readTorqueRange(size_t first, size_t end, bool sourceDiagnosticRead = false) {
  for (size_t slot = first; slot < end; ++slot) {
    const size_t i = preparingArm && armDiagnostic.stage == 9 ? (slot + (activeArmFirstIndex % motion::kMotorCount)) % motion::kMotorCount : slot;
    const uint16_t bit = uint16_t(1u << i);
    const ServoReply reply = readServo(motion::kMotorIds[i], 40, 1, sourceDiagnosticRead);
    const bool valid = reply.found && reply.error == 0 && reply.size == 1 && reply.data[0] <= 1;
    torqueHistory.record(i, reply, valid);
    if (preparingArm && (armDiagnostic.stage == 4 || armDiagnostic.stage == 9) && !armDiagnostic.id &&
        (!valid || reply.data[0] != (armDiagnostic.stage == 9 ? 1 : 0))) {
      // Preserve the first failed read, before safety STOP changes subsequent
      // readbacks. The version-1 byte pair also carries expected/actual for
      // register 40; gain diagnostics retain their existing interpretation.
      retainArmFailedRead();
      armDiagnostic.id = motion::kMotorIds[i]; armDiagnostic.address = 40;
      armDiagnostic.found = reply.found; armDiagnostic.error = reply.error;
      armDiagnostic.size = reply.size;
      armDiagnostic.outcome = uint8_t(reply.outcome == motion::ValidNormalReply && !valid ?
                                      motion::InvalidRegisterReply : reply.outcome);
      armDiagnostic.expectedKp = armDiagnostic.stage == 9 ? 1 : 0;
      armDiagnostic.actualKp = reply.size ? reply.data[0] : 0;
    }
    if (valid) {
      torqueKnown |= bit; torqueReadUs[i] = micros(); torqueFaults &= ~bit;
      if (reply.data[0] == 1) feedback.torqueMask |= bit; else feedback.torqueMask &= ~bit;
    } else {
      torqueKnown &= ~bit; feedback.torqueMask &= ~bit;
      if (reply.found && reply.error) torqueFaults |= bit;
    }
    sampleImu(); checkSafety(); flushDisable();
  }
}

void finishTorqueVerification() {
  verifyingTorque = false; updateMasks(micros());
  if (lock.armed && (torqueKnown != motion::kAllMotors || feedback.torqueMask != motion::kAllMotors))
    requestDisarm(torqueKnown != motion::kAllMotors ? motion::Stale : motion::BadState,
                  torqueKnown != motion::kAllMotors ? OfflineStop : TorqueLostStop);
  flushDisable();
}

void refreshTorque() {
  // ARM/STOP still require one complete synchronous readback. A completed
  // explicit verification also postpones the next periodic background scan.
  verifyingTorque = true;
  const bool positionFirst = preparingArm && armDiagnostic.stage == 9 && activeArmFirstIndex == motion::kMotorCount;
  if (positionFirst && lock.armed) sampleServos();
  readTorqueRange(0, 8);
  // At ARM, torque is enabled but the post-enable readback is not complete.
  // Refresh the entire position array halfway through that bounded check so
  // its last completed timestamp cannot expire during the next full cycle.
  // preparingArm continues to prevent recursive host command handling.
  if (lock.armed && !positionFirst) sampleServos();
  readTorqueRange(8, motion::kMotorCount);
  finishTorqueVerification();
  lastTorqueRefreshUs = micros();
  nextBackgroundTorque = motion::kMotorCount;
  torqueVerifyPending = false;
}

void respond(uint8_t type, uint32_t seq, const uint8_t* data, size_t size) {
  uint8_t out[motion::kHeaderSize + motion::kMaxPayload + 2];
  const size_t count = motion::encode(type, seq, data, size, out);
  host.write(out, count);
}
void ack(const motion::Frame& frame, motion::Error error) {
  if (frame.type == motion::Target && error == motion::BadState &&
      (frame.length == 34 || (frame.length == 35 && frame.payload[34] == motion::kPolicyPositionProfile)) &&
      !lock.armed && safetyDiagnostic.seen &&
      motion::u32(frame.payload) == safetyDiagnostic.cookie) {
    // Optional version-1 TARGET failure detail, bound to the stopped cookie.
    // Legacy successful ACKs and unrelated/malformed refusals remain unchanged.
    uint8_t data[40] = {frame.type, uint8_t(error), 1, safetyDiagnostic.cause,
      safetyDiagnostic.error, safetyDiagnostic.imuOk};
    motion::put16(data + 6, safetyDiagnostic.online); motion::put16(data + 8, safetyDiagnostic.fault);
    motion::put16(data + 10, safetyDiagnostic.torque); motion::put32(data + 12, safetyDiagnostic.atUs);
    motion::put32(data + 16, safetyDiagnostic.positionAge); motion::put32(data + 20, safetyDiagnostic.imuAge);
    motion::put32(data + 24, safetyDiagnostic.targetAge); motion::put32(data + 28, safetyDiagnostic.uartErrors);
    data[32] = safetyDiagnostic.readId; data[33] = safetyDiagnostic.address;
    data[34] = safetyDiagnostic.outcome; data[35] = safetyDiagnostic.deviceError;
    motion::put32(data + 36, safetyDiagnostic.readElapsed);
    respond(0x80, frame.sequence, data, sizeof(data)); return;
  }
  if (frame.type == motion::Arm && error != motion::Ok) {
    if(sourceArmFailure.seen) {
      uint8_t data[30]={frame.type,uint8_t(error),3,armDiagnostic.stage};
      motion::put32(data+14,sourceArmFailure.positionAge);motion::put32(data+18,sourceArmFailure.imuAge);
      motion::put16(data+22,sourceArmFailure.online);motion::put16(data+24,sourceArmFailure.fault);
      motion::put16(data+26,sourceArmFailure.torque);data[28]=sourceArmFailure.imuOk;data[29]=1;
      respond(0x80,frame.sequence,data,sizeof(data));return;
    }
    // Optional, versioned failure detail. Successful and legacy ACKs stay
    // two bytes; the motion host accepts this exact 22-byte ARM extension.
    static_assert(kArmReadTraceAckSize <= motion::kMaxPayload, "ARM evidence fits frame");
    const bool extended = armFailedReadSeen && (armDiagnostic.stage == 4 || armDiagnostic.stage == 9);
    uint8_t data[kArmReadTraceAckSize] = {frame.type, uint8_t(error), uint8_t(extended ? 2 : 1), armDiagnostic.stage,
      armDiagnostic.id, armDiagnostic.address, armDiagnostic.found, armDiagnostic.error,
      armDiagnostic.size, armDiagnostic.outcome, armDiagnostic.expectedKp, armDiagnostic.actualKp,
      armDiagnostic.expectedKd, armDiagnostic.actualKd};
    const uint32_t now = micros();
    motion::put32(data + 14, uint32_t(now - feedback.sampleUs));
    motion::put32(data + 18, uint32_t(now - feedback.imuUs));
    if (extended) encodeArmReadTrace(data);
    respond(0x80, frame.sequence, data, extended ? sizeof(data) : 22); return;
  }
  const uint8_t data[] = {frame.type, uint8_t(error)}; respond(0x80, frame.sequence, data, sizeof(data));
}
void fillState(uint8_t* data) {
  motion::put32(data, feedback.sampleUs); motion::put32(data + 4, feedback.imuUs);
  uint16_t legacyOnline=feedback.onlineMask;
  if(lock.sourceRuntime) {
    legacyOnline &= torqueKnown;
    for(size_t i=0;i<15;++i) if(uint32_t(micros()-torqueReadUs[i])>kTorqueFreshUs) legacyOnline &= ~uint16_t(1u<<i);
  }
  motion::put16(data + 8, legacyOnline); motion::put16(data + 10, feedback.faultMask);
  motion::put16(data + 12, feedback.torqueMask); data[14] = lock.armed;
  data[15] = feedback.imuOk && uint32_t(micros() - feedback.imuUs) <= motion::kStateFreshUs;
  for (size_t i = 0; i < 15; ++i) {
    motion::put16(data + 16 + i * 2, feedback.positions[i]);
    motion::put16(data + 46 + i * 2, uint16_t(velocities[i]));
  }
  static_assert(sizeof(float) == 4, "Protocol floats must be float32");
  memcpy(data + 76, accel, sizeof(accel)); memcpy(data + 88, gyro, sizeof(gyro));
  motion::put32(data + 100, lock.cookie); data[104] = uint8_t(lock.lastError); data[105] = lock.configured;
}
void state(uint32_t seq, bool withVoltage = false, uint8_t evidenceVersion = 0) {
  checkSafety(); flushDisable();
  uint8_t data[motion::kMotionEvidenceSize] = {};
  fillState(data);
  if (!withVoltage) { respond(0x82, seq, data, motion::kStateSize); return; }
  const uint32_t now = micros();
  uint16_t valid = maintenanceMode ? 0 : voltageValid;
  data[106] = 1; motion::put16(data + 107, voltageSeen);
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    const uint16_t bit = uint16_t(1u << i);
    const uint32_t age = voltageSeen & bit ? uint32_t(now - voltageReadUs[i]) : UINT32_MAX;
    if (age > motion::kVoltageFreshUs) valid &= ~bit;
    data[111 + i] = voltageRaw[i]; motion::put32(data + 126 + 4 * i, age);
  }
  motion::put16(data + 109, valid);
  if (evidenceVersion == 2) {
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
  if (evidenceVersion == 1) {
    uint8_t* evidence = data + motion::kVoltageStateSize;
    evidence[0] = 1;
    motion::put32(evidence + 1, lastWrittenGoalsUs);
    motion::put32(evidence + 5, servoSampleSequence);
    for (size_t i = 0; i < motion::kMotorCount; ++i) {
      motion::put16(evidence + 9 + 2*i, lastWrittenGoals[i]);
      motion::put16(evidence + 39 + 2*i, presentLoadRaw[i]);
    }
  }
  respond(0x93, seq, data, evidenceVersion == 1 ? motion::kMotionEvidenceSize : motion::kVoltageStateSize);
}

void sampleVoltage() {
  // One fixed-register READ per loop, under the existing UART owner. Never
  // write a goal/torque or extend position/IMU/watchdog freshness for voltage.
  const size_t i = nextVoltage; nextVoltage = (nextVoltage + 1) % motion::kMotorCount;
  const uint16_t bit = uint16_t(1u << i);
  const ServoReply reply = readServo(motion::kMotorIds[i], 62, 1);
  if (reply.found && !reply.error && reply.size == 1 && reply.data[0] != 0) {
    voltageRaw[i] = reply.data[0]; voltageReadUs[i] = micros();
    voltageSeen |= bit; voltageValid |= bit;
  } else voltageValid &= ~bit;  // Retain explicitly dated last valid value.
  sampleImu(); checkSafety(); flushDisable();
}

void faultDetails(uint32_t seq) {
  static_assert(motion::kFaultDetailsSize <= motion::kMaxPayload, "Fault details must fit one frame");
  uint8_t data[motion::kFaultDetailsSize] = {};
  data[0] = motion::kFaultDetailsVersion; data[1] = motion::kMotorCount;
  motion::put32(data + 2, micros()); motion::put32(data + 6, servoErrors.load());
  motion::put16(data + 10, feedback.onlineMask); motion::put16(data + 12, feedback.faultMask);
  motion::put16(data + 14, feedback.torqueMask); motion::put16(data + 16, presentFaults);
  motion::put16(data + 18, torqueFaults); data[20] = lock.armed;
  motion::put32(data + 21, lock.cookie);
  presentHistory.encode(data + motion::kFaultDetailsHeaderSize);
  torqueHistory.encode(data + motion::kFaultDetailsHeaderSize + motion::kFaultDetailsChannelSize);
  respond(0x87, seq, data, sizeof(data));
}

void positionReadDetails(uint32_t seq, size_t index) {
  static_assert(motion::kPositionReadDetailsSize <= motion::kMaxPayload,
                "Position read details must fit one frame");
  const PositionReadEvidence& evidence = positionReadEvidence[index];
  const ServoReply& reply = evidence.reply;
  uint8_t data[motion::kPositionReadDetailsSize] = {};
  data[0] = motion::kPositionReadDetailsVersion; data[1] = motion::kMotorIds[index];
  data[2] = 56; data[3] = 6;
  data[4] = (evidence.attempted ? 1 : 0) | (reply.found ? 2 : 0) |
            (reply.invalidReplySeen ? 4 : 0) | (evidence.registerRejected ? 8 : 0);
  data[5] = reply.size; data[6] = reply.error;
  data[7] = evidence.registerRejected ? uint8_t(motion::InvalidRegisterReply) : uint8_t(reply.outcome);
  motion::put32(data + 8, evidence.attemptUs); motion::put32(data + 12, micros());
  memcpy(data + 16, reply.data, sizeof(reply.data));
  respond(0x95, seq, data, sizeof(data));
}

void goalReadBlock(uint8_t* data, uint8_t id, uint8_t address, uint8_t size) {
  motion::put32(data, micros());
  const ServoReply reply = readServo(id, address, size);
  motion::put32(data + 4, micros());
  const bool valid = reply.found && !reply.error && reply.size == size &&
                     motion::u16(reply.data) <= 4095;
  // Preserve readServo's outcome and bytes even when register validation fails.
  data[8] = uint8_t(reply.outcome); data[9] = reply.error; data[10] = reply.size;
  data[11] = (reply.found ? 1 : 0) | (reply.invalidReplySeen ? 2 : 0) | (valid ? 4 : 0);
  memcpy(data + 12, reply.data, sizeof(reply.data));
}

void goalReadDetails(const motion::Frame& frame) {
  size_t index = 0;
  if (frame.length != 1) { ack(frame, motion::BadPacket); return; }
  while (index < motion::kMotorCount && motion::kMotorIds[index] != frame.payload[0]) ++index;
  if (index == motion::kMotorCount) { ack(frame, motion::BadPacket); return; }
  if (lock.armed || lock.cookie || maintenanceMode || preparingArm || verifyingTorque ||
      disablePending || torqueVerifyPending || feedback.torqueMask) {
    ack(frame, motion::BadState); return;
  }
  const uint32_t now = micros();
  if (torqueKnown != motion::kAllMotors || !servoSampleSequence ||
      uint32_t(now - feedback.sampleUs) > motion::kStateFreshUs) {
    ack(frame, motion::Stale); return;
  }
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    if (uint32_t(now - torqueReadUs[i]) > kTorqueFreshUs) {
      ack(frame, motion::Stale); return;
    }
  }
  if (goalReadDetailsSeen && uint32_t(now - lastGoalReadDetailsUs) < motion::kGoalReadDetailsPeriodUs) {
    ack(frame, motion::BadState); return;
  }
  // This quota is global for this boot, including failed reads and STOP/HELLO.
  // Rejected requests neither consume it nor touch servo UART/write sequence.
  goalReadDetailsSeen = true; lastGoalReadDetailsUs = now;
  static_assert(motion::kGoalReadDetailsSize <= motion::kMaxPayload,
                "Goal read details must fit one frame");
  uint8_t data[motion::kGoalReadDetailsSize] = {};
  data[0] = motion::kGoalReadDetailsVersion; data[1] = motion::kMotorIds[index];
  data[2] = uint8_t(index);
  motion::put16(data + 4, lock.desired[index]); motion::put16(data + 6, lock.goals[index]);
  data[8] = lock.armed; motion::put32(data + 10, lock.cookie);
  motion::put32(data + 18, feedback.sampleUs); motion::put32(data + 22, feedback.imuUs);
  motion::put16(data + 34, torqueKnown); motion::put16(data + 36, feedback.torqueMask);
  goalReadBlock(data + 42, motion::kMotorIds[index], 42, 2);
  goalReadBlock(data + 60, motion::kMotorIds[index], 56, 6);
  data[9] = lock.armed; motion::put32(data + 14, lock.cookie);
  motion::put32(data + 26, feedback.sampleUs); motion::put32(data + 30, feedback.imuUs);
  motion::put16(data + 38, torqueKnown); motion::put16(data + 40, feedback.torqueMask);
  respond(0x96, frame.sequence, data, sizeof(data));
}

#include "off_register_details.h"
#include "maintenance_impl.h"

void sampleServos();
void command(const motion::Frame& frame) {
  // A diagnostic refusal must not flush a pending servo write or overwrite a
  // safety error. Accepted reads retain readServo's existing safety checks.
  // Explicit one-shot diagnostic only: change next ARM post-enable read
  // order, not motor IDs, target order, gains, budgets or accepted feedback.
  if (frame.type == motion::SourceRuntime) {
    if (frame.length) { ack(frame, motion::BadPacket); return; }
    if (lock.armed || lock.cookie || maintenanceMode || preparingArm || disablePending ||
        torqueKnown != motion::kAllMotors || feedback.torqueMask) {
      ack(frame, motion::BadState); return;
    }
    lock.sourceRuntime = true; sourceVoltageEnabled = false; updateMasks(micros());
    ack(frame, motion::Ok); return;
  }
  if (frame.type == motion::SourceState) {
    const bool withVoltage = frame.length == 1 && frame.payload[0] == 1;
    if ((frame.length && !withVoltage) || !lock.sourceRuntime) { ack(frame, motion::BadState); return; }
    if (withVoltage) sourceVoltageEnabled = true;
    if(!lock.armed && !maintenanceMode) {
      bool expired=torqueKnown!=motion::kAllMotors;
      for(size_t i=0;i<15;++i) expired |= uint32_t(micros()-torqueReadUs[i])>kTorqueFreshUs;
      // SOURCE-STATE-20261013: upstream publishes a cached StatePacket;
      // this ESP32 port added synchronous per-servo OFF verification here.
      // After a sensor fault, loop() already performs that scan. Repeating
      // all 15 reads inside this request can delay the fault reply beyond
      // the existing host deadline when the bus is silent. Schedule the
      // existing verification instead; reply with the current known mask.
      // Unknown/expired OFF bits remain unknown. STOP's explicit verification
      // and all existing read deadlines, gains and motion gates are unchanged.
      if(expired) sourceOffVerifyPending = true;
    }
    checkSafety(); flushDisable();
    uint8_t data[249] = {}; fillState(data);
    motion::put16(data+8,feedback.onlineMask); // source state separates online from torque-known.
    uint16_t known = torqueKnown; const uint32_t now=micros();
    for(size_t i=0;i<15;++i) if(uint32_t(now-torqueReadUs[i])>kTorqueFreshUs) known &= ~uint16_t(1u<<i);
    motion::put16(data+106,known); data[108]=1; data[109]=1;
    for(size_t i=0;i<15;++i) motion::put32(data+110+4*i,
      positionSeen & (1u<<i) ? uint32_t(now-positionReadUs[i]) : UINT32_MAX);
    size_t size = 170;
    // Legacy replies retain the exact 170/214-byte source packet. Attach
    // the first autonomous stop only AFTER disarm, on the existing STATE read.
    // No added servo transaction, request, running bytes or watchdog renewal.
    if (!lock.armed && safetyDiagnostic.seen) {
      data[109] = 2;
      uint8_t* stop = data + 170;
      stop[0] = motion::Target; stop[1] = motion::BadState; stop[2] = 1;
      stop[3] = safetyDiagnostic.cause; stop[4] = safetyDiagnostic.error;
      stop[5] = safetyDiagnostic.imuOk;
      motion::put16(stop+6,safetyDiagnostic.online); motion::put16(stop+8,safetyDiagnostic.fault);
      motion::put16(stop+10,safetyDiagnostic.torque); motion::put32(stop+12,safetyDiagnostic.atUs);
      motion::put32(stop+16,safetyDiagnostic.positionAge); motion::put32(stop+20,safetyDiagnostic.imuAge);
      motion::put32(stop+24,safetyDiagnostic.targetAge); motion::put32(stop+28,safetyDiagnostic.uartErrors);
      stop[32]=safetyDiagnostic.readId; stop[33]=safetyDiagnostic.address;
      stop[34]=safetyDiagnostic.outcome; stop[35]=safetyDiagnostic.deviceError;
      motion::put32(stop+36,safetyDiagnostic.readElapsed); motion::put32(stop+40,safetyDiagnostic.cookie);
      size = 214;
    }
    if (withVoltage) {
      // Compact extension: version, seen/valid masks, 15 raw bytes, and
      // 15 ages rounded UP to 100ms (255 means at least 25.5s old).
      // Keeps the first-stop record and stays within the 256-byte envelope.
      data[109] += 2;
      uint8_t* v = data + size; v[0] = 1;
      motion::put16(v+1, voltageSeen); uint16_t valid=voltageValid;
      for(size_t i=0;i<15;++i) {
        const uint32_t age=uint32_t(now-voltageReadUs[i]);
        if(age>motion::kVoltageFreshUs || maintenanceMode) valid &= ~uint16_t(1u<<i);
        v[5+i]=voltageRaw[i];
        v[20+i]=age>=25500000 ? 255 : uint8_t((age+99999)/100000);
      }
      motion::put16(v+3,valid); size += 35;
    }
    respond(0xa0,frame.sequence,data,size);
    return;
  }
  if (frame.type == motion::ArmReadOrder) {
    if (frame.length != 1 || frame.payload[0] > motion::kMotorCount) { ack(frame, motion::BadPacket); return; }
    if (lock.armed || lock.cookie || maintenanceMode || preparingArm || verifyingTorque || disablePending || torqueVerifyPending ||
        torqueKnown != motion::kAllMotors || feedback.torqueMask || !feedback.fresh(micros())) { ack(frame, motion::BadState); return; }
    nextArmFirstIndex = frame.payload[0]; ack(frame, motion::Ok); return;
  }
  if (frame.type == motion::ArmBusDetails) {
    if (frame.length != 1 || frame.payload[0] >= armBusCount) { ack(frame, motion::BadPacket); return; }
    const uint8_t index = frame.payload[0]; const ArmBusEntry& entry = armBusEntries[index];
    const ServoReadTrace& trace = entry.trace;
    uint8_t data[148] = {4, 0, 2, 9, trace.id, trace.address, trace.reply.found,
      trace.reply.error, trace.reply.size, uint8_t(trace.reply.outcome)};
    encodeArmReadTrace(data, trace);
    data[134] = 2; data[135] = armBusCount; data[136] = index; data[137] = entry.armed;
    motion::put32(data + 138, entry.disabledUs); motion::put32(data + 142, trace.transmittedUs); data[146] = activeArmFirstIndex;
    respond(0x9c, frame.sequence, data, sizeof(data)); return;
  }
  if (frame.type == motion::GoalReadDetails) { goalReadDetails(frame); return; }
  if (frame.type == motion::OffRegisterDetails) { offRegisterDetails(frame); return; }
  if (frame.type == motion::Arm) {
    armDiagnostic = ArmDiagnostic{}; sourceArmFailure=SourceArmFailure{}; armFailedReadSeen = false; armBusCount = 0;
    activeArmFirstIndex = nextArmFirstIndex; nextArmFirstIndex = 0;
    armFailedReadTrace = ServoReadTrace{}; armEnableCompletedUs = 0;
    armReadSequence = frame.sequence; armReadCookie = frame.length == 4 ? motion::u32(frame.payload) : 0;
  }
  checkSafety(); flushDisable();
  if (frame.type == motion::Stop) {
    if (maintenanceMode) (void)maintenanceClose();
    requestDisarm(frame.length ? motion::BadPacket : motion::Ok);
    flushDisable(); refreshTorque();
    ack(frame, frame.length ? motion::BadPacket : motion::Ok); return;
  }
  if (frame.type == motion::Hello) {
    if (frame.length) { ack(frame, lock.record(motion::BadPacket)); return; }
    if (!lock.armed) { sequence.reset(); lock.sourceRuntime = false; }
    constexpr char greeting[] = "XGODUCK_MOTION_1";
    respond(0x81, frame.sequence, reinterpret_cast<const uint8_t*>(greeting), sizeof(greeting) - 1); return;
  }
  if (frame.type == motion::State || frame.type == motion::VoltageState) {
    const uint8_t evidenceVersion = frame.type == motion::VoltageState && frame.length == 1 &&
      (frame.payload[0] == 1 || frame.payload[0] == 2) ? frame.payload[0] : 0;
    if (frame.length && !evidenceVersion) ack(frame, lock.record(motion::BadPacket));
    else state(frame.sequence, frame.type == motion::VoltageState, evidenceVersion);
    return;
  }
  if (frame.type == motion::PositionRetryDetails) {
    if (frame.length) { ack(frame, motion::BadPacket); return; }
    uint8_t data[28] = {1, positionRetryDiagnostic.id,
      positionRetryDiagnostic.firstOutcome, positionRetryDiagnostic.retryOutcome};
    motion::put32(data + 4, positionRetryDiagnostic.attempts);
    motion::put32(data + 8, positionRetryDiagnostic.recovered);
    motion::put32(data + 12, positionRetryDiagnostic.failed);
    motion::put32(data + 16, positionRetryDiagnostic.atUs);
    motion::put32(data + 20, positionRetryDiagnostic.firstElapsedUs);
    motion::put32(data + 24, positionRetryDiagnostic.retryElapsedUs);
    respond(0x9a, frame.sequence, data, sizeof(data)); return;
  }
  if (frame.type == motion::MotionEvidenceCaps) {
    if (frame.length == 1 && frame.payload[0] == 2) {
      constexpr char capability[] = "XGODUCK_MOTION_EVIDENCE_2";
      respond(0x99, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability)-1);
    } else if (frame.length) ack(frame, lock.record(motion::BadPacket));
    else {
      constexpr char capability[] = "XGODUCK_MOTION_EVIDENCE_1";
      respond(0x99, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability)-1);
    }
    return;
  }
  if (frame.type == motion::TelemetryCaps) {
    if (frame.length) ack(frame, lock.record(motion::BadPacket));
    else {
      constexpr char capability[] = "XGODUCK_TELEMETRY_1";
      respond(0x94, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability) - 1);
    }
    return;
  }
  if (frame.type == motion::OfficialRuntimeCaps) {
    ack(frame, frame.length ? motion::BadPacket : motion::Ok); return;
  }
  if (frame.type == motion::PolicyPositionCaps) {
    ack(frame, frame.length ? motion::BadPacket : motion::Ok); return;
  }
  if (frame.type == motion::FaultDetails) {
    if (frame.length) ack(frame, lock.record(motion::BadPacket)); else faultDetails(frame.sequence);
    return;
  }
  if (frame.type == motion::PositionReadDetails) {
    size_t index = 0;
    if (frame.length != 1) { ack(frame, motion::BadPacket); return; }
    while (index < motion::kMotorCount && motion::kMotorIds[index] != frame.payload[0]) ++index;
    if (index == motion::kMotorCount) { ack(frame, motion::BadPacket); return; }
    if (lock.armed || lock.cookie || maintenanceMode || preparingArm || verifyingTorque) {
      ack(frame, motion::BadState); return;
    }
    positionReadDetails(frame.sequence, index); return;
  }
  if (maintenanceCommand(frame)) return;
  if (maintenanceMode) { ack(frame, motion::BadState); return; }
  if (frame.type != motion::ConfigRequest && frame.type != motion::Arm && frame.type != motion::Target && frame.type != motion::SourceTarget) {
    ack(frame, lock.record(motion::BadPacket)); return;
  }
  // Consume write sequence even for invalid payloads, before side effects.
  if (!sequence.consume(frame.sequence)) {
    if(frame.type==motion::SourceTarget) {requestDisarm(motion::BadPacket);flushDisable();}
    else ack(frame, lock.record(motion::BadPacket));
    return;
  }
  if (frame.type == motion::ConfigRequest) { ack(frame, lock.configure(frame)); return; }
  if (frame.type == motion::SourceTarget) {
    // No request/ACK wait on ordinary policy targets, as in upstream notify().
    // Refusals remain visible in STATE and cannot leave an old target active.
    motion::Error result = !lock.sourceRuntime ? motion::BadState : lock.target(frame,micros());
    if(result!=motion::Ok) {requestDisarm(result);flushDisable();}
    return;
  }
  if (frame.type == motion::Target) { ack(frame, lock.target(frame, micros())); return; }
  if (lock.sourceRuntime) {
    // Validate current pose before writes, then preserve upstream startup order:
    // gain writes -> current goals -> enable. Normal loop reads follow enable.
    // Do not insert another synchronous read between gain and target writes.
    preparingArm=true;
    const uint8_t kp[]={50,6}, kd[]={51,20}, mouth[]={50,10};
    armDiagnostic.stage=1;
    motion::Error result=lock.prepareArm(frame,feedback,micros());
    if(result==motion::Ok) {armDiagnostic.stage=5;sampleServos();sampleImu();result=lock.prepareArm(frame,feedback,micros());}
    if(result==motion::Ok) armDiagnostic.stage=2;
    if(result==motion::Ok && (!sendServo(0xfe,3,kp,2)||!txComplete()||!sendServo(0xfe,3,kd,2)||!txComplete()||
       !sendServo(34,3,mouth,2)||!txComplete())) result=motion::BadState;
    if(result==motion::Ok) armDiagnostic.stage=6;
    if(result==motion::Ok && !writeGoals(lock.goals)) result=motion::BadState;
    if(result==motion::Ok) armDiagnostic.stage=7;
    if(result==motion::Ok && !feedback.fresh(micros())) result=motion::Stale;
    if(result==motion::Ok) armDiagnostic.stage=8;
    if(result==motion::Ok && !writeTorque(true)) result=motion::BadState;
    if(result==motion::Ok) {
      torqueKnown=0; feedback.torqueMask=0; // ON broadcast is not measured ON.
      lock.armed=true; ++causalArmEpoch; safetyDiagnostic=SafetyDiagnostic{};
    } else {
      const uint32_t now=micros();sourceArmFailure.seen=true;
      sourceArmFailure.online=feedback.onlineMask;sourceArmFailure.fault=feedback.faultMask;
      sourceArmFailure.torque=feedback.torqueMask;sourceArmFailure.imuOk=feedback.imuOk;
      sourceArmFailure.positionAge=uint32_t(now-feedback.sampleUs);sourceArmFailure.imuAge=uint32_t(now-feedback.imuUs);
      requestDisarm(result);flushDisable();
    }
    preparingArm=false; ack(frame,result); return;
  }
  struct PreparationGuard {
    PreparationGuard() { preparingArm = true; }
    ~PreparationGuard() { preparingArm = false; }
  } preparationGuard;
  armDiagnostic.stage = 1;
  motion::Error result = lock.prepareArm(frame, feedback, micros());
  if (result == motion::Ok) {
    result = prepareRunGains();
    if (result != motion::Ok) { requestDisarm(result); flushDisable(); }
  }
  if (result == motion::Ok) {
    // Preparation reads can outlive the pre-ARM sample. Re-read torque,
    // then every actual position/IMU, and revalidate the current pose while
    // torque is still off. No timeout/range/watchdog threshold is relaxed.
    armDiagnostic = ArmDiagnostic{}; armDiagnostic.stage = 4;
    refreshTorque();
    if (torqueKnown != motion::kAllMotors || feedback.faultMask || feedback.torqueMask != 0) {
      result = torqueKnown != motion::kAllMotors ? motion::Stale : motion::BadState;
    } else {
      armDiagnostic = ArmDiagnostic{}; armDiagnostic.stage = 5;
      sampleServos(); sampleImu();
      result = lock.prepareArm(frame, feedback, micros());
    }
    if (result != motion::Ok) { requestDisarm(result); flushDisable(); }
  }
  if (result == motion::Ok) {
    // Preserve current position as goal before enabling all motor torque.
    armDiagnostic.stage = 6;
    if (!writeGoals(lock.goals)) {
      requestDisarm(motion::BadState); flushDisable(); result = motion::BadState;
    } else if (!feedback.fresh(micros()) || feedback.torqueMask != 0) {
      // Gain readback and TX time must not turn a once-fresh state into an
      // unsafe enable. A refused ARM requires an explicit retry.
      armDiagnostic.stage = 7;
      requestDisarm(motion::Stale); flushDisable(); result = motion::Stale;
    } else if (!writeTorque(true)) {
      armDiagnostic.stage = 8;
      requestDisarm(motion::BadState); flushDisable(); result = motion::BadState;
    } else {
      armDiagnostic.stage = 9;
      safetyDiagnostic = SafetyDiagnostic{};
      lock.armed = true;
      ++causalArmEpoch;
      refreshTorque();
      checkSafety(); flushDisable();
      if (!lock.armed) result = lock.lastError;
    }
  }
  ack(frame, result);
}

void serviceHost() {
  // sampleServos() is also called inside ARM. Do not recursively process
  // another host command until that bounded preparation has finished.
  if (preparingArm) return;
  // Bound host work: a hostile input stream cannot starve sampling/watchdog.
  for (size_t n = 0; n < 512 && host.available(); ++n) {
    const int input = host.read(); if (input < 0) break;
    if (parser.pending() && uint32_t(micros() - lastHostByteUs) > 25000) parser.reset();
    lastHostByteUs = micros();
    if (parser.feed(uint8_t(input))) command(parser.frame);
    checkSafety(); flushDisable();
  }
}

void sampleServos() {
  if(lock.sourceRuntime && !maintenanceMode) {
    source_cycle::Scan scan(sourceVoltageEnabled);
    // Same drain-before-sync-read ordering as upstream; bounded to bytes
    // already buffered, no invented quiet-period or repeated clearing.
    int buffered=Serial1.available(); while(buffered-->0) (void)Serial1.read();
    uint8_t packet[23]; source_cycle::request(packet,sourceVoltageEnabled);
    if(Serial1.write(packet,sizeof(packet))!=sizeof(packet)||!txComplete()) {
      if(lock.armed) {requestDisarm(motion::BadState);flushDisable();}
      return;
    }
    const uint32_t began=micros();
    auto receive=[&]() {
      int available=Serial1.available();
      while(available-->0) scan.feed(uint8_t(Serial1.read()),micros());
    };
    sampleImu();
    while(uint32_t(micros()-began)<source_cycle::replyUs) {
      receive(); checkSafety(); flushDisable(); delayMicroseconds(100);
    }
    receive(); // Include bytes present at the deadline, matching upstream.
    if(sourceVoltageEnabled) {
      voltageValid=0;
      for(size_t i=0;i<15;++i) if((scan.mask & (1u<<i)) && scan.voltages[i]) {
        voltageRaw[i]=scan.voltages[i]; voltageReadUs[i]=scan.receivedUs[i];
        voltageSeen |= uint16_t(1u<<i); voltageValid |= uint16_t(1u<<i);
      }
    }
    for(size_t i=0;i<15;++i) if(scan.mask & (1u<<i)) {
      feedback.positions[i]=scan.positions[i]; velocities[i]=scan.speeds[i];
      presentLoadRaw[i]=scan.loads[i]; positionReadUs[i]=scan.receivedUs[i];
      positionSeen |= uint16_t(1u<<i);
    }
    // Missing replies keep last measured values and original per-ID ages;
    // onlineMask describes THIS cycle, never an invented all-online state.
    presentMask=scan.mask; presentFaults=scan.faults;
    feedback.sampleUs=micros(); ++servoSampleSequence; sampleBoundGoals.valid=false;
    updateMasks(micros()); checkSafety();flushDisable(); return;
  }
  const uint32_t startingSequence = servoSampleSequence;
  SampleBoundGoals candidate;
  candidate.sampleStartUs = micros(); candidate.armCookie = lock.cookie;
  candidate.armEpoch = causalArmEpoch;
  candidate.goalWriteUs = lastWrittenGoalsUs; candidate.goalWriteSequence = writtenGoalsSequence;
  memcpy(candidate.goals, lastWrittenGoals, sizeof(candidate.goals));
  candidate.valid = lock.armed && lock.cookie && writtenGoalsSeen && !maintenanceMode &&
    uint32_t(candidate.sampleStartUs - candidate.goalWriteUs) < 0x80000000u;
  uint16_t positions[15] = {}, loads[15] = {}; int16_t speeds[15] = {};
  uint16_t cycleOnline = 0, cycleFaults = 0;
  bool positionRetryUsed = false;
  for (size_t i = 0; i < 15; ++i) {
    const uint16_t bit = uint16_t(1u << i);
    uint32_t attemptUs = micros();
    ServoReply reply = readServo(motion::kMotorIds[i], 56, 6);
    if (canRetryPosition(reply, positionRetryUsed, micros())) {
      positionRetryUsed = true;
      positionRetryDiagnostic.id = motion::kMotorIds[i];
      positionRetryDiagnostic.firstOutcome = uint8_t(reply.outcome);
      positionRetryDiagnostic.firstElapsedUs = lastServoReadTrace.elapsedUs;
      positionRetryDiagnostic.atUs = micros();
      incrementRetryCounter(positionRetryDiagnostic.attempts);
      // Keep the previous complete sample timestamp and masks. A retry
      // cannot renew sensor freshness, the host lease, or the ARM cookie.
      attemptUs = micros();
      reply = readServo(motion::kMotorIds[i], 56, 6);
      positionRetryDiagnostic.retryOutcome = uint8_t(reply.outcome);
      if (reply.outcome == motion::ValidNormalReply &&
          (!reply.found || reply.error || reply.size != 6 || motion::u16(reply.data) > 4095))
        positionRetryDiagnostic.retryOutcome = uint8_t(motion::InvalidRegisterReply);
      positionRetryDiagnostic.retryElapsedUs = lastServoReadTrace.elapsedUs;
      const bool recovered = lock.armed && reply.found && !reply.error &&
        reply.size == 6 && motion::u16(reply.data) <= 4095;
      incrementRetryCounter(recovered ? positionRetryDiagnostic.recovered : positionRetryDiagnostic.failed);
    }
    positionReadEvidence[i].record(reply,
      reply.found && reply.error == 0 && reply.size == 6 && motion::u16(reply.data) <= 4095, attemptUs);
    presentHistory.record(i, reply, reply.found && reply.error == 0 && reply.size == 6 && motion::u16(reply.data) <= 4095);
    if (reply.found && reply.error == 0 && reply.size == 6 && motion::u16(reply.data) <= 4095) {
      cycleOnline |= bit; positions[i] = motion::u16(reply.data);
      speeds[i] = motion::speedSignMagnitude(motion::u16(reply.data + 2));
      // Bytes60..61 already arrive in the existing56/6 read. Preserve raw
      // words; no torque, sign or current unit is assumed here.
      loads[i] = motion::u16(reply.data + 4);
    } else {
      presentMask &= ~bit;
      if (reply.found && reply.error) { cycleFaults |= bit; presentFaults |= bit; }
    }
    sampleImu(); checkSafety(); flushDisable(); serviceHost();
    // An Enter command can arrive between fixed-map reads. Do not continue
    // that cycle or publish its partial arrays after maintenance takes UART.
    if (maintenanceMode || servoSampleSequence != startingSequence) return;
  }
  // Host never sees half-updated arrays carrying a completed-cycle timestamp.
  memcpy(feedback.positions, positions, sizeof(positions)); memcpy(velocities, speeds, sizeof(speeds));
  memcpy(presentLoadRaw, loads, sizeof(loads));
  presentMask = cycleOnline; presentFaults = cycleFaults; feedback.sampleUs = micros();
  ++servoSampleSequence;
  candidate.sampleSequence = servoSampleSequence;
  candidate.valid = candidate.valid && lock.armed && lock.cookie == candidate.armCookie &&
    causalArmEpoch == candidate.armEpoch && writtenGoalsSequence == candidate.goalWriteSequence &&
    cycleOnline == motion::kAllMotors && cycleFaults == 0;
  sampleBoundGoals = candidate;
  updateMasks(micros()); checkSafety(); flushDisable();
}
}  // namespace

void setup() {
  host.setRxBufferSize(2048); host.begin(921600, SERIAL_8N1, 44, 43);
  Serial1.setRxBufferSize(256); Serial1.begin(1000000, SERIAL_8N1, 18, 17);
  Serial1.setRxFIFOFull(1); Serial1.onReceiveError(uartError);
  // Boot never moves goals or enables torque. Disarm even before IMU init.
  requestDisarm(motion::Ok); flushDisable();
  Wire.setTimeOut(2); i2cReady = Wire.begin(8, 9, 400000);
  initImu(); refreshTorque();
}

void loop() {
  static uint32_t sampleStart = 0, lastImuPoll = 0, lastGoals = 0;
  serviceHost();
  uint32_t now = micros();
  if (uint32_t(now - lastImuPoll) >= 2000) { lastImuPoll = now; sampleImu(); }
  if (maintenanceMode) {
    voltageValid = 0;
    maintenanceTick();
    delay(1); return;
  }
  checkSafety(); flushDisable();
  now = micros();
  if (uint32_t(now - sampleStart) >= kSamplePeriodUs) { sampleStart = now; sampleServos(); }
  now = micros();
  // An OFF status request may be followed by ARM before this loop slot.
  // Drop only that observational request after a mode transition; never
  // cancel torqueVerifyPending, which belongs to required STOP verification.
  const bool verifySourceOff = sourceOffVerifyPending && lock.sourceRuntime && !lock.armed;
  sourceOffVerifyPending = false;
  if (torqueVerifyPending || verifySourceOff) {
    refreshTorque();
  } else if (!lock.sourceRuntime) {
    if (nextBackgroundTorque == motion::kMotorCount && uint32_t(now - lastTorqueRefreshUs) >= kTorquePeriodUs) {
      lastTorqueRefreshUs = now; nextBackgroundTorque = 0;
    }
    if (nextBackgroundTorque < motion::kMotorCount) {
      // A full torque scan followed by a full position scan can age the last
      // completed position sample past 50 ms even when every READ succeeds.
      // Interleave bounded torque batches with normal position cycles instead;
      // retain each real readback's timestamp, masks and strict safety checks.
      const size_t end = nextBackgroundTorque + kBackgroundTorqueBatch < motion::kMotorCount ?
                         nextBackgroundTorque + kBackgroundTorqueBatch : motion::kMotorCount;
      verifyingTorque = true;
      readTorqueRange(nextBackgroundTorque, end);
      finishTorqueVerification(); nextBackgroundTorque = end;
    }
  }
  if (!maintenanceMode && !lock.sourceRuntime) sampleVoltage();
  if (!lock.armed && (feedback.torqueMask || disablePending) && uint32_t(micros() - lastDisableUs) > 1000000) {
    disablePending = true; torqueVerifyPending = true; flushDisable();
  }
  checkSafety(); flushDisable();
  maybeRecoverImu(micros());
  now = micros();
  if (lock.armed && uint32_t(now - lastGoals) >= kSamplePeriodUs) {
    lastGoals = now; lock.slew(now);
    if (!writeGoals(lock.goals)) { requestDisarm(motion::BadState, GoalWriteStop); flushDisable(); }
  }
  delay(1);
}
