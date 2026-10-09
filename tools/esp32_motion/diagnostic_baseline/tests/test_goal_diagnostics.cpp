// Run the actual command/readServo handlers with local UART fakes only.
#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main

namespace {
enum ReplyMode { Normal, Missing, DeviceError, EmptyDeviceError, InvalidRaw,
                 Corrupt, WrongId, HardwareError, WrongSize };
ReplyMode goalMode = Normal, presentMode = Normal;
uint16_t servoGoal = 2002, servoPresent = 1941;
bool queueLifecycle = false;
std::vector<std::vector<uint8_t>> diagnosticTransactions;

void diagnosticBus(const uint8_t* packet, size_t count) {
  diagnosticTransactions.emplace_back(packet, packet + count);
  assert(count == 8 && packet[0] == 255 && packet[1] == 255 && packet[4] == 2);
  assert((packet[5] == 42 && packet[6] == 2) || (packet[5] == 56 && packet[6] == 6));
  uint8_t checksum = 0; for (size_t i = 2; i < count; ++i) checksum += packet[i];
  assert(checksum == 255);
  if (packet[5] == 42 && queueLifecycle) {
    queueLifecycle = false;
    motion::Frame stop; stop.type = motion::Stop;
    const auto arm = armCommand(4);
    const motion::Frame* queued[] = {&stop,&arm};
    for (const auto* frame : queued) {
      uint8_t bytes[64];
      const size_t n = motion::encode(frame->type,frame->sequence,frame->payload,frame->length,bytes);
      Serial.input.insert(Serial.input.end(),bytes,bytes+n);
    }
  }
  const ReplyMode mode = packet[5] == 42 ? goalMode : presentMode;
  if (mode == Missing) return;
  if (mode == HardwareError) uartError(1);
  Serial1.input.insert(Serial1.input.end(), packet, packet + count);
  const uint8_t error = mode == DeviceError || mode == EmptyDeviceError ? 0x20 : 0;
  const uint8_t size = mode == EmptyDeviceError ? 0 : mode == WrongSize ? 1 : packet[6];
  std::vector<uint8_t> reply = {255,255,uint8_t(packet[2] + (mode == WrongId ? 1 : 0)),uint8_t(size + 2),error};
  const uint16_t value = mode == InvalidRaw ? 4096 : packet[5] == 42 ? servoGoal : servoPresent;
  if (size) reply.push_back(uint8_t(value));
  if (size >= 2) reply.push_back(uint8_t(value >> 8));
  if (size == 6) reply.insert(reply.end(), {0x34,0x80,0x78,0x56});
  checksum = 0; for (size_t i = 2; i < reply.size(); ++i) checksum += reply[i];
  reply.push_back(uint8_t(~checksum));
  if (mode == Corrupt) ++reply.back();
  Serial1.input.insert(Serial1.input.end(), reply.begin(), reply.end());
}

void offFixture(uint32_t at = 1000000, bool resetQuota = true) {
  mockMicros = at;
  lock.armed = false; lock.cookie = 0; lock.configured = true;
  lock.lastError = motion::Stale; lock.lastCommandUs = at - 200000;
  feedback.sampleUs = at; feedback.imuUs = at - 1000000;
  feedback.imuOk = false;  // IMU health must not authorize or block this READ.
  feedback.onlineMask = presentMask = torqueKnown = motion::kAllMotors;
  feedback.faultMask = presentFaults = torqueFaults = feedback.torqueMask = 0;
  servoSampleSequence = 1;
  for (size_t i = 0; i < 15; ++i) {
    torqueReadUs[i] = at;
    feedback.positions[i] = uint16_t(1800 + i);
    lock.desired[i] = uint16_t(2000 + i);
    lock.goals[i] = uint16_t(1900 + i);
    lock.continuousGoals[i] = float(1900 + i) + .25f;
  }
  maintenanceMode = preparingArm = verifyingTorque = disablePending = torqueVerifyPending = false;
  Serial1.input.clear(); Serial1.scheduledInput.clear(); Serial.input.clear(); Serial.output.clear();
  Serial1.writeResultOverride = -1; Serial1.onWrite = diagnosticBus;
  goalMode = presentMode = Normal; diagnosticTransactions.clear();
  queueLifecycle = false;
  if (resetQuota) { goalReadDetailsSeen = false; lastGoalReadDetailsUs = 0; }
}

motion::Frame request(uint8_t id = 11, uint16_t length = 1) {
  motion::Frame frame; frame.type = motion::GoalReadDetails; frame.sequence = 0xfffffff0;
  frame.length = length; frame.payload[0] = id;
  return frame;
}

motion::Frame issue(const motion::Frame& frame, bool accepted) {
  const motion::Feedback before = feedback;
  const motion::Interlock oldLock = lock;
  const auto oldPresent = presentHistory, oldTorque = torqueHistory;
  const auto oldPositionEvidence = positionReadEvidence[1];
  const auto oldTorqueReads = std::vector<uint32_t>(torqueReadUs, torqueReadUs + 15);
  const size_t oldWrites = writes.size(), busBytes = Serial1.output.size();
  command(frame); const motion::Frame response = lastResponse();
  assert(response.sequence == frame.sequence);
  assert(lock.armed == oldLock.armed && lock.cookie == oldLock.cookie && lock.configured == oldLock.configured);
  assert(lock.lastError == oldLock.lastError && lock.lastCommandUs == oldLock.lastCommandUs);
  assert(memcmp(lock.desired, oldLock.desired, sizeof(lock.desired)) == 0);
  assert(memcmp(lock.goals, oldLock.goals, sizeof(lock.goals)) == 0);
  assert(memcmp(lock.continuousGoals, oldLock.continuousGoals, sizeof(lock.continuousGoals)) == 0);
  assert(feedback.sampleUs == before.sampleUs && feedback.imuUs == before.imuUs && feedback.imuOk == before.imuOk);
  assert(memcmp(feedback.positions, before.positions, sizeof(feedback.positions)) == 0);
  assert(feedback.faultMask == before.faultMask && feedback.torqueMask == before.torqueMask);
  // Existing safety checks may expire online bits; diagnostics cannot promote them.
  assert((feedback.onlineMask & ~before.onlineMask) == 0);
  assert(memcmp(&presentHistory, &oldPresent, sizeof(oldPresent)) == 0);
  assert(memcmp(&torqueHistory, &oldTorque, sizeof(oldTorque)) == 0);
  assert(positionReadEvidence[1].attemptUs == oldPositionEvidence.attemptUs);
  for (size_t i = 0; i < 15; ++i) assert(torqueReadUs[i] == oldTorqueReads[i]);
  assert(writes.size() == oldWrites);
  if (!accepted) assert(Serial1.output.size() == busBytes && diagnosticTransactions.empty());
  return response;
}

void refusal(motion::Error error, uint8_t id = 11, uint16_t length = 1) {
  const bool quotaSeen = goalReadDetailsSeen;
  const uint32_t quotaAt = lastGoalReadDetailsUs;
  diagnosticTransactions.clear();
  const auto inputBefore = Serial1.input;
  const motion::Frame response = issue(request(id,length), false);
  assert(response.type == 0x80 && response.length == 2 && response.payload[0] == 22 && response.payload[1] == error);
  assert(goalReadDetailsSeen == quotaSeen && lastGoalReadDetailsUs == quotaAt && Serial1.input == inputBefore);
}

void testLayoutAndReadOnlyBus() {
  offFixture();
  const auto before = feedback;
  const motion::Frame reply = issue(request(), true);
  assert(reply.type == 0x96 && reply.length == motion::kGoalReadDetailsSize && reply.length == 78);
  const uint8_t* p = reply.payload;
  assert(p[0] == 1 && p[1] == 11 && p[2] == 1 && p[3] == 0);
  assert(motion::u16(p+4) == 2001 && motion::u16(p+6) == 1901);
  assert(p[8] == 0 && p[9] == 0 && motion::u32(p+10) == 0 && motion::u32(p+14) == 0);
  assert(motion::u32(p+18) == before.sampleUs && motion::u32(p+26) == before.sampleUs);
  assert(motion::u32(p+22) == before.imuUs && motion::u32(p+30) == before.imuUs);
  assert(motion::u16(p+34) == motion::kAllMotors && motion::u16(p+38) == motion::kAllMotors);
  assert(motion::u16(p+36) == 0 && motion::u16(p+40) == 0);
  for (size_t offset : {size_t(42),size_t(60)}) {
    assert(p[offset+8] == motion::ValidNormalReply && p[offset+9] == 0 && p[offset+11] == 5);
    assert(uint32_t(motion::u32(p+offset+4) - motion::u32(p+offset)) > 0);
  }
  assert(p[52] == 2 && p[70] == 6 && motion::u16(p+54) == servoGoal && motion::u16(p+72) == servoPresent);
  for (size_t i = 56; i < 60; ++i) assert(p[i] == 0);
  assert(motion::u16(p+74) == 0x8034 && motion::u16(p+76) == 0x5678);
  assert(diagnosticTransactions.size() == 2);
  assert(diagnosticTransactions[0][2] == 11 && diagnosticTransactions[0][5] == 42 && diagnosticTransactions[0][6] == 2);
  assert(diagnosticTransactions[1][2] == 11 && diagnosticTransactions[1][5] == 56 && diagnosticTransactions[1][6] == 6);
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    offFixture(); const auto r = issue(request(motion::kMotorIds[i]),true);
    assert(r.payload[1] == motion::kMotorIds[i] && r.payload[2] == i);
  }
}

void testAllRefusalsHaveZeroBusAndPreserveError() {
  for (uint16_t size : {uint16_t(0),uint16_t(2),uint16_t(256)}) { offFixture(); refusal(motion::BadPacket,11,size); }
  for (uint8_t id : {uint8_t(0),uint8_t(15),uint8_t(254),uint8_t(255)}) { offFixture(); refusal(motion::BadPacket,id); }
  for (bool* busy : {&lock.armed,&maintenanceMode,&preparingArm,&verifyingTorque,&disablePending,&torqueVerifyPending}) {
    offFixture(); *busy = true; refusal(motion::BadState);
  }
  offFixture(); lock.armed = true; lock.cookie = 123; feedback.sampleUs = 0; refusal(motion::BadState);
  offFixture(); lock.cookie = 123; refusal(motion::BadState);
  offFixture(); feedback.torqueMask = 1; refusal(motion::BadState);
  offFixture(); torqueKnown &= ~1; refusal(motion::Stale);
  offFixture(); torqueReadUs[14] -= kTorqueFreshUs + 1; refusal(motion::Stale);
  offFixture(); feedback.sampleUs -= motion::kStateFreshUs + 1; refusal(motion::Stale);
  offFixture(); servoSampleSequence = 0; refusal(motion::Stale);
}

void testGlobalRateAndWrap() {
  offFixture(0); issue(request(),true);  // Timestamp zero is not "never used".
  const uint32_t first = lastGoalReadDetailsUs;
  offFixture(first + motion::kGoalReadDetailsPeriodUs - 1,false); refusal(motion::BadState,22);
  offFixture(first + motion::kGoalReadDetailsPeriodUs,false); issue(request(22),true);
  offFixture(0xfffffff0u); issue(request(),true);
  const uint32_t wrapped = lastGoalReadDetailsUs;
  offFixture(wrapped + 10000,false); refusal(motion::BadState);
  offFixture(wrapped + motion::kGoalReadDetailsPeriodUs,false); issue(request(),true);
  offFixture(); goalMode = presentMode = Missing; issue(request(),true);
  goalMode = presentMode = Normal; refusal(motion::BadState); // Failed reads consume quota too.
}

void testFailedAndRejectedRawRepliesNeverRefreshState() {
  for (ReplyMode mode : {Missing,DeviceError,EmptyDeviceError,InvalidRaw,Corrupt,WrongId,HardwareError,WrongSize}) {
    offFixture(); goalMode = presentMode = mode;
    const motion::Frame r = issue(request(),true); const uint8_t* p = r.payload;
    assert(r.type == 0x96 && diagnosticTransactions.size() == 2);
    for (size_t offset : {size_t(42),size_t(60)}) {
      assert(!(p[offset+11] & 4));
      if (mode == InvalidRaw) {
        assert(p[offset+8] == motion::ValidNormalReply && p[offset+9] == 0 && (p[offset+11] & 1));
        assert(motion::u16(p+offset+12) == 4096);
      } else if (mode == DeviceError || mode == EmptyDeviceError) {
        assert(p[offset+8] == motion::ValidDeviceErrorReply && p[offset+9] == 0x20 && (p[offset+11] & 1));
        assert(p[offset+10] == (mode == EmptyDeviceError ? 0 : offset == 42 ? 2 : 6));
        if (mode == DeviceError) assert(motion::u16(p+offset+12) == (offset == 42 ? servoGoal : servoPresent));
      } else {
        assert(p[offset+8] == (mode == HardwareError ? motion::HardwareUartError : motion::NoValidReplyByDeadline));
        assert(!(p[offset+11] & 1) && p[offset+9] == 0 && p[offset+10] == 0);
        for (size_t i = 12; i < 18; ++i) assert(p[offset+i] == 0);
      }
    }
  }
  offFixture(); goalMode = Missing;
  auto r = issue(request(),true);
  assert(r.payload[50] == motion::NoValidReplyByDeadline && !(r.payload[53] & 4));
  assert(r.payload[68] == motion::ValidNormalReply && (r.payload[71] & 4) && motion::u16(r.payload+72) == servoPresent);
  offFixture(); Serial1.writeResultOverride = 3;
  r = issue(request(),true); assert(r.payload[50] == motion::ShortUartWrite && r.payload[68] == motion::ShortUartWrite);
  offFixture(); for (size_t i = 0; i < 1000; ++i) Serial1.input.push_back(0x55);
  r = issue(request(),true); assert(r.payload[50] == motion::BusNotQuiet && r.payload[68] == motion::BusNotQuiet);
  assert(diagnosticTransactions.empty());
  // Valid diagnostic bytes cannot repair an invalid existing feedback bit.
  offFixture(); presentMask &= ~2; feedback.onlineMask &= ~2;
  r = issue(request(),true); assert((r.payload[53] & 4) && (r.payload[71] & 4));
  assert(!(feedback.onlineMask & 2) && !(presentMask & 2));
  // Keep readServo's existing safety checks: actual age expiry may clear bits.
  offFixture(); torqueReadUs[1] -= kTorqueFreshUs - 1;
  r = issue(request(),true); assert(!(feedback.onlineMask & 2));
  assert(motion::u32(r.payload+18) == motion::u32(r.payload+26));
}

void testStopAndNewArmKeepTheirOriginalSemantics() {
  Serial1.onWrite = busWrite; Serial1.input.clear(); Serial1.scheduledInput.clear();
  Serial1.writeResultOverride = -1; Wire.registers[0x0f] = 0x6b; Wire.registers[0x1e] = 3;
  lock = motion::Interlock{}; maintenanceMode = preparingArm = verifyingTorque = false;
  setup(); mockMicros += 110000; readyFeedback();
  command(configuration(1)); assert(lastResponse().payload[1] == motion::Ok);
  command(armCommand(2)); assert(lastResponse().payload[1] == motion::Ok && lock.armed);
  motion::Frame target; target.type = motion::Target; target.length = 34; target.sequence = 3;
  motion::put32(target.payload,123);
  for (size_t i = 0; i < 15; ++i) motion::put16(target.payload+4+2*i,2150);
  command(target); assert(lastResponse().payload[1] == motion::Ok);
  lock.slew(micros()+20000);
  motion::Frame stop; stop.type = motion::Stop; stop.sequence = 0;
  command(stop); assert(lastResponse().payload[1] == motion::Ok && !lock.armed && !lock.cookie);
  sampleServos(); sampleImu();
  const uint16_t desired = lock.desired[1], goal = lock.goals[1];
  goalReadDetailsSeen = false; diagnosticTransactions.clear(); Serial1.onWrite = diagnosticBus;
  queueLifecycle = true;
  const auto r = issue(request(),true);
  assert(motion::u16(r.payload+4) == desired && motion::u16(r.payload+6) == goal);
  assert(!Serial.input.empty() && !lock.armed && !lock.cookie);
  assert(r.payload[8] == 0 && r.payload[9] == 0 && !motion::u32(r.payload+10) && !motion::u32(r.payload+14));
  const uint32_t quota = lastGoalReadDetailsUs;
  // STOP/new ARM queued during the two reads run only after the OFF response.
  // A high diagnostic sequence cannot consume the next write sequence.
  Serial1.onWrite = busWrite;
  serviceHost(); assert(lastResponse().payload[1] == motion::Ok && lock.armed && lock.cookie == 123);
  assert(lock.desired[1] == normalPosition && lock.goals[1] == normalPosition);
  diagnosticTransactions.clear(); refusal(motion::BadState);
  command(stop); assert(lastResponse().payload[1] == motion::Ok && !lock.armed);
  sampleServos(); sampleImu();
  motion::Frame hello; hello.type = motion::Hello; command(hello); lastResponse();
  assert(goalReadDetailsSeen && lastGoalReadDetailsUs == quota);
  Serial1.onWrite = diagnosticBus; refusal(motion::BadState);
}
}  // namespace

int main() {
  testLayoutAndReadOnlyBus(); testAllRefusalsHaveZeroBusAndPreserveError(); testGlobalRateAndWrap();
  testFailedAndRejectedRawRepliesNeverRefreshState(); testStopAndNewArmKeepTheirOriginalSemantics();
  puts("PASS OFF-only GoalReadDetails: exact 78-byte raw layout, mapped READ42/2+56/6 only, zero-bus refusals, global rate/wrap/failures, unchanged safety checks/no freshness promotion, STOP/new ARM/write sequence unchanged");
}
