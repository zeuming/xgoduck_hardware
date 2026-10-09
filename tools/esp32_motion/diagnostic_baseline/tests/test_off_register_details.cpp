// Exercise the actual Type23 handler and readServo with local UART fakes only.
#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main

namespace {
enum RawMode { RawNormal, RawMissing, RawDeviceError, RawEmptyError,
               RawCorrupt, RawWrongId, RawWrongSize, RawShortAck, RawHardwareError };
RawMode rawModes[8] = {};
std::vector<std::vector<uint8_t>> rawTransactions;
bool rawQueueLifecycle = false;

void rawBus(const uint8_t* packet, size_t count) {
  rawTransactions.emplace_back(packet, packet + count);
  assert(count == 8 && packet[0] == 255 && packet[1] == 255 && packet[4] == 2);
  uint8_t checksum = 0; for (size_t i = 2; i < count; ++i) checksum += packet[i];
  assert(checksum == 255);
  size_t block = 0;
  while (block < 8 && (kOffRegisterReadBlocks[block][0] != packet[5] ||
                      kOffRegisterReadBlocks[block][1] != packet[6])) ++block;
  // Type22 is used solely to verify that both handlers share one quota.
  assert(block < 8 || (packet[5] == 42 && packet[6] == 2) ||
                     (packet[5] == 56 && packet[6] == 6));
  if (rawQueueLifecycle) {
    rawQueueLifecycle = false;
    motion::Frame stop; stop.type = motion::Stop;
    const auto arm = armCommand(4);
    const motion::Frame* queued[] = {&stop, &arm};
    for (const auto* frame : queued) {
      uint8_t bytes[64];
      const size_t size = motion::encode(frame->type, frame->sequence, frame->payload, frame->length, bytes);
      Serial.input.insert(Serial.input.end(), bytes, bytes + size);
    }
  }
  const RawMode mode = block < 8 ? rawModes[block] : RawNormal;
  if (mode == RawMissing) return;
  if (mode == RawHardwareError) uartError(1);
  Serial1.input.insert(Serial1.input.end(), packet, packet + count);  // Exact READ echo.
  const uint8_t error = mode == RawDeviceError || mode == RawEmptyError ? 0x20 : 0;
  const uint8_t size = mode == RawEmptyError || mode == RawShortAck ? 0 :
      mode == RawWrongSize ? (packet[6] == 1 ? 2 : 1) : packet[6];
  std::vector<uint8_t> reply = {255,255,uint8_t(packet[2] + (mode == RawWrongId ? 1 : 0)),
                              uint8_t(size + 2),error};
  // High raw words must remain valid without any position-range interpretation.
  for (uint8_t i = 0; i < size; ++i) reply.push_back(0xff);
  checksum = 0; for (size_t i = 2; i < reply.size(); ++i) checksum += reply[i];
  reply.push_back(uint8_t(~checksum));
  if (mode == RawCorrupt) ++reply.back();
  Serial1.input.insert(Serial1.input.end(), reply.begin(), reply.end());
}

void rawOff(uint32_t at = 1000000, bool resetQuota = true) {
  mockMicros = at;
  lock.armed = false; lock.cookie = 0; lock.configured = true;
  lock.lastError = motion::Stale; lock.lastCommandUs = at - 200000;
  feedback.sampleUs = at; feedback.imuUs = at - 1000000; feedback.imuOk = false;
  feedback.onlineMask = presentMask = torqueKnown = motion::kAllMotors;
  feedback.faultMask = presentFaults = torqueFaults = feedback.torqueMask = 0;
  servoSampleSequence = 1;
  for (size_t i = 0; i < 15; ++i) {
    torqueReadUs[i] = at; feedback.positions[i] = uint16_t(1800 + i);
    lock.desired[i] = uint16_t(2000 + i); lock.goals[i] = uint16_t(1900 + i);
    lock.continuousGoals[i] = float(1900 + i) + .25f;
  }
  maintenanceMode = preparingArm = verifyingTorque = disablePending = torqueVerifyPending = false;
  Serial1.input.clear(); Serial1.scheduledInput.clear(); Serial.input.clear(); Serial.output.clear();
  Serial1.writeResultOverride = -1; Serial1.onWrite = rawBus;
  for (auto& mode : rawModes) mode = RawNormal;
  rawTransactions.clear(); rawQueueLifecycle = false;
  if (resetQuota) { goalReadDetailsSeen = false; lastGoalReadDetailsUs = 0; }
}

motion::Frame rawRequest(uint8_t id = 11, uint16_t length = 1) {
  motion::Frame frame; frame.type = motion::OffRegisterDetails;
  frame.sequence = 0xfffffff0; frame.length = length; frame.payload[0] = id;
  return frame;
}

motion::Frame rawIssue(const motion::Frame& frame, bool accepted) {
  const auto before = feedback;
  uint8_t oldLock[sizeof(lock)], oldPresent[sizeof(presentHistory)], oldTorque[sizeof(torqueHistory)];
  uint8_t oldTrace[sizeof(lastServoReadTrace)], oldEvidence[sizeof(positionReadEvidence)];
  memcpy(oldLock, &lock, sizeof(lock)); memcpy(oldPresent, &presentHistory, sizeof(presentHistory));
  memcpy(oldTorque, &torqueHistory, sizeof(torqueHistory));
  memcpy(oldTrace, &lastServoReadTrace, sizeof(lastServoReadTrace));
  memcpy(oldEvidence, positionReadEvidence, sizeof(positionReadEvidence));
  const auto oldTorqueReads = std::vector<uint32_t>(torqueReadUs, torqueReadUs + 15);
  const size_t oldWrites = writes.size(), busBytes = Serial1.output.size();
  const bool oldDisable = disablePending, oldVerify = torqueVerifyPending;
  command(frame); const motion::Frame reply = lastResponse();
  assert(reply.sequence == frame.sequence);
  assert(memcmp(&lock, oldLock, sizeof(lock)) == 0);
  assert(feedback.sampleUs == before.sampleUs && feedback.imuUs == before.imuUs && feedback.imuOk == before.imuOk);
  assert(memcmp(feedback.positions, before.positions, sizeof(feedback.positions)) == 0);
  assert(feedback.faultMask == before.faultMask && feedback.torqueMask == before.torqueMask);
  assert((feedback.onlineMask & ~before.onlineMask) == 0);  // Natural expiry is retained.
  assert(memcmp(&presentHistory, oldPresent, sizeof(oldPresent)) == 0);
  assert(memcmp(&torqueHistory, oldTorque, sizeof(oldTorque)) == 0);
  assert(memcmp(positionReadEvidence, oldEvidence, sizeof(positionReadEvidence)) == 0);
  for (size_t i = 0; i < 15; ++i) assert(torqueReadUs[i] == oldTorqueReads[i]);
  assert(writes.size() == oldWrites && disablePending == oldDisable && torqueVerifyPending == oldVerify);
  if (!accepted) {
    assert(Serial1.output.size() == busBytes && rawTransactions.empty());
    assert(memcmp(&lastServoReadTrace, oldTrace, sizeof(oldTrace)) == 0);
  }
  return reply;
}

void rawRefusal(motion::Error error, uint8_t id = 11, uint16_t length = 1) {
  const bool seen = goalReadDetailsSeen; const uint32_t quota = lastGoalReadDetailsUs;
  rawTransactions.clear(); const auto input = Serial1.input;
  const auto reply = rawIssue(rawRequest(id, length), false);
  assert(reply.type == 0x80 && reply.length == 2 && reply.payload[0] == 23 && reply.payload[1] == error);
  assert(goalReadDetailsSeen == seen && lastGoalReadDetailsUs == quota && Serial1.input == input);
}

void testRawLayout() {
  const uint8_t expected[8][2] = {{0,6},{16,2},{33,1},{40,1},{42,6},{48,4},{62,2},{69,2}};
  assert(memcmp(kOffRegisterReadBlocks, expected, sizeof(expected)) == 0);
  rawOff(); const auto before = feedback;
  const auto reply = rawIssue(rawRequest(), true); const auto* p = reply.payload;
  assert(reply.type == 0x97 && reply.length == 202 && reply.length == motion::kOffRegisterDetailsSize);
  assert(p[0] == 1 && p[1] == 11 && p[2] == 1 && p[3] == 8);
  assert(motion::u16(p+4) == 2001 && motion::u16(p+6) == 1901);
  assert(!p[8] && !p[9] && !motion::u32(p+10) && !motion::u32(p+14));
  assert(motion::u32(p+18) == before.sampleUs && motion::u32(p+26) == before.sampleUs);
  assert(motion::u32(p+22) == before.imuUs && motion::u32(p+30) == before.imuUs);
  assert(motion::u16(p+34) == motion::kAllMotors && motion::u16(p+38) == motion::kAllMotors);
  assert(!motion::u16(p+36) && !motion::u16(p+40));
  assert(rawTransactions.size() == 8);
  for (size_t i = 0; i < 8; ++i) {
    const size_t offset = 42 + i * 20; const auto& tx = rawTransactions[i];
    assert(p[offset] == expected[i][0] && p[offset+1] == expected[i][1]);
    assert(tx[2] == 11 && tx[5] == expected[i][0] && tx[6] == expected[i][1]);
    assert(uint32_t(motion::u32(p+offset+6) - motion::u32(p+offset+2)) > 0);
    assert(p[offset+10] == motion::ValidNormalReply && !p[offset+11]);
    assert(p[offset+12] == expected[i][1] && p[offset+13] == 5);
    for (size_t j = 0; j < 6; ++j) assert(p[offset+14+j] == (j < expected[i][1] ? 255 : 0));
  }
  for (uint8_t id : motion::kMotorIds) {
    rawOff(); const auto r = rawIssue(rawRequest(id), true);
    assert(r.payload[1] == id && motion::kMotorIds[r.payload[2]] == id);
  }
}

void testRawRefusals() {
  for (uint16_t size : {uint16_t(0),uint16_t(2),uint16_t(256)}) { rawOff(); rawRefusal(motion::BadPacket,11,size); }
  for (uint8_t id : {uint8_t(0),uint8_t(15),uint8_t(254),uint8_t(255)}) { rawOff(); rawRefusal(motion::BadPacket,id); }
  for (bool* busy : {&lock.armed,&maintenanceMode,&preparingArm,&verifyingTorque,&disablePending,&torqueVerifyPending}) {
    rawOff(); *busy = true; rawRefusal(motion::BadState);
  }
  rawOff(); lock.armed = true; lock.cookie = 123; feedback.sampleUs = 0; rawRefusal(motion::BadState);
  rawOff(); lock.cookie = 123; rawRefusal(motion::BadState);
  rawOff(); feedback.torqueMask = 1; rawRefusal(motion::BadState);
  rawOff(); torqueKnown &= ~1; rawRefusal(motion::Stale);
  for (size_t i = 0; i < 15; ++i) { rawOff(); torqueReadUs[i] -= kTorqueFreshUs + 1; rawRefusal(motion::Stale); }
  rawOff(); feedback.sampleUs -= motion::kStateFreshUs + 1; rawRefusal(motion::Stale);
  rawOff(); servoSampleSequence = 0; rawRefusal(motion::Stale);
  // Exact existing age boundaries are not silently tightened or relaxed.
  rawOff(); feedback.sampleUs -= motion::kStateFreshUs;
  for (auto& at : torqueReadUs) at -= kTorqueFreshUs;
  rawIssue(rawRequest(), true);
}

void testRawReplyFailuresAndNoFreshness() {
  for (RawMode mode : {RawMissing,RawDeviceError,RawEmptyError,RawCorrupt,RawWrongId,
                       RawWrongSize,RawShortAck,RawHardwareError}) {
    rawOff(); for (auto& item : rawModes) item = mode;
    const auto r = rawIssue(rawRequest(), true); const auto* p = r.payload;
    assert(r.type == 0x97 && rawTransactions.size() == 8);
    for (size_t i = 0; i < 8; ++i) {
      const size_t offset = 42 + i * 20;
      assert(!(p[offset+13] & 4));
      if (mode == RawDeviceError || mode == RawEmptyError) {
        assert(p[offset+10] == motion::ValidDeviceErrorReply && p[offset+11] == 0x20 && (p[offset+13] & 1));
        assert(p[offset+12] == (mode == RawEmptyError ? 0 : kOffRegisterReadBlocks[i][1]));
      } else {
        assert(p[offset+10] == (mode == RawHardwareError ? motion::HardwareUartError : motion::NoValidReplyByDeadline));
        assert(!(p[offset+13] & 1) && !p[offset+11] && !p[offset+12]);
      }
      for (size_t j = p[offset+12]; j < 6; ++j) assert(!p[offset+14+j]);
    }
  }
  rawOff(); rawModes[0] = RawMissing;
  auto r = rawIssue(rawRequest(), true);
  assert(r.payload[52] == motion::NoValidReplyByDeadline && !(r.payload[55] & 4));
  for (size_t i = 1; i < 8; ++i) assert(r.payload[42+i*20+13] & 4);
  rawRefusal(motion::BadState);  // A failed block still consumes the attempt.
  rawOff(); Serial1.writeResultOverride = 3;
  r = rawIssue(rawRequest(), true);
  for (size_t i = 0; i < 8; ++i) assert(r.payload[42+i*20+10] == motion::ShortUartWrite);
  rawOff(); for (size_t i = 0; i < 1000; ++i) Serial1.input.push_back(0x55);
  r = rawIssue(rawRequest(), true); assert(rawTransactions.empty());
  for (size_t i = 0; i < 8; ++i) assert(r.payload[42+i*20+10] == motion::BusNotQuiet);
  rawOff(); presentMask &= ~2; feedback.onlineMask &= ~2;
  rawIssue(rawRequest(), true); assert(!(feedback.onlineMask & 2));
  rawOff(); torqueReadUs[1] -= kTorqueFreshUs - 1;
  r = rawIssue(rawRequest(), true); assert(!(feedback.onlineMask & 2));
  assert(motion::u32(r.payload+18) == motion::u32(r.payload+26));
}

void testSharedQuotaAndWrap() {
  for (uint32_t start : {uint32_t(0),uint32_t(0xfffffff0)}) {
    rawOff(start); rawIssue(rawRequest(), true); const uint32_t at = lastGoalReadDetailsUs;
    rawOff(at + motion::kGoalReadDetailsPeriodUs - 1, false); rawRefusal(motion::BadState,22);
    rawOff(at + motion::kGoalReadDetailsPeriodUs, false); rawIssue(rawRequest(22), true);
  }
  for (bool firstRaw : {false,true}) {
    rawOff(); auto first = rawRequest(); first.type = firstRaw ? motion::OffRegisterDetails : motion::GoalReadDetails;
    rawIssue(first, true); const uint32_t at = lastGoalReadDetailsUs;
    rawTransactions.clear(); auto second = rawRequest(21);
    second.type = firstRaw ? motion::GoalReadDetails : motion::OffRegisterDetails;
    auto r = rawIssue(second, false);
    assert(r.type == 0x80 && r.payload[0] == second.type && r.payload[1] == motion::BadState);
    rawOff(at + motion::kGoalReadDetailsPeriodUs, false);
    r = rawIssue(second, true); assert(r.type == (firstRaw ? 0x96 : 0x97));
  }
}

void testOriginalLifecycleAndQueuedCommands() {
  Serial1.onWrite = busWrite; Serial1.input.clear(); Serial1.scheduledInput.clear();
  Serial1.writeResultOverride = -1; Wire.registers[0x0f] = 0x6b; Wire.registers[0x1e] = 3;
  lock = motion::Interlock{}; maintenanceMode = preparingArm = verifyingTorque = false;
  setup(); mockMicros += 110000; readyFeedback();
  command(configuration(1)); assert(lastResponse().payload[1] == motion::Ok);
  command(armCommand(2)); assert(lastResponse().payload[1] == motion::Ok && lock.armed);
  motion::Frame target; target.type = motion::Target; target.length = 34; target.sequence = 3;
  motion::put32(target.payload,123);
  for (size_t i = 0; i < 15; ++i) motion::put16(target.payload+4+2*i,2150);
  command(target); assert(lastResponse().payload[1] == motion::Ok); lock.slew(micros()+20000);
  motion::Frame stop; stop.type = motion::Stop;
  command(stop); assert(lastResponse().payload[1] == motion::Ok && !lock.armed && !lock.cookie);
  sampleServos(); sampleImu();
  const uint16_t desired = lock.desired[1], goal = lock.goals[1];
  goalReadDetailsSeen = false; rawTransactions.clear(); Serial1.onWrite = rawBus; rawQueueLifecycle = true;
  const auto r = rawIssue(rawRequest(), true);
  assert(motion::u16(r.payload+4) == desired && motion::u16(r.payload+6) == goal);
  assert(!Serial.input.empty() && !lock.armed && !lock.cookie && !r.payload[8] && !r.payload[9]);
  const uint32_t quota = lastGoalReadDetailsUs;
  Serial1.onWrite = busWrite; serviceHost();
  assert(lastResponse().payload[1] == motion::Ok && lock.armed && lock.cookie == 123);
  assert(lock.desired[1] == normalPosition && lock.goals[1] == normalPosition);
  rawRefusal(motion::BadState);
  command(stop); assert(lastResponse().payload[1] == motion::Ok && !lock.armed);
  sampleServos(); sampleImu(); motion::Frame hello; hello.type = motion::Hello;
  command(hello); assert(lastResponse().type == 0x81);
  assert(goalReadDetailsSeen && lastGoalReadDetailsUs == quota);
  rawRefusal(motion::BadState);  // Neither STOP nor HELLO grants a new read.
}
}  // namespace

int main() {
  testRawLayout(); testRawRefusals(); testRawReplyFailuresAndNoFreshness();
  testSharedQuotaAndWrap(); testOriginalLifecycleAndQueuedCommands();
  puts("PASS OFF-only Type23: exact 202-byte raw blocks, all mapped IDs, high raw values, zero-bus refusals, failed replies, shared Type22 quota/wrap, no feedback promotion, unchanged ARM/TARGET/STOP and queued commands");
}
