#include <assert.h>
#include <stdio.h>
#include <string>
#include "mocks/Arduino.h"
#include "mocks/Wire.h"
uint32_t mockMicros = 1000;
HardwareSerial Serial, Serial1;
TwoWire Wire;
#include "../firmware/main.cpp"

const uint8_t expectedIds[15] = {10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34};
bool mockTorque[35] = {};
uint8_t mockServoFault[35] = {};
uint8_t mockKp[35] = {}, mockKd[35] = {};
uint8_t absentServo = 0, ignoresTorqueServo = 0, ignoresGainsServo = 0, absentGainsServo = 0;
uint8_t corruptChecksumServo = 0, wrongReplyIdServo = 0, uartErrorServo = 0;
uint8_t invalidTorqueValueServo = 0, invalidPositionServo = 0;
uint8_t absentPositionServo = 0, outsideRangePositionServo = 0;
bool errorOnlyReply = false;
bool delayGainsUntilStale = false;
bool delayedGainWriteAck = false;
bool corruptGainWriteAck = false;
uint8_t gainWriteAckError = 0;
uint32_t gainWriteReplyUntil = 0;
bool delayedTorqueWriteEcho = false;
bool corruptTorqueWriteEcho = false;
uint32_t torqueWriteReplyUntil = 0;
uint8_t absentEnabledTorqueServo = 0;
uint8_t invalidEnabledTorqueServo = 0;
bool receiveErrorOnTorqueOff = false;
bool queueArmInPositionCycle = false;
uint16_t normalPosition = 2047;
std::vector<std::string> writes;
std::vector<uint8_t> gainReads;
size_t imuBusReads = 0, imuDataReads = 0;

void busWrite(const uint8_t* packet, size_t count) {
  assert(count >= 6 && packet[0] == 0xff && packet[1] == 0xff);
  uint8_t checksum = 0; for (size_t i = 2; i < count; ++i) checksum += packet[i];
  assert(checksum == 0xff);
  const uint8_t id = packet[2], instruction = packet[4];
  if (instruction == 0x03) {
    assert(count == 8);
    if (packet[5] == 40) {
      assert(id == 0xfe);
      writes.push_back(packet[6] ? "torque1" : "torque0");
      if (!packet[6] && receiveErrorOnTorqueOff) uartError(1);
      for (uint8_t servoId : expectedIds) if (servoId != ignoresTorqueServo) mockTorque[servoId] = packet[6] != 0;
      if (packet[6] && delayedTorqueWriteEcho) {
        // TX completion does not mean that a delayed half-duplex echo has
        // finished. A READ sent during this window cannot receive a reply.
        const uint32_t begin = mockMicros + 300;
        for (size_t n = 0; n < count; ++n)
          Serial1.scheduledInput.emplace_back(begin + uint32_t(n * 10),
            uint8_t(packet[n] + (corruptTorqueWriteEcho && n + 1 == count ? 1 : 0)));
        torqueWriteReplyUntil = begin + uint32_t(count * 10);
      }
    } else {
      // All gain writes must be exact official volatile-RAM settings.
      assert((id == 0xfe && packet[5] == 50 && packet[6] == 6) ||
             (id == 0xfe && packet[5] == 51 && packet[6] == 20) ||
             (id == 34 && packet[5] == 50 && packet[6] == 10));
      for (uint8_t servoId : expectedIds) {
        assert(!mockTorque[servoId]);
        if (servoId == ignoresGainsServo || (id != 0xfe && id != servoId)) continue;
        if (packet[5] == 50) mockKp[servoId] = packet[6]; else mockKd[servoId] = packet[6];
      }
      writes.push_back(id == 34 ? "mouthkp10" : packet[5] == 50 ? "kp6" : "kd20");
      if (id == 34 && delayedGainWriteAck) {
        // Optional WRITE echo/status arrive after TX has completed. Sending
        // the next READ before that reply ends collides on a half-duplex bus.
        std::vector<uint8_t> bytes(packet, packet + count);
        std::vector<uint8_t> ack = {255,255,id,2,gainWriteAckError,0};
        ack.back() = uint8_t(~uint8_t(id + 2 + gainWriteAckError));
        if (corruptGainWriteAck) ++ack.back();
        bytes.insert(bytes.end(), ack.begin(), ack.end());
        const uint32_t begin = mockMicros + 300;
        for (size_t n = 0; n < bytes.size(); ++n)
          Serial1.scheduledInput.emplace_back(begin + uint32_t(n * 10), bytes[n]);
        gainWriteReplyUntil = begin + uint32_t(bytes.size() * 10);
      }
    }
    return;
  }
  if (instruction == 0x83) {
    assert(id == 0xfe && count == 113 && packet[5] == 42 && packet[6] == 6);
    for (size_t i = 0; i < 15; ++i) assert(packet[7 + i * 7] == expectedIds[i]);
    writes.push_back("goals"); return;
  }
  bool knownId = false; for (uint8_t servoId : expectedIds) knownId |= id == servoId;
  assert(instruction == 0x02 && knownId && count == 8);
  assert((packet[5] == 40 && packet[6] == 1) || (packet[5] == 56 && packet[6] == 6) ||
         (packet[5] == 50 && packet[6] == 2) || (packet[5] == 62 && packet[6] == 1));
  if (id == absentServo) return;
  if (packet[5] == 40 && ((mockTorque[id] && id == absentEnabledTorqueServo) ||
      (delayedTorqueWriteEcho && int32_t(mockMicros - torqueWriteReplyUntil) < 0))) return;
  if (packet[5] == 56 && id == absentPositionServo) return;
  if (id == uartErrorServo) uartError(1);
  if (packet[5] == 50) {
    gainReads.push_back(id);
    if (id == absentGainsServo) return;
    if (delayedGainWriteAck && int32_t(mockMicros - gainWriteReplyUntil) < 0) return;
  }
  // Echo followed by real reply verifies firmware's read parser on exact code.
  Serial1.input.insert(Serial1.input.end(), packet, packet + count);
  std::vector<uint8_t> reply = {0xff, 0xff, id, uint8_t(packet[6] + 2), mockServoFault[id]};
  if (errorOnlyReply && mockServoFault[id]) reply[3] = 2;
  else if (packet[5] == 40) reply.push_back(id == invalidTorqueValueServo ||
    (mockTorque[id] && id == invalidEnabledTorqueServo) ? 255 : mockTorque[id] ? 1 : 0);
  else if (packet[5] == 50) reply.insert(reply.end(), {mockKp[id], mockKd[id]});
  else if (packet[5] == 62) reply.push_back(71);
  else if (id == outsideRangePositionServo) reply.insert(reply.end(), {0xac, 0x0d, 100, 0x80, 0, 0}); // Valid 3500, outside config.
  else if (id == invalidPositionServo) reply.insert(reply.end(), {0, 0x10, 100, 0x80, 0, 0});  // Invalid position 4096.
  else reply.insert(reply.end(), {uint8_t(normalPosition), uint8_t(normalPosition >> 8), 100, 0x80, 0, 0});
  if (id == wrongReplyIdServo) reply[2] = uint8_t(id + 1);
  uint8_t sum = 0; for (size_t i = 2; i < reply.size(); ++i) sum += reply[i];
  reply.push_back(uint8_t(~sum));
  if (id == corruptChecksumServo) ++reply.back();
  Serial1.input.insert(Serial1.input.end(), reply.begin(), reply.end());
  if (packet[5] == 56 && id == 12 && queueArmInPositionCycle) {
    queueArmInPositionCycle = false; normalPosition = 2200;
    const uint8_t cookie[4] = {123,0,0,0}; uint8_t encoded[32];
    const size_t n = motion::encode(motion::Arm,12,cookie,sizeof(cookie),encoded);
    Serial.input.insert(Serial.input.end(),encoded,encoded+n);
  }
}
motion::Frame configuration(uint32_t seq) {
  motion::Frame frame; frame.type = motion::ConfigRequest; frame.sequence = seq; frame.length = 105;
  for (size_t i = 0; i < 15; ++i) {
    motion::put16(frame.payload + i * 2, 2047); frame.payload[30 + i] = 1;
    motion::put16(frame.payload + 45 + i * 2, 1000); motion::put16(frame.payload + 75 + i * 2, 3000);
  }
  return frame;
}
motion::Frame armCommand(uint32_t seq) {
  motion::Frame frame; frame.type = motion::Arm; frame.length = 4; frame.sequence = seq;
  motion::put32(frame.payload, 123); return frame;
}
motion::Frame lastResponse() {
  motion::Parser parser; motion::Frame last;
  for (uint8_t byte : Serial.output) if (parser.feed(byte)) last = parser.frame;
  Serial.output.clear(); return last;
}
void readyFeedback() { sampleImu(); sampleServos(); refreshTorque(); }
void assertHistoryPayload(const uint8_t* payload, const ServoReadHistory& history) {
  assert(motion::u16(payload) == history.attemptedMask);
  assert(motion::u16(payload + 2) == history.latestFoundMask);
  assert(motion::u16(payload + 4) == history.lastValidSeenMask);
  for (size_t i = 0; i < 15; ++i) {
    assert(payload[6 + i] == history.latestError[i]);
    assert(payload[21 + i] == history.lastValidError[i]);
    assert(payload[36 + i] == history.latestOutcome[i]);
    assert(motion::u32(payload + 51 + i * 4) == history.lastValidUs[i]);
  }
}
void testFaultDetailsExport() {
  motion::Frame request; request.type = motion::FaultDetails; request.sequence = 0xfffffff0;
  const size_t writesBefore = writes.size(), busBytesBefore = Serial1.output.size(), gainsBefore = gainReads.size();
  const bool armedBefore = lock.armed, configuredBefore = lock.configured;
  const uint32_t cookieBefore = lock.cookie;
  const uint32_t before = mockMicros;
  command(request); motion::Frame response = lastResponse();
  assert(response.type == 0x87 && response.sequence == request.sequence && response.length == 247);
  assert(response.payload[0] == 1 && response.payload[1] == 15);
  const uint32_t snapshot = motion::u32(response.payload + 2);
  assert(uint32_t(snapshot - before) < 20 && uint32_t(mockMicros - snapshot) < 20);
  assert(motion::u32(response.payload + 6) == servoErrors.load());
  assert(motion::u16(response.payload + 10) == feedback.onlineMask);
  assert(motion::u16(response.payload + 12) == feedback.faultMask);
  assert(motion::u16(response.payload + 14) == feedback.torqueMask);
  assert(motion::u16(response.payload + 16) == presentFaults);
  assert(motion::u16(response.payload + 18) == torqueFaults);
  assert(response.payload[20] == armedBefore && motion::u32(response.payload + 21) == cookieBefore);
  assertHistoryPayload(response.payload + 25, presentHistory);
  assertHistoryPayload(response.payload + 136, torqueHistory);
  assert(writes.size() == writesBefore && Serial1.output.size() == busBytesBefore && gainReads.size() == gainsBefore);
  assert(lock.armed == armedBefore && lock.cookie == cookieBefore && lock.configured == configuredBefore);
}
motion::Frame positionDetails(uint8_t id, uint16_t length = 1) {
  motion::Frame request; request.type = motion::PositionReadDetails;
  request.sequence = 0xfffffff2; request.length = length; request.payload[0] = id;
  const size_t writesBefore = writes.size(), busBytesBefore = Serial1.output.size();
  const auto inputBefore = Serial1.input;
  const bool armedBefore = lock.armed, configuredBefore = lock.configured;
  const uint32_t cookieBefore = lock.cookie;
  const motion::Error errorBefore = lock.lastError;
  const uint16_t onlineBefore = feedback.onlineMask, faultBefore = feedback.faultMask, torqueBefore = feedback.torqueMask;
  uint16_t goalsBefore[15], desiredBefore[15];
  memcpy(goalsBefore, lock.goals, sizeof(goalsBefore)); memcpy(desiredBefore, lock.desired, sizeof(desiredBefore));
  command(request); const motion::Frame response = lastResponse();
  assert(response.sequence == request.sequence);
  assert(writes.size() == writesBefore && Serial1.output.size() == busBytesBefore && Serial1.input == inputBefore);
  assert(lock.armed == armedBefore && lock.configured == configuredBefore && lock.cookie == cookieBefore);
  assert(lock.lastError == errorBefore && feedback.onlineMask == onlineBefore && feedback.faultMask == faultBefore && feedback.torqueMask == torqueBefore);
  assert(memcmp(lock.goals, goalsBefore, sizeof(goalsBefore)) == 0 && memcmp(lock.desired, desiredBefore, sizeof(desiredBefore)) == 0);
  return response;
}
void testPositionDetailsNotAttempted() {
  for (uint8_t id : expectedIds) {
    const motion::Frame response = positionDetails(id);
    assert(response.type == 0x95 && response.length == 22);
    assert(response.payload[0] == 1 && response.payload[1] == id && response.payload[2] == 56 && response.payload[3] == 6);
    for (size_t i = 4; i < 12; ++i) assert(response.payload[i] == 0);
    for (size_t i = 16; i < 22; ++i) assert(response.payload[i] == 0);
  }
}
void testPositionDetails() {
  constexpr uint8_t id = 22;
  constexpr uint16_t bit = 1u << 7;
  motion::Frame response = positionDetails(id);
  assert(response.type == 0x95 && response.length == 22 && response.payload[4] == 3);
  assert(response.payload[5] == 6 && response.payload[6] == 0 && response.payload[7] == motion::ValidNormalReply);
  assert(motion::u16(response.payload + 16) == normalPosition);
  assert(motion::u16(response.payload + 18) == 0x8064 && motion::u16(response.payload + 20) == 0);
  assert(uint32_t(motion::u32(response.payload + 12) - motion::u32(response.payload + 8)) < motion::kStateFreshUs);
  const uint32_t validAttempt = motion::u32(response.payload + 8);

  invalidPositionServo = id; sampleServos(); response = positionDetails(id);
  assert(response.payload[4] == 11 && response.payload[5] == 6 && response.payload[6] == 0);
  assert(response.payload[7] == motion::InvalidRegisterReply && motion::u16(response.payload + 16) == 4096);
  assert(motion::u32(response.payload + 8) != validAttempt);
  assert(!(feedback.onlineMask & bit) && !(presentMask & bit) && feedback.faultMask == 0);
  invalidPositionServo = 0;

  // The next miss must expose its own empty reply rather than stale raw bytes.
  absentPositionServo = id; sampleServos(); response = positionDetails(id);
  assert(response.payload[4] == 1 && response.payload[5] == 0 && response.payload[6] == 0);
  assert(response.payload[7] == motion::NoValidReplyByDeadline);
  for (size_t i = 16; i < 22; ++i) assert(response.payload[i] == 0);
  absentPositionServo = 0;

  mockServoFault[id] = 0x20; errorOnlyReply = true; sampleServos(); response = positionDetails(id);
  assert(response.payload[4] == 3 && response.payload[5] == 0 && response.payload[6] == 0x20);
  assert(response.payload[7] == motion::ValidDeviceErrorReply && (feedback.faultMask & bit));
  mockServoFault[id] = 0; errorOnlyReply = false; readyFeedback();

  corruptChecksumServo = id; sampleServos(); response = positionDetails(id);
  assert(response.payload[4] == 5 && response.payload[5] == 0 && response.payload[6] == 0);
  assert(response.payload[7] == motion::NoValidReplyByDeadline);
  corruptChecksumServo = 0; readyFeedback();

  // Diagnostic requests do not consume the write sequence or overwrite a
  // prior safety error, even when request shape or operating mode is invalid.
  for (const uint16_t length : {uint16_t(0), uint16_t(2), uint16_t(256)}) {
    response = positionDetails(id, length);
    assert(response.type == 0x80 && response.length == 2 && response.payload[0] == motion::PositionReadDetails && response.payload[1] == motion::BadPacket);
  }
  for (uint8_t badId : {uint8_t(0), uint8_t(15), uint8_t(254), uint8_t(255)}) {
    response = positionDetails(badId);
    assert(response.type == 0x80 && response.payload[1] == motion::BadPacket);
  }
  for (bool* mode : {&maintenanceMode, &preparingArm, &verifyingTorque}) {
    *mode = true; response = positionDetails(id); *mode = false;
    assert(response.type == 0x80 && response.payload[1] == motion::BadState);
  }
  lock.cookie = 123; response = positionDetails(id); lock.cookie = 0;
  assert(response.type == 0x80 && response.payload[1] == motion::BadState);

  // Timestamp zero is a valid attempt at wrap, not the never-attempted marker.
  const auto saved = positionReadEvidence[7];
  positionReadEvidence[7].attemptUs = 0; response = positionDetails(id);
  assert((response.payload[4] & 1) && motion::u32(response.payload + 8) == 0);
  positionReadEvidence[7] = saved;
}
void testPositionDetailsCannotAuthorizeArm() {
  motion::Frame hello; hello.type = motion::Hello; command(hello); lastResponse();
  command(configuration(1)); assert(lastResponse().payload[1] == motion::Ok);
  invalidPositionServo = 22; readyFeedback();
  motion::Frame response = positionDetails(22);
  assert(response.payload[7] == motion::InvalidRegisterReply && motion::u16(response.payload + 16) == 4096);
  const size_t writesBefore = writes.size(), gainsBefore = gainReads.size();
  command(armCommand(2)); response = lastResponse();
  assert(response.type == 0x80 && response.payload[1] == motion::Stale && !lock.armed && !lock.cookie);
  assert(writes.size() == writesBefore && gainReads.size() == gainsBefore);
  assert(!(feedback.onlineMask & (1u << 7)));
  invalidPositionServo = 0; readyFeedback();
}
void testFaultDetailsHistory() {
  constexpr size_t index = 3; constexpr uint8_t id = 13; constexpr uint16_t bit = 1u << index;
  mockServoFault[id] = 0x20; errorOnlyReply = true;
  refreshTorque(); sampleServos();
  assert((presentFaults & bit) && (torqueFaults & bit) && (feedback.faultMask & bit));
  for (const auto* history : {&presentHistory, &torqueHistory}) {
    assert((history->latestFoundMask & bit) && (history->lastValidSeenMask & bit));
    assert(history->latestError[index] == 0x20 && history->lastValidError[index] == 0x20);
    assert(history->latestOutcome[index] == motion::ValidDeviceErrorReply);
  }
  testFaultDetailsExport();
  const uint32_t presentLastValid = presentHistory.lastValidUs[index], torqueLastValid = torqueHistory.lastValidUs[index];
  absentServo = id; refreshTorque(); sampleServos();
  assert(!(presentFaults & bit) && (torqueFaults & bit) && (feedback.faultMask & bit));
  assert(!(feedback.onlineMask & bit) && !(feedback.torqueMask & bit));
  for (const auto* history : {&presentHistory, &torqueHistory}) {
    assert(!(history->latestFoundMask & bit) && (history->lastValidSeenMask & bit));
    assert(history->latestError[index] == 0 && history->lastValidError[index] == 0x20);
    assert(history->latestOutcome[index] == motion::NoValidReplyByDeadline);
  }
  assert(presentHistory.lastValidUs[index] == presentLastValid && torqueHistory.lastValidUs[index] == torqueLastValid);
  testFaultDetailsExport();
  absentServo = 0; mockServoFault[id] = 0; errorOnlyReply = false; readyFeedback();
  assert(presentFaults == 0 && torqueFaults == 0 && feedback.faultMask == 0);
  for (const auto* history : {&presentHistory, &torqueHistory}) {
    assert(history->latestFoundMask == 0x7fff && history->lastValidSeenMask == 0x7fff);
    assert(history->latestError[index] == 0 && history->lastValidError[index] == 0);
    assert(history->latestOutcome[index] == motion::ValidNormalReply);
  }
  assert(presentHistory.lastValidUs[index] != presentLastValid && torqueHistory.lastValidUs[index] != torqueLastValid);
  testFaultDetailsExport();

  // A zero timestamp is valid at micros() rollover; seen, not time, carries validity.
  ServoReadHistory wrapped; ServoReply reply; reply.found = true; reply.outcome = motion::ValidNormalReply;
  const uint32_t previousTime = mockMicros; mockMicros = 0; wrapped.record(index, reply, true); mockMicros = previousTime;
  assert(wrapped.lastValidUs[index] == 0 && (wrapped.lastValidSeenMask & bit));
}
void testInvalidRegisterDiagnostic() {
  constexpr size_t index = 3; constexpr uint8_t id = 13; constexpr uint16_t bit = 1u << index;
  invalidTorqueValueServo = id; refreshTorque();
  assert(!(torqueKnown & bit) && !(feedback.torqueMask & bit) && !(feedback.onlineMask & bit));
  assert(torqueFaults == 0 && feedback.faultMask == 0);
  assert((torqueHistory.latestFoundMask & bit) && (torqueHistory.lastValidSeenMask & bit));
  assert(torqueHistory.latestError[index] == 0 && torqueHistory.lastValidError[index] == 0);
  assert(torqueHistory.latestOutcome[index] == motion::InvalidRegisterReply);
  testFaultDetailsExport();
  invalidTorqueValueServo = 0; refreshTorque();
  assert(torqueHistory.latestOutcome[index] == motion::ValidNormalReply);
  invalidPositionServo = id; sampleServos();
  assert(!(presentMask & bit) && !(feedback.onlineMask & bit));
  assert(presentFaults == 0 && feedback.faultMask == 0);
  assert((presentHistory.latestFoundMask & bit) && (presentHistory.lastValidSeenMask & bit));
  assert(presentHistory.latestError[index] == 0 && presentHistory.lastValidError[index] == 0);
  assert(presentHistory.latestOutcome[index] == motion::InvalidRegisterReply);
  testFaultDetailsExport();
  invalidPositionServo = 0; readyFeedback();
  assert(feedback.onlineMask == 0x7fff && presentHistory.latestOutcome[index] == motion::ValidNormalReply);
}
void testReadOutcomeFailures() {
  constexpr uint8_t id = 13;
  auto read = []() { return readServo(id, 40, 1); };
  Serial1.input.assign(1000, 0x55); const size_t before = Serial1.output.size();
  ServoReply reply = read(); assert(!reply.found && reply.outcome == motion::BusNotQuiet);
  assert(Serial1.output.size() == before); Serial1.input.clear();
  Serial1.writeResultOverride = 7; reply = read();
  assert(!reply.found && reply.outcome == motion::ShortUartWrite);
  Serial1.writeResultOverride = -1; Serial1.input.clear();
  uartErrorServo = id; reply = read(); assert(!reply.found && reply.outcome == motion::HardwareUartError);
  uartErrorServo = 0; Serial1.input.clear();
  corruptChecksumServo = id; reply = read(); assert(!reply.found && reply.outcome == motion::NoValidReplyByDeadline);
  corruptChecksumServo = 0; Serial1.input.clear();
  wrongReplyIdServo = id; reply = read(); assert(!reply.found && reply.outcome == motion::NoValidReplyByDeadline);
  wrongReplyIdServo = 0; Serial1.input.clear();
  reply = read(); assert(reply.found && reply.error == 0 && reply.outcome == motion::ValidNormalReply);
}
void testImuSettlingAndLongUptime() {
  // A failed/restarted initialization must not retain a sampling permission.
  mockMicros = 1000000;
  assert(initImu());
  Wire.online = false; assert(!initImu());
  const size_t readsAfterFailure = imuBusReads;
  mockMicros += 100010; sampleImu();
  assert(imuBusReads == readsAfterFailure && !feedback.imuOk);
  Wire.online = true; assert(initImu());
  const uint32_t initializedAt = mockMicros;
  const uint32_t previousSample = feedback.imuUs;
  const size_t readsBeforeSettling = imuBusReads;
  const size_t dataReadsBeforeSettling = imuDataReads;
  mockMicros = initializedAt + 99990; sampleImu();
  assert(imuBusReads == readsBeforeSettling && feedback.imuUs == previousSample);
  mockMicros = initializedAt + 100010; sampleImu();
  assert(imuDataReads == dataReadsBeforeSettling + 1 && feedback.imuUs != previousSample && feedback.imuOk);
  assert(uint32_t(mockMicros - feedback.imuUs) < 10);

  // Keep returning new sensor data across the old signed-deadline boundary.
  // These observable read/timestamp checks fail on the previous exact firmware.
  const uint32_t settledAt = initializedAt + 100010;
  const uint32_t intervals[] = {0x7fffff00u, 0x80000100u, 0x90000000u, 0xfffffff0u};
  for (uint32_t interval : intervals) {
    const uint32_t before = feedback.imuUs;
    const size_t dataReadsBefore = imuDataReads;
    mockMicros = settledAt + interval; sampleImu();
    assert(imuDataReads == dataReadsBefore + 1 && feedback.imuUs != before);
    assert(feedback.imuOk && uint32_t(mockMicros - feedback.imuUs) < 10);
    state(0x12345678); const motion::Frame response = lastResponse();
    assert(response.type == 0x82 && response.payload[15] == 1);
  }

  // Start the 100 ms settling period before micros() rolls over to zero.
  mockMicros = 0xffff0000u; assert(initImu());
  const uint32_t initializedNearWrap = mockMicros;
  const size_t readsBeforeWrapSettling = imuBusReads;
  const uint32_t sampleBeforeWrapSettling = feedback.imuUs;
  mockMicros = initializedNearWrap + 60000; sampleImu();
  mockMicros = initializedNearWrap + 99990; sampleImu();
  assert(imuBusReads == readsBeforeWrapSettling && feedback.imuUs == sampleBeforeWrapSettling);
  const size_t dataReadsBeforeWrap = imuDataReads;
  mockMicros = initializedNearWrap + 100010; sampleImu();
  assert(imuDataReads == dataReadsBeforeWrap + 1 && feedback.imuUs != sampleBeforeWrapSettling);
  assert(feedback.imuOk && uint32_t(mockMicros - feedback.imuUs) < 10);
}
void refreshRecoveryProof(uint32_t now) {
  presentMask = torqueKnown = feedback.onlineMask = motion::kAllMotors;
  presentFaults = torqueFaults = feedback.faultMask = feedback.torqueMask = 0;
  feedback.sampleUs = now;
  for (size_t i = 0; i < motion::kMotorCount; ++i) torqueReadUs[i] = now;
}
void prepareRecoveryTest(uint32_t now) {
  mockMicros = now;
  lock.armed = false; lock.cookie = 0; lock.configured = true;
  lock.lastError = motion::Stale;
  maintenanceMode = preparingArm = verifyingTorque = disablePending = torqueVerifyPending = false;
  refreshRecoveryProof(now);
  i2cReady = imuConfigured = Wire.busReady = Wire.online = true;
  imuSettling = feedback.imuOk = false; imuAddress = 0x6a;
  feedback.imuUs = now - kImuRecoveryUs - 1;
  lastImuRecoveryUs = now - kImuRecoveryUs;
  Wire.beginResult = Wire.endResult = true;
  Wire.resetStuck = false;
  Wire.beginDelayUs = Wire.endDelayUs = 0;
}
void testSafeImuRecovery() {
  const motion::Frame configFrame = configuration(123);
  assert(lock.config.decode(configFrame.payload, configFrame.length));
  // Retain nontrivial calibration/goals and a fault latch across every
  // reinitialization. I2C repair is not a CONFIG, STOP, ARM, or TARGET action.
  lock.config.sign[4] = -1; lock.config.zero[4] = 2100;
  lock.desired[4] = 2200; lock.goals[4] = 2190;
  lock.continuousGoals[4] = 2190.5f; lock.lastCommandUs = 987654;
  const motion::Config savedConfig = lock.config;
  const size_t writesBefore = writes.size(), uartBefore = Serial1.output.size();
  auto assertNoMotion = [&]() {
    assert(writes.size() == writesBefore && Serial1.output.size() == uartBefore);
    assert(!lock.armed && lock.cookie == 0 && lock.configured && lock.lastError == motion::Stale);
    assert(memcmp(lock.config.zero, savedConfig.zero, sizeof(savedConfig.zero)) == 0);
    assert(memcmp(lock.config.sign, savedConfig.sign, sizeof(savedConfig.sign)) == 0);
    assert(memcmp(lock.config.minimum, savedConfig.minimum, sizeof(savedConfig.minimum)) == 0);
    assert(memcmp(lock.config.maximum, savedConfig.maximum, sizeof(savedConfig.maximum)) == 0);
    assert(lock.desired[4] == 2200 && lock.goals[4] == 2190 &&
           lock.continuousGoals[4] == 2190.5f && lock.lastCommandUs == 987654);
  };

  // Replugging power is not equivalent to restoring the ODR settings. The
  // status-register reads still succeed but never deliver a new sample.
  prepareRecoveryTest(5000000);
  assert(initImu()); mockMicros += 100010; sampleImu();
  assert(feedback.imuOk);
  const uint32_t lastSampleBeforePowerLoss = feedback.imuUs;
  Wire.online = false; sampleImu(); assert(!feedback.imuOk);
  Wire.powerCycle(); Wire.online = true;
  assert(Wire.registers[0x10] == 0 && Wire.registers[0x11] == 0 && Wire.registers[0x1e] == 0);
  mockMicros += 100010; sampleImu();
  assert(feedback.imuUs == lastSampleBeforePowerLoss && !feedback.imuOk);
  size_t beginBefore = Wire.beginCalls, endBefore = Wire.endCalls;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore && Wire.endCalls == endBefore);
  mockMicros = lastSampleBeforePowerLoss + kImuRecoveryUs + 1;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && Wire.endCalls == endBefore + 1);
  assert(i2cReady && imuConfigured && imuSettling && !feedback.imuOk);
  assert(Wire.beginSda == 8 && Wire.beginScl == 9 && Wire.beginFrequency == 400000 && Wire.timeOut == 2);
  assert(Wire.registers[0x10] == 0x48 && Wire.registers[0x11] == 0x44 &&
         Wire.registers[0x12] == 0x44 && Wire.registers[0x13] == 8 && Wire.registers[0x1e] == 3);
  const uint32_t reinitializedAt = imuSettleStartUs;
  const size_t dataReadsBefore = imuDataReads;
  mockMicros = reinitializedAt + 99990; sampleImu();
  assert(imuDataReads == dataReadsBefore && feedback.imuUs == lastSampleBeforePowerLoss);
  motion::put16(Wire.registers + 0x28, 1000);
  mockMicros = reinitializedAt + 100010; sampleImu();
  assert(imuDataReads == dataReadsBefore + 1 && feedback.imuOk &&
         feedback.imuUs != lastSampleBeforePowerLoss && accel[0] > 1.0f);
  assertNoMotion();

  // One failed read is recovered by the next ordinary sample, without a
  // bus reset or clearing the error latch.
  prepareRecoveryTest(8000000); feedback.imuUs = mockMicros; feedback.imuOk = true;
  beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
  Wire.online = false; sampleImu(); assert(!feedback.imuOk);
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  Wire.online = true; mockMicros += 2000; sampleImu();
  assert(feedback.imuOk); refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore && Wire.endCalls == endBefore); assertNoMotion();

  // Driver teardown failure must not call begin on the old driver. A
  // later explicit loop opportunity may retry after the same one-second
  // throttle; begin failures also leave sampling invalid until a full init.
  prepareRecoveryTest(10000000); Wire.endResult = false;
  beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
  maybeRecoverImu(mockMicros);
  assert(Wire.endCalls == endBefore + 1 && Wire.beginCalls == beginBefore);
  assert(!i2cReady && !imuConfigured && !feedback.imuOk && !imuSettling);
  Wire.endResult = true; Wire.beginResult = false;
  mockMicros = lastImuRecoveryUs + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.endCalls == endBefore + 2 && Wire.beginCalls == beginBefore + 1);
  assert(!Wire.busReady && !i2cReady && !imuConfigured && !feedback.imuOk);
  const uint32_t failedAt = lastImuRecoveryUs;
  const size_t readsWhileDriverFailed = imuBusReads;
  sampleImu(); assert(imuBusReads == readsWhileDriverFailed);
  Wire.beginResult = true; mockMicros = failedAt + kImuRecoveryUs - 1;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1);
  mockMicros = failedAt + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.endCalls == endBefore + 3 && Wire.beginCalls == beginBefore + 2 && imuSettling);
  mockMicros = imuSettleStartUs + 100010; sampleImu(); assert(feedback.imuOk); assertNoMotion();

  // A sensor can acknowledge every I2C read yet never finish SW_RESET.
  // Runtime repair must yield after its smaller 20 ms budget, retain false
  // health, and throttle from completion instead of blocking the host for
  // the boot initialization's 100 ms budget on every loop iteration.
  prepareRecoveryTest(14000000); Wire.resetStuck = true;
  const uint32_t stuckResetStarted = mockMicros;
  const size_t readsBeforeStuckReset = imuBusReads;
  beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
  maybeRecoverImu(mockMicros);
  assert(imuBusReads > readsBeforeStuckReset + 1 && (Wire.registers[0x12] & 1));
  assert(uint32_t(mockMicros - stuckResetStarted) >= kImuRecoveryResetUs &&
         uint32_t(mockMicros - stuckResetStarted) < kImuRecoveryResetUs + 1000);
  assert(Wire.beginCalls == beginBefore + 1 && Wire.endCalls == endBefore + 1 &&
         i2cReady && !imuConfigured && !imuSettling && !feedback.imuOk);
  const uint32_t stuckResetFinished = lastImuRecoveryUs;
  const size_t readsAfterStuckReset = imuBusReads;
  sampleImu(); assert(imuBusReads == readsAfterStuckReset && !feedback.imuOk);
  mockMicros = stuckResetFinished + kImuRecoveryUs - 1;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && Wire.endCalls == endBefore + 1);
  Wire.resetStuck = false; mockMicros = stuckResetFinished + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 2 && imuConfigured && imuSettling && !feedback.imuOk);
  mockMicros = imuSettleStartUs + 100010; sampleImu(); assert(feedback.imuOk); assertNoMotion();

  // The bounded runtime retry must not shorten the normal boot API's reset
  // budget. Both paths still refuse to publish a sample before reset clears.
  prepareRecoveryTest(17000000); Wire.resetStuck = true;
  const uint32_t bootResetStarted = mockMicros;
  assert(!initImu());
  assert(uint32_t(mockMicros - bootResetStarted) >= 100000 &&
         uint32_t(mockMicros - bootResetStarted) < 101000);
  assert(!imuConfigured && !feedback.imuOk); assertNoMotion();

  // Fresh telemetry does not supersede in-progress maintenance, ARM,
  // shutdown, or unknown/faulted torque. Each independent unsafe condition
  // must prevent any I2C reconstruction, even with an old IMU sample.
  const std::function<void()> unsafeConditions[] = {
    [] { lock.armed = true; }, [] { lock.cookie = 42; },
    [] { maintenanceMode = true; }, [] { preparingArm = true; },
    [] { verifyingTorque = true; }, [] { disablePending = true; },
    [] { torqueVerifyPending = true; }, [] { torqueKnown &= ~uint16_t(1); },
    [] { presentMask &= ~uint16_t(1); }, [] { presentFaults = 1; },
    [] { torqueFaults = 1; }, [] { feedback.torqueMask = 1; },
    [] { feedback.sampleUs = mockMicros - motion::kStateFreshUs - 1; },
    [] { torqueReadUs[0] = mockMicros - motion::kStateFreshUs - 1; },
    [] { torqueReadUs[14] = mockMicros - motion::kStateFreshUs - 1; },
    // updateMasks clears this old torque bit. That zero must still not be
    // interpreted as a real disabled-torque readback.
    [] { feedback.torqueMask = 1; torqueReadUs[0] = mockMicros - kTorqueFreshUs - 1; },
    [] { torqueKnown = 0; feedback.torqueMask = 0; },
    [] { imuSettling = true; },
    [] { feedback.imuUs = mockMicros - kImuRecoveryUs; },
    [] { lastImuRecoveryUs = mockMicros - kImuRecoveryUs + 1; }
  };
  for (const auto& condition : unsafeConditions) {
    prepareRecoveryTest(20000000); condition();
    beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
    maybeRecoverImu(mockMicros);
    assert(Wire.beginCalls == beginBefore && Wire.endCalls == endBefore);
    assert(writes.size() == writesBefore && Serial1.output.size() == uartBefore);
    assert(lock.lastError == motion::Stale && lock.configured);
  }
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    prepareRecoveryTest(21000000);
    torqueReadUs[i] = mockMicros - motion::kStateFreshUs - 1;
    beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
    maybeRecoverImu(mockMicros);
    assert(Wire.beginCalls == beginBefore && Wire.endCalls == endBefore); assertNoMotion();
  }

  // Exactly 50 ms old real torque/position readbacks remain accepted.
  prepareRecoveryTest(22000000);
  feedback.sampleUs -= motion::kStateFreshUs;
  for (size_t i = 0; i < motion::kMotorCount; ++i) torqueReadUs[i] -= motion::kStateFreshUs;
  beginBefore = Wire.beginCalls; maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && imuConfigured); assertNoMotion();

  // Retry spacing starts after teardown/initialization completes, including
  // a slow failed begin. It must not spin once per loop after a long call.
  prepareRecoveryTest(25000000); Wire.beginResult = false; Wire.beginDelayUs = 20000;
  const uint32_t slowStarted = mockMicros;
  maybeRecoverImu(mockMicros);
  assert(uint32_t(lastImuRecoveryUs - slowStarted) >= 20000);
  const uint32_t slowFinished = lastImuRecoveryUs;
  beginBefore = Wire.beginCalls; mockMicros = slowStarted + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore);
  mockMicros = slowFinished + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1); assertNoMotion();

  // Recovery timestamps, proof ages, and sensor settling all cross uint32
  // micros rollover without a signed-deadline failure or an early retry.
  prepareRecoveryTest(0xffff0000u);
  feedback.sampleUs -= motion::kStateFreshUs;
  for (size_t i = 0; i < motion::kMotorCount; ++i) torqueReadUs[i] -= motion::kStateFreshUs;
  beginBefore = Wire.beginCalls; maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && imuSettling);
  const uint32_t initializedNearWrap = imuSettleStartUs;
  mockMicros = initializedNearWrap + 99990; sampleImu(); assert(!feedback.imuOk);
  mockMicros = initializedNearWrap + 100010; sampleImu(); assert(feedback.imuOk);
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1); assertNoMotion();
  prepareRecoveryTest(0x00010000u);
  beginBefore = Wire.beginCalls; maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && imuSettling); assertNoMotion();

  // A completed full init and settling period is still necessary when the
  // sensor is genuinely absent; a successful bus begin is not IMU health.
  prepareRecoveryTest(30000000); Wire.online = false;
  beginBefore = Wire.beginCalls; maybeRecoverImu(mockMicros);
  assert(Wire.beginCalls == beginBefore + 1 && i2cReady && !imuConfigured && !feedback.imuOk);
  Wire.online = true;
  mockMicros = lastImuRecoveryUs + kImuRecoveryUs;
  refreshRecoveryProof(mockMicros); maybeRecoverImu(mockMicros);
  assert(imuConfigured && imuSettling && !feedback.imuOk);
  mockMicros = imuSettleStartUs + 100010; sampleImu(); assert(feedback.imuOk); assertNoMotion();

  // Exercise the real loop placement: maintenance must not retain the old
  // unguarded !imuConfigured retry, and normal off-mode cycles may recover
  // only after their real complete UART position/torque readbacks.
  prepareRecoveryTest(35000000); imuConfigured = false; maintenanceMode = true;
  maintenance.lastRequestUs = mockMicros; maintenance.state = 0;
  beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
  const size_t readsBeforeMaintenanceLoop = imuBusReads;
  loop();
  assert(maintenanceMode && Wire.beginCalls == beginBefore && Wire.endCalls == endBefore);
  assert(imuBusReads == readsBeforeMaintenanceLoop && !imuConfigured); assertNoMotion();
  prepareRecoveryTest(40000000); imuConfigured = false;
  beginBefore = Wire.beginCalls; endBefore = Wire.endCalls;
  loop();
  assert(Wire.beginCalls == beginBefore + 1 && Wire.endCalls == endBefore + 1);
  assert(torqueKnown == motion::kAllMotors && presentMask == motion::kAllMotors &&
         feedback.onlineMask == motion::kAllMotors && !feedback.torqueMask);
  assert(imuConfigured && imuSettling && !feedback.imuOk);
  assert(writes.size() == writesBefore && !lock.armed && !lock.cookie &&
         lock.configured && lock.lastError == motion::Stale);
}
int main() {
  Serial1.onWrite = busWrite;
  Wire.onRead = [](uint8_t reg) {
    ++imuBusReads;
    if (reg == 0x22) ++imuDataReads;
    if (delayGainsUntilStale && reg == 0x1e && !gainReads.empty() && gainReads.back() == 34) {
      mockMicros += motion::kStateFreshUs + 1; delayGainsUntilStale = false;
    }
  };
  Wire.registers[0x0f] = 0x6b; Wire.registers[0x1e] = 3;
  motion::put16(Wire.registers + 0x22, uint16_t(-100));
  motion::put16(Wire.registers + 0x28, 1000);
  testPositionDetailsNotAttempted();
  testFaultDetailsExport();
  assert(presentHistory.attemptedMask == 0 && torqueHistory.attemptedMask == 0);
  setup(); assert(Serial.baud == 921600 && Serial1.baud == 1000000);
  assert(writes.size() == 1 && writes[0] == "torque0");
  assert(!lock.armed && !lock.configured && feedback.torqueMask == 0);
  assert(Wire.registers[0x10] == 0x48 && Wire.registers[0x11] == 0x44 && Wire.registers[0x12] == 0x44);
  mockMicros += 100100;
  readyFeedback(); assert(feedback.onlineMask == 0x7fff && feedback.imuOk);
  assert(velocities[0] == -100 && fabsf(gyro[0] + 1.75f) < 0.0001f);
  assert(fabsf(accel[0] - 1.1964113f) < 0.0001f);
  testFaultDetailsHistory(); testInvalidRegisterDiagnostic(); testReadOutcomeFailures(); readyFeedback();
  testPositionDetails();

  // FaultDetails, even a malformed high-sequence request, cannot consume a
  // write sequence or perform servo transactions. CONFIG sequence 1 below
  // remains accepted after both diagnostics.
  state(77); const motion::Frame stateBeforeDetails = lastResponse();
  testFaultDetailsExport(); state(78); const motion::Frame stateAfterDetails = lastResponse();
  assert(stateBeforeDetails.length == 106 && stateAfterDetails.length == 106);
  assert(memcmp(stateBeforeDetails.payload, stateAfterDetails.payload, 106) == 0);
  motion::Frame invalidDetails; invalidDetails.type = motion::FaultDetails; invalidDetails.length = 1;
  invalidDetails.sequence = 0xfffffff1;
  const size_t beforeInvalidBus = Serial1.output.size(), beforeInvalidWrites = writes.size();
  command(invalidDetails); motion::Frame invalidReply = lastResponse();
  assert(invalidReply.type == 0x80 && invalidReply.sequence == invalidDetails.sequence && invalidReply.length == 2);
  assert(invalidReply.payload[0] == motion::FaultDetails && invalidReply.payload[1] == motion::BadPacket);
  assert(Serial1.output.size() == beforeInvalidBus && writes.size() == beforeInvalidWrites && !lock.armed && lock.cookie == 0);

  motion::Frame hello; hello.type = motion::Hello; command(hello);
  assert(lastResponse().type == 0x81 && writes.size() == 1);
  command(configuration(1)); assert(lastResponse().payload[1] == 0 && writes.size() == 1);
  testFaultDetailsExport(); command(invalidDetails); assert(lastResponse().payload[1] == motion::BadPacket);
  const size_t before = writes.size(); command(armCommand(2));
  motion::Frame response = lastResponse();
  assert(response.type == 0x80 && response.payload[1] == 0 && lock.armed && feedback.torqueMask == 0x7fff);
  assert(writes[before] == "kp6" && writes[before + 1] == "kd20" && writes[before + 2] == "mouthkp10");
  assert(writes[before + 3] == "goals" && writes[before + 4] == "torque1");
  testFaultDetailsExport();  // Armed diagnostic returns flags without changing goals, gains or torque.
  response = positionDetails(22);
  assert(response.type == 0x80 && response.payload[1] == motion::BadState && lock.armed);
  assert(gainReads.size() == 15);
  for (size_t i = 0; i < 15; ++i) {
    assert(gainReads[i] == expectedIds[i]);
    assert(mockKp[expectedIds[i]] == (expectedIds[i] == 34 ? 10 : 6) && mockKd[expectedIds[i]] == 20);
  }
  command(armCommand(2)); assert(lastResponse().payload[1] == motion::BadPacket);

  // HELLO during active session must not reset monotonic write protection.
  command(hello); lastResponse(); command(configuration(1));
  assert(lastResponse().payload[1] == motion::BadPacket);
  motion::Frame stop; stop.type = motion::Stop; stop.sequence = 0;
  command(stop); assert(lastResponse().payload[1] == 0 && !lock.armed && lock.cookie == 0);
  assert(writes.back() == "torque0" && feedback.torqueMask == 0);
  command(stop); assert(lastResponse().payload[1] == 0);  // STOP accepts duplicate/backward seq.
  stop.length = 1; command(stop); assert(lastResponse().payload[1] == motion::BadPacket && !lock.armed);

  // An enable broadcast that an individual servo ignores cannot report success.
  command(hello); lastResponse(); readyFeedback(); command(configuration(1)); lastResponse();
  ignoresTorqueServo = 21; command(armCommand(2)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && !lock.armed && lock.cookie == 0);
  assert(response.length == 22 && response.payload[3] == 9 && response.payload[4] == 21);
  assert(response.payload[5] == 40 && response.payload[6] == 1 && response.payload[9] == motion::ValidNormalReply);
  assert(response.payload[10] == 1 && response.payload[11] == 0);
  assert(writes.back() == "torque0"); ignoresTorqueServo = 0; refreshTorque();

  // Missing readback makes online/torque validity unknown, never optimistic.
  absentServo = 23; readyFeedback();
  assert(!(feedback.onlineMask & (1 << 8)) && !(feedback.torqueMask & (1 << 8)));
  command(armCommand(3)); assert(lastResponse().payload[1] == motion::Stale && !lock.armed);
  absentServo = 0; readyFeedback(); command(armCommand(4)); assert(lastResponse().payload[1] == 0 && lock.armed);
  mockServoFault[13] = 1; sampleServos();
  assert(!lock.armed && lock.cookie == 0 && (feedback.faultMask & (1 << 3)) && writes.back() == "torque0");
  mockServoFault[13] = 0; readyFeedback();

  // Missing new IMU data or expired target stream releases torque and does not resume.
  command(armCommand(5)); assert(lastResponse().payload[1] == 0 && lock.armed);
  mockMicros += motion::kWatchdogUs + 1; checkSafety(); flushDisable();
  assert(!lock.armed && lock.cookie == 0 && writes.back() == "torque0");
  readyFeedback(); assert(!lock.armed);

  // A servo rejecting a RAM-gain write prevents torque enable, including mouth.
  auto enableCount = []() { size_t count = 0; for (const auto& write : writes) count += write == "torque1"; return count; };
  const size_t enablesBeforeGainFailures = enableCount();
  ignoresGainsServo = 24; mockKp[24] = 5;
  command(armCommand(6)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && !lock.armed && lock.cookie == 0);
  assert(enableCount() == enablesBeforeGainFailures);
  ignoresGainsServo = 34; mockKp[34] = 6; readyFeedback();
  command(armCommand(7)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && !lock.armed && enableCount() == enablesBeforeGainFailures);
  ignoresGainsServo = 0; absentGainsServo = 31; readyFeedback();
  command(armCommand(8)); response = lastResponse();
  assert(response.payload[1] == motion::Stale && !lock.armed && enableCount() == enablesBeforeGainFailures);
  assert(response.length == 22 && response.payload[2] == 1 && response.payload[3] == 3);
  assert(response.payload[4] == 31 && response.payload[5] == 50 && response.payload[6] == 0);
  absentGainsServo = 0; readyFeedback(); delayGainsUntilStale = true;
  const uint32_t beforePreparationSample = feedback.sampleUs;
  command(armCommand(9)); response = lastResponse();
  // Valid preparation can age the cached pre-ARM positions. Obtain a new
  // complete position/IMU snapshot, rather than rejecting healthy hardware
  // because that earlier snapshot expired while gains were checked.
  assert(response.payload[1] == motion::Ok && lock.armed);
  assert(feedback.sampleUs != beforePreparationSample && feedback.fresh(micros()));
  assert(enableCount() == enablesBeforeGainFailures + 1);
  stop.length = 0; command(stop); assert(lastResponse().payload[1] == motion::Ok);
  delayGainsUntilStale = false; readyFeedback();

  // A valid old preflight must not hide loss/range changes discovered by
  // the fresh snapshot after preparation. Neither case may write goals or
  // enable torque, and both retain the exact preparation stage.
  auto goalCount = []() { size_t count = 0; for (const auto& write : writes) count += write == "goals"; return count; };
  const size_t goalsBeforeFreshFailures = goalCount(), enablesBeforeFreshFailures = enableCount();
  absentPositionServo = 12; command(armCommand(10)); response = lastResponse();
  assert(response.payload[1] == motion::Stale && response.length == 22 && response.payload[3] == 5);
  assert(!lock.armed && goalCount() == goalsBeforeFreshFailures && enableCount() == enablesBeforeFreshFailures);
  assert(!preparingArm); absentPositionServo = 0; readyFeedback();
  outsideRangePositionServo = 12; command(armCommand(11)); response = lastResponse();
  assert(response.payload[1] == motion::Range && response.length == 22 && response.payload[3] == 5);
  assert(!lock.armed && goalCount() == goalsBeforeFreshFailures && enableCount() == enablesBeforeFreshFailures);
  assert(!preparingArm); outsideRangePositionServo = 0; readyFeedback();

  // ARM can arrive between position reads. Its fresh nested snapshot must
  // not be overwritten by the older half-built outer position cycle.
  queueArmInPositionCycle = true; sampleServos(); response = lastResponse();
  assert(response.payload[1] == motion::Ok && lock.armed && !preparingArm);
  for (size_t i = 0; i < 15; ++i) assert(feedback.positions[i] == 2200 && lock.goals[i] == 2200);
  command(stop); assert(lastResponse().payload[1] == motion::Ok);
  normalPosition = 2047; readyFeedback();

  // The first gain READ must wait for the preceding WRITE's optional echo
  // and ACK. Readback remains mandatory, and neither writes nor ARM retry.
  delayedGainWriteAck = true; gainWriteReplyUntil = 0;
  const size_t gainWriteCountBefore = writes.size();
  command(armCommand(13)); response = lastResponse();
  assert(response.payload[1] == motion::Ok && lock.armed);
  assert(writes[gainWriteCountBefore] == "kp6" && writes[gainWriteCountBefore + 1] == "kd20" &&
         writes[gainWriteCountBefore + 2] == "mouthkp10");
  command(stop); assert(lastResponse().payload[1] == motion::Ok);
  readyFeedback(); corruptGainWriteAck = true;
  const size_t beforeRejectedWrite = enableCount();
  command(armCommand(14)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && response.payload[3] == 2 && !lock.armed);
  assert(enableCount() == beforeRejectedWrite);
  corruptGainWriteAck = false; gainWriteAckError = 1; readyFeedback();
  command(armCommand(15)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && response.payload[3] == 2 && !lock.armed);
  assert(enableCount() == beforeRejectedWrite);
  gainWriteAckError = 0; delayedGainWriteAck = false; gainWriteReplyUntil = 0;
  Serial1.input.clear(); Serial1.scheduledInput.clear(); readyFeedback();

  // Exactly the same ownership rule applies to the torque-enable broadcast:
  // delayed write echo must finish before the first torque verification READ.
  delayedTorqueWriteEcho = true; torqueWriteReplyUntil = 0;
  const size_t enablesBeforeDelayedTorque = enableCount();
  command(armCommand(16)); response = lastResponse();
  assert(response.payload[1] == motion::Ok && lock.armed);
  assert(enableCount() == enablesBeforeDelayedTorque + 1);
  command(stop); assert(lastResponse().payload[1] == motion::Ok);
  corruptTorqueWriteEcho = true; readyFeedback();
  command(armCommand(17)); response = lastResponse();
  assert(response.payload[1] == motion::BadState && response.payload[3] == 8 && !lock.armed);
  assert(enableCount() == enablesBeforeDelayedTorque + 2 && writes.back() == "torque0");
  corruptTorqueWriteEcho = false; delayedTorqueWriteEcho = false;
  torqueWriteReplyUntil = 0; readyFeedback();

  // A genuine missing torque readback still disarms after a single enable.
  // Keep the first failing ID/outcome even after STOP changes later readings.
  absentEnabledTorqueServo = 21;
  const size_t enablesBeforeMissingTorque = enableCount();
  command(armCommand(18)); response = lastResponse();
  assert(response.payload[1] == motion::Stale && !lock.armed && lock.cookie == 0);
  assert(response.length == 22 && response.payload[3] == 9 && response.payload[4] == 21);
  assert(response.payload[5] == 40 && response.payload[6] == 0 && response.payload[9] == motion::NoValidReplyByDeadline);
  assert(response.payload[10] == 1 && enableCount() == enablesBeforeMissingTorque + 1);
  assert(writes.back() == "torque0");
  absentEnabledTorqueServo = 0; readyFeedback();
  assert(feedback.torqueMask == 0 && feedback.onlineMask == motion::kAllMotors);
  invalidEnabledTorqueServo = 12;
  command(armCommand(19)); response = lastResponse();
  assert(response.payload[1] == motion::Stale && !lock.armed && lock.cookie == 0);
  assert(response.payload[3] == 9 && response.payload[4] == 12 && response.payload[6] == 1);
  assert(response.payload[8] == 1 && response.payload[9] == motion::InvalidRegisterReply);
  assert(response.payload[10] == 1 && response.payload[11] == 255 && writes.back() == "torque0");
  invalidEnabledTorqueServo = 0; readyFeedback();

  // An unpowered servo board can leave receive noise at boot. Disabling must
  // still transmit; optional RX traffic cannot block STOP or invent a torque
  // readback. The enable path keeps its strict echo/error validation.
  receiveErrorOnTorqueOff = true;
  assert(writeTorque(false));
  receiveErrorOnTorqueOff = false;

  motion::Frame request; request.type = motion::State; request.sequence = 10; command(request);
  response = lastResponse(); assert(response.type == 0x82 && response.length == 106);
  assert(motion::u16(response.payload + 8) == 0x7fff && response.payload[14] == 0 && response.payload[105] == 1);
  testImuSettlingAndLongUptime();
  testSafeImuRecovery();
  testPositionDetailsCannotAuthorizeArm();
  puts("PASS exact firmware handlers: boot disarm, IMU settings/scaling+one-time settling+long uptime+micros wrap+safe power/bus recovery+fresh torque gates+fault latch/no motion writes, torque readback, ARM gains+ID34+readback+freshness ordering, HELLO replay, unconditional STOP, missing-servo/fault/watchdog, unchanged STATE layout, read-only FaultDetails and cached PositionReadDetails exact layout+outcomes+residual-history+no bus transactions+no write sequence consumption+off-only guard+rejected raw position never authorizes ARM");
  return 0;
}
