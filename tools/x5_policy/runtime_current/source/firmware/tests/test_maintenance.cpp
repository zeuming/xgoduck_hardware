#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include "mocks/Arduino.h"
#include "mocks/Wire.h"
uint32_t mockMicros = 1000;
HardwareSerial Serial, Serial1;
TwoWire Wire;
#include "../firmware/main.cpp"

struct TestServo {
  bool present = false, torque = false, locked = true;
  uint16_t position = 2048, goal = 99;
  uint8_t error = 0, voltage = 74;
} testServos[254];
std::vector<std::vector<uint8_t>> transactions;
bool followGoals = true, ignoreUnlock = false, ignoreIdWrite = false, ignoreRelease = false, shortGoalWrite = false;
bool delayedWriteAck = false;
bool writeEcho = false, ignoreGoalWrite = false, ignoreEnableWrite = false;
bool goalAutoEnables = false;
uint8_t writeAckMode = 1;
uint8_t torquePreflightFault = 0;
bool goalReadBeforeTorque = false;
int corruptId = -1, shortIdWrite = -1, noReadAfterRename = -1, invalidTorqueId = -1;
uint32_t currentCookie = 100, currentSequence = 0;

void maintenanceBus(const uint8_t* packet, size_t count) {
  assert(count >= 6 && packet[0] == 255 && packet[1] == 255);
  uint8_t check = 0; for (size_t i = 2; i < count; ++i) check += packet[i]; assert(check == 255);
  transactions.emplace_back(packet, packet + count);
  const uint8_t id = packet[2], op = packet[4], address = packet[5];
  if (op == 3) {
    if (id == 254) {
      assert(address == 40 && count == 8 && packet[6] == 0);
      for (auto& servo : testServos) if (!ignoreRelease) servo.torque = false;
      return;
    }
    assert(id <= 253);
    if (!testServos[id].present) return;
    TestServo& servo = testServos[id];
    if (address == 40) {
      assert(count == 8 && packet[6] <= 1);
      if ((packet[6] && !ignoreEnableWrite) || (!packet[6] && !ignoreRelease)) servo.torque = packet[6];
    } else if (address == 42) {
      assert(count == 13 && !packet[8] && !packet[9] && !packet[10] && !packet[11]);
      if (!ignoreGoalWrite) {
        servo.goal = motion::u16(packet + 6);
        if (goalAutoEnables) servo.torque = true;
      }
      if (followGoals && servo.torque) servo.position = servo.goal;
      if (shortGoalWrite) Serial1.writeResultOverride = int(count - 1);
    } else if (address == 55) {
      assert(count == 8 && packet[6] <= 1);
      if (packet[6] || !ignoreUnlock) servo.locked = packet[6];
    } else if (address == 5) {
      assert(count == 8 && packet[6] <= 253 && !servo.locked && !servo.torque);
      if (!ignoreIdWrite) { testServos[packet[6]] = servo; servo.present = false; }
      if (id == shortIdWrite) Serial1.writeResultOverride = int(count - 1);
    } else assert(false);  // No gains/offset/other EEPROM writes via maintenance.
    if (writeEcho) Serial1.input.insert(Serial1.input.end(), packet, packet + count);
    if (delayedWriteAck && writeAckMode) {
      const uint8_t actual = writeAckMode == 3 ? uint8_t(id + 1) : id;
      const uint8_t error = writeAckMode == 4 ? 1 : 0;
      uint8_t ack[] = {255, 255, actual, 2, error, uint8_t(~uint8_t(actual + 2 + error))};
      if (writeAckMode == 2) ++ack[5];
      const size_t size = writeAckMode == 6 ? 5 : 6;
      for (size_t i = 0; i < size; ++i)
        Serial1.scheduledInput.emplace_back(mockMicros + 400 + uint32_t(i * 10), ack[i]);
      if (writeAckMode == 5)
        for (size_t i = 0; i < sizeof(ack); ++i)
          Serial1.scheduledInput.emplace_back(mockMicros + 500 + uint32_t(i * 10), ack[i]);
    }
    return;
  }
  Serial1.writeResultOverride = -1;
  assert(op == 2 && id <= 253 && count == 8);
  if (!testServos[id].present || id == noReadAfterRename) return;
  TestServo& servo = testServos[id];
  Serial1.input.insert(Serial1.input.end(), packet, packet + count); // Echo must not count as another servo.
  const uint8_t fault = address == 40 && goalReadBeforeTorque ? torquePreflightFault : 0;
  if (address == 42) goalReadBeforeTorque = true;
  if (fault) { goalReadBeforeTorque = false; torquePreflightFault = 0; }
  if (fault == 1) return;  // Complete request echo, but no status reply.
  if (fault == 4) { ++servoErrors; return; }
  std::vector<uint8_t> reply = {255, 255, id, uint8_t(servo.error ? 2 : packet[6] + 2), servo.error};
  if (!servo.error) {
    if (address == 5) { assert(packet[6] == 1); reply.push_back(id); }
    else if (address == 62) { assert(packet[6] == 1); reply.push_back(servo.voltage); }
    else if (address == 40) { assert(packet[6] == 1); reply.push_back(fault == 3 ? 2 : id == invalidTorqueId ? 2 : servo.torque); }
    else if (address == 55) { assert(packet[6] == 1); reply.push_back(servo.locked); }
    else if (address == 42) { assert(packet[6] == 2); reply.push_back(uint8_t(servo.goal)); reply.push_back(uint8_t(servo.goal >> 8)); }
    else if (address == 56) {
      assert(packet[6] == 6); reply.insert(reply.end(), {uint8_t(servo.position), uint8_t(servo.position >> 8), 0, 0, 0, 0});
    } else assert(false);
  }
  uint8_t sum = 0; for (size_t i = 2; i < reply.size(); ++i) sum += reply[i]; reply.push_back(uint8_t(~sum));
  if (id == corruptId || fault == 2) ++reply.back();
  if (fault == 5) reply.resize(5);
  if (fault == 6) {
    const uint8_t ack[] = {255, 255, id, 2, 0, uint8_t(~uint8_t(id + 2))};
    Serial1.input.insert(Serial1.input.end(), ack, ack + sizeof(ack));
  }
  if (delayedWriteAck) {
    for (size_t i = 0; i < reply.size(); ++i)
      Serial1.scheduledInput.emplace_back(mockMicros + 700 + uint32_t(i * 10), reply[i]);
  } else Serial1.input.insert(Serial1.input.end(), reply.begin(), reply.end());
}

motion::Frame result() {
  motion::Parser decode; motion::Frame last; bool found = false;
  for (uint8_t byte : Serial.output) if (decode.feed(byte)) { last = decode.frame; found = true; }
  Serial.output.clear(); assert(found); return last;
}
motion::Frame maintenanceRequest(uint8_t type, uint8_t id = 255) {
  motion::Frame frame; frame.type = type; frame.sequence = ++currentSequence;
  frame.length = (type == motion::MaintenanceRead || type == motion::MaintenanceRelease || type == motion::MaintenanceCenterCheck || type == motion::MaintenanceVoltageRead) ? 5 : 4;
  motion::put32(frame.payload, currentCookie); frame.payload[4] = id; return frame;
}
void expectAck(const motion::Frame& frame, motion::Error error) {
  command(frame); const auto reply = result();
  if (reply.type != 0x80 || reply.payload[1] != error)
    fprintf(stderr, "ACK request=%u seq=%u expected=%u got_type=%u got_error=%u\n", frame.type, frame.sequence, error, reply.type, reply.payload[1]);
  const bool diagnostic = error == motion::CenterGoalRead || error == motion::CenterTorqueOffRead || error == motion::CenterEnableRead;
  const size_t expectedSize = frame.type == motion::Arm && error != motion::Ok ? 22 : diagnostic ? motion::kCenterFailureAckSize : 2;
  assert(reply.type == 0x80 && reply.length == expectedSize && reply.sequence == frame.sequence && reply.payload[0] == frame.type && reply.payload[1] == error);
  if (expectedSize == 22) assert(reply.payload[2] == 1 && reply.payload[3] <= 9);
}
void resetScene() {
  maintenanceMode = false; maintenanceGuard.close(); maintenance = MaintenanceState{};
  invalidateMotionFeedback(); disablePending = torqueVerifyPending = false;
  ++currentCookie; currentSequence = 0;
  for (auto& servo : testServos) servo = TestServo{};
  followGoals = true; ignoreUnlock = ignoreIdWrite = ignoreRelease = shortGoalWrite = false;
  delayedWriteAck = false;
  writeEcho = ignoreGoalWrite = ignoreEnableWrite = false; writeAckMode = 1;
  goalAutoEnables = false;
  torquePreflightFault = 0; goalReadBeforeTorque = false;
  corruptId = shortIdWrite = noReadAfterRename = invalidTorqueId = -1;
  Serial.input.clear(); Serial.output.clear(); Serial1.input.clear(); Serial1.output.clear();
  Serial1.scheduledInput.clear();
  Serial1.writeResultOverride = -1; transactions.clear(); mockMicros += 2000;
}
void enter() { expectAck(maintenanceRequest(motion::MaintenanceEnter), motion::Ok); assert(maintenanceMode && !lock.configured); }
motion::Frame centerRequest(uint8_t id, uint16_t target, uint16_t minimum = 1900, uint16_t maximum = 2200) {
  auto frame = maintenanceRequest(motion::MaintenanceCenter, id); frame.length = 11;
  motion::put16(frame.payload + 5, target); motion::put16(frame.payload + 7, minimum); motion::put16(frame.payload + 9, maximum); return frame;
}
motion::Frame renameRequest(uint8_t oldId, uint8_t newId) {
  auto frame = maintenanceRequest(motion::MaintenanceRename, oldId); frame.length = 6; frame.payload[5] = newId; return frame;
}
size_t countWrites(uint8_t address, int id = -1) {
  size_t count = 0;
  for (const auto& tx : transactions) count += tx[4] == 3 && tx[5] == address && (id < 0 || tx[2] == id);
  return count;
}
void poll() { command(maintenanceRequest(motion::MaintenancePoll)); assert(result().type == 0x8d); }
void tick() { mockMicros += kMaintenanceStepUs; maintenanceTick(); }

void testCapabilitiesAndIsolation() {
  resetScene(); motion::Frame caps; caps.type = motion::MaintenanceCaps; command(caps);
  auto reply = result();
  assert(reply.type == 0x88 && reply.length == 15 && memcmp(reply.payload, "XGODUCK_MAINT_1", 15) == 0 && transactions.empty());
  caps.length = 1; expectAck(caps, motion::BadPacket);
  motion::Frame hello; hello.type = motion::Hello; command(hello); reply = result();
  assert(reply.type == 0x81 && reply.length == 16 && memcmp(reply.payload, "XGODUCK_MOTION_1", 16) == 0);
  auto enterFrame = maintenanceRequest(motion::MaintenanceEnter);
  lock.armed = true; lock.cookie = 7;
  // Use a fresh, valid armed motion sample so normal safety checks do not
  // disarm before the entry precondition is evaluated.
  presentMask = torqueKnown = motion::kAllMotors; feedback.onlineMask = feedback.torqueMask = motion::kAllMotors;
  feedback.imuOk = true; feedback.sampleUs = feedback.imuUs = lock.lastCommandUs = mockMicros;
  for (auto& stamp : torqueReadUs) stamp = mockMicros;
  expectAck(enterFrame, motion::BadState); assert(!maintenanceMode);
  lock.disarm(); lock.configured = true; enter();
  for (uint8_t type : {motion::ConfigRequest, motion::Arm, motion::Target}) {
    motion::Frame forbidden; forbidden.type = type; expectAck(forbidden, motion::BadState);
  }
  motion::Frame stateFrame; stateFrame.type = motion::State; command(stateFrame); reply = result();
  assert(reply.type == 0x82 && reply.length == 106 && !motion::u16(reply.payload + 8) && !reply.payload[14] && !reply.payload[105]);
  const size_t before = transactions.size();
  mockMicros += 20000; loop(); assert(transactions.size() == before); // No fixed 15-servo background scanner.
  auto read = maintenanceRequest(motion::MaintenanceRead, 12); command(read); result();
  command(hello); result(); // Must not reset the maintenance write guard.
  const size_t afterRead = transactions.size(); expectAck(read, motion::BadPacket); assert(transactions.size() == afterRead);
  auto wrongCookie = maintenanceRequest(motion::MaintenanceRelease, 12); motion::put32(wrongCookie.payload, currentCookie + 1);
  expectAck(wrongCookie, motion::BadPacket); assert(transactions.size() == afterRead);
  expectAck(maintenanceRequest(motion::MaintenanceExit), motion::Ok);
  expectAck(maintenanceRequest(motion::MaintenanceEnter), motion::BadPacket); // Burned nonce cannot reopen.
}

void testReadResultsAndFaultLatch() {
  resetScene(); enter(); testServos[12].present = true;
  auto read = maintenanceRequest(motion::MaintenanceRead, 12); command(read); auto reply = result();
  assert(reply.type == 0x8a && reply.length == 10 && motion::u32(reply.payload) == currentCookie && reply.payload[4] == 12 &&
         reply.payload[5] == 0 && !reply.payload[6] && motion::u16(reply.payload + 7) == 2048 && reply.payload[9] == 0);
  testServos[12].position = 2055;
  command(maintenanceRequest(motion::MaintenancePoll)); reply = result();
  assert(reply.payload[4] == 12 && motion::u16(reply.payload + 7) == 2055); // Poll is an actual new sample.
  command(maintenanceRequest(motion::MaintenanceRead, 13)); reply = result();
  assert(reply.payload[5] == 1 && motion::u16(reply.payload + 7) == 0xffff && reply.payload[9] == 255);
  const size_t beforeEmptyPoll = transactions.size();
  command(maintenanceRequest(motion::MaintenancePoll)); reply = result();
  assert(reply.payload[4] == 255 && reply.payload[5] == 0 && motion::u16(reply.payload + 7) == 0xffff && reply.payload[9] == 255);
  assert(transactions.size() == beforeEmptyPoll);
  testServos[12].error = 1;
  command(maintenanceRequest(motion::MaintenanceRead, 12)); reply = result();
  assert(reply.payload[5] == 2 && reply.payload[6] == 1 && maintenance.faultSeen);
  testServos[12].error = 0;
  const size_t writesBefore = countWrites(42) + countWrites(5);
  expectAck(centerRequest(12, 2100), motion::BadState);
  expectAck(renameRequest(12, 99), motion::BadState);
  assert(countWrites(42) + countWrites(5) == writesBefore);
  expectAck(maintenanceRequest(motion::MaintenanceRelease, 12), motion::Ok); // Release remains available.

  resetScene(); enter(); testServos[12].present = true; corruptId = 12;
  command(maintenanceRequest(motion::MaintenanceRead, 12)); reply = result(); assert(reply.payload[5] == 3);
  corruptId = -1; invalidTorqueId = 12;
  command(maintenanceRequest(motion::MaintenanceRead, 12)); reply = result(); assert(reply.payload[5] == 3 && reply.payload[9] == 255);
  const size_t before = transactions.size(); expectAck(maintenanceRequest(motion::MaintenanceRead, 254), motion::BadPacket);
  assert(transactions.size() == before);
}

void testVoltageReadOnlyExtension() {
  resetScene();
  motion::Frame caps; caps.type = motion::MaintenanceVoltageCaps;
  command(caps); auto reply = result();
  assert(reply.type == 0x91 && reply.length == 17 &&
         memcmp(reply.payload, "XGODUCK_VOLTAGE_1", 17) == 0 && transactions.empty());
  caps.length = 1; expectAck(caps, motion::BadPacket);
  expectAck(maintenanceRequest(motion::MaintenanceVoltageRead, 12), motion::BadPacket);
  enter(); testServos[12].present = true;
  for (uint8_t raw : {0, 74, 255}) {
    testServos[12].voltage = raw;
    const auto read = maintenanceRequest(motion::MaintenanceVoltageRead, 12);
    const size_t before = transactions.size();
    command(read); reply = result();
    assert(reply.type == 0x92 && reply.length == 8 && reply.sequence == read.sequence &&
           motion::u32(reply.payload) == currentCookie && reply.payload[4] == 12 &&
           reply.payload[5] == 0 && reply.payload[6] == 0 && reply.payload[7] == raw);
    assert(transactions.size() == before + 1 && transactions.back()[4] == 2 &&
           transactions.back()[5] == 62 && transactions.back()[6] == 1);
    assert(maintenance.id == 255 && !testServos[12].torque && !lock.armed);
    expectAck(read, motion::BadPacket); assert(transactions.size() == before + 1);
  }
  auto wrong = maintenanceRequest(motion::MaintenanceVoltageRead, 12);
  motion::put32(wrong.payload, currentCookie + 1);
  const size_t before = transactions.size();
  expectAck(wrong, motion::BadPacket);
  expectAck(maintenanceRequest(motion::MaintenanceVoltageRead, 254), motion::BadPacket);
  assert(transactions.size() == before);
  command(maintenanceRequest(motion::MaintenanceVoltageRead, 13)); reply = result();
  assert(reply.payload[5] == 1 && reply.payload[6] == 0 && reply.payload[7] == 0);
  testServos[12].error = 1;
  command(maintenanceRequest(motion::MaintenanceVoltageRead, 12)); reply = result();
  assert(reply.payload[5] == 2 && reply.payload[6] == 1 && reply.payload[7] == 0 && maintenance.faultSeen);
  testServos[12].error = 0;
  expectAck(centerRequest(12, 2100), motion::BadState);
  corruptId = 12;
  command(maintenanceRequest(motion::MaintenanceVoltageRead, 12)); reply = result();
  assert(reply.payload[5] == 3 && reply.payload[7] == 0);
  corruptId = -1; maintenance.state = 1; maintenance.startedUs = maintenance.stepUs = mockMicros;
  const size_t movingBefore = transactions.size();
  expectAck(maintenanceRequest(motion::MaintenanceVoltageRead, 12), motion::BadState);
  assert(transactions.size() == movingBefore);
  for (const auto& tx : transactions) assert(tx[4] == 2); // No goals, torque, ID or EEPROM writes.
}

void testMidCycleEntryAndRollover() {
  resetScene();
  for (uint8_t id : motion::kMotorIds) testServos[id].present = true;
  const auto enterFrame = maintenanceRequest(motion::MaintenanceEnter);
  uint8_t bytes[64]; const size_t count = motion::encode(enterFrame.type, enterFrame.sequence, enterFrame.payload, enterFrame.length, bytes);
  Serial.input.insert(Serial.input.end(), bytes, bytes + count);
  sampleServos(); const auto reply = result();
  assert(reply.type == 0x80 && reply.payload[1] == motion::Ok && maintenanceMode);
  assert(transactions.size() == 1 && transactions[0][2] == 10);
  assert(feedback.onlineMask == 0 && presentMask == 0 && !lock.configured);

  resetScene(); mockMicros = 0xfffffff0; enter(); testServos[12].present = true;
  expectAck(centerRequest(12, 2080), motion::Ok);
  tick(); poll(); assert(maintenanceMode && maintenance.state == 1);
  mockMicros += kMaintenanceWatchdogUs + 1; maintenanceTick();
  assert(!maintenanceMode && !testServos[12].torque);
}

void testCenterAndStop() {
  resetScene(); enter(); testServos[12].present = testServos[10].present = true;
  expectAck(centerRequest(12, 2201), motion::Range);
  expectAck(centerRequest(12, 2100, 2050, 2200), motion::Range);
  assert(countWrites(42) == 0 && countWrites(40) == 0);
  const auto center = centerRequest(12, 2080); expectAck(center, motion::Ok);
  assert(maintenance.state == 1 && testServos[12].torque && !testServos[10].torque);
  assert(testServos[12].goal == 2048); // Holds measured position before the first tick.
  uint16_t lastGoal = 2048;
  for (unsigned i = 0; i < 10 && maintenance.state == 1; ++i) {
    tick(); assert(abs(int(testServos[12].goal) - int(lastGoal)) <= 8); lastGoal = testServos[12].goal; poll();
  }
  assert(maintenance.state == 2 && testServos[12].position == 2080 && !testServos[12].torque && maintenance.torque == 0);
  assert(countWrites(40, 254) == 0 && countWrites(40, 10) == 0 && countWrites(42, 10) == 0);
  assert(countWrites(5) == 0 && countWrites(55) == 0);
  const size_t beforeReplay = transactions.size(); expectAck(center, motion::BadPacket); assert(transactions.size() == beforeReplay);

  expectAck(centerRequest(12, 2150), motion::Ok);
  testServos[12].position += 100; tick(); assert(maintenance.state == 3 && !testServos[12].torque && !lock.configured);
  expectAck(centerRequest(12, 2100), motion::Ok); // No device-error latch; still requires a fresh valid snapshot.
  motion::Frame stop; stop.type = motion::Stop; stop.length = 1;
  expectAck(stop, motion::BadPacket); assert(!maintenanceMode && !testServos[12].torque && !lock.configured && !maintenanceGuard.cookie());
}

void testCenterWithDelayedWriteAck() {
  resetScene(); enter(); testServos[31].present = true;
  testServos[31].position = 2036; delayedWriteAck = writeEcho = goalAutoEnables = true;
  expectAck(centerRequest(31, 2056, 1054, 2879), motion::Ok);
  for (unsigned i = 0; i < 10 && maintenance.state == 1; ++i) { tick(); poll(); }
  assert(maintenance.state == 2 && testServos[31].position == 2056 && !testServos[31].torque);

  // Return-level settings can omit WRITE ACK, but readback remains mandatory.
  resetScene(); enter(); testServos[31].present = true;
  delayedWriteAck = writeEcho = true; writeAckMode = 0;
  expectAck(centerRequest(31, 2056, 1054, 2879), motion::Ok);
  expectAck(maintenanceRequest(motion::MaintenanceRelease, 31), motion::Ok);

  // An echo is not an ACK, and malformed/foreign/duplicate/error/partial
  // status responses abort before any torque-enable write, even if the
  // subsequent register would otherwise read back correctly.
  for (uint8_t mode : {2, 3, 4, 5, 6}) {
    resetScene(); enter(); testServos[31].present = true;
    delayedWriteAck = writeEcho = goalAutoEnables = true; writeAckMode = mode;
    expectAck(centerRequest(31, 2056, 1054, 2879), motion::CenterGoalWrite);
    for (const auto& tx : transactions) assert(!(tx[4] == 3 && tx[5] == 40 && tx[6] == 1));
    assert(maintenance.state == 3 && !testServos[31].torque && countWrites(42) == 1);
    if (mode == 4) assert(maintenance.error == 1 && maintenance.faultSeen);
  }

  resetScene(); enter(); testServos[31].present = true; ignoreGoalWrite = true;
  expectAck(centerRequest(31, 2056, 1054, 2879), motion::CenterGoalRead);
  assert(!testServos[31].torque);
  resetScene(); enter(); testServos[31].present = true; ignoreEnableWrite = true;
  expectAck(centerRequest(31, 2056, 1054, 2879), motion::CenterEnableRead);
  assert(!testServos[31].torque);
  resetScene(); enter(); testServos[31].present = true; invalidTorqueId = 31;
  expectAck(centerRequest(31, 2056, 1054, 2879), motion::CenterSnapshot);
  assert(countWrites(42) == 0 && countWrites(40) == 0);
}

void testCenterFailedReadEvidence() {
  for (uint8_t mode : {1, 2, 3, 4, 5, 6}) {
    resetScene(); enter(); testServos[31].present = true;
    testServos[31].position = 2185; torquePreflightFault = mode;
    const auto request = centerRequest(31, 2205, 1054, 2879);
    command(request); const auto reply = result();
    assert(reply.type == 0x80 && reply.length == motion::kCenterFailureAckSize);
    const uint8_t* d = reply.payload;
    assert(d[0] == motion::MaintenanceCenter && d[1] == motion::CenterTorqueOffRead);
    assert(d[2] == 1 && d[3] == 31 && d[4] == 40 && d[5] == 1 && motion::u16(d + 6) == 0);
    // The release read succeeds, but must not overwrite the failing read.
    assert(maintenance.state == 3 && maintenance.torque == 0 && !testServos[31].torque);
    assert(lastServoReadTrace.reply.found && !lastServoReadTrace.reply.invalidReplySeen);
    for (const auto& tx : transactions) assert(!(tx[4] == 3 && tx[5] == 40 && tx[6] == 1));
    assert(countWrites(42) == 1 && countWrites(5) == 0);
    if (mode == 1) assert(!d[8] && !d[9] && d[10] == motion::NoValidReplyByDeadline && motion::u16(d + 33) == 8);
    if (mode == 2 || mode == 5) assert(!d[8] && d[9] && d[10] == motion::NoValidReplyByDeadline);
    if (mode == 3) assert(d[8] && !d[9] && d[10] == motion::ValidNormalReply && d[12] == 1 && d[13] == 2);
    if (mode == 4) assert(!d[8] && d[10] == motion::HardwareUartError && motion::u32(d + 27) == motion::u32(d + 23) + 1);
    if (mode == 6) assert(d[8] && d[9] && d[10] == motion::ValidNormalReply && d[13] == 0);
    assert(d[36] == motion::u16(d + 33));
    for (size_t i = 53 + d[36]; i < reply.length; ++i) assert(!d[i]);
    if (mode == 6 && getenv("XGODUCK_CENTER_TRACE_FIXTURE")) {
      FILE* file = fopen(getenv("XGODUCK_CENTER_TRACE_FIXTURE"), "wb"); assert(file);
      assert(fwrite(d, 1, reply.length, file) == reply.length); assert(fclose(file) == 0);
    }
  }
}

void testGoalAutoEnableCenter() {
  // Reproduce the captured hardware behavior: goal write changes torque 0->1.
  resetScene(); enter(); testServos[31].present = testServos[10].present = true;
  testServos[31].position = 2185; goalAutoEnables = true;
  expectAck(centerRequest(31, 2205, 1054, 2879), motion::Ok);
  assert(maintenance.state == 1 && testServos[31].torque && !testServos[10].torque);
  assert(testServos[31].goal == 2185 && testServos[31].position == 2185);
  assert(countWrites(40) == 0); // Already holding; no redundant explicit enable.
  uint16_t previous = 2185;
  for (unsigned i = 0; i < 10 && maintenance.state == 1; ++i) {
    tick(); assert(abs(int(testServos[31].goal) - int(previous)) <= 8);
    previous = testServos[31].goal; poll();
  }
  assert(maintenance.state == 2 && testServos[31].position == 2205 && !testServos[31].torque);
  assert(countWrites(40, 31) == 1 && countWrites(40, 10) == 0 && countWrites(42, 10) == 0);
  assert(countWrites(5) == 0 && countWrites(55) == 0);

  // Auto-enabled holding must still be released if its goal cannot be verified.
  resetScene(); enter(); testServos[31].present = true; goalAutoEnables = true;
  shortGoalWrite = true;
  expectAck(centerRequest(31, 2205, 1054, 2879), motion::CenterGoalWrite);
  assert(maintenance.state == 3 && !testServos[31].torque);
}

void testCurrentPositionCenterCheck() {
  for (bool autoEnable : {false, true}) {
    resetScene(); enter(); testServos[31].present = testServos[10].present = true;
    testServos[31].position = 2185; goalAutoEnables = autoEnable;
    const auto frame = maintenanceRequest(motion::MaintenanceCenterCheck, 31);
    expectAck(frame, motion::Ok);
    assert(testServos[31].position == 2185 && testServos[31].goal == 2185);
    assert(!testServos[31].torque && !testServos[10].torque && maintenance.state == 0 && maintenance.torque == 0);
    tick(); poll(); assert(testServos[31].position == 2185);
    assert(countWrites(42) == 1 && countWrites(40, 31) == 1 && countWrites(40, 10) == 0 && countWrites(5) == 0 && countWrites(55) == 0);
    for (const auto& tx : transactions) assert(!(tx[4] == 3 && tx[5] == 40 && tx[6] == 1));
    const size_t beforeReplay = transactions.size(); expectAck(frame, motion::BadPacket);
    assert(transactions.size() == beforeReplay);
  }

  resetScene(); enter(); testServos[31].present = true; torquePreflightFault = 3;
  expectAck(maintenanceRequest(motion::MaintenanceCenterCheck, 31), motion::CenterTorqueOffRead);
  assert(!testServos[31].torque && maintenance.state == 3);
  for (const auto& tx : transactions) assert(!(tx[4] == 3 && tx[5] == 40 && tx[6] == 1));

  resetScene(); enter(); testServos[31].present = true; testServos[31].torque = true;
  expectAck(maintenanceRequest(motion::MaintenanceCenterCheck, 31), motion::CenterSnapshot);
  assert(countWrites(42) == 0 && countWrites(40) == 0);

  resetScene(); enter(); testServos[31].present = true;
  goalAutoEnables = ignoreRelease = true;
  expectAck(maintenanceRequest(motion::MaintenanceCenterCheck, 31), motion::BadState);
  assert(testServos[31].torque && maintenance.state == 3 && maintenance.torque == kMaintenanceUnknownTorque);
  assert(countWrites(42) == 1 && countWrites(40) == 1); // No release retry or ramp.
}

void testWatchdogsAndWriteFailure() {
  resetScene(); enter(); testServos[12].present = true;
  expectAck(centerRequest(12, 2200), motion::Ok);
  const uint32_t requestTime = maintenance.lastRequestUs;
  auto invalid = maintenanceRequest(motion::MaintenancePoll); motion::put32(invalid.payload, currentCookie + 1);
  expectAck(invalid, motion::BadPacket); assert(maintenance.lastRequestUs == requestTime);
  mockMicros += kMaintenanceWatchdogUs + 1; maintenanceTick();
  assert(!maintenanceMode && !testServos[12].torque && !lock.configured && lock.lastError == motion::Stale);
  expectAck(maintenanceRequest(motion::MaintenancePoll), motion::BadPacket);

  resetScene(); enter(); testServos[12].present = true;
  expectAck(centerRequest(12, 2200), motion::Ok);
  mockMicros += kMaintenanceMoveTimeoutUs + 1;
  maintenance.lastRequestUs = mockMicros; // Valid heartbeat cannot defeat total move deadline.
  maintenanceTick(); assert(!maintenanceMode && !testServos[12].torque);

  resetScene(); enter(); testServos[12].present = true;
  shortGoalWrite = true;
  expectAck(centerRequest(12, 2100), motion::CenterGoalWrite);
  assert(!testServos[12].torque); Serial1.writeResultOverride = -1;

  resetScene(); enter(); testServos[12].present = true;
  expectAck(centerRequest(12, 2100), motion::Ok); testServos[12].error = 1; tick();
  assert(maintenance.state == 3 && maintenance.error == 1 && maintenance.torque == 255 && !testServos[12].torque);

  resetScene(); enter(); testServos[12].present = testServos[13].present = true;
  expectAck(centerRequest(12, 2100), motion::Ok); testServos[13].error = 1;
  command(maintenanceRequest(motion::MaintenanceRead, 13)); const auto diagnostic = result();
  assert(diagnostic.payload[4] == 13 && diagnostic.payload[5] == 2);
  assert(maintenance.id == 12 && maintenance.state == 3 && !testServos[12].torque);
}

void testRename() {
  resetScene(); enter(); testServos[12].present = true;
  const uint32_t started = mockMicros;
  const auto rename = renameRequest(12, 99); expectAck(rename, motion::Ok);
  assert(uint32_t(mockMicros - started) < 600000);
  assert(!testServos[12].present && testServos[99].present && testServos[99].locked && !testServos[99].torque);
  assert(maintenance.id == 99 && countWrites(5) == 1);
  const size_t before = transactions.size(); expectAck(rename, motion::BadPacket); assert(transactions.size() == before);

  resetScene(); enter(); testServos[12].present = testServos[13].present = true;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && countWrites(55) == 0);
  resetScene(); enter(); testServos[12].present = testServos[99].present = true;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && countWrites(55) == 0);
  resetScene(); enter(); testServos[12].present = true; testServos[12].error = 1;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && countWrites(55) == 0);

  // A corrupted reply must not be interpreted as an unused address.
  resetScene(); enter(); testServos[12].present = testServos[99].present = true; corruptId = 99;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && countWrites(55) == 0);
  resetScene(); enter(); testServos[12].present = true; ignoreUnlock = true;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && testServos[12].locked);
  resetScene(); enter(); testServos[12].present = true; ignoreIdWrite = true;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 1 && testServos[12].present && testServos[12].locked);
  resetScene(); enter(); testServos[12].present = true; shortIdWrite = 12;
  expectAck(renameRequest(12, 99), motion::BadState);
  assert(countWrites(5) == 1 && testServos[99].present && testServos[99].locked && !testServos[99].torque);
  resetScene(); enter(); testServos[12].present = true; testServos[12].torque = true; ignoreRelease = true;
  expectAck(renameRequest(12, 99), motion::BadState); assert(countWrites(5) == 0 && countWrites(55) == 0);
}

int main() {
  Serial1.onWrite = maintenanceBus;
  testCapabilitiesAndIsolation(); testReadResultsAndFaultLatch(); testVoltageReadOnlyExtension(); testCenterAndStop();
  testCenterWithDelayedWriteAck(); testCenterFailedReadEvidence(); testGoalAutoEnableCenter(); testCurrentPositionCenterCheck();
  testWatchdogsAndWriteFailure(); testRename(); testMidCycleEntryAndRollover();
  puts("PASS exact maintenance handlers: capabilities, exclusive mode, stale motion masks, bounded read outcomes, fault latch, single-axis center+release, two watchdogs, replay/nonce, isolated ID scan, duplicate/corrupt/fault rejection, uncertain ID write never retried, EEPROM relock");
}
