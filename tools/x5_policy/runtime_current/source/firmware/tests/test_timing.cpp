// Exercise the actual firmware scheduler with healthy but slow, bounded UART
// replies and a 104 Hz IMU. No hardware or mocked safety functions are used.
#include <assert.h>
#include <stdio.h>
#include "mocks/Arduino.h"
#include "mocks/Wire.h"
uint32_t mockMicros = 0xfffc0000u;
HardwareSerial Serial, Serial1;
TwoWire Wire;
#include "../firmware/main.cpp"

namespace {
bool enabled[35] = {};
uint8_t kp[35] = {}, kd[35] = {};
uint8_t missingTorqueId = 0;
uint8_t missingVoltageId = 0;
size_t voltageReads = 0;
size_t enables = 0, torqueReads = 0;
uint32_t lastImuData = 0;

void bus(const uint8_t* packet, size_t count) {
  const uint8_t id = packet[2], instruction = packet[4], address = packet[5];
  if (instruction == 3) {
    assert(count == 8);
    if (address == 40) {
      assert(id == 254);
      if (packet[6]) ++enables;
      for (uint8_t sid : motion::kMotorIds) enabled[sid] = packet[6] != 0;
    } else {
      assert(address == 50 || address == 51);
      for (uint8_t sid : motion::kMotorIds) {
        assert(!enabled[sid]);
        if (id != 254 && id != sid) continue;
        (address == 50 ? kp[sid] : kd[sid]) = packet[6];
      }
    }
    mockMicros += uint32_t(count * 10); return;
  }
  if (instruction == 0x83) {
    assert(count == 113 && id == 254 && address == 42);
    mockMicros += uint32_t(count * 10); return;
  }
  assert(instruction == 2 && count == 8);
  if (address == 40) {
    ++torqueReads;
    if (id == missingTorqueId) return;
  }
  if (address == 62) { ++voltageReads; if (id == missingVoltageId) return; }
  std::vector<uint8_t> bytes(packet, packet + count);
  std::vector<uint8_t> reply = {255,255,id,uint8_t(packet[6] + 2),0};
  if (address == 40) reply.push_back(enabled[id] ? 1 : 0);
  else if (address == 50) reply.insert(reply.end(), {kp[id],kd[id]});
  else if (address == 62) reply.push_back(id == 10 ? 39 : 71);
  else { assert(address == 56); reply.insert(reply.end(), {255,7,0,0,0,0}); }
  uint8_t sum = 0; for (size_t i = 2; i < reply.size(); ++i) sum += reply[i];
  reply.push_back(uint8_t(~sum)); bytes.insert(bytes.end(), reply.begin(), reply.end());
  // Each complete valid response, including echo, fits the unchanged 1.5 ms
  // READ deadline. Delayed but valid responses must not look like bus faults.
  uint32_t at = mockMicros + 1250;
  for (uint8_t value : bytes) { Serial1.scheduledInput.emplace_back(at,value); at += 8; }
}

motion::Frame response() {
  motion::Parser decoder; motion::Frame last;
  for (uint8_t value : Serial.output) if (decoder.feed(value)) last = decoder.frame;
  Serial.output.clear(); return last;
}
void sendTarget(uint32_t seq) {
  motion::Frame target; target.type = motion::Target; target.length = 34; target.sequence = seq;
  motion::put32(target.payload,123);
  for (size_t i = 0; i < motion::kMotorCount; ++i) motion::put16(target.payload + 4 + i * 2,2047);
  command(target); assert(response().payload[1] == motion::Ok);
}
void stopAndArm(uint32_t seq) {
  motion::Frame stop; stop.type = motion::Stop; command(stop);
  assert(response().payload[1] == motion::Ok && !lock.armed && feedback.torqueMask == 0);
  sampleServos(); sampleImu();
  motion::Frame arm; arm.type = motion::Arm; arm.length = 4; arm.sequence = seq;
  motion::put32(arm.payload,123);
  const size_t before = torqueReads, enablesBefore = enables;
  const uint32_t began = mockMicros;
  command(arm);
  assert(response().payload[1] == motion::Ok && lock.armed);
  assert(uint32_t(mockMicros - began) < 160000);
  // Both pre-enable and post-enable checks still cover every mapped servo.
  assert(torqueReads - before == 30 && enables == enablesBefore + 1);
  assert(feedback.torqueMask == motion::kAllMotors && !verifyingTorque);
}
}

int main() {
  Serial1.onWrite = bus;
  Wire.onRead = [](uint8_t reg) {
    mockMicros += reg == 0x22 ? 400 : 150;
    if (reg == 0x1e) Wire.registers[0x1e] = uint32_t(mockMicros - lastImuData) >= 9615 ? 3 : 0;
    if (reg == 0x22) lastImuData = mockMicros;
  };
  Wire.registers[0x0f] = 0x6b; setup(); mockMicros += 110000;
  sampleServos(); sampleImu(); refreshTorque();
  motion::Frame hello; hello.type = motion::Hello; command(hello); response();
  motion::Frame config; config.type = motion::ConfigRequest; config.length = 105; config.sequence = 1;
  for (size_t i = 0; i < motion::kMotorCount; ++i) {
    motion::put16(config.payload + i * 2,2047); config.payload[30 + i] = 1;
    motion::put16(config.payload + 45 + i * 2,1000); motion::put16(config.payload + 75 + i * 2,3000);
  }
  command(config); assert(response().payload[1] == motion::Ok);
  stopAndArm(2);
  uint32_t seq = 3;
  for (size_t n = 0; n < 200; ++n) {
    sendTarget(seq++);
    const size_t before = torqueReads, voltageBefore = voltageReads;
    loop();
    assert(lock.armed && lock.cookie == 123 && lock.lastError == motion::Ok);
    assert(torqueReads - before <= 3);
    assert(voltageReads - voltageBefore == 1);
    assert(feedback.onlineMask == motion::kAllMotors && feedback.faultMask == 0);
    for (uint32_t readAt : torqueReadUs) assert(uint32_t(mockMicros - readAt) < kTorqueFreshUs);
  }
  assert(mockMicros < 0x80000000u);  // The sustained test crosses micros() wrap.
  const size_t voltageBefore = voltageReads, enableBefore = enables;
  motion::Frame caps; caps.type = motion::TelemetryCaps; command(caps);
  auto capReply = response(); assert(capReply.type == 0x94);
  assert(std::string(reinterpret_cast<char*>(capReply.payload),capReply.length) == "XGODUCK_TELEMETRY_1");
  motion::Frame extended; extended.type = motion::VoltageState; command(extended);
  auto voltageReply = response(); assert(voltageReply.type == 0x93 && voltageReply.length == 186);
  assert(voltageReply.payload[106] == 1 && motion::u16(voltageReply.payload + 107) == motion::kAllMotors);
  assert(motion::u16(voltageReply.payload + 109) == motion::kAllMotors);
  assert(voltageReply.payload[111] == 39 && voltageReply.payload[112] == 71);
  for (size_t i = 0; i < 15; ++i) assert(motion::u32(voltageReply.payload + 126 + 4*i) < motion::kVoltageFreshUs);
  motion::Frame legacy; legacy.type = motion::State; command(legacy);
  assert(response().length == 106 && voltageReads == voltageBefore && enables == enableBefore);
  // Losing a voltage reply invalidates only its live reading, retaining the
  // dated value without inventing 0 V or touching torque/motion freshness.
  missingVoltageId = 10;
  for (size_t n = 0; n < 16; ++n) { sendTarget(seq++); loop(); }
  assert(lock.armed && voltageSeen == motion::kAllMotors && !(voltageValid & 1) && voltageRaw[0] == 39);
  command(extended); voltageReply = response(); assert(!(motion::u16(voltageReply.payload + 109) & 1));
  missingVoltageId = 0;

  // A real loss of observed torque still disarms on its next periodic read.
  const size_t healthyEnables = enables;
  enabled[10] = false;
  for (size_t n = 0; n < 12 && lock.armed; ++n) { sendTarget(seq++); loop(); }
  assert(!lock.armed && lock.cookie == 0 && lock.lastError == motion::BadState);
  assert(enables == healthyEnables);

  stopAndArm(seq++);
  missingTorqueId = 21;
  for (size_t n = 0; n < 16 && lock.armed; ++n) { sendTarget(seq++); loop(); }
  assert(!lock.armed && lock.cookie == 0 && lock.lastError == motion::Stale);
  missingTorqueId = 0;

  stopAndArm(seq++);
  const uint32_t lastTarget = lock.lastCommandUs;
  for (size_t n = 0; n < 16 && lock.armed; ++n) loop();  // No valid TARGET heartbeat.
  assert(!lock.armed && uint32_t(mockMicros - lastTarget) > motion::kWatchdogUs);
  assert(lock.lastError == motion::Stale && lock.cookie == 0);
  const size_t stoppedEnables = enables;
  for (size_t n = 0; n < 3; ++n) loop();
  assert(!lock.armed && enables == stoppedEnables);

  stopAndArm(seq++);
  Wire.online = false; loop();
  assert(!lock.armed && !feedback.imuOk && lock.cookie == 0);
  puts("PASS slow valid UART + 104 Hz IMU: sustained ARM/TARGET across micros wrap, full ARM/STOP readbacks, bounded periodic batches; real torque loss/missing reply/IMU loss/TARGET watchdog still disarm without rearm");
}
