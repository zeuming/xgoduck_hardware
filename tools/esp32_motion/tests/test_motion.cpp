#include <assert.h>
#include <stdio.h>
#include <vector>
#include "../firmware/motion_protocol.h"

using namespace motion;

Frame configFrame(uint32_t seq = 1) {
  Frame frame; frame.type = ConfigRequest; frame.sequence = seq; frame.length = kConfigSize;
  for (size_t i = 0; i < 15; ++i) {
    put16(frame.payload + i * 2, 2047); frame.payload[30 + i] = i & 1 ? 0xff : 1;
    put16(frame.payload + 45 + i * 2, 1000); put16(frame.payload + 75 + i * 2, 3000);
  }
  return frame;
}
Feedback goodFeedback(uint32_t now = 1000000) {
  Feedback feedback; feedback.sampleUs = feedback.imuUs = now;
  feedback.onlineMask = kAllMotors; feedback.imuOk = true;
  for (size_t i = 0; i < 15; ++i) feedback.positions[i] = 2047;
  return feedback;
}
Frame armFrame(uint32_t cookie = 123) {
  Frame frame; frame.type = Arm; frame.length = 4; frame.sequence = 2; put32(frame.payload, cookie); return frame;
}
Frame targetFrame(uint32_t cookie = 123, uint16_t target = 2400) {
  Frame frame; frame.type = Target; frame.length = 34; frame.sequence = 3; put32(frame.payload, cookie);
  for (size_t i = 0; i < 15; ++i) put16(frame.payload + 4 + i * 2, target);
  return frame;
}
void testWire() {
  const uint8_t reference[] = "123456789";
  assert(crc16(reference, 9) == 0x29b1);
  const Frame original = configFrame(0x12345678);
  uint8_t encoded[268];
  const size_t count = encode(original.type, original.sequence, original.payload, original.length, encoded);
  assert(count == 117 && encoded[0] == 'X' && encoded[1] == 'D' && u16(encoded + 4) == 105);
  Parser parser;
  for (size_t i = 0; i < count - 1; ++i) assert(!parser.feed(encoded[i]));
  assert(parser.feed(encoded[count - 1]));
  assert(parser.frame.sequence == original.sequence && parser.frame.length == original.length);
  assert(memcmp(parser.frame.payload, original.payload, original.length) == 0);

  // Corrupt packet, noise, wrong version and oversize length all recover to a
  // valid subsequent command without reporting the corrupt packet as valid.
  std::vector<uint8_t> damaged(encoded, encoded + count); damaged[20] ^= 1;
  unsigned accepted = 0;
  for (uint8_t byte : damaged) accepted += parser.feed(byte);
  assert(accepted == 0);
  const uint8_t noise[] = {0, 0x58, 0x58, 0x44, 2, 4, 0xff, 0xff, 0, 0, 0, 0};
  for (uint8_t byte : noise) accepted += parser.feed(byte);
  for (size_t i = 0; i < count; ++i) accepted += parser.feed(encoded[i]);
  assert(accepted == 1 && parser.rejected >= 2);
  assert(encode(1, 1, encoded, 257, encoded) == 0);

  // Header sequence is CRC protected as well as payload.
  encoded[6] ^= 1; accepted = 0;
  for (size_t i = 0; i < count; ++i) accepted += parser.feed(encoded[i]);
  assert(accepted == 0);
}
void testSequence() {
  SequenceGuard guard;
  assert(guard.consume(0)); assert(guard.consume(1));
  assert(!guard.consume(1)); assert(!guard.consume(0));
  assert(guard.consume(0xffffffff)); assert(!guard.consume(0));
  guard.reset(); assert(guard.consume(0));
}
void testMaintenanceGuard() {
  MaintenanceGuard guard;
  assert(!guard.enter(0, 1));
  assert(guard.enter(123, 1));
  assert(!guard.enter(456, 2));
  assert(!guard.accept(456, 2));
  assert(guard.accept(123, 2));
  assert(!guard.accept(123, 2));
  assert(!guard.accept(123, 1));
  guard.close();
  assert(!guard.accept(123, 3) && !guard.enter(123, 3));
  assert(guard.enter(456, 1));
  assert(guard.accept(456, 0xffffffff));
  assert(!guard.accept(456, 0));
  guard.close();
  for (uint32_t i = 2; i < MaintenanceGuard::kCapacity; ++i) {
    assert(guard.enter(1000 + i, 0)); guard.close();
  }
  assert(!guard.enter(9999, 1));
}
void testConfiguration() {
  Interlock lock;
  Frame good = configFrame();
  assert(lock.configure(good) == Ok && lock.configured && !lock.armed);
  assert(lock.config.sign[1] == -1 && lock.config.zero[14] == 2047);
  Frame bad = good; bad.payload[30] = 0;
  assert(lock.configure(bad) == BadConfig && lock.config.sign[0] == 1);
  bad = good; put16(bad.payload + 75, 4096);
  assert(lock.configure(bad) == BadConfig && lock.config.maximum[0] == 3000);
  bad = good; put16(bad.payload, 999);
  assert(lock.configure(bad) == BadConfig);
  bad = good; put16(bad.payload + 45, 3001);
  assert(lock.configure(bad) == BadConfig);
  bad = good; bad.length = 104;
  assert(lock.configure(bad) == BadConfig);
  lock.armed = true; assert(lock.configure(good) == BadState);
}
void testArming() {
  Interlock lock; Feedback feedback = goodFeedback();
  const Frame arm = armFrame();
  assert(lock.prepareArm(arm, feedback, 1000000) == BadState);
  assert(lock.configure(configFrame()) == Ok);
  assert(lock.prepareArm(armFrame(0), feedback, 1000000) == BadState);
  feedback.onlineMask &= ~1; assert(lock.prepareArm(arm, feedback, 1000000) == Stale);
  feedback = goodFeedback(); feedback.imuOk = false;
  assert(lock.prepareArm(arm, feedback, 1000000) == Stale);
  feedback = goodFeedback(); feedback.faultMask = 4;
  assert(lock.prepareArm(arm, feedback, 1000000) == BadState);
  feedback = goodFeedback(); feedback.torqueMask = 4;
  assert(lock.prepareArm(arm, feedback, 1000000) == BadState);
  feedback = goodFeedback(); feedback.positions[8] = 3001;
  assert(lock.prepareArm(arm, feedback, 1000000) == Range);
  feedback = goodFeedback();
  assert(lock.prepareArm(arm, feedback, 1050001) == Stale);
  assert(lock.prepareArm(arm, feedback, 1000000) == Ok);
  assert(!lock.armed && lock.cookie == 123 && lock.goals[0] == 2047 && lock.desired[14] == 2047);
  // Preparation never enables torque; caller must write/verify physical goals.
  lock.armed = true; assert(lock.prepareArm(arm, feedback, 1000000) == BadState);
}
void testTargetsAndSlew() {
  Interlock lock; assert(lock.configure(configFrame()) == Ok);
  const Feedback feedback = goodFeedback();
  assert(lock.prepareArm(armFrame(), feedback, 1000000) == Ok); lock.armed = true;
  const uint32_t original = lock.lastCommandUs;
  Frame malformed = targetFrame(); malformed.length = 33;
  assert(lock.target(malformed, 1010000) == BadPacket && lock.lastCommandUs == original);
  assert(lock.target(targetFrame(456), 1010000) == BadState && lock.lastCommandUs == original);
  Frame outside = targetFrame(); put16(outside.payload + 4 + 28, 3001);
  assert(lock.target(outside, 1010000) == Range && lock.lastCommandUs == original);
  assert(lock.desired[0] == 2047);  // Invalid full group cannot partially update.
  assert(lock.target(targetFrame(), 1010000) == Ok && lock.lastCommandUs == 1010000);
  lock.slew(1010000); assert(lock.goals[0] == 2053 && lock.goals[14] == 2053);
  lock.slew(1030000); assert(lock.goals[0] == 2065);
  // A long delayed loop is bounded to a 50ms step rather than a large jump.
  lock.slew(1230000); assert(lock.goals[0] == 2095);
  assert(lock.target(targetFrame(123, 1000), 1230000) == Ok);
  lock.slew(1240000); assert(lock.goals[0] == 2089);
}
void testWatchdog() {
  Interlock lock; lock.configure(configFrame());
  Feedback feedback = goodFeedback(); lock.prepareArm(armFrame(), feedback, 1000000); lock.armed = true;
  feedback = goodFeedback(1240000); assert(!lock.check(feedback, 1240000));
  assert(lock.check(feedback, 1250001) && !lock.armed && lock.cookie == 0 && lock.lastError == Stale);
  feedback = goodFeedback(1500000); assert(!lock.check(feedback, 1500000) && !lock.armed);
  assert(lock.target(targetFrame(), 1500000) == BadState);  // No autoresume.
  assert(lock.prepareArm(armFrame(), feedback, 1500000) == Ok); lock.armed = true;
  feedback.imuOk = false; assert(lock.check(feedback, 1500001) && lock.lastError == Stale);
  feedback = goodFeedback(1600000); lock.prepareArm(armFrame(), feedback, 1600000); lock.armed = true;
  feedback.faultMask = 1; assert(lock.check(feedback, 1600001) && lock.lastError == BadState);

  // micros rollover uses modulo elapsed arithmetic, not naive ordering.
  feedback = goodFeedback(0xfffffff0); lock.prepareArm(armFrame(), feedback, 0xfffffff0); lock.armed = true;
  assert(!lock.check(feedback, 0x20));
  lock.slew(0x20); assert(lock.goals[0] == 2047);
}
void testSpeed() {
  assert(speedSignMagnitude(0) == 0); assert(speedSignMagnitude(0x8000) == 0);
  assert(speedSignMagnitude(0x7fff) == 32767); assert(speedSignMagnitude(0xffff) == -32767);
  assert(speedSignMagnitude(0x8064) == -100);
}
int main() {
  testWire(); testSequence(); testMaintenanceGuard(); testConfiguration(); testArming(); testTargetsAndSlew(); testWatchdog(); testSpeed();
  puts("PASS motion framing, corruption recovery, replay, configuration, arming, bounds, slew, watchdog, rollover, speed decode");
}
