// Exercise exact handlers: a refused TARGET must retain the first stop, not
// the BadState written by that refusal. No UART operations are added to ACK.
#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main

void freshArmed(uint32_t cookie) {
  safetyDiagnostic = SafetyDiagnostic{};
  const uint32_t now = micros();
  lock.armed = lock.configured = true; lock.cookie = cookie;
  lock.lastError = motion::Ok; lock.lastCommandUs = now;
  feedback.imuOk = true; feedback.sampleUs = feedback.imuUs = now;
  feedback.onlineMask = presentMask = torqueKnown = motion::kAllMotors;
  feedback.faultMask = presentFaults = torqueFaults = 0;
  feedback.torqueMask = motion::kAllMotors;
  for (size_t i = 0; i < 15; ++i) torqueReadUs[i] = now;
  disablePending = torqueVerifyPending = verifyingTorque = false;
}
motion::Frame targetFor(uint32_t cookie, uint32_t seq) {
  motion::Frame target; target.type = motion::Target; target.length = 34; target.sequence = seq;
  motion::put32(target.payload,cookie);
  for (size_t i = 0; i < 15; ++i) motion::put16(target.payload + 4 + 2*i,2047);
  return target;
}
int main() {
  Serial1.onWrite = busWrite; Wire.registers[0x0f] = 0x6b; Wire.registers[0x1e] = 3;
  setup(); mockMicros += 110000; readyFeedback();
  command(configuration(1)); assert(lastResponse().payload[1] == motion::Ok);
  freshArmed(123); mockMicros = 0xfffffff0u;
  const uint32_t now = mockMicros;
  feedback.sampleUs = feedback.imuUs = now;
  for (size_t i = 0; i < 15; ++i) torqueReadUs[i] = now;
  lock.lastCommandUs = now - motion::kWatchdogUs - 1;
  checkSafety();
  assert(!lock.armed && lock.lastError == motion::Stale && safetyDiagnostic.seen);
  assert(safetyDiagnostic.cause == TargetWatchdogStop && safetyDiagnostic.targetAge > 250000);
  const SafetyDiagnostic first = safetyDiagnostic;
  command(targetFor(123,2)); auto reply = lastResponse();
  assert(reply.length == 40 && reply.payload[0] == motion::Target && reply.payload[1] == motion::BadState);
  assert(reply.payload[2] == 1 && reply.payload[3] == TargetWatchdogStop && reply.payload[4] == motion::Stale);
  assert(motion::u32(reply.payload + 24) == first.targetAge);
  assert(lock.lastError == motion::BadState && safetyDiagnostic.error == motion::Stale);
  const size_t busBefore = Serial1.output.size();
  ack(targetFor(123,3),motion::BadState); reply = lastResponse();
  assert(reply.length == 40 && Serial1.output.size() == busBefore);
  command(targetFor(456,4)); assert(lastResponse().length == 2); // Other cookie cannot borrow it.
  motion::Frame stop; stop.type = motion::Stop; command(stop); lastResponse();
  assert(safetyDiagnostic.atUs == first.atUs && safetyDiagnostic.targetAge == first.targetAge);

  struct Case { StopCause cause; motion::Error error; } cases[] = {
    {DeviceFaultStop,motion::BadState},{OfflineStop,motion::Stale},{ImuInvalidStop,motion::Stale},
    {PositionStaleStop,motion::Stale},{ImuStaleStop,motion::Stale},{TorqueLostStop,motion::BadState}
  };
  uint32_t sequenceForTest = 4;
  for (const auto& test : cases) {
    freshArmed(123);
    if (test.cause == DeviceFaultStop) presentFaults = 1;
    if (test.cause == OfflineStop) torqueKnown &= ~(1u << 6);
    if (test.cause == ImuInvalidStop) feedback.imuOk = false;
    if (test.cause == PositionStaleStop) feedback.sampleUs -= motion::kStateFreshUs + 1;
    if (test.cause == ImuStaleStop) feedback.imuUs -= motion::kStateFreshUs + 1;
    if (test.cause == TorqueLostStop) feedback.torqueMask &= ~(1u << 6);
    checkSafety();
    assert(!lock.armed && safetyDiagnostic.seen);
    assert(safetyDiagnostic.cause == test.cause && safetyDiagnostic.error == test.error);
    command(targetFor(123,++sequenceForTest));
    reply = lastResponse(); assert(reply.length == 40 && reply.payload[3] == test.cause);
  }
  freshArmed(123); requestDisarm(motion::BadState,GoalWriteStop);
  assert(safetyDiagnostic.cause == GoalWriteStop && !lock.armed);
  // A new explicit valid ARM clears only the historical diagnostic. It still
  // performs unchanged real gain/position/torque verification before arming.
  command(stop); lastResponse(); readyFeedback();
  command(armCommand(100)); reply = lastResponse();
  assert(reply.payload[1] == motion::Ok && lock.armed && !safetyDiagnostic.seen);
  command(stop); lastResponse();
  puts("PASS first safety stop retained across TARGET refusal and STOP, cookie-bound ACK, cause distinctions, wrap, no extra bus read, explicit valid ARM resets history");
}
