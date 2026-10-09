// Fixed OFF-only raw register evidence. Addresses are not assertions about
// the connected servo model's register meanings. No arbitrary host READ API.
constexpr uint8_t kOffRegisterReadBlocks[motion::kOffRegisterBlockCount][2] = {
  {0, 6}, {16, 2}, {33, 1}, {40, 1}, {42, 6}, {48, 4}, {62, 2}, {69, 2}
};

void offRegisterReadBlock(uint8_t* data, uint8_t id, uint8_t address, uint8_t size) {
  data[0] = address; data[1] = size;
  motion::put32(data + 2, micros());
  const ServoReply reply = readServo(id, address, size);
  motion::put32(data + 6, micros());
  const bool valid = reply.found && !reply.error && reply.size == size;
  data[10] = uint8_t(reply.outcome); data[11] = reply.error; data[12] = reply.size;
  data[13] = (reply.found ? 1 : 0) | (reply.invalidReplySeen ? 2 : 0) | (valid ? 4 : 0);
  memcpy(data + 14, reply.data, sizeof(reply.data));
}

void offRegisterDetails(const motion::Frame& frame) {
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
  // Share Type22's boot-scoped quota, including failed attempts. Rejected
  // requests do not touch UART, safety errors or the motion write sequence.
  goalReadDetailsSeen = true; lastGoalReadDetailsUs = now;
  static_assert(motion::kOffRegisterDetailsSize <= motion::kMaxPayload,
                "OFF raw register details must fit one frame");
  uint8_t data[motion::kOffRegisterDetailsSize] = {};
  data[0] = motion::kOffRegisterDetailsVersion; data[1] = motion::kMotorIds[index];
  data[2] = uint8_t(index); data[3] = motion::kOffRegisterBlockCount;
  motion::put16(data + 4, lock.desired[index]); motion::put16(data + 6, lock.goals[index]);
  data[8] = lock.armed; motion::put32(data + 10, lock.cookie);
  motion::put32(data + 18, feedback.sampleUs); motion::put32(data + 22, feedback.imuUs);
  motion::put16(data + 34, torqueKnown); motion::put16(data + 36, feedback.torqueMask);
  for (size_t i = 0; i < motion::kOffRegisterBlockCount; ++i) {
    offRegisterReadBlock(data + 42 + i * 20, motion::kMotorIds[index],
                         kOffRegisterReadBlocks[i][0], kOffRegisterReadBlocks[i][1]);
  }
  data[9] = lock.armed; motion::put32(data + 14, lock.cookie);
  motion::put32(data + 26, feedback.sampleUs); motion::put32(data + 30, feedback.imuUs);
  motion::put16(data + 38, torqueKnown); motion::put16(data + 40, feedback.torqueMask);
  respond(0x97, frame.sequence, data, sizeof(data));
}
