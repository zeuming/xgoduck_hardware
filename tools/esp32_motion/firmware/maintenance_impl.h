#pragma once

// Included inside main.cpp's anonymous namespace, after its bounded UART read
// and framed response helpers. No raw UART/XFER command is exposed to hosts.
constexpr uint32_t kMaintenanceWatchdogUs = 1000000;
constexpr uint32_t kMaintenanceMoveTimeoutUs = 10000000;
constexpr uint32_t kMaintenanceStepUs = 35000;
constexpr uint16_t kMaintenanceUnknownPosition = 0xffff;
constexpr uint8_t kMaintenanceUnknownTorque = 0xff;
motion::MaintenanceGuard maintenanceGuard;

struct MaintenanceState {
  uint8_t id = 255, state = 0, error = 0, torque = kMaintenanceUnknownTorque;
  uint16_t position = kMaintenanceUnknownPosition, previous = 0, goal = 0, target = 0;
  uint16_t minimum = 0, maximum = 0;
  uint8_t settled = 0;
  bool faultSeen = false;
  uint32_t lastRequestUs = 0, startedUs = 0, stepUs = 0;
} maintenance;

// Copy the failed read BEFORE torque release performs its own reads. The
// evidence is returned in the same failure ACK, with no extra host request.
ServoReadTrace centerFailureTrace;
uint16_t centerFailureExpected = 0;
bool centerFailureTraceValid = false;

void maintenanceCenterAck(const motion::Frame& frame, motion::Error failure) {
  if (!centerFailureTraceValid || (failure != motion::CenterGoalRead &&
      failure != motion::CenterHoldingTorqueRead && failure != motion::CenterEnableRead)) {
    ack(frame, failure); return;
  }
  uint8_t data[motion::kCenterFailureAckSize] = {};
  const ServoReadTrace& trace = centerFailureTrace;
  const ServoReply& reply = trace.reply;
  data[0] = frame.type; data[1] = failure; data[2] = 1;
  data[3] = trace.id; data[4] = trace.address; data[5] = trace.requestedSize;
  motion::put16(data + 6, centerFailureExpected);
  data[8] = reply.found; data[9] = reply.invalidReplySeen;
  data[10] = reply.outcome; data[11] = reply.error; data[12] = reply.size;
  memcpy(data + 13, reply.data, sizeof(reply.data));
  motion::put32(data + 19, trace.elapsedUs);
  motion::put32(data + 23, trace.errorsBefore); motion::put32(data + 27, trace.errorsAfter);
  motion::put16(data + 31, trace.drainedCount); motion::put16(data + 33, trace.receivedCount);
  data[35] = trace.drainedCount < 16 ? trace.drainedCount : 16;
  data[36] = trace.receivedCount < 48 ? trace.receivedCount : 48;
  memcpy(data + 37, trace.drained, sizeof(trace.drained));
  memcpy(data + 53, trace.received, sizeof(trace.received));
  respond(0x80, frame.sequence, data, sizeof(data));
}

struct MaintenanceSample {
  uint8_t outcome = 3, error = 0, torque = kMaintenanceUnknownTorque;
  uint16_t position = kMaintenanceUnknownPosition;
};

void invalidateMotionFeedback() {
  lock.disarm(); lock.configured = false; lock.config = motion::Config{};
  presentMask = presentFaults = torqueFaults = torqueKnown = 0;
  feedback.onlineMask = feedback.faultMask = feedback.torqueMask = 0;
  feedback.sampleUs = micros() - motion::kStateFreshUs - 1;
  memset(feedback.positions, 0, sizeof(feedback.positions));
  memset(velocities, 0, sizeof(velocities));
  memset(torqueReadUs, 0, sizeof(torqueReadUs));
}

uint8_t maintenanceReplyOutcome(const ServoReply& reply, uint8_t size) {
  if (reply.found && reply.error) return 2;
  if (reply.invalidReplySeen) return 3;
  if (reply.found && reply.size == size) return 0;
  return reply.outcome == motion::NoValidReplyByDeadline ? 1 : 3;
}

MaintenanceSample maintenanceSnapshot(uint8_t id) {
  MaintenanceSample result;
  const ServoReply identity = readServo(id, 5, 1);
  result.outcome = maintenanceReplyOutcome(identity, 1); result.error = identity.error;
  if (result.outcome) return result;
  if (identity.data[0] != id) { result.outcome = 3; return result; }
  const ServoReply position = readServo(id, 56, 6);
  result.outcome = maintenanceReplyOutcome(position, 6); result.error = position.error;
  if (result.outcome) return result;
  if (motion::u16(position.data) > 4095) { result.outcome = 3; return result; }
  result.position = motion::u16(position.data);
  const ServoReply torque = readServo(id, 40, 1);
  result.outcome = maintenanceReplyOutcome(torque, 1); result.error = torque.error;
  if (result.outcome) return result;
  if (torque.data[0] > 1) { result.outcome = 3; return result; }
  result.torque = torque.data[0]; return result;
}

void maintenanceRemember(const MaintenanceSample& sample) {
  maintenance.position = sample.position; maintenance.torque = sample.torque;
  maintenance.error = sample.error;
  if (sample.error) maintenance.faultSeen = true;
}

bool maintenanceWrite(uint8_t id, uint8_t address, const uint8_t* value, uint8_t size) {
  if (id > 253 || size > 6) return false;
  uint8_t error = 0;
  const bool written = writeServoRegister(id, address, value, size, &error);
  if (error) { maintenance.error = error; maintenance.faultSeen = true; }
  return written;
}

bool maintenanceWriteByte(uint8_t id, uint8_t address, uint8_t value) {
  return maintenanceWrite(id, address, &value, 1);
}

bool maintenanceRegisterEquals(uint8_t id, uint8_t address, uint16_t expected, uint8_t size) {
  const ServoReply reply = readServo(id, address, size);
  if (reply.error) { maintenance.error = reply.error; maintenance.faultSeen = true; }
  return reply.found && !reply.error && !reply.invalidReplySeen && reply.size == size &&
         (size == 1 ? reply.data[0] : motion::u16(reply.data)) == expected;
}

bool maintenanceReadHoldingTorque(uint8_t id, uint8_t& value) {
  const ServoReply reply = readServo(id, 40, 1);
  if (reply.error) { maintenance.error = reply.error; maintenance.faultSeen = true; }
  if (maintenanceReplyOutcome(reply, 1) || reply.data[0] > 1) return false;
  value = reply.data[0];
  return true;
}

// Release and re-lock are the only writes permitted after a device fault.
// They do not clear the fault latch or report optimistic torque-off.
bool maintenanceReleaseOne(uint8_t id) {
  if (id > 253) return false;
  const bool sent = maintenanceWriteByte(id, 40, 0);
  const bool verified = maintenanceRegisterEquals(id, 40, 0, 1);
  if (id == maintenance.id) maintenance.torque = verified ? 0 : kMaintenanceUnknownTorque;
  return sent && verified;
}

void maintenanceFault(uint8_t error = 0) {
  if (error) { maintenance.error = error; maintenance.faultSeen = true; }
  maintenance.state = 3;
  invalidateMotionFeedback();
  if (maintenance.id <= 253) (void)maintenanceReleaseOne(maintenance.id);
}

bool maintenanceClose() {
  bool released = true;
  if (maintenance.id <= 253) released = maintenanceReleaseOne(maintenance.id);
  // Preserve the existing STOP/boot broadcast fail-safe, including unknown IDs.
  const bool broadcast = writeTorque(false);
  maintenanceMode = false; maintenanceGuard.close();
  invalidateMotionFeedback();
  maintenance.id = 255; maintenance.state = released && broadcast ? 0 : 3;
  disablePending = !broadcast; torqueVerifyPending = false;
  return released && broadcast;
}

void maintenanceWatchdog() {
  if (!maintenanceMode) return;
  const uint32_t now = micros();
  if (uint32_t(now - maintenance.lastRequestUs) > kMaintenanceWatchdogUs ||
      (maintenance.state == 1 && uint32_t(now - maintenance.startedUs) > kMaintenanceMoveTimeoutUs)) {
    (void)maintenanceClose(); lock.record(motion::Stale);
  }
}

bool maintenanceGoal(uint8_t id, uint16_t position) {
  uint8_t data[6] = {}; motion::put16(data, position);
  return maintenanceWrite(id, 42, data, sizeof(data));
}

void maintenanceTick() {
  maintenanceWatchdog();
  if (!maintenanceMode || maintenance.state != 1) return;
  const uint32_t now = micros();
  if (uint32_t(now - maintenance.stepUs) < kMaintenanceStepUs) return;
  maintenance.stepUs = now;
  const MaintenanceSample sample = maintenanceSnapshot(maintenance.id);
  maintenanceRemember(sample);
  if (sample.outcome || sample.torque != 1 || sample.position < maintenance.minimum ||
      sample.position > maintenance.maximum ||
      abs(int(sample.position) - int(maintenance.goal)) > 64 ||
      abs(int(sample.position) - int(maintenance.previous)) > 96) {
    maintenanceFault(sample.error); return;
  }
  maintenance.previous = sample.position;
  if (maintenance.goal == maintenance.target) {
    maintenance.settled = abs(int(sample.position) - int(maintenance.target)) <= 12 ? maintenance.settled + 1 : 0;
    if (maintenance.settled >= 3) {
      if (maintenanceReleaseOne(maintenance.id)) maintenance.state = 2;
      else maintenanceFault();
    }
    return;
  }
  const int delta = int(maintenance.target) - int(maintenance.goal);
  const uint16_t next = uint16_t(int(maintenance.goal) + (delta > 8 ? 8 : delta < -8 ? -8 : delta));
  if (!maintenanceGoal(maintenance.id, next)) { maintenanceFault(); return; }
  maintenance.goal = next;
}

motion::Error maintenanceStartCenter(uint8_t id, uint16_t target, uint16_t minimum, uint16_t maximum, bool checkOnly = false) {
  centerFailureTraceValid = false;
  if (maintenance.state == 1 || maintenance.faultSeen) return motion::BadState;
  if (maximum > 4095 || minimum > maximum || target < minimum || target > maximum) return motion::Range;
  const MaintenanceSample sample = maintenanceSnapshot(id);
  maintenanceRemember(sample);
  if (sample.outcome || sample.torque != 0) return motion::CenterSnapshot;
  if (sample.position < minimum || sample.position > maximum) return motion::Range;
  maintenance.id = id; maintenance.goal = maintenance.previous = sample.position;
  maintenance.target = target; maintenance.minimum = minimum; maintenance.maximum = maximum;
  maintenance.settled = 0;
  // A goal write can itself enable holding on HLS servos. The first goal must
  // therefore be the freshly measured current position, never the destination.
  motion::Error failure = motion::Ok;
  uint8_t holdingTorque = kMaintenanceUnknownTorque;
  if (!maintenanceGoal(id, sample.position)) failure = motion::CenterGoalWrite;
  else if (!maintenanceRegisterEquals(id, 42, sample.position, 2)) failure = motion::CenterGoalRead;
  else if (!maintenanceReadHoldingTorque(id, holdingTorque)) failure = motion::CenterHoldingTorqueRead;
  else if (!checkOnly && holdingTorque == 0 && !maintenanceWriteByte(id, 40, 1)) failure = motion::CenterEnableWrite;
  else if (!checkOnly && !maintenanceRegisterEquals(id, 40, 1, 1)) failure = motion::CenterEnableRead;
  if (failure != motion::Ok) {
    if (failure == motion::CenterGoalRead || failure == motion::CenterHoldingTorqueRead ||
        failure == motion::CenterEnableRead) {
      centerFailureTrace = lastServoReadTrace;
      centerFailureExpected = failure == motion::CenterGoalRead ? sample.position :
                              failure == motion::CenterEnableRead ? 1 : 0;
      centerFailureTraceValid = true;
    }
    maintenanceFault(); return failure;
  }
  if (checkOnly) {
    maintenance.target = maintenance.goal;
    // Even without an explicit enable write, the goal may have activated
    // holding. A diagnostic succeeds only after one release and its readback.
    if (!maintenanceReleaseOne(id)) {
      maintenance.state = 3; invalidateMotionFeedback(); return motion::BadState;
    }
    maintenance.state = 0;
    return motion::Ok;
  }
  maintenance.torque = 1; maintenance.state = 1;
  maintenance.startedUs = maintenance.stepUs = micros();
  return motion::Ok;
}

motion::Error maintenanceRename(uint8_t oldId, uint8_t newId) {
  if (maintenance.state == 1 || maintenance.faultSeen || oldId == newId) return motion::BadState;
  // One bounded read of register 5 at every valid unicast address. A device
  // error, malformed reply, UART error, or more than one responder aborts
  // before any EEPROM write. A scan cannot distinguish duplicate physical IDs;
  // the UI must require that only the intended servo is physically connected.
  unsigned responders = 0; bool oldFound = false;
  for (unsigned id = 0; id <= 253; ++id) {
    const ServoReply reply = readServo(uint8_t(id), 5, 1);
    maintenanceWatchdog();
    if (!maintenanceMode) return motion::Stale;
    const uint8_t outcome = maintenanceReplyOutcome(reply, 1);
    if (outcome == 1) continue;
    if (outcome || reply.data[0] != id) {
      if (reply.error) { maintenance.error = reply.error; maintenance.faultSeen = true; }
      return motion::BadState;
    }
    if (++responders > 1 || id == newId) return motion::BadState;
    oldFound |= id == oldId;
  }
  if (responders != 1 || !oldFound) return motion::BadState;
  const MaintenanceSample sample = maintenanceSnapshot(oldId);
  maintenanceRemember(sample); maintenance.id = oldId;
  if (sample.outcome) return motion::BadState;
  if (!maintenanceReleaseOne(oldId)) return motion::BadState;
  bool unlocked = false, success = false;
  // Do not retry any ID write, even if UART TX or the reply is uncertain.
  unlocked = true;  // A partial unlock transmission is also uncertain.
  if (maintenanceWriteByte(oldId, 55, 0) && maintenanceRegisterEquals(oldId, 55, 0, 1) &&
      maintenanceWriteByte(oldId, 5, newId)) {
    delay(50);  // Existing workbench EEPROM settling; readback decides success.
    if (maintenanceRegisterEquals(newId, 5, newId, 1)) {
      maintenance.id = newId;
      success = maintenanceWriteByte(newId, 55, 1) && maintenanceRegisterEquals(newId, 55, 1, 1) &&
                maintenanceRegisterEquals(newId, 40, 0, 1);
    }
  }
  if (!success && unlocked) {
    // No rollback. Re-lock each plausible endpoint once, even if its current
    // status reports a fault. Lock/torque readback determines what is known.
    (void)maintenanceWriteByte(newId, 55, 1);
    (void)maintenanceRegisterEquals(newId, 55, 1, 1);
    (void)maintenanceWriteByte(oldId, 55, 1);
    (void)maintenanceRegisterEquals(oldId, 55, 1, 1);
    maintenanceFault(); return motion::BadState;
  }
  const MaintenanceSample after = maintenanceSnapshot(newId);
  maintenanceRemember(after);
  if (after.outcome || after.torque != 0) { maintenanceFault(after.error); return motion::BadState; }
  maintenance.state = 0; return motion::Ok;
}

bool maintenanceCommand(const motion::Frame& frame) {
  if (frame.type < motion::MaintenanceCaps || frame.type > motion::MaintenanceVoltageRead) return false;
  maintenanceWatchdog();
  if (frame.type == motion::MaintenanceCaps) {
    if (frame.length) ack(frame, motion::BadPacket);
    else {
      constexpr char capability[] = "XGODUCK_MAINT_1";
      respond(0x88, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability) - 1);
    }
    return true;
  }
  if (frame.type == motion::MaintenanceVoltageCaps) {
    if (frame.length) ack(frame, motion::BadPacket);
    else {
      constexpr char capability[] = "XGODUCK_VOLTAGE_1";
      respond(0x91, frame.sequence, reinterpret_cast<const uint8_t*>(capability), sizeof(capability) - 1);
    }
    return true;
  }
  const uint16_t length = frame.type == motion::MaintenanceRename ? 6 :
      frame.type == motion::MaintenanceCenter ? 11 :
      (frame.type == motion::MaintenanceRead || frame.type == motion::MaintenanceRelease ||
       frame.type == motion::MaintenanceCenterCheck || frame.type == motion::MaintenanceVoltageRead) ? 5 : 4;
  if (frame.length != length) { ack(frame, motion::BadPacket); return true; }
  const uint32_t cookie = motion::u32(frame.payload);
  if (frame.type == motion::MaintenanceEnter) {
    if (lock.armed || maintenanceMode || disablePending) ack(frame, motion::BadState);
    else if (!maintenanceGuard.enter(cookie, frame.sequence)) ack(frame, motion::BadPacket);
    else {
      maintenanceMode = true; maintenance = MaintenanceState{};
      maintenance.lastRequestUs = micros(); torqueVerifyPending = false;
      invalidateMotionFeedback(); ack(frame, motion::Ok);
    }
    return true;
  }
  if (!maintenanceMode || !maintenanceGuard.accept(cookie, frame.sequence)) {
    ack(frame, motion::BadPacket); return true;
  }
  // Consume sequence BEFORE validating command-specific data or sending UART.
  if ((length > 4 && frame.payload[4] > 253) || (frame.type == motion::MaintenanceRename && frame.payload[5] > 253)) {
    ack(frame, motion::BadPacket); return true;
  }
  maintenance.lastRequestUs = micros();
  const uint8_t id = length > 4 ? frame.payload[4] : 255;
  if (frame.type == motion::MaintenanceVoltageRead) {
    // Fixed read-only register; no host-supplied addresses or writes. Never
    // interrupt a center operation or replace its position/torque evidence.
    if (maintenance.state == 1) { ack(frame, motion::BadState); return true; }
    const ServoReply reply = readServo(id, 62, 1);
    const uint8_t outcome = maintenanceReplyOutcome(reply, 1);
    if (reply.error || outcome == 3) maintenance.faultSeen = true;
    uint8_t data[8] = {};
    motion::put32(data, cookie); data[4] = id;
    data[5] = outcome; data[6] = reply.error;
    // Unknown is encoded separately by outcome, never as a usable zero volt.
    data[7] = outcome == 0 ? reply.data[0] : 0;
    respond(0x92, frame.sequence, data, sizeof(data));
  } else if (frame.type == motion::MaintenanceRead) {
    const bool wasMoving = maintenance.state == 1;
    const MaintenanceSample sample = maintenanceSnapshot(id);
    if (sample.error) maintenance.faultSeen = true;
    if (wasMoving && (sample.error || (id == maintenance.id && sample.outcome))) maintenanceFault(sample.error);
    // A diagnostic on another address must not replace the controlled ID
    // after a movement fault, especially if its release was unconfirmed.
    if (!wasMoving) {
      maintenanceRemember(sample);
      if (sample.outcome == 1) {
        // Silence is normal while scanning an unoccupied address. Do not
        // select it for polling or turn an inventory scan into a fault.
        maintenance.id = 255;
        maintenance.position = kMaintenanceUnknownPosition;
        maintenance.torque = kMaintenanceUnknownTorque;
        if (maintenance.state != 3) maintenance.state = 0;
      } else {
        maintenance.id = id;
        if (sample.outcome) maintenance.state = 3;
      }
    }
    uint8_t data[10]; motion::put32(data, cookie); data[4] = id;
    data[5] = sample.outcome; data[6] = sample.error;
    motion::put16(data + 7, sample.position); data[9] = sample.torque;
    respond(0x8a, frame.sequence, data, sizeof(data));
  } else if (frame.type == motion::MaintenancePoll) {
    if (maintenance.id <= 253) {
      // Every host-visible position/torque here is freshly read, including
      // idle/done/fault sessions. Poll never disguises a cached value as live.
      const MaintenanceSample sample = maintenanceSnapshot(maintenance.id);
      maintenanceRemember(sample);
      if (maintenance.state == 1 && (sample.outcome || sample.torque != 1 ||
          sample.position < maintenance.minimum || sample.position > maintenance.maximum ||
          abs(int(sample.position) - int(maintenance.goal)) > 64 ||
          abs(int(sample.position) - int(maintenance.previous)) > 96)) maintenanceFault(sample.error);
      else if (sample.outcome) { maintenance.state = 3; invalidateMotionFeedback(); }
    }
    uint8_t data[10]; motion::put32(data, cookie); data[4] = maintenance.id;
    data[5] = maintenance.state; data[6] = maintenance.error;
    motion::put16(data + 7, maintenance.position); data[9] = maintenance.torque;
    respond(0x8d, frame.sequence, data, sizeof(data));
  } else if (frame.type == motion::MaintenanceRename) {
    ack(frame, maintenanceRename(id, frame.payload[5]));
  } else if (frame.type == motion::MaintenanceCenter) {
    maintenanceCenterAck(frame, maintenanceStartCenter(id, motion::u16(frame.payload + 5), motion::u16(frame.payload + 7), motion::u16(frame.payload + 9)));
  } else if (frame.type == motion::MaintenanceCenterCheck) {
    maintenanceCenterAck(frame, maintenanceStartCenter(id, 0, 0, 4095, true));
  } else if (frame.type == motion::MaintenanceRelease) {
    const bool released = maintenanceReleaseOne(id);
    if (id == maintenance.id) maintenance.state = released ? 0 : 3;
    ack(frame, released ? motion::Ok : motion::BadState);
  } else {
    ack(frame, maintenanceClose() ? motion::Ok : motion::BadState);
  }
  return true;
}
