#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace motion {
constexpr size_t kMotorCount = 15;
constexpr uint8_t kMotorIds[kMotorCount] = {10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34};
constexpr uint16_t kAllMotors = 0x7fff;
constexpr size_t kHeaderSize = 10;
constexpr size_t kMaxPayload = 256;
constexpr size_t kStateSize = 106;
constexpr size_t kVoltageStateSize = kStateSize + 80;
constexpr uint32_t kVoltageFreshUs = 1000000;
constexpr size_t kConfigSize = 105;
constexpr uint8_t kFaultDetailsVersion = 1;
constexpr size_t kFaultDetailsHeaderSize = 25;
constexpr size_t kFaultDetailsChannelSize = 111;
constexpr size_t kFaultDetailsSize = kFaultDetailsHeaderSize + 2 * kFaultDetailsChannelSize;
constexpr uint8_t kPositionReadDetailsVersion = 1;
constexpr size_t kPositionReadDetailsSize = 22;
constexpr uint8_t kGoalReadDetailsVersion = 1;
constexpr size_t kGoalReadDetailsSize = 78;
constexpr uint32_t kGoalReadDetailsPeriodUs = 500000;
constexpr uint8_t kOffRegisterDetailsVersion = 1;
constexpr uint8_t kOffRegisterBlockCount = 8;
constexpr size_t kOffRegisterDetailsSize = 42 + kOffRegisterBlockCount * 20;
constexpr size_t kCenterFailureAckSize = 101;
constexpr uint32_t kStateFreshUs = 50000;
constexpr uint32_t kWatchdogUs = 250000;
constexpr float kSlewCountsPerSec = 600.0f;
enum Type : uint8_t {
  Hello = 1, State = 2, ConfigRequest = 3, Arm = 4, Target = 5, Stop = 6, FaultDetails = 7,
  MaintenanceCaps = 8, MaintenanceEnter = 9, MaintenanceRead = 10,
  MaintenanceRename = 11, MaintenanceCenter = 12, MaintenancePoll = 13,
  MaintenanceRelease = 14, MaintenanceExit = 15, MaintenanceCenterCheck = 16,
  MaintenanceVoltageCaps = 17, MaintenanceVoltageRead = 18,
  VoltageState = 19, TelemetryCaps = 20, PositionReadDetails = 21, GoalReadDetails = 22,
  OffRegisterDetails = 23
};
enum Error : uint8_t {
  Ok = 0, BadPacket = 1, BadState = 2, BadConfig = 3, Stale = 4, Range = 5,
  CenterSnapshot = 16, CenterGoalWrite = 17, CenterGoalRead = 18,
  CenterHoldingTorqueRead = 19, CenterEnableWrite = 20, CenterEnableRead = 21,
  CenterTorqueOffRead = CenterHoldingTorqueRead // Historical wire-code alias.
};
enum ReadOutcome : uint8_t {
  NotAttempted = 0, ValidNormalReply = 1, ValidDeviceErrorReply = 2,
  BusNotQuiet = 3, ShortUartWrite = 4, HardwareUartError = 5, NoValidReplyByDeadline = 6,
  InvalidRegisterReply = 7
};

inline uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
inline uint32_t u32(const uint8_t* p) { return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16); }
inline void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) { put16(p, uint16_t(v)); put16(p + 2, uint16_t(v >> 16)); }
inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xffff;
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (int b = 0; b < 8; ++b) crc = crc & 0x8000 ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
inline int16_t speedSignMagnitude(uint16_t raw) {
  const int16_t magnitude = int16_t(raw & 0x7fff);
  return raw & 0x8000 ? int16_t(-magnitude) : magnitude;
}

struct Frame {
  uint8_t type = 0;
  uint16_t length = 0;
  uint32_t sequence = 0;
  uint8_t payload[kMaxPayload] = {};
};
inline size_t encode(uint8_t type, uint32_t sequence, const uint8_t* payload, size_t length, uint8_t* out) {
  if (length > kMaxPayload) return 0;
  out[0] = 'X'; out[1] = 'D'; out[2] = 1; out[3] = type;
  put16(out + 4, uint16_t(length)); put32(out + 6, sequence);
  if (length) memcpy(out + kHeaderSize, payload, length);
  put16(out + kHeaderSize + length, crc16(out, kHeaderSize + length));
  return kHeaderSize + length + 2;
}

// Sliding resynchronization keeps a valid frame following corrupt input usable.
// CRC failures/version errors never cause any command or reflected ACK.
class Parser {
 public:
  Frame frame;
  uint32_t rejected = 0;
  size_t pending() const { return used_; }
  void reset() { used_ = 0; }
  bool feed(uint8_t byte) {
    if (used_ == sizeof(buffer_)) discard(1);
    buffer_[used_++] = byte;
    while (used_) {
      if (buffer_[0] != 'X') { discard(1); continue; }
      if (used_ < 2) return false;
      if (buffer_[1] != 'D') { discard(1); continue; }
      if (used_ < kHeaderSize) return false;
      const uint16_t length = u16(buffer_ + 4);
      if (buffer_[2] != 1 || length > kMaxPayload) { ++rejected; discard(1); continue; }
      const size_t total = kHeaderSize + length + 2;
      if (used_ < total) return false;
      if (crc16(buffer_, total - 2) != u16(buffer_ + total - 2)) { ++rejected; discard(1); continue; }
      frame.type = buffer_[3]; frame.length = length; frame.sequence = u32(buffer_ + 6);
      if (length) memcpy(frame.payload, buffer_ + kHeaderSize, length);
      discard(total);
      return true;
    }
    return false;
  }
 private:
  uint8_t buffer_[kHeaderSize + kMaxPayload + 2] = {};
  size_t used_ = 0;
  void discard(size_t count) { used_ -= count; memmove(buffer_, buffer_ + count, used_); }
};

class SequenceGuard {
 public:
  void reset() { seen_ = false; last_ = 0; }
  bool consume(uint32_t sequence) {
    if (seen_ && sequence <= last_) return false;
    last_ = sequence; seen_ = true; return true;
  }
 private:
  bool seen_ = false;
  uint32_t last_ = 0;
};

// Independent of HELLO's motion sequence guard. A nonce is burned on entry,
// and is never reused during this boot, including after a timeout or STOP.
// Refuse new sessions when capacity is exhausted rather than evicting replay
// evidence. Each fresh nonce may begin at a new host sequence number.
class MaintenanceGuard {
 public:
  static constexpr size_t kCapacity = 64;
  bool enter(uint32_t cookie, uint32_t sequence) {
    if (!cookie || active_ || used_ == kCapacity) return false;
    for (size_t i = 0; i < used_; ++i) if (cookies_[i] == cookie) return false;
    cookies_[used_++] = cookie; active_ = cookie;
    sequence_.reset(); return sequence_.consume(sequence);
  }
  bool accept(uint32_t cookie, uint32_t sequence) {
    return active_ && cookie == active_ && sequence_.consume(sequence);
  }
  void close() { active_ = 0; }
  uint32_t cookie() const { return active_; }
 private:
  uint32_t cookies_[kCapacity] = {}, active_ = 0;
  size_t used_ = 0;
  SequenceGuard sequence_;
};

struct Config {
  uint16_t zero[kMotorCount] = {};
  int8_t sign[kMotorCount] = {};
  uint16_t minimum[kMotorCount] = {};
  uint16_t maximum[kMotorCount] = {};
  bool decode(const uint8_t* payload, size_t length) {
    if (length != kConfigSize) return false;
    Config candidate;
    for (size_t i = 0; i < kMotorCount; ++i) {
      candidate.zero[i] = u16(payload + i * 2);
      candidate.sign[i] = int8_t(payload[30 + i]);
      candidate.minimum[i] = u16(payload + 45 + i * 2);
      candidate.maximum[i] = u16(payload + 75 + i * 2);
      if (candidate.maximum[i] > 4095 || candidate.minimum[i] > candidate.maximum[i] ||
          candidate.zero[i] < candidate.minimum[i] || candidate.zero[i] > candidate.maximum[i] ||
          (candidate.sign[i] != 1 && candidate.sign[i] != -1)) return false;
    }
    *this = candidate; return true;
  }
  bool contains(const uint16_t* targets) const {
    for (size_t i = 0; i < kMotorCount; ++i)
      if (targets[i] < minimum[i] || targets[i] > maximum[i]) return false;
    return true;
  }
};

struct Feedback {
  uint32_t sampleUs = 0, imuUs = 0;
  uint16_t onlineMask = 0, faultMask = 0, torqueMask = 0;
  bool imuOk = false;
  uint16_t positions[kMotorCount] = {};
  bool fresh(uint32_t now) const {
    // Separate booleans/masks, rather than time == 0, support micros() wrap.
    return onlineMask == kAllMotors && faultMask == 0 && imuOk &&
           uint32_t(now - sampleUs) <= kStateFreshUs && uint32_t(now - imuUs) <= kStateFreshUs;
  }
};

// Hardware independent interlocks are tested natively, and also run during
// bounded bus reads. Hardware torque writes are owned only by main.cpp.
class Interlock {
 public:
  Config config;
  bool configured = false, armed = false;
  uint32_t cookie = 0, lastCommandUs = 0;
  Error lastError = Ok;
  uint16_t desired[kMotorCount] = {}, goals[kMotorCount] = {};
  float continuousGoals[kMotorCount] = {};
  Error configure(const Frame& frame) {
    if (armed) return record(BadState);
    if (!config.decode(frame.payload, frame.length)) return record(BadConfig);
    configured = true; return record(Ok);
  }
  Error prepareArm(const Frame& frame, const Feedback& feedback, uint32_t now) {
    if (frame.length != 4) return record(BadPacket);
    if (armed || !configured || u32(frame.payload) == 0 || feedback.faultMask != 0 || feedback.torqueMask != 0)
      return record(BadState);
    if (!feedback.fresh(now)) return record(Stale);
    if (!config.contains(feedback.positions)) return record(Range);
    cookie = u32(frame.payload);
    for (size_t i = 0; i < kMotorCount; ++i) {
      desired[i] = goals[i] = feedback.positions[i]; continuousGoals[i] = feedback.positions[i];
    }
    lastCommandUs = now; slewUs_ = now;
    // Caller writes these current-position goals BEFORE enabling torque,
    // then verifies torque and sets armed. prepareArm itself cannot arm.
    return record(Ok);
  }
  Error target(const Frame& frame, uint32_t now) {
    if (frame.length != 34) return record(BadPacket);
    if (!armed || cookie == 0 || u32(frame.payload) != cookie) return record(BadState);
    uint16_t candidate[kMotorCount];
    for (size_t i = 0; i < kMotorCount; ++i) candidate[i] = u16(frame.payload + 4 + i * 2);
    if (!config.contains(candidate)) return record(Range);
    memcpy(desired, candidate, sizeof(desired));
    lastCommandUs = now; return record(Ok);
  }
  bool check(const Feedback& feedback, uint32_t now) {
    if (!armed) return false;
    if (feedback.faultMask) { disarm(BadState); return true; }
    if (!feedback.fresh(now) || uint32_t(now - lastCommandUs) > kWatchdogUs) { disarm(Stale); return true; }
    return false;
  }
  void disarm(Error error = Ok) { armed = false; cookie = 0; lastError = error; }
  void slew(uint32_t now) {
    if (!armed) return;
    const uint32_t elapsed = uint32_t(now - slewUs_);
    slewUs_ = now;
    const float step = kSlewCountsPerSec * float(elapsed > 50000 ? 50000 : elapsed) / 1000000.0f;
    for (size_t i = 0; i < kMotorCount; ++i) {
      const float delta = float(desired[i]) - continuousGoals[i];
      continuousGoals[i] += delta < -step ? -step : delta > step ? step : delta;
      goals[i] = uint16_t(lroundf(continuousGoals[i]));
    }
  }
  Error record(Error error) { lastError = error; return error; }
 private:
  uint32_t slewUs_ = 0;
};
}  // namespace motion
