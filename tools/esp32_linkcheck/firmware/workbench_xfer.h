#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace linkcheck {

constexpr size_t kXferMaxPacket = 13;
constexpr size_t kXferSessionCapacity = 64;

struct XferRequest {
  char session[17] = {};
  uint64_t sessionKey = 0;
  uint32_t seq = 0;
  uint8_t packet[kXferMaxPacket] = {};
  size_t size = 0;
  bool hasSession = false;
  bool hasSeq = false;
};

inline int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// This validator is separate from the diagnostic read-only encoder. There is
// no escape to another opcode, address, or payload through the XFER command.
inline const char* validateXferPacket(const uint8_t* packet, size_t size) {
  if (size < 8 || size > kXferMaxPacket || packet[0] != 0xff || packet[1] != 0xff)
    return "invalid_packet_header_or_size";
  if (packet[2] > 253) return "broadcast_or_invalid_id";
  if (static_cast<size_t>(packet[3]) + 4 != size) return "invalid_packet_length";
  uint8_t sum = 0;
  for (size_t i = 2; i < size; ++i) sum += packet[i];
  if (sum != 0xff) return "invalid_packet_checksum";
  const uint8_t op = packet[4], address = packet[5];
  if (op == 2) {
    if (size != 8) return "read_not_allowed";
    const uint8_t count = packet[6];
    const bool allowed = ((address == 5 || address == 40 || address == 55) && count == 1) ||
                         (address == 42 && count == 2) || (address == 56 && count == 6);
    return allowed ? nullptr : "read_not_allowed";
  }
  if (op != 3) return "opcode_not_allowed";
  if (size == 8) {
    const uint8_t value = packet[6];
    const bool allowed = (address == 5 && value <= 253) ||
                         ((address == 40 || address == 55) && value <= 1);
    return allowed ? nullptr : "write_not_allowed";
  }
  if (size == 13 && address == 42) {
    const uint16_t position = packet[6] | (static_cast<uint16_t>(packet[7]) << 8);
    if (position <= 4095 && packet[8] == 0 && packet[9] == 0 && packet[10] == 0 && packet[11] == 0)
      return nullptr;
  }
  return "write_not_allowed";
}

inline const char* parseXfer(const char* line, XferRequest& request) {
  request = XferRequest{};
  if (strncmp(line, "XFER ", 5) != 0) return "invalid_xfer_syntax";
  const char* session = line + 5;
  const char* separator = strchr(session, ' ');
  if (!separator || separator - session != 16) return "invalid_session";
  constexpr char digits[] = "0123456789abcdef";
  for (size_t i = 0; i < 16; ++i) {
    const int value = hexDigit(session[i]);
    if (value < 0) return "invalid_session";
    request.session[i] = digits[value];
    request.sessionKey = (request.sessionKey << 4) | static_cast<uint64_t>(value);
  }
  request.hasSession = true;
  const char* sequence = separator + 1;
  separator = strchr(sequence, ' ');
  if (!separator || separator == sequence || separator - sequence > 10) return "invalid_seq";
  for (const char* p = sequence; p != separator; ++p) {
    if (*p < '0' || *p > '9') return "invalid_seq";
    const uint32_t value = static_cast<uint32_t>(*p - '0');
    if (request.seq > (UINT32_MAX - value) / 10) return "invalid_seq";
    request.seq = request.seq * 10 + value;
  }
  request.hasSeq = true;
  const char* hex = separator + 1;
  const size_t hexSize = strlen(hex);
  if (!hexSize || hexSize % 2 || hexSize > kXferMaxPacket * 2) return "invalid_packet_hex_size";
  request.size = hexSize / 2;
  for (size_t i = 0; i < request.size; ++i) {
    const int high = hexDigit(hex[i * 2]), low = hexDigit(hex[i * 2 + 1]);
    if (high < 0 || low < 0) return "invalid_packet_hex";
    request.packet[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return validateXferPacket(request.packet, request.size);
}

class XferReplayGuard {
 public:
  // Call only after packet validation and BEFORE attempting UART transmission.
  // All accepted sequences are consumed, even when transmission later fails.
  const char* consume(uint64_t session, uint32_t seq) {
    for (size_t i = 0; i < used_; ++i) {
      if (sessions_[i].key == session) {
        if (seq <= sessions_[i].seq) return "replay_or_out_of_order";
        sessions_[i].seq = seq;
        return nullptr;
      }
    }
    if (used_ == kXferSessionCapacity) return "session_capacity_exhausted";
    sessions_[used_].key = session;
    sessions_[used_++].seq = seq;
    return nullptr;
  }

  size_t used() const { return used_; }

 private:
  struct Session { uint64_t key = 0; uint32_t seq = 0; };
  Session sessions_[kXferSessionCapacity] = {};
  size_t used_ = 0;
};

}  // namespace linkcheck
