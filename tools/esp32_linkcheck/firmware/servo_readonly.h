#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Deliberately no generic instruction encoder and no write-register function.
// The only possible request is READ (0x02), to three fixed readback registers.
namespace linkcheck {

struct ParseStats {
  uint32_t noise = 0;
  uint32_t badLength = 0;
  uint32_t badChecksum = 0;
  uint32_t wrongId = 0;
  uint32_t echo = 0;
  uint32_t overflow = 0;
};

inline size_t makeRead(uint16_t id, uint8_t address, uint8_t size, uint8_t out[8]) {
  if (id > 253 || !((address == 5 && size == 1) ||
                    (address == 56 && size == 6) ||
                    (address == 40 && size == 1))) return 0;
  out[0] = out[1] = 0xff;
  out[2] = static_cast<uint8_t>(id);
  out[3] = 4;
  out[4] = 2;
  out[5] = address;
  out[6] = size;
  uint8_t sum = 0;
  for (size_t i = 2; i < 7; ++i) sum += out[i];
  out[7] = static_cast<uint8_t>(~sum);
  return 8;
}

struct Reply {
  bool found = false;
  uint8_t responseId = 0;
  uint8_t error = 0;
  uint8_t data[6] = {};
  uint8_t size = 0;
};

class ReplyParser {
 public:
  ReplyParser(const uint8_t request[8], uint8_t expectedSize)
      : expected_(expectedSize) { memcpy(request_, request, sizeof(request_)); }

  ParseStats stats;
  Reply reply;

  bool feed(uint8_t byte) {
    if (used_ == sizeof(buffer_)) { discard(1); ++stats.overflow; }
    buffer_[used_++] = byte;
    while (used_ >= 2) {
      if (buffer_[0] != 0xff || buffer_[1] != 0xff) {
        discard(1); ++stats.noise; continue;
      }
      if (used_ < 4) return false;
      const uint8_t length = buffer_[3];
      // A read reply, an error-only reply, or our 8-byte request echo.
      // Reject other lengths immediately so a bogus length cannot hide a reply.
      if (length != expected_ + 2 && length != 2 && length != 4) {
        discard(1); ++stats.badLength; continue;
      }
      const size_t frameSize = static_cast<size_t>(length) + 4;
      if (used_ < frameSize) return false;
      uint8_t sum = 0;
      for (size_t i = 2; i < frameSize; ++i) sum += buffer_[i];
      if (sum != 0xff) { discard(1); ++stats.badChecksum; continue; }
      if (frameSize == sizeof(request_) &&
          memcmp(buffer_, request_, sizeof(request_)) == 0) {
        discard(frameSize); ++stats.echo; continue;
      }
      if (buffer_[2] != request_[2]) {
        discard(frameSize); ++stats.wrongId; continue;
      }
      const uint8_t error = buffer_[4];
      if (length != expected_ + 2 && !(length == 2 && error != 0)) {
        discard(frameSize); ++stats.badLength; continue;
      }
      reply.found = true;
      reply.responseId = buffer_[2];
      reply.error = error;
      reply.size = length - 2;
      if (reply.size) memcpy(reply.data, buffer_ + 5, reply.size);
      discard(frameSize);
      return true;
    }
    return false;
  }

  size_t pendingBytes() const { return used_; }

 private:
  uint8_t request_[8] = {};
  uint8_t buffer_[16] = {};
  size_t used_ = 0;
  uint8_t expected_;

  void discard(size_t count) {
    used_ -= count;
    memmove(buffer_, buffer_ + count, used_);
  }
};

}  // namespace linkcheck
