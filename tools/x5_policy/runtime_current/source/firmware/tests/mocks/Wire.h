#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  uint8_t registers[256] = {};
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  uint8_t address = 0, pointer = 0;
  bool online = true;
  bool busReady = false, beginResult = true, endResult = true;
  bool deriveDataReady = false, resetStuck = false;
  size_t beginCalls = 0, endCalls = 0;
  int beginSda = -1, beginScl = -1;
  unsigned beginFrequency = 0, timeOut = 0;
  uint32_t beginDelayUs = 0, endDelayUs = 0;
  std::function<void(uint8_t)> onRead;
  void setTimeOut(unsigned value) { timeOut = value; }
  bool begin(int sda, int scl, unsigned frequency) {
    ++beginCalls; beginSda = sda; beginScl = scl; beginFrequency = frequency;
    mockMicros += beginDelayUs; busReady = beginResult; return beginResult;
  }
  bool end() {
    ++endCalls; mockMicros += endDelayUs;
    if (endResult) { busReady = false; tx.clear(); rx.clear(); }
    return endResult;
  }
  void powerCycle() {
    // After actual sensor power loss, both ODRs default to power-down. A
    // still-running host must reconfigure them before data becomes ready.
    for (size_t i = 0; i < sizeof(registers); ++i) registers[i] = 0;
    registers[0x0f] = 0x6b; deriveDataReady = true;
  }
  void beginTransmission(uint8_t target) { address = target; tx.clear(); }
  void write(uint8_t value) { tx.push_back(value); }
  uint8_t endTransmission(bool) {
    if (!busReady || !online || address != 0x6a || tx.empty()) return 2;
    pointer = tx[0];
    for (size_t i = 1; i < tx.size(); ++i) registers[uint8_t(pointer + i - 1)] = tx[i];
    if (pointer == 0x12 && tx.size() == 2 && tx[1] == 1) {
      if (deriveDataReady) powerCycle();
      registers[0x12] = resetStuck ? 1 : 0;
    }
    if (deriveDataReady)
      registers[0x1e] = uint8_t(((registers[0x10] & 0xf0) ? 1 : 0) |
                                ((registers[0x11] & 0xf0) ? 2 : 0));
    return 0;
  }
  size_t requestFrom(uint8_t target, size_t count, bool) {
    rx.clear(); if (!busReady || !online || target != 0x6a) return 0;
    if (onRead) onRead(pointer);
    for (size_t i = 0; i < count; ++i) rx.push_back(registers[uint8_t(pointer + i)]);
    return count;
  }
  int available() { return int(rx.size()); }
  int read() { int value = rx.front(); rx.pop_front(); return value; }
};
extern TwoWire Wire;
