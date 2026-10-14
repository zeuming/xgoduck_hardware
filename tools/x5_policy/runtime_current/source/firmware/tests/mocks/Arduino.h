#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <vector>
#include <functional>
#define CONFIG_IDF_TARGET_ESP32S3 1
#define ARDUINO_USB_CDC_ON_BOOT 0
#define SERIAL_8N1 0
extern uint32_t mockMicros;
inline uint32_t micros() { return mockMicros++; }
inline void delayMicroseconds(uint32_t us) { mockMicros += us; }
inline void delay(uint32_t ms) { mockMicros += ms * 1000; }
using hardwareSerial_error_t = int;
class HardwareSerial {
 public:
  std::deque<uint8_t> input;
  std::deque<std::pair<uint32_t, uint8_t>> scheduledInput;
  std::vector<uint8_t> output;
  std::function<void(const uint8_t*, size_t)> onWrite;
  int writeResultOverride = -1;
  unsigned baud = 0;
  uint32_t readCostUs = 0;
  void setRxBufferSize(size_t) {}
  void setRxFIFOFull(size_t) {}
  void onReceiveError(void (*)(hardwareSerial_error_t)) {}
  void begin(unsigned speed, int, int, int) { baud = speed; }
  int available() {
    while (!scheduledInput.empty() && int32_t(mockMicros - scheduledInput.front().first) >= 0) {
      input.push_back(scheduledInput.front().second); scheduledInput.pop_front();
    }
    return int(input.size());
  }
  int read() { if (input.empty()) return -1; int value = input.front(); input.pop_front(); mockMicros += readCostUs; return value; }
  size_t write(const uint8_t* bytes, size_t size) {
    output.insert(output.end(), bytes, bytes + size);
    if (onWrite) onWrite(bytes, size);
    return writeResultOverride < 0 ? size : size_t(writeResultOverride);
  }
};
extern HardwareSerial Serial, Serial1;
