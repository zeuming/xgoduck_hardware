/*
 * XGO-Duck link diagnostics and explicit workbench transfer bridge.
 * ESP32-S3 / Arduino-ESP32 2.0.17.
 * UART0/COM: 115200 8N1. UART1: RX18 TX17, 1 Mbps, external URT-2.
 * I2C: SDA8 SCL9, 100 kHz. This does NOT configure an IMU sampling mode.
 * No automatic downstream transactions. Diagnostic commands remain read-only.
 * XFER permits only the documented workbench read/write packet allowlist.
 */
#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <ctype.h>
#include <driver/uart.h>
#include <esp_arduino_version.h>
#include <freertos/FreeRTOS.h>
#include "servo_readonly.h"
#include "workbench_xfer.h"

// Core 2.x calls hardware UART0 Serial when native USB CDC is disabled;
// when CDC is enabled it calls the same hardware port Serial0 instead.
// Never use the native-USB Serial object for the board's COM adapter.
#if ESP_ARDUINO_VERSION_MAJOR < 3 && !ARDUINO_USB_CDC_ON_BOOT
static HardwareSerial& hostPort = Serial;
#else
static HardwareSerial& hostPort = Serial0;
#endif

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This diagnostic firmware targets ESP32-S3 only."
#endif

namespace {
constexpr int kServoRx = 18;
constexpr int kServoTx = 17;
constexpr int kSda = 8;
constexpr int kScl = 9;
constexpr uint32_t kReplyTimeoutUs = 30000;
constexpr uint32_t kTxTimeoutMs = 10;
constexpr uint32_t kXferRxTimeoutUs = 40000;
constexpr uint32_t kXferRxIdleUs = 2000;
constexpr size_t kXferRxLimit = 96;
constexpr char kPrefix[] = "XGODUCK_LINKCHECK";
bool i2cReady = false;
std::atomic<uint32_t> uartFrame{0}, uartParity{0}, uartFifo{0}, uartBuffer{0}, uartBreak{0};
linkcheck::XferReplayGuard xferReplay;

struct UartErrors {
  uint32_t frame, parity, fifo, buffer, brk;
};

UartErrors uartErrorSnapshot() {
  return {uartFrame.load(), uartParity.load(), uartFifo.load(), uartBuffer.load(), uartBreak.load()};
}

bool uartErrorsChanged(const UartErrors& a, const UartErrors& b) {
  return a.frame != b.frame || a.parity != b.parity || a.fifo != b.fifo ||
         a.buffer != b.buffer || a.brk != b.brk;
}

struct Result {
  linkcheck::Reply reply;
  linkcheck::ParseStats stats;
  uint32_t elapsedUs = 0;
  uint32_t staleBytes = 0;
  size_t pendingBytes = 0;
  bool timeout = false;
  bool busBusy = false;
  bool txError = false;
  int32_t txErrorCode = ESP_OK;
};

const char* boolean(bool value) { return value ? "true" : "false"; }

void beginJson(const char* command) {
  hostPort.printf("{\"prefix\":\"%s\",\"protocol\":1,\"command\":\"%s\"", kPrefix, command);
}

void uartError(hardwareSerial_error_t error) {
  switch (error) {
    case UART_FRAME_ERROR: ++uartFrame; break;
    case UART_PARITY_ERROR: ++uartParity; break;
    case UART_FIFO_OVF_ERROR: ++uartFifo; break;
    case UART_BUFFER_FULL_ERROR: ++uartBuffer; break;
    case UART_BREAK_ERROR: ++uartBreak; break;
    default: break;
  }
}

void printUartErrors() {
  hostPort.printf("\"uart_errors_total\":{\"frame\":%lu,\"parity\":%lu,\"fifo\":%lu,\"buffer\":%lu,\"break\":%lu}",
    static_cast<unsigned long>(uartFrame.load()), static_cast<unsigned long>(uartParity.load()),
    static_cast<unsigned long>(uartFifo.load()), static_cast<unsigned long>(uartBuffer.load()),
    static_cast<unsigned long>(uartBreak.load()));
}

void printResult(const Result& r) {
  hostPort.printf("{\"found\":%s,\"fault\":%s,\"response_id\":", boolean(r.reply.found), boolean(r.reply.found && r.reply.error != 0));
  if (r.reply.found) hostPort.print(r.reply.responseId); else hostPort.print("null");
  hostPort.print(",\"error\":");
  if (r.reply.found) hostPort.print(r.reply.error); else hostPort.print("null");
  hostPort.print(",\"data\":[");
  for (uint8_t i = 0; i < r.reply.size; ++i) {
    if (i) hostPort.print(',');
    hostPort.print(r.reply.data[i]);
  }
  hostPort.printf("],\"elapsed_us\":%lu,\"timeout\":%s,\"bus_busy\":%s,\"tx_error\":%s,\"tx_error_code\":%ld,\"stale_bytes\":%lu,\"partial_bytes\":%u,",
    static_cast<unsigned long>(r.elapsedUs), boolean(r.timeout), boolean(r.busBusy), boolean(r.txError),
    static_cast<long>(r.txErrorCode), static_cast<unsigned long>(r.staleBytes), static_cast<unsigned>(r.pendingBytes));
  hostPort.printf("\"parse\":{\"noise\":%lu,\"bad_length\":%lu,\"bad_checksum\":%lu,\"wrong_id\":%lu,\"echo\":%lu,\"overflow\":%lu}}",
    static_cast<unsigned long>(r.stats.noise), static_cast<unsigned long>(r.stats.badLength),
    static_cast<unsigned long>(r.stats.badChecksum), static_cast<unsigned long>(r.stats.wrongId),
    static_cast<unsigned long>(r.stats.echo), static_cast<unsigned long>(r.stats.overflow));
}

Result readServo(uint16_t id, uint8_t address, uint8_t size) {
  Result result;
  uint8_t request[8];
  const size_t requestSize = linkcheck::makeRead(id, address, size, request);
  if (!requestSize) { result.txError = true; result.txErrorCode = ESP_ERR_INVALID_ARG; return result; }
  const uint32_t start = micros();
  uint32_t quietSince = start;
  // Bound the drain: a continuously driven RX must not hang a command or
  // prompt us to transmit into an already busy bus.
  while (static_cast<uint32_t>(micros() - start) < 2000) {
    if (Serial1.available()) {
      Serial1.read(); ++result.staleBytes; quietSince = micros();
    } else if (static_cast<uint32_t>(micros() - quietSince) >= 300) {
      break;
    } else {
      delayMicroseconds(20);
    }
  }
  if (Serial1.available() || static_cast<uint32_t>(micros() - quietSince) < 300) {
    result.busBusy = true; result.elapsedUs = micros() - start; return result;
  }
  linkcheck::ReplyParser parser(request, size);
  if (Serial1.write(request, requestSize) != requestSize) {
    result.txError = true; result.txErrorCode = ESP_FAIL;
    result.elapsedUs = micros() - start; return result;
  }
  // Bounded TX completion; do not flush RX because it may hold a fast reply.
  const esp_err_t txStatus = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(kTxTimeoutMs));
  if (txStatus != ESP_OK) {
    result.txError = true; result.txErrorCode = txStatus;
    result.elapsedUs = micros() - start; return result;
  }
  const uint32_t sentAt = micros();
  while (static_cast<uint32_t>(micros() - sentAt) < kReplyTimeoutUs) {
    if (Serial1.available()) {
      const int byte = Serial1.read();
      if (byte >= 0 && parser.feed(static_cast<uint8_t>(byte))) break;
    } else {
      delay(1);
    }
  }
  result.reply = parser.reply;
  result.stats = parser.stats;
  result.pendingBytes = parser.pendingBytes();
  result.timeout = !result.reply.found;
  result.elapsedUs = micros() - start;
  return result;
}

bool idMatches(const Result& result, uint16_t id) {
  return result.reply.found && result.reply.error == 0 && result.reply.size == 1 && result.reply.data[0] == id;
}

void info(bool boot) {
  beginJson("INFO");
  hostPort.printf(",\"event\":\"%s\",\"firmware\":\"0.2.0\",\"core\":\"%u.%u.%u\",\"uptime_ms\":%lu,",
    boot ? "boot" : "reply", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
    ESP_ARDUINO_VERSION_PATCH, static_cast<unsigned long>(millis()));
  hostPort.printf("\"host\":{\"uart\":0,\"rx\":44,\"tx\":43,\"baud\":115200},\"servo\":{\"uart\":1,\"rx\":18,\"tx\":17,\"baud\":1000000,\"timeout_us\":%lu,\"tx_timeout_ms\":%lu},",
    static_cast<unsigned long>(kReplyTimeoutUs), static_cast<unsigned long>(kTxTimeoutMs));
  hostPort.printf("\"imu\":{\"sda\":8,\"scl\":9,\"hz\":100000,\"bus_initialized\":%s},\"automatic_queries\":false,\"automatic_servo_writes\":false,\"imu_configuration_writes\":false,",
    boolean(i2cReady));
  hostPort.printf("\"capabilities\":{\"workbench_xfer\":true,\"servo_register_writes_supported\":true,\"session_capacity\":%u},\"session_capacity\":%u,\"sessions_used\":%u,",
    static_cast<unsigned>(linkcheck::kXferSessionCapacity), static_cast<unsigned>(linkcheck::kXferSessionCapacity),
    static_cast<unsigned>(xferReplay.used()));
  printUartErrors();
  hostPort.println('}');
}

void imu() {
  beginJson("IMU");
  hostPort.print(",\"register\":15,\"expected_who_am_i\":107,\"results\":[");
  unsigned found = 0;
  for (uint8_t address = 0x6a; address <= 0x6b; ++address) {
    if (address != 0x6a) hostPort.print(',');
    const uint32_t start = micros();
    uint8_t error = 255;
    size_t count = 0;
    int value = -1;
    if (i2cReady) {
      Wire.beginTransmission(address);
      Wire.write(static_cast<uint8_t>(0x0f));  // Register pointer only, no data byte.
      error = Wire.endTransmission(false);
      if (error == 0) {
        count = Wire.requestFrom(address, static_cast<size_t>(1), true);
        if (count == 1 && Wire.available()) value = Wire.read();
      }
    }
    const bool matched = value == 0x6b;
    if (matched) ++found;
    hostPort.printf("{\"address\":%u,\"found\":%s,\"who_am_i\":", address, boolean(matched));
    if (value < 0) hostPort.print("null"); else hostPort.print(value);
    hostPort.printf(",\"i2c_error\":%u,\"bytes\":%u,\"elapsed_us\":%lu,\"read_failed\":%s}",
      error, static_cast<unsigned>(count), static_cast<unsigned long>(micros() - start),
      boolean(error != 0 || count != 1 || value < 0));
  }
  hostPort.printf("],\"found_count\":%u,\"found\":%s,\"sampling_initialized\":false}\n", found, boolean(found != 0));
}

void read(uint16_t id) {
  const Result identity = readServo(id, 5, 1);
  const bool identified = idMatches(identity, id);
  Result position, torque;
  if (identified) {
    position = readServo(id, 56, 6);
    torque = readServo(id, 40, 1);
  }
  const bool positionValid = identified && position.reply.found && position.reply.error == 0 && position.reply.size == 6;
  const bool torqueValid = identified && torque.reply.found && torque.reply.error == 0 && torque.reply.size == 1;
  const uint16_t raw = position.reply.data[0] | (static_cast<uint16_t>(position.reply.data[1]) << 8);
  const bool fault = (identity.reply.found && identity.reply.error != 0) ||
                     (position.reply.found && position.reply.error != 0) ||
                     (torque.reply.found && torque.reply.error != 0);
  const uint8_t firstError = identity.reply.error ? identity.reply.error :
                            position.reply.error ? position.reply.error : torque.reply.error;
  beginJson("READ");
  hostPort.printf(",\"id\":%u,\"found\":%s,\"id_matches\":%s,\"ok\":%s,\"fault\":%s,\"response_id\":",
    id, boolean(identity.reply.found), boolean(identified), boolean(positionValid && raw <= 4095 && torqueValid), boolean(fault));
  if (identity.reply.found) hostPort.print(identity.reply.responseId); else hostPort.print("null");
  hostPort.print(",\"read_id\":");
  if (identity.reply.found && identity.reply.size == 1) hostPort.print(identity.reply.data[0]); else hostPort.print("null");
  hostPort.print(",\"error\":");
  if (identity.reply.found) hostPort.print(firstError); else hostPort.print("null");
  hostPort.print(",\"identity\":");
  printResult(identity);
  hostPort.print(",\"position_read\":"); if (identified) printResult(position); else hostPort.print("null");
  hostPort.print(",\"torque_read\":"); if (identified) printResult(torque); else hostPort.print("null");
  hostPort.print(",\"position\":"); if (positionValid) hostPort.print(raw); else hostPort.print("null");
  hostPort.print(",\"torque\":"); if (torqueValid) hostPort.print(torque.reply.data[0]); else hostPort.print("null");
  hostPort.print(','); printUartErrors(); hostPort.println('}');
}

void scan() {
  const uint32_t start = micros();
  unsigned found = 0, matched = 0, timeouts = 0, busy = 0, txErrors = 0;
  uint32_t noise = 0, badLength = 0, badChecksum = 0, wrongId = 0, echo = 0, partial = 0, stale = 0;
  beginJson("SCAN");
  hostPort.print(",\"first_id\":0,\"last_id\":253,\"method\":\"read_id_register\",\"results\":[");
  for (uint16_t id = 0; id <= 253; ++id) {
    const Result result = readServo(id, 5, 1);
    timeouts += result.timeout; busy += result.busBusy; txErrors += result.txError;
    noise += result.stats.noise; badLength += result.stats.badLength;
    badChecksum += result.stats.badChecksum; wrongId += result.stats.wrongId;
    echo += result.stats.echo; partial += result.pendingBytes != 0; stale += result.staleBytes;
    if (result.reply.found) {
      if (found++) hostPort.print(',');
      const bool matches = idMatches(result, id);
      matched += matches;
      hostPort.printf("{\"id\":%u,\"response_id\":%u,\"id_matches\":%s,\"fault\":%s,\"error\":%u,\"read_id\":",
        id, result.reply.responseId, boolean(matches), boolean(result.reply.error != 0), result.reply.error);
      if (result.reply.size == 1) hostPort.print(result.reply.data[0]); else hostPort.print("null");
      hostPort.print(",\"reply\":");
      printResult(result); hostPort.print('}');
    }
    delay(1);
  }
  hostPort.printf("],\"attempted\":254,\"found\":%s,\"found_count\":%u,\"matched_count\":%u,\"elapsed_us\":%lu,",
    boolean(found != 0), found, matched, static_cast<unsigned long>(micros() - start));
  hostPort.printf("\"timeouts\":%u,\"bus_busy\":%u,\"tx_errors\":%u,\"parse_total\":{\"noise\":%lu,\"bad_length\":%lu,\"bad_checksum\":%lu,\"wrong_id\":%lu,\"echo\":%lu,\"partial_transactions\":%lu,\"stale_bytes\":%lu},",
    timeouts, busy, txErrors, static_cast<unsigned long>(noise), static_cast<unsigned long>(badLength),
    static_cast<unsigned long>(badChecksum), static_cast<unsigned long>(wrongId), static_cast<unsigned long>(echo),
    static_cast<unsigned long>(partial), static_cast<unsigned long>(stale));
  printUartErrors(); hostPort.println('}');
}

struct XferResult {
  uint8_t rx[kXferRxLimit] = {};
  size_t rxSize = 0;
  uint32_t elapsedUs = 0;
  uint32_t staleBytes = 0;
  int32_t txErrorCode = ESP_OK;
  const char* error = nullptr;
  bool ok = false;
  bool sequenceConsumed = false;
  bool txSent = false;
  bool txComplete = false;
  bool uncertain = false;
};

// Exactly one UART write per accepted request. This function never retries a
// frame, even if no acknowledgement arrives or transmission status is unknown.
XferResult transferOnce(const linkcheck::XferRequest& request) {
  XferResult result;
  result.sequenceConsumed = true;
  const uint32_t start = micros();
  uint32_t quietSince = start;
  const UartErrors beforeDrain = uartErrorSnapshot();
  while (static_cast<uint32_t>(micros() - start) < 2000) {
    if (Serial1.available()) {
      Serial1.read(); ++result.staleBytes; quietSince = micros();
    } else if (static_cast<uint32_t>(micros() - quietSince) >= 300) {
      break;
    } else {
      delayMicroseconds(20);
    }
  }
  if (Serial1.available() || static_cast<uint32_t>(micros() - quietSince) < 300) {
    result.error = "bus_busy_before_tx";
    result.elapsedUs = micros() - start;
    return result;
  }
  const UartErrors beforeTx = uartErrorSnapshot();
  if (uartErrorsChanged(beforeDrain, beforeTx)) {
    result.error = "uart_error_before_tx";
    result.elapsedUs = micros() - start;
    return result;
  }
  // From this point onward a failed transaction may already have affected the
  // servo. The sequence was consumed before reaching this function.
  result.txSent = true;
  if (Serial1.write(request.packet, request.size) != request.size) {
    result.error = "tx_short_write";
    result.txErrorCode = ESP_FAIL;
    result.uncertain = true;
    result.elapsedUs = micros() - start;
    return result;
  }
  const esp_err_t txStatus = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(kTxTimeoutMs));
  if (txStatus != ESP_OK) {
    result.error = txStatus == ESP_ERR_TIMEOUT ? "tx_timeout" : "tx_driver_error";
    result.txErrorCode = txStatus;
    result.uncertain = true;
    result.elapsedUs = micros() - start;
    return result;
  }
  result.txComplete = true;
  const uint32_t rxStart = micros();
  uint32_t lastByteUs = rxStart;
  while (static_cast<uint32_t>(micros() - rxStart) < kXferRxTimeoutUs) {
    if (uartErrorsChanged(beforeTx, uartErrorSnapshot())) {
      result.error = "uart_hardware_error";
      result.uncertain = true;
      break;
    }
    if (Serial1.available()) {
      const int value = Serial1.read();
      if (value >= 0) {
        if (result.rxSize == kXferRxLimit) {
          result.error = "rx_overflow";
          result.uncertain = true;
          break;
        }
        result.rx[result.rxSize++] = static_cast<uint8_t>(value);
        lastByteUs = micros();
      }
    } else if (result.rxSize && static_cast<uint32_t>(micros() - lastByteUs) >= kXferRxIdleUs) {
      break;
    } else {
      delay(1);
    }
  }
  if (!result.error && uartErrorsChanged(beforeTx, uartErrorSnapshot())) {
    result.error = "uart_hardware_error";
    result.uncertain = true;
  }
  if (!result.error && Serial1.available()) {
    // Data arriving at the deadline cannot be called a complete raw capture.
    result.error = result.rxSize == kXferRxLimit ? "rx_overflow" : "rx_deadline_with_pending_data";
    result.uncertain = true;
  }
  result.ok = result.error == nullptr;
  result.elapsedUs = micros() - start;
  return result;
}

void printXfer(const linkcheck::XferRequest& request, const XferResult& result) {
  beginJson("XFER");
  hostPort.print(",\"session\":");
  if (request.hasSession) hostPort.printf("\"%s\"", request.session); else hostPort.print("null");
  hostPort.print(",\"seq\":");
  if (request.hasSeq) hostPort.printf("%lu", static_cast<unsigned long>(request.seq)); else hostPort.print("null");
  hostPort.printf(",\"ok\":%s,\"rx_hex\":\"", boolean(result.ok));
  constexpr char hex[] = "0123456789abcdef";
  for (size_t i = 0; i < result.rxSize; ++i) {
    hostPort.print(hex[result.rx[i] >> 4]); hostPort.print(hex[result.rx[i] & 15]);
  }
  hostPort.print("\",\"error\":");
  if (result.error) hostPort.printf("\"%s\"", result.error); else hostPort.print("null");
  hostPort.printf(",\"sequence_consumed\":%s,\"tx_sent\":%s,\"tx_complete\":%s,\"uncertain\":%s,\"tx_error_code\":%ld,\"elapsed_us\":%lu,\"stale_bytes\":%lu,\"session_capacity\":%u,",
    boolean(result.sequenceConsumed), boolean(result.txSent), boolean(result.txComplete), boolean(result.uncertain),
    static_cast<long>(result.txErrorCode), static_cast<unsigned long>(result.elapsedUs),
    static_cast<unsigned long>(result.staleBytes), static_cast<unsigned>(linkcheck::kXferSessionCapacity));
  printUartErrors(); hostPort.println('}');
}

void xfer(const char* line) {
  linkcheck::XferRequest request;
  XferResult result;
  result.error = linkcheck::parseXfer(line, request);
  if (!result.error) {
    result.error = xferReplay.consume(request.sessionKey, request.seq);
    if (!result.error) result = transferOnce(request);
    else if (strcmp(result.error, "replay_or_out_of_order") == 0) result.uncertain = true;
  }
  printXfer(request, result);
}

void error(const char* reason) {
  beginJson("ERROR"); hostPort.printf(",\"ok\":false,\"error\":\"%s\"}\n", reason);
}

void command(char* line) {
  if (strncmp(line, "XFER", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) { xfer(line); return; }
  if (strcmp(line, "INFO") == 0) { info(false); return; }
  if (strcmp(line, "IMU") == 0) { imu(); return; }
  if (strcmp(line, "SCAN") == 0) { scan(); return; }
  if (strncmp(line, "READ ", 5) == 0) {
    const char* p = line + 5;
    if (!*p) { error("expected_id_0_to_253"); return; }
    unsigned id = 0;
    while (*p) {
      if (!isdigit(static_cast<unsigned char>(*p))) { error("expected_id_0_to_253"); return; }
      id = id * 10 + (*p++ - '0');
      if (id > 253) { error("expected_id_0_to_253"); return; }
    }
    read(static_cast<uint16_t>(id)); return;
  }
  error("unknown_command");
}
}  // namespace

void setup() {
  hostPort.begin(115200, SERIAL_8N1, 44, 43);
  Serial1.setRxBufferSize(256);
  Serial1.begin(1000000, SERIAL_8N1, kServoRx, kServoTx);
  Serial1.setRxFIFOFull(1);
  Serial1.onReceiveError(uartError);
  Wire.setTimeOut(20);
  i2cReady = Wire.begin(kSda, kScl, 100000);
  info(true);
}

void loop() {
  static char line[180];
  static size_t used = 0;
  static bool rejected = false;
  static bool sawCr = false;
  while (hostPort.available()) {
    const int input = hostPort.read();
    if (input < 0) break;
    const char c = static_cast<char>(input);
    if (c == '\r') {
      if (sawCr) rejected = true;
      sawCr = true;
      continue;
    }
    if (c == '\n') {
      if (rejected) {
        line[used] = '\0';
        if (used >= 4 && strncmp(line, "XFER", 4) == 0) {
          linkcheck::XferRequest request;
          linkcheck::parseXfer(line, request);  // Correlation only; never send.
          XferResult result;
          result.error = "line_too_long_or_non_ascii";
          printXfer(request, result);
        } else error("line_too_long_or_non_ascii");
      }
      else if (used) { line[used] = '\0'; command(line); }
      used = 0; rejected = false; sawCr = false;
    } else if (sawCr || c < 0x20 || c > 0x7e || used >= sizeof(line) - 1) {
      rejected = true;
    } else if (!rejected) {
      line[used++] = c;
    }
  }
  delay(1);
}
