# ESP32-S3 link diagnostics and workbench bridge (0.2.0)

Source: `main.cpp`, with PlatformIO `src_dir = firmware`. Target the official
**PlatformIO espressif32@6.12.0 / Arduino-ESP32 2.0.17** package and an ESP32-S3
board definition matched to the board. The core's
`Arduino`, `HardwareSerial`, `Wire`, and bundled ESP-IDF UART APIs are used; no ArduinoJson or sensor
library is needed. PSRAM is not used. Select flash size and partition layout
only after verifying the board; this directory does not set or flash them.

| Interface | Settings |
| --- | --- |
| X5 through onboard COM adapter | UART0, 115200, 8N1; RX GPIO44 / TX GPIO43 |
| URT-2 UART interface | UART1 RX GPIO18, TX GPIO17, 1,000,000, 8N1 |
| LSM6DSR I2C | SDA GPIO8, SCL GPIO9, 100 kHz, transaction timeout 20 ms |

The code selects hardware UART0 as `Serial` with CDC disabled (Core 2.x),
or `Serial0` otherwise. Use UART0/COM, not a native-USB CDC endpoint. Configure the
host terminal with no hardware/software flow control. ROM boot messages can
precede the JSON boot line. UART error messages printed by the core at a
nonzero debug level are outside this protocol; keep the core debug level off.

## Commands and replies

ASCII commands are case-sensitive, newline terminated, one at a time. Every
application reply is one JSON line with `prefix: "XGODUCK_LINKCHECK"`,
`command: "INFO" / "IMU" / "SCAN" / "READ" / "XFER" / "ERROR"`, and `protocol: 1`.
The startup INFO line is the **only automatic output** and does
not query either downstream bus. Empty lines are ignored. Other commands,
non-ASCII input, malformed line endings, and lines longer than 179 characters
return JSON errors. LF and CRLF are accepted; embedded CR, tabs, extra XFER
spaces, and trailing whitespace are rejected. Send the next command only
after the current command's complete JSON reply.

- `INFO`: firmware/core version, pin settings, uptime, and cumulative UART
  framing/parity/FIFO/buffer/break counters. No downstream transaction.
  `capabilities.workbench_xfer` and `servo_register_writes_supported` are true.
  It also reports `session_capacity: 64` and `sessions_used`.
- `IMU`: only reads register `0x0F` at seven-bit addresses `0x6A` and `0x6B`.
  `found` means WHO_AM_I returned `0x6B` (decimal 107). It reports I2C errors,
  received byte count and elapsed time. No CTRL-register writes, reset, or
  sampling initialization. Sending the register pointer byte is required for
  a register read and does not write its contents.
- `SCAN`: individually reads servo register 5 at IDs **0 through 253**.
  Expect approximately 8 seconds with no responses; allow at least 20 seconds
  on the host. `results` contains valid replies (including servo fault replies),
  while `matched_count` counts successful ID-register readbacks. Timeout and
  parser counts cover all attempts. Never uses broadcast ID 254.
- `READ <id>`: reads register 5; only if the returned ID matches, reads 6 bytes
  starting at register 56 and current torque register 40 (1 byte). The first
  two little-endian bytes at 56 are the position. All are reads.
  Unavailable values are `null`, not fabricated zero values. `ok` requires
  successful readbacks and a position in the HD1910 single-turn range 0–4095.
  Top-level `response_id` is the validated frame ID and `read_id` is the raw ID
  register byte; `position`, `torque`, and `error` are raw values. `fault`
  distinguishes a device fault from a missing response. Per-register replies
  retain every raw error byte even if a later read fails.

The diagnostic servo encoder is deliberately restricted to READ instruction `0x02` and
these three address/length combinations. It has no generic instruction or
write function. SCAN uses a register read rather than PING because a PING
request echo can be byte-identical to a fault response. READ replies are
validated for header, ID, length and checksum; exact request echoes are
discarded. Position readback uses 6 bytes (matching the existing workbench),
so its 12-byte response cannot be confused with the 8-byte request echo even
when a fault and position bytes happen to match the instruction and parameters.
Fragmented replies and leading noise are supported. Fault replies
are visible as `found: true` plus a nonzero `error`, not reported as healthy.
Malformed replies never count as found. A valid frame does not prove that
only one physical servo has that ID.

Each servo transaction reports elapsed time, timeout, a busy bus, a short TX,
stale/partial bytes, and parser counts. UART hardware error counters are
cumulative since boot. TX completion uses IDF `uart_wait_tx_done` with a 10 ms
limit; a failure returns `tx_error: true` and the raw `tx_error_code` without
waiting for a servo reply. RX is not cleared after transmission. I2C
`i2c_error` is the Wire return code; 255 means the
MCU I2C bus did not initialize. With a repeated-start read, some core versions
defer the pointer phase until `requestFrom`; code 0 by itself is not proof of
an ACK. Always check `bytes`, `read_failed`, and the returned identity together.
WHO_AM_I alone verifies the identity link,
not the current sensor sampling settings or axes.

The four diagnostic commands never change servo registers. Startup does not
query a servo, enable or disable torque, or write a target. Only an explicit,
valid `XFER` command can issue one of the writes documented below. Do not run
another controller on the same bus during diagnostics or workbench transfers.

## Explicit workbench transfers

```
XFER <16hex_session> <decimal_seq> <hex_packet>
```

The session is exactly 16 hexadecimal digits, accepted in either case and
returned in lowercase. `seq` is a decimal uint32 (0 through 4294967295), with
1–10 digits. The packet is continuous hexadecimal without a `0x` prefix or
spaces; case is ignored. An allowed packet is 8 or 13 bytes, and must have
`FF FF`, a destination ID of 0–253, its exact protocol length, and a valid
one's-complement checksum. Broadcast IDs and all unlisted packets are refused.

| Instruction | Address | Bytes | Allowed payload |
| --- | --- | --- | --- |
| READ `0x02` | 5 (ID), 40 (torque), 55 (lock) | 1 each | Exact read count |
| READ `0x02` | 42 (target) | 2 | Exact read count |
| READ `0x02` | 56 (position and following state) | 6 | Exact read count |
| WRITE `0x03` | 5 (ID) | 1 | New ID 0–253 |
| WRITE `0x03` | 40 (torque), 55 (lock) | 1 each | 0 or 1 |
| WRITE `0x03` | 42 (target block) | 6 | Little-endian target 0–4095, followed by four zero bytes |

No PING, broadcast, synchronized write, factory reset, baud-rate change, or
other register operation is available through XFER. This is a packet
allowlist, not a motion controller: the host workbench must still enforce its
own ID selection, user approval, motion limits, and write/readback sequence.

Every XFER reply contains `prefix`, `protocol`, `command: "XFER"`, `session`,
`seq`, `ok`, `rx_hex`, and `error`. Invalid session/sequence fields are `null`;
valid fields are preserved even if the packet is refused. `error` is `null`
on transport completion, otherwise a fixed error string. `rx_hex` contains
the captured bytes in lowercase, including any echo, noise, or invalid frame;
the host's servo parser must validate reply ID, length, checksum, and device
faults. The bridge does not interpret raw RX as a servo acknowledgement.

The request sequence is consumed **before** any UART transmission attempt.
Within a session, every new accepted sequence must be strictly greater than
the previous sequence; duplicates and backwards values are refused without
resending. All 64 session records remain until ESP restart: there is no
eviction. Once full, a new session returns `session_capacity_exhausted`, while
existing sessions can continue with increasing sequences. A restart clears
the replay cache. No command remotely restarts the ESP. Use one process-wide
session and sequence counter and do not retry a write with a new sequence to
work around an uncertain result.

After a bounded 2 ms stale-RX drain that requires 300 us of quiet, an accepted
packet is written to UART1 once. TX completion has a 10 ms timeout. RX is
captured for at most 40 ms after TX completion, or until 2 ms of quiet after
the first received byte; at most 96 bytes are retained. No RX bytes are
cleared after TX, and no frame is retransmitted. No response is a valid
transport outcome: `ok: true, rx_hex: ""` means the frame was sent, **not** that
a write took effect or a servo was present. Read back the relevant register
to verify a write when its result can be read safely.

Additional fields describe failures without implying a safe retry:

- `sequence_consumed`: this request passed validation and reserved its sequence;
  it stays consumed even if the bus was busy or TX failed.
- `tx_sent`: a UART write was attempted; a short write may have sent only part
  of the packet. `tx_complete` separately confirms driver TX completion.
- `uncertain`: an attempted operation had a TX/RX hardware error, overflow,
  incomplete deadline capture, or a repeated sequence refers to a prior
  outcome. Do not automatically retransmit it.
- `tx_error_code`, `elapsed_us`, `stale_bytes`, `session_capacity`, and
  cumulative `uart_errors_total` provide diagnostic context.

Hardware failure errors include `tx_short_write`, `tx_timeout`,
`tx_driver_error`, `uart_hardware_error`, `rx_overflow`, and
`rx_deadline_with_pending_data`. `bus_busy_before_tx` and
`uart_error_before_tx` mean no UART write was attempted. Invalid input,
allowlist refusal, `replay_or_out_of_order`, and session-capacity refusal
also never transmit the current command. An invalid or oversized XFER line
returns an XFER error when its command prefix is recognizable.

## Native protocol validation

`python3 tools/esp32_linkcheck/firmware/tests/test_protocol.py` runs from the
repository root. It uses an existing native C++ compiler, C++11 and address /
undefined-behavior sanitizers. It tests the fixed read-only encoder allowlist,
rejected broadcast IDs, checksum/ID/length validation, request echoes,
fragmented replies, device faults, and noise recovery. It additionally checks
the XFER allowlist across all read addresses/counts, all one-byte write
addresses/values, opcode/ID bounds, goal bounds and required zero bytes,
strict ASCII token parsing, checksum/length rejection, sequence monotonicity,
session switching, and full-capacity behavior. It never imports a
serial library or accesses real hardware. It is not an ESP32 firmware build
or an electrical link test.

## Source references

- Local verified HD1910 readback map: `../../servo_workbench/protocol.py`
  (ID 5, torque 40, position 56).
- [Espressif UART API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- [ESP-IDF 4.4 UART driver](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32s3/api-reference/peripherals/uart.html)
  — uses bounded `uart_wait_tx_done`, without clearing an early RX reply.
- [Espressif Wire API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html)
- [ST LSM6DSR datasheet](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf),
  sections 5.1.1 and 9.14: I2C addresses and read-only WHO_AM_I.
