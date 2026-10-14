# ARM register-40 readback investigation

The initial diagnostic-v2 candidate is described below. The later, separately
reviewed ARM receive-budget correction is described at the end. These preparation
and verification tools themselves never flash, restart services, or open UART.

The installed application `201f4e7a10d10c3cdbaa71ffd75a1aa70dd61dbf45e91230b382966427b56b55`
reports a stage-9 ARM failure at ID10/register40, before any TARGET. Its
22-byte failure ACK cannot distinguish missing bytes, rejected bytes, or bytes
queued when the reader's deadline expires. Later STOP/background reads overwrite
the one raw-read trace. Increasing the host ACK wait does not fix this MCU-side
read failure.

`prepare.py` hash-checks the reviewed installed firmware source and production
transport, then emits a separate candidate. It does not overwrite active sources.
This is deliberately based on the installed official-runtime firmware rather
than the older generic `tools/esp32_motion/firmware/main.cpp`.

The candidate retains the first failed ARM torque read before any subsequent
STOP/read can overwrite it. That failure alone receives a version-2, 134-byte
ACK; legacy failures and successful ACKs keep their existing layouts. A paired
host decoder shows the raw trace in the existing error text, and checks the
ARM request sequence and cookie. Host and firmware must be installed together.

No read retry, extra servo transaction, new grace period, gain/range change,
watchdog change, optimistic torque confirmation, or automatic re-ARM is added.
Recording timestamps and copying evidence have execution overhead; offline bus
parity is not proof of unchanged physical scheduling.

## Evidence layout

Bytes 0–21 retain the existing ARM failure fields, with version byte 2.

| Offset | Size | Meaning |
|---|---:|---|
| 22 | 1 | Flags: rejected traffic, drain truncated, RX truncated, request written, RX observed |
| 23 | 1 | Requested register length |
| 24/26/28 | 2 each | Drained, consumed RX and pending RX counts |
| 30/34/38/42 | 4 each | Start, write-return, first/last consumed RX timestamps |
| 46/50/54 | 4 each | Elapsed microseconds and UART error counters before/after |
| 58 | 4 | Torque-enable broadcast TX completion timestamp |
| 62/66 | 4 each | ARM request sequence and cookie |
| 70 | 16 | First drained bytes |
| 86 | 48 | First consumed RX bytes |

All timestamps are ESP32 uint32 microseconds, with wrap-aware differences.
Consumption timestamps are not electrical byte-arrival timestamps. Pending RX
at exit does not prove those bytes arrived before the deadline. No bytes at exit
does not prove a power fault or distinguish a later reply from permanent silence.

`verify.py` compares baseline/candidate packets and decisions for 11 exact-handler
mock scenarios, uses ASan/UBSan, tests the actual host decoder/transport, runs the
complete inherited firmware suite, and builds the ESP32 image. It never flashes.

The simulation cases cover silence, echo only, bad checksum, foreign ID, UART
error, complete queued reply after a 2 ms CPU pause, partial reply, reply later
than the deadline, bounded truncation, delayed success and uint32 clock wrap.
The 2 ms CPU pause reproduces a software timeout with a complete buffered reply;
it is a hypothesis discriminator, not proof that the physical ID10 failed that way.

Before an attended installation, recheck the deployed baseline and absence of
UART ownership. Support the body and disconnect the servo battery before ESP32
reset/flash. After installation, an explicitly attended single enable/readback
check is needed to capture the physical failure. Do not repeatedly start walking
to generate evidence. Confirm STOP by actual advancing all-OFF feedback.

## ARM receive-budget correction, 2026-10-12

Upstream `sketch/duck_config.h` sets `SERVO_REPLY_US = 6000`.
`sketch.ino` starts that receive window after `Serial1.flush()` for a synchronous
group read. Our custom 1500-us individual-read cutoff was not a measured servo
capability. Hardware captured stage-9 ID10 torque-read failures with zero bytes
or only `ff` at that cutoff, with no UART error or pending byte. A matched basic
STATE stream also failed, so extended feedback is not required for this failure.
These observations alone do not prove when a complete reply eventually arrives.

`prepare_timeout.py` checks the exact diagnostic-v2 firmware hashes and applies
`arm-timeout-reference.patch` into a new directory. It corrects explicit ARM
register-40 verification to receive for up to 6000 us after confirmed request TX
completion. ARM, torque enable, and reads are never retried. Complete ID/checksum/
register validation, feedback freshness, STOP, watchdog, ranges and gains remain.
Normal-loop individual polls still retain their separate 1500-us budget; this
patch does **not** claim to reproduce upstream synchronous polling or repair
normal control-cycle timing.

The patch also keeps up to 32 existing stage-9 read traces in RAM. Cached command
28 takes a one-byte record index and returns a version-2, 148-byte reply (0x9c).
It emits no servo traffic. The original failure-ACK format is unchanged. Trace
bytes 0–133 use the original layout; bytes 134–137 are history version, count,
index and armed flag; uint32 at 138 is last disable TX completion; uint32 at 142
is request TX completion (zero for non-ARM-torque reads); 146–147 are reserved.
Copying diagnostic records still has overhead.

`test_timeout.cpp` exercises the real handlers under sanitizers, including full
and partial replies after the old cutoff, 5700-us acceptance, 6100-us refusal,
silence, corruption, UART errors, wrap, cached read isolation and no repeated
enable. Run the complete inherited firmware suite as well. Preparation does not
install anything; installation requires the current board/calibration/ownership
checks and supported hardware state.

The reviewed correction application is
`815d98b0199bf90d5bbebe7e3c61a5043a085841583928043363d84d90b900cd`.
It was installed and read back exactly; calibration revision27 was preserved.
The first post-update check sent no ARM because the head/neck was outside the
saved ranges. After the user restored the head/neck, one current-position ARM
still failed at stage9, ID10/register40: no RX byte in 6006 us after TX completion
(6154 us whole transaction); later cached reads contained no delayed ID10 reply.
STOP and all15 OFF were confirmed. The longer receive budget did not repair this
failure; walking remains unverified.
Evidence: `output/duck_sim/arm-readback-20261012/install-timeout6-v2/`.


Further supported diagnostics selected ID11 as the first post-enable torque read;
it also timed out with zero RX. An isolated position-first variant passed all30
readbacks, but a subsequent four-attempt old/new/new/old comparison all passed.
This does not prove a causal repair or rule out intermittent timing. Current
v6 is a diagnostic snapshot, with default order unchanged, not an upstream
communication restoration. Full evidence and active source-backed repair
direction: `output/duck_sim/arm-readback-20261012/install-current/HANDOFF.md`.
