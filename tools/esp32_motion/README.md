# X5 / ESP32-S3 motion bridge

This is a separate motion firmware, leaving `tools/esp32_linkcheck`,
`output/esp32_linkcheck`, and `output/esp32_workbench` intact. It has not been
flashed or physically tested by its build/test workflow. The assembled robot
must be supported before installing it: **startup and STOP disable torque**.
Torque release does not hold a standing robot upright.

The firmware starts disarmed, holds calibration in RAM, and never automatically
arms after reset or after a fault. In normal motion mode it writes only goal position/duration/speed
(register 42, six bytes), torque enable (register 40), and the official
temporary RAM gains (KP register 50 and KD register 51) during explicit ARM.
Normal motion commands never write servo IDs, EPROM, zero offsets, permanent gains, or unlock registers. The explicit, mutually exclusive maintenance mode described below can change one isolated servo's ID. A CONFIG command
does not change the physical servo's zero; it supplies host-calibrated limits
used to reject unsafe goals.

## Wiring and timing

| Interface | Pins / configuration |
|---|---|
| Host UART0 COM adapter | RX44, TX43, 921600 baud, 8N1 |
| Servo UART1 to URT-2 | RX18, TX17, 1 Mbps, 8N1 |
| LSM6DSR I²C | SDA8, SCL9, 400 kHz, 2 ms I²C driver timeout |
| Servo identities / frame order | `10,11,12,13,14,20,21,22,23,24,30,31,32,33,34` |

Position/speed readback targets one completed cycle every 10 ms. Each individual
servo READ is bounded to approximately 1.5 ms, including a bounded RX drain.
The actual sample rate depends on UART turnaround, sensor/bus errors, and host
traffic; **100 Hz is a scheduling target, not a measured hardware result**.
The firmware checks the watchdog during every bus transaction, and services
host commands between position reads. It publishes a new sample timestamp
only after completing a full servo cycle; arrays are not partially published.

`online_mask` requires both a valid present-state read in the last completed
cycle and a known torque read no older than 500 ms. A current failed position
read clears the corresponding online bit immediately. `fault_mask` reports
device status error bytes from position and torque reads. A missed packet
reports offline/unknown, without inventing a device error byte.

Live supply voltage uses the same UART owner: one READ of register 62 (one
byte, 0.1 V/count) per normal loop, rotating over all 15 IDs. It is disabled
in maintenance and never refreshes position/IMU timestamps or the watchdog.
`TELEMETRY_CAPS` (20, response 0x94) returns `XGODUCK_TELEMETRY_1`;
`VOLTAGE_STATE` (19, response 0x93) returns the unchanged 106-byte STATE prefix,
then version 1 (u8), seen mask (u16), valid mask (u16), 15 raw bytes and 15
sample ages (u32 microseconds), all little endian: 186 bytes total.
Unseen values use raw 0 and age UINT32_MAX. Missing/error/zero replies clear
current validity and retain the last dated nonzero value. Valid readings
expire after 1 second. The legacy STATE request and layout remain unchanged.
The host falls back only on the exact legacy BadPacket ACK to capability 20;
corrupt, mismatched or timed-out replies fail the connection. After that legacy
probe a STOP clears its BadPacket latch; no ARM or TARGET is sent.

The motion table updates individual cells, wraps within its container, and
labels retained readings as historical. A fresh voltage below 4.0 V or above
8.4 V triggers a visible supply warning, based on the HD1910 manufacturer
specification in `output/reference/HD1910-spec-20260907.pdf` (PDF page 3).
This is servo-terminal voltage, not battery charge or a load-capability test.
The 0.1 V register conversion still needs comparison against a real meter;
the software tests cannot establish that hardware calibration.

`torque_mask` contains observed register-40 values, refreshed at boot and
ARM/STOP. A periodic scan starts every nominal 250 ms while the bus is healthy;
it reads three motors per main-loop pass, interleaved with complete position
cycles. Each motor retains its own 500 ms validity limit. A single 15-motor
torque scan followed by a full position cycle could exceed the unchanged
50 ms position-feedback limit even with valid replies near the 1.5 ms READ
deadline. Splitting the background scan avoids that scheduling gap.
It is **readback**, not an optimistic copy of a torque command. A missing
torque reply clears validity and its torque bit; `online_mask` distinguishes
this unknown state from a confirmed torque-off state. The protocol has no
per-servo torque timestamp; observed changes can therefore lag up to the
refresh interval. Even a successful STOP ACK proves the command was handled,
not that an unreachable servo physically released torque. Check the online,
fault and torque masks together.

ARM requires valid calibration, 15 online servos, no faults, confirmed
torque-off, and servo/IMU samples no older than 50 ms. While torque is off it
applies and reads back the official runtime's volatile RAM KP=6 and KD=20 on
all 15 servos, except mouth ID 34 uses KP=10. Every mapped ID must return the
expected two RAM bytes; a missing reply, fault or mismatch refuses ARM before
enabling torque. Configuration follows `setRunKpKd()` in the checked upstream
[`scs_bus.h`](../../output/xgoduck_runtime_upstream/sketch/scs_bus.h) and
[`duck_config.h`](../../output/xgoduck_runtime_upstream/sketch/duck_config.h).
No EEPROM register 21 or unlock register is written. HELLO, CONFIG, STATE, FAULT_DETAILS,
STOP and startup never apply gains.

ARM gain writes use the same bounded echo/status collection as maintenance
writes. Each WRITE owns its optional echo and status response for 1.5 ms before
the first gain READ; this includes the mouth's unicast KP write. A late WRITE
reply must not overlap that READ on the half-duplex bus. Omitted status ACKs
still require exact per-ID KP/KD readback. Corrupt, foreign, duplicate, partial,
device-error or UART-error replies reject ARM before goals/torque; writes and
ARM are never automatically retried. Freshness/watchdog limits are unchanged.

Torque-enable broadcasts use this same 1.5 ms bounded write-reply window before
torque readback. UART TX completion alone does not prove a
delayed echo has left the half-duplex bus. A corrupt/foreign/partial echo or
UART error fails the write; an absent echo is allowed, and all 15 observed
torque registers remain mandatory. This does not retry enable or mask a
missing torque readback.
STOP/boot retain the original TX-only disable: receive noise from an unpowered
bus must not block that broadcast or latch a new startup error. TX success
still does not confirm torque-off; actual per-servo readbacks remain required.

After gain readbacks it obtains new torque, position and IMU feedback, then
revalidates the current positions and confirmed torque-off. This avoids aging
the cached pre-ARM sample during preparation. It writes the newly read current
positions as goals, rechecks freshness again before enabling torque, then
verifies every servo's torque readback. During that full post-enable check it
refreshes all positions once between the first eight and last seven torque
reads, while the ARM handler still excludes recursive host commands. This
keeps the position sample valid through verification and the next normal
position cycle; no partial position array or incomplete ARM is acknowledged.
Legacy 34-byte TARGETs retain the port's 600 raw counts/s pose slew
(about 52.7°/s), with a maximum elapsed step of 50 ms. This value was an
unvalidated conservative addition, **not a servo capability or upstream policy
requirement**. Paired offline trials found it disrupted the official walk.

The explicit policy-position extension accepts a 35-byte TARGET: the same
`<I15H` payload followed by profile byte `1`. After unchanged cookie, sequence
and full-group range checks, the next bus tick writes those position goals
without the extra MCU slew, as upstream does. Duration/speed register fields
remain zero. The host preserves EMA and its separate stance/policy takeover.
Before any ARM, the new host must query empty `POLICY_POSITION_CAPS (24)` and
receive exactly `ACK [24,0]`; unsupported firmware fails closed with no legacy
fallback. The probe itself changes no profile, goals, torque or watchdog time.

A later legacy TARGET resumes pose slew from the last applied goals. STOP and
each new ARM clear the profile; ARM still initializes goals at actual encoder
positions before enabling torque. Unknown profile bytes, wrong lengths,
cookies, replayed sequences and out-of-range goals are rejected atomically and
do not renew the watchdog. Tilt/freshness/range protection remains required.
This fixes the execution contract, not the outstanding saved ankle-range
conflict or physical walking qualification. No automatic flash/deploy occurs.

ARM failures include an optional version-1 22-byte ACK with the preparation
stage, first failed gain/torque-read ID/register/result and servo/IMU sample ages.
For torque-read stages 4/9, bytes 10/11 contain expected/actual register-40
values; for gain stage 3 their KP/KD meaning is unchanged. The first failed
torque read is retained before STOP changes later readbacks.
Successful ARM ACKs remain two bytes. Host commands cannot be re-entered from
the position refresh inside ARM, and preparation never retries ARM automatically.
The X5 waits up to 200 ms for this preparation ACK; ordinary command waits
remain 160 ms. These are response deadlines only: the 50 ms firmware feedback
limit and 250 ms valid-TARGET watchdog are unchanged.

Missing or stale servo/IMU feedback, any servo fault, lost torque while armed,
or more than 250 ms since a valid TARGET disarms. Disarming clears the cookie.
An explicit new ARM is always required to resume. Software stop and serial
watchdogs complement a physical power cutoff; they cannot turn off a servo
through a disconnected or damaged bus.

## IMU settings and units

The [ST LSM6DSR datasheet, DS11976 Rev 2](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf)
defines WHO_AM_I `0x6b`, control registers and sensitivity. Firmware probes
addresses `0x6a` and `0x6b`, resets the device, verifies configuration readback,
and waits for both accelerometer and gyroscope data-ready bits.

| Register | Value | Configuration |
|---|---|---|
| CTRL1_XL `0x10` | `0x48` | 104 Hz, ±4 g |
| CTRL2_G `0x11` | `0x44` | 104 Hz, ±500°/s |
| CTRL3_C `0x12` | `0x44` | BDU and address auto-increment |
| CTRL4_C `0x13` | `0x08` | Mask data-ready during filter settling |

Acceleration converts with `0.122 mg/LSB × 9.80665 / 1000`; gyro converts with
`0.0175 °/s/LSB`. Values retain the sensor's X/Y/Z axes. The X5 must apply
the actual mounting transform, measured gyro bias, and its attitude estimator;
the firmware does not assume a body-axis mapping.

After at least one second without a complete IMU sample, the main loop can
rebuild the I²C master and reset/reconfigure the sensor. Recovery is allowed
only while disarmed, outside maintenance/ARM/torque verification, after fresh
position feedback and all 15 torque readbacks confirm torque is off. Attempts
are separated by at least one second, followed by the normal 100 ms sensor
settling period. Automatic recovery bounds reset-bit polling to 20 ms so a
responsive but malfunctioning sensor cannot stall host handling for the boot
initialization's longer reset budget. Failed bus teardown, bus setup or sensor configuration leaves
IMU feedback invalid. Recovery preserves calibration and the existing fault
latch; it never writes motor goals, enables torque or resumes an old action.

This is recovery from communication or sensor power loss, not a guarantee of
electrical hot-swap support. Switch off ESP32/IMU power before moving DuPont
wires, especially ground or supply. Persistent wiring faults or a slave holding
the bus low can still require correcting the wiring and a power cycle.

## Binary protocol, version 1

All integer fields and float32 values are little endian. Frame:

```
"XD" | version:u8=1 | type:u8 | payload_length:u16 | sequence:u32 |
payload[0..256] | CRC16:u16
```

CRC is CCITT, initial `0xffff`, polynomial `0x1021`, calculated over all ten
header bytes (including `XD`) plus payload; the CRC itself is excluded.
Corrupt CRC/version/oversize frames are discarded and the parser resynchronizes.
STATE/HELLO/FAULT_DETAILS requests must have empty payloads. CONFIG, ARM and TARGET require
strictly increasing uint32 sequences; duplicates and backwards values fail.
HELLO resets this write-sequence guard only when disarmed. Sequence wrap
requires a disarmed HELLO. STOP is accepted regardless of sequence and always
disarms, even when its payload is malformed.

| Request | Payload | Reply |
|---|---|---|
| HELLO `1` | Empty | `0x81`, ASCII `XGODUCK_MOTION_1` |
| STATE `2` | Empty | `0x82`, 106-byte state |
| CONFIG `3` | `<15H15b15H15H`, 105 bytes | ACK; RAM only, disarmed only |
| ARM `4` | `<I`, nonzero random arm cookie | ACK after observed torque verification |
| TARGET `5` | `<I15H`, matching arm cookie and 15 raw goals | ACK |
| TARGET `5`, policy position | `<I15H` followed by byte `1`; capability check required | ACK |
| POLICY_POSITION_CAPS `24` | empty; read-only support check | exact ACK `[24,0]` |
| STOP `6` | Empty | ACK after disable command/readback attempt |
| FAULT_DETAILS `7` | Empty | `0x87`, 247-byte read-only diagnostic snapshot |

### OFF-only goal read diagnostic

`GoalReadDetails` (`22`) accepts exactly one mapped motor ID and returns type
`0x96`, version 1, exactly 78 bytes. It performs one READ of goal register 42
(2 bytes), then one READ of present register 56 (6 bytes), without retry or any
servo WRITE. It requires disarmed/cookie-zero motion, no maintenance, ARM
preparation, torque verification or pending disable/verification, a completed
position cycle no older than 50 ms, and all 15 known torque-off readbacks no
older than 500 ms. IMU health is not required. A global 500 ms interval applies
between admitted attempts, including failed reads and across STOP/HELLO; the
first attempt is allowed. Invalid shape/ID, busy, stale or rate-limited requests
issue no servo transaction and do not consume the write sequence or overwrite
the existing safety error. Refused requests do not advance the rate timer.

All multi-byte values below are little endian. Header byte 3 is reserved zero.

| Byte offset | Value |
|---|---|
| 0, 1, 2 | Version, motor ID, mapped index (ID 11 has index 1) |
| 4, 6 | Retained MCU desired and slew goal, u16 each |
| 8, 9 | Armed before/after, u8 each |
| 10, 14 | Cookie before/after, u32 each |
| 18, 22 | Feedback sampleUs and imuUs before, u32 each |
| 26, 30 | Feedback sampleUs and imuUs after, u32 each |
| 34, 36 | Raw torqueKnown and torqueMask before, u16 each |
| 38, 40 | Raw torqueKnown and torqueMask after, u16 each |
| 42–59 | Register-42 READ record |
| 60–77 | Register-56 READ record |

Each 18-byte READ record contains start/end micros timestamps (u32 at +0/+4),
the unchanged `readServo` outcome/error/size (u8 at +8/+9/+10), flags at +11
(`found=1`, `invalidReplySeen=2`, `registerValid=4`), and its six raw reply-data
bytes at +12. Bytes outside reply size are zero. Register-valid requires a
found, error-free reply of the requested size and a first u16 no greater than
4095. An out-of-range raw word remains visible with its original normal-reply
outcome and validity false. The outcome codes are the existing normal reply
1, device-error reply 2, bus-not-quiet 3, short UART write 4, UART error 5 and
no valid reply by deadline 6; corrupt traffic is also marked by the flag.

The diagnostic does not publish these values as STATE, refresh sample/IMU
timestamps, renew the target watchdog, repair masks or authorize ARM. Existing
`readServo` safety checks still run and may expire genuinely old mask bits.
Its 1.5 ms polling deadline is nominal, not a hard bound on underlying UART
calls or a promised 3 ms total. MCU desired/goals are retained values after
STOP, not a current active target. TARGET ACK remains MCU acceptance rather
than servo goal readback. A goal/present difference after torque release does
not establish tracking under load, output torque or independent standing.

### Explicit maintenance extension

This extension must be detected with `MAINT_CAPS`; the unchanged HELLO string
alone does not imply maintenance support. Old firmware rejects request 8 with
`BadPacket`, and clients must then keep maintenance actions disabled. The
original command numbers, HELLO greeting, STATE size and FaultDetails payload
remain unchanged.

All multi-byte fields below are little endian. Requests use the same XD/CRC
framing and ACK type `0x80` as normal motion.

| Request | Payload | Reply |
|---|---|---|
| MAINT_CAPS `8` | Empty | `0x88`, ASCII `XGODUCK_MAINT_1` |
| MAINT_ENTER `9` | `<I`, fresh nonzero random cookie | ACK |
| MAINT_READ `10` | `<IB`, cookie, ID | `0x8a`, `<IBBBHB`: cookie, ID, outcome, servo error, position, torque |
| MAINT_RENAME `11` | `<IBB`, cookie, old ID, new ID | ACK after isolated-servo scan and verified write/relock |
| MAINT_CENTER `12` | `<IBHHH`, cookie, ID, target, minimum, maximum | ACK starts a nonblocking move of this ID only |
| MAINT_POLL `13` | `<I`, cookie | `0x8d`, `<IBBBHB`: cookie, active ID, state, servo error, position, torque |
| MAINT_RELEASE `14` | `<IB`, cookie, ID | ACK after torque-off write/readback attempt |
| MAINT_EXIT `15` | `<I`, cookie | ACK after release attempt and exit |
| MAINT_CENTER_CHECK `16` | `<IB`, cookie, ID | Checks the current-position holding goal and valid torque mode, then releases and verifies torque off; the goal write may activate holding |

READ outcome is `0` normal, `1` no valid reply with a silent bus, `2` a
checksum-valid device error, or `3` malformed/register-invalid/UART-error
feedback. Nonzero outcomes do not certify position or torque. Unknown values
use position `0xffff` and torque `0xff`; a partially completed READ can contain
a position plus unknown torque, which clients must not treat as a healthy
sample. POLL state is `0` idle, `1` moving, `2` done, `3` fault. With no selected
servo, active ID is `255` and position/torque are unknown. A successful READ
selects that ID for subsequent POLLs. An absent ID while scanning clears the
selection without latching a fault. A malformed/device-error response is a
fault, not an unused address. Every POLL for a selected ID performs a new
identity/position/torque read; it never refreshes the age of cached feedback.

Only a disarmed bridge can enter maintenance. Enter, exit and movement faults
clear motion configuration and invalidate all fixed-map servo feedback. No
normal CONFIG, ARM or TARGET is accepted while maintenance owns the bus.
Background fixed-ID position and torque scans pause, including when Enter
arrives between reads of a cycle; IMU polling continues. Serial ownership must
also be exclusive on the X5: close and verify the calibration reader before
opening the maintenance connection. Returning to calibration requires a new
explicit connection and fresh physical feedback.

Maintenance uses its own sequence guard; HELLO cannot reset it. Every accepted
maintenance session cookie is retained until reboot (capacity 64); reused
cookies and capacity exhaustion are refused, not evicted. Each fresh cookie
may begin with a new sequence. Subsequent requests require strictly increasing
sequence numbers, consumed before UART side effects. Invalid cookie/replay/
length/ID requests cannot extend the maintenance lease. There is no raw XFER,
arbitrary register write, broadcast ID write, or bare torque-enable API.

The bridge releases and exits after one second without a valid maintenance
request or after ten seconds of movement, even if POLL requests continue. STOP
works regardless of cookie/sequence/payload, releases torque and exits.
Transactions have bounded reads (~1.5 ms) and TX waits (5 ms); UART code does
not recursively service another host request. MAINT_RENAME's single 0–253 ID
scan is approximately 381 ms worst-case at the read deadline, plus bounded
write/readback operations and 50 ms EEPROM settling, normally under 600 ms.
Its watchdog remains active at transaction boundaries; there is no unbounded
lease exemption for programming.

RENAME requires exactly one responding ID across the scan, matching old ID,
with no response at new ID and no corrupt/UART/device-error feedback. The
operator must also physically isolate the intended servo: a bus scan cannot
distinguish multiple physical servos sharing an ID. It releases and verifies
torque, unlocks register 55, writes register 5 once, reads the new ID, then
relocks and verifies register 55 plus torque. An uncertain write is never
retried or rolled back. Failure attempts to relock each plausible old/new ID;
success certifies current readback, not persistence after power cycling.

CENTER requires current and target positions inside host-supplied limits. It
reads healthy torque-off feedback, writes and verifies the current position
as the first goal. A goal write may itself activate holding: torque modes 0
and 1 are accepted only with complete healthy readback of that holding goal.
If still off, it explicitly enables only the selected ID; either path then
verifies torque on before starting the ramp. Goals advance by at most
8 counts per 35 ms. Lost/invalid feedback, device error, unexpected torque,
out-of-range position, tracking error over 64 counts, or a position jump over
96 counts aborts and releases. Three successive final samples within 12
counts complete the move and release, rather than leaving it holding torque.
Unicast writes now collect their optional exact request echo and status ACK
within a bounded 1.5 ms window before sending a register read. Corrupt,
foreign, duplicate, device-error or incomplete write responses reject the
operation; an omitted ACK still requires the usual register readback. Writes
are never retried. CENTER rejection codes identify the failed start step:
`16` snapshot, `17` holding-goal write/ACK, `18` holding-goal readback,
`19` holding torque-mode readback, `20` explicit enable write/ACK, `21` enable readback.
For wire compatibility, stage 19 retains expected-value field 0 in failure
evidence; it no longer rejects a valid mode 1. Historical traces can still
show the old firmware rejecting 1. Invalid modes, malformed replies and
device errors remain failures.
Read failures `18`, `19` and `21` attach bounded evidence to their failure ACK.
The failing read is copied before release, so a successful cleanup read cannot
replace it. The 101-byte ACK contains the usual request/error bytes, then
version/ID/register/size, expected value, found/invalid flags, read outcome,
device error, actual size/data, elapsed microseconds, UART error counters,
drained/received byte counts and the first 16 drained/48 received bytes.
Unused capture bytes are zero and capture truncation is explicit. The host
validates the same wire layout, saves the raw ACK and decoded evidence, then
closes the existing owner without retrying. Old two-byte rejection ACKs remain
supported. Evidence never clears a fault hold or certifies the bus as healthy.
`MAINT_CENTER_CHECK` writes only the freshly measured current position and
requires a fresh torque-off snapshot first. This write may briefly activate
holding even though the check sends no explicit enable. It releases the
selected ID once and requires torque-off readback before success; release
failure returns an error without claiming torque off or starting a ramp.
It remains protected by the maintenance nonce/sequence guard
and watchdog; successful checks do not clear host holds or authorize movement.
The controller must derive assembled-joint limits and the actual mechanical
zero from saved, confirmed calibration; raw encoder midpoint 2048 is not a
substitute. The robot still needs physical support with its feet unloaded.

The optional voltage extension leaves `XGODUCK_MAINT_1`, the 106-byte STATE,
and all existing response layouts unchanged. `MAINT_VOLTAGE_CAPS` (17, empty
payload) returns `XGODUCK_VOLTAGE_1` (reply 0x91). The host negotiates this only
for an explicit voltage collection. A legacy firmware's exact BadPacket ACK
`17,1` means this optional feature is unavailable, without issuing a register
read or disabling legacy maintenance; other malformed/error responses fail.

`MAINT_VOLTAGE_READ` (18, `<IB`: maintenance cookie and ID) reads only register
62, length 1. It requires the exclusive disarmed maintenance session, uses the
existing nonce/sequence checks, and refuses during a center movement. Reply
0x92 is `<IBBBB`: cookie, ID, outcome, servo error, raw voltage byte. Outcomes
0/1/2/3 mean valid/unresponsive/device-error/invalid-reply. Only outcome 0 has
a usable byte; all others encode zero as an unavailable placeholder, which
the host converts to `None`, never a measured 0 V. No torque/goal/ID/EEPROM
write is made by the voltage command, and this reading cannot certify torque
off, clear a fault hold, or alter saved calibration.

The workbench's explicit **采集舵机电压** action reads the 15 configured IDs
once, using the existing owner and heartbeat. Its table retains per-servo
sample ages and errors, and is labelled as sampling records rather than live
battery telemetry. It is blocked while motion or calibration owns the UART.
Opening or refreshing the page never collects voltage automatically.

Register 62 is defined in the [manufacturer's HLSCL SDK](https://github.com/ftservo/FTServo_Arduino/blob/main/src/HLSCL.h).
The raw byte is converted with the manufacturer's common 0.1 V/count convention
([published STS example](https://www.feetechrc.com/20210430-56680.html)).
HD-1910-C001's local manufacturer specification confirms input-voltage feedback,
but does not document this scale. The HD1910 scale therefore still requires an
actual battery-meter comparison on first installation; the UI and firmware
manifest state this explicitly. This is servo-terminal voltage including cable
drop, not a direct URT2 input measurement or battery state-of-charge estimate.

Any device error observed during maintenance latches write/motion rejection
for that session. Release and best-effort parameter relocking remain
available. The maintenance UI cannot clear a pre-existing hardware fault
hold by merely changing tabs. Reassigning an ID also does not migrate the
fixed `10–14,20–24,30–34` model/calibration mapping; preserve records but
require physical mapping verification before normal motion resumes.
The unified workbench offers an explicit fault-feedback review after the
previous owner has closed. It reads all 15 mapped IDs, requires healthy
torque-off responses, exits and verifies UART/process closure, then archives
the original fault hold with a separate resolution audit. It never enables
torque, changes IDs/calibration or automatically retries the failed move.

CONFIG arrays are zero positions, signs (`+1/-1`), raw minima and raw maxima.
Array index and every mask bit follow the physical-ID order in the table;
bit zero is ID 10, bit five is ID 20, and bit fourteen is mouth ID 34.
All values must fit `0..4095`, minima must not exceed maxima, and each zero
must lie in its range. Equal min/max is permitted to keep one joint fixed.

ACK type `0x80`: two bytes, request type and error, except the bounded ARM,
TARGET and CENTER failure evidence. Error codes:
`0` success, `1` invalid packet/replay, `2` invalid state or hardware fault,
`3` invalid calibration, `4` missing/stale feedback or watchdog timeout,
`5` requested/current positions outside calibration bounds.
Reply header sequence echoes the request sequence.

An autonomous stop during an armed motion session now retains its first cause.
A subsequent TARGET `BadState` for that same cookie carries exactly 40 bytes,
format `<6B3H5I4BI`: request, rejection error, detail version `1`, cause, first
error, IMU-valid flag, online/fault/torque masks, stop uptime, position/IMU/target
ages in microseconds, UART-error counter, read ID/address/outcome/device error,
and read elapsed microseconds. Read ID `255` means no traced read; outcome `0`
can mean a read was still in progress when the protection fired. These fields
describe the stop event rather than the later torque-off readback.

Causes `1..9` distinguish device fault, offline feedback, invalid IMU, position
age over 50 ms, IMU age over 50 ms, TARGET gap over 250 ms, lost torque, goal TX
failure and other explicit protection stops. The record survives subsequent
TARGET refusals and STOP, and clears at the next explicit valid ARM's enable
verification. Other cookies and unrelated refusals keep legacy two-byte ACKs.
No new bus reads, retries or extended safety deadlines are introduced.
The X5 decodes this record and retains the last 64 request timings with the
first host stop evidence, before issuing STOP. Historical readings alone do
not identify battery droop or prove a physical stance completed.

STATE payload format is `<IIHHHBB15H15h6fIBB`:

```
sample_us, imu_us, online_mask, fault_mask, torque_mask, armed, imu_ok,
positions[15], velocities[15], accel_ms2[3], gyro_dps[3],
arm_cookie, last_error, configured
```

Firmware timestamps use modulo-32-bit `micros()`. Velocity fields are signed
raw servo speed units, decoded from the speed register's bit-15 sign-magnitude
encoding. The X5 must apply the servo's speed scale and calibration sign.
`imu_ok` additionally checks that the IMU sample is at most 50 ms old.

### Read-only fault details, payload version 1

FAULT_DETAILS does not read additional servo registers, issue new bus
transactions, clear faults, arm, configure, or consume the write-sequence
guard. It exports history already captured by the existing background
register-56 position and register-40 torque reads. A nonempty payload receives
ACK `BadPacket`; the normal watchdog and torque-disable protections still run
for every host command. STATE remains exactly 106 bytes and HELLO is unchanged.

The 247-byte payload has a 25-byte header (`<BBIIHHHHHBI`) and two 111-byte
banks (`<HHH15B15B15B15I>`), in the fixed motor-ID order above:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Diagnostic payload version, `1` |
| 1 | u8 | Motor count, `15` |
| 2 | u32 | Snapshot `micros()` |
| 6 | u32 | UART hardware-error callback counter |
| 10, 12, 14 | u16 each | Current online, fault, observed torque masks |
| 16, 18 | u16 each | Current position-source faults, torque-source faults |
| 20 | u8 | Current armed boolean |
| 21 | u32 | Current arm cookie |
| 25 | 111 bytes | Register-56 position-read diagnostic bank |
| 136 | 111 bytes | Register-40 torque-read diagnostic bank |

Each diagnostic bank uses these offsets relative to its start:

| Relative offset | Type | Field |
|---|---|---|
| 0, 2, 4 | u16 each | Ever attempted, latest attempt found valid reply, ever seen valid reply masks |
| 6 | 15 × u8 | Latest attempt's device-error byte (zero if no valid reply) |
| 21 | 15 × u8 | Last checksum/ID-valid reply's device-error byte |
| 36 | 15 × u8 | Latest attempt outcome code |
| 51 | 15 × u32 | Last checksum/ID-valid reply's `micros()` timestamp |

Outcome codes are `0` never attempted, `1` valid reply with zero device-error
byte, `2` valid reply with a nonzero device-error byte, `3` the pre-request bus
quiet check failed, `4` UART request write was short, `5` a hardware UART-error
callback occurred during reception, and `6` the deadline elapsed without a
valid reply, and `7` a checksum/ID-valid frame with zero device-error byte whose
register data fails the original size/value checks (register-40 torque must
be one byte in `0..1`; register-56 position data must be six bytes with position
in `0..4095`). Code `6` does not imply no bytes arrived: echoes, bad checksums,
wrong-ID replies, or incomplete packets can also produce it. The history maps
an otherwise normal frame to code `7` without changing the original online or
fault-mask gates. Existing STATE validation still checks freshness separately.

A missing reply sets latest-found false and latest-error zero while preserving
last-valid error and timestamp. Always interpret error bytes using their
associated found/seen mask. Timestamp zero can be valid at `micros()` rollover.
An old error plus a currently missing reply is different from a fresh device
error. This diagnostic only exposes that distinction; it does not change the
original fault-mask update rules. Position faults are replaced at the end of
a full position-read cycle; a torque fault can remain across missing torque
replies until a valid healthy torque reply clears it. Snapshots can occur
between position reads, so latest history and the last completed-cycle source
faults need not agree. Explicit ARM gain readback can also set a position-source
fault; gain reads are not included in these two diagnostic banks.

### Cached raw position-read details

`POSITION_READ_DETAILS` (request 21, response `0x95`) provides the latest
normal position-read attempt for one ID in the fixed 15-motor map. The request
payload is exactly one ID byte. An unmapped ID or another payload length gets
`BadPacket`; an armed session, nonzero ARM cookie, ARM preparation, torque
verification or maintenance mode gets `BadState`. These requests, including
rejected requests, do not consume the motion write sequence or overwrite the
last motion error. As with every host command, the existing safety checks run
first and can still release torque for an independent fault.

The response is version 1, exactly 22 bytes (`<8BII6B`, little endian):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Version, `1` |
| 1 | u8 | Requested mapped motor ID |
| 2, 3 | u8 each | Fixed register address `56`, requested length `6` |
| 4 | u8 | Flags: bit 0 attempted, bit 1 checksum/ID-valid reply found, bit 2 invalid traffic seen, bit 3 register data rejected; other bits zero |
| 5 | u8 | Returned register-data length, `0..6` |
| 6 | u8 | Reply's device-error byte; interpret only when found |
| 7 | u8 | Latest outcome code, same meanings as FAULT_DETAILS above |
| 8 | u32 | Beginning of this position-read attempt, `micros()` |
| 12 | u32 | Diagnostic response snapshot, `micros()` |
| 16 | 6 bytes | Exact returned register data, with zero padding beyond the returned length |

Before the first attempt, flags, returned length, error, outcome, attempt time
and register bytes are zero. Timestamp zero also occurs at timer wrap, so the
attempted flag determines whether there is a sample. Age uses unsigned 32-bit
subtraction from snapshot time. A miss replaces the latest cache with an empty
reply; it does not present an older successful reply as current. A valid frame
with position above 4095 retains its original bytes and outcome 7 while its
motion online bit remains clear. For example, bytes `00 10` mean raw position
4096 in this diagnostic and are still rejected for motion.

This command only copies the existing `sampleServos()` register-56 read into
a small cache and exports it. It performs no additional servo UART reads or
writes, accepts no arbitrary address/length, and changes no calibration,
register validation, range, ARM, watchdog or fault condition. STATE,
FAULT_DETAILS and capability payloads keep their existing layouts. Older
firmware rejects request 21 with `BadPacket`; clients must not substitute a
raw UART command or interpret that rejection as an empty successful sample.

## Build and verification

Run from the repository root:

```sh
sh tools/esp32_motion/tests/run.sh
PLATFORMIO_CORE_DIR=/tmp/xgoduck-pio-core /tmp/xgoduck-pio-venv/bin/pio run -d tools/esp32_motion
python3 tools/esp32_motion/package_firmware.py --output output/esp32_motion/firmware-fault-details-register-valid-20261004
```

Native tests run with AddressSanitizer and UndefinedBehaviorSanitizer. They
cover framing/CRC corruption recovery, replay, invalid configuration, arm
interlocks, atomic range rejection, slew limits, watchdog, timestamp rollover,
and sign-magnitude decoding. A second test compiles the actual firmware
against mocked UART/I²C and verifies startup disarm, sensor conversion,
transient RAM-gain packet values, all mapped gain readbacks, mouth ID 34's
exception, refusal before enable on gain mismatch/missing replies/stale state,
current-position goal writes before torque enable, observed torque failure,
unconditional STOP, missing servo, fault disarm, and the exact STATE layout.
It also tests the exact FAULT_DETAILS layout, malformed requests, no extra bus
transactions or write-sequence consumption, all eight read-outcome meanings,
checksum/wrong-ID rejection, invalid torque `255` / position `4096` rejection,
and fault → missing reply → healthy reply history. Cached raw position tests
also verify the new 22-byte response, never-attempted and timestamp-wrap
semantics, invalid ID/length and active-mode rejection, no UART traffic or
write-sequence consumption, raw 4096 retention while offline, and continued
ARM rejection before gains, goals or torque writes.
Mocks validate logic and packet contents; they do not verify physical timing,
electrical behavior, servo torque, or real IMU alignment.

Packaging writes checksummed images to `output/esp32_motion/firmware-fault-details-register-valid-20261004`, refusing
to replace different existing binaries. Prior firmware backups remain intact.
The earlier no-gain image remains in `output/esp32_motion/firmware`.
The new image uses the same factory partition layout as the diagnostic image.
Build/packaging commands do not open serial ports and do not flash the ESP32.

The checked X5 can also act as the ESP32 flashing host over SSH, with COM
left connected to X5 and the URT2 servo battery disconnected. The current
maintenance script is `tools/esp32_motion/flash_via_x5.py`; it pins the reviewed
X5 boot/card, ESP32-S3 MAC, partition table and application hashes. It uploads
pure Python esptool/pyserial into `/dev/shm`, stops the web service, takes one
exclusive USB serial open, and copies the original application region to the
laptop. The laptop fsyncs and confirms that backup before any erase/write.
Only the backed-up application sectors are written, once, then fully read
back. Bootloader, partitions and other persistent ESP32 data are preserved.
After a successful verified reset, the tool archives bounded boot UART output
before restoring the web service. It never starts a motion or rearms motors.
This script is pinned to the current reviewed image/update, rather than an
unrestricted arbitrary firmware uploader. A failed/uncertain write is not
automatically retried or rolled back.
