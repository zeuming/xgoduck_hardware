# Bounded position-read retry — OFF communication passed, motion acceptance pending

2026-10-10: the candidate application `59033a26323411641e9a5f73466463087dc8f288f66478e5b4af76977925e677` was installed with explicit battery-off confirmation and full readback verification. The original c27 application was backed up before writing. Post-flash battery-off IMU/OFF checks passed. Two earlier powered checks had no valid joint-position feedback (0/40 healthy samples each). After a subsequently observed X5 reboot, the latest read-only check (`powered-readonly-07`) passed **40/40 samples with all 15 joints online, fresh IMU/state timestamps, no fault bits, and zero recorded servo UART errors in that MCU boot**. Torque remained off, calibration was unchanged, and no reset or motion was requested by this check. The recovery cause is not established. This short OFF-mode result does not validate armed retry behavior, stance, or walking; attended motion acceptance remains pending. Evidence: `output/duck_sim/position-read-retry-install/installation-status.json`.

This candidate addresses one specific failure mode: a single silent 1.5 ms
position read currently clears the motor's online bit and releases torque,
even when the previous complete position sample is still within the existing
50 ms validity window. A recorded stop involved ID30, with no hardware UART
error and 14.582 ms position age. A retry recovering that real incident is
**not proven**; this candidate needs an attended hardware validation.

The base is the exact source archive whose application hash is
`c27cfd6fcdf8128f53b24d21a6b34c3a1689d8119fad40dc4f6b928acb8d64ce`,
recorded as restored on 2026-10-09. `base.json` pins every firmware source
hash. This is historical deployment evidence, not a fresh MCU identity read.
The current generic `tools/esp32_motion/firmware` differs in gains and must
not be substituted for this pinned base.

## Change and bounds

- At most **one extra fixed-register 56/6 READ per complete 15-servo cycle**.
- Only while already armed, outside ARM preparation/torque verification or
  maintenance, and with no pending release or torque verification.
- Only after pure silence: no received/drained bytes, rejected packet, device
  error or UART error. Malformed replies and invalid positions still fail.
- Position, IMU and target ages must have room for the existing 1.5 ms read
  deadline. The normal safety checks continue throughout the retry.
- Previous feedback timestamps, ARM cookie and host lease are not renewed.
  A second miss, another servo miss that cycle, or an expired budget uses
  the original stop path. No automatic re-arm or retried motion write.
- Original gains, 600-count/s slew limit, 50 ms freshness, 250 ms target
  watchdog, packet layouts and host tilt/standing/walking policies remain.

A successful retry uses a newly validated position response. This does not
provide an electrical diagnosis, guarantee bus timing, or resolve mechanical
tracking/model issues. A late response from the first identical request can
arrive during the second window; the protocol has no transaction nonce. The
existing quiet-bus drain and ID/checksum/register validation still apply.

## Cached diagnostics

Optional request Type 26, zero-length payload, returns Type `0x9a`, length 28:

| Offset | Meaning |
| --- | --- |
| 0–3 | Version 1, last servo ID, first outcome, retry outcome |
| 4, 8, 12 | Saturating uint32 LE attempts, recovered while armed, failed |
| 16, 20, 24 | Last retry time, first read duration, retry duration (µs) |

Counters are cumulative for the boot. Reads do not consume write sequence
numbers, touch the servo bus, or refresh feedback/target timestamps. Existing
safety checks can still stop an already stale controller. The new query was verified on the installed candidate. It is not automatically polled by the workbench; the finite diagnostic only reads cached records and STATE. Retry counters stayed zero during OFF checks, as required.

## Reproduce and review

From the repository root:

```sh
python3 tools/esp32_motion/position_retry_candidate/prepare.py --output /tmp/duck-position-retry-candidate
sh /tmp/duck-position-retry-candidate/tests/run.sh
```

Preparation refuses changed base source/application hashes and existing output
directories. `candidate.patch` is the complete firmware diff. Preparation
and tests do not access robot hardware. A PlatformIO build is a separate
compile-only operation; **do not use upload targets unattended**.

Evidence is under `output/duck_sim/position-read-retry-candidate/`: ASan/UBSan
suite output, an ESP32-S3 build log, and a CPU-mock original/candidate
comparison with one ID30 reply dropped. Tests cover all 15 IDs, persistent
loss, two misses in one cycle, malformed/device/UART failures, insufficient
time budgets, watchdog expiry during retry, STOP, timer wrap, saturation,
legacy response layouts, and read-only diagnostics.

Before any attended trial, first establish the current firmware identity and
back up/retain the installed image. Flashing resets ESP32 and interrupts
support, so the operator must disconnect the servo battery first. Then verify
readback, boot OFF, unchanged gains/calibration, and communication with motors
off. Hardware validation of recovered/missed-read counts and deadlines comes
before standing and a bounded walking trial. This candidate is not permission
to run or flash while the operator is absent.
