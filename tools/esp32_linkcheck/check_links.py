#!/usr/bin/env python3
"""Read-only diagnostics for the XGODUCK_LINKCHECK ESP32 firmware.

This client sends only the firmware's INFO, IMU, SCAN and READ commands.
It is not a transparent servo bridge and accepts no arbitrary writes.
"""

import argparse
import datetime
import json
import time

import serial


PREFIX = "XGODUCK_LINKCHECK"


def exchange(port, command, timeout):
    port.write((command + "\n").encode("ascii"))
    port.flush()
    expected = command.split()[0]
    deadline = time.monotonic() + timeout
    unrelated = 0
    pending = bytearray()
    while time.monotonic() < deadline:
        raw = port.read(min(port.in_waiting or 1, 4096))
        if not raw:
            continue
        pending.extend(raw)
        if len(pending) > 262144:
            raise ValueError("Diagnostic response exceeded 256 KiB")
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending = bytearray(rest)
            try:
                result = json.loads(line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                unrelated += 1
                continue
            if (
                isinstance(result, dict)
                and result.get("prefix") == PREFIX
                and result.get("command") == expected
            ):
                return result
            unrelated += 1
    raise TimeoutError(f"No {expected} reply from diagnostic firmware ({unrelated} unrelated lines)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--output")
    parser.add_argument("--read", type=int, nargs="*", default=[],
                        help="Additional IDs to read; all scanned IDs are read automatically")
    parser.add_argument("--skip-scan", action="store_true")
    args = parser.parse_args()
    if any(sid < 0 or sid > 253 for sid in args.read):
        parser.error("Servo IDs must be 0–253")
    port = serial.Serial(port=None, baudrate=115200, timeout=0.25)
    port.dtr = False
    port.rts = False
    port.port = args.port
    results = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "port": args.port,
        "read_only_servo_commands": True,
        "responses": [],
    }
    try:
        port.open()
        # Some USB-UART boards reset on open. Allow the bootloader to finish.
        time.sleep(2)
        port.reset_input_buffer()
        results["responses"].append(exchange(port, "INFO", 5))
        results["responses"].append(exchange(port, "IMU", 5))
        read_ids = set(args.read)
        if not args.skip_scan:
            scan = exchange(port, "SCAN", 25)
            results["responses"].append(scan)
            read_ids.update(
                item["id"] for item in scan.get("results", [])
                if isinstance(item.get("id"), int) and 0 <= item["id"] <= 253
            )
        for sid in sorted(read_ids):
            results["responses"].append(exchange(port, f"READ {sid}", 5))
    except (OSError, serial.SerialException, TimeoutError, ValueError) as exc:
        results["error"] = str(exc)
    finally:
        if port.is_open:
            port.close()
    encoded = json.dumps(results, ensure_ascii=False, indent=2)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as output:
            output.write(encoded + "\n")
    print(encoded)
    return 1 if "error" in results else 0


if __name__ == "__main__":
    raise SystemExit(main())
