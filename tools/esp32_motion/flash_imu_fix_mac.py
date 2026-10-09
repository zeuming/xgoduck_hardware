#!/usr/bin/env python3
"""Explicit Mac-only, one-attempt ESP32-S3 application update.

Requires URT battery disconnected and ESP32 disconnected from X5. Importing
this module does not import esptool, enumerate devices, or open a serial port.
No bootloader/partition writes, whole-chip erase, reconnect, write retry or rollback.
ROM synchronization uses the bounded standard esptool handshake on one open port.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import struct
import subprocess
import sys
import termios
import time


WORKSPACE = Path("/Users/lo/Documents/project/xgoduck_hardware")
LIBRARY = Path("/tmp/xgoduck-pio-core/packages/tool-esptoolpy")
IMAGE = WORKSPACE / "output/esp32_motion/firmware-imu-settling-20261004/firmware.bin"
PORT = "/dev/cu.usbserial-1110"
EXPECTED_MAC = "44:bd:8d:6a:6c:00"
APP_OFFSET, APP_SIZE, REGION_SIZE, SECTOR_SIZE = 0x10000, 280608, 0x45000, 0x1000
APP_SHA256 = "632c367355948b8d7f673a393b87eb9842cf448f5f82430e3558ae675e188a9d"
OLD_SIZE = 280576
OLD_SHA256 = "7f58df2f9fb6fabe7040f88b81c3f584439adaaee10ddeeafd458af8343a9306"
PARTITION_SHA256 = "13daf80a09a7a4ee743883aed6779c6833ec4880709ce967a5799c0bd38c06e0"
TOTAL_TIMEOUT = 180
MANUAL_WAIT_SECONDS = 180


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _save_new(path, data):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            fd = None
            if stream.write(data) != len(data):
                raise RuntimeError("Incomplete local evidence write")
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if fd is not None:
            os.close(fd)


def _report(directory, report):
    temp = directory / ".report-next.json"
    _save_new(temp, (json.dumps(report, indent=2) + "\n").encode())
    os.replace(temp, directory / "report.json")


def _phase(directory, report, phase, **extra):
    report.update(phase=phase, **extra)
    _report(directory, report)
    print(json.dumps({"phase": phase, **extra}), flush=True)


def _read_image():
    fd = os.open(IMAGE, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size != APP_SIZE:
            raise RuntimeError("Unexpected packaged application size/type")
        image = stream.read(APP_SIZE + 1)
    if len(image) != APP_SIZE or _sha(image) != APP_SHA256:
        raise RuntimeError("Packaged application hash mismatch")
    if (APP_OFFSET % SECTOR_SIZE or REGION_SIZE % SECTOR_SIZE or
            (APP_SIZE + SECTOR_SIZE - 1) // SECTOR_SIZE * SECTOR_SIZE != REGION_SIZE):
        raise RuntimeError("Application/backup bounds do not match sector alignment")
    return image


def _libraries():
    sys.path.insert(0, str(LIBRARY / "_contrib"))
    sys.path.insert(0, str(LIBRARY))
    import esptool
    import serial
    from serial.tools import list_ports
    from esptool import cmds
    from esptool import loader as loader_module
    from esptool.reset import HardReset, UnixTightReset
    from esptool.targets.esp32s3 import ESP32S3ROM
    if esptool.__version__ != "4.9.0" or Path(esptool.__file__).resolve().parent != LIBRARY.resolve() / "esptool":
        raise RuntimeError("Unreviewed esptool version/path")
    loader_module.SYNC_TIMEOUT = 0.5
    return esptool, serial, list_ports, cmds, HardReset, UnixTightReset, ESP32S3ROM


def _single_loader_class(base, reset_class):
    class SingleAttemptLoader(base):
        def _construct_reset_strategy_sequence(self, mode):
            if mode == "no_reset":
                return (lambda: None,)
            if mode != "default_reset":
                raise RuntimeError("Unexpected reset mode")
            # Call reset(), not ResetStrategy.__call__(), which can reopen/retry.
            return (reset_class(self._port, 0.05).reset,)

        def _connect_attempt(self, reset_strategy, mode="default_reset"):
            if mode not in ("default_reset", "no_reset"):
                raise RuntimeError("Unexpected connection mode")
            # Keep upstream's bounded ROM initialization (up to five SYNCs).
            # SYNC trains the ROM UART baud detection; it never erases/writes
            # flash. The supplied reset strategy is reset() directly, so this
            # remains one open port and at most one reset, without reopening.
            return super()._connect_attempt(reset_strategy, mode=mode)
    return SingleAttemptLoader


def _manual_boot_ready(path):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
        return False
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                info.st_mode & 0o077 or info.st_nlink != 1 or not 0 < info.st_size <= 128):
            raise RuntimeError("Manual boot flag must be an owned private regular file")
        raw = os.read(fd, 129)
        if len(raw) != info.st_size or len(raw) > 128:
            raise RuntimeError("Manual boot flag read was incomplete or too large")
        value = json.loads(raw)
        if not isinstance(value, dict) or set(value) != {"ready"} or value["ready"] is not True:
            raise RuntimeError("Manual boot flag must contain exactly ready=true")
        return True
    finally:
        os.close(fd)


def _wait_manual_boot(directory, report, overall_deadline):
    _phase(directory, report, "waiting_manual_boot", manual_boot_mode=True)
    deadline = min(time.monotonic() + MANUAL_WAIT_SECONDS, overall_deadline)
    path = directory / "manual-boot-ready.json"
    while time.monotonic() < deadline:
        if _manual_boot_ready(path):
            _phase(directory, report, "manual_boot_ready")
            return
        time.sleep(min(0.1, max(0, deadline - time.monotonic())))
    raise TimeoutError("Manual boot readiness was not confirmed within 180 seconds")


def _progress(phase):
    last = [-1]
    def callback(done, total):
        bucket = done // 65536
        if bucket != last[0] or done == total:
            last[0] = bucket
            print(json.dumps({"phase": phase, "bytes": done, "total": total}), flush=True)
    return callback


def _check_partition(table):
    if len(table) != SECTOR_SIZE or _sha(table[:3072]) != PARTITION_SHA256:
        raise RuntimeError("Unknown partition table; nothing written")
    entries = []
    for index in range(0, 3072, 32):
        magic, kind, subtype, offset, size, _label, _flags = struct.unpack("<HBBII16sI", table[index:index + 32])
        if magic != 0x50aa:
            break
        entries.append((kind, subtype, offset, size))
    factory = [entry for entry in entries if entry[:2] == (0, 0)]
    if (len(factory) != 1 or factory[0][2] != APP_OFFSET or
            factory[0][2] + factory[0][3] < APP_OFFSET + REGION_SIZE):
        raise RuntimeError("Factory partition does not cover the application write")
    for entry in entries:
        if entry != factory[0] and entry[2] < APP_OFFSET + REGION_SIZE and entry[2] + entry[3] > APP_OFFSET:
            raise RuntimeError("Another partition overlaps the application write")


def _write_app_once(stub, payload):
    if len(payload) != REGION_SIZE:
        raise RuntimeError("Unexpected application write extent")
    # Smaller explicit blocks avoid default 16 KiB padding past backed-up sectors.
    stub.FLASH_WRITE_SIZE = SECTOR_SIZE
    if stub.get_erase_size(APP_OFFSET, len(payload)) != REGION_SIZE:
        raise RuntimeError("Unexpected loader erase extent")
    blocks = stub.flash_begin(REGION_SIZE, APP_OFFSET)
    if blocks != REGION_SIZE // SECTOR_SIZE:
        raise RuntimeError("Unexpected loader block count")
    for sequence in range(blocks):
        block = payload[sequence * SECTOR_SIZE:(sequence + 1) * SECTOR_SIZE]
        if len(block) != SECTOR_SIZE:
            raise RuntimeError("Unexpected partial write block")
        # Direct check_command has no flash_block/flash_defl_block retry loop.
        stub.check_command("write one application block", stub.ESP_FLASH_DATA,
                           struct.pack("<IIII", len(block), sequence, 0, 0) + block,
                           stub.checksum(block), timeout=3)
        if sequence % 16 == 0 or sequence + 1 == blocks:
            print(json.dumps({"phase": "writing_app", "blocks": sequence + 1,
                              "total_blocks": blocks}), flush=True)
    # Stub ACK precedes physical write completion; this barriers the final block.
    stub.read_reg(stub.CHIP_DETECT_MAGIC_REG_ADDR, timeout=3)


def run(port, run_dir, start_verified_app=False, manual_boot_mode=False):
    if sys.platform != "darwin" or port != PORT:
        raise RuntimeError("Only the reviewed Mac serial port is permitted")
    image = _read_image()
    directory = Path(run_dir).resolve()
    output_root = (WORKSPACE / "output").resolve()
    if not directory.is_relative_to(output_root) or directory == output_root:
        raise RuntimeError("Run evidence must be stored inside this Mac workspace output")
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    if directory.stat().st_uid != os.getuid():
        raise RuntimeError("Run evidence directory has an unexpected owner")
    os.chmod(directory, 0o700)
    # Persistent marker is created before any device inspection/open. Never reuse
    # this run after failure, success, interruption, or an uncertain operation.
    _save_new(directory / "attempt-started.json", (json.dumps({
        "port": port, "app_sha256": APP_SHA256, "pid": os.getpid(),
        "manual_boot_mode": bool(manual_boot_mode),
        "started_unix": time.time()}) + "\n").encode())
    report = {"phase": "prepared", "port": port, "app_sha256": APP_SHA256,
              "app_offset": APP_OFFSET, "application_size": APP_SIZE,
              "backup_size": REGION_SIZE, "write_attempted": False,
              "readback_verified": False, "serial_closed": False,
              "manual_boot_mode": bool(manual_boot_mode),
              "no_reconnect_or_rollback": True, "electrical_safety_verified": False}
    serial_port = None
    started = time.monotonic()
    session_timeout = TOTAL_TIMEOUT + (MANUAL_WAIT_SECONDS if manual_boot_mode else 0)
    previous_handler = signal.getsignal(signal.SIGALRM)
    if signal.getitimer(signal.ITIMER_REAL) != (0.0, 0.0):
        raise RuntimeError("An existing process alarm prevents exclusive timeout control")
    def timed_out(_signal, _frame):
        raise TimeoutError("One-attempt application update deadline expired")
    signal.signal(signal.SIGALRM, timed_out)
    signal.setitimer(signal.ITIMER_REAL, session_timeout)
    try:
        esptool, serial, list_ports, cmds, hard_reset, tight_reset, base = _libraries()
        ports = [item for item in list_ports.comports() if item.device == port]
        if (len(ports) != 1 or ports[0].vid != 0x1a86 or ports[0].pid != 0x7523 or
                ports[0].location != "1-1.1" or ports[0].serial_number is not None):
            raise RuntimeError("Reviewed USB serial identity/location changed")
        expected_device = os.stat(port, follow_symlinks=False)
        if not stat.S_ISCHR(expected_device.st_mode):
            raise RuntimeError("Serial path is not a direct character device")
        holders = subprocess.run(["/usr/sbin/lsof", "-nP", port,
                                  "/dev/tty.usbserial-1110"],
                                 capture_output=True, timeout=5)
        if holders.returncode != 1 or holders.stdout or holders.stderr:
            raise RuntimeError("Serial/cu alias is occupied or ownership check failed")
        serial_port = serial.Serial(port=None, baudrate=115200, timeout=3,
                                    write_timeout=3, exclusive=True)
        serial_port.dtr = False
        serial_port.rts = False
        serial_port.port = port
        _phase(directory, report, "opening_single_loader")
        serial_port.open()
        fcntl.ioctl(serial_port.fileno(), termios.TIOCEXCL)
        actual_device = os.fstat(serial_port.fileno())
        if (actual_device.st_rdev != expected_device.st_rdev or
                actual_device.st_ino != expected_device.st_ino or
                actual_device.st_dev != expected_device.st_dev):
            raise RuntimeError("Serial device changed while opening")
        loader = _single_loader_class(base, tight_reset)(serial_port, baud=115200, trace_enabled=False)
        serial_port.write_timeout = 3
        if manual_boot_mode:
            _wait_manual_boot(directory, report, started + session_timeout)
            signal.setitimer(signal.ITIMER_REAL,
                             min(TOTAL_TIMEOUT, max(.001, started + session_timeout - time.monotonic())))
            pending = serial_port.in_waiting
            if pending > 8192:
                raise RuntimeError("Unexpected continuous UART output before manual sync")
            boot_output = serial_port.read(pending) if pending else b""
            _save_new(directory / "manual-boot-uart.bin", boot_output)
            _phase(directory, report, "manual_boot_uart_recorded",
                   boot_uart_bytes=len(boot_output),
                   boot_uart_text=boot_output.decode("utf-8", "replace")[:2048])
        loader.connect(mode="no_reset" if manual_boot_mode else "default_reset",
                       attempts=1, detecting=False)
        if loader.secure_download_mode or loader.get_chip_id() != 9 or loader.CHIP_NAME != "ESP32-S3":
            raise RuntimeError("Unexpected chip or secure download mode")
        mac = ":".join(f"{byte:02x}" for byte in loader.read_mac())
        if mac != EXPECTED_MAC:
            raise RuntimeError("ESP32 MAC does not match the reviewed controller")
        if loader.get_secure_boot_enabled() or loader.get_flash_encryption_enabled():
            raise RuntimeError("Secure boot/encrypted flash is not permitted")
        stub = loader.run_stub()  # RAM only; shares this same open serial object.
        if stub._port is not serial_port or not stub.IS_STUB:
            raise RuntimeError("Unexpected RAM stub transport")
        flash_id = stub.flash_id()
        if cmds.DETECTED_FLASH_SIZES.get(flash_id >> 16) != "16MB":
            raise RuntimeError("Unexpected or unrecognized flash capacity")
        stub.flash_set_parameters(16 * 1024 * 1024)
        _phase(directory, report, "checking_partition", mac=mac, chip="ESP32-S3", flash_id=flash_id)
        partition = stub.read_flash(0x8000, SECTOR_SIZE, _progress("checking_partition"))
        if len(partition) != SECTOR_SIZE:
            raise RuntimeError("Incomplete partition-table read")
        _save_new(directory / "original-partition-sector.bin", partition)
        _phase(directory, report, "partition_persisted", partition_sector_sha256=_sha(partition),
               partition_table_sha256=_sha(partition[:3072]))
        _check_partition(partition)
        _phase(directory, report, "backing_up_app", mac=mac, chip="ESP32-S3", flash_id=flash_id)
        backup = stub.read_flash(APP_OFFSET, REGION_SIZE, _progress("backing_up_app"))
        if len(backup) != REGION_SIZE:
            raise RuntimeError("Incomplete application-region backup")
        _save_new(directory / "original-app-region.bin", backup)
        backup_hash = _sha(backup)
        if _sha((directory / "original-app-region.bin").read_bytes()) != backup_hash:
            raise RuntimeError("Persisted local backup hash mismatch")
        _save_new(directory / "original-app-region.sha256", (backup_hash + "\n").encode())
        _phase(directory, report, "backup_persisted", backup_sha256=backup_hash,
               original_app_prefix_sha256=_sha(backup[:OLD_SIZE]))
        if _sha(backup[:APP_SIZE]) == APP_SHA256:
            _phase(directory, report, "already_fixed", readback_verified=True)
        else:
            if _sha(backup[:OLD_SIZE]) != OLD_SHA256:
                raise RuntimeError("Unknown existing application; backup kept, nothing written")
            payload = image + backup[APP_SIZE:]
            expected_region_hash = _sha(payload)
            _phase(directory, report, "writing_app", write_attempted=True,
                   write_extent_end=APP_OFFSET + REGION_SIZE,
                   expected_region_sha256=expected_region_hash,
                   preserved_tail_size=REGION_SIZE - APP_SIZE)
            _write_app_once(stub, payload)
            _phase(directory, report, "reading_back_app")
            readback = stub.read_flash(APP_OFFSET, REGION_SIZE, _progress("reading_back_app"))
            if len(readback) != REGION_SIZE or readback != payload or _sha(readback[:APP_SIZE]) != APP_SHA256:
                raise RuntimeError("Application readback mismatch; no retry or rollback")
            _save_new(directory / "verified-app-region.sha256", (_sha(readback) + "\n").encode())
            _phase(directory, report, "verified", readback_verified=True,
                   verified_region_sha256=_sha(readback))
        if start_verified_app:
            # Only an explicitly requested reset after successful readback. Calling
            # reset() directly avoids the reset strategy's reopen/retry wrapper.
            _phase(directory, report, "starting_verified_app")
            hard_reset(serial_port).reset()
            report["app_start_requested"] = True
        else:
            report["left_in_loader"] = True
        report["phase"] = "complete"
        return report
    except BaseException as error:
        report.update(phase="stopped", error=str(error), error_type=type(error).__name__)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        if serial_port is not None:
            try:
                serial_port.close()
                report["serial_closed"] = not serial_port.is_open
            except BaseException as error:
                report["close_error"] = str(error)
        report["elapsed_seconds"] = time.monotonic() - started
        _report(directory, report)
        print(json.dumps(report), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, choices=[PORT])
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--start-verified-app", action="store_true")
    parser.add_argument("--manual-boot-mode", action="store_true")
    args = parser.parse_args()
    run(args.port, args.run_dir, args.start_verified_app, args.manual_boot_mode)


if __name__ == "__main__":
    main()
