#!/usr/bin/env python3
"""Back up, flash, and verify one explicitly identified external macOS card.

Without --execute this performs read-only preflight. Execution requires root.
The plan is supplied by the operator; this program never guesses a target disk.
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import signal
import stat
import subprocess
import sys
import time
import uuid


CHUNK = 4 * 1024 * 1024
DKIOCSYNCHRONIZECACHE = 0x20006416
F_NOCACHE = 48  # Darwin fcntl.h; F_FULLFSYNC is 51 and is not F_NOCACHE.
DISKUTIL = "/usr/sbin/diskutil"
IOREG = "/usr/sbin/ioreg"
IDENTITY_FIELDS = (
    "DeviceIdentifier", "TotalSize", "Internal", "WholeDisk", "BusProtocol",
    "DeviceTreePath", "MediaName", "DeviceBlockSize", "VirtualOrPhysical",
)


class FlashError(RuntimeError):
    pass


def command(args: list[str]) -> bytes:
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            check=False, timeout=45)
    if result.returncode:
        detail = b"\n".join((result.stdout, result.stderr)).decode("utf-8", "replace").strip()
        raise FlashError(f"Command failed ({result.returncode}): {' '.join(args)}: {detail}")
    return result.stdout


def disk_info(identifier: str) -> dict:
    return plistlib.loads(command([DISKUTIL, "info", "-plist", identifier]))


def walk(value):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from walk(child)


def disk_listing() -> dict:
    return plistlib.loads(command([DISKUTIL, "list", "-plist", "external", "physical"]))


def registry_identity(disk: str) -> dict:
    tree = plistlib.loads(command([IOREG, "-a", "-r", "-c", "IOMedia"]))
    matches = [node for node in walk(tree)
               if node.get("BSD Name") == disk and node.get("Whole") is True]
    if len(matches) != 1:
        raise FlashError(f"Expected exactly one whole IOMedia object for {disk}; got {len(matches)}")
    return matches[0]


def path_absolute(value, label: str) -> Path:
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise FlashError(f"{label} must be an absolute path")
    return Path(value)


def load_plan(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        plan = json.load(handle)
    if not re.fullmatch(r"disk[0-9]+", str(plan.get("disk", ""))):
        raise FlashError("Plan disk must be an exact whole-disk identifier such as disk4")
    expected = plan.get("expected_whole_identity", {})
    if any(key not in expected for key in IDENTITY_FIELDS):
        raise FlashError("Plan lacks required whole-disk identity fields")
    if expected["DeviceIdentifier"] != plan["disk"]:
        raise FlashError("Plan disk and expected DeviceIdentifier disagree")
    if expected["Internal"] is not False or expected["WholeDisk"] is not True:
        raise FlashError("Plan must identify an external whole disk")
    if expected["BusProtocol"] != "USB" or expected["VirtualOrPhysical"] != "Physical":
        raise FlashError("Only external physical USB media are supported")
    size = plan.get("expected_size")
    block = expected["DeviceBlockSize"]
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        raise FlashError("Invalid expected_size")
    if size != expected["TotalSize"] or block != 512 or size % block:
        raise FlashError("Expected size and 512-byte sector geometry disagree")
    if not expected["DeviceTreePath"] or not expected["MediaName"]:
        raise FlashError("Expected USB path and media name are required")
    entry = plan.get("expected_registry_entry_id")
    if isinstance(entry, bool) or not isinstance(entry, int) or entry <= 0:
        raise FlashError("A positive expected_registry_entry_id is required")
    boot = plan.get("expected_boot_partition", plan["disk"] + "s1")
    if not re.fullmatch(re.escape(plan["disk"]) + r"s[0-9]+", boot):
        raise FlashError("Boot partition must belong to the planned whole disk")
    plan["expected_boot_partition"] = boot
    if not plan.get("expected_boot_partition_uuid"):
        raise FlashError("A pre-write boot partition UUID is required")
    if not re.fullmatch(r"[0-9a-fA-F]{64}", str(plan.get("image_sha256", ""))):
        raise FlashError("Plan image_sha256 is not a SHA-256 digest")
    for key in ("image_path", "backup_dir", "report_path"):
        path_absolute(plan.get(key), key)
    if path_absolute(plan["report_path"], "report_path").parent == Path("/dev"):
        raise FlashError("Report must not be under /dev")
    for key in ("owner_uid", "owner_gid"):
        value = plan.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, int) or value < 0):
            raise FlashError(f"Invalid {key}")
    return plan


class Flasher:
    def __init__(self, plan: dict, execute: bool):
        self.plan = plan
        self.execute = execute
        self.disk = plan["disk"]
        self.raw = "/dev/r" + self.disk
        self.block = plan["expected_whole_identity"]["DeviceBlockSize"]
        self.report = {
            "schema_version": 1, "disk": self.disk, "raw_device": self.raw,
            "image_path": plan["image_path"], "expected_image_sha256": plan["image_sha256"].lower(),
            "expected_size": plan["expected_size"], "execute": execute,
            "status": "in_progress", "phase": "created", "started_at": time.time(),
            "bytes_written": 0, "backup": {}, "flush": {},
        }
        self.last_progress = 0.0
        self.lock_fd = None

    def acquire_lock(self):
        target = Path(self.plan["report_path"])
        self.private_dir(target.parent)
        lock = Path(str(target) + ".lock")
        fd = os.open(lock, os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0), 0o600)
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                raise FlashError("Execution lock is not a regular file")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.fchmod(fd, 0o600)
            self.ownership(lock)
            self.lock_fd = fd
        except BaseException:
            os.close(fd)
            raise FlashError("This plan is already running, or its private execution lock is unsafe")

    def release_lock(self):
        if self.lock_fd is not None:
            os.close(self.lock_fd)
            self.lock_fd = None

    def ownership(self, path: Path):
        if os.geteuid() == 0:
            os.chown(path, self.plan.get("owner_uid", -1), self.plan.get("owner_gid", -1))

    def private_dir(self, path: Path):
        # Never redirect privileged output through a pre-existing symlink.
        for part in (path, *path.parents):
            if part.is_symlink():
                raise FlashError(f"Output directory has a symlink component: {part}")
        created = not path.exists()
        path.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(path, 0o700)
        if created:
            self.ownership(path)

    def save_report(self):
        target = Path(self.plan["report_path"])
        self.private_dir(target.parent)
        if target.is_symlink():
            raise FlashError("Refusing symlink report target")
        temporary = target.with_name(target.name + f".tmp-{os.getpid()}")
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(self.report, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            self.ownership(temporary)
            os.replace(temporary, target)
        finally:
            if temporary.exists():
                temporary.unlink()

    def phase(self, name: str, message: str):
        self.report["phase"] = name
        self.report["updated_at"] = time.time()
        print(message, flush=True)
        if self.execute:
            self.save_report()

    def progress(self, count: int, total: int):
        now = time.monotonic()
        if now - self.last_progress >= 5 or count == total:
            self.last_progress = now
            self.report["progress_bytes"] = count
            self.report["progress_total"] = total
            self.report["updated_at"] = time.time()
            print(f"{self.report['phase']}: {count}/{total} bytes ({100 * count / total:.1f}%)", flush=True)
            if self.execute:
                self.save_report()
                self.validate_registry()
                if self.report["phase"] == "backup_read":
                    self.validate_target(mounted=True)

    def validate_registry(self):
        info = registry_identity(self.disk)
        if info.get("IORegistryEntryID") != self.plan["expected_registry_entry_id"]:
            raise FlashError("IOMedia session changed; stop without retrying")
        if info.get("Size") != self.plan["expected_size"] or info.get("Whole") is not True:
            raise FlashError("IOMedia geometry changed")
        if info.get("Preferred Block Size") != self.block:
            raise FlashError("IOMedia block size changed")

    def validate_target(self, boot_uuid: bool = True, mounted: bool = False):
        listing = disk_listing()
        wholes = [node for node in walk(listing)
                  if node.get("DeviceIdentifier") == self.disk
                  and node.get("Size", node.get("TotalSize")) == self.plan["expected_size"]]
        if not wholes:
            raise FlashError("Planned disk is not in the external physical disk inventory")
        current = disk_info(self.disk)
        expected = self.plan["expected_whole_identity"]
        for key, value in expected.items():
            if current.get(key) != value:
                raise FlashError(f"Target identity mismatch: {key}: expected {value!r}, got {current.get(key)!r}")
        if current.get("Writable") is not True:
            raise FlashError("Target is not writable")
        self.validate_registry()
        if boot_uuid:
            info = disk_info(self.plan["expected_boot_partition"])
            field = self.plan.get("expected_boot_partition_uuid_field", "VolumeUUID")
            if str(info.get(field, "")).upper() != str(self.plan["expected_boot_partition_uuid"]).upper():
                raise FlashError("Boot partition UUID changed")
        if mounted:
            identifiers = {node.get("DeviceIdentifier") for node in walk(listing)
                           if re.fullmatch(re.escape(self.disk) + r"s[0-9]+", str(node.get("DeviceIdentifier", "")))}
            identifiers.add(self.disk)
            for identifier in sorted(identifiers):
                info = disk_info(identifier)
                if info.get("Mounted") is True or info.get("MountPoint"):
                    raise FlashError(f"Target remains mounted: {identifier}")
        return current

    def unmount(self):
        self.validate_target()
        command([DISKUTIL, "unmountDisk", "/dev/" + self.disk])
        self.validate_target(mounted=True)

    def open_raw(self, write: bool = False) -> int:
        self.validate_target(mounted=True)
        before = os.stat(self.raw, follow_symlinks=False)
        if not stat.S_ISCHR(before.st_mode):
            raise FlashError("Raw disk path is not a character device")
        flags = os.O_RDWR if write else os.O_RDONLY
        flags |= getattr(os, "O_NOFOLLOW", 0)
        if write:
            flags |= getattr(os, "O_EXLOCK", 0) | os.O_NONBLOCK
            if not getattr(os, "O_EXLOCK", 0):
                raise FlashError("macOS O_EXLOCK is required for exclusive raw writes")
        fd = os.open(self.raw, flags)
        try:
            actual = os.fstat(fd)
            after = os.stat(self.raw, follow_symlinks=False)
            if not stat.S_ISCHR(actual.st_mode) or actual.st_rdev != before.st_rdev or after.st_rdev != before.st_rdev:
                raise FlashError("Raw device identity changed during open")
            # A whole-media writer terminates partition nodes. Check only the
            # still-existing whole media session after opening it.
            self.validate_registry()
            if sys.platform == "darwin":
                fcntl.fcntl(fd, F_NOCACHE, 1)
            self.report["raw_device_rdev"] = actual.st_rdev
            return fd
        except BaseException:
            os.close(fd)
            raise

    def read_chunk(self, fd: int, amount: int) -> bytes:
        data = os.read(fd, amount)
        if not data:
            raise FlashError("Unexpected end of source/device")
        if len(data) % self.block:
            raise FlashError("Unaligned short raw read; stop without retrying")
        return data

    @staticmethod
    def write_all(fd: int, data: bytes, block: int | None = None, on_write=None):
        view = memoryview(data)
        while view:
            written = os.write(fd, view)
            if written <= 0:
                raise FlashError("Write made no progress")
            if on_write is not None:
                on_write(written)
            if block is not None and written % block:
                raise FlashError("Unaligned short raw write; stop without retrying")
            view = view[written:]

    def hash_fd(self, fd: int, total: int, raw: bool = False) -> str:
        digest = hashlib.sha256()
        count = 0
        while count < total:
            amount = min(CHUNK, total - count)
            data = self.read_chunk(fd, amount) if raw else os.read(fd, amount)
            if not data:
                raise FlashError("Unexpected end while hashing")
            digest.update(data)
            count += len(data)
            self.progress(count, total)
        return digest.hexdigest()

    def open_image(self) -> tuple[int, os.stat_result]:
        path = Path(self.plan["image_path"])
        before = path.lstat()
        if not stat.S_ISREG(before.st_mode):
            raise FlashError("Image must be a regular file, not a symlink or device")
        if before.st_size <= 0 or before.st_size % self.block or before.st_size > self.plan["expected_size"]:
            raise FlashError("Image size is invalid, unaligned, or exceeds target capacity")
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
        current = os.fstat(fd)
        if (current.st_dev, current.st_ino, current.st_size) != (before.st_dev, before.st_ino, before.st_size):
            os.close(fd)
            raise FlashError("Image changed during open")
        return fd, current

    def verify_image(self, fd: int, initial: os.stat_result):
        os.lseek(fd, 0, os.SEEK_SET)
        digest = self.hash_fd(fd, initial.st_size)
        current = os.fstat(fd)
        if (current.st_size, current.st_mtime_ns, current.st_ctime_ns) != (initial.st_size, initial.st_mtime_ns, initial.st_ctime_ns):
            raise FlashError("Image changed while hashing")
        if digest != self.plan["image_sha256"].lower():
            raise FlashError("Image SHA-256 does not match the approved plan")
        self.report["image_size"] = initial.st_size
        self.report["image_sha256"] = digest
        os.lseek(fd, 0, os.SEEK_SET)

    def backup(self):
        self.phase("backup_preflight", "Unmounting the identified card before a full backup.")
        self.unmount()
        directory = Path(self.plan["backup_dir"])
        self.private_dir(directory)
        available = os.statvfs(directory).f_bavail * os.statvfs(directory).f_frsize
        if available < self.plan["expected_size"] + 1024 * 1024 * 1024:
            raise FlashError("Insufficient free space for a complete backup plus 1 GiB reserve")
        path = directory / f"{self.disk}-before-flash-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}-{uuid.uuid4().hex[:8]}.img"
        self.report["backup"] = {"path": str(path), "expected_size": self.plan["expected_size"],
                                 "bytes_read": 0, "verified": False}
        target = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.ownership(path)
        source = None
        try:
            source = self.open_raw()
            self.phase("backup_read", "Reading the entire card into a private backup; no card writes yet.")
            digest = hashlib.sha256()
            count = 0
            while count < self.plan["expected_size"]:
                data = self.read_chunk(source, min(CHUNK, self.plan["expected_size"] - count))
                self.write_all(target, data)
                digest.update(data)
                count += len(data)
                self.report["backup"]["bytes_read"] = count
                self.progress(count, self.plan["expected_size"])
            os.fsync(target)
            if os.fstat(target).st_size != self.plan["expected_size"]:
                raise FlashError("Backup size is incomplete")
            self.report["backup"]["source_sha256"] = digest.hexdigest()
        finally:
            os.close(target)
            if source is not None:
                os.close(source)
        self.validate_target()
        self.phase("backup_verify", "Independently hashing the saved full-card backup.")
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
        try:
            digest = self.hash_fd(fd, self.plan["expected_size"])
            if os.fstat(fd).st_size != self.plan["expected_size"]:
                raise FlashError("Backup size changed during verification")
        finally:
            os.close(fd)
        self.report["backup"]["saved_sha256"] = digest
        if digest != self.report["backup"]["source_sha256"]:
            raise FlashError("Saved backup hash differs from the card read digest; no card writes allowed")
        self.report["backup"]["verified"] = True
        self.save_report()

    def flash(self):
        if self.report["backup"].get("verified") is not True:
            raise FlashError("A verified full-card backup is required before flash")
        self.phase("write_preflight", "Backup is verified. Rechecking image and target before any card write.")
        image, initial = self.open_image()
        raw = None
        try:
            self.verify_image(image, initial)
            self.unmount()
            raw = self.open_raw(write=True)
            self.phase("write_image", "Writing the verified image under an exclusive whole-card lock.")
            digest = hashlib.sha256()
            count = 0
            while count < initial.st_size:
                data = os.read(image, min(CHUNK, initial.st_size - count))
                if not data or len(data) % self.block:
                    raise FlashError("Image read failed or became unaligned during write")
                if not self.report.get("write_started"):
                    self.report["write_started"] = True
                    self.save_report()
                def recorded_write(amount):
                    self.report["bytes_written"] += amount
                self.write_all(raw, data, self.block, recorded_write)
                digest.update(data)
                count += len(data)
                self.progress(count, initial.st_size)
            current = os.fstat(image)
            if digest.hexdigest() != self.plan["image_sha256"].lower() or (current.st_size, current.st_mtime_ns, current.st_ctime_ns) != (initial.st_size, initial.st_mtime_ns, initial.st_ctime_ns):
                raise FlashError("Image changed while writing; written card is not approved")
            self.phase("flush", "Requesting the physical device cache flush before readback.")
            try:
                os.fsync(raw)
                self.report["flush"]["fsync"] = "returned_success; Darwin character-device fsync alone does not flush hardware"
            except OSError as error:
                self.report["flush"]["fsync"] = {"errno": error.errno, "error": str(error)}
            try:
                fcntl.ioctl(raw, DKIOCSYNCHRONIZECACHE)
                self.report["flush"]["DKIOCSYNCHRONIZECACHE"] = "success"
            except OSError as error:
                self.report["flush"]["DKIOCSYNCHRONIZECACHE"] = {"errno": error.errno, "error": str(error)}
                self.save_report()
                raise FlashError("Device cache flush failed; persistence is unconfirmed. Stop without retrying.") from error
            self.validate_registry()
            self.phase("readback_verify", "Reading the written image back from the locked raw device with uncached reads.")
            os.lseek(raw, 0, os.SEEK_SET)
            if sys.platform == "darwin":
                fcntl.fcntl(raw, F_NOCACHE, 1)
            readback = self.hash_fd(raw, initial.st_size, raw=True)
            self.report["readback_sha256"] = readback
            if readback != self.plan["image_sha256"].lower():
                raise FlashError("Raw device readback SHA-256 mismatch")
            self.validate_registry()
            self.report["readback_verified"] = True
        finally:
            os.close(image)
            if raw is not None:
                os.close(raw)
        # Closing the writer may recreate and mount the new partition nodes.
        self.phase("eject", "Readback matches. Unmounting and ejecting the identified card.")
        self.validate_target(boot_uuid=False)
        command([DISKUTIL, "unmountDisk", "/dev/" + self.disk])
        self.validate_target(boot_uuid=False, mounted=True)
        command([DISKUTIL, "eject", "/dev/" + self.disk])
        self.report["ejected"] = True

    def run(self):
        self.phase("preflight", "Checking the exact external physical disk identity and source image.")
        self.validate_target()
        image, initial = self.open_image()
        try:
            self.verify_image(image, initial)
        finally:
            os.close(image)
        if not self.execute:
            self.report["status"] = "dry_run_passed"
            print(json.dumps(self.report, ensure_ascii=False, indent=2), flush=True)
            return
        if os.geteuid() != 0:
            raise FlashError("--execute requires macOS administrator/root authentication")
        self.backup()
        self.flash()
        self.report["status"] = "success"
        self.report["finished_at"] = time.time()
        self.phase("complete", "Complete: full backup verified, image written, device flushed, readback matched, card ejected.")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    flasher = None
    try:
        plan = load_plan(args.plan)
        if args.execute and os.geteuid() != 0:
            raise FlashError("--execute requires root; no card access has occurred")
        os.umask(0o077)
        flasher = Flasher(plan, args.execute)
        if args.execute:
            flasher.acquire_lock()
        def interrupted(signum, _frame):
            raise FlashError(f"Interrupted by signal {signum}; no automatic retry")
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, interrupted)
        flasher.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f"FAILED: {error}", file=sys.stderr, flush=True)
        if flasher is not None:
            flasher.report["status"] = "failed"
            flasher.report["error"] = str(error)
            flasher.report["finished_at"] = time.time()
            if flasher.execute and flasher.lock_fd is not None:
                try:
                    flasher.save_report()
                except Exception as report_error:
                    print(f"Could not save failure report: {report_error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if flasher is not None:
            flasher.release_lock()


if __name__ == "__main__":
    raise SystemExit(main())
