#!/usr/bin/env python3
"""Read-only X5 kernel/root-filesystem monitor; all output is saved on the Mac.

Use an already authenticated SSH control socket. This program never repairs,
reboots, installs, retries a failed connection, or writes a remote file.
Consumers must require a live local_pid and a heartbeat age of at most four
seconds; a saved safe=true value is not valid after this process stops. This
observer cannot turn off power or stop other processes operating the board.
Reviewed modes grant separate RAM-only capabilities; safe always remains false.
The calibration capability also requires its consumer to verify the live serial
identity and prohibit ARM, target, and motion commands. This monitor never opens
serial devices and does not grant servo movement or firmware-flashing authority.
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import signal
import stat
import subprocess
import sys
import tempfile
import time
import uuid


def classify_log(line):
    """Return a stop reason for an unsafe kernel message, otherwise None."""
    # Whole-message exceptions confirmed in D-Robotics/x5-kernel sources:
    # drivers/regulator/core.c::of_get_regulator (dev_dbg, absent DT property),
    # fs/ext4/super.c (KERN_INFO after journal recovery). Do not ignore arbitrary
    # "failed"/"recovery" lines or a message with an appended fault.
    message = re.sub(r"^\s*(?:<\d+>)?(?:\[\s*\d+(?:\.\d+)?\]\s*)?", "", line)
    if re.fullmatch(r"sdhci-dwcmshc [0-9a-f]+\.sdhci: Looking up "
                    r"v(?:q)?mmc-supply property in node /\S+ failed", message):
        return None
    if re.fullmatch(r"EXT4-fs \([A-Za-z0-9_.:/-]+\): recovery complete", message):
        return None
    if re.search(r"\b(?:fat-fs|vfat|fat32)\b.*(?:not properly unmounted|unclean|"
                 r"corrupt|\berrors?\b(?!\s*=)|fail(?:ed|ure)?|read.only|please run fsck)", line, re.I):
        return "Kernel reported an unclean FAT volume or filesystem fault"
    if re.search(r"\b(?:systemd-)?journald(?:\[\d+\])?:.*(?:corrupt|\bunclean)", line, re.I):
        return "Journald reported a corrupt or uncleanly closed journal file"
    if re.search(r"\b(?:systemd-)?journald(?:\[\d+\])?:.*failed to (?:read|write) journal", line, re.I):
        return "Journald reported a journal-file read/write failure"
    if re.search(r"(?:\bi/o\s+error\b|buffer\s+i/o\s+error|blk_update_request.*i/o\s+error|"
                 r"end_request.*i/o\s+error|aborting\s+journal|"
                 r"remount(?:ing|ed)?.*read.only)", line, re.I):
        return "Kernel reported a disk I/O or journal failure"
    if re.search(r"\b(?:ext4(?:-fs)?|jbd2)\b.*(?:\berrors?\b(?!\s*=)|abort|failed|failure|"
                 r"timed?\s*out|read.only)", line, re.I):
        return "Kernel reported an EXT4/JBD2 failure"
    if (re.search(r"\b(?:mmc\w*|sdhci\w*|sdmmc\w*|sd[a-z]+|sd)\b", line, re.I)
            and re.search(r"(?:i/o|\berrors?\b(?!\s*=)|fail(?:ed|ure)?|timeout|timed?\s*out|"
                          r"\bcrc\b|recovery|\breset(?:ting)?\b|removed|removal|\bstuck\b|"
                          r"never.*\b(?:busy|programming)\b)", line, re.I)):
        return "Kernel reported an SD/MMC failure"
    if re.search(r"\bcard\b.*(?:stuck|never.*left.*busy)", line, re.I):
        return "Kernel reported a card stuck in a busy/programming state"
    if re.search(r"\busb\b.*(?:disconnect|reset.*\busb\b.*device|"
                 r"device.*(?:error|timeout|timed?\s*out))", line, re.I):
        return "Kernel reported a USB disconnect/reset/error"
    if re.search(r"\bxhci\w*\b.*(?:not responding|hc died|timeout|error)", line, re.I):
        return "Kernel reported a USB host-controller failure"
    if re.search(r"\b(?:under[- ]?voltage|brown[- ]?out)\b.*(?:detect|warn|error|fail|reset)", line, re.I):
        return "Kernel reported an unstable power supply"
    if re.search(r"(?:\b(?:dmesg|kmsg)\b.*(?:overflow|overrun|lost|too small|read.*fail)|"
                 r"messages?\s+(?:lost|dropped|suppressed)|callbacks\s+suppressed)", line, re.I):
        return "Kernel log loss prevents a reliable health check"
    return None


def parse_root_mount(text):
    """Read /proc/self/mountinfo without invoking findmnt on every tick."""
    roots = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 10:
            raise ValueError("Malformed mountinfo entry")
        try:
            separator = fields.index("-", 6)
        except ValueError:
            raise ValueError("Missing mountinfo separator")
        if len(fields) < separator + 4:
            raise ValueError("Incomplete mountinfo entry")
        if fields[4] == "/":
            roots.append({"device": fields[2], "source": fields[separator + 2],
                          "filesystem": fields[separator + 1],
                          "options": fields[5].split(","),
                          "super_options": fields[separator + 3].split(",")})
    if len(roots) != 1:
        raise ValueError("Root mount was not uniquely identified")
    root = roots[0]
    if "rw" not in root["options"] or "ro" in root["options"] or "ro" in root["super_options"]:
        raise ValueError("Root filesystem is not writable")
    if not root["source"] or not root["filesystem"] or not re.fullmatch(r"\d+:\d+", root["device"]):
        raise ValueError("Invalid root mount identity")
    return root


def mount_identity(root):
    return (root["device"], root["source"], root["filesystem"])


def normalized_kernel_hash(text):
    return hashlib.sha256(("\n".join(text.splitlines()) + "\n").encode("utf-8")).hexdigest()


def logical_follow_lines(raw_line):
    """Match finite dmesg splitlines semantics, including CRLF/embedded CR."""
    return [line for line in raw_line.decode("utf-8", "strict").splitlines() if line.strip()]


def validate_review_data(review):
    if (not isinstance(review, dict)
            or review.get("scope") not in ("ram_model_web_only", "ram_motion_calibration_only")):
        raise ValueError("Review must authorize a supported, separate RAM-only scope")
    boot = review.get("expected_boot_id")
    if not isinstance(boot, str) or str(uuid.UUID(boot)) != boot:
        raise ValueError("Invalid reviewed boot ID")
    if not re.fullmatch(r"[0-9a-f]{32}", review.get("expected_cid", "")):
        raise ValueError("Invalid reviewed card CID")
    expected_root = review.get("expected_root")
    if (not isinstance(expected_root, dict) or set(expected_root) != {"device", "source", "filesystem"}
            or expected_root.get("device") != "179:2" or expected_root.get("filesystem") != "ext4"
            or expected_root.get("source") not in ("/dev/mmcblk1p2", "/dev/root")):
        raise ValueError("Review must bind root 179:2/ext4 to /dev/mmcblk1p2 or /dev/root")
    if review.get("required_unmounted_device") != "179:1":
        raise ValueError("Review must require every 179:1 mount to remain absent")
    if not re.fullmatch(r"[0-9a-f]{64}", review.get("kernel_log_sha256", "")):
        raise ValueError("Invalid reviewed kernel-log SHA256")
    allowed = review.get("allowed_historical_lines")
    if (not isinstance(allowed, list) or len(allowed) != 3
            or any(not isinstance(line, str) or "\n" in line or "\r" in line or len(line) > 8192 for line in allowed)
            or len(set(allowed)) != 3):
        raise ValueError("Review must contain exactly three distinct single-line historical messages")
    patterns = [
        r"systemd-journald\[\d+\]: File /\S+ corrupted or uncleanly shut down, renaming and replacing\.",
        r"systemd-journald\[\d+\]: Failed to read journal file /\S+ for rotation, trying to move it out of the way: Text file busy",
        r"FAT-fs \(mmcblk1p1\): Volume was not properly unmounted\. Some data may be corrupt\. Please run fsck\.",
    ]
    found = []
    for line in allowed:
        prefix = re.match(r"^\[\s*\d+\.\d+\]\s*", line)
        if not prefix or not classify_log(line):
            raise ValueError("Historical allowances must include exact kernel timestamps and known warnings")
        matches = [i for i, pattern in enumerate(patterns) if re.fullmatch(pattern, line[prefix.end():])]
        if len(matches) != 1:
            raise ValueError("Review cannot allow arbitrary faults or warning families")
        found.extend(matches)
    if sorted(found) != [0, 1, 2]:
        raise ValueError("Review must contain precisely the three reviewed journal/FAT warning kinds")
    for flag in ("boot_filesystem_warning_cleared", "generic_storage_writes_authorized_by_this_review",
                 "servo_motion_authorized_by_this_review", "servo_arm_authorized"):
        if flag in review and review[flag] is not False:
            raise ValueError("RAM-only review must not clear filesystem warnings or authorize writes/motion")
    if review["scope"] == "ram_motion_calibration_only":
        if review.get("physical_support_confirmed") is not True or review.get("servo_arm_authorized") is not False:
            raise ValueError("Calibration review requires confirmed body support and explicitly forbids ARM")
        if review.get("servo_motion_authorized_by_this_review") is not False:
            raise ValueError("Calibration review must explicitly forbid servo motion")
        serial = review.get("serial_device")
        if not isinstance(serial, dict) or set(serial) != {"path", "vid", "pid", "serial_number", "syspath"}:
            raise ValueError("Calibration review must bind one exact serial path/VID/PID/serial identity")
        if (not isinstance(serial["path"], str)
                or not re.fullmatch(r"/dev/tty(?:USB|ACM)[0-9]+", serial["path"])
                or not isinstance(serial["vid"], str) or not re.fullmatch(r"[0-9a-f]{4}", serial["vid"])
                or not isinstance(serial["pid"], str) or not re.fullmatch(r"[0-9a-f]{4}", serial["pid"])
                # CH340 devices may expose no serial number. Exact USB topology,
                # path, VID/PID, boot, and runtime character-device proof remain
                # mandatory; absence is explicitly bound as the empty string.
                or not isinstance(serial["serial_number"], str) or len(serial["serial_number"]) > 256
                or serial["serial_number"] != serial["serial_number"].strip()
                or any(ord(character) < 32 or ord(character) == 127 for character in serial["serial_number"])
                or not isinstance(serial["syspath"], str) or not serial["syspath"].startswith("/sys/devices/")
                or any(part in ("", ".", "..") for part in serial["syspath"].split("/")[1:])
                or any(ord(character) < 33 or ord(character) == 127 for character in serial["syspath"])):
            raise ValueError("Invalid exact reviewed serial identity")
    return review


def load_review_file(value, return_digest=False):
    path = Path(value)
    if not path.is_absolute():
        raise ValueError("Reviewed baseline must be an absolute local file")
    fd = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o600:
            raise ValueError("Reviewed baseline must be owned by this Mac user with permissions 0600")
        if info.st_size > 65536:
            raise ValueError("Reviewed baseline is too large")
        with os.fdopen(fd, "rb") as stream:
            fd = None
            payload = stream.read(65537)
            if len(payload) > 65536:
                raise ValueError("Reviewed baseline grew beyond the size limit")
            review = json.loads(payload.decode("utf-8"))
    finally:
        if fd is not None:
            os.close(fd)
    validated = validate_review_data(review)
    if return_digest:
        return validated, hashlib.sha256(payload).hexdigest()
    return validated


def root_block_identity(device_stat):
    if not stat.S_ISBLK(device_stat.st_mode):
        raise RuntimeError("/dev/mmcblk1p2 is not a block device")
    device = "%d:%d" % (os.major(device_stat.st_rdev), os.minor(device_stat.st_rdev))
    if device != "179:2":
        raise RuntimeError("/dev/mmcblk1p2 block identity changed")
    return {"path": "/dev/mmcblk1p2", "device": device, "is_block": True}


def validate_ram_snapshot(review, boot, root, cid, mounted_devices, swap_entries, root_block_device=None):
    if boot != review["expected_boot_id"] or mount_identity(root) != mount_identity(review["expected_root"]):
        raise RuntimeError("Reviewed boot/root identity changed")
    if ("rw" not in root["options"] or "ro" in root["options"] or "ro" in root["super_options"]
            or cid != review["expected_cid"]):
        raise RuntimeError("Reviewed root writability/card identity changed")
    if (not isinstance(mounted_devices, list) or not mounted_devices
            or any(not isinstance(device, str) or not re.fullmatch(r"\d+:\d+", device) for device in mounted_devices)
            or root["device"] not in mounted_devices
            or review["required_unmounted_device"] in mounted_devices):
        raise RuntimeError("Boot partition is mounted or mount inventory is invalid")
    if not isinstance(swap_entries, list) or swap_entries:
        raise RuntimeError("RAM-only mode requires no active swap")
    if root_block_device != {"path": "/dev/mmcblk1p2", "device": "179:2", "is_block": True}:
        raise RuntimeError("Verified /dev/mmcblk1p2 block identity is missing or changed")


def parse_swap_entries(text):
    lines = text.splitlines()
    if not lines or lines[0].split() != ["Filename", "Type", "Size", "Used", "Priority"]:
        raise ValueError("Malformed /proc/swaps")
    entries = []
    for line in lines[1:]:
        if line.strip():
            fields = line.split()
            if len(fields) != 5:
                raise ValueError("Malformed swap entry")
            entries.append(fields[0])
    return entries


def validate_reviewed_kernel(review, text):
    lines = text.splitlines()
    if not any(line.strip() for line in lines) or normalized_kernel_hash(text) != review["kernel_log_sha256"]:
        raise RuntimeError("Full retained kernel log does not match the reviewed SHA256")
    failures = [line for line in lines if classify_log(line)]
    if failures != review["allowed_historical_lines"]:
        raise RuntimeError("Kernel warnings differ from the three exact reviewed historical messages")
    return lines


class BaselineReplay:
    """One finite, ordered replay; identical messages after its end are new."""
    def __init__(self, lines, review):
        self.lines = [line for line in lines if line.strip()]
        self.allowed = set(review["allowed_historical_lines"])
        self.position = 0

    @property
    def done(self):
        return self.position == len(self.lines)

    def accept(self, line):
        if self.done:
            return False
        if line != self.lines[self.position]:
            raise RuntimeError("Initial follower replay differs from the reviewed kernel sequence")
        self.position += 1
        return line in self.allowed


def check_log_event(event, review, baseline_reviewed, healthy):
    line = event["line"]
    if not isinstance(line, str):
        raise ValueError("Invalid kernel line")
    reason = classify_log(line)
    marked = event.get("reviewed", False)
    if not isinstance(marked, bool):
        raise ValueError("Invalid reviewed marker")
    if marked:
        if (review is None or not baseline_reviewed or healthy or reason is None
                or event.get("phase") not in ("baseline", "initial_follower_replay")
                or line not in review["allowed_historical_lines"]):
            raise RuntimeError("Historical review is invalid outside its verified initial replay")
    elif reason:
        raise RuntimeError(reason + ": " + line)


def check_health_event(event, baseline_boot, baseline_mount):
    boot = event["boot_id"]
    root = event["root"]
    if (str(uuid.UUID(boot)) != boot or "rw" not in root["options"]
            or "ro" in root["options"] or "ro" in root["super_options"]):
        raise ValueError("Invalid health event")
    identity = mount_identity(root)
    if not all(identity) or not re.fullmatch(r"\d+:\d+", identity[0]):
        raise ValueError("Invalid root mount identity in health event")
    if event["event"] == "healthy":
        if baseline_boot is not None or event.get("baseline_lines", 0) <= 0:
            raise ValueError("Invalid or repeated baseline")
    elif (event["event"] != "heartbeat" or baseline_boot is None
          or boot != baseline_boot or identity != baseline_mount):
        raise RuntimeError("Unexpected boot/root identity or heartbeat before baseline")
    return boot, identity


# Only these standard-library, read-only operations run on the X5. -B prevents
# Python bytecode files; no output directory, log, or temporary file is remote.
REMOTE_MAIN = r'''
import os, selectors, signal, stat, subprocess, sys, time, uuid

def emit(event, **fields):
    fields["event"] = event
    print(json.dumps(fields, ensure_ascii=False), flush=True)

def snapshot():
    with open("/proc/self/mountinfo", "r") as stream:
        mount_text = stream.read()
    root = parse_root_mount(mount_text)
    with open("/proc/sys/kernel/random/boot_id", "r") as stream:
        boot = stream.read().strip()
    if str(uuid.UUID(boot)) != boot:
        raise ValueError("Invalid boot ID")
    fields = {"boot_id": boot, "root": root}
    if REMOTE_REVIEW is not None:
        with open("/sys/class/block/mmcblk1/device/cid", "r") as stream:
            cid = stream.read().strip().lower()
        with open("/proc/swaps", "r") as stream:
            swap_entries = parse_swap_entries(stream.read())
        block_device = root_block_identity(os.stat("/dev/mmcblk1p2"))
        mounted_devices = sorted(set(line.split()[2] for line in mount_text.splitlines()))
        validate_ram_snapshot(REMOTE_REVIEW, boot, root, cid, mounted_devices, swap_entries, block_device)
        fields.update(card_cid=cid, mounted_devices=mounted_devices, swap_entries=swap_entries,
                      root_block_device=block_device)
    return fields

def log_line(line, phase, reviewed=False):
    if not line.strip():
        return
    emit("log", phase=phase, line=line, reviewed=reviewed)
    reason = classify_log(line)
    if reason and not reviewed:
        raise RuntimeError(reason + ": " + line)

def main():
    follower = None
    def interrupted(signum, frame):
        raise RuntimeError("Read-only monitor interrupted")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        # Start following first and observe an actual record before the finite
        # read. A successful baseline covers the retained ring buffer only.
        follower = subprocess.Popen(["dmesg", "--follow", "--color=never"],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        selector = selectors.DefaultSelector()
        selector.register(follower.stdout, selectors.EVENT_READ, "log")
        selector.register(follower.stderr, selectors.EVENT_READ, "error")
        buffers = {"log": b"", "error": b""}
        pending = []
        replay = None
        startup_verified = REMOTE_REVIEW is None

        def drain(timeout):
            nonlocal replay
            records = 0
            for key, _ in selector.select(timeout):
                data = os.read(key.fileobj.fileno(), 65536)
                if not data:
                    raise RuntimeError("dmesg follower stream closed")
                kind = key.data
                if kind == "error":
                    raise RuntimeError("dmesg follower error: " + data.decode("utf-8", "replace").strip())
                buffers[kind] += data
                if len(buffers[kind]) > 1048576:
                    raise RuntimeError("Malformed oversized dmesg line")
                while b"\n" in buffers[kind]:
                    raw_line, buffers[kind] = buffers[kind].split(b"\n", 1)
                    for line in logical_follow_lines(raw_line):
                        records += 1
                        if REMOTE_REVIEW is not None and not startup_verified:
                            # No capability is granted yet. Defer only the three
                            # exact candidate historical records until full proof.
                            if classify_log(line) and line not in REMOTE_REVIEW["allowed_historical_lines"]:
                                log_line(line, "startup_pending")
                            pending.append(line)
                        elif replay is not None and not replay.done:
                            reviewed = replay.accept(line)
                            log_line(line, "initial_follower_replay", reviewed)
                        else:
                            log_line(line, "follow")
            return records

        ready_deadline = time.monotonic() + 5
        while drain(0.25) == 0:
            if follower.poll() is not None or time.monotonic() >= ready_deadline:
                raise RuntimeError("dmesg follower did not produce a startup record")
        baseline = subprocess.run(["dmesg", "--color=never"], stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, timeout=5, check=False)
        if baseline.returncode != 0:
            raise RuntimeError("dmesg baseline failed: " + baseline.stderr.decode("utf-8", "replace").strip())
        lines = baseline.stdout.decode("utf-8", "strict").splitlines()
        if not any(line.strip() for line in lines):
            raise RuntimeError("dmesg baseline was empty; health cannot be established")
        fields = snapshot()
        boot, root = fields["boot_id"], fields["root"]
        if REMOTE_REVIEW is not None:
            validate_reviewed_kernel(REMOTE_REVIEW, baseline.stdout.decode("utf-8", "strict"))
            emit("baseline_reviewed", kernel_log_sha256=REMOTE_REVIEW["kernel_log_sha256"],
                 allowed_historical_lines=REMOTE_REVIEW["allowed_historical_lines"], **fields)
            startup_verified = True
            replay = BaselineReplay(lines, REMOTE_REVIEW)
        for line in lines:
            reviewed = REMOTE_REVIEW is not None and line in REMOTE_REVIEW["allowed_historical_lines"]
            log_line(line, "baseline", reviewed)
        if replay is not None:
            for line in pending:
                was_initial = not replay.done
                reviewed = replay.accept(line)
                log_line(line, "initial_follower_replay" if was_initial else "follow", reviewed)
            pending.clear()
            replay_deadline = time.monotonic() + 5
            while not replay.done:
                drain(0.25)
                if follower.poll() is not None or time.monotonic() >= replay_deadline:
                    raise RuntimeError("Reviewed initial follower replay did not complete")
        if follower.poll() is not None:
            raise RuntimeError("dmesg follower exited during baseline")

        # Consume already queued follower messages before declaring healthy.
        while selector.select(0):
            drain(0)
        # Recheck identities, mount inventory and swap immediately before grant.
        fields = snapshot()
        if fields["boot_id"] != boot or mount_identity(fields["root"]) != mount_identity(root):
            raise RuntimeError("Boot/root changed during baseline startup")
        emit("healthy", baseline_lines=len(lines), replay_complete=replay is not None and replay.done, **fields)
        next_snapshot = time.monotonic() + 1
        while True:
            drain(max(0, min(1, next_snapshot - time.monotonic())))
            if follower.poll() is not None:
                raise RuntimeError("dmesg follower exited")
            if time.monotonic() >= next_snapshot:
                current = snapshot()
                current_boot, current_root = current["boot_id"], current["root"]
                if current_boot != boot:
                    raise RuntimeError("Boot ID changed unexpectedly")
                if mount_identity(current_root) != mount_identity(root):
                    raise RuntimeError("Root mount identity changed unexpectedly")
                emit("heartbeat", **current)
                next_snapshot = time.monotonic() + 1
    except Exception as error:
        emit("fatal", reason=str(error))
        return 2
    finally:
        if follower is not None:
            follower.terminate()
            try:
                follower.wait(timeout=2)
            except subprocess.TimeoutExpired:
                follower.kill()
                follower.wait()
    return 0

sys.exit(main())
'''


def remote_source(review=None):
    import inspect
    names = (classify_log, parse_root_mount, mount_identity, normalized_kernel_hash,
             logical_follow_lines, validate_review_data, root_block_identity, validate_ram_snapshot, parse_swap_entries,
             validate_reviewed_kernel, BaselineReplay)
    return ("import json, re, hashlib, uuid\n"
            + "\n".join(inspect.getsource(item) for item in names)
            + "\nREMOTE_REVIEW = json.loads(" + repr(json.dumps(review)) + ")\n"
            + "if REMOTE_REVIEW is not None: validate_review_data(REMOTE_REVIEW)\n" + REMOTE_MAIN)


def timestamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


class Recorder:
    def __init__(self, directory, review=None):
        self.directory = directory
        self.ram_mode = review is not None
        self.scope = review["scope"] if self.ram_mode else "strict_storage_monitor"
        self.status_path = directory / "status.json"
        self.events = self.new_file("events.jsonl")
        self.kernel = self.new_file("kernel.log")
        self.state = {"safe": False, "state": "starting", "reason": "Awaiting retained-log baseline",
                      "local_pid": os.getpid(), "freshness_limit_seconds": 4,
                      "safe_for_ram_model_web": False,
                      "safe_for_ram_motion_calibration": False,
                      "capability_scope": self.scope}
        if self.scope == "ram_motion_calibration_only":
            self.state.update(physical_support_confirmed=review["physical_support_confirmed"],
                              servo_arm_authorized=review["servo_arm_authorized"],
                              servo_motion_authorized_by_this_review=False,
                              serial_device=dict(review["serial_device"]),
                              requires_serial_identity_proof=True,
                              serial_identity_runtime_verified_by_guard=False)
        self.status()

    def new_file(self, name):
        fd = os.open(str(self.directory / name), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        return os.fdopen(fd, "w", encoding="utf-8", buffering=1)

    def status(self, **updates):
        if self.ram_mode and updates.get("safe") is True:
            raise ValueError("RAM-only mode must never authorize generic storage operations")
        if (updates.get("safe_for_ram_model_web") is True and self.scope != "ram_model_web_only"
                or updates.get("safe_for_ram_motion_calibration") is True
                and self.scope != "ram_motion_calibration_only"):
            raise ValueError("A RAM-only capability cannot cross its reviewed scope")
        if "capability_scope" in updates and updates["capability_scope"] != self.scope:
            raise ValueError("A running guard cannot change its reviewed scope")
        if self.scope == "ram_motion_calibration_only":
            for field in ("physical_support_confirmed", "servo_arm_authorized",
                          "servo_motion_authorized_by_this_review", "serial_device",
                          "requires_serial_identity_proof", "serial_identity_runtime_verified_by_guard"):
                if field in updates and updates[field] != self.state[field]:
                    raise ValueError("A guard cannot change reviewed calibration authority or invent serial proof")
        self.state.update(updates)
        self.state["timestamp"] = timestamp()
        fd, temporary = tempfile.mkstemp(prefix=".status-", dir=str(self.directory))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.state, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            os.replace(temporary, self.status_path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def event(self, event):
        self.events.write(json.dumps({"received_at": timestamp(), **event}, ensure_ascii=False) + "\n")
        if event.get("event") == "log":
            self.kernel.write(event["line"] + "\n")

    def close(self):
        self.events.close()
        self.kernel.close()


def validate_directory(value):
    directory = Path(value)
    if not directory.is_absolute() or directory.is_symlink():
        raise ValueError("Output directory must be an absolute local directory, not a symlink")
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = directory.stat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o077:
        raise ValueError("Output directory must be owned by this user with permissions 0700")
    if any((directory / name).exists() for name in ("events.jsonl", "kernel.log", "status.json")):
        raise ValueError("Use a fresh output directory; existing monitor records are never overwritten")
    return directory


def validate_ssh(args):
    if not re.fullmatch(r"[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.:-]+", args.target):
        raise ValueError("Target must be a plain user@host")
    control = Path(args.control_path)
    known = Path(args.known_hosts)
    if not control.is_absolute() or not stat.S_ISSOCK(control.stat().st_mode):
        raise ValueError("An existing absolute SSH control socket is required")
    if not known.is_absolute() or known.is_symlink() or not known.is_file():
        raise ValueError("An absolute regular known-hosts file is required")


def monitor(args):
    review_path = getattr(args, "reviewed_baseline", None)
    review, review_digest = load_review_file(review_path, return_digest=True) if review_path else (None, None)
    validate_ssh(args)
    recorder = Recorder(validate_directory(args.output_dir), review)
    process = None
    try:
        command = ["/usr/bin/ssh", "-T", "-S", args.control_path,
                   "-F", "/dev/null", "-o", "ProxyCommand=/usr/bin/false",
                   "-o", "ControlMaster=no", "-o", "BatchMode=yes",
                   "-o", "StrictHostKeyChecking=yes", "-o", "UserKnownHostsFile=" + args.known_hosts,
                   "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=1",
                   args.target, "python3 -B -S -u -"]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.stdin.write(remote_source(review).encode("utf-8"))
        process.stdin.close()
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        buffer = b""
        started_at = time.monotonic()
        last_heartbeat = started_at
        baseline_boot = None
        baseline_mount = None
        baseline_reviewed = False
        print("Guard starting; retained-log baseline must pass before any board changes.", flush=True)
        while True:
            for key, _ in selector.select(0.25):
                data = os.read(key.fileobj.fileno(), 65536)
                if not data:
                    raise RuntimeError("SSH monitor stream closed; stop all board operations")
                if key.data == "stderr":
                    message = data.decode("utf-8", "replace").strip()
                    recorder.event({"event": "ssh_stderr", "message": message})
                    raise RuntimeError("SSH monitor error: " + message)
                buffer += data
                if len(buffer) > 1048576:
                    raise RuntimeError("Oversized or malformed monitor event")
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    event = json.loads(raw.decode("utf-8", "strict"))
                    if not isinstance(event, dict):
                        raise ValueError("Monitor event is not an object")
                    kind = event.get("event")
                    if kind not in ("log", "baseline_reviewed", "healthy", "heartbeat", "fatal"):
                        raise ValueError("Unknown monitor event")
                    recorder.event(event)
                    if kind == "log":
                        check_log_event(event, review, baseline_reviewed, baseline_boot is not None)
                    elif kind == "baseline_reviewed":
                        if (review is None or baseline_reviewed or baseline_boot is not None
                                or event.get("kernel_log_sha256") != review["kernel_log_sha256"]
                                or event.get("allowed_historical_lines") != review["allowed_historical_lines"]):
                            raise RuntimeError("Unexpected or mismatched reviewed-baseline proof")
                        validate_ram_snapshot(review, event["boot_id"], event["root"], event["card_cid"],
                                              event["mounted_devices"], event["swap_entries"], event["root_block_device"])
                        baseline_reviewed = True
                    elif kind == "fatal":
                        raise RuntimeError(event.get("reason", "Remote monitor failed"))
                    else:
                        boot, identity = check_health_event(event, baseline_boot, baseline_mount)
                        root = event["root"]
                        if review is not None:
                            validate_ram_snapshot(review, boot, root, event["card_cid"],
                                                  event["mounted_devices"], event["swap_entries"], event["root_block_device"])
                            if not baseline_reviewed or (kind == "healthy" and event.get("replay_complete") is not True):
                                raise RuntimeError("RAM capability requires a verified baseline and complete finite replay")
                        if kind == "healthy":
                            baseline_boot, baseline_mount = boot, identity
                            if review is None:
                                print("Guard healthy: writable root, valid kernel baseline; monitoring continuously.", flush=True)
                            else:
                                print("RAM-only guard ready; generic storage operations remain blocked.", flush=True)
                        last_heartbeat = time.monotonic()
                        updates = {}
                        if review is not None:
                            updates.update(card_cid=event["card_cid"], kernel_log_sha256=review["kernel_log_sha256"],
                                           reviewed_kernel_log_sha256=review["kernel_log_sha256"],
                                           reviewed_baseline_sha256=review_digest,
                                           mounted_devices=event["mounted_devices"], swap_entries=event["swap_entries"],
                                           root_block_device=event["root_block_device"],
                                           boot_filesystem_warning_cleared=False)
                        recorder.status(safe=review is None,
                                        safe_for_ram_model_web=review is not None and review["scope"] == "ram_model_web_only",
                                        safe_for_ram_motion_calibration=review is not None and review["scope"] == "ram_motion_calibration_only",
                                        state="monitoring", reason=None,
                                        boot_id=boot, root=root, last_heartbeat=timestamp(),
                                        last_heartbeat_monotonic=last_heartbeat, **updates)
            if process.poll() is not None:
                raise RuntimeError("SSH monitor exited (code %s); stop all board operations" % process.returncode)
            limit = 10 if baseline_boot is None else 4
            if time.monotonic() - last_heartbeat > limit:
                raise RuntimeError("Monitor heartbeat timed out; stop all board operations")
    except (Exception, KeyboardInterrupt) as error:
        reason = str(error) or "Monitor interrupted; stop all board operations"
        recorder.status(safe=False, safe_for_ram_model_web=False,
                        safe_for_ram_motion_calibration=False, state="stopped", reason=reason)
        recorder.event({"event": "local_stop", "reason": reason})
        print("STOP: " + reason, file=sys.stderr, flush=True)
        return 2
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for stream in (process.stdout, process.stderr):
                if stream is not None:
                    stream.close()
        recorder.close()


def self_test():
    import unittest

    historical = [
        "[    7.527571] systemd-journald[275]: File /var/log/journal/example/system.journal corrupted or uncleanly shut down, renaming and replacing.",
        "[   13.049554] systemd-journald[275]: Failed to read journal file /var/log/journal/example/user-1000.journal for rotation, trying to move it out of the way: Text file busy",
        "[   20.013542] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.",
    ]
    kernel_text = "[    0.000000] Booting Linux\n" + "\n".join(historical) + "\n[   21.000000] ready\n"
    fixture_review = {"scope": "ram_model_web_only", "expected_boot_id": "7f8fb795-0e97-46df-b5f5-e1b5be5d2ed7",
                      "expected_cid": "fe34325344313647200000052e013c00",
                      "expected_root": {"device": "179:2", "source": "/dev/mmcblk1p2", "filesystem": "ext4"},
                      "kernel_log_sha256": normalized_kernel_hash(kernel_text),
                      "allowed_historical_lines": historical, "required_unmounted_device": "179:1"}
    fixture_motion_review = {**fixture_review, "scope": "ram_motion_calibration_only",
                             "physical_support_confirmed": True, "servo_arm_authorized": False,
                             "servo_motion_authorized_by_this_review": False,
                             "serial_device": {"path": "/dev/ttyUSB0", "vid": "1a86", "pid": "7523",
                                               "serial_number": "",
                                               "syspath": "/sys/devices/platform/soc/35000000.hsio_apb/35100000.usb3/35100000.usb/xhci-hcd.2.auto/usb1/1-1/1-1.3"}}
    root = parse_root_mount("24 1 179:2 / / rw - ext4 /dev/mmcblk1p2 rw\n")
    health_fields = {"boot_id": fixture_review["expected_boot_id"], "root": root,
                     "card_cid": fixture_review["expected_cid"], "mounted_devices": ["179:2", "0:3"], "swap_entries": [],
                     "root_block_device": {"path": "/dev/mmcblk1p2", "device": "179:2", "is_block": True}}

    class GuardTests(unittest.TestCase):
        def test_failure_messages(self):
            for line in ("mmc1: timeout waiting for hardware interrupt",
                         "mmcblk1: error -84 transferring data",
                         "mmc1: reset controller/card",
                         "mmc1: card never left busy state",
                         "Card stuck in programming state!",
                         "sd 0:0:0:0: [sda] tag#0 FAILED Result",
                         "Buffer I/O error on dev mmcblk1p2, logical block 0",
                         "I/O error, dev nvme0n1, sector 1234",
                         "EXT4-fs error (device mmcblk1p2): ext4_journal_check_start",
                         "JBD2: Detected IO errors while flushing file data",
                         "Aborting journal on device mmcblk1p2-8.",
                         "EXT4-fs (mmcblk1p2): Remounting filesystem read-only",
                         "usb 1-1: USB disconnect, device number 3",
                         "usb 1-1: reset high-speed USB device number 3 using xhci-hcd",
                         "Under-voltage detected!", "Brown-out reset detected",
                         "dmesg: kmsg buffer is too small", "printk: 25 messages dropped"):
                with self.subTest(line=line):
                    self.assertIsNotNone(classify_log(line))

        def test_fat_and_journald_failures(self):
            for line in (
                    "[   20.013542] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.",
                    "FAT-fs (mmcblk1p1): error, corrupted directory",
                    "FAT-fs (mmcblk1p1): Filesystem has been set read-only",
                    "FAT-fs (mmcblk1p1): FAT read failed (blocknr 400)",
                    "vfat (mmcblk1p1): unclean volume; please run fsck",
                    "[    7.527571] systemd-journald[275]: File /var/log/journal/example/system.journal corrupted or uncleanly shut down, renaming and replacing.",
                    "journald[275]: journal file corrupted",
                    "systemd-journald: journal was uncleanly shut down",
                    "[   13.049554] systemd-journald[275]: Failed to read journal file /var/log/journal/example/user-1000.journal for rotation, trying to move it out of the way: Text file busy"):
                with self.subTest(line=line):
                    self.assertIsNotNone(classify_log(line))

        def test_normal_messages(self):
            for line in ("mmc1: new ultra high speed SDR104 SDXC card",
                         "mmcblk1: mmc1:1234 SD32G 29.7 GiB",
                         "EXT4-fs (mmcblk1p2): mounted filesystem with ordered data mode",
                         "EXT4-fs (mmcblk1p2): re-mounted. Opts: errors=remount-ro. Quota mode: none.",
                         "[    4.108098] EXT4-fs (mmcblk1p2): recovery complete",
                         "[    3.164875] sdhci-dwcmshc 35030000.sdhci: Looking up vmmc-supply property in node /soc/hsio_apb/sdhci@35030000 failed",
                         "[    3.215987] sdhci-dwcmshc 35020000.sdhci: Looking up vqmmc-supply property in node /soc/hsio_apb/sdhci@35020000 failed",
                         "systemd-journald[275]: Received client request to flush runtime journal.",
                         "hpu3501-regulator hpu3501-regulator.0.auto: fault_cfgr will be default value",
                         "usb 1-1: new high-speed USB device number 3 using xhci-hcd",
                         "sd 0:0:0:0: [sda] Attached SCSI removable disk"):
                self.assertIsNone(classify_log(line))

        def test_source_proven_exceptions_are_exact(self):
            lookup = ("sdhci-dwcmshc 35030000.sdhci: Looking up vmmc-supply property "
                      "in node /soc/hsio_apb/sdhci@35030000 failed")
            recovery = "EXT4-fs (mmcblk1p2): recovery complete"
            for line in (lookup + "; mmc1: timeout", lookup + ": error -110",
                         "sdhci-dwcmshc 35030000.sdhci: enabling vqmmc regulator failed",
                         "mmc1: recovery failed", recovery + "; I/O error, dev mmcblk1",
                         recovery + "; journal aborted", "EXT4-fs (mmcblk1p2): recovery error"):
                with self.subTest(line=line):
                    self.assertIsNotNone(classify_log(line))

        def test_root_mount(self):
            line = "24 1 179:2 / / rw,relatime - ext4 /dev/mmcblk1p2 rw,errors=remount-ro\n"
            root = parse_root_mount(line)
            self.assertEqual(mount_identity(root), ("179:2", "/dev/mmcblk1p2", "ext4"))
            for text in ("", line.replace("/ / rw", "/ / ro"),
                         line.replace("rw,errors", "ro,errors"), line + line, "malformed"):
                with self.assertRaises(ValueError):
                    parse_root_mount(text)

        def test_remote_compiles_without_execution(self):
            compile(remote_source(), "<remote-read-only-monitor>", "exec")
            compile(remote_source(fixture_review), "<remote-read-only-ram-monitor>", "exec")
            compile(remote_source(fixture_motion_review), "<remote-read-only-calibration-monitor>", "exec")

        def test_motion_review_requires_support_and_forbids_motion(self):
            import copy
            validate_review_data(fixture_motion_review)
            for field, value in (("physical_support_confirmed", False), ("physical_support_confirmed", 1),
                                 ("servo_arm_authorized", True), ("servo_arm_authorized", 0),
                                 ("servo_motion_authorized_by_this_review", True),
                                 ("servo_motion_authorized_by_this_review", None),
                                 ("scope", "ram_model_and_motion")):
                changed = copy.deepcopy(fixture_motion_review)
                changed[field] = value
                with self.assertRaises(ValueError):
                    validate_review_data(changed)
            for field in ("physical_support_confirmed", "servo_arm_authorized",
                          "servo_motion_authorized_by_this_review", "serial_device"):
                changed = copy.deepcopy(fixture_motion_review)
                del changed[field]
                with self.assertRaises(ValueError):
                    validate_review_data(changed)
            for field, value in (("path", "/dev/ttyUSB0/../ttyUSB1"), ("path", "/dev/ttyS0"),
                                 ("vid", "1A86"), ("vid", "1a8"), ("pid", 7523),
                                 ("serial_number", None), ("serial_number", "line\nbreak"),
                                 ("serial_number", " padded"), ("syspath", "/sys/class/tty/ttyUSB0/device"),
                                 ("syspath", "/sys/devices/../ttyUSB0"), ("syspath", "/sys/devices//ttyUSB0"),
                                 ("syspath", "/sys/devices/ttyUSB0/")):
                changed = copy.deepcopy(fixture_motion_review)
                changed["serial_device"][field] = value
                with self.assertRaises(ValueError):
                    validate_review_data(changed)
            for field in fixture_motion_review["serial_device"]:
                changed = copy.deepcopy(fixture_motion_review)
                del changed["serial_device"][field]
                with self.assertRaises(ValueError):
                    validate_review_data(changed)
            validate_review_data({**fixture_motion_review,
                                  "serial_device": {**fixture_motion_review["serial_device"], "serial_number": "known-serial-1"}})

        def test_scope_capabilities_cannot_cross_or_invent_runtime_serial_proof(self):
            for review, expected in ((None, "strict_storage_monitor"),
                                     (fixture_review, "ram_model_web_only"),
                                     (fixture_motion_review, "ram_motion_calibration_only")):
                with tempfile.TemporaryDirectory() as temporary:
                    recorder = Recorder(Path(temporary), review)
                    try:
                        self.assertEqual(recorder.state["capability_scope"], expected)
                        self.assertFalse(recorder.state["safe_for_ram_model_web"])
                        self.assertFalse(recorder.state["safe_for_ram_motion_calibration"])
                        for capability, authorized_scope in (("safe_for_ram_model_web", "ram_model_web_only"),
                                                             ("safe_for_ram_motion_calibration", "ram_motion_calibration_only")):
                            if expected == authorized_scope:
                                recorder.status(**{capability: True})
                            else:
                                with self.assertRaises(ValueError):
                                    recorder.status(**{capability: True})
                        if review is not None:
                            with self.assertRaises(ValueError):
                                recorder.status(safe=True)
                        with self.assertRaises(ValueError):
                            recorder.status(capability_scope="different_scope")
                        if expected == "ram_motion_calibration_only":
                            self.assertTrue(recorder.state["physical_support_confirmed"])
                            self.assertFalse(recorder.state["servo_arm_authorized"])
                            self.assertEqual(recorder.state["serial_device"], fixture_motion_review["serial_device"])
                            self.assertTrue(recorder.state["requires_serial_identity_proof"])
                            self.assertFalse(recorder.state["serial_identity_runtime_verified_by_guard"])
                            for updates in ({"servo_arm_authorized": True},
                                            {"serial_identity_runtime_verified_by_guard": True},
                                            {"requires_serial_identity_proof": False},
                                            {"serial_device": {**fixture_motion_review["serial_device"], "path": "/dev/ttyUSB1"}}):
                                with self.assertRaises(ValueError):
                                    recorder.status(**updates)
                        recorder.status(safe=False, safe_for_ram_model_web=False,
                                        safe_for_ram_motion_calibration=False, state="stopped")
                        self.assertFalse(recorder.state["safe_for_ram_model_web"])
                        self.assertFalse(recorder.state["safe_for_ram_motion_calibration"])
                    finally:
                        recorder.close()

        def test_review_is_narrow_and_private(self):
            import copy
            validate_review_data(fixture_review)
            for field, value in (("scope", "anything"), ("expected_cid", "invalid"),
                                 ("expected_root", {**fixture_review["expected_root"], "device": "179:3"}),
                                 ("required_unmounted_device", "179:2"),
                                 ("allowed_historical_lines", historical + ["mmc1: error -5"]),
                                 ("allowed_historical_lines", [historical[0], historical[1], "[ 1.0] mmc1: error -5"]),
                                 ("generic_storage_writes_authorized_by_this_review", True)):
                changed = copy.deepcopy(fixture_review)
                changed[field] = value
                with self.assertRaises((ValueError, TypeError)):
                    validate_review_data(changed)
            with tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / "review.json"
                path.write_text(json.dumps(fixture_review))
                path.chmod(0o600)
                self.assertEqual(load_review_file(str(path)), fixture_review)
                self.assertEqual(load_review_file(str(path), return_digest=True)[1], hashlib.sha256(path.read_bytes()).hexdigest())
                path.chmod(0o644)
                with self.assertRaises(ValueError):
                    load_review_file(str(path))
                link = Path(temporary) / "link.json"
                link.symlink_to(path)
                with self.assertRaises(OSError):
                    load_review_file(str(link))

        def test_reviewed_kernel_is_complete_and_exact(self):
            validate_reviewed_kernel(fixture_review, kernel_text)
            for changed in (kernel_text + "[ 22.000000] mmc1: error -5\n", kernel_text.replace("ready", "changed"),
                            kernel_text.replace(historical[0], historical[0] + " error")):
                with self.assertRaises(RuntimeError):
                    validate_reviewed_kernel(fixture_review, changed)
            bad_review = {**fixture_review, "allowed_historical_lines": list(reversed(historical))}
            with self.assertRaises(RuntimeError):
                validate_reviewed_kernel(bad_review, kernel_text)

        def test_ram_identity_mount_and_swap_gate(self):
            validate_ram_snapshot(fixture_review, health_fields["boot_id"], root, health_fields["card_cid"],
                                  health_fields["mounted_devices"], [], health_fields["root_block_device"])
            for change in ({"boot_id": str(uuid.uuid4())}, {"card_cid": "0" * 32},
                           {"root": {**root, "device": "179:3"}}, {"root": {**root, "options": ["ro"]}},
                           {"mounted_devices": ["179:2", "179:1"]}, {"swap_entries": ["/swapfile"]},
                           {"root_block_device": {"path": "/dev/mmcblk1p2", "device": "179:3", "is_block": True}},
                           {"root_block_device": None}):
                fields = {**health_fields, **change}
                with self.assertRaises(RuntimeError):
                    validate_ram_snapshot(fixture_review, fields["boot_id"], fields["root"], fields["card_cid"],
                                          fields["mounted_devices"], fields["swap_entries"], fields["root_block_device"])
            self.assertEqual(parse_swap_entries("Filename\tType\tSize\tUsed\tPriority\n"), [])
            self.assertEqual(parse_swap_entries("Filename Type Size Used Priority\n/swapfile file 100 0 -2\n"), ["/swapfile"])
            with self.assertRaises(ValueError):
                parse_swap_entries("")

        def test_actual_root_alias_requires_exact_review_and_real_block_node(self):
            from types import SimpleNamespace
            from unittest.mock import patch
            alias_root = parse_root_mount("21 1 179:2 / / rw,relatime shared:1 - ext4 /dev/root rw\n")
            alias_review = {**fixture_review, "expected_root": {**fixture_review["expected_root"], "source": "/dev/root"}}
            validate_review_data(alias_review)
            # Linux device-number decoding is mocked on the Mac; mode checks are real.
            with patch.object(os, "major", return_value=179), patch.object(os, "minor", return_value=2):
                block = root_block_identity(SimpleNamespace(st_mode=stat.S_IFBLK | 0o660, st_rdev=123))
            self.assertEqual(block, health_fields["root_block_device"])
            validate_ram_snapshot(alias_review, health_fields["boot_id"], alias_root, health_fields["card_cid"],
                                  health_fields["mounted_devices"], [], block)
            for review, actual in ((fixture_review, alias_root), (alias_review, root)):
                with self.assertRaises(RuntimeError):
                    validate_ram_snapshot(review, health_fields["boot_id"], actual, health_fields["card_cid"],
                                          health_fields["mounted_devices"], [], block)
            for source in ("/dev/mmcblk0p2", "/dev/disk/by-uuid/anything", "/dev/root2"):
                with self.assertRaises(ValueError):
                    validate_review_data({**fixture_review, "expected_root": {**fixture_review["expected_root"], "source": source}})
            for mode, major, minor in ((stat.S_IFCHR, 179, 2), (stat.S_IFREG, 0, 0),
                                      (stat.S_IFBLK, 179, 3), (stat.S_IFBLK, 180, 2)):
                with patch.object(os, "major", return_value=major), patch.object(os, "minor", return_value=minor):
                    with self.assertRaises(RuntimeError):
                        root_block_identity(SimpleNamespace(st_mode=mode | 0o660, st_rdev=123))

        def test_review_only_applies_to_one_ordered_replay(self):
            replay = BaselineReplay(kernel_text.splitlines(), fixture_review)
            for line in kernel_text.splitlines():
                marked = replay.accept(line)
                check_log_event({"line": line, "phase": "initial_follower_replay", "reviewed": marked},
                                fixture_review, True, False)
            self.assertTrue(replay.done)
            self.assertFalse(replay.accept(historical[0]))
            for event in ({"line": historical[0], "phase": "follow", "reviewed": False},
                          {"line": historical[0], "phase": "initial_follower_replay", "reviewed": True},
                          {"line": historical[0].replace("7.527571", "27.527571"), "phase": "baseline", "reviewed": True}):
                with self.assertRaises(RuntimeError):
                    check_log_event(event, fixture_review, True, True)
            with self.assertRaises(RuntimeError):
                check_log_event({"line": historical[0], "phase": "baseline", "reviewed": True},
                                fixture_review, False, False)
            with self.assertRaises(RuntimeError):
                BaselineReplay(kernel_text.splitlines(), fixture_review).accept(historical[0])

        def test_actual_saved_baseline_passes_only_ram_review(self):
            directory = Path(__file__).resolve().parents[2] / "output/x5_recovery/new-board-20261003T114727Z"
            if not (directory / "ram-only-reviewed-baseline.json").exists():
                self.skipTest("Saved local incident fixture is not present")
            review = load_review_file(str(directory / "ram-only-reviewed-baseline.json"))
            text = (directory / "sunrise-kernel-baseline.log").read_text()
            lines = validate_reviewed_kernel(review, text)
            replay = BaselineReplay(lines, review)
            marked_count = 0
            for line in lines:
                if line.strip():
                    marked = replay.accept(line)
                    marked_count += int(marked)
                    check_log_event({"line": line, "phase": "initial_follower_replay", "reviewed": marked}, review, True, False)
            self.assertTrue(replay.done)
            self.assertEqual(marked_count, 3)
            with self.assertRaises(RuntimeError):
                check_log_event({"line": review["allowed_historical_lines"][2], "phase": "follow"}, review, True, True)

        def test_raw_follow_normalization_and_fragmented_replay(self):
            sample = "first\r\nsecond\rthird\n\r\n雪\rfinal\n".encode("utf-8")
            directory = Path(__file__).resolve().parents[2] / "output/x5_recovery/new-board-20261003T114727Z"
            sources = [(sample, None)]
            raw_path = directory / "current-kernel-raw.log"
            if raw_path.exists():
                review = load_review_file(str(directory / "ram-only-reviewed-baseline.json"))
                raw = raw_path.read_bytes()
                self.assertGreater(raw.count(b"\r"), 0)
                self.assertEqual(normalized_kernel_hash(raw.decode("utf-8")), review["kernel_log_sha256"])
                sources.append((raw, review))
            for raw, review in sources:
                expected = [line for line in raw.decode("utf-8").splitlines() if line.strip()]
                for width in (1, 7, 64, 4096, 65536):
                    buffer = b""
                    result = []
                    for offset in range(0, len(raw), width):
                        buffer += raw[offset:offset + width]
                        while b"\n" in buffer:
                            physical, buffer = buffer.split(b"\n", 1)
                            result.extend(logical_follow_lines(physical))
                    self.assertEqual(buffer, b"")
                    self.assertEqual(result, expected)
                    if review is not None:
                        replay = BaselineReplay(raw.decode("utf-8").splitlines(), review)
                        marked = [replay.accept(line) for line in result]
                        self.assertTrue(replay.done)
                        self.assertEqual(sum(marked), 3)

        def test_host_ram_capabilities_are_separate_and_faults_revoke_both(self):
            import contextlib
            import io
            from unittest.mock import patch
            for scoped_review in (fixture_review, fixture_motion_review):
                for failure in ({"event": "log", "line": historical[0], "phase": "follow", "reviewed": False},
                                {"event": "heartbeat", **health_fields, "mounted_devices": ["179:2", "179:1"]},
                                {"event": "heartbeat", **health_fields, "swap_entries": ["/swapfile"]},
                                {"event": "heartbeat", **health_fields, "card_cid": "0" * 32},
                                {"event": "heartbeat", **health_fields, "root_block_device": {"path": "/dev/mmcblk1p2", "device": "179:3", "is_block": True}}):
                    with tempfile.TemporaryDirectory() as temporary:
                        review_path = Path(temporary) / "review.json"
                        review_path.write_text(json.dumps(scoped_review))
                        review_path.chmod(0o600)
                        proof = {"event": "baseline_reviewed", **health_fields,
                                 "kernel_log_sha256": scoped_review["kernel_log_sha256"],
                                 "allowed_historical_lines": historical}
                        healthy = {"event": "healthy", **health_fields, "baseline_lines": 5, "replay_complete": True}
                        payload = "".join(json.dumps(event) + "\n" for event in (proof, healthy, failure)).encode()
                        output_read, output_write = os.pipe()
                        error_read, error_write = os.pipe()
                        os.write(output_write, payload)
                        os.close(output_write)

                        class FakeProcess:
                            def __init__(self):
                                self.stdin = io.BytesIO()
                                self.stdout = os.fdopen(output_read, "rb")
                                self.stderr = os.fdopen(error_read, "rb")
                                self.returncode = None
                            def poll(self):
                                return self.returncode
                            def terminate(self):
                                self.returncode = -15
                                os.close(error_write)
                            def wait(self, timeout=None):
                                return self.returncode
                            def kill(self):
                                self.terminate()

                        captured = []
                        actual_status = Recorder.status
                        def record_status(recorder, **updates):
                            actual_status(recorder, **updates)
                            captured.append(dict(recorder.state))
                        args = argparse.Namespace(target="unused@unused", control_path="/unused", known_hosts="/unused",
                                                  output_dir=str(Path(temporary) / "monitor"), reviewed_baseline=str(review_path))
                        with patch(__name__ + ".validate_ssh"), patch(__name__ + ".subprocess.Popen", return_value=FakeProcess()), \
                                patch.object(Recorder, "status", record_status), contextlib.redirect_stdout(io.StringIO()), \
                                contextlib.redirect_stderr(io.StringIO()):
                            self.assertEqual(monitor(args), 2)
                        capability = "safe_for_ram_model_web" if scoped_review["scope"] == "ram_model_web_only" else "safe_for_ram_motion_calibration"
                        other = "safe_for_ram_motion_calibration" if capability == "safe_for_ram_model_web" else "safe_for_ram_model_web"
                        grants = [state for state in captured if state[capability]]
                        self.assertTrue(grants)
                        self.assertTrue(all(state["kernel_log_sha256"] == scoped_review["kernel_log_sha256"]
                                            and state["reviewed_baseline_sha256"] == hashlib.sha256(review_path.read_bytes()).hexdigest()
                                            for state in grants))
                        self.assertTrue(all(state["safe"] is False and state["capability_scope"] == scoped_review["scope"] for state in captured))
                        self.assertTrue(all(state[other] is False for state in captured))
                        self.assertFalse(captured[-1]["safe_for_ram_model_web"])
                        self.assertFalse(captured[-1]["safe_for_ram_motion_calibration"])
                        if scoped_review["scope"] == "ram_motion_calibration_only":
                            self.assertTrue(all(state["serial_device"] == scoped_review["serial_device"]
                                                and state["servo_arm_authorized"] is False
                                                and state["serial_identity_runtime_verified_by_guard"] is False
                                                for state in captured))

        def test_private_atomic_status(self):
            with tempfile.TemporaryDirectory() as temporary:
                recorder = Recorder(Path(temporary))
                recorder.status(safe=True, state="monitoring")
                self.assertTrue(json.loads(recorder.status_path.read_text())["safe"])
                recorder.status(safe=False, state="stopped", reason="mock disconnect")
                self.assertFalse(json.loads(recorder.status_path.read_text())["safe"])
                for name in ("status.json", "events.jsonl", "kernel.log"):
                    self.assertEqual((Path(temporary) / name).stat().st_mode & 0o777, 0o600)
                recorder.close()

        def test_health_events(self):
            boot = "7f8fb795-0e97-46df-b5f5-e1b5be5d2ed7"
            root = parse_root_mount("24 1 179:2 / / rw - ext4 /dev/mmcblk1p2 rw\n")
            event = {"event": "healthy", "boot_id": boot, "root": root, "baseline_lines": 2}
            identity = check_health_event(event, None, None)[1]
            heartbeat = {**event, "event": "heartbeat"}
            self.assertEqual(check_health_event(heartbeat, boot, identity), (boot, identity))
            for invalid, expected_boot, expected_mount in (
                    ({**event, "baseline_lines": 0}, None, None),
                    (event, boot, identity), (heartbeat, None, None),
                    ({**heartbeat, "boot_id": str(uuid.uuid4())}, boot, identity),
                    ({**heartbeat, "root": {**root, "options": ["ro"]}}, boot, identity),
                    ({**heartbeat, "root": {**root, "source": "/dev/mmcblk0p2"}}, boot, identity)):
                with self.assertRaises((ValueError, RuntimeError)):
                    check_health_event(invalid, expected_boot, expected_mount)

    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(GuardTests))
    return 0 if result.wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target")
    parser.add_argument("--control-path")
    parser.add_argument("--known-hosts")
    parser.add_argument("--output-dir")
    parser.add_argument("--reviewed-baseline", help="Private local review for one separate RAM-only capability; never grants generic storage or servo motion safety")
    parser.add_argument("--self-test", action="store_true", help="Only mocked/local tests; no SSH or hardware access")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not all((args.target, args.control_path, args.known_hosts, args.output_dir)):
        parser.error("--target, --control-path, --known-hosts and --output-dir are required")
    def interrupted(signum, frame):
        raise KeyboardInterrupt("Monitor interrupted by signal %s; stop all board operations" % signum)
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        return monitor(args)
    except Exception as error:
        print("STOP before connection: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
