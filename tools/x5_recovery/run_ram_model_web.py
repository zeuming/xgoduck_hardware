#!/usr/bin/env python3
"""Supervise the reviewed model-only X5 web page in temporary RAM.

This is not a generic deployment tool. It accepts one pinned, reviewed archive,
reuses an authenticated SSH socket and requires the RAM-only storage guard.
It never opens a serial port, repairs storage, installs with pip, or retries.
All persistent output belongs to the Mac. The X5 OS may still write its logs.
"""

import argparse
import hashlib
import inspect
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import selectors
import shlex
import signal
import socket
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid
import zipfile


APPROVED_ARCHIVE_SHA256 = "0d71cf6b66933fd21395bb709b3eaa3460f97a2e1306f2d0eee39f14bac4789f"
APPROVED_ARCHIVE_BYTES = 39198438
ALLOWED_MEMBERS = {
    "policy/web_app.py", "policy/infer.py", "policy/rl_core.py", "policy/index.html",
    "policy/models/manifest.json", "policy/assets/zero-reference.png",
    "policy/assets/positive-direction.png", "policy/models/xgoduck_getup.onnx",
    "policy/models/xgoduck_pick.onnx", "policy/models/xgoduck_roulade.onnx",
    "policy/models/xgoduck_walk.onnx",
    "wheels/coloredlogs-15.0.1-py2.py3-none-any.whl",
    "wheels/flatbuffers-25.12.19-py2.py3-none-any.whl",
    "wheels/humanfriendly-10.0-py2.py3-none-any.whl",
    "wheels/mpmath-1.3.0-py3-none-any.whl",
    "wheels/numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl",
    "wheels/onnxruntime-1.22.1-cp310-cp310-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl",
    "wheels/packaging-26.3-py3-none-any.whl",
    "wheels/protobuf-7.36.2-cp310-abi3-manylinux2014_aarch64.whl",
    "wheels/pyserial-3.5-py2.py3-none-any.whl",
    "wheels/sympy-1.14.0-py3-none-any.whl",
}
MAX_EXPANDED_WHEEL_BYTES = 160 * 1024 * 1024
FRESHNESS_SECONDS = 4


def safe_relative_path(name):
    if not isinstance(name, str) or not name or "\\" in name or "\x00" in name:
        raise ValueError("Invalid archive member path")
    path = PurePosixPath(name)
    if path.is_absolute() or any(p in ("", ".", "..") for p in name.rstrip("/").split("/")):
        raise ValueError("Unsafe archive member path: " + repr(name))
    return name.rstrip("/")


def normalized_kernel_sha256(text):
    if not isinstance(text, str) or not text.splitlines():
        raise ValueError("Kernel log is unavailable or empty")
    return hashlib.sha256(("\n".join(text.splitlines()) + "\n").encode("utf-8")).hexdigest()


def validate_review(review):
    if review.get("scope") != "ram_model_web_only":
        raise ValueError("A reviewed RAM model-only capability is required")
    if str(uuid.UUID(review["expected_boot_id"])) != review["expected_boot_id"]:
        raise ValueError("Invalid reviewed boot ID")
    if not re.fullmatch(r"[0-9a-f]{32}", review["expected_cid"]):
        raise ValueError("Invalid reviewed card CID")
    root = review["expected_root"]
    if (set(root) != {"device", "source", "filesystem"}
            or root["device"] != "179:2" or root["filesystem"] != "ext4"
            or root["source"] not in ("/dev/mmcblk1p2", "/dev/root")):
        raise ValueError("Unexpected reviewed root identity")
    if review.get("required_unmounted_device") != "179:1":
        raise ValueError("The CONFIG partition must remain unmounted")
    if not re.fullmatch(r"[0-9a-f]{64}", review["kernel_log_sha256"]):
        raise ValueError("Invalid reviewed kernel hash")


def verify_root_block_node(info):
    if (not stat.S_ISBLK(info.st_mode) or os.major(info.st_rdev) != 179
            or os.minor(info.st_rdev) != 2):
        raise RuntimeError("Canonical root block node is not the reviewed 179:2 device")


def validate_manifest(manifest):
    archive = manifest["archive"]
    if (archive.get("sha256") != APPROVED_ARCHIVE_SHA256
            or archive.get("size_bytes") != APPROVED_ARCHIVE_BYTES
            or archive.get("file_count") != len(ALLOWED_MEMBERS)):
        raise ValueError("Only the pinned, reviewed minimal RAM archive is supported")
    files = manifest["files"]
    names = [safe_relative_path(f["archive_path"]) for f in files]
    if len(names) != len(ALLOWED_MEMBERS) or set(names) != ALLOWED_MEMBERS:
        raise ValueError("Archive manifest must contain exactly the 21 approved regular files")
    for entry in files:
        if (type(entry["size_bytes"]) is not int or entry["size_bytes"] <= 0
                or not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"])):
            raise ValueError("Invalid manifest member size/hash")
    return {entry["archive_path"]: {"size_bytes": entry["size_bytes"], "sha256": entry["sha256"]}
            for entry in files}


def inspect_tar(data, files):
    result = {}
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for member in archive:
            name = safe_relative_path(member.name)
            if name not in files or name in result or not member.isfile():
                raise ValueError("Unexpected, repeated, or non-regular tar member: " + name)
            expected = files[name]
            if member.size != expected["size_bytes"]:
                raise ValueError("Tar member size mismatch: " + name)
            stream = archive.extractfile(member)
            content = stream.read(member.size + 1)
            if len(content) != member.size or hashlib.sha256(content).hexdigest() != expected["sha256"]:
                raise ValueError("Tar member hash mismatch: " + name)
            result[name] = content
    if set(result) != set(files):
        raise ValueError("Missing tar members")
    return result


def inspect_wheels(members):
    total = 0
    paths = set()
    for name, content in members.items():
        if not name.startswith("wheels/"):
            continue
        with zipfile.ZipFile(io.BytesIO(content)) as archive:
            for item in archive.infolist():
                path = safe_relative_path(item.filename)
                mode = item.external_attr >> 16
                if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
                    raise ValueError("Wheel links/special files are forbidden")
                if item.is_dir():
                    continue
                if path in paths:
                    raise ValueError("Wheels overwrite a shared member: " + path)
                paths.add(path)
                total += item.file_size
                if item.file_size > 80 * 1024 * 1024 or total > MAX_EXPANDED_WHEEL_BYTES:
                    raise ValueError("Wheel expansion exceeds the reviewed RAM budget")
    return total


def read_regular_json(path, private=False):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError("An absolute regular local JSON file is required")
    info = path.stat()
    if private and (info.st_uid != os.geteuid() or info.st_mode & 0o077):
        raise ValueError("Guard status must be user-owned and private (0600)")
    return json.loads(path.read_text(encoding="utf-8"))


def check_guard(path, review, now=None):
    status = read_regular_json(path, private=True)
    if (status.get("safe_for_ram_model_web") is not True
            or status.get("capability_scope") != "ram_model_web_only"
            or status.get("safe") is not False):
        raise RuntimeError("Storage guard does not authorize this RAM-only operation")
    pid = status.get("local_pid")
    if type(pid) is not int or pid <= 1 or pid == os.getpid():
        raise RuntimeError("Invalid storage guard PID")
    os.kill(pid, 0)
    heartbeat = status.get("last_heartbeat_monotonic")
    if isinstance(heartbeat, bool) or not isinstance(heartbeat, (int, float)):
        raise RuntimeError("Missing storage guard heartbeat")
    age = (time.monotonic() if now is None else now) - heartbeat
    if not 0 <= age <= FRESHNESS_SECONDS:
        raise RuntimeError("Storage guard heartbeat is stale")
    if (status.get("boot_id") != review["expected_boot_id"]
            or status.get("card_cid") != review["expected_cid"]):
        raise RuntimeError("Storage guard boot/card identity differs from the review")
    if status.get("kernel_log_sha256") != review["kernel_log_sha256"]:
        raise RuntimeError("Storage guard kernel review differs from this RAM-only review")
    root = status.get("root", {})
    if {k: root.get(k) for k in ("device", "source", "filesystem")} != review["expected_root"]:
        raise RuntimeError("Storage guard root identity differs from the review")
    return status


def ssh_base(connection):
    if not re.fullmatch(r"sunrise@[A-Za-z0-9_.:-]+", connection["target"]):
        raise ValueError("Only the sunrise user on the identified board is supported")
    control = Path(connection["control_path"])
    known = Path(connection["known_hosts"])
    if (not control.is_absolute() or control.is_symlink()
            or not stat.S_ISSOCK(control.stat().st_mode)
            or control.stat().st_uid != os.geteuid()):
        raise ValueError("An existing, user-owned SSH control socket is required")
    if not known.is_absolute() or known.is_symlink() or not known.is_file():
        raise ValueError("A regular private known-hosts file is required")
    return ["/usr/bin/ssh", "-T", "-F", "/dev/null", "-S", str(control),
            "-o", "ProxyCommand=/usr/bin/false", "-o", "ControlMaster=no",
            "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "UserKnownHostsFile=" + str(known),
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=1"]


CHILD_PARENT_GUARD = r'''
import ctypes, os, signal
expected_parent = int(os.environ["XGODUCK_SUPERVISOR_PID"])
if os.getppid() != expected_parent:
    raise RuntimeError("RAM supervisor disappeared before the child startup")
libc = ctypes.CDLL(None, use_errno=True)
libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
libc.prctl.restype = ctypes.c_int
if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), "Cannot arm the model child parent-death signal")
if os.getppid() != expected_parent:
    raise RuntimeError("RAM supervisor disappeared before the child startup")
'''


SERVER_WRAPPER = CHILD_PARENT_GUARD + r'''
import os, signal, sys
sys.path.insert(0, os.environ["XGODUCK_POLICY_DIR"])
def no_device_output(event, args):
    if event == "open" and args and isinstance(args[0], (str, bytes)):
        path = os.fsdecode(args[0])
        if path.startswith(("/dev/tty", "/dev/serial", "/dev/bus/usb")):
            raise PermissionError("Serial/device access is disabled for this model-only page")
        mode = args[1] if len(args) > 1 else None
        flags = args[2] if len(args) > 2 else 0
        writes = (isinstance(mode, str) and any(x in mode for x in "wax+")) or (isinstance(flags, int) and flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND))
        if writes and not os.path.abspath(path).startswith(os.environ["XGODUCK_RAM_ROOT"] + "/"):
            raise PermissionError("Model-only file writes are restricted to this temporary RAM directory")
sys.addaudithook(no_device_output)
import web_app
workbench = web_app.Workbench()
if workbench.motion is not None or len(workbench.sessions) != 4:
    raise RuntimeError("Model-only preflight failed: " + repr(workbench.errors))
original = web_app.make_handler(workbench)
class Handler(original):
    def valid_host(self):
        return self.headers.get("Host") in (
            "127.0.0.1:" + os.environ["XGODUCK_LOCAL_PORT"],
            "localhost:" + os.environ["XGODUCK_LOCAL_PORT"],
            "127.0.0.1:" + os.environ["XGODUCK_REMOTE_PORT"],
            "localhost:" + os.environ["XGODUCK_REMOTE_PORT"])
server = web_app.ThreadingHTTPServer(("127.0.0.1", int(os.environ["XGODUCK_REMOTE_PORT"])), Handler)
print("MODEL_WEB_READY", flush=True)
def terminate(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, terminate)
signal.signal(signal.SIGHUP, terminate)
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
finally:
    server.server_close()
'''


REMOTE_MAIN = r'''
import platform, pwd, shutil, socket, threading
CONFIG = __CONFIG__
stop = threading.Event()
received = threading.Event()
shared = {"last_input": time.monotonic(), "archive": None, "error": None, "process": None,
          "child_stop_confirmed": True, "stopped_process": None,
          "ram_mount_identity": None, "ram_inode": None}
lock = threading.Lock()
child_stop_lock = threading.Lock()
output_lock = threading.Lock()
ram_directory = None
child_readers = []

def emit(event, **fields):
    with output_lock:
        print(json.dumps({"event": event, **fields}, ensure_ascii=False), flush=True)

def stop_child():
    # PID is always our own start_new_session child, never a discovered PID.
    with child_stop_lock:
        with lock:
            child = shared["process"]
        if child is None:
            return True
        if shared["stopped_process"] is child:
            return True
        try:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(child.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    child.wait(timeout=1)
            # Confirm the owned group disappeared, including descendants if
            # the group leader exited before a request thread completed.
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                shared["child_stop_confirmed"] = True
                shared["stopped_process"] = child
                return True
            os.killpg(child.pid, signal.SIGKILL)
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    shared["child_stop_confirmed"] = True
                    shared["stopped_process"] = child
                    return True
                time.sleep(0.02)
        except BaseException as error:
            shared["error"] = (shared["error"] or "Stopping RAM child failed") + "; " + str(error)
        shared["child_stop_confirmed"] = False
        return False

def fail(reason):
    with lock:
        if shared["error"] is None:
            shared["error"] = str(reason)
    stop.set()
    received.set()
    stop_child()

def assert_live():
    if stop.is_set() or shared["error"] is not None:
        raise RuntimeError(shared["error"] or "RAM operation stopped")

def parsed_mounts():
    entries = []
    with open("/proc/self/mountinfo", "r") as stream:
        for line in stream:
            f = line.split()
            if len(f) < 10:
                raise RuntimeError("Malformed mountinfo")
            sep = f.index("-", 6)
            entries.append({"mount_id": f[0], "device": f[2], "target": f[4], "options": f[5].split(","),
                            "filesystem": f[sep+1], "source": f[sep+2],
                            "super_options": f[sep+3].split(",")})
    return entries

def snapshot():
    review = CONFIG["review"]
    mounts = parsed_mounts()
    roots = [x for x in mounts if x["target"] == "/"]
    ram = [x for x in mounts if x["target"] == "/dev/shm"]
    if len(roots) != 1 or len(ram) != 1:
        raise RuntimeError("Root/RAM mount is not unique")
    root, tmp = roots[0], ram[0]
    if {k: root[k] for k in ("device", "source", "filesystem")} != review["expected_root"]:
        raise RuntimeError("Root mount identity changed")
    # /dev/root is a kernel mount alias and need not be a filesystem node.
    # The reviewed raw mount source must match exactly, and its canonical
    # device must independently remain the expected physical block node.
    verify_root_block_node(os.stat("/dev/mmcblk1p2", follow_symlinks=False))
    if "rw" not in root["options"] or "ro" in root["options"] or "ro" in root["super_options"]:
        raise RuntimeError("Root filesystem became read-only")
    if (tmp["filesystem"] != "tmpfs" or "rw" not in tmp["options"]
            or any(x in tmp["options"] + tmp["super_options"] for x in ("ro", "noexec"))):
        raise RuntimeError("RAM mount is not writable, executable tmpfs")
    shm_info = os.stat("/dev/shm")
    shm_identity = (tmp["mount_id"], tmp["device"], tmp["source"], tmp["filesystem"],
                    shm_info.st_dev, shm_info.st_ino)
    if shared["ram_mount_identity"] is None:
        shared["ram_mount_identity"] = shm_identity
    elif shared["ram_mount_identity"] != shm_identity:
        raise RuntimeError("RAM mount identity changed")
    if any(x["device"] == review["required_unmounted_device"] for x in mounts):
        raise RuntimeError("CONFIG must remain unmounted")
    with open("/proc/swaps", "r") as stream:
        if len(stream.read().splitlines()) != 1:
            raise RuntimeError("Swap is active; RAM-only files could reach storage")
    with open("/proc/sys/kernel/random/boot_id", "r") as stream:
        if stream.read().strip() != review["expected_boot_id"]:
            raise RuntimeError("Boot ID changed")
    with open("/sys/block/mmcblk1/device/cid", "r") as stream:
        if stream.read().strip() != review["expected_cid"]:
            raise RuntimeError("Card CID changed")
    kernel = subprocess.run(["dmesg", "--color=never"], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=1.5, check=False)
    if kernel.returncode or kernel.stderr:
        raise RuntimeError("Kernel log cannot be read reliably")
    if normalized_kernel_sha256(kernel.stdout.decode("utf-8", "strict")) != review["kernel_log_sha256"]:
        raise RuntimeError("Kernel log changed since the review")
    if ram_directory is not None:
        info = os.lstat(ram_directory)
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077
                or (info.st_dev, info.st_ino) != shared["ram_inode"]):
            raise RuntimeError("Our RAM directory identity/permissions changed")
        if any(x["target"].startswith(ram_directory + "/") or x["target"] == ram_directory for x in mounts):
            raise RuntimeError("Unexpected mount inside our RAM directory")
    return tmp

def watch_health():
    while not stop.wait(0.25):
        if shared["error"] is not None:
            fail(shared["error"])
            return
        if time.monotonic() - shared["last_input"] > 4:
            fail("Mac heartbeat/data timed out")
            return
        if time.monotonic() >= watch_health.next_check:
            try:
                snapshot()
            except BaseException as error:
                fail(error)
                return
            watch_health.next_check = time.monotonic() + 1

def read_input():
    try:
        remaining = CONFIG["archive_size"]
        data = bytearray()
        while remaining:
            part = os.read(0, min(65536, remaining))
            if not part:
                raise RuntimeError("Mac SSH input closed during archive transfer")
            shared["last_input"] = time.monotonic()
            data.extend(part)
            remaining -= len(part)
            assert_live()
        if hashlib.sha256(data).hexdigest() != CONFIG["archive_sha256"]:
            raise RuntimeError("Transferred archive hash mismatch")
        shared["archive"] = data
        received.set()
        buffer = b""
        while not stop.is_set():
            part = os.read(0, 1024)
            if not part:
                raise RuntimeError("Mac SSH input closed")
            buffer += part
            if len(buffer) > 128:
                raise RuntimeError("Malformed Mac heartbeat")
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line != b"HB":
                    raise RuntimeError("Unexpected Mac heartbeat")
                shared["last_input"] = time.monotonic()
    except BaseException as error:
        fail(error)

def child_pipe(stream, kind):
    try:
        while True:
            line = stream.readline(65537)
            if not line:
                return
            if len(line) > 65536:
                fail("Oversized model process log")
                return
            text = line.decode("utf-8", "replace").rstrip("\n")
            if kind == "stdout" and text == "MODEL_WEB_READY":
                emit("ready", local_port=CONFIG["local_port"], remote_port=CONFIG["remote_port"],
                     ram_directory=ram_directory, motor_output=False)
            else:
                emit("child_log", stream=kind, line=text)
    except BaseException as error:
        fail(error)

def spawn_child(command, env):
    assert_live()
    with lock:
        if stop.is_set():
            raise RuntimeError(shared["error"] or "RAM operation stopped")
        child = subprocess.Popen(command, cwd=ram_directory + "/policy", env=env,
                                 stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, start_new_session=True)
        shared["process"] = child
        shared["child_stop_confirmed"] = False
        shared["stopped_process"] = None
    readers = []
    for stream, name in ((child.stdout, "stdout"), (child.stderr, "stderr")):
        reader = threading.Thread(target=child_pipe, args=(stream, name), daemon=True)
        readers.append(reader)
        reader.start()
    return child, readers

def main():
    global ram_directory
    readers = []
    try:
        if os.getuid() == 0 or pwd.getpwuid(os.getuid()).pw_name != "sunrise":
            raise RuntimeError("RAM runner must be the unprivileged sunrise user")
        if platform.machine() != "aarch64" or sys.version_info[:2] != (3, 10):
            raise RuntimeError("Archive requires aarch64 CPython 3.10")
        version = platform.libc_ver()
        if version[0] != "glibc" or tuple(int(x) for x in version[1].split(".")[:2]) < (2, 27):
            raise RuntimeError("Archive requires glibc 2.27 or newer")
        validate_review(CONFIG["review"])
        snapshot()
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", CONFIG["remote_port"]))
        shm = os.statvfs("/dev/shm")
        required = CONFIG["archive_size"] + CONFIG["member_bytes"] + CONFIG["wheel_bytes"] + 128*1024*1024
        with open("/proc/meminfo", "r") as stream:
            available = dict(line.split(":", 1) for line in stream)["MemAvailable"]
        if shm.f_bavail * shm.f_frsize < required or int(available.split()[0])*1024 < required + 256*1024*1024:
            raise RuntimeError("Insufficient free RAM/tmpfs for reviewed model bundle")
        # The archive itself remains in memory; all extracted files have a fresh
        # exclusive directory under the checked tmpfs, with no fixed shared path.
        ram_directory = tempfile.mkdtemp(prefix="xgoduck-model-", dir="/dev/shm")
        os.chmod(ram_directory, 0o700)
        created = os.lstat(ram_directory)
        shared["ram_inode"] = (created.st_dev, created.st_ino)
        for name in ("site", "tmp", "cache", "config", "state", "data"):
            os.mkdir(ram_directory + "/" + name, 0o700)
        watch_health.next_check = time.monotonic() + 1
        threading.Thread(target=watch_health, daemon=True).start()
        threading.Thread(target=read_input, daemon=True).start()
        emit("transfer_ready", archive_bytes=CONFIG["archive_size"], ram_directory=ram_directory)
        deadline = time.monotonic() + 120
        while not received.wait(0.1):
            assert_live()
            if time.monotonic() > deadline:
                raise RuntimeError("Archive transfer exceeded 120 seconds")
        assert_live()
        members = inspect_tar(shared["archive"], CONFIG["files"])
        if inspect_wheels(members) != CONFIG["wheel_bytes"]:
            raise RuntimeError("Expanded wheel budget differs from the local review")
        assert_live()
        for name, content in members.items():
            assert_live()
            target = ram_directory + "/" + name
            os.makedirs(os.path.dirname(target), mode=0o700, exist_ok=True)
            with open(target, "xb") as stream:
                stream.write(content)
            os.chmod(target, 0o600)
        shared["archive"] = None
        wheel_count = 0
        for name, content in members.items():
            if not name.startswith("wheels/"):
                continue
            with zipfile.ZipFile(io.BytesIO(content)) as archive:
                for item in archive.infolist():
                    assert_live()
                    relative = safe_relative_path(item.filename)
                    destination = ram_directory + "/site/" + relative
                    if item.is_dir():
                        os.makedirs(destination, mode=0o700, exist_ok=True)
                        continue
                    os.makedirs(os.path.dirname(destination), mode=0o700, exist_ok=True)
                    with archive.open(item) as source, open(destination, "xb") as target:
                        while True:
                            assert_live()
                            chunk = source.read(65536)
                            if not chunk:
                                break
                            target.write(chunk)
                    os.chmod(destination, 0o600)
            wheel_count += 1
        del members
        # This edit is intentionally limited to our temporary RAM HTML copy.
        html = ram_directory + "/policy/index.html"
        with open(html, "r", encoding="utf-8") as stream:
            content = stream.read()
        banner = '<aside style="padding:10px;background:#fff3cd;color:#493b06">当前是 X5 临时模型测试，重启后需重新开启。舵机控制尚未启用。</aside>'
        content, count = re.subn(r'(<body\b[^>]*>)', r'\1' + banner, content, count=1, flags=re.I)
        if count != 1:
            raise RuntimeError("Cannot place the RAM-only page notice")
        with open(html, "w", encoding="utf-8") as stream:
            stream.write(content)
        env = {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8",
               "HOME": os.environ.get("HOME", ""), "USER": "sunrise", "LOGNAME": "sunrise",
               "PYTHONDONTWRITEBYTECODE": "1", "PYTHONNOUSERSITE": "1",
               "PYTHONPATH": ram_directory + "/site", "TMPDIR": ram_directory + "/tmp",
               "XDG_CACHE_HOME": ram_directory + "/cache", "XDG_CONFIG_HOME": ram_directory + "/config",
               "XDG_STATE_HOME": ram_directory + "/state", "XDG_DATA_HOME": ram_directory + "/data",
               "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1",
               "XGODUCK_POLICY_DIR": ram_directory + "/policy",
               "XGODUCK_RAM_ROOT": ram_directory,
               "XGODUCK_SUPERVISOR_PID": str(os.getpid()),
               "XGODUCK_LOCAL_PORT": str(CONFIG["local_port"]),
               "XGODUCK_REMOTE_PORT": str(CONFIG["remote_port"])}
        snapshot()
        emit("unpacked", wheel_count=wheel_count, native_import_preflight=True)
        child, readers = spawn_child([sys.executable, "-B", "-S", "-u", "-c",
                                      CONFIG["child_parent_guard"] + "\nimport numpy,onnxruntime;print('Native RAM dependencies ready',flush=True)"], env)
        deadline = time.monotonic() + 30
        while child.poll() is None:
            assert_live()
            if time.monotonic() > deadline:
                raise RuntimeError("Native dependency import timed out")
            time.sleep(0.1)
        if child.returncode:
            raise RuntimeError("Native dependency import failed")
        for reader in readers:
            reader.join(timeout=1)
        snapshot()
        child, readers = spawn_child([sys.executable, "-B", "-S", "-u", "-c", CONFIG["server_wrapper"]], env)
        while child.poll() is None:
            assert_live()
            time.sleep(0.1)
        raise RuntimeError("Model web server exited (code %s)" % child.returncode)
    except BaseException as error:
        fail(error)
        try:
            emit("fatal", reason=shared["error"] or str(error))
        except BaseException:
            pass
        return 2
    finally:
        stop.set()
        child_stopped = stop_child()
        for reader in readers:
            reader.join(timeout=0.5)
        if ram_directory is not None:
            try:
                # Only remove the private directory created by this process.
                # Refuse cleanup if it was replaced or gained a nested mount.
                mounts = parsed_mounts()
                info = os.lstat(ram_directory)
                shm_now = os.stat("/dev/shm")
                if (stat.S_ISDIR(info.st_mode) and info.st_uid == os.getuid()
                        and (info.st_dev, info.st_ino) == shared["ram_inode"]
                        and shared["ram_mount_identity"][-2:] == (shm_now.st_dev, shm_now.st_ino)
                        and not any(x["target"] == ram_directory or x["target"].startswith(ram_directory + "/") for x in mounts)):
                    shutil.rmtree(ram_directory)
                    emit("cleaned", ram_directory=ram_directory, child_stopped=child_stopped)
            except BaseException:
                pass
        try:
            emit("stopped", child_stopped=child_stopped)
        except BaseException:
            pass

def interrupted(signum, frame):
    # A signal may interrupt Popen while shared lock is held. Do not acquire
    # locks, kill, or raise here; the main loop/watchdog performs teardown.
    shared["error"] = "Remote runner interrupted by signal %s" % signum
signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGHUP, interrupted)
sys.exit(main())
'''


def remote_source(config):
    functions = (safe_relative_path, normalized_kernel_sha256, validate_review, verify_root_block_node, inspect_tar, inspect_wheels)
    return ("import hashlib,io,json,os,re,signal,stat,subprocess,sys,tarfile,tempfile,time,uuid,zipfile\n"
            "from pathlib import PurePosixPath\nMAX_EXPANDED_WHEEL_BYTES=%d\n" % MAX_EXPANDED_WHEEL_BYTES
            + "\n".join(inspect.getsource(function) for function in functions)
            + REMOTE_MAIN.replace("__CONFIG__", repr(config), 1))


def tunnel_reaper(config):
    """Own only our multiplex forward; EOF also handles the parent being killed."""
    selector = selectors.DefaultSelector()
    selector.register(sys.stdin.fileno(), selectors.EVENT_READ)
    last = time.monotonic()
    buffer = b""
    try:
        while time.monotonic() - last <= 4:
            if selector.select(0.25):
                data = os.read(sys.stdin.fileno(), 1024)
                if not data:
                    break
                buffer += data
                if len(buffer) > 128:
                    break
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if line != b"HB":
                        return 2
                    last = time.monotonic()
    finally:
        try:
            result = subprocess.run(config["cancel_command"], capture_output=True, timeout=2)
            print(json.dumps({"event": "forward_cancel", "returncode": result.returncode,
                              "stderr": result.stderr.decode("utf-8", "replace")}), flush=True)
        except BaseException as error:
            print(json.dumps({"event": "forward_cancel_error", "reason": str(error)}), flush=True)
    return 0


class Recorder:
    def __init__(self, directory):
        directory = Path(directory)
        if not directory.is_absolute() or directory.is_symlink():
            raise ValueError("An absolute private output directory is required")
        directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        info = directory.stat()
        if info.st_uid != os.geteuid() or info.st_mode & 0o077 or any(directory.iterdir()):
            raise ValueError("Output directory must be empty, user-owned and mode 0700")
        self.directory = directory
        self.events = self.new_file("events.jsonl")
        self.reaper_log = self.new_file("tunnel-reaper.jsonl")
        self.report = {"state": "starting", "local_pid": os.getpid(), "motor_output": False,
                       "remote_stop_confirmed": False,
                       "capability_scope": "ram_model_web_only"}
        self.status()

    def new_file(self, name):
        fd = os.open(str(self.directory / name), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        return os.fdopen(fd, "w", encoding="utf-8", buffering=1)

    def status(self, **updates):
        self.report.update(updates)
        self.report["updated_monotonic"] = time.monotonic()
        fd, temporary = tempfile.mkstemp(prefix=".status-", dir=str(self.directory))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.report, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            os.replace(temporary, self.directory / "status.json")
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def event(self, event):
        self.events.write(json.dumps({"received_monotonic": time.monotonic(), **event}, ensure_ascii=False) + "\n")

    def close(self):
        self.events.close()
        self.reaper_log.close()


def supervise(args):
    connection = read_regular_json(args.connection_json)
    review = read_regular_json(args.reviewed_baseline)
    validate_review(review)
    manifest = read_regular_json(args.manifest)
    files = validate_manifest(manifest)
    bundle_path = Path(args.bundle)
    if not bundle_path.is_absolute() or bundle_path.is_symlink() or not bundle_path.is_file():
        raise ValueError("An absolute regular reviewed bundle is required")
    with bundle_path.open("rb") as stream:
        data = stream.read(APPROVED_ARCHIVE_BYTES + 1)
    if len(data) != APPROVED_ARCHIVE_BYTES or hashlib.sha256(data).hexdigest() != APPROVED_ARCHIVE_SHA256:
        raise ValueError("Local archive length/hash differs from the reviewed bundle")
    wheel_bytes = inspect_wheels(inspect_tar(data, files))
    base = ssh_base(connection)
    check_guard(args.guard_status, review)
    if not 1024 <= args.local_port <= 65535 or not 1024 <= args.remote_port <= 65535:
        raise ValueError("Ports must be between 1024 and 65535")
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", args.local_port))
    config = {"review": review, "files": files, "archive_size": len(data),
              "archive_sha256": APPROVED_ARCHIVE_SHA256, "wheel_bytes": wheel_bytes,
              "member_bytes": sum(f["size_bytes"] for f in files.values()),
              "local_port": args.local_port, "remote_port": args.remote_port,
              "server_wrapper": SERVER_WRAPPER, "child_parent_guard": CHILD_PARENT_GUARD}
    forward = "127.0.0.1:%d:127.0.0.1:%d" % (args.local_port, args.remote_port)
    cancel = base + ["-O", "cancel", "-L", forward, connection["target"]]
    command = (base + ["-o", "ExitOnForwardFailure=yes", "-L", forward, connection["target"],
                      shlex.join(["python3", "-B", "-S", "-u", "-c", remote_source(config)])])
    recorder = Recorder(args.output_dir)
    process = None
    reaper = None
    reason = None
    try:
        check_guard(args.guard_status, review)
        reaper_command = [sys.executable, "-B", str(Path(__file__).resolve()),
                          "--internal-tunnel-reaper", json.dumps({"cancel_command": cancel})]
        reaper = subprocess.Popen(reaper_command, stdin=subprocess.PIPE, stdout=recorder.reaper_log,
                                  stderr=recorder.reaper_log, close_fds=True)
        os.set_blocking(reaper.stdin.fileno(), False)
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for stream in (process.stdin, process.stdout, process.stderr):
            os.set_blocking(stream.fileno(), False)
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        stdout_buffer = b""
        offset = 0
        transfer_ready = False
        pending = b""
        next_hb = time.monotonic()
        started = time.monotonic()
        ready = False
        print("RAM model web: checking the live reviewed board; no serial or motor output.", flush=True)
        while True:
            check_guard(args.guard_status, review)
            if process.poll() is not None:
                raise RuntimeError("Supervised SSH channel exited (code %s)" % process.returncode)
            if reaper.poll() is not None:
                raise RuntimeError("Local tunnel cleanup watchdog exited")
            now = time.monotonic()
            if now >= next_hb:
                os.write(reaper.stdin.fileno(), b"HB\n")
                if offset == len(data):
                    pending += b"HB\n"
                next_hb = now + 1
            if transfer_ready and offset < len(data):
                try:
                    offset += os.write(process.stdin.fileno(), memoryview(data)[offset:offset+65536])
                    if offset == len(data):
                        recorder.event({"event": "transfer_complete", "archive_bytes": offset})
                        print("Verified archive transferred to temporary RAM; preparing model dependencies.", flush=True)
                        pending += b"HB\n"
                except BlockingIOError:
                    pass
            elif pending:
                try:
                    count = os.write(process.stdin.fileno(), pending)
                    pending = pending[count:]
                except BlockingIOError:
                    pass
                if len(pending) > 128:
                    raise RuntimeError("SSH heartbeat output is blocked")
            for key, _ in selector.select(0.01 if transfer_ready and offset < len(data) else 0.25):
                chunk = os.read(key.fileobj.fileno(), 65536)
                if not chunk:
                    raise RuntimeError("Supervised SSH stream closed")
                if key.data == "stderr":
                    recorder.event({"event": "ssh_stderr", "message": chunk.decode("utf-8", "replace")})
                    raise RuntimeError("SSH reported an error; stopping the RAM web service")
                stdout_buffer += chunk
                if len(stdout_buffer) > 1024*1024:
                    raise RuntimeError("Remote event buffer exceeds its limit")
                while b"\n" in stdout_buffer:
                    line, stdout_buffer = stdout_buffer.split(b"\n", 1)
                    event = json.loads(line.decode("utf-8", "strict"))
                    if not isinstance(event, dict):
                        raise RuntimeError("Malformed remote event")
                    kind = event.get("event")
                    if kind not in ("transfer_ready", "unpacked", "ready", "child_log", "fatal", "cleaned", "stopped"):
                        raise RuntimeError("Unknown remote event")
                    recorder.event(event)
                    if kind == "fatal":
                        raise RuntimeError(event.get("reason", "Remote RAM watchdog failed"))
                    if kind == "transfer_ready":
                        if transfer_ready or event.get("archive_bytes") != len(data):
                            raise RuntimeError("Invalid remote transfer handshake")
                        transfer_ready = True
                        recorder.status(state="transferring", ram_directory=event["ram_directory"])
                    elif kind == "ready":
                        if (ready or event.get("motor_output") is not False
                                or event.get("local_port") != args.local_port
                                or event.get("remote_port") != args.remote_port):
                            raise RuntimeError("Invalid model-only web readiness event")
                        ready = True
                        recorder.status(state="running", url="http://127.0.0.1:%d/" % args.local_port)
                        print("Model web ready: http://127.0.0.1:%d/ (motor output disabled)." % args.local_port, flush=True)
            if not ready and time.monotonic() - started > 180:
                raise RuntimeError("RAM web startup timed out")
    except (BaseException) as error:
        reason = str(error) or "RAM supervisor interrupted"
        recorder.status(state="stopping", reason=reason)
        recorder.event({"event": "local_stop", "reason": reason})
        print("STOP: " + reason, file=sys.stderr, flush=True)
        return 2
    finally:
        # Close the channel's stdin first: the remote input watcher stops its
        # child group independently, even if SIGTERM or the SSH reply is lost.
        if process is not None:
            try:
                process.stdin.close()
            except BaseException:
                pass
        if reaper is not None:
            try:
                reaper.stdin.close()
            except BaseException:
                pass
            try:
                reaper.wait(timeout=3)
            except subprocess.TimeoutExpired:
                reaper.kill()
                reaper.wait()
        if process is not None:
            # Keep draining the SSH channel during remote kill/cleanup so its
            # final acknowledgement cannot block behind an unread log pipe.
            cleanup_selector = selectors.DefaultSelector()
            for stream, kind in ((process.stdout, "stdout"), (process.stderr, "stderr")):
                cleanup_selector.register(stream, selectors.EVENT_READ, kind)
            cleanup_buffer = stdout_buffer if "stdout_buffer" in locals() else b""
            deadline = time.monotonic() + 4
            while cleanup_selector.get_map() and time.monotonic() < deadline:
                for key, _ in cleanup_selector.select(0.1):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        cleanup_selector.unregister(key.fileobj)
                        continue
                    if key.data == "stderr":
                        recorder.event({"event": "cleanup_ssh_stderr", "message": chunk.decode("utf-8", "replace")})
                        continue
                    cleanup_buffer += chunk
                    while b"\n" in cleanup_buffer:
                        line, cleanup_buffer = cleanup_buffer.split(b"\n", 1)
                        try:
                            event = json.loads(line.decode("utf-8", "strict"))
                            if isinstance(event, dict):
                                recorder.event(event)
                                if event.get("event") == "stopped" and event.get("child_stopped") is True:
                                    recorder.report["remote_stop_confirmed"] = True
                        except (ValueError, UnicodeError):
                            recorder.event({"event": "cleanup_unparsed_log", "line": line.decode("utf-8", "replace")})
                    if len(cleanup_buffer) > 1024*1024:
                        cleanup_buffer = b""
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for stream in (process.stdout, process.stderr):
                stream.close()
        recorder.status(state="stopped", reason=reason)
        recorder.close()


def self_test():
    import unittest
    from unittest import mock

    class Tests(unittest.TestCase):
        def remote_namespace(self, archive=b"hello"):
            config = {"archive_size": len(archive), "archive_sha256": hashlib.sha256(archive).hexdigest()}
            source = remote_source(config).rsplit("signal.signal(signal.SIGTERM, interrupted)", 1)[0]
            namespace = {}
            # Definitions/events only: omit signal installation and main().
            # The tested readers use mocked os.read; no board/device is opened.
            exec(compile(source, "<mock-remote-ram-runner>", "exec"), namespace)
            return namespace

        def test_archive_paths(self):
            for path in ("/absolute", "../parent", "a/../b", "a//b", "a\\b", "./a", "a/./b"):
                with self.assertRaises(ValueError):
                    safe_relative_path(path)
            self.assertEqual(safe_relative_path("a/b/"), "a/b")

        def test_kernel_normalization(self):
            self.assertEqual(normalized_kernel_sha256("a\r\nb\n"), normalized_kernel_sha256("a\nb"))
            self.assertNotEqual(normalized_kernel_sha256("a \nb"), normalized_kernel_sha256("a\nb"))
            with self.assertRaises(ValueError):
                normalized_kernel_sha256("")

        def test_actual_root_alias_still_requires_canonical_block_identity(self):
            from types import SimpleNamespace
            review = {"scope": "ram_model_web_only", "expected_boot_id": str(uuid.uuid4()),
                      "expected_cid": "a" * 32, "expected_root": {"device": "179:2", "source": "/dev/root", "filesystem": "ext4"},
                      "required_unmounted_device": "179:1", "kernel_log_sha256": "b" * 64}
            validate_review(review)
            validate_review({**review, "expected_root": {**review["expected_root"], "source": "/dev/mmcblk1p2"}})
            for changed in ({"source": "/dev/mmcblk0p2"}, {"device": "179:3"}, {"filesystem": "vfat"}):
                with self.assertRaises(ValueError):
                    validate_review({**review, "expected_root": {**review["expected_root"], **changed}})
            # Darwin's signed dev_t representation differs from Linux's; mock
            # major/minor rather than manufacturing a Linux device on the Mac.
            with mock.patch("os.major", return_value=179), mock.patch("os.minor", return_value=2):
                verify_root_block_node(SimpleNamespace(st_mode=stat.S_IFBLK | 0o600, st_rdev=1))
            for mode, major, minor in ((stat.S_IFREG, 179, 2), (stat.S_IFBLK, 179, 3), (stat.S_IFBLK, 180, 2)):
                with mock.patch("os.major", return_value=major), mock.patch("os.minor", return_value=minor):
                    with self.assertRaises(RuntimeError):
                        verify_root_block_node(SimpleNamespace(st_mode=mode, st_rdev=1))

        def test_remote_compiles(self):
            compile(remote_source({}), "<remote-ram-model-only>", "exec")
            compile(SERVER_WRAPPER, "<model-only-server>", "exec")

        def test_child_parent_death_guard_is_fail_closed(self):
            import ctypes
            for result, parents, error in ((0, [123, 123], None), (-1, [123], OSError),
                                           (0, [1], RuntimeError), (0, [123, 1], RuntimeError)):
                library = mock.Mock()
                library.prctl.return_value = result
                with mock.patch.dict(os.environ, {"XGODUCK_SUPERVISOR_PID": "123"}), mock.patch("os.getppid", side_effect=parents), mock.patch.object(ctypes, "CDLL", return_value=library), mock.patch.object(ctypes, "get_errno", return_value=22):
                    if error is None:
                        exec(CHILD_PARENT_GUARD, {})
                        library.prctl.assert_called_once_with(1, signal.SIGKILL, 0, 0, 0)
                    else:
                        with self.assertRaises(error):
                            exec(CHILD_PARENT_GUARD, {})

        def test_remote_eof_and_heartbeat_framing(self):
            for chunks in ([b""], [b"hello", b""], [b"hello", b"BAD\n"],
                           [b"hello", b"H", b"B\nHB\n", b""]):
                namespace = self.remote_namespace()
                with mock.patch("os.read", side_effect=chunks):
                    namespace["read_input"]()
                self.assertTrue(namespace["stop"].is_set())
                self.assertIsNotNone(namespace["shared"]["error"])
                self.assertTrue(namespace["shared"]["child_stop_confirmed"])
            namespace = self.remote_namespace()
            with namespace["lock"]:
                namespace["interrupted"](signal.SIGTERM, None)
            self.assertIn("signal", namespace["shared"]["error"])

        def test_remote_stale_and_changed_health_stop(self):
            namespace = self.remote_namespace()
            namespace["shared"]["last_input"] = 100
            namespace["watch_health"].next_check = 200
            with mock.patch.object(namespace["stop"], "wait", return_value=False), mock.patch("time.monotonic", return_value=105):
                namespace["watch_health"]()
            self.assertIn("timed out", namespace["shared"]["error"])
            namespace = self.remote_namespace()
            namespace["shared"]["last_input"] = 100
            namespace["watch_health"].next_check = 0
            namespace["snapshot"] = mock.Mock(side_effect=RuntimeError("mock card CID changed"))
            with mock.patch.object(namespace["stop"], "wait", return_value=False), mock.patch("time.monotonic", return_value=101):
                namespace["watch_health"]()
            self.assertIn("CID changed", namespace["shared"]["error"])

        def test_owned_process_group_stop_is_confirmed_and_idempotent(self):
            namespace = self.remote_namespace()
            child = mock.Mock(pid=41234)
            child.poll.return_value = None
            child.wait.side_effect = [subprocess.TimeoutExpired("mock", 1), 0]
            namespace["shared"]["process"] = child
            with mock.patch("os.killpg", side_effect=[None, None, ProcessLookupError]) as kill:
                self.assertTrue(namespace["stop_child"]())
                self.assertEqual(kill.call_args_list, [mock.call(41234, signal.SIGTERM), mock.call(41234, signal.SIGKILL), mock.call(41234, 0)])
                self.assertTrue(namespace["stop_child"]())
                self.assertEqual(kill.call_count, 3)

        def test_tunnel_reaper_handles_split_and_combined_heartbeats(self):
            class Selector:
                def register(self, *args):
                    pass
                def select(self, *args):
                    return [True]
            with mock.patch("selectors.DefaultSelector", return_value=Selector()), mock.patch("os.read", side_effect=[b"H", b"B\nHB\n", b""]), mock.patch("subprocess.run") as cancel, mock.patch("builtins.print"):
                cancel.return_value = subprocess.CompletedProcess(["mock"], 0, b"", b"")
                self.assertEqual(tunnel_reaper({"cancel_command": ["mock-cancel-own-forward"]}), 0)
                cancel.assert_called_once_with(["mock-cancel-own-forward"], capture_output=True, timeout=2)

        def test_tar_rejects_link_and_duplicate(self):
            for link, duplicate in ((True, False), (False, True)):
                output = io.BytesIO()
                with tarfile.open(fileobj=output, mode="w:gz") as archive:
                    entry = tarfile.TarInfo("x")
                    entry.size = 1
                    if link:
                        entry.type = tarfile.SYMTYPE
                        entry.linkname = "/dev/ttyUSB0"
                    archive.addfile(entry, io.BytesIO(b"x") if not link else None)
                    if duplicate:
                        archive.addfile(entry, io.BytesIO(b"x"))
                files = {"x": {"size_bytes": 1, "sha256": hashlib.sha256(b"x").hexdigest()}}
                with self.assertRaises(ValueError):
                    inspect_tar(output.getvalue(), files)

        def test_wheels_reject_link_path_and_collision(self):
            def wheel(path, link=False):
                output = io.BytesIO()
                with zipfile.ZipFile(output, "w") as archive:
                    item = zipfile.ZipInfo(path)
                    if link:
                        item.external_attr = (stat.S_IFLNK | 0o777) << 16
                    archive.writestr(item, b"x")
                return output.getvalue()
            for members in ({"wheels/a.whl": wheel("../outside")},
                            {"wheels/a.whl": wheel("link", True)},
                            {"wheels/a.whl": wheel("same"), "wheels/b.whl": wheel("same")}):
                with self.assertRaises(ValueError):
                    inspect_wheels(members)

        def test_guard_liveness(self):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "status.json"
                review = {"expected_boot_id": "boot", "expected_cid": "cid", "kernel_log_sha256": "hash", "expected_root": {"device": "179:2", "source": "/dev/mmcblk1p2", "filesystem": "ext4"}}
                status = {"safe": False, "safe_for_ram_model_web": True, "capability_scope": "ram_model_web_only", "local_pid": os.getpid()+100, "last_heartbeat_monotonic": 100, "boot_id": "boot", "card_cid": "cid", "kernel_log_sha256": "hash", "root": review["expected_root"]}
                path.write_text(json.dumps(status))
                path.chmod(0o600)
                with mock.patch("os.kill"):
                    self.assertTrue(check_guard(path, review, now=102)["safe_for_ram_model_web"])
                    with self.assertRaises(RuntimeError):
                        check_guard(path, review, now=105)
                    status["safe_for_ram_model_web"] = False
                    path.write_text(json.dumps(status))
                    with self.assertRaises(RuntimeError):
                        check_guard(path, review, now=102)
                status["safe_for_ram_model_web"] = True
                path.write_text(json.dumps(status))
                with mock.patch("os.kill", side_effect=ProcessLookupError):
                    with self.assertRaises(ProcessLookupError):
                        check_guard(path, review, now=102)

        def test_private_reports(self):
            with tempfile.TemporaryDirectory() as directory:
                recorder = Recorder(directory)
                recorder.status(state="stopped")
                for name in ("status.json", "events.jsonl", "tunnel-reaper.jsonl"):
                    self.assertEqual((Path(directory)/name).stat().st_mode & 0o777, 0o600)
                recorder.close()

    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    return 0 if result.wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection-json")
    parser.add_argument("--guard-status")
    parser.add_argument("--bundle")
    parser.add_argument("--manifest")
    parser.add_argument("--reviewed-baseline")
    parser.add_argument("--output-dir")
    parser.add_argument("--local-port", type=int, default=8768)
    parser.add_argument("--remote-port", type=int, default=8767)
    parser.add_argument("--self-test", action="store_true", help="Local/mocked tests only; no SSH")
    parser.add_argument("--internal-tunnel-reaper", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.internal_tunnel_reaper:
        return tunnel_reaper(json.loads(args.internal_tunnel_reaper))
    if not all((args.connection_json, args.guard_status, args.bundle, args.manifest, args.reviewed_baseline, args.output_dir)):
        parser.error("connection, guard, bundle, manifest, review and output arguments are required")
    def interrupted(signum, frame):
        raise KeyboardInterrupt("RAM supervisor interrupted by signal %s" % signum)
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        return supervise(args)
    except BaseException as error:
        print("STOP before deployment: " + (str(error) or "Interrupted"), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
