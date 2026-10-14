"""Guarded local children for the X5 calibration and maintenance workbench.

The wire protocols are the existing reviewed transports.  This module changes
only how their single owner is launched: no SSH, modem reset, retry or fallback.
Child exit and UART CLOSED are retained as separate observations.
"""
from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import select
import stat
import subprocess
import sys
import threading
import time

import feedback_stream as reader
import feedback_connection_check as connection
import feedback_fault_probe as fault_probe
import maintenance_transport as maintenance
from run_feedback_fault_details import private_new, strict_json


MAX_SOURCE = 512 * 1024


def pinned_uart_checker(path, node, *, resolve_path, error_type):
    """Check the already reviewed open UART without walking USB sysfs.

    Full USB identity is checked at connect/open/ARM and by the independent
    health monitor. An unplug/replug creates a different character node; each
    hot-path request must still match both that node and its open descriptor.
    """
    if (type(path) is not str or re.fullmatch(r'/dev/ttyUSB(?:0|[1-9][0-9]{0,2})', path) is None
            or type(node) is not tuple or len(node) != 3
            or any(type(value) is not int or value < 0 for value in node)
            or not callable(resolve_path) or not isinstance(error_type, type)
            or not issubclass(error_type, Exception)):
        raise ValueError('An explicitly reviewed UART path and node are required')

    def check(expected, descriptor):
        if type(descriptor) is not int or descriptor < 0 or resolve_path(expected) != path:
            raise error_type('Connected UART path or descriptor changed')
        current = os.stat(path, follow_symlinks=False)
        opened = os.fstat(descriptor)
        for info in (current, opened):
            if not stat.S_ISCHR(info.st_mode) or (info.st_dev, info.st_ino, info.st_rdev) != node:
                raise error_type('Connected UART node changed; reconnect explicitly')
        return current, dict(path=path, proof='same_character_node_and_open_descriptor')

    return check


def usb_recovery_proof(kernel, expected, serial, validate_kernel):
    """Recognize complete CH340 unplug/replug sequences after the fixed review.

    This only reconciles kernel history for an explicit reconnect. Actual board,
    UART ownership, firmware and servo feedback must still pass their own gates.
    """
    validate_kernel(expected['kernel_prefix'], expected)
    prefix, lines = expected['kernel_prefix'].splitlines(), kernel.splitlines()
    if lines[:len(prefix)] != prefix:
        raise ValueError('启动日志已变化或丢失，不能按 USB 重连恢复。')
    tail = lines[len(prefix):]
    if not tail:
        return None
    if len(tail) > 512 or len(kernel.encode()) > reader.MAX_KERNEL_BYTES:
        raise ValueError('USB 重连记录超过检查范围。')
    port, tty = re.escape(serial['sysfs_name']), re.escape(Path(serial['path']).name)
    if (serial['vid'], serial['pid'], serial['serial_number']) != ('1a86', '7523', ''):
        raise ValueError('USB 控制器身份变化，不能自动恢复。')
    patterns = [
        rf'usb {port}: USB disconnect, device number [1-9][0-9]*',
        rf'ch341-uart {tty}: ch341-uart converter now disconnected from {tty}',
        rf'ch341 {port}:1\.0: device disconnected',
        rf'usb {port}: new full-speed USB device number [1-9][0-9]* using xhci-hcd',
        rf'usb {port}: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice=\s*2\.64',
        rf'usb {port}: New USB device strings: Mfr=0, Product=2, SerialNumber=0',
        rf'usb {port}: Product: USB Serial',
        rf'ch341 {port}:1\.0: ch341-uart converter detected',
        rf'usb {port}: ch341-uart converter now attached to {tty}',
    ]
    # xHCI can announce a port disable immediately before the same complete
    # disconnect/re-enumeration sequence. Match only this controller's parent
    # hub and physical port; a warning alone never proves recovery.
    hub, separator, port_number = serial['sysfs_name'].rpartition('.')
    if not separator:
        hub, separator, port_number = serial['sysfs_name'].partition('-')
    hub_warning = rf'usb {re.escape(hub)}-port{re.escape(port_number)}: disabled by hub \(EMI\?\), re-enabling\.\.\.'
    removed_callback = rf'usb {port}: ch341_read_int_callback - usb_submit_urb failed: -19'
    faults, informational, cycles, previous, stage, hub_pending = [], [], 0, -1., 0, False
    disconnect_time, callback_seen = None, False
    enumeration_time, descriptor_errors = None, 0
    for line in tail:
        match = re.fullmatch(r'\[\s*(\d+\.\d+)\] (.*)', line)
        if not match or float(match[1]) < previous:
            raise ValueError('日志包含未完成重连或其他硬件异常，不能自动解除：' + line[:256])
        previous = float(match[1])
        # Network messages can interleave with CH341 teardown/enumeration,
        # especially while moving the USB network cable or changing Wi-Fi.
        # Keep these exact informational forms in the retained history without
        # advancing the USB state machine. The original kernel validator must
        # independently accept each line; arbitrary non-USB text is not skipped.
        network_info = (
            re.fullmatch(r'IPv6: ADDRCONF\(NETDEV_CHANGE\): (?:usb[01]|wlan0): link becomes ready', match[2])
            or re.fullmatch(r'rwnx_send_sm_connect_req drv_vif_index:0 connect to [^\s]+ channel:[1-9][0-9]{0,4} auth_type:[0-9]+', match[2])
            or re.fullmatch(r'get_txpwr_max:txpwr_max:[0-9]+\s*', match[2])
            or re.fullmatch(r'(?:deinit:macaddr:|reord_mac:)(?:[0-9a-f]{2},){5}[0-9a-f]{2}', match[2])
            or re.fullmatch(r'device wlan0 (?:entered|left) promiscuous mode', match[2]))
        if network_info:
            validate_kernel(expected['kernel_prefix'] + line + '\n', expected)
            informational.append(line)
            continue
        if stage == 0 and not hub_pending and re.fullmatch(hub_warning, match[2]):
            faults.append(dict(line=line, reason='Kernel reported a USB disconnect/reset/error'))
            hub_pending = True
            continue
        # CH341 may finish its interrupt callback after USB disconnect. Linux
        # ENODEV (-19) is expected only inside this same, completed return:
        # one callback, immediately after the matching disconnect, before the
        # UART teardown. Other errors, ports, ordering or incomplete returns
        # still fail; retain the callback in the historical evidence.
        if (stage == 1 and not callback_seen and disconnect_time is not None
                and previous - disconnect_time <= .1
                and re.fullmatch(removed_callback, match[2])):
            faults.append(dict(line=line, reason='CH341 reported ENODEV during USB disconnect'))
            callback_seen = True
            continue
        # A transient descriptor failure is historical only after this same
        # enumeration completes with the reviewed identity and UART attachment.
        if (stage == 4 and enumeration_time is not None
                and previous - enumeration_time <= 2.0 and descriptor_errors < 2
                and re.fullmatch(rf'usb {port}: device descriptor read/64, error -32', match[2])):
            faults.append(dict(line=line, reason='USB descriptor failed during completed re-enumeration'))
            descriptor_errors += 1
            continue
        if not re.fullmatch(patterns[stage], match[2]):
            raise ValueError('日志包含未完成重连或其他硬件异常，不能自动解除：' + line[:256])
        if stage == 3:
            enumeration_time, descriptor_errors = previous, 0
        if stage == 0:
            faults.append(dict(line=line, reason='Kernel reported a USB disconnect/reset/error'))
            hub_pending = False
            disconnect_time, callback_seen = previous, False
        if stage == len(patterns)-1:
            cycles += 1
        stage = (stage + 1) % len(patterns)
    if not cycles or stage or hub_pending:
        raise ValueError('USB 尚未完整恢复，请接好 COM 线后再次检查。')
    return dict(schema_version=1, kernel_prefix=kernel,
        kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),
        base_sha256=hashlib.sha256(expected['kernel_prefix'].encode()).hexdigest(),
        boot_id=expected['boot_id'], cid=expected['cid'], serial=copy.deepcopy(serial),
        faults=faults, informational_lines=informational,
        reconnect_count=cycles, historical_faults_cleared=False,
        motion_resumed=False)


USB_RECOVERY_VALIDATOR = r'''
_native_strict_validate_kernel = validate_kernel
def validate_kernel(kernel, expected):
    proof = NATIVE_USB_RECOVERY
    if 'restore_kernel_history' in globals():
        try:
            kernel = restore_kernel_history(kernel, proof['kernel_prefix'])
        except ValueError as error:
            raise ProbeError(str(error)) from error
    baseline = _native_strict_validate_kernel(expected['kernel_prefix'], expected)
    if (proof['boot_id'] != expected['boot_id'] or proof['cid'] != expected['cid'] or
            proof['base_sha256'] != hashlib.sha256(expected['kernel_prefix'].encode()).hexdigest() or
            proof['kernel_sha256'] != hashlib.sha256(proof['kernel_prefix'].encode()).hexdigest() or
            any(proof['serial'][key] != expected['serial'][key]
                for key in ('vid','pid','serial_number'))):
        raise ProbeError('USB recovery identity or retained history changed')
    accepted, lines = proof['kernel_prefix'].splitlines(), kernel.splitlines()
    if lines[:len(accepted)] != accepted:
        raise ProbeError('Retained USB recovery history changed or lost its prefix')
    for line in lines[len(accepted):]:
        reason = classify_feedback_log(line)
        if (reason or re.fullmatch(r'\[\s*\d+\.\d+\] usb \S+-port[1-9][0-9]*: disabled by hub \(EMI\?\), re-enabling\.\.\.', line)
                or re.fullmatch(r'\[\s*\d+\.\d+\] usb \S+: ch341_read_int_callback - usb_submit_urb failed: -[0-9]+', line)):
            raise ProbeError('USB 恢复后发生新的硬件异常，请重新检查连接：' + line[:512])
    return dict(kernel_log=kernel, kernel_sha256=hashlib.sha256(('\n'.join(lines)+'\n').encode()).hexdigest(),
        kernel_lines=len(lines),historical_faults=baseline['historical_faults']+proof['faults'],
        historical_faults_cleared=False)
'''


def install_usb_recovery(namespace, source, proof):
    if proof is None:
        return source
    addition = '\nNATIVE_USB_RECOVERY = ' + repr(proof) + '\n' + USB_RECOVERY_VALIDATOR
    exec(compile(addition, '<native-usb-recovery-guard>', 'exec'), namespace)
    if 'restore_kernel_history' in namespace:
        reader = namespace['bounded_dmesg']
        restore = namespace['restore_kernel_history']
        namespace['bounded_dmesg'] = lambda: restore(reader(), proof['kernel_prefix'])
    return source + addition


def current_usb_recovery_proof(kernel, expected, serial, validate_kernel):
    """Retain USB history without requiring a particular enumeration sequence.

    serial_proof has checked the current device. Connection/feedback gates still
    run afterward; accepting history never enables torque or resumes motion.
    """
    prefix = expected['kernel_prefix']
    lines, baseline = kernel.splitlines(), prefix.splitlines()
    if lines[:len(baseline)] != baseline:
        raise ValueError('启动日志已变化或丢失，不能按 USB 重连恢复。')
    if len(kernel.encode()) > reader.MAX_KERNEL_BYTES:
        raise ValueError('USB 重连记录超过检查范围。')
    if (serial['vid'], serial['pid'], serial['serial_number']) != ('1a86', '7523', ''):
        raise ValueError('USB 控制器身份变化，不能自动恢复。')
    port, tty = re.escape(serial['sysfs_name']), re.escape(Path(serial['path']).name)
    hub, separator, number = serial['sysfs_name'].rpartition('.')
    if not separator:
        hub, _, number = serial['sysfs_name'].partition('-')
    device_line = re.compile(
        rf'\[\s*\d+\.\d+\] (?:usb {port}: |ch341 {port}:1\.0: |'
        rf'ch341-uart {tty}: |usb {re.escape(hub)}-port{re.escape(number)}: )')
    faults, other = [], []
    for line in lines[len(baseline):]:
        if device_line.match(line):
            faults.append(dict(line=line, reason='USB history retained; current connection checked separately'))
        else:
            other.append(line)
    # Unrelated hardware faults remain subject to the original validator.
    validate_kernel(prefix + ''.join(line + '\n' for line in other), expected)
    if not faults:
        return None
    return dict(schema_version=1, kernel_prefix=kernel,
        kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),
        base_sha256=hashlib.sha256(prefix.encode()).hexdigest(),
        boot_id=expected['boot_id'], cid=expected['cid'], serial=copy.deepcopy(serial),
        faults=faults, informational_lines=other, reconnect_count=0,
        historical_faults_cleared=False, motion_resumed=False)


def prepare_usb_recovery(namespace, expected):
    kernel = namespace['bounded_dmesg']()
    if 'restore_kernel_history' in namespace:
        kernel = namespace['restore_kernel_history'](kernel, expected['kernel_prefix'])
    _, serial = namespace['serial_proof'](expected)
    try:
        namespace['validate_kernel'](kernel, expected)
    except namespace['ProbeError']:
        try:
            return usb_recovery_proof(kernel, expected, serial, namespace['validate_kernel'])
        except ValueError:
            return current_usb_recovery_proof(kernel, expected, serial, namespace['validate_kernel'])
    # Some USB enumeration messages are not classified by the legacy guard.
    # Retain them as history after checking the current serial identity.
    tail = kernel.splitlines()[len(expected['kernel_prefix'].splitlines()):]
    if any('Device not responding to setup address' in line
            or re.fullmatch(r'\[\s*\d+\.\d+\] usb \S+-port[1-9][0-9]*: disabled by hub \(EMI\?\), re-enabling\.\.\.', line)
            or re.fullmatch(r'\[\s*\d+\.\d+\] usb \S+: ch341_read_int_callback - usb_submit_urb failed: -[0-9]+', line) for line in tail):
        try:
            return usb_recovery_proof(kernel, expected, serial, namespace['validate_kernel'])
        except ValueError:
            return current_usb_recovery_proof(kernel, expected, serial, namespace['validate_kernel'])
    return None
LOCAL_CHILD_BOOTSTRAP = '''import hashlib,os,select,sys,time
n=int(sys.argv[1]); digest=sys.argv[2]; parent=int(sys.argv[3])
if not 0<n<=524288 or len(digest)!=64 or os.getppid()!=parent: raise SystemExit(21)
raw=bytearray(); deadline=time.monotonic()+8.
while len(raw)<n:
    if os.getppid()!=parent or time.monotonic()>=deadline: raise SystemExit(21)
    if not select.select([sys.stdin.fileno()],[],[],.2)[0]: continue
    chunk=os.read(sys.stdin.fileno(),min(65536,n-len(raw)))
    if not chunk: raise SystemExit(21)
    raw.extend(chunk)
if len(raw)!=n or hashlib.sha256(raw).hexdigest()!=digest or os.getppid()!=parent: raise SystemExit(21)
exec(compile(bytes(raw).decode('utf-8'),'<reviewed-native-child>','exec'),{'__name__':'__main__'})
'''


def child_command(source, *, parent_pid=None, executable=None):
    if type(source) is not bytes or not 0 < len(source) <= MAX_SOURCE:
        raise ValueError('Native child source exceeds its fixed bound')
    source.decode('utf-8')
    parent_pid = os.getpid() if parent_pid is None else parent_pid
    if type(parent_pid) is not int or parent_pid <= 1:
        raise ValueError('Native child requires a live parent identity')
    return [sys.executable if executable is None else executable, '-I', '-B', '-S', '-c',
            LOCAL_CHILD_BOOTSTRAP, str(len(source)), hashlib.sha256(source).hexdigest(), str(parent_pid)]


def allowed_child_command(executable, command):
    """Exact audit allowlist for the service, never a general Python launcher."""
    if executable != sys.executable or type(command) not in (list, tuple) or len(command) != 9:
        return False
    if list(command[:6]) != [sys.executable, '-I', '-B', '-S', '-c', LOCAL_CHILD_BOOTSTRAP]:
        return False
    n, digest, parent = command[6:]
    return (type(n) is str and n.isascii() and n.isdecimal() and str(int(n)) == n
            and 0 < int(n) <= MAX_SOURCE and type(digest) is str and len(digest) == 64
            and all(c in '0123456789abcdef' for c in digest) and parent == str(os.getpid()))


def write_once(fd, raw, seconds):
    deadline, offset = time.monotonic() + seconds, 0
    while offset < len(raw):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('Native child pipe transfer timed out')
        _, writable, exceptional = select.select([], [fd], [fd], remaining)
        if exceptional:
            raise OSError('Native child pipe transfer failed')
        if writable:
            try:
                count = os.write(fd, raw[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise OSError('Native child pipe made no progress')
            offset += count


IMU_TRACKER = r'''
class ImuCollectionStateTracker:
    """IMU-only scope preserves servo masks; it never grants joint readiness."""
    def __init__(self):
        self.previous = None
        self.sample_progress = self.imu_progress = None
    def accept(self, state, now):
        if state['armed'] is not False or state['arm_cookie'] or state['torque_mask']:
            raise ProbeError('IMU collection requires disarmed, cookie-zero, torque-off feedback')
        if any(type(state[key]) is not int or not 0 <= state[key] <= ALL_MASK
               for key in ('online_mask', 'fault_mask')):
            raise ProbeError('Invalid servo masks in IMU-only feedback')
        if state['last_error'] or state['imu_ok'] is not True:
            raise ProbeError('Firmware error or invalid IMU feedback')
        if any(not math.isfinite(value) for value in state['accel_ms2'] + state['gyro_dps']):
            raise ProbeError('Non-finite IMU data')
        age = (state['sample_us'] - state['imu_us']) & 0xffffffff
        if 500000 < age < 0x80000000:
            raise ProbeError('IMU sample is stale relative to servo sample time')
        if age >= 0x80000000 and ((state['imu_us']-state['sample_us']) & 0xffffffff) > 500000:
            raise ProbeError('IMU and servo timestamps are inconsistent')
        if self.previous is None:
            self.previous = state
            self.sample_progress = self.imu_progress = now
            return True
        advances = []
        for key, clock_key in (('sample_us','sample_progress'), ('imu_us','imu_progress')):
            delta = (state[key]-self.previous[key]) & 0xffffffff
            if delta >= 0x80000000 or delta > 500000:
                raise ProbeError('Feedback sample time reversed/reset or jumped')
            if delta:
                setattr(self, clock_key, now)
            elif now-getattr(self, clock_key) >= .5:
                raise ProbeError('Feedback timestamp stopped advancing')
            advances.append(delta > 0)
        if not all(advances):
            return False
        self.previous = state
        return True
'''


CHILD_GUARD = r'''
import hmac
_native_health_snapshot = health_snapshot
_native_serial_proof = serial_proof
_native_calibration_revision = NATIVE_PIN.get('calibration_revision',0)
def _native_private_read(path, maximum):
    if os.path.realpath(path)!=path:
        raise ProbeError('Native authority path is no longer canonical')
    descriptor=os.open(path,os.O_RDONLY|os.O_CLOEXEC|os.O_NOFOLLOW|os.O_NONBLOCK)
    try:
        info=os.fstat(descriptor)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or
                info.st_mode&0o077 or info.st_nlink!=1 or info.st_size>maximum):
            raise ProbeError('Native authority file is no longer a private regular file')
        raw=bytearray()
        while len(raw)<=maximum:
            data=os.read(descriptor,min(65536,maximum+1-len(raw)))
            if not data: break
            raw.extend(data)
        if len(raw)>maximum:
            raise ProbeError('Native authority file exceeds its bounded size')
        return bytes(raw)
    finally:
        os.close(descriptor)
def serial_proof(expected, fd=None):
    node, receipt = _native_serial_proof(expected, fd)
    if ([node.st_dev,node.st_ino,node.st_rdev] != NATIVE_PIN['serial_node'] or
            serial_path(expected) != NATIVE_PIN['serial_path']):
        raise ProbeError('Native child UART differs from the explicit parent gate')
    return node, receipt
def health_snapshot(expected, fd=None):
    global _native_calibration_revision
    if os.getppid() != NATIVE_PARENT:
        raise ProbeError('Native workbench parent disappeared')
    result = _native_health_snapshot(expected, fd)
    node = os.stat(NATIVE_PIN['root_path'], follow_symlinks=False)
    if (not stat.S_ISDIR(node.st_mode) or [node.st_dev,node.st_ino] != NATIVE_PIN['root_node'] or
            node.st_uid!=os.getuid() or stat.S_IMODE(node.st_mode)!=0o700 or
            os.path.realpath(NATIVE_PIN['root_path']) != NATIVE_PIN['root_path']):
        raise ProbeError('Native child RAM directory changed')
    mounts = bounded_read('/proc/self/mountinfo',262144).splitlines()
    config = [line for line in mounts if len(line.split())>4 and line.split()[2]=='179:1']
    shm = [line for line in mounts if len(line.split())>4 and line.split()[4]=='/dev/shm']
    if config != NATIVE_PIN['config_mount'] or shm != NATIVE_PIN['shm_mount']:
        raise ProbeError('Native child CONFIG/RAM mount changed')
    if 'calibration_path' in NATIVE_PIN:
        info = os.stat(NATIVE_PIN['calibration_path'],follow_symlinks=False)
        if (not stat.S_ISREG(info.st_mode) or info.st_mode & 0o077 or
                os.path.realpath(NATIVE_PIN['calibration_path']) != NATIVE_PIN['calibration_path']):
            raise ProbeError('Native child calibration file changed type or permissions')
        if 'calibration_authority_path' in NATIVE_PIN:
            if os.path.realpath(NATIVE_PIN['calibration_lock_path'])!=NATIVE_PIN['calibration_lock_path']:
                raise ProbeError('Native calibration authority lock path changed')
            lock = os.open(NATIVE_PIN['calibration_lock_path'],os.O_RDONLY|os.O_CLOEXEC|os.O_NOFOLLOW)
            try:
                lock_info=os.fstat(lock)
                if (not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid!=os.getuid() or
                        lock_info.st_mode&0o077 or lock_info.st_nlink!=1):
                    raise ProbeError('Native calibration authority lock changed')
                deadline = time.monotonic()+.20
                while True:
                    try:
                        fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
                        break
                    except BlockingIOError:
                        if time.monotonic() >= deadline:
                            raise ProbeError('Native calibration authority lock timed out')
                        time.sleep(.002)
                def unique(values):
                    result = {}
                    for key,value in values:
                        if key in result: raise ProbeError('Duplicate native calibration authority key')
                        result[key]=value
                    return result
                authority=json.loads(_native_private_read(NATIVE_PIN['calibration_authority_path'],4096),object_pairs_hook=unique)
                if (type(authority) is not dict or set(authority)!={'sha256','revision','mac'} or
                        type(authority['revision']) is not int or
                        authority['revision']<max(NATIVE_PIN['calibration_revision'],_native_calibration_revision) or
                        type(authority['sha256']) is not str or len(authority['sha256'])!=64 or
                        any(c not in '0123456789abcdef' for c in authority['sha256']) or
                        type(authority['mac']) is not str or len(authority['mac'])!=64):
                    raise ProbeError('Native calibration authority record differs')
                message=json.dumps({'revision':authority['revision'],'sha256':authority['sha256']},
                    sort_keys=True,separators=(',',':')).encode()
                mac=hmac.new(bytes.fromhex(NATIVE_PIN['calibration_authority_key']),message,hashlib.sha256).hexdigest()
                if not hmac.compare_digest(mac,authority['mac']):
                    raise ProbeError('Native calibration authority MAC differs')
                raw=_native_private_read(NATIVE_PIN['calibration_path'],262144)
                if hashlib.sha256(raw).hexdigest()!=authority['sha256']:
                    raise ProbeError('Native calibration differs from its authorized commit')
                _native_calibration_revision=authority['revision']
            finally:
                os.close(lock)
        else:
            raw = _native_private_read(NATIVE_PIN['calibration_path'],262144)
            if hashlib.sha256(raw).hexdigest() != NATIVE_PIN['calibration_sha256']:
                raise ProbeError('Native child calibration changed without an explicit reload')
    return result
'''


NATIVE_READER_HEALTH = r'''
import threading
class NativeReaderHealth:
    """Full identity/log checks without pausing the only UART sampler."""
    def __init__(self, expected, fd):
        self.expected, self.fd = expected, fd
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.started = self.completed = time.monotonic()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
    def _run(self):
        while not self.stop.wait(max(.001, self.started + HEALTH_INTERVAL - time.monotonic())):
            with self.lock:
                self.started = began = time.monotonic()
            try:
                health_snapshot(self.expected, self.fd)
                finished = time.monotonic()
                if finished - began > .5:
                    raise ProbeError('Storage/USB health check took too long')
            except BaseException as error:
                with self.lock:
                    self.error = error
                return
            with self.lock:
                self.completed = finished
    def check(self):
        with self.lock:
            if self.error is not None:
                raise self.error
            if not self.thread.is_alive() or time.monotonic() - self.started > .5:
                raise ProbeError('Storage/USB health checks lost their timing guarantee')
            return self.completed
    def close(self):
        self.stop.set()
        self.thread.join(.55)
        if self.thread.is_alive():
            raise ProbeError('Storage/USB health worker did not stop')
        with self.lock:
            if self.error is not None:
                raise self.error
reader_health_watcher = NativeReaderHealth
'''


def _private_directory(directory):
    directory = Path(directory)
    info = directory.lstat()
    if (not directory.is_absolute() or directory.resolve() != directory
            or not stat.S_ISDIR(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o700
            or info.st_uid != os.getuid()):
        raise ValueError('Native transport evidence requires a private canonical RAM directory')
    return directory


def preflight_stack(error):
    """Bounded callsite metadata only; never inspect locals or source lines."""
    frames=[]
    trace=error.__traceback__
    while trace is not None:
        code=trace.tb_frame.f_code
        frames.append(dict(file=os.path.basename(code.co_filename)[:160],
                           function=code.co_name[:80],line=trace.tb_lineno))
        del frames[:-8]
        trace=trace.tb_next
    return frames


def _guard_pin(namespace, root, supplied=None):
    if not isinstance(namespace, dict) or not callable(namespace.get('ram_health')):
        raise ValueError('Fresh native connection guards are required')
    namespace['ram_health']()
    expected = copy.deepcopy(namespace['EXPECTED'])
    node, _ = namespace['serial_proof'](expected)
    if namespace['_holders'](node.st_rdev, time.monotonic() + .2):
        raise ValueError('UART already has an owner')
    spec = namespace.get('NATIVE_RAM_GUARD') if supplied is None else supplied
    if type(spec) is not dict:
        raise ValueError('Serializable native RAM guard is required')
    pin = copy.deepcopy(spec)
    required = {'root_path', 'root_node', 'shm_mount', 'config_mount'}
    authority_fields={'calibration_authority_path','calibration_authority_key',
                      'calibration_lock_path','calibration_revision'}
    if not required <= set(pin) or set(pin) - required - {'calibration_path', 'calibration_sha256'} - authority_fields:
        raise ValueError('Native RAM guard fields differ')
    if pin['root_path'] != str(root):
        raise ValueError('Native RAM root differs')
    actual = root.lstat()
    if (root.is_symlink() or root.resolve() != root or not stat.S_ISDIR(actual.st_mode)
            or actual.st_uid!=os.getuid() or stat.S_IMODE(actual.st_mode)!=0o700
            or pin['root_node'] != [actual.st_dev, actual.st_ino]):
        raise ValueError('Native RAM root identity differs')
    if any(type(pin[name]) is not list or not pin[name] or
           any(type(line) is not str for line in pin[name]) for name in ('shm_mount', 'config_mount')):
        raise ValueError('Exact native CONFIG/RAM mounts are required')
    if ('calibration_path' in pin) != ('calibration_sha256' in pin):
        raise ValueError('Incomplete native calibration pin')
    if 'calibration_path' in pin:
        path = Path(pin['calibration_path'])
        if not path.is_absolute() or path.parent != root / 'policy':
            raise ValueError('Native calibration path differs')
        digest = pin['calibration_sha256']
        if type(digest) is not str or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('Native calibration digest differs')
    if authority_fields & set(pin):
        if not authority_fields <= set(pin) or 'calibration_path' not in pin:
            raise ValueError('Native calibration authority is incomplete')
        for name in ('calibration_authority_path','calibration_lock_path'):
            path=Path(pin[name])
            if not path.is_absolute() or not path.is_relative_to(root):
                raise ValueError('Native calibration authority path differs')
        key=pin['calibration_authority_key']
        if type(key) is not str or len(key)!=128 or any(c not in '0123456789abcdef' for c in key):
            raise ValueError('Native calibration authority key differs')
        if type(pin['calibration_revision']) is not int or pin['calibration_revision']<0:
            raise ValueError('Native calibration authority revision differs')
    pin.update(serial_path=namespace['serial_path'](expected),
               serial_node=[node.st_dev, node.st_ino, node.st_rdev])
    return expected, pin


PARTIAL_SERVO_DIAGNOSTIC = r'''
def _native_partial_servo_diagnostic(fd, sequence, state, expected, metadata):
    # Terminal diagnostics only: never accept the rejected STATE, reopen UART,
    # retry, or change the original close reason. Histories can advance between
    # STATE and this reply; they are not a frozen explanation of the failure.
    if (state['armed'] is not False or state['arm_cookie'] or state['torque_mask']
            or state['fault_mask'] or state['last_error']
            or state['online_mask'] in (0, ALL_MASK)):
        return
    started = time.monotonic()
    record = dict(rejected_sequence=sequence, rejected_online_mask=state['online_mask'],
                  rejected_sample_us=state['sample_us'], snapshot_is_latched=False,
                  diagnostic_only=True, motion_authorized=False)
    metadata['partial_servo_diagnostic'] = record
    try:
        if sequence >= 0xffffffff:
            raise ProbeError('Diagnostic protocol sequence exhausted')
        serial_proof(expected, fd)
        payload = _fault_exchange(fd, sequence + 1, started + .10, metadata)
        record['details'] = _decode_fault_details(payload)
    except Exception as error:
        record['error'] = (str(error) or type(error).__name__)[:512]
    record['elapsed_seconds'] = time.monotonic() - started
'''


JOINT_COLLECTION_READER = r'''
_joint_exchange = _exchange
_joint_decode_state = _decode_state
_joint_tracker = StateTracker
_joint_hello = _hello_feedback
_joint_torque_known = 0

def _hello_feedback(fd, metadata):
    _joint_hello(fd, metadata)
    # Type30 selects the interface in MCU RAM. Its firmware handler refuses
    # enabled torque and never writes motor goals.
    reply = _joint_exchange(fd, 30, 2, time.monotonic() + REPLY_SECONDS, metadata)
    if reply != bytes((30, 0)):
        raise ProbeError('Joint collection feedback interface refused')

def _exchange(fd, kind, sequence, total_deadline, metadata):
    global _joint_torque_known
    if kind != 2:
        return _joint_exchange(fd, kind, sequence, total_deadline, metadata)
    payload = _joint_exchange(fd, 32, sequence, total_deadline, metadata)
    # SOURCE_STATE independently reports torque validity. An unreadable
    # position must not close the entire calibration session.
    versions = {170: 1, 214: 2, 205: 3, 249: 4}
    if payload[108:110] != bytes((1, versions[len(payload)])):
        raise ProbeError('Joint collection state version mismatch')
    _joint_torque_known = struct.unpack_from('<H', payload, 106)[0]
    if _joint_torque_known & ~ALL_MASK:
        raise ProbeError('Invalid joint collection torque mask')
    return payload[:STATE_PACKET.size]

def _decode_state(payload):
    state = _joint_decode_state(payload)
    state['torque_known_mask'] = _joint_torque_known
    return state

class JointCollectionStateTracker(_joint_tracker):
    def accept(self, state, now):
        # Publish the actual online mask. Keep the original torque, fault,
        # IMU and clock checks. Missing torque feedback pauses capture in the
        # parent instead of destroying the whole collection session.
        return super().accept(dict(state, online_mask=ALL_MASK), now)

StateTracker = JointCollectionStateTracker
'''


def joint_collection_source(base_source):
    """Use independent torque validity for passive joint collection."""
    changes = (
        ('if kind not in (1, 2):', 'if kind not in (1, 2, 30, 32):'),
        ('(0x81, 16) if kind == 1 else (0x82, STATE_PACKET.size)',
         '(0x81, 16) if kind == 1 else (0x80, 2) if kind == 30 else (0xa0, None) if kind == 32 else (0x82, STATE_PACKET.size)'),
        ('size != expected_size',
         '((kind == 32 and size not in (170, 214, 205, 249)) or (kind != 32 and size != expected_size))'),
        ('        sequence = 1\n', '        sequence = 2\n'),
    )
    for before, after in changes:
        if base_source.count(before) != 1:
            raise ValueError('Joint collection reader boundary differs: ' + before)
        base_source = base_source.replace(before, after, 1)
    return base_source + '\n' + JOINT_COLLECTION_READER


def partial_servo_diagnostic_source(base_source):
    """Add one bounded read-only diagnostic only to a terminal joint fault."""
    anchor = '            accepted = tracker.accept(state, now)\n'
    if base_source.count(anchor) != 1:
        raise ValueError('Reviewed joint-reader acceptance boundary differs')
    replacement = '''            try:
                accepted = tracker.accept(state, now)
            except ProbeError as failure:
                if str(failure) == 'Partial servo communication; reader will close':
                    _native_partial_servo_diagnostic(fd, sequence, state, expected, metadata)
                raise
'''
    definitions = fault_probe.FAULT_RUNTIME.split('def run_fault_probe(', 1)[0]
    return base_source.replace(anchor, replacement) + '\n' + definitions + PARTIAL_SERVO_DIAGNOSTIC


def child_source(namespace, root, generation, *, scope, guard_spec=None, parent_pid=None):
    if type(generation) is not int or not 0 < generation < 0x80000000:
        raise ValueError('Native transport generation differs')
    if scope not in ('imu', 'joints', 'maintenance', 'metadata'):
        raise ValueError('Native transport scope differs')
    expected, pin = _guard_pin(namespace, Path(root), guard_spec)
    base_source=namespace.get('NATIVE_RUNTIME_SOURCE')
    if type(base_source) is not str or not 0<len(base_source.encode('utf-8'))<MAX_SOURCE:
        raise ValueError('Pinned native runtime source is required')
    expected['generation'] = generation
    parent_pid = os.getpid() if parent_pid is None else parent_pid
    if type(parent_pid) is not int or parent_pid <= 1:
        raise ValueError('Native transport parent differs')
    if scope == 'maintenance':
        source, entry = maintenance._identity_runtime(base_source), 'run_maintenance'
    elif scope == 'metadata':
        source, entry = connection.runtime_source(base_source), 'check_connection_metadata'
    else:
        source, entry = base_source, 'run_reader'
        if scope == 'joints':
            source = joint_collection_source(source)
    source += ('\nNATIVE_PIN = ' + repr(pin) + '\nNATIVE_PARENT = ' + str(parent_pid)
               + '\n' + CHILD_GUARD)
    if scope in ('imu', 'joints'):
        source += '\n' + NATIVE_READER_HEALTH
        source += ('\n_native_emit = emit\ndef emit(kind,expected,**values):\n'
                   '    values["collection_scope"] = ' + repr(scope) + '\n'
                   '    return _native_emit(kind,expected,**values)\n')
    if scope == 'imu':
        source += '\n' + IMU_TRACKER + '\nStateTracker = ImuCollectionStateTracker\n'
    source += '\nEXPECTED = ' + repr(expected) + '\nraise SystemExit(' + entry + '(EXPECTED))\n'
    raw = source.encode('utf-8')
    child_command(raw, parent_pid=parent_pid)
    compile(source, '<native-transport-source>', 'exec')
    return expected, raw


class NativeLocalChannel:
    """Bounded JSONL channel for one exact local child, without a reopen path."""
    send = maintenance._ProcessChannel.send
    receive = maintenance._ProcessChannel.receive

    def __init__(self, source, directory, known_hosts=None, helper=None, *,
                 popen=subprocess.Popen, clock=time.monotonic, absolute_deadline=None, **unused):
        self.process = None
        self.clock, self.buffers, self.stderr = clock, bytearray(), bytearray()
        self.active = set()
        self.helper = {'write_pipe_once': write_once}
        command = child_command(source)
        self.process = popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, start_new_session=True)
        try:
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                os.set_blocking(stream.fileno(), False)
            self.active = {self.process.stdout.fileno(), self.process.stderr.fileno()}
            transfer_seconds=8. if absolute_deadline is None else max(0.,absolute_deadline-time.monotonic())
            write_once(self.process.stdin.fileno(), source, transfer_seconds)
        except BaseException:
            self.finish()
            raise

    def observe_exit(self):
        code = None if self.process is None else self.process.poll()
        if code is not None:
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                if not stream.closed:
                    stream.close()
        return dict(transport='local', ssh_started=False, process_exited=code is not None,
                    process_exit_code=code, stderr=bytes(self.stderr).decode('utf-8', 'replace'))

    def finish(self):
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.wait(timeout=.25)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    try:
                        self.process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        pass
        return self.observe_exit()


_HOLDER_RUNTIME_CACHE={}
_HOLDER_RUNTIME_LOCK=threading.Lock()
_HOLDER_HELPER_NAMES=('ProbeError','_check_deadline','_holders')


def _holder_source_digest(base_source):
    if type(base_source) is not str:
        raise ValueError('Reviewed holder runtime source is required')
    raw=base_source.encode('utf-8')
    if not 0<len(raw)<=MAX_SOURCE:
        raise ValueError('Reviewed holder runtime source is required')
    return hashlib.sha256(raw).hexdigest()


def prepare_holder_runtime(base_source):
    """Prepare only exact reviewed helper code before a scan's deadline exists.

    Bootstrap calls this before CPU-model/web threads are running. Nothing is
    executed or probed: the cache contains source definitions, never a device
    identity, owner inventory, successful guard result or refreshed deadline.
    """
    digest=_holder_source_digest(base_source)
    with _HOLDER_RUNTIME_LOCK:
        source=_HOLDER_RUNTIME_CACHE.get(digest)
        if source is None:
            tree=ast.parse(base_source)
            blocks=[]
            for name in _HOLDER_HELPER_NAMES:
                matches=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name==name]
                if len(matches)!=1:
                    raise ValueError('Reviewed holder helper is missing or duplicated: '+name)
                blocks.append(ast.get_source_segment(base_source,matches[0]))
            source='import errno,json,os,stat,time\n'+'\n\n'.join(blocks)+'\n'
            # Only fixed reviewed helper definitions are copied, never module
            # entry points, UART openers or the larger guard dependency tree.
            compile(source,'<isolated-holder-definitions>','exec')
            if len(_HOLDER_RUNTIME_CACHE)>=4:
                _HOLDER_RUNTIME_CACHE.clear()
            _HOLDER_RUNTIME_CACHE[digest]=source
    raw=source.encode('utf-8')
    return dict(kind='native_holder_runtime_prepared',base_source_sha256=digest,
        helper_source_sha256=hashlib.sha256(raw).hexdigest(),helper_names=list(_HOLDER_HELPER_NAMES),
        helper_source_bytes=len(raw),serial_open_attempts=0,serial_bytes_written=0,
        owners_cached=False,board_files_written=False,automatic_retry=False)


def _holder_runtime(base_source):
    """Retrieve already prepared definitions; never parse during a scan."""
    digest=_holder_source_digest(base_source)
    with _HOLDER_RUNTIME_LOCK:
        source=_HOLDER_RUNTIME_CACHE.get(digest)
    if source is None:
        raise ValueError('Reviewed holder runtime must be prepared before its absolute scan deadline')
    return source


def _isolated_holder_source(base_source,rdev,deadline,parent):
    source=_holder_runtime(base_source)
    source+='\nNATIVE_HOLDER_RDEV='+repr(rdev)+'\nNATIVE_HOLDER_DEADLINE='+repr(deadline)+'\nNATIVE_HOLDER_PARENT='+repr(parent)+'\n'
    source+=r'''
def isolated_holder_main():
    record=dict(parent_pid=NATIVE_HOLDER_PARENT,rdev=NATIVE_HOLDER_RDEV,
        deadline=NATIVE_HOLDER_DEADLINE,serial_open_attempts=0,serial_bytes_written=0,
        board_files_written=False,automatic_retry=False)
    try:
        _check_deadline(NATIVE_HOLDER_DEADLINE)
        if os.getppid()!=NATIVE_HOLDER_PARENT:
            raise ProbeError('Native holder-check parent disappeared')
        owners=_holders(NATIVE_HOLDER_RDEV,NATIVE_HOLDER_DEADLINE)
        _check_deadline(NATIVE_HOLDER_DEADLINE)
        if os.getppid()!=NATIVE_HOLDER_PARENT:
            raise ProbeError('Native holder-check parent disappeared')
        record.update(kind='holders',owners=owners,observed_monotonic=time.monotonic())
        status=0
    except BaseException as error:
        record.update(kind='fault',reason=(str(error) or type(error).__name__)[:2048],
            observed_monotonic=time.monotonic())
        status=20
    raw=(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n').encode()
    if len(raw)>65536: raise SystemExit(20)
    offset=0
    while offset<len(raw):
        count=os.write(1,raw[offset:])
        if count<=0: raise SystemExit(20)
        offset+=count
    return status
raise SystemExit(isolated_holder_main())
'''
    return source.encode('utf-8')


def _validate_holder_record(value,rdev,deadline,parent):
    fields={'kind','parent_pid','rdev','deadline','serial_open_attempts','serial_bytes_written',
            'board_files_written','automatic_retry','owners','observed_monotonic'}
    if (type(value) is not dict or set(value)!=fields or value['kind']!='holders'
            or type(value['parent_pid']) is not int or value['parent_pid']!=parent
            or type(value['rdev']) is not int or value['rdev']!=rdev
            or type(value['deadline']) is not float or value['deadline']!=deadline
            or type(value['serial_open_attempts']) is not int or value['serial_open_attempts']!=0
            or type(value['serial_bytes_written']) is not int or value['serial_bytes_written']!=0
            or value['board_files_written'] is not False or value['automatic_retry'] is not False
            or type(value['observed_monotonic']) not in (int,float)
            or not math.isfinite(value['observed_monotonic']) or not 0<=value['observed_monotonic']<deadline
            or type(value['owners']) is not list or len(value['owners'])>1024):
        raise ValueError('Isolated holder receipt schema, scope or absolute deadline differs')
    for owner in value['owners']:
        if (type(owner) is not dict or set(owner)!={'pid','fd'} or type(owner['pid']) is not int
                or owner['pid']<1 or type(owner['fd']) is not str or not owner['fd'].isascii()
                or not owner['fd'].isdecimal() or len(owner['fd'])>16 or str(int(owner['fd']))!=owner['fd']):
            raise ValueError('Isolated holder inventory owner schema differs')
    return copy.deepcopy(value['owners'])


def isolated_holders(base_source,rdev,absolute_deadline,directory,*,channel_factory=NativeLocalChannel):
    """One child /proc inventory within the caller's original absolute deadline.

    No UART opener is in this source.  Isolation removes Python thread/GIL
    contention; it neither adds time nor bypasses a missing or partial receipt.
    """
    if (type(rdev) is not int or rdev<0 or type(absolute_deadline) not in (int,float)
            or not math.isfinite(absolute_deadline)):
        raise ValueError('Isolated holder request differs')
    deadline=float(absolute_deadline)
    now=time.monotonic()
    if now>=deadline:
        raise TimeoutError('Original holder-check absolute deadline expired before launch')
    if deadline-now>.200001:
        raise ValueError('Isolated holder deadline exceeds the unchanged 0.2 second scope')
    directory=Path(directory)
    if not os.path.lexists(directory):
        try:
            directory.mkdir(mode=0o700)
        except FileExistsError:
            pass
    directory=_private_directory(directory)
    trace=directory/('holder-check-'+secrets.token_hex(12))
    trace.mkdir(mode=0o700)
    parent=os.getpid()
    source=_isolated_holder_source(base_source,rdev,deadline,parent)
    private_new(trace/'local-source.py',source)
    if time.monotonic()>=deadline:
        raise TimeoutError('Original holder-check absolute deadline expired before Popen')
    channel=None
    record=process=None
    error=None
    try:
        channel=channel_factory(source,trace,absolute_deadline=deadline)
        record=channel.receive(deadline)
        # A line is not an EOF/exit proof.  A second line or missing EOF cannot
        # certify an inventory, including when the first line looks valid.
        try:
            channel.receive(deadline)
        except EOFError:
            pass
        else:
            raise ValueError('Isolated holder child emitted trailing records')
        child=channel.process
        if child is not None and child.poll() is None:
            remaining=deadline-time.monotonic()
            if remaining<=0:
                raise TimeoutError('Original holder-check absolute deadline expired before child exit')
            child.wait(timeout=remaining)
        process=channel.observe_exit()
        if time.monotonic()>=deadline:
            raise TimeoutError('Original holder-check absolute deadline expired before completed receipt')
        if type(record) is dict and record.get('kind')=='fault':
            raise ValueError(record.get('reason','Isolated holder child failed'))
        if (process.get('transport')!='local' or process.get('ssh_started') is not False
                or process.get('process_exited') is not True or process.get('process_exit_code')!=0
                or process.get('stderr')):
            raise ValueError('Isolated holder child lacks a clean actual exit/EOF receipt')
        return _validate_holder_record(record,rdev,deadline,parent)
    except BaseException as failure:
        error=str(failure) or type(failure).__name__
        if isinstance(failure,TimeoutError):
            error='Isolated holder check did not complete within its original absolute deadline: '+error
            raise TimeoutError(error) from failure
        raise
    finally:
        if channel is not None:
            process=channel.finish()
        private_new(trace/'result.json',json.dumps(dict(record=record,process=process,error=error,
            parent_pid=parent,rdev=rdev,absolute_deadline=deadline,
            completed_monotonic=time.monotonic(),source_sha256=hashlib.sha256(source).hexdigest(),
            serial_open_attempts=0,serial_bytes_written=0,board_files_written=False,
            automatic_retry=False),ensure_ascii=False,allow_nan=False,indent=2)+'\n')


class NativeMaintenanceTransport(maintenance.SSHMaintenanceTransport):
    """Reuse wire/record validation with truthful local process receipts."""
    def __init__(self, expected, source, directory, *, validation_source,
                 channel_factory=NativeLocalChannel,
                 clock=time.monotonic):
        self.expected, self.directory, self.clock = copy.deepcopy(expected), _private_directory(directory), clock
        self.generation = expected['generation']
        self.sequence = self.record_count = 0
        self.ready = self.closed_record = self.fault = self.close_report = None
        self.channel = self.error = self.center_diagnostic = None
        self.channel_start_attempted = False
        self.startup_preflight_proof = None
        self.lock = threading.RLock()
        self.source_sha256 = hashlib.sha256(source).hexdigest()
        validation = {}
        exec(compile(maintenance._identity_runtime(validation_source), '<native-maintenance-validation>', 'exec'), validation)
        self.validate_kernel = validation['validate_kernel']
        private_new(self.directory / 'local-source.py', source)
        private_new(self.directory / 'transport-config.json', json.dumps(dict(
            transport='local', ssh_started=False, automatic_retry=False)) + '\n')
        try:
            self.channel_start_attempted = True
            self.channel = channel_factory(source, self.directory)
            value = self._receive(self.clock() + 3.)
            if value.get('kind') != 'ready' or value.get('scope') != 'maintenance':
                raise ValueError('Native maintenance READY is missing')
            if value.get('allowed_requests') != list(maintenance.ALLOWED_REPLIES):
                raise ValueError('Maintenance READY allowlist differs')
            query = value.get('query')
            if (type(query) is not dict or query.get('serial_open_attempts') != 1
                    or query.get('exclusive') is not True or query.get('automatic_retry') is not False
                    or query.get('board_files_written_by_transport') is not False):
                raise ValueError('Maintenance READY ownership proof differs')
            identity = value['identity']
            for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
                if identity[key] != expected[key]:
                    raise ValueError('Maintenance identity differs: ' + key)
            if not reader.serial_receipt_matches(identity.get('serial'), expected):
                raise ValueError('Maintenance identity differs: serial')
            self.validate_kernel(identity['kernel_log'], expected)
            self.ready = value
        except BaseException as error:
            self.error = str(error) or type(error).__name__
            self.close()
            raise maintenance.MaintenanceTransportError(self.error, self) from error

    def close(self):
        proof=super().close()
        if 'transport' not in proof:
            # A constructor failure may have no returned channel object.  It
            # remains uncertain, with no fabricated child/UART closure proof.
            self.close_report.update(transport='local',ssh_started=False,process_exit_code=None)
            self.close_report.pop('ssh_exit_code',None)
            self._persist_close_report()
            proof=copy.deepcopy(self.close_report)
        return proof


class NativeLocalTransports:
    def __init__(self, guard_factory, ram_directory, *, guard_spec_factory=None,
                 channel_factory=NativeLocalChannel, recovery_guard_factory=None):
        if not callable(guard_factory):
            raise ValueError('An explicit native guard factory is required')
        self.guard_factory, self.root = guard_factory, Path(ram_directory)
        self.guard_spec_factory = guard_spec_factory
        self.channel_factory = channel_factory
        if recovery_guard_factory is not None and not callable(recovery_guard_factory):
            raise ValueError('Explicit USB recovery guard factory must be callable')
        self.recovery_guard_factory = recovery_guard_factory

    def _source(self, generation, scope):
        # Every acquisition is explicit. Reconcile a completed USB reconnect
        # before binding readers, including the first reader after reboot.
        factory = self.recovery_guard_factory or self.guard_factory
        namespace = factory()
        spec = None if self.guard_spec_factory is None else self.guard_spec_factory(namespace)
        expected,source=child_source(namespace, self.root, generation, scope=scope, guard_spec=spec)
        return expected,source,namespace['NATIVE_RUNTIME_SOURCE']

    def launch_reader(self, session, generation, trace):
        import feedback_calibration_server as host
        scope = 'joints' if session.feedback_scope is None else session.feedback_scope
        trace = _private_directory(trace)
        preflight_started=time.monotonic()
        try:
            expected, source, _ = self._source(generation, scope)
            private_new(trace / 'local-source.py', source)
            private_new(trace / 'expectations.json', json.dumps(expected, ensure_ascii=False, allow_nan=False) + '\n')
        except BaseException as error:
            with session.lock:
                if session.generation==generation and session.process is None:
                    session._reader_preflight_failure_generation=generation
                    private_new(trace/'local-preflight-failure.json',json.dumps(dict(
                        generation=generation,error=str(error),transport='local',
                        process_started=False,serial_open_attempts=0,automatic_retry=False,
                        stage='native_reader_preflight',
                        preflight_elapsed_seconds=max(0.,time.monotonic()-preflight_started),
                        failure_stack=preflight_stack(error)),
                        ensure_ascii=False,allow_nan=False)+'\n')
            raise
        def preflight(current, raw, known_hosts, directory, actual_generation):
            if current is not session or raw != source or actual_generation != generation or directory != trace:
                raise ValueError('Native reader launch identity changed')
            # The service already fixes its environment before installing its
            # audit hook.  Inherit it; an explicit Popen env is forbidden.
            return child_command(source), None, trace / 'no-ssh-askpass'
        host.run_reader(session, source, Path('/unused-native-known-hosts'), trace,
                        launch_preflight=preflight)

    def check_connection(self, generation, trace):
        trace = _private_directory(trace)
        expected, source, validation_source = self._source(generation, 'metadata')
        private_new(trace / 'local-source.py', source)
        channel = self.channel_factory(source, trace)
        try:
            value = channel.receive(time.monotonic() + 3.5)
            # Require the actual complete stdout EOF and child exit, not just a
            # successful JSON line. Extra records are a protocol failure.
            try:
                extra = channel.receive(time.monotonic() + .5)
            except EOFError:
                extra = None
            if extra is not None:
                raise ValueError('Native metadata child emitted extra records')
            process = channel.finish()
            private_new(trace / 'local-check-result.json',json.dumps(dict(
                record=value,process=process,automatic_retry=False,
                serial_open_attempts=0,closure_not_inferred=True),
                ensure_ascii=False,allow_nan=False,indent=2)+'\n')
            if value.get('kind') == 'fault':
                raise ValueError(value.get('reason', 'Native metadata check failed'))
            if process.get('process_exited') is not True or process.get('process_exit_code') != 0:
                raise ValueError('Native metadata child did not exit normally')
            if process.get('stderr'):
                raise ValueError('Native metadata child reported stderr: ' + process['stderr'][:1024])
            proof = connection._validate_remote(value, expected,validation_source=validation_source)
            proof.update(process, source_sha256=hashlib.sha256(source).hexdigest())
            private_new(trace / 'proof.json', json.dumps(proof, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
            return proof
        finally:
            channel.finish()

    def maintenance_factory(self):
        generation = secrets.randbelow(0x7ffffffe) + 1
        try:
            expected, source, validation_source = self._source(generation, 'maintenance')
        except BaseException as error:
            refusal=maintenance.MaintenancePreflightError(str(error) or type(error).__name__)
            refusal.close_report.update(transport='local',ssh_started=False,
                                        process_exit_code=None,process_exited=True)
            refusal.close_report.pop('ssh_exit_code',None)
            raise refusal from error
        evidence = self.root / 'native-maintenance'
        evidence.mkdir(mode=0o700, exist_ok=True)
        _private_directory(evidence)
        trace = evidence / ('attempt-' + secrets.token_hex(12))
        trace.mkdir(mode=0o700)
        return NativeMaintenanceTransport(expected, source, trace,
                                          validation_source=validation_source,channel_factory=self.channel_factory)
