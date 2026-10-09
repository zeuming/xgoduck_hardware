#!/usr/bin/env python3
"""One reviewed ESP32 upgrade using X5 RAM only; no storage repair or retry.

The old/new RAM guard provides storage evidence only. A separate private upgrade
review, supported body, exact serial binding, pinned payload, and verified Mac
backup must all pass before the single firmware write. No ARM or servo target
commands exist here. SSH uses only its existing authenticated control socket.
"""
import argparse
import ast
import base64
import hashlib
import importlib.util
import inspect
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import selectors
import shlex
import signal
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid
import zipfile

_HELPER_PATH = Path(__file__).with_name('run_ram_model_web.py')
BASE_HELPERS_SHA256 = '5a57a9151e2d397f98aba440969a021ed7a03d60fb5dfdb72173f3ccf2304903'
if hashlib.sha256(_HELPER_PATH.read_bytes()).hexdigest() != BASE_HELPERS_SHA256:
    raise RuntimeError('The reviewed RAM supervisor helpers changed; review before reuse')
_spec = importlib.util.spec_from_file_location('xgoduck_ram_base_helpers', _HELPER_PATH)
base_helpers = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base_helpers)
PINNED_PAYLOAD_SHA256 = 'a9721911a492309511f7378b7cd78fc5a3b9de9c00790dee31be31f26bfcdf8d'
PINNED_PAYLOAD_BYTES = 7722905
PINNED_SEGMENTS = [
    {'offset': 0, 'file': 'bootloader.bin', 'size': 14032, 'sha256': '1033730a6df733f53a7a347353c1c5450547f76e98e0746da633079310a563b9'},
    {'offset': 32768, 'file': 'partitions.bin', 'size': 3072, 'sha256': '13daf80a09a7a4ee743883aed6779c6833ec4880709ce967a5799c0bd38c06e0'},
    {'offset': 57344, 'file': 'boot_app0.bin', 'size': 8192, 'sha256': 'f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0'},
    {'offset': 65536, 'file': 'firmware.bin', 'size': 280576, 'sha256': '7f58df2f9fb6fabe7040f88b81c3f584439adaaee10ddeeafd458af8343a9306'},
]
ALLOWED_WHEELS = {
    'argcomplete-3.7.2-py3-none-any.whl',
    'bitarray-3.11.0-cp310-cp310-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl',
    'bitstring-4.4.0-py3-none-any.whl',
    'cffi-2.1.1-cp310-cp310-manylinux2014_aarch64.manylinux_2_17_aarch64.whl',
    'cryptography-50.0.2-cp39-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl',
    'ecdsa-0.19.2-py2.py3-none-any.whl', 'esptool-4.11.0-py3-none-any.whl',
    'intelhex-2.3.0-py2.py3-none-any.whl', 'pycparser-3.0-py3-none-any.whl',
    'pyserial-3.5-py2.py3-none-any.whl',
    'pyyaml-6.0.3-cp310-cp310-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl',
    'reedsolo-1.7.0-py3-none-any.whl', 'six-1.17.0-py2.py3-none-any.whl',
    'tibs-0.5.7-cp38-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl',
    'typing_extensions-4.16.0-py3-none-any.whl',
}
MAX_EXPANDED_WHEEL_BYTES = base_helpers.MAX_EXPANDED_WHEEL_BYTES
safe_relative_path = base_helpers.safe_relative_path
normalized_kernel_sha256 = base_helpers.normalized_kernel_sha256
verify_root_block_node = base_helpers.verify_root_block_node
inspect_tar = base_helpers.inspect_tar
inspect_wheels = base_helpers.inspect_wheels


def validate_upgrade_review(review):
    if not isinstance(review, dict) or review.get('scope') != 'esp32_ram_firmware_upgrade_once':
        raise ValueError('A separate one-time ESP32 RAM upgrade review is required')
    if (review.get('physical_support_confirmed') is not True
            or review.get('servo_arm_authorized') is not False
            or review.get('servo_motion_authorized_by_this_review') is not False
            or review.get('generic_storage_writes_authorized_by_this_review') is not False):
        raise ValueError('Body support and explicit no-motion/no-storage authority are required')
    if str(uuid.UUID(review['expected_boot_id'])) != review['expected_boot_id']:
        raise ValueError('Invalid boot ID')
    if not re.fullmatch(r'[0-9a-f]{32}', review['expected_cid']):
        raise ValueError('Invalid card CID')
    root = review['expected_root']
    if (set(root) != {'device', 'source', 'filesystem'} or root['device'] != '179:2'
            or root['filesystem'] != 'ext4' or root['source'] not in ('/dev/root', '/dev/mmcblk1p2')):
        raise ValueError('Unexpected root identity')
    if review.get('required_unmounted_device') != '179:1' or not re.fullmatch(r'[0-9a-f]{64}', review['kernel_log_sha256']):
        raise ValueError('Invalid storage binding')
    expected = {'expected_esp32_chip': 'esp32s3', 'expected_esp32_mac': '44:bd:8d:6a:6c:00',
                'expected_esp32_flash_bytes': 16777216, 'affected_range_backup_offset': 0,
                'affected_range_backup_bytes': 393216, 'max_runtime_seconds': 180,
                'payload_sha256': PINNED_PAYLOAD_SHA256, 'payload_size_bytes': PINNED_PAYLOAD_BYTES}
    for key, value in expected.items():
        if type(review.get(key)) is not type(value) or review[key] != value:
            raise ValueError('Upgrade review differs from the fixed operation: ' + key)
    if review.get('firmware_segments') != PINNED_SEGMENTS:
        raise ValueError('Firmware offsets, bytes, and hashes must match all four fixed images')
    serial = review.get('serial_device')
    if (not isinstance(serial, dict) or set(serial) != {'path', 'vid', 'pid', 'serial_number', 'syspath'}
            or serial.get('path') != '/dev/ttyUSB0' or serial.get('vid') != '1a86'
            or serial.get('pid') != '7523' or serial.get('serial_number') != ''
            or serial.get('syspath') != '/sys/devices/platform/soc/35000000.hsio_apb/35100000.usb3/35100000.usb/xhci-hcd.2.auto/usb1/1-1/1-1.3'):
        raise ValueError('Only the physically identified CH340 USB serial port is allowed')
    return review


def validate_payload(data, manifest, review):
    archive = manifest['archive']
    if (len(data) != PINNED_PAYLOAD_BYTES or hashlib.sha256(data).hexdigest() != PINNED_PAYLOAD_SHA256
            or archive.get('size_bytes') != PINNED_PAYLOAD_BYTES
            or archive.get('sha256') != PINNED_PAYLOAD_SHA256 or archive.get('file_count') != 19):
        raise ValueError('Only the pinned 19-member upgrade payload is allowed')
    files = manifest['files']
    allowed = {'wheels/' + name for name in ALLOWED_WHEELS} | {'firmware/' + f['file'] for f in PINNED_SEGMENTS}
    if len(files) != 19 or {f['archive_path'] for f in files} != allowed:
        raise ValueError('Unexpected payload member list')
    table = {}
    for entry in files:
        name = safe_relative_path(entry['archive_path'])
        if name in table or type(entry['size_bytes']) is not int or entry['size_bytes'] <= 0 or not re.fullmatch(r'[0-9a-f]{64}', entry['sha256']):
            raise ValueError('Invalid or duplicated member size/hash')
        table[name] = {'size_bytes': entry['size_bytes'], 'sha256': entry['sha256']}
    for segment in review['firmware_segments']:
        if table['firmware/' + segment['file']] != {'size_bytes': segment['size'], 'sha256': segment['sha256']}:
            raise ValueError('Firmware manifest differs from upgrade review')
    members = inspect_tar(data, table)
    return table, inspect_wheels(members)


def check_storage_guard(path, review, now=None):
    state = base_helpers.read_regular_json(path, private=True)
    scope = state.get('capability_scope')
    capability = {'ram_model_web_only': 'safe_for_ram_model_web',
                  'ram_motion_calibration_only': 'safe_for_ram_motion_calibration'}.get(scope)
    if capability is None or state.get(capability) is not True or state.get('safe') is not False:
        raise RuntimeError('A live RAM-only storage proof is required; upgrade authority comes from its separate review')
    pid = state.get('local_pid')
    if type(pid) is not int or pid <= 1 or pid == os.getpid():
        raise RuntimeError('Invalid guard PID')
    os.kill(pid, 0)
    heartbeat = state.get('last_heartbeat_monotonic')
    if type(heartbeat) not in (float, int) or not 0 <= (time.monotonic() if now is None else now) - heartbeat <= 4:
        raise RuntimeError('Storage guard heartbeat is stale')
    root = state.get('root', {})
    if ({k: root.get(k) for k in ('device', 'source', 'filesystem')} != review['expected_root']
            or 'rw' not in root.get('options', []) or 'ro' in root.get('options', [])
            or 'ro' in root.get('super_options', []) or state.get('boot_id') != review['expected_boot_id']
            or state.get('card_cid') != review['expected_cid']
            or state.get('kernel_log_sha256') != review['kernel_log_sha256']
            or state.get('root_block_device') != {'path': '/dev/mmcblk1p2', 'device': '179:2', 'is_block': True}
            or state.get('swap_entries') != [] or '179:1' in state.get('mounted_devices', ['179:1'])):
        raise RuntimeError('Storage guard identity/mount/swap proof changed')
    return state


def serial_identity(expected):
    path = expected['path']
    node = os.lstat(path)
    if not stat.S_ISCHR(node.st_mode) or os.path.realpath(path) != path:
        raise RuntimeError('Reviewed serial node changed')
    device = os.path.realpath('/sys/class/tty/' + os.path.basename(path) + '/device')
    ancestor = Path(device)
    while str(ancestor) != '/':
        if (ancestor / 'idVendor').is_file() and (ancestor / 'idProduct').is_file():
            break
        ancestor = ancestor.parent
    if str(ancestor) == '/':
        raise RuntimeError('Serial USB ancestor unavailable')
    def read(name):
        file = ancestor / name
        return file.read_text().strip() if file.exists() else ''
    actual = {'path': path, 'vid': read('idVendor'), 'pid': read('idProduct'),
              'serial_number': read('serial'), 'syspath': str(ancestor)}
    if actual != expected:
        raise RuntimeError('Live USB serial identity differs from review')
    return {'identity': actual, 'char_node': [node.st_dev, node.st_ino, node.st_rdev]}


def validate_flash_id(text):
    if (not re.search(r'Chip is ESP32-S3(?:\s|\()', text)
            or not re.search(r'^MAC:\s*44:bd:8d:6a:6c:00\s*$', text, re.M | re.I)
            or not re.search(r'^Detected flash size:\s*16MB\s*$', text, re.M)):
        raise RuntimeError('ESP32-S3 MAC or 16 MB flash identification did not match; no write permitted')


def tool_arguments(stage, directory):
    common = ['--chip', 'esp32s3', '--port', '/dev/ttyUSB0', '--baud', '460800',
              '--connect-attempts', '1', '--before', 'default_reset',
              '--after', 'hard_reset' if stage == 'verify' else 'no_reset']
    images = [word for f in PINNED_SEGMENTS for word in (hex(f['offset']), directory + '/firmware/' + f['file'])]
    if stage == 'identify':
        return common + ['flash_id']
    if stage == 'backup':
        return common + ['read_flash', '0x0', '0x60000', directory + '/affected-range-backup.bin']
    if stage == 'write':
        return common + ['write_flash', '--flash_mode', 'keep', '--flash_size', 'keep', '--flash_freq', 'keep'] + images
    if stage == 'verify':
        return common + ['verify_flash'] + images
    raise ValueError('Unknown ESP32 upgrade stage')


def assert_write_path(path, ram_root, allowed_serial=None):
    resolved = os.path.realpath(os.fsdecode(path))
    if allowed_serial and resolved == allowed_serial:
        return
    if resolved != ram_root and not resolved.startswith(ram_root + '/'):
        raise PermissionError('File writes are restricted to this private RAM directory')


CHILD_WRAPPER = base_helpers.CHILD_PARENT_GUARD + r'''
import json, runpy, stat, sys, fcntl, termios
from pathlib import Path
root = os.environ['XGODUCK_RAM_ROOT']
expected = json.loads(os.environ['XGODUCK_SERIAL_IDENTITY'])
install_write_audit(root, expected['path'])
proof = serial_identity(expected)
import serial
original_open = serial.Serial.open
def checked_open(self):
    if self.port != expected['path']:
        raise PermissionError('Only the reviewed serial port is allowed')
    before = serial_identity(expected)
    original_open(self)
    try:
        info = os.fstat(self.fd)
        if not stat.S_ISCHR(info.st_mode) or [info.st_dev, info.st_ino, info.st_rdev] != before['char_node'] or serial_identity(expected) != before:
            raise RuntimeError('Serial file descriptor identity changed during open')
        fcntl.ioctl(self.fd, termios.TIOCEXCL)
    except BaseException:
        self.close()
        raise
serial.Serial.open = checked_open
import esptool.loader
# Disable image reconnection/rewrite and block retries. The outer watchdog
# remains independent of native serial writes and kills this owned group.
esptool.loader.ESPLoader.WRITE_FLASH_ATTEMPTS = 1
esptool.loader.WRITE_BLOCK_ATTEMPTS = 1
esptool.loader.DEFAULT_OPEN_PORT_ATTEMPTS = 1
import esptool
def gate_operation(operation):
    def checked(esp, args):
        mac = ':'.join(format(byte, '02x') for byte in esp.read_mac())
        capacity = (esp.flash_id() >> 16) & 0xff
        if esp.CHIP_NAME != 'ESP32-S3' or mac != '44:bd:8d:6a:6c:00' or capacity != 24:
            raise esptool.FatalError('Current live ESP32 chip/MAC/16MB identity changed; operation forbidden')
        print('Current live ESP32 identity verified before operation', flush=True)
        return operation(esp, args)
    return checked
for operation_name in ('flash_id', 'read_flash', 'write_flash', 'verify_flash'):
    setattr(esptool, operation_name, gate_operation(getattr(esptool, operation_name)))
sys.argv = ['esptool'] + json.loads(os.environ['XGODUCK_ESPTOOL_ARGS'])
runpy.run_module('esptool', run_name='__main__')
'''

REMOTE_MAIN = r'''
import platform, pwd, shutil, threading
CONFIG = __CONFIG__
stop = threading.Event()
received = threading.Event()
go = threading.Event()
lock = threading.Lock()
child_stop_lock = threading.Lock()
output_lock = threading.Lock()
ram_directory = None
shared = {'last_input': time.monotonic(), 'archive': None, 'error': None, 'process': None,
          'child_stop_confirmed': True, 'stopped_process': None, 'ram_mount_identity': None,
          'ram_inode': None, 'serial_proof': None, 'backup_sha256': None, 'complete': False}
started = time.monotonic()

def emit(event, **fields):
    with output_lock:
        print(json.dumps({'event': event, **fields}), flush=True)

def fail(reason):
    with lock:
        if shared['error'] is None:
            shared['error'] = str(reason)
    stop.set()
    received.set()
    go.set()
    stop_child()

def assert_live():
    if stop.is_set() or shared['error']:
        raise RuntimeError(shared['error'] or 'Upgrade stopped')
    if time.monotonic() - started > 180:
        raise RuntimeError('One-time upgrade exceeded 180 seconds')

def checked_snapshot():
    snapshot()
    proof = serial_identity(CONFIG['review']['serial_device'])
    if shared['serial_proof'] is None:
        shared['serial_proof'] = proof
    elif proof != shared['serial_proof']:
        raise RuntimeError('Serial topology or character-node identity changed')
    return proof

def watch():
    next_check = 0
    while not stop.wait(0.25):
        try:
            assert_live()
            if time.monotonic() - shared['last_input'] > 4:
                raise RuntimeError('Mac heartbeat/data timed out')
            if time.monotonic() >= next_check:
                checked_snapshot()
                next_check = time.monotonic() + 1
        except BaseException as error:
            fail(error)
            return

def read_input():
    try:
        remaining = CONFIG['archive_size']
        data = bytearray()
        while remaining:
            part = os.read(0, min(65536, remaining))
            if not part:
                raise RuntimeError('Mac input closed during archive transfer')
            shared['last_input'] = time.monotonic()
            data.extend(part)
            remaining -= len(part)
            assert_live()
        if hashlib.sha256(data).hexdigest() != CONFIG['archive_sha256']:
            raise RuntimeError('Transferred payload hash mismatch')
        shared['archive'] = data
        received.set()
        buffer = b''
        while not stop.is_set():
            part = os.read(0, 256)
            if not part:
                raise RuntimeError('Mac input closed')
            buffer += part
            if len(buffer) > 256:
                raise RuntimeError('Malformed Mac protocol')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if line == b'HB':
                    shared['last_input'] = time.monotonic()
                elif (line.startswith(b'GO ') and shared['backup_sha256'] is not None
                      and line[3:].decode('ascii') == shared['backup_sha256'] and not go.is_set()):
                    shared['last_input'] = time.monotonic()
                    go.set()
                else:
                    raise RuntimeError('Unexpected Mac protocol command')
    except BaseException as error:
        fail(error)

def run_tool(stage):
    assert_live()
    checked_snapshot()
    arguments = tool_arguments(stage, ram_directory)
    env = {'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8', 'LC_ALL': 'C.UTF-8',
           'HOME': os.environ.get('HOME', ''), 'USER': 'sunrise', 'LOGNAME': 'sunrise',
           'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONNOUSERSITE': '1',
           'PYTHONPATH': ram_directory + '/site', 'TMPDIR': ram_directory + '/tmp',
           'XDG_CACHE_HOME': ram_directory + '/cache', 'XDG_CONFIG_HOME': ram_directory + '/config',
           'XDG_STATE_HOME': ram_directory + '/state', 'XDG_DATA_HOME': ram_directory + '/data',
           'ESPTOOL_CFGFILE': ram_directory + '/esptool.cfg', 'ESPTOOL_OPEN_PORT_ATTEMPTS': '1',
           'XGODUCK_SUPERVISOR_PID': str(os.getpid()), 'XGODUCK_RAM_ROOT': ram_directory,
           'XGODUCK_SERIAL_IDENTITY': json.dumps(CONFIG['review']['serial_device']),
           'XGODUCK_ESPTOOL_ARGS': json.dumps(arguments)}
    emit('stage', stage=stage, arguments=arguments, serial_identity_verified=True)
    with lock:
        assert_live()
        child = subprocess.Popen([sys.executable, '-B', '-S', '-u', '-c', CONFIG['child_wrapper']],
                                 cwd=ram_directory, env=env, stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        shared['process'] = child
        shared['child_stop_confirmed'] = False
        shared['stopped_process'] = None
    output = bytearray()
    selector = selectors.DefaultSelector()
    selector.register(child.stdout, selectors.EVENT_READ)
    while True:
        assert_live()
        for key, _ in selector.select(0.1):
            chunk = os.read(key.fileobj.fileno(), 65536)
            if chunk:
                output.extend(chunk)
                if len(output) > 1024 * 1024:
                    raise RuntimeError('Flasher output exceeded its bound')
                emit('tool_log', stage=stage, text=chunk.decode('utf-8', 'replace'))
            else:
                selector.unregister(child.stdout)
        if child.poll() is not None and not selector.get_map():
            break
    child.stdout.close()
    if child.returncode:
        raise RuntimeError('ESP32 ' + stage + ' failed: ' + str(child.returncode))
    # Verify only this child's group ended before changing stages.
    if not stop_child():
        raise RuntimeError('Flasher child group could not be confirmed stopped')
    checked_snapshot()
    return output.decode('utf-8', 'strict')

def main():
    global ram_directory
    try:
        if os.getuid() == 0 or pwd.getpwuid(os.getuid()).pw_name != 'sunrise':
            raise RuntimeError('Upgrade requires unprivileged sunrise')
        if platform.machine() != 'aarch64' or sys.version_info[:2] != (3, 10):
            raise RuntimeError('Payload requires aarch64 CPython 3.10')
        validate_upgrade_review(CONFIG['review'])
        checked_snapshot()
        space = os.statvfs('/dev/shm')
        required = CONFIG['archive_size'] + CONFIG['member_bytes'] + CONFIG['wheel_bytes'] + 64*1024*1024
        with open('/proc/meminfo') as f:
            available = int(dict(line.split(':', 1) for line in f)['MemAvailable'].split()[0]) * 1024
        if space.f_bavail * space.f_frsize < required or available < required + 128*1024*1024:
            raise RuntimeError('Insufficient RAM for the bounded upgrade')
        ram_directory = tempfile.mkdtemp(prefix='xgoduck-esp32-upgrade-', dir='/dev/shm')
        os.chmod(ram_directory, 0o700)
        info = os.lstat(ram_directory)
        shared['ram_inode'] = (info.st_dev, info.st_ino)
        for name in ('site', 'tmp', 'cache', 'config', 'state', 'data'):
            os.mkdir(ram_directory + '/' + name, 0o700)
        install_write_audit(ram_directory)
        threading.Thread(target=watch, daemon=True).start()
        threading.Thread(target=read_input, daemon=True).start()
        emit('transfer_ready', ram_directory=ram_directory, serial_proof=shared['serial_proof'])
        while not received.wait(0.1):
            assert_live()
        assert_live()
        members = inspect_tar(shared['archive'], CONFIG['files'])
        if inspect_wheels(members) != CONFIG['wheel_bytes']:
            raise RuntimeError('Wheel expansion budget changed')
        for name, content in members.items():
            assert_live()
            destination = ram_directory + '/' + name
            os.makedirs(os.path.dirname(destination), mode=0o700, exist_ok=True)
            with open(destination, 'xb') as f:
                f.write(content)
            os.chmod(destination, 0o600)
        for name, content in members.items():
            if not name.startswith('wheels/'):
                continue
            with zipfile.ZipFile(io.BytesIO(content)) as wheel:
                for item in wheel.infolist():
                    assert_live()
                    destination = ram_directory + '/site/' + safe_relative_path(item.filename)
                    if item.is_dir():
                        os.makedirs(destination, mode=0o700, exist_ok=True)
                        continue
                    os.makedirs(os.path.dirname(destination), mode=0o700, exist_ok=True)
                    with wheel.open(item) as source, open(destination, 'xb') as target:
                        shutil.copyfileobj(source, target, 65536)
                    os.chmod(destination, 0o600)
        shared['archive'] = None
        del members
        with open(ram_directory + '/esptool.cfg', 'x') as f:
            f.write('[esptool]\nwrite_block_attempts=1\nopen_port_attempts=1\n')
        validate_flash_id(run_tool('identify'))
        emit('chip_verified', mac='44:bd:8d:6a:6c:00', flash_bytes=16777216)
        run_tool('backup')
        with open(ram_directory + '/affected-range-backup.bin', 'rb') as f:
            data = f.read(393217)
        if len(data) != 393216:
            raise RuntimeError('Affected-range backup did not contain exactly 0x60000 bytes')
        shared['backup_sha256'] = hashlib.sha256(data).hexdigest()
        emit('backup_begin', offset=0, size_bytes=len(data), sha256=shared['backup_sha256'])
        for index, offset in enumerate(range(0, len(data), 32768)):
            assert_live()
            emit('backup_chunk', index=index, data=base64.b64encode(data[offset:offset+32768]).decode('ascii'))
        emit('backup_complete', sha256=shared['backup_sha256'])
        while not go.wait(0.1):
            assert_live()
        assert_live()
        checked_snapshot()
        emit('flash_write_started', backup_sha256=shared['backup_sha256'])
        run_tool('write')
        run_tool('verify')
        shared['complete'] = True
        emit('verified', firmware_sha256=PINNED_SEGMENTS[-1]['sha256'], hard_reset_after_verify=True)
        return 0
    except BaseException as error:
        fail(error)
        emit('fatal', reason=shared['error'] or str(error), no_retry=True)
        return 2
    finally:
        stop.set()
        child_stopped = stop_child()
        if ram_directory is not None and child_stopped:
            try:
                info = os.lstat(ram_directory)
                shm = os.stat('/dev/shm')
                if (stat.S_ISDIR(info.st_mode) and info.st_uid == os.getuid()
                        and (info.st_dev, info.st_ino) == shared['ram_inode']
                        and shared['ram_mount_identity'][-2:] == (shm.st_dev, shm.st_ino)
                        and not any(x['target'] == ram_directory or x['target'].startswith(ram_directory + '/') for x in parsed_mounts())):
                    shutil.rmtree(ram_directory)
                    emit('cleaned', ram_directory=ram_directory)
            except BaseException as error:
                emit('cleanup_unconfirmed', reason=str(error))
        emit('stopped', child_stopped=child_stopped, success=shared['complete'])

def interrupted(signum, frame):
    shared['error'] = 'Upgrade interrupted by signal %s' % signum
signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGHUP, interrupted)
sys.exit(main())
'''


def install_write_audit(root, allowed_serial=None):
    def audit(event, args):
        if event == 'open' and args and isinstance(args[0], (str, bytes)):
            path = os.path.realpath(os.fsdecode(args[0]))
            if path.startswith(('/dev/tty', '/dev/serial', '/dev/bus/usb', '/dev/mmc', '/dev/sd', '/dev/nvme')) and path != allowed_serial:
                raise PermissionError('Only the explicitly reviewed UART may be opened')
            mode = args[1] if len(args) > 1 else None
            flags = args[2] if len(args) > 2 else 0
            writes = (isinstance(mode, str) and any(c in mode for c in 'wax+')) or (isinstance(flags, int) and flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
            if writes and path != '/dev/null':
                assert_write_path(path, root, allowed_serial)
        if event in ('os.remove', 'os.rmdir', 'os.mkdir', 'os.chmod', 'os.truncate', 'os.utime') and args and isinstance(args[0], (str, bytes)):
            path = os.fsdecode(args[0])
            index = {'os.remove': 1, 'os.rmdir': 1, 'os.mkdir': 2, 'os.chmod': 2, 'os.utime': 3}.get(event)
            descriptor = args[index] if index is not None and len(args) > index else -1
            if not os.path.isabs(path) and isinstance(descriptor, int) and descriptor >= 0:
                directory = os.path.realpath('/proc/self/fd/' + str(descriptor))
                assert_write_path(directory, root)
                path = os.path.join(directory, path)
            assert_write_path(path, root)
        if event in ('os.rename', 'os.link', 'os.symlink'):
            raise PermissionError('Renames and links are forbidden')
    sys.addaudithook(audit)


def _base_remote_functions(names):
    tree = ast.parse(base_helpers.REMOTE_MAIN)
    found = {node.name: ast.get_source_segment(base_helpers.REMOTE_MAIN, node)
             for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names}
    if set(found) != set(names):
        raise RuntimeError('Reviewed base supervisor helper changed')
    return '\n\n'.join(found[name] for name in names)


def remote_source(config):
    functions = (safe_relative_path, normalized_kernel_sha256, verify_root_block_node, inspect_tar, inspect_wheels,
                 validate_upgrade_review, serial_identity, validate_flash_id, tool_arguments, assert_write_path, install_write_audit)
    constants = {'PINNED_PAYLOAD_SHA256': PINNED_PAYLOAD_SHA256, 'PINNED_PAYLOAD_BYTES': PINNED_PAYLOAD_BYTES,
                 'PINNED_SEGMENTS': PINNED_SEGMENTS, 'MAX_EXPANDED_WHEEL_BYTES': MAX_EXPANDED_WHEEL_BYTES}
    return ('import base64,hashlib,io,json,os,re,selectors,signal,stat,subprocess,sys,tarfile,tempfile,time,uuid,zipfile\nfrom pathlib import Path,PurePosixPath\n'
            + '\n'.join(name + '=' + repr(value) for name, value in constants.items()) + '\n'
            + '\n'.join(inspect.getsource(f) for f in functions) + '\n'
            + _base_remote_functions(('parsed_mounts', 'snapshot', 'stop_child')) + '\n'
            + REMOTE_MAIN.replace('__CONFIG__', repr(config), 1))

class BackupReceiver:
    def __init__(self, directory):
        self.path = directory / 'esp32-affected-range-0-0x60000.bin'
        self.file = None
        self.size = 0
        self.index = 0
        self.expected_sha256 = None
        self.digest = hashlib.sha256()
        self.confirmed = False

    def begin(self, event):
        if self.file is not None or event.get('offset') != 0 or event.get('size_bytes') != 393216 or not re.fullmatch(r'[0-9a-f]{64}', event.get('sha256', '')):
            raise RuntimeError('Invalid or repeated backup metadata')
        self.expected_sha256 = event['sha256']
        fd = os.open(str(self.path), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self.file = os.fdopen(fd, 'wb')

    def chunk(self, event):
        if self.file is None or self.confirmed or type(event.get('index')) is not int or event['index'] != self.index:
            raise RuntimeError('Out-of-order backup chunk')
        data = base64.b64decode(event['data'], validate=True)
        if len(data) != 32768 or self.size + len(data) > 393216:
            raise RuntimeError('Invalid backup chunk length')
        self.file.write(data)
        self.digest.update(data)
        self.size += len(data)
        self.index += 1

    def complete(self, event):
        if (self.file is None or self.confirmed or self.size != 393216
                or event.get('sha256') != self.expected_sha256 or self.digest.hexdigest() != self.expected_sha256):
            raise RuntimeError('Mac backup length/SHA256 did not match; GO is forbidden')
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        self.file = None
        info = self.path.lstat()
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o600
                or info.st_size != 393216 or hashlib.sha256(self.path.read_bytes()).hexdigest() != self.expected_sha256):
            raise RuntimeError('Saved private Mac backup failed independent read-back verification')
        self.confirmed = True
        return self.expected_sha256

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None


def _private_output(value):
    path = Path(value)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError('Output must be an absolute private Mac directory')
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = path.stat()
    if info.st_uid != os.geteuid() or info.st_mode & 0o077 or any(path.iterdir()):
        raise ValueError('Output must be user-owned 0700 and empty')
    return path


def run(args):
    review = base_helpers.read_regular_json(args.upgrade_review, private=True)
    validate_upgrade_review(review)
    connection = base_helpers.read_regular_json(args.connection_json, private=True)
    manifest = base_helpers.read_regular_json(args.manifest, private=True)
    bundle = Path(args.bundle)
    if not bundle.is_absolute() or bundle.is_symlink() or not bundle.is_file():
        raise ValueError('Bundle must be an absolute regular local file')
    data = bundle.read_bytes()
    files, wheel_bytes = validate_payload(data, manifest, review)
    command = base_helpers.ssh_base(connection)
    check_storage_guard(args.guard_status, review)
    directory = _private_output(args.output_dir)
    backup = BackupReceiver(directory)
    fd = os.open(str(directory / 'events.jsonl'), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    events = os.fdopen(fd, 'w', encoding='utf-8', buffering=1)
    config = {'review': review, 'archive_sha256': PINNED_PAYLOAD_SHA256, 'archive_size': len(data),
              'files': files, 'wheel_bytes': wheel_bytes, 'member_bytes': sum(f['size_bytes'] for f in files.values()),
              'child_wrapper': child_wrapper_source()}
    source = remote_source(config)
    compile(source, '<reviewed-esp32-ram-upgrade>', 'exec')
    process = None
    selector = None
    state = {'scope': review['scope'], 'state': 'starting', 'success': False, 'flash_write_started': False,
             'backup_verified_on_mac': False, 'remote_stop_confirmed': False, 'local_pid': os.getpid()}
    def status():
        descriptor, temporary = tempfile.mkstemp(prefix='.status-', dir=str(directory))
        with os.fdopen(descriptor, 'w') as f:
            json.dump(state, f, indent=2)
            f.write('\n')
        os.replace(temporary, directory / 'status.json')
    def handle(event):
        kind = event.get('event')
        if kind == 'backup_chunk':
            backup.chunk(event)
            events.write(json.dumps({'event': kind, 'index': event['index']}) + '\n')
            return None
        events.write(json.dumps(event) + '\n')
        if kind == 'backup_begin':
            backup.begin(event)
        elif kind == 'backup_complete':
            digest = backup.complete(event)
            check_storage_guard(args.guard_status, review)
            state.update(backup_verified_on_mac=True, backup_path=str(backup.path), backup_sha256=digest)
            status()
            print('Affected-range backup verified on the Mac; the one-time firmware write is now permitted.', flush=True)
            return ('GO ' + digest + '\n').encode()
        elif kind == 'flash_write_started':
            if not backup.confirmed or event.get('backup_sha256') != backup.expected_sha256:
                raise RuntimeError('Remote write was requested without the verified Mac backup')
            state['flash_write_started'] = True
        elif kind == 'stage':
            state['stage'] = event['stage']
            print('ESP32 stage: ' + event['stage'], flush=True)
        elif kind == 'verified':
            state['firmware_verified'] = event.get('firmware_sha256') == PINNED_SEGMENTS[-1]['sha256'] and event.get('hard_reset_after_verify') is True
        elif kind == 'stopped':
            state['remote_stop_confirmed'] = event.get('child_stopped') is True
            state['remote_success'] = event.get('success') is True
        elif kind == 'fatal':
            raise RuntimeError(event.get('reason', 'Remote upgrade failed'))
        elif kind not in ('transfer_ready', 'chip_verified', 'tool_log', 'cleaned', 'cleanup_unconfirmed'):
            raise RuntimeError('Unexpected remote upgrade event')
        status()
        return None
    status()
    try:
        process = subprocess.Popen(command + [connection['target'], 'python3 -B -S -u -c ' + shlex.quote(source)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        os.set_blocking(process.stdin.fileno(), False)
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, 'stdout')
        selector.register(process.stderr, selectors.EVENT_READ, 'stderr')
        output = b''
        queued = bytearray()
        offset = 0
        ready = False
        started = time.monotonic()
        next_guard = started
        next_hb = started
        state['state'] = 'running'
        status()
        while True:
            now = time.monotonic()
            if now - started > 180:
                raise RuntimeError('One-time operation exceeded 180 seconds')
            if now >= next_guard:
                check_storage_guard(args.guard_status, review)
                next_guard = now + 0.25
            if ready and offset < len(data) and not queued:
                queued.extend(data[offset:offset + 65536])
                offset += min(65536, len(data) - offset)
            elif ready and offset == len(data) and now >= next_hb:
                queued.extend(b'HB\n')
                next_hb = now + 1
            if queued:
                try:
                    count = os.write(process.stdin.fileno(), queued)
                    del queued[:count]
                except BlockingIOError:
                    pass
            for key, _ in selector.select(0.01):
                chunk = os.read(key.fileobj.fileno(), 65536)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                if key.data == 'stderr':
                    raise RuntimeError('SSH stderr: ' + chunk.decode('utf-8', 'replace').strip())
                output += chunk
                if len(output) > 1048576:
                    raise RuntimeError('Malformed oversized remote event')
                while b'\n' in output:
                    raw, output = output.split(b'\n', 1)
                    event = json.loads(raw.decode('utf-8'))
                    if event.get('event') == 'transfer_ready':
                        if ready:
                            raise RuntimeError('Repeated transfer-ready event')
                        ready = True
                    response = handle(event)
                    if response:
                        queued.extend(response)
            if process.poll() is not None and not selector.get_map():
                break
        if (process.returncode != 0 or not state.get('firmware_verified') or not state.get('remote_success')
                or not state['remote_stop_confirmed'] or not backup.confirmed):
            raise RuntimeError('Upgrade did not finish with verified firmware and confirmed child stop')
        state.update(state='complete', success=True)
        status()
        print('ESP32 firmware verified; its supported robot starts disarmed.', flush=True)
        return 0
    except BaseException as error:
        state.update(state='stopped', reason=str(error) or 'Interrupted', success=False)
        if process is not None:
            try:
                process.stdin.close()
            except BaseException:
                pass
            # Do not maintain heartbeats after a fault. EOF triggers remote kill;
            # keep receiving only bounded stop evidence instead of claiming it.
            deadline = time.monotonic() + 8
            pending = b''
            while (selector is not None and time.monotonic() < deadline
                   and (process.poll() is None or selector.get_map())):
                for key, _ in selector.select(0.1):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    if key.data != 'stdout':
                        continue
                    pending += chunk
                    if len(pending) > 1048576:
                        pending = b''
                    while b'\n' in pending:
                        raw, pending = pending.split(b'\n', 1)
                        try:
                            event = json.loads(raw)
                            events.write(json.dumps({'event': 'backup_chunk', 'index': event.get('index')}
                                                    if event.get('event') == 'backup_chunk' else event) + '\n')
                            if event.get('event') == 'stopped':
                                state['remote_stop_confirmed'] = event.get('child_stopped') is True
                        except BaseException:
                            pass
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        status()
        print('STOP: ' + state['reason'] + '; no automatic retry.', file=sys.stderr, flush=True)
        return 2
    finally:
        backup.close()
        if process is not None:
            for stream in (process.stdin, process.stdout, process.stderr):
                try:
                    stream.close()
                except BaseException:
                    pass
        events.close()


def child_wrapper_source():
    helpers = 'import os,stat,sys\nfrom pathlib import Path\n' + '\n'.join(inspect.getsource(f) for f in (serial_identity, assert_write_path, install_write_audit))
    return helpers + '\n' + CHILD_WRAPPER


def self_test():
    import unittest
    from unittest.mock import patch
    class Tests(unittest.TestCase):
        def test_fixed_commands(self):
            for stage in ('identify', 'backup', 'write', 'verify'):
                args = tool_arguments(stage, '/dev/shm/test')
                self.assertNotIn('erase_flash', args)
                self.assertNotIn('--force', args)
                self.assertNotIn('--encrypt', args)
            self.assertEqual(tool_arguments('write', '/dev/shm/test').count('keep'), 3)
            self.assertEqual(tool_arguments('verify', '/dev/shm/test')[tool_arguments('verify', '/dev/shm/test').index('--after')+1], 'hard_reset')
            for stage in ('identify', 'backup', 'write'):
                args = tool_arguments(stage, '/dev/shm/test')
                self.assertEqual(args[args.index('--after')+1], 'no_reset')
        def test_mac_gate(self):
            good = 'Chip is ESP32-S3 (revision v0.2)\nMAC: 44:bd:8d:6a:6c:00\nDetected flash size: 16MB\n'
            validate_flash_id(good)
            for text in (good.replace('ESP32-S3', 'ESP32'), good.replace('6a:6c:00', '6a:6c:01'), good.replace('16MB', '8MB')):
                with self.assertRaises(RuntimeError):
                    validate_flash_id(text)
        def test_live_connection_gate_precedes_each_operation(self):
            from types import SimpleNamespace
            function = next(node for node in ast.parse(CHILD_WRAPPER).body
                            if isinstance(node, ast.FunctionDef) and node.name == 'gate_operation')
            namespace = {'esptool': SimpleNamespace(FatalError=RuntimeError)}
            exec(compile(ast.Module(body=[function], type_ignores=[]), '<mock-live-chip-gate>', 'exec'), namespace)
            called = []
            operation = namespace['gate_operation'](lambda esp, args: called.append(True))
            good = SimpleNamespace(CHIP_NAME='ESP32-S3', read_mac=lambda: (0x44,0xbd,0x8d,0x6a,0x6c,0),
                                   flash_id=lambda: (24 << 16) | 0x40ef)
            operation(good, None)
            self.assertEqual(called, [True])
            for bad in (SimpleNamespace(**{**good.__dict__, 'CHIP_NAME':'ESP32'}),
                        SimpleNamespace(**{**good.__dict__, 'read_mac':lambda: (0,0,0,0,0,0)}),
                        SimpleNamespace(**{**good.__dict__, 'flash_id':lambda: (23 << 16) | 0x40ef})):
                with self.assertRaises(RuntimeError):
                    operation(bad, None)
            self.assertEqual(called, [True])
        def test_remote_stage_executes_an_innocuous_local_child(self):
            # Execute the actual generated supervisor namespace but replace the
            # hardware-bound health proof and tool child with local mocks.
            with tempfile.TemporaryDirectory() as temporary:
                config = {'review': {'serial_device': {}}, 'child_wrapper': "print('mock flasher child', flush=True)"}
                tree = ast.parse(remote_source(config))
                safe_nodes = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef, ast.Assign))]
                namespace = {}
                exec(compile(ast.Module(body=safe_nodes, type_ignores=[]), '<local-mock-supervisor>', 'exec'), namespace)
                namespace.update(ram_directory=temporary, checked_snapshot=lambda: {}, assert_live=lambda: None,
                                 stop_child=lambda: True, emit=lambda *args, **kwargs: None)
                result = namespace['run_tool']('identify')
                self.assertIn('mock flasher child', result)
                self.assertEqual(namespace['shared']['process'].returncode, 0)
        def test_audit_blocks_real_outside_writes_in_a_local_subprocess(self):
            with tempfile.TemporaryDirectory() as temporary:
                root = os.path.realpath(temporary + '/ram')
                os.mkdir(root)
                outside = os.path.realpath(temporary + '/forbidden')
                source = 'import os,sys\n' + inspect.getsource(assert_write_path) + '\n' + inspect.getsource(install_write_audit)
                source += '\ninstall_write_audit(' + repr(root) + ')\n'
                source += "with open(" + repr(root + '/inside') + ", 'wb') as f: f.write(b'ok')\n"
                source += "try:\n open(" + repr(outside) + ", 'wb')\n raise RuntimeError('Outside write unexpectedly passed')\nexcept PermissionError:\n pass\n"
                source += "os.unlink(" + repr(root + '/inside') + ")\n"
                result = subprocess.run([sys.executable, '-B', '-S', '-c', source], capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr.decode())
                self.assertFalse(Path(outside).exists())
        def test_backup_go_requires_exact_saved_readback(self):
            data = bytes(range(256)) * 1536
            digest = hashlib.sha256(data).hexdigest()
            with tempfile.TemporaryDirectory() as temporary:
                receiver = BackupReceiver(Path(temporary))
                with self.assertRaises(RuntimeError):
                    receiver.complete({'sha256': digest})
                receiver.begin({'offset': 0, 'size_bytes': 393216, 'sha256': digest})
                with self.assertRaises(RuntimeError):
                    receiver.chunk({'index': 1, 'data': base64.b64encode(data[:32768]).decode()})
                for index, offset in enumerate(range(0, len(data), 32768)):
                    receiver.chunk({'index': index, 'data': base64.b64encode(data[offset:offset+32768]).decode()})
                self.assertEqual(receiver.complete({'sha256': digest}), digest)
                self.assertEqual(receiver.path.stat().st_mode & 0o777, 0o600)
                self.assertEqual(receiver.path.read_bytes(), data)
                with self.assertRaises(RuntimeError):
                    receiver.complete({'sha256': digest})
        def test_writes_cannot_escape_ram(self):
            with tempfile.TemporaryDirectory() as temporary:
                root = os.path.realpath(temporary)
                assert_write_path(root + '/image.bin', root)
                for path in ('/tmp/image.bin', '/home/sunrise/.cache/a', root + '/../../a'):
                    with self.assertRaises(PermissionError):
                        assert_write_path(path, root)
        def test_real_local_payload_and_generated_source(self):
            run_path = Path(__file__).resolve().parents[2] / 'output/x5_recovery/new-board-20261003T114727Z'
            review_path = run_path / 'esp32-ram-upgrade-review.json'
            if not review_path.exists():
                self.skipTest('Private real review is not present')
            review = json.loads(review_path.read_text())
            validate_upgrade_review(review)
            manifest = json.loads((run_path / 'prepared/esp32-motion-ram-upgrade.manifest.json').read_text())
            data = (run_path / 'prepared/esp32-motion-ram-upgrade.tar.gz').read_bytes()
            files, wheel_bytes = validate_payload(data, manifest, review)
            self.assertEqual(len(files), 19)
            self.assertLess(wheel_bytes, 160*1024*1024)
            config = {'review': review, 'archive_size': len(data), 'archive_sha256': PINNED_PAYLOAD_SHA256,
                      'files': files, 'wheel_bytes': wheel_bytes, 'member_bytes': sum(v['size_bytes'] for v in files.values()),
                      'child_wrapper': child_wrapper_source()}
            compile(remote_source(config), '<mock-remote>', 'exec')
            compile(child_wrapper_source(), '<mock-child>', 'exec')
            for field, value in (('physical_support_confirmed', False), ('servo_arm_authorized', True), ('expected_esp32_mac', '44:bd:8d:6a:6c:01'), ('affected_range_backup_bytes', 393215), ('max_runtime_seconds', 181)):
                changed = {**review, field: value}
                with self.assertRaises(ValueError):
                    validate_upgrade_review(changed)
        def test_both_guard_scopes_are_storage_proof_only_and_stale_denied(self):
            review = {'expected_boot_id': str(uuid.uuid4()), 'expected_cid': 'f'*32,
                      'expected_root': {'device': '179:2', 'source': '/dev/root', 'filesystem': 'ext4'}, 'kernel_log_sha256': 'a'*64}
            state = {'safe': False, 'local_pid': 1234, 'last_heartbeat_monotonic': 10,
                     'root': {**review['expected_root'], 'options':['rw'], 'super_options':['rw']},
                     'boot_id': review['expected_boot_id'], 'card_cid': review['expected_cid'],
                     'kernel_log_sha256': review['kernel_log_sha256'],
                     'root_block_device': {'path':'/dev/mmcblk1p2','device':'179:2','is_block':True},
                     'swap_entries':[], 'mounted_devices':['179:2']}
            for scope, cap in (('ram_model_web_only','safe_for_ram_model_web'), ('ram_motion_calibration_only','safe_for_ram_motion_calibration')):
                state.update(capability_scope=scope, **{cap:True})
                with patch.object(base_helpers,'read_regular_json',return_value=state), patch.object(os,'kill'):
                    check_storage_guard('/unused',review,now=11)
                    with self.assertRaises(RuntimeError):
                        check_storage_guard('/unused',review,now=15)
                    with patch.object(base_helpers,'read_regular_json',return_value={**state,'safe':True}):
                        with self.assertRaises(RuntimeError):
                            check_storage_guard('/unused',review,now=11)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    return 0 if result.wasSuccessful() else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('connection-json','guard-status','upgrade-review','bundle','manifest','output-dir'):
        parser.add_argument('--'+name)
    parser.add_argument('--self-test',action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not all(getattr(args,name) for name in ('connection_json','guard_status','upgrade_review','bundle','manifest','output_dir')):
        parser.error('All six fixed input/output options are required')
    def interrupted(signum, frame):
        raise KeyboardInterrupt('Upgrade interrupted by signal %s' % signum)
    signal.signal(signal.SIGTERM,interrupted)
    signal.signal(signal.SIGHUP,interrupted)
    try:
        return run(args)
    except BaseException as error:
        print('STOP before hardware: '+str(error),file=sys.stderr)
        return 2

if __name__ == '__main__':
    sys.exit(main())
