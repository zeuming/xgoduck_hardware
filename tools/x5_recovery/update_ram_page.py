#!/usr/bin/env python3
"""Bounded, guarded replacement of one live RAM HTML page; no serial access."""
import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import stat
import subprocess
import sys
import time
import uuid

import run_ram_model_web as base
import run_ram_motion_web as calibration

MAX_HTML_BYTES = 256 * 1024
normalized_kernel_sha256 = base.normalized_kernel_sha256
verify_root_block_node = base.verify_root_block_node
serial_device_proof = calibration.serial_device_proof


def private_bytes(path, maximum=MAX_HTML_BYTES):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError('An absolute private regular input is required')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or stat.S_IMODE(before.st_mode) != 0o600 or before.st_nlink != 1
                or not 0 < before.st_size <= maximum):
            raise ValueError('Input must be private 0600, regular, one link and bounded')
        value = os.read(fd, maximum + 1)
        after = os.fstat(fd)
        if file_identity(before) != file_identity(after) or len(value) != before.st_size:
            raise ValueError('Input changed while reading')
        return value
    finally:
        os.close(fd)


def file_identity(info):
    return (info.st_dev, info.st_ino, info.st_uid, stat.S_IMODE(info.st_mode),
            info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def validate_ram_path(path):
    if not isinstance(path, str) or not re.fullmatch(r'/dev/shm/xgoduck-model-[a-z0-9_]{8}', path):
        raise ValueError('Only the currently supervised exclusive RAM directory is allowed')
    return path


class PageBinding:
    """Hold directory descriptors and immutable identities until atomic replace."""
    def __init__(self, directory, uid, expected_old_sha256):
        self.directory, self.uid, self.expected_old_sha256 = directory, uid, expected_old_sha256
        self.root_fd = self.policy_fd = None
        try:
            self.root_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            self.root_identity = self.directory_identity(os.fstat(self.root_fd))
            self.policy_fd = os.open('policy', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=self.root_fd)
            self.policy_identity = self.directory_identity(os.fstat(self.policy_fd))
            self.old_identity = self.index_identity()
        except BaseException:
            self.close()
            raise

    def directory_identity(self, info):
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != self.uid or stat.S_IMODE(info.st_mode) != 0o700:
            raise RuntimeError('RAM directories must remain private, owned and non-symlink')
        return (info.st_dev, info.st_ino, info.st_uid, stat.S_IMODE(info.st_mode))

    def index_identity(self):
        fd = os.open('index.html', os.O_RDONLY | os.O_NOFOLLOW, dir_fd=self.policy_fd)
        try:
            info = os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != self.uid or stat.S_IMODE(info.st_mode) != 0o600
                    or info.st_nlink != 1 or not 0 < info.st_size <= MAX_HTML_BYTES):
                raise RuntimeError('The current RAM index must be a private bounded regular file')
            data = os.read(fd, MAX_HTML_BYTES + 1)
            if (len(data) != info.st_size or file_identity(info) != file_identity(os.fstat(fd))
                    or hashlib.sha256(data).hexdigest() != self.expected_old_sha256):
                raise RuntimeError('The current HTML changed or differs from the reviewed old SHA')
            return file_identity(info)
        finally:
            os.close(fd)

    def check(self):
        if self.directory_identity(os.lstat(self.directory)) != self.root_identity:
            raise RuntimeError('RAM directory was replaced')
        if self.directory_identity(os.stat('policy', dir_fd=self.root_fd, follow_symlinks=False)) != self.policy_identity:
            raise RuntimeError('RAM policy directory was replaced')
        if self.index_identity() != self.old_identity:
            raise RuntimeError('RAM index inode changed')

    def close(self):
        for attr in ('policy_fd', 'root_fd'):
            fd = getattr(self, attr, None)
            if fd is not None:
                os.close(fd)
                setattr(self, attr, None)


def snapshot(review):
    mounts = []
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        fields = line.split(); sep = fields.index('-', 6)
        mounts.append({'mount_id': fields[0], 'device': fields[2], 'target': fields[4], 'options': fields[5].split(','),
                       'filesystem': fields[sep + 1], 'source': fields[sep + 2], 'super_options': fields[sep + 3].split(',')})
    roots, rams = [x for x in mounts if x['target'] == '/'], [x for x in mounts if x['target'] == '/dev/shm']
    if len(roots) != 1 or len(rams) != 1:
        raise RuntimeError('Root/RAM mounts are not unique')
    root, ram = roots[0], rams[0]
    if ({key: root[key] for key in ('device', 'source', 'filesystem')} != review['expected_root']
            or 'rw' not in root['options'] or 'ro' in root['options'] + root['super_options']):
        raise RuntimeError('Root filesystem identity or writable state changed')
    verify_root_block_node(os.stat('/dev/mmcblk1p2', follow_symlinks=False))
    if (ram['filesystem'] != 'tmpfs' or 'rw' not in ram['options']
            or any(value in ram['options'] + ram['super_options'] for value in ('ro', 'noexec'))
            or not stat.S_ISDIR(os.lstat('/dev/shm').st_mode)):
        raise RuntimeError('RAM mount is no longer safe writable tmpfs')
    if any(x['device'] == review['required_unmounted_device'] for x in mounts):
        raise RuntimeError('CONFIG is mounted')
    if len(Path('/proc/swaps').read_text().splitlines()) != 1:
        raise RuntimeError('Swap is active')
    if (Path('/proc/sys/kernel/random/boot_id').read_text().strip() != review['expected_boot_id']
            or Path('/sys/block/mmcblk1/device/cid').read_text().strip() != review['expected_cid']):
        raise RuntimeError('Board boot/card identity changed')
    kernel = subprocess.run(['dmesg', '--color=never'], capture_output=True, timeout=.35)
    if (kernel.returncode or kernel.stderr or normalized_kernel_sha256(kernel.stdout.decode('utf-8', 'strict')) != review['kernel_log_sha256']):
        raise RuntimeError('Kernel evidence changed')
    serial_device_proof(review['serial_device'])
    info = os.lstat('/dev/shm')
    return (ram['mount_id'], ram['device'], ram['source'], info.st_dev, info.st_ino)


def process_identity(pid):
    path = Path('/proc') / str(pid)
    if os.stat(path, follow_symlinks=False).st_uid != 1000:
        raise RuntimeError('RAM page process owner changed')
    text = (path / 'stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    if fields[0] == 'Z':
        raise RuntimeError('RAM page process has exited')
    return (int(pid), int(fields[1]), fields[19])


def find_page_process(config):
    matches = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            if os.stat('/proc/' + entry, follow_symlinks=False).st_uid != 1000:
                continue
            with open('/proc/' + entry + '/environ', 'rb') as stream:
                raw = stream.read(65537)
            if len(raw) > 65536:
                continue
            env = dict(item.split(b'=', 1) for item in raw.split(b'\0') if b'=' in item)
            if (env.get(b'XGODUCK_RAM_ROOT') != config['directory'].encode()
                    or env.get(b'XGODUCK_POLICY_DIR') != (config['directory'] + '/policy').encode()
                    or env.get(b'XGODUCK_REMOTE_PORT') != str(config['remote_port']).encode()):
                continue
            identity = process_identity(int(entry))
            parent = int(env[b'XGODUCK_SUPERVISOR_PID'])
            if identity[1] != parent:
                raise RuntimeError('RAM page supervisor parent changed')
            matches.append((identity, process_identity(parent)))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(matches) != 1:
        raise RuntimeError('Exactly one live supervised page must own this RAM directory')
    return matches[0]


def live_page_status(config):
    import http.client
    connection = http.client.HTTPConnection('127.0.0.1', config['remote_port'], timeout=.25)
    try:
        connection.request('GET', '/api/motion/status', headers={'Host': '127.0.0.1:' + str(config['remote_port'])})
        response = connection.getresponse()
        raw = response.read(65537)
        if response.status != 200 or len(raw) > 65536:
            raise RuntimeError('Live calibration HTTP status is unavailable')
        state = json.loads(raw)
    finally:
        connection.close()
    if (state.get('generation') != config['generation'] or type(state.get('generation')) is not int
            or state.get('mode') != 'off' or state.get('armed') is not False or state.get('motor_output') is not False
            or state.get('connected') is not True or state.get('paused') is not False
            or state.get('pending_stop') is not False or state.get('fault_ids') != [] or state.get('torque_ids') != []
            or state.get('online_ids') != [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
            or state.get('imu_feedback_ok') is not True
            or any(type(state.get(key)) not in (int, float) or not 0 <= state[key] < 100 for key in ('age_ms', 'imu_age_ms'))
            or state.get('imu_wizard', {}).get('busy') is not False
            or state.get('imu_wizard', {}).get('stage') not in ('idle', 'complete')
            or state.get('imu', {}).get('calibrating') is not False):
        raise RuntimeError('Live page is not idle, fresh, released and bound to the expected generation')
    return state


def page_transaction(config, read_exact, read_line, emit, gate):
    binding, temporary, fd, committed = None, None, None, False
    try:
        gate()
        binding = PageBinding(config['directory'], config['uid'], config['old_sha256'])
        binding.check()
        emit({'phase': 'READY'})
        data = read_exact(config['new_size'])
        if len(data) != config['new_size'] or hashlib.sha256(data).hexdigest() != config['new_sha256']:
            raise RuntimeError('New HTML transfer length/hash mismatch')
        data.decode('utf-8', 'strict')
        temporary = '.page-update-' + uuid.uuid4().hex
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=binding.policy_fd)
        os.fchmod(fd, 0o600)
        if os.fstat(fd).st_dev != binding.policy_identity[0]:
            raise RuntimeError('New HTML temporary file is outside the pinned RAM filesystem')
        offset = 0
        while offset < len(data):
            offset += os.write(fd, data[offset:])
        os.close(fd); fd = None
        gate(); binding.check()
        emit({'phase': 'PREPARED', 'new_sha256': config['new_sha256']})
        if read_line() != b'COMMIT\n':
            raise RuntimeError('Host declined the guarded commit')
        gate(); binding.check()
        os.replace(temporary, 'index.html', src_dir_fd=binding.policy_fd, dst_dir_fd=binding.policy_fd)
        committed = True; temporary = None
        final_fd = os.open('index.html', os.O_RDONLY | os.O_NOFOLLOW, dir_fd=binding.policy_fd)
        try:
            info = os.fstat(final_fd)
            final_sha = hashlib.sha256(os.read(final_fd, MAX_HTML_BYTES + 1)).hexdigest()
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != config['uid'] or stat.S_IMODE(info.st_mode) != 0o600
                    or info.st_nlink != 1 or info.st_dev != binding.policy_identity[0]
                    or final_sha != config['new_sha256'] or info.st_size != len(data)):
                raise RuntimeError('Final RAM HTML cannot be confirmed')
        finally:
            os.close(final_fd)
        emit({'phase': 'DONE', 'committed': True, 'old_sha256': config['old_sha256'],
              'new_sha256': final_sha, 'bytes': len(data),
              'ram_identity': list(binding.root_identity), 'policy_identity': list(binding.policy_identity)})
    finally:
        if fd is not None:
            os.close(fd)
        if binding is not None:
            if temporary is not None:
                os.unlink(temporary, dir_fd=binding.policy_fd)
            binding.close()


def remote_source(config):
    functions = (file_identity, PageBinding, snapshot, process_identity, find_page_process, live_page_status, page_transaction)
    helpers = (base.normalized_kernel_sha256, base.verify_root_block_node,
               calibration.validate_serial_review, calibration.serial_device_proof)
    source = ('import hashlib,json,os,re,signal,stat,subprocess,sys,time,uuid\nfrom pathlib import Path,PurePosixPath\n'
              'MAX_HTML_BYTES=262144\nCONFIG=' + repr(config) + '\n'
              + '\n'.join(inspect.getsource(f) for f in helpers + functions))
    source += r'''
def emit(value):
    print(json.dumps(value,ensure_ascii=False),flush=True)
def timed_out(signum,frame):
    raise TimeoutError('RAM page transaction exceeded 1.5 seconds')
signal.signal(signal.SIGALRM,timed_out)
signal.signal(signal.SIGTERM,timed_out)
signal.signal(signal.SIGHUP,timed_out)
signal.setitimer(signal.ITIMER_REAL,1.5)
def read_exact(count):
    value=bytearray()
    while len(value)<count:
        chunk=os.read(0,min(65536,count-len(value)))
        if not chunk: raise RuntimeError('Host transfer closed')
        value.extend(chunk)
    return bytes(value)
def read_line():
    value=bytearray()
    while len(value)<16:
        value.extend(read_exact(1))
        if value[-1:]==b'\n': return bytes(value)
    raise RuntimeError('Invalid host commit framing')
try:
    if os.getuid()!=1000: raise RuntimeError('Only the reviewed sunrise UID is allowed')
    baseline=snapshot(CONFIG['review'])
    process=find_page_process(CONFIG)
    def gate():
        if snapshot(CONFIG['review'])!=baseline: raise RuntimeError('RAM mount identity changed')
        if find_page_process(CONFIG)!=process: raise RuntimeError('Supervised page process identity changed')
        live_page_status(CONFIG)
        mounts=Path('/proc/self/mountinfo').read_text().splitlines()
        if any(line.split()[4]==CONFIG['directory'] or line.split()[4].startswith(CONFIG['directory']+'/') for line in mounts):
            raise RuntimeError('Unexpected nested mount in RAM directory')
    page_transaction(CONFIG,read_exact,read_line,emit,gate)
except BaseException as error:
    emit({'phase':'ERROR','error':str(error),'commit_outcome':'unconfirmed; inspect current HTML before any retry'})
    raise SystemExit(1)
finally:
    signal.setitimer(signal.ITIMER_REAL,0)
'''
    return source


def host_check(args, review, runner_expected=None, guard_pid=None):
    guard = calibration.check_guard(args.guard_status, review)
    if guard_pid is not None and guard['local_pid'] != guard_pid:
        raise RuntimeError('Storage guard process changed during transaction')
    runner = json.loads(private_bytes(args.runner_status, 65536))
    if (runner.get('state') != 'running' or runner.get('capability_scope') != calibration.SCOPE
            or runner.get('motor_output') is not False or runner.get('servo_arm_authorized') is not False
            or runner.get('remote_stop_confirmed') is not False
            or type(runner.get('local_pid')) is not int or runner['local_pid'] <= 1
            or runner['local_pid'] == os.getpid()
            or runner.get('feedback_paused') is True):
        raise RuntimeError('Current RAM page supervisor is not running in the reviewed scope')
    validate_ram_path(runner.get('ram_directory'))
    os.kill(runner['local_pid'], 0)
    identity = (runner['local_pid'], runner['ram_directory'])
    if runner_expected is not None and identity != runner_expected:
        raise RuntimeError('RAM page supervisor changed during transaction')
    return guard['local_pid'], identity


def update(args):
    if sys.version_info < (3, 10):
        raise RuntimeError('Use the bundled modern Python for shared monotonic guard timestamps')
    review = json.loads(private_bytes(args.reviewed_baseline, 65536))
    calibration.validate_review(review)
    connection = json.loads(private_bytes(args.connection_json, 65536))
    for value in (args.expected_old_sha256, args.expected_new_sha256):
        if not re.fullmatch('[0-9a-f]{64}', value):
            raise ValueError('Explicit old and new reviewed SHA256 values are required')
    if type(args.expected_generation) is not int or args.expected_generation < 0 or not 1024 <= args.remote_port <= 65535:
        raise ValueError('Explicit current generation and loopback remote port are required')
    html = private_bytes(args.new_html)
    if hashlib.sha256(html).hexdigest() != args.expected_new_sha256:
        raise ValueError('New HTML differs from the reviewed SHA256')
    html.decode('utf-8', 'strict')
    guard_pid, runner = host_check(args, review)
    config = dict(review=review, directory=runner[1], uid=1000, old_sha256=args.expected_old_sha256,
                  new_sha256=args.expected_new_sha256, new_size=len(html), generation=args.expected_generation,
                  remote_port=args.remote_port)
    command = base.ssh_base(connection) + [connection['target'], shlex.join(['python3', '-B', '-S', '-u', '-c', remote_source(config)])]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    selector = selectors.DefaultSelector(); output = bytearray(); errors = bytearray(); pending = bytearray()
    phase = 'START'; result = None; began = time.monotonic()
    try:
        for stream in (process.stdin, process.stdout, process.stderr):
            os.set_blocking(stream.fileno(), False)
        selector.register(process.stdout, selectors.EVENT_READ, 'stdout')
        selector.register(process.stderr, selectors.EVENT_READ, 'stderr')
        while result is None:
            host_check(args, review, runner, guard_pid)
            if time.monotonic() - began > 3:
                raise TimeoutError('Host RAM page transaction exceeded 3 seconds')
            if process.poll() is not None and not selector.get_map():
                raise RuntimeError('Bounded SSH page update exited without confirmation')
            if pending:
                try:
                    sent = os.write(process.stdin.fileno(), pending[:16384]); del pending[:sent]
                except BlockingIOError:
                    pass
            for key, _ in selector.select(.02):
                data = os.read(key.fileobj.fileno(), 4096)
                if not data:
                    selector.unregister(key.fileobj); continue
                if key.data == 'stderr':
                    errors.extend(data)
                    if len(errors) > 4096: raise RuntimeError('Unexpected excessive SSH diagnostic output')
                    continue
                output.extend(data)
                if len(output) > 8192: raise RuntimeError('Unexpected excessive page-update output')
                while b'\n' in output:
                    line, _, tail = output.partition(b'\n'); output = bytearray(tail)
                    event = json.loads(line)
                    if event.get('phase') == 'ERROR': raise RuntimeError(event.get('error', 'Remote update failed'))
                    if event.get('phase') == 'READY' and phase == 'START':
                        phase='TRANSFER'; pending.extend(html)
                    elif event.get('phase') == 'PREPARED' and phase == 'TRANSFER' and not pending:
                        if event.get('new_sha256') != args.expected_new_sha256: raise RuntimeError('Prepared SHA mismatch')
                        host_check(args, review, runner, guard_pid)
                        phase='COMMIT'; pending.extend(b'COMMIT\n')
                    elif event.get('phase') == 'DONE' and phase == 'COMMIT' and event.get('committed') is True:
                        if event.get('new_sha256') != args.expected_new_sha256 or event.get('old_sha256') != args.expected_old_sha256:
                            raise RuntimeError('Committed SHA evidence mismatch')
                        result=event
                    else: raise RuntimeError('Unexpected page transaction phase')
        host_check(args, review, runner, guard_pid)
        return result
    finally:
        selector.close()
        process.stdin.close()
        if process.poll() is None:
            process.terminate()
        try: process.wait(timeout=.3)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=.3)
        process.stdout.close(); process.stderr.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('connection-json','guard-status','runner-status','reviewed-baseline','new-html','expected-old-sha256','expected-new-sha256'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--expected-generation',required=True,type=int)
    parser.add_argument('--remote-port',required=True,type=int)
    args=parser.parse_args()
    try: print(json.dumps(update(args),ensure_ascii=False))
    except Exception as error:
        print(json.dumps({'committed':'unconfirmed','error':str(error),'automatic_retry':False},ensure_ascii=False))
        return 1
    return 0

if __name__=='__main__':
    raise SystemExit(main())
