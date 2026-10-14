#!/usr/bin/env python3
"""Single-owner, no-retry RAM transport for explicit X5 maintenance sessions.

Import and factory construction are local only. The reviewed feedback profile
is reused solely for board identity and kernel history, never as authorization
for a maintenance operation. The controller owns authorization and cookies.
"""
from __future__ import annotations

import ast
import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import select
import shlex
import stat
import struct
import subprocess
import threading
import time

import feedback_stream as reader
from maintenance_diagnostics import WIRE_DECODER, decode_center_failure
from run_feedback_fault_details import (FINGERPRINT, TARGET, helpers, private_new,
                                       private_read, strict_json)
from transport_config import resolve_transport

ALLOWED_REPLIES = {8: 0x88, 9: 0x80, 10: 0x8A, 11: 0x80,
                   12: 0x80, 13: 0x8D, 14: 0x80, 15: 0x80, 16: 0x80, 17: 0x91, 18: 0x92}
PAYLOAD_LENGTHS = {8: 0, 9: 4, 10: 5, 11: 6, 12: 11, 13: 4, 14: 5, 15: 4, 16: 5, 17: 0, 18: 5}
MAX_RECORD = 3 * reader.MAX_KERNEL_BYTES
TRANSACTION_SECONDS = 2.0


def _never_opened_proof():
    # process_exited is the controller's "no child remains" compatibility
    # field. never_opened distinguishes this from observing a real child exit.
    return dict(serial_closed=True, process_exited=True, ssh_exit_code=None,
                never_opened=True, ssh_started=False, serial_open_attempts=0,
                remote_closed_report=None, automatic_retry=False)


class MaintenancePreflightError(ValueError):
    """A proved local failure before any SSH Popen invocation was attempted."""
    def __init__(self, message):
        super().__init__(message)
        self.transport = None
        self.close_report = _never_opened_proof()


REMOTE = r'''
MAINTENANCE_REPLIES = {8: 136, 9: 128, 10: 138, 11: 128,
                       12: 128, 13: 141, 14: 128, 15: 128, 16: 128, 17: 145, 18: 146}
MAINTENANCE_LENGTHS = {8: 0, 9: 4, 10: 5, 11: 6, 12: 11, 13: 4, 14: 5, 15: 4, 16: 5, 17: 0, 18: 5}

def maintenance_request(value, generation, sequence):
    if (type(value) is not dict or set(value) !=
            {'generation','sequence','type','payload','reply_type','timeout'}
            or type(value['generation']) is not int or value['generation'] != generation
            or type(value['sequence']) is not int or value['sequence'] != sequence
            or not 0 < sequence <= 0xffffffff or type(value['type']) is not int
            or value['type'] not in MAINTENANCE_REPLIES
            or type(value['reply_type']) is not int
            or value['reply_type'] != MAINTENANCE_REPLIES[value['type']]
            or type(value['timeout']) not in (int,float)
            or not math.isfinite(value['timeout']) or not 0 < value['timeout'] <= 1.8
            or type(value['payload']) is not str):
        raise ProbeError('Invalid maintenance transaction')
    try:
        payload = base64.b64decode(value['payload'], validate=True)
    except (ValueError, base64.binascii.Error) as error:
        raise ProbeError('Invalid maintenance payload encoding') from error
    if len(payload) != MAINTENANCE_LENGTHS[value['type']]:
        raise ProbeError('Maintenance request payload length differs')
    if value['type'] not in (8, 17) and struct.unpack('<I', payload[:4])[0] == 0:
        raise ProbeError('Nonzero maintenance cookie required')
    return payload

def maintenance_exchange(fd, value, payload, deadline, metadata):
    # Never discard stale data or retry a frame. Partial writes only complete
    # the one original frame; even a timeout after writing is indeterminate.
    _check_deadline(deadline)
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError('Unexpected serial input before maintenance request')
    kind, sequence = value['type'], value['sequence']
    header = HEADER.pack(b'XD', 1, kind, len(payload), sequence)
    packet = header + payload
    packet += struct.pack('<H', _crc16(packet))
    wire_deadline = min(deadline, time.monotonic() + value['timeout'])
    metadata['last_request_type'] = kind
    metadata['last_sequence'] = sequence
    _write_once(fd, packet, wire_deadline, metadata)
    raw = _read_exact(fd, HEADER.size, wire_deadline)
    magic, version, reply, length, actual_sequence = HEADER.unpack(raw)
    if (magic != b'XD' or version != 1 or actual_sequence != sequence
            or reply not in (value['reply_type'], 128) or length > 512):
        raise ProbeError('Mismatched maintenance response header')
    tail = _read_exact(fd, length + 2, wire_deadline)
    response = tail[:-2]
    if _crc16(raw + response) != struct.unpack('<H', tail[-2:])[0]:
        raise ProbeError('Maintenance response CRC mismatch')
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError('Unexpected trailing maintenance serial input')
    if reply == 128:
        if kind in (12, 16) and len(response) == 101:
            if response[0] != kind: raise ProbeError('Mismatched diagnostic ACK')
            decode_center_failure(response, payload[4])
            # Return this failure to the host as evidence, never as success.
            # The host records it and closes this same owner without retrying.
            return reply, response
        if len(response) != 2 or response[0] != kind:
            raise ProbeError('Mismatched maintenance ACK')
        # Old firmware rejects this optional, read-only capability query with
        # BadPacket. Preserve the exact ACK for the host without faulting an
        # otherwise compatible maintenance session. All other errors fail.
        if kind == 17 and response == b'\x11\x01':
            return reply, response
        if response[1]:
            reason = ('Maintenance unsupported by connected firmware' if kind == 8
                      else 'Maintenance request rejected')
            raise ProbeError(reason + ': ACK error ' + str(response[1]))
        if value['reply_type'] != 128:
            raise ProbeError('Unexpected success ACK instead of maintenance reply')
    return reply, response

def maintenance_json(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result: raise ProbeError('Duplicate maintenance JSON key')
            result[key] = value
        return result
    def constant(value): raise ProbeError('Nonfinite maintenance JSON value')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)

def run_maintenance(expected):
    started, parent, fd = time.monotonic(), os.getppid(), None
    sequence, closed, normal = 0, False, False
    reason = 'Maintenance transport did not close normally'
    metadata = {'serial_open_attempts': 0, 'request_bytes_written': 0,
                'last_request_type': None, 'last_sequence': 0,
                'automatic_retry': False, 'board_files_written_by_transport': False,
                'modem_lines_actively_changed': False, 'mcu_reset_possible': True,
                'electrical_safety_verified': False, 'exclusive': False}
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        if os.geteuid() != 0:
            raise ProbeError('Maintenance transport requires the pinned root SSH identity')
        os.set_blocking(sys.stdout.fileno(), False)
        os.set_blocking(sys.stdin.fileno(), False)
        bind_serial(expected)
        health_snapshot(expected)
        node, _ = serial_proof(expected)
        if _holders(node.st_rdev, time.monotonic() + .2):
            raise ProbeError('UART already has an open holder')
        health_snapshot(expected)
        metadata['serial_open_attempts'] = 1
        fd = os.open(serial_path(expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        if [owner for owner in _holders(node.st_rdev, time.monotonic() + .2)
                if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]:
            raise ProbeError('Another UART holder appeared while opening')
        before = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        after = _modem_bits(fd)
        if any(before[key] != after[key] for key in ('dtr','rts')):
            raise ProbeError('Observed modem signals changed during UART configuration')
        identity = health_snapshot(expected, fd)
        emit('ready', expected, scope='maintenance', identity=identity, query=dict(metadata),
             allowed_requests=list(MAINTENANCE_REPLIES))
        buffer, partial_started = bytearray(), None
        last_health = time.monotonic()
        while True:
            if os.getppid() != parent:
                raise ProbeError('Maintenance parent process disappeared')
            if partial_started is not None and time.monotonic() - partial_started > .2:
                raise ProbeError('Incomplete maintenance control message')
            if time.monotonic() - last_health >= .25:
                tick = time.monotonic()
                health_snapshot(expected, fd)
                last_health = time.monotonic()
                if last_health - tick > .5:
                    raise ProbeError('Maintenance health check lost timing guarantee')
            readable, _, exceptional = select.select([sys.stdin.fileno()], [],
                                                     [sys.stdin.fileno()], .05)
            if exceptional:
                raise ProbeError('Maintenance control pipe failed')
            if not readable:
                continue
            chunk = os.read(sys.stdin.fileno(), 4097 - len(buffer))
            if not chunk:
                raise ProbeError('Maintenance control stdin ended')
            if not buffer:
                partial_started = time.monotonic()
            buffer.extend(chunk)
            if len(buffer) > 4096:
                raise ProbeError('Maintenance control message exceeds bound')
            if b'\n' not in buffer:
                continue
            raw, _, tail = buffer.partition(b'\n')
            if tail:
                raise ProbeError('Pipelined maintenance requests are forbidden')
            value = maintenance_json(raw)
            buffer.clear()
            partial_started = None
            if (type(value) is dict and type(value.get('generation')) is int
                    and value == {'generation': expected['generation'], 'kind': 'close'}):
                normal, reason = True, 'Maintenance transport ended by Mac'
                break
            sequence += 1
            payload = maintenance_request(value, expected['generation'], sequence)
            deadline = time.monotonic() + 2.0
            health_snapshot(expected, fd)
            _check_deadline(deadline)
            reply, response = maintenance_exchange(fd, value, payload, deadline, metadata)
            health_snapshot(expected, fd)
            _check_deadline(deadline)
            last_health = time.monotonic()
            emit('response', expected, sequence=sequence, type=reply,
                 payload=base64.b64encode(response).decode('ascii'))
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try: emit('fault', expected, reason=reason, query=dict(metadata))
        except BaseException: pass
    finally:
        if fd is None:
            closed = True
        else:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason, normal = 'Serial close failed: ' + str(error), False
        try:
            emit('closed', expected, serial_closed=closed, normal=normal,
                 reason=reason, query=dict(metadata), elapsed_seconds=time.monotonic()-started)
        except BaseException: pass
    return 0 if normal and closed else 20
'''


def _identity_runtime(base_source=None):
    """Keep the reviewed health/serial helpers, excluding all feedback commands."""
    source = reader.runtime_source() if base_source is None else base_source
    wanted = {'ProbeError', 'ReaderStopped', '_signal_stop', '_check_deadline',
              '_crc16', '_holders', '_modem_bits', '_raw_settings', '_ready',
              '_read_exact', '_write_once', 'bounded_read', 'bounded_dmesg',
              'validate_kernel', 'persistent_kernel', 'restore_kernel_history', 'serial_proof', 'bind_serial', 'serial_identity', 'serial_path',
              '_usb_serial_identity', 'validate_os', 'health_snapshot',
              'classify_log', 'additional_feedback_fault', 'classify_feedback_log', 'emit'}
    blocks = [ast.get_source_segment(source, node) for node in ast.parse(source).body
              if isinstance(node, (ast.Import, ast.ImportFrom, ast.Assign))
              or isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in wanted]
    return '\n\n'.join(blocks) + '\n' + WIRE_DECODER + '\n' + REMOTE


def prepare_source(expected, reviewed_profile_path, reviewed_profile_sha256):
    """Pure local evidence checks; the profile does not grant motion authority."""
    expected = copy.deepcopy(expected)
    expected.pop('reviewed_reconnect_proof', None)
    _, expected = reader._source_expected(expected, reviewed_profile_path, reviewed_profile_sha256)
    source = _identity_runtime()
    namespace = {}
    exec(compile(source, '<maintenance-definitions>', 'exec'), namespace)
    namespace['validate_kernel'](expected['kernel_prefix'], expected)
    source += '\nEXPECTED = ' + repr(expected) + '\nraise SystemExit(run_maintenance(EXPECTED))\n'
    return expected, source.encode('utf-8')


class _ProcessChannel:
    """Bounded JSONL pipes for one SSH process. Never opens a replacement child."""
    def __init__(self, source, directory, known_hosts, helper, *, run=subprocess.run,
                 popen=subprocess.Popen, clock=time.monotonic, transport_config=None):
        self.process = None
        self.clock = clock
        self.buffers = bytearray()
        self.stderr = bytearray()
        self.active = set()
        self.helper = helper
        self.askpass = directory / 'askpass.sh'
        self.launch_error = None
        self.popen_attempted = False
        try:
            self.transport_config = resolve_transport(transport_config)
            route = run(self.transport_config.route_command(), capture_output=True, text=True, timeout=2)
            self.transport_config.verify_route(route, helper)
            private_new(self.askpass, '#!/bin/sh\nexec /usr/bin/printenv X5_MAINTENANCE_PASSWORD\n')
            self.askpass.chmod(0o700)
            command = ['ssh', '-F', '/dev/null', '-T', '-o', 'ConnectTimeout=5',
                '-o', 'ConnectionAttempts=1', '-o', 'ServerAliveInterval=2',
                '-o', 'ServerAliveCountMax=1', '-o', 'NumberOfPasswordPrompts=1',
                '-o', 'PreferredAuthentications=password', '-o', 'PubkeyAuthentication=no',
                '-o', 'StrictHostKeyChecking=yes', '-o', 'UserKnownHostsFile=' + str(known_hosts),
                '-o', 'GlobalKnownHostsFile=/dev/null', '-o', 'ControlMaster=no',
                '-o', 'ControlPath=none', '-o', 'ControlPersist=no', '-o', 'LogLevel=ERROR',
                *self.transport_config.ssh_options(), self.transport_config.target,
                'exec python3 -I -B -S -c ' + shlex.quote(helper['reader_bootstrap'](len(source)))]
            env = dict(os.environ, SSH_ASKPASS=str(self.askpass), SSH_ASKPASS_REQUIRE='force',
                       DISPLAY='codex-maintenance', X5_MAINTENANCE_PASSWORD='root')
            self.popen_attempted = True
            self.process = popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, env=env, start_new_session=True)
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                os.set_blocking(stream.fileno(), False)
            self.active = {self.process.stdout.fileno(), self.process.stderr.fileno()}
            helper['write_pipe_once'](self.process.stdin.fileno(), source, 8.)
        except BaseException as error:
            self.launch_error = error
            self.finish()
            if not self.popen_attempted:
                raise MaintenancePreflightError(str(error) or type(error).__name__) from error
            raise

    def send(self, value, timeout):
        raw = (json.dumps(value, allow_nan=False, separators=(',', ':')) + '\n').encode()
        self.helper['write_pipe_once'](self.process.stdin.fileno(), raw, timeout)

    def receive(self, deadline):
        while True:
            if b'\n' in self.buffers:
                raw, _, tail = self.buffers.partition(b'\n')
                self.buffers = bytearray(tail)
                if len(raw) > MAX_RECORD: raise ValueError('Maintenance record exceeds bound')
                return strict_json(raw)
            if len(self.buffers) > MAX_RECORD: raise ValueError('Maintenance record exceeds bound')
            remaining = deadline - self.clock()
            if remaining <= 0: raise TimeoutError('Maintenance transaction timed out')
            if not self.active: raise EOFError('Maintenance SSH output closed')
            readable, _, exceptional = select.select(list(self.active), [], list(self.active), remaining)
            if exceptional: raise OSError('Maintenance SSH output pipe failed')
            for fd in readable:
                chunk = os.read(fd, 65536)
                if not chunk:
                    self.active.remove(fd)
                elif fd == self.process.stdout.fileno():
                    self.buffers.extend(chunk)
                else:
                    self.stderr.extend(chunk)
                    del self.stderr[:-4096]

    def finish(self):
        """Observe/wait/terminate the child, never infer remote serial closure."""
        process = self.process
        if process is not None:
            if process.poll() is None:
                try: process.wait(timeout=.25)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try: process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        try: process.wait(timeout=.5)
                        except subprocess.TimeoutExpired: pass
            if process.poll() is not None:
                for stream in (process.stdin, process.stdout, process.stderr): stream.close()
        self.askpass.unlink(missing_ok=True)
        return self.observe_exit()

    def observe_exit(self):
        """Reap an already-exited child; never send another close or signal."""
        process = self.process
        if process is not None and process.poll() is not None:
            for stream in (process.stdin, process.stdout, process.stderr):
                if not stream.closed:
                    stream.close()
        return {'process_exited': process is not None and process.poll() is not None,
                'ssh_exit_code': None if process is None else process.poll(),
                'stderr': bytes(self.stderr).decode('utf-8', 'replace')}


class SSHMaintenanceTransport:
    def __init__(self, expected, source, directory, known_hosts, helper, *,
                 channel_factory=_ProcessChannel, clock=time.monotonic, transport_config=None):
        self.expected, self.directory, self.clock = copy.deepcopy(expected), Path(directory), clock
        self.transport_config = resolve_transport(transport_config)
        self.generation = expected['generation']
        self.sequence = self.record_count = 0
        self.ready = self.closed_record = self.fault = self.close_report = None
        self.channel, self.error = None, None
        self.center_diagnostic = None
        self.channel_start_attempted = False
        self.startup_preflight_proof = None
        self.lock = threading.RLock()
        self.source_sha256 = hashlib.sha256(source).hexdigest()
        namespace = {}
        exec(compile(_identity_runtime(), '<maintenance-validation>', 'exec'), namespace)
        self.validate_kernel = namespace['validate_kernel']
        private_new(self.directory / 'remote-source.py', source)
        private_new(self.directory / 'transport-config.json',
                    json.dumps(self.transport_config.as_record()) + '\n')
        try:
            self.channel_start_attempted = True
            kwargs = {} if transport_config is None else {'transport_config': self.transport_config}
            self.channel = channel_factory(source, self.directory, known_hosts, helper, **kwargs)
            value = self._receive(self.clock() + 3.)
            if value.get('kind') != 'ready': raise ValueError('Maintenance READY is missing')
            if value.get('scope') != 'maintenance' or value.get('allowed_requests') != list(ALLOWED_REPLIES):
                raise ValueError('Maintenance READY allowlist differs')
            query = value.get('query')
            if (type(query) is not dict or query.get('serial_open_attempts') != 1
                    or query.get('exclusive') is not True or query.get('automatic_retry') is not False
                    or query.get('board_files_written_by_transport') is not False):
                raise ValueError('Maintenance READY ownership proof differs')
            identity = value['identity']
            for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
                if identity[key] != expected[key]: raise ValueError('Maintenance identity differs: ' + key)
            if not reader.serial_receipt_matches(identity.get('serial'), expected):
                raise ValueError('Maintenance identity differs: serial')
            self.validate_kernel(identity['kernel_log'], expected)
            self.ready = value
        except BaseException as error:
            self.error = str(error) or type(error).__name__
            if isinstance(error, MaintenancePreflightError):
                self.startup_preflight_proof = copy.deepcopy(error.close_report)
            self.close()
            raise MaintenanceTransportError(self.error, self) from error

    def _receive(self, deadline):
        value = self.channel.receive(deadline)
        if (type(value) is not dict or type(value.get('generation')) is not int
                or value['generation'] != self.generation or self.closed_record is not None):
            raise ValueError('Invalid maintenance record generation/order')
        self.record_count += 1
        private_new(self.directory / ('record-%08d.json' % self.record_count),
                    json.dumps(value, allow_nan=False, ensure_ascii=False) + '\n')
        if value.get('kind') == 'fault':
            if type(value.get('reason')) is not str: raise ValueError('Invalid maintenance fault')
            self.fault = value
            raise ValueError(value['reason'])
        if value.get('kind') == 'closed':
            query = value.get('query')
            if (type(value.get('serial_closed')) is not bool or type(value.get('normal')) is not bool
                    or type(query) is not dict or type(query.get('serial_open_attempts')) is not int
                    or query['serial_open_attempts'] not in (0, 1)
                    or query.get('automatic_retry') is not False
                    or query.get('board_files_written_by_transport') is not False):
                raise ValueError('Invalid maintenance CLOSED proof')
            self.closed_record = value
        return value

    def request(self, kind: int, payload: bytes, reply_type: int, timeout: float) -> bytes:
        with self.lock:
            if self.close_report is not None or self.closed_record is not None or self.ready is None:
                raise ValueError('Maintenance transport is closed; no automatic reopen')
            try:
                if (type(kind) is not int or ALLOWED_REPLIES.get(kind) != reply_type
                        or type(reply_type) is not int or type(payload) is not bytes
                        or len(payload) != PAYLOAD_LENGTHS.get(kind)
                        or type(timeout) not in (int, float) or not math.isfinite(timeout)
                        or not 0 < timeout <= 1.8
                        or kind not in (8, 17) and not any(payload[:4])):
                    raise ValueError('Invalid maintenance request contract')
                self.sequence += 1
                if self.sequence > 0xffffffff: raise ValueError('Maintenance sequence exhausted')
                deadline = self.clock() + TRANSACTION_SECONDS
                self.channel.send({'generation': self.generation, 'sequence': self.sequence,
                    'type': kind, 'payload': base64.b64encode(payload).decode('ascii'),
                    'reply_type': reply_type, 'timeout': timeout}, .2)
                value = self._receive(deadline)
                if (value.get('kind') != 'response' or type(value.get('sequence')) is not int
                        or value['sequence'] != self.sequence or value.get('type') not in (reply_type, 0x80)
                        or type(value.get('payload')) is not str):
                    raise ValueError('Mismatched maintenance reply')
                response = base64.b64decode(value['payload'], validate=True)
                if len(response) > 512: raise ValueError('Maintenance reply exceeds bound')
                if value['type'] == 0x80:
                    if kind in (12, 16) and len(response) == 101:
                        if response[0] != kind: raise ValueError('Mismatched diagnostic ACK')
                        self.center_diagnostic = decode_center_failure(response, payload[4])
                    elif len(response) != 2 or response[0] != kind:
                        raise ValueError('Mismatched maintenance ACK')
                    if kind == 17 and response == b'\x11\x01':
                        return response
                    if response[1]:
                        prefix = 'Maintenance unsupported by connected firmware' if kind == 8 else 'Maintenance request rejected'
                        raise ValueError(prefix + ': ACK error ' + str(response[1]))
                    if reply_type != 0x80: raise ValueError('Unexpected ACK in place of maintenance reply')
                return response
            except BaseException as error:
                self.error = str(error) or type(error).__name__
                self.close()
                raise

    def close(self) -> dict:
        with self.lock:
            if self.close_report is not None:
                # A late child exit can complete an existing CLOSED proof.
                # Never resend remote close, reopen, or fabricate a missing
                # serial proof while checking that same process again.
                if (self.close_report.get('process_exited') is not True
                        and self.channel is not None):
                    process = self.channel.observe_exit()
                    if process != {key: self.close_report.get(key) for key in process}:
                        self.close_report.update(process)
                        self._persist_close_report()
                return copy.deepcopy(self.close_report)
            if self.channel is not None:
                deadline = self.clock() + 1.
                if self.closed_record is None:
                    try: self.channel.send({'generation': self.generation, 'kind': 'close'}, .2)
                    except BaseException: pass
                while self.closed_record is None and self.clock() < deadline:
                    try:
                        value = self._receive(deadline)
                        if value.get('kind') not in ('closed', 'response'):
                            break
                    except ValueError:
                        if self.fault is None: break
                    except BaseException: break
                process = self.channel.finish()
            else:
                process = self.startup_preflight_proof or {
                    'process_exited': False, 'ssh_exit_code': None, 'stderr': ''}
            serial_closed = (self.closed_record['serial_closed'] if self.closed_record is not None
                             else True if process.get('never_opened') is True else None)
            self.close_report = dict(process, serial_closed=serial_closed, generation=self.generation,
                source_sha256=self.source_sha256, remote_closed_report=self.closed_record,
                ready_report=self.ready, fault_report=self.fault, error=self.error,
                center_diagnostic=copy.deepcopy(self.center_diagnostic),
                automatic_retry=False, serial_open_limit=1)
            self._persist_close_report()
            return copy.deepcopy(self.close_report)

    def _persist_close_report(self):
        """Replace only our local report after observing newer closure evidence."""
        path = self.directory / 'report.json'
        raw = json.dumps(self.close_report, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        if not os.path.lexists(path):
            private_new(path, raw)
            return
        info = path.lstat()
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or info.st_nlink != 1):
            raise ValueError('Maintenance report is no longer a private regular file')
        temporary = self.directory / ('report-update-' + secrets.token_hex(12) + '.tmp')
        try:
            private_new(temporary, raw)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


class MaintenanceTransportError(ValueError):
    """Startup may have opened UART: preserve the owner and its closure proof."""
    def __init__(self, message, transport):
        super().__init__(message)
        self.transport = transport
        self.close_report = copy.deepcopy(transport.close_report)


def make_transport_factory(expected, reviewed_profile_path, reviewed_profile_sha256,
                           known_hosts, directory, *, channel_factory=_ProcessChannel,
                           transport_config=None):
    """Build locally; each explicit invocation makes exactly one private attempt.

    No route/SSH/serial operation occurs before invocation. The caller must not
    invoke again until the previous transport supplied actual closure proof.
    """
    try:
        config = resolve_transport(transport_config)
        expected = copy.deepcopy(expected)
        prepare_source(expected, reviewed_profile_path, reviewed_profile_sha256)
        helper = helpers()
        known_hosts, directory = Path(known_hosts), Path(directory)
        if not directory.is_absolute() or not known_hosts.is_absolute():
            raise ValueError('Absolute maintenance evidence paths required')
    except BaseException as error:
        raise MaintenancePreflightError(str(error) or type(error).__name__) from error
    active = None
    factory_lock = threading.Lock()
    def connect():
        nonlocal active
        with factory_lock:
            if active is not None:
                proof = active.close_report or {}
                if proof.get('serial_closed') is not True or proof.get('process_exited') is not True:
                    raise MaintenanceTransportError('Previous maintenance owner lacks closure proof', active)
            return start()
    def start():
        nonlocal active
        try:
            info = directory.lstat()
            if (not stat.S_ISDIR(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o700
                    or info.st_uid != os.getuid() or directory.resolve() != directory):
                raise ValueError('Maintenance evidence parent must be a private owned directory')
            if helper['known_hosts_fingerprint'](private_read(known_hosts, 16384).decode()) != FINGERPRINT:
                raise ValueError('Pinned SSH host identity changed')
            local = copy.deepcopy(expected)
            local['generation'] = secrets.randbelow(0x7ffffffe) + 1
            prepared, source = prepare_source(local, reviewed_profile_path, reviewed_profile_sha256)
            attempt = directory / ('maintenance-%08x-%s' % (local['generation'], secrets.token_hex(6)))
            attempt.mkdir(mode=0o700, exist_ok=False)
            private_new(attempt / 'expectations.json', json.dumps(prepared, ensure_ascii=False, indent=2) + '\n')
        except BaseException as error:
            raise MaintenancePreflightError(str(error) or type(error).__name__) from error
        active = SSHMaintenanceTransport.__new__(SSHMaintenanceTransport)
        try:
            SSHMaintenanceTransport.__init__(active, prepared, source, attempt, known_hosts, helper,
                                             channel_factory=channel_factory,
                                             **({} if transport_config is None else {'transport_config': config}))
        except BaseException as error:
            if not getattr(active, 'channel_start_attempted', False):
                active = None
                raise MaintenancePreflightError(str(error) or type(error).__name__) from error
            raise
        return active
    return connect
