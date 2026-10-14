#!/usr/bin/env python3
"""One explicit metadata-only check after a lost feedback connection.

This does not reconstruct a lost CLOSED record. It proves only that the pinned
board, card, reviewed kernel history and UART node still match, and that two
bounded /proc inventories found no UART holders. A subsequent feedback reader
must still acquire and recheck its own exclusive UART ownership. Import and
source preparation are local only; no automatic retry or baseline learning.
"""
from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shlex
import stat
import subprocess
import time

import feedback_stream as reader
from run_feedback_fault_details import (FINGERPRINT, TARGET, helpers,
                                       private_new, private_read, strict_json)
from transport_config import resolve_transport

MAX_OUTPUT = 3 * reader.MAX_KERNEL_BYTES
MAX_STDERR = 8192
SSH_SECONDS = 18.0
REMOTE_SECONDS = 3.0
REMOTE_FIELDS = {
    'kind', 'generation', 'serial_path', 'serial_holders', 'identity_checked',
    'guards_passed', 'observation_count', 'serial_node', 'identity',
    'serial_open_attempts', 'serial_bytes_written', 'board_files_written',
    'automatic_retry', 'elapsed_seconds',
}
LOCAL_FIELDS = {'ssh_exit_code', 'ssh_process_exited', 'source_sha256'}
IDENTITY_FIELDS = {
    'boot_id', 'cid', 'root_mount_line', 'model', 'swaps', 'serial', 'kernel_log',
    'kernel_sha256', 'kernel_lines', 'historical_faults', 'historical_faults_cleared',
}

REMOTE = r'''
def check_connection_metadata(expected):
    started, parent = time.monotonic(), os.getppid()
    deadline = started + 2.5
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT, signal.SIGALRM):
        signal.signal(signum, _signal_stop)
    signal.setitimer(signal.ITIMER_REAL, 3.0)
    try:
        if os.geteuid() != 0:
            raise ProbeError('Connection check requires the pinned root SSH identity')
        os.set_blocking(sys.stdout.fileno(), False)
        bind_serial(expected)
        first_node = None
        identity = None
        for index in range(2):
            _check_deadline(deadline)
            if os.getppid() != parent:
                raise ProbeError('Connection check parent process disappeared')
            identity = health_snapshot(expected)
            _check_deadline(deadline)
            node, actual = serial_proof(expected)
            current_node = {'device': node.st_dev, 'inode': node.st_ino,
                            'rdev': node.st_rdev}
            if first_node is None:
                first_node = current_node
            elif current_node != first_node:
                raise ProbeError('Reviewed UART node changed during connection check')
            if actual != identity['serial']:
                raise ProbeError('Reviewed UART identity changed during connection check')
            holders = _holders(node.st_rdev, min(deadline, time.monotonic() + .20))
            if holders:
                raise ProbeError('UART already has an open holder: ' +
                                 json.dumps(holders[:32], separators=(',', ':')))
            _check_deadline(deadline)
            if index == 0:
                time.sleep(.10)
        # Catch a replaced node immediately after the final /proc inventory.
        node, actual = serial_proof(expected)
        if ({'device': node.st_dev, 'inode': node.st_ino, 'rdev': node.st_rdev}
                != first_node or actual != identity['serial']):
            raise ProbeError('Reviewed UART node changed during connection check')
        _check_deadline(deadline)
        if os.getppid() != parent:
            raise ProbeError('Connection check parent process disappeared')
        emit('quiescent', expected, serial_path=serial_path(expected),
             serial_holders=[], identity_checked=True, guards_passed=True,
             observation_count=2, serial_node=first_node, identity=identity,
             serial_open_attempts=0, serial_bytes_written=0,
             board_files_written=False, automatic_retry=False,
             elapsed_seconds=time.monotonic() - started)
        return 0
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason[:16384])
        except BaseException:
            pass
        return 20
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
'''


def runtime_source(base_source=None):
    """Copy reviewed metadata definitions only, excluding every UART opener."""
    source = reader.runtime_source() if base_source is None else base_source
    wanted = {
        'ProbeError', 'ReaderStopped', '_signal_stop', '_check_deadline',
        '_holders', 'bounded_read', 'bounded_dmesg', 'validate_kernel', 'persistent_kernel', 'restore_kernel_history',
        'serial_proof', 'bind_serial', 'serial_identity', 'serial_path', '_usb_serial_identity',
        'validate_os', 'health_snapshot', 'classify_log',
        'additional_feedback_fault', 'classify_feedback_log', 'emit',
    }
    blocks = [ast.get_source_segment(source, node) for node in ast.parse(source).body
              if isinstance(node, (ast.Import, ast.ImportFrom, ast.Assign))
              or isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in wanted]
    return '\n\n'.join(blocks) + '\n' + REMOTE


def prepare_source(expected, reviewed_profile_path=None, reviewed_profile_sha256=None):
    """Validate the fixed profile locally; never read a live board baseline."""
    expected = copy.deepcopy(expected)
    expected.pop('reviewed_reconnect_proof', None)
    _, expected = reader._source_expected(expected, reviewed_profile_path,
                                           reviewed_profile_sha256)
    # Stable source pins must not depend on callers' dictionary insertion order.
    expected = json.loads(json.dumps(expected, sort_keys=True, allow_nan=False))
    source = runtime_source()
    namespace = {}
    exec(compile(source, '<connection-check-definitions>', 'exec'), namespace)
    namespace['validate_kernel'](expected['kernel_prefix'], expected)
    source += ('\nEXPECTED = ' + repr(expected) +
               '\nraise SystemExit(check_connection_metadata(EXPECTED))\n')
    return expected, source.encode('utf-8')


def _validate_remote(value, expected, *, validation_source=None):
    if type(value) is not dict or set(value) != REMOTE_FIELDS:
        raise ValueError('Invalid connection-check proof schema')
    if (value['kind'] != 'quiescent'
            or type(value['generation']) is not int
            or value['generation'] != expected['generation']
            or type(value['serial_holders']) is not list or value['serial_holders']
            or value['identity_checked'] is not True or value['guards_passed'] is not True
            or type(value['observation_count']) is not int or value['observation_count'] != 2
            or type(value['serial_open_attempts']) is not int or value['serial_open_attempts'] != 0
            or type(value['serial_bytes_written']) is not int or value['serial_bytes_written'] != 0
            or value['board_files_written'] is not False or value['automatic_retry'] is not False
            or type(value['elapsed_seconds']) not in (int, float)
            or not math.isfinite(value['elapsed_seconds'])
            or not .10 <= value['elapsed_seconds'] < REMOTE_SECONDS):
        raise ValueError('Connection-check generation, ownership, timing or scope differs')
    identity = value['identity']
    # The installed persistent-boot guard retains a reviewed USB-return
    # receipt. Accept only this named extension, and verify it against the
    # fixed guard below, just like every other kernel-evidence field.
    if type(identity) is not dict or set(identity) not in (
            IDENTITY_FIELDS, IDENTITY_FIELDS | {'reviewed_usb_return'}):
        raise ValueError('Connection-check identity proof schema differs')
    if (not reader.serial_receipt_matches(identity['serial'], expected)
            or value['serial_path'] != identity['serial']['path']):
        raise ValueError('Connection-check identity differs: serial')
    minor = int(identity['serial']['path'].removeprefix('/dev/ttyUSB'))
    node = value['serial_node']
    if (type(node) is not dict or set(node) != {'device', 'inode', 'rdev'}
            or any(type(node[key]) is not int or node[key] < 0 for key in node)
            # This receipt comes from Linux, even when the host is macOS.
            # Linux encodes ttyUSB0..255 as major 188 and its tty minor;
            # host os.major/minor would decode Darwin's different layout.
            or node['inode'] == 0 or node['rdev'] != (188 << 8) + minor):
        raise ValueError('Connection-check UART node proof differs')
    for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
        if identity[key] != expected[key]:
            raise ValueError('Connection-check identity differs: ' + key)
    if (type(identity['kernel_log']) is not str
            or len(identity['kernel_log'].encode('utf-8')) > reader.MAX_KERNEL_BYTES
            or type(identity['swaps']) is not str or len(identity['swaps'].encode()) > 16384
            or type(identity['kernel_lines']) is not int
            or identity['historical_faults_cleared'] is not False):
        raise ValueError('Connection-check bounded metadata differs')
    namespace = {}
    exec(compile(runtime_source(base_source=validation_source), '<connection-check-validation>', 'exec'), namespace)
    namespace['validate_os'](identity, expected)
    kernel = namespace['validate_kernel'](identity['kernel_log'], expected)
    if ('reviewed_usb_return' in identity) != ('reviewed_usb_return' in kernel):
        raise ValueError('Connection-check reviewed USB receipt differs from fixed guard')
    for key, actual in kernel.items():
        if type(identity[key]) is not type(actual) or identity[key] != actual:
            raise ValueError('Connection-check kernel evidence differs: ' + key)
    return copy.deepcopy(value)


def validate_proof(proof, expected):
    """Strict local validation; returns a detached proof, never a CLOSED record."""
    if type(proof) is not dict or set(proof) != REMOTE_FIELDS | LOCAL_FIELDS:
        raise ValueError('Invalid completed connection-check proof schema')
    if (type(proof['ssh_exit_code']) is not int or proof['ssh_exit_code'] != 0
            or proof['ssh_process_exited'] is not True
            or type(proof['source_sha256']) is not str
            or len(proof['source_sha256']) != 64
            or any(char not in '0123456789abcdef' for char in proof['source_sha256'])):
        raise ValueError('Connection-check SSH exit or source proof differs')
    _validate_remote({key: proof[key] for key in REMOTE_FIELDS}, expected)
    canonical = json.loads(json.dumps(expected, sort_keys=True, allow_nan=False))
    source = (runtime_source() + '\nEXPECTED = ' + repr(canonical) +
              '\nraise SystemExit(check_connection_metadata(EXPECTED))\n').encode('utf-8')
    if proof['source_sha256'] != hashlib.sha256(source).hexdigest():
        raise ValueError('Connection-check source proof differs from current reviewed definitions')
    return copy.deepcopy(proof)


def _private_directory(directory):
    directory = Path(directory)
    info = directory.lstat()
    if (not directory.is_absolute() or directory.resolve() != directory
            or not stat.S_ISDIR(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o700
            or info.st_uid != os.getuid()):
        raise ValueError('Connection-check evidence requires a private owned directory')
    return directory


def _run_ssh(source, directory, known_hosts, helper, *, run=subprocess.run,
             popen=subprocess.Popen, clock=time.monotonic, transport_config=None):
    """Consume full bounded stdout/EOF and observe this one actual child exit."""
    started, process = clock(), None
    deadline = started + SSH_SECONDS
    askpass = directory / 'askpass.sh'
    output, stderr = bytearray(), bytearray()
    source_error = None
    try:
        config = resolve_transport(transport_config)
        route = run(config.route_command(), capture_output=True, text=True, timeout=2)
        config.verify_route(route, helper)
        private_new(askpass, '#!/bin/sh\nexec /usr/bin/printenv X5_CONNECTION_CHECK_PASSWORD\n')
        askpass.chmod(0o700)
        command = ['ssh', '-F', '/dev/null', '-T', '-o', 'ConnectTimeout=5',
            '-o', 'ConnectionAttempts=1', '-o', 'ServerAliveInterval=2',
            '-o', 'ServerAliveCountMax=1', '-o', 'NumberOfPasswordPrompts=1',
            '-o', 'PreferredAuthentications=password', '-o', 'PubkeyAuthentication=no',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'UserKnownHostsFile=' + str(known_hosts),
            '-o', 'GlobalKnownHostsFile=/dev/null', '-o', 'ControlMaster=no',
            '-o', 'ControlPath=none', '-o', 'ControlPersist=no', '-o', 'LogLevel=ERROR',
            *config.ssh_options(), config.target,
            'exec python3 -I -B -S -c ' + shlex.quote(helper['reader_bootstrap'](len(source)))]
        env = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force',
                   DISPLAY='codex-connection-check', X5_CONNECTION_CHECK_PASSWORD='root')
        process = popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, env=env, start_new_session=True)
        for stream in (process.stdin, process.stdout, process.stderr):
            os.set_blocking(stream.fileno(), False)
        try:
            helper['write_pipe_once'](process.stdin.fileno(), source,
                                      min(8., max(.001, deadline - clock() - 2.)))
        except (OSError, TimeoutError) as error:
            # SSH may fail before the remote bootstrap consumes its source.
            # Do not replay any bytes. Close the incomplete source, then drain
            # this same child's diagnostic pipes and observe its actual exit.
            source_error = str(error)
            process.stdin.close()
        active = {process.stdout.fileno(), process.stderr.fileno()}
        while active:
            remaining = deadline - clock() - 2.
            if remaining <= 0:
                raise TimeoutError('Single SSH connection-check deadline expired')
            readable, _, exceptional = select.select(list(active), [], list(active), min(.10, remaining))
            if exceptional:
                raise OSError('Connection-check output pipe failed')
            for fd in readable:
                chunk = os.read(fd, 65536)
                if not chunk:
                    active.remove(fd)
                    continue
                buffer, maximum = ((output, MAX_OUTPUT) if fd == process.stdout.fileno()
                                   else (stderr, MAX_STDERR))
                buffer.extend(chunk)
                if len(buffer) > maximum:
                    raise ValueError('Connection-check stdout/stderr exceeds its bounded size')
        status = process.wait(timeout=max(.001, deadline - clock() - 1.))
        if source_error is not None:
            private_new(directory / 'stdout.jsonl', bytes(output))
            if stderr:
                private_new(directory / 'stderr.txt', bytes(stderr))
            private_new(directory / 'transport-failure.json', json.dumps(dict(
                stage='source_transfer', source_error=source_error,
                ssh_exit_code=status, ssh_process_exited=process.poll() is not None,
                elapsed_seconds=clock() - started,
                source_transfer_complete=False, automatic_retry=False,
                serial_open_authorized=False), ensure_ascii=False, allow_nan=False) + '\n')
            detail = bytes(stderr).decode('utf-8', 'replace').strip()[:1024]
            if any(mark in bytes(stderr).lower() for mark in
                   (b'host key verification failed', b'remote host identification has changed')):
                raise ValueError('Pinned SSH host identity changed: ' + detail)
            # An incomplete send can never produce a successful proof, even
            # if a broken peer claims otherwise on stdout with exit zero.
            raise ValueError('Connection-check SSH transport failed: ' +
                             (detail or source_error) + '; exit ' + str(status))
        return bytes(output), bytes(stderr), status, process.poll() is not None
    finally:
        # Cleanup only this local SSH child. No remote holder is signalled.
        if process is not None and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=.5)
            except BaseException:
                process.kill()
                try:
                    process.wait(timeout=.5)
                except BaseException:
                    pass
        if process is not None and process.poll() is not None:
            for stream in (process.stdin, process.stdout, process.stderr):
                stream.close()
        askpass.unlink(missing_ok=True)


def check_connection(expected, known_hosts, directory, reviewed_profile_path=None,
                     reviewed_profile_sha256=None, *, runner=_run_ssh, transport_config=None):
    """Perform one explicit check and return proof only after all guards pass.

    ``directory`` must be a fresh private local evidence directory. Errors are
    raised with the original guarded reason; no failed check grants recovery.
    The optional runner is solely for pure local process-contract tests.
    """
    config = resolve_transport(transport_config)
    try:
        prepared, source = prepare_source(expected, reviewed_profile_path, reviewed_profile_sha256)
        helper = helpers()
    except Exception as error:
        raise ValueError('Connection-check source proof differs from fixed reviewed inputs: ' +
                         str(error)) from error
    directory, known_hosts = _private_directory(directory), Path(known_hosts)
    try:
        fingerprint = helper['known_hosts_fingerprint'](private_read(known_hosts, 16384).decode())
        if fingerprint != FINGERPRINT:
            raise ValueError('fingerprint differs from the fixed pin')
    except Exception as error:
        raise ValueError('Pinned SSH host identity changed: ' + str(error)) from error
    digest = hashlib.sha256(source).hexdigest()
    private_new(directory / 'remote-source.py', source)
    private_new(directory / 'expectations.json',
                json.dumps(prepared, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
    private_new(directory / 'transport-config.json', json.dumps(config.as_record()) + '\n')
    # Existing injected local test runners can keep their four-argument form
    # when no explicit transport was supplied. Real launches use the same config.
    kwargs = {} if transport_config is None else {'transport_config': config}
    output, stderr, status, exited = runner(source, directory, known_hosts, helper, **kwargs)
    if (type(output) is not bytes or len(output) > MAX_OUTPUT
            or type(stderr) is not bytes or len(stderr) > MAX_STDERR
            or type(status) is not int or type(exited) is not bool):
        raise ValueError('Invalid bounded SSH connection-check result')
    private_new(directory / 'stdout.jsonl', output)
    if stderr:
        private_new(directory / 'stderr.txt', stderr)
        if any(mark in stderr.lower() for mark in
               (b'host key verification failed', b'remote host identification has changed')):
            raise ValueError('Pinned SSH host identity changed: ' +
                             stderr.decode('utf-8', 'replace')[:1024])
        raise ValueError('Connection-check SSH reported stderr: ' + stderr.decode('utf-8', 'replace')[:1024])
    if not output.endswith(b'\n') or output.count(b'\n') != 1:
        if status != 0 or exited is not True:
            raise ValueError('Connection-check SSH transport failed: no complete result, exit ' + str(status))
        raise ValueError('Connection-check requires exactly one complete JSON record and full EOF')
    try:
        value = strict_json(output[:-1])
    except (ValueError, UnicodeError) as error:
        if status not in (0, 20) or exited is not True:
            raise ValueError('Connection-check SSH transport failed: interrupted result, exit ' + str(status)) from error
        raise ValueError('Invalid connection-check proof schema: ' + str(error)) from error
    if (type(value) is dict and set(value) == {'kind', 'generation', 'reason'}
            and value['kind'] == 'fault' and type(value['generation']) is int
            and value['generation'] == prepared['generation']
            and type(value['reason']) is str and 0 < len(value['reason']) <= 16384):
        raise ValueError(value['reason'])
    if status != 0 or exited is not True:
        raise ValueError('Connection-check SSH did not exit normally: ' + str(status))
    proof = dict(_validate_remote(value, prepared), ssh_exit_code=status,
                 ssh_process_exited=exited, source_sha256=digest)
    validate_proof(proof, prepared)
    private_new(directory / 'proof.json',
                json.dumps(proof, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
    return proof
