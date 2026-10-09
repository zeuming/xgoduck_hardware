#!/usr/bin/env python3
"""Explicit single root SSH FaultDetails launch; no web/calibration integration.

All profile, identity and source pins are required before connecting. Import
performs no SSH, route query or device access. Diagnostic success, including
dangerous raw flags, never authorizes motion or clears the held calibration.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import select
import shlex
import stat
import subprocess
import time

import feedback_fault_probe as probe
import feedback_stream as reader

HERE = Path(__file__).resolve().parent
BASE = reader.EVIDENCE_DIR
TARGET = '192.168.128.10'
FINGERPRINT = 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg'
BOOTSTRAP_HELPERS_SHA = 'b9d781f2bc00d0c257872edfcb0ad9e20f80c75a360b0659900c519f36fc5ae3'
IDENTITY_HELPERS_SHA = '4354cb6c234cfde1294df25b292dfb7a72ad89b9cff56d285c936c07156bae1c'
MAX_RECORD = 262144


def strict_json(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result: raise ValueError('Duplicate JSON field')
            result[key] = value
        return result
    def constant(value): raise ValueError('Nonfinite JSON field')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)


def private_read(path, maximum):
    path = Path(path)
    if not path.is_absolute(): raise ValueError('Absolute private file required')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        before = os.fstat(stream.fileno())
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or stat.S_IMODE(before.st_mode) != 0o600 or before.st_nlink != 1
                or not 0 < before.st_size <= maximum):
            raise ValueError('Invalid private input file')
        raw = stream.read(maximum + 1)
        after = os.fstat(stream.fileno())
    fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns')
    if len(raw) != before.st_size or any(getattr(before, k) != getattr(after, k) for k in fields):
        raise ValueError('Private input changed while reading')
    return raw


def private_new(path, data):
    if isinstance(data, str): data = data.encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        if stream.write(data) != len(data): raise OSError('Incomplete local evidence write')
        stream.flush()
        os.fsync(stream.fileno())


def helpers():
    blocks = (reader._extract(HERE / 'feedback_calibration_server.py', ('reader_bootstrap', 'write_pipe_once')),
              reader._extract(BASE / 'run_postflash_passive.py', ('known_hosts_fingerprint', 'parse_route')))
    for value, digest in zip(blocks, (BOOTSTRAP_HELPERS_SHA, IDENTITY_HELPERS_SHA)):
        if hashlib.sha256(value.encode()).hexdigest() != digest:
            raise ValueError('Reviewed launcher helper source changed')
    ns = dict(os=os, time=time, select=select, base64=base64, hashlib=hashlib, re=re, TARGET=TARGET)
    exec(compile('\n'.join(blocks), '<reviewed-launcher-helpers>', 'exec'), ns)
    return ns


def prepare_source(expected_path, profile_path, profile_sha, generation):
    """Pure local preparation; never invent or review a new kernel baseline."""
    if type(generation) is not int or not 0 < generation < 0x80000000:
        raise ValueError('Explicit nonzero 31-bit generation required')
    expected = strict_json(private_read(expected_path, 2 * 1024 * 1024))
    profile = reader.load_reviewed_profile(profile_path, profile_sha)
    for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial'):
        if expected[key] != profile[key]: raise ValueError('Reviewed board identity changed: ' + key)
    expected = copy.deepcopy(expected)
    expected.update(generation=generation, kernel_prefix=profile['kernel_prefix'],
                    historical_faults=copy.deepcopy(profile['historical_faults']))
    expected.pop('reviewed_reconnect_proof', None)
    source = probe.remote_fault_source(expected, reviewed_profile_path=profile_path,
                                      reviewed_profile_sha256=profile_sha).encode()
    expected['reviewed_reconnect_proof'] = copy.deepcopy(reader._validate_reviewed_profile(profile)['proof'])
    return expected, source


def validated_diagnostic(value, namespace):
    """Round-trip exact diagnostic JSON through the frozen wire decoder."""
    if type(value) is not dict or type(value.get('armed')) is not bool:
        raise ValueError('Invalid diagnostic object/armed flag')
    keys = ('version', 'count', 'snapshot_us', 'servo_errors', 'online_mask', 'fault_mask',
            'torque_mask', 'present_faults', 'torque_faults', 'arm_cookie')
    if any(type(value.get(key)) is not int for key in keys): raise ValueError('Invalid diagnostic scalar types')
    header = namespace['FAULT_HEADER'].pack(*(value[k] for k in keys[:-1]), int(value['armed']), value['arm_cookie'])
    banks = []
    for name in ('present_bank', 'torque_bank'):
        bank = value[name]
        if type(bank) is not dict: raise ValueError('Invalid diagnostic bank object')
        masks = [bank[k] for k in ('attempted_mask', 'latest_found_mask', 'last_valid_seen_mask')]
        arrays = [bank[k] for k in ('latest_attempt_error', 'last_valid_error', 'latest_outcome', 'last_valid_us')]
        if any(type(v) is not int for v in masks) or any(type(a) is not list or len(a) != 15
                or any(type(v) is not int for v in a) for a in arrays):
            raise ValueError('Invalid diagnostic bank scalar types')
        ages = bank.get('last_valid_age_us_mod32')
        rejected = bank.get('register_rejected_ids')
        if (type(ages) is not list or len(ages) != 15
                or any(v is not None and (type(v) is not int or not 0 <= v <= 0xffffffff) for v in ages)
                or type(rejected) is not list or any(type(v) is not int for v in rejected)):
            raise ValueError('Invalid diagnostic derived array types')
        banks.append(namespace['FAULT_BANK'].pack(*masks, *(v for a in arrays for v in a)))
    decoded = namespace['_decode_fault_details'](header + b''.join(banks))
    for key in ('unsafe_flags_present', 'motion_authorized', 'fault_cause_determined', 'last_valid_age_is_freshness_proof'):
        if type(value.get(key)) is not bool: raise ValueError('Invalid diagnostic capability flag')
    if value != decoded: raise ValueError('Diagnostic fields/derived values differ from wire contract')
    return copy.deepcopy(decoded)


class Collector:
    def __init__(self, generation, expected):
        self.generation, self.expected = generation, expected
        self.ready = self.diagnostic = self.fault = self.closed = None
        self.namespace = {}
        exec(compile(probe.runtime_source(), '<fault-collector-definitions>', 'exec'), self.namespace)

    def accept(self, message):
        if (type(message) is not dict or type(message.get('generation')) is not int
                or message['generation'] != self.generation or self.closed is not None):
            raise ValueError('Invalid generation or feedback after closure')
        kind = message.get('kind')
        if kind not in ('ready', 'diagnostic', 'fault', 'closed'):
            raise ValueError('Only diagnostic reader record types are allowed')
        query = message.get('query')
        if (type(query) is not dict or query.get('servo_motion_authorized') is not False
                or query.get('board_files_written_by_reader') is not False
                or type(query.get('state_records')) is not int or query['state_records'] != 0
                or type(query.get('serial_open_attempts')) is not int
                or query['serial_open_attempts'] not in (0, 1)):
            raise ValueError('Reader scope or open count differs')
        requests = query.get('request_types_attempted')
        replies = query.get('reply_types_received')
        if requests not in ([], [1], [1, 7]) or replies not in ([], [0x81], [0x81, 0x87]):
            raise ValueError('Unexpected serial request/reply allowlist')
        if any(type(v) is not int for v in requests + replies): raise ValueError('Invalid request type')
        if 'last_diagnostic' in query:
            validated_diagnostic(query['last_diagnostic'], self.namespace)
        if kind == 'ready':
            if self.ready is not None or self.diagnostic is not None or self.fault is not None:
                raise ValueError('Repeated or out-of-order ready')
            identity = message['identity']
            if requests != [1] or replies != [0x81]: raise ValueError('Ready protocol evidence differs')
            for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
                if identity[key] != self.expected[key]: raise ValueError('Ready identity differs: ' + key)
            if not reader.serial_receipt_matches(identity['serial'], self.expected):
                raise ValueError('Ready identity differs: serial')
            self.namespace['validate_kernel'](identity['kernel_log'], self.expected)
            self.ready = copy.deepcopy(message)
        elif kind == 'diagnostic':
            if self.ready is None or self.diagnostic is not None or self.fault is not None:
                raise ValueError('Repeated or out-of-order diagnostic')
            if type(message.get('sequence')) is not int or message['sequence'] != 2:
                raise ValueError('Diagnostic sequence differs')
            if requests != [1, 7] or replies != [0x81, 0x87]:
                raise ValueError('Diagnostic protocol evidence differs')
            validated_diagnostic(message['diagnostic'], self.namespace)
            self.diagnostic = copy.deepcopy(message)
        elif kind == 'fault':
            if self.fault is not None: raise ValueError('Repeated fault')
            if type(message.get('reason')) is not str: raise ValueError('Invalid fault reason')
            self.fault = copy.deepcopy(message)
        else:
            if type(message.get('serial_closed')) is not bool or type(message.get('normal')) is not bool:
                raise ValueError('Invalid closure proof')
            if query.get('serial_closed') is not message['serial_closed']:
                raise ValueError('Closure fields disagree')
            self.closed = copy.deepcopy(message)


def execute(source, expected, directory, phase, known_hosts, helper, *, reviewed_source_sha=None):
    """One SSH child; source stdin remains open until this child has exited."""
    if (phase not in ('battery-off', 'powered-supported') or not reviewed_source_sha
            or hashlib.sha256(source).hexdigest() != reviewed_source_sha):
        raise ValueError('Explicit phase and reviewed source digest required before SSH')
    started, process = time.monotonic(), None
    deadline = started + 18.0
    collector = Collector(expected['generation'], expected)
    collector.namespace['validate_kernel'](expected['kernel_prefix'], expected)
    if helper['known_hosts_fingerprint'](private_read(known_hosts, 16384).decode()) != FINGERPRINT:
        raise ValueError('Pinned SSH host identity changed')
    report = dict(phase=phase, physical_phase_confirmed_by_caller=True,
        generation=expected['generation'], source_sha256=hashlib.sha256(source).hexdigest(),
        maximum_ssh_sessions=1, automatic_retry=False, servo_motion_authorized=False,
        calibration_written=False, board_files_written_by_reader=False, error=None)
    askpass = directory / 'askpass.sh'
    stderr, buffers, sequence = bytearray(), {}, 0
    try:
        route = subprocess.run(['route', '-n', 'get', TARGET], capture_output=True, text=True, timeout=2)
        if route.returncode != 0 or helper['parse_route'](route.stdout) != 'en5':
            raise ValueError('Reviewed USB route is unavailable')
        private_new(directory / 'remote-source.py', source)
        private_new(askpass, '#!/bin/sh\nexec /usr/bin/printenv X5_DIAGNOSTIC_PASSWORD\n')
        askpass.chmod(0o700)
        command = ['ssh', '-F', '/dev/null', '-T', '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=1', '-o', 'NumberOfPasswordPrompts=1',
            '-o', 'PreferredAuthentications=password', '-o', 'PubkeyAuthentication=no',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'UserKnownHostsFile=' + str(known_hosts),
            '-o', 'GlobalKnownHostsFile=/dev/null', '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
            '-o', 'ControlPersist=no', '-o', 'LogLevel=ERROR', 'root@' + TARGET,
            'exec python3 -I -B -S -c ' + shlex.quote(helper['reader_bootstrap'](len(source)))]
        env = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force',
                   DISPLAY='codex-fault-details', X5_DIAGNOSTIC_PASSWORD='root')
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, env=env, start_new_session=True)
        for stream in (process.stdin, process.stdout, process.stderr): os.set_blocking(stream.fileno(), False)
        helper['write_pipe_once'](process.stdin.fileno(), source, min(8., deadline - time.monotonic() - 4.2))
        out_fd, err_fd = process.stdout.fileno(), process.stderr.fileno()
        active = {out_fd, err_fd}
        buffers[out_fd] = bytearray()
        while active:
            if time.monotonic() >= deadline - 4.2: raise TimeoutError('Single SSH diagnostic deadline expired')
            readable, _, exceptional = select.select(list(active), [], list(active), .1)
            if exceptional: raise OSError('Diagnostic output pipe exception')
            for fd in readable:
                chunk = os.read(fd, 65536)
                if not chunk:
                    active.remove(fd)
                    if fd == out_fd and buffers[out_fd]: raise ValueError('Incomplete final diagnostic record')
                    continue
                if fd == err_fd:
                    stderr.extend(chunk)
                    del stderr[:-4096]
                    continue
                buffers[out_fd].extend(chunk)
                if len(buffers[out_fd]) > MAX_RECORD and b'\n' not in buffers[out_fd]:
                    raise ValueError('Diagnostic record exceeds bound')
                while b'\n' in buffers[out_fd]:
                    raw, _, tail = buffers[out_fd].partition(b'\n')
                    buffers[out_fd] = bytearray(tail)
                    sequence += 1
                    if sequence > 4 or len(raw) > MAX_RECORD: raise ValueError('Diagnostic record count/size exceeds bound')
                    private_new(directory / f'record-{sequence:02d}.json', raw + b'\n')
                    try:
                        collector.accept(strict_json(raw))
                    except (ValueError, KeyError, TypeError, OverflowError,
                            collector.namespace['struct'].error, collector.namespace['ProbeError']) as error:
                        if report['error'] is None: report['error'] = str(error)
        process.wait(timeout=max(.001, deadline - time.monotonic() - 4.2))
    except BaseException as error:
        if report['error'] is None: report['error'] = str(error) or type(error).__name__
    finally:
        if process is not None and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=max(.001, min(2., deadline - time.monotonic())))
            except BaseException:
                process.kill()
                try: process.wait(timeout=max(.001, min(2., deadline - time.monotonic())))
                except BaseException: pass
        if process is not None and process.poll() is not None:
            for stream in (process.stdin, process.stdout, process.stderr): stream.close()
        askpass.unlink(missing_ok=True)
    closed = collector.closed
    report.update(ready_report=collector.ready, diagnostic_report=collector.diagnostic,
        fault_report=collector.fault, remote_closed_report=closed,
        serial_closed=None if closed is None else closed['serial_closed'],
        ssh_exit_code=None if process is None else process.poll(),
        ssh_process_exited=process is not None and process.poll() is not None,
        stderr=bytes(stderr).decode('utf-8', 'replace'), elapsed_seconds=time.monotonic() - started)
    report['query_completed'] = (report['error'] is None and collector.diagnostic is not None
        and collector.fault is None and closed is not None and closed['normal'] is True
        and closed['serial_closed'] is True and report['ssh_exit_code'] == 0)
    private_new(directory / 'report.json', json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--expectations', required=True, type=Path)
    parser.add_argument('--reviewed-profile', required=True, type=Path)
    parser.add_argument('--reviewed-profile-sha256', required=True)
    parser.add_argument('--generation', required=True, type=int)
    parser.add_argument('--source-sha256', required=True)
    parser.add_argument('--phase', required=True, choices=('battery-off', 'powered-supported'))
    parser.add_argument('--execute-once', required=True, action='store_true')
    args = parser.parse_args()
    expected, source = prepare_source(args.expectations, args.reviewed_profile,
                                     args.reviewed_profile_sha256, args.generation)
    if hashlib.sha256(source).hexdigest() != args.source_sha256:
        raise ValueError('Reviewed final remote source digest differs; SSH refused')
    helper = helpers()
    known_hosts = BASE / 'known_hosts'
    if helper['known_hosts_fingerprint'](private_read(known_hosts, 16384).decode()) != FINGERPRINT:
        raise ValueError('Pinned SSH host identity changed')
    directory = args.run_dir
    if (not directory.is_absolute() or directory.parent.resolve() != BASE
            or not directory.name.startswith('fault-details-')):
        raise ValueError('Fresh fault-details-* run directory required')
    os.umask(0o077)
    directory.mkdir(mode=0o700, exist_ok=False)
    private_new(directory / 'expectations.json', json.dumps(expected, ensure_ascii=False, indent=2) + '\n')
    report = execute(source, expected, directory, args.phase, known_hosts, helper,
                     reviewed_source_sha=args.source_sha256)
    print(json.dumps({key: report[key] for key in ('phase', 'error', 'query_completed', 'serial_closed',
        'ssh_exit_code', 'ssh_process_exited', 'elapsed_seconds')}, ensure_ascii=False), flush=True)
    return 0 if report['query_completed'] else 20


if __name__ == '__main__':
    raise SystemExit(main())
