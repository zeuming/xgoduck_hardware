#!/usr/bin/env python3
"""Inspect local passive evidence; propose a fixed profile only after exact review.

No SSH, subprocess, /dev access, hardware enumeration, or file writing. This
does not approve recovery, clear an unsafe hold, or modify reader/server pins.
The normal CLI emits inspection only. An explicitly supplied exact event list
can produce proposed profile bytes on stdout for a separate root review.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import stat

import feedback_stream

PRIOR_SHA = 'dd634b2e8dfe4432a70024254c6533f2f57df56043b0b9edf92793bcfa9fb529'
HOST_FINGERPRINT = 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg'
USB_REASON = 'Kernel reported a USB disconnect/reset/error'
DISCONNECT = re.compile(r'^\[\s*\d+(?:\.\d+)?\] usb 1-1\.3: USB disconnect, device number [1-9]\d*$')
MAX_INPUT_BYTES = 2 * 1024 * 1024


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def strict_json(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError('Duplicate JSON key')
            value[key] = item
        return value
    def reject(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)


def read_private_local(path):
    path = Path(path)
    if not path.absolute().is_relative_to(feedback_stream.EVIDENCE_DIR.absolute()):
        raise ValueError('Inputs must remain inside the local replacement-board evidence directory')
    if not path.resolve().is_relative_to(feedback_stream.EVIDENCE_DIR.resolve()):
        raise ValueError('Inputs must remain inside the local replacement-board evidence directory')
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(descriptor)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or stat.S_IMODE(before.st_mode) != 0o600 or before.st_nlink != 1
                or not 0 <= before.st_size <= MAX_INPUT_BYTES):
            raise ValueError('Input must be a private, bounded, singly linked local regular file')
        raw = os.read(descriptor, MAX_INPUT_BYTES + 1)
        after = os.fstat(descriptor)
        signature = lambda info: (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
        if signature(before) != signature(after) or len(raw) != after.st_size:
            raise ValueError('Local evidence changed during reading')
        return raw
    finally:
        os.close(descriptor)


def classifiers():
    # Definitions only: never import/execute the passive capture's main body.
    namespace = {'re': re}
    source = feedback_stream._extract(feedback_stream.EVIDENCE_DIR / 'passive-snapshot.py', ('classify_log',))
    exec(compile(source, '<local-kernel-classifier>', 'exec'), namespace)
    original = namespace['classify_log']
    return original, lambda line: original(line) or feedback_stream.additional_feedback_fault(line)


def inspect_snapshot(prior_raw, result_raw, connection_raw, stderr_raw=b''):
    if any(len(raw) > MAX_INPUT_BYTES for raw in (prior_raw, result_raw, connection_raw, stderr_raw)):
        raise ValueError('Local evidence exceeds size bound')
    if digest(prior_raw) != PRIOR_SHA:
        raise ValueError('Prior profile must be the exact fixed dd634b2 artifact')
    prior, result, connection = map(strict_json, (prior_raw, result_raw, connection_raw))
    feedback_stream._validate_reviewed_profile(prior)
    if stderr_raw or result.get('transport_status') != 'diagnostic_completed':
        raise ValueError('Passive SSH capture did not complete cleanly')
    reply = result.get('diagnostic_reply')
    if not isinstance(reply, dict):
        raise ValueError('Missing passive diagnostic object')
    refusal = reply.get('identity_validation_errors') == ['Board boot identity changed']
    expected_status = 'stopped' if refusal else 'metadata_captured_requires_local_review'
    if (reply.get('identity_validation_errors') not in ([], ['Board boot identity changed'])
            or result.get('status') != expected_status or reply.get('status') != expected_status
            or (not refusal and (result.get('stop_reason') is not None or reply.get('stop_reason') is not None))
            or (refusal and (result.get('stop_reason') != 'Board boot identity changed'
                             or reply.get('stop_reason') != 'Board boot identity changed'))):
        raise ValueError('Unexpected passive identity refusal')
    fixed_connection = {'host_key_fingerprint': HOST_FINGERPRINT, 'route_interface': 'en5',
                        'ssh_sessions_attempted': 1, 'ssh_process_closed': True,
                        'whole_session_timeout': False, 'ssh_exit_code': 20 if refusal else 0,
                        'status': expected_status, 'transport_status': 'diagnostic_completed'}
    if any(type(connection.get(key)) is not type(value) or connection.get(key) != value
           for key, value in fixed_connection.items()):
        raise ValueError('Passive capture connection proof changed')
    false_fields = ('serial_opened', 'block_device_opened', 'remote_files_written_by_script',
                    'motion_authorized', 'storage_health_certified')
    if (reply.get('scope') != 'postflash_one_shot_passive_os_metadata'
            or any(reply.get(key) is not False for key in false_fields)
            or reply.get('capture_errors') != [] or reply.get('swap_entries') != []):
        raise ValueError('Passive capture scope/completeness changed')
    if (reply.get('boot_id') != prior['boot_id'] or reply.get('kernel') != '6.1.83'
            or reply.get('architecture') != 'aarch64' or reply.get('board_model') != prior['model']
            or reply.get('root', {}).get('line') != prior['root_mount_line']):
        raise ValueError('Same-boot board/root identity changed')
    card = reply.get('root_card', {})
    fixed_card = {'cid': prior['cid'], 'block_name': 'mmcblk1', 'name': 'SK32G',
                  'type': 'SD', 'capacity_bytes': 31914983424, 'root_is_partition': True}
    if any(type(card.get(key)) is not type(value) or card.get(key) != value for key, value in fixed_card.items()):
        raise ValueError('Same card identity changed')
    usb = [row for row in reply.get('usb_devices', []) if row.get('sysfs_name') == '1-1.3']
    tty = reply.get('serial_sysfs_metadata', [])
    if (len(usb) != 1 or usb[0].get('vid') != '1a86' or usb[0].get('pid') != '7523'
            or usb[0].get('serial', '') != '' or len(tty) != 1
            or tty[0].get('name') != 'ttyUSB0' or tty[0].get('device_number') != '188:0'):
        raise ValueError('Reviewed ttyUSB0/USB1-1.3 CH340 identity is absent or changed')
    tty_usb = tty[0].get('usb', {})
    if (any(tty_usb.get(key) != prior['serial'][key] for key in ('sysfs_name', 'vid', 'pid'))
            or tty_usb.get('serial', '') != ''):
        raise ValueError('TTY is attached to a different USB identity')
    kernel = reply.get('kernel_capture', {})
    log = kernel.get('kernel_log')
    if (kernel.get('exit_code') != 0 or kernel.get('stderr') != '' or type(log) is not str
            or not log.endswith('\n') or not prior['kernel_prefix'].endswith('\n')
            or not log.encode().startswith(prior['kernel_prefix'].encode())
            or kernel.get('kernel_log_sha256') != digest(log.encode())
            or kernel.get('kernel_line_count') != len(log.splitlines())):
        raise ValueError('Full unchanged kernel prefix/capture hash/count missing')
    original, classify = classifiers()
    original_faults = [{'line': line, 'reason': reason} for line in log.splitlines()
                       if (reason := original(line))]
    if kernel.get('kernel_faults') != original_faults:
        raise ValueError('Recorded passive classifier evidence is incomplete')
    prior_faults = [{'line': line, 'reason': reason} for line in prior['kernel_prefix'].splitlines()
                    if (reason := classify(line))]
    if prior_faults != prior['historical_faults']:
        raise ValueError('Prior unresolved history changed')
    tail = log[len(prior['kernel_prefix']):].splitlines()
    faults = [{'line': line, 'reason': reason} for line in tail if (reason := classify(line))]
    if any(item['reason'] != USB_REASON or not DISCONNECT.fullmatch(item['line']) for item in faults):
        raise ValueError('New fault is outside the specific human USB1-1.3 detach review')
    # One intentional detach before Mac update, then one return to X5. This
    # helper cannot approve resets, another topology, or repeated disconnects.
    if len(faults) != 1:
        raise ValueError('Exactly one specific intentional USB1-1.3 disconnect must be reviewed')
    return {'prior': prior, 'log': log, 'new_faults': faults, 'tail_lines': tail,
            'old_boot_pin_refusal_retained': refusal, 'result_sha256': digest(result_raw),
            'connection_sha256': digest(connection_raw)}


def proposed_profile(inspection, reviewed_usb_lines, *, reviewed_at, human_context):
    expected_lines = [item['line'] for item in inspection['new_faults']]
    if (type(reviewed_usb_lines) is not list or reviewed_usb_lines != expected_lines
            or not reviewed_at or type(reviewed_at) is not str
            or not human_context or type(human_context) is not str):
        raise ValueError('Root must review the exact captured event list and supply review context')
    profile = copy.deepcopy(inspection['prior'])
    profile.update(kernel_prefix=inspection['log'], kernel_prefix_sha256=digest(inspection['log'].encode()),
                   kernel_line_count=len(inspection['log'].splitlines()),
                   historical_faults=profile['historical_faults'] + copy.deepcopy(inspection['new_faults']))
    profile['review']['same_boot_usb_reconnect_review'] = {
        'reviewer': 'root', 'reviewed_at': reviewed_at, 'human_context': human_context,
        'prior_profile_sha256': PRIOR_SHA, 'diagnostic_result_sha256': inspection['result_sha256'],
        'connection_sha256': inspection['connection_sha256'],
        'reviewed_usb_events': copy.deepcopy(inspection['new_faults']),
        'old_boot_pin_refusal_retained': inspection['old_boot_pin_refusal_retained'],
        'prior_faults_cleared': False, 'filesystem_repaired': False,
        'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}
    raw = (json.dumps(profile, ensure_ascii=False, indent=2) + '\n').encode()
    if len(raw) > feedback_stream.MAX_PROFILE_BYTES:
        raise ValueError('Proposed profile exceeds the existing reader profile bound')
    return raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--reviewed-usb-lines', type=Path)
    parser.add_argument('--reviewed-at')
    parser.add_argument('--human-context')
    args = parser.parse_args()
    prior = feedback_stream.EVIDENCE_DIR / 'feedback-reboot-reviewed-20261004-01.json'
    directory = args.result.parent
    inspection = inspect_snapshot(read_private_local(prior), read_private_local(args.result),
                                  read_private_local(directory / 'connection.json'),
                                  read_private_local(directory / 'stderr.txt'))
    if args.reviewed_usb_lines is not None:
        raw = proposed_profile(inspection, strict_json(read_private_local(args.reviewed_usb_lines)),
                               reviewed_at=args.reviewed_at, human_context=args.human_context)
        print(raw.decode(), end='')
    else:
        print(json.dumps({'inspection_only': True, 'profile_written': False,
                          'prior_profile_sha256': PRIOR_SHA,
                          'result_sha256': inspection['result_sha256'],
                          'kernel_sha256': digest(inspection['log'].encode()),
                          'kernel_line_count': len(inspection['log'].splitlines()),
                          'old_boot_pin_refusal_retained': inspection['old_boot_pin_refusal_retained'],
                          'new_faults_requiring_exact_root_review': inspection['new_faults'],
                          'new_tail_lines': inspection['tail_lines']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
