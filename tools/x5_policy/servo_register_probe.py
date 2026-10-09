"""Build a pinned OFF-only raw-register comparison (default IDs 11,12,21,22).

Generation is local and never opens UART. The generated script permits only
HELLO, STATE and Type23 for explicit mapped IDs, once each without retries. Register
addresses are evidence, not model-specific names, physical units or torque.
"""
from __future__ import annotations

import argparse
import ast
import inspect
import os
from pathlib import Path
import time

from servo_goal_probe import (REMOTE_SETUP, build_remote_source as build_goal_source,
                              checked_off_state)
from servo_register_diagnostics import decode_register_read_details_frame

PROBE_IDS = (11, 12, 21, 22)
MOTOR_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)


def checked_probe_ids(probe_ids):
    if (type(probe_ids) is not tuple or not 1 <= len(probe_ids) <= 4
            or any(type(sid) is not int or sid not in MOTOR_IDS for sid in probe_ids)
            or len(set(probe_ids)) != len(probe_ids)):
        raise ValueError('Select one to four distinct mapped servo IDs')
    return probe_ids


def adapt_helpers(source, namespace, *, probe_ids=PROBE_IDS):
    """Extend exactly the reviewed frame/exchange helpers, with a fixed allowlist."""
    allowed_payloads = tuple(bytes([sid]) for sid in checked_probe_ids(probe_ids))
    definitions = {}
    for name in ('_frame', '_exchange'):
        nodes = [n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == name]
        if len(nodes) != 1:
            raise ValueError('Missing or duplicate reviewed helper: ' + name)
        definitions[name] = ast.get_source_segment(source, nodes[0])
    frame, exchange = definitions['_frame'], definitions['_exchange']
    for before, after in (
        ('def _frame(kind, sequence):', 'def _frame(kind, sequence, payload=b""):'),
        ('if kind not in (1, 2):',
         'if not ((kind in (1, 2) and payload == b"") or (kind == 23 and payload in '+repr(allowed_payloads)+')):'),
        ('HEADER.pack(b"XD", 1, kind, 0, sequence)',
         'HEADER.pack(b"XD", 1, kind, len(payload), sequence) + payload'),
    ):
        if frame.count(before) != 1:
            raise ValueError('Reviewed frame helper shape changed')
        frame = frame.replace(before, after)
    for before, after in (
        ('def _exchange(fd, kind, sequence, total_deadline, metadata):',
         'def _exchange(fd, kind, sequence, total_deadline, metadata, *, payload=b""):'),
        ('_frame(kind, sequence)', '_frame(kind, sequence, payload)'),
        ('(0x81, 16) if kind == 1 else (0x82, STATE_PACKET.size)',
         '{1: (0x81, 16), 2: (0x82, STATE_PACKET.size), 23: (0x97, 202)}[kind]'),
    ):
        if exchange.count(before) != 1:
            raise ValueError('Reviewed exchange helper shape changed')
        exchange = exchange.replace(before, after)
    exec(compile(frame + '\n\n' + exchange, '<OFF-only-register-exchange>', 'exec'), namespace)


def run_probe(namespace, expected, guard_check, *, clock=time.monotonic, sleeper=time.sleep,
              probe_ids=PROBE_IDS):
    """One exclusive fd, four single diagnostic attempts, close on every exit."""
    import fcntl
    import termios
    probe_ids = checked_probe_ids(probe_ids)
    started, fd = clock(), None
    result = dict(kind='off_servo_register_probe', ids=list(probe_ids), success=False,
                  serial_closed=False, motion_authorized=False, feedback_updated=False,
                  sensor_freshness_established=False, correlated_with_previous_hold=False,
                  mcu_reset_possible=True, modem_lines_actively_changed=False,
                  hupcl_actively_changed=False, diagnostic_attempts=0, diagnostics=[],
                  states=[], request_timing=[],
                  query=dict(request_types_attempted=[], reply_types_received=[], request_bytes_written=0))
    metadata, sequence, previous = result['query'], 1, None
    def exchange(kind, seq, **options):
        if clock() - started >= 15:
            raise TimeoutError('OFF register comparison exceeded its 15 second deadline')
        timing = dict(kind=kind, sequence=seq, started_monotonic=clock())
        result['request_timing'].append(timing)
        try:
            return namespace['_exchange'](fd, kind, seq, min(started + 15, clock() + .4),
                                           metadata, **options)
        finally:
            timing['ended_monotonic'] = clock()
    def health():
        return {key: value for key, value in guard_check(fd).items() if key != 'kernel_log'}
    def state_read():
        nonlocal sequence, previous
        sequence += 1
        state = checked_off_state(namespace, exchange(2, sequence))
        if previous is not None:
            for key in ('sample_us', 'imu_us'):
                delta = (state[key] - previous[key]) & 0xffffffff
                if delta >= 0x80000000 or delta > 1000000:
                    raise ValueError('STATE clock reversed/reset or jumped')
        result['states'].append(dict(sequence=sequence, received_monotonic=clock(), state=state))
        previous = state
        return state
    try:
        namespace['bind_serial'](expected)
        guard_check()
        node, _ = namespace['serial_proof'](expected)
        if namespace['_holders'](node.st_rdev, clock() + .2):
            raise ValueError('UART already has an owner')
        guard_check()
        result['open_started_monotonic'] = clock()
        fd = os.open(namespace['serial_path'](expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        result['opened_monotonic'] = clock()
        namespace['serial_proof'](expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        if any(not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))
               for owner in namespace['_holders'](node.st_rdev, clock() + .2)):
            raise ValueError('Another UART owner appeared')
        if termios.tcgetattr(fd)[2] & termios.HUPCL:
            raise ValueError('HUPCL is set; refusing to change modem lifecycle')
        result['modem_before_raw'] = namespace['_modem_bits'](fd)
        result['serial_settings'] = namespace['_raw_settings'](fd)
        result['modem_after_raw'] = namespace['_modem_bits'](fd)
        if (termios.tcgetattr(fd)[2] & termios.HUPCL or
                result['modem_before_raw'] != result['modem_after_raw']):
            raise ValueError('UART raw setup changed modem lines or HUPCL')
        result['health_before'] = health()
        hello_started = clock()
        namespace['_hello_feedback'](fd, metadata)
        result['request_timing'].append(dict(kind=1, sequence=1, started_monotonic=hello_started,
                                             ended_monotonic=clock()))
        ready_deadline = min(started + 15, clock() + 2)
        while clock() < ready_deadline:
            before = previous
            state = state_read()
            if clock() >= ready_deadline:
                raise ValueError('OFF feedback readiness deadline expired')
            if (before is not None and state['online_mask'] == 0x7fff
                    and all((state[k] - before[k]) & 0xffffffff for k in ('sample_us', 'imu_us'))):
                break
            sleeper(min(.025, max(0., ready_deadline - clock())))
        else:
            raise ValueError('Healthy complete OFF feedback did not advance within 2 seconds')
        last_attempt = None
        for sid in probe_ids:
            # Keep STATE clocks correlated while respecting the shared MCU quota.
            while last_attempt is not None and clock() - last_attempt < .55:
                sleeper(max(.001, min(.025, .55 - (clock() - last_attempt))))
                state = state_read()
            result['health_pre_diagnostic'] = health()
            state = state_read()
            if state['online_mask'] != 0x7fff:
                raise ValueError('Pre-diagnostic STATE is not fully online')
            sequence += 1
            result['diagnostic_attempts'] += 1
            raw = exchange(23, sequence, payload=bytes([sid]))
            # Response arrival is later than MCU acceptance, even with latency.
            last_attempt = clock()
            entry = dict(id=sid, raw_payload_hex=raw.hex())
            result['diagnostics'].append(entry)
            entry['decoded'] = decode_register_read_details_frame((0x97, sequence, raw),
                expected_id=sid, expected_sequence=sequence)
            after = state_read()
            diagnostic = entry['decoded']
            for key, diag_key in (('sample_us', 'sample_before_us'), ('imu_us', 'imu_before_us')):
                first = (diagnostic[diag_key] - state[key]) & 0xffffffff
                second = (after[key] - diagnostic[diag_key]) & 0xffffffff
                if first >= 0x80000000 or second >= 0x80000000 or first + second > 500000:
                    raise ValueError('Diagnostic clocks do not correlate with surrounding STATE')
            if after['online_mask'] != 0x7fff:
                raise ValueError('Post-diagnostic STATE is not fully online')
            if not diagnostic['valid']:
                raise ValueError('Register READ evidence invalid; raw outcomes retained without retry')
        result['health_after'] = health()
        result['success'] = True
    except BaseException as error:
        result.update(success=False, error=str(error) or type(error).__name__, error_type=type(error).__name__)
        if getattr(error, 'raw_payload_hex', None) is not None:
            result['rejected_raw_payload_hex'] = error.raw_payload_hex
    finally:
        if fd is not None:
            try:
                result['close_started_monotonic'] = clock()
                os.close(fd)
                result.update(closed_monotonic=clock(), serial_closed=True)
            except BaseException as error:
                result.update(success=False, close_error=str(error) or type(error).__name__)
        else:
            result['serial_closed'] = True
        result['elapsed_seconds'] = clock() - started
    return result


def build_remote_source(installation_archive, closure_evidence, persistent_calibration_sha256, *, boot_id, cid,
                        probe_ids=PROBE_IDS):
    """Reuse the reviewed closure/installation pin builder; generate source only."""
    probe_ids = checked_probe_ids(probe_ids)
    original = build_goal_source(installation_archive, closure_evidence,
                                 persistent_calibration_sha256, boot_id=boot_id, cid=cid)
    nodes = [n for n in ast.parse(original).body if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name) and t.id == 'PINS' for t in n.targets)]
    if len(nodes) != 1:
        raise ValueError('Expected one original pin manifest')
    pins = ast.literal_eval(nodes[0].value)
    decoder = Path(__file__).with_name('servo_register_diagnostics.py').read_text()
    definitions = '\n\n'.join(inspect.getsource(fn) for fn in (checked_probe_ids, adapt_helpers, checked_off_state, run_probe))
    setup = REMOTE_SETUP.replace("kind='off_servo_goal_probe'", "kind='off_servo_register_probe'")
    setup = setup.replace('adapt_helpers(guard,namespace)',
        'adapt_helpers(guard,namespace,probe_ids='+repr(probe_ids)+')').replace(
        'result=run_probe(namespace,expected,health)',
        'result=run_probe(namespace,expected,health,probe_ids='+repr(probe_ids)+')')
    source = (decoder + '\nimport ast,math,os,time\nPINS=' + repr(pins)
              + '\nPROBE_IDS=' + repr(probe_ids) + '\n' + definitions + '\n' + setup)
    compile(source, '<generated-OFF-register-probe>', 'exec')
    return source


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('installation-archive', 'closure-evidence', 'persistent-calibration-sha256', 'boot-id', 'cid', 'output'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    source = build_remote_source(args.installation_archive, args.closure_evidence,
        args.persistent_calibration_sha256, boot_id=args.boot_id, cid=args.cid)
    Path(args.output).write_text(source)
