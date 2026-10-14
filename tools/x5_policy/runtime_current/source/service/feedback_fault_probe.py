"""Build one finite, explicitly launched HELLO/FaultDetails query.

Import/build never opens SSH or a device. The fixed reviewed profile supplies
only the unchanged boot/card/kernel guards, not a movement permission. Request
7 is an independently bounded, empty read-only diagnostic request; the original
HELLO/STATE frame allowlist and safety gate remain unchanged. FaultDetails is
never emitted as STATE or sent to a calibration collector. Device error bytes
are retained verbatim and do not establish a voltage/overload/root cause.
"""
from __future__ import annotations

import ast

import feedback_stream


FAULT_RUNTIME = r'''
FAULT_HEADER = struct.Struct('<BBIIHHHHHBI')
FAULT_BANK = struct.Struct('<HHH15B15B15B15I')
FAULT_PAYLOAD_BYTES = 247
FAULT_TOTAL_SECONDS = 4.0
FAULT_OUTCOMES = ('not_attempted', 'valid_normal', 'valid_device_error',
                  'quiet_failure', 'short_uart_write', 'hardware_uart_error',
                  'deadline_without_valid_reply', 'valid_reply_register_rejected')

def _fault_frame(sequence):
    if type(sequence) is not int or not 1 <= sequence <= 0xffffffff:
        raise ProbeError('Invalid FaultDetails request sequence')
    # No caller-supplied kind or payload: request 7 is always empty.
    body = HEADER.pack(b'XD', 1, 7, 0, sequence)
    return body + struct.pack('<H', _crc16(body))

def _fault_exchange(fd, sequence, total_deadline, metadata):
    _check_deadline(total_deadline)
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError('Unexpected serial input before FaultDetails request')
    deadline = min(total_deadline, time.monotonic() + REPLY_SECONDS)
    metadata['request_types_attempted'].append(7)
    _write_once(fd, _fault_frame(sequence), deadline, metadata)
    raw_header = _read_exact(fd, HEADER.size, deadline)
    magic, version, kind, size, response_sequence = HEADER.unpack(raw_header)
    if (magic != b'XD' or version != 1 or kind != 0x87
            or size != FAULT_PAYLOAD_BYTES or response_sequence != sequence):
        raise ProbeError('Unknown or mismatched FaultDetails reply header')
    tail = _read_exact(fd, size + 2, deadline)
    payload, supplied_crc = tail[:-2], struct.unpack('<H', tail[-2:])[0]
    if _crc16(raw_header + payload) != supplied_crc:
        raise ProbeError('FaultDetails reply CRC mismatch')
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError('Unexpected trailing FaultDetails serial input')
    metadata['reply_types_received'].append(0x87)
    return payload

def _decode_fault_bank(payload, base, snapshot_us):
    values = FAULT_BANK.unpack_from(payload, base)
    attempted, found, seen = values[:3]
    latest = list(values[3:18])
    valid = list(values[18:33])
    outcomes = list(values[33:48])
    times = list(values[48:63])
    if ((attempted | found | seen) & ~ALL_MASK or found & ~seen or seen & ~attempted):
        raise ProbeError('Invalid FaultDetails bank masks')
    ages = []
    for index in range(len(IDS)):
        bit = 1 << index
        outcome = outcomes[index]
        if outcome not in range(8):
            raise ProbeError('Invalid FaultDetails outcome')
        if not attempted & bit:
            if (found & bit or seen & bit or latest[index] or valid[index]
                    or outcome or times[index]):
                raise ProbeError('Unattempted FaultDetails slot contains values')
        elif outcome == 0:
            raise ProbeError('Attempted FaultDetails slot has no outcome')
        if not seen & bit and (valid[index] or times[index]):
            raise ProbeError('Unseen FaultDetails slot contains valid-reply history')
        if outcome in (1, 2, 7):
            if not found & bit or (outcome in (1, 7) and latest[index] != 0) or (outcome == 2 and latest[index] == 0):
                raise ProbeError('FaultDetails valid outcome/error/found disagree')
            if latest[index] != valid[index]:
                raise ProbeError('FaultDetails latest and last-valid errors disagree')
        elif outcome in (3, 4, 5, 6):
            if found & bit or latest[index]:
                raise ProbeError('FaultDetails failed attempt contains a valid reply')
        # A seen timestamp of zero is legal at micros wrap. Old valid replies
        # may be retained indefinitely: this modulo age is NOT a freshness proof.
        ages.append(((snapshot_us - times[index]) & 0xffffffff) if seen & bit else None)
    return {'attempted_mask': attempted, 'latest_found_mask': found,
            'last_valid_seen_mask': seen, 'latest_attempt_error': latest,
            'last_valid_error': valid, 'latest_outcome': outcomes,
            'last_valid_us': times, 'last_valid_age_us_mod32': ages,
            'register_rejected_ids': [IDS[index] for index, value in enumerate(outcomes) if value == 7]}

def _decode_fault_details(payload):
    if len(payload) != FAULT_PAYLOAD_BYTES:
        raise ProbeError('FaultDetails length mismatch')
    (version, count, snapshot_us, servo_errors, online, fault, torque,
     present_faults, torque_faults, armed, cookie) = FAULT_HEADER.unpack_from(payload)
    if (version != 1 or count != len(IDS) or armed not in (0, 1)
            or (online | fault | torque | present_faults | torque_faults) & ~ALL_MASK
            or fault != (present_faults | torque_faults)):
        raise ProbeError('Invalid FaultDetails header fields')
    present = _decode_fault_bank(payload, FAULT_HEADER.size, snapshot_us)
    torque_bank = _decode_fault_bank(payload, FAULT_HEADER.size + FAULT_BANK.size, snapshot_us)
    return {'version': version, 'count': count, 'snapshot_us': snapshot_us,
            'servo_errors': servo_errors, 'motor_ids': list(IDS),
            'online_mask': online, 'fault_mask': fault, 'torque_mask': torque,
            'present_faults': present_faults, 'torque_faults': torque_faults,
            'armed': bool(armed), 'arm_cookie': cookie,
            'present_bank': present, 'torque_bank': torque_bank,
            'unsafe_flags_present': bool(armed or cookie or fault or torque),
            'motion_authorized': False, 'fault_cause_determined': False,
            'last_valid_age_is_freshness_proof': False,
            'outcome_names': list(FAULT_OUTCOMES)}

def run_fault_probe(expected):
    started = time.monotonic()
    deadline = started + FAULT_TOTAL_SECONDS
    parent = os.getppid()
    fd = None
    closed = normal = False
    reason = 'FaultDetails query stopped before completion'
    metadata = {'request_types_attempted': collections.deque(maxlen=2),
                'reply_types_received': collections.deque(maxlen=2),
                'request_bytes_written': 0, 'diagnostic_requests': 0,
                'diagnostic_records': 0, 'state_records': 0,
                'serial_open_attempts': 0, 'exclusive': False, 'serial_closed': False,
                'board_files_written_by_reader': False, 'mcu_reset_possible': True,
                'electrical_safety_verified': False, 'modem_lines_actively_changed': False,
                'firmware_binary_hash_verified': False,
                'firmware_may_autonomously_write_disable_or_imu_config': True,
                'servo_motion_authorized': False, 'storage_health_certified': False}
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        os.set_blocking(sys.stdout.fileno(), False)
        os.set_blocking(sys.stdin.fileno(), False)
        control = ControlChannel(sys.stdin.fileno())
        def check_control():
            _check_deadline(deadline)
            if os.getppid() != parent:
                raise ProbeError('FaultDetails parent process disappeared')
            if control.poll():
                raise FeedbackEnded('FaultDetails query ended by Mac')
        check_control()
        bind_serial(expected)
        proof = health_snapshot(expected)
        check_control()
        node, _ = serial_proof(expected)
        if _holders(node.st_rdev, min(deadline, time.monotonic() + .20)):
            raise ProbeError('UART already has an open holder')
        proof = health_snapshot(expected)
        check_control()
        metadata['serial_open_attempts'] += 1
        fd = os.open(serial_path(expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        owners = [owner for owner in _holders(node.st_rdev, min(deadline, time.monotonic() + .20))
                  if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]
        if owners:
            raise ProbeError('Another UART holder appeared while opening')
        metadata['modem_before_raw'] = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        metadata['modem_after_raw'] = _modem_bits(fd)
        if any(metadata['modem_after_raw'][line] != metadata['modem_before_raw'][line]
               for line in ('dtr', 'rts')):
            raise ProbeError('Observed modem signals changed during UART configuration')
        proof = health_snapshot(expected, fd)
        check_control()
        if _exchange(fd, 1, 1, min(deadline, time.monotonic() + REPLY_SECONDS), metadata) != b'XGODUCK_MOTION_1':
            raise ProbeError('Unknown motion firmware identity')
        proof = health_snapshot(expected, fd)
        check_control()
        emit('ready', expected, identity=proof, allowed_requests=['HELLO', 'FAULT_DETAILS'],
             diagnostic_only=True, query=_query_summary(metadata))
        metadata['diagnostic_requests'] = 1
        payload = _fault_exchange(fd, 2, deadline, metadata)
        diagnostic = _decode_fault_details(payload)
        metadata['last_diagnostic'] = diagnostic
        # Keep the valid raw flags in fault/closed evidence if the post-query
        # health check stops us. No further device request is ever sent.
        health_snapshot(expected, fd)
        check_control()
        emit('diagnostic', expected, sequence=2, diagnostic=diagnostic,
             query=_query_summary(metadata), reader_elapsed_seconds=time.monotonic() - started)
        metadata['diagnostic_records'] = 1
        reason = 'FaultDetails query completed; no motion capability granted'
        normal = True
    except FeedbackEnded as error:
        reason = str(error)
        normal = True
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason, query=_query_summary(metadata))
        except BaseException:
            pass
    finally:
        if fd is None:
            closed = True  # No UART descriptor was opened.
        else:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason = 'Serial close failed: ' + str(error)
                normal = False
        metadata['serial_closed'] = closed
        try:
            emit('closed', expected, reason=reason, normal=normal,
                 serial_closed=closed, query=_query_summary(metadata),
                 elapsed_seconds=time.monotonic() - started,
                 diagnostic_only=True, servo_motion_authorized=False)
        except BaseException:
            pass
    return 0 if normal and closed else 20
'''


def runtime_source():
    """Definitions only; no expected data or invocation of either reader."""
    return feedback_stream.runtime_source() + '\n' + FAULT_RUNTIME


def remote_fault_source(expected, *, reviewed_profile_path, reviewed_profile_sha256):
    """Reuse the pinned reader source but replace only its terminal invocation.

    The profile remains immutable and its original request scope unchanged.
    This separate source requires the caller's explicit diagnostic launch;
    neither the local web server nor the stream's allowlist is extended.
    """
    source = feedback_stream.remote_source(
        expected, reviewed_profile_path=reviewed_profile_path,
        reviewed_profile_sha256=reviewed_profile_sha256)
    tree = ast.parse(source)
    last = tree.body[-1]
    terminal = ast.parse('raise SystemExit(run_reader(EXPECTED))').body[0]
    if ast.dump(last, include_attributes=False) != ast.dump(terminal, include_attributes=False):
        raise ValueError('Reviewed reader terminal invocation changed')
    prefix = ''.join(source.splitlines(keepends=True)[:last.lineno - 1])
    result = prefix + '\n' + FAULT_RUNTIME + '\nraise SystemExit(run_fault_probe(EXPECTED))\n'
    compile(result, '<reviewed-fault-details-probe>', 'exec')
    return result
