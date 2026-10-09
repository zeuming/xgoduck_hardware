"""Pure OFF-only fixed-register READ evidence decoder; imports no transport.

Register bytes are uninterpreted. A valid reply proves only its wire shape,
not a parameter's meaning, a physical output force, or fresh STATE feedback.
"""
import struct

REGISTER_READ_DETAILS = 23
REGISTER_READ_DETAILS_REPLY = 0x97
REGISTER_READ_DETAILS_VERSION = 1
REGISTER_READ_DETAILS_SIZE = 202
REGISTER_READ_BLOCKS = ((0, 6), (16, 2), (33, 1), (40, 1),
                        (42, 6), (48, 4), (62, 2), (69, 2))
MOTOR_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
ALL_MOTORS_MASK = (1 << len(MOTOR_IDS)) - 1
READ_OUTCOMES = {
    1: 'normal_reply', 2: 'device_error_reply', 3: 'bus_not_quiet',
    4: 'short_uart_write', 5: 'hardware_uart_error', 6: 'no_valid_reply_by_deadline',
}


class RegisterDiagnosticError(ValueError):
    """Reject contradictory wire evidence without discarding supplied bytes."""
    def __init__(self, message, payload):
        super().__init__(message)
        self.raw_payload_hex = payload.hex() if type(payload) is bytes else None
        self.invalid = True


def _read_block(payload, offset, address, requested_size):
    if tuple(payload[offset:offset + 2]) != (address, requested_size):
        raise RegisterDiagnosticError('Fixed register READ address/size mismatch', payload)
    started, ended = struct.unpack_from('<II', payload, offset + 2)
    outcome, error, size, flags = payload[offset + 10:offset + 14]
    raw = payload[offset + 14:offset + 20]
    if outcome not in READ_OUTCOMES or flags & ~7 or size > requested_size:
        raise RegisterDiagnosticError('Invalid register READ outcome/flags/size', payload)
    found, invalid_seen, register_valid = bool(flags & 1), bool(flags & 2), bool(flags & 4)
    if any(raw[size:]):
        raise RegisterDiagnosticError('Nonzero register READ padding', payload)
    if outcome == 1:
        if not found or error or size != requested_size or not register_valid:
            raise RegisterDiagnosticError('Inconsistent normal register READ evidence', payload)
    elif outcome == 2:
        if not found or not error or size not in (0, requested_size) or register_valid:
            raise RegisterDiagnosticError('Inconsistent device-error register READ evidence', payload)
    elif found or error or size or register_valid:
        raise RegisterDiagnosticError('Inconsistent failed register READ evidence', payload)
    # These integer views retain all bits. They are not physical quantities or
    # normalized parameter values; trailing padding is excluded from each view.
    return dict(address=address, requested_size=requested_size,
                started_us=started, ended_us=ended,
                elapsed_us=(ended - started) & 0xffffffff,
                outcome=outcome, outcome_name=READ_OUTCOMES[outcome],
                device_error=error, actual_size=size, found=found,
                invalid_reply_seen=invalid_seen, register_valid=register_valid,
                valid=register_valid, invalid=not register_valid,
                raw_data_hex=raw.hex(), reply_data_hex=raw[:size].hex(),
                raw_bytes=list(raw[:size]),
                raw_words_le=[int.from_bytes(raw[i:i + 2], 'little')
                              for i in range(0, size - 1, 2)])


def decode_register_read_details(kind, payload, *, expected_id):
    """Decode an already CRC/sequence-checked 0x97 payload for one mapped ID.

    Structural errors raise RegisterDiagnosticError. Genuine READ failures
    remain results with valid=False and untouched outcome/data. No register
    range, sign, unit, model identity or physical-force inference is applied.
    Deployment identity and timestamp correlation belong to the caller.
    """
    if type(kind) is not int or kind != REGISTER_READ_DETAILS_REPLY:
        raise RegisterDiagnosticError('Wrong RegisterReadDetails reply type', payload)
    if type(payload) is not bytes or len(payload) != REGISTER_READ_DETAILS_SIZE:
        raise RegisterDiagnosticError('Wrong RegisterReadDetails payload type/length', payload)
    if type(expected_id) is not int or expected_id not in MOTOR_IDS:
        raise RegisterDiagnosticError('Expected ID must be one mapped motor ID', payload)
    version, sid, index, count = payload[:4]
    if version != REGISTER_READ_DETAILS_VERSION or count != len(REGISTER_READ_BLOCKS):
        raise RegisterDiagnosticError('Wrong RegisterReadDetails version/block count', payload)
    if sid != expected_id or sid not in MOTOR_IDS or index != MOTOR_IDS.index(sid):
        raise RegisterDiagnosticError('RegisterReadDetails mapped ID/index mismatch', payload)
    desired, goals = struct.unpack_from('<HH', payload, 4)
    # These are MCU control arrays with the existing protocol's 12-bit bounds,
    # separate from the arbitrary raw words in register READ records below.
    if desired > 4095 or goals > 4095:
        raise RegisterDiagnosticError('MCU desired/goals outside raw position bounds', payload)
    armed_before, armed_after = payload[8:10]
    (cookie_before, cookie_after, sample_before, imu_before,
     sample_after, imu_after) = struct.unpack_from('<6I', payload, 10)
    if armed_before or armed_after or cookie_before or cookie_after:
        raise RegisterDiagnosticError('RegisterReadDetails violates OFF-only armed/cookie contract', payload)
    if sample_before != sample_after or imu_before != imu_after:
        raise RegisterDiagnosticError('RegisterReadDetails unexpectedly refreshed feedback timestamps', payload)
    known_before, torque_before, known_after, torque_after = struct.unpack_from('<4H', payload, 34)
    if (known_before | torque_before | known_after | torque_after) & ~ALL_MOTORS_MASK:
        raise RegisterDiagnosticError('RegisterReadDetails torque mask exceeds mapped motors', payload)
    if known_before != ALL_MOTORS_MASK or torque_before or torque_after:
        raise RegisterDiagnosticError('RegisterReadDetails violates entry torque-known/OFF mask contract', payload)
    reads = [_read_block(payload, 42 + i * 20, address, size)
             for i, (address, size) in enumerate(REGISTER_READ_BLOCKS)]
    valid = all(read['valid'] for read in reads)
    return dict(kind='servo_register_diagnostic', request_type=REGISTER_READ_DETAILS,
                reply_type=REGISTER_READ_DETAILS_REPLY, version=version, id=sid, index=index,
                block_count=count, raw_payload_hex=payload.hex(), valid=valid, invalid=not valid,
                mcu_desired_ticks=desired, mcu_goals_ticks=goals,
                armed_before=False, armed_after=False,
                cookie_before=cookie_before, cookie_after=cookie_after,
                sample_before_us=sample_before, imu_before_us=imu_before,
                sample_after_us=sample_after, imu_after_us=imu_after,
                torque_known_before=known_before, torque_mask_before=torque_before,
                torque_known_after=known_after, torque_mask_after=torque_after,
                reads=reads, feedback_updated=False, sensor_freshness_established=False,
                motion_authorized=False)


def decode_register_read_details_frame(frame, *, expected_id, expected_sequence):
    """Require the original sequence from an existing CRC-checked frame parser."""
    payload = frame[2] if type(frame) is tuple and len(frame) == 3 else None
    if (type(frame) is not tuple or len(frame) != 3
            or type(expected_sequence) is not int or not 0 <= expected_sequence <= 0xffffffff
            or type(frame[1]) is not int or frame[1] != expected_sequence):
        raise RegisterDiagnosticError('RegisterReadDetails frame/sequence mismatch', payload)
    result = decode_register_read_details(frame[0], payload, expected_id=expected_id)
    result['sequence'] = frame[1]
    return result
