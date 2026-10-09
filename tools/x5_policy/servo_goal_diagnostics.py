"""Pure GoalReadDetails v1 decoder; imports no transport or motion runtime.

This is separate evidence from two OFF-only register READs, not STATE feedback.
Failed reads and invalid register values retain their original bytes/outcome.
No result updates a feedback cache, establishes freshness or authorizes motion.
"""
from __future__ import annotations

import struct

GOAL_READ_DETAILS = 22
GOAL_READ_DETAILS_REPLY = 0x96
GOAL_READ_DETAILS_VERSION = 1
GOAL_READ_DETAILS_SIZE = 78
MOTOR_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
ALL_MOTORS_MASK = (1 << len(MOTOR_IDS)) - 1
READ_OUTCOMES = {
    1: 'normal_reply', 2: 'device_error_reply', 3: 'bus_not_quiet',
    4: 'short_uart_write', 5: 'hardware_uart_error', 6: 'no_valid_reply_by_deadline',
}


class GoalDiagnosticError(ValueError):
    """Rejected wire evidence, retaining the exact supplied bytes for auditing."""
    def __init__(self, message, payload):
        super().__init__(message)
        self.raw_payload_hex = payload.hex() if type(payload) is bytes else None
        self.invalid = True


def _read_block(payload, offset, address, requested_size):
    started, ended = struct.unpack_from('<II', payload, offset)
    outcome, error, size, flags = payload[offset + 8:offset + 12]
    raw = payload[offset + 12:offset + 18]
    if outcome not in READ_OUTCOMES or flags & ~7 or size > requested_size:
        raise GoalDiagnosticError('Invalid register READ outcome/flags/size', payload)
    found, invalid_seen, register_valid = bool(flags & 1), bool(flags & 2), bool(flags & 4)
    if any(raw[size:]):
        raise GoalDiagnosticError('Nonzero register READ padding', payload)
    position_word = int.from_bytes(raw[:2], 'little')
    if outcome == 1:
        if not found or error or size != requested_size or register_valid != (position_word <= 4095):
            raise GoalDiagnosticError('Inconsistent normal register READ evidence', payload)
    elif outcome == 2:
        if not found or not error or size not in (0, requested_size) or register_valid:
            raise GoalDiagnosticError('Inconsistent device-error register READ evidence', payload)
    elif found or error or size or register_valid:
        raise GoalDiagnosticError('Inconsistent failed register READ evidence', payload)
    return dict(address=address, requested_size=requested_size,
                started_us=started, ended_us=ended,
                elapsed_us=(ended-started) & 0xffffffff,
                outcome=outcome, outcome_name=READ_OUTCOMES[outcome],
                device_error=error, actual_size=size, found=found,
                invalid_reply_seen=invalid_seen, register_valid=register_valid,
                valid=register_valid, invalid=not register_valid,
                raw_data_hex=raw.hex(), reply_data_hex=raw[:size].hex(),
                raw_position_word=position_word,
                raw_speed_word=int.from_bytes(raw[2:4], 'little') if requested_size == 6 else None,
                raw_load_word=int.from_bytes(raw[4:6], 'little') if requested_size == 6 else None,
                position_ticks=position_word if register_valid else None)


def decode_goal_read_details(kind, payload, *, expected_id):
    """Decode an already CRC/sequence-checked 0x96 payload for one mapped ID.

    Structural contradictions raise GoalDiagnosticError with raw_payload_hex.
    A genuine failed READ or >4095 register word is returned with valid=False,
    position_ticks=None and untouched raw evidence. UART freshness, sequence and
    deployment identity must be checked by the caller, never inferred here.
    """
    if type(kind) is not int or kind != GOAL_READ_DETAILS_REPLY:
        raise GoalDiagnosticError('Wrong GoalReadDetails reply type', payload)
    if type(payload) is not bytes or len(payload) != GOAL_READ_DETAILS_SIZE:
        raise GoalDiagnosticError('Wrong GoalReadDetails payload type/length', payload)
    if type(expected_id) is not int or expected_id not in MOTOR_IDS:
        raise GoalDiagnosticError('Expected ID must be one mapped motor ID', payload)
    version, sid, index, reserved = payload[:4]
    if version != GOAL_READ_DETAILS_VERSION or reserved:
        raise GoalDiagnosticError('Wrong GoalReadDetails version/reserved byte', payload)
    if sid != expected_id or sid not in MOTOR_IDS or index != MOTOR_IDS.index(sid):
        raise GoalDiagnosticError('GoalReadDetails mapped ID/index mismatch', payload)
    desired, goals = struct.unpack_from('<HH', payload, 4)
    if desired > 4095 or goals > 4095:
        raise GoalDiagnosticError('MCU desired/goals outside raw position bounds', payload)
    armed_before, armed_after = payload[8:10]
    (cookie_before, cookie_after, sample_before, imu_before,
     sample_after, imu_after) = struct.unpack_from('<6I', payload, 10)
    if armed_before or armed_after or cookie_before or cookie_after:
        raise GoalDiagnosticError('GoalReadDetails violates the OFF-only armed/cookie contract', payload)
    if sample_before != sample_after or imu_before != imu_after:
        raise GoalDiagnosticError('GoalReadDetails unexpectedly refreshed feedback timestamps', payload)
    known_before, torque_before, known_after, torque_after = struct.unpack_from('<4H', payload, 34)
    if (known_before | torque_before | known_after | torque_after) & ~ALL_MOTORS_MASK:
        raise GoalDiagnosticError('GoalReadDetails torque mask exceeds mapped motors', payload)
    if known_before != ALL_MOTORS_MASK or torque_before or torque_after:
        raise GoalDiagnosticError('GoalReadDetails violates entry torque-known/OFF mask contract', payload)
    goal_read = _read_block(payload, 42, 42, 2)
    present_read = _read_block(payload, 60, 56, 6)
    return dict(kind='servo_goal_diagnostic', request_type=GOAL_READ_DETAILS,
                reply_type=GOAL_READ_DETAILS_REPLY, version=version, id=sid, index=index,
                raw_payload_hex=payload.hex(), valid=goal_read['valid'] and present_read['valid'],
                invalid=not (goal_read['valid'] and present_read['valid']),
                mcu_desired_ticks=desired, mcu_goals_ticks=goals,
                armed_before=False, armed_after=False,
                cookie_before=cookie_before, cookie_after=cookie_after,
                sample_before_us=sample_before, imu_before_us=imu_before,
                sample_after_us=sample_after, imu_after_us=imu_after,
                torque_known_before=known_before, torque_mask_before=torque_before,
                torque_known_after=known_after, torque_mask_after=torque_after,
                goal_read=goal_read, present_read=present_read,
                feedback_updated=False, sensor_freshness_established=False,
                motion_authorized=False)


def decode_goal_read_details_frame(frame, *, expected_id, expected_sequence):
    """Require an exact reply sequence from the existing CRC-checked frame parser."""
    payload = frame[2] if type(frame) is tuple and len(frame) == 3 else None
    if (type(frame) is not tuple or len(frame) != 3
            or type(expected_sequence) is not int or not 0 <= expected_sequence <= 0xffffffff
            or type(frame[1]) is not int or frame[1] != expected_sequence):
        raise GoalDiagnosticError('GoalReadDetails frame/sequence mismatch', payload)
    decoded = decode_goal_read_details(frame[0], payload, expected_id=expected_id)
    decoded['sequence'] = frame[1]
    return decoded
