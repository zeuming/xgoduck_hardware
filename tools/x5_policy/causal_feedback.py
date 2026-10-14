"""Versioned sample-bound MCU TX evidence; never claims servo acceptance.

No I/O and no motion authority. Packet-v1 remains a diagnostic live TX cache;
only packet-v2 can establish that a target preceded the associated full scan.
"""
import math
import struct

PAIRED_EXTENSION = struct.Struct('<BB5I15H')
ALL_MASK = 0x7fff
LEG_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24)
FOLLOWING_TOLERANCE_DEG = 5.


def unsigned_interval(later, earlier):
    return (later - earlier) & 0xffffffff


def decode_paired_extension(payload, state):
    """Validate an explicit v2 response, including truthful validity claims."""
    if len(payload) != PAIRED_EXTENSION.size:
        raise ValueError('采样配对状态长度不匹配')
    version, valid, sequence, start, cookie, goal_us, goal_sequence, *goals = PAIRED_EXTENSION.unpack(payload)
    if version != 2 or valid not in (0, 1) or any(p > 4095 for p in goals):
        raise ValueError('采样配对状态字段无效')
    if valid and (not state.get('armed') or not cookie or cookie != state.get('cookie')
            or state.get('online_mask') != ALL_MASK or state.get('fault_mask') != 0
            or unsigned_interval(state['sample_us'], start) >= 0x80000000
            or unsigned_interval(start, goal_us) >= 0x80000000):
        raise ValueError('MCU声称有效的目标与反馈配对不一致')
    # Sequence and timestamps are modulo uint32. Zero is legal after wrap;
    # the MCU's explicit validity bit carries the successful-TX provenance.
    return dict(protocol=2, pairing_valid=bool(valid),
        complete_servo_sample_sequence=sequence, scan_start_us=start,
        scan_arm_cookie=cookie, goal_write_us=goal_us,
        goal_write_sequence=goal_sequence, written_goals_raw=goals,
        target_source='MCU_sample_bound_transmitted_goal',
        servo_goal_register_readback=False)


def following_status(state, joints, *, mode, age_s, fresh_seconds, expected_cookie):
    """Compare positions from the same raw scan, with no host-goal fallback."""
    base = dict(protocol=2, ready=False, status='unknown',
        tolerance_deg=FOLLOWING_TOLERANCE_DEG,
        tolerance_basis='existing_fixed_stance_arrival',
        target_source='unavailable_sample_bound_goal',
        servo_goal_register_readback=False, physical_stability_verified=False,
        joints=[], worst=None)
    state = state or {}
    if mode not in ('hold', 'walk') or not state.get('armed'):
        return dict(base, status='inactive', reason='当前没有使能中的站姿或行走')
    evidence = state.get('motion_evidence') or {}
    if evidence.get('protocol') != 2:
        return dict(base, reason='固件尚未提供与采样绑定的目标，不能判定关节跟随')
    if not evidence.get('pairing_valid'):
        return dict(base, reason='本帧目标与采样未形成有效配对，不能判定关节跟随')
    if (not math.isfinite(age_s) or not 0 <= age_s < fresh_seconds
            or state.get('online_mask') != ALL_MASK or state.get('fault_mask') != 0
            or state.get('torque_mask') != ALL_MASK
            or not expected_cookie or state.get('cookie') != expected_cookie
            or evidence.get('scan_arm_cookie') != expected_cookie):
        return dict(base, reason='配对反馈已过期、会话已变化或舵机状态不完整')
    actual, goals = state.get('positions'), evidence.get('written_goals_raw')
    if (not isinstance(actual, (list, tuple)) or not isinstance(goals, (list, tuple))
            or len(actual) != 15 or len(goals) != 15 or len(joints) != 15
            or any(type(v) is not int or not 0 <= v <= 4095 for v in (*actual, *goals))):
        return dict(base, reason='配对的原始角度数据无效')
    rows = []
    for i, sid in enumerate(LEG_IDS):
        joint = joints[i]
        zero, sign = joint.get('zero'), joint.get('sign')
        if type(zero) is not int or not 0 <= zero <= 4095 or type(sign) is not int or sign not in (-1, 1):
            return dict(base, reason='关节校准无效')
        measured = (actual[i] - zero) * 360 / 4095 * sign
        desired = (goals[i] - zero) * 360 / 4095 * sign
        rows.append(dict(id=sid, actual_deg=measured, target_deg=desired,
                         error_deg=desired-measured))
    worst = max(rows, key=lambda row: abs(row['error_deg']))
    ready = abs(worst['error_deg']) < FOLLOWING_TOLERANCE_DEG
    return dict(base, ready=ready, status='ready' if ready else 'tracking_error',
        reason=None if ready else '同一轮目标与反馈的关节偏差达到跟随界限',
        target_source='MCU_sample_bound_transmitted_goal',
        joints=rows, worst=worst,
        complete_servo_sample_sequence=evidence.get('complete_servo_sample_sequence'),
        goal_write_sequence=evidence.get('goal_write_sequence'))
