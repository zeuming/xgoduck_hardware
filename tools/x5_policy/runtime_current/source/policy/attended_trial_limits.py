"""Consistent limits for the attended HOLD/zero/forward experiments.

15 degrees is the pre-existing HOLD/zero trial ceiling, NOT a calibrated fall
angle. Reuse that ceiling; do not introduce a tighter limit when a forward
pulse starts, or infer a mechanical fault from a cutoff. Invalid/stale feedback,
joint limits, STOP and MCU watchdogs remain independent and immediate.
"""
import math

TILT_LIMIT_DEG = 15.
ZERO_WINDOW_S = 30.
ZERO_COMPLETE = '零速试验30秒窗口结束，正在缓慢恢复固定站姿，扭矩保持'
PAGE_LOST_HOLD = '网页控制心跳中断，已取消行走；正在缓慢恢复固定站姿，扭矩保持'
TRACKING_COMPLETE = '关节跟随检查5秒结束，继续保持固定站姿，扭矩保持'
TILT_PREFIX = '实验倾角达到15°边界'
INVALID_ATTITUDE = '实验姿态无效，已中止并请求释放扭矩，请托稳机身'


def tilt_stop_reason(tilt):
    if tilt is None or not math.isfinite(tilt):
        return INVALID_ATTITUDE
    if tilt >= TILT_LIMIT_DEG:
        return f'{TILT_PREFIX}（当前 {tilt:.2f}°），已中止并请求释放扭矩，请托稳机身'
    return None


def classify_stop(reason):
    if reason == ZERO_COMPLETE:
        return dict(outcome='completed', cause='zero_time_limit')
    if reason == PAGE_LOST_HOLD:
        return dict(outcome='interrupted', cause='page_heartbeat_lost')
    if reason == TRACKING_COMPLETE:
        return dict(outcome='completed', cause='tracking_time_limit')
    if reason == INVALID_ATTITUDE:
        return dict(outcome='aborted', cause='invalid_attitude')
    if isinstance(reason, str) and reason.startswith(TILT_PREFIX):
        return dict(outcome='aborted', cause='tilt_limit')
    if reason is None:
        return dict(outcome='requested_stop', cause='stop_request')
    return dict(outcome='aborted', cause='control_error')


def stop_policy():
    return dict(protocol=1, tilt_limit_deg=TILT_LIMIT_DEG,
        applies_to=['hold', 'zero_walk', 'forward_pulse', 'zero_recovery', 'tracking_probe'],
        basis='Reuse pre-existing HOLD/zero experiment ceiling; remove unvalidated forward/probe-specific 8-degree cutoff.',
        fall_threshold_validated=False, physical_walking_verified=False,
        automatic_tilt_limit_change=False, tilt_debounce_s=0.,
        zero_window_s=ZERO_WINDOW_S, forward_completion='return_to_zero',
        zero_window_completion='blend_to_existing_hold',
        tracking_window_completion='continue_existing_hold',
        page_heartbeat_loss='cancel_walk_then_blend_to_hold_only_with_fresh_healthy_following',
        fault_action='request_torque_release',
        automatic_rearm=False, automatic_fault_hold=False)
