"""Attended ID11 check relative to fresh encoder positions, never policy HOME.

This module opens no device. The injected native controller retains ownership of
ARM, feedback validation, TARGET, STOP and firmware watchdog handling.
"""
import copy
import math
import struct

JOINT_ID = 11
DELTA_DEG = -3.0
SETTLE_S = 1.0
RAMP_S = 1.2
PLATEAU_S = 1.5
RETURN_S = 1.2
RECOVERY_S = 1.1
DURATION_S = SETTLE_S + RAMP_S + PLATEAU_S + RETURN_S + RECOVERY_S


def offset_at(elapsed):
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('当前位置检查时钟无效')
    t = elapsed - SETTLE_S
    if t <= 0:
        return 0.0
    if t < RAMP_S:
        return DELTA_DEG * t / RAMP_S
    if t < RAMP_S + PLATEAU_S:
        return DELTA_DEG
    if t < RAMP_S + PLATEAU_S + RETURN_S:
        return DELTA_DEG * (1 - (t - RAMP_S - PLATEAU_S) / RETURN_S)
    return 0.0


def controller_class(base, motion_error, motor_ids, all_mask, target_kind):
    index = motor_ids.index(JOINT_ID)

    class CurrentPoseProbeController(base):
        def __init__(self, *args, **kwargs):
            self._current_probe = None
            self._current_probe_samples = []
            super().__init__(*args, **kwargs)

        def _probe_active(self):
            return self._current_probe is not None and self._current_probe['phase'] in (
                'preparing', 'running', 'completed')

        def _probe_baseline_locked(self):
            # Use raw encoder positions, not an assumed standing pose or an
            # averaged historical display value. Parent feedback gates apply.
            self._require_feedback_locked()
            raw = list(self._state['positions'])
            angles = [(p-j['zero']) * 360 / 4095 * j['sign']
                      for p, j in zip(raw, self._config['joints'])]
            moved = list(angles)
            moved[index] += DELTA_DEG
            raw_base = super()._raw_targets_locked(angles, mode='hold')
            raw_end = super()._raw_targets_locked(moved, mode='hold')
            joint = self._config['joints'][index]
            if raw_base != raw or not all(joint['min']+3 <= p <= joint['max']-3
                                          for p in (raw[index], raw_end[index])):
                raise motion_error('左髋当前位置或微动终点距已保存范围不足3刻度')
            return angles, raw, raw_end[index]

        def current_pose_probe(self, *, expected_generation=None,
                               support_confirmed=False, clearance_confirmed=False):
            with self._lock:
                if type(expected_generation) is not int or expected_generation != self._generation:
                    raise motion_error('状态已变化，当前位置检查未启动')
                if support_confirmed is not True or clearance_confirmed is not True:
                    raise motion_error('请确认托稳机身、双脚着地和当前头颈及腿部间隙')
                self._require_off_locked()
                self._require_feedback_locked()
                if (self._pending_stop or self._state['armed'] or self._state['torque_mask']
                        or self._state['last_error'] or not self._calibrated_locked()
                        or self._state['online_mask'] != all_mask or self._state['fault_mask']
                        or self._filtered is None):
                    raise motion_error('当前位置检查需要完整校准和已确认停止、卸力的状态')
                if self._current_probe is not None:
                    raise motion_error('本连接已执行过当前位置检查；不能重复使能')
                angles, raw, end = self._probe_baseline_locked()
                self._current_probe = dict(phase='preparing', baseline_deg=angles,
                    baseline_raw=raw, endpoint_raw=end, started=None, generation=None,
                    calibration=copy.deepcopy(self._config), result=None)
                # Explicit clearance confirmation is scoped to this captured
                # pose. Clear it on STOP; never save it as a standing reference.
                self._hold_head_deg = dict(zip((30, 31, 32, 33), angles[10:14]))
                self._standing_reference_head = dict(self._hold_head_deg)
                self._standing_reference_calibration = copy.deepcopy(self._config)
                try:
                    super().start_mode('hold', support_confirmed=True,
                                       expected_generation=expected_generation)
                    # ARM itself latches fresh positions. Recapture its reply
                    # before the first TARGET; the outer lock prevents the
                    # sampling/control thread from transmitting in between.
                    angles, raw, end = self._probe_baseline_locked()
                    self._current_probe.update(baseline_deg=angles, baseline_raw=raw,
                        endpoint_raw=end, started=self._clock(), generation=self._generation,
                        phase='running')
                    self._hold_head_deg = dict(zip((30, 31, 32, 33), angles[10:14]))
                    self._standing_reference_head = dict(self._hold_head_deg)
                    self._hold_initial = list(angles)
                    self._hold_begin = self._clock() - 3.0
                    self._current_probe_samples = []
                    self._retain_trial_trace_locked('diagnostic-boundary', '当前位置检查前的记录')
                    self._append_trace(self._trace_sample('current-pose-probe-start') + (self._clock(), None))
                except BaseException as error:
                    # An ARM may already have succeeded: use the same STOP and
                    # release verification as every other native start failure.
                    self._stop_locked('当前位置检查启动失败：' + str(error))
                    try:
                        self._service_stop_locked()
                    except Exception as stop_error:
                        self._disconnect('当前位置检查停止确认失败：' + str(stop_error))
                    raise
            return self.status()

        def _fixed_stance_locked(self):
            if not self._probe_active():
                return super()._fixed_stance_locked()
            p = self._current_probe
            values = list(p['baseline_deg'])
            if p['phase'] == 'running':
                try:
                    values[index] += offset_at(self._clock() - p['started'])
                except ValueError as error:
                    raise motion_error(str(error)) from error
            return values

        def _motion_gate_locked(self, mode=None):
            if self._probe_active():
                p = self._current_probe
                if (mode or self._mode) != 'hold' or self._config != p['calibration']:
                    raise motion_error('当前位置检查模式或校准已变化')
                if p['phase'] != 'preparing' and self._generation != p['generation']:
                    raise motion_error('当前位置检查状态已变化')
            return super()._motion_gate_locked(mode)

        def _request(self, kind, payload=b''):
            if kind == target_kind and self._probe_active():
                p = self._current_probe
                values = struct.unpack('<I15H', payload)[1:]
                low, high = sorted((p['baseline_raw'][index], p['endpoint_raw']))
                if (any(v != p['baseline_raw'][i] for i, v in enumerate(values) if i != index)
                        or not low <= values[index] <= high):
                    raise motion_error('当前位置检查拒绝非左髋变化或超过3°的目标')
            return super()._request(kind, payload)

        def _control_once(self):
            # Only the explicit current-pose diagnostic needs an outer lock.
            # Ordinary policy control must retain its own unlocked inference
            # phase so STOP/generation changes can discard an in-flight result.
            with self._lock:
                if self._probe_active():
                    return self._control_probe_locked()
            return super()._control_once()

        def _control_probe_locked(self):
            # Caller holds the lock across this non-inference HOLD diagnostic.
            p = self._current_probe
            self._motion_gate_locked('hold')
            super()._control_once()
            if p['phase'] != 'running' or self._mode != 'hold':
                return
            elapsed = self._clock() - p['started']
            actual = self._filtered.angles[index]
            # Bounded diagnostics: captures before the next feedback poll.
            if len(self._current_probe_samples) < 600:
                self._current_probe_samples.append((elapsed, float(actual)))
            if elapsed < DURATION_S:
                return
            if (self._pending_stop or not self._state['armed']
                    or self._state['torque_mask'] != all_mask
                    or self._state['cookie'] != self._cookie
                    or self._last_executed_target != p['baseline_deg']):
                raise motion_error('当前位置检查未确认回到起点，不能标记完成')
            plateau = [a for t, a in self._current_probe_samples
                       if SETTLE_S+RAMP_S+.5 <= t < SETTLE_S+RAMP_S+PLATEAU_S]
            p['result'] = dict(sample_count=len(self._current_probe_samples),
                plateau_samples=len(plateau),
                requested_delta_deg=DELTA_DEG,
                mean_plateau_delta_deg=(sum(plateau)/len(plateau)-p['baseline_deg'][index]) if plateau else None,
                return_error_deg=actual-p['baseline_deg'][index],
                encoder_following_only=True, physical_support_verified=False)
            p['phase'] = 'completed'
            self._retain_trial_trace_locked('diagnostic-complete', '左髋当前位置检查结束，保持起始位置和扭矩')

        def _hold_ready_locked(self):
            # Reaching an arbitrary current pose must never authorize WALK or
            # gamepad takeover as though the robot had reached validated HOME.
            if self._probe_active():
                return False
            return super()._hold_ready_locked()

        def start_mode(self, *args, **kwargs):
            with self._lock:
                if self._probe_active():
                    raise motion_error('当前位置检查及保持期间不能切换动作；请先托稳并停止')
                return super().start_mode(*args, **kwargs)

        def _stop_locked(self, reason=None):
            p = self._current_probe
            try:
                return super()._stop_locked(reason)
            finally:
                if p is not None:
                    p['phase'] = 'stopped' if p['phase'] in ('completed', 'stopped') else 'aborted'
                    p['stop_reason'] = reason
                    self._hold_head_deg = None
                    self._standing_reference_head = None
                    self._standing_reference_calibration = None

        def status(self):
            result = super().status()
            with self._lock:
                p = self._current_probe
                result['current_pose_probe'] = dict(protocol=1, joint_id=JOINT_ID,
                    delta_deg=DELTA_DEG, duration_s=DURATION_S,
                    phase=p['phase'] if p else 'idle',
                    baseline_deg=copy.deepcopy(p['baseline_deg']) if p else None,
                    result=copy.deepcopy(p['result']) if p else None,
                    all_servos_enabled_during_probe=True, other_joint_targets_frozen=True,
                    uses_policy_home=False, actor_used=False, automatic_release=False,
                    completed_pose_is_standing_reference=False,
                    stop_reason=p.get('stop_reason') if p else None)
            return result

    return CurrentPoseProbeController
