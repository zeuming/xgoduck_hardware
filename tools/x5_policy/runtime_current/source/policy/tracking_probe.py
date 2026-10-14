"""Explicit bounded HOLD diagnostic. No model, gain or calibration changes."""
import copy
import math
from attended_trial_limits import TRACKING_COMPLETE, TILT_LIMIT_DEG

KNEE_ID = 23
JOINTS = {14: "左踝", 11: "左髋侧摆", 21: "右髋侧摆", 23: "右膝"}
AMPLITUDE_DEG = 3.
RAMP_S = 1.2
PLATEAU_S = 1.5
RETURN_S = 1.2
RECOVERY_S = 1.1
DURATION_S = RAMP_S + PLATEAU_S + RETURN_S + RECOVERY_S
COMPLETE = TRACKING_COMPLETE


def offset_at(elapsed):
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('关节检查单调时钟无效')
    if elapsed < RAMP_S:
        return AMPLITUDE_DEG * elapsed / RAMP_S
    if elapsed < RAMP_S + PLATEAU_S:
        return AMPLITUDE_DEG
    if elapsed < RAMP_S + PLATEAU_S + RETURN_S:
        return AMPLITUDE_DEG * (1 - (elapsed - RAMP_S - PLATEAU_S) / RETURN_S)
    return 0.


def controller_class(base, motion_error, motor_ids, all_mask, fresh_seconds):
    class TrackingMotionController(base):
        def __init__(self, *args, **kwargs):
            self._tracking = None
            super().__init__(*args, **kwargs)

        def _tracking_block_locked(self, joint_id=KNEE_ID):
            if type(joint_id) is not int or joint_id not in JOINTS:
                return "关节检查仅支持左踝、左右髋侧摆和右膝"
            if self._tracking is not None:
                return '本轮关节检查已使用；先停止，再重新恢复站姿'
            if self._mode != 'hold' or not self._hold_ready_locked():
                return '先恢复固定站姿，等待实际关节到位'
            try:
                self._motion_gate_locked('hold')
                if self._heartbeat is None or not 0 <= self._clock() - self._heartbeat < .4:
                    raise motion_error('关节检查需要本页新鲜控制心跳')
                if (not self._state.get('armed') or self._state.get('torque_mask') != all_mask
                        or not self._target_times
                        or not 0 <= self._clock() - self._target_times[-1] < fresh_seconds):
                    raise motion_error('关节检查需要15舵机使能及新鲜的成功站姿目标')
                target = super()._fixed_stance_locked()
                target[motor_ids.index(joint_id)] += AMPLITUDE_DEG
                raw = self._raw_targets_locked(target, mode='hold')
                index = motor_ids.index(joint_id)
                joint = self._config['joints'][index]
                baseline = self._raw_targets_locked(super()._fixed_stance_locked(), mode='hold')[index]
                if not all(joint['min'] + 3 <= tick <= joint['max'] - 3 for tick in (raw[index], baseline)):
                    raise motion_error(JOINTS[joint_id] + '检查目标距已保存安全范围不足3刻度')
            except motion_error as error:
                return str(error)
            return None

        def tracking_probe(self, *, expected_generation=None, support_confirmed=False, joint_id=KNEE_ID):
            # Scheduling only. ARM/TARGET remain in the original controller loop.
            with self._lock:
                if type(joint_id) is not int or joint_id not in JOINTS:
                    raise motion_error('关节检查仅支持左踝、左右髋侧摆和右膝')
                if type(expected_generation) is not int or expected_generation != self._generation:
                    raise motion_error('状态已变化，请刷新后再试')
                if support_confirmed is not True:
                    raise motion_error('请确认本次托稳机身、双脚平放并观察所选关节')
                reason = self._tracking_block_locked(joint_id)
                if reason:
                    raise motion_error(reason)
                self._tracking = dict(generation=self._generation, started=self._clock(), phase='running', id=joint_id)
                # Isolate the five-second evidence from a long preceding HOLD.
                self._retain_trial_trace_locked('diagnostic-boundary', '站姿关节检查前的记录')
                self._append_trace(self._trace_sample('tracking-start') + (self._clock(), None))
            return self.status()

        def _fixed_stance_locked(self):
            target = super()._fixed_stance_locked()
            probe = self._tracking
            if probe and probe['phase'] == 'running' and self._mode == 'hold':
                if probe['generation'] != self._generation:
                    raise motion_error('关节检查状态已变化')
                try:
                    offset = offset_at(self._clock() - probe['started'])
                except ValueError as error:
                    raise motion_error(str(error)) from error
                target[motor_ids.index(probe['id'])] += offset
            return target

        def _motion_gate_locked(self, mode=None):
            super()._motion_gate_locked(mode)
            probe = self._tracking
            if probe and probe['phase'] == 'running':
                if self._mode != 'hold' or self._generation != probe['generation']:
                    raise motion_error('关节检查状态已变化')
                elapsed = self._clock() - probe['started']
                if elapsed < 0:
                    raise motion_error('关节检查单调时钟倒退')
                # Completion is committed only by the control loop after the
                # full live gate. Display/status reads must never finish it.
                if elapsed >= DURATION_S:
                    return

        def _heartbeat_expired_locked(self):
            if self._tracking and self._tracking['phase'] == 'running':
                return self._heartbeat is None or not 0 <= self._clock() - self._heartbeat < .4
            return super()._heartbeat_expired_locked()

        def _control_once(self):
            with self._lock:
                probe = self._tracking
                if probe and probe['phase'] == 'running' and self._clock()-probe['started'] >= DURATION_S:
                    self._motion_gate_locked('hold')
                    if (self._heartbeat_expired_locked() or self._pending_stop
                            or not self._state.get('configured') or not self._state.get('armed')
                            or self._state.get('cookie') != self._cookie
                            or self._state.get('torque_mask') != all_mask):
                        raise motion_error('检查结束时舵机状态无效，不能继续保持站姿')
                    self._tracking = dict(probe, phase='completed')
                    self._retain_trial_trace_locked('completion', COMPLETE)
                    self._last_trial_stop = dict(outcome='completed', cause='tracking_time_limit',
                        reason=COMPLETE, mode_before='hold', recorded_monotonic=self._clock(),
                        tilt_deg=self._tilt_locked(), tilt_limit_deg=TILT_LIMIT_DEG,
                        torque_action='maintain_enabled', result='hold')
            return super()._control_once()

        def _stop_locked(self, reason=None):
            if self._tracking and self._tracking['phase'] == 'running':
                self._tracking = dict(self._tracking,
                    phase='completed' if reason == COMPLETE else 'cancelled')
            return super()._stop_locked(reason)

        def start_mode(self, *args, **kwargs):
            with self._lock:
                if self._tracking and self._tracking['phase'] == 'running':
                    raise motion_error('关节检查进行中；请先使用停止按钮')
            return super().start_mode(*args, **kwargs)

        def _mode_started_locked(self, mode):
            self._tracking = None
            return super()._mode_started_locked(mode)

        def status(self):
            value = super().status()
            with self._lock:
                probe = copy.deepcopy(self._tracking)
                joint_id = probe['id'] if probe else KNEE_ID
                reasons = {sid: self._tracking_block_locked(sid) for sid in JOINTS}
                reason = reasons[joint_id]
                value['tracking_probe'] = dict(protocol=1, id=joint_id, joint_name=JOINTS[joint_id],
                    allowed_joint_ids=list(JOINTS),
                    allowed_by_joint={str(sid): why is None for sid, why in reasons.items()},
                    blocked_reasons_by_joint={str(sid): why for sid, why in reasons.items()},
                    amplitude_deg=AMPLITUDE_DEG,
                    duration_s=DURATION_S, tilt_limit_deg=TILT_LIMIT_DEG, ramp_s=RAMP_S, plateau_s=PLATEAU_S,
                    return_s=RETURN_S, recovery_s=RECOVERY_S, automatic_stop=False, automatic_hold=True,
                    mode_generation=self._generation, allowed=reason is None, blocked_reason=reason,
                    phase=probe['phase'] if probe else 'idle', used=probe is not None,
                    started_monotonic=probe['started'] if probe else None,
                    remaining_s=max(0., DURATION_S - (self._clock() - probe['started']))
                        if probe and probe['phase'] == 'running' else 0.,
                    actor_used=False, firmware_gains_calibration_unchanged=True,
                    physical_following_verified=False, physical_walking_verified=False)
            return value

    return TrackingMotionController
