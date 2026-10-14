"""Explicit native guard adapter for the reviewed ZERO-ONLY candidate.

Construction without a transport is CPU validation only. It cannot start or
connect. NativeRuntime injects its original exclusive ReviewedTransport for
every live connection. Only the explicit, one-shot bounded forward preset can schedule nonzero motion.
"""
import collections
import copy
import hashlib
import json
import math
import struct
import threading
import time
import numpy as np
from production_motion import *
from production_motion import _axis_matrix, _default_config
from candidate_controller import ResearchMotionController
from control_readiness import ControlReadinessMixin
from takeover_math import TAKEOVER_SECONDS, blend_stance, takeover_weight
from attended_trial_limits import (TILT_LIMIT_DEG, ZERO_WINDOW_S, ZERO_COMPLETE, PAGE_LOST_HOLD,
    tilt_stop_reason, classify_stop, stop_policy)
from bounded_forward import ForwardPulse, ForwardTargetExpired, PEAK_M_S, DURATION_S, RAMP_S, WARMUP_S, RECOVERY_S


def _no_io_factory(port):
    raise MotionError('CPU validation-only controller cannot open a transport')


class MotionController(ControlReadinessMixin, ResearchMotionController):
    def __init__(self, sessions, config_path=None, port='native-validation-only', *,
                 transport_factory=None, clock=time.monotonic,
                 require_head_clearance=False, allow_fixed_head_walk=False,
                 hold_hip_pitch_bias_deg=0):
        if type(allow_fixed_head_walk) is not bool:
            raise MotionError('Candidate walk permission must be an explicit boolean')
        self._validation_only = transport_factory is None
        self._forward = ForwardPulse()
        self._trial_generation = None
        self._trial_began = None
        self._last_trial_stop = None
        self._trial_trace = collections.deque(maxlen=2100)
        self._trial_trace_total = 0
        self._completed_traces = collections.deque(maxlen=4)
        self._trace_sequence = 0
        self._inference_attempts = {'control': None, 'preflight': None}
        self._last_control_attempt = None
        super().__init__(sessions, config_path, port,
            transport_factory=_no_io_factory if self._validation_only else transport_factory,
            clock=clock, research_walk_enabled=allow_fixed_head_walk,
            research_nonzero_command=False, hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg)
        # Manual head capture was removed. Use the existing model HOME and
        # policy head targets; keep feedback, ranges, tilt and trial gates.
        self._require_head_clearance = False

    def capture_head_clearance(self, **kwargs):
        raise MotionError('手动头颈记录已移除，请刷新页面；站姿使用模型配置目标')


    def _forward_block_locked(self):
        if self._mode != 'walk': return '先恢复站姿并启动零速试跑'
        if self._forward.used: return '本轮试步已使用；不能重复触发'
        if not self._walk_warmup_ready_locked(): return '等待本轮连续5秒成功的零速控制'
        if self._trial_began is None or ZERO_WINDOW_S-(self._clock()-self._trial_began) < DURATION_S+RECOVERY_S+1.:
            return '本轮剩余时间不足，请停止后重新测试'
        try:
            self._motion_gate_locked('walk')
            if self._heartbeat_expired_locked(): raise MotionError('网页控制心跳已过期')
            if not self._target_times or not 0<=self._clock()-self._target_times[-1]<FRESH_SECONDS:
                raise MotionError('最近一次零速目标已过期')
            if np.any(self._command): raise MotionError('试步前指令必须为零速')
            if not self._state.get('armed') or self._state.get('torque_mask')!=ALL_MASK:
                raise MotionError('需要15舵机全部使能且无故障')
            following = self._leg_following_status_locked()
            if not following['ready']:
                worst = following['worst']
                raise MotionError(('零速关节跟随未到位：ID %d 偏差 %.2f°，要求小于 5°；先检查负载下跟随' % (worst['id'], abs(worst['error_deg']))) if worst else (following.get('reason') or '尚无有效关节跟随数据')+'；不能开始前进')
        except MotionError as error: return str(error)
        return None

    def forward_trial(self, *, expected_generation=None, support_confirmed=False):
        # Scheduling only: no transport request, ARM, heartbeat or mode change.
        with self._lock:
            if type(expected_generation) is not int or expected_generation!=self._generation:
                raise MotionError('状态已变化，请刷新后再试')
            reason=self._forward_block_locked()
            if reason: raise MotionError(reason)
            self._forward.begin(self._clock(),self._generation)
        return self.status()

    def command(self, forward, side, yaw):
        result=super().command(forward,side,yaw)
        with self._lock:
            self._forward.cancel()
        return result

    def _forward_command_locked(self):
        if self._mode=='walk' and self._forward.used:
            try:self._command[:]=self._forward.command_at(self._clock(),self._generation)
            except ValueError as error:raise MotionError(str(error)) from error

    def _execution_target_locked(self, mode, policy_target):
        target = super()._execution_target_locked(mode, policy_target)
        if mode != 'walk' or self._mode != 'walk':
            # Preflight still validates the complete original execution target
            # while HOLD is active. The ramp cannot hide a rejected model frame.
            return target
        if self._trial_generation != self._generation or self._trial_began is None:
            raise MotionError('零速接管计时状态失效，已停止')
        if np.any(self._command):
            if (self._clock()-self._trial_began<TAKEOVER_SECONDS or
                    not 0<float(self._command[0])<=PEAK_M_S+1e-7 or np.any(self._command[1:])):
                raise MotionError('受限试步指令无效')
            try:self._forward.check_inflight(self._clock(),self._generation,self._forward.epoch)
            except ValueError as error:raise MotionError(str(error)) from error
        pulse_epoch=getattr(policy_target,'forward_pulse_epoch',None)
        if pulse_epoch is not None:
            try:self._forward.check_inflight(self._clock(),self._generation,pulse_epoch)
            except ValueError as error:raise MotionError(str(error)) from error
        # Preserve the existing full leg-range checks on every inference, even
        # when the requested blended target would lie inside its bounds.
        self._raw_targets_locked(target, mode=mode)
        return blend_stance(self._fixed_stance_locked(), target,
                            self._clock() - self._trial_began)

    def start(self):
        if self._validation_only:
            raise MotionError('CPU validation-only controller cannot start')
        return super().start()

    def _connect_once(self):
        if self._validation_only:
            raise MotionError('CPU validation-only controller cannot connect')
        return super()._connect_once()

    def _trace_sample(self, event, target=None, error=None, at=None):
        """Capture only immutable scalars/tuples; no serialization or I/O."""
        state = self._state or {}
        filtered = self._filtered
        return (event, self._clock() if at is None else at, self._generation, self._mode,
            state.get('sample_us'), state.get('imu_us'), self._age_locked(),
            tuple(state.get('positions', ())), tuple(state.get('velocities', ())),
            tuple(filtered.angles) if filtered is not None else (),
            tuple(filtered.grav) if filtered is not None else (),
            tuple(filtered.gyro_rad) if filtered is not None else (),
            tuple(state.get('gyro', ())), tuple(state.get('acc', ())),
            tuple(map(float, self._command)), target, error,
            (state.get('motion_evidence') or {}).get('goal_write_us'),
            (state.get('motion_evidence') or {}).get('complete_servo_sample_sequence'),
            tuple((state.get('motion_evidence') or {}).get('written_goals_raw', ())),
            tuple((state.get('motion_evidence') or {}).get('present_load_raw', ())),
            (state.get('motion_evidence') or {}).get('protocol'),
            (state.get('motion_evidence') or {}).get('pairing_valid'),
            (state.get('motion_evidence') or {}).get('scan_start_us'),
            (state.get('motion_evidence') or {}).get('goal_write_sequence'))

    def _append_trace(self, sample):
        self._trial_trace.append(sample)
        self._trial_trace_total += 1

    def _request(self, kind, payload=b''):
        if kind != TARGET:
            return super()._request(kind, payload)
        # Sample before transmission. Do not include the ARM cookie in logs.
        target = tuple(struct.unpack('<I15H', payload)[1:])
        sample = self._trace_sample('target-attempt', target)
        try:
            result = super()._request(kind, payload)
        except BaseException as error:
            self._append_trace(sample + (self._clock(), str(error)))
            raise
        self._append_trace(sample + (self._clock(), None))
        self._forward.note_successful_target(sample[2],sample[1],self._clock(),sample[14])
        return result

    def _mode_started_locked(self, mode):
        self._forward.reset()
        self._last_trial_stop = None
        if mode != 'walk' and self._trial_generation is not None and self._trial_trace:
            # Retain the model segment before a later long HOLD can overwrite
            # its bounded ring. This records a transition, not an unload.
            self._retain_trial_trace_locked('mode-transition', '主动切换到 '+mode)
        # Commit the trial lease atomically with the parent mode/generation;
        # inference still releases the controller lock for concurrent STOP.
        if mode == 'walk':
            self._trial_generation = self._generation
            self._trial_began = self._clock()
        else:
            self._trial_generation = self._trial_began = None
        self._append_trace(self._trace_sample('start') + (self._clock(), None))

    def _motion_gate_locked(self, mode=None):
        if self._mode=='walk': self._forward_command_locked()
        selected = self._mode if mode is None else mode
        if selected in ('hold', 'walk'):
            # Do not change the tilt ceiling when a pulse starts or ends.
            # Read freshness remains a separate fail-closed check.
            self._require_feedback_locked()
            reason = tilt_stop_reason(self._tilt_locked())
            if reason: raise MotionError(reason)
        super()._motion_gate_locked(mode)
        selected = self._mode if mode is None else mode
        if selected == 'walk':
            if self._mode == 'walk':
                if self._trial_generation != self._generation or self._trial_began is None:
                    raise MotionError('零速试验计时状态失效，已停止')
                if self._clock() - self._trial_began >= ZERO_WINDOW_S:
                    raise MotionError(ZERO_COMPLETE)

    def _infer_policy_target(self, *args, **kwargs):
        # Bounded numeric diagnostics only. These timestamps never refresh a
        # sensor sample, authorize a target or change an inference result.
        began, cpu = time.perf_counter(), time.thread_time()
        role = 'control' if threading.current_thread() is self._thread else 'preflight'
        with self._lock:
            generation, sample = self._generation, self._last_sample
            age_before = self._age_locked()
        error = None
        try:
            with self._lock: pulse_epoch=self._forward.epoch
            result=super()._infer_policy_target(*args, **kwargs)
            if np.any(args[3]): result[0].forward_pulse_epoch=pulse_epoch
            return result
        except BaseException as exc:
            error = str(exc)
            raise
        finally:
            computed, computed_cpu = time.perf_counter(), time.thread_time()
            with self._lock:
                acquired = time.perf_counter()
                self._inference_attempts[role] = dict(
                    generation=generation, current_generation=self._generation,
                    sample_us=sample, current_sample_us=self._last_sample,
                    began_monotonic=began, ended_monotonic=acquired,
                    compute_wall_ms=(computed-began)*1000,
                    compute_cpu_ms=(computed_cpu-cpu)*1000,
                    record_lock_wait_ms=(acquired-computed)*1000,
                    feedback_age_before_ms=age_before*1000 if math.isfinite(age_before) else None,
                    feedback_age_after_ms=self._age_locked()*1000 if math.isfinite(self._age_locked()) else None,
                    error=error)

    def _control_once(self):
        began, cpu = time.perf_counter(), time.thread_time()
        with self._lock:
            _phase_outer_acquired = time.perf_counter()
            generation, mode, sample = self._generation, self._mode, self._last_sample
            age_before = self._age_locked()
        _phase_outer_released = time.perf_counter()
        reason = None
        try:
            return self._control_with_window_stop()
        except BaseException as error:
            reason = str(error)
            raise
        finally:
            _phase_record_wait = time.perf_counter()
            with self._lock:
                _phase_record_acquired = time.perf_counter()
                self._last_control_attempt = dict(generation=generation, mode=mode,
                    current_generation=self._generation, sample_us=sample,
                    current_sample_us=self._last_sample, began_monotonic=began,
                    ended_monotonic=time.perf_counter(),
                    wall_ms=(time.perf_counter()-began)*1000,
                    cpu_ms=(time.thread_time()-cpu)*1000,
                    feedback_age_before_ms=age_before*1000 if math.isfinite(age_before) else None,
                    feedback_age_after_ms=self._age_locked()*1000 if math.isfinite(self._age_locked()) else None, error=reason,
                    phase_timing_protocol=1,
                    wrapper_phases=(('wrapper_entry',began),('wrapper_lock_acquired',_phase_outer_acquired),
                        ('wrapper_lock_released',_phase_outer_released),('record_lock_wait',_phase_record_wait),
                        ('record_lock_acquired',_phase_record_acquired)),
                    candidate_precontrol_phases=getattr(self,'_last_candidate_precontrol_phases',()),
                    production_phases=getattr(self,'_last_production_control_phases',()),
                    recent_status_lock_timings=getattr(self,'_status_lock_timings',()))

    def _control_with_window_stop(self):
        # The deadline can be crossed during inference, after the cycle's
        # first gate. Complete its normal STOP/STATE verification in-place;
        # real feedback, target, tilt and UART errors still propagate.
        try:
            with self._lock:self._forward_command_locked()
            return super()._control_once()
        except ForwardTargetExpired as error:
            # Only a still-current, normally completed pulse may discard an
            # in-flight target without STOP. The next cycle re-infers at zero;
            # no old target, action history or sensor timestamp is committed.
            with self._lock:
                if (self._mode != 'walk' or self._generation != error.generation
                        or self._forward.generation != error.generation
                        or self._forward.epoch != error.epoch
                        or not self._forward.used
                        or self._forward.phase not in ('running', 'returned-to-zero')
                        or self._forward.started is None
                        or self._clock() < self._forward.started + DURATION_S):
                    raise MotionError('试步推理已取消，过期目标未执行') from error
                self._forward_command_locked()
            return None
        except MotionError as error:
            if str(error) != ZERO_COMPLETE:
                raise
            with self._lock:
                if (self._mode != 'walk' or self._trial_generation != self._generation
                        or self._trial_began is None or self._clock()-self._trial_began < ZERO_WINDOW_S):
                    raise
                self._complete_zero_trial_locked()

    def _handle_motion_gate_failure_locked(self, error):
        if str(error) != ZERO_COMPLETE:
            return super()._handle_motion_gate_failure_locked(error)
        try:
            self._complete_zero_trial_locked()
        except MotionError as failure:
            self._stop_locked(str(failure))

    def _complete_zero_trial_locked(self):
        """Normal completion only: no ARM/STOP, no new lease or generation.

        Reuse the existing three-second HOLD interpolation. Anchor at the last
        successfully executed target so the transition never steps the target.
        A bad live state raises into the unchanged emergency STOP path.
        """
        if (self._mode != 'walk' or self._trial_generation != self._generation
                or self._trial_began is None
                or self._clock()-self._trial_began < ZERO_WINDOW_S):
            raise MotionError('试跑结束状态已变化')
        self._return_walk_to_hold_locked(ZERO_COMPLETE)

    def _return_walk_to_hold_locked(self, reason, *, page_lost=False):
        self._motion_gate_locked('hold')
        if ((not page_lost and self._heartbeat_expired_locked()) or self._pending_stop
                or not self._state.get('configured')
                or not self._state.get('armed') or self._state.get('cookie') != self._cookie
                or self._state.get('torque_mask') != ALL_MASK
                or not self._target_times
                or not 0 <= self._clock()-self._target_times[-1] < FRESH_SECONDS
                or self._last_executed_target is None):
            raise MotionError('试跑结束时未满足保持站姿条件，不能继续驱动')
        if page_lost:
            following = self._leg_following_status_locked()
            if not following['ready']:
                raise MotionError('页面失联时不能确认恢复站姿条件：'+
                    (following.get('reason') or '关节跟随未到位'))
        anchor = list(self._last_executed_target)
        self._raw_targets_locked(anchor, mode='hold')
        self._raw_targets_locked(self._fixed_stance_locked(), mode='hold')
        self._retain_trial_trace_locked('completion', reason)
        self._last_trial_stop = dict(classify_stop(reason), reason=reason,
            mode_before='walk', recorded_monotonic=self._clock(),
            tilt_deg=self._tilt_locked(), tilt_limit_deg=TILT_LIMIT_DEG,
            torque_action='maintain_enabled', result='hold',
            page_heartbeat=self._page_heartbeat_status_locked(),
            leg_following=self._leg_following_status_locked())
        self._forward.cancel()
        self._command.fill(0)
        self._previous_action.fill(0)
        self._smooth_action.fill(0)
        self._last_action = None
        self._last_inference_ms = None
        self._hold_initial = anchor
        self._hold_begin = self._clock()
        self._mode = 'hold'
        self._trial_generation = self._trial_began = None
        if page_lost:
            # Invalidate queued commands and in-flight model results. Keep the
            # MCU cookie; do not re-arm or renew the expired browser lease.
            self._generation += 1
        # Normal completion retains generation; both paths reuse existing HOLD.

    def _retain_trial_trace_locked(self, event, reason):
        if self._trial_trace:
            terminal = self._trace_sample(event, error=reason) + (self._clock(), None)
            self._trace_sequence += 1
            self._completed_traces.append((self._trace_sequence, reason,
                tuple(self._trial_trace), terminal, self._trial_trace_total,
                dict(inference=copy.deepcopy(self._inference_attempts),
                     control=copy.deepcopy(self._last_control_attempt))))
            self._trial_trace.clear()
            self._trial_trace_total = 0

    def _stop_locked(self, reason=None):
        if reason == '网页控制心跳超时，已停止' and self._mode == 'walk':
            try:
                self._return_walk_to_hold_locked(PAGE_LOST_HOLD, page_lost=True)
                return
            except MotionError as failure:
                reason += '；恢复站姿条件未通过：' + str(failure)
        if self._mode in ('hold', 'walk'):
            tilt = self._tilt_locked()
            self._last_trial_stop = dict(classify_stop(reason), reason=reason,
                mode_before=self._mode, recorded_monotonic=self._clock(),
                tilt_deg=tilt if tilt is not None and math.isfinite(tilt) else None,
                tilt_limit_deg=TILT_LIMIT_DEG, torque_action='release_requested',
                page_heartbeat=self._page_heartbeat_status_locked(),
                leg_following=self._leg_following_status_locked())
        self._forward.cancel()
        self._retain_trial_trace_locked('stop', reason)
        self._trial_generation = self._trial_began = None
        return super()._stop_locked(reason)

    def completed_trial_traces(self, after_sequence=0):
        """Called only by the independent audit worker, never by HTTP polling."""
        with self._lock:
            pending = tuple(row for row in self._completed_traces if row[0] > after_sequence)
        names = ('event','monotonic','generation','mode','sample_us','imu_us','age_s',
            'actual_raw_ticks','velocity_raw','actual_angle_deg','gravity','filtered_body_gyro_rad_s',
            'raw_gyro_dps','raw_accel_ms2',
             'command','attempted_target_ticks','stop_reason',
            'MCU_written_goal_us','MCU_servo_sample_sequence','MCU_written_goal_ticks',
            'servo_present_load_raw','MCU_evidence_protocol','MCU_pairing_valid',
            'MCU_scan_start_us','MCU_goal_write_sequence',
            'request_ended_monotonic','request_error')
        def decode(row):
            result = dict(zip(names,row))
            if not math.isfinite(result['age_s']): result['age_s'] = None
            return result
        return [dict(sequence=seq, reason=reason, total_frames=total,
            stop_policy=stop_policy(),
            stop_classification=classify_stop(reason) if terminal[0] in ('stop','completion') else None,
            retained_frames=len(rows), truncated=total>len(rows), motor_ids=list(MOTOR_IDS),
            calibration=copy.deepcopy(self._config),
            latency_at_stop=copy.deepcopy(latency),
            actual_sample_is_pre_target=True, MCU_applied_goal_not_measured=True,
            MCU_written_goal_and_raw_load_optional=True,
            evidence_limits='MCU successful TX cache is not servo goal register readback; load words have no calibrated torque/current units.',
            nominal_firmware_slew_ticks_s=600, frames=[decode(row) for row in rows], terminal=decode(terminal))
            for seq,reason,rows,terminal,total,latency in pending]

    def status(self):
        value = super().status()
        value['candidate_profile'].update(native_zero_only_adapter=False, bounded_forward_trial_protocol=1,
            CPU_validation_only=self._validation_only, nonzero_command_locked=True,
            physical_standing_verified=False, physical_walking_verified=False,
            backend_trial_limit_s=ZERO_WINDOW_S, backend_trial_tilt_limit_deg=TILT_LIMIT_DEG,
            trial_stop_policy=stop_policy(), last_trial_stop=copy.deepcopy(self._last_trial_stop),
            trial_seconds_remaining=max(0.,ZERO_WINDOW_S-(self._clock()-self._trial_began))
                if self._mode=='walk' and self._trial_began is not None else None,
            completed_trace_sequence=self._trace_sequence,
            latency_diagnostics=dict(protocol=1,
                inference=copy.deepcopy(self._inference_attempts),
                control=copy.deepcopy(self._last_control_attempt)),
            trace_recording_frames=len(self._trial_trace), trace_recording_max_frames=2100)
        with self._lock:
            value['candidate_profile']['page_heartbeat'] = self._page_heartbeat_status_locked()
            value['candidate_profile']['leg_following'] = self._leg_following_status_locked()
        elapsed = (self._clock() - self._trial_began if self._mode == 'walk'
                   and self._trial_began is not None else None)
        value['candidate_profile']['zero_takeover'] = dict(
            duration_seconds=TAKEOVER_SECONDS, elapsed_seconds=elapsed,
            policy_weight=takeover_weight(elapsed) if elapsed is not None else None,
            reference='unchanged fixed standing target',
            actor_and_raw_previous_unchanged=True,
            full_unblended_leg_range_check=True, physical_validation_pending=True)
        with self._lock:
            reason=self._forward_block_locked()
            value['forward_trial']=dict(protocol=1,experimental=True,physical_walking_verified=False,
                mode_generation=self._generation,phase=self._forward.phase,used=self._forward.used,
                allowed=reason is None,blocked_reason=reason,peak_forward_m_s=PEAK_M_S,
                duration_s=DURATION_S,ramp_s=RAMP_S,automatic_return_to_zero=True,
                one_pulse_per_episode=True,required_zero_success_seconds=WARMUP_S,
                firmware_slew_ticks_s=600,firmware_changed=False,
                execution=dict(successful_nonzero_TARGETs=self._forward.positive_targets,
                    successful_zero_TARGETs_after_forward=self._forward.zero_targets_after_positive,
                    peak_sent_command_m_s=self._forward.peak_sent_command,
                    first_positive_at=self._forward.first_positive_at,
                    last_positive_at=self._forward.last_positive_at,
                    first_zero_after_positive_at=self._forward.first_zero_after_positive_at,
                    actual_walking_verified=False),
                remaining_s=max(0.,DURATION_S-(self._clock()-self._forward.started))
                    if self._forward.phase=='running' else 0.)
        return value


# Explicit fixed-stance diagnostic; generic model commands remain locked.
from tracking_probe import controller_class
MotionController = controller_class(MotionController, MotionError, MOTOR_IDS, ALL_MASK, FRESH_SECONDS)
