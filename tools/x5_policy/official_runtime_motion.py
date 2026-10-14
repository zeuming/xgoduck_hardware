"""Native transport adapter for the unmodified official walking policy.

No experimental HOME, action remapping, head fixation, following gate, timed
trial, warmup lease, or takeover interpolation. Construction performs no I/O.
Calibration, command ownership, explicit ARM, STOP and feedback checks remain.
"""
import collections
import copy
import math
import struct
import threading
import numpy as np
from production_motion import *
from production_motion import MotionController as TransportController, _axis_matrix, _default_config
from official_recovery import Recovery

POLICY_POSITION_CAPS = 24
OFFICIAL_RUNTIME_CAPS = 27


def _no_io_factory(port):
    raise MotionError('Validation-only controller cannot open hardware')


class MotionController(TransportController):
    def __init__(self, sessions, config_path=None, port='validation-only', *,
                 transport_factory=None, clock=time.monotonic,
                 require_head_clearance=False, allow_fixed_head_walk=False,
                 hold_hip_pitch_bias_deg=0):
        if hold_hip_pitch_bias_deg != 0:
            raise MotionError('Official runtime does not accept experimental stance offsets')
        self._validation_only = transport_factory is None
        self._official_firmware = False
        self._recovery = Recovery()
        self._trace = collections.deque(maxlen=3000)
        self._trace_total = 0
        self._completed = collections.deque(maxlen=4)
        self._trace_sequence = 0
        self._last_trial_stop = None
        self._head_projection = []
        super().__init__(sessions, config_path, port,
            transport_factory=transport_factory or _no_io_factory, clock=clock,
            require_head_clearance=False, allow_fixed_head_walk=False,
            hold_hip_pitch_bias_deg=0)

    def start(self):
        if self._validation_only:
            raise MotionError('Validation-only controller cannot start')
        return super().start()

    def _connect_once(self):
        self._official_firmware = False
        super()._connect_once()
        try:
            with self._lock:
                if self._mode != 'off' or self._state['armed'] or self._state['torque_mask']:
                    raise MotionError('Firmware negotiation requires torque OFF')
                # Old profile-1 firmware still sets KP8/10/12.
                if self._request(OFFICIAL_RUNTIME_CAPS) != bytes([OFFICIAL_RUNTIME_CAPS, 0]):
                    raise MotionError('请安装配套官方增益固件；旧固件不能启动新版运控')
                self._official_firmware = True
        except BaseException as error:
            self._disconnect(error)
            raise

    def _disconnect(self, error):
        self._official_firmware = False
        return super()._disconnect(error)

    def _target_payload_locked(self, raw):
        if not self._official_firmware:
            raise MotionError('Official execution firmware is not verified')
        # Preserve the separately installed slider-maintenance wire contract.
        follow = getattr(self, '_joint_follow', None)
        if follow and follow['active']:
            return struct.pack('<I15H', self._cookie, *raw)
        return struct.pack('<I15H', self._cookie, *raw) + b'\x01'

    def _fixed_stance_locked(self):
        return None if self._stand_deg is None else list(self._stand_deg)

    def _walk_head_command_locked(self):
        return np.zeros(4, dtype=np.float32)

    def _execution_target_locked(self, mode, policy_target):
        result = list(policy_target)
        self._head_projection = []
        if mode == 'walk':
            # Physical head travel saved by the user can be smaller than the
            # actor's head requests. Preserve every leg target and raw history.
            # This projection has a separate four-seed paired gait acceptance.
            for index, joint in enumerate(self._config['joints']):
                if joint['id'] not in (30, 31, 32, 33):
                    continue
                lo, hi = sorted((tick-joint['zero'])*360/4095*joint['sign']
                                for tick in (joint['min'], joint['max']))
                bounded = min(hi, max(lo, result[index]))
                if bounded != result[index]:
                    self._head_projection.append(dict(id=joint['id'],
                        requested_deg=float(result[index]), executed_deg=float(bounded)))
                result[index] = bounded
        return result

    def capture_head_clearance(self, **kwargs):
        raise MotionError('官方运行模式不固定头颈；不需要手动记录头颈位置')

    def _hold_ready_locked(self):
        # Readiness is connection/enablement, not an arbitrary 5-degree pose gate.
        try:
            self._require_feedback_locked()
            return bool(self._mode == 'hold' and self._state['armed'] and
                        self._state['torque_mask'] == ALL_MASK and not self._pending_stop)
        except MotionError:
            return False

    def _motion_gate_locked(self, mode=None):
        selected = self._mode if mode is None else mode
        self._require_feedback_locked()
        if not self._calibrated_locked():
            raise MotionError('需要完整的关节零点、方向、保存范围和 IMU 校准')
        state = self._state
        if state['online_mask'] != ALL_MASK or state['fault_mask']:
            raise MotionError('舵机反馈缺失或报告故障')
        if not state['imu_ok'] or self._imu_age_locked() >= FRESH_SECONDS or self._filtered is None:
            raise MotionError('需要新鲜有效的 IMU 与关节反馈')
        if not all(math.isfinite(v) for v in self._filtered.grav) or sum(v*v for v in self._filtered.grav) < 1e-12:
            raise MotionError('姿态反馈无效')
        if selected != 'shadow' and not self._official_firmware:
            raise MotionError('配套官方增益固件尚未确认')
        if self._stand_deg is None:
            raise MotionError('行走模型元数据未加载')
        for p, j in zip(state['positions'], self._config['joints']):
            if not j['min'] <= p <= j['max']:
                raise MotionError(f"ID {j['id']} 实际位置超出保存范围")

    def _model_gate_locked(self, mode):
        self._motion_gate_locked(mode)
        if mode not in self.sessions:
            raise MotionError('所选官方模型未加载')
        if mode == 'walk' and 'getup' not in self.sessions:
            raise MotionError('官方行走恢复流程需要同时加载起身模型')

    def command(self, forward, side, yaw):
        values = (forward, side, yaw)
        limits = (.4, .3, 1.)  # official python/controller.py command()
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or
               not math.isfinite(v) or abs(v) > limit for v, limit in zip(values, limits)):
            raise MotionError('速度指令超出官方范围：前后±0.4、侧向±0.3 m/s，转向±1 rad/s')
        with self._lock:
            if self._mode not in ('walk', 'shadow'):
                raise MotionError('先启动行走模型')
            self._command_revision += 1
            self._command[:] = values
        return self.status()

    def forward_trial(self, **kwargs):
        # An old tab must never turn its formerly bounded button into unbounded motion.
        raise MotionError('旧版2秒试步接口已移除；请刷新页面使用持续前进/回零控制')

    def tracking_probe(self, **kwargs):
        raise MotionError('正常运控已移除实验性关节试探；请使用舵机工作台的独立诊断')

    def current_pose_probe(self, **kwargs):
        raise MotionError('正常运控已移除实验性姿态试探；请使用舵机工作台的独立诊断')

    def _mode_started_locked(self, mode):
        self._recovery.reset()
        self._last_trial_stop = None

    def _heartbeat_timeout_locked(self):
        return HEARTBEAT_SECONDS

    def _heartbeat_expired_locked(self):
        return self._mode != 'off' and (self._heartbeat is None or
            self._clock() - self._heartbeat >= HEARTBEAT_SECONDS)

    def _policy_context_locked(self):
        return super()._policy_context_locked() + (self._recovery.phase,)

    def _control_once(self):
        with self._lock:
            if self._mode == 'walk':
                self._motion_gate_locked('walk')
                if self._recovery.update(self._filtered.grav, self._clock(), True):
                    self._cancel_policy_job()
                    self._previous_action.fill(0)
                    self._smooth_action.fill(0)
        return super()._control_once()

    def _select_policy_locked(self, mode):
        if mode == 'walk' and self._recovery.phase != 'walk':
            return 'getup'
        return self._preview_model if mode == 'shadow' else mode

    def _default_recovery_target_locked(self):
        if self._mode == 'walk' and self._recovery.phase == 'default_pose':
            return stance_from_home(self._policy_profiles['getup']['home_rad'])
        return None

    def _decode_policy_action(self, selected, raw, previous, smooth, elapsed, inference_ms):
        raw = np.asarray(raw)
        if raw.shape != (1, 14) or not np.isfinite(raw).all():
            raise MotionError('官方模型输出必须为有限的 [1,14]')
        action = raw.reshape(14).astype(np.float32, copy=True)
        filtered = np.asarray(smooth, dtype=np.float32).copy()
        if filtered.shape != (14,) or not np.isfinite(filtered).all():
            raise MotionError('动作滤波状态无效')
        for sl, alpha in ((slice(0,5), ACTION_ALPHA), (slice(5,10), ACTION_ALPHA_HEAD), (slice(10,14), ACTION_ALPHA)):
            if selected == 'roulade': alpha = .15
            filtered[sl] *= alpha
            filtered[sl] += action[sl] * (1-alpha)
        profile = self._policy_profiles[selected]
        mouth = 30. if selected == 'pick' and elapsed / EPISODE_SECONDS['pick'] < .4 else 0.
        target = action_to_deg(filtered, mouth_deg=mouth, home_rad=profile['home_rad'], action_scale=profile['scale'])
        return target, action, filtered, inference_ms

    def _trace_sample(self, event, target=None, error=None):
        state = self._state or {}
        return dict(event=event, monotonic=self._clock(), mode=self._mode,
            generation=self._generation, sample_us=state.get('sample_us'),
            actual_raw_ticks=list(state.get('positions', ())), command=self._command.tolist(),
            attempted_target_ticks=target, error=error, recovery=self._recovery.status())

    def _append_trace(self, row):
        self._trace.append(row); self._trace_total += 1

    def _request(self, kind, payload=b''):
        if kind != TARGET:
            return super()._request(kind, payload)
        row = self._trace_sample('target', list(struct.unpack_from('<I15H', payload)[1:]))
        try:
            return super()._request(kind, payload)
        except BaseException as error:
            row['error'] = str(error)
            raise
        finally:
            row['request_ended_monotonic'] = self._clock()
            self._append_trace(row)

    def _retain_trial_trace_locked(self, event, reason):
        if self._trace:
            self._trace_sequence += 1
            self._completed.append(dict(sequence=self._trace_sequence, event=event,
                reason=reason, frames=list(self._trace), total_frames=self._trace_total,
                truncated=self._trace_total > len(self._trace), hardware_walking_verified=False))
            self._trace.clear(); self._trace_total = 0

    def completed_trial_traces(self, after_sequence=0):
        with self._lock:
            return copy.deepcopy([x for x in self._completed if x['sequence'] > after_sequence])

    def _stop_locked(self, reason=None):
        self._retain_trial_trace_locked('stop', reason)
        self._recovery.reset()
        return super()._stop_locked(reason)

    def status(self):
        result = super().status()
        result.update(official_runtime=dict(protocol=1, recovery=self._recovery.status(),
            firmware_confirmed=self._official_firmware, trial_time_limit_s=None,
            one_pulse_per_episode=False, following_error_start_gate_deg=None,
            head_fixed=False, takeover_seconds=0, stance_transition_seconds=0,
            additional_goal_slew=None, command_limits=[.4,.3,1.],
            head_range_projection=dict(enabled=True, bounds='saved calibration',
                last=copy.deepcopy(self._head_projection), leg_targets_changed=False),
            hardware_walking_verified=False),
            page_heartbeat_required=self._mode != 'off',
            hold_control_protocol=1,
            candidate_profile=dict(native_zero_only_adapter=False, nonzero_command_locked=False,
                physical_walking_verified=False, trial_seconds_remaining=None),
            forward_trial=dict(protocol=2, removed=True, allowed=False,
                blocked_reason='请使用持续前进/回零控制'),
            policy_goal_execution=dict(protocol='official-runtime-v2',
                firmware_capability_confirmed=self._official_firmware,
                walk_extra_slew_ticks_s=None, pose_extra_slew_ticks_s=None),
            active_model=self._select_policy_locked(self._mode))
        return result
