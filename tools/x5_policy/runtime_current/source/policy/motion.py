"""Native transport adapter; official ONNX weights and action decoding unchanged.

本机差异 HEAD-PROFILE-20261012（官方基准 8cdbbd84710d856581982c9eaf0d5e2970666232）：
官方 python/controller.py 的 head 默认是四个零，可由 command(head=...) 设置；
python/rl_core.py:build_obs 将其放进 observation[51:55]。本机从安装配置读取
一个固定的 head 指令，还让显式 HOLD 使用对应头颈角度。不是重训或修改模型 HOME。
这组角度来自 2026-10-08 用户确认独立承重的 30 秒记录：
output/x5_gait_trial/20261008/leg-kp8-candidate/attended-trial-01/hold-check.json
ID30..33 = [-7.56043956, -3.42857143, -5.80219780, -2.98901099] 度。
该历史试验还包含 KP8 和固定头部输出等其他改动；本版本不恢复那些改动。
因此这只是有历史依据的单因素候选，尚未证明现在的实机能够站稳或行走。
未设置 profile 时保持原零指令。构造不连接硬件，不自动 ARM 或发送目标。
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
SOURCE_RUNTIME = 30
SOURCE_TARGET = 31
SOURCE_STATE = 32


def _no_io_factory(port):
    raise MotionError('Validation-only controller cannot open hardware')


class MotionController(TransportController):
    def __init__(self, sessions, config_path=None, port='validation-only', *,
                 transport_factory=None, clock=time.monotonic,
                 require_head_clearance=False, allow_fixed_head_walk=False,
                 hold_hip_pitch_bias_deg=0, walk_head_command_rad=None):
        if hold_hip_pitch_bias_deg != 0:
            raise MotionError('Official runtime does not accept experimental stance offsets')
        # 与官方差异：将默认零 head 改为安装时明确选定的四维指令。
        # 顺序为 neck_pitch/head_pitch/head_yaw/head_roll，单位弧度、相对
        # ONNX 元数据 HOME；这里不修改 HOME 本身或腿部观察/输出。
        # +/-1 rad 沿用官方 controller.py:command 的边界，不新增经验阈值。
        # 连接内保持不变；数值来源见模块注释及 deployment 的 provenance。
        head = [0., 0., 0., 0.] if walk_head_command_rad is None else walk_head_command_rad
        if (not isinstance(head, (list, tuple)) or len(head) != 4 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) or abs(v) > 1. for v in head)):
            raise MotionError('Head command requires four finite values within the official +/-1rad contract')
        self._walk_head_profile = tuple(float(v) for v in head)
        self._validation_only = transport_factory is None
        self._official_firmware = False
        self._recovery = Recovery()
        self._trace = collections.deque(maxlen=3000)
        self._trace_total = 0
        self._completed = collections.deque(maxlen=4)
        self._trace_sequence = 0
        self._policy_trace = collections.deque(maxlen=3000)
        self._policy_trace_total = 0
        self._pending_policy_evidence = None
        self._last_trial_stop = None
        self._head_projection = []
        self._target_projection = []
        super().__init__(sessions, config_path, port,
            transport_factory=transport_factory or _no_io_factory, clock=clock,
            require_head_clearance=False, allow_fixed_head_walk=False,
            hold_hip_pitch_bias_deg=0)
        from model_reference import WALK_HOME_RAD
        if not np.array_equal(self._policy_profiles['walk']['home_rad'], WALK_HOME_RAD):
            raise MotionError('行走模型 HOME 与校准参照不一致；先重新生成配套参照，不能静默使用旧偏置')

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
                if self._request(SOURCE_RUNTIME) != bytes([SOURCE_RUNTIME, 0]):
                    raise MotionError('需要配套同步读取固件；不自动降级到旧通信流程')
                self._official_firmware = True
                self._state_request = SOURCE_STATE
                self._state_request_payload = b''
                self._motion_evidence_supported = False
                self._accept_state_locked(self._read_state())
        except BaseException as error:
            self._disconnect(error)
            raise

    def _disconnect(self, error):
        self._official_firmware = False
        return super()._disconnect(error)

    def _read_state(self):
        if self._state_request != SOURCE_STATE:
            return super()._read_state()
        payload = self._request(SOURCE_STATE)
        voltage = len(payload) in (205,249) and payload[108:110] in (b'\x01\x03',b'\x01\x04')
        if voltage:
            base_size = 214 if payload[109] == 4 else 170
            if len(payload) != base_size+35: raise MotionError('同步电压状态长度不匹配')
            extension, payload = payload[base_size:], payload[:base_size]
            if extension[0] != 1: raise MotionError('同步电压版本不匹配')
            payload = bytearray(payload); payload[109] -= 2; payload = bytes(payload)
        legacy = len(payload) == 170 and payload[108:110] == b'\x01\x01'
        stopped = len(payload) == 214 and payload[108:110] == b'\x01\x02'
        if not (legacy or stopped):
            raise MotionError('同步读取状态版本或长度不匹配')
        state = decode_state(payload[:106])
        if voltage:
            seen, valid = struct.unpack_from('<HH',extension,1)
            if seen & ~ALL_MASK or valid & ~seen: raise MotionError('同步电压有效位无效')
            raw = list(extension[5:20])
            if any(valid & (1<<i) and raw[i] == 0 for i in range(15)): raise MotionError('同步电压有效值为零')
            state['voltage'] = dict(seen_mask=seen,valid_mask=valid,raw=raw,
                ages_us=[v*100000 for v in extension[20:35]])
        known = struct.unpack_from('<H',payload,106)[0]
        ages = list(struct.unpack_from('<15I',payload,110))
        if known & ~ALL_MASK or state['torque_mask'] & ~known:
            raise MotionError('扭矩观测有效位无效')
        state.update(source_runtime=True,torque_known_mask=known,position_ages_us=ages)
        if stopped:
            if state['armed']:
                raise MotionError('运行中的状态不能携带已停止会话记录')
            detail, text = target_stop_detail(payload[170:210])
            detail['arm_cookie'] = struct.unpack_from('<I', payload, 210)[0]
            if not detail['arm_cookie']:
                raise MotionError('首次停机会话标识无效')
            state.update(firmware_first_stop=detail, firmware_first_stop_text=text)
            self._transport.last_firmware_stop = copy.deepcopy(detail)
        return state

    def _session_failure_locked(self, state):
        detail = state.get('firmware_first_stop')
        if detail and self._cookie and detail['arm_cookie'] == self._cookie:
            return '下位机已停止；' + state['firmware_first_stop_text']
        reasons = []
        if not state['armed']:
            error = state.get('last_error')
            reasons.append('下位机退出使能' + (f'（固件错误码 {error}：{ERRORS[error]}）' if error in ERRORS else ''))
        if not state['configured']:
            reasons.append('下位机校准配置失效')
        if state['armed'] and state['cookie'] != self._cookie:
            reasons.append('下位机会话标识不匹配')
        if state['armed'] and not self._enabled_feedback(state):
            reasons.append('下位机扭矩状态不符合当前控制模式')
        if detail:
            reasons.append('首次停机记录属于其他使能会话，不能作为本轮原因')
        return '；'.join(reasons) or '下位机已停止或会话失效'

    def _request_transport(self, transport, kind, payload=b''):
        follow = getattr(self, '_joint_follow', None)
        if kind == TARGET and self._official_firmware and not (follow and follow['active']):
            began, failed = self._clock(), True
            try:
                response=transport.request(SOURCE_TARGET,payload)
                if response != b'': raise MotionError('异步目标接口不能伪造应答')
                failed=False
                return response
            finally: self._record_timing('target_request',began,failed)
        if kind == SOURCE_STATE:
            began, failed = self._clock(), True
            try:
                response=transport.request(kind,payload)
                self._state_request_sequence=getattr(transport,'seq',None)
                failed=False
                return response
            finally: self._record_timing('state_request',began,failed)
        return super()._request_transport(transport,kind,payload)

    def _target_payload_locked(self, raw):
        if not self._official_firmware:
            raise MotionError('Official execution firmware is not verified')
        # Preserve the separately installed slider-maintenance wire contract.
        follow = getattr(self, '_joint_follow', None)
        if follow and follow['active']:
            return struct.pack('<I15H', self._cookie, *raw)
        return struct.pack('<I15H', self._cookie, *raw) + b'\x01'

    def _fixed_stance_locked(self):
        # 本机 HOLD 扩展：官方 controller.py 固定阶段直接使用 DEFAULT_DEG。
        # 此处仅给四个头颈目标加 head 指令对应的角度，便于 HOLD 与后续
        # WALK 的指令一致；不加腿部偏置，不改变校准/范围。恢复起身路径
        # 仍使用原模型默认姿态，不在此处施加这组头颈姿态。
        target = None if self._stand_deg is None else list(self._stand_deg)
        if target is not None:
            for sid, delta in zip((30, 31, 32, 33), self._walk_head_profile):
                target[MOTOR_IDS.index(sid)] += math.degrees(delta)
        return target

    def _walk_head_command_locked(self):
        # 仅作为官方 build_obs 的输入；WALK 不强制覆盖模型的头部输出。
        # 与历史“固定头部输出”的实现不同，不能把历史成功当作本版验收。
        return np.asarray(self._walk_head_profile, dtype=np.float32).copy()

    def _execution_target_locked(self, mode, policy_target):
        # TARGET-CLAMP-20261013: user explicitly requested saved-range target
        # saturation instead of stopping on an out-of-range policy request.
        # Upstream 8cdbbd scs_bus.h:satPos only bounds encoder values 0..4095;
        # this device-specific adapter retains the user's saved bounds. A goal
        # below min becomes min+1 tick; above max becomes max-1 tick. Goals at
        # or within either endpoint are unchanged. This is NOT a global margin
        # or calibration rewrite, and does not modify actor/EMA/raw history.
        # Convert only changed targets back to degrees for accurate execution
        # logs. _raw_targets_locked and the MCU still validate every sent goal.
        if len(policy_target) != 15 or not all(math.isfinite(v) for v in policy_target):
            raise MotionError('目标角度无效')
        result = list(policy_target)
        projections = []
        for index, joint in enumerate(self._config['joints']):
            low, high = joint['min'], joint['max']
            if not (type(low) is int and type(high) is int and 0 <= low < high <= 4095):
                raise MotionError(f"ID {joint['id']} 已保存范围无效")
            value = int(round(joint['zero'] + result[index] / 360 * 4095 * joint['sign']))
            bounded = low + 1 if value < low else high - 1 if value > high else value
            if bounded == value:
                continue
            requested = float(result[index])
            result[index] = (bounded - joint['zero']) * 360 / 4095 * joint['sign']
            projections.append(dict(id=joint['id'], requested_ticks=value,
                executed_ticks=bounded, requested_deg=requested,
                executed_deg=float(result[index]), min=low, max=high))
        self._target_projection = projections
        # Compatibility for existing head diagnostics; the scope now includes
        # every joint and every normal motion mode, including recovery goals.
        self._head_projection = [dict(row) for row in projections if row['id'] in (30,31,32,33)]
        return result

    def capture_head_clearance(self, **kwargs):
        raise MotionError('官方运行模式不固定头颈；不需要手动记录头颈位置')

    def _hold_ready_locked(self):
        # Readiness is connection/enablement, not an arbitrary 5-degree pose gate.
        try:
            self._require_feedback_locked()
            return bool(self._mode == 'hold' and self._state['armed'] and
                        self._enabled_feedback(self._state) and not self._pending_stop)
        except MotionError:
            return False

    def _motion_gate_locked(self, mode=None):
        selected = self._mode if mode is None else mode
        self._require_feedback_locked()
        if not self._calibrated_locked():
            raise MotionError('需要完整的关节零点、方向、保存范围和 IMU 校准')
        state = self._state
        # STOP-POLICY-20261013, upstream controller.step/scs_bus.h:
        # retain bad/missing reply diagnostics, not a new all-joint OFF rule.
        # New ARM and attended joint-follow still require a complete scan.
        strict_entry = self._mode == 'off' or bool(
            getattr(self, '_joint_follow', None) and self._joint_follow['active'])
        if strict_entry and (state['fault_mask'] or state['online_mask'] != ALL_MASK):
            raise MotionError('舵机反馈缺失或报告故障')
        if self._mode == 'off' and state.get('torque_known_mask', ALL_MASK) != ALL_MASK:
            raise MotionError('停止后的扭矩关闭回读尚未完整确认')
        if not state['imu_ok'] or self._imu_age_locked() >= FRESH_SECONDS or self._filtered is None:
            raise MotionError('需要新鲜有效的 IMU 与关节反馈')
        if not all(math.isfinite(v) for v in self._filtered.grav) or sum(v*v for v in self._filtered.grav) < 1e-12:
            raise MotionError('姿态反馈无效')
        if selected != 'shadow' and not self._official_firmware:
            raise MotionError('配套官方增益固件尚未确认')
        if self._stand_deg is None:
            raise MotionError('行走模型元数据未加载')
        # Upstream has no saved-range actual-position stop. At
        # normal model entry and while running, record excursions and constrain
        # every outgoing goal using the user's min+1/max-1 adapter. Do not
        # interpret the hand-recorded interval as an all-joint OFF trigger.
        # Only the independent attended joint-follow workflow retains its
        # measured-pose gate; ordinary HOLD/WALK/model start does not.
        if getattr(self, '_joint_follow', None) and self._joint_follow['active']:
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

    def _policy_start_smooth_locked(self, mode):
        # HEAD-HANDOFF-20261013：修复本机新增的非零头颈 HOLD 与原版零滤波
        # 初值不一致。官方 policy.py:reset_action_filter() 清零适用于 HOME；
        # 这里仅在已成功发送 HOLD 目标后进入 WALK 时，保留四个头关节的
        # 动作滤波初值：(上一条成功发送的HOLD目标 - 模型HOME) / 原动作尺度。
        # 当前 head profile 下直接清零会使 ID30/31 第一帧回跳约13/15度。
        # 依据为 zero-after-head 的实测输入重放，不新增插值时长/限速/KP。
        # 腿部滤波、模型的 previous_raw_action、EMA系数、恢复流程保持原版。
        # 这是目标连续性修复，并不证明承重平衡或实机行走已解决。
        seed = super()._policy_start_smooth_locked(mode)
        if (mode != 'walk' or self._mode != 'hold'
                or not any(self._walk_head_profile)
                or self._last_executed_target is None):
            return seed
        profile = self._policy_profiles['walk']
        target = np.asarray(self._last_executed_target, dtype=np.float32)
        if target.shape != (15,) or not np.isfinite(target).all():
            raise MotionError('Invalid committed HOLD target for head handoff')
        seed[5:9] = ((np.deg2rad(target[10:14]) - profile['home_rad'][5:9])
                     / profile['scale'])
        return seed

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
            attempted_target_ticks=target, error=error, recovery=self._recovery.status(),
            target_range_projection=copy.deepcopy(self._target_projection),
            feedback_provenance=self._snapshot_policy_feedback_locked())

    def _snapshot_policy_feedback_locked(self):
        # SOURCE_STATE can keep one motor's older reading while advancing the
        # whole-packet timestamp. Preserve that distinction without extra I/O
        # or a new freshness threshold. Missing ages stay unknown, never zero.
        state = self._state or {}
        ages = state.get('position_ages_us')
        return dict(scope='received_state_fields_not_motor_execution_proof',
            state_sample_us=state.get('sample_us'), filtered_sample_us=self._last_sample,
            host_sample_received_monotonic=self._sample_received,
            host_recorded_monotonic=self._clock(),
            position_ages_us=None if ages is None else list(ages),
            online_mask=state.get('online_mask'), fault_mask=state.get('fault_mask'),
            raw_positions=list(state.get('positions', ())),
            raw_velocities=list(state.get('velocities', ())),
            torque_known_mask=state.get('torque_known_mask'),
            armed=state.get('armed'), cookie=state.get('cookie'),
            configured=state.get('configured'), last_error=state.get('last_error'),
            torque_mask=state.get('torque_mask'),
            firmware_first_stop=copy.deepcopy(state.get('firmware_first_stop')))

    def _append_trace(self, row):
        self._trace.append(row); self._trace_total += 1

    def _record_policy_frame_locked(self, frame, target, action, smooth, inference_ms):
        row = dict(generation=frame['generation'], mode=frame['mode'],
            selected_model=frame['selected'], observation_sample_us=frame['observation_sample'],
            commit_sample_us=self._last_sample,
            observation_received_monotonic=frame['observation_received'],
            submission_monotonic=frame['submission_monotonic'],
            validation_monotonic=self._clock(),
            submitted_observation=frame['submitted_observation'].tolist(),
            previous_raw_action=frame['previous'].tolist(),
            previous_EMA=frame['smooth'].tolist(), raw_action=action.tolist(),
            EMA=smooth.tolist(), requested_target_deg=[float(v) for v in target],
            input_feedback_provenance=copy.deepcopy(frame.get('feedback_provenance')),
            commit_feedback_provenance=self._snapshot_policy_feedback_locked(),
            inference_ms=inference_ms, target_write_attempted=False,
            target_write_completed=False, motor_execution_confirmed=False)
        self._policy_trace.append(row); self._policy_trace_total += 1
        self._pending_policy_evidence = row

    def _request(self, kind, payload=b''):
        if kind != TARGET:
            return super()._request(kind, payload)
        row = self._trace_sample('target', list(struct.unpack_from('<I15H', payload)[1:]))
        evidence = self._pending_policy_evidence
        if evidence is not None:
            evidence.update(target_write_attempted=True,
                target_write_started_monotonic=self._clock(), transmitted_ticks=row['attempted_target_ticks'],
                target_range_projection=copy.deepcopy(self._target_projection))
        try:
            response = super()._request(kind, payload)
            if evidence is not None:
                evidence['target_write_completed'] = True
            return response
        except BaseException as error:
            row['error'] = str(error)
            if evidence is not None:
                evidence['target_write_error'] = str(error)
            raise
        finally:
            row['request_ended_monotonic'] = self._clock()
            self._append_trace(row)
            self._pending_policy_evidence = None

    def _retain_trial_trace_locked(self, event, reason):
        if self._trace or self._policy_trace:
            self._trace_sequence += 1
            self._completed.append(dict(sequence=self._trace_sequence, event=event,
                reason=reason, terminal_feedback=self._snapshot_policy_feedback_locked(),
                frames=list(self._trace), total_frames=self._trace_total,
                policy_frames=list(self._policy_trace), total_policy_frames=self._policy_trace_total,
                policy_frames_truncated=self._policy_trace_total > len(self._policy_trace),
                truncated=self._trace_total > len(self._trace), hardware_walking_verified=False))
            self._trace.clear(); self._trace_total = 0
            self._policy_trace.clear(); self._policy_trace_total = 0
            self._pending_policy_evidence = None

    def completed_trial_traces(self, after_sequence=0):
        with self._lock:
            return copy.deepcopy([x for x in self._completed if x['sequence'] > after_sequence])

    def _stop_locked(self, reason=None):
        self._retain_trial_trace_locked('stop', reason)
        self._recovery.reset()
        return super()._stop_locked(reason)

    def status(self):
        result = super().status()
        state=self._state or {}
        known=state.get('torque_known_mask', ALL_MASK)
        result['torque_observation']=dict(
            known_ids=[sid for i,sid in enumerate(MOTOR_IDS) if known & (1<<i)],
            commanded_enabled=bool(state.get('armed')),
            on_broadcast_is_not_readback=True)
        result['position_feedback_ages_us']=state.get('position_ages_us')
        result['non_stopping_feedback_warnings'] = dict(
            fault_ids=list(result.get('fault_ids', [])),
            outside_saved_range=[dict(id=j['id'], position=p, min=j['min'], max=j['max'])
                for p, j in zip(state.get('positions', []), self._config['joints'])
                if j['min'] is not None and j['max'] is not None
                and not j['min'] <= p <= j['max']],
            scope='source-entry-and-running; fault/online startup and maintenance checks retained')
        result['firmware_first_stop']=copy.deepcopy(state.get('firmware_first_stop'))
        result['servo_voltage_polling']=dict(enabled=False,
            reason='运控恢复原始寄存器56..61同步读取；电压仅在独立维护时采集')
        for i,row in enumerate(result.get('rawdata', [])):
            row['torque']=int(bool(state.get('torque_mask',0)&(1<<i))) if known&(1<<i) else None
        result['walk_head_command_rad'] = list(self._walk_head_profile)
        result.update(official_runtime=dict(protocol=1, recovery=self._recovery.status(),
            firmware_confirmed=self._official_firmware, trial_time_limit_s=None,
            one_pulse_per_episode=False, following_error_start_gate_deg=None,
            head_fixed=False, takeover_seconds=0, stance_transition_seconds=0,
            additional_goal_slew=None, command_limits=[.4,.3,1.],
            head_range_projection=dict(enabled=True, bounds='saved calibration',
                last=copy.deepcopy(self._head_projection), leg_targets_changed=any(
                    row['id'] < 30 for row in self._target_projection)),
            target_range_projection=dict(enabled=True, units='encoder ticks',
                below_min='min + 1', above_max='max - 1', in_range='unchanged',
                scope='planned target only; actual position checks unchanged',
                last=copy.deepcopy(self._target_projection)),
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
