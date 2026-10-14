"""Attended slider control through the existing native HOLD/ARM/TARGET path.

No serial port, firmware or calibration writes. The parent owns all transport,
feedback, tilt, torque and STOP checks. All fifteen servos hold their targets;
one selected target may change at a time. This is not a walking mode.
"""
import copy
import math
import struct

PROTOCOL = 1
SPEED_DEG_S = 5.0
MIN_SPEED_DEG_S = 1.0
# Stay below the legacy TARGET firmware slew (600 ticks/s ~= 52.75 deg/s).
MAX_SPEED_DEG_S = 50.0
EXCURSION_DEG = 10.0
# Browser/Wi-Fi delivery can occasionally exceed 400 ms even while UART and
# IMU remain current. Bound this attended page lease at one second; the native
# feedback, bounded slew, saved range and tracking-error gates remain separate.
LEASE_S = 1.0
TRACKING_ERROR_DEG = 5.0


def validate_target(joint_id, angle_deg, sequence, motor_ids, speed_deg_s=None):
    if type(joint_id) is not int or joint_id not in motor_ids:
        raise ValueError('未知随动关节')
    if type(angle_deg) not in (int, float) or not math.isfinite(angle_deg) or abs(angle_deg) > 360:
        raise ValueError('随动角度必须是有限数字')
    if type(sequence) is not int or not 1 <= sequence <= 2**31-1:
        raise ValueError('随动指令序号无效')
    if speed_deg_s is not None and (type(speed_deg_s) not in (int, float)
            or not math.isfinite(speed_deg_s) or not MIN_SPEED_DEG_S <= speed_deg_s <= MAX_SPEED_DEG_S):
        raise ValueError('舵机转动速度必须在 1–50°/秒之间')


def controller_class(base, motion_error, motor_ids, all_mask, target_kind):
    class JointFollowController(base):
        def __init__(self, *args, **kwargs):
            self._joint_follow = None
            super().__init__(*args, **kwargs)

        def _following(self):
            return self._joint_follow is not None and self._joint_follow['active']

        def _follow_capture_locked(self):
            self._require_feedback_locked()
            raw = self._state['positions']
            angles = [(p-j['zero'])*360/4095*j['sign'] for p, j in zip(raw, self._config['joints'])]
            if super()._raw_targets_locked(angles, mode='hold') != list(raw):
                raise motion_error('当前位置换算无法精确回到编码器刻度')
            bounds = []
            for a, j in zip(angles, self._config['joints']):
                low, high = sorted(( (p-j['zero'])*360/4095*j['sign'] for p in (j['min'], j['max']) ))
                bounds.append([max(low, a-EXCURSION_DEG), min(high, a+EXCURSION_DEG)])
            return angles, bounds

        def _follow_head_locked(self, angles):
            self._hold_head_deg = {sid: angles[motor_ids.index(sid)] for sid in (30,31,32,33)}
            self._standing_reference_head = dict(self._hold_head_deg)
            self._standing_reference_calibration = copy.deepcopy(self._config)

        def start_joint_follow(self, *, expected_generation, support_confirmed=False, clearance_confirmed=False):
            with self._lock:
                if type(expected_generation) is not int or expected_generation != self._generation:
                    raise motion_error('状态已变化，请重新检查后启用随动')
                if support_confirmed is not True or clearance_confirmed is not True:
                    raise motion_error('请先确认机身已托稳且所有关节有活动间隙')
                self._require_off_locked()
                self._require_feedback_locked()
                if (self._following() or self._pending_stop or self._state['armed']
                        or self._state['torque_mask'] or self._state['cookie'] or self._state['last_error']
                        or not self._calibrated_locked() or self._state['online_mask'] != all_mask
                        or self._state['fault_mask']):
                    raise motion_error('随动需要完整校准、全部在线且已确认卸力')
                if getattr(self, '_serial_owner_enabled', False):
                    raise motion_error('当前异步串口版本尚未验证滑块随动')
                angles, bounds = self._follow_capture_locked()
                now = self._clock()
                self._joint_follow = dict(active=True, preparing=True, generation=self._generation,
                    baseline=angles, target=list(angles), desired=list(angles), bounds=bounds,
                    selected=None, sequence=0, speed_deg_s=SPEED_DEG_S, tick=now, calibration=copy.deepcopy(self._config), reason=None)
                self._follow_head_locked(angles)
                try:
                    # The outer lock excludes control ticks through ARM and its
                    # reply. The first target is recaptured after ARM, not HOME.
                    super().start_mode('hold', support_confirmed=True, expected_generation=expected_generation)
                    angles, bounds = self._follow_capture_locked()
                    self._follow_head_locked(angles)
                    self._joint_follow = dict(self._joint_follow, preparing=False, generation=self._generation,
                        baseline=angles, target=list(angles), desired=list(angles), bounds=bounds, tick=self._clock())
                    self._hold_initial = list(angles)
                    self._hold_begin = self._clock()-3.
                    self._heartbeat = self._clock()
                except BaseException as error:
                    self._stop_locked('随动启用失败：'+str(error))
                    try:
                        self._service_stop_locked()
                    except Exception as stop_error:
                        self._disconnect('随动停止确认失败：'+str(stop_error))
                    raise
            return self.status()

        def joint_target(self, *, expected_generation, joint_id, angle_deg, sequence, speed_deg_s=None):
            validate_target(joint_id, angle_deg, sequence, motor_ids, speed_deg_s)
            with self._lock:
                p = self._joint_follow
                if (not self._following() or p['preparing'] or type(expected_generation) is not int
                        or expected_generation != self._generation):
                    raise motion_error('随动会话已结束或已变化')
                self._motion_gate_locked('hold')
                if sequence <= p['sequence']:
                    raise motion_error('随动指令已过期，未执行')
                i = motor_ids.index(joint_id)
                if not p['bounds'][i][0] <= angle_deg <= p['bounds'][i][1]:
                    raise motion_error('目标超过已保存范围或本轮起点 ±10°')
                # Switching joint freezes the previous one at its last sent
                # target; no queued old trajectory continues in the background.
                desired = list(p['target'])
                desired[i] = float(angle_deg)
                super()._raw_targets_locked(desired, mode='hold')
                self._joint_follow = dict(p, desired=desired, selected=joint_id, sequence=sequence,
                    speed_deg_s=p['speed_deg_s'] if speed_deg_s is None else float(speed_deg_s))
            return dict(target_accepted=True, generation=self._generation, sequence=sequence)

        def _fixed_stance_locked(self):
            if self._following():
                return list(self._joint_follow['target'])
            return super()._fixed_stance_locked()

        def _hold_ready_locked(self):
            return False if self._following() else super()._hold_ready_locked()

        def start_mode(self, *args, **kwargs):
            with self._lock:
                if self._following():
                    raise motion_error('请先停止关节随动，再切换动作')
                return super().start_mode(*args, **kwargs)

        def _heartbeat_timeout_locked(self):
            return LEASE_S if self._following() else super()._heartbeat_timeout_locked()

        def _heartbeat_expired_locked(self):
            if self._following():
                return self._heartbeat is None or not 0 <= self._clock()-self._heartbeat < LEASE_S
            return super()._heartbeat_expired_locked()

        def heartbeat(self, **kwargs):
            with self._lock:
                if self._following() and self._heartbeat_expired_locked():
                    raise motion_error('随动心跳已过期，不能恢复旧会话')
                return super().heartbeat(**kwargs)

        def _motion_gate_locked(self, mode=None):
            if self._following():
                p = self._joint_follow
                if (mode or self._mode) != 'hold' or self._config != p['calibration']:
                    raise motion_error('随动模式或校准已变化')
                if not p['preparing']:
                    if p['generation'] != self._generation or self._heartbeat_expired_locked():
                        raise motion_error('随动页面心跳超时或会话已变化')
                    if self._filtered is None or max(abs(a-b) for a,b in zip(self._filtered.angles,p['target'])) >= TRACKING_ERROR_DEG:
                        raise motion_error('随动实测与目标偏差达到 5°，请托稳并检查关节')
            return super()._motion_gate_locked(mode)

        def _control_once(self):
            with self._lock:
                if self._following():
                    self._motion_gate_locked('hold')
                    p = self._joint_follow
                    now = self._clock()
                    # A delayed tick cannot catch up with a large target jump.
                    step = p['speed_deg_s'] * max(0., min(.04, now-p['tick']))
                    target = [a+max(-step,min(step,b-a)) for a,b in zip(p['target'],p['desired'])]
                    self._joint_follow = dict(p, target=target, tick=now)
                    self._follow_head_locked(target)
                    return super()._control_once()
            return super()._control_once()

        def _request(self, kind, payload=b''):
            if kind == target_kind and self._following():
                self._motion_gate_locked('hold')
                expected = super()._raw_targets_locked(self._joint_follow['target'], mode='hold')
                if list(struct.unpack('<I15H', payload)[1:]) != expected:
                    raise motion_error('拒绝不属于当前滑块随动的目标')
            return super()._request(kind, payload)

        def _stop_locked(self, reason=None):
            followed = self._following()
            try:
                return super()._stop_locked(reason)
            finally:
                if followed:
                    self._joint_follow = dict(self._joint_follow, active=False, reason=reason or '已停止')
                    self._hold_head_deg = None
                    self._standing_reference_head = None
                    self._standing_reference_calibration = None

        def status(self):
            result = super().status()
            with self._lock:
                p = self._joint_follow
                result['joint_follow'] = dict(protocol=PROTOCOL, active=self._following(),
                    speed_deg_s=p['speed_deg_s'] if p else SPEED_DEG_S,
                    min_speed_deg_s=MIN_SPEED_DEG_S, max_speed_deg_s=MAX_SPEED_DEG_S,
                    excursion_deg=EXCURSION_DEG, heartbeat_ms=LEASE_S*1000,
                    all_servos_hold=True, physical_following_verified=False,
                    selected=p['selected'] if p else None, sequence=p['sequence'] if p else 0,
                    reason=p['reason'] if p else None,
                    joints=[dict(id=sid, target_deg=p['target'][i], requested_deg=p['desired'][i],
                        min_deg=p['bounds'][i][0], max_deg=p['bounds'][i][1])
                        for i,sid in enumerate(motor_ids)] if p else [])
                if self._following():
                    result.update(page_heartbeat_required=True, hold_control_protocol=1)
            return result

    return JointFollowController
