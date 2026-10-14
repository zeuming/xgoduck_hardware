"""Explicit, capability-checked policy position goals; no device entrypoint.

Mixed into the attended native adapter, after its original target checks.
The base controller calls _target_payload_locked for BOTH synchronous and
single-UART-owner dispatch. Revalidation reuses the exact untransmitted bytes.
"""
from production_motion import MotionError

POLICY_POSITION_CAPS = 24
PROFILE_BYTE = b'\x01'


class OfficialGoalExecutionMixin:
    def __init__(self, *args, **kwargs):
        self._policy_position_negotiated = False
        super().__init__(*args, **kwargs)

    def _connect_once(self):
        self._policy_position_negotiated = False
        super()._connect_once()  # Existing STOP, identity and calibration checks.
        try:
            with self._lock:
                if self._mode != 'off' or self._state['armed'] or self._state['torque_mask']:
                    raise MotionError('执行协议核对需要扭矩关闭')
                if self._request(POLICY_POSITION_CAPS) != bytes([POLICY_POSITION_CAPS, 0]):
                    raise MotionError('固件尚不支持模型位置直通，不能沿用旧限速启动')
                self._policy_position_negotiated = True
        except BaseException as error:
            self._disconnect(error)
            raise

    def _disconnect(self, error):
        self._policy_position_negotiated = False
        return super()._disconnect(error)

    def _motion_gate_locked(self, mode=None):
        selected = self._mode if mode is None else mode
        if selected in ('hold', 'walk') and not self._policy_position_negotiated:
            raise MotionError('尚未核对模型位置执行协议；不能启动动作')
        return super()._motion_gate_locked(mode)

    def _target_payload_locked(self, raw):
        payload = super()._target_payload_locked(raw)
        if self._mode != 'walk':
            return payload  # HOLD/probes/return retain their reviewed transition.
        if not self._policy_position_negotiated:
            raise MotionError('模型位置执行协议已失效')
        if self._trial_generation != self._generation or self._trial_began is None:
            raise MotionError('零速接管会话已失效')
        # The native adapter has already blended takeover for two seconds and
        # validated the entire target. Do not reset EMA or raw previous action.
        return payload + PROFILE_BYTE

    def status(self):
        result = super().status()
        result['policy_goal_execution'] = dict(
            protocol='explicit-policy-position-v1',
            firmware_capability_confirmed=self._policy_position_negotiated,
            walk_extra_slew_ticks_s=None, pose_extra_slew_ticks_s=600,
            stance_transition_seconds=3, zero_takeover_seconds=2,
            range_checks_preserved=True, hardware_walking_verified=False)
        return result
