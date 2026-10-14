"""Read-only actuator following checks and page lease diagnostics.

The forward-entry tolerance reuses the existing fixed-stance arrival test:
strictly less than 5 degrees. It is not a walking-stability qualification.
No control gain, command amplitude, torque policy or watchdog is changed.
"""
import math

LEG_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24)
FOLLOWING_TOLERANCE_DEG = 5.


class ControlReadinessMixin:
    def __init__(self, *args, **kwargs):
        self._page_heartbeat_count = 0
        self._page_heartbeat_max_gap = 0.
        super().__init__(*args, **kwargs)

    def _mode_started_locked(self, mode):
        self._page_heartbeat_count = 0
        self._page_heartbeat_max_gap = 0.
        return super()._mode_started_locked(mode)

    def heartbeat(self, **kwargs):
        with self._lock:
            before = self._heartbeat
            result = super().heartbeat(**kwargs)
            if self._mode != 'off':
                self._page_heartbeat_count += 1
                if before is not None:
                    self._page_heartbeat_max_gap = max(self._page_heartbeat_max_gap,
                                                       self._heartbeat - before)
            return result

    def _page_heartbeat_status_locked(self):
        return dict(accepted_count=self._page_heartbeat_count,
            age_ms=None if self._heartbeat is None else (self._clock()-self._heartbeat)*1000,
            maximum_accepted_gap_ms=self._page_heartbeat_max_gap*1000,
            timeout_ms=self._heartbeat_timeout_locked()*1000)

    def _leg_following_status_locked(self):
        state = self._state or {}
        evidence = state.get('motion_evidence') or {}
        raw = evidence.get('written_goals_raw')
        basis = 'MCU_last_transmitted_goal'
        active = self._mode in ('hold', 'walk') and state.get('armed') and state.get('torque_mask') == 0x7fff
        if not active:
            target, basis = None, 'inactive_no_execution_target'
        elif (isinstance(raw, (list, tuple)) and len(raw) == 15
                and evidence.get('goal_write_us', 0) != 0
                and all(type(tick) is int and 0 <= tick <= 4095 for tick in raw)):
            target = [(p-j['zero'])*360/4095*j['sign'] for p,j in zip(raw,self._config['joints'])]
        else:
            target = self._last_executed_target
            basis = 'host_last_executed_target'
        rows = []
        if self._filtered is not None and target is not None:
            for index, sid in enumerate(LEG_IDS):
                actual, desired = self._filtered.angles[index], target[index]
                if not math.isfinite(actual) or not math.isfinite(desired):
                    rows = []
                    break
                rows.append(dict(id=sid, actual_deg=float(actual), target_deg=float(desired),
                                 error_deg=float(desired-actual)))
        worst = max(rows, key=lambda row: abs(row['error_deg'])) if rows else None
        ready = bool(worst and abs(worst['error_deg']) < FOLLOWING_TOLERANCE_DEG)
        return dict(protocol=1, ready=ready, tolerance_deg=FOLLOWING_TOLERANCE_DEG,
            tolerance_basis='existing_fixed_stance_arrival', target_source=basis,
            servo_goal_register_readback=False, physical_stability_verified=False,
            joints=rows, worst=worst)
