"""Read-only actuator following checks and page lease diagnostics.

The forward-entry tolerance reuses the existing fixed-stance arrival test:
strictly less than 5 degrees. It is not a walking-stability qualification.
No control gain, command amplitude, torque policy or watchdog is changed.
"""
from causal_feedback import (LEG_IDS, FOLLOWING_TOLERANCE_DEG, following_status)


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
        # FRESH_SECONDS is the existing controller bound. Use raw positions
        # atomically delivered with the target, not a later filtered snapshot.
        from production_motion import FRESH_SECONDS
        return following_status(self._state, self._config['joints'],
            mode=self._mode, age_s=self._age_locked(), fresh_seconds=FRESH_SECONDS,
            expected_cookie=self._cookie)
