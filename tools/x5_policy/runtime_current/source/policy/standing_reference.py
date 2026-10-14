"""Connection-scoped stance preparation, bound to explicit head capture.

Calibration edits must invalidate the preparation. A new, confirmed capture
can prepare the corrected calibration without copying an old pose or enabling
torque. Leg targets continue to come from the controller's existing profile.
"""
import copy
import hashlib
import json


class StandingReferenceMixin:
    def __init__(self, *args, **kwargs):
        self._standing_reference_calibration = None
        self._standing_reference_head = None
        super().__init__(*args, **kwargs)

    def _connect_once(self):
        with self._lock:
            self._standing_reference_calibration = None
            self._standing_reference_head = None
        return super()._connect_once()

    def _standing_reference_valid_locked(self):
        return (self._connected and self._standing_reference_calibration is not None
                and self._config == self._standing_reference_calibration
                and self._hold_head_deg is not None
                and self._hold_head_deg == self._standing_reference_head)

    def capture_head_clearance(self, *, clearance_confirmed=False, expected_generation=None):
        # Keep validation, head capture and binding atomic against STOP/reload.
        # The parent enforces explicit confirmation, fresh feedback, all torque
        # OFF, all servos online, no faults, generation and target ranges.
        with self._lock:
            super().capture_head_clearance(clearance_confirmed=clearance_confirmed,
                                           expected_generation=expected_generation)
            self._standing_reference_calibration = copy.deepcopy(self._config)
            self._standing_reference_head = dict(self._hold_head_deg)
            return self.status()

    def _standing_reference_status_locked(self):
        valid = self._standing_reference_valid_locked()
        fingerprint = hashlib.sha256(json.dumps(self._config, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        return dict(standing_reference_protocol=2,
            standing_reference_restore=False, standing_reference_valid=valid,
            standing_reference_head_deg=copy.deepcopy(self._standing_reference_head) if valid else None,
            standing_reference_source='current_connection_head_capture',
            standing_reference_calibration_fingerprint=fingerprint,
            standing_reference_qualification='Current calibration and explicitly captured head; physical standing remains unverified.',
            standing_HOME_matches_physical_baseline=False,
            head_moves_during_HOLD_transition=False)
