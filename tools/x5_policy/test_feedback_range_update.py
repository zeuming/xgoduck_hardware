"""Boundary saves use synthetic feedback only; no UART or deployment."""
import copy
import threading
import types
import unittest
from unittest.mock import Mock
import native_workbench as web


class RangeUpdateTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(joints=[dict(id=12, min=823, max=3975, zero=2209, sign=-1),
                                  dict(id=33, min=756, max=950, zero=947, sign=1)], imu={'saved': True})
        self.hardware = dict(generation=4, mode='off', armed=False, release_confirmed=True,
                             torque_ids=[], age_ms=10, online_ids=[12, 33],
                             rawdata=[dict(id=12, position=678), dict(id=33, position=951)])
        self.updates = [dict(id=12, position=678, min=823, max=3975),
                        dict(id=33, position=951, min=756, max=950)]

    def test_both_edges_preserve_original_and_other_fields(self):
        original = copy.deepcopy(self.config)
        updated = web.expanded_feedback_ranges(self.config, self.hardware, self.updates)
        self.assertEqual(updated['joints'][0]['min'], 678)
        self.assertEqual(updated['joints'][1]['max'], 951)
        updated['joints'][0]['min'] = 823
        updated['joints'][1]['max'] = 950
        self.assertEqual(updated, original)
        self.assertEqual(self.config, original)

    def test_stale_or_invalid_selection_rejected(self):
        for key, value in [('position', 677), ('min', 822), ('id', True), ('position', 4096)]:
            updates = copy.deepcopy(self.updates)
            updates[0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                web.expanded_feedback_ranges(self.config, self.hardware, updates)
        with self.assertRaises(ValueError):
            web.expanded_feedback_ranges(self.config, self.hardware, [self.updates[0]] * 2)

    def fake_workbench(self):
        session = types.SimpleNamespace(config=self.config, lock=threading.RLock(),
                                        _write_calibration=Mock(return_value=None))
        return types.SimpleNamespace(calibration_service=types.SimpleNamespace(session=session),
            runtime=types.SimpleNamespace(disconnect=Mock()), snapshot=Mock(return_value={}))

    def test_save_disconnects_and_never_calls_motor_action(self):
        wb = self.fake_workbench()
        before = dict(hardware=self.hardware, phase='ready', owns_connection=True, feedback_age_ms=0)
        web.NativeWorkbench._expand_feedback_ranges(wb, dict(generation=4, updates=self.updates), before)
        wb.runtime.disconnect.assert_called_once()
        wb.calibration_service.session._write_calibration.assert_called_once()
        self.assertEqual(wb.calibration_service.session.config['joints'][0]['min'], 678)

    def test_running_stale_or_unreleased_rejected_before_disconnect(self):
        for changes in [dict(mode='hold'), dict(armed=True), dict(release_confirmed=False), dict(age_ms=100), dict(torque_ids=[12])]:
            wb = self.fake_workbench()
            before = dict(hardware=dict(self.hardware, **changes), phase='ready', owns_connection=True, feedback_age_ms=0)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                web.NativeWorkbench._expand_feedback_ranges(wb, dict(generation=4, updates=self.updates), before)
            wb.runtime.disconnect.assert_not_called()
            wb.calibration_service.session._write_calibration.assert_not_called()

    def test_endpoint_uses_actual_feedback_instead_of_display_publication(self):
        wb = self.fake_workbench()
        wb.generation, wb.closed, wb.control_owner = 90, False, None
        wb.io_lock, wb.lock = threading.RLock(), threading.RLock()
        fresh = dict(hardware=self.hardware, phase='ready', owns_connection=True, feedback_age_ms=15)
        wb.runtime.range_feedback_snapshot = Mock(return_value=fresh)
        wb.runtime.snapshot = Mock(side_effect=AssertionError('Display publication is not a save proof'))
        wb._expand_feedback_ranges = lambda body, before: web.NativeWorkbench._expand_feedback_ranges(wb, body, before)
        web.NativeWorkbench.motion_action(wb, dict(action='expand-feedback-ranges', page_generation=90,
            generation=4, control_id='a'*32, updates=self.updates))
        wb.runtime.range_feedback_snapshot.assert_called_once_with()
        wb.runtime.snapshot.assert_not_called()
        wb.calibration_service.session._write_calibration.assert_called_once()


if __name__ == '__main__':
    unittest.main()
