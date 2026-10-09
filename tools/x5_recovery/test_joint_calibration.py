"""Pure encoder fixtures only; no controller, UART, SSH, UI or HTTP service."""
import ast
import copy
from pathlib import Path
import unittest

import joint_calibration as joint


def config():
    return dict(joints=[dict(id=sid, zero=2000, sign=1, min=None, max=None,
                             direction_confirmed=False) for sid in joint.IDS],
                imu=dict(axes=['-z', '-x', 'y'], calibrated=True, axes_confirmed=True,
                         bias_dps=[.1, -.2, .3]))


def state(raw=2000, sample=1000000, sid=10):
    positions = [2000] * 15
    positions[joint.IDS.index(sid)] = raw
    return dict(positions=positions, sample_us=sample & 0xffffffff)


class MathTests(unittest.TestCase):
    def test_direction_requires_real_delta_and_maps_both_signs(self):
        self.assertEqual(joint.infer_sign(2000, 2012), 1)
        self.assertEqual(joint.infer_sign(2000, 1988), -1)
        for end in (2000, 2011, 3024, 4096, True):
            with self.subTest(end=end), self.assertRaises(ValueError):
                joint.infer_sign(2000, end)

    def test_linear_limits_margin_zero_and_actual_default_stand_gate(self):
        self.assertEqual(joint.safe_limits(1700, 2300, 2000, 1, 23), (1712, 2288))
        for args in ((0, 4095, 2000, 1, 0), (2000, 2300, 2000, 1, 0),
                     (1800, 2200, 2000, 1, 23), (1980, 2005, 2000, 1, 0)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                joint.safe_limits(*args)
        # This boundary distinguishes the actual 4095 controller scale.
        stand = round(2000 + 23 * 4095 / 360)
        self.assertEqual(joint.safe_limits(1700, stand + 12, 2000, 1, 23)[1], stand)

    def test_mouth_closed_endpoint_preserves_original_zero_with_both_signs(self):
        self.assertEqual(joint.safe_limits(2006, 2400, 2000, 1, 0, 34), (2000, 2388))
        self.assertEqual(joint.safe_limits(1600, 1994, 2000, -1, 0, 34), (1612, 2000))
        self.assertEqual(joint.safe_limits(2000, 2036, 2000, 1, 0, 34), (2000, 2024))
        self.assertEqual(joint.safe_limits(1964, 2000, 2000, -1, 0, 34), (1976, 2000))
        for args in ((2007, 2400, 2000, 1, 0, 34),
                     (1993, 1600, 2000, -1, 0, 34),
                     (2000, 1600, 2000, 1, 0, 34),
                     (2000, 2400, 2000, -1, 0, 34),
                     (2000, 2035, 2000, 1, 0, 34)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                joint.safe_limits(*args)

    def test_only_normal_range_geometry_is_retryable(self):
        cases = (((1988, 2012, 2000, 1, 0, 10), 'empty_range'),
                 ((1800, 2005, 2000, 1, 0, 10), 'zero_outside'),
                 ((1800, 2200, 2000, 1, 23, 14), 'stand_outside'))
        for args, kind in cases:
            with self.subTest(kind=kind), self.assertRaises(joint.RangeValidationError) as raised:
                joint.safe_limits(*args)
            details = raised.exception.details
            self.assertEqual(details['kind'], kind)
            self.assertEqual((details['range_a'], details['range_b'], details['zero']), args[:3])
            self.assertEqual((details['trimmed_min'], details['trimmed_max']),
                             (min(args[:2]) + 12, max(args[:2]) - 12))
            self.assertIn(f'A={args[0]}、B={args[1]}', str(raised.exception))
            self.assertIn(f'原零点 {args[2]}', str(raised.exception))
        invalid = ((0, 2048, 2000, 1, 0, 10),
                   (True, 2400, 2000, 1, 0, 10),
                   (1800, 2400, True, 1, 0, 10),
                   (1800, 2400, 2000, True, 0, 10),
                   (1800, 2400, 2000, 0, 0, 10),
                   (1800, 2400, 2000, 1, float('nan'), 10),
                   (1800, 2400, 2000, 1, '0', 10),
                   (1800, 2400, 2000, 1, True, 10),
                   (1800, 2400, 2000, 1, 0, 99),
                   (2007, 2400, 2000, 1, 0, 34),
                   (2000, 2035, 2000, 1, 0, 34),
                   (2000, 2040, 2000, 1, 23, 34))
        for args in invalid:
            with self.subTest(invalid=args), self.assertRaises(ValueError) as raised:
                joint.safe_limits(*args)
            self.assertNotIsInstance(raised.exception, joint.RangeValidationError)

    def test_zero_and_default_stand_are_inclusive_without_widening_trim(self):
        self.assertEqual(joint.safe_limits(1988, 2013, 2000, 1, 0, 10), (2000, 2001))
        self.assertEqual(joint.safe_limits(1987, 2012, 2000, 1, 0, 10), (1999, 2000))
        with self.assertRaises(joint.RangeValidationError):
            joint.safe_limits(1988, 2012, 2000, 1, 0, 10)
        # Both zero and this explicit -23-degree test target must fit.
        self.assertEqual(joint.safe_limits(1758, 2044, 2032, 1, -23, 12), (1770, 2032))
        self.assertEqual(joint.safe_limits(2020, 2306, 2032, -1, -23, 12), (2032, 2294))
        for args in ((1759, 2044, 2032, 1, -23, 12),
                     (2020, 2305, 2032, -1, -23, 12)):
            with self.subTest(args=args), self.assertRaises(joint.RangeValidationError) as raised:
                joint.safe_limits(*args)
            self.assertEqual(raised.exception.details['kind'], 'stand_outside')

    def test_fixed_stance_matches_policy_home_plus_clearance_override_and_controller_scale(self):
        root = Path(__file__).resolve().parents[1] / 'x5_policy'
        tree = ast.parse((root / 'rl_core.py').read_text())
        statements = [node for node in tree.body if isinstance(node, ast.Assign)
                      and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                      and node.targets[0].id in ('q', 'DEFAULT_DEG', 'FIXED_STANCE_OVERRIDES', 'MOTOR_IDS')]
        values = {}
        exec(compile(ast.Module(body=statements, type_ignores=[]), '<pure-pose-constants>', 'exec'), values)
        expected = list(values['DEFAULT_DEG'])
        for sid, degrees in values['FIXED_STANCE_OVERRIDES'].items():
            expected[values['MOTOR_IDS'].index(sid)] = degrees
        self.assertEqual(tuple(expected), joint.DEFAULT_DEG)
        self.assertEqual(round(1977 - values['DEFAULT_DEG'][10] * 4095 / 360), 1750)
        self.assertEqual(joint.DEFAULT_DEG[10], 0.)
        motion = ast.parse((root / 'motion.py').read_text())
        target = next(node for node in ast.walk(motion) if isinstance(node, ast.FunctionDef)
                      and node.name == '_raw_targets_locked')
        self.assertIn(4095, [node.value for node in ast.walk(target) if isinstance(node, ast.Constant)])

    def test_recorded_neck_range_contains_fixed_stance_without_expanding_limits_or_changing_zero(self):
        self.assertEqual(joint.safe_limits(1802, 2782, 1977, -1, joint.DEFAULT_DEG[10], 30),
                         (1814, 2770))
        cfg = config()
        cfg['joints'][10].update(zero=1977, sign=-1, min=1814, max=2770, direction_confirmed=True)
        before = copy.deepcopy(cfg)
        self.assertEqual(joint.completed_count(cfg), 1)
        self.assertEqual(cfg, before)
        # The old model home remains outside the recorded safe range.
        with self.assertRaises(joint.RangeValidationError):
            joint.safe_limits(1802, 2782, 1977, -1, 19.99622705006573, 30)


class WizardTests(unittest.TestCase):
    def setUp(self):
        self.config = config()
        self.wizard = joint.JointWizard()
        self.now = 20.
        self.sample = 1000000
        self.raw = 2000
        self.sid = 10

    def advance(self, raw=None, wall=.02, sample=20000):
        if raw is not None:
            self.raw = raw
        self.now += wall
        self.sample = (self.sample + sample) & 0xffffffff
        self.wizard.tick(state(self.raw, self.sample, self.sid), self.now, self.config)

    def capture(self, action, raw):
        if self.wizard.active():
            self.advance(raw)
        else:
            self.raw = raw
        self.wizard.start(action, self.sid, state(self.raw, self.sample, self.sid), self.now)
        for _ in range(20):
            self.advance()
        self.assertFalse(self.wizard.value['busy'])

    def rejected_range(self, a=1800, b=2005):
        for action, raw in (('direction-start', 2000), ('direction-end', 2050),
                            ('range-a', a), ('range-b', b)):
            self.capture(action, raw)
        self.assertTrue(self.wizard.value['range_retry_allowed'])

    def assert_cleared(self):
        for field in ('id', 'start_position', 'end_position', 'inferred_sign',
                      'range_a', 'range_b', 'last_range_rejection'):
            self.assertIsNone(self.wizard.value[field], field)
        self.assertFalse(self.wizard.value['range_retry_allowed'])
        self.assertFalse(self.wizard.value['ready_to_save'])
        self.assertIsNone(self.wizard.collection)
        self.assertIsNone(self.wizard.limits)

    def test_complete_draft_proposes_only_selected_direction_ranges_preserving_zero_imu(self):
        for action, raw in (('direction-start', 2000), ('direction-end', 2050),
                            ('range-a', 1600), ('range-b', 2400)):
            self.capture(action, raw)
        self.assertTrue(self.wizard.value['ready_to_save'])
        proposed = self.wizard.proposed_config(self.config, 10)
        self.assertEqual(proposed['joints'][0], dict(id=10, zero=2000, sign=1,
                          min=1612, max=2388, direction_confirmed=True))
        self.assertEqual(proposed['joints'][1:], self.config['joints'][1:])
        self.assertEqual(proposed['imu'], self.config['imu'])
        self.assertEqual(joint.zero_signature(proposed), joint.zero_signature(self.config))
        self.assertFalse(self.config['joints'][0]['direction_confirmed'])
        self.wizard.saved()
        self.assertEqual(self.wizard.snapshot(proposed)['completed_count'], 1)
        self.assertEqual(self.wizard.value['stage'], 'complete')

    def test_direction_only_reuses_trimmed_bounds_exactly_and_changes_only_sign(self):
        self.config['joints'][0].update(sign=-1, min=1612, max=2388, direction_confirmed=True)
        original = copy.deepcopy(self.config)
        self.capture('direction-start', 2000)
        self.capture('direction-end', 2050)
        self.assertTrue(self.wizard.snapshot(self.config)['direction_reuse_allowed'])
        updated = self.wizard.proposed_direction_config(self.config, 10)
        expected = copy.deepcopy(original)
        expected['joints'][0]['sign'] = 1
        self.assertEqual(updated, expected)
        self.assertEqual(self.config, original)
        self.assertIsNone(self.wizard.value['range_a'])
        self.assertIsNone(self.wizard.value['range_b'])
        self.assertFalse(self.wizard.value['ready_to_save'])

    def test_direction_only_rejects_reversed_stand_outside_existing_bounds_without_expansion(self):
        self.sid = 12
        self.config['joints'][2].update(sign=1, min=1712, max=2200, direction_confirmed=True)
        original = copy.deepcopy(self.config)
        self.capture('direction-start', 2000)
        self.capture('direction-end', 1950)
        view = self.wizard.snapshot(self.config)
        self.assertFalse(view['direction_reuse_allowed'])
        self.assertIn('默认站姿超出', view['direction_reuse_reason'])
        with self.assertRaisesRegex(ValueError, '范围未扩大'):
            self.wizard.proposed_direction_config(self.config, 12)
        self.assertEqual(self.config, original)

    def test_direction_only_requires_two_completed_samples_original_bounds_and_same_joint(self):
        self.config['joints'][0].update(sign=-1, min=1612, max=2388, direction_confirmed=True)
        self.capture('direction-start', 2000)
        with self.assertRaises(ValueError):
            self.wizard.proposed_direction_config(self.config, 10)
        self.capture('direction-end', 2050)
        for changes in (dict(min=None), dict(min=True), dict(max=1612), dict(max=4095),
                        dict(min=2001), dict(direction_confirmed=False), dict(id=11)):
            bad = copy.deepcopy(self.config)
            bad['joints'][0].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.wizard.proposed_direction_config(bad, 10)
        for sid in (11, True, 99):
            with self.subTest(sid=sid), self.assertRaises(ValueError):
                self.wizard.proposed_direction_config(self.config, sid)
        self.wizard.value['inferred_sign'] = -1
        with self.assertRaisesRegex(ValueError, '不一致'):
            self.wizard.proposed_direction_config(self.config, 10)
        self.wizard.value['inferred_sign'] = 1
        self.capture('range-a', 1600)
        with self.assertRaises(ValueError):
            self.wizard.proposed_direction_config(self.config, 10)

    def test_mouth_requires_original_closed_opened_direction_range_checks(self):
        self.sid = 34
        self.config['joints'][-1].update(sign=1, min=2000, max=2400, direction_confirmed=True)
        self.capture('direction-start', 2000)
        self.capture('direction-end', 2050)
        self.assertFalse(self.wizard.snapshot(self.config)['direction_reuse_allowed'])
        with self.assertRaisesRegex(ValueError, '轻合、张开'):
            self.wizard.proposed_direction_config(self.config, 34)

    def test_geometry_rejection_preserves_direction_and_a_but_cannot_save(self):
        original = copy.deepcopy(self.config)
        self.rejected_range()
        view = self.wizard.snapshot(self.config)
        self.assertEqual((view['id'], view['start_position'], view['end_position'],
                          view['inferred_sign'], view['range_a']), (10, 2000, 2050, 1, 1800))
        self.assertEqual(view['stage'], 'await-range-b')
        self.assertFalse(view['busy'])
        self.assertFalse(view['ready_to_save'])
        self.assertIsNone(view['range_b'])
        self.assertIsNone(self.wizard.limits)
        self.assertIsNone(self.wizard.collection)
        self.assertEqual(view['last_range_rejection'], dict(
            id=10, range_a=1800, range_b=2005, raw_min=1800, raw_max=2005,
            trimmed_min=1812, trimmed_max=1993, zero=2000, stand=2000,
            kind='zero_outside', reason='两侧各留 12 ticks 余量后未包含原零点，本关节未保存'))
        self.assertIn('A=1800、B=2005', view['error'])
        self.assertIn('1812–1993', view['error'])
        self.assertIn('原零点 2000', view['error'])
        with self.assertRaises(ValueError):
            self.wizard.proposed_config(self.config, 10)
        self.assertEqual(self.config, original)
        view['last_range_rejection']['zero'] = 0
        self.assertEqual(self.wizard.value['last_range_rejection']['zero'], 2000)

    def test_retry_b_still_needs_explicit_save_and_preserves_saved_configuration(self):
        original = copy.deepcopy(self.config)
        self.rejected_range()
        self.wizard.start('range-b', 10, state(self.raw, self.sample), self.now)
        self.assertFalse(self.wizard.value['range_retry_allowed'])
        self.assertIsNone(self.wizard.value['error'])
        # A failed request is retained only as diagnostic history, not an endpoint.
        self.assertEqual(self.wizard.value['last_range_rejection']['range_b'], 2005)
        self.raw = 2400
        for _ in range(20):
            self.advance()
        self.assertEqual(self.wizard.value['stage'], 'ready')
        self.assertTrue(self.wizard.value['ready_to_save'])
        self.assertFalse(self.wizard.value['range_retry_allowed'])
        self.assertEqual(self.wizard.limits, (1812, 2388))
        self.assertEqual(self.config, original)
        self.assertEqual(self.wizard.snapshot(self.config)['completed_count'], 0)
        proposed = self.wizard.proposed_config(self.config, 10)
        self.assertEqual(proposed['joints'][0]['min'], 1812)
        self.assertEqual(proposed['joints'][0]['max'], 2388)
        self.assertEqual(joint.zero_signature(proposed), joint.zero_signature(original))
        self.assertEqual(proposed['imu'], original['imu'])
        self.assertEqual(self.config, original)
        self.wizard.saved()
        self.assertEqual(self.wizard.value['stage'], 'complete')
        self.assertEqual(self.wizard.snapshot(proposed)['completed_count'], 1)

    def test_retry_a_clears_both_endpoints_without_repeating_direction(self):
        self.rejected_range(a=2000, b=2400)
        self.wizard.start('range-a', 10, state(self.raw, self.sample), self.now)
        self.assertEqual((self.wizard.value['start_position'], self.wizard.value['end_position'],
                          self.wizard.value['inferred_sign']), (2000, 2050, 1))
        self.assertIsNone(self.wizard.value['range_a'])
        self.assertIsNone(self.wizard.value['range_b'])
        self.assertIsNone(self.wizard.value['error'])
        self.assertFalse(self.wizard.value['range_retry_allowed'])
        self.raw = 1800
        for _ in range(20):
            self.advance()
        self.assertEqual(self.wizard.value['range_a'], 1800)
        self.assertEqual(self.wizard.value['stage'], 'await-range-b')
        self.assertFalse(self.wizard.value['ready_to_save'])
        self.capture('range-b', 2400)
        self.assertEqual(self.wizard.limits, (1812, 2388))

    def test_non_geometry_errors_after_rejection_still_fail_entire_draft(self):
        for cause in ('timeout', 'gap', 'reversal', 'crossing', 'invalid-config'):
            with self.subTest(cause=cause):
                self.setUp()
                self.rejected_range()
                self.wizard.start('range-b', 10, state(self.raw, self.sample), self.now)
                try:
                    if cause == 'timeout':
                        self.advance(wall=3.01, sample=0)
                    elif cause == 'gap':
                        self.advance(sample=100001)
                    elif cause == 'reversal':
                        self.advance(sample=0xffffffff)
                    elif cause == 'crossing':
                        self.advance(raw=4090)
                    else:
                        self.config['joints'][0]['id'] = 20
                        for _ in range(20):
                            self.advance()
                except ValueError as error:
                    self.assertNotIsInstance(error, joint.RangeValidationError)
                    # Host health/transport paths already call fail on ordinary exceptions.
                    self.wizard.fail(error)
                else:
                    self.fail('unsafe feedback/configuration was accepted')
                self.assertEqual(self.wizard.value['stage'], 'failed')
                self.assert_cleared()

    def test_crossing_sized_range_is_not_retryable_and_cancel_or_clear_discards_history(self):
        for action, raw in (('direction-start', 2000), ('direction-end', 2050), ('range-a', 900)):
            self.capture(action, raw)
        # Intermediate feedback avoids a one-frame jump; the endpoint span is still forbidden.
        self.advance(raw=2000)
        self.advance(raw=2948)
        self.wizard.start('range-b', 10, state(self.raw, self.sample), self.now)
        with self.assertRaises(ValueError) as raised:
            for _ in range(20):
                self.advance()
        self.assertNotIsInstance(raised.exception, joint.RangeValidationError)
        self.wizard.fail(raised.exception)
        self.assert_cleared()
        for reset in ('cancel', 'clear', 'health-fail'):
            with self.subTest(reset=reset):
                self.setUp()
                self.rejected_range()
                if reset == 'health-fail':
                    self.wizard.fail('真实反馈失联')
                else:
                    getattr(self.wizard, reset)()
                self.assert_cleared()

    def test_twelve_samples_and_wall_time_alone_do_not_replace_sensor_time_span(self):
        self.wizard.start('direction-start', 10, state(), self.now)
        self.advance(wall=.15, sample=0)
        for _ in range(12):
            self.advance(wall=.02, sample=20000)
        self.assertGreater(self.now - 20, .35)
        self.assertTrue(self.wizard.value['busy'])
        for _ in range(7):
            self.advance()
        self.assertFalse(self.wizard.value['busy'])

    def test_duplicate_sample_clock_never_counts_and_timeout_still_fires(self):
        self.wizard.start('direction-start', 10, state(), self.now)
        self.advance(wall=2.9, sample=0)
        self.assertEqual(self.wizard.value['sample_count'], 0)
        with self.assertRaisesRegex(ValueError, '3 秒'):
            self.advance(wall=.11, sample=0)

    def test_gap_wrap_reversal_and_unstable_capture_are_rejected(self):
        for delta in (100001, 0xffffffff):
            self.wizard = joint.JointWizard()
            self.wizard.start('direction-start', 10, state(self.raw, self.sample), self.now)
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                self.advance(sample=delta)
        self.wizard = joint.JointWizard()
        self.raw = 3000
        self.wizard.start('direction-start', 10, state(self.raw, self.sample), self.now)
        with self.assertRaisesRegex(ValueError, '跨零点'):
            self.advance(raw=0)
        self.wizard = joint.JointWizard()
        self.raw = 2000
        self.wizard.start('direction-start', 10, state(self.raw, self.sample), self.now)
        with self.assertRaisesRegex(ValueError, '不稳定'):
            for index in range(20):
                self.advance(raw=2000 + (index % 2) * 7)

    def test_exact_100ms_gap_and_timestamp_unsigned_wrap_remain_valid(self):
        self.sample = 0xffffffff - 10000
        self.wizard.start('direction-start', 10, state(self.raw, self.sample), self.now)
        self.advance(sample=100000, wall=.1)
        for _ in range(20):
            self.advance()
        self.assertEqual(self.wizard.value['start_position'], 2000)

    def test_cross_zero_between_clicks_fails_entire_unsaved_draft(self):
        self.capture('direction-start', 4000)
        self.assertFalse(self.wizard.value['busy'])
        with self.assertRaisesRegex(ValueError, '跨零点'):
            self.advance(raw=2)
        self.wizard.fail('跨零点')
        self.assertIsNone(self.wizard.value['start_position'])
        self.assertFalse(self.wizard.value['ready_to_save'])

    def test_order_id_and_cancel_prevent_reusing_partial_capture(self):
        with self.assertRaises(ValueError):
            self.wizard.start('direction-end', 10, state(), self.now)
        self.capture('direction-start', 2000)
        with self.assertRaises(ValueError):
            self.wizard.start('direction-end', 20, state(), self.now)
        self.wizard.cancel()
        self.assertFalse(self.wizard.active())
        self.assertIsNone(self.wizard.value['id'])
        self.assertEqual(self.wizard.snapshot(self.config)['completed_count'], 0)


if __name__ == '__main__':
    unittest.main()
