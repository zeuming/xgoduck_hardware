#!/usr/bin/env python3
"""Synthetic, hardware-free IMU wizard checks against the actual controller."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'x5_policy'))
import motion
import imu_wizard
imu_wizard.motion = motion


def gravity(front=0., left=0.):
    front, left = np.radians([front, left])
    # Individually controlled tilts; mixed tilt is used only to test rejection.
    return 9.81 * np.array([np.sin(front), np.sin(left), np.cos(front) * np.cos(left)])


class Controller(imu_wizard.ImuWizardMixin, motion.MotionController):
    def __init__(self):
        self.now = 10.
        super().__init__({}, config_path=Path(tempfile.gettempdir()) / 'xgoduck-synthetic-no-file', clock=lambda: self.now)
        self._connected = True
        self._generation = 17
        for index, joint in enumerate(self._config['joints']):
            joint.update(zero=1800 + index, min=1000 + index, max=3000 + index,
                         sign=1 if index % 2 else -1, direction_confirmed=True)
        self._config['mechanical_reference_confirmed'] = True
        self._config['imu'].update(axes=['y', '-x', 'z'], axes_confirmed=True,
                                   calibrated=True, bias_dps=[.2, -.1, .05])
        self.saved = []
        self.save_error = False
        self.stamp = 500000
        self.feed(gravity(), dt=.02)

    def _save_locked(self):
        if self.save_error:
            raise OSError('模拟保存失败')
        self.saved.append(copy.deepcopy(self._config))

    def feed(self, acc, gyro=(.12, -.06, .04), dt=.02, **changes):
        self.now += dt
        self.stamp = (self.stamp + 20000) & 0xffffffff
        state = dict(sample_us=self.stamp, imu_us=self.stamp, online_mask=motion.ALL_MASK,
                     fault_mask=0, torque_mask=0, armed=False, imu_ok=True,
                     positions=list(range(1800, 1815)), velocities=[0] * 15,
                     acc=list(acc), gyro=list(gyro), cookie=0, last_error=0, configured=True)
        state.update(changes)
        with self._lock:
            self._accept_state_locked(state)
        return self.status()['imu_wizard']

    def action(self, action, **changes):
        args = dict(support_confirmed=True, mounted_confirmed=True)
        args.update(changes)
        return self.imu_wizard_action(action, self._generation, **args)['imu_wizard']

    def batch(self, acc, gyro=(.12, -.06, .04), count=100):
        for _ in range(count):
            result = self.feed(acc, gyro)
        return result


class ImuWizardTests(unittest.TestCase):
    def test_all_24_mountings_and_physical_force_signs(self):
        rotations = imu_wizard.wizard_rotations()
        self.assertEqual(24, len(rotations))
        for axes, matrix in rotations:
            with self.subTest(axes=axes):
                self.assertEqual(1, round(np.linalg.det(matrix)))
                self.assertEqual(axes, imu_wizard.wizard_find_axes(matrix.T @ gravity(), matrix.T @ gravity(front=30)))
                body = imu_wizard.wizard_verify_pose('left-up', matrix.T @ gravity(left=30), axes)
                self.assertGreater(body[1], 0)
                self.assertAlmostEqual(body[0], 0)

    def test_all_24_full_controller_runs_and_joint_preservation(self):
        for axes, matrix in imu_wizard.wizard_rotations():
            with self.subTest(axes=axes):
                ctl = Controller()
                original = copy.deepcopy(ctl._config)
                ctl.feed(matrix.T @ gravity())
                self.assertTrue(ctl.action('level')['busy'])
                self.assertEqual('front-up', ctl.batch(matrix.T @ gravity())['expected_action'])
                self.assertEqual(original, ctl._config)
                ctl.action('front-up')
                self.assertEqual('left-up', ctl.batch(matrix.T @ gravity(front=30))['expected_action'])
                self.assertEqual(original, ctl._config)
                ctl.action('left-up')
                self.assertEqual('finish', ctl.batch(matrix.T @ gravity(left=30))['expected_action'])
                self.assertEqual([], ctl.saved)
                ctl.action('finish')
                status = ctl.batch(matrix.T @ gravity())
                self.assertEqual('complete', status['stage'])
                self.assertEqual(axes, ctl._config['imu']['axes'])
                self.assertEqual(original['joints'], ctl._config['joints'])
                self.assertTrue(ctl._config['mechanical_reference_confirmed'])
                self.assertTrue(ctl._config['imu']['axes_confirmed'])
                self.assertTrue(ctl._config['imu']['calibrated'])
                np.testing.assert_allclose([.12, -.06, .04], ctl._config['imu']['bias_dps'])
                self.assertEqual(1, len(ctl.saved))

    def test_wrong_front_then_correct_left_fails_independent_validation(self):
        axes = imu_wizard.wizard_find_axes(gravity(), gravity(front=-30))
        with self.assertRaisesRegex(ValueError, '左右方向验证'):
            imu_wizard.wizard_verify_pose('left-up', gravity(left=30), axes)

    def test_ambiguous_mixed_front_rejected(self):
        with self.assertRaisesRegex(ValueError, '无法明确'):
            imu_wizard.wizard_find_axes(gravity(), gravity(front=25, left=25))

    def test_too_little_too_much_and_side_tilt_rejected(self):
        for acc in (gravity(front=5), gravity(front=60), gravity(front=30, left=14)):
            with self.subTest(acc=acc), self.assertRaises(ValueError):
                imu_wizard.wizard_find_axes(gravity(), acc)
        with self.assertRaises(ValueError):
            imu_wizard.wizard_find_axes(gravity(front=15), gravity(front=30))

    def test_wrong_left_or_finish_tilt_rejected(self):
        for acc in (gravity(left=-30), gravity(front=30), gravity(left=60), gravity(left=30, front=15)):
            with self.subTest(acc=acc), self.assertRaises(ValueError):
                imu_wizard.wizard_verify_pose('left-up', acc, ['x', 'y', 'z'])
        with self.assertRaises(ValueError):
            imu_wizard.wizard_verify_pose('level', gravity(front=12), ['x', 'y', 'z'])

    def test_moving_and_bad_gravity_batch_rejected(self):
        samples = [list(gravity()) + [.1, .1, .1] for _ in range(100)]
        for index in range(50):
            samples[index][0] += 1.2
            samples[index][3] += 2.
        with self.assertRaisesRegex(ValueError, '发生移动'):
            imu_wizard.wizard_validate_batch(samples)
        for acc in ([0, 0, 0], [0, 0, 12], [np.nan, 0, 9.81]):
            with self.subTest(acc=acc), self.assertRaises(ValueError):
                imu_wizard.wizard_validate_batch([list(acc) + [0, 0, 0]] * 100)

    def test_order_confirmations_and_concurrent_actions_rejected(self):
        ctl = Controller()
        with self.assertRaises(motion.MotionError):
            ctl.action('front-up')
        for args in ({'support_confirmed': False}, {'mounted_confirmed': False}):
            with self.subTest(args=args), self.assertRaises(motion.MotionError):
                ctl.action('level', **args)
        ctl.action('level')
        with self.assertRaises(motion.MotionError):
            ctl.action('level')
        with self.assertRaises(motion.MotionError):
            ctl.set_imu_axes(['x', 'y', 'z'], True)
        with self.assertRaises(motion.MotionError):
            ctl.calibrate_imu()
        ctl.batch(gravity())
        with self.assertRaises(motion.MotionError):
            ctl._require_no_imu_wizard_locked()
        ctl.action('cancel', support_confirmed=False, mounted_confirmed=False)
        ctl._require_no_imu_wizard_locked()
        self.assertEqual([], ctl.saved)

    def test_any_fault_stale_imu_or_generation_interrupts_without_write(self):
        changes = ({'online_mask': motion.ALL_MASK ^ 1}, {'fault_mask': 1}, {'torque_mask': 1},
                   {'armed': True}, {'imu_ok': False}, {'imu_us': 1}, {'sample_us': 0})
        for change in changes:
            with self.subTest(change=change):
                ctl = Controller()
                original = copy.deepcopy(ctl._config)
                ctl.action('level')
                self.assertEqual('failed', ctl.feed(gravity(), **change)['stage'])
                self.assertEqual(original, ctl._config)
                self.assertEqual([], ctl.saved)
        ctl = Controller()
        ctl.action('level')
        ctl._generation += 1
        self.assertEqual('failed', ctl.feed(gravity())['stage'])
        self.assertEqual('level', ctl.status()['imu_wizard']['expected_action'])

    def test_stale_servo_clock_rejected(self):
        ctl = Controller()
        ctl.action('level')
        old_sample = ctl._state['sample_us']
        result = ctl.feed(gravity(), dt=.12, sample_us=old_sample)
        self.assertEqual('failed', result['stage'])
        self.assertEqual([], ctl.saved)

    def test_unique_timestamps_duration_timeout_and_gap(self):
        ctl = Controller()
        ctl.action('level')
        result = ctl.feed(gravity(), imu_us=ctl._state['imu_us'])
        self.assertEqual(0, result['sample_count'])
        self.assertTrue(result['busy'])
        # 100 distinct ticks arriving in only 100ms must not complete.
        ctl = Controller()
        ctl.action('level')
        for _ in range(100):
            result = ctl.feed(gravity(), dt=.001)
        self.assertTrue(result['busy'])
        self.assertEqual(100, result['sample_count'])
        self.assertEqual([], ctl.saved)
        # Deadline is independent of whether a newly received sample is fresh.
        ctl.now += 6.1
        result = ctl.feed(gravity())
        self.assertEqual('failed', result['stage'])
        self.assertIn('超时', result['error'])
        ctl = Controller()
        ctl.action('level')
        ctl.stamp += 100000
        result = ctl.feed(gravity())
        self.assertEqual('failed', result['stage'])
        self.assertIn('连续采样中断', result['error'])

    def test_timestamp_wrap_is_allowed(self):
        ctl = Controller()
        ctl.stamp = 0xffff0000
        ctl.feed(gravity())
        ctl.action('level')
        self.assertEqual('front-up', ctl.batch(gravity())['expected_action'])

    def test_save_failure_rolls_back_imu_and_keeps_joints(self):
        ctl = Controller()
        original = copy.deepcopy(ctl._config)
        for action, acc in [('level', gravity()), ('front-up', gravity(front=30)), ('left-up', gravity(left=30))]:
            ctl.action(action)
            ctl.batch(acc)
        ctl.save_error = True
        ctl.action('finish')
        result = ctl.batch(gravity())
        self.assertEqual('failed', result['stage'])
        self.assertEqual('finish', result['expected_action'])
        self.assertEqual(original, ctl._config)
        self.assertEqual([], ctl.saved)
        ctl.save_error = False
        ctl.action('finish')
        self.assertEqual('complete', ctl.batch(gravity())['stage'])
        self.assertEqual(1, len(ctl.saved))

    def test_cancel_and_stop_never_save_or_open_serial(self):
        ctl = Controller()
        original = copy.deepcopy(ctl._config)
        ctl.action('level')
        ctl.feed(gravity())
        ctl.imu_wizard_action('cancel', -100, support_confirmed=False, mounted_confirmed=False)
        self.assertEqual('idle', ctl.status()['imu_wizard']['stage'])
        ctl.action('level')
        with ctl._lock:
            ctl._stop_locked('测试停止')
        self.assertEqual('idle', ctl.status()['imu_wizard']['stage'])
        self.assertEqual(original, ctl._config)
        self.assertEqual([], ctl.saved)
        self.assertIsNone(ctl._transport)

    def test_runtime_source_contains_no_cli_and_loads_same_mixin(self):
        namespace = {'motion': motion}
        source = imu_wizard.runtime_source()
        self.assertNotIn("__name__", source)
        exec(compile(source, '<imu-wizard-child-test>', 'exec'), namespace)
        self.assertEqual(24, len(namespace['wizard_rotations']()))
        self.assertTrue(hasattr(namespace['ImuWizardMixin'], 'imu_wizard_action'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
