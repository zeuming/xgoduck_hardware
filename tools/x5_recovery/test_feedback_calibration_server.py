#!/usr/bin/env python3
"""Pure host tests: synthetic packets only; never SSH, serial, or run_reader."""
import copy
import contextlib
from email.message import Message
import io
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest import mock

import numpy as np

import feedback_calibration_server as host


class Clock:
    def __init__(self):
        self.now = 20.

    def __call__(self):
        return self.now


class FakeStdin:
    closed = False

    def fileno(self):
        return 777


class FakeProcess:
    def __init__(self):
        self.stdin = FakeStdin()
        self.returncode = None
        self.terminate_count = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminate_count += 1
        self.returncode = -15


def config():
    ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
    return dict(version=1, mechanical_reference_confirmed=False,
                joints=[dict(id=sid, zero=1000 + index * 73, sign=1,
                             min=None, max=None, direction_confirmed=False)
                        for index, sid in enumerate(ids)],
                imu=dict(axes=['x', 'y', 'z'], axes_confirmed=False,
                         calibrated=False, bias_dps=[0., 0., 0.]))


class HostTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.clock = Clock()
        self.original = config()
        host.atomic_calibration(self.directory, self.original)
        self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock)
        self.session.process = FakeProcess()
        self.control_records = []
        self.control_writer = mock.patch.object(host, 'write_pipe_once',
                                                 side_effect=lambda fd, data, seconds:
                                                 self.control_records.append((fd, data, seconds)))
        self.control_writer.start()
        self.sequence = 1
        self.sample = 1000000
        # Protect all tests from accidentally reaching the hardware launcher.
        self.forbid_process = mock.patch.object(host.subprocess, 'Popen',
                                               side_effect=AssertionError('Hardware process forbidden in pure tests'))
        self.forbid_process.start()
        self.ready()
        self.feed([0., 0., 9.81], advance=0)

    def tearDown(self):
        self.control_writer.stop()
        self.forbid_process.stop()
        self.temporary.cleanup()

    def state(self, accel, gyro=None):
        return dict(sample_us=self.sample & 0xffffffff,
                    imu_us=(self.sample - 1000) & 0xffffffff,
                    online_mask=0, fault_mask=0, torque_mask=0,
                    armed=False, arm_cookie=0, imu_ok=True,
                    accel_ms2=[float(v) for v in accel],
                    gyro_dps=[float(v) for v in (gyro if gyro is not None else [0., 0., 0.])],
                    positions=[0] * 15)

    def ready(self):
        self.session.accept(dict(kind='ready', generation=7,
                                 emitted_monotonic=self.clock(), reader_elapsed_seconds=0.))

    def event(self, state, **changes):
        return dict(kind='state', generation=7, sequence=self.sequence,
                    state=state, received_monotonic=self.clock(),
                    reader_elapsed_seconds=self.clock() - self.session.started, **changes)

    def feed(self, accel, gyro=None, advance=20000):
        self.sample = (self.sample + advance) & 0xffffffff
        self.clock.now += max(20000, advance) / 1000000
        self.sequence += 1
        self.session.accept(self.event(self.state(accel, gyro)))

    def action(self, action, **changes):
        body = dict(action=action, generation=7, mounted_confirmed=True, support_confirmed=True)
        body.update(changes)
        return self.session.action(body)

    def collect(self, action, accel, gyro=None):
        self.action(action)
        for _ in range(100):
            self.feed(accel, gyro)

    def saved(self):
        return json.loads((self.directory / 'calibration-latest.json').read_text())

    def prepare_finish(self):
        self.collect('level', [0., 0., 9.81])
        self.collect('front-up', [4.905, 0., 9.81 * math.cos(math.pi / 6)])
        self.collect('left-up', [0., 4.905, 9.81 * math.cos(math.pi / 6)])

    def test_four_poses_all_24_mounts_save_only_on_finish_preserve_15_zeros(self):
        rotations = host.imu_wizard.wizard_rotations()
        self.assertEqual(len(rotations), 24)
        body_poses = [('level', [0., 0., 9.81]),
                      ('front-up', [4.905, 0., 9.81 * math.cos(math.pi / 6)]),
                      ('left-up', [0., 4.905, 9.81 * math.cos(math.pi / 6)]),
                      ('finish', [0., 0., 9.81])]
        raw_bias = [.1, -.2, .3]
        for axes, matrix in rotations:
            with self.subTest(axes=axes):
                host.atomic_calibration(self.directory, self.original)
                self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock)
                self.session.process = FakeProcess()
                self.ready()
                self.feed(matrix.T @ np.array(body_poses[0][1]))
                for action, body_acc in body_poses:
                    self.collect(action, matrix.T @ np.array(body_acc), raw_bias)
                    self.assertIsNone(self.session.wizard['error'])
                    if action != 'finish':
                        self.assertEqual(self.saved(), self.original)
                        self.assertEqual(self.session.config, self.original)
                self.assertEqual(self.session.wizard['stage'], 'complete')
                self.assertEqual(self.session.config['imu']['axes'], axes)
                self.assertTrue(self.session.config['imu']['calibrated'])
                np.testing.assert_allclose(self.session.config['imu']['bias_dps'], raw_bias)
                self.assertEqual(self.saved(), self.session.config)
                self.assertEqual(self.saved()['joints'], self.original['joints'])
                self.assertEqual(self.saved()['mechanical_reference_confirmed'], False)
                self.assertEqual(self.session.status, 'stopping')
                self.assertEqual(self.session.end_reason, 'completed')
                self.assertIsNone(self.session.snapshot()['feedback'])
                # A separate pure session verifies display conversion of the
                # saved mapping; completed sessions do not resume collecting.
                self.session = host.CalibrationSession(self.saved(), self.directory, 7, self.clock)
                self.session.process = FakeProcess()
                self.ready()
                self.feed(matrix.T @ np.array([0., 0., 9.81]), raw_bias)
                transformed = self.session.snapshot()['body_imu']
                np.testing.assert_allclose(transformed['accel_ms2'], [0., 0., 9.81])
                np.testing.assert_allclose(transformed['gyro_dps'], [0., 0., 0.], atol=1e-12)

    def test_unstable_gyro_fails_without_saving(self):
        self.action('level')
        for index in range(100):
            self.feed([0., 0., 9.81], [(-1.) ** index, 0., 0.])
        self.assertEqual(self.session.wizard['stage'], 'failed')
        self.assertIn('机身发生移动', self.session.wizard['error'])
        self.assertIsNone(self.session.collection)
        self.assertEqual(self.saved(), self.original)
        self.assertEqual(self.control_records, [])

    def test_clock_reset_and_100_ms_gap_fail_without_saving(self):
        for delta, text in ((-20000, '倒退'), (100000, '100 毫秒')):
            with self.subTest(delta=delta):
                self.action('level')
                self.feed([0., 0., 9.81], advance=delta)
                self.assertEqual(self.session.wizard['stage'], 'failed')
                self.assertIn(text, self.session.wizard['error'])
                self.assertIsNone(self.session.collection)
                self.assertEqual(self.saved(), self.original)

    def test_old_generation_unmounted_unsupported_and_stale_cannot_collect(self):
        for changes in (dict(generation=6), dict(mounted_confirmed=False),
                        dict(support_confirmed=False)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.action('level', **changes)
        self.assertIsNone(self.session.collection)
        with self.assertRaises(ValueError):
            old_event = self.event(self.state([0., 0., 9.81]))
            old_event['generation'] = 6
            self.session.accept(old_event)
        self.clock.now += host.FRESH
        with self.assertRaises(ValueError):
            self.action('level')
        self.assertIsNone(self.session.snapshot()['feedback'])
        self.assertEqual(self.saved(), self.original)

    def test_servo_powered_feedback_cannot_start_imu_calibration(self):
        state = self.state([0., 0., 9.81])
        state['online_mask'] = 32767
        self.session.accept(self.event(state))
        with self.assertRaisesRegex(ValueError, '电池断开'):
            self.action('level')
        self.assertEqual(self.saved(), self.original)

    def test_backlogged_or_clock_inconsistent_state_is_rejected_before_collecting(self):
        self.action('level')
        previous = copy.deepcopy(self.session.feedback)
        received_at = self.session.received_at
        for lag in (.201, -.051):
            with self.subTest(lag=lag):
                event = self.event(self.state([0., 0., 9.81]))
                event['received_monotonic'] = self.clock() - lag
                with self.assertRaisesRegex(ValueError, '积压或传输计时异常'):
                    self.session.accept(event)
                self.assertEqual(self.session.feedback, previous)
                self.assertEqual(self.session.received_at, received_at)
                self.assertEqual(self.session.collection['samples'], [])
                self.assertEqual(self.saved(), self.original)
        # The consumer catches that exception and locks the session. Exercise
        # the pure fault operation, without starting the real reader.
        self.session.fault('backlogged sensor data')
        self.assertIsNone(self.session.collection)
        self.assertIsNone(self.session.snapshot()['feedback'])

    def test_accepted_transport_lag_counts_toward_feedback_age(self):
        event = self.event(self.state([0., 0., 9.81]))
        event['received_monotonic'] = self.clock() - .1
        self.session.accept(event)
        self.assertAlmostEqual(self.session.snapshot()['session']['received_age_ms'], 100.)
        self.clock.now += .41
        self.assertIsNone(self.session.snapshot()['feedback'])
        with self.assertRaisesRegex(ValueError, '已过期'):
            self.action('level')

    def test_timestamp_wrap_is_valid(self):
        self.sample = 0xfffff000
        self.feed([0., 0., 9.81], advance=0)
        self.collect('level', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['expected_action'], 'front-up')
        self.assertIsNone(self.session.wizard['error'])

    def test_save_failure_does_not_replace_old_in_memory_or_saved_calibration(self):
        self.prepare_finish()
        with mock.patch.object(host, 'atomic_calibration', side_effect=OSError('test save failed')):
            self.collect('finish', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'failed')
        self.assertIn('test save failed', self.session.wizard['error'])
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.saved(), self.original)
        self.assertEqual(self.control_records, [])
        self.assertIsNone(self.session.end_reason)

    def test_precommit_file_sync_failure_keeps_old_calibration(self):
        self.prepare_finish()
        with mock.patch.object(host.os, 'fsync', side_effect=OSError('test file fsync failed')):
            self.collect('finish', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'failed')
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.saved(), self.original)

    def test_postcommit_directory_sync_failure_reports_warning_and_keeps_memory_consistent(self):
        self.prepare_finish()
        original_fsync = host.os.fsync

        def fail_directory_sync(descriptor):
            if stat.S_ISDIR(host.os.fstat(descriptor).st_mode):
                raise OSError('test directory fsync failed after rename')
            return original_fsync(descriptor)

        with mock.patch.object(host.os, 'fsync', side_effect=fail_directory_sync):
            self.collect('finish', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertTrue(self.session.wizard['durability_warning'])
        self.assertIn('持久化确认失败', self.session.wizard['message'])
        self.assertEqual(self.saved(), self.session.config)
        self.assertTrue(self.saved()['imu']['calibrated'])
        self.assertEqual(self.saved()['joints'], self.original['joints'])

    def test_fault_and_closed_keep_actual_reader_reason(self):
        self.session.accept(dict(kind='fault', generation=7, reason='specific reader fault'))
        self.assertEqual(self.session.error, 'specific reader fault')
        self.session.accept(dict(kind='closed', generation=7, reason='closed after specific fault',
                                 serial_closed=True))
        self.assertEqual(self.session.error, 'specific reader fault')
        self.assertIsNone(self.session.snapshot()['feedback'])

    def events(self):
        path = self.directory / 'wizard-events.jsonl'
        return [json.loads(line) for line in path.read_text().splitlines()]

    def test_no_overall_deadline_and_discrete_success_audit(self):
        self.assertFalse(hasattr(host, 'SESSION_SECONDS'))
        self.assertFalse(hasattr(self.session, 'deadline'))
        self.assertNotIn('remaining_seconds', self.session.snapshot()['session'])
        self.collect('level', [0., 0., 9.81])
        records = self.events()
        self.assertEqual([r['event'] for r in records], ['start', 'success'])
        self.assertEqual(records[-1]['sample_count'], 100)
        self.assertAlmostEqual(records[-1]['elapsed_ms'], 2000.)
        self.assertEqual(records[-1]['action'], 'level')
        self.assertEqual(records[-1]['generation'], 7)
        self.assertTrue(all(set(r) == {'event', 'generation', 'action', 'sample_count',
                                     'elapsed_ms', 'error'} for r in records))
        self.assertEqual((self.directory / 'wizard-events.jsonl').stat().st_mode & 0o777, 0o600)

    def test_failure_and_cancel_audit_are_discrete(self):
        self.action('level')
        self.feed([0., 0., 9.81], advance=100000)
        records = self.events()
        self.assertEqual([r['event'] for r in records], ['start', 'fail'])
        self.assertIn('100 毫秒', records[-1]['error'])
        self.assertEqual(records[-1]['sample_count'], 0)
        self.action('level')
        for _ in range(5):
            self.feed([0., 0., 9.81])
        self.action('cancel')
        records = self.events()
        self.assertEqual([r['event'] for r in records], ['start', 'fail', 'start', 'cancel'])
        self.assertEqual(records[-1]['sample_count'], 5)
        self.assertFalse(self.session.wizard['busy'])

    def test_audit_failure_warns_without_hanging_or_changing_calibration(self):
        original_open = host.os.open

        def fail_only_audit(path, *args, **kwargs):
            if Path(path).name == 'wizard-events.jsonl':
                raise OSError('audit test failure')
            return original_open(path, *args, **kwargs)

        with mock.patch.object(host.os, 'open', side_effect=fail_only_audit):
            self.collect('level', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'front-up')
        self.assertFalse(self.session.wizard['busy'])
        self.assertTrue(self.session.wizard['audit_warning'])
        self.assertIn('步骤记录写入失败', self.session.wizard['message'])
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.saved(), self.original)

    def test_normal_closed_message_is_clear_and_no_calibration_change(self):
        self.session.end_action({'generation': 7})
        self.session.accept(dict(kind='closed', generation=7, normal=True,
                                 serial_closed=True, reason='Feedback ended by Mac'))
        self.assertEqual(self.session.status, 'stopping')
        self.assertIn('正在确认采集进程退出', self.session.message)
        self.session.confirm_closed(0)
        self.assertEqual(self.session.status, 'closed')
        self.assertIsNone(self.session.error)
        self.assertIn('手动结束', self.session.message)
        self.assertEqual(self.session.closed_proof['reason'], 'Feedback ended by Mac')
        self.assertEqual(self.saved(), self.original)
        self.assertIsNone(self.session.snapshot()['feedback'])

    def test_manual_end_cancels_partial_sampling_sends_once_and_discards_inflight_state(self):
        self.action('level')
        self.feed([0., 0., 9.81])
        snapshot = self.session.end_action({'generation': 7})
        self.assertEqual(snapshot['session']['status'], 'stopping')
        self.assertEqual(snapshot['session']['end_reason'], 'manual')
        self.assertIsNone(self.session.collection)
        self.assertFalse(self.session.wizard['busy'])
        self.assertEqual(self.session.wizard['stage'], 'cancelled')
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        self.feed([0., 0., 9.81])
        self.assertEqual(self.session.status, 'stopping')
        self.assertIsNone(self.session.feedback)
        self.assertIsNone(self.session.collection)
        buffered_event = self.event(self.state([0., 0., 9.81]))
        buffered_event['received_monotonic'] -= 1
        self.session.accept(buffered_event)
        self.assertEqual(self.session.status, 'stopping')
        self.session.end_action({'generation': 7})
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        self.assertEqual(self.saved(), self.original)

    def test_end_rejects_old_generation_bool_and_extra_fields_before_writing(self):
        for body in ({'generation': 6}, {'generation': True}, {},
                     {'generation': 7, 'support_confirmed': False}):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.session.end_action(body)
        self.assertEqual(self.control_records, [])
        self.assertEqual(self.session.status, 'active')

    def test_manual_end_needs_no_fresh_sensor_or_support_checkboxes(self):
        self.clock.now += 1
        snapshot = self.session.end_action({'generation': 7})
        self.assertEqual(snapshot['session']['status'], 'stopping')
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])

    def test_manual_end_during_waiting_accepts_ready_then_ignores_state(self):
        self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock)
        self.session.process = FakeProcess()
        self.session.end_action({'generation': 7})
        self.ready()
        self.feed([0., 0., 9.81])
        self.assertEqual(self.session.status, 'stopping')
        self.assertIsNone(self.session.feedback)

    def test_completed_automatic_end_and_manual_race_send_only_one_record(self):
        self.prepare_finish()
        self.collect('finish', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.session.end_reason, 'completed')
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        threads = [threading.Thread(target=self.session.request_end,
                                    args=('manual' if index % 2 else 'completed',))
                   for index in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        self.session.accept(dict(kind='closed', generation=7, normal=True,
                                 serial_closed=True, reason='Feedback ended by Mac'))
        self.assertEqual(self.session.status, 'stopping')
        self.session.confirm_closed(0)
        self.assertEqual(self.session.status, 'closed')
        self.assertIn('四步校准已保存', self.session.message)
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.saved(), self.session.config)

    def test_successful_save_and_control_write_failure_keep_complete_calibration(self):
        self.prepare_finish()
        with mock.patch.object(host, 'write_pipe_once', side_effect=TimeoutError('control test failure')) as writer:
            self.collect('finish', [0., 0., 9.81])
        self.assertEqual(writer.call_count, 1)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertIsNone(self.session.wizard['error'])
        self.assertTrue(self.session.config['imu']['calibrated'])
        self.assertEqual(self.saved(), self.session.config)
        self.assertEqual(self.saved()['joints'], self.original['joints'])
        self.assertIn('校准已保存', self.session.message)
        self.assertFalse(self.session.end_request_sent)
        self.session.request_end('manual')
        self.assertEqual(writer.call_count, 1)

    def test_save_complete_then_reader_fault_or_failed_close_does_not_undo_success(self):
        self.prepare_finish()
        self.collect('finish', [0., 0., 9.81])
        self.session.accept(dict(kind='fault', generation=7, reason='close test failure'))
        self.session.accept(dict(kind='closed', generation=7, normal=False,
                                 serial_closed=False, reason='close test failure'))
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.saved(), self.session.config)

    def test_unrequested_or_unproven_normal_close_is_not_declared_successful(self):
        self.session.accept(dict(kind='closed', generation=7, normal=True,
                                 serial_closed=True, reason='unexpected normal close'))
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.error, 'unexpected normal close')
        self.assertIsNone(self.session.end_reason)

    def test_requested_stop_without_closed_proof_is_bounded_fault(self):
        self.session.end_action({'generation': 7})
        self.clock.now += 1.99
        self.assertFalse(self.session.check_watchdog())
        self.clock.now += .02
        self.assertTrue(self.session.check_watchdog())
        self.assertEqual(self.session.status, 'fault')
        self.assertIn('关闭证明', self.session.error)
        self.assertEqual(self.session.process.terminate_count, 0)
        self.assertTrue(self.session.cleanup_due())
        self.assertIsNone(self.session.closed_proof)
        self.assertEqual(self.saved(), self.original)

    def test_late_close_evidence_or_process_error_marks_fault_preserving_saved_success(self):
        self.prepare_finish()
        self.collect('finish', [0., 0., 9.81])
        proof = dict(kind='closed', generation=7, normal=True,
                     serial_closed=True, reason='Feedback ended by Mac')
        self.session.accept(proof)
        self.assertEqual(self.session.status, 'stopping')
        self.session.confirm_closed(0)
        self.assertEqual(self.session.status, 'closed')
        self.session.fault('late evidence/process error')
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.closed_proof, proof)
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.saved(), self.session.config)

    def test_uart_close_proof_with_nonzero_ssh_exit_faults_without_undoing_saved_calibration(self):
        self.prepare_finish()
        self.collect('finish', [0., 0., 9.81])
        proof = dict(kind='closed', generation=7, normal=True,
                     serial_closed=True, reason='Feedback ended by Mac')
        self.session.accept(proof)
        self.assertEqual(self.session.status, 'stopping')
        self.session.confirm_closed(255)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.closed_proof, proof)
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.saved(), self.session.config)

    def test_active_feedback_after_long_time_has_no_absolute_expiry(self):
        self.clock.now += 24 * 3600
        self.feed([0., 0., 9.81])
        self.assertFalse(self.session.check_watchdog())
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.end_reason)
        self.assertEqual(self.control_records, [])

    def test_strict_json_rejects_duplicate_fields_and_nan(self):
        for raw in ('{"action":"level","action":"finish"}', '{"reading":NaN}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                host.strict_json(raw)

    def test_pure_handler_host_origin_and_csrf_gate_without_network(self):
        port = 8767
        handler = host.handler_for(self.session, port)
        valid_host = f'127.0.0.1:{port}'
        valid_headers = {'Host': valid_host, 'Origin': 'http://' + valid_host,
                         'X-CSRF-Token': self.session.token, 'Content-Type': 'application/json'}
        body = json.dumps(dict(action='cancel', generation=7, mounted_confirmed=False,
                               support_confirmed=False))

        def request(method, path, headers, body=None):
            data = b'' if body is None else body.encode()
            instance = handler.__new__(handler)
            instance.path = path
            instance.headers = Message()
            for key, value in {**headers, 'Content-Length': str(len(data))}.items():
                instance.headers[key] = value
            instance.rfile = io.BytesIO(data)
            result = []
            instance.reply = lambda code, value, *args, **kwargs: result.append((code, value))
            getattr(instance, 'do_' + method)()
            return result[0]

        status, _ = request('GET', '/api/status', {'Host': 'evil.invalid'})
        self.assertEqual(status, 403)
        status, snapshot = request('GET', '/api/status', {'Host': valid_host})
        self.assertEqual(status, 200)
        self.assertEqual(snapshot['token'], self.session.token)
        for changes in ({'Host': 'evil.invalid'}, {'Origin': 'https://evil.invalid'},
                        {'Origin': ''}, {'X-CSRF-Token': 'wrong'}):
            for route in ('/api/imu-wizard', '/api/end-feedback', '/api/recover-feedback'):
                status, _ = request('POST', route, {**valid_headers, **changes}, body)
                self.assertEqual(status, 403)
        status, _ = request('POST', '/api/target', valid_headers, body)
        self.assertEqual(status, 403)
        status, _ = request('POST', '/api/imu-wizard', valid_headers, body)
        self.assertEqual(status, 200)
        status, _ = request('POST', '/api/end-feedback', valid_headers, json.dumps({'generation': 6}))
        self.assertEqual(status, 409)
        status, _ = request('POST', '/api/end-feedback', valid_headers, json.dumps({'generation': 7, 'extra': True}))
        self.assertEqual(status, 409)
        status, result = request('POST', '/api/end-feedback', valid_headers, json.dumps({'generation': 7}))
        self.assertEqual(status, 200)
        self.assertEqual(result['session']['status'], 'stopping')
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])

    def test_fault_grace_preserves_queued_closed_and_first_reason(self):
        self.action('level')
        self.session.check_watchdog()
        self.clock.now += .61
        self.assertTrue(self.session.check_watchdog())
        first_error = self.session.error
        self.assertIn('0.6 秒', first_error)
        self.assertEqual(self.session.process.terminate_count, 0)
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        self.assertFalse(self.session.cleanup_due())
        self.assertIsNone(self.session.snapshot()['feedback'])
        self.assertFalse(self.session.wizard['busy'])
        self.feed([0., 0., 9.81])  # A legitimate queued STATE is discarded.
        self.session.accept(dict(kind='fault', generation=7, reason='later remote detail'))
        proof = dict(kind='closed', generation=7, normal=True, serial_closed=True,
                     reason='Feedback ended by Mac')
        self.session.accept(proof)
        self.session.confirm_closed(0)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.error, first_error)
        self.assertEqual(self.session.closed_proof, proof)
        self.assertEqual(len(self.control_records), 1)
        self.clock.now += 2
        self.assertTrue(self.session.cleanup_due())
        self.session.process.returncode = 255
        self.session.write_fault_evidence(self.directory, self.session.process)
        evidence = json.loads((self.directory / 'fault-evidence.json').read_text())
        self.assertEqual(evidence['error'], first_error)
        self.assertEqual(evidence['ssh_exit_code'], 255)
        self.assertEqual(evidence['last_state']['sequence'], 2)
        self.assertAlmostEqual(evidence['watch_gap_ms'], 610)
        self.assertTrue(evidence['end_request_sent'])
        self.assertNotIn(self.session.token, json.dumps(evidence))
        self.assertEqual(self.saved(), self.original)

    def test_source_transfer_defers_end_without_locking_snapshot_or_duplicate_control(self):
        self.session.source_transmitting = True
        result = self.session.end_action({'generation': 7})
        self.assertEqual(result['session']['status'], 'stopping')
        self.assertIsNone(result['feedback'])
        self.assertFalse(self.session.end_request_attempted)
        self.assertEqual(self.control_records, [])
        self.session.fault('source transfer interrupted')
        self.session.source_finished(6, self.session.process)
        self.assertTrue(self.session.source_transmitting)
        self.session.source_finished(7, self.session.process)
        self.session.source_finished(7, self.session.process)
        self.assertEqual(self.control_records, [(777, host.END_FEEDBACK, .2)])
        self.assertEqual(self.session.status, 'fault')

    def test_raw_fault_and_closed_are_saved_before_generation_rejection(self):
        for kind in ('fault', 'closed'):
            line = (json.dumps(dict(kind=kind, generation=6, reason='raw evidence')) + '\n').encode()
            with self.assertRaises(ValueError):
                host.persist_reader_record(self.session, self.directory, line, False)
            records = list(self.directory.glob(kind + '-*.json'))
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0].read_bytes(), line)

    def test_manual_recovery_claim_rotates_identity_keeps_full_saved_config_and_one_attempt(self):
        saved = copy.deepcopy(self.original)
        saved['imu'].update(axes=['-y', 'x', 'z'], axes_confirmed=True, calibrated=True,
                            bias_dps=[.1, .2, .3])
        self.session.config = saved
        old_process = self.session.process
        old_process.returncode = 255
        self.session.connection_done.set()
        self.session.fault('previous interruption')
        previous_token = self.session.token
        calls, workers = [], []

        class DeferredThread:
            def __init__(self, *, target, daemon):
                workers.append(target)
            def start(self):
                pass

        self.session.recovery_launcher = lambda session, generation, path: calls.append((generation, path))
        self.assertTrue(self.session.snapshot()['session']['recovery_allowed'])
        with mock.patch.object(host.threading, 'Thread', DeferredThread):
            result = self.session.recover_action({'generation': 7})
        new_generation = self.session.generation
        self.assertNotEqual(new_generation, 7)
        self.assertNotEqual(self.session.token, previous_token)
        self.assertTrue(result['session']['recovery_pending'])
        self.assertEqual(self.session.config, saved)
        self.assertEqual(self.session.original_joints, self.original['joints'])
        self.assertEqual(self.session.wizard['stage'], 'idle')
        self.assertFalse(self.session.current_attempt(7, old_process))
        self.assertFalse(self.session.connection_done.is_set())
        self.assertEqual(calls, [])
        for body in ({'generation': 7}, {'generation': new_generation}):
            with self.assertRaises(ValueError):
                self.session.recover_action(body)
        workers[0]()
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], new_generation)
        self.assertTrue(calls[0][1].name.startswith('feedback-attempt-'))
        self.assertFalse(self.session.recovery_pending)
        self.assertEqual(self.session.config, saved)

    def test_recovery_requires_old_workers_done_and_child_exit_and_exact_generation(self):
        self.session.recovery_launcher = mock.Mock()
        self.session.fault('old fault')
        for done, exit_code in ((False, 255), (True, None)):
            self.session.process.returncode = exit_code
            if done:
                self.session.connection_done.set()
            with self.assertRaises(ValueError):
                self.session.recover_action({'generation': 7})
        self.session.process.returncode = 255
        for body in ({'generation': 6}, {'generation': True}, {'generation': 7, 'extra': True}):
            with self.assertRaises(ValueError):
                self.session.recover_action(body)
        self.session.recovery_launcher.assert_not_called()

    def test_failed_recovery_guard_is_specific_without_retries_or_config_loss(self):
        self.session.process.returncode = 255
        self.session.connection_done.set()
        self.session.fault('old fault')
        calls, workers = [], []
        def launcher(*_args):
            calls.append(1)
            raise ValueError('Early kernel history unavailable')
        class DeferredThread:
            def __init__(self, *, target, daemon):
                workers.append(target)
            def start(self):
                pass
        self.session.recovery_launcher = launcher
        with mock.patch.object(host.threading, 'Thread', DeferredThread):
            self.session.recover_action({'generation': 7})
        workers[0]()
        self.assertEqual(calls, [1])
        self.assertIn('早期日志', self.session.error)
        self.assertEqual(self.session.error_details, 'Early kernel history unavailable')
        self.assertTrue(self.session.connection_done.is_set())
        self.assertFalse(self.session.snapshot()['session']['recovery_allowed'])
        self.assertIn('需重新核对', self.session.snapshot()['session']['recovery_blocked_reason'])
        self.assertEqual(self.session.config, self.original)

    def test_fatal_guard_after_timeout_latches_recheck_without_erasing_original_fault(self):
        self.session.recovery_launcher = mock.Mock()
        self.session.fault('feedback timeout')
        self.session.process.returncode = 20
        self.session.connection_done.set()
        self.assertTrue(self.session.snapshot()['session']['recovery_allowed'])
        detail = 'New kernel fault; classifier result requires source review: precise new event'
        self.session.accept(dict(kind='fault', generation=7, reason=detail))
        self.assertEqual(self.session.error, 'feedback timeout')
        snapshot = self.session.snapshot()['session']
        self.assertFalse(snapshot['recovery_allowed'])
        self.assertEqual(snapshot['recovery_recheck']['technical_reason'], detail)
        with self.assertRaisesRegex(ValueError, '需重新核对'):
            self.session.recover_action({'generation': 7})
        self.session.recovery_launcher.assert_not_called()

    def test_kernel_identity_fatal_guards_block_but_transient_routes_and_holders_do_not(self):
        fatal = ('Retained kernel history changed or lost its reviewed prefix',
                 'Early kernel history is unavailable',
                 'Reviewed baseline faults changed or are incomplete',
                 'Reviewed boot/root/card/model identity changed')
        for detail in fatal:
            self.assertIsNotNone(host.recovery_recheck_error(detail))
        for detail in ('X5 USB 闪联路由不可用', 'UART already has an open holder',
                       'Another UART holder appeared while opening', 'feedback timeout'):
            self.assertIsNone(host.recovery_recheck_error(detail))

    def test_paused_initialization_never_calls_launcher_or_invents_closure(self):
        launcher = mock.Mock(side_effect=AssertionError('Paused hardware launch forbidden'))
        session = host.CalibrationSession(self.original, self.directory, 9, self.clock,
                                          recovery_launcher=launcher)
        session.start_initial(paused=True)
        launcher.assert_not_called()
        self.assertEqual(session.status, 'fault')
        self.assertTrue(session.connection_done.is_set())
        self.assertIsNone(session.process)
        self.assertIsNone(session.closed_proof)
        self.assertTrue(session.snapshot()['session']['recovery_allowed'])
        self.assertEqual(session.config, self.original)

    def test_cancel_from_previous_generation_cannot_clear_new_collection(self):
        self.action('level')
        with self.assertRaises(ValueError):
            self.action('cancel', generation=6)
        self.assertTrue(self.session.wizard['busy'])

    def test_actual_kernel_guard_strings_have_chinese_text_and_technical_detail(self):
        for detail in ('Early kernel history is unavailable',
                       'Retained kernel history changed or lost its reviewed prefix'):
            text, technical = host.fault_text(detail)
            self.assertIn('早期日志', text)
            self.assertEqual(technical, detail)

    def test_handshake_timeout_distinguishes_esp32_from_x5_network(self):
        detail = 'ESP32 handshake response timed out'
        text, technical = host.fault_text(detail)
        self.assertIn('X5 已连接', text)
        self.assertIn('ESP32 未回应通信握手', text)
        self.assertEqual(technical, detail)

    def test_recovery_pure_handler_returns_snapshot_with_rotated_token(self):
        self.session.process.returncode = 255
        self.session.connection_done.set()
        self.session.fault('old fault')
        self.session.recovery_launcher = mock.Mock()
        instance = host.handler_for(self.session, 8767).__new__(host.handler_for(self.session, 8767))
        instance.path = '/api/recover-feedback'
        instance.headers = Message()
        body = b'{"generation":7}'
        for key, value in {'Host': '127.0.0.1:8767', 'Origin': 'http://127.0.0.1:8767',
                           'X-CSRF-Token': self.session.token, 'Content-Type': 'application/json',
                           'Content-Length': str(len(body))}.items():
            instance.headers[key] = value
        instance.rfile = io.BytesIO(body)
        replies = []
        instance.reply = lambda code, value: replies.append((code, value))
        with mock.patch.object(host.threading, 'Thread'):
            instance.do_POST()
        self.assertEqual(replies[0][0], 200)
        self.assertEqual(replies[0][1]['token'], self.session.token)
        self.assertNotEqual(replies[0][1]['session']['generation'], 7)

    def enable_saved_imu_for_joints(self):
        self.session.config['imu'] = dict(axes=['-z', '-x', 'y'], axes_confirmed=True,
                                         calibrated=True, bias_dps=[.1127, -.43645, .243775])
        self.session.config['mechanical_reference_confirmed'] = True
        host.atomic_calibration(self.directory, self.session.config)
        self.joint_feed(self.session.config['joints'][0]['zero'])

    def joint_feed(self, raw, sid=10, advance=20000):
        self.sample = (self.sample + advance) & 0xffffffff
        self.clock.now += max(20000, advance) / 1000000
        self.sequence += 1
        current = self.state([0., 0., 9.81])
        current.update(online_mask=32767, last_error=0,
                       positions=[joint['zero'] for joint in self.session.config['joints']])
        current['positions'][host.joint_calibration.IDS.index(sid)] = raw
        self.session.accept(self.event(current))

    def joint_body(self, action, sid=10, **changes):
        body = dict(generation=7, action=action, id=sid,
                    support_confirmed=True, positive_confirmed=True)
        body.update(changes)
        return body

    def joint_capture(self, action, raw, sid=10):
        self.joint_feed(raw, sid)
        self.session.joint_action(self.joint_body(action, sid))
        for _ in range(20):
            self.joint_feed(raw, sid)
        self.assertFalse(self.session.joint_wizard.value['busy'])
        self.assertIsNone(self.session.joint_wizard.value['error'])

    def prepare_joint_save(self):
        self.enable_saved_imu_for_joints()
        zero = self.session.config['joints'][0]['zero']
        for action, raw in (('direction-start', zero), ('direction-end', zero + 50),
                            ('range-a', zero - 400), ('range-b', zero + 400)):
            self.joint_capture(action, raw)
        self.assertTrue(self.session.joint_wizard.value['ready_to_save'])

    def prepare_direction_reuse(self):
        self.prepare_joint_save()
        self.session.joint_action(self.joint_body('save'))
        zero = self.session.config['joints'][0]['zero']
        self.joint_capture('direction-start', zero)
        self.joint_capture('direction-end', zero - 50)

    def test_direction_only_save_reuses_range_and_is_passive_atomic_selected_joint_update(self):
        self.prepare_direction_reuse()
        previous = copy.deepcopy(self.session.config)
        self.assertTrue(self.session.snapshot()['joint_wizard']['direction_reuse_allowed'])
        result = self.session.joint_action(self.joint_body('save-direction'))
        expected = copy.deepcopy(previous)
        expected['joints'][0]['sign'] = -1
        self.assertEqual(self.saved(), expected)
        self.assertEqual(self.session.config, expected)
        self.assertEqual(result['joint_wizard']['stage'], 'complete')
        self.assertIn('沿用已保存安全范围', result['joint_wizard']['message'])
        self.assertEqual(self.control_records, [])

    def test_direction_only_save_requires_fresh_disarmed_feedback_and_explicit_confirmations(self):
        self.prepare_direction_reuse()
        previous = copy.deepcopy(self.session.config)
        for changes in (dict(positive_confirmed=False), dict(support_confirmed=False),
                        dict(generation=6), dict(id=11)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.session.joint_action(self.joint_body('save-direction', **changes))
        for field, value in (('torque_mask', 1), ('armed', True), ('fault_mask', 1),
                             ('arm_cookie', 123), ('online_mask', 32766), ('imu_ok', False)):
            old_value = self.session.feedback[field]
            self.session.feedback[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.session.joint_action(self.joint_body('save-direction'))
            self.session.feedback[field] = old_value
        self.clock.now += .6
        with self.assertRaises(ValueError):
            self.session.joint_action(self.joint_body('save-direction'))
        self.assertEqual(self.saved(), previous)
        self.assertEqual(self.control_records, [])

    def test_direction_only_save_disk_failure_keeps_entire_previous_calibration(self):
        self.prepare_direction_reuse()
        previous = copy.deepcopy(self.session.config)
        with mock.patch.object(host, 'atomic_calibration', side_effect=OSError('test disk failure')):
            with self.assertRaisesRegex(ValueError, '旧校准保持不变'):
                self.session.joint_action(self.joint_body('save-direction'))
        self.assertEqual(self.saved(), previous)
        self.assertEqual(self.session.config, previous)
        self.assertEqual(self.control_records, [])

    def test_joint_save_is_atomic_preserves_zeros_imu_reference_and_does_not_end_feedback(self):
        self.prepare_joint_save()
        previous = copy.deepcopy(self.session.config)
        result = self.session.joint_action(self.joint_body('save'))
        saved = self.saved()
        self.assertEqual(saved, self.session.config)
        self.assertEqual(saved['imu'], previous['imu'])
        self.assertTrue(saved['mechanical_reference_confirmed'])
        self.assertEqual(saved['joints'][1:], previous['joints'][1:])
        self.assertEqual(host.joint_calibration.zero_signature(saved), self.session.original_zeros)
        self.assertEqual(saved['joints'][0], dict(previous['joints'][0], sign=1,
                         direction_confirmed=True, min=612, max=1388))
        self.assertEqual(result['joint_wizard']['stage'], 'complete')
        self.assertEqual(result['joint_wizard']['completed_count'], 1)
        self.assertEqual(self.session.status, 'active')
        self.assertIsNone(self.session.end_reason)
        self.assertEqual(self.control_records, [])

    def test_joint_save_failure_keeps_entire_previous_file_and_memory(self):
        self.prepare_joint_save()
        previous = copy.deepcopy(self.session.config)
        with mock.patch.object(host, 'atomic_calibration', side_effect=OSError('disk refused')):
            with self.assertRaisesRegex(ValueError, '旧校准保持不变'):
                self.session.joint_action(self.joint_body('save'))
        self.assertEqual(self.saved(), previous)
        self.assertEqual(self.session.config, previous)
        self.assertTrue(self.session.joint_wizard.value['ready_to_save'])
        self.assertEqual(self.control_records, [])

    def test_joint_actions_require_saved_imu_fresh_full_online_actual_off_and_confirmation(self):
        with self.assertRaisesRegex(ValueError, 'IMU'):
            self.session.joint_action(self.joint_body('direction-start'))
        self.enable_saved_imu_for_joints()
        valid = copy.deepcopy(self.session.feedback)
        for fields in (dict(online_mask=0), dict(online_mask=1), dict(torque_mask=1),
                       dict(armed=True), dict(arm_cookie=1), dict(fault_mask=1),
                       dict(imu_ok=False), dict(last_error=1)):
            self.session.feedback = dict(valid, **fields)
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.session.joint_action(self.joint_body('direction-start'))
        self.session.feedback = valid
        with self.assertRaisesRegex(ValueError, '支撑'):
            self.session.joint_action(self.joint_body('direction-start', support_confirmed=False))
        self.clock.now += .501
        with self.assertRaisesRegex(ValueError, '过期'):
            self.session.joint_action(self.joint_body('direction-start'))

    def test_joint_endpoint_and_save_cannot_infer_human_positive_confirmation(self):
        self.enable_saved_imu_for_joints()
        zero = self.session.config['joints'][0]['zero']
        self.joint_capture('direction-start', zero)
        with self.assertRaisesRegex(ValueError, '正方向'):
            self.session.joint_action(self.joint_body('direction-end', positive_confirmed=False))
        self.joint_capture('direction-end', zero + 50)
        self.joint_capture('range-a', zero - 400)
        self.joint_capture('range-b', zero + 400)
        with self.assertRaisesRegex(ValueError, '正方向'):
            self.session.joint_action(self.joint_body('save', positive_confirmed=False))
        self.assertFalse(self.saved()['joints'][0]['direction_confirmed'])

    def test_joint_cancel_end_fault_and_new_generation_discard_only_unsaved_draft(self):
        for action in ('cancel', 'end', 'fault'):
            with self.subTest(action=action):
                self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock)
                self.session.process = FakeProcess()
                self.ready()
                self.feed([0., 0., 9.81])
                self.enable_saved_imu_for_joints()
                previous = copy.deepcopy(self.session.config)
                self.session.joint_action(self.joint_body('direction-start'))
                if action == 'cancel':
                    self.session.joint_action(self.joint_body('cancel', support_confirmed=False,
                                                              positive_confirmed=False))
                elif action == 'end':
                    self.session.end_action({'generation': 7})
                else:
                    self.session.fault('disconnected')
                self.assertFalse(self.session.joint_wizard.value['busy'])
                self.assertIsNone(self.session.joint_wizard.value['id'])
                self.assertEqual(self.session.config, previous)
        self.session.recovery_launcher = mock.Mock()
        self.session.process.returncode = 255
        self.session.connection_done.set()
        with mock.patch.object(host.threading, 'Thread'):
            self.session.recover_action({'generation': 7})
        self.assertEqual(self.session.joint_wizard.value['stage'], 'idle')
        with self.assertRaises(ValueError):
            self.session.joint_action(self.joint_body('cancel'))

    def test_joint_cross_zero_between_clicks_clears_draft_and_keeps_saved_config(self):
        self.enable_saved_imu_for_joints()
        self.joint_capture('direction-start', 4000)
        previous = copy.deepcopy(self.session.config)
        self.joint_feed(2)
        self.assertEqual(self.session.joint_wizard.value['stage'], 'failed')
        self.assertIn('跨零点', self.session.joint_wizard.value['error'])
        self.assertIsNone(self.session.joint_wizard.value['start_position'])
        self.assertEqual(self.session.config, previous)
        self.assertEqual(self.control_records, [])

    def test_joint_api_fields_id_generation_are_strict_and_handler_has_existing_origin_gate(self):
        self.enable_saved_imu_for_joints()
        valid = self.joint_body('direction-start')
        for changes in (dict(extra=True), dict(id=True), dict(id=99), dict(generation=True),
                        dict(generation=6), dict(action='TARGET'), dict(support_confirmed='yes')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.session.joint_action(dict(valid, **changes))
        handler = host.handler_for(self.session, 8767)
        instance = handler.__new__(handler)
        instance.path = '/api/joint-calibration'
        instance.headers = Message()
        payload = json.dumps(valid).encode()
        headers = dict(Host='127.0.0.1:8767', Origin='http://127.0.0.1:8767',
                       **{'X-CSRF-Token': self.session.token, 'Content-Type': 'application/json',
                          'Content-Length': str(len(payload))})
        for key, value in headers.items():
            instance.headers[key] = value
        instance.rfile = io.BytesIO(payload)
        replies = []
        instance.reply = lambda code, value: replies.append((code, value))
        instance.do_POST()
        self.assertEqual(replies[0][0], 200)
        self.assertTrue(replies[0][1]['joint_wizard']['busy'])
        instance.headers.replace_header('Origin', 'http://evil.invalid')
        instance.rfile = io.BytesIO(payload)
        instance.do_POST()
        self.assertEqual(replies[-1][0], 403)

    def test_saved_imu_paused_page_shows_saved_closed_without_fake_uart_proof(self):
        self.enable_saved_imu_for_joints()
        launcher = mock.Mock(side_effect=AssertionError('Paused hardware forbidden'))
        session = host.CalibrationSession(self.session.config, self.directory, 8, self.clock,
                                          recovery_launcher=launcher)
        session.start_initial(paused=True)
        snapshot = session.snapshot()
        launcher.assert_not_called()
        self.assertEqual(snapshot['session']['status'], 'closed')
        self.assertIsNone(snapshot['session']['error'])
        self.assertIsNone(snapshot['session']['closed_proof'])
        self.assertTrue(snapshot['calibration']['imu']['calibrated'])
        self.assertTrue(snapshot['calibration']['mechanical_reference_confirmed'])
        self.assertEqual(snapshot['wizard']['stage'], 'complete')

    def test_imu_finish_can_preserve_previously_saved_direction_ranges(self):
        self.prepare_joint_save()
        self.session.joint_action(self.joint_body('save'))
        joints = copy.deepcopy(self.session.config['joints'])
        self.feed([0., 0., 9.81])  # Pure battery-off state for IMU collection.
        self.prepare_finish()
        self.collect('finish', [0., 0., 9.81])
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.saved()['joints'], joints)
        self.assertTrue(self.saved()['mechanical_reference_confirmed'])

    def diagnostic_scalars(self, **changes):
        value = dict(sequence=2, sample_us=1234567, imu_us=1234000,
                     online_mask=32767, fault_mask=0, torque_mask=0,
                     armed=False, arm_cookie=0, imu_ok=True, last_error=0)
        value.update(changes)
        return value

    def test_unsafe_fault_has_verified_id_diagnostics_never_live_data_and_blocks_recovery(self):
        self.enable_saved_imu_for_joints()
        previous = copy.deepcopy(self.session.config)
        scalars = self.diagnostic_scalars(fault_mask=(1 << 0) | (1 << 10) | (1 << 14),
                                           torque_mask=(1 << 5) | (1 << 13))
        self.session.accept(dict(kind='fault', generation=7, reason=host.UNSAFE_FEEDBACK_REASON,
                                 query=dict(last_feedback=scalars)))
        snapshot = self.session.snapshot()
        self.assertEqual(snapshot['session']['fault_ids'], [10, 30, 34])
        self.assertEqual(snapshot['session']['torque_ids'], [20, 33])
        self.assertIn('故障标志 ID：10、30、34', self.session.error)
        self.assertIn('扭矩仍开启的舵机 ID：20、33', self.session.error)
        self.assertNotIn('过载', self.session.error)
        self.assertEqual(self.session.error_details, host.UNSAFE_FEEDBACK_REASON)
        self.assertIsNone(snapshot['feedback'])
        self.assertEqual(snapshot['calibration'], previous)
        self.assertEqual(self.saved(), previous)
        self.session.recovery_launcher = mock.Mock()
        self.session.process.returncode = 20
        self.session.connection_done.set()
        self.assertFalse(self.session.snapshot()['session']['recovery_allowed'])
        with self.assertRaisesRegex(ValueError, '实际舵机反馈'):
            self.session.recover_action({'generation': 7})
        self.session.recovery_launcher.assert_not_called()
        scalars['fault_mask'] = 0
        snapshot['session']['fault_feedback']['fault_mask'] = 0
        self.assertEqual(self.session.fault_feedback['fault_mask'], (1 << 0) | (1 << 10) | (1 << 14))
        self.session.write_fault_evidence(self.directory, self.session.process)
        evidence = json.loads((self.directory / 'fault-evidence.json').read_text())
        self.assertEqual(evidence['fault_ids'], [10, 30, 34])
        self.assertEqual(evidence['torque_ids'], [20, 33])
        self.assertEqual(evidence['fault_feedback']['sequence'], 2)

    def test_old_unsafe_report_without_trigger_values_stays_unknown_and_held(self):
        self.session.accept(dict(kind='fault', generation=7, reason=host.UNSAFE_FEEDBACK_REASON))
        self.assertIn('触发值未保存', self.session.error)
        self.assertIn('暂不能区分', self.session.error)
        self.assertIsNone(self.session.fault_feedback)
        self.assertIsNone(self.session.snapshot()['session']['fault_ids'])
        self.assertIsNotNone(self.session.recovery_recheck)
        self.assertFalse(self.session.snapshot()['session']['recovery_allowed'])

    def test_armed_and_cookie_decoder_reasons_are_chinese_and_held(self):
        cases = [('Firmware reports ARMED; no further command will be sent',
                  dict(armed=True), '控制器报告已使能'),
                 ('Disarmed firmware reports a nonzero arm cookie',
                  dict(arm_cookie=17), '控制器解锁标记异常（非零）'),
                 ('Unsafe armed/cookie/fault/torque feedback: detailed flags',
                  dict(torque_mask=1), '扭矩仍开启的舵机')]
        for reason, changes, text in cases:
            value = host.validated_fault_feedback(dict(last_feedback=self.diagnostic_scalars(**changes)))
            explanation, technical = host.fault_text(reason, value)
            self.assertIn(text, explanation)
            self.assertEqual(technical, reason)
            self.assertIsNotNone(host.recovery_recheck_error(reason))

    def test_diagnostic_whitelist_bounds_reject_invalid_scalars_without_guessing(self):
        changes = (dict(sequence=True), dict(sequence=1), dict(sequence=0x100000000),
                   dict(sample_us=-1), dict(imu_us=1.), dict(arm_cookie=float('nan')),
                   dict(online_mask=32768), dict(fault_mask=-1), dict(torque_mask=True),
                   dict(armed=1), dict(imu_ok=1), dict(last_error=6), dict(positions=[0] * 15))
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                host.validated_fault_feedback(dict(last_feedback=self.diagnostic_scalars(**change)))
        incomplete = self.diagnostic_scalars()
        del incomplete['imu_ok']
        with self.assertRaises(ValueError):
            host.validated_fault_feedback(dict(last_feedback=incomplete))
        self.session.accept(dict(kind='fault', generation=7, reason=host.UNSAFE_FEEDBACK_REASON,
                                 query=dict(last_feedback=self.diagnostic_scalars(armed=1))))
        self.assertIsNone(self.session.fault_feedback)
        self.assertEqual(self.session.fault_feedback_invalid, 'invalid_host_diagnostic_fields')
        self.assertIn('暂不能区分', self.session.error)

    def test_invalid_reader_scalar_marker_is_kept_and_close_can_fill_diagnostics_without_normalizing_fault(self):
        self.session.accept(dict(kind='fault', generation=7, reason=host.UNSAFE_FEEDBACK_REASON,
                                 query=dict(last_feedback_invalid='invalid_scalar_fields')))
        self.assertEqual(self.session.fault_feedback_invalid, 'invalid_scalar_fields')
        original_reason = self.session.error_details
        proof = dict(kind='closed', generation=7, normal=False, serial_closed=True,
                     reason=host.UNSAFE_FEEDBACK_REASON,
                     query=dict(last_feedback=self.diagnostic_scalars(fault_mask=1)))
        self.session.accept(proof)
        self.session.confirm_closed(0)
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.error_details, original_reason)
        self.assertIn('故障标志 ID：10', self.session.error)
        self.assertEqual(self.session.closed_proof, proof)
        self.assertFalse(self.session.snapshot()['session']['recovery_allowed'])

    def test_later_unsafe_report_latches_but_preserves_original_timeout_reason(self):
        self.session.fault('local data timeout')
        self.session.accept(dict(kind='fault', generation=7, reason=host.UNSAFE_FEEDBACK_REASON,
                                 query=dict(last_feedback=self.diagnostic_scalars(torque_mask=1))))
        self.assertEqual(self.session.error, 'local data timeout')
        self.assertEqual(self.session.error_details, 'local data timeout')
        self.assertEqual(self.session.fault_feedback['torque_mask'], 1)
        self.assertIsNotNone(self.session.recovery_recheck)

    def test_explicit_paused_unsafe_hold_has_no_hardware_or_fabricated_proof(self):
        self.enable_saved_imu_for_joints()
        previous = copy.deepcopy(self.session.config)
        launcher = mock.Mock(side_effect=AssertionError('Held page may not touch hardware'))
        session = host.CalibrationSession(previous, self.directory, 19, self.clock,
                                          recovery_launcher=launcher)
        with self.assertRaises(ValueError):
            session.start_initial(hold_unsafe=True)
        session.start_initial(paused=True, hold_unsafe=True)
        launcher.assert_not_called()
        snapshot = session.snapshot()
        self.assertEqual(snapshot['session']['status'], 'fault')
        self.assertIn('触发值未保存', snapshot['session']['error'])
        self.assertEqual(session.error_details, host.UNSAFE_FEEDBACK_REASON)
        self.assertIsNone(session.process)
        self.assertIsNone(session.closed_proof)
        self.assertIsNone(session.fault_feedback)
        self.assertIsNone(session.fault_record)
        self.assertFalse(session.end_request_attempted)
        self.assertTrue(session.connection_done.is_set())
        self.assertFalse(snapshot['session']['recovery_allowed'])
        self.assertEqual(session.config, previous)
        self.assertEqual(snapshot['wizard']['stage'], 'complete')

    def test_cli_unsafe_hold_requires_paused_and_explicit_resume_before_any_service(self):
        invalid = [[], ['--start-paused'], ['--resume-run', str(self.directory)]]
        for options in invalid:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), \
                    mock.patch.object(sys, 'argv', ['feedback_calibration_server',
                        '--run-dir', str(self.directory / 'unused'), '--hold-unsafe-feedback', *options]), \
                    mock.patch.object(host, 'ThreadingHTTPServer',
                                      side_effect=AssertionError('No service permitted in pure test')):
                with self.assertRaises(SystemExit) as error:
                    host.main()
                self.assertEqual(error.exception.code, 2)


class MaintenanceIntegrationTests(unittest.TestCase):
    """The unified page cannot bypass held faults or share an unclosed UART."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.clock = Clock()
        self.original = config()
        self.original['imu'].update(axes=['-z', '-x', 'y'], axes_confirmed=True,
                                    calibrated=True, bias_dps=[.1, -.4, .2])
        self.original['joints'][0].update(sign=-1, min=900, max=1200,
                                          direction_confirmed=True)
        host.atomic_calibration(self.directory, self.original)
        self.saved_bytes = (self.directory / 'calibration-latest.json').read_bytes()
        self.launcher = mock.Mock(side_effect=AssertionError('Hardware recovery forbidden'))
        self.factory = mock.Mock(side_effect=AssertionError('Hardware maintenance forbidden'))
        self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock,
                                               recovery_launcher=self.launcher)
        self.session.start_initial(paused=True)
        self.session.configure_maintenance(self.factory)
        self.forbid_process = mock.patch.object(host.subprocess, 'Popen',
                                                side_effect=AssertionError('Hardware process forbidden'))
        self.forbid_process.start()

    def tearDown(self):
        self.forbid_process.stop()
        self.temporary.cleanup()

    def assert_saved_unchanged(self):
        self.assertEqual(self.session.config, self.original)
        self.assertEqual((self.directory / 'calibration-latest.json').read_bytes(), self.saved_bytes)

    def enter(self, **changes):
        return self.session.maintenance_action(dict(action='enter', generation=7, **changes))

    def post(self, body, **changes):
        handler = host.handler_for(self.session, 8767).__new__(host.handler_for(self.session, 8767))
        raw = json.dumps(body).encode()
        headers = {'Host': '127.0.0.1:8767', 'Origin': 'http://127.0.0.1:8767',
                   'X-CSRF-Token': self.session.token, 'Content-Length': str(len(raw)),
                   'Content-Type': 'application/json'}
        headers.update(changes)
        handler.headers = Message()
        for key, value in headers.items(): handler.headers[key] = value
        handler.path = '/api/servo-maintenance'
        handler.rfile = io.BytesIO(raw)
        replies = []
        handler.reply = lambda code, value: replies.append((code, value))
        handler.do_POST()
        return replies[0]

    def test_configure_and_status_are_lazy_without_factory_or_recovery(self):
        for _ in range(3):
            snapshot = self.session.snapshot()
            self.assertEqual(snapshot['maintenance']['phase'], 'closed')
            self.assertFalse(snapshot['maintenance']['owns_connection'])
            self.assertIsNone(snapshot['maintenance']['blocked_reason'])
        self.factory.assert_not_called()
        self.launcher.assert_not_called()
        self.assert_saved_unchanged()

    def test_held_unsafe_feedback_refuses_enter_before_factory(self):
        self.session.start_initial(paused=True, hold_unsafe=True)
        with self.assertRaises(ValueError): self.enter()
        self.factory.assert_not_called()
        self.assertFalse(self.session.maintenance.owns_connection)
        self.assertIsNotNone(self.session.snapshot()['maintenance']['blocked_reason'])
        self.assert_saved_unchanged()

    def test_active_feedback_and_recovery_pending_refuse_enter_before_factory(self):
        for status in ('waiting', 'active', 'stopping'):
            with self.subTest(status=status):
                self.session.status = status
                with self.assertRaises(ValueError): self.enter()
        self.session.status = 'closed'
        self.session.recovery_pending = True
        with self.assertRaises(ValueError): self.enter()
        self.factory.assert_not_called()
        self.assert_saved_unchanged()

    def test_feedback_missing_any_close_proof_refuses_enter_before_factory(self):
        self.session.process = FakeProcess()
        self.session.process.returncode = 0
        valid = dict(normal=True, serial_closed=True)
        cases = [(None, 0), ({'normal': True}, 0), ({'normal': False, 'serial_closed': True}, 0),
                 ({'normal': True, 'serial_closed': False}, 0), (valid, None), (valid, 20)]
        for proof, exit_code in cases:
            with self.subTest(proof=proof, exit_code=exit_code):
                self.session.closed_proof = copy.deepcopy(proof)
                self.session.process.returncode = exit_code
                with self.assertRaisesRegex(ValueError, '关闭证明'): self.enter()
        self.session.closed_proof = valid
        self.session.process.returncode = 0
        self.assertIsNone(self.session.maintenance_blocked_reason())
        self.session.connection_done.clear()
        with self.assertRaisesRegex(ValueError, '收尾'): self.enter()
        self.session.connection_done.set()
        self.session.source_transmitting = True
        with self.assertRaisesRegex(ValueError, '收尾'): self.enter()
        self.factory.assert_not_called()
        self.assert_saved_unchanged()

    def test_unsaved_collection_prevents_switching_to_maintenance(self):
        self.session.collection = {'action': 'level'}
        with self.assertRaisesRegex(ValueError, '采样尚未结束'): self.enter()
        self.session.collection = None
        self.session.joint_wizard.value['busy'] = True
        with self.assertRaisesRegex(ValueError, '采样尚未结束'): self.enter()
        self.factory.assert_not_called()

    def test_generation_types_and_extra_fields_fail_before_controller_or_factory(self):
        with mock.patch.object(self.session.maintenance, 'action') as action:
            for body in ({'action': 'enter', 'generation': 6},
                         {'action': 'enter', 'generation': True},
                         {'action': 'enter'},
                         {'action': 'enter', 'generation': 7, 'force': True}):
                with self.subTest(body=body), self.assertRaises(ValueError):
                    self.session.maintenance_action(body)
            action.assert_not_called()
        self.factory.assert_not_called()
        self.assert_saved_unchanged()

    def test_generation_is_removed_before_controller_request(self):
        with mock.patch.object(self.session.maintenance, 'action') as action:
            self.session.maintenance_action({'action': 'read', 'id': 12, 'generation': 7})
            action.assert_called_once_with({'action': 'read', 'id': 12})
        self.factory.assert_not_called()

    def test_center_target_is_forwarded_once_without_changing_saved_zero_or_ranges(self):
        body = dict(action='center', generation=7, id=10, target=1050, support_confirmed=True)
        with mock.patch.object(self.session.maintenance, 'action') as action:
            code, snapshot = self.post(body)
            self.assertEqual(code, 200)
            action.assert_called_once_with(dict(action='center', id=10, target=1050,
                                                support_confirmed=True))
            self.assertEqual(snapshot['calibration'], self.original)
        self.factory.assert_not_called()
        self.assert_saved_unchanged()

    def test_center_target_does_not_allow_other_extra_fields(self):
        with mock.patch.object(self.session.maintenance, 'action') as action:
            code, result = self.post(dict(action='center', generation=7, id=10, target=1050,
                                          support_confirmed=True, speed=100))
            self.assertEqual(code, 409)
            self.assertIn('不支持的字段', result['error'])
            action.assert_not_called()
        for target in (True, '1050', -1, 4096):
            with self.subTest(target=target):
                code, _ = self.post(dict(action='center', generation=7, id=10, target=target,
                                        support_confirmed=True))
                self.assertEqual(code, 409)
        code, _ = self.post(dict(action='read', generation=7, id=10, target=1050))
        self.assertEqual(code, 409)
        self.factory.assert_not_called()
        self.assert_saved_unchanged()

    def test_closed_one_click_scan_routes_one_operation_without_separate_enter(self):
        self.assertEqual(self.session.status, 'closed')
        self.assertFalse(self.session.maintenance.owns_connection)
        with mock.patch.object(self.session.maintenance, 'action') as action:
            code, snapshot = self.post(dict(action='scan', generation=7))
            self.assertEqual(code, 200)
            action.assert_called_once_with(dict(action='scan'))
            self.assertEqual(snapshot['calibration'], self.original)
        self.factory.assert_not_called()
        self.launcher.assert_not_called()
        self.assert_saved_unchanged()

    def test_one_click_scan_does_not_bypass_a_held_hardware_fault(self):
        self.session.start_initial(paused=True, hold_unsafe=True)
        code, _ = self.post(dict(action='scan', generation=7))
        self.assertEqual(code, 409)
        self.factory.assert_not_called()
        self.assertFalse(self.session.maintenance.owns_connection)
        self.assert_saved_unchanged()

    def test_maintenance_owner_blocks_recover_including_unproven_close(self):
        for phase in ('connecting', 'active', 'closing', 'fault'):
            with self.subTest(phase=phase):
                self.session.maintenance._state.update(phase=phase, owns_connection=True,
                                                      connected=False, close_proof=None)
                with self.assertRaisesRegex(ValueError, '维护占用连接'):
                    self.session.recover_action({'generation': 7})
                self.assertFalse(self.session.snapshot()['session']['recovery_allowed'])
        self.launcher.assert_not_called()
        self.assert_saved_unchanged()

    def test_identity_callback_persists_mapping_hold_and_preserves_calibration(self):
        self.session.maintenance._identity_changed(12, 42)
        marker = self.directory / 'maintenance-identity-hold.json'
        first_bytes = marker.read_bytes()
        value = json.loads(first_bytes)
        self.assertEqual(value, dict(reason='servo_id_write_attempted', old_id=12, new_id=42,
            generation=7, calibration_preserved=True, requires_physical_mapping_review=True))
        self.assertEqual(stat.S_IMODE(marker.stat().st_mode), 0o600)
        self.session.maintenance._identity_changed(42, 43)
        self.assertEqual(marker.read_bytes(), first_bytes)
        self.assertTrue(self.session.snapshot()['maintenance']['identity_changed'])
        self.assertIsNone(self.session.maintenance_blocked_reason())
        with self.assertRaisesRegex(ValueError, '关节映射'):
            self.session.recover_action({'generation': 7})
        with mock.patch.object(self.session.maintenance, 'action') as action:
            with self.assertRaisesRegex(ValueError, '原关节映射'):
                self.session.maintenance_action(dict(action='center', id=12, generation=7,
                                                     support_confirmed=True))
            action.assert_not_called()
        self.factory.assert_not_called()
        self.launcher.assert_not_called()
        self.assert_saved_unchanged()

    def test_mapping_marker_survives_a_new_session(self):
        self.session.maintenance._identity_changed(12, 42)
        restored = host.CalibrationSession(self.original, self.directory, 8, self.clock,
                                          recovery_launcher=self.launcher)
        restored.start_initial(paused=True)
        restored.configure_maintenance(self.factory)
        self.assertTrue(restored.maintenance_mapping_changed)
        self.assertTrue(restored.snapshot()['maintenance']['identity_changed'])
        with self.assertRaisesRegex(ValueError, '关节映射'):
            restored.recover_action({'generation': 8})
        self.factory.assert_not_called()
        self.launcher.assert_not_called()
        self.assertEqual(restored.config, self.original)
        self.assert_saved_unchanged()

    def test_fault_callback_persists_hold_and_blocks_new_session(self):
        self.session.maintenance._fault_callback('voltage alarm')
        marker = self.directory / 'maintenance-fault-hold.json'
        first_bytes = marker.read_bytes()
        self.assertEqual(json.loads(first_bytes), dict(reason='voltage alarm', generation=7,
                                                       requires_feedback_review=True))
        self.assertEqual(stat.S_IMODE(marker.stat().st_mode), 0o600)
        self.session.maintenance._fault_callback('later error')
        self.assertEqual(marker.read_bytes(), first_bytes)
        with self.assertRaisesRegex(ValueError, '维护出现故障'): self.enter()
        with self.assertRaisesRegex(ValueError, '维护出现故障'):
            self.session.recover_action({'generation': 7})
        restored = host.CalibrationSession(self.original, self.directory, 8, self.clock,
                                          recovery_launcher=self.launcher)
        restored.start_initial(paused=True)
        restored.configure_maintenance(self.factory)
        self.assertTrue(restored.maintenance_fault_hold)
        with self.assertRaisesRegex(ValueError, '维护出现故障'):
            restored.maintenance_action(dict(action='enter', generation=8))
        self.factory.assert_not_called()
        self.launcher.assert_not_called()
        self.assert_saved_unchanged()

    def test_explicit_fault_review_archives_hold_only_after_all_servos_and_closure(self):
        from test_servo_maintenance import Transport, Clock as MaintenanceClock
        transport = Transport(MaintenanceClock())
        transport.servos = {sid: dict(position=2048, torque=0) for sid in list(host.joint_calibration.IDS)}
        self.factory.side_effect = lambda: transport
        self.session._maintenance_fault('Maintenance request rejected: ACK error 2')
        marker = self.directory / 'maintenance-fault-hold.json'
        held_bytes = marker.read_bytes()
        snapshot = self.session.snapshot()['maintenance']
        self.assertTrue(snapshot['fault_review_allowed'])
        self.assertTrue(snapshot['fault_hold'])
        self.session.maintenance_action(dict(action='review', generation=7))
        self.session.maintenance._thread.join(2)
        self.assertFalse(self.session.maintenance._thread.is_alive())
        self.assertIsNone(self.session.maintenance.snapshot()['error'])
        self.assertFalse(self.session.maintenance_fault_hold)
        self.assertFalse(marker.exists())
        archives = list(self.directory.glob('maintenance-fault-reviewed-*.hold.json'))
        self.assertEqual(len(archives), 1)
        self.assertEqual(archives[0].read_bytes(), held_bytes)
        audit = json.loads(archives[0].with_name(archives[0].name.replace('.hold.json', '.json')).read_text())
        self.assertEqual(audit['hold_sha256'], hashlib.sha256(held_bytes).hexdigest())
        self.assertEqual([row['id'] for row in audit['rows']], list(host.joint_calibration.IDS))
        self.assertFalse(audit['motion_attempted'])
        self.assert_saved_unchanged()
        restored = host.CalibrationSession(self.original, self.directory, 8, self.clock)
        restored.start_initial(paused=True); restored.configure_maintenance(self.factory)
        self.assertFalse(restored.maintenance_fault_hold)

    def test_fault_review_cannot_clear_hold_after_offline_torque_or_closure_failure(self):
        from test_servo_maintenance import Transport, Clock as MaintenanceClock
        for failure in ('offline', 'torque', 'closure'):
            with self.subTest(failure=failure):
                transport = Transport(MaintenanceClock())
                transport.servos = {sid: dict(position=2048, torque=0) for sid in list(host.joint_calibration.IDS)}
                if failure == 'offline': transport.servos.pop(31)
                if failure == 'torque': transport.servos[31]['torque'] = 1
                if failure == 'closure': transport.proof['serial_closed'] = False
                self.factory.side_effect = lambda: transport
                self.session._maintenance_fault('held')
                held_bytes = (self.directory / 'maintenance-fault-hold.json').read_bytes()
                self.session.maintenance_action(dict(action='review', generation=7))
                self.session.maintenance._thread.join(2)
                self.assertTrue(self.session.maintenance_fault_hold)
                self.assertEqual((self.directory / 'maintenance-fault-hold.json').read_bytes(), held_bytes)
                self.assertEqual(list(self.directory.glob('maintenance-fault-reviewed-*')), [])
                transport.proof['serial_closed'] = True
                self.session.maintenance.close()
                self.assert_saved_unchanged()

    def test_review_remains_blocked_for_mapping_or_unclosed_feedback_connection(self):
        self.session._maintenance_fault('held')
        self.session.maintenance_mapping_changed = True
        with self.assertRaisesRegex(ValueError, '编号'):
            self.session.maintenance_action(dict(action='review', generation=7))
        self.session.maintenance_mapping_changed = False
        self.session.connection_done.clear()
        with self.assertRaisesRegex(ValueError, '收尾'):
            self.session.maintenance_action(dict(action='review', generation=7))
        self.factory.assert_not_called()
        self.assertTrue(self.session.maintenance_fault_hold)

    def test_maintenance_http_reuses_same_origin_and_csrf_gates_without_socket(self):
        with mock.patch.object(self.session.maintenance, 'action') as action:
            for headers in ({'Origin': 'http://attacker.invalid'}, {'Host': 'attacker.invalid'},
                            {'X-CSRF-Token': 'wrong'}):
                code, _ = self.post(dict(action='enter', generation=7), **headers)
                self.assertEqual(code, 403)
            action.assert_not_called()
            code, _ = self.post(dict(action='enter', generation=6))
            self.assertEqual(code, 409)
            code, _ = self.post(dict(action='enter', generation=7, force=True))
            self.assertEqual(code, 409)
            action.assert_not_called()
            code, snapshot = self.post(dict(action='read', generation=7, id=12))
            self.assertEqual(code, 200)
            action.assert_called_once_with(dict(action='read', id=12))
            self.assertEqual(snapshot['calibration'], self.original)
        self.factory.assert_not_called()
        self.assert_saved_unchanged()


class HeldDiagnosticTests(unittest.TestCase):
    """Local historical files only; any subprocess attempt is a test failure."""
    def setUp(self):
        import feedback_fault_probe
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.root.chmod(0o700)
        self.directory = self.root / 'fault-details-prior'
        self.directory.mkdir(mode=0o700)
        self.path = self.directory / 'report.json'
        self.namespace = {}
        exec(compile(feedback_fault_probe.runtime_source(), '<synthetic-held-definitions>', 'exec'), self.namespace)
        self.report = self.make_report()
        self.root_patch = mock.patch.object(host, 'RECOVERY', self.root)
        self.root_patch.start()
        self.forbid_process = mock.patch.object(host.subprocess, 'Popen',
                                               side_effect=AssertionError('Historical tests cannot launch processes'))
        self.forbid_process.start()
        self.write_report(self.report)

    def tearDown(self):
        self.forbid_process.stop()
        self.root_patch.stop()
        self.temporary.cleanup()

    def make_report(self):
        header = self.namespace['FAULT_HEADER'].pack(1, 15, 1000000, 3, 0, 32767, 0, 32767, 32767, 0, 0)
        bank = self.namespace['FAULT_BANK'].pack(32767, 32767, 32767,
            *([1] * 15), *([1] * 15), *([2] * 15), *([999000] * 15))
        diagnostic = self.namespace['_decode_fault_details'](header + bank + bank)
        def query(kind):
            ready, closed = kind == 'ready', kind == 'closed'
            value = dict(request_types_attempted=[1] if ready else [1, 7],
                         reply_types_received=[129] if ready else [129, 135],
                         request_bytes_written=12 if ready else 24,
                         diagnostic_requests=0 if ready else 1,
                         diagnostic_records=1 if closed else 0, state_records=0,
                         serial_open_attempts=1, exclusive=True, serial_closed=closed,
                         board_files_written_by_reader=False, servo_motion_authorized=False)
            if not ready:
                value['last_diagnostic'] = copy.deepcopy(diagnostic)
            return value
        return dict(generation=17, query_completed=True, serial_closed=True, ssh_process_exited=True,
                    ssh_exit_code=0, error=None, fault_report=None, servo_motion_authorized=False,
                    board_files_written_by_reader=False, calibration_written=False,
                    ready_report=dict(kind='ready', generation=17, allowed_requests=['HELLO', 'FAULT_DETAILS'],
                                      diagnostic_only=True, query=query('ready')),
                    diagnostic_report=dict(kind='diagnostic', generation=17, sequence=2,
                                           diagnostic=diagnostic, query=query('diagnostic')),
                    remote_closed_report=dict(kind='closed', generation=17, normal=True, serial_closed=True,
                                              diagnostic_only=True, servo_motion_authorized=False,
                                              query=query('closed')))

    def write_report(self, report):
        raw = (json.dumps(report, ensure_ascii=False, allow_nan=False) + '\n').encode()
        self.path.write_bytes(raw)
        self.path.chmod(0o600)
        self.sha = hashlib.sha256(raw).hexdigest()
        return raw

    def load(self):
        return host.load_held_diagnostic_report(self.path, self.sha)

    def test_pinned_private_wire_report_produces_only_historical_voltage_summary(self):
        expected = self.path.read_bytes()
        value, raw = self.load()
        self.assertEqual(raw, expected)
        self.assertEqual(value['scope'], 'historical_diagnostic')
        self.assertEqual(value['source_sha256'], self.sha)
        self.assertEqual(value['voltage_alarm_ids'], list(host.joint_calibration.IDS))
        self.assertEqual(value['generation'], 17)
        self.assertEqual(value['sample_us'], 1000000)
        self.assertTrue(all(item['present_error'] == item['torque_error'] == 1 for item in value['device_errors']))
        self.assertIn('15个舵机均报 0x01 电压报警', value['message'])
        self.assertIn('不是实时读数', value['message'])
        self.assertNotIn('欠压', value['message'])
        self.assertNotIn('过压', value['message'])

    def test_invalid_sha_modes_file_links_and_parent_links_are_rejected(self):
        with self.assertRaises(ValueError):
            host.load_held_diagnostic_report(self.path, '0' * 64)
        with self.assertRaises(ValueError):
            host.load_held_diagnostic_report(self.path, 'not-a-digest')
        self.path.chmod(0o644)
        with self.assertRaises(ValueError):
            self.load()
        self.path.chmod(0o600)
        alias = self.directory / 'alias.json'
        alias.symlink_to(self.path)
        with self.assertRaises((ValueError, OSError)):
            host.load_held_diagnostic_report(alias, self.sha)
        directory_link = self.root / 'alias'
        directory_link.symlink_to(self.directory, target_is_directory=True)
        with self.assertRaises(ValueError):
            host.load_held_diagnostic_report(directory_link / 'report.json', self.sha)
        self.directory.chmod(0o755)
        with self.assertRaises(ValueError):
            self.load()
        self.directory.chmod(0o700)
        with self.assertRaises(ValueError):
            host.load_held_diagnostic_report(Path('report.json'), self.sha)

    def test_closure_scope_generation_and_exact_query_contract_are_required(self):
        cases = (
            ('query_completed', False), ('serial_closed', False), ('ssh_process_exited', False),
            ('ssh_exit_code', 20), ('ssh_exit_code', False), ('error', 'failure'),
            ('servo_motion_authorized', True), ('board_files_written_by_reader', True),
            ('calibration_written', True), ('fault_report', {}), ('generation', True))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                report = copy.deepcopy(self.report)
                report[key] = value
                self.write_report(report)
                with self.assertRaises(ValueError):
                    self.load()
        mutations = (
            ('ready_report', 'generation', 18), ('diagnostic_report', 'generation', 18),
            ('remote_closed_report', 'generation', 18), ('diagnostic_report', 'sequence', 3),
            ('remote_closed_report', 'normal', False), ('remote_closed_report', 'serial_closed', False),
            ('remote_closed_report', 'servo_motion_authorized', True))
        for name, key, value in mutations:
            with self.subTest(record=name, key=key):
                report = copy.deepcopy(self.report)
                report[name][key] = value
                self.write_report(report)
                with self.assertRaises(ValueError):
                    self.load()
        for name in ('ready_report', 'diagnostic_report', 'remote_closed_report'):
            for key, value in (('request_types_attempted', [1, 2]), ('reply_types_received', [129, 130]),
                               ('state_records', 1), ('board_files_written_by_reader', True),
                               ('servo_motion_authorized', True), ('diagnostic_records', 7)):
                with self.subTest(record=name, key=key):
                    report = copy.deepcopy(self.report)
                    report[name]['query'][key] = value
                    self.write_report(report)
                    with self.assertRaises(ValueError):
                        self.load()

    def test_roundtrip_rejects_unsafe_false_or_forged_derived_diagnostics(self):
        for name, value in (('unsafe_flags_present', False), ('motor_ids', [10] * 15),
                            ('motion_authorized', True), ('last_valid_age_is_freshness_proof', True)):
            with self.subTest(field=name):
                report = copy.deepcopy(self.report)
                report['diagnostic_report']['diagnostic'][name] = value
                self.write_report(report)
                with self.assertRaises(ValueError):
                    self.load()
        report = copy.deepcopy(self.report)
        report['remote_closed_report']['query']['last_diagnostic']['snapshot_us'] += 1
        self.write_report(report)
        with self.assertRaises(ValueError):
            self.load()

    def test_old_valid_error_on_failed_latest_attempt_is_not_a_voltage_id(self):
        report = copy.deepcopy(self.report)
        diagnostic = report['diagnostic_report']['diagnostic']
        present, torque = diagnostic['present_bank'], diagnostic['torque_bank']
        present['latest_found_mask'] &= ~1
        present['latest_attempt_error'][0] = 0
        present['latest_outcome'][0] = 6
        torque['latest_attempt_error'][0] = torque['last_valid_error'][0] = 0
        torque['latest_outcome'][0] = 1
        report['diagnostic_report']['query']['last_diagnostic'] = copy.deepcopy(diagnostic)
        report['remote_closed_report']['query']['last_diagnostic'] = copy.deepcopy(diagnostic)
        self.write_report(report)
        value, _ = self.load()
        self.assertNotIn(10, value['voltage_alarm_ids'])
        self.assertIsNone(value['device_errors'][0]['present_error'])
        self.assertEqual(value['device_errors'][0]['torque_error'], 0)

    def test_history_held_start_never_changes_live_proof_recovery_or_saved_config(self):
        history, _ = self.load()
        saved = config()
        saved['imu'].update(calibrated=True, axes_confirmed=True)
        launcher = mock.Mock(side_effect=AssertionError('Held history cannot touch hardware'))
        session = host.CalibrationSession(saved, self.directory, 9, Clock(), recovery_launcher=launcher)
        with self.assertRaises(ValueError):
            session.start_initial(held_diagnostic=history)
        with self.assertRaises(ValueError):
            session.start_initial(paused=True, held_diagnostic=history)
        session.start_initial(paused=True, hold_unsafe=True, held_diagnostic=history)
        expected = copy.deepcopy(saved)
        history['device_errors'][0]['present_error'] = 7
        snapshot = session.snapshot()
        self.assertEqual(snapshot['session']['status'], 'fault')
        self.assertIn('历史诊断：15个舵机均报 0x01', snapshot['session']['error'])
        self.assertNotIn('触发值未保存', snapshot['session']['error'])
        self.assertEqual(snapshot['session']['error_details'], host.UNSAFE_FEEDBACK_REASON)
        self.assertIsNone(snapshot['feedback'])
        self.assertIsNone(snapshot['body_imu'])
        self.assertIsNone(session.fault_feedback)
        self.assertIsNone(session.received_at)
        self.assertIsNone(session.closed_proof)
        self.assertIsNone(session.process)
        self.assertIsNone(session.fault_record)
        self.assertFalse(session.end_request_attempted)
        self.assertTrue(session.connection_done.is_set())
        self.assertFalse(snapshot['session']['recovery_allowed'])
        self.assertEqual(snapshot['session']['recovery_recheck']['technical_reason'], host.UNSAFE_FEEDBACK_REASON)
        self.assertEqual(snapshot['calibration'], expected)
        snapshot['session']['held_diagnostic']['device_errors'][0]['present_error'] = 0
        self.assertEqual(session.held_diagnostic['device_errors'][0]['present_error'], 1)
        with self.assertRaises(ValueError):
            session.recover_action(dict(generation=9))
        launcher.assert_not_called()

    def test_summary_schema_rechecks_ids_bytes_conclusion_and_does_not_clear_hold(self):
        original, _ = self.load()
        mutations = ((('scope',), 'live_feedback'), (('generation',), True),
                     (('device_errors', 0, 'present_error'), True), (('device_errors', 0, 'id'), 99),
                     (('voltage_alarm_ids',), []), (('message',), 'all safe'))
        for path, value in mutations:
            with self.subTest(path=path):
                history = copy.deepcopy(original)
                target = history
                for name in path[:-1]:
                    target = target[name]
                target[path[-1]] = value
                with self.assertRaises(ValueError):
                    host.validated_held_diagnostic(history)

    def test_cli_history_requires_pair_and_paused_resumed_unsafe_hold_before_service(self):
        prefix = ['feedback_calibration_server', '--run-dir', str(self.root / 'feedback-calibration-new')]
        pair = ['--held-diagnostic-report', str(self.path), '--held-diagnostic-sha256', self.sha]
        cases = [pair, ['--held-diagnostic-report', str(self.path)],
                 ['--held-diagnostic-sha256', self.sha],
                 [*pair, '--start-paused', '--resume-run', str(self.directory)],
                 [*pair, '--start-paused', '--hold-unsafe-feedback']]
        for options in cases:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), \
                    mock.patch.object(sys, 'argv', prefix + options), \
                    mock.patch.object(host, 'ThreadingHTTPServer',
                                      side_effect=AssertionError('Invalid CLI cannot start service')):
                with self.assertRaises(SystemExit) as error:
                    host.main()
                self.assertEqual(error.exception.code, 2)


class ResumeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name).resolve()
        self.root = mock.patch.object(host, 'RECOVERY', self.directory)
        self.root.start()
        self.run = self.directory / 'feedback-calibration-prior'
        self.run.mkdir(mode=0o700)
        self.original = config()
        self.saved = copy.deepcopy(self.original)
        self.saved['imu'].update(axes=['-y', 'x', 'z'], calibrated=True, axes_confirmed=True,
                                 bias_dps=[.1, -.2, .3])
        host.atomic_calibration(self.run, self.saved)

    def tearDown(self):
        self.root.stop()
        self.temporary.cleanup()

    def test_resume_preserves_complete_calibration_and_all_original_joints(self):
        resumed, raw = host.load_resume_calibration(self.run, self.original)
        self.assertEqual(resumed, self.saved)
        self.assertEqual(resumed['joints'], self.original['joints'])
        self.assertEqual(json.loads(raw), self.saved)
        self.assertTrue(resumed['imu']['calibrated'])

    def test_resume_preserves_saved_direction_ranges_imu_and_true_mechanical_reference(self):
        self.saved['mechanical_reference_confirmed'] = True
        self.saved['joints'][0].update(sign=-1, direction_confirmed=True, min=900, max=1100)
        host.atomic_calibration(self.run, self.saved)
        resumed, _ = host.load_resume_calibration(self.run, self.original)
        self.assertEqual(resumed, self.saved)
        self.assertEqual(host.joint_calibration.zero_signature(resumed),
                         host.joint_calibration.zero_signature(self.original))

    def test_resume_rejects_changed_joints_nonprivate_file_or_symlink(self):
        changed = copy.deepcopy(self.saved)
        changed['joints'][0]['zero'] += 1
        host.atomic_calibration(self.run, changed)
        with self.assertRaisesRegex(ValueError, '15'):
            host.load_resume_calibration(self.run, self.original)
        host.atomic_calibration(self.run, self.saved)
        file = self.run / 'calibration-latest.json'
        file.chmod(0o644)
        with self.assertRaises(ValueError):
            host.load_resume_calibration(self.run, self.original)
        file.chmod(0o600)
        file.rename(self.run / 'saved.json')
        file.symlink_to(self.run / 'saved.json')
        with self.assertRaises(OSError):
            host.load_resume_calibration(self.run, self.original)

    def test_resume_rejects_other_roots_relative_paths_or_malformed_calibration(self):
        for path in (Path('feedback-calibration-prior'), self.directory / 'wrong-name'):
            with self.assertRaises(ValueError):
                host.load_resume_calibration(path, self.original)
        (self.run / 'calibration-latest.json').write_text('{"joints":[]}')
        with self.assertRaises(ValueError):
            host.load_resume_calibration(self.run, self.original)


class BootstrapTests(unittest.TestCase):
    def test_exact_source_bytes_leave_immediately_queued_end_record_for_child(self):
        # This child only reads stdin and writes stdout. No SSH, reader, device,
        # filesystem access, HTTP service, or hardware command is invoked.
        source = ("import os, json\n"
                  "label = '校准'\n"
                  "control = bytearray()\n"
                  f"while len(control) < {len(host.END_FEEDBACK)}:\n"
                  f"    part = os.read(0, {len(host.END_FEEDBACK)} - len(control))\n"
                  "    if not part: raise SystemExit('Control bytes lost')\n"
                  "    control.extend(part)\n"
                  "print(json.dumps({'label': label, 'control': bytes(control).decode()}))\n").encode()
        child = subprocess.Popen([sys.executable, '-I', '-B', '-S', '-c', host.reader_bootstrap(len(source))],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = child.communicate(input=source + host.END_FEEDBACK, timeout=2)
        self.assertEqual(child.returncode, 0, error.decode())
        self.assertEqual(error, b'')
        self.assertEqual(json.loads(output), {'label': '校准', 'control': 'END_FEEDBACK\n'})

    def test_bootstrap_size_bounds_are_pure_and_strict(self):
        for size in (True, 0, -1, 262145):
            with self.subTest(size=size), self.assertRaises(ValueError):
                host.reader_bootstrap(size)

    def test_pipe_partial_writes_finish_one_record_without_replay(self):
        writes = []

        def write(descriptor, data):
            writes.append((descriptor, data))
            return min(2, len(data))

        with mock.patch.object(host.select, 'select', return_value=([], [777], [])), \
                mock.patch.object(host.os, 'write', side_effect=write):
            host.write_pipe_once(777, b'ABCDE', .2)
        self.assertEqual(writes, [(777, b'ABCDE'), (777, b'CDE'), (777, b'E')])


class ReviewedBaselineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name).resolve()
        self.recovery_root = mock.patch.object(host, 'RECOVERY', self.directory)
        self.recovery_root.start()
        self.expected = dict(boot_id='reviewed-boot', cid='reviewed-card',
                             root_mount_line='reviewed-mount', model='reviewed-board',
                             serial=dict(path='/dev/ttyUSB0', sysfs_name='1-1.3'),
                             kernel_prefix='original-prefix',
                             historical_faults=[{'line': 'old fault', 'reason': 'old'}],
                             generation=7)
        self.profile = copy.deepcopy(self.expected)
        del self.profile['generation']
        self.profile.update(kernel_prefix='original-prefix plus explicitly reviewed reconnect',
                            historical_faults=[{'line': 'old fault', 'reason': 'old'},
                                               {'line': 'specific unplug', 'reason': 'USB'}])
        self.path = self.directory / 'explicit-reviewed-baseline.json'
        self.raw = json.dumps(self.profile).encode()
        host.private_new(self.path, self.raw)
        self.digest = host.hashlib.sha256(self.raw).hexdigest()
        self.loader = mock.Mock(return_value=self.profile)

    def tearDown(self):
        self.recovery_root.stop()
        self.temporary.cleanup()

    def test_default_uses_original_expectation_without_reading_profile(self):
        result, raw = host.reviewed_expectations(self.expected, None, None, loader=self.loader)
        self.assertEqual(result, self.expected)
        self.assertIsNone(raw)
        self.loader.assert_not_called()
        result['historical_faults'].append({'line': 'changed'})
        self.assertEqual(len(self.expected['historical_faults']), 1)

    def test_explicit_private_digest_profile_changes_only_reviewed_kernel_fields(self):
        original = copy.deepcopy(self.expected)
        result, raw = host.reviewed_expectations(self.expected, self.path, self.digest, loader=self.loader)
        self.assertEqual(raw, self.raw)
        self.loader.assert_called_once_with(self.path, self.digest)
        self.assertEqual(result['kernel_prefix'], self.profile['kernel_prefix'])
        self.assertEqual(result['historical_faults'], self.profile['historical_faults'])
        for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'generation'):
            self.assertEqual(result[key], original[key])
        self.assertEqual(self.expected, original)
        self.assertEqual(self.path.read_bytes(), self.raw)

    def test_cli_pair_digest_and_private_file_rules_reject_before_loader(self):
        for path, digest in ((None, self.digest), (self.path, None),
                             (self.path, '0' * 64), (self.path, 'not-a-digest'),
                             (Path('relative.json'), self.digest)):
            with self.subTest(path=path, digest=digest), self.assertRaises(ValueError):
                host.reviewed_expectations(self.expected, path, digest, loader=self.loader)
        self.path.chmod(0o644)
        with self.assertRaises(ValueError):
            host.reviewed_expectations(self.expected, self.path, self.digest, loader=self.loader)
        self.loader.assert_not_called()

    def test_reviewed_profile_cannot_change_any_original_board_identity(self):
        for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial'):
            changed = copy.deepcopy(self.profile)
            changed[key] = {'path': 'other'} if key == 'serial' else 'other'
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations(self.expected, self.path, self.digest,
                                           loader=lambda *_args: changed)
        self.assertEqual(self.expected['generation'], 7)
        self.assertEqual(self.expected['kernel_prefix'], 'original-prefix')

    def reboot_fixture(self):
        self.expected['boot_id'] = host.REVIEWED_BOOT_ID
        self.profile['boot_id'] = host.REBOOT_BOOT_ID
        self.raw = json.dumps(self.profile).encode()
        self.path.unlink()
        host.private_new(self.path, self.raw)
        self.digest = host.hashlib.sha256(self.raw).hexdigest()

    def test_only_explicit_fixed_reboot_artifact_changes_old_to_exact_new_boot(self):
        self.reboot_fixture()
        with mock.patch.object(host, 'REBOOT_PROFILE_SHA256', self.digest):
            updated, raw = host.reviewed_expectations(self.expected, self.path, self.digest,
                                                      loader=self.loader)
        self.assertEqual(raw, self.raw)
        self.assertEqual(updated['boot_id'], host.REBOOT_BOOT_ID)
        self.assertEqual(self.expected['boot_id'], host.REVIEWED_BOOT_ID)
        for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
            self.assertEqual(updated[key], self.expected[key])
        # A current attempt revalidates the same artifact with the new boot.
        with mock.patch.object(host, 'REBOOT_PROFILE_SHA256', self.digest):
            repeated, _ = host.reviewed_expectations(updated, self.path, self.digest,
                                                     loader=self.loader)
        self.assertEqual(repeated, updated)

    def test_new_boot_without_fixed_artifact_or_wrong_original_boot_is_rejected(self):
        self.reboot_fixture()
        with self.assertRaisesRegex(ValueError, '板卡身份'):
            host.reviewed_expectations(self.expected, self.path, self.digest, loader=self.loader)
        with mock.patch.object(host, 'REBOOT_PROFILE_SHA256', self.digest):
            self.expected['boot_id'] = 'another-original-boot'
            with self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations(self.expected, self.path, self.digest, loader=self.loader)
        default, raw = host.reviewed_expectations(self.expected, None, None, loader=self.loader)
        self.assertEqual(default['boot_id'], self.expected['boot_id'])
        self.assertIsNone(raw)

    def test_reboot_artifact_never_allows_other_identity_changes_or_loader_failure(self):
        self.reboot_fixture()
        with mock.patch.object(host, 'REBOOT_PROFILE_SHA256', self.digest):
            for key in ('cid', 'root_mount_line', 'model', 'serial'):
                changed = copy.deepcopy(self.profile)
                changed[key] = {'path': 'other'} if key == 'serial' else 'other'
                with self.subTest(key=key), self.assertRaisesRegex(ValueError, '板卡身份'):
                    host.reviewed_expectations(self.expected, self.path, self.digest,
                                               loader=lambda *_args: changed)
            with self.assertRaisesRegex(ValueError, 'strict loader refused'):
                host.reviewed_expectations(self.expected, self.path, self.digest,
                    loader=mock.Mock(side_effect=ValueError('strict loader refused')))

    def test_actual_returned_esp32_artifact_maps_only_original_to_exact_reviewed_boot(self):
        import feedback_stream
        path=feedback_stream.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        expected={key:copy.deepcopy(profile[key]) for key in
                  ('boot_id','cid','root_mount_line','model','serial','kernel_prefix','historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID,generation=7)
        original=copy.deepcopy(expected)
        with mock.patch.object(host,'RECOVERY',feedback_stream.EVIDENCE_DIR):
            updated,raw=host.reviewed_expectations(expected,path,host.RETURNED_ESP32_PROFILE_SHA256)
            repeated,_=host.reviewed_expectations(updated,path,host.RETURNED_ESP32_PROFILE_SHA256)
        self.assertEqual(raw,path.read_bytes());self.assertEqual(repeated,updated)
        self.assertEqual(updated['boot_id'],host.REBOOT_BOOT_ID)
        self.assertEqual(updated['historical_faults'],feedback_stream.RETURNED_ESP32_HISTORICAL_FAULTS)
        self.assertEqual(expected,original)
        for key in ('cid','root_mount_line','model','serial','generation'):
            self.assertEqual(updated[key],expected[key])
        self.assertFalse(profile['review']['same_boot_usb_reconnect_review']['unsafe_feedback_hold_cleared'])

    def test_maintenance_firmware_artifact_preserves_identity_and_accepts_exact_reviewed_boot(self):
        import feedback_stream
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-esp32-reviewed-20261004-01.json'
        digest = feedback_stream.MAINTENANCE_ESP32_PROFILE_SHA256
        self.assertEqual(host.MAINTENANCE_ESP32_PROFILE_SHA256, digest)
        profile = feedback_stream.load_reviewed_profile(path, digest)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID, generation=7)
        with mock.patch.object(host, 'RECOVERY', feedback_stream.EVIDENCE_DIR):
            updated, raw = host.reviewed_expectations(expected, path, digest)
            repeated, _ = host.reviewed_expectations(updated, path, digest)
            self.assertEqual(updated, repeated)
            self.assertEqual(updated['boot_id'], host.REBOOT_BOOT_ID)
            self.assertEqual(raw, path.read_bytes())
            for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
                self.assertEqual(updated[key], expected[key])
            with self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations({**expected, 'boot_id': 'another-origin'}, path, digest)
            with self.assertRaises(ValueError):
                host.reviewed_expectations(expected, path, host.RETURNED_ESP32_PROFILE_SHA256)

    def test_write_ack_firmware_artifact_accepts_only_the_exact_reviewed_identity(self):
        import feedback_stream
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-write-ack-reviewed-20261004-01.json'
        digest = feedback_stream.WRITE_ACK_ESP32_PROFILE_SHA256
        self.assertEqual(host.WRITE_ACK_ESP32_PROFILE_SHA256, digest)
        profile = feedback_stream.load_reviewed_profile(path, digest)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID, generation=7)
        with mock.patch.object(host, 'RECOVERY', feedback_stream.EVIDENCE_DIR):
            actual, raw = host.reviewed_expectations(expected, path, digest)
            self.assertEqual(actual['boot_id'], host.REBOOT_BOOT_ID)
            self.assertEqual(raw, path.read_bytes())
            for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
                self.assertEqual(actual[key], expected[key])
            with self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations({**expected, 'boot_id': 'another-origin'}, path, digest)

    def test_center_trace_firmware_artifact_accepts_only_the_exact_reviewed_identity(self):
        import feedback_stream
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-center-trace-reviewed-20261005-01.json'
        digest = feedback_stream.CENTER_TRACE_ESP32_PROFILE_SHA256
        self.assertEqual(host.CENTER_TRACE_ESP32_PROFILE_SHA256, digest)
        profile = feedback_stream.load_reviewed_profile(path, digest)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID, generation=7)
        with mock.patch.object(host, 'RECOVERY', feedback_stream.EVIDENCE_DIR):
            actual, raw = host.reviewed_expectations(expected, path, digest)
            self.assertEqual(actual['boot_id'], host.REBOOT_BOOT_ID)
            self.assertEqual(raw, path.read_bytes())
            for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
                self.assertEqual(actual[key], expected[key])
            with self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations({**expected, 'boot_id': 'another-origin'}, path, digest)

    def test_cold_reconnect_accepts_only_the_reviewed_old_boots_and_same_board(self):
        import feedback_stream
        path = feedback_stream.EVIDENCE_DIR / 'feedback-cold-reconnect-reviewed-20261005-01.json'
        digest = feedback_stream.COLD_RECONNECT_PROFILE_SHA256
        self.assertEqual(host.COLD_RECONNECT_PROFILE_SHA256, digest)
        profile = feedback_stream.load_reviewed_profile(path, digest)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected['generation'] = 7
        with mock.patch.object(host, 'RECOVERY', feedback_stream.EVIDENCE_DIR):
            for previous in (host.REVIEWED_BOOT_ID, host.REBOOT_BOOT_ID, host.COLD_RECONNECT_BOOT_ID):
                actual, raw = host.reviewed_expectations({**expected, 'boot_id': previous}, path, digest)
                self.assertEqual(actual['boot_id'], host.COLD_RECONNECT_BOOT_ID)
                self.assertEqual(raw, path.read_bytes())
                for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
                    self.assertEqual(actual[key], expected[key])
            for change in ({'boot_id': 'another-origin'}, {'cid': '0' * 32},
                           {'serial': {**expected['serial'], 'vid': 'ffff'}}):
                with self.subTest(change=change), self.assertRaisesRegex(ValueError, '板卡身份'):
                    host.reviewed_expectations({**expected, **change}, path, digest)

    def test_center_holding_artifact_accepts_only_the_reviewed_board(self):
        import feedback_stream
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-center-holding-reviewed-20261005-01.json'
        digest = feedback_stream.CENTER_HOLDING_ESP32_PROFILE_SHA256
        self.assertEqual(host.CENTER_HOLDING_ESP32_PROFILE_SHA256, digest)
        profile = feedback_stream.load_reviewed_profile(path, digest)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID, generation=7)
        with mock.patch.object(host, 'RECOVERY', feedback_stream.EVIDENCE_DIR):
            actual, raw = host.reviewed_expectations(expected, path, digest)
            self.assertEqual(actual['boot_id'], host.REBOOT_BOOT_ID)
            self.assertEqual(raw, path.read_bytes())
            for key in ('cid', 'root_mount_line', 'model', 'serial', 'generation'):
                self.assertEqual(actual[key], expected[key])
            with self.assertRaisesRegex(ValueError, '板卡身份'):
                host.reviewed_expectations({**expected, 'boot_id': 'another-origin'}, path, digest)

    def test_returned_esp32_artifact_does_not_allow_another_origin_or_other_identity(self):
        import feedback_stream
        path=feedback_stream.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        expected={key:copy.deepcopy(profile[key]) for key in
                  ('boot_id','cid','root_mount_line','model','serial','kernel_prefix','historical_faults')}
        expected.update(boot_id=host.REVIEWED_BOOT_ID,generation=7)
        with mock.patch.object(host,'RECOVERY',feedback_stream.EVIDENCE_DIR):
            with self.assertRaises(ValueError):
                host.reviewed_expectations(expected,path,host.REBOOT_PROFILE_SHA256)
            with self.assertRaisesRegex(ValueError,'板卡身份'):
                host.reviewed_expectations({**expected,'boot_id':'another-origin'},path,
                                           host.RETURNED_ESP32_PROFILE_SHA256)
            for key in ('cid','root_mount_line','model','serial'):
                changed=copy.deepcopy(profile);changed[key]='other'
                with self.subTest(key=key),self.assertRaisesRegex(ValueError,'板卡身份'):
                    host.reviewed_expectations(expected,path,host.RETURNED_ESP32_PROFILE_SHA256,
                                               loader=lambda *_args:changed)


if __name__ == '__main__':
    unittest.main()
