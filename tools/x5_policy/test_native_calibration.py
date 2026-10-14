"""Synthetic native calibration integration tests; no UART, SSH or motors."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'x5_recovery'))
import feedback_calibration_server as host
import native_calibration as app


class Clock:
    now = 20.
    def __call__(self):
        return self.now


class Process:
    returncode = None
    stdin = type('Stdin', (), {'closed': False, 'fileno': lambda self: 777})()
    def poll(self):
        return self.returncode


def config():
    return dict(version=1, mechanical_reference_confirmed=True,
        joints=[dict(id=sid, zero=2000, sign=1, min=1000, max=3000,
                     direction_confirmed=True) for sid in host.joint_calibration.IDS],
        imu=dict(axes=['x', 'y', 'z'], axes_confirmed=True, calibrated=True,
                 bias_dps=[0., 0., 0.]))


class NativeCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name).resolve()
        self.clock = Clock()
        self.clock.now = 20.
        self.original = config()
        host.atomic_calibration(self.directory, self.original)
        self.commits = []
        self.controls = []
        self.forbid = mock.patch.object(host.subprocess, 'Popen', side_effect=AssertionError('No hardware'))
        self.forbid.start()
        self.pipe = mock.patch.object(host, 'write_pipe_once',
            side_effect=lambda descriptor, data, seconds: self.controls.append(data))
        self.pipe.start()
        self.service = app.NativeCalibrationService(self.original, self.directory, 7,
            reader_launcher=lambda *args: None, connection_checker=lambda *args: None,
            calibration_writer=self.commit, clock=self.clock)
        self.session = self.service.session
        self.sequence, self.sample = 1, 1000000
        self.positions = [2000] * 15

    def tearDown(self):
        self.pipe.stop()
        self.forbid.stop()
        self.tmp.cleanup()

    def commit(self, cfg, ledger):
        self.commits.append((copy.deepcopy(cfg), copy.deepcopy(ledger)))
        return host.atomic_calibration(self.directory, cfg)

    def save_after_reader_close(self, path, body):
        threads = []
        def end_reader(descriptor, data, seconds):
            self.controls.append(data)
            def close():
                self.session.accept(dict(kind='closed', generation=7, normal=True, serial_closed=True))
                self.session.process.returncode = 0
                self.session.confirm_closed(0)
                self.session.connection_done.set()
            thread = threading.Thread(target=close)
            threads.append(thread)
            thread.start()
        try:
            with mock.patch.object(host, 'write_pipe_once', side_effect=end_reader):
                return self.service.dispatch(path, body)
        finally:
            for thread in threads:
                thread.join(timeout=1)
                self.assertFalse(thread.is_alive())

    def test_native_status_reuses_lifecycle_ownership_instead_of_repeated_sensor_snapshots(self):
        motion = mock.Mock()
        motion.connection_reserved.return_value = True
        motion.snapshot.return_value = dict(phase='ready', owns_connection=True)
        self.session.motion = motion
        value = self.service.snapshot()
        self.assertTrue(value['motion']['owns_connection'])
        self.assertGreaterEqual(motion.connection_reserved.call_count, 4)
        motion.snapshot.assert_called_once_with()
        self.assertEqual(self.controls, [])
        motion.connection_reserved.return_value = 'unknown'
        with self.assertRaisesRegex(ValueError, '占用状态无效'):
            self.session.motion_reserved()

    def test_live_projection_skips_kernel_proof_and_duplicate_motion_without_changing_gates(self):
        class Proof:
            def __deepcopy__(self, memo):
                raise AssertionError('Live status must not copy full historical kernel proof')
        motion = mock.Mock()
        motion.connection_reserved.return_value = True
        motion.snapshot.side_effect = AssertionError('Outer native workbench owns the motion projection')
        self.session.motion = motion
        self.session.feedback_recheck = dict(phase='passed', purpose='imu', frame_count=6,
            connection_reconciled=Proof())
        live = self.service.snapshot(compact=True, include_motion=False)
        self.assertIsNone(live['motion'])
        self.assertEqual(live['session']['feedback_recheck'], dict(phase='passed', purpose='imu',
            frame_count=6, connection_reconciled=dict(details_available=True)))
        self.assertFalse(live['session']['imu_read_allowed'])
        self.assertIn('运控', live['session']['imu_read_blocked_reason'])
        motion.snapshot.assert_not_called()
        self.assertEqual(self.controls, [])

    def test_full_projection_retains_original_reconnection_evidence(self):
        proof = dict(kernel_log='original complete kernel lines', historical_faults=[dict(line='USB disconnect')])
        self.session.feedback_recheck = dict(phase='passed', connection_reconciled=proof)
        full = self.service.snapshot()
        self.assertEqual(full['session']['feedback_recheck']['connection_reconciled'], proof)
        full['session']['feedback_recheck']['connection_reconciled']['historical_faults'].clear()
        self.assertEqual(len(proof['historical_faults']), 1)

    def test_joint_capture_action_does_not_copy_history_or_motion_under_feedback_lock(self):
        self.start('joints')
        class SlowHistory:
            def __deepcopy__(self, memo):
                raise AssertionError('Capture action must not copy complete historical logs')
        motion = mock.Mock()
        motion.connection_reserved.return_value = False
        motion.snapshot.side_effect = AssertionError('Capture action must not rebuild motion history')
        self.session.motion = motion
        self.session.feedback_recheck['connection_reconciled'] = SlowHistory()
        self.capture_joint('direction-start', 2437, sid=14)
        self.capture_joint('direction-end', 2384, sid=14)
        wizard = self.session.joint_wizard.value
        self.assertEqual(wizard['stage'], 'await-range-a')
        self.assertEqual(wizard['inferred_sign'], -1)
        self.assertGreaterEqual(wizard['sample_count'], 12)
        self.assertEqual(self.session.status, 'active')
        self.assertEqual(self.session.config, self.original)
        motion.snapshot.assert_not_called()
        self.assertEqual(self.commits, [])
        self.assertEqual(self.controls, [])

    def start(self, scope):
        s = self.session
        s.status, s.feedback_scope = 'waiting', scope
        s.feedback_recheck = dict(purpose=scope, phase='checking', frame_count=0, span_ms=0.,
            previous_hold=None, pending_servo_hold=None, error=None)
        s.process = Process()
        s.connection_done.clear()
        s.accept(dict(kind='ready', generation=7, emitted_monotonic=self.clock()))
        for _ in range(12):
            self.feed()
        self.assertEqual(s.status, 'active')

    def state(self, accel=None):
        return dict(sample_us=self.sample, imu_us=self.sample - 1000,
            online_mask=0 if self.session.feedback_scope == 'imu' else 32767,
            fault_mask=32767 if self.session.feedback_scope == 'imu' else 0, torque_mask=0,
            armed=False, arm_cookie=0, imu_ok=True, last_error=0,
            accel_ms2=accel or [0., 0., 9.81], gyro_dps=[0., 0., 0.], positions=list(self.positions))

    def test_reader_cadence_reports_sensor_gaps_without_authorizing_sampling(self):
        self.start('imu')
        before = self.session.snapshot()['session']['reader_cadence']
        self.feed()
        current = self.session.snapshot()['session']['reader_cadence']
        self.assertEqual(current['frames'], before['frames'] + 1)
        self.assertEqual(current['max_imu_gap_ms'], 20.)
        self.sample += 120000
        self.feed()
        current = self.session.snapshot()['session']['reader_cadence']
        self.assertEqual(current['max_sample_gap_ms'], 140.)
        self.assertEqual(current['gaps_at_least_100ms'], 1)
        current['frames'] = 0
        self.assertGreater(self.session.snapshot()['session']['reader_cadence']['frames'], 0)
        self.assertIsNone(self.session.collection)

    def feed(self, accel=None, **changes):
        self.clock.now += .02
        self.sample += 20000
        self.sequence += 1
        value = self.state(accel)
        value.update(changes)
        self.session.accept(dict(kind='state', generation=7, sequence=self.sequence,
            received_monotonic=self.clock(), collection_scope=self.session.feedback_scope, state=value))

    def imu(self, action, accel):
        self.service.dispatch('/api/imu-wizard', dict(action=action, generation=7,
            mounted_confirmed=True, support_confirmed=True))
        for _ in range(100):
            self.feed(accel)

    def joint_body(self, action, sid=10):
        return dict(action=action, id=sid, generation=7, support_confirmed=True,
                    positive_confirmed=True)

    def capture_joint(self, action, position, sid=10):
        self.positions[host.joint_calibration.IDS.index(sid)] = position
        self.feed()
        self.service.dispatch('/api/joint-calibration', self.joint_body(action, sid))
        for _ in range(20):
            self.feed()

    def test_construction_is_paused_preserves_saved_calibration(self):
        value = self.service.snapshot()
        self.assertEqual(value['session']['status'], 'closed')
        self.assertFalse(self.service.owns_connection)
        self.assertTrue(self.service.connection_closed())
        self.assertEqual(value['calibration'], self.original)
        self.assertEqual(value['connection_target']['page_url'], 'http://192.168.1.11:8767/')
        self.assertEqual(self.commits, [])

    def offline_service(self, reason, checker=None):
        reader = mock.Mock(side_effect=AssertionError('Offline startup must not open reader'))
        checker = checker or mock.Mock(side_effect=AssertionError('Offline startup must not check hardware'))
        with mock.patch.object(app.NativeCalibrationSession, 'fault',
                               side_effect=AssertionError('Offline startup must not send END')):
            service = app.NativeCalibrationService(self.original, self.directory, 8,
                reader_launcher=reader, connection_checker=checker,
                calibration_writer=self.commit, clock=self.clock, startup_fault=reason)
        return service, reader, checker

    def test_offline_startup_retains_fault_without_fabricated_hardware_or_closure(self):
        reason = 'New kernel fault; classifier result requires source review: ' + json.dumps(dict(
            new_fault_count=1, faults=[dict(line='USB disconnect',
                classifier_reason='Kernel reported a USB disconnect/reset/error')],
            fault_details_truncated=False))
        service, reader, checker = self.offline_service(reason)
        s = service.session; value = service.snapshot()
        self.assertEqual(value['session']['status'], 'fault')
        self.assertEqual(value['session']['error_details'], reason)
        self.assertEqual(value['session']['recovery_recheck'], host.recovery_recheck_error(reason))
        self.assertFalse(value['session']['recovery_allowed'])

        self.assertFalse(value['session']['recheck_allowed'])
        self.assertTrue(value['session']['connection_check']['allowed'])
        self.assertEqual(value['session']['connection_check']['phase'], 'failed')
        self.assertEqual(value['session']['connection_check']['technical_reason'], reason)
        self.assertIsNone(value['feedback']); self.assertIsNone(value['body_imu'])
        self.assertIsNone(value['session']['closed_proof'])
        self.assertIsNone(value['session']['fault_feedback'])
        self.assertIsNone(value['session']['torque_ids'])
        self.assertIsNone(s.process); self.assertIsNone(s.ready)
        self.assertTrue(s._paused_without_attempt); self.assertTrue(s.connection_done.is_set())
        self.assertFalse(s.end_request_attempted); self.assertFalse(s.end_request_sent)
        self.assertFalse(service.owns_connection); self.assertTrue(service.connection_closed())
        self.assertEqual(value['calibration'], self.original)
        self.assertEqual(value['wizard']['stage'], 'complete')
        self.assertEqual(self.commits, []); self.assertEqual(self.controls, [])
        reader.assert_not_called(); checker.assert_not_called()
        with self.assertRaisesRegex(ValueError, '关闭证明'):
            service.end_for_motion(timeout=0)
        with self.assertRaisesRegex(ValueError, '核对板端'):
            service.dispatch('/api/recover-feedback', dict(generation=8, purpose='imu'))
        reader.assert_not_called(); checker.assert_not_called()

    def test_diagnostic_only_initialization_is_pending_not_a_hardware_fault(self):
        service,reader,checker=self.offline_service(app.DIAGNOSTIC_STARTUP_MESSAGE)
        value=service.snapshot()
        self.assertEqual(value['session']['status'],'closed')
        self.assertIsNone(value['session']['error'])
        self.assertIsNone(value['session']['recovery_recheck'])
        self.assertIsNone(value['feedback'])
        self.assertTrue(service.connection_closed())
        reader.assert_not_called();checker.assert_not_called()

    def test_maintenance_review_reconciles_unopened_trust_only_after_all_feedback_and_closure(self):
        service,reader,checker=self.offline_service('USB startup connection check failed')
        session=service.session
        raw=json.dumps(dict(reason='USB connection lost',requires_feedback_review=True)).encode()
        host.private_new(self.directory/'maintenance-fault-hold.json',raw)
        session.configure_maintenance(lambda:None,hold_directory=self.directory)
        self.assertIsNone(session.maintenance_review_blocked_reason())
        session.maintenance_review_hold_sha=hashlib.sha256(raw).hexdigest()
        rows=[dict(id=sid,online=True,state=0,error=0,torque=0,position=2000)
              for sid in host.joint_calibration.IDS]
        proof=dict(serial_closed=True,process_exited=True,process_exit_code=0)
        bad=copy.deepcopy(rows);bad[0]['torque']=1
        with self.assertRaises(ValueError):session._maintenance_reviewed(bad,proof)
        with self.assertRaises(ValueError):session._maintenance_reviewed(rows,{**proof,'serial_closed':False})
        self.assertTrue(session.maintenance_fault_hold)
        self.assertEqual(session.status,'fault')
        self.assertIsNotNone(session.recovery_recheck)
        session._maintenance_reviewed(rows,proof)
        self.assertFalse(session.maintenance_fault_hold)
        self.assertIsNone(session.recovery_recheck)
        self.assertEqual(session.status,'closed')
        self.assertEqual(session.config,self.original)
        self.assertEqual(self.controls,[]);self.assertEqual(self.commits,[])
        reader.assert_not_called();checker.assert_not_called()

    def test_independent_imu_checks_fresh_connection_and_preserves_servo_holds(self):
        s=self.session
        raw=json.dumps(dict(reason='舵机 ID 10 反馈异常',requires_feedback_review=True)).encode()
        host.private_new(self.directory/'maintenance-fault-hold.json',raw)
        s.configure_maintenance(lambda:None,hold_directory=self.directory)
        s.maintenance_mapping_changed=True
        s.recovery_recheck=host.recovery_recheck_error('USB startup connection check failed')
        s.status='fault'
        value=self.service.snapshot()
        self.assertTrue(value['session']['imu_read_allowed'])
        self.assertFalse(value['session']['connection_check']['allowed'])
        self.assertFalse(value['session']['recheck_allowed'])
        proof=dict(kind='quiescent',generation=7,serial_path='/dev/ttyUSB0',serial_holders=[],
            identity_checked=True,guards_passed=True,observation_count=2,serial_open_attempts=0,
            serial_bytes_written=0,board_files_written=False,automatic_retry=False,
            transport='local',ssh_started=False,process_exit_code=0,process_exited=True)
        checker=mock.Mock(return_value=proof);s.connection_checker=checker
        def reader(session,generation,trace):
            self.assertEqual(session.feedback_scope,'imu')
            session.process=Process()
            session.accept(dict(kind='ready',generation=generation,emitted_monotonic=self.clock()))
            for sequence in range(2,15):
                self.clock.now+=.02
                state=dict(sample_us=sequence*20000,imu_us=sequence*20000-1000,
                    online_mask=2,fault_mask=32765,torque_mask=0,armed=False,arm_cookie=0,
                    imu_ok=True,last_error=0,accel_ms2=[0.,0.,9.81],gyro_dps=[0.,0.,0.],positions=[2000]*15)
                session.accept(dict(kind='state',generation=generation,sequence=sequence,
                    received_monotonic=self.clock(),collection_scope='imu',state=state))
        s.recovery_launcher=mock.Mock(side_effect=reader)
        with mock.patch.object(app.threading,'Thread') as thread:
            thread.side_effect=lambda *,target,daemon:type('ImmediateThread',(),{'start':lambda self:target()})()
            result=self.service.dispatch('/api/check-connection',dict(generation=7,purpose='imu'))
        checker.assert_called_once();s.recovery_launcher.assert_called_once()
        self.assertEqual(result['session']['status'],'active')
        self.assertEqual(result['session']['feedback_recheck']['phase'],'passed')
        self.assertIsNotNone(result['body_imu']['attitude'])
        self.assertEqual(result['feedback']['fault_mask'],32765)
        self.assertEqual(result['feedback']['torque_mask'],0)
        self.assertTrue(s.maintenance_fault_hold);self.assertTrue(s.maintenance_mapping_changed)
        self.assertEqual((self.directory/'maintenance-fault-hold.json').read_bytes(),raw)
        self.assertFalse(result['capabilities']['joint_feedback'])
        self.assertEqual(result['calibration'],self.original)
        self.assertEqual(self.controls,[]);self.assertEqual(self.commits,[])
        with self.assertRaises(ValueError):
            self.service.dispatch('/api/joint-calibration',{**self.joint_body('direction-start'),'generation':s.generation})
        with self.assertRaises(ValueError):
            self.service.end_for_motion(timeout=0)

    def test_independent_imu_does_not_open_reader_when_metadata_check_fails(self):
        s=self.session;s.maintenance_fault_hold=True
        s.recovery_recheck=host.recovery_recheck_error('USB startup connection check failed')
        s.status='fault'
        s.connection_checker=mock.Mock(side_effect=ValueError('Unknown new kernel fault'))
        s.recovery_launcher=mock.Mock(side_effect=AssertionError('Must not open UART'))
        with mock.patch.object(app.threading,'Thread') as thread:
            thread.side_effect=lambda *,target,daemon:type('ImmediateThread',(),{'start':lambda self:target()})()
            result=self.service.dispatch('/api/check-connection',dict(generation=7,purpose='imu'))
        s.recovery_launcher.assert_not_called()
        self.assertEqual(result['session']['connection_check']['phase'],'failed')
        self.assertTrue(s.maintenance_fault_hold)
        self.assertIsNone(result['feedback']);self.assertEqual(self.controls,[])

    def test_independent_imu_rechecks_owner_and_requires_actual_previous_uart_closure(self):
        s=self.session;s.maintenance_fault_hold=True
        self.assertIsNone(s.imu_read_blocked_reason())
        s.motion=mock.Mock(spec=['snapshot']);s.motion.snapshot.return_value=dict(phase='ready',owns_connection=True)
        self.assertIsNotNone(s.imu_read_blocked_reason())
        s.motion=None;s.process=Process();s.process.returncode=0;s._paused_without_attempt=False
        for proof in (None,dict(kind='closed',generation=6,serial_closed=True),dict(kind='closed',generation=7,serial_closed=False)):
            s.closed_proof=proof;self.assertIsNotNone(s.imu_read_blocked_reason())
        s.closed_proof=dict(kind='closed',generation=7,serial_closed=True)
        self.assertIsNone(s.imu_read_blocked_reason())
        s.process.returncode=None;self.assertIsNotNone(s.imu_read_blocked_reason())

    def test_independent_imu_refuses_owner_acquired_during_metadata_check(self):
        s=self.session;s.maintenance_fault_hold=True
        proof=dict(kind='quiescent',generation=7,serial_path='/dev/ttyUSB0',serial_holders=[],
            identity_checked=True,guards_passed=True,observation_count=2,serial_open_attempts=0,
            serial_bytes_written=0,board_files_written=False,automatic_retry=False,
            transport='local',ssh_started=False,process_exit_code=0,process_exited=True)
        def checker(*args):
            s.motion=mock.Mock(spec=['snapshot']);s.motion.snapshot.return_value=dict(phase='ready',owns_connection=True)
            return proof
        s.connection_checker=checker;s.recovery_launcher=mock.Mock()
        with mock.patch.object(app.threading,'Thread') as thread:
            thread.side_effect=lambda *,target,daemon:type('ImmediateThread',(),{'start':lambda self:target()})()
            result=self.service.dispatch('/api/check-connection',dict(generation=7,purpose='imu'))
        s.recovery_launcher.assert_not_called()
        self.assertEqual(result['session']['connection_check']['phase'],'failed')
        self.assertTrue(s.maintenance_fault_hold)
        self.assertEqual(self.controls,[])

    def test_unknown_actual_startup_fault_stays_metadata_recheckable_without_end(self):
        reason = 'Overall probe deadline expired before any UART attempt'
        service, reader, checker = self.offline_service(reason)
        value = service.snapshot()
        hold = value['session']['recovery_recheck']
        self.assertEqual(hold['category'], 'trust')
        self.assertEqual(hold['technical_reason'], reason)
        self.assertEqual(value['session']['error'], reason)
        self.assertTrue(value['session']['connection_check']['allowed'])
        self.assertFalse(value['session']['recheck_allowed'])
        self.assertEqual(self.controls, [])
        reader.assert_not_called(); checker.assert_not_called()
        with self.assertRaisesRegex(ValueError, '从未启动'):
            service.session.start_offline(reason)

    def test_offline_metadata_recheck_failure_keeps_reader_unopened(self):
        reason = 'New kernel fault; classifier result requires source review: USB changed'
        checker = mock.Mock(side_effect=ValueError(reason))
        service, reader, checker = self.offline_service(reason, checker)
        with mock.patch.object(app.threading, 'Thread') as thread:
            thread.side_effect = lambda *, target, daemon: type('ImmediateThread', (), {'start': lambda self: target()})()
            result = service.dispatch('/api/check-connection', dict(generation=8, purpose='imu'))
        checker.assert_called_once(); reader.assert_not_called()
        self.assertEqual(result['session']['status'], 'fault')
        self.assertEqual(result['session']['connection_check']['phase'], 'failed')
        self.assertTrue(result['session']['connection_check']['allowed'])
        self.assertFalse(result['session']['recheck_allowed'])
        self.assertIsNone(result['session']['closed_proof'])
        self.assertIsNone(result['feedback'])
        self.assertEqual(self.controls, [])

    def test_offline_initializer_rejects_invalid_or_already_started_context(self):
        for reason in ('', ' ', 3, 'x' * 16385):
            with self.subTest(reason_type=type(reason).__name__), self.assertRaises(ValueError):
                self.offline_service(reason)
        self.start('imu')
        with self.assertRaisesRegex(ValueError, '从未启动'):
            self.session.start_offline('actual fault')
        self.assertEqual(self.controls, [])

    def test_four_imu_steps_save_only_at_finish_retaining_raw_offline_faults(self):
        self.start('imu')
        value = self.service.snapshot()
        self.assertEqual(value['feedback']['fault_mask'], 32767)
        self.assertEqual(value['feedback']['scope'], 'imu')
        self.assertFalse(value['capabilities']['joint_feedback'])
        self.imu('level', [0., 0., 9.81])
        self.imu('front-up', [4.905, 0., 9.81 * math.cos(math.pi / 6)])
        self.imu('left-up', [0., 4.905, 9.81 * math.cos(math.pi / 6)])
        self.assertEqual(self.commits, [])
        self.imu('finish', [0., 0., 9.81])
        self.assertEqual(len(self.commits), 1)
        self.assertEqual(self.commits[0][0]['joints'], self.original['joints'])
        self.assertEqual(self.session.wizard['stage'], 'complete')
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.assertEqual(self.session.status, 'stopping')

    def test_imu_display_projects_motion_angles_and_preserves_legacy_body_fields(self):
        self.start('imu')
        angle = math.radians(30)
        acceleration = [9.81 * math.sin(angle), 0., 9.81 * math.cos(angle)]
        self.feed(acceleration)
        value = self.service.snapshot(); body = value['body_imu']
        self.assertEqual(body['accel_ms2'], acceleration)
        self.assertEqual(body['gyro_dps'], [0., 0., 0.])
        self.assertAlmostEqual(body['pitch_deg'], 30.)
        self.assertAlmostEqual(body['roll_deg'], 0.)
        self.assertAlmostEqual(body['attitude']['tilt_deg'], 30.)
        self.assertAlmostEqual(body['attitude']['pitch_deg'], -30.)
        self.assertAlmostEqual(body['attitude']['roll_deg'], 0.)
        self.assertEqual(body['attitude']['protocol'], 1)
        self.assertEqual(body['attitude_protocol'], 1)
        self.assertEqual(body['source'], 'calibration-feedback')
        self.assertEqual(body['collection_scope'], 'imu')
        self.assertEqual(body['sample_us'], value['feedback']['sample_us'])
        self.assertEqual(body['imu_us'], value['feedback']['imu_us'])
        self.assertEqual(body['imu_age_ms'], 1.)
        self.assertEqual(value['feedback']['online_mask'], 0)
        self.assertEqual(value['feedback']['fault_mask'], 32767)
        self.assertEqual(value['feedback']['torque_mask'], 0)
        self.assertFalse(value['capabilities']['joint_feedback'])
        self.assertEqual(self.controls, []); self.assertEqual(self.commits, [])
        with self.assertRaisesRegex(ValueError, 'IMU 专用'):
            self.service.dispatch('/api/joint-calibration', self.joint_body('direction-start'))

    def test_imu_display_uses_saved_axes_bias_and_matches_motion_gravity_convention(self):
        self.session.config['imu'].update(axes=['y', '-x', 'z'], bias_dps=[1., 2., 3.])
        self.start('imu')
        self.feed([-4.905, 0., 9.81 * math.cos(math.pi / 6)], gyro_dps=[2., 4., 6.])
        body = self.service.snapshot()['body_imu']
        self.assertEqual(body['gyro_dps'], [2., -1., 3.])
        self.assertAlmostEqual(body['attitude']['tilt_deg'], 30.)
        self.assertAlmostEqual(body['attitude']['pitch_deg'], 0.)
        self.assertAlmostEqual(body['attitude']['roll_deg'], 30.)
        # Compare all three display equations to the actual motion filter's
        # reset output rather than copying a second angle implementation.
        from rl_core import ImuFilter, State
        ax, ay, az = body['accel_ms2']
        filtered = ImuFilter().update(State([ay, az, -ax], [0., 0., 0.], [0.] * 15, [0.] * 15))
        gx, gy, gz = filtered.grav
        expected = (math.degrees(math.acos(-gz)), math.degrees(math.atan2(gx, -gz)),
                    math.degrees(math.atan2(-gy, -gz)))
        for name, angle in zip(('tilt_deg', 'pitch_deg', 'roll_deg'), expected):
            self.assertAlmostEqual(body['attitude'][name], angle)

    def test_imu_display_hides_stale_or_invalid_attitude_without_changing_feedback_gates(self):
        self.start('imu')
        self.feed([0., 0., 0.])
        value = self.service.snapshot()
        self.assertIsNone(value['body_imu'])
        self.assertIsNotNone(value['feedback'])
        self.assertEqual(value['session']['status'], 'active')
        self.feed()
        fresh = self.service.snapshot()
        for change in (dict(accel_ms2=[float('nan'), 0., 9.81]),
                       dict(gyro_dps=[0., float('inf'), 0.]), dict(accel_ms2=[0., 0., 0.])):
            value = copy.deepcopy(fresh); value['body_imu'].update(change)
            with self.subTest(change=change): self.assertIsNone(app._imu_display_body(value))
        for change in (dict(calibrated=False), dict(axes_confirmed=False), dict(axes=['x', 'x', 'z'])):
            value = copy.deepcopy(fresh); value['calibration']['imu'].update(change)
            with self.subTest(change=change): self.assertIsNone(app._imu_display_body(value))
        self.clock.now += .5
        value = self.service.snapshot()
        self.assertIsNone(value['body_imu']); self.assertIsNone(value['feedback'])
        self.assertEqual(self.session.status, 'active')
        self.assertEqual(self.controls, [])

    def test_imu_display_signed_modulo_clock_age_and_exact_frame_binding(self):
        self.start('imu')
        value = self.service.snapshot()
        value['feedback'].update(sample_us=1000, imu_us=0xfffff000)
        body = app._imu_display_body(value)
        self.assertEqual(body['sample_us'], 1000); self.assertEqual(body['imu_us'], 0xfffff000)
        self.assertAlmostEqual(body['imu_age_ms'], 5.096)
        value['feedback'].update(sample_us=1000, imu_us=6000)
        self.assertEqual(app._imu_display_body(value)['imu_age_ms'], 0.)
        value['feedback'].update(sample_us=1000, imu_us=501000)
        self.assertEqual(app._imu_display_body(value)['imu_age_ms'], 0.)
        for timestamps in ((1000, 501001), (501001, 1000), (0, 0x80000000), (True, 1000)):
            value['feedback'].update(zip(('sample_us', 'imu_us'), timestamps))
            with self.subTest(timestamps=timestamps): self.assertIsNone(app._imu_display_body(value))
        value['feedback'].update(sample_us=490000, imu_us=0)
        value['session']['received_age_ms'] = 10.
        self.assertIsNone(app._imu_display_body(value))
        value['session']['received_age_ms'] = 9.
        self.assertEqual(app._imu_display_body(value)['imu_age_ms'], 490.)
        for age in (float('nan'), float('inf'), -1., None):
            value['session']['received_age_ms'] = age
            with self.subTest(age=age): self.assertIsNone(app._imu_display_body(value))

    def test_display_projection_stays_out_of_joint_and_unconfirmed_calibration_snapshots(self):
        self.start('joints')
        body = self.service.snapshot()['body_imu']
        self.assertEqual(set(body), {'accel_ms2', 'gyro_dps', 'pitch_deg', 'roll_deg'})
        self.session.config['imu']['calibrated'] = False
        self.assertIsNone(self.service.snapshot()['body_imu'])
        self.assertEqual(self.controls, [])

    def test_offline_imu_never_authorizes_joint_or_zero_samples(self):
        self.start('imu')
        with self.assertRaisesRegex(ValueError, 'IMU 专用'):
            self.service.dispatch('/api/joint-calibration', self.joint_body('direction-start'))
        with self.assertRaises(ValueError):
            self.service.dispatch('/api/joint-zero', dict(action='capture', id=10,
                generation=7, support_confirmed=True, zero_reference_confirmed=True))
        self.assertEqual(self.commits, [])

    def test_imu_scope_preserves_partial_and_online_masks_without_joint_authority(self):
        self.start('imu')
        for online,fault in ((2,32765),(32767,0),(32767,1),(0,0)):
            with self.subTest(online=online,fault=fault):
                self.feed(online_mask=online,fault_mask=fault)
                snapshot=self.service.snapshot()
                self.assertEqual(snapshot['session']['status'],'active')
                self.assertEqual(snapshot['feedback']['online_mask'],online)
                self.assertEqual(snapshot['feedback']['fault_mask'],fault)
                self.assertFalse(snapshot['feedback']['joint_feedback_valid'])
                self.assertIsNotNone(snapshot['body_imu']['attitude'])
                with self.assertRaisesRegex(ValueError,'IMU 专用'):
                    self.service.dispatch('/api/joint-calibration',self.joint_body('direction-start'))
                with self.assertRaises(ValueError):
                    self.service.dispatch('/api/joint-zero',dict(action='capture',id=10,
                        generation=7,support_confirmed=True,zero_reference_confirmed=True))
        self.assertEqual(self.commits,[])
        self.assertEqual(self.controls,[])

    def test_imu_scope_rejects_invalid_masks_or_armed_or_firmware_fault_state(self):
        self.start('imu')
        for change in ({'online_mask': -1}, {'online_mask': 32768}, {'online_mask': '1'},
                       {'fault_mask': True}, {'fault_mask': -1}, {'fault_mask': 32768}, {'torque_mask': 1},
                       {'armed': True}, {'arm_cookie': 3}, {'imu_ok': False},
                       {'last_error': 1}, {'online_mask': False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.feed(**change)
        with self.assertRaisesRegex(ValueError, '绑定'):
            self.session.accept(dict(kind='state', generation=7, sequence=999,
                received_monotonic=self.clock(), state=self.state()))

    def test_frozen_imu_and_old_generation_do_not_collect(self):
        self.start('imu')
        self.service.dispatch('/api/imu-wizard', dict(action='level', generation=7,
            mounted_confirmed=True, support_confirmed=True))
        state = self.state()
        for _ in range(5):
            self.clock.now += .02
            self.sequence += 1
            self.session.accept(dict(kind='state', generation=7, sequence=self.sequence,
                received_monotonic=self.clock(), collection_scope='imu', state=state))
        self.assertEqual(self.session.wizard['sample_count'], 0)
        with self.assertRaisesRegex(ValueError, '变化'):
            self.service.dispatch('/api/imu-wizard', dict(action='level', generation=6,
                mounted_confirmed=True, support_confirmed=True))
        self.clock.now += .6
        self.session.check_watchdog()
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.feedback)

    def test_servo_clock_advance_does_not_refresh_frozen_imu(self):
        self.start('imu')
        previous_received = self.session.received_at
        frozen_imu = self.session.feedback['imu_us']
        self.service.dispatch('/api/imu-wizard', dict(action='level', generation=7,
            mounted_confirmed=True, support_confirmed=True))
        for _ in range(20):
            self.feed(imu_us=frozen_imu)
        self.assertEqual(self.session.received_at, previous_received)
        self.assertEqual(self.session.wizard['sample_count'], 0)
        self.assertEqual(self.commits, [])

    def test_motion_switch_during_source_transfer_does_not_reopen_uart(self):
        self.start('joints')
        self.session.source_transmitting = True
        self.session.recovery_pending = True
        with self.assertRaisesRegex(ValueError, '关闭尚未确认'):
            self.service.end_for_motion(timeout=0)
        self.assertEqual(self.controls, [])
        self.assertEqual(self.session.status, 'stopping')
        self.session.source_finished(7, self.session.process)
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.assertTrue(self.service.owns_connection)

    def test_motion_switch_never_uses_old_closed_proof_during_metadata_recheck(self):
        self.session.connection_check['phase'] = 'checking'
        self.session.recovery_pending = True
        with self.assertRaisesRegex(ValueError, '复查连接'):
            self.service.end_for_motion(timeout=0)
        self.assertTrue(self.service.owns_connection)
        self.assertEqual(self.controls, [])

    def test_direction_range_sampling_commits_only_one_joint(self):
        self.start('joints')
        for action, position in (('direction-start', 2000), ('direction-end', 2050),
                                 ('range-a', 1600), ('range-b', 2400)):
            self.capture_joint(action, position)
        self.assertEqual(self.commits, [])
        self.save_after_reader_close('/api/joint-calibration', self.joint_body('save'))
        self.assertEqual(len(self.commits), 1)
        saved = self.commits[0][0]
        self.assertEqual(saved['joints'][0]['min'], 1612)
        self.assertEqual(saved['joints'][0]['max'], 2388)
        self.assertEqual(saved['joints'][1:], self.original['joints'][1:])
        self.assertEqual(saved['imu'], self.original['imu'])
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.assertEqual(self.session.status, 'closed')

    def test_range_wider_than_half_turn_saves_through_native_api(self):
        self.start('joints')
        for action, position in (('direction-start', 2000), ('direction-end', 2050),
                                 ('range-a', 900)):
            self.capture_joint(action, position)
        for position in range(1000, 3500, 100):
            self.positions[0] = position
            self.feed()
        self.capture_joint('range-b', 3500)
        self.assertTrue(self.session.joint_wizard.value['ready_to_save'])
        self.assertEqual(self.commits, [])
        self.save_after_reader_close('/api/joint-calibration', self.joint_body('save'))
        self.assertEqual(len(self.commits), 1)
        saved = self.commits[0][0]
        expected = copy.deepcopy(self.original)
        expected['joints'][0].update(min=912, max=3488)
        self.assertEqual(saved, expected)
        self.assertEqual(json.loads((self.directory / 'calibration-latest.json').read_text()), saved)
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.assertEqual(self.session.status, 'closed')

    def test_zero_capture_is_preview_save_invalidates_only_selected_joint(self):
        self.start('joints')
        self.positions[0] = 2004
        self.feed()
        body = dict(action='capture', id=10, generation=7, support_confirmed=True,
                    zero_reference_confirmed=True)
        self.service.dispatch('/api/joint-zero', body)
        for _ in range(20):
            self.feed()
        self.assertEqual(self.session.joint_zero['stage'], 'ready')
        self.assertEqual(self.commits, [])
        self.save_after_reader_close('/api/joint-zero', dict(body, action='save'))
        saved, ledger = self.commits[0]
        self.assertEqual(saved['joints'][0]['zero'], 2004)
        self.assertFalse(saved['joints'][0]['direction_confirmed'])
        self.assertIsNone(saved['joints'][0]['min'])
        self.assertEqual(saved['joints'][1:], self.original['joints'][1:])
        self.assertEqual(saved['imu'], self.original['imu'])
        self.assertEqual(ledger['entries'][0]['id'], 10)
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.assertEqual(self.session.status, 'closed')
        self.assertIsNone(self.session.feedback)
        self.assertEqual(self.session.joint_zero['stage'], 'saved')

    def test_slow_zero_commit_ends_source_before_write_and_does_not_collect_queued_frames(self):
        self.start('joints')
        body = dict(action='capture', id=10, generation=7, support_confirmed=True,
                    zero_reference_confirmed=True)
        self.service.dispatch('/api/joint-zero', body)
        for _ in range(20):
            self.feed()
        generated = self.clock()
        queued = self.state()
        def slow_commit(cfg, ledger):
            self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
            self.assertEqual(self.session.status, 'closed')
            self.assertTrue(self.session.connection_done.is_set())
            self.assertEqual(self.session.process.poll(), 0)
            self.clock.now += 3.5
            self.session.check_watchdog()
            self.session.accept(dict(kind='state', generation=7, sequence=self.sequence + 1,
                received_monotonic=generated, collection_scope='joints', state=queued))
            self.assertIsNone(self.session.feedback)
            return self.commit(cfg, ledger)
        self.session.calibration_writer = slow_commit
        self.save_after_reader_close('/api/joint-zero', dict(body, action='save'))
        self.assertEqual(len(self.commits), 1)
        self.assertEqual(self.session.status, 'closed')
        self.assertIsNone(self.session.error)
        self.assertEqual(self.session.joint_zero['stage'], 'saved')

    def test_zero_commit_refuses_to_write_when_readonly_end_cannot_be_delivered(self):
        self.start('joints')
        self.positions[0] = 2004
        self.feed()
        body = dict(action='capture', id=10, generation=7, support_confirmed=True,
                    zero_reference_confirmed=True)
        self.service.dispatch('/api/joint-zero', body)
        for _ in range(20):
            self.feed()
        with mock.patch.object(host, 'write_pipe_once', side_effect=OSError('pipe closed')):
            with self.assertRaisesRegex(ValueError, '结束反馈请求未送达'):
                self.service.dispatch('/api/joint-zero', dict(body, action='save'))
        self.assertEqual(self.commits, [])
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.closed_proof)

    def test_joint_commit_waits_for_close_before_slow_persistent_write(self):
        self.start('joints')
        for action, position in (('direction-start', 2000), ('direction-end', 2050),
                                 ('range-a', 1600), ('range-b', 2400)):
            self.capture_joint(action, position)
        def slow_commit(cfg, ledger):
            self.assertEqual(self.session.status, 'closed')
            self.assertTrue(self.session.connection_done.is_set())
            self.assertTrue(self.session.closed_proof['serial_closed'])
            self.assertEqual(self.session.process.poll(), 0)
            self.clock.now += 3.5
            self.session.check_watchdog()
            self.assertIsNone(self.session.error)
            return self.commit(cfg, ledger)
        self.session.calibration_writer = slow_commit
        self.save_after_reader_close('/api/joint-calibration', self.joint_body('save'))
        self.assertEqual(len(self.commits), 1)
        self.assertEqual(self.session.status, 'closed')
        self.assertEqual(self.session.joint_wizard.value['stage'], 'complete')
        self.assertEqual(self.session.config['imu'], self.original['imu'])
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])

    def test_joint_commit_without_actual_close_proof_does_not_write_or_relax_deadline(self):
        self.start('joints')
        self.capture_joint('direction-start', 2000)
        self.capture_joint('direction-end', 2050)
        def stalled_wait(condition, timeout):
            self.clock.now += 2.01
        with mock.patch.object(threading.Condition, 'wait', stalled_wait):
            with self.assertRaisesRegex(ValueError, '未收到正常关闭证明'):
                self.service.dispatch('/api/joint-calibration', self.joint_body('save-direction'))
        self.assertEqual(self.commits, [])
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.session.status, 'fault')
        self.assertIsNone(self.session.closed_proof)
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])

    def test_writer_failure_preserves_original_joint_configuration(self):
        self.start('joints')
        self.capture_joint('direction-start', 2000)
        self.capture_joint('direction-end', 2050)
        self.session.calibration_writer = mock.Mock(side_effect=OSError('disk full'))
        with self.assertRaisesRegex(ValueError, '保存失败'):
            self.save_after_reader_close('/api/joint-calibration', self.joint_body('save-direction'))
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(json.loads((self.directory / 'calibration-latest.json').read_text()), self.original)

    def test_motion_switch_waits_for_actual_normal_uart_and_child_close(self):
        self.start('joints')
        with self.assertRaisesRegex(ValueError, '关闭尚未确认'):
            self.service.end_for_motion(timeout=0)
        self.assertEqual(self.controls, [b'END_FEEDBACK\n'])
        self.session.accept(dict(kind='closed', generation=7, normal=True, serial_closed=True))
        self.session.process.returncode = 0
        self.session.confirm_closed(0)
        self.session.connection_done.set()
        self.assertEqual(self.service.end_for_motion(timeout=0), self.original)
        self.assertTrue(self.service.connection_closed())
        self.assertFalse(self.service.owns_connection)

    def test_motion_switch_after_unopened_reader_preflight_keeps_original_fault(self):
        reason = 'New kernel tail contains an untimestamped record'
        self.session.status = 'fault'
        self.session.error = reason
        self.session.error_details = reason
        self.session.recovery_recheck = dict(category='trust', message=reason,
                                             technical_reason=reason)
        self.session._paused_without_attempt = False
        self.session._reader_preflight_failure_generation = self.session.generation
        before = self.service.snapshot()['session']
        self.assertTrue(self.service.connection_closed())

        saved = self.service.end_for_motion(timeout=0)

        self.assertEqual(saved, self.original)
        saved['joints'][0]['min'] = 1
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.service.snapshot()['session'], before)
        self.assertIsNone(self.session.process)
        self.assertIsNone(self.session.closed_proof)
        self.assertFalse(self.session.end_request_attempted)
        self.assertEqual(self.controls, [])
        self.assertEqual(self.commits, [])

    def test_motion_switch_cannot_infer_unopened_reader_from_missing_process(self):
        self.session.status = 'fault'
        self.session._paused_without_attempt = False
        for marker in (None, self.session.generation - 1):
            with self.subTest(preflight_generation=marker):
                self.session._reader_preflight_failure_generation = marker
                self.assertFalse(self.service.connection_closed())
                with self.assertRaisesRegex(ValueError, '正常关闭证明'):
                    self.service.end_for_motion(timeout=0)
        self.assertEqual(self.controls, [])

    def test_motion_switch_preflight_marker_cannot_bypass_started_reader_closure(self):
        self.session._paused_without_attempt = False
        self.session._reader_preflight_failure_generation = self.session.generation
        self.session.process = Process()
        valid = dict(generation=self.session.generation, normal=True, serial_closed=True)
        cases = [('fault', 0, valid), ('closed', None, valid), ('closed', 1, valid),
                 ('closed', 0, {}), ('closed', 0, dict(valid, normal=False)),
                 ('closed', 0, dict(valid, serial_closed=False)),
                 ('closed', 0, dict(valid, generation=self.session.generation - 1))]
        for status, returncode, proof in cases:
            with self.subTest(status=status, returncode=returncode, proof=proof):
                self.session.status = status
                self.session.process.returncode = returncode
                self.session.closed_proof = proof
                with self.assertRaisesRegex(ValueError, '正常关闭证明'):
                    self.service.end_for_motion(timeout=0)
        self.assertEqual(self.controls, [])

    def test_motion_switch_unopened_preflight_still_waits_for_finished_attempt(self):
        self.session.status = 'fault'
        self.session._paused_without_attempt = False
        self.session._reader_preflight_failure_generation = self.session.generation
        for pending, transmitting in ((True, False), (False, True)):
            with self.subTest(recovery_pending=pending, source_transmitting=transmitting):
                self.session.recovery_pending = pending
                self.session.source_transmitting = transmitting
                with self.assertRaisesRegex(ValueError, '正常关闭证明'):
                    self.service.end_for_motion(timeout=0)
        self.session.recovery_pending = self.session.source_transmitting = False
        self.session.connection_done.clear()
        with self.assertRaisesRegex(ValueError, '关闭尚未确认'):
            self.service.end_for_motion(timeout=0)
        self.assertEqual(self.controls, [])

    def test_local_receipts_never_substitute_fake_ssh_exit_proof(self):
        proof = dict(kind='quiescent', generation=7, serial_path='/dev/ttyUSB0', serial_holders=[],
            identity_checked=True, guards_passed=True, observation_count=2, serial_open_attempts=0,
            serial_bytes_written=0, board_files_written=False, automatic_retry=False,
            transport='local', ssh_started=False, process_exit_code=0, process_exited=True)
        self.session._validate_local_quiescence(proof, 7)
        fake = dict(proof, ssh_exit_code=0, ssh_process_exited=True)
        fake.pop('process_exited')
        with self.assertRaises(ValueError):
            self.session._validate_local_quiescence(fake, 7)

    def test_stale_recovery_does_not_touch_owner_gate(self):
        gate = mock.Mock()
        self.service.before_acquire = gate
        with self.assertRaisesRegex(ValueError, '变化'):
            self.service.dispatch('/api/recover-feedback', dict(generation=6, purpose='imu'))
        gate.assert_not_called()

    def test_native_preflight_failure_can_recheck_without_fake_ssh_record(self):
        self.session.status = 'fault'
        self.session.recovery_recheck = dict(category='trust', message='identity failed')
        self.session._paused_without_attempt = False
        self.session._reader_preflight_failure_generation = 7
        self.assertIsNone(self.session.connection_check_blocked_reason())
        self.session._reader_preflight_failure_generation = None
        self.assertIn('未得到确认', self.session.connection_check_blocked_reason())


if __name__ == '__main__':
    unittest.main()
