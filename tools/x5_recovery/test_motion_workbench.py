"""No network/UART: deployment integrity, one-shot control and ownership tests."""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

import motion_workbench as app
import feedback_calibration_server as host
from transport_config import TransportConfig

sys.path.insert(0, str(app.ROOT / 'tools/x5_policy'))
import motion
from test_motion import FakeClock, FakeTransport, FakePolicy, ready_config


def controller_class():
    node = next(n for n in ast.walk(ast.parse(app.WORKER))
                if isinstance(n, ast.ClassDef) and n.name == 'OneShotController')
    namespace = dict(motion=motion, exit_event=threading.Event(), health_error=None, request_counts={})
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<one-shot-test>', 'exec'), namespace)
    return namespace['OneShotController'], namespace


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'calibration.json'
        self.path.write_text(json.dumps(ready_config()))
        self.clock = FakeClock()
        self.transport = FakeTransport('fake-only')
        cls, self.namespace = controller_class()
        self.controller = cls({'walk': FakePolicy()}, config_path=self.path,
                              clock=self.clock, transport_factory=lambda _: self.transport)

    def tearDown(self):
        self.controller._disconnect('test complete')
        self.temp.cleanup()

    def controller_with_all_policies(self):
        # Models and their validated metadata are loaded together at construction.
        self.controller = self.controller.__class__(
            {name: FakePolicy(name) for name in motion.MODEL_MODES},
            config_path=self.path, clock=self.clock,
            transport_factory=lambda _: self.transport)
        return self.controller

    def test_connect_only_stops_configures_and_reads_no_arm_or_targets(self):
        self.controller._connect_once()
        kinds = [k for k, _ in self.transport.requests]
        self.assertEqual([motion.STOP, motion.HELLO, motion.STATE, motion.CONFIG, motion.STATE], kinds)
        self.assertTrue(self.controller.status()['release_confirmed'])
        self.assertIsNone(self.controller.status()['start_blocked_reason'])
        self.assertEqual(ready_config(), json.loads(self.path.read_text()))

    def test_motion_requires_support_generation_and_valid_actual_positions(self):
        c = self.controller
        c._connect_once()
        g = c.status()['generation']
        for support, generation in ((False, g), (True, g-1)):
            with self.assertRaises(motion.MotionError):
                c.start_mode('hold', support_confirmed=support, expected_generation=generation)
        self.assertNotIn(motion.ARM, [k for k, _ in self.transport.requests])
        self.transport.positions[0] = 0
        c._cycle()
        self.assertIn('ID 10', c.status()['start_blocked_reason'])
        with self.assertRaises(motion.MotionError):
            c.start_mode('hold', support_confirmed=True, expected_generation=g)
        self.assertNotIn(motion.ARM, [k for k, _ in self.transport.requests])

    def test_explicit_hold_sends_targets_and_stop_proves_torque_off(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True, expected_generation=c.status()['generation'])
        self.clock.advance(.02); c.heartbeat(); c._cycle()
        self.assertTrue(any(k == motion.TARGET for k, _ in self.transport.requests))
        stopped = c.stop()
        self.assertTrue(stopped['release_confirmed'])
        self.assertFalse(stopped['armed'])
        self.assertEqual([], stopped['torque_ids'])
        self.assertEqual('off', stopped['mode'])

    def test_fixed_stance_is_owned_by_x5_without_browser_heartbeats(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        for _ in range(600):
            self.clock.advance(.02); c._cycle()
        status=c.status()
        self.assertEqual('hold', status['mode'])
        self.assertTrue(status['armed'])
        self.assertFalse(status['page_heartbeat_required'])
        self.assertEqual(2,status['hold_control_protocol'])
        self.assertIsNone(status['heartbeat_remaining_ms'])
        self.assertEqual(600,sum(k==motion.TARGET for k,_ in self.transport.requests))
        self.assertEqual(1,sum(k==motion.ARM for k,_ in self.transport.requests))
        c.stop();self.assertTrue(c.status()['release_confirmed'])

    def test_legacy_background_heartbeat_loss_does_not_end_fixed_stance(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c.heartbeat(background=True)
        for _ in range(65):
            self.clock.advance(1.)
            c._cycle()
            self.assertEqual('hold',c.status()['mode'])
            self.assertEqual(motion.MOTOR_IDS,c.status()['torque_ids'])
            c.heartbeat(background=True)
        self.clock.advance(motion.BACKGROUND_HOLD_HEARTBEAT_SECONDS+.01)
        c._cycle()
        self.assertEqual('hold',c.status()['mode'])
        self.assertTrue(c.status()['armed'])
        self.assertIsNone(c.status()['last_stop_diagnostic'])

    def test_background_hold_never_relaxes_servo_feedback_or_range_checks(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c.heartbeat(background=True)
        self.transport.positions[motion.MOTOR_IDS.index(11)]=0
        self.clock.advance(.02); c._cycle()
        self.assertEqual('off',c.status()['mode'])
        self.assertTrue(c.status()['release_confirmed'])
        self.assertIn('ID 11',c.status()['last_stop_diagnostic']['reason'])

    def test_foreground_changes_do_not_end_stance_and_stop_cannot_restart(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c.heartbeat(background=True)
        self.assertTrue(c.status()['background_hold'])
        c.heartbeat()
        self.assertFalse(c.status()['background_hold'])
        self.clock.advance(.5); c._cycle()
        self.assertEqual('hold',c.status()['mode'])
        c.stop()
        sent=len(self.transport.requests)
        c.heartbeat(background=True)
        self.assertEqual('off',c.status()['mode'])
        self.assertFalse(c.status()['background_hold'])
        self.assertEqual(sent,len(self.transport.requests))

    def test_background_hold_stops_on_frozen_hardware_feedback_before_lease_expires(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c.heartbeat(background=True)
        self.transport.freeze_sample=True
        self.clock.advance(.11); c._cycle()
        self.assertEqual('off',c.status()['mode'])
        self.assertIn('反馈',c.status()['last_stop_diagnostic']['reason'])

    def test_background_flag_does_not_extend_dynamic_model_deadline(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c._hold_begin=self.clock()-4
        c._filtered.angles=list(c._fixed_stance_locked())
        c.start_mode('walk',support_confirmed=True)
        c.heartbeat(background=True)
        self.assertFalse(c.status()['background_hold'])
        self.clock.advance(.5); c._cycle()
        self.assertEqual('off',c.status()['mode'])

    def test_stop_diagnostic_captures_active_target_and_torque_before_clearing(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        self.clock.advance(.02); c.heartbeat(); c._cycle()
        target = c.status()['last_action'][:]
        stopped = c.stop()
        evidence = stopped['last_stop_diagnostic']
        self.assertEqual(1, evidence['sequence'])
        self.assertIsNone(evidence['reason'])
        self.assertEqual('hold', evidence['before']['mode'])
        self.assertEqual(motion.MOTOR_IDS, evidence['before']['torque_ids'])
        self.assertEqual(target, evidence['before']['last_action'])
        self.assertIsNone(stopped['last_action'])
        stopped['last_stop_diagnostic']['before']['rawdata'][0]['position'] = -1
        self.assertNotEqual(-1, c.status()['last_stop_diagnostic']['before']['rawdata'][0]['position'])
        c.stop()
        self.assertEqual(evidence['sequence'], c.status()['last_stop_diagnostic']['sequence'])

    def test_slow_historical_status_copy_does_not_block_target_or_stop(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        entered, release, cycle_done = threading.Event(), threading.Event(), threading.Event()
        values, failures = [], []

        class SlowHistory(dict):
            def __deepcopy__(self, memo):
                entered.set()
                if not release.wait(2):
                    raise AssertionError('historical copy was never released')
                return copy.deepcopy(dict(self), memo)

        c._last_stop_diagnostic = SlowHistory(sequence=1, reason='prior stop')
        def display():
            try:
                values.append(c.status())
            except BaseException as error:
                failures.append(error)
        def control():
            try:
                self.clock.advance(.02)
                c._cycle()
            except BaseException as error:
                failures.append(error)
            finally:
                cycle_done.set()
        reader = threading.Thread(target=display)
        worker = threading.Thread(target=control)
        reader.start()
        try:
            self.assertTrue(entered.wait(1))
            worker.start()
            self.assertTrue(cycle_done.wait(.5), 'historical display copy blocked live control')
            self.assertEqual(1, sum(k == motion.TARGET for k, _ in self.transport.requests))
            stopped = c.stop()
            self.assertTrue(stopped['release_confirmed'])
            self.assertFalse(release.is_set(), 'STOP must finish before slow display resumes')
        finally:
            release.set()
            reader.join(2)
            if worker.ident is not None:
                worker.join(2)
        self.assertEqual([], failures)
        self.assertEqual(1, values[0]['last_stop_diagnostic']['sequence'])
        self.assertEqual(2, c.status()['last_stop_diagnostic']['sequence'])

    def test_hold_feedback_and_target_cannot_be_separated_by_display_lock(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        original = c._control_once
        phases = []
        def control_phase():
            phases.append(c._lock._is_owned())
            return original()
        with mock.patch.object(c, '_control_once', side_effect=control_phase):
            self.clock.advance(.02)
            c._cycle()
        self.assertEqual([True], phases)
        self.assertEqual(1, sum(k == motion.TARGET for k, _ in self.transport.requests))

    def test_stop_retains_transport_failure_before_stop_changes_the_request_history(self):
        c=self.controller;c._connect_once();c.start_mode('hold',support_confirmed=True)
        captured=dict(requests=[dict(kind=motion.TARGET,elapsed_ms=155.,target_attempt_gap_ms=300.)],
                      firmware_first_stop=dict(cause=6,target_age_us=250001))
        self.transport.timing_diagnostics=lambda:copy.deepcopy(captured)
        with c._lock:
            c._stop_locked('目标指令间隔超时')
            captured['requests'].append(dict(kind=motion.STOP))
            c._service_stop_locked()
        evidence=c.status()['last_stop_diagnostic']['transport']
        self.assertEqual([motion.TARGET],[item['kind'] for item in evidence['requests']])
        self.assertEqual(evidence['firmware_first_stop']['target_age_us'],250001)
        evidence['requests'].clear()
        self.assertEqual(len(c.status()['last_stop_diagnostic']['transport']['requests']),1)

    def test_first_browser_stop_source_survives_redundant_stop(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        stopped = c.stop(source='window-blur')
        evidence = stopped['last_stop_diagnostic']
        self.assertEqual('window-blur', evidence['source'])
        self.assertIsNone(evidence['reason'])
        self.assertEqual('hold', evidence['before']['mode'])
        self.assertTrue(stopped['release_confirmed'])
        c.stop(source='page-hidden')
        self.assertEqual(evidence, c.status()['last_stop_diagnostic'])

    def test_stop_readback_failure_keeps_original_heartbeat_trigger(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        c._hold_begin=self.clock()-4
        c._filtered.angles=list(c._fixed_stance_locked())
        c.start_mode('walk',support_confirmed=True)
        original = self.transport.request
        def unchanged(kind, payload=b''):
            if kind == motion.STOP:
                self.transport.requests.append((kind, payload))
                return bytes([kind, 0])
            return original(kind, payload)
        self.transport.request = unchanged
        self.clock.advance(.5)
        with self.assertRaisesRegex(motion.MotionError, '网页控制心跳超时.*尚未确认'):
            c._cycle()
        status = c.status()
        evidence = status['last_stop_diagnostic']
        self.assertEqual('网页控制心跳超时，已停止', evidence['reason'])
        self.assertEqual('walk', evidence['before']['mode'])
        self.assertEqual(0, evidence['before']['heartbeat_remaining_ms'])
        self.assertTrue(evidence['verification_failure']['after']['armed'])
        self.assertFalse(status['release_confirmed'])
        self.assertEqual(1, sum(kind == motion.ARM for kind, _ in self.transport.requests))
        self.assertNotIn(motion.TARGET, [kind for kind, _ in self.transport.requests])

    def test_stop_diagnostic_records_joint_range_trigger_before_release(self):
        c = self.controller
        c._connect_once()
        c.start_mode('hold', support_confirmed=True)
        self.transport.positions[1] = 4095
        self.clock.advance(.02); c.heartbeat(); c._cycle()
        status = c.status()
        evidence = status['last_stop_diagnostic']
        self.assertIn('ID 11 实际位置超出记录范围', evidence['reason'])
        self.assertEqual(4095, evidence['before']['rawdata'][1]['position'])
        self.assertEqual(motion.MOTOR_IDS, evidence['before']['torque_ids'])
        self.assertTrue(status['release_confirmed'])
        self.assertEqual(1, sum(kind == motion.ARM for kind, _ in self.transport.requests))
        self.assertNotIn(motion.TARGET, [kind for kind, _ in self.transport.requests])

    def test_arm_refusal_keeps_original_reason_and_confirms_stop_without_retry(self):
        c = self.controller
        c._connect_once()
        original = self.transport.request
        def refused(kind, payload=b''):
            if kind == motion.ARM:
                self.transport.requests.append((kind, payload))
                raise motion.MotionError('下位机反馈超时（指令 ARM，错误码 4）')
            return original(kind, payload)
        self.transport.request = refused
        with self.assertRaisesRegex(motion.MotionError, '错误码 4'):
            c.start_mode('hold', support_confirmed=True, expected_generation=c.status()['generation'])
        result = c.status()
        self.assertIn('错误码 4', result['error'])
        self.assertTrue(result['release_confirmed'])
        self.assertEqual('off', result['mode'])
        self.assertEqual(1, sum(k == motion.ARM for k, _ in self.transport.requests))
        self.assertNotIn(motion.TARGET, [k for k, _ in self.transport.requests])

    def test_stop_clears_previous_shadow_inference_display(self):
        self.controller._connect_once()
        self.controller._last_inference_ms = .6
        result = self.controller.stop()
        self.assertIsNone(result['inference_ms'])

    def test_per_model_readiness_does_not_treat_loaded_models_as_stand_ready(self):
        c=self.controller_with_all_policies()
        c._connect_once();status=c.status()
        self.assertEqual(status['behavior_protocol'],1)
        self.assertIsNone(status['model_start_blocked_reasons']['getup'])
        for name in ('walk','pick','roulade'):
            self.assertIn('先启动站姿',status['model_start_blocked_reasons'][name])
        self.assertNotIn(motion.ARM,[k for k,_ in self.transport.requests])

    def test_episode_end_uses_actual_stop_readback_without_an_automatic_mode_transition(self):
        c=self.controller_with_all_policies()
        c._connect_once();c.start_mode('hold',support_confirmed=True)
        for _ in range(155):self.clock.advance(.02);c.heartbeat();c._cycle()
        c.start_mode('pick',support_confirmed=True)
        for _ in range(202):self.clock.advance(.02);c.heartbeat();c._cycle()
        status=c.status();self.assertEqual(status['mode'],'off')
        self.assertTrue(status['release_confirmed'])
        self.assertEqual([],status['torque_ids'])
        self.assertIn('俯身本轮模型运行结束',status['behavior_result'])
        kinds=[k for k,_ in self.transport.requests]
        stopped_at=max(i for i,k in enumerate(kinds) if k==motion.STOP)
        self.assertEqual(motion.STATE,kinds[stopped_at+1])
        self.assertNotIn(motion.TARGET,kinds[stopped_at+1:])

    def test_episode_stop_failure_does_not_claim_release(self):
        c=self.controller_with_all_policies()
        c._connect_once();c.start_mode('getup',support_confirmed=True,behavior_confirmed=True)
        self.clock.advance(8.01);c.heartbeat()
        c._accept_state_locked(motion.decode_state(self.transport.request(motion.STATE)))
        original=self.transport.request
        def uncertain(kind,payload=b''):
            if kind==motion.STOP:
                self.transport.requests.append((kind,payload))
                return bytes([kind,0])  # No actual torque release; STATE proves it.
            return original(kind,payload)
        self.transport.request=uncertain
        with self.assertRaisesRegex(motion.MotionError,'尚未确认'):c._control_once()
        self.assertFalse(c.status()['release_confirmed'])
        self.assertTrue(self.transport.armed)

    def test_failed_stop_preserves_arm_refusal_and_does_not_reconnect(self):
        c = self.controller
        c._connect_once()
        def refused(kind, payload=b''):
            self.transport.requests.append((kind, payload))
            raise motion.MotionError('ARM 原始原因' if kind == motion.ARM else 'STOP 断开')
        self.transport.request = refused
        with self.assertRaisesRegex(motion.MotionError, 'ARM 原始原因'):
            c.start_mode('hold', support_confirmed=True)
        self.assertIn('ARM 原始原因', c.status()['error'])
        self.assertIn('STOP 断开', c.status()['error'])
        self.assertFalse(c.status()['connected'])
        self.assertTrue(self.transport.closed)

    def test_failed_stop_never_claims_release_or_reopens_uart(self):
        c = self.controller
        c._connect_once()
        original = self.transport.request
        def broken_stop(kind, payload=b''):
            if kind == motion.STOP:
                self.transport.requests.append((kind, payload))
                raise motion.MotionError('uncertain STOP acknowledgement')
            return original(kind, payload)
        self.transport.request = broken_stop
        result = c.stop()
        self.assertFalse(result['release_confirmed'])
        self.assertFalse(result['connected'])
        self.assertTrue(self.transport.closed)

    def test_worker_initial_connection_failure_has_one_attempt_and_no_retry(self):
        with mock.patch.object(self.controller, '_connect_once', side_effect=motion.MotionError('offline')) as connect:
            self.controller._run()
        connect.assert_called_once_with()
        self.assertEqual('offline', self.controller.status()['error'])
        self.assertFalse(self.controller.status()['connected'])
        self.assertTrue(self.namespace['exit_event'].is_set())

    def test_control_thread_failure_terminates_rpc_and_preserves_original_cause(self):
        reason='ID 11 目标超过已记录的机械范围'
        with mock.patch.object(self.controller,'_cycle',side_effect=motion.MotionError(reason)):
            self.controller._run()
        self.assertTrue(self.namespace['exit_event'].is_set())
        self.assertEqual(reason,self.controller._terminal_error)
        self.assertEqual(reason,self.controller.status()['error'])
        self.assertFalse(self.controller.status()['connected'])
        self.assertTrue(self.transport.closed)
        self.assertEqual(0,self.transport.torque)
        self.assertNotIn(motion.ARM,[k for k,_ in self.transport.requests])

    def test_worker_stop_rpc_never_reports_success_when_disconnected_or_release_unproved(self):
        node=next(n for n in ast.walk(ast.parse(app.WORKER)) if isinstance(n,ast.If)
                  and ast.unparse(n.test)=="action == 'stop'")
        ns=dict(controller=self.controller,action='stop',request={'stop_source':'button'},ProbeError=ValueError)
        self.controller._disconnect('原目标越界原因')
        before=self.controller.status()['error']
        with self.assertRaisesRegex(ValueError,before):
            exec(compile(ast.Module(body=[node],type_ignores=[]),'<stop-rpc-test>','exec'),ns)
        self.assertEqual(before,self.controller.status()['error'])
        self.assertNotIn('result',ns)
        self.controller._connect_once()
        original=self.transport.request
        def uncertain(kind,payload=b''):
            if kind==motion.STOP:
                raise motion.MotionError('停止读回超时')
            return original(kind,payload)
        self.transport.request=uncertain
        with self.assertRaisesRegex(ValueError,'停止读回超时'):
            exec(compile(ast.Module(body=[node],type_ignores=[]),'<stop-rpc-test>','exec'),ns)
        self.assertFalse(self.controller.status()['release_confirmed'])

    def test_state_read_accept_is_serialized_with_arm_and_stop(self):
        c = self.controller
        c._connect_once()
        original = self.transport.request
        held = []
        def checked(kind, payload=b''):
            if kind == motion.STATE:
                held.append(c._lock._is_owned())
            return original(kind, payload)
        self.transport.request = checked
        c._cycle()
        self.assertEqual([True], held)

    def test_normal_worker_shutdown_closes_serial_and_sends_no_motion(self):
        self.namespace['exit_event'].set()
        self.controller._run()
        self.assertTrue(self.transport.closed)
        self.assertIsNone(self.controller._transport)
        self.assertNotIn(motion.ARM, [k for k, _ in self.transport.requests])
        self.assertNotIn(motion.TARGET, [k for k, _ in self.transport.requests])


class DeploymentTests(unittest.TestCase):
    def test_new_connection_waits_for_closed_serial_and_ssh_exit_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            w=app.MotionWorkbench({},None,None,'/no-hosts',directory)
            w.channel=mock.Mock();w.phase='closed'
            w.closed_report=dict(serial_closed=True)
            self.assertTrue(w.snapshot()['owns_connection'])
            with self.assertRaisesRegex(ValueError,'已关闭'):
                w.begin_connect(ready_config(),lambda:None)
            w.closed_report['ssh_exit_code']=20
            self.assertFalse(w.snapshot()['owns_connection'])

    def test_ram_health_accepts_only_the_independently_reviewed_system_mount(self):
        import types
        node = next(n for n in ast.parse(app.WORKER).body if isinstance(n, ast.FunctionDef) and n.name == 'ram_health')
        mount = '26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'
        namespace = dict(health_snapshot=lambda _: {}, EXPECTED={},
                         DEPLOY={'config_mount_review': app.CONFIG_MOUNT_REVIEW},
                         ram_identity=None, parent_pid=os.getppid(), os=os, ProbeError=RuntimeError)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<ram-health-test>','exec'),namespace)
        for rows, allowed in (([mount,*app.CONFIG_MOUNT_REVIEW],True),([mount],False),
                              ([mount,app.CONFIG_MOUNT_REVIEW[0].replace('425 21','426 21')],False),
                              ([mount.replace('rw,nosuid','rw,noexec,nosuid'),*app.CONFIG_MOUNT_REVIEW],False)):
            namespace['ram_identity']=None
            namespace['bounded_read']=lambda *_args, rows=rows: '\n'.join(rows)
            with mock.patch.object(os,'stat',return_value=types.SimpleNamespace(st_dev=22,st_ino=30)):
                if allowed: namespace['ram_health']()
                else:
                    with self.assertRaises(RuntimeError): namespace['ram_health']()

    def test_config_mount_is_bound_to_each_reviewed_boot_and_never_learns_a_mount(self):
        import feedback_stream
        self.assertEqual(app.config_mount_for_boot(feedback_stream.REBOOT_BOOT_ID), app.CONFIG_MOUNT_REVIEW)
        actual = app.config_mount_for_boot(feedback_stream.COLD_RECONNECT_BOOT_ID)
        self.assertEqual(actual, app.COLD_CONFIG_MOUNT_REVIEW)
        self.assertNotEqual(actual, app.CONFIG_MOUNT_REVIEW)
        actual.clear()
        self.assertTrue(app.COLD_CONFIG_MOUNT_REVIEW)
        wifi = app.config_mount_for_boot(feedback_stream.WIFI_RECONNECT_BOOT_ID)
        self.assertEqual(wifi, app.COLD_CONFIG_MOUNT_REVIEW)
        wifi.clear()
        self.assertTrue(app.COLD_CONFIG_MOUNT_REVIEW)
        imu_recovery = app.config_mount_for_boot(feedback_stream.IMU_RECOVERY_RECONNECT_BOOT_ID)
        self.assertEqual(imu_recovery, app.IMU_RECOVERY_CONFIG_MOUNT_REVIEW)
        self.assertIn('shared:224 ', imu_recovery[0])
        imu_recovery.clear()
        self.assertEqual(1, len(app.IMU_RECOVERY_CONFIG_MOUNT_REVIEW))
        intentional = app.config_mount_for_boot(feedback_stream.INTENTIONAL_POWERCYCLE_BOOT_ID)
        self.assertEqual(intentional, app.INTENTIONAL_POWERCYCLE_CONFIG_MOUNT_REVIEW)
        self.assertIn('shared:189 ', intentional[0])
        intentional.clear()
        self.assertEqual(1, len(app.INTENTIONAL_POWERCYCLE_CONFIG_MOUNT_REVIEW))
        with self.assertRaises(ValueError): app.config_mount_for_boot('unreviewed-boot')

    def test_bundle_contains_exact_models_and_saved_calibration_without_changes(self):
        base = app.ROOT / 'output/x5_recovery/replacement-board-20261004T011959+0800'
        source = base / 'feedback-calibration-20261005-center-holding-ready-01/calibration-latest.json'
        raw = source.read_bytes(); config = json.loads(raw)
        data, deploy = app.prepare_bundle(config)
        members = app.archive_tools.inspect_tar(data, deploy['files'])
        self.assertEqual(raw, members['policy/calibration.json'])
        self.assertEqual('2e005fdf5156195946df0afb20c5b7372d1a0014d20a17b820297658d6e976eb', deploy['calibration_sha256'])
        self.assertEqual(raw, source.read_bytes())
        for name, digest in app.MODEL_HASHES.items():
            self.assertEqual(digest, hashlib.sha256(members[f'policy/models/xgoduck_{name}.onnx']).hexdigest())
        expected = json.loads((source.parent / 'expectations.json').read_text())
        _, runtime = app.prepare_source(expected, base/'feedback-maintenance-center-holding-reviewed-20261005-01.json',
            '3071e1cfab266ef8764eab9e856bc086a379425933c578ed884523b74d2da6b7', deploy)
        self.assertLess(len(runtime), 262144)
        self.assertFalse(deploy['automatic_motor_start'])
        self.assertNotIn('policy/web_app.py', members)

    def test_constructor_and_status_cannot_open_hardware(self):
        with mock.patch.object(app.maintenance_transport, '_ProcessChannel', side_effect=AssertionError('hardware forbidden')):
            workbench = app.MotionWorkbench({}, None, None, '/no-hosts', '/no-directory')
            self.assertEqual('idle', workbench.snapshot()['phase'])
            self.assertFalse(workbench.snapshot()['owns_connection'])

    def test_rejected_user_start_is_persisted_with_before_and_after_and_sent_once(self):
        with tempfile.TemporaryDirectory() as directory:
            w = app.MotionWorkbench({}, None, None, '/no-hosts', directory)
            w.attempt = Path(directory)
            w.phase = 'ready'
            w.hardware = dict(mode='off',generation=1,torque_ids=[])
            sent = []
            class Channel:
                def send(self, request, _timeout):
                    sent.append(request)
                    pending = w.pending[request['request_id']]
                    pending['value'] = dict(error='ARM 错误码 4',hardware=dict(mode='off',generation=2,release_confirmed=True))
                    pending['event'].set()
            w.channel = Channel()
            with self.assertRaisesRegex(ValueError, '错误码 4'):
                w.action(dict(action='start',mode='hold',generation=1,support_confirmed=True,control_id='a'*32))
            self.assertEqual(1,len(sent))
            record = json.loads(next(Path(directory).glob('operation-*.json')).read_text())
            self.assertEqual('failed',record['state'])
            self.assertEqual('ARM 错误码 4',record['error'])
            self.assertEqual(1,record['before']['generation'])
            self.assertTrue(record['after']['release_confirmed'])
            self.assertEqual(record,w.snapshot()['last_operation'])
            self.assertEqual({},w.pending)


class LostPipeTests(unittest.TestCase):
    """A dead local SSH child never becomes proof of remote torque release."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        from test_feedback_connection_check import fixture, PROFILE, PROFILE_SHA
        expected,source=fixture()
        self.expected,self.source=expected,source
        self.workbench=app.MotionWorkbench(expected,PROFILE,PROFILE_SHA,'/no-hosts',self.temp.name)
        self.workbench.attempt=Path(self.temp.name)/'attempt'
        self.workbench.attempt.mkdir()
        self.workbench.remote_generation=42
        self.workbench.phase='ready'
        self.workbench.hardware=dict(connected=True,mode='off',generation=1,release_confirmed=True)
        self.workbench.updated_at=app.time.monotonic()
        self.workbench.reader_done.clear()
        self.records=[]
        self.sent=[]
        owner=self
        class Process:
            returncode=255
            def poll(self): return self.returncode
            def wait(self,timeout): return self.returncode
        class Channel:
            process=Process()
            stderr=b'client_loop: send disconnect: Connection reset by peer\n'
            def receive(self,deadline):
                if owner.records:return owner.records.pop(0)
                raise EOFError('Maintenance SSH output closed')
            def send(self,request,timeout):owner.sent.append(request)
            def finish(self):
                return dict(process_exited=True,ssh_exit_code=255,stderr=self.stderr.decode())
        self.workbench.channel=Channel()

    def test_eof_retains_exit_and_stderr_then_stop_never_writes_dead_pipe(self):
        w=self.workbench
        pending=dict(event=threading.Event(),value=None);w.pending['pending-stop']=pending
        w._read()
        self.assertTrue(w.reader_done.is_set())
        self.assertTrue(pending['event'].is_set())
        self.assertIsNone(pending['value']['hardware'])
        self.assertTrue(w.snapshot()['owns_connection'])
        self.assertIsNone(w.snapshot()['hardware'])
        self.assertIsNone(w.closed_report)
        record=json.loads((w.attempt/'transport-exit.json').read_text())
        self.assertEqual(record['ssh_exit_code'],255)
        self.assertIn('Connection reset by peer',record['stderr'])
        self.assertFalse(record['remote_closure_inferred'])
        with self.assertRaisesRegex(ValueError,'指令未发送'):
            w.action(dict(action='stop'))
        self.assertEqual([],self.sent)
        self.assertEqual('Maintenance SSH output closed',w.error)

    def test_remote_fault_survives_later_eof(self):
        self.records=[dict(kind='fault',deployment_generation=42,reason='STATE CRC mismatch')]
        self.workbench._read()
        self.assertEqual('STATE CRC mismatch',self.workbench.error)
        self.assertEqual('STATE CRC mismatch',self.workbench.transport_exit['original_error'])
        self.assertEqual('Maintenance SSH output closed',self.workbench.transport_exit['reader_error'])
        self.assertTrue((self.workbench.attempt/'remote-fault.json').exists())

    def test_fault_persists_stop_trigger_and_later_verification_failure(self):
        evidence=dict(sequence=1,reason='ID 11 实际位置超出记录范围',
                      before=dict(mode='hold',torque_ids=motion.MOTOR_IDS,rawdata=[dict(id=11,position=2200)]))
        failed=copy.deepcopy(evidence)
        failed['verification_failure']=dict(reason='停止回读未确认',after=dict(armed=True))
        self.records=[dict(kind='status',deployment_generation=42,
            hardware=dict(generation=2,last_stop_diagnostic=evidence)),
            dict(kind='fault',deployment_generation=42,reason='ID 11 实际位置超出记录范围；停止回读未确认',
                 last_stop_diagnostic=failed)]
        self.workbench._read()
        recorded=json.loads((self.workbench.attempt/'last-stop-diagnostic.json').read_text())
        self.assertEqual(failed,recorded)
        self.assertEqual(2200,recorded['before']['rawdata'][0]['position'])
        self.assertIn('ID 11',self.workbench.error)
        self.assertTrue(self.workbench.snapshot()['owns_connection'])
        self.assertEqual([],self.sent)

    def test_explicit_disconnect_requires_quiescence_without_inventing_closed(self):
        from test_feedback_connection_check import remote_proof
        w=self.workbench;w._read()
        proof=remote_proof(self.expected)
        proof.update(ssh_exit_code=0,ssh_process_exited=True,
                     source_sha256=hashlib.sha256(self.source).hexdigest())
        with mock.patch.object(app.feedback_connection_check,'check_connection',return_value=proof) as check:
            result=w.action(dict(action='disconnect'))
        self.assertEqual(1,check.call_count)
        self.assertFalse(result['owns_connection'])
        self.assertTrue(result['exit_reconciled'])
        self.assertEqual('closed',result['phase'])
        self.assertIsNone(result['closed_proof'])
        self.assertEqual([],self.sent)
        record=json.loads((w.attempt/'disconnect-reconciled.json').read_text())
        self.assertFalse(record['old_remote_closure_inferred'])
        self.assertFalse(record['motor_stop_confirmed'])
        self.assertEqual(record['serial_open_attempts'],0)
        self.assertEqual('Maintenance SSH output closed',w.error)

    def test_failed_disconnect_check_keeps_ownership_and_original_fault(self):
        w=self.workbench;w._read()
        with mock.patch.object(app.feedback_connection_check,'check_connection',side_effect=ValueError('UART still held')):
            with self.assertRaisesRegex(ValueError,'UART still held'):
                w.action(dict(action='disconnect'))
        self.assertTrue(w.snapshot()['owns_connection'])
        self.assertFalse(w.exit_reconciled)
        self.assertIsNone(w.closed_report)
        self.assertEqual([],self.sent)
        self.assertEqual('Maintenance SSH output closed',w.error)

    def test_broken_pipe_is_value_error_for_the_page_and_is_not_retried(self):
        w=self.workbench
        w.channel.process.returncode=None
        w.channel.send=mock.Mock(side_effect=BrokenPipeError(32,'Broken pipe'))
        with self.assertRaisesRegex(ValueError,'不会重发'):
            w.action(dict(action='stop',stop_source='button'))
        self.assertEqual(w.channel.send.call_count,1)
        self.assertEqual('fault',w.phase)
        self.assertIsNone(w.snapshot()['hardware'])
        self.assertEqual({},w.pending)

    def test_reconciliation_record_failure_does_not_release_ownership(self):
        from test_feedback_connection_check import remote_proof
        w=self.workbench;w._read()
        proof=remote_proof(self.expected)
        proof.update(ssh_exit_code=0,ssh_process_exited=True,
                     source_sha256=hashlib.sha256(self.source).hexdigest())
        with mock.patch.object(app.feedback_connection_check,'check_connection',return_value=proof), \
                mock.patch.object(w,'record',side_effect=OSError('evidence write failed')):
            with self.assertRaisesRegex(ValueError,'evidence write failed'):
                w.action(dict(action='disconnect'))
        self.assertTrue(w.snapshot()['owns_connection'])
        self.assertFalse(w.exit_reconciled)

    def test_service_close_never_launches_a_metadata_check_or_reopens_a_dead_pipe(self):
        w=self.workbench;w._read()
        with mock.patch.object(app.feedback_connection_check,'check_connection',side_effect=AssertionError('no automatic check')):
            w.close()
        self.assertTrue(w.snapshot()['owns_connection'])
        self.assertEqual([],self.sent)


class RestoredMotionTests(unittest.TestCase):
    """Private local evidence restores a hold, never a channel or STOP proof."""
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.parent = Path(self.temp.name).resolve()
        self.parent.chmod(0o700)
        self.source = self.parent / ('motion-' + 'a' * 16)
        self.source.mkdir(mode=0o700)
        self.destination = self.parent / 'new-run'
        self.destination.mkdir(mode=0o700)
        from test_feedback_connection_check import fixture, remote_proof, PROFILE, PROFILE_SHA
        self.expected, self.check_source = fixture()
        self.profile, self.profile_sha = PROFILE, PROFILE_SHA
        source_expected = copy.deepcopy(self.expected)
        source_expected.pop('reviewed_reconnect_proof', None)
        self.parent_expected = source_expected
        self.parent_record = dict(source_path=str(PROFILE), sha256=PROFILE_SHA, copy_sha256=PROFILE_SHA,
                                  applied_fields=['kernel_prefix', 'historical_faults', 'boot_id'],
                                  boot_generation_changed=True, original_boot_id=app.feedback_stream.REVIEWED_BOOT_ID,
                                  reviewed_boot_id=self.expected['boot_id'], board_identity_changed=False, calibration_changed=False)
        app.maintenance_transport.private_new(self.parent / 'expectations.json', json.dumps(source_expected) + '\n')
        app.maintenance_transport.private_new(self.parent / 'reviewed-baseline-record.json', json.dumps(self.parent_record) + '\n')
        app.maintenance_transport.private_new(self.parent / 'reviewed-baseline.json', PROFILE.read_bytes())
        self.config = ready_config()
        calibration = app.calibration_bytes(self.config)
        digest = hashlib.sha256(calibration).hexdigest()
        files = {'policy/calibration.json': dict(size_bytes=len(calibration), sha256=digest)}
        files.update({f'policy/models/xgoduck_{name}.onnx': dict(size_bytes=1024, sha256=sha)
                      for name, sha in app.MODEL_HASHES.items()})
        deployment = dict(archive_sha256='1' * 64, archive_size=2048, scope='explicit_web_motion',
                          automatic_motor_start=False, config_mount_review=app.config_mount_for_boot(self.expected['boot_id']),
                          persistent_model_writes=False, member_bytes=4096, wheel_bytes=8192,
                          calibration_sha256=digest, model_hashes=copy.deepcopy(app.MODEL_HASHES),
                          files=files, started_monotonic=1.)
        exited = dict(process_exited=True, ssh_exit_code=255, stderr='Connection reset by peer',
                      original_error='Maintenance SSH output closed', reader_error='Maintenance SSH output closed',
                      closed_record_received=False, remote_closure_inferred=False, automatic_retry=False)
        self.values = {
            'ready.json': dict(kind='ready', deployment_generation=42, host='X5', runtime='CPUExecutionProvider',
                               ram_directory='/dev/shm/xgoduck-motion-test', calibration_sha256=digest,
                               model_hashes=copy.deepcopy(app.MODEL_HASHES), hardware=dict(connected=True, generation=1),
                               request_counts={'6': 1}, motor_start_attempted=False),
            'transfer-result.json': dict(kind='transfer_ready', deployment_generation=42, archive_bytes=2048,
                                         identity=remote_proof(self.expected)['identity'], ram_directory='/dev/shm/xgoduck-motion-test'),
            'deployment.json': deployment,
            'transport-exit.json': exited,
            'live-state.json': dict(recorded_monotonic=2., phase='fault', error='Maintenance SSH output closed',
                                    host='X5', ram_only=True, owns_connection=True, feedback_age_ms=None,
                                    hardware=None, deployment=copy.deepcopy(deployment), closed_proof=None,
                                    last_operation=None, transport_exit=copy.deepcopy(exited), exit_reconciled=False),
        }
        self.write_values(self.values)
        self.network = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        self.workbench = app.MotionWorkbench(self.expected, self.profile, self.profile_sha, '/no-hosts',
                                            self.destination, transport_config=self.network)

    def write_values(self, values):
        for name, value in values.items():
            path = self.source / name
            path.write_text(json.dumps(value, allow_nan=False) + '\n')
            path.chmod(0o600)

    def proof(self):
        from test_feedback_connection_check import remote_proof
        proof = remote_proof(self.expected)
        proof.update(ssh_exit_code=0, ssh_process_exited=True,
                     source_sha256=hashlib.sha256(self.check_source).hexdigest())
        return proof

    def test_restore_is_offline_and_preserves_uncertain_ownership_and_original_files(self):
        before = {name: (self.source / name).read_bytes() for name in self.values}
        w = self.workbench
        with mock.patch.object(app.maintenance_transport, '_ProcessChannel', side_effect=AssertionError('no SSH')), \
                mock.patch.object(app.feedback_connection_check, 'check_connection', side_effect=AssertionError('no check')), \
                mock.patch.object(app, 'prepare_bundle', side_effect=AssertionError('no deploy')):
            result = w.restore_uncertain_motion(self.source, self.config)
            self.assertEqual('fault', result['phase'])
            self.assertTrue(result['owns_connection'])
            self.assertTrue(w.uncertain_owner)
            self.assertTrue(w.reader_done.is_set())
            self.assertTrue(w._pipe_exited())
            self.assertIsNone(w.channel)
            self.assertIsNone(result['hardware'])
            self.assertIsNone(result['closed_proof'])
            self.assertEqual(42, w.remote_generation)
            self.assertEqual(255, result['transport_exit']['ssh_exit_code'])
            self.assertEqual('Maintenance SSH output closed', result['error'])
            for action in ('stop', 'start', 'command', 'heartbeat'):
                with self.subTest(action=action), self.assertRaisesRegex(ValueError, '指令未发送'):
                    w.action(dict(action=action))
            with self.assertRaisesRegex(ValueError, '已关闭'):
                w.begin_connect(self.config, lambda: self.fail('before-connect invoked'))
            w.close()
        self.assertEqual(before, {name: (self.source / name).read_bytes() for name in self.values})
        self.assertNotEqual(w.attempt, self.source)
        receipt = json.loads((w.attempt / 'resume-source.json').read_text())
        self.assertTrue(receipt['restored_uncertain_owner'])
        self.assertFalse(receipt['motor_stop_confirmed'])
        self.assertFalse(receipt['old_remote_closure_inferred'])
        self.assertEqual(0, receipt['serial_open_attempts'])
        self.assertFalse((w.attempt / 'closed.json').exists())

    def test_only_explicit_successful_metadata_disconnect_releases_imported_owner(self):
        w = self.workbench
        w.restore_uncertain_motion(self.source, self.config)
        with mock.patch.object(app.feedback_connection_check, 'check_connection', return_value=self.proof()) as check:
            result = w.action(dict(action='disconnect'))
        self.assertEqual(1, check.call_count)
        self.assertIs(check.call_args.kwargs['transport_config'], self.network)
        self.assertFalse(result['owns_connection'])
        self.assertTrue(result['exit_reconciled'])
        self.assertEqual('closed', result['phase'])
        self.assertIsNone(result['closed_proof'])
        self.assertIsNone(w.channel)
        receipt = json.loads((w.attempt / 'disconnect-reconciled.json').read_text())
        self.assertFalse(receipt['motor_stop_confirmed'])
        self.assertFalse(receipt['old_closed_record_received'])
        self.assertFalse(receipt['old_remote_closure_inferred'])
        self.assertFalse((self.source / 'disconnect-reconciled.json').exists())

    def test_failed_metadata_disconnect_keeps_imported_owner_and_original_error(self):
        w = self.workbench
        w.restore_uncertain_motion(self.source, self.config)
        with mock.patch.object(app.feedback_connection_check, 'check_connection', side_effect=ValueError('UART still held')):
            with self.assertRaisesRegex(ValueError, 'UART still held'):
                w.action(dict(action='disconnect'))
        self.assertTrue(w.snapshot()['owns_connection'])
        self.assertFalse(w.exit_reconciled)
        self.assertTrue(w._pipe_exited())
        self.assertEqual('Maintenance SSH output closed', w.error)

    def test_restore_rejects_mismatched_board_boot_card_root_serial_model_calibration_and_generation(self):
        mutations = [
            ('transfer-result.json', ('identity', 'boot_id'), 'wrong-boot'),
            ('transfer-result.json', ('identity', 'cid'), 'wrong-card'),
            ('transfer-result.json', ('identity', 'root_mount_line'), 'wrong-root'),
            ('transfer-result.json', ('identity', 'model'), 'wrong-board'),
            ('transfer-result.json', ('identity', 'serial', 'sysfs_name'), '1-1.9'),
            ('ready.json', ('model_hashes', 'walk'), '0' * 64),
            ('ready.json', ('calibration_sha256',), '0' * 64),
            ('ready.json', ('deployment_generation',), True),
            ('transfer-result.json', ('deployment_generation',), 43),
            ('transfer-result.json', ('archive_bytes',), 1),
            ('transfer-result.json', ('ram_directory',), '/dev/shm/xgoduck-motion-other'),
            ('deployment.json', ('files', 'policy/calibration.json', 'sha256'), '0' * 64),
            ('live-state.json', ('deployment', 'archive_sha256'), '0' * 64),
            ('live-state.json', ('deployment', 'persistent_model_writes'), 0),
            ('live-state.json', ('transport_exit', 'ssh_exit_code'), 0),
            ('live-state.json', ('transport_exit', 'process_exited'), 1),
            ('live-state.json', ('exit_reconciled',), True),
            ('live-state.json', ('owns_connection',), False),
            ('transport-exit.json', ('process_exited',), False),
            ('transport-exit.json', ('ssh_exit_code',), True),
            ('transport-exit.json', ('closed_record_received',), True),
            ('transport-exit.json', ('remote_closure_inferred',), True),
            ('transport-exit.json', ('automatic_retry',), True),
        ]
        for filename, keys, replacement in mutations:
            with self.subTest(filename=filename, keys=keys):
                values = copy.deepcopy(self.values)
                current = values[filename]
                for key in keys[:-1]:
                    current = current[key]
                current[keys[-1]] = replacement
                self.write_values(values)
                with self.assertRaises(ValueError):
                    self.workbench.restore_uncertain_motion(self.source, self.config)
                self.assertEqual('idle', self.workbench.phase)
                self.assertIsNone(self.workbench.attempt)
                self.assertFalse(self.workbench._owns_connection())
        self.write_values(self.values)
        other_config = copy.deepcopy(self.config)
        other_config['joints'][0]['zero'] += 1
        with self.assertRaises(ValueError):
            self.workbench.restore_uncertain_motion(self.source, other_config)

    def test_restore_requires_private_nonlinked_complete_and_unreleased_evidence(self):
        path = self.source / 'transport-exit.json'
        path.chmod(0o644)
        with self.assertRaises(ValueError):
            self.workbench.restore_uncertain_motion(self.source, self.config)
        path.chmod(0o600)
        saved = self.parent / 'saved-exit.json'
        path.rename(saved)
        path.symlink_to(saved)
        with self.assertRaises(OSError):
            self.workbench.restore_uncertain_motion(self.source, self.config)
        path.unlink(); saved.rename(path)
        for name in ('closed.json', 'disconnect-reconciled.json'):
            with self.subTest(name=name):
                (self.source / name).write_text('{}')
                with self.assertRaisesRegex(ValueError, '关闭或释放'):
                    self.workbench.restore_uncertain_motion(self.source, self.config)
                (self.source / name).unlink()
        path.write_text('{"process_exited":true,"process_exited":false}')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON'):
            self.workbench.restore_uncertain_motion(self.source, self.config)

    def test_imported_exit_receipt_cannot_be_substituted_and_restore_is_initialization_only(self):
        w = self.workbench
        w.restore_uncertain_motion(self.source, self.config)
        w.transport_exit['ssh_exit_code'] = 0
        self.assertFalse(w._pipe_exited())
        self.assertTrue(w._owns_connection())
        with self.assertRaisesRegex(ValueError, '初始化'):
            w.restore_uncertain_motion(self.source, self.config)

    def test_real_run15_receipt_uses_reviewed_profile_and_existing_discovery_policy_offline(self):
        source_run = app.ROOT / 'output/x5_recovery/replacement-board-20261004T011959+0800/feedback-calibration-20261005-check-retry-15'
        expected = json.loads((source_run / 'expectations.json').read_text())
        baseline = json.loads((source_run / 'reviewed-baseline-record.json').read_text())
        config = json.loads((source_run / 'calibration-latest.json').read_text())
        w = app.MotionWorkbench(expected, baseline['source_path'], baseline['sha256'], '/no-hosts',
                                self.destination, transport_config=self.network)
        with mock.patch.object(app.maintenance_transport, '_ProcessChannel', side_effect=AssertionError('no SSH')), \
                mock.patch.object(app.feedback_connection_check, 'check_connection', side_effect=AssertionError('no check')):
            result = w.restore_uncertain_motion(source_run / 'motion-c16d0aa99c95c598', config)
        self.assertEqual('fault', result['phase'])
        self.assertTrue(result['owns_connection'])
        self.assertTrue(w._pipe_exited())
        self.assertEqual(848932496, w.remote_generation)
        self.assertEqual(255, result['transport_exit']['ssh_exit_code'])
        self.assertIsNone(w.channel)
        self.assertIsNone(result['closed_proof'])
        self.assertEqual('1-1.3', w.expected['serial']['sysfs_name'])

    def test_old_owner_survives_a_different_fixed_reviewed_boot_until_new_metadata_check(self):
        profile_path = app.feedback_stream.EVIDENCE_DIR / 'feedback-cold-reconnect-reviewed-20261005-01.json'
        profile_sha = app.feedback_stream.COLD_RECONNECT_PROFILE_SHA256
        profile = app.feedback_stream.load_reviewed_profile(profile_path, profile_sha)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected.update(generation=42, discover_serial=True)
        w = app.MotionWorkbench(expected, profile_path, profile_sha, '/no-hosts', self.destination,
                                transport_config=self.network)
        self.assertNotEqual(self.parent_expected['boot_id'], expected['boot_id'])
        with mock.patch.object(app.feedback_connection_check, 'check_connection', side_effect=ValueError('new boot UART held')):
            restored = w.restore_uncertain_motion(self.source, self.config)
            self.assertTrue(restored['owns_connection'])
            self.assertTrue(w._pipe_exited())
            self.assertEqual('fault', restored['phase'])
            self.assertIsNone(w.channel)
            self.assertIsNone(restored['closed_proof'])
            with self.assertRaisesRegex(ValueError, 'new boot UART held'):
                w.action(dict(action='disconnect'))
            self.assertTrue(w._owns_connection())
        with mock.patch.object(app.feedback_connection_check, 'check_connection', return_value=self.proof()):
            with self.assertRaisesRegex(ValueError, 'identity differs'):
                w.action(dict(action='disconnect'))
            self.assertTrue(w._owns_connection())
        from test_feedback_connection_check import remote_proof
        prepared, source = app.feedback_connection_check.prepare_source(expected, profile_path, profile_sha)
        proof = remote_proof(prepared)
        proof.update(ssh_exit_code=0, ssh_process_exited=True, source_sha256=hashlib.sha256(source).hexdigest())
        with mock.patch.object(app.feedback_connection_check, 'check_connection', return_value=proof) as check:
            released = w.action(dict(action='disconnect'))
        self.assertEqual(expected['boot_id'], check.call_args.args[0]['boot_id'])
        self.assertEqual(42, check.call_args.args[0]['generation'])
        self.assertEqual(profile_sha, check.call_args.kwargs['reviewed_profile_sha256'])
        self.assertFalse(released['owns_connection'])
        self.assertTrue(released['exit_reconciled'])
        self.assertIsNone(released['closed_proof'])
        receipt = json.loads((w.attempt / 'resume-source.json').read_text())
        self.assertEqual(self.parent_expected['boot_id'], receipt['old_boot_id'])
        self.assertEqual(expected['boot_id'], receipt['current_reviewed_boot_id'])
        self.assertFalse(receipt['motor_stop_confirmed'])

    def test_unreviewed_current_boot_cannot_restore_or_clear_the_old_owner(self):
        expected = copy.deepcopy(self.expected)
        expected['boot_id'] = '11111111-2222-4333-8444-555555555555'
        w = app.MotionWorkbench(expected, self.profile, self.profile_sha, '/no-hosts', self.destination)
        with self.assertRaises(ValueError):
            w.restore_uncertain_motion(self.source, self.config)
        self.assertEqual('idle', w.phase)
        self.assertIsNone(w.channel)
        self.assertIsNone(w.attempt)

    def test_real_c16_owner_requires_new_fixed_wifi_boot_proof_after_restore(self):
        old_run = app.feedback_stream.EVIDENCE_DIR / 'feedback-calibration-20261005-check-retry-15'
        profile_path = app.feedback_stream.EVIDENCE_DIR / 'feedback-wifi-reboot-reviewed-20261006-01.json'
        profile_sha = app.feedback_stream.WIFI_RECONNECT_PROFILE_SHA256
        profile = app.feedback_stream.load_reviewed_profile(profile_path, profile_sha)
        current = {key: copy.deepcopy(profile[key]) for key in
                   ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        current.update(generation=43, discover_serial=True)
        config = json.loads((old_run / 'calibration-latest.json').read_text())
        w = app.MotionWorkbench(current, profile_path, profile_sha, '/no-hosts', self.destination,
                                transport_config=self.network)
        with mock.patch.object(app.maintenance_transport, '_ProcessChannel', side_effect=AssertionError('no SSH')), \
                mock.patch.object(app.feedback_connection_check, 'check_connection', side_effect=AssertionError('no automatic check')):
            result = w.restore_uncertain_motion(old_run / 'motion-c16d0aa99c95c598', config)
        self.assertEqual('f2070288-4492-4ddc-9594-ad00640d01b7', w.expected['boot_id'])
        self.assertEqual(848932496, w.remote_generation)
        self.assertTrue(result['owns_connection'])
        self.assertTrue(w._pipe_exited())
        self.assertIsNone(w.channel)
        self.assertIsNone(result['closed_proof'])
        provenance = json.loads((w.attempt / 'resume-source.json').read_text())
        self.assertEqual(app.feedback_stream.COLD_RECONNECT_BOOT_ID, provenance['old_boot_id'])
        self.assertEqual(app.feedback_stream.WIFI_RECONNECT_BOOT_ID, provenance['current_reviewed_boot_id'])
        self.assertEqual('b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                         provenance['old_review']['source_profile_sha256'])
        from test_feedback_connection_check import remote_proof
        check_expected = dict(current, generation=w.remote_generation)
        prepared, source = app.feedback_connection_check.prepare_source(check_expected, profile_path, profile_sha)
        proof = remote_proof(prepared)
        proof.update(generation=w.remote_generation, ssh_exit_code=0, ssh_process_exited=True,
                     source_sha256=hashlib.sha256(source).hexdigest())
        with mock.patch.object(app.feedback_connection_check, 'check_connection', return_value=proof) as check:
            released = w.action(dict(action='disconnect'))
        self.assertEqual(app.feedback_stream.WIFI_RECONNECT_BOOT_ID, check.call_args.args[0]['boot_id'])
        self.assertIs(check.call_args.kwargs['transport_config'], self.network)
        self.assertFalse(released['owns_connection'])
        self.assertTrue(released['exit_reconciled'])
        self.assertIsNone(released['closed_proof'])
        self.assertFalse((w.attempt / 'closed.json').exists())

    def test_source_parent_review_copy_expectations_and_digest_tampering_are_rejected(self):
        for key, value in (('sha256', '0' * 64), ('copy_sha256', '0' * 64),
                           ('source_path', '/tmp/unreviewed-profile.json'), ('board_identity_changed', True),
                           ('reviewed_boot_id', 'unreviewed-boot'), ('original_boot_id', 'unreviewed-boot'),
                           ('calibration_changed', True), ('boot_generation_changed', False)):
            with self.subTest(key=key):
                record = dict(self.parent_record, **{key: value})
                (self.parent / 'reviewed-baseline-record.json').write_text(json.dumps(record))
                with self.assertRaises(ValueError):
                    self.workbench.restore_uncertain_motion(self.source, self.config)
                self.assertFalse(self.workbench._owns_connection())
        (self.parent / 'reviewed-baseline-record.json').write_text(json.dumps(self.parent_record))
        profile = self.profile.read_bytes()
        (self.parent / 'reviewed-baseline.json').write_bytes(profile + b' ')
        with self.assertRaisesRegex(ValueError, '副本或来源摘要'):
            self.workbench.restore_uncertain_motion(self.source, self.config)
        (self.parent / 'reviewed-baseline.json').write_bytes(profile)
        for key, replacement in (('boot_id', 'f2070288-4492-4ddc-9594-ad00640d01b7'),
                                 ('cid', '0' * 32), ('kernel_prefix', 'forged-kernel\n')):
            with self.subTest(key=key):
                expected = dict(self.parent_expected, **{key: replacement})
                (self.parent / 'expectations.json').write_text(json.dumps(expected))
                with self.assertRaises(ValueError):
                    self.workbench.restore_uncertain_motion(self.source, self.config)
                self.assertIsNone(self.workbench.attempt)
        (self.parent / 'expectations.json').write_text(json.dumps(self.parent_expected))
        (self.parent / 'reviewed-baseline-record.json').unlink()
        with self.assertRaises(OSError):
            self.workbench.restore_uncertain_motion(self.source, self.config)

    def test_motion_deployment_passes_explicit_transport_config_to_one_channel(self):
        w = self.workbench
        hosts = self.parent / 'known_hosts'
        hosts.write_text('fake-known-hosts')
        w.known_hosts = hosts
        deployment = copy.deepcopy(self.values['deployment.json'])
        bundle = b'bundle'
        channel = mock.Mock()
        channel.receive.side_effect = [dict(kind='transfer_ready', archive_bytes=len(bundle)), self.values['ready.json']]
        helper = dict(known_hosts_fingerprint=lambda _: app.maintenance_transport.FINGERPRINT, write_pipe_once=mock.Mock())
        with mock.patch.object(app, 'prepare_bundle', return_value=(bundle, deployment)), \
                mock.patch.object(app, 'prepare_source', side_effect=lambda expected, *_args: (expected, b'mocked-source')), \
                mock.patch.object(app.maintenance_transport, 'helpers', return_value=helper), \
                mock.patch.object(app.maintenance_transport, '_ProcessChannel', return_value=channel) as factory, \
                mock.patch.object(app.threading, 'Thread') as thread:
            w.begin_connect(self.config, lambda: None)
            deploy = thread.call_args.kwargs['target']
            deploy()
        self.assertEqual(1, factory.call_count)
        self.assertIs(factory.call_args.kwargs['transport_config'], self.network)
        self.assertEqual('ready', w.phase)


class OwnershipTests(unittest.TestCase):
    def test_old_tabs_cannot_stop_or_feed_current_motion_and_manual_stop_is_available(self):
        with tempfile.TemporaryDirectory() as directory:
            w=app.MotionWorkbench({},None,None,'/no-hosts',directory)
            w.hardware=dict(connected=True,mode='hold',generation=3)
            w.control_owner=dict(id='a'*32,generation=3)
            for body in (dict(action='stop',generation=3,stop_source='window-blur'),
                         dict(action='stop',generation=3,stop_source='page-hidden'),
                         dict(action='stop',generation=3,stop_source='pagehide',control_id='b'*32),
                         dict(action='stop',generation=3),
                         dict(action='heartbeat',generation=3),
                         dict(action='heartbeat',generation=3,control_id='b'*32)):
                with self.assertRaises(ValueError): w._validate_control_locked(body)
            w._validate_control_locked(dict(action='stop',stop_source='button'))
            w._validate_control_locked(dict(action='heartbeat',generation=3,control_id='a'*32,background=True))
            w._validate_control_locked(dict(action='stop',generation=3,stop_source='pagehide',control_id='a'*32))
            w.hardware['generation']=4
            with self.assertRaises(ValueError):
                w._validate_control_locked(dict(action='stop',generation=3,stop_source='pagehide',control_id='a'*32))

    def test_independent_stance_rejects_old_page_lifecycle_stops_and_allows_manual_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            w=app.MotionWorkbench({},None,None,'/no-hosts',directory)
            w.hardware=dict(connected=True,mode='hold',generation=3,hold_control_protocol=2)
            w.control_owner=dict(id='a'*32,generation=3)
            for source in ('window-blur','page-hidden','pagehide','workspace-leave'):
                with self.subTest(source=source),self.assertRaises(ValueError):
                    w._validate_control_locked(dict(action='stop',generation=3,
                        stop_source=source,control_id='a'*32))
            w._validate_control_locked(dict(action='stop',stop_source='button',control_id='b'*32))

    def test_start_acknowledgment_binds_owner_and_stop_releases_it(self):
        with tempfile.TemporaryDirectory() as directory:
            w=app.MotionWorkbench({},None,None,'/no-hosts',directory)
            w.attempt=Path(directory);w.phase='ready'
            w.hardware=dict(connected=True,mode='off',generation=1)
            sent=[]
            class Channel:
                def send(self,request,_timeout):
                    sent.append(request)
                    w.hardware=dict(connected=True,mode='hold' if request['action']=='start' else 'off',
                                    generation=2 if request['action']=='start' else 3,release_confirmed=request['action']=='stop')
                    p=w.pending[request['request_id']]
                    p['value']=dict(error=None,hardware=copy.deepcopy(w.hardware));p['event'].set()
            w.channel=Channel()
            with self.assertRaisesRegex(ValueError,'刷新'):
                w.action(dict(action='start',mode='hold',generation=1,support_confirmed=True))
            self.assertEqual([],sent)
            w.action(dict(action='start',mode='hold',generation=1,support_confirmed=True,control_id='a'*32))
            self.assertEqual(dict(id='a'*32,generation=2),w.control_owner)
            with self.assertRaises(ValueError): w.action(dict(action='stop',generation=2,stop_source='window-blur'))
            self.assertEqual(1,len(sent))
            w.action(dict(action='stop',generation=2,stop_source='button',control_id='b'*32))
            self.assertEqual(2,len(sent));self.assertIsNone(w.control_owner)

    def test_background_heartbeat_parameter_is_typed_and_not_a_motion_start_option(self):
        with tempfile.TemporaryDirectory() as directory:
            c = host.CalibrationSession(ready_config(), Path(directory), 7)
            c.motion = mock.Mock(); c.snapshot = mock.Mock(return_value={})
            c.motion_action(dict(action='heartbeat',page_generation=7,generation=3,background=True))
            c.motion.action.assert_called_once_with(dict(action='heartbeat',generation=3,background=True))
            c.motion.action.reset_mock()
            for body in (dict(action='heartbeat',page_generation=7,background='yes'),
                         dict(action='start',page_generation=7,background=True)):
                with self.assertRaisesRegex(ValueError,'后台心跳参数无效'): c.motion_action(body)
            c.motion.action.assert_not_called()

    def test_stop_source_is_forwarded_only_for_stop_requests(self):
        with tempfile.TemporaryDirectory() as directory:
            c = host.CalibrationSession(ready_config(), Path(directory), 7)
            c.motion = mock.Mock()
            c.snapshot = mock.Mock(return_value={})
            c.motion_action(dict(action='stop',page_generation=7,stop_source='window-blur'))
            c.motion.action.assert_called_once_with(dict(action='stop',stop_source='window-blur'))
            c.motion.action.reset_mock()
            for body in (dict(action='stop',page_generation=7,stop_source='made-up'),
                         dict(action='start',page_generation=7,mode='hold',stop_source='button')):
                with self.assertRaisesRegex(ValueError, '停止来源无效'): c.motion_action(body)
            c.motion.action.assert_not_called()

    def test_motion_reservation_blocks_all_calibration_and_maintenance_connections(self):
        with tempfile.TemporaryDirectory() as directory:
            c = host.CalibrationSession(ready_config(), Path(directory), 7)
            c.motion = mock.Mock()
            for state in (dict(phase='deploying', owns_connection=False), dict(phase='ready', owns_connection=True)):
                c.motion.snapshot.return_value = state
                for fn in (c.recovery_blocked_reason, c.recheck_blocked_reason,
                           c.connection_check_blocked_reason, c._maintenance_connection_blocked_reason):
                    self.assertIn('运控', fn())

    def test_api_rejects_old_page_generation_and_incomplete_calibration_before_deploy(self):
        with tempfile.TemporaryDirectory() as directory:
            c = host.CalibrationSession(ready_config(), Path(directory), 7)
            c.motion = mock.Mock()
            for body in (dict(action='connect', page_generation=6),
                         dict(action='connect', page_generation=7, unsupported=True)):
                with self.assertRaises(ValueError): c.motion_action(body)
            c.config['imu']['calibrated'] = False
            with self.assertRaises(ValueError): c.motion_action(dict(action='connect', page_generation=7))
            c.motion.begin_connect.assert_not_called()


if __name__ == '__main__':
    unittest.main()
