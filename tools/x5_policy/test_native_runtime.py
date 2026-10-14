"""CPU-only lifecycle tests with the real reviewed controller and fake UART.

No socket, serial module, device node or SSH process is opened by these tests.
"""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import stat
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

import numpy as np

import motion
import native_runtime as app
from test_motion import FakePolicy, FakeTransport, ready_config

RECOVERY = Path(__file__).resolve().parents[1] / 'x5_recovery'
sys.path.insert(0, str(RECOVERY))
import motion_workbench
import feedback_stream


class FakeCPUModel(FakePolicy):
    def get_providers(self):
        return ['CPUExecutionProvider']


class RangeFeedbackSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.runtime = app.NativeRuntime.__new__(app.NativeRuntime)
        self.runtime.lock = threading.RLock()
        self.runtime._lifecycle = threading.RLock()
        self.runtime.phase = 'ready'
        self.runtime._connection_sequence = 2
        self.runtime._hardware_generation_floor = 7
        self.runtime._namespace = dict(health_error=None)
        self.runtime._owns_locked = mock.Mock(return_value=True)
        self.hardware = dict(generation=7, age_ms=12., imu_age_ms=15.)
        self.runtime._controller = types.SimpleNamespace(status=mock.Mock(
            side_effect=lambda: copy.deepcopy(self.hardware)))
        # A slow publication must never substitute for the actual sample.
        self.runtime._hardware_snapshot = dict(generation=7, age_ms=320.)

    def test_reads_actual_feedback_without_using_or_relabelling_display(self):
        with mock.patch.object(app.time, 'monotonic', side_effect=[10., 10.006]):
            result = self.runtime.range_feedback_snapshot()
        self.assertAlmostEqual(result['hardware']['age_ms'], 18.)
        self.assertAlmostEqual(result['feedback_age_ms'], 18.)
        self.assertEqual(self.runtime._hardware_snapshot['age_ms'], 320.)
        self.runtime._controller.status.assert_called_once_with()

    def test_preserves_stale_actual_measurement_for_save_gate(self):
        self.hardware['age_ms'] = 160.
        self.assertGreaterEqual(self.runtime.range_feedback_snapshot()['hardware']['age_ms'], 160.)

    def test_joint_boundary_save_does_not_consume_stale_imu(self):
        self.hardware['imu_age_ms'] = 60000.
        self.assertLess(self.runtime.range_feedback_snapshot()['feedback_age_ms'], 100.)

    def test_rejects_connection_transition_and_superseded_generation(self):
        controller = self.runtime._controller
        def changed():
            self.runtime._connection_sequence += 1
            return copy.deepcopy(self.hardware)
        controller.status.side_effect = changed
        with self.assertRaisesRegex(ValueError, '连接已变化'):
            self.runtime.range_feedback_snapshot()
        controller.status.side_effect = lambda: dict(self.hardware, generation=6)
        with self.assertRaisesRegex(ValueError, '连接已变化'):
            self.runtime.range_feedback_snapshot()

    def test_unconnected_or_unhealthy_never_reads_feedback(self):
        for phase, health in [('closed', None), ('ready', 'transport failed')]:
            self.runtime.phase = phase
            self.runtime._namespace['health_error'] = health
            with self.assertRaisesRegex(ValueError, '连接并载入模型'):
                self.runtime.range_feedback_snapshot()
        self.runtime._controller.status.assert_not_called()


class NativeRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name).resolve()
        (self.directory / 'policy/models').mkdir(parents=True)
        self.calibration = ready_config()
        raw = app._calibration_bytes(self.calibration)
        self.calibration_path = self.directory / 'policy/calibration.json'
        self.calibration_path.write_bytes(raw)
        self.calibration_path.chmod(0o600)
        self.deployment = dict(calibration_sha256=hashlib.sha256(raw).hexdigest(),
                               model_hashes={}, config_mount_review=[], runtime='CPUExecutionProvider')
        for name in app.MODEL_NAMES:
            data = ('fake-cpu-model-' + name).encode()
            (self.directory / 'policy/models' / ('xgoduck_' + name + '.onnx')).write_bytes(data)
            self.deployment['model_hashes'][name] = hashlib.sha256(data).hexdigest()
        self.transports = []
        self.holders = []
        self.health_fault = None
        self.health_calls = 0
        self.proof_calls = 0
        self.factory_calls = 0
        self.fail_state = False
        owner = self

        class UART(FakeTransport):
            def __init__(self, port):
                super().__init__(port)
                for key, value in getattr(owner, 'initial_feedback', {}).items():
                    setattr(self, key, copy.deepcopy(value))
                self.serial = None
                owner.transports.append(self)

            def open(self):
                super().open()
                self.serial = types.SimpleNamespace(fileno=lambda: 42)
                if hasattr(owner, 'serial_inventory'):
                    self.opened_node = copy.copy(owner.serial_inventory[self.port][0])

            def close(self):
                super().close()
                self.serial = None

            def request(self, kind, payload=b''):
                if owner.fail_state and kind == motion.STATE:
                    raise motion.MotionError('fake exact UART feedback failure')
                return super().request(kind, payload)

        def health():
            self.health_calls += 1
            if self.health_fault:
                raise ValueError(self.health_fault)
            return dict(board='same-reviewed-board', root='same-reviewed-root')

        def serial_proof(expected, descriptor=None):
            self.proof_calls += 1
            return types.SimpleNamespace(st_rdev=42), dict(path='cpu-test-only')

        self.guard = dict(EXPECTED=dict(discover_serial=True), ram_health=health,
            serial_proof=serial_proof, serial_path=lambda expected: 'cpu-test-only',
            _holders=lambda node, deadline: copy.deepcopy(self.holders), ProbeError=ValueError)
        def factory():
            self.factory_calls += 1
            return dict(self.guard)
        self.guard['connection_guard_factory'] = factory
        fake_infer = types.SimpleNamespace(np=np, session=lambda name: FakeCPUModel(name),
            infer=lambda session, observation: session.run(None, {'obs': observation})[0])
        fake_fcntl = types.SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=lambda *args: None,
                                           ioctl=lambda *args: None)
        self.patches = [mock.patch.object(motion, 'SerialTransport', UART),
                        mock.patch.dict(sys.modules, {'infer': fake_infer}),
                        mock.patch.object(app, 'fcntl', fake_fcntl)]
        for patch in self.patches:
            patch.start()
        self.runtime = None

    def test_explicit_connect_selects_recovery_guard_without_falling_back_on_failure(self):
        runtime=self.create(startup_fault='connection pending')
        recovery=mock.Mock(side_effect=ValueError('USB connection is not stable'))
        runtime.guard['usb_recovery_guard_factory']=recovery
        with self.assertRaisesRegex(ValueError,'not stable'):
            runtime.connect()
        recovery.assert_called_once_with()
        self.assertEqual(self.factory_calls,0)
        self.assertEqual(runtime.snapshot()['closed_proof']['serial_open_attempts'],0)

    def tearDown(self):
        if self.runtime:
            self.runtime.close()
            monitor = self.runtime._monitor
            if monitor:
                monitor.join(2)
            if self.runtime._publisher:
                self.runtime._publisher.join(2)
        for patch in reversed(self.patches):
            patch.stop()
        self.tmp.cleanup()

    def create(self, source=None, *, startup_fault=None, allow_fixed_head_walk=False,
               hold_hip_pitch_bias_deg=0):
        self.runtime = app.NativeRuntime(self.guard, self.directory, self.deployment,
            self.calibration, motion_workbench.WORKER if source is None else source,
            startup_fault=startup_fault, require_head_clearance=False,
            allow_fixed_head_walk=allow_fixed_head_walk,
            hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg)
        return self.runtime

    def test_hold_hip_bias_rejects_invalid_configuration_before_cpu_or_uart_work(self):
        for value in (True, False, None, '-2', float('nan'), float('inf'),
                      -float('inf'), -3.0001, 3.0001, 10 ** 400):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, '髋偏置'):
                self.create(hold_hip_pitch_bias_deg=value)
        self.assertEqual(self.factory_calls, 0)
        self.assertEqual(self.transports, [])
        self.assertFalse((self.directory / 'native-evidence').exists())

    def test_hold_hip_bias_is_forwarded_recorded_and_visible_in_compact_status(self):
        runtime = self.create(allow_fixed_head_walk=True, hold_hip_pitch_bias_deg=-2)
        self.assertEqual(runtime.hold_hip_pitch_bias_deg, -2.)
        with self.assertRaises(AttributeError):
            runtime.hold_hip_pitch_bias_deg = 0
        state = runtime.connect()
        self.assertEqual(runtime._controller.hold_hip_pitch_bias_deg, -2.)
        full = state['hardware']
        self.assertEqual(full['hold_hip_pitch_bias_deg'], -2.)
        self.assertFalse(full['fixed_head_walk_allowed'])
        self.assertFalse(full['fixed_head_walk_ready'])
        self.assertIsNone(full['start_blocked_reason'])
        for name in app.MODEL_NAMES:
            self.assertIn('模型接管尚未审查', full['model_start_blocked_reasons'][name])
            with self.assertRaisesRegex(motion.MotionError, '模型接管尚未审查'):
                runtime.action_receipt(dict(action='start', mode=name, generation=full['generation'],
                    support_confirmed=True, behavior_confirmed=True))
        compact = runtime.snapshot(compact=True)['hardware']
        self.assertEqual(compact['hold_hip_pitch_bias_deg'], -2.)
        self.assertEqual(compact['model_start_blocked_reasons'], full['model_start_blocked_reasons'])
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))
        record = json.loads((runtime._evidence / 'deployment.json').read_text())
        self.assertEqual(record['hold_hip_pitch_bias_deg'], -2.)
        runtime.disconnect()
        reconnected = runtime.connect()
        self.assertEqual(reconnected['hardware']['hold_hip_pitch_bias_deg'], -2.)
        self.assertEqual(runtime._controller.hold_hip_pitch_bias_deg, -2.)

    def test_fixed_head_walk_requires_explicit_runtime_opt_in(self):
        runtime = self.create(allow_fixed_head_walk=True)
        state = runtime.connect()
        self.assertTrue(runtime.allow_fixed_head_walk)
        self.assertTrue(state['hardware']['fixed_head_walk_allowed'])
        self.assertTrue(runtime._controller._allow_fixed_head_walk)
        self.assertEqual(runtime.hold_hip_pitch_bias_deg, 0.)
        self.assertEqual(state['hardware']['hold_hip_pitch_bias_deg'], 0.)

    def test_fault_page_loads_cpu_assets_but_explicit_connect_keeps_hardware_guard(self):
        self.health_fault = 'new unreviewed USB disconnect'
        runtime = self.create(startup_fault=self.health_fault)
        value = runtime.snapshot()
        self.assertEqual(self.health_calls, 0)
        self.assertEqual(self.factory_calls, 0)
        self.assertEqual(value['phase'], 'fault')
        self.assertEqual(value['startup_fault'], self.health_fault)
        self.assertEqual(set(value['models_loaded']), set(app.MODEL_NAMES))
        self.assertIsNone(value['hardware'])
        self.assertFalse(value['owns_connection'])
        self.assertEqual(value['closed_proof']['serial_open_attempts'], 0)
        self.assertFalse(value['closed_proof']['release_confirmed'])
        with self.assertRaisesRegex(ValueError, 'new unreviewed USB disconnect'):
            runtime.connect()
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(self.transports, [])
        self.assertEqual(runtime.snapshot()['closed_proof']['serial_open_attempts'], 0)

    def test_ownership_checks_do_not_sample_the_controller_or_open_uart(self):
        runtime = self.create()
        self.assertFalse(runtime.connection_reserved())
        runtime.phase = 'deploying'
        self.assertTrue(runtime.connection_reserved())
        runtime.phase = 'fault'
        controller = types.SimpleNamespace(_transport=object(), _connected=True, _thread=None,
            status=mock.Mock(side_effect=AssertionError('Ownership must not sample hardware')))
        with mock.patch.object(runtime, '_controller', controller), mock.patch.object(
                runtime, '_namespace', dict(opened_transports=[])):
            self.assertTrue(runtime.connection_reserved(), 'An unclosed faulty connection still owns UART')
            controller._transport = None
            controller._connected = False
            self.assertFalse(runtime.connection_reserved())
            controller.status.assert_not_called()
        self.assertEqual(self.transports, [])

    def test_normal_startup_still_rejects_hardware_fault_before_models_or_uart(self):
        self.health_fault = 'new unreviewed USB disconnect'
        with self.assertRaisesRegex(ValueError, self.health_fault):
            self.create()
        self.assertEqual(self.health_calls, 1)
        self.assertEqual(self.transports, [])

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.025)
        self.fail('CPU-only condition did not complete: ' + repr(self.runtime.snapshot()))

    def use_real_serial_binding_factory(self):
        """Execute actual reviewed discovery/proof functions over fake nodes."""
        tree = ast.parse(feedback_stream.runtime_source())
        names = ('bind_serial', 'serial_identity', 'serial_path', 'serial_proof')
        nodes = [next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == name) for name in names]
        code = compile(ast.Module(body=nodes, type_ignores=[]), '<actual-serial-guards>', 'exec')
        self.serial_inventory = {}
        owner = self
        class InventoryPath:
            def __init__(self, value):
                self.value = value
            def glob(self, pattern):
                assert self.value == '/sys/class/tty' and pattern == 'ttyUSB*'
                return [Path(Path(path).name) for path in owner.serial_inventory]
        def identity(path):
            if path not in self.serial_inventory:
                raise FileNotFoundError(path)
            node, actual = self.serial_inventory[path]
            return copy.copy(node), copy.deepcopy(actual)
        def factory():
            self.factory_calls += 1
            namespace = dict(self.guard, json=json, Path=InventoryPath, stat=stat,
                os=types.SimpleNamespace(fstat=lambda fd: copy.copy(self.transports[-1].opened_node)),
                _SERIAL_BINDINGS={}, _usb_serial_identity=identity)
            namespace['EXPECTED'] = copy.deepcopy(self.guard['EXPECTED'])
            exec(code, namespace)
            def health():
                self.guard['ram_health']()
                namespace['serial_proof'](namespace['EXPECTED'])
                return dict(board='same-reviewed-board')
            namespace['ram_health'] = health
            namespace['bind_serial'](namespace['EXPECTED'])
            health()
            return namespace
        self.guard['EXPECTED'] = dict(generation=123, discover_serial=True,
            serial=dict(vid='1a86', pid='7523', serial_number='', path='/dev/ttyUSB0', sysfs_name='1-1.3'))
        self.guard['connection_guard_factory'] = factory
        self.set_serial_node('/dev/ttyUSB0', 100)

    def set_serial_node(self, path, inode):
        self.serial_inventory = {path: (types.SimpleNamespace(st_dev=5, st_ino=inode,
            st_rdev=os.makedev(188, int(path.removeprefix('/dev/ttyUSB'))), st_mode=stat.S_IFCHR | 0o600),
            dict(path=path, vid='1a86', pid='7523', serial_number='', sysfs_name='1-1.3'))}

    def test_construction_loads_four_cpu_models_but_opens_no_uart(self):
        runtime = self.create(motion_workbench.WORKER + '\nraise AssertionError("worker bootstrap executed")\n')
        state = runtime.snapshot()
        self.assertEqual(state['phase'], 'idle')
        self.assertEqual(state['models_loaded'], list(app.MODEL_NAMES))
        self.assertIsNone(state['hardware'])
        self.assertFalse(state['owns_connection'])
        self.assertEqual(self.transports, [])
        self.assertEqual(self.proof_calls, 0)
        self.assertEqual(self.factory_calls, 0)
        self.assertEqual(self.calibration_path.read_bytes(), app._calibration_bytes(self.calibration))
        self.assertFalse(json.loads((runtime._evidence / 'deployment.json').read_text())['motor_start_attempted'])

    def test_explicit_connect_reads_configs_stops_without_arm_target(self):
        runtime = self.create()
        state = runtime.connect()
        self.assertEqual(state['phase'], 'ready')
        self.assertTrue(state['owns_connection'])
        self.assertTrue(state['hardware']['release_confirmed'])
        self.assertEqual(len(state['hardware']['online_ids']), 15)
        self.assertFalse(state['hardware']['armed'])
        self.assertEqual(len(self.transports), 1)
        kinds = [kind for kind, _ in self.transports[0].requests]
        self.assertIn(motion.STOP, kinds)
        self.assertIn(motion.CONFIG, kinds)
        self.assertNotIn(motion.ARM, kinds)
        self.assertNotIn(motion.TARGET, kinds)
        before_factory = self.factory_calls
        before_sequence = runtime.snapshot()['connection_sequence']
        with mock.patch.object(runtime, '_new_namespace', side_effect=AssertionError('Repeated connect opened UART')):
            repeated = runtime.connect()
        self.assertEqual(repeated['phase'], 'ready')
        self.assertEqual(repeated['connection_sequence'], before_sequence)
        self.assertEqual(repeated['hardware']['generation'], state['hardware']['generation'])
        self.assertFalse(repeated['hardware']['armed'])
        self.assertEqual(self.factory_calls, before_factory)
        self.assertEqual(len(self.transports), 1)
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))

    def test_gc_heap_is_prepared_before_any_uart_and_restored_after_thread_close(self):
        runtime = self.create()
        window = runtime._gc_window
        prepare, finish = window.prepare, window.finish
        events = []
        def prepare_before_open():
            self.assertTrue(runtime._serial_closed(runtime._controller, runtime._namespace))
            self.assertFalse(self.transports)
            events.append('prepare')
            return prepare()
        def finish_after_close(*, serial_closed):
            if serial_closed:
                self.assertTrue(runtime._serial_closed(runtime._controller, runtime._namespace))
                self.assertTrue(all(t.serial is None for t in self.transports))
            events.append(('finish', serial_closed))
            return finish(serial_closed=serial_closed)
        with mock.patch.object(window, 'prepare', side_effect=prepare_before_open), \
                mock.patch.object(window, 'finish', side_effect=finish_after_close):
            runtime.connect()
            self.assertTrue(window.active)
            runtime.disconnect()
        self.assertFalse(window.active)
        self.assertEqual(events[0], 'prepare')
        self.assertIn(('finish', True), events)

    def test_failed_gc_preparation_never_opens_uart(self):
        runtime = self.create()
        with mock.patch.object(runtime._gc_window, 'prepare', side_effect=MemoryError('GC preparation failed')):
            with self.assertRaisesRegex(ValueError, 'GC preparation failed'):
                runtime.connect()
        self.assertFalse(self.transports)
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertFalse(runtime._gc_window.active)

    def test_failed_controller_start_restores_gc_without_any_uart(self):
        runtime = self.create()
        with mock.patch.object(motion.MotionController, 'start', side_effect=RuntimeError('injected start failure')):
            with self.assertRaisesRegex(ValueError, 'injected start failure'):
                runtime.connect()
        self.assertFalse(self.transports)
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertFalse(runtime._gc_window.active)

    def test_outside_saved_range_keeps_connection_and_configuration_but_blocks_motion(self):
        positions = [2047] * 15
        positions[1] = 99
        self.initial_feedback = dict(positions=positions)
        runtime = self.create()
        state = runtime.connect()
        self.assertEqual(state['phase'], 'ready')
        self.assertTrue(state['hardware']['release_confirmed'])
        self.assertIn('ID 11 实际位置超出', state['hardware']['start_blocked_reason'])
        sample = runtime.range_feedback_snapshot()
        self.assertEqual(sample['hardware']['rawdata'][1]['position'], 99)
        self.assertEqual(runtime.connect()['connection_sequence'], state['connection_sequence'])
        with self.assertRaisesRegex(motion.MotionError, 'ID 11 实际位置超出'):
            runtime.action(dict(action='start', mode='hold', generation=state['hardware']['generation'], support_confirmed=True))
        self.assertTrue(runtime.snapshot()['owns_connection'])
        kinds = [kind for kind, _ in self.transports[0].requests]
        self.assertIn(motion.CONFIG, kinds)
        self.assertNotIn(motion.ARM, kinds)
        self.assertNotIn(motion.TARGET, kinds)

    def test_repeated_connect_with_stale_imu_keeps_motion_blocked(self):
        runtime = self.create()
        runtime.connect()
        self.transports[0].imu_ok = False
        self.transports[0].imu_age_us = 60000000
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            state = runtime.snapshot(compact=True)
            hardware = state.get('hardware') or {}
            if hardware.get('imu_ok') is False and hardware.get('imu_age_ms', 0) >= 500:
                break
            time.sleep(.01)
        repeated = runtime.connect()
        self.assertTrue(repeated['owns_connection'])
        self.assertTrue(repeated['hardware']['connected'])
        self.assertGreaterEqual(repeated['hardware']['imu_age_ms'], 500)
        self.assertFalse(repeated['hardware']['armed'])
        with self.assertRaises(motion.MotionError):
            runtime.action_receipt(dict(action='start', mode='hold',
                generation=repeated['hardware']['generation'], support_confirmed=True))
        self.assertEqual(len(self.transports), 1)
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))

    def test_connect_waits_for_initial_missing_position_without_arming(self):
        self.initial_feedback = dict(online=motion.ALL_MASK & ~(1 << motion.MOTOR_IDS.index(22)))
        original = FakeTransport.request
        state_requests = []
        def delayed_position(transport, kind, payload=b''):
            if kind == motion.STATE:
                state_requests.append(kind)
                if len(state_requests) >= 6:
                    transport.online = motion.ALL_MASK
            return original(transport, kind, payload)
        runtime = self.create()
        with mock.patch.object(FakeTransport, 'request', delayed_position):
            state = runtime.connect()
        self.assertGreaterEqual(len(state_requests), 6)
        self.assertEqual(state['phase'], 'ready')
        self.assertTrue(state['hardware']['release_confirmed'])
        self.assertEqual(len(self.transports), 1)
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))

    def test_connect_names_missing_position_without_claiming_torque_is_on(self):
        self.initial_feedback = dict(online=motion.ALL_MASK & ~(1 << motion.MOTOR_IDS.index(22)))
        runtime = self.create()
        with self.assertRaisesRegex(ValueError, '位置反馈无效或缺失的舵机 ID：22'):
            runtime.connect()
        state = runtime.snapshot()
        self.assertEqual(state['phase'], 'fault')
        self.assertFalse(state['owns_connection'])
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertNotIn('扭矩反馈为开启', state['error'])
        self.assertNotIn('全部舵机扭矩关闭尚未确认', state['error'])
        kinds = [kind for kind, _ in self.transports[0].requests]
        self.assertNotIn(motion.ARM, kinds)
        self.assertNotIn(motion.TARGET, kinds)

    def test_failed_connection_message_distinguishes_fault_torque_and_stale_state(self):
        state = dict(online_ids=list(motion.MOTOR_IDS), fault_ids=[13], torque_ids=[24],
                     age_ms=150., armed=True, pending_stop=True, firmware_error=4)
        reason = app._release_unconfirmed_reason(state, motion.MOTOR_IDS, 100.)
        for text in ('报告故障的舵机 ID：13', '扭矩反馈为开启的舵机 ID：24',
                     '状态反馈已过期', '控制器仍处于使能状态', '停止指令尚未完成',
                     '下位机错误代码：4'):
            self.assertIn(text, reason)
        self.assertNotIn('位置反馈无效或缺失', reason)

    def test_failed_connection_message_handles_missing_feedback_and_unknown_reason(self):
        state = dict(online_ids=list(motion.MOTOR_IDS), fault_ids=[], torque_ids=[], age_ms=None)
        self.assertIn('尚未收到有效状态反馈',
                      app._release_unconfirmed_reason(state, motion.MOTOR_IDS, 100.))
        state['age_ms'] = 0.
        self.assertEqual('连接检查未通过：未取得完整的舵机扭矩关闭确认',
                         app._release_unconfirmed_reason(state, motion.MOTOR_IDS, 100.))

    def test_status_read_does_not_wait_for_control_lock_or_call_controller_status(self):
        runtime = self.create()
        runtime.connect()
        controller = runtime._controller
        held, release, completed = threading.Event(), threading.Event(), threading.Event()
        values = []
        def hold_control():
            with controller._lock:
                held.set()
                release.wait(2)
        holder = threading.Thread(target=hold_control)
        holder.start()
        self.assertTrue(held.wait(1))
        def read_status():
            values.append(runtime.snapshot())
            completed.set()
        reader = threading.Thread(target=read_status)
        try:
            reader.start()
            self.assertTrue(completed.wait(.5), 'A web status read waited on the live control lock')
            self.assertTrue(values[0]['hardware']['connected'])
            values[0]['hardware']['online_ids'].clear()
            self.assertEqual(len(runtime.snapshot()['hardware']['online_ids']), 15)
        finally:
            release.set()
            holder.join(2)
            reader.join(2)
        self.assertFalse(any(kind in (motion.ARM,motion.TARGET) for kind,_ in self.transports[0].requests))

    def test_native_hot_requests_use_pinned_descriptor_and_arm_keeps_full_usb_proof(self):
        fast=mock.Mock(side_effect=lambda expected,descriptor: self.guard['serial_proof'](expected,descriptor))
        self.guard['serial_request_proof']=fast
        runtime=self.create()
        state=runtime.connect()
        self.assertGreater(fast.call_count,0)
        runtime.action(dict(action='start',mode='hold',support_confirmed=True,generation=state['hardware']['generation']))
        self.wait_for(lambda:sum(kind==motion.TARGET for kind,_ in self.transports[0].requests)>=5)
        with mock.patch.dict(runtime._namespace,serial_proof=mock.Mock(side_effect=AssertionError('Hot loop walked USB identity'))):
            before=sum(kind==motion.TARGET for kind,_ in self.transports[0].requests)
            self.wait_for(lambda:sum(kind==motion.TARGET for kind,_ in self.transports[0].requests)>=before+5)
        self.assertTrue(runtime.snapshot()['hardware']['armed'])
        self.assertGreater(fast.call_count,10)
        runtime.action(dict(action='stop'))
        with mock.patch.dict(runtime._namespace,serial_proof=mock.Mock(side_effect=ValueError('full USB identity changed'))):
            with self.assertRaisesRegex(ValueError,'full USB identity changed'):
                runtime.action(dict(action='start',mode='hold',support_confirmed=True,
                                    generation=runtime.snapshot()['hardware']['generation']))

    def test_published_frame_age_advances_and_expired_cache_cannot_look_live(self):
        runtime = self.create()
        runtime.connect()
        with runtime.lock:
            runtime._hardware_snapshot_at = time.monotonic() - .7
            value = runtime.snapshot()
        self.assertIsNone(value['hardware'])
        self.assertGreaterEqual(value['sample_age_ms'], 700)
        self.assertGreaterEqual(value['last_hardware']['age_ms'], 700)
        self.assertTrue(value['owns_connection'])

    def test_compact_projection_selects_before_copy_and_keeps_full_stop_evidence(self):
        runtime = self.create()
        runtime.connect()
        controller = runtime._controller
        with controller._lock:
            controller._last_action = [1.] * 15
            controller._last_stop_diagnostic = dict(sequence=7, reason='test stop', source='button',
                before=dict(mode='hold', generation=3, rawdata=[dict(id=11, position=1234)],
                            last_stop_diagnostic=dict(sequence=6)),
                transport=dict(requests=[list(range(100)) for _ in range(50)]))
            runtime._publish_hardware(controller)
        with mock.patch.object(runtime, '_publish_hardware'):
            compact = runtime.snapshot(compact=True)
            full = runtime.snapshot()
        self.assertIsNone(compact['last_hardware'])
        small = compact['hardware']['last_stop_diagnostic']
        self.assertEqual(small['sequence'], 7)
        self.assertEqual(small['before']['mode'], 'hold')
        self.assertEqual(small['before']['generation'], 3)
        self.assertNotIn('rawdata', small['before'])
        self.assertNotIn('transport', small)
        self.assertNotIn('transport', full['hardware']['last_stop_diagnostic'])
        self.assertNotIn('rawdata', full['hardware']['last_stop_diagnostic']['before'])
        self.assertTrue(full['hardware']['last_stop_diagnostic']['details_deferred_until_UART_closed'])
        for key in ('rawdata', 'last_executed_target_deg', 'command', 'imu', 'hold_ready',
                    'hold_head_deg', 'fixed_head_walk_allowed', 'age_ms', 'imu_age_ms',
                    'online_ids', 'fault_ids', 'torque_ids', 'armed', 'release_confirmed'):
            self.assertIn(key, compact['hardware'])
        compact['hardware']['rawdata'][0]['position'] = -999
        compact['hardware']['last_action'][0] = -999
        self.assertNotEqual(runtime._hardware_snapshot['rawdata'][0]['position'], -999)
        self.assertEqual(runtime._hardware_snapshot['last_action'][0], 1.)
        self.assertEqual(controller._last_stop_diagnostic['transport']['requests'][0][0], 0)
        self.assertIsNot(full['hardware'], full['last_hardware'])
        class ExcludedHistory:
            def __deepcopy__(self, memo):
                raise AssertionError('compact traversed excluded audit history')
        packet = dict(runtime._hardware_snapshot, excluded_history=ExcludedHistory())
        app._hardware_projection(packet, True)
        runtime.disconnect()
        closed=runtime.snapshot()
        saved=closed['last_hardware']['last_stop_diagnostic']
        self.assertIn('transport',saved)
        self.assertIn('rawdata',saved['before'])
        saved['transport']['requests'][0][0]=-999
        self.assertEqual(controller._last_stop_diagnostic['transport']['requests'][0][0],0)

    def test_live_native_publication_and_persist_never_traverse_historical_arrays(self):
        runtime=self.create();runtime.connect();controller=runtime._controller
        class HistoricalArray:
            def __deepcopy__(self,memo):
                raise AssertionError('active control traversed historical STOP array')
        old=controller._last_stop_diagnostic if hasattr(controller,'_last_stop_diagnostic') else None
        try:
            with controller._lock:
                controller._last_stop_diagnostic=dict(sequence=1,reason='old stop',
                    before=dict(mode='walk',generation=3,rawdata=HistoricalArray()),
                    transport=dict(requests=HistoricalArray()))
                value=controller.status()
                self.assertEqual(value['last_stop_diagnostic']['sequence'],1)
                self.assertNotIn('transport',value['last_stop_diagnostic'])
                runtime._publish_hardware(controller)
            with runtime.lock:
                runtime._hardware_snapshot=dict(runtime._hardware_snapshot,excluded_history=HistoricalArray())
            runtime._persist()
            saved=json.loads((runtime._evidence/'live-state.json').read_text())
            self.assertTrue(saved['snapshot_compact'])
            self.assertNotIn('transport',saved['hardware']['last_stop_diagnostic'])
        finally:
            controller._last_stop_diagnostic=old

    def test_status_adoption_and_external_publication_have_owned_mutable_containers(self):
        runtime = self.create()
        runtime.connect()
        controller = runtime._controller
        def mutable_ids(value):
            result = set()
            def visit(item):
                if isinstance(item, (dict, list, set)):
                    if id(item) in result:
                        return
                    result.add(id(item))
                    for child in item.values() if isinstance(item, dict) else item:
                        visit(child)
                elif isinstance(item, tuple):
                    for child in item:
                        visit(child)
            visit(value)
            return result
        with controller._lock:
            controller._last_action = [1.] * 15
            controller._hold_head_deg = {sid: float(sid) for sid in (30, 31, 32, 33)}
            controller._last_executed_target = [2.] * 15
            controller._last_policy_target = [3.] * 14
            controller._last_stop_diagnostic = dict(sequence=1, before=dict(rawdata=[dict(id=11)]),
                                                    transport=dict(requests=[[1, 2]]))
            adopted = app._own_controller_status(controller.status())
            self.assertEqual(mutable_ids(adopted) & mutable_ids(controller.__dict__), set())
            with mock.patch.object(controller, 'status', return_value=adopted):
                runtime._publish_hardware(controller)
            self.assertIs(runtime._hardware_snapshot, adopted)
            external = controller.status()
            runtime._publish_hardware(controller, external)
            self.assertEqual(mutable_ids(external) & mutable_ids(runtime._hardware_snapshot), set())
            self.assertIsNot(runtime._hardware_snapshot, external)

    def test_projection_does_not_hold_runtime_lock_and_late_pre_stop_frame_is_invalidated(self):
        runtime = self.create()
        state = runtime.connect()
        entered, release = threading.Event(), threading.Event()
        original = app._hardware_projection
        values, errors = [], []
        def slow_projection(*args, **kwargs):
            if threading.current_thread().name == 'blocked-display-copy':
                entered.set()
                if not release.wait(2):
                    raise AssertionError('display test was not released')
            return original(*args, **kwargs)
        def read():
            try:
                values.append(runtime.snapshot(compact=True))
            except BaseException as error:
                errors.append(error)
        with mock.patch.object(app, '_hardware_projection', side_effect=slow_projection):
            reader = threading.Thread(target=read, name='blocked-display-copy')
            reader.start()
            self.assertTrue(entered.wait(1))
            try:
                self.assertEqual(runtime.control_context()['hardware']['generation'],
                                 state['hardware']['generation'])
                stopped = runtime.action_receipt(dict(action='stop', stop_source='button'))
                self.assertTrue(stopped['release_confirmed'])
                self.assertTrue(reader.is_alive(), 'copy should remain blocked, independently of STOP')
            finally:
                release.set()
                reader.join(2)
        self.assertEqual(errors, [])
        self.assertTrue(values[0]['snapshot_superseded'])
        self.assertIsNone(values[0]['hardware'])
        self.assertIsNone(values[0]['last_hardware'])

    def test_late_projection_cannot_be_relabelled_after_disconnect_and_reconnect(self):
        runtime = self.create()
        old = runtime.connect()
        entered, release = threading.Event(), threading.Event()
        original = app._hardware_projection
        values = []
        def slow_projection(*args, **kwargs):
            if threading.current_thread().name == 'old-connection-copy':
                entered.set()
                release.wait(3)
            return original(*args, **kwargs)
        with mock.patch.object(app, '_hardware_projection', side_effect=slow_projection):
            reader = threading.Thread(target=lambda: values.append(runtime.snapshot(compact=True)),
                                      name='old-connection-copy')
            reader.start()
            self.assertTrue(entered.wait(1))
            try:
                runtime.disconnect()
                new = runtime.connect()
            finally:
                release.set()
                reader.join(2)
        self.assertGreater(new['connection_sequence'], old['connection_sequence'])
        self.assertEqual(values[0]['connection_sequence'], old['connection_sequence'])
        self.assertTrue(values[0]['snapshot_superseded'])
        self.assertIsNone(values[0]['hardware'])
        self.assertIsNone(values[0]['last_hardware'])

    def test_display_publisher_period_does_not_change_actual_action_feedback_gate(self):
        runtime = self.create()
        state = runtime.connect()
        self.assertEqual(app.DISPLAY_PUBLISH_SECONDS, .1)
        self.assertEqual(motion.HEARTBEAT_SECONDS, .4)
        controller = runtime._controller
        with mock.patch.object(runtime, '_publish_hardware'):
            with runtime.lock:
                runtime._hardware_snapshot_at = time.monotonic() - .7
            self.assertIsNone(runtime.snapshot(compact=True)['hardware'])
            # A stale display does not block a genuinely fresh controller.
            started = runtime.action_receipt(dict(action='start', mode='shadow',
                generation=state['hardware']['generation']))
            runtime.action_receipt(dict(action='stop'))
            # Conversely a falsely fresh display cannot bypass actual feedback.
            self.assertIsNotNone(runtime.snapshot(compact=True)['hardware'])
            with controller._lock:
                with mock.patch.object(controller, '_sample_received', time.monotonic() - 1.):
                    with self.assertRaisesRegex(motion.MotionError, '反馈已过期'):
                        runtime.action_receipt(dict(action='start', mode='shadow',
                            generation=runtime.control_context()['hardware']['generation']))
        self.assertEqual(started['mode'], 'shadow')

    def test_publication_age_includes_status_tail_and_delayed_projection(self):
        runtime = self.create()
        runtime.connect()
        controller = runtime._controller
        original_status, original_publish = controller.status, runtime._publish_hardware
        def slow_status():
            value = original_status()
            time.sleep(.12)
            return value
        original_projection = app._hardware_projection
        def slow_projection(*args, **kwargs):
            time.sleep(.08)
            return original_projection(*args, **kwargs)
        with mock.patch.object(runtime, '_publish_hardware'):
            with mock.patch.object(controller, 'status', side_effect=slow_status):
                original_publish(controller)
            with mock.patch.object(app, '_hardware_projection', side_effect=slow_projection):
                state = runtime.snapshot(compact=True)
            self.assertGreaterEqual(state['hardware']['age_ms'], 200)
            self.assertGreaterEqual(state['hardware']['imu_age_ms'], 200)
            with runtime.lock:
                runtime._hardware_snapshot_at -= .5
            expired = runtime.snapshot(compact=True)
        self.assertIsNone(expired['hardware'])
        self.assertGreaterEqual(expired['last_hardware']['age_ms'], 700)
        timings = state['runtime_timing']['operations']['snapshot_compact']
        self.assertGreaterEqual(timings['projection_ms'], 80)
        self.assertGreaterEqual(timings['wall_ms'], timings['cpu_ms'])
        self.assertGreaterEqual(timings['lock_wait_ms'], 0.)

    def test_slow_health_inventory_does_not_freeze_feedback_and_failure_still_closes(self):
        runtime=self.create()
        runtime.connect()
        entered,release=threading.Event(),threading.Event()
        def delayed_failure():
            entered.set()
            release.wait(3)
            raise ValueError('reviewed health failure after slow inventory')
        runtime._namespace['ram_health']=delayed_failure
        try:
            self.assertTrue(entered.wait(2))
            observed=[]
            # Longer than the 500ms stale-publication gate. A serialized
            # health/publication loop would make hardware disappear here.
            for _ in range(8):
                time.sleep(.08)
                value=runtime.snapshot()
                self.assertIsNotNone(value['hardware'])
                observed.append(value['hardware']['age_ms'])
                self.assertEqual(value['hardware']['mode'],'off')
            self.assertLess(max(observed),250)
            self.assertFalse(any(kind in (motion.ARM,motion.TARGET) for kind,_ in self.transports[0].requests))
        finally:
            release.set()
        self.wait_for(lambda:runtime.snapshot()['phase']=='fault')
        value=runtime.snapshot()
        self.assertEqual(value['error'],'reviewed health failure after slow inventory')
        self.assertTrue(value['closed_proof']['serial_closed'])
        runtime._publisher.join(1)
        self.assertFalse(runtime._publisher.is_alive())
        self.assertIsNone(runtime.snapshot()['hardware'])

    def test_start_validates_generation_support_and_behavior(self):
        runtime = self.create()
        state = runtime.connect()
        generation = state['hardware']['generation']
        for value in (generation - 1, True, None):
            with self.assertRaisesRegex(ValueError, 'state changed'):
                runtime.action(dict(action='start', mode='hold', generation=value, support_confirmed=True))
        with self.assertRaises(motion.MotionError):
            runtime.action(dict(action='start', mode='hold', generation=generation, support_confirmed=False))
        with self.assertRaises(motion.MotionError):
            runtime.action(dict(action='start', mode='getup', generation=generation,
                                support_confirmed=True, behavior_confirmed=False))
        self.assertNotIn(motion.ARM, [kind for kind, _ in self.transports[0].requests])
        result = runtime.action(dict(action='start', mode='hold', generation=generation,
                                     support_confirmed=True, page_generation=123))
        self.assertTrue(result['hardware']['armed'])
        stopped = runtime.action(dict(action='stop', stop_source='button'))
        self.assertTrue(stopped['hardware']['release_confirmed'])
        self.assertEqual(stopped['hardware']['torque_ids'], [])

    def test_native_head_clearance_capture_requires_fresh_off_state_and_does_not_arm(self):
        runtime = self.create()
        runtime.require_head_clearance = True
        state = runtime.connect()
        self.assertFalse(runtime.allow_fixed_head_walk)
        self.assertFalse(state['hardware']['fixed_head_walk_allowed'])
        self.assertTrue(state['hardware']['head_clearance_required'])
        generation = state['hardware']['generation']
        with self.assertRaisesRegex(motion.MotionError, '记录头颈'):
            runtime.action(dict(action='start', mode='hold', generation=generation, support_confirmed=True))
        with self.assertRaises(motion.MotionError):
            runtime.action(dict(action='capture-head-clearance', generation=generation, clearance_confirmed=False))
        captured = runtime.action(dict(action='capture-head-clearance', generation=generation, clearance_confirmed=True))
        self.assertIsNotNone(captured['hardware']['hold_head_deg'])
        self.assertEqual(captured['hardware']['mode'], 'off')
        self.assertFalse(captured['hardware']['armed'])
        self.assertNotIn(motion.ARM, [kind for kind, _ in self.transports[0].requests])
        self.assertNotIn(motion.TARGET, [kind for kind, _ in self.transports[0].requests])
        self.assertTrue(all(captured['hardware']['model_start_blocked_reasons'][name] for name in app.MODEL_NAMES))
        runtime.disconnect()
        reconnected = runtime.connect()
        self.assertIsNone(reconnected['hardware']['hold_head_deg'])

    def test_capture_receipt_keeps_actual_gates_and_publishes_owned_configuration_without_io(self):
        runtime = self.create()
        runtime.require_head_clearance = True
        state = runtime.connect()
        generation = state['hardware']['generation']
        with self.assertRaises(motion.MotionError):
            runtime.action_receipt(dict(action='capture-head-clearance', generation=generation,
                                        clearance_confirmed=False))
        with self.assertRaises(motion.MotionError):
            runtime.action_receipt(dict(action='capture-head-clearance', generation=generation - 1,
                                        clearance_confirmed=True))
        main = threading.current_thread()
        def background_only(original):
            def guarded(*args, **kwargs):
                if threading.current_thread() is main:
                    raise AssertionError('capture receipt waited for full display/evidence IO')
                return original(*args, **kwargs)
            return guarded
        with (mock.patch.object(runtime, '_persist', background_only(runtime._persist)),
              mock.patch.object(runtime, '_record', background_only(runtime._record)),
              mock.patch.object(runtime, 'snapshot', background_only(runtime.snapshot)),
              mock.patch.object(runtime, '_publish_hardware')):
            receipt = runtime.action_receipt(dict(action='capture-head-clearance', generation=generation,
                                                 clearance_confirmed=True))
            self.assertEqual(receipt['mode'], 'off')
            self.assertTrue(receipt['release_confirmed'])
            self.assertTrue(receipt['connected'])
            self.assertEqual(receipt['generation'], generation + 1)
            self.assertEqual(set(receipt['captured_head_deg']), {'30', '31', '32', '33'})
            self.assertNotIn('hardware', receipt)
            self.assertNotIn('sample_us', receipt)
            self.assertNotIn('imu', receipt)
            with runtime.lock:
                self.assertIs(runtime._hardware_snapshot, runtime.last_operation['after'])
                self.assertEqual(runtime._hardware_snapshot_sequence, receipt['connection_sequence'])
            receipt['captured_head_deg']['30'] = -999
            self.assertNotEqual(runtime.last_operation['after']['hold_head_deg'][30], -999)
        live = runtime.snapshot(compact=True)
        self.assertEqual(live['hardware']['generation'], receipt['generation'])
        self.assertIsNotNone(live['hardware']['hold_head_deg'])
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))
        self.wait_for(lambda: len(runtime._action_evidence) == 0)
        rows = [json.loads(path.read_text()) for path in runtime._evidence.glob('operation-*.json')]
        completed = [row for row in rows if row['state'] == 'complete']
        self.assertEqual(len(completed), 1)
        self.assertIsNotNone(completed[0]['after']['hold_head_deg'])
        timing = receipt['runtime_timing']['operations']['evidence_enqueue']
        self.assertTrue(timing['ownership_transferred'])
        self.assertGreaterEqual(timing['copy_cpu_ms'], 0.)
        self.assertLessEqual(len(receipt['runtime_timing']['operations']), 8)

    def test_capture_receipt_rejects_late_generation_and_actual_stale_or_active_state(self):
        runtime = self.create()
        state = runtime.connect()
        controller = runtime._controller
        with controller._lock:
            with mock.patch.object(controller, '_sample_received', time.monotonic() - 1.):
                with self.assertRaisesRegex(motion.MotionError, '反馈已过期'):
                    runtime.action_receipt(dict(action='capture-head-clearance',
                        generation=state['hardware']['generation'], clearance_confirmed=True))
        started = runtime.action_receipt(dict(action='start', mode='shadow',
            generation=state['hardware']['generation']))
        with self.assertRaises(motion.MotionError):
            runtime.action_receipt(dict(action='capture-head-clearance', generation=started['generation'],
                                        clearance_confirmed=True))
        stopped = runtime.action_receipt(dict(action='stop'))
        original = controller.capture_head_clearance
        def interrupted(*args, **kwargs):
            result = original(*args, **kwargs)
            controller.stop(source='button')
            return result
        with mock.patch.object(controller, 'capture_head_clearance', side_effect=interrupted):
            with self.assertRaisesRegex(ValueError, 'state changed'):
                runtime.action_receipt(dict(action='capture-head-clearance', generation=stopped['generation'],
                                            clearance_confirmed=True))
        context = runtime.control_context()['hardware']
        self.assertEqual(context['mode'], 'off')
        self.assertTrue(context['release_confirmed'])
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET) for kind, _ in self.transports[0].requests))

    def test_reconnect_has_new_generation_and_genuine_close_proof(self):
        runtime = self.create()
        first = runtime.connect()['hardware']['generation']
        closed = runtime.disconnect()
        self.assertTrue(closed['closed_proof']['serial_closed'])
        self.assertTrue(closed['closed_proof']['release_confirmed'])
        self.assertIsNone(closed['hardware'])
        self.assertFalse(closed['owns_connection'])
        self.assertTrue(self.transports[0].closed)
        proof = copy.deepcopy(closed['closed_proof'])
        with mock.patch.object(runtime._controller, 'status',
                               side_effect=AssertionError('Retired controller polled')):
            for _ in range(5):
                value = runtime.snapshot()
                self.assertIsNone(value['hardware'])
                self.assertIsNone(value['sample_age_ms'])
                self.assertFalse(value['owns_connection'])
                self.assertEqual(value['last_hardware'], proof['after'])
                self.assertEqual(value['closed_proof'], proof)
        self.assertEqual(runtime.disconnect()['closed_proof'], proof)
        second = runtime.connect()['hardware']['generation']
        self.assertGreater(second, first)
        with self.assertRaisesRegex(ValueError, 'state changed'):
            runtime.action(dict(action='start', mode='hold', generation=first, support_confirmed=True))
        self.assertEqual(len(self.transports), 2)
        self.assertEqual(runtime._namespace['serial_open_attempts'], 1)

    def test_explicit_calibration_reload_accepts_zero_invalidation_without_opening_uart(self):
        runtime = self.create()
        changes = copy.deepcopy(self.calibration)
        changes['joints'][0].update(zero=2054, min=None, max=None, direction_confirmed=False)
        def publish(raw):
            self.calibration_path.write_bytes(raw)
            return hashlib.sha256(raw).hexdigest()
        runtime.guard['publish_calibration'] = publish
        digest = runtime.reload_calibration(changes, {'version': 1})
        self.assertEqual(runtime.calibration, changes)
        self.assertEqual(runtime.deployment['calibration_sha256'], digest)
        self.assertEqual(runtime.deployment['model_hashes'], self.deployment['model_hashes'])
        self.assertEqual(self.transports, [])
        self.assertEqual(self.factory_calls, 0)

    def test_reopening_after_zero_save_keeps_workbench_usable_but_motion_blocked(self):
        runtime = self.create()
        changed = copy.deepcopy(self.calibration)
        changed['joints'][0].update(zero=2054, min=None, max=None, direction_confirmed=False)
        def publish(raw):
            self.calibration_path.write_bytes(raw)
            return hashlib.sha256(raw).hexdigest()
        runtime.guard['publish_calibration'] = publish
        runtime.reload_calibration(changed, {'version': 1})
        deployment = copy.deepcopy(runtime.deployment)
        previous = self.directory
        runtime.close()
        # A reboot/redeployment receives the exact saved incomplete snapshot
        # in a fresh RAM directory; models remain intact and can still load.
        reopened = previous / 'reopened'
        reopened.mkdir(mode=0o700)
        (reopened / 'policy/models').mkdir(parents=True)
        calibration_path = reopened / 'policy/calibration.json'
        calibration_path.write_bytes(app._calibration_bytes(changed))
        calibration_path.chmod(0o600)
        for name in app.MODEL_NAMES:
            filename = 'xgoduck_' + name + '.onnx'
            (reopened / 'policy/models' / filename).write_bytes(
                (previous / 'policy/models' / filename).read_bytes())
        self.runtime = app.NativeRuntime(self.guard, reopened, deployment,
                                        changed, motion_workbench.WORKER)
        self.assertEqual(self.runtime.calibration, changed)
        self.assertEqual(self.runtime.snapshot()['phase'], 'idle')
        self.assertEqual(set(self.runtime.snapshot()['models_loaded']), set(app.MODEL_NAMES))
        self.assertEqual(self.transports, [])
        with self.assertRaisesRegex(ValueError, 'calibration is incomplete'):
            self.runtime.connect()
        self.assertEqual(self.transports, [])
        self.assertEqual(self.runtime.closed_proof['serial_open_attempts'], 0)
        self.assertTrue(self.runtime.closed_proof['no_uart_opened'])

    def test_calibration_reload_does_not_save_over_an_owned_motion_uart(self):
        runtime = self.create()
        runtime.connect()
        publish = mock.Mock()
        runtime.guard['publish_calibration'] = publish
        with self.assertRaisesRegex(ValueError, '停止并断开'):
            runtime.reload_calibration(self.calibration)
        publish.assert_not_called()
        self.assertEqual(self.calibration_path.read_bytes(), app._calibration_bytes(self.calibration))
        runtime.disconnect()
        def commit(raw):
            self.calibration_path.write_bytes(raw)
            return hashlib.sha256(raw).hexdigest()
        runtime.guard['publish_calibration'] = commit
        runtime.reload_calibration(self.calibration)
        self.assertFalse(runtime.snapshot()['owns_connection'])
        self.assertEqual(len(self.transports), 1)

    def test_other_uart_owner_blocks_before_open_and_explicit_retry_works(self):
        runtime = self.create()
        self.holders = [dict(pid=99)]
        with self.assertRaisesRegex(ValueError, 'already has an owner'):
            runtime.connect()
        self.assertEqual(runtime.snapshot()['phase'], 'fault')
        self.assertEqual(self.transports, [])
        self.assertTrue(runtime.snapshot()['closed_proof']['no_uart_opened'])
        self.holders = []
        state = runtime.connect()
        self.assertEqual(state['phase'], 'ready')
        self.assertEqual(len(self.transports), 1)

    def test_health_fault_stops_closes_keeps_page_state_and_never_retries(self):
        runtime = self.create()
        runtime.connect()
        self.health_fault = 'reviewed CONFIG mount changed'
        self.wait_for(lambda: runtime.snapshot()['phase'] == 'fault')
        state = runtime.snapshot()
        self.assertEqual(state['error'], self.health_fault)
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertIsNone(state['hardware'])
        self.assertFalse(state['owns_connection'])
        self.assertEqual(state['models_loaded'], list(app.MODEL_NAMES))
        time.sleep(.15)
        self.assertEqual(len(self.transports), 1)
        with self.assertRaisesRegex(ValueError, 'CONFIG mount changed'):
            runtime.connect()
        self.assertEqual(len(self.transports), 1)
        self.health_fault = None
        self.assertEqual(runtime.connect()['phase'], 'ready')

    def test_terminal_uart_failure_has_precise_reason_and_no_fake_release(self):
        runtime = self.create()
        runtime.connect()
        self.fail_state = True
        self.wait_for(lambda: runtime.snapshot()['phase'] == 'fault')
        state = runtime.snapshot()
        self.assertEqual(state['error'], 'fake exact UART feedback failure')
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertFalse(state['closed_proof']['release_confirmed'])
        self.assertIsNone(state['hardware'])
        self.assertEqual(len(self.transports), 1)

    def test_ram_evidence_failure_closes_uart_and_keeps_exact_memory_proof(self):
        runtime = self.create()
        state = runtime.connect()
        with mock.patch.object(runtime, '_record', side_effect=OSError('fake RAM exhausted')):
            with self.assertRaisesRegex(ValueError, 'RAM exhausted'):
                runtime.action(dict(action='start', mode='hold', support_confirmed=True,
                                    generation=state['hardware']['generation']))
        state = runtime.snapshot()
        self.assertEqual(state['phase'], 'fault')
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertTrue(state['closed_proof']['release_confirmed'])
        self.assertIn('RAM exhausted', state['error'])
        self.assertIn('RAM exhausted', state['closed_proof']['evidence_error'])
        self.assertFalse(state['owns_connection'])

    def test_feedback_age_includes_real_imu_age(self):
        runtime = self.create()
        runtime.connect()
        self.transports[0].imu_age_us = 200000
        self.wait_for(lambda: runtime.snapshot()['sample_age_ms'] >= 200)
        state = runtime.snapshot()
        self.assertGreaterEqual(state['hardware']['imu_age_ms'], 200)
        self.assertGreaterEqual(state['sample_age_ms'], state['hardware']['imu_age_ms'])
        self.assertEqual(state['feedback_age_ms'], 0.)
        with self.assertRaises(motion.MotionError):
            runtime.action(dict(action='start', mode='hold', support_confirmed=True,
                                generation=state['hardware']['generation']))

    def test_fixed_stance_survives_browser_heartbeat_loss_and_manual_stop_works(self):
        runtime = self.create()
        state = runtime.connect()
        runtime.action(dict(action='start', mode='hold', support_confirmed=True,
                            generation=state['hardware']['generation']))
        time.sleep(.65)
        state = runtime.snapshot()
        self.assertEqual(state['hardware']['mode'], 'hold')
        self.assertTrue(state['hardware']['armed'])
        self.assertFalse(state['hardware']['page_heartbeat_required'])
        self.assertIsNone(state['hardware']['heartbeat_remaining_ms'])
        self.assertTrue(any(kind == motion.TARGET for kind, _ in self.transports[0].requests))
        runtime.action(dict(action='stop', generation=state['hardware']['generation']))
        self.wait_for(lambda: runtime.snapshot()['hardware']['release_confirmed'])
        self.assertEqual(runtime.snapshot()['hardware']['mode'], 'off')
        self.assertEqual(len(self.transports), 1)

    def test_browser_heartbeat_timeout_stops_dynamic_preview_without_reopening(self):
        runtime = self.create()
        state = runtime.connect()
        runtime.action(dict(action='start', mode='shadow', preview_model='walk',
                            generation=state['hardware']['generation']))
        self.wait_for(lambda: runtime.snapshot()['hardware']['mode'] == 'off')
        self.wait_for(lambda: runtime.snapshot()['hardware']['release_confirmed'])
        self.assertIn('心跳超时', runtime.snapshot()['hardware']['error'])
        self.assertEqual(len(self.transports), 1)
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET)
                             for kind, _ in self.transports[0].requests))
        self.assertEqual(motion.HEARTBEAT_SECONDS, .4)
        self.assertEqual(motion.BACKGROUND_HOLD_HEARTBEAT_SECONDS, 3.)

    def test_heartbeat_uses_atomic_generation_without_main_thread_status_or_evidence_io(self):
        runtime = self.create()
        state = runtime.connect()
        running = runtime.action(dict(action='start', mode='shadow', preview_model='walk',
                                     generation=state['hardware']['generation']))
        generation = running['hardware']['generation']
        main_thread = threading.current_thread()
        def background_only(original):
            def guarded(*args, **kwargs):
                if threading.current_thread() is main_thread:
                    raise AssertionError('heartbeat waited for status/evidence IO')
                return original(*args, **kwargs)
            return guarded
        controller = runtime._controller
        with (mock.patch.object(runtime, '_persist', background_only(runtime._persist)),
              mock.patch.object(runtime, '_publish_hardware', background_only(runtime._publish_hardware)),
              mock.patch.object(runtime, 'snapshot', background_only(runtime.snapshot)),
              mock.patch.object(controller, 'status', background_only(controller.status))):
            accepted = runtime.action(dict(action='heartbeat', generation=generation))
            self.assertEqual(accepted, dict(heartbeat_accepted=True, generation=generation))
            with self.assertRaisesRegex(motion.MotionError, 'state changed'):
                runtime.action(dict(action='heartbeat', generation=generation - 1))
        self.assertFalse(any(kind in (motion.ARM, motion.TARGET)
                             for kind, _ in self.transports[0].requests))

    def test_action_receipts_and_actual_context_avoid_full_http_status_and_evidence_io(self):
        runtime = self.create()
        state = runtime.connect()
        main = threading.current_thread()
        def background_only(original):
            def guarded(*args, **kwargs):
                if threading.current_thread() is main:
                    raise AssertionError('receipt waited for full status/evidence IO')
                return original(*args, **kwargs)
            return guarded
        with (mock.patch.object(runtime, '_persist', background_only(runtime._persist)),
              mock.patch.object(runtime, '_record', background_only(runtime._record)),
              mock.patch.object(runtime, '_publish_hardware', background_only(runtime._publish_hardware)),
              mock.patch.object(runtime, 'snapshot', background_only(runtime.snapshot))):
            started = runtime.action_receipt(dict(action='start', mode='shadow', preview_model='walk',
                generation=state['hardware']['generation']))
            self.assertEqual(started['action_receipt_protocol'], 1)
            self.assertEqual(started['mode'], 'shadow')
            self.assertEqual(set(started['transition_timing_ms']),
                             {'controller_action', 'context', 'evidence_enqueue'})
            self.assertNotIn('hardware', started)
            self.assertNotIn('imu', started)
            with runtime.lock:
                # Keep publication copy-on-write even in a deliberately stale
                # cache fixture: its prior root is also an immutable audit event.
                runtime._hardware_snapshot = dict(runtime._hardware_snapshot,
                    generation=started['generation'] - 1, mode='off')
            # Even a deliberately stale published projection cannot substitute
            # for actual START/STOP generation in ownership checks.
            context = runtime.control_context()
            self.assertEqual(context['hardware']['generation'], started['generation'])
            self.assertEqual(context['hardware']['mode'], 'shadow')
            self.assertEqual(context['hardware']['hold_control_protocol'], 2)
            self.assertEqual(context['connection_sequence'], started['connection_sequence'])
            self.assertEqual(runtime.action(dict(action='heartbeat', generation=started['generation'])),
                             dict(heartbeat_accepted=True, generation=started['generation']))
            stopped = runtime.action_receipt(dict(action='stop', stop_source='button'))
            self.assertEqual(stopped['mode'], 'off')
            self.assertTrue(stopped['connected'])
            self.assertTrue(stopped['release_confirmed'])
            self.assertGreater(stopped['generation'], started['generation'])
        self.wait_for(lambda: len(runtime._action_evidence) == 0)
        records = [json.loads(path.read_text()) for path in runtime._evidence.glob('operation-*.json')]
        self.assertEqual([row['action'] for row in sorted(records,key=lambda row:row['recorded_monotonic'])],
                         ['start', 'stop'])
        self.assertEqual(runtime._action_evidence_reserved, 0)
        self.assertEqual(motion.HEARTBEAT_SECONDS, .4)

    def test_receipt_evidence_write_failure_fault_stops_and_retains_pending_event(self):
        runtime = self.create()
        state = runtime.connect()
        original = runtime._record
        def failing_record(name, value):
            if name.startswith('operation-'):
                raise OSError('receipt audit RAM exhausted')
            return original(name, value)
        with mock.patch.object(runtime, '_record', side_effect=failing_record):
            runtime.action_receipt(dict(action='start', mode='shadow',
                generation=state['hardware']['generation']))
            self.wait_for(lambda: bool(runtime.closed_proof and runtime.closed_proof['serial_closed']
                                      and runtime.closed_proof.get('evidence_error')))
        self.assertEqual(runtime.phase, 'fault')
        self.assertIn('receipt audit RAM exhausted', runtime.error)
        self.assertTrue(runtime.closed_proof['release_confirmed'])
        self.assertGreaterEqual(len(runtime._action_evidence), 1)
        self.assertEqual(runtime._action_evidence[0][1]['action'], 'start')
        self.assertFalse(self.transports[0].armed)
        self.assertEqual(self.transports[0].torque, 0)

    def test_receipt_queue_overflow_fault_stops_before_new_start(self):
        runtime = self.create()
        state = runtime.connect()
        with mock.patch.object(runtime, '_flush_action_evidence'):
            with runtime.lock:
                runtime._action_evidence.extend((f'operation-reserved-{i}.json', {'index':i})
                                               for i in range(app.ACTION_EVIDENCE_LIMIT))
            with mock.patch.object(runtime._controller, 'start_mode',
                                   side_effect=AssertionError('overflow executed a new start')):
                with self.assertRaisesRegex(ValueError, 'queue is full'):
                    runtime.action_receipt(dict(action='start', mode='shadow',
                        generation=state['hardware']['generation']))
        self.assertEqual(runtime.phase, 'fault')
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertTrue(runtime.closed_proof['release_confirmed'])
        self.assertEqual(len(runtime._action_evidence), app.ACTION_EVIDENCE_LIMIT)
        self.assertEqual(runtime._action_evidence_reserved, 0)

    def test_receipt_rejects_start_generation_superseded_by_actual_stop(self):
        runtime = self.create()
        state = runtime.connect()
        controller = runtime._controller
        original = controller.start_mode
        def interrupted(*args, **kwargs):
            result = original(*args, **kwargs)
            controller.stop(source='button')
            return result
        with mock.patch.object(controller, 'start_mode', side_effect=interrupted):
            with self.assertRaisesRegex(ValueError, 'state changed'):
                runtime.action_receipt(dict(action='start', mode='shadow',
                    generation=state['hardware']['generation']))
        self.assertEqual(runtime.control_context()['hardware']['mode'], 'off')
        self.assertTrue(runtime.control_context()['hardware']['release_confirmed'])
        self.assertEqual(runtime._action_evidence_reserved, 0)

    def test_blocked_audit_writer_does_not_block_receipts_and_disconnect_flushes_both_events(self):
        runtime = self.create()
        state = runtime.connect()
        entered, release = threading.Event(), threading.Event()
        original = runtime._record
        def blocked_record(name, value):
            if name.startswith('operation-'):
                entered.set()
                if not release.wait(2):
                    raise OSError('test did not release audit writer')
            return original(name, value)
        errors, receipts = [], []
        def stop():
            try:
                receipts.append(runtime.action_receipt(dict(action='stop', stop_source='button')))
            except BaseException as error:
                errors.append(error)
        with mock.patch.object(runtime, '_record', side_effect=blocked_record):
            runtime.action_receipt(dict(action='start', mode='shadow',
                generation=state['hardware']['generation']))
            self.assertTrue(entered.wait(1))
            stopper = threading.Thread(target=stop)
            stopper.start()
            try:
                stopper.join(.4)
                self.assertFalse(stopper.is_alive(), 'STOP receipt waited for blocked audit writer')
                self.assertEqual(errors, [])
                self.assertTrue(receipts[0]['release_confirmed'])
                self.assertEqual(len(runtime._action_evidence), 2)
            finally:
                release.set()
                stopper.join(2)
            runtime.disconnect()
        self.assertEqual(len(runtime._action_evidence), 0)
        self.assertEqual(runtime._action_evidence_reserved, 0)
        records = [json.loads(path.read_text()) for path in runtime._evidence.glob('operation-*.json')]
        self.assertEqual(sorted(row['action'] for row in records), ['start', 'stop'])
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertTrue(runtime.closed_proof['release_confirmed'])

    def test_actual_one_shot_cycle_stop_interrupts_inference_and_discards_target(self):
        runtime = self.create()
        state = runtime.connect()
        runtime.action(dict(action='start', mode='hold', support_confirmed=True,
                            generation=state['hardware']['generation']))
        controller = runtime._controller
        with controller._lock:
            controller._hold_begin -= 4
        self.wait_for(lambda: runtime.snapshot()['hardware']['hold_ready'])
        state = runtime.snapshot()
        running = runtime.action(dict(action='start', mode='walk', support_confirmed=True,
                                      generation=state['hardware']['generation']))
        policy = runtime.sessions['walk']
        policy.entered, policy.release = threading.Event(), threading.Event()
        completed = threading.Event()
        errors = []
        def stop():
            try:
                runtime.action(dict(action='stop', generation=running['hardware']['generation']))
            except BaseException as error:
                errors.append(error)
            finally:
                completed.set()
        stopper = threading.Thread(target=stop)
        try:
            self.assertTrue(policy.entered.wait(1), 'actual OneShot cycle never entered inference')
            before = sum(kind == motion.TARGET for kind, _ in self.transports[0].requests)
            stopper.start()
            self.assertTrue(completed.wait(.4), 'STOP waited on the inference outer cycle lock')
            self.assertEqual(errors, [])
            self.assertFalse(self.transports[0].armed)
            policy.release.set()
            stopper.join(1)
            time.sleep(.05)
            self.assertEqual(sum(kind == motion.TARGET for kind, _ in self.transports[0].requests), before)
            self.assertEqual(controller.status()['mode'], 'off')
        finally:
            policy.release.set()
            if stopper.ident is not None:
                stopper.join(2)

    def test_changed_calibration_and_model_fail_before_uart(self):
        self.calibration_path.write_bytes(b'{}\n')
        with self.assertRaisesRegex(ValueError, 'calibration differs'):
            self.create()
        self.assertEqual(self.transports, [])
        self.calibration_path.write_bytes(app._calibration_bytes(self.calibration))
        (self.directory / 'policy/models/xgoduck_walk.onnx').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'model digest differs'):
            self.create()
        self.assertEqual(self.transports, [])

    def test_source_requires_only_existing_two_class_shapes(self):
        for source in ('class OneShotController: pass',
                       motion_workbench.WORKER.replace('class ReviewedTransport(motion.SerialTransport):',
                                                       'class ReviewedTransport(object):')):
            with self.assertRaises(ValueError):
                app.controller_classes(source)

    def test_real_binding_cache_is_fresh_only_after_explicit_closed_connect(self):
        self.use_real_serial_binding_factory()
        runtime = self.create()
        first = runtime.connect()
        first_namespace = runtime._namespace
        self.assertIs(first_namespace['bind_serial'].__globals__, first_namespace['serial_proof'].__globals__)
        runtime.disconnect()
        self.set_serial_node('/dev/ttyUSB0', 200)
        with self.assertRaisesRegex(ValueError, 'serial device changed'):
            first_namespace['serial_proof'](first_namespace['EXPECTED'])
        second = runtime.connect()
        self.assertEqual(second['phase'], 'ready')
        self.assertIsNot(runtime._namespace['_SERIAL_BINDINGS'], first_namespace['_SERIAL_BINDINGS'])
        self.assertEqual(runtime._namespace['EXPECTED'], first_namespace['EXPECTED'])
        self.assertGreater(second['hardware']['generation'], first['hardware']['generation'])
        runtime.disconnect()
        self.set_serial_node('/dev/ttyUSB1', 300)
        self.assertEqual(runtime.connect()['hardware']['port'], '/dev/ttyUSB1')
        self.assertEqual(self.factory_calls, 3)
        for transport in self.transports:
            self.assertNotIn(motion.ARM, [kind for kind, _ in transport.requests])
            self.assertNotIn(motion.TARGET, [kind for kind, _ in transport.requests])

    def test_live_node_change_faults_without_factory_or_auto_rebinding(self):
        self.use_real_serial_binding_factory()
        runtime = self.create(); runtime.connect()
        self.set_serial_node('/dev/ttyUSB0', 200)
        self.wait_for(lambda: runtime.snapshot()['phase'] == 'fault')
        state = runtime.snapshot()
        self.assertIn('serial device changed', state['error'])
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertFalse(state['closed_proof']['release_confirmed'])
        time.sleep(.15)
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(len(self.transports), 1)

    def test_factory_refusal_reports_fault_zero_open_and_explicit_retry(self):
        self.use_real_serial_binding_factory()
        runtime = self.create()
        self.health_fault = 'New unreviewed kernel fault'
        with self.assertRaisesRegex(ValueError, 'unreviewed kernel fault'):
            runtime.connect()
        state = runtime.snapshot()
        self.assertEqual(state['phase'], 'fault')
        self.assertTrue(state['closed_proof']['no_uart_opened'])
        self.assertTrue(state['closed_proof']['serial_closed'])
        self.assertEqual(self.transports, [])
        self.health_fault = None
        self.assertEqual(runtime.connect()['phase'], 'ready')
        self.assertEqual(self.factory_calls, 2)

    def test_no_factory_until_previous_thread_and_transport_are_actually_closed(self):
        self.use_real_serial_binding_factory()
        runtime = self.create(); runtime.connect(); runtime.disconnect()
        runtime._monitor.join(2)
        previous = runtime._controller
        with mock.patch.object(previous, '_thread', types.SimpleNamespace(is_alive=lambda: True)):
            with self.assertRaises(ValueError): runtime.connect()
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(len(self.transports), 1)
        self.assertEqual(runtime.connect()['phase'], 'ready')

    def test_actual_discovery_rejects_nonunique_or_wrong_ch340_before_uart(self):
        self.use_real_serial_binding_factory()
        runtime = self.create()
        node, identity = self.serial_inventory['/dev/ttyUSB0']
        self.serial_inventory['/dev/ttyUSB1'] = (types.SimpleNamespace(st_dev=5, st_ino=200,
            st_rdev=os.makedev(188, 1), st_mode=stat.S_IFCHR | 0o600), dict(identity, path='/dev/ttyUSB1'))
        with self.assertRaisesRegex(ValueError, 'Multiple matching'):
            runtime.connect()
        self.assertEqual(self.transports, [])
        self.serial_inventory = {'/dev/ttyUSB0': (node, dict(identity, vid='ffff'))}
        with self.assertRaisesRegex(ValueError, 'not found'):
            runtime.connect()
        self.assertEqual(self.transports, [])

    def test_actual_bootstrap_factory_rejects_changed_first_runtime_inode_without_open(self):
        from test_deploy_native_workbench import NativeConnectionFactoryTests
        fixture = NativeConnectionFactoryTests()
        with fixture.factory_fixture() as (namespace, model, _):
            self.guard = namespace['connection_guard_factory'](initial=True)
            runtime = self.create()
            model.inode = 200  # The device changed while CPU models loaded.
            with self.assertRaisesRegex(ValueError, 'initial UART'):
                runtime.connect()
            state = runtime.snapshot()
            self.assertEqual(state['phase'], 'fault')
            self.assertTrue(state['closed_proof']['serial_closed'])
            self.assertTrue(state['closed_proof']['no_uart_opened'])
            self.assertEqual(state['closed_proof']['serial_open_attempts'], 0)
            self.assertEqual(self.transports, [])
            self.assertTrue(namespace['initial_connection_gate_pending'])


if __name__ == '__main__':
    unittest.main()
