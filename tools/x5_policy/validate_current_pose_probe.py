"""Validate against the exact staged native controller with an in-memory UART."""
import argparse
import copy
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--paired-feedback', action='store_true',
                        help='Use full v2 sample-bound packets with the offline fake plant')
    args = parser.parse_args()
    sys.path[:0] = [str(args.source.resolve()/'policy'), str(args.source.resolve()/'service'),
                   str(Path(__file__).resolve().parent)]
    def deny_hardware(event, values):
        if event == 'open' and values and isinstance(values[0], (str, bytes)):
            path = os.path.realpath(os.fsdecode(values[0]))
            if path.startswith(('/dev/tty', '/dev/serial', '/dev/cu.', '/dev/bus/usb')):
                raise PermissionError('Offline validation may not open hardware')
    sys.addaudithook(deny_hardware)
    import motion
    from test_motion import FakeClock, FakeTransport, FakePolicy, ready_config
    if args.paired_feedback:
        from causal_test_transport import PairedFakeTransport
        FakeTransport = PairedFakeTransport
    from current_pose_probe import controller_class, offset_at, DURATION_S
    cls = motion.MotionController
    if not hasattr(cls, 'current_pose_probe'):
        cls = controller_class(cls, motion.MotionError, motion.MOTOR_IDS, motion.ALL_MASK, motion.TARGET)

    class Transport(FakeTransport):
        stuck = False
        arm_drift = False
        def request(self, kind, payload=b''):
            old = list(self.positions)
            result = super().request(kind, payload)
            if kind == motion.ARM and self.arm_drift:
                self.positions[2] += 5
            if kind == motion.TARGET and self.stuck:
                self.positions[1] = old[1]
            return result

    class Checks(unittest.TestCase):
        def setUp(self):
            self.tmp = tempfile.TemporaryDirectory()
            self.addCleanup(self.tmp.cleanup)
            self.clock = FakeClock()
            cfg = Path(self.tmp.name)/'config.json'
            cfg.write_text(json.dumps(ready_config()))
            self.t = Transport('fake-only')
            self.t.positions = [2100,2200,2300,2000,2150,2050,2120,2250,2150,2100,2047,2048,2049,2050,2047]
            self.policy = FakePolicy()
            self.c = cls({'walk': self.policy}, cfg, clock=self.clock,
                         transport_factory=lambda port:self.t, allow_fixed_head_walk=True)
            self.c._connect_once()
            self.initial = list(self.t.positions)
            self.calibration = copy.deepcopy(self.c._config)

        def start(self, **options):
            values = dict(expected_generation=self.c._generation,
                          support_confirmed=True, clearance_confirmed=True)
            values.update(options)
            return self.c.current_pose_probe(**values)

        def cycle(self, count=1):
            for _ in range(count):
                self.clock.advance(.02)
                self.c._cycle()

        def targets(self):
            return [list(struct.unpack('<I15H', p)[1:]) for k,p in self.t.requests if k==motion.TARGET]

        def test_relative_path_never_visits_home_and_normal_completion_keeps_torque(self):
            self.start(); self.c._control_once()
            self.assertEqual(self.targets()[0], self.initial)
            self.c._heartbeat = -1000  # Independent HOLD, no webpage lease.
            self.cycle(320)
            p=self.c.status()['current_pose_probe']
            self.assertEqual(p['phase'], 'completed')
            self.assertEqual(self.targets()[-1], self.initial)
            self.assertTrue(self.t.armed)
            self.assertEqual(self.t.torque,motion.ALL_MASK)
            self.assertEqual(self.c._mode,'hold')
            self.assertFalse(self.c._hold_ready_locked())
            self.assertEqual(self.policy.calls,0)
            self.assertEqual(self.c._config,self.calibration)
            for values in self.targets():
                self.assertEqual(values[:1]+values[2:],self.initial[:1]+self.initial[2:])
                self.assertGreaterEqual(values[1],self.initial[1])
                self.assertLessEqual(values[1],self.initial[1]+34)
            self.assertGreater(p['result']['plateau_samples'],10)
            self.assertAlmostEqual(p['result']['mean_plateau_delta_deg'],-3,delta=.09)

        def test_stuck_encoder_is_reported_not_called_physical_success(self):
            self.t.stuck=True;self.start();self.cycle(320)
            result=self.c.status()['current_pose_probe']['result']
            self.assertAlmostEqual(result['mean_plateau_delta_deg'],0)
            self.assertFalse(result['physical_support_verified'])
            self.assertTrue(result['encoder_following_only'])

        def test_arm_time_position_change_is_recaptured_before_first_goal(self):
            self.t.arm_drift=True;self.start();latched=list(self.t.positions)
            self.c._control_once()
            self.assertNotEqual(latched,self.initial)
            self.assertEqual(self.targets()[0],latched)

        def test_missing_support_clearance_old_generation_reject_before_arm(self):
            for options in ({'support_confirmed':False},{'clearance_confirmed':False},
                            {'expected_generation':self.c._generation-1}):
                with self.subTest(options=options):
                    with self.assertRaises(motion.MotionError):self.start(**options)
                    self.assertFalse(any(k==motion.ARM for k,_ in self.t.requests))

        def test_endpoint_out_of_range_rejects_before_arm(self):
            self.c._config['joints'][1]['max']=self.initial[1]+10
            with self.assertRaises(motion.MotionError):self.start()
            self.assertFalse(any(k==motion.ARM for k,_ in self.t.requests))

        def test_actual_fault_or_missing_feedback_rejects_before_arm(self):
            self.t.online &= ~2;self.cycle()
            with self.assertRaises(motion.MotionError):self.start()
            self.assertFalse(self.t.armed)

        def test_target_boundary_rejects_any_other_joint_change(self):
            self.start();bad=list(self.initial);bad[2]+=1
            n=len(self.targets())
            with self.assertRaises(motion.MotionError):
                self.c._request(motion.TARGET,struct.pack('<I15H',self.c._cookie,*bad))
            self.assertEqual(len(self.targets()),n)

        def test_stale_feedback_aborts_with_original_release_path(self):
            self.start();self.t.freeze_sample=True;self.cycle(12)
            self.assertEqual(self.c._mode,'off')
            self.assertFalse(self.t.armed)
            self.assertEqual(self.c.status()['current_pose_probe']['phase'],'aborted')

        def test_tilt_fault_still_releases_and_never_marks_complete(self):
            self.start();self.t.acc=[9.8,0,0];self.cycle(30)
            self.assertEqual(self.c._mode,'off')
            self.assertFalse(self.t.armed)
            self.assertIsNone(self.c.status()['current_pose_probe']['result'])

        def test_stop_blocks_late_target_and_one_connection_cannot_rearm(self):
            self.start();self.cycle(50);self.c.stop()
            n=len(self.targets());self.cycle(320)
            self.assertEqual(len(self.targets()),n)
            self.assertFalse(self.t.armed)
            self.assertIsNone(self.c._hold_head_deg)
            with self.assertRaises(motion.MotionError):self.start()

        def test_other_modes_and_old_probe_cannot_take_over(self):
            self.start()
            for mode in ('hold','walk','shadow'):
                with self.assertRaises(motion.MotionError):
                    self.c.start_mode(mode,support_confirmed=True)
            with self.assertRaises(motion.MotionError):
                self.c.tracking_probe(expected_generation=self.c._generation,support_confirmed=True,joint_id=11)
            self.cycle(320)
            with self.assertRaises(motion.MotionError):
                self.c.start_mode('walk',support_confirmed=True)

        def test_repeated_stop_preserves_completed_result(self):
            self.start();self.cycle(320)
            result=self.c.status()['current_pose_probe']['result']
            self.c.stop();self.c.stop()
            probe=self.c.status()['current_pose_probe']
            self.assertEqual(probe['phase'],'stopped')
            self.assertEqual(probe['result'],result)

        def test_clock_regression_does_not_command_a_new_pose(self):
            self.start();self.clock.advance(-.1)
            with self.assertRaises((motion.MotionError,ValueError)):
                self.c._control_once()
            self.assertFalse(self.targets())

        def test_public_waveform_is_bounded_and_returns_to_zero(self):
            self.assertEqual(offset_at(0),0)
            self.assertEqual(offset_at(DURATION_S),0)
            self.assertEqual(offset_at(DURATION_S+10),0)
            with self.assertRaises(ValueError):offset_at(float('nan'))

        def test_api_rejects_missing_confirmation_old_generation_and_active_mode(self):
            import native_workbench as web
            from test_native_workbench import Runtime, SOURCE, PASSWORD, CLIENT
            root=Path(self.tmp.name)/'web';root.mkdir()
            (root/'index.html').write_bytes(SOURCE.read_bytes())
            runtime=Runtime();wb=web.NativeWorkbench(runtime,web_root=root,password=PASSWORD)
            body=dict(action='current-pose-probe',page_generation=wb.generation,
                      generation=5,control_id=CLIENT,support_confirmed=True,clearance_confirmed=True)
            for changes in ({'support_confirmed':False},{'clearance_confirmed':1},
                            {'generation':4},{'joint_id':22},{'amplitude_deg':10}):
                with self.assertRaises(ValueError):wb.motion_action(dict(body,**changes),live=True)
            self.assertFalse(runtime.calls)
            runtime.motion['hardware']['mode']='hold'
            with self.assertRaises(ValueError):wb.motion_action(body,live=True)
            self.assertFalse(runtime.calls)

        def test_api_routes_valid_request_to_short_control_receipt(self):
            import native_workbench as web
            from test_native_workbench import Runtime, SOURCE, PASSWORD, CLIENT
            root=Path(self.tmp.name)/'web';root.mkdir();(root/'index.html').write_bytes(SOURCE.read_bytes())
            class ProbeRuntime(Runtime):
                def action_receipt(inner,body):
                    inner.calls.append(('receipt',copy.deepcopy(body)))
                    h=inner.motion['hardware'];h.update(generation=6,mode='hold',release_confirmed=False)
                    return dict(action_receipt_protocol=1,action=body['action'],connection_sequence=1,
                                generation=6,mode='hold',connected=True,release_confirmed=False)
            runtime=ProbeRuntime();wb=web.NativeWorkbench(runtime,web_root=root,password=PASSWORD)
            body=dict(action='current-pose-probe',page_generation=wb.generation,generation=5,
                      control_id=CLIENT,support_confirmed=True,clearance_confirmed=True)
            result=wb.motion_action(body,live=True)
            self.assertEqual(result['action_receipt_protocol'],1)
            self.assertEqual(runtime.calls,[('receipt',body)])

        def test_native_receipt_runs_real_probe_and_enqueues_evidence(self):
            import threading
            from unittest.mock import Mock
            from native_runtime import NativeRuntime
            runtime=NativeRuntime.__new__(NativeRuntime)
            runtime.lock=threading.RLock()
            runtime._controller=self.c
            runtime._namespace=dict(health_error=None,exit_event=threading.Event())
            runtime._connection_sequence=1
            runtime.phase='ready';runtime.error=None
            runtime._reserve_action_evidence=Mock()
            runtime._publish_owned_hardware=Mock()
            runtime._queue_action_evidence=Mock()
            runtime._remember_timing=Mock()
            runtime._timing_snapshot=lambda:{}
            runtime.control_context=lambda:dict(connection_sequence=1,
                hardware=dict(self.c.status(),release_confirmed=not self.t.armed and not self.t.torque))
            runtime._persist=Mock(side_effect=AssertionError('synchronous evidence IO'))
            runtime._record=Mock(side_effect=AssertionError('synchronous evidence IO'))
            receipt=runtime.action_receipt(dict(action='current-pose-probe',
                generation=self.c._generation,support_confirmed=True,clearance_confirmed=True))
            self.assertEqual(receipt['mode'],'hold')
            self.assertEqual(self.c.status()['current_pose_probe']['phase'],'running')
            runtime._reserve_action_evidence.assert_called_once()
            runtime._queue_action_evidence.assert_called_once()
            event=runtime._queue_action_evidence.call_args.args[0]
            self.assertEqual(event['action'],'current-pose-probe')
            self.assertEqual(event['state'],'complete')
            self.assertEqual(event['after']['current_pose_probe']['phase'],'running')
            runtime._persist.assert_not_called();runtime._record.assert_not_called()

    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Checks))
    args.output.write_text(json.dumps(dict(tests=result.testsRun,failures=len(result.failures),
        errors=len(result.errors),passed=result.wasSuccessful(),hardware_opened=False,
        fake_feedback_protocol=2 if args.paired_feedback else 1),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__=='__main__':
    main()
