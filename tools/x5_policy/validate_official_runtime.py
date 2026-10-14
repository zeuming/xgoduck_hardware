"""Behavioral checks against the actual packaged controller, with fake UART only."""
import argparse
import ast
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import types
import unittest
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--candidate', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(args.candidate / 'host/policy'))
import motion
from rl_core import POLICY_INDEX, MOTOR_IDS
from test_motion import FakeTransport, FakePolicy, FakeClock, ready_config


class Transport(FakeTransport):
    def request(self, kind, payload=b''):
        if kind == motion.TARGET:
            if len(payload) != 35 or payload[-1] != 1: raise AssertionError('Expected direct position target')
            result = super().request(kind, payload[:34])
            self.requests[-1] = (kind, payload)
            return result
        return super().request(kind, payload)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = FakeClock()
        self.transport = Transport('fake-only')
        self.transport.gyro = [0., 0., 0.]
        cfg = ready_config(); cfg['imu']['bias_dps'] = [0.,0.,0.]
        self.path = Path(self.temp.name) / 'calibration.json'
        self.path.write_text(json.dumps(cfg))
        self.sessions = {n:FakePolicy(n) for n in motion.MODEL_MODES}
        self.c = motion.MotionController(self.sessions, self.path,
            transport_factory=lambda port:self.transport, clock=self.clock)
        self.c._connect_once()

    def refresh(self, tilt=0.):
        self.c._accept_state_locked(self.c._read_state())
        self.c._filtered.grav = [math.sin(math.radians(tilt)), 0., -math.cos(math.radians(tilt))]

    def step(self, tilt=0., dt=.02):
        self.clock.advance(dt); self.refresh(tilt)
        self.c.heartbeat(expected_generation=self.c._generation)
        self.c._control_once()

    def start_walk(self):
        self.c.start_mode('walk', expected_generation=self.c._generation)

    def test_construction_and_connect_never_arm(self):
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,p in self.transport.requests))
        self.assertEqual(self.c._mode,'off')
        self.assertNotIn('ResearchMotionController',[x.__name__ for x in type(self.c).__mro__])

    def test_old_firmware_refused_before_arm(self):
        t = Transport('fake-only'); original = t.request
        t.request = lambda kind,payload=b'': b'old' if kind==motion.OFFICIAL_RUNTIME_CAPS else original(kind,payload)
        c=motion.MotionController(self.sessions,self.path,transport_factory=lambda p:t,clock=self.clock)
        with self.assertRaises(motion.MotionError): c._connect_once()
        self.assertFalse(any(k==motion.ARM for k,p in t.requests))
        self.assertFalse(c._connected)
        self.assertIsNone(c._transport)

    def test_direct_start_and_full_official_commands_without_warmup(self):
        self.start_walk()
        for values in ((.4,.3,1.),(-.4,-.3,-1.),(0.,0.,0.),(.3,0.,0.)):
            self.c.command(*values);self.step()
            np.testing.assert_allclose(self.sessions['walk'].observation[0,48:51], values)
        self.assertEqual(self.c._mode,'walk')
        with self.assertRaises(motion.MotionError): self.c.command(.4001,0,0)

    def test_no_two_or_thirty_second_timeout(self):
        self.start_walk();self.c.command(.3,0,0)
        for _ in range(1700): self.step()
        self.assertEqual(self.c._mode,'walk')
        self.assertGreater(float(self.c._command[0]),.29)
        self.assertTrue(self.transport.armed)

    def test_hold_is_original_home_without_interpolation(self):
        self.c.start_mode('hold',expected_generation=self.c._generation)
        self.step()
        np.testing.assert_array_equal(self.c._last_executed_target,self.c._stand_deg)
        self.assertTrue(self.c._hold_ready_locked())

    def test_head_outputs_and_leg_outputs_are_not_overwritten(self):
        self.sessions['walk'].output[:] = .05
        self.start_walk(); self.step()
        np.testing.assert_array_equal(self.c._last_action,self.c._last_executed_target)
        self.assertGreater(self.c._last_action[MOTOR_IDS.index(32)],0.)

    def test_15_degree_crossing_does_not_release(self):
        self.start_walk(); self.step(tilt=20.)
        self.assertTrue(self.transport.armed)
        self.assertEqual(self.c._recovery.phase,'walk')

    def test_official_fall_default_getup_and_return_preserve_torque(self):
        self.start_walk();self.c.command(.3,.2,.5)
        self.c._policy_profiles['getup']['home_rad'] = self.c._policy_profiles['getup']['home_rad'].copy()
        self.c._policy_profiles['getup']['home_rad'][0] = .1
        for _ in range(7): self.step(tilt=60.)
        self.assertEqual(self.c._recovery.phase,'walk')
        self.step(tilt=60.)
        self.assertEqual(self.c._recovery.phase,'default_pose')
        self.assertAlmostEqual(self.c._last_executed_target[0],math.degrees(.1),places=5)
        for _ in range(51): self.step(tilt=60.)
        self.assertEqual(self.c._recovery.phase,'getup')
        self.assertGreater(self.sessions['getup'].calls,0)
        np.testing.assert_array_equal(self.sessions['getup'].observation[0,48:51],[0,0,0])
        for _ in range(52): self.step(tilt=5.)
        self.assertEqual(self.c._recovery.phase,'walk')
        self.assertTrue(self.transport.armed)

    def test_heartbeat_matches_official_second_and_stop_wins(self):
        self.start_walk();self.clock.advance(.8);self.refresh();self.c._control_once()
        self.assertEqual(self.c._mode,'walk')
        self.clock.advance(.21);self.refresh();self.c._control_once()
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.transport.armed)

    def test_saved_range_and_fault_checks_remain(self):
        self.start_walk()
        with self.assertRaises(motion.MotionError): self.c._raw_targets_locked([999.]*15)
        self.transport.fault=1;self.refresh()
        with self.assertRaises(motion.MotionError):self.c._motion_gate_locked()

    def test_stale_state_rejected_without_timestamp_refresh(self):
        self.clock.advance(.151)
        sample = self.c._sample_received
        with self.assertRaises(motion.MotionError):self.c._motion_gate_locked('hold')
        self.assertEqual(sample,self.c._sample_received)

    def test_old_forward_button_cannot_start_continuous_motion(self):
        self.start_walk()
        with self.assertRaises(motion.MotionError): self.c.forward_trial(expected_generation=self.c._generation)
        np.testing.assert_array_equal(self.c._command,[0,0,0])

    def test_ema_and_raw_history_2048_frames(self):
        spec=importlib.util.spec_from_file_location('official_oracle',ROOT/'output/xgoduck_runtime_upstream/python/policy.py')
        ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)
        p=ref.Policy();p.session=FakePolicy();p.in_name='obs';p._outputs=['actions'];p._feed={'obs':p._obs.reshape(1,61)}
        previous=np.zeros(14,np.float32);smooth=previous.copy();rng=np.random.default_rng(17)
        for _ in range(2048):
            raw=rng.uniform(-.7,.7,(1,14)).astype(np.float32);p.session.output=raw
            expected=p.infer([0,0,-1],[0]*3,[0]*15,[0]*15)
            target,actual,smooth,_=self.c._decode_policy_action('walk',raw,previous,smooth,0,0)
            np.testing.assert_array_equal(actual,raw[0]);np.testing.assert_array_equal(smooth,expected)
            expected_target=np.zeros(15,np.float32)
            expected_target[POLICY_INDEX]=np.rad2deg(self.c._policy_profiles['walk']['home_rad']+expected)
            np.testing.assert_array_equal(target,expected_target);previous=actual

    def test_status_copy_cannot_mutate_live_recovery(self):
        from status_snapshot import capture_display_view
        view=capture_display_view(self.c)
        view._recovery.phase='getup'
        self.assertEqual(self.c._recovery.phase,'walk')

    def test_hold_requires_heartbeat_and_advertises_it(self):
        self.c.start_mode('hold',expected_generation=self.c._generation)
        self.assertNotEqual(self.c.status()['hold_control_protocol'],2)
        self.clock.advance(1.01);self.refresh();self.c._control_once()
        self.assertEqual(self.c._mode,'off')
        self.assertFalse(self.transport.armed)

    def test_stop_during_inference_discards_returned_target(self):
        self.start_walk();entered=threading.Event();release=threading.Event()
        original=self.sessions['walk'].run
        def blocked(*args,**kwargs):
            entered.set();assert release.wait(3);return original(*args,**kwargs)
        self.sessions['walk'].run=blocked
        failures=[]
        def control():
            try:self.c._control_once()
            except BaseException as e:failures.append(e)
        thread=threading.Thread(target=control);thread.start()
        try:
            self.assertTrue(entered.wait(3))
            self.c.stop()
            count=sum(k==motion.TARGET for k,p in self.transport.requests)
        finally:release.set();thread.join(3)
        self.assertFalse(thread.is_alive());self.assertFalse(failures)
        self.assertEqual(count,sum(k==motion.TARGET for k,p in self.transport.requests))
        self.assertFalse(self.transport.armed)

    def test_actual_native_request_wrapper_allows_only_empty_capability(self):
        tree=ast.parse((args.candidate/'host/service/worker-classes.py').read_text())
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ReviewedTransport')
        ns=dict(motion=types.SimpleNamespace(**{**vars(motion),'SerialTransport':Transport}),
            ProbeError=motion.MotionError,health_error=None,time=time,EXPECTED={},request_counts={},serial_proof=lambda *a:None)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual native wrapper>','exec'),ns)
        t=ns['ReviewedTransport']('memory-only');t.serial=types.SimpleNamespace(fileno=lambda:123)
        self.assertEqual(t.request(27),bytes([27,0]))
        with self.assertRaises(Exception):t.request(27,b'x')

    def test_actual_native_controller_status_and_stop_integration(self):
        tree=ast.parse((args.candidate/'host/service/worker-classes.py').read_text())
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='OneShotController')
        ns=dict(motion=motion,time=time,request_counts={})
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual native controller>','exec'),ns)
        c=ns['OneShotController'](self.sessions,self.path,transport_factory=lambda p:self.transport,clock=self.clock)
        c._connect_once();c.start_mode('walk',expected_generation=c._generation)
        c.command(.3,0,0);self.clock.advance(.02);c._cycle()
        self.assertEqual(c.status()['official_runtime']['recovery']['phase'],'walk')
        self.assertIsNone(c.status()['model_start_blocked_reasons']['walk'])
        c.stop(source='button')
        self.assertTrue(c.status()['release_confirmed']);self.assertFalse(self.transport.armed)

    def test_head_projection_uses_saved_endpoints_and_never_changes_legs(self):
        self.c._config['joints'][13].update(min=2020,max=2050)
        angles=list(range(15));out=self.c._execution_target_locked('walk',angles)
        self.assertEqual(out[:10],angles[:10])
        self.assertNotEqual(out[13],angles[13])
        self.assertEqual(self.c._head_projection[0]['id'],33)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RuntimeTests))
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        errors=[str(x) for x in result.errors+result.failures],hardware_access=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
