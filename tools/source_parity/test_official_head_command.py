"""Actual packaged controller, original ONNX contract, memory-only transport."""
import ast
import copy
import json
import math
import threading
import types
import unittest
from pathlib import Path
import numpy as np
import test_source_host as base

motion, args = base.motion, base.args
PROFILE = [-0.2733776720317895, 0.09951392995630493, 0., 0.]


class HeadCommandTests(base.RuntimeTests):
    def profiled(self, profile=None):
        # Resolve at call time so an explicitly supplied evidence profile can
        # reuse these checks without retaining the module's first default.
        if profile is None:
            profile = PROFILE
        transport = base.Transport('memory-only')
        transport.gyro = [0., 0., 0.]
        sessions = {n:base.FakePolicy(n) for n in motion.MODEL_MODES}
        c = motion.MotionController(sessions, self.path,
            transport_factory=lambda p:transport, clock=self.clock,
            walk_head_command_rad=profile)
        self.assertFalse(transport.requests)
        c._connect_once()
        self.assertFalse(any(k == motion.ARM for k,p in transport.requests))
        return c, transport, sessions

    def test_head_command_changes_only_original_four_input_fields(self):
        c,t,s = self.profiled()
        previous = np.linspace(-.1,.1,14,dtype=np.float32)
        command = np.array([.3,.02,-.1],np.float32)
        st = copy.deepcopy(self.c._filtered)
        expected = self.c._prepare_policy_observation('walk', st, previous, command,0.)
        actual = c._prepare_policy_observation('walk',st,previous,command,0.,
            head_command=c._walk_head_command_locked())
        keep = [i for i in range(61) if not 51 <= i < 55]
        np.testing.assert_array_equal(actual[:,keep], expected[:,keep])
        np.testing.assert_array_equal(actual[0,51:55],np.asarray(PROFILE,np.float32))
        np.testing.assert_array_equal(c._policy_profiles['walk']['home_rad'],
                                      self.c._policy_profiles['walk']['home_rad'])

    def test_fixed_stance_changes_only_four_head_goals_and_not_home(self):
        c,t,s = self.profiled()
        original = list(c._stand_deg)
        target = c._fixed_stance_locked()
        np.testing.assert_array_equal(c._stand_deg, original)
        head = dict(zip((30,31,32,33), PROFILE))
        for i,sid in enumerate(motion.MOTOR_IDS):
            self.assertAlmostEqual(target[i],original[i]+math.degrees(head.get(sid,0.)),places=10)
        c.start_mode('hold',expected_generation=c._generation)
        self.clock.advance(.02);c._cycle()
        np.testing.assert_array_equal(c._last_executed_target,target)

    def test_actor_outputs_not_offset_and_history_unchanged_by_profile(self):
        c,t,s = self.profiled()
        rng=np.random.default_rng(44)
        previous=np.zeros(14,np.float32);smooth=previous.copy()
        for _ in range(120):
            raw=rng.uniform(-.4,.4,(1,14)).astype(np.float32)
            expected=self.c._decode_policy_action('walk',raw,previous,smooth,0.,0.)
            actual=c._decode_policy_action('walk',raw,previous,smooth,0.,0.)
            for a,b in zip(actual[:3],expected[:3]):np.testing.assert_array_equal(a,b)
            previous,smooth=actual[1:3]

    def test_profile_not_applied_to_official_recovery_model(self):
        c,t,s=self.profiled();c.start_mode('walk',expected_generation=c._generation)
        for _ in range(60):
            self.clock.advance(.02);c._accept_state_locked(c._read_state())
            c._filtered.grav=[math.sin(math.radians(60)),0.,-math.cos(math.radians(60))]
            c.heartbeat(expected_generation=c._generation);c._control_once()
        self.assertEqual(c._recovery.phase,'getup');self.assertGreater(s['getup'].calls,0)
        np.testing.assert_array_equal(s['getup'].observation[0,51:61],np.zeros(10))

    def test_pinned_official_onnx_receives_unmapped_profile_input(self):
        import onnxruntime as ort
        from rl_core import build_obs
        c,t,s=self.profiled();st=copy.deepcopy(c._filtered)
        previous=np.linspace(-.2,.2,14,dtype=np.float32);command=np.array([.3,0.,0.],np.float32)
        expected=build_obs(st,previous,command,np.asarray(PROFILE,np.float32),
            home_rad=c._policy_profiles['walk']['home_rad']).reshape(1,61)
        actual=c._prepare_policy_observation('walk',st,previous,command,0.,head_command=c._walk_head_command_locked())
        np.testing.assert_array_equal(actual,expected)
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        session=ort.InferenceSession(str(base.ROOT/'output/xgoduck_runtime_upstream/python/xgoduck_walk.onnx'),options,providers=['CPUExecutionProvider'])
        key=session.get_inputs()[0].name
        np.testing.assert_array_equal(session.run(None,{key:actual})[0],session.run(None,{key:expected})[0])

    def test_command_copied_and_status_cannot_mutate_profile(self):
        original=PROFILE.copy();c,t,s=self.profiled(original);original[0]=.99
        command=c._walk_head_command_locked();command[0]=.88
        status=c.status();status['walk_head_command_rad'][0]=.77
        np.testing.assert_array_equal(c._walk_head_command_locked(),np.asarray(PROFILE,np.float32))

    def test_invalid_profiles_reject_before_transport_creation(self):
        for profile in ([0]*3,[0]*5,[True,0,0,0],[float('nan'),0,0,0],
                        [float('inf'),0,0,0],[1.00001,0,0,0],['0',0,0,0]):
            with self.subTest(profile=profile):
                with self.assertRaises(motion.MotionError):self.profiled(profile)

    def test_profiled_actual_cycle_uses_head_command_and_original_decode(self):
        c,t,s=self.profiled()
        s['walk'].output[:]=.025
        c.start_mode('walk',expected_generation=c._generation)
        before=c._previous_action.copy();smooth=c._smooth_action.copy()
        self.clock.advance(.02);c._cycle()
        np.testing.assert_array_equal(s['walk'].observation[0,51:55],np.asarray(PROFILE,np.float32))
        expected=self.c._decode_policy_action('walk',s['walk'].output,before,smooth,0.,0.)
        np.testing.assert_array_equal(c._last_action,expected[0])

    def test_stop_during_full_state_and_inference_cycle_discards_target(self):
        c,t,s=self.profiled();c.start_mode('walk',expected_generation=c._generation)
        entered,release=threading.Event(),threading.Event();original=s['walk'].run
        def blocked(*a,**kw):
            entered.set();assert release.wait(3);return original(*a,**kw)
        s['walk'].run=blocked;failures=[]
        def cycle():
            try:c._cycle()
            except BaseException as e:failures.append(e)
        thread=threading.Thread(target=cycle);thread.start()
        try:
            self.assertTrue(entered.wait(3));c.stop()
            targets=sum(k in (motion.TARGET,motion.SOURCE_TARGET) for k,p in t.requests)
        finally:release.set();thread.join(3)
        self.assertFalse(thread.is_alive());self.assertFalse(failures)
        self.assertEqual(targets,sum(k in (motion.TARGET,motion.SOURCE_TARGET) for k,p in t.requests))
        self.assertFalse(t.armed)
        state=c._read_state()
        self.assertEqual(state['torque_known_mask'],motion.ALL_MASK)
        self.assertEqual(state['torque_mask'],0)

    def test_profile_cannot_bypass_leg_range(self):
        c,t,s=self.profiled();c.start_mode('walk',expected_generation=c._generation)
        s['walk'].output[0,0]=100.
        count=sum(k in (motion.TARGET,motion.SOURCE_TARGET) for k,p in t.requests)
        self.clock.advance(.02)
        with self.assertRaisesRegex(motion.MotionError,'目标超过'):c._cycle()
        self.assertEqual(count,sum(k in (motion.TARGET,motion.SOURCE_TARGET) for k,p in t.requests))
        c.stop()
        self.assertFalse(t.armed)

    def test_native_close_without_worker_still_disconnects(self):
        tree=ast.parse((args.candidate/'service/native_runtime.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='NativeRuntime')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='close')
        ns={};exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual native close>','exec'),ns)
        calls=[];obj=types.SimpleNamespace(_lifecycle=threading.RLock(),lock=threading.RLock(),
             _policy_worker=None,disconnect=lambda:calls.append('disconnect'))
        ns['close'](obj);self.assertTrue(obj._closed);self.assertEqual(calls,['disconnect'])


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(HeadCommandTests))
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        errors=[str(x) for x in result.errors+result.failures],hardware_access=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
