"""CPU-only tests of actual native guard injection and candidate integration."""
import hashlib
import copy
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(HERE/'source/policy'), str(HERE/'source/service'), str(ROOT/'tools/x5_policy')]
import motion
import native_runtime as app
import test_native_runtime as fixture
from test_motion import FakeClock, FakePolicy, FakeTransport, ready_config


class NativeZeroAdapterTests(unittest.TestCase):
    wait_for = fixture.NativeRuntimeTests.wait_for
    setUp = fixture.NativeRuntimeTests.setUp
    tearDown = fixture.NativeRuntimeTests.tearDown

    def test_failed_first_inference_retains_latency_without_TARGET_or_refresh(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c.start_mode('hold',support_confirmed=True)
        for _ in range(155):
            c._clock.advance(.02);c._cycle()
        c.start_mode('walk',support_confirmed=True,expected_generation=c.status()['generation'])
        c.heartbeat(expected_generation=c.status()['generation'],minimal=True)
        t=c._transport
        before=sum(kind==motion.TARGET for kind,_ in t.requests)
        policy=c.sessions['walk'];original=policy.run
        def delayed(*args,**kwargs):
            value=original(*args,**kwargs);c._clock.advance(.12);return value
        policy.run=delayed;c._thread=threading.current_thread()
        try:
            with self.assertRaisesRegex(motion.MotionError,'舵机反馈已过期'):
                c._cycle()
            timing=c._inference_attempts['control']
            self.assertGreaterEqual(timing['feedback_age_after_ms'],119.9)
            self.assertEqual(timing['sample_us'],timing['current_sample_us'])
            self.assertEqual(c._last_control_attempt['error'],'舵机反馈已过期')
            self.assertEqual(sum(kind==motion.TARGET for kind,_ in t.requests),before)
            self.assertGreaterEqual(c._age_locked(),.1199)
            c.stop()
            trace=c.completed_trial_traces()[-1]
            self.assertEqual(trace['latency_at_stop']['control']['error'],'舵机反馈已过期')
            trace['latency_at_stop']['control']['error']='caller mutation'
            self.assertEqual(c.completed_trial_traces()[-1]['latency_at_stop']['control']['error'],'舵机反馈已过期')
        finally:
            c._thread=None;c.close()

    def prepare(self):
        self.calibration = ready_config()
        # CPU fixtures bind the unchanged standing-reference gate to a
        # synthetic calibration; no private robot calibration is published.
        if not hasattr(self, '_synthetic_reference_patch'):
            fingerprint = hashlib.sha256(json.dumps(self.calibration,
                sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
            self._synthetic_reference_patch = mock.patch.object(motion,
                'STANDING_REFERENCE_CALIBRATION_SHA256', fingerprint)
            self._synthetic_reference_patch.start()
            self.patches.append(self._synthetic_reference_patch)
        raw = app._calibration_bytes(self.calibration)
        self.calibration_path.write_bytes(raw)
        self.deployment['calibration_sha256'] = hashlib.sha256(raw).hexdigest()
        _, matrix = motion._axis_matrix(self.calibration['imu']['axes'])
        self.initial_feedback = dict(positions=[r['zero'] for r in self.calibration['joints']],
            acc=(matrix.T @ [0.,0.,9.8]).tolist(),
            gyro=self.calibration['imu']['bias_dps'].copy())

    def create(self):
        self.prepare()
        self.runtime = app.NativeRuntime(self.guard, self.directory, self.deployment,
            self.calibration, fixture.motion_workbench.WORKER,
            require_head_clearance=True, allow_fixed_head_walk=True, hold_hip_pitch_bias_deg=0)
        return self.runtime

    def controller(self, transport=True):
        self.prepare()
        policies = {n:FakePolicy(n) for n in app.MODEL_NAMES}
        def factory(port):
            t = FakeTransport(port)
            t.positions = [r['zero'] for r in self.calibration['joints']]
            t.acc=self.initial_feedback['acc'].copy()
            t.gyro=self.initial_feedback['gyro'].copy()
            return t
        return motion.MotionController(policies, self.calibration_path,
            transport_factory=factory if transport else None,
            clock=FakeClock(), allow_fixed_head_walk=True)

    def test_runtime_passes_selected_joint_to_receipt_and_full_action(self):
        runtime = self.create()
        runtime.connect()
        c = runtime._controller
        generation = c.status()['generation']
        runtime.action_receipt(dict(action='capture-head-clearance', generation=generation,
                                    clearance_confirmed=True))
        for sid, live in ((14, True), (11, False)):
            generation = c.status()['generation']
            started = runtime.action_receipt(dict(action='start', mode='hold', generation=generation,
                                                  support_confirmed=True))
            deadline = time.monotonic() + 6.
            while time.monotonic() < deadline:
                c.heartbeat(expected_generation=started['generation'], minimal=True)
                if c.status()['tracking_probe']['allowed_by_joint'][str(sid)]:
                    break
                time.sleep(.05)
            else:
                self.fail('Fake HOLD did not reach diagnostic readiness')
            operation = dict(action='tracking-probe', generation=started['generation'],
                             support_confirmed=True, joint_id=sid)
            if live:
                receipt = runtime.action_receipt(operation)
                self.assertEqual(receipt['mode'], 'hold')
            else:
                runtime.action(operation)
            self.assertEqual(c._tracking['id'], sid)
            self.assertEqual(c.status()['tracking_probe']['phase'], 'running')
            c.heartbeat(expected_generation=started['generation'], minimal=True)
            time.sleep(.05)
            stopped = runtime.action_receipt(dict(action='stop', stop_source='button'))
            self.assertTrue(stopped['release_confirmed'])
            self.assertFalse(c._transport.armed)
        runtime.disconnect()
        self.assertTrue(runtime.snapshot()['closed_proof']['serial_closed'])

    def test_cpu_validation_cannot_open_or_start(self):
        c = self.controller(transport=False)
        self.assertTrue(c.status()['candidate_profile']['CPU_validation_only'])
        for operation in (c.start,c._connect_once):
            with self.assertRaisesRegex(motion.MotionError,'validation-only'):
                operation()
        self.assertEqual(self.transports, [])
        runtime = self.create()
        self.assertEqual(runtime.phase,'idle')
        self.assertEqual(self.factory_calls,0)
        self.assertEqual(self.transports,[])

    def test_required_head_and_bias_and_permission_flags(self):
        self.prepare()
        policies = {n:FakePolicy(n) for n in app.MODEL_NAMES}
        for kw in (dict(require_head_clearance=False),dict(hold_hip_pitch_bias_deg=-2),
                   dict(allow_fixed_head_walk=1),dict(hold_hip_pitch_bias_deg=True)):
            with self.subTest(kw=kw),self.assertRaises(motion.MotionError):
                motion.MotionController(policies,self.calibration_path,**kw)

    def test_native_guard_injection_OFF_only_and_current_calibration(self):
        runtime = self.create()
        state = runtime.connect()
        c=runtime._controller
        self.assertEqual(self.factory_calls,1)
        self.assertIsInstance(c,motion.MotionController)
        self.assertFalse(c.status()['candidate_profile']['CPU_validation_only'])
        self.assertEqual(state['hardware']['mode'],'off')
        self.assertEqual(state['hardware']['torque_ids'],[])
        self.assertTrue(state['hardware']['release_confirmed'])
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in self.transports[0].requests))
        for _ in range(100):
            compact=runtime.snapshot(compact=True)
            if compact.get('hardware'): break
            time.sleep(.01)
        self.assertFalse(compact['hardware']['candidate_profile']['native_zero_only_adapter'])
        self.assertEqual(c._config,self.calibration)
        runtime.disconnect()
        self.assertTrue(runtime.snapshot()['closed_proof']['serial_closed'])

    def test_native_guard_failure_has_zero_open_and_no_fallback(self):
        runtime=self.create()
        self.health_fault='injected RAM identity refusal'
        with self.assertRaisesRegex(Exception,'identity refusal'):
            runtime.connect()
        self.assertEqual(self.transports,[])
        self.assertEqual(runtime.snapshot()['closed_proof']['serial_open_attempts'],0)

    def test_zero_model_takeover_observation_history_and_STOP(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c.start_mode('hold',support_confirmed=True)
        for _ in range(155):
            c._clock.advance(.02);c._cycle()
        self.assertTrue(c.status()['hold_ready'])
        hold=c.status()['stand_target_deg']
        self.assertAlmostEqual(hold[2],-20.99622705006573)
        self.assertAlmostEqual(hold[3],-.2864788975654116)
        c.start_mode('walk',support_confirmed=True,expected_generation=c.status()['generation'])
        for _ in range(255):
            c.heartbeat(expected_generation=c.status()['generation'],minimal=True)
            c._clock.advance(.02);c._cycle()
        s=c.status()
        self.assertEqual(s['mode'],'walk')
        self.assertTrue(s['candidate_profile']['zero_walk_warmup']['ready'])
        self.assertTrue(s['fixed_head_walk_zero_velocity_only'])
        with self.assertRaisesRegex(motion.MotionError,'constructor-only'):
            c.command(.2,0,0)
        for mode in ('getup','pick','roulade'):
            with self.subTest(mode=mode),self.assertRaises(motion.MotionError):
                c.start_mode(mode,support_confirmed=True,behavior_confirmed=True)
        g=s['generation'];c.stop();c._clock.advance(.02);c._cycle()
        self.assertGreater(c.status()['generation'],g)
        self.assertFalse(c.status()['armed'])
        self.assertEqual(c.status()['torque_ids'],[])
        self.assertFalse(c.status()['candidate_profile']['zero_walk_warmup']['ready'])

    def test_heartbeat_expiry_and_actual_range_fault_stop_model(self):
        for fault in ('heartbeat','actualrange'):
            c=self.controller();c._connect_once()
            c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
            c.start_mode('hold',support_confirmed=True)
            for _ in range(155):c._clock.advance(.02);c._cycle()
            c.start_mode('walk',support_confirmed=True)
            if fault=='heartbeat':c._clock.advance(.401)
            else:
                c._transport.positions[1]=self.calibration['joints'][1]['min']-1
                c._clock.advance(.02)
            c._cycle()
            self.assertEqual(c.status()['mode'],'off')
            self.assertFalse(c._transport.armed)

    def ready_walk(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c.start_mode('hold',support_confirmed=True)
        for _ in range(155):c._clock.advance(.02);c._cycle()
        c.start_mode('walk',support_confirmed=True)
        return c

    def test_deadline_crossed_during_inference_keeps_native_STOP_readback(self):
        runtime=self.create();runtime.connect();c=runtime._controller
        runtime.action(dict(action='capture-head-clearance',clearance_confirmed=True,
                            generation=c.status()['generation']))
        runtime.action(dict(action='start',mode='hold',support_confirmed=True,
                            generation=c.status()['generation']))
        with c._lock:c._hold_begin-=4.
        self.wait_for(lambda:c.status()['hold_ready'])
        runtime.action(dict(action='start',mode='walk',support_confirmed=True,
                            generation=c.status()['generation']))
        original=c.sessions['walk'].run
        def cross_deadline(*a,**kw):
            # Force the boundary AFTER the first cycle gate, with fresh
            # feedback; scheduler jitter cannot end the episode beforehand.
            result=original(*a,**kw)
            with c._lock:c._trial_began=c._clock()-30.001
            return result
        with c._lock:c.sessions['walk'].run=cross_deadline
        self.wait_for(lambda:c.status()['mode']=='off')
        time.sleep(.08)  # Let a terminal worker exit and its watcher observe it.
        self.assertTrue(c._thread.is_alive())
        self.assertEqual(runtime.phase,'ready')
        self.assertTrue(c.status()['release_confirmed'])
        self.assertFalse(c.status()['pending_stop'])
        self.assertFalse(c._transport.armed)
        self.assertIn('30秒',c.status()['error'])
        runtime.disconnect()
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertTrue(runtime.closed_proof['release_confirmed'])

    def test_normal_window_does_not_swallow_STOP_verification_failure(self):
        c=self.ready_walk()
        c._trial_began=c._clock()-29.99
        original=c.sessions['walk'].run
        def cross_deadline(*a,**kw):
            c._clock.advance(.02)
            return original(*a,**kw)
        c.sessions['walk'].run=cross_deadline
        with mock.patch.object(c,'_service_stop_locked',side_effect=motion.MotionError('STOP readback failed')):
            with self.assertRaisesRegex(motion.MotionError,'STOP readback failed'):
                c._control_once()
        self.assertTrue(c._pending_stop)
        self.assertEqual(c._mode,'off')

    def test_backend_deadline_independent_of_browser_and_restarts(self):
        c=self.ready_walk()
        for episode in range(2):
            if episode:
                c._clock.advance(.02);c._cycle()
                c.start_mode('hold',support_confirmed=True)
                for _ in range(155):c._clock.advance(.02);c._cycle()
                c.start_mode('walk',support_confirmed=True)
            for _ in range(1499):
                c.heartbeat(expected_generation=c.status()['generation'],minimal=True)
                c._clock.advance(.02);c._cycle()
            self.assertEqual(c.status()['mode'],'walk')
            c.heartbeat(expected_generation=c.status()['generation'],minimal=True)
            c._clock.advance(.021);c._cycle()
            self.assertEqual(c.status()['mode'],'off')
            self.assertIn('30秒',c.status()['error'])
            self.assertFalse(c._transport.armed)
        traces=c.completed_trial_traces()
        self.assertEqual(len(traces),2)
        self.assertTrue(all('30秒' in row['reason'] for row in traces))

    def test_running_trial_stops_at15_without_another_TARGET(self):
        c=self.ready_walk()
        # Test the running gate using measured filter attitude, bypassing no
        # feedback or target-range checks. The inherited40deg walk limit alone
        # would accept this tilt.
        c._tilt_locked=lambda:15.
        before=len([1 for kind,_ in c._transport.requests if kind==motion.TARGET])
        c.heartbeat(expected_generation=c.status()['generation'],minimal=True)
        with self.assertRaisesRegex(motion.MotionError,'15°'):
            with c._lock:c._motion_gate_locked()
        c._tilt_locked=lambda:16.
        c._clock.advance(.02);c._cycle()
        self.assertEqual(c.status()['mode'],'off')
        self.assertFalse(c._transport.armed)
        self.assertEqual(before,len([1 for kind,_ in c._transport.requests if kind==motion.TARGET]))

    def test_trace_records_pre_target_actual_and_rejection_terminal(self):
        c=self.ready_walk()
        c._clock.advance(.02);c._cycle()
        attempted=c._transport.positions.copy()
        # A stop preserves the actual terminal state separately from the last
        # successful command and remains isolated from returned caller data.
        c._transport.positions[2]=c._config['joints'][2]['min']-1
        c._clock.advance(.02);c._cycle()
        trace=c.completed_trial_traces()[0]
        self.assertTrue(trace['actual_sample_is_pre_target'])
        self.assertTrue(trace['MCU_applied_goal_not_measured'])
        self.assertEqual(trace['terminal']['actual_raw_ticks'][2],c._config['joints'][2]['min']-1)
        frames=[f for f in trace['frames'] if f['event']=='target-attempt']
        self.assertEqual(list(frames[-1]['attempted_target_ticks']),attempted)
        self.assertNotIn('cookie',json.dumps(trace))
        trace['frames'].clear()
        self.assertTrue(c.completed_trial_traces()[0]['frames'])

    def test_trace_serializes_only_after_UART_closed(self):
        runtime=self.create();runtime.connect();c=runtime._controller
        with c._lock:
            c._append_trace(c._trace_sample('test-marker')+(c._clock(),None))
            c._stop_locked('test-stop')
        records=[];record=runtime._record
        def checked(name,value):
            if '-trial-' in name:
                self.assertTrue(runtime._serial_closed(c,runtime._namespace))
                records.append(name)
            return record(name,value)
        runtime._record=checked
        runtime.disconnect()
        self.assertEqual(len(records),1)
        saved=json.loads((runtime._evidence/records[0]).read_text())
        self.assertEqual(saved['terminal']['stop_reason'],'test-stop')

    def test_bounded_trace_and_repeated_stop_preserve_first_reason(self):
        c=self.controller()
        for _ in range(2105):
            c._append_trace(c._trace_sample('bounded')+(c._clock(),None))
        c._stop_locked('first-stop');c._stop_locked('later-close')
        traces=c.completed_trial_traces()
        self.assertEqual(len(traces),1)
        self.assertEqual(traces[0]['reason'],'first-stop')
        self.assertTrue(traces[0]['truncated'])
        self.assertEqual(traces[0]['total_frames'],2105)
        self.assertEqual(traces[0]['retained_frames'],2100)
        json.dumps(traces,allow_nan=False)

    def test_capture_is_read_only_and_HOLD_interpolates_all_reference_joints(self):
        c=self.controller();c._connect_once()
        heads={30:22.417582417582416,31:-3.1648351648351647,
               32:2.4615384615384617,33:10.813186813186814}
        for sid,angle in heads.items():
            j=c._config['joints'][motion.MOTOR_IDS.index(sid)]
            c._transport.positions[motion.MOTOR_IDS.index(sid)]=round(j['zero']+angle/360*4095*j['sign'])
        c._clock.advance(.02);c._cycle()
        before=copy.deepcopy(c._config);initial=list(c._filtered.angles)
        count=len(c._transport.requests)
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in c._transport.requests[count:]))
        self.assertEqual(c._config,before)
        self.assertEqual(c._hold_head_deg,heads)
        self.assertTrue(c.status()['candidate_profile']['standing_reference_valid'])
        c.start_mode('hold',support_confirmed=True)
        self.assertEqual(c._hold_initial,initial)
        self.assertEqual(c._hold_head_deg,motion.STANDING_REFERENCE_HEAD)
        reference=c._fixed_stance_locked()
        for step in range(1,156):
            c._clock.advance(.02);c._cycle()
            self.assertEqual(c._mode,'hold')
            fraction=min(1.,(c._clock()-c._hold_begin)/3.)
            wanted=[(1-fraction)*a+fraction*b for a,b in zip(initial,reference)]
            last=[p for k,p in c._transport.requests if k==motion.TARGET][-1]
            raw=struct.unpack('<I15H',last)[1:]
            self.assertEqual(list(raw),c._raw_targets_locked(wanted))
            for tick,j in zip(raw,c._config['joints']):self.assertTrue(j['min']<=tick<=j['max'])
            if step==1:
                self.assertLess(abs(c._last_executed_target[10]-initial[10]),.3)
                self.assertGreater(abs(c._last_executed_target[10]-reference[10]),29.)
        self.assertTrue(c._hold_ready_locked())
        self.assertEqual(c._config,before)
        c.start_mode('walk',support_confirmed=True)
        expected=[math.radians(motion.STANDING_REFERENCE_HEAD[sid])-c._policy_profiles['walk']['home_rad'][i+5]
                  for i,sid in enumerate((30,31,32,33))]
        self.assertTrue(motion.np.allclose(c._walk_head_command_locked(),expected))
        c._clock.advance(.02);c._cycle()
        self.assertEqual(c._last_executed_target[10:14],list(motion.STANDING_REFERENCE_HEAD.values()))

    def test_standing_reference_calibration_binding_rejects_before_ARM(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c._config['joints'][1]['zero']+=1
        before=len(c._transport.requests)
        self.assertFalse(c.status()['candidate_profile']['standing_reference_valid'])
        with self.assertRaisesRegex(motion.MotionError,'校准与上次站姿记录不一致'):
            c.start_mode('hold',support_confirmed=True)
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in c._transport.requests[before:]))

    def test_STOP_during_reference_transition_cannot_resume(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c.start_mode('hold',support_confirmed=True)
        for _ in range(20):c._clock.advance(.02);c._cycle()
        c.stop();before=len(c._transport.requests)
        for _ in range(160):c._clock.advance(.02);c._cycle()
        self.assertEqual(c._mode,'off');self.assertFalse(c._transport.armed)
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in c._transport.requests[before:]))


if __name__=='__main__':unittest.main()
