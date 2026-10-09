"""CPU/fake UART only: status formatting cannot block a control/STOP lock."""
import copy,threading,time,unittest
from unittest import mock
import test_native_zero_adapter as fixture
import motion,production_motion as production
from status_snapshot import capture_display_view

class DisplaySnapshotTests(unittest.TestCase):
    setUp=fixture.NativeZeroAdapterTests.setUp
    tearDown=fixture.NativeZeroAdapterTests.tearDown
    prepare=fixture.NativeZeroAdapterTests.prepare
    create=fixture.NativeZeroAdapterTests.create
    controller=fixture.NativeZeroAdapterTests.controller
    def test_formatter_delay_leaves_live_control_lock_available(self):
        runtime=self.create();ns=runtime._new_namespace()
        c=ns['OneShotController']({n:fixture.FakePolicy(n) for n in fixture.app.MODEL_NAMES},self.calibration_path,clock=fixture.FakeClock(),allow_fixed_head_walk=True)
        entered,release,acquired=threading.Event(),threading.Event(),threading.Event()
        original=production.MotionController.status;results=[]
        def slow(view):
            self.assertIsNot(view,c)
            self.assertIsNone(view._transport)
            entered.set()
            if not release.wait(2):raise AssertionError('formatter test timeout')
            return original(view)
        with mock.patch.object(production.MotionController,'status',slow):
            t=threading.Thread(target=lambda:results.append(c.status()));t.start()
            self.assertTrue(entered.wait(1))
            def control():
                with c._lock:
                    c._generation+=1
                    acquired.set()
            other=threading.Thread(target=control);other.start()
            self.assertTrue(acquired.wait(.2),'control lock held by display formatter')
            release.set();t.join(2);other.join(2)
        self.assertFalse(t.is_alive());self.assertFalse(other.is_alive())
        self.assertNotEqual(results[0]['generation'],c._generation,'old view must retain captured epoch')
        self.assertEqual(ns['request_counts'],{})
        self.assertIsNone(c._thread);self.assertIsNone(c._transport)
    def test_inputs_detached_and_sensor_age_never_refreshed(self):
        c=self.controller();c._connect_once()
        try:
            with c._lock:view=capture_display_view(c)
            old=view._state['positions'][0]
            c._state['positions'][0]+=1;c._command[0]=.05
            c._config['imu']['bias_dps'][0]+=1
            self.assertEqual(view._state['positions'][0],old)
            self.assertEqual(view._command[0],0.)
            self.assertNotEqual(view._config['imu']['bias_dps'],c._config['imu']['bias_dps'])
            before=view._age_locked();c._clock.advance(.12)
            self.assertGreaterEqual(view._age_locked()-before,.119)
            self.assertEqual(view._sample_received,c._sample_received)
        finally:c.close()

class TraceTransitionTests(unittest.TestCase):
    setUp=fixture.NativeZeroAdapterTests.setUp
    tearDown=fixture.NativeZeroAdapterTests.tearDown
    prepare=fixture.NativeZeroAdapterTests.prepare
    controller=fixture.NativeZeroAdapterTests.controller
    ready_walk=fixture.NativeZeroAdapterTests.ready_walk
    def test_return_to_hold_retains_model_frames_before_ring_rollover(self):
        c=self.ready_walk()
        try:
            for _ in range(10):
                c._clock.advance(.02);c.heartbeat(expected_generation=c._generation,minimal=True);c._cycle()
            gen=c._generation
            c.start_mode('hold',support_confirmed=True)
            saved=c.completed_trial_traces()
            self.assertEqual(len(saved),1)
            self.assertEqual(saved[0]['terminal']['event'],'mode-transition')
            self.assertIn('hold',saved[0]['reason'])
            self.assertTrue(any(r['generation']==gen and r['mode']=='walk' for r in saved[0]['frames']))
            for _ in range(2200):c._append_trace(c._trace_sample('later-hold')+(c._clock(),None))
            self.assertEqual(c.completed_trial_traces()[0],saved[0])
        finally:c.close()

if __name__=='__main__':unittest.main()
