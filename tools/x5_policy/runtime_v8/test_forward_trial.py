"""Fake UART: explicit preset, one shot, deadlines, gates, cancellation and replay."""
import math,unittest
import numpy as np
import test_native_zero_adapter as fixture
import motion
class ForwardTrialTests(unittest.TestCase):
    setUp=fixture.NativeZeroAdapterTests.setUp
    tearDown=fixture.NativeZeroAdapterTests.tearDown
    prepare=fixture.NativeZeroAdapterTests.prepare
    controller=fixture.NativeZeroAdapterTests.controller
    ready_walk=fixture.NativeZeroAdapterTests.ready_walk
    def advance(self,c,duration):
        for _ in range(round(duration/.02)):
            c._clock.advance(.02);c.heartbeat(expected_generation=c._generation,minimal=True);c._cycle()
    def warmed(self):
        c=self.ready_walk();self.advance(c,5.1);self.assertTrue(c.status()['forward_trial']['allowed']);return c
    def trigger(self,c,**kw):
        return c.forward_trial(expected_generation=c._generation,support_confirmed=True,**kw)
    def test_requires_live_zero_warmup_support_and_generation(self):
        c=self.ready_walk()
        try:
            before=len(c._transport.requests)
            with self.assertRaisesRegex(motion.MotionError,'5秒'):self.trigger(c)
            self.assertEqual(len(c._transport.requests),before)
            self.advance(c,5.1)
            for kw in [dict(expected_generation=c._generation-1,support_confirmed=True),dict(expected_generation=c._generation,support_confirmed=False)]:
                with self.assertRaises(motion.MotionError):c.forward_trial(**kw)
            self.assertFalse(c._forward.used)
        finally:c.close()
    def test_exact_profile_returns_zero_without_browser_scheduler_or_new_ARM(self):
        c=self.warmed()
        try:
            gen=c._generation;before=list(c._transport.requests);start=c._clock();self.trigger(c)
            self.assertEqual(c._transport.requests,before,'request cannot itself ARM/TARGET')
            samples=[]
            for _ in range(110):
                self.advance(c,.02);t=c._clock()-start;cmd=c._command.copy();samples.append(cmd[0])
                expected=.2*max(0.,min(1.,t/.5,(2.-t)/.5))
                self.assertAlmostEqual(cmd[0],expected,places=6)
                self.assertEqual(list(cmd[1:]),[0.,0.])
            self.assertGreater(max(samples),.1999)
            self.assertEqual(list(c._command),[0.,0.,0.]);self.assertEqual(c._generation,gen)
            self.assertEqual(c._mode,'walk');self.assertTrue(c._transport.armed)
            self.assertEqual(c.status()['forward_trial']['phase'],'returned-to-zero')
            self.assertFalse(any(k==motion.ARM for k,_ in c._transport.requests[len(before):]))
            with self.assertRaisesRegex(motion.MotionError,'已使用'):self.trigger(c)
        finally:c.close()
    def test_generic_nonzero_negative_side_yaw_and_nonfinite_remain_rejected(self):
        c=self.warmed()
        try:
            for args in [(.18,0,0),(-.18,0,0),(0,.01,0),(0,0,.1),(float('nan'),0,0),(True,0,0)]:
                with self.subTest(args=args),self.assertRaises(motion.MotionError):c.command(*args)
            self.assertFalse(c._forward.used);self.assertFalse(any(c._command))
        finally:c.close()
    def test_zero_command_cancels_pulse_and_cannot_resume_it(self):
        c=self.warmed()
        try:
            self.trigger(c);self.advance(c,.1);self.assertGreater(c._command[0],0)
            c.command(0,0,0);self.advance(c,.3)
            self.assertFalse(any(c._command));self.assertEqual(c._forward.phase,'cancelled')
            with self.assertRaises(motion.MotionError):self.trigger(c)
        finally:c.close()
    def test_stale_tilt_and_late_window_reject_before_any_motion(self):
        for case in ('stale','tilt','window','heartbeat'):
            c=self.warmed()
            try:
                if case=='stale':c._clock.advance(.101)
                elif case=='tilt':c._filtered.grav=[math.sin(math.radians(16)),0.,-math.cos(math.radians(16))]
                elif case=='window':c._trial_began=c._clock()-25.01
                else:c._heartbeat=c._clock()-.401
                before=len(c._transport.requests)
                with self.assertRaises(motion.MotionError):self.trigger(c)
                self.assertEqual(len(c._transport.requests),before);self.assertFalse(c._forward.used)
            finally:c.close()
    def test_long_zero_target_gap_invalidates_warmup(self):
        c=self.warmed()
        try:
            c._clock.advance(.101);c.heartbeat(expected_generation=c._generation,minimal=True);c._cycle()
            self.assertFalse(c.status()['forward_trial']['allowed'])
            self.advance(c,5.1);self.assertTrue(c.status()['forward_trial']['allowed'])
        finally:c.close()
    def test_STOP_clears_active_command_and_old_generation_cannot_trigger(self):
        c=self.warmed()
        try:
            gen=c._generation;self.trigger(c);self.advance(c,.1);c.stop()
            self.assertFalse(any(c._command));self.assertEqual(c._mode,'off');self.assertFalse(c._transport.armed)
            with self.assertRaises(motion.MotionError):c.forward_trial(expected_generation=gen,support_confirmed=True)
            self.assertEqual(c._forward.phase,'cancelled')
        finally:c.close()

    def test_cancel_or_STOP_during_inference_never_sends_its_nonzero_target(self):
        for case in ('cancel','STOP'):
            c=self.warmed()
            try:
                self.trigger(c);self.advance(c,.1)
                before=sum(k==motion.TARGET for k,_ in c._transport.requests)
                p=c.sessions['walk'];original=p.run
                def delayed(*a,**kw):
                    value=original(*a,**kw)
                    if case=='cancel':c.command(0,0,0)
                    else:c.stop()
                    return value
                class OneCycle:
                    done=False
                    def is_set(self):return self.done
                    def set(self):self.done=True
                    def wait(self,seconds):self.done=True
                p.run=delayed;uart=c._transport;c._closed=OneCycle()
                reasons=[];disconnect=c._disconnect
                def record_disconnect(reason):
                    reasons.append(str(reason));return disconnect(reason)
                c._disconnect=record_disconnect
                c._clock.advance(.02);c.heartbeat(expected_generation=c._generation,minimal=True)
                c._run()  # Actual production exception -> STOP -> release -> disconnect.
                self.assertEqual(sum(k==motion.TARGET for k,_ in uart.requests),before)
                self.assertEqual(c._mode,'off')
                self.assertFalse(uart.armed)

            finally:c.close()

    def test_forward_start_tighter_tilt_gate_and_missing_torque_reject(self):
        for case in ('tilt8','torque'):
            c=self.warmed()
            try:
                if case=='tilt8':c._filtered.grav=[math.sin(math.radians(8.01)),0.,-math.cos(math.radians(8.01))]
                else:c._state['torque_mask']=0
                before=list(c._transport.requests)
                with self.assertRaises(motion.MotionError):self.trigger(c)
                self.assertEqual(c._transport.requests,before)
            finally:c.close()

    def test_active_pulse_stops_on_lost_browser_heartbeat_or_actual_range_fault(self):
        for fault in ('heartbeat','actualrange'):
            c=self.warmed()
            try:
                self.trigger(c);self.advance(c,.1)
                before=sum(k==motion.TARGET for k,_ in c._transport.requests)
                if fault=='heartbeat':c._clock.advance(.401)
                else:
                    c._transport.positions[1]=self.calibration['joints'][1]['min']-1
                    c._clock.advance(.02)
                c._cycle()
                self.assertEqual(c._mode,'off');self.assertFalse(c._transport.armed)
                self.assertFalse(any(c._command));self.assertEqual(c._forward.phase,'cancelled')
                self.assertEqual(sum(k==motion.TARGET for k,_ in c._transport.requests),before)
            finally:c.close()

    def test_execution_evidence_counts_only_successful_TARGET_replies(self):
        c=self.warmed()
        try:
            self.trigger(c);self.advance(c,.1)
            count=c._forward.positive_targets;self.assertGreater(count,0)
            original=c._transport.request
            def fail(kind,payload=b''):
                if kind==motion.TARGET:raise TimeoutError('test failed target')
                return original(kind,payload)
            c._transport.request=fail
            with self.assertRaisesRegex(TimeoutError,'test failed target'):self.advance(c,.02)
            self.assertEqual(c._forward.positive_targets,count)
            c._transport.request=original;c.command(0,0,0);self.advance(c,.1)
            e=c.status()['forward_trial']['execution']
            self.assertEqual(e['successful_nonzero_TARGETs'],count)
            self.assertGreater(e['successful_zero_TARGETs_after_forward'],0)
            self.assertFalse(e['actual_walking_verified'])
        finally:c.close()

    def test_display_view_cannot_mutate_live_pulse(self):
        from status_snapshot import capture_display_view
        c=self.warmed()
        try:
            self.trigger(c)
            with c._lock:view=capture_display_view(c)
            view._forward.cancel()
            self.assertEqual(c._forward.phase,'running')
            self.assertNotEqual(view._forward.epoch,c._forward.epoch)
        finally:c.close()
if __name__=='__main__':unittest.main()
