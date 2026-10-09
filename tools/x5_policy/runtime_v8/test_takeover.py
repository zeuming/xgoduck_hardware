"""CPU-only transition and unchanged safety boundaries; fake transport only."""
from pathlib import Path
import math
import sys
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(HERE/'source/policy'),str(HERE/'source/service'),str(HERE),str(ROOT/'tools/x5_policy')]
import motion
from takeover_math import blend_stance,takeover_weight
from test_native_zero_adapter import NativeZeroAdapterTests


class TakeoverTests(unittest.TestCase):
    setUp=NativeZeroAdapterTests.setUp
    tearDown=NativeZeroAdapterTests.tearDown
    prepare=NativeZeroAdapterTests.prepare
    controller=NativeZeroAdapterTests.controller

    def ready(self):
        c=self.controller();c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True,expected_generation=c.status()['generation'])
        c.start_mode('hold',support_confirmed=True)
        for _ in range(155):c._clock.advance(.02);c._cycle()
        self.assertTrue(c.status()['hold_ready'])
        return c

    def test_endpoint_monotonic_and_head_exact(self):
        a=[1.25]*15;b=[5.75]*10+[1.25]*5
        self.assertEqual(blend_stance(a,b,0),a)
        self.assertEqual(blend_stance(a,b,2),b)
        self.assertEqual(blend_stance(a,b,3),b)
        weights=[takeover_weight(i/100)for i in range(201)]
        self.assertEqual(weights,sorted(weights))
        self.assertLess(takeover_weight(.001),.000001)
        self.assertLess(1-takeover_weight(1.999),.000001)
        self.assertEqual(blend_stance(a,b,.765)[10:],a[10:])
        for v in (float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):takeover_weight(v)

    def test_preflight_is_full_target_then_active_ramp(self):
        c=self.ready();held=c._fixed_stance_locked();target=list(held);target[3]+=4
        self.assertEqual(c._execution_target_locked('walk',target),target)
        c.start_mode('walk',support_confirmed=True,expected_generation=c.status()['generation'])
        self.assertEqual(c._execution_target_locked('walk',target),held)
        c._clock.advance(1)
        got=c._execution_target_locked('walk',target)
        self.assertAlmostEqual(got[3],held[3]+2)
        self.assertEqual(got[10:14],held[10:14])
        c._clock.advance(1)
        self.assertEqual(c._execution_target_locked('walk',target),target)
        c.stop();c.close()

    def test_full_unblended_range_error_is_not_hidden(self):
        c=self.ready();c.start_mode('walk',support_confirmed=True)
        target=c._fixed_stance_locked();target[2]=200
        with self.assertRaises(motion.MotionError):c._execution_target_locked('walk',target)
        c.stop();c.close()

    def test_raw_previous_preserved_and_first_step_reduced(self):
        c=self.ready();p=c.sessions['walk'];p.output[0,3]=math.radians(6)
        held=c._fixed_stance_locked();c.start_mode('walk',support_confirmed=True)
        c._clock.advance(.02);c._cycle()
        self.assertTrue(np.array_equal(c._previous_action,p.output[0]))
        self.assertLess(abs(c._last_executed_target[3]-held[3]),.01)
        self.assertGreater(abs(c._last_policy_target[3]-held[3]),3)
        c.stop();self.assertIsNone(c._trial_began)
        self.assertIsNone(c.status()['candidate_profile']['zero_takeover']['policy_weight'])
        c.close()

    def test_elapsed_clock_includes_delay_without_advancing_30s_deadline(self):
        c=self.ready();c.start_mode('walk',support_confirmed=True)
        began=c._trial_began;c._clock.advance(.25)
        s=c.status()['candidate_profile']
        self.assertAlmostEqual(s['zero_takeover']['policy_weight'],takeover_weight(.25))
        self.assertAlmostEqual(s['trial_seconds_remaining'],29.75)
        self.assertEqual(c._trial_began,began)
        c.stop();c.close()

    def test_nonzero_cannot_enable_ramp(self):
        c=self.ready();c.start_mode('walk',support_confirmed=True)
        with self.assertRaises(motion.MotionError):c.command(.01,0,0)
        c._command[0]=.01
        with self.assertRaisesRegex(motion.MotionError,'受限试步'):
            c._execution_target_locked('walk',c._fixed_stance_locked())
        c.stop();c.close()

    def test_stop_during_inference_does_not_send_blended_target(self):
        c=self.ready();c.start_mode('walk',support_confirmed=True)
        p=c.sessions['walk'];original=p.run
        before=sum(kind==motion.TARGET for kind,_ in c._transport.requests)
        def stopped(*a,**kw):
            result=original(*a,**kw);c.stop();return result
        p.run=stopped;c._clock.advance(.02);c._cycle()
        self.assertEqual(sum(kind==motion.TARGET for kind,_ in c._transport.requests),before)
        c._clock.advance(.02);c._cycle()
        self.assertFalse(c.status()['armed']);c.close()


if __name__=='__main__':
    unittest.main(defaultTest='TakeoverTests')
