import unittest
import motion
from test_forward_trial import ForwardTrialTests

class ForwardTiltTests(unittest.TestCase):
    setUp=ForwardTrialTests.setUp
    tearDown=ForwardTrialTests.tearDown
    prepare=ForwardTrialTests.prepare
    controller=ForwardTrialTests.controller
    ready_walk=ForwardTrialTests.ready_walk
    advance=ForwardTrialTests.advance
    warmed=ForwardTrialTests.warmed
    trigger=ForwardTrialTests.trigger

    def test_eight_degree_stop_during_pulse_and_zero_recovery(self):
        for duration in (.2,2.2):
            c=self.warmed()
            try:
                self.trigger(c);self.advance(c,duration)
                before=sum(k==motion.TARGET for k,_ in c._transport.requests)
                c._tilt_locked=lambda:8.01
                self.advance(c,.02)
                self.assertEqual(c._mode,'off')
                self.assertFalse(c._transport.armed)
                self.assertEqual(sum(k==motion.TARGET for k,_ in c._transport.requests),before)
            finally:c.close()

del ForwardTrialTests
