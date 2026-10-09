"""Pure pulse deadline and cancellation regressions; no hardware or sleeps."""
import unittest
from bounded_forward import ForwardPulse,PEAK_M_S,DURATION_S

class PulseTests(unittest.TestCase):
    def test_fixed_envelope_and_return_are_clock_based(self):
        p=ForwardPulse();p.begin(100.,7)
        for dt in (0.,.1,.4,.5,.8,1.,1.199,1.2,20.):
            cmd=p.command_at(100.+dt,7)
            self.assertEqual(cmd[1:],(0.,0.))
            self.assertTrue(0<=cmd[0]<=PEAK_M_S)
            if dt>=DURATION_S:self.assertEqual(cmd,(0.,0.,0.))
        self.assertEqual(p.phase,'returned-to-zero')
        with self.assertRaises(ValueError):p.begin(200.,7)

    def test_cancel_reset_generation_and_inflight_deadline(self):
        p=ForwardPulse();p.begin(10.,3);epoch=p.epoch
        p.check_inflight(10.1,3,epoch)
        for time,gen,stamp in ((11.2,3,epoch),(9.,3,epoch),(10.1,4,epoch),(10.1,3,epoch-1)):
            with self.subTest(time=time,gen=gen),self.assertRaises(ValueError):p.check_inflight(time,gen,stamp)
        p.cancel();self.assertEqual(p.command_at(10.2,3),(0.,0.,0.))
        with self.assertRaises(ValueError):p.check_inflight(10.2,3,epoch)
        with self.assertRaises(ValueError):p.begin(10.2,3)
        p.reset();p.begin(12.,4)
        with self.assertRaises(ValueError):p.check_inflight(12.1,3,epoch)

    def test_dispatch_evidence_never_starts_or_changes_clock_and_survives_cancel(self):
        p=ForwardPulse();p.note_successful_target(7,10.,10.01,(.1,0.,0.))
        self.assertEqual(p.positive_targets,0);self.assertFalse(p.used)
        p.begin(10.,7);epoch=p.epoch
        p.note_successful_target(6,10.1,10.11,(.1,0.,0.))
        p.note_successful_target(7,10.1,10.11,(.025,0.,0.))
        p.note_successful_target(7,10.4,10.41,(.1,0.,0.))
        p.note_successful_target(7,11.21,11.22,(0.,0.,0.))
        self.assertEqual((p.positive_targets,p.zero_targets_after_positive),(2,1))
        self.assertEqual((p.started,p.epoch,p.phase),(10.,epoch,'running'))
        p.cancel();self.assertEqual(p.positive_targets,2)
        p.reset();self.assertEqual(p.positive_targets,0);self.assertIsNone(p.first_positive_at)

    def test_invalid_clock_is_rejected(self):
        for bad in (True,float('nan'),float('inf'),'100'):
            with self.subTest(bad=bad),self.assertRaises(ValueError):ForwardPulse().begin(bad,1)
        p=ForwardPulse();p.begin(10.,1)
        with self.assertRaises(ValueError):p.command_at(9.,1)

if __name__=='__main__':unittest.main()
