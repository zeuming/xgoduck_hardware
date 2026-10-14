"""Deterministic reservation boundary tests; hardware-denied fixture imported."""
import json,threading,unittest
from types import SimpleNamespace
import validate_serial_owner as base
from serial_owner import SerialOwner,Ticket,StateSnapshot


class PollPriorityTests(unittest.TestCase):
    def setUp(self):
        self.o=SerialOwner.__new__(SerialOwner);self.o.clock=lambda:100.;self.o.epoch=7
        self.o.target=Ticket('target',(5,b'x'),7,100.08,state_serial=3)
        self.o.latest=StateSnapshot(b'',99.99,4,7,3)

    def test_pending_deadline_reservation_is_pure_and_bounded(self):
        t=self.o.target;before=dict(vars(t))
        self.assertTrue(self.o._reserve_pending_target(22.,22.02))
        self.assertEqual(vars(t),before)
        self.assertFalse(self.o._reserve_pending_target(22.,22.09))
        self.assertFalse(self.o._reserve_pending_target(22.,22.))
        self.assertFalse(self.o._reserve_pending_target(22.,None))
        self.assertEqual(t.deadline,100.08)

    def test_invalidated_completed_or_changed_state_does_not_delay_sampling(self):
        t=self.o.target
        t.cancelled=True;self.assertFalse(self.o._reserve_pending_target(22.,22.02));t.cancelled=False
        t.epoch=6;self.assertFalse(self.o._reserve_pending_target(22.,22.02));t.epoch=7
        t.done.set();self.assertFalse(self.o._reserve_pending_target(22.,22.02));t.done.clear()
        self.o.latest=StateSnapshot(b'',99.99,4,7,4)
        self.assertFalse(self.o._reserve_pending_target(22.,22.02))
        self.o.latest=None;self.assertFalse(self.o._reserve_pending_target(22.,22.02))
        self.o.target=None;self.assertFalse(self.o._reserve_pending_target(22.,22.02))


if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(PollPriorityTests))
    base.a.output.write_text(json.dumps(dict(tests=r.testsRun,passed=r.wasSuccessful(),hardware_access=False,failures=[str(x) for x in r.failures+r.errors]),indent=2)+'\n')
    raise SystemExit(0 if r.wasSuccessful() else 1)
