import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from control_time_ledger import ControlTimeLedger, ObservedLock


class LedgerTests(unittest.TestCase):
    def test_retry_inherits_logical_identity_but_not_ticket(self):
        class Owner:
            def __init__(self):
                self.cv=threading.Condition(threading.RLock());self._transport=SimpleNamespace()
            def submit_target(self,payload,deadline,state_serial,*,retry_of=None):
                return SimpleNamespace(deadline=deadline,state_serial=state_serial)
            def target_write_guard(self):raise AssertionError('No write requested')
            def _execute(self,ticket):raise AssertionError('No execution requested')
        owner=Owner();ledger=ControlTimeLedger();ledger.attach_owner(owner)
        first=owner.submit_target(b'not recorded',10,3)
        retry=owner.submit_target(b'not recorded',10,4,retry_of=first)
        next_action=owner.submit_target(b'not recorded',11,4)
        self.assertEqual(first._ledger_logical,retry._ledger_logical)
        self.assertNotEqual(first._ledger_ticket,retry._ledger_ticket)
        self.assertNotEqual(first._ledger_logical,next_action._ledger_logical)
        self.assertEqual(retry.deadline,10)
        self.assertNotIn('not recorded',json.dumps(ledger.events))

    def test_worker_exception_propagates_and_no_extra_call(self):
        class Worker:
            calls=0
            def submit(self,*args):self.calls+=1;return 4
            def take(self,*args):self.calls+=1;raise ValueError('expected')
            def discard(self,*args):self.calls+=1
        worker=Worker();ledger=ControlTimeLedger();ledger.attach_worker(worker)
        self.assertEqual(worker.submit('walk',object()),4)
        with self.assertRaisesRegex(ValueError,'expected'):worker.take(4)
        self.assertEqual(worker.calls,2)
        self.assertEqual(ledger.events[-1]['error'],'ValueError')

    def test_reentrant_lock_preserves_original(self):
        original=threading.RLock();ledger=ControlTimeLedger();lock=ObservedLock(original,ledger)
        with lock:
            with lock:self.assertTrue(original._is_owned())
        self.assertFalse(original._is_owned())
        self.assertEqual(len(ledger.events),2)

    def test_overflow_fails_evidence_save(self):
        ledger=ControlTimeLedger(maximum_events=1);ledger.emit('first');ledger.emit('second')
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError,'overflow'):ledger.save(Path(tmp)/'ledger')
        self.assertEqual(len(ledger.events),1);self.assertEqual(ledger.dropped,1)

    def test_output_refuses_overwrite_and_sorts_time(self):
        ledger=ControlTimeLedger();ledger.emit('first');ledger.emit('second')
        ledger.events.reverse()
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'ledger';ledger.save(path)
            lines=[json.loads(row) for row in (path/'events.jsonl').read_text().splitlines()]
            self.assertEqual([x['event'] for x in lines],['first','second'])
            with self.assertRaises(FileExistsError):ledger.save(path)


if __name__=='__main__':unittest.main()
