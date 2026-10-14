"""Fake-UART tests for bounded ACK/IPC continuation; same CLI as owner tests."""
import json
import socket
import threading
import time
import unittest
from types import SimpleNamespace
import validate_serial_owner as base
from policy_worker import PolicyWorker, PolicyWorkerError, _encode


class DeferredTests(unittest.TestCase):
    setUp=base.OwnerTests.setUp
    cleanup=base.OwnerTests.cleanup
    tick=base.OwnerTests.tick
    cycles=base.OwnerTests.cycles
    fresh=base.OwnerTests.fresh
    hold=base.OwnerTests.hold
    walk=base.OwnerTests.walk

    def defer_at_ack(self):
        self.walk()
        self.t.pause_acks=threading.Event();self.t.sent.clear()
        self.c._cycle()
        def written():
            # A newly published STATE may cancel an untransmitted ticket.
            # Let the production ACK/revalidation path run, as its wait does.
            self.c._io_drain_ack()
            return self.t.sent.is_set()
        base.until(written)
        self.c._cycle()
        self.assertIsNotNone(self.c._io_deferred_context)

    def test_ack_resumes_full_control_within_wait(self):
        self.defer_at_ack()
        before=len(self.t.target_times);resumes=self.c._io_deferred_resumes
        timer=threading.Timer(.005,self.t.pause_acks.set);timer.start()
        try:self.c._io_wait_until(time.monotonic()+.06)
        finally:timer.join()
        self.assertGreater(self.c._io_deferred_resumes,resumes)
        self.assertGreater(len(self.t.target_times),before)
        self.assertEqual(self.c._mode,'walk')

    def test_stop_invalidates_deferred_continuation(self):
        self.defer_at_ack();before=len(self.t.target_times)
        with self.c._lock:self.c._stop_locked('STOP while deferred')
        self.t.pause_acks.set();self.c.stop()
        self.c._io_wait_until(time.monotonic()+.04)
        self.assertIsNone(self.c._io_deferred_context)
        self.assertEqual(len(self.t.target_times),before)
        self.assertFalse(self.t.armed)

    def test_new_command_invalidates_deferred_context(self):
        self.walk()
        old=self.c._policy_context_locked()
        self.c._io_defer_control(old)
        self.c.command(0,0,0)
        self.assertIsNone(self.c._io_deferred_context)
        # A late scheduler notification also cannot revive the old revision.
        self.c._io_deferred_context=old
        self.assertFalse(self.c._io_can_resume())
        self.assertIsNone(self.c._io_deferred_context)

    def test_resumed_control_checks_expired_feedback(self):
        self.walk();self.c._io_drain_ack()
        self.c._io_owner.sampling=None
        time.sleep(.12)
        before=len(self.t.target_times)
        self.c._io_defer_control(self.c._policy_context_locked())
        self.c._io_wait_until(time.monotonic()+.02)
        self.assertEqual(self.c._mode,'off')
        self.assertFalse(self.t.armed)
        self.assertEqual(len(self.t.target_times),before)

    def test_fast_cycles_cannot_burst_uart_writes(self):
        self.walk();begin=len(self.t.target_times)
        for _ in range(60):
            self.c.heartbeat(minimal=True);self.c._cycle()
            self.c._io_wait_until(time.monotonic()+.002)
        targets=self.t.target_times[begin:]
        self.assertGreaterEqual(len(targets),3)
        for earlier,later in zip(targets,targets[1:]):
            self.assertGreaterEqual(later-earlier,.020)
        self.assertEqual(self.c._mode,'walk')

    def test_readiness_cannot_consume_or_relabel_ipc(self):
        worker=PolicyWorker.__new__(PolicyWorker)
        left,right=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.addCleanup(left.close);self.addCleanup(right.close)
        left.setblocking(False)
        worker._socket=left;worker._lock=threading.Lock()
        worker._pending=3;worker._abandoned=False;worker._closed=False
        worker.process=SimpleNamespace(poll=lambda:None)
        worker.completed=0
        self.assertFalse(worker.result_ready(3))
        right.send(_encode(dict(id=3,action=[[0.]*14],compute_ms=.5)))
        self.assertTrue(worker.result_ready(3));self.assertTrue(worker.result_ready(3))
        self.assertEqual(worker.completed,0);self.assertEqual(worker._pending,3)
        with self.assertRaises(PolicyWorkerError):worker.result_ready(2)
        worker._lock.acquire()
        try:self.assertFalse(worker.result_ready(3))
        finally:worker._lock.release()
        self.assertIsNotNone(worker.take(3));self.assertEqual(worker.completed,1)

    def test_cancel_during_readiness_cannot_fault_new_context(self):
        self.walk()
        self.c._io_wait_until(time.monotonic()+.04)
        context=self.c._policy_context_locked()
        self.c._policy_job={'request_id':99}
        self.c._io_defer_control(context)
        def cancelled(_):
            with self.c._lock:
                self.c._command_revision+=1
                self.c._policy_job=None
            raise PolicyWorkerError('cancelled old request')
        self.c._policy_worker=SimpleNamespace(result_ready=cancelled)
        try:self.assertFalse(self.c._io_can_resume())
        finally:self.c._policy_worker=None;self.c._policy_job=None
        self.assertEqual(self.c._mode,'walk')

    def test_current_readiness_failure_remains_a_fault(self):
        self.walk()
        self.c._io_wait_until(time.monotonic()+.04)
        self.c._policy_job={'request_id':99}
        self.c._io_defer_control(self.c._policy_context_locked())
        def failed(_):raise PolicyWorkerError('current child failed')
        self.c._policy_worker=SimpleNamespace(result_ready=failed)
        try:
            with self.assertRaises(PolicyWorkerError):self.c._io_can_resume()
        finally:self.c._policy_worker=None;self.c._policy_job=None


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(DeferredTests))
    report=dict(tests=result.testsRun,passed=result.wasSuccessful(),
                failures=[str(x) for x in result.failures+result.errors],
                hardware_access=False,source=str(base.a.source.resolve()))
    base.a.output.parent.mkdir(parents=True,exist_ok=True)
    base.a.output.write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
