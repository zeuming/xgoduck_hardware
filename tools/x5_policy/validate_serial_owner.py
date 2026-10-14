"""Hardware-denied UART-owner regression and optional genuine-ONNX benchmark."""
import argparse
import contextlib
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--benchmark',action='store_true')
p.add_argument('--only',help='Run tests whose names contain this string')
p.add_argument('--models',type=Path)
p.add_argument('--transport-timing',type=Path)
p.add_argument('--delay-mode',choices=('none','state','target','both'),default='both')
p.add_argument('--ledger-dir',type=Path,help='Opt-in bounded fake-transport event log; must be new')
p.add_argument('--warmup-seconds',type=float,default=0.)
p.add_argument('--segment-seconds',type=float,default=3.)
a=p.parse_args()
sys.path[:0]=[str(a.source.resolve()/'policy'),str(a.source.resolve()/'service'),str(Path(__file__).resolve().parent)]
def no_devices(event,values):
    if event=='open' and values and isinstance(values[0],(str,bytes)):
        path=os.path.realpath(os.fsdecode(values[0]))
        if path.startswith('/dev/') and path!='/dev/null' and not path.startswith('/dev/shm/'):raise PermissionError('Offline owner validation cannot open devices')
sys.addaudithook(no_devices)
import numpy as np
import motion
from serial_owner import SerialOwner,OwnerError,TargetCancelled,SnapshotAdvanced
from test_motion import FakePolicy,ready_config,FakeClock
from causal_test_transport import PairedFakeTransport
from current_pose_probe import controller_class,DURATION_S
from attended_trial_limits import ZERO_WINDOW_S,PAGE_LOST_HOLD,ZERO_COMPLETE


class GuardedTransport(PairedFakeTransport):
    supports_target_write_guard=True
    def __init__(self,port):
        super().__init__(port)
        self.thread_ids=set();self.operations=[];self.target_times=[];self.sample_times=[]
        self.delays={};self.delay_cursor={};self.wait_rows=[]
        self.before_target=None;self.release_target=None;self.before_state=None;self.release_state=None
        self.pause_acks=None;self.sent=threading.Event();self.reject_ack=False;self.short_write=False
        self.active=0;self.max_active=0;self.last_goal=None;self.watchdog_stops=0
        self.origin=time.monotonic();self.close_error=False;self.seq=0;self.ledger=None
    def injected_wait(self,delay,kind):
        start=time.monotonic();cpu=time.thread_time()
        time.sleep(delay)
        if self.ledger is not None:
            self.ledger.emit('injected_wait',start=start,cpu_start=cpu,kind=kind,requested_ms=delay*1000)
    @contextlib.contextmanager
    def operation(self,name):
        self.thread_ids.add(threading.get_ident());self.operations.append(name)
        self.active+=1;self.max_active=max(self.max_active,self.active)
        try:yield
        finally:self.active-=1
    def open(self):
        with self.operation('open'):return super().open()
    def close(self):
        with self.operation('close'):
            if self.close_error:raise OwnerError('injected close failure')
            return super().close()
    def negotiate_voltage(self):
        with self.operation('caps-voltage'):return super().negotiate_voltage()
    def negotiate_motion_evidence(self):
        with self.operation('caps-causal'):return super().negotiate_motion_evidence()
    def request(self,kind,payload=b''):
        with self.operation(kind):
            self.seq+=1;began=time.monotonic()
            delays=self.delays.get(kind,[]);delay=0
            if delays:
                index=self.delay_cursor.get(kind,0);delay=delays[index%len(delays)]
                self.delay_cursor[kind]=index+1
            # Replayed wait represents a transaction's serial turnaround. For
            # TARGET delay is after the guarded write, as the real ACK wait is.
            if kind in (motion.STATE,motion.VOLTAGE_STATE):
                if self.before_state is not None:
                    self.before_state.set();self.release_state.wait(.3)
                if delay:self.injected_wait(delay,kind)
            now=time.monotonic()
            if self.armed and self.last_goal is not None and now-self.last_goal>=.250:
                self.armed=False;self.torque=0;self.cookie=0;self.watchdog_stops+=1
            if kind in (motion.STATE,motion.VOLTAGE_STATE) and not self.freeze_sample:
                self.sample_us=int((now-self.origin)*1e6)&0xffffffff
                self.freeze_sample=True
                try:answer=super().request(kind,payload)
                finally:self.freeze_sample=False
                self.sample_times.append(now)
            elif kind==motion.TARGET:
                if self.before_target is not None:
                    self.before_target.set();self.release_target.wait(.3)
                with self._target_write_guard():
                    if self.short_write:raise OwnerError('injected short write')
                    answer=super().request(kind,payload)
                    self.last_goal=time.monotonic();self.target_times.append(self.last_goal);self.sent.set()
                if delay:self.injected_wait(delay,kind)
                if self.pause_acks is not None:self.pause_acks.wait(.3)
                if self.reject_ack:answer=bytes([kind,2])
            else:
                with self._request_write_guard(kind):
                    answer=super().request(kind,payload)
                if kind==motion.ARM:self.last_goal=now
            self.wait_rows.append(dict(kind=kind,injected_wait_ms=delay*1000,
                elapsed_ms=(time.monotonic()-began)*1000))
            return answer


def until(predicate,seconds=.5):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.001)
    raise AssertionError('Condition did not complete within deadline')


class OwnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.cfg=Path(self.tmp.name)/'calibration.json';self.cfg.write_text(json.dumps(ready_config()))
        self.t=GuardedTransport('fake-only')
        self.c=motion.MotionController({'walk':FakePolicy()},self.cfg,
            transport_factory=lambda _:self.t,allow_fixed_head_walk=True,serial_owner=True)
        self.addCleanup(self.cleanup)
        self.c._connect_once()
    def cleanup(self):
        for name in ('release_state','release_target','pause_acks'):
            event=getattr(self.t,name,None)
            if event:event.set()
        self.t.close_error=False
        self.c.close()
    def tick(self):
        self.c.heartbeat(minimal=True);self.c._cycle();self.c._io_wait_until(time.monotonic()+.02)
    def cycles(self,count):
        for _ in range(count):self.tick()
    def fresh(self):
        old=self.c._last_sample
        until(lambda:self.c._io_owner.snapshot() is not None and
            struct.unpack_from('<I',self.c._io_owner.snapshot().payload)[0]!=old)
        self.c._cycle()
    def hold(self):
        self.fresh();self.c.capture_head_clearance(clearance_confirmed=True,expected_generation=self.c._generation)
        self.c.start_mode('hold',support_confirmed=True);self.c._hold_begin-=3.1
        # This fixture has a real asynchronous owner thread. Four calls are
        # not proof its ACK and the following feedback have been consumed.
        # Wait for the actual readiness condition, within a bounded budget.
        deadline=time.monotonic()+.5
        while True:
            self.tick()
            status=self.c.status()
            if status['hold_ready']:break
            if time.monotonic()>=deadline:
                self.fail('Fake HOLD did not become ready: '+str(status.get('error')))
    def walk(self):
        self.hold();self.c.start_mode('walk',support_confirmed=True);self.cycles(4)
    def test_single_owner_all_operations_and_stop_close(self):
        self.hold();self.c.stop();self.c.close()
        self.assertEqual(self.t.thread_ids,{self.c._io_owner.thread.ident})
        self.assertEqual(self.t.max_active,1);self.assertTrue(self.t.closed)
        self.assertFalse(self.t.armed)
    def test_delayed_ack_does_not_commit_until_success(self):
        self.walk();self.t.pause_acks=threading.Event();self.t.sent.clear()
        before=self.c._previous_action.copy();count=len(self.c._target_times)
        self.c.sessions['walk'].output[:]=.01
        self.c._cycle();until(self.t.sent.is_set)
        np.testing.assert_array_equal(self.c._previous_action,before)
        self.assertEqual(len(self.c._target_times),count)
        self.t.pause_acks.set();until(lambda:self.c._io_pending['ticket'].done.is_set())
        self.c._io_drain_ack();self.assertGreater(len(self.c._target_times),count)
        np.testing.assert_allclose(self.c._previous_action,.01)
    def test_rejected_ack_never_commits_history(self):
        self.walk();self.t.reject_ack=True
        before=self.c._previous_action.copy();self.c._cycle()
        until(lambda:self.c._io_pending['ticket'].done.is_set())
        with self.assertRaises(motion.MotionError):self.c._io_drain_ack()
        np.testing.assert_array_equal(self.c._previous_action,before)
    def test_stop_cancels_target_before_atomic_write(self):
        self.hold();self.t.before_target=threading.Event();self.t.release_target=threading.Event()
        n=len(self.t.target_times);self.c._cycle();until(self.t.before_target.is_set)
        with self.c._lock:self.c._stop_locked('fixture STOP')
        self.t.release_target.set()
        self.c.stop();self.assertEqual(len(self.t.target_times),n)
        self.assertFalse(self.t.armed);self.assertEqual(self.c._mode,'off')
    def test_rearm_does_not_consume_old_ack(self):
        self.hold();self.t.pause_acks=threading.Event();self.t.sent.clear()
        self.c._cycle();until(self.t.sent.is_set)
        old=self.c._io_pending['ticket']
        with self.c._lock:self.c._stop_locked('fixture')
        self.t.pause_acks.set();self.c.stop();self.c._io_drain_ack()
        self.assertIsNone(self.c._io_pending);self.assertEqual(self.c._mode,'off')
        self.assertTrue(old.done.is_set())
    def test_rx_cache_timestamp_not_refreshed_on_consume(self):
        self.c._io_owner.sampling=None
        snap=self.c._io_owner.snapshot();self.assertIsNotNone(snap)
        time.sleep(.025);self.c._cycle()
        self.assertEqual(self.c._sample_received,snap.received_at)
        self.assertGreaterEqual(self.c._age_locked(),.025)
    def test_frozen_feedback_and_imu_staleness_stop(self):
        self.hold();self.t.freeze_sample=True
        self.cycles(8)
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.t.armed)
    def test_controller_pause_does_not_autonomously_refresh_watchdog(self):
        self.hold();n=len(self.t.target_times)
        time.sleep(.29)
        self.assertEqual(len(self.t.target_times),n);self.assertEqual(self.t.watchdog_stops,1)
        self.assertFalse(self.t.armed);self.c._cycle();self.assertEqual(self.c._mode,'off')
    def test_queue_has_single_slot_and_no_target_replay(self):
        self.hold();self.t.pause_acks=threading.Event();self.t.sent.clear()
        self.c._cycle();until(self.t.sent.is_set)
        ticket=self.c._io_pending['ticket'];n=len(self.t.target_times)
        with self.assertRaises(OwnerError):self.c._io_owner.submit_target(ticket.args[1],time.monotonic()+.1,self.c._io_snapshot_serial)
        for _ in range(20):self.c._cycle()
        self.assertEqual(len(self.t.target_times),n)
        self.t.pause_acks.set();until(ticket.done.is_set);self.c._io_drain_ack()
        self.assertEqual(self.c._io_owner.metrics['max_target_slots'],1)
    def test_normal_window_completes_to_hold_without_stop_or_rearm(self):
        self.walk();before=len(self.t.operations);self.c._trial_began=time.monotonic()-ZERO_WINDOW_S
        self.cycles(3);self.assertEqual(self.c._mode,'hold');self.assertTrue(self.t.armed)
        self.assertEqual(self.c._last_trial_stop['reason'],ZERO_COMPLETE)
        self.assertFalse(any(x in (motion.STOP,motion.ARM) for x in self.t.operations[before:]))
    def test_page_loss_valid_pair_returns_to_hold(self):
        self.walk();self.c._walk_warmup_complete=True
        self.c._heartbeat=time.monotonic()-.5
        self.c._cycle();self.c._io_wait_until(time.monotonic()+.03)
        self.assertEqual(self.c._mode,'hold');self.assertTrue(self.t.armed)
        self.assertEqual(self.c._last_trial_stop['reason'],PAGE_LOST_HOLD)
    def test_page_loss_unknown_pair_stops(self):
        self.walk();self.t.pair_valid=False
        until(lambda:self.c._io_owner.snapshot().payload[187]==0)
        self.c._cycle()
        self.c._heartbeat=time.monotonic()-.5;self.c._cycle()
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.t.armed)
    def test_current_pose_probe_real_wrapper_completes_and_preserves_axis_gate(self):
        self.c.close()
        cls=controller_class(motion.MotionController,motion.MotionError,motion.MOTOR_IDS,motion.ALL_MASK,motion.TARGET)
        self.t=GuardedTransport('fake-only')
        self.c=cls({'walk':FakePolicy()},self.cfg,transport_factory=lambda _:self.t,
                   allow_fixed_head_walk=True,serial_owner=True)
        self.c._connect_once();self.fresh()
        self.c.current_pose_probe(expected_generation=self.c._generation,support_confirmed=True,clearance_confirmed=True)
        self.cycles(4)
        self.c._current_probe['started']-=DURATION_S
        self.cycles(4)
        self.assertEqual(self.c._current_probe['phase'],'completed');self.assertTrue(self.t.armed)
        bad=list(self.c._current_probe['baseline_raw']);bad[2]+=1
        with self.assertRaises(motion.MotionError):self.c._validate_async_target_locked(struct.pack('<I15H',self.c._cookie,*bad))
    def test_new_fault_state_cancels_queued_target_before_write(self):
        self.hold();self.t.before_state=threading.Event();self.t.release_state=threading.Event()
        until(self.t.before_state.is_set)
        self.t.fault=1;n=len(self.t.target_times);self.c._cycle()
        self.t.release_state.set();until(lambda:self.c._io_pending['ticket'].done.is_set())
        with self.assertRaises(motion.MotionError):self.c._io_drain_ack()
        self.assertEqual(len(self.t.target_times),n)
    def test_old_arm_and_config_queued_behind_state_are_cancelled_by_stop(self):
        owner=self.c._io_owner
        self.t.before_state=threading.Event();self.t.release_state=threading.Event()
        until(self.t.before_state.is_set)
        errors=[]
        def call(kind,payload):
            try:owner.request(kind,payload)
            except BaseException as error:errors.append(error)
        arm=threading.Thread(target=call,args=(motion.ARM,struct.pack('<I',999)))
        arm.start();until(lambda:any(t.args[0]==motion.ARM for t in owner.queue))
        stop=threading.Thread(target=call,args=(motion.STOP,b''))
        stop.start();until(lambda:any(t.args[0]==motion.STOP for t in owner.queue))
        self.t.release_state.set();arm.join(.5);stop.join(.5)
        self.assertFalse(self.t.armed)
        self.assertFalse(any(k==motion.ARM for k,_ in self.t.requests))
        self.assertTrue(any(isinstance(e,TargetCancelled) for e in errors))

    def test_expired_barrier_is_rejected_before_transport(self):
        from serial_owner import Ticket
        owner=self.c._io_owner
        with owner.cv:
            ticket=Ticket('request',(motion.ARM,struct.pack('<I',999)),owner.epoch,
                          deadline=time.monotonic()-.001)
            owner.queue.append(ticket);owner.cv.notify_all()
        until(ticket.done.is_set)
        self.assertIsInstance(ticket.error,TargetCancelled)
        self.assertFalse(any(k==motion.ARM for k,_ in self.t.requests))

    def test_forward_deadline_rejects_delayed_write_then_returns_to_zero(self):
        self.walk()
        self.c._trial_began-=6.
        self.c._forward.begin(time.monotonic()-1.97,self.c._generation)
        self.t.before_target=threading.Event();self.t.release_target=threading.Event()
        history=self.c._previous_action.copy();n=len(self.t.target_times)
        self.c._cycle();until(self.t.before_target.is_set)
        time.sleep(.04);self.t.release_target.set()
        until(lambda:self.c._io_pending['ticket'].done.is_set());self.c._io_drain_ack()
        self.assertEqual(len(self.t.target_times),n)
        np.testing.assert_array_equal(self.c._previous_action,history)
        self.t.before_target=self.t.release_target=None
        self.cycles(3);self.assertEqual(self.c._mode,'walk')
        np.testing.assert_array_equal(self.c._command,0)
        self.assertEqual(self.c._forward.phase,'returned-to-zero')

    def test_zero_deadline_rejects_delayed_write_then_holds(self):
        self.walk();self.c._trial_began=time.monotonic()-ZERO_WINDOW_S+.03
        self.t.before_target=threading.Event();self.t.release_target=threading.Event()
        n=len(self.t.target_times);self.c._cycle();until(self.t.before_target.is_set)
        time.sleep(.04);self.t.release_target.set()
        until(lambda:self.c._io_pending['ticket'].done.is_set());self.c._io_drain_ack()
        self.assertEqual(len(self.t.target_times),n)
        self.t.before_target=self.t.release_target=None
        self.cycles(3);self.assertEqual(self.c._mode,'hold');self.assertTrue(self.t.armed)

    def test_close_failure_is_not_reported_as_confirmed_closure(self):
        self.c.stop();self.t.close_error=True
        try:
            with self.assertRaisesRegex(OwnerError,'close failure'):
                self.c._io_owner.close()
            self.assertFalse(self.c._io_owner.finished)
            self.assertTrue(self.c._io_owner.thread.is_alive())
            self.assertFalse(self.t.closed)
        finally:
            self.t.close_error=False;self.c._io_owner.close()

    def test_same_sample_never_refreshes_age_and_uint32_wrap_remains_valid(self):
        self.hold()
        self.t.freeze_sample=True
        self.t.sample_us=0xfffffff0
        until(lambda:struct.unpack_from('<I',self.c._io_owner.snapshot().payload)[0]==0xfffffff0)
        self.c._io_accept_cache_locked();stamp=self.c._sample_received
        time.sleep(.025);self.c._io_accept_cache_locked()
        self.assertEqual(self.c._sample_received,stamp)
        self.t.sample_us=0x20
        until(lambda:struct.unpack_from('<I',self.c._io_owner.snapshot().payload)[0]==0x20)
        self.c._io_accept_cache_locked()
        self.assertEqual(self.c._last_sample,0x20)
        self.assertTrue(self.c._leg_following_status_locked()['ready'])

    def test_imu_age_is_not_hidden_by_fresh_uart_receive(self):
        self.hold();self.t.imu_age_us=120000
        until(lambda:((struct.unpack_from('<I',self.c._io_owner.snapshot().payload)[0]-
            struct.unpack_from('<I',self.c._io_owner.snapshot().payload,4)[0])&0xffffffff)==120000)
        self.c._cycle();self.assertEqual(self.c._mode,'off');self.assertFalse(self.t.armed)

    def test_real_oneshot_stop_fences_before_cached_diagnostics(self):
        import native_runtime
        self.c.close()
        namespace=dict(motion=motion,time=time,request_counts={},health_error=None,
                       exit_event=threading.Event())
        exec(native_runtime.controller_classes((a.source/'service/worker-classes.py').read_text()),namespace)
        self.t=GuardedTransport('fake-only')
        self.c=namespace['OneShotController']({'walk':FakePolicy()},self.cfg,
            transport_factory=lambda _:self.t,allow_fixed_head_walk=True,serial_owner=True)
        self.c._connect_once();self.hold()
        self.t.before_target=threading.Event();self.t.release_target=threading.Event()
        n=len(self.t.target_times);self.c._cycle();until(self.t.before_target.is_set)
        diagnostics_started=threading.Event();release_diagnostics=threading.Event()
        def broken_diagnostic():
            diagnostics_started.set();release_diagnostics.wait(.3)
            raise RuntimeError('injected diagnostic failure')
        self.c._io_owner.timing_diagnostics=broken_diagnostic
        errors=[]
        def stop():
            try:self.c.stop()
            except BaseException as error:errors.append(error)
        thread=threading.Thread(target=stop);thread.start();until(diagnostics_started.is_set)
        self.t.release_target.set();time.sleep(.015)
        self.assertEqual(len(self.t.target_times),n)
        release_diagnostics.set();thread.join(.5)
        self.assertFalse(thread.is_alive());self.assertFalse(errors)
        self.assertFalse(self.t.armed);self.assertEqual(self.c._mode,'off')
        self.assertIn('diagnostic_error',self.c._last_stop_diagnostic['transport'])

    def _make_revalidation_pending(self):
        self.hold();owner=self.c._io_owner
        # Establish the intended race with a *new* target. Otherwise an ACK
        # left from fixture setup can make _cycle return before enqueueing it.
        self.c._io_wait_until(time.monotonic()+.04)
        self.assertIsNone(self.c._io_pending)
        self.t.before_state=threading.Event();self.t.release_state=threading.Event()
        until(self.t.before_state.is_set)
        self.c._cycle()
        ticket=self.c._io_pending['ticket']
        # Make the next periodic poll already due at first cancellation. This
        # deterministically reproduces v1's self-invalidating recheck race.
        with owner.cv:
            owner.next_poll=time.monotonic()-.01
            self.revalidation_frame_serial=owner.snapshot_serial+1
        self.t.release_state.set();until(ticket.done.is_set)
        self.assertIsInstance(ticket.error,SnapshotAdvanced)
        return owner,ticket

    def test_revalidation_priority_prevents_self_invalidating_poll(self):
        owner,ticket=self._make_revalidation_pending()
        snapshot=owner.snapshot();n=len(self.t.target_times)
        # Emulate six ms of X5 control scheduling/validation, longer than the
        # old overdue polling budget; the current validated frame must survive.
        time.sleep(.006)
        self.assertEqual(owner.snapshot().serial,self.revalidation_frame_serial)
        self.c._io_wait_until(time.monotonic()+.02)
        self.assertEqual(len(self.t.target_times),n+1)
        self.assertTrue(ticket.done.is_set());self.assertFalse(ticket.transmitted)
        # Event handling alone cannot create the next policy/control target.
        self.c._io_wait_until(time.monotonic()+.025)
        self.assertEqual(len(self.t.target_times),n+1)
        self.assertEqual(owner.metrics['max_target_slots'],1)

    def test_revalidation_window_expires_without_control_and_cannot_renew(self):
        owner,ticket=self._make_revalidation_pending()
        snapshot=owner.snapshot();n=len(self.t.target_times)
        window=ticket.revalidation_until
        time.sleep(.03)
        until(lambda:owner.snapshot().serial>snapshot.serial)
        self.assertEqual(len(self.t.target_times),n)
        owner.take_target(ticket)
        # Revalidation against an obsolete snapshot is intentionally rejected;
        # carrying this same logical target cannot mint a fresh 20ms pause.
        retry=owner.submit_target(ticket.args[1],ticket.deadline,snapshot.serial,retry_of=ticket)
        until(retry.done.is_set)
        self.assertIsInstance(retry.error,SnapshotAdvanced)
        self.assertEqual(retry.revalidation_until,window)
        self.assertLessEqual(retry.deadline,ticket.deadline)
        self.assertLess(owner.revalidation_until,time.monotonic())
        self.assertEqual(len(self.t.target_times),n)

    def test_revalidation_window_does_not_delay_stop(self):
        owner,ticket=self._make_revalidation_pending();n=len(self.t.target_times)
        self.c.stop()
        self.assertFalse(self.t.armed);self.assertEqual(len(self.t.target_times),n)
        self.assertEqual(owner.revalidation_until,0.)

    def test_short_write_never_acknowledges_or_commits(self):
        self.hold();self.t.short_write=True;n=len(self.c._target_times);self.c._cycle()
        until(lambda:self.c._io_pending['ticket'].done.is_set())
        with self.assertRaises(motion.MotionError):self.c._io_drain_ack()
        self.assertEqual(len(self.c._target_times),n)


def run_tests():
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(OwnerTests)
    if a.only:suite=unittest.TestSuite(t for t in suite if a.only in t.id())
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                passed=result.wasSuccessful(),hardware_access=False,owner_enabled=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    return 0 if result.wasSuccessful() else 1

def run_benchmark():
    import hashlib
    import signal
    import subprocess
    import onnxruntime as ort
    from policy_worker import PolicyWorker
    if a.models is None:raise ValueError('--benchmark requires --models')
    if not 0<=a.warmup_seconds<=5 or not 1<=a.segment_seconds<=8:
        raise ValueError('Bounded benchmark duration exceeded')
    if a.ledger_dir and a.ledger_dir.exists():raise ValueError('Ledger directory must be new')
    ledger=None
    if a.ledger_dir:
        from control_time_ledger import ControlTimeLedger
        ledger=ControlTimeLedger()
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
    model=a.models/'xgoduck_walk.onnx';digest=hashlib.sha256(model.read_bytes()).hexdigest()
    if digest!='5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f':
        raise ValueError('Benchmark requires the reviewed original actor')
    session=ort.InferenceSession(str(model),sess_options=opts,providers=['CPUExecutionProvider'])
    with tempfile.TemporaryDirectory() as tmp:
        config=Path(tmp)/'calibration.json';config.write_text(json.dumps(ready_config()))
        t=GuardedTransport('fake-only')
        if a.transport_timing:
            rows=json.loads(a.transport_timing.read_text())['transport']['requests']
            for row in rows:
                if row.get('error') is None and row['kind'] in (motion.STATE,motion.VOLTAGE_STATE,motion.TARGET):
                    kind=motion.VOLTAGE_STATE if row['kind'] in (motion.STATE,motion.VOLTAGE_STATE) else row['kind']
                    delay=(row['elapsed_ms']+row.get('identity_proof_ms',0))/1000
                    if not np.isfinite(delay) or not 0<=delay<.16:raise ValueError('Invalid recorded duration')
                    t.delays.setdefault(kind,[]).append(delay)
            if set(t.delays)!={motion.VOLTAGE_STATE,motion.TARGET}:raise ValueError('Incomplete delay replay')
            if a.delay_mode=='none':t.delays.clear()
            elif a.delay_mode=='state':t.delays.pop(motion.TARGET)
            elif a.delay_mode=='target':t.delays.pop(motion.VOLTAGE_STATE)
        c=motion.MotionController({'walk':session},config,transport_factory=lambda _:t,
            allow_fixed_head_walk=True,serial_owner=True)
        worker=PolicyWorker(a.models,{'walk':digest});c.attach_policy_worker(worker)
        if ledger:
            ledger.attach_controller(c);ledger.attach_worker(worker);t.ledger=ledger
        display=None;done=threading.Event();publisher=None
        def tick():
            start=time.monotonic();cpu=time.thread_time()
            if ledger:ledger.emit('benchmark_tick_begin',planned_end=start+.02-ledger.origin)
            c.heartbeat(minimal=True);c._cycle()
            active_end=time.monotonic()
            c._io_wait_until(max(start+.02,time.monotonic()+.001))
            if ledger:ledger.emit('benchmark_tick_end',start=start,cpu_start=cpu,
                overrun_ms=max(0.,(time.monotonic()-start-.02)*1000))
            if c._mode!='walk':raise AssertionError('Fake plant stopped: '+str(c._error))
            return dict(cpu_ms=(time.thread_time()-cpu)*1000,active_ms=(active_end-start)*1000,
                        cycle_ms=(time.monotonic()-start)*1000)
        cases=[]
        def segment(name,seconds):
            if ledger:ledger.segment=name
            began=time.monotonic();target_start=len(t.target_times);sample_start=len(t.sample_times);rows=[]
            while time.monotonic()-began<seconds:rows.append(tick())
            targets=t.target_times[target_start:];samples=t.sample_times[sample_start:]
            def hz(values):return (len(values)-1)/(values[-1]-values[0]) if len(values)>1 else None
            gaps=[(b-a)*1000 for a,b in zip(targets,targets[1:])]
            cases.append(dict(name=name,seconds=time.monotonic()-began,targets=len(targets),samples=len(samples),
                target_write_hz=hz(targets),sample_receive_hz=hz(samples),
                min_target_gap_ms=min(gaps,default=None),
                max_target_gap_ms=max(gaps,default=0),p95_target_gap_ms=float(np.percentile(gaps,95)) if gaps else None,
                mean_control_cpu_ms=float(np.mean([r['cpu_ms'] for r in rows])),
                mean_active_control_ms=float(np.mean([r['active_ms'] for r in rows])),
                mean_cycle_ms=float(np.mean([r['cycle_ms'] for r in rows]))))
        try:
            sys.setswitchinterval(.001)
            c._connect_once();c.capture_head_clearance(clearance_confirmed=True,expected_generation=c._generation)
            if ledger:ledger.attach_owner(c._io_owner)
            c.start_mode('hold',support_confirmed=True);c._hold_begin-=3.1
            for _ in range(8):
                start=time.monotonic();c._cycle();c._io_wait_until(start+.02)
            c.start_mode('walk',support_confirmed=True)
            if a.warmup_seconds:segment('warmup',a.warmup_seconds)
            segment('no_display_readers',a.segment_seconds)
            snapshot=Path(tmp)/'display.json';snapshot.write_text(json.dumps(c.status(),allow_nan=False))
            def publish():
                while not done.is_set():
                    temp=snapshot.with_suffix('.tmp');temp.write_text(json.dumps(c.status(),allow_nan=False))
                    os.replace(temp,snapshot);done.wait(.2)
            display=subprocess.Popen([sys.executable,'-I','-B',str(Path(__file__).with_name('validate_async_policy.py')),
                '--display-worker',str(snapshot),str(Path(tmp)/'http-proof.json')])
            publisher=threading.Thread(target=publish);publisher.start()
            segment('cached_http_process_eight_readers',a.segment_seconds)
            os.kill(worker.process.pid,signal.SIGSTOP);before=len(t.sample_times);history=c._previous_action.copy()
            segment('inference_paused_120ms',.12)
            during=len(t.sample_times)-before
            os.kill(worker.process.pid,signal.SIGCONT)
            segment('recovered',1.)
            if during<4:raise AssertionError('Inference stalled feedback owner')
            if c._late_policy_frames<1:raise AssertionError('Late inference was not rejected')
            worker.process.terminate();worker.process.wait(timeout=2.)
            error=None
            for _ in range(5):
                try:tick()
                except Exception as exc:error=str(exc);break
            if error is None:raise AssertionError('Inference exit was not detected')
            c.stop();display.wait(timeout=15.)
            http=json.loads((Path(tmp)/'http-proof.json').read_text())
            if http['errors'] or http['requests']<100:raise AssertionError('HTTP pressure failed')
            report=dict(hardware_access=False,owner_enabled=True,plant='idealized_fake_UART',
                timing_ledger_enabled=ledger is not None,delay_mode=a.delay_mode,
                deferred_control_resumes=getattr(c,'_io_deferred_resumes',None),
                model_sha256=digest,cases=cases,http_pressure=http,
                owner=dict(c._io_owner.metrics),owner_thread_only=len(t.thread_ids)==1,
                max_simultaneous_transactions=t.max_active,watchdog_stops=t.watchdog_stops,
                stale_model_frames=c._late_policy_frames,feedback_during_inference_pause=during,
                inference_exit_error=error,torque_released=not t.armed,
                replayed_delays={str(k):dict(count=len(v),mean_ms=float(np.mean(v))*1000) for k,v in t.delays.items()})
            a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
        finally:
            done.set()
            if publisher:publisher.join(1.)
            if display and display.poll() is None:display.terminate();display.wait(timeout=2.)
            if worker.process.poll() is None:
                os.kill(worker.process.pid,signal.SIGCONT)
            c.close();worker.close()
            if ledger:ledger.save(a.ledger_dir)
    return 0

if __name__=='__main__':raise SystemExit(run_benchmark() if a.benchmark else run_tests())
