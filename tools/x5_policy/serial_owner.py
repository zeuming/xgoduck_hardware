"""Single UART owner, bounded mailboxes and a write-time cancellation fence.

No autonomous motor authority. Only explicitly submitted, unexpired TARGETs
are sent once. STATE polling cannot renew the MCU target watchdog.
"""
from collections import deque
import copy
from contextlib import contextmanager
from dataclasses import dataclass, field
import threading
import time


class OwnerError(RuntimeError):
    pass


class TargetCancelled(OwnerError):
    pass


class SnapshotAdvanced(TargetCancelled):
    pass


@dataclass
class Ticket:
    kind: str
    args: tuple
    epoch: int
    deadline: float = float('inf')
    done: threading.Event = field(default_factory=threading.Event)
    result: object = None
    error: object = None
    began: float = None
    ended: float = None
    transmitted: bool = False
    received_at: float = None
    state_serial: object = None
    cancelled: bool = False
    request_sequence: object = None
    revalidation_until: object = None


@dataclass(frozen=True)
class StateSnapshot:
    payload: bytes
    received_at: float
    sequence: object
    epoch: int
    serial: int


class SerialOwner:
    """A proxy whose worker alone constructs/opens/requests/closes transport.

    The condition is never held during blocking reads. The same condition is
    used only across the transport's one nonblocking TARGET write, making
    cancel-vs-write linearizable. Once bytes were sent, STOP cannot unsend them;
    it is the next transaction, bounded by the original request timeout.
    """
    def __init__(self, factory, port, *, clock=time.monotonic, wake=None, period=.02,
                 target_kind=5, stop_kind=6, state_kinds=(2,19), barrier_kinds=(3,4,6), timing_callback=None):
        self.factory, self.port, self.clock = factory, port, clock
        self.wake = wake or threading.Event()
        self.period, self.target_kind, self.stop_kind = period, target_kind, stop_kind
        self.state_kinds, self.barrier_kinds = state_kinds, barrier_kinds
        self.timing_callback = timing_callback
        self.cv = threading.Condition(threading.RLock())
        self.epoch = 0
        self.queue = deque()
        self.target = None
        self.latest = None
        self.snapshot_serial = 0
        self.sampling = None
        self.next_poll = 0.
        self.revalidation_until = 0.
        self.fault = None
        self.finished = False
        self._transport = None
        self._active_target = None
        self._active = None
        self.seq = None
        self.last_sync_state_received = None
        self.last_sync_state_sequence = None
        self.diagnostics = {}
        self.metrics = dict(target_submitted=0,target_write_attempted=0,target_acked=0,target_cancelled=0,
                            max_target_slots=0,state_received=0,revalidation_windows=0)
        self.thread = threading.Thread(target=self._run,name='duck-uart-owner',daemon=True)
        self.thread.start()

    @property
    def serial(self):
        return getattr(self._transport,'serial',None)

    def timing_diagnostics(self):
        # Immutable replace-only publication. This MUST NOT enqueue or wait:
        # OneShot records it on the STOP path before asking for a STOP reply.
        snapshot=self.diagnostics
        result=copy.deepcopy(snapshot)
        result['snapshot_only']=True
        result['inflight_not_included']=self._active is not None
        return result

    def _cancel_locked(self, clear_state=False):
        self.epoch += 1
        self.revalidation_until = 0.
        if clear_state:
            self.latest = None
        ticket = self.target
        if ticket is not None and not ticket.done.is_set() and ticket is not self._active:
            ticket.error = TargetCancelled('TARGET cancelled before transmission')
            ticket.ended = self.clock(); ticket.done.set()
            self.metrics['target_cancelled'] += 1
        for queued in self.queue:
            if queued.kind == 'request' and queued.args[0] not in (*self.state_kinds,self.stop_kind):
                queued.cancelled=True
                queued.error=TargetCancelled('Barrier invalidated before dispatch')
                queued.ended=self.clock();queued.done.set()
        self.cv.notify_all(); self.wake.set()

    def cancel_targets(self, *, clear_state=False):
        with self.cv:
            self._cancel_locked(clear_state)
            # In-flight tickets retain their own identity, but no caller can
            # accidentally consume their later ACK as a new command's result.
            self.target = None

    @contextmanager
    def target_write_guard(self):
        with self.cv:
            ticket = self._active_target
            if ticket is None or ticket.epoch != self.epoch or self.clock() >= ticket.deadline:
                raise TargetCancelled('TARGET cancelled or expired before UART write')
            if self.fault is not None:
                raise OwnerError(str(self.fault))
            if self.latest is None or self.latest.serial != ticket.state_serial:
                raise SnapshotAdvanced('STATE advanced; TARGET requires control revalidation')
            # This is an attempt, not a full write, ACK, or servo acceptance.
            # An uncertain partial write must never become a retry candidate.
            ticket.transmitted = True
            self.metrics['target_write_attempted'] += 1
            yield

    @contextmanager
    def request_write_guard(self, kind):
        if kind==self.target_kind:
            with self.target_write_guard():yield
            return
        with self.cv:
            ticket=self._active
            if kind in self.barrier_kinds and kind!=self.stop_kind:
                if (ticket is None or ticket.cancelled or ticket.epoch!=self.epoch or
                        self.clock()>=ticket.deadline):
                    raise TargetCancelled('Barrier cancelled or expired before UART write')
            yield

    def _call(self, kind, *args):
        with self.cv:
            if self.finished:
                raise OwnerError('UART owner is closed')
            if self.fault is not None and kind not in ('close',) and not (
                    kind=='request' and args[0] in (*self.state_kinds,self.stop_kind)):
                raise OwnerError(str(self.fault))
            if len(self.queue) >= 4:
                raise OwnerError('UART barrier queue is full')
            if kind=='request' and args[0] in self.barrier_kinds:
                self._cancel_locked(clear_state=True)
                self.target = None
            ticket = Ticket(kind,args,self.epoch,self.clock()+2.)
            if kind=='close' or kind=='request' and args[0]==self.stop_kind:
                self.queue.appendleft(ticket)
            else:
                self.queue.append(ticket)
            self.cv.notify_all()
        # Existing startup/ARM/STOP paths are synchronous barriers. Runtime
        # STATE and TARGET use mailboxes, never this wait. No controller lock
        # is acquired by the owner, so callers may hold their existing lock.
        if not ticket.done.wait(2.):
            with self.cv:
                ticket.cancelled=True
                self._cancel_locked(clear_state=True)
            raise OwnerError('UART owner barrier did not complete within 2 seconds')
        if ticket.error is not None:
            raise ticket.error
        if kind == "request" and args[0] in self.state_kinds:
            self.last_sync_state_received = ticket.received_at
            self.last_sync_state_sequence = ticket.request_sequence
        return ticket.result

    def open(self): return self._call('open')
    def request(self,kind,payload=b''):
        if kind==self.target_kind:
            raise OwnerError('Owner TARGET requires a fresh, explicit single-slot submission')
        return self._call('request',kind,bytes(payload))
    def negotiate_voltage(self): return self._call('negotiate_voltage')
    def negotiate_motion_evidence(self): return self._call('negotiate_motion_evidence')

    def start_sampling(self,kind,payload):
        with self.cv:
            self.sampling=(kind,bytes(payload)); self.next_poll=time.monotonic()
            self.cv.notify_all()

    def snapshot(self):
        with self.cv:
            if self.fault is not None: raise OwnerError(str(self.fault))
            return self.latest

    def submit_target(self,payload,deadline,state_serial,*,retry_of=None):
        with self.cv:
            if self.fault is not None or self.finished:
                raise OwnerError(str(self.fault or 'UART owner is closed'))
            if self.target is not None:
                raise OwnerError('Only one TARGET may be unacknowledged')
            if self.clock() >= deadline:
                raise TargetCancelled('TARGET expired before enqueue')
            if retry_of is not None:
                if (not retry_of.done.is_set() or retry_of.transmitted or
                        not isinstance(retry_of.error,SnapshotAdvanced) or
                        retry_of.epoch!=self.epoch or retry_of.args[1]!=bytes(payload)):
                    raise OwnerError('Invalid untransmitted TARGET revalidation')
                # A second cancellation must never grant a fresh window or
                # extend the original action/feedback deadline.
                deadline=min(deadline,retry_of.deadline)
            ticket=Ticket('target',(self.target_kind,bytes(payload)),self.epoch,deadline,
                state_serial=state_serial,revalidation_until=None if retry_of is None else retry_of.revalidation_until)
            self.revalidation_until=0.
            self.target=ticket
            self.metrics['target_submitted']+=1;self.metrics['max_target_slots']=1
            self.cv.notify_all()
            return ticket

    def take_target(self,ticket):
        with self.cv:
            if not ticket.done.is_set(): return None
            if self.target is ticket: self.target=None
            return ticket

    def close(self):
        with self.cv:
            if self.finished: return
            self._cancel_locked(clear_state=True);self.target=None;self.sampling=None
            for queued in self.queue:
                queued.cancelled=True;queued.error=OwnerError('UART owner is closing')
                queued.ended=self.clock();queued.done.set()
            self.queue.clear()
        try: self._call('close')
        finally:
            self.thread.join(2.)
        if self.thread.is_alive(): raise OwnerError('UART owner closure is unconfirmed')

    def _execute(self,ticket):
        ticket.began=self.clock()
        try:
            if ticket.cancelled:raise TargetCancelled('Cancelled barrier cannot execute')
            if (ticket.kind not in ('close',) and not (ticket.kind=='request' and
                    ticket.args[0] in (*self.state_kinds,self.stop_kind)) and
                    (ticket.epoch!=self.epoch or self.clock()>=ticket.deadline)):
                raise TargetCancelled('Expired or superseded barrier cannot execute')
            if ticket.kind=='open':
                self._transport=self.factory(self.port)
                if not getattr(self._transport,'supports_target_write_guard',False):
                    raise OwnerError('Transport must implement the atomic TARGET write guard')
                self._transport._target_write_guard=self.target_write_guard
                self._transport._request_write_guard=self.request_write_guard
                ticket.result=self._transport.open()
            elif ticket.kind=='close':
                if self._transport is not None: self._transport.close()
                self.finished=True
            elif ticket.kind.startswith('negotiate_'):
                ticket.result=getattr(self._transport,ticket.kind)()
            else:
                if ticket.kind=='target':
                    self._active_target=ticket
                    with self.cv:
                        if ticket.epoch!=self.epoch or self.clock()>=ticket.deadline:
                            raise TargetCancelled('TARGET cancelled or expired before dispatch')
                ticket.result=self._transport.request(*ticket.args)
                self.seq=getattr(self._transport,'seq',None)
                ticket.request_sequence=self.seq
                received=self.clock()
                ticket.received_at=received
                if ticket.args[0] in self.state_kinds:
                    with self.cv:
                        if ticket.epoch==self.epoch:
                            self.snapshot_serial+=1
                            self.latest=StateSnapshot(bytes(ticket.result),received,self.seq,
                                                      ticket.epoch,self.snapshot_serial)
                            self.metrics['state_received']+=1
                if ticket.kind=='target':
                    if ticket.result!=bytes([self.target_kind,0]) or not ticket.transmitted:
                        raise OwnerError('TARGET did not receive its matching success ACK')
                    self.metrics['target_acked']+=1
            method=getattr(self._transport,'timing_diagnostics',None)
            try:
                if callable(method):
                    diagnostic=method()
                    self.diagnostics=dict(diagnostic,snapshot_received_monotonic=self.clock())
            except Exception as diagnostic_error:
                # Optional diagnostics can neither fail a successful command
                # nor prevent the owner from serving a priority STOP.
                self.diagnostics=dict(self.diagnostics,diagnostic_error=str(diagnostic_error))
        except BaseException as error:
            ticket.error=error
            if isinstance(error,TargetCancelled):
                self.metrics['target_cancelled']+=1
                if isinstance(error,SnapshotAdvanced) and not ticket.transmitted:
                    with self.cv:
                        if ticket.epoch==self.epoch:
                            if ticket.revalidation_until is None:
                                # Give the existing control thread one nominal
                                # period to recheck this received STATE, instead
                                # of invalidating it ourselves with another poll.
                                ticket.revalidation_until=time.monotonic()+min(
                                    self.period,max(0.,ticket.deadline-self.clock()))
                                self.metrics['revalidation_windows']+=1
                            self.revalidation_until=ticket.revalidation_until
            elif ticket.kind not in ('close',):
                with self.cv:
                    self.fault=error;self.sampling=None;self._cancel_locked(clear_state=True)
        finally:
            self._active_target=None
            ticket.ended=self.clock()
            if self.timing_callback is not None and ticket.kind in ('request','target'):
                name=('state_request' if ticket.args[0] in self.state_kinds else
                      'target_request' if ticket.args[0]==self.target_kind else None)
                if name is not None:self.timing_callback(name,ticket.began,ticket.error is not None)
            ticket.done.set();self.wake.set()

    def _run(self):
        while True:
            with self.cv:
                while True:
                    while self.queue and self.queue[0].cancelled:self.queue.popleft()
                    if self.queue:
                        ticket=self.queue.popleft();break
                    if self.finished:return
                    if self.target is not None and not self.target.done.is_set():
                        ticket=self.target;break
                    now=time.monotonic()
                    poll_at=max(self.next_poll,self.revalidation_until)
                    if self.sampling is not None and now>=poll_at:
                        ticket=Ticket('request',self.sampling,self.epoch)
                        # One deadline, no catch-up burst or stale request FIFO.
                        self.next_poll=now+self.period;break
                    timeout=max(0,poll_at-now) if self.sampling else None
                    self.cv.wait(timeout)
                self._active=ticket
            self._execute(ticket)
            with self.cv:self._active=None
