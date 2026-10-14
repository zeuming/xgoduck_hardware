"""Opt-in fake-transport diagnostics. No control decisions or hardware authority.

Events are bounded in memory and written only after the control run. Logical
target identities survive SnapshotAdvanced retries; ticket identities do not.
Timing here is observational and its overhead must be measured separately.
"""
from contextlib import contextmanager
import functools
import json
from pathlib import Path
import threading
import time


class ObservedLock:
    def __init__(self, underlying, ledger):
        self.underlying, self.ledger = underlying, ledger

    def acquire(self, *args, **kwargs):
        start, cpu = time.monotonic(), time.thread_time()
        acquired = self.underlying.acquire(*args, **kwargs)
        self.ledger.emit('control_lock', start=start, cpu_start=cpu, acquired=acquired,
                         stage=getattr(self.ledger.local, 'stage', None))
        return acquired

    def release(self):
        return self.underlying.release()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *args):
        self.release()


class ControlTimeLedger:
    def __init__(self, maximum_events=250000):
        self.maximum_events = maximum_events
        self.events, self.dropped = [], 0
        self.origin = time.monotonic()
        self.local = threading.local()
        self.logical_sequence = self.ticket_sequence = self.tick_sequence = 0
        self.segment = 'setup'

    def emit(self, name, *, start=None, cpu_start=None, **fields):
        now = time.monotonic()
        if len(self.events) >= self.maximum_events:
            self.dropped += 1
            return
        event = dict(event=name, at=now-self.origin, thread=threading.get_ident(),
                     segment=self.segment, **fields)
        if start is not None:
            event.update(start=start-self.origin, wall_ms=(now-start)*1000)
        if cpu_start is not None:
            event['cpu_ms'] = (time.thread_time()-cpu_start)*1000
        self.events.append(event)

    @staticmethod
    def identity(ticket):
        return dict(logical_id=getattr(ticket, '_ledger_logical', None),
                    ticket_id=getattr(ticket, '_ledger_ticket', None))

    def attach_owner(self, owner):
        original_submit = owner.submit_target
        def submit(payload, deadline, state_serial, *, retry_of=None):
            # Existing condition is reentrant. Assign identity before waking
            # the writer, without changing dispatch order or deadline.
            with owner.cv:
                ticket = original_submit(payload, deadline, state_serial, retry_of=retry_of)
                if retry_of is None:
                    self.logical_sequence += 1
                    logical_id = self.logical_sequence
                else:
                    logical_id = retry_of._ledger_logical
                self.ticket_sequence += 1
                ticket._ledger_logical = logical_id
                ticket._ledger_ticket = self.ticket_sequence
                self.emit('target_enqueue', **self.identity(ticket), retry=retry_of is not None,
                          state_serial=state_serial, deadline=ticket.deadline-self.origin,
                          inference_request_id=getattr(self.local, 'target_request_id', None))
                return ticket
        owner.submit_target = submit
        original_guard = owner.target_write_guard
        @contextmanager
        def write_guard():
            with original_guard():
                ticket = owner._active_target
                self.emit('target_write', **self.identity(ticket),
                          state_serial=ticket.state_serial, deadline=ticket.deadline-self.origin)
                yield
        # Transport receives the guard during open, so replace both references.
        owner.target_write_guard = write_guard
        owner._transport._target_write_guard = write_guard
        original_execute = owner._execute
        def execute(ticket):
            start, cpu = time.monotonic(), time.thread_time()
            self.emit('transaction_begin', kind=ticket.args[0] if ticket.kind in ('request','target') else ticket.kind,
                      **self.identity(ticket))
            try:
                return original_execute(ticket)
            finally:
                self.emit('transaction_end', start=start, cpu_start=cpu,
                          kind=ticket.args[0] if ticket.kind in ('request','target') else ticket.kind,
                          **self.identity(ticket), transmitted=ticket.transmitted,
                          error=None if ticket.error is None else type(ticket.error).__name__,
                          snapshot_serial=owner.snapshot_serial,
                          request_sequence=ticket.request_sequence)
        owner._execute = execute

    def attach_controller(self, controller):
        controller._lock = ObservedLock(controller._lock, self)
        self.controller = controller
        names = ('_cycle', '_io_accept_cache_locked', '_motion_gate_locked',
                 '_raw_targets_locked', '_require_feedback_locked', '_async_policy_step',
                 '_submit_policy_frame', '_io_drain_ack', '_io_submit_target_locked')
        for name in names:
            original = getattr(controller, name)
            def make_wrapper(method, label):
                @functools.wraps(method)
                def wrapped(*args, **kwargs):
                    start, cpu = time.monotonic(), time.thread_time()
                    previous_stage = getattr(self.local, 'stage', None)
                    self.local.stage = label
                    job = getattr(controller, '_policy_job', None)
                    request_before = None if job is None else job.get('request_id')
                    pending_before = controller._io_pending is not None
                    if label == '_cycle':
                        self.tick_sequence += 1
                        self.local.tick_id = self.tick_sequence
                        self.local.async_state = None
                        self.local.enqueues = 0
                    if label == '_io_submit_target_locked':
                        self.local.enqueues = getattr(self.local, 'enqueues', 0)+1
                        self.local.target_request_id = getattr(self.local, 'last_consumed_request', None)
                    result, error = None, None
                    try:
                        result = method(*args, **kwargs)
                        if label == '_async_policy_step':
                            self.local.async_state = ('inference_submitted' if request_before is None else 'inference_not_ready') if result is None else 'inference_consumed'
                            if result is not None:
                                self.local.last_consumed_request = request_before
                        return result
                    except BaseException as exc:
                        error = type(exc).__name__
                        raise
                    finally:
                        job_after = getattr(controller, '_policy_job', None)
                        fields = dict(tick_id=getattr(self.local,'tick_id',None),
                            mode=controller._mode, request_before=request_before,
                            request_after=None if job_after is None else job_after.get('request_id'),
                            pending_before=pending_before, pending_after=controller._io_pending is not None,
                            snapshot_serial=controller._io_snapshot_serial, error=error)
                        if label == '_cycle':
                            fields['outcome'] = ('target_enqueued' if self.local.enqueues else
                                self.local.async_state or ('target_ack_pending' if pending_before else 'no_target_other'))
                        self.emit(label, start=start, cpu_start=cpu, **fields)
                        self.local.stage = previous_stage
                return wrapped
            setattr(controller, name, make_wrapper(original, name))

    def attach_worker(self, worker):
        for name in ('submit', 'take', 'discard'):
            original = getattr(worker, name)
            def make_wrapper(method, label):
                def wrapped(*args, **kwargs):
                    start, cpu = time.monotonic(), time.thread_time()
                    result, error = None, None
                    try:
                        result = method(*args, **kwargs)
                        return result
                    except BaseException as exc:
                        error = type(exc).__name__
                        raise
                    finally:
                        request_id = result if label == 'submit' else (args[0] if args else None)
                        self.emit('inference_'+label, start=start, cpu_start=cpu,
                            request_id=request_id, ready=result is not None, error=error,
                            compute_ms=result[1] if label == 'take' and result is not None else None)
                return wrapped
            setattr(worker, name, make_wrapper(original, name))

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=False)
        with (directory/'events.jsonl').open('w') as output:
            for event in sorted(self.events, key=lambda row:row['at']):
                output.write(json.dumps(event, allow_nan=False)+'\n')
        (directory/'meta.json').write_text(json.dumps(dict(events=len(self.events), dropped=self.dropped,
            clock='monotonic; relative seconds', physical_uart=False, instrumentation_overhead_unknown=True),indent=2)+'\n')
        if self.dropped:
            raise RuntimeError('Timing ledger overflow; incomplete evidence')
