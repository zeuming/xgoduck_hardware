"""Bounded passive host timing; never changes GC, scheduling or control gates."""
import collections
import gc
import threading
import time


class HostLoopEvidence:
    def __init__(self, clock=time.monotonic, cpu_clock=time.thread_time):
        self.clock, self.cpu_clock = clock, cpu_clock
        self.cycles = collections.deque(maxlen=150)
        self.collections = collections.deque(maxlen=16)
        self.current = None
        self.previous_end = None
        self.requested_wait = 0.
        self._gc_start = None
        self._callback = self._on_gc

    def start(self):
        gc.callbacks.append(self._callback)

    def close(self):
        # Remove only our callback; leave the collector and other observers alone.
        if self._callback in gc.callbacks:
            gc.callbacks.remove(self._callback)

    def begin_cycle(self, mode=None, generation=None):
        now = self.clock()
        gap = None if self.previous_end is None else now - self.previous_end
        self.current = dict(began_monotonic=now, began_cpu=self.cpu_clock(),
            mode=mode, generation=generation,
            previous_cycle_ended_monotonic=self.previous_end,
            requested_wait_ms=self.requested_wait * 1000,
            between_cycles_ms=None if gap is None else gap * 1000,
            wake_lateness_ms=None if gap is None else max(0., gap-self.requested_wait) * 1000)

    def end_cycle(self):
        now, cpu = self.clock(), self.cpu_clock()
        if self.current is not None:
            self.current = dict(self.current, ended_monotonic=now,
                wall_ms=(now-self.current['began_monotonic'])*1000,
                thread_cpu_ms=(cpu-self.current['began_cpu'])*1000)
            self.cycles.append(self.current)
        self.previous_end, self.requested_wait = now, 0.

    def waiting(self, seconds):
        self.requested_wait = seconds

    def _on_gc(self, phase, info):
        # CPython runs callbacks with the GIL. No locks, heap traversal, file
        # writes or controller calls here; even collection on another service
        # thread can delay the control thread from reacquiring the GIL.
        if phase == 'start':
            self._gc_start = (self.clock(), self.cpu_clock(), threading.get_ident())
        elif phase == 'stop' and self._gc_start is not None:
            start, cpu, thread = self._gc_start
            self._gc_start = None
            self.collections.append(dict(began_monotonic=start,
                ended_monotonic=self.clock(),
                thread_cpu_ms=(self.cpu_clock()-cpu)*1000,
                thread_id=thread, generation=info.get('generation'),
                collected=info.get('collected'), uncollectable=info.get('uncollectable')))

    def snapshot(self):
        # A dict allocation below may itself trigger GC and its callback may
        # append to collections. Detach the rings before formatting their rows.
        cycles, collections = tuple(self.cycles), tuple(self.collections)
        return dict(protocol=1, scope='passive_host_timing_not_root_cause_attribution',
            current=None if self.current is None else dict(self.current),
            cycles=[dict(row) for row in cycles],
            garbage_collections=[dict(row) for row in collections])
