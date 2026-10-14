"""Resume an explicitly deferred control cycle; never add UART authority.

The UART owner enforces the minimum write interval independently. This mixin
only schedules the same full controller path after its ACK/inference blocker
clears. A mode/command/STOP invalidation removes the deferred work.
"""


class DeferredControlMixin:
    def _io_initialize(self, enabled):
        self._io_deferred_context = None
        self._io_deferred_resumes = 0
        super()._io_initialize(enabled)

    def _io_cancel_targets_locked(self):
        self._io_deferred_context = None
        return super()._io_cancel_targets_locked()

    def _io_defer_control(self, context):
        if not self._serial_owner_enabled:
            return
        with self._lock:
            if context == self._policy_context_locked():
                self._io_deferred_context = context

    def _io_can_resume(self):
        with self._lock:
            context = self._io_deferred_context
            if context is None:
                return False
            if context != self._policy_context_locked() or self._mode == 'off':
                self._io_deferred_context = None
                return False
            if self._io_pending is not None:
                return False
            job = self._policy_job
            worker = self._policy_worker
        # Readiness only; the original control path consumes and validates the
        # result. Polling cannot renew any lease or a pending action deadline.
        try:
            ready = job is None or worker.result_ready(job['request_id'])
        except Exception:
            # STOP or a new command may cancel this request after releasing
            # the controller lock. An obsolete readiness error cannot fault
            # the replacement mode; current-request failures still propagate.
            with self._lock:
                if (context != self._policy_context_locked() or
                        self._io_deferred_context != context or
                        self._policy_job is not job):
                    return False
            raise
        with self._lock:
            return (ready and self._io_deferred_context == context and
                    context == self._policy_context_locked() and
                    self._policy_job is job and self._io_pending is None)

    def _io_wait_until(self, deadline):
        if not self._serial_owner_enabled:
            return super()._io_wait_until(deadline)
        while not self._closed.is_set():
            self._io_wake.clear()
            try:
                self._io_drain_ack()
                remaining = deadline-self._clock()
                if remaining <= 0:
                    return
                if self._io_can_resume():
                    self._io_deferred_resumes += 1
                    self._cycle()
                remaining = deadline-self._clock()
                if remaining <= 0:
                    return
            except Exception as error:
                self._io_event_error = error
                return
            # UART events wake immediately. The child IPC socket is checked
            # without consuming it, at most once/ms while work is deferred.
            timeout = min(remaining,.001) if self._io_deferred_context is not None else remaining
            self._io_wake.wait(timeout)
