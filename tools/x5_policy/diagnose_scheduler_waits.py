"""Observe requested and actual waits in the existing hardware-denied harness.

No scheduler, timeout, action, plant or priority changes. Diagnostic wrappers
append bounded in-memory events; their overhead is not assumed to be zero.
"""
import time
import control_time_ledger
import validate_serial_owner as base


class WaitLedger(control_time_ledger.ControlTimeLedger):
    def observe_wait(self, obj, label):
        original = obj.wait
        def wait(timeout=None):
            start, cpu = time.monotonic(), time.thread_time()
            value = original(timeout)
            end = time.monotonic()
            self.emit(label, start=start, cpu_start=cpu, requested_ms=None if timeout is None else timeout*1000,
                      measured_wait_ms=(end-start)*1000, signalled=bool(value),
                      excess_over_requested_ms=None if timeout is None else max(0.,end-start-timeout)*1000)
            return value
        obj.wait = wait

    def attach_controller(self, controller):
        super().attach_controller(controller)
        self.observe_wait(controller._io_wake, 'controller_event_wait')

    def attach_owner(self, owner):
        super().attach_owner(owner)
        self.observe_wait(owner.cv, 'owner_condition_wait')


if __name__ == '__main__':
    if not base.a.benchmark or not base.a.ledger_dir:
        raise ValueError('Requires existing bounded fake benchmark and new ledger output')
    control_time_ledger.ControlTimeLedger = WaitLedger
    raise SystemExit(base.run_benchmark())
