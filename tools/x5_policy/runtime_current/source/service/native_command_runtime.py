"""X5-owned lifetime for explicitly started, non-streaming model commands.

No hardware connection or motion command originates here. Only an existing
authenticated command owner can receive the existing controller heartbeat.
Gamepad and joint-follow input streams retain their own input watchdogs.
"""
import copy
import threading
import time

from native_gamepad import PRINCIPAL


class CommandRuntime:
    def __init__(self, workbench, monitor):
        self.wb, self.monitor = workbench, monitor
        self.stop = threading.Event()
        self.thread = None
        self.receipt = dict(heartbeat_count=0, last_error=None, last_accepted_at=None)

    def tick(self):
        with self.wb.lock:
            if self.wb.closed:
                return
            owner = copy.deepcopy(self.wb.control_owner)
        if not owner or owner['principal'] == PRINCIPAL:
            return
        state = self.wb.runtime.control_context()
        h = state.get('hardware') or {}
        if (state.get('phase') != 'ready' or not state.get('owns_connection')
                or not h.get('connected') or h.get('mode') not in
                ('shadow', 'hold', 'walk', 'getup', 'pick', 'roulade')
                or h.get('joint_follow', {}).get('active')
                or h.get('generation') != owner['generation']):
            return
        body = dict(action='heartbeat', page_generation=self.wb.generation,
            generation=owner['generation'], control_id=owner['id'], background=False)
        self.monitor.call('x5_runtime', body, lambda: self.wb.motion_action(
            body, principal=owner['principal'], live=True))
        previous = self.receipt
        self.receipt = dict(heartbeat_count=previous['heartbeat_count']+1,
            last_error=None, last_accepted_at=time.monotonic())

    def snapshot(self):
        return dict(self.receipt, protocol=1, scope='normal_model_commands', command_lifetime_owner='x5',
            requires_browser_heartbeat=False, reads_are_observation_only=True,
            automatic_start=False, input_stream_watchdogs_preserved=True)

    def _run(self):
        while not self.stop.wait(.1):
            try:
                self.tick()
            except Exception as error:
                # A refused/failed heartbeat never substitutes a command,
                # reconnect or retry of ARM. Existing runtime/MCU expiry wins.
                self.receipt = dict(self.receipt, last_error=str(error)[:500])

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=.5)
