"""Exercise the packaged command heartbeat and voltage rollback with fake UART."""
import argparse
import json
from pathlib import Path
import runpy
import sys
import threading
import types
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--candidate', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
sys.argv = ['test_source_host', '--candidate', str(args.candidate), '--output', str(args.output)]
helpers = runpy.run_path(str(Path(__file__).with_name('test_source_host.py')),
                        run_name='source_test_helpers')
sys.path.insert(0, str(args.candidate / 'service'))
from native_gamepad import ControlMonitor
from native_command_runtime import CommandRuntime


class StanceHeartbeatTests(helpers['RuntimeTests']):
    def command_runtime(self):
        calls = []
        controller = self.c

        def action(body, *, principal, live):
            calls.append(body)
            return controller.heartbeat(expected_generation=body['generation'])

        wb = types.SimpleNamespace(lock=threading.Lock(), closed=False,
            generation=42, control_owner=dict(id='a' * 32, principal='browser',
                                              generation=controller._generation),
            runtime=types.SimpleNamespace(control_context=lambda: dict(
                phase='ready', owns_connection=True, hardware=controller.status())),
            motion_action=action)
        monitor = ControlMonitor(clock=self.clock)
        return CommandRuntime(wb, monitor), monitor, calls, wb

    def test_running_requests_do_not_opt_into_voltage(self):
        original = self.transport.request
        requests = []

        def request(kind, payload=b''):
            requests.append((kind, payload))
            return original(kind, payload)

        self.transport.request = request
        self.refresh()
        self.assertEqual(requests, [(helpers['motion'].SOURCE_STATE, b'')])
        self.assertFalse(self.c.status()['servo_voltage_polling']['enabled'])

    def test_real_registry_renews_hold_beyond_existing_watchdog(self):
        self.c.start_mode('hold', expected_generation=self.c._generation)
        runtime, monitor, calls, _ = self.command_runtime()
        for _ in range(30):
            self.clock.advance(.1)
            self.refresh()
            runtime.tick()
            self.c._control_once()
        self.assertEqual(self.c._mode, 'hold')
        self.assertTrue(self.transport.armed)
        self.assertEqual(monitor.snapshot()['sources']['x5_runtime']['heartbeat_accepted'], 30)
        self.assertTrue(all(body['action'] == 'heartbeat' for body in calls))

    def test_explicit_stop_prevents_any_more_renewal_or_restart(self):
        self.c.start_mode('hold', expected_generation=self.c._generation)
        runtime, _, calls, _ = self.command_runtime()
        runtime.tick()
        self.c.stop()
        before = len(calls)
        runtime.tick()
        self.assertEqual(len(calls), before)
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)

    def test_missing_owner_and_stale_generation_cannot_renew(self):
        self.c.start_mode('hold', expected_generation=self.c._generation)
        runtime, _, calls, wb = self.command_runtime()
        wb.control_owner['generation'] -= 1
        runtime.tick()
        wb.control_owner = None
        runtime.tick()
        self.assertEqual(calls, [])
        self.clock.advance(1.01)
        self.refresh()
        self.c._control_once()
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)


names = [name for name in StanceHeartbeatTests.__dict__ if name.startswith('test_')]
result = unittest.TextTestRunner(verbosity=2).run(
    unittest.TestSuite(StanceHeartbeatTests(name) for name in names))
args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(), tests=result.testsRun,
                                      hardware_access=False), indent=2) + '\n')
raise SystemExit(not result.wasSuccessful())
