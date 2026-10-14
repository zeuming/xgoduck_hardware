"""Stop-policy regression on a staged package; fake UART/clock only.

The historical suite remains unchanged so it still checks its old baseline.
This suite replaces its deliberately retired running-fault expectation, then
exercises continued control and the independent STOP/session/freshness paths.
"""
import copy
import json
from pathlib import Path
import runpy
import sys
import threading
import types
import unittest

ROOT = Path(__file__).resolve().parents[2]
helpers = runpy.run_path(str(ROOT/'tools/source_parity/test_source_host.py'), run_name='helpers')
args, motion = helpers['args'], helpers['motion']
sys.path.insert(0, str(args.candidate/'service'))
from native_gamepad import GamepadControl, ControlMonitor, PRINCIPAL, HOLD_PRINCIPAL
from native_command_runtime import CommandRuntime


class StopPolicyTests(helpers['RuntimeTests']):
    def test_saved_range_and_fault_checks_remain(self):
        # New ARM retains the original full-scan gate, but a running policy
        # must not convert the same reply into a broadcast all-OFF.
        self.transport.fault = 1
        self.refresh()
        with self.assertRaises(motion.MotionError): self.c._motion_gate_locked()
        self.transport.fault = 0
        self.refresh()
        self.start_walk()
        with self.assertRaises(motion.MotionError): self.c._raw_targets_locked([999.]*15)
        self.transport.fault = 1
        self.transport.online &= ~1
        before = len(self.transport.requests)
        self.step()
        self.assertEqual(self.c._mode, 'walk')
        self.assertTrue(self.transport.armed)
        self.assertEqual(self.c.status()['non_stopping_feedback_warnings']['fault_ids'], [10])
        self.assertNotIn(motion.STOP, [k for k, _ in self.transport.requests[before:]])
        self.assertIn(motion.SOURCE_TARGET, [k for k, _ in self.transport.requests[before:]])

    def test_running_excursion_warns_and_outgoing_goals_still_clamp(self):
        self.start_walk()
        j = self.c._config['joints'][0]
        for p in (j['min']-1, j['max']+1):
            self.transport.positions[0] = p
            self.refresh()
            self.c._motion_gate_locked()
            self.assertEqual(self.c.status()['non_stopping_feedback_warnings']['outside_saved_range'],
                [dict(id=j['id'], position=p, min=j['min'], max=j['max'])])
            # Actual-position warning does not mutate feedback/history.
            self.assertEqual(self.c._state['positions'][0], p)
            self.step()
            self.assertEqual(self.c._mode, 'walk')
            self.assertTrue(self.transport.armed)
            for raw, config in zip(self.transport.positions, self.c._config['joints']):
                self.assertLessEqual(config['min'], raw)
                self.assertLessEqual(raw, config['max'])

    def test_startup_actual_excursion_warns_then_hold_and_walk_continue(self):
        # Screenshot case ID32, plus every leg/mouth/head joint at both ends.
        # Check the real start state machine, not just the gate in isolation.
        for index, j in enumerate(self.c._config['joints']):
            for p in (j['min']-1, j['max']+1):
                with self.subTest(id=j['id'], position=p):
                    self.transport.positions[index] = p
                    self.refresh()
                    warning = self.c.status()['non_stopping_feedback_warnings']['outside_saved_range']
                    self.assertEqual(warning, [dict(id=j['id'], position=p, min=j['min'], max=j['max'])])
                    self.c.start_mode('hold', expected_generation=self.c._generation)
                    self.assertEqual(self.c._state['positions'][index], p)
                    self.step()
                    before = len(self.transport.requests)
                    self.start_walk(); self.step()
                    self.assertEqual(self.c._mode, 'walk')
                    self.assertTrue(self.transport.armed)
                    self.assertNotIn(motion.STOP, [k for k, _ in self.transport.requests[before:]])
                    for raw, config in zip(self.transport.positions, self.c._config['joints']):
                        self.assertLessEqual(config['min'], raw)
                        self.assertLessEqual(raw, config['max'])
                    self.c.stop(); self.c._service_stop_locked(); self.refresh()

    def test_fault_never_suppresses_real_stop_or_invalid_session(self):
        self.start_walk()
        self.transport.fault = 1
        self.transport.cookie += 1
        self.c._cycle_once()
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)

    def test_direct_walk_start_outside_id32_saved_range(self):
        j = self.c._config['joints'][12]
        self.assertEqual(j['id'], 32)
        self.transport.positions[12] = j['max']+1
        self.refresh()
        before = len(self.transport.requests)
        self.start_walk(); self.step()
        self.assertEqual(self.c._mode, 'walk')
        self.assertTrue(self.transport.armed)
        self.assertNotIn(motion.STOP, [k for k, _ in self.transport.requests[before:]])

    def test_fault_never_suppresses_imu_loss(self):
        self.start_walk()
        self.transport.fault = 1
        self.transport.imu_ok = False
        self.c._cycle_once()
        self.assertEqual(self.c._mode, 'off')
        self.assertFalse(self.transport.armed)

    def pad_fixture(self, *, origin=False):
        self.start_walk()
        c = self.c
        wb = types.SimpleNamespace(lock=threading.RLock(), closed=False, generation=1,
            control_owner=dict(id='a'*32, principal=PRINCIPAL, generation=c._generation))
        wb.runtime = types.SimpleNamespace(control_context=lambda: dict(phase='ready',
            owns_connection=True, connection_sequence=1, hardware=c.status()))
        calls = []
        def action(body, *, principal, live):
            calls.append(dict(body))
            if body['action'] == 'heartbeat':
                return c.heartbeat(expected_generation=body['generation'])
            assert body['action'] == 'start' and body['mode'] == 'hold'
            result = c.start_mode('hold', expected_generation=body['generation'])
            wb.control_owner = dict(id=body['control_id'], principal=principal, generation=c._generation)
            return result
        wb.motion_action = action
        state = dict(connected=True, session='b'*32, sequence=0, axes=[0]*8, buttons=[0]*15, events=[])
        monitor = ControlMonitor(clock=self.clock)
        pad = GamepadControl(wb, reader=lambda: copy.deepcopy(state), clock=self.clock,
                             monitor=monitor, standalone=True, shared_buttons=True)
        pad.session = dict(id='a'*32, connection=1, input_session='b'*32)
        pad.origin = dict(id='c'*32, principal='browser', generation=c._generation) if origin else None
        pad.last_input = copy.deepcopy(state)
        pad.last_poll = self.clock()
        return wb, pad, state, CommandRuntime(wb, monitor), calls

    def test_input_loss_transfers_confirmed_hold_and_maintains_heartbeat(self):
        wb, pad, state, runtime, calls = self.pad_fixture()
        state.update(sequence=1, events=[dict(sequence=1, kind=1, number=3, value=0, age_ms=301)])
        pad.poll_once()
        self.assertEqual(self.c._mode, 'hold')
        self.assertIsNone(pad.session)
        self.assertEqual(wb.control_owner['principal'], HOLD_PRINCIPAL)
        for _ in range(15):
            self.clock.advance(.1); self.refresh()
            pad.heartbeat_once(); runtime.tick(); self.c._control_once()
        self.assertEqual(self.c._mode, 'hold')
        self.assertTrue(self.transport.armed)
        self.assertEqual(runtime.receipt['heartbeat_count'], 15)
        self.assertEqual([x['mode'] for x in calls if x['action']=='start'], ['hold'])

    def test_failed_hold_cannot_restore_browser_and_renew_old_walk(self):
        wb, pad, state, runtime, calls = self.pad_fixture(origin=True)
        original = wb.motion_action
        def fail_start(body, **kw):
            if body['action']=='start': raise ValueError('injected HOLD rejection')
            return original(body, **kw)
        wb.motion_action = fail_start
        with self.assertRaises(ValueError): pad._revoke('input loss', recover=True)
        runtime.tick()
        self.assertEqual(wb.control_owner['principal'], PRINCIPAL)
        self.assertEqual(runtime.receipt['heartbeat_count'], 0)
        self.assertEqual(self.c._mode, 'walk')  # rejection must not invent HOLD
        self.clock.advance(1.01); self.refresh(); self.c._control_once()
        self.assertFalse(self.transport.armed)

    def test_stop_after_recovered_hold_prevents_renewal_or_rearm(self):
        wb, pad, state, runtime, calls = self.pad_fixture()
        pad._revoke('input loss', recover=True)
        self.c.stop()
        before = runtime.receipt['heartbeat_count']
        runtime.tick(); self.c._control_once()
        self.assertEqual(runtime.receipt['heartbeat_count'], before)
        self.assertFalse(self.transport.armed)
        self.assertIsNone(pad.session)


suite = unittest.defaultTestLoader.loadTestsFromTestCase(StopPolicyTests)
clamp = runpy.run_path(str(ROOT/'tools/source_parity/test_target_range_clamp.py'), run_name='helpers')['TargetClampTests']
# All existing target saturation cases remain relevant; the retired measured-
# position stop assertion is covered by the paired entry/running tests above.
for name in vars(clamp):
    if name.startswith('test_') and name != 'test_actual_position_gate_still_rejects_physical_outside_range':
        suite.addTest(clamp(name))
result = unittest.TextTestRunner(verbosity=2).run(suite)
args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(), tests=result.testsRun,
    hardware_access=False, scope='actual staged controller, fake UART/clock; gamepad uses dispatch adapter'), indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
