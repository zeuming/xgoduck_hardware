"""Pure fake-transport maintenance ownership/command tests; no hardware or HTTP."""
import copy
import struct
import threading
import time
import unittest

import servo_maintenance as maintenance


def calibration():
    return {'joints': [dict(id=sid, zero=2000 + i, min=1600, max=2400, sign=1,
                            direction_confirmed=True)
                       for i, sid in enumerate(maintenance.IDS)]}


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Transport:
    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.events = []
        self.cookie = None
        self.servos = {10: dict(position=2010, torque=0)}
        self.capability = maintenance.CAPABILITY
        self.voltage_capability = maintenance.VOLTAGE_CAPABILITY
        self.voltage_error = 0
        self.fail_kind = None
        self.bad_cookie = False
        self.closes = 0
        self.proof = dict(serial_closed=True, process_exited=True)
        self.center = None
        self.center_torque = 0
        self.center_delta = 0
        self.error = 0
        self.in_request = 0
        self.max_parallel = 0

    def request(self, kind, payload, reply, timeout):
        self.in_request += 1
        self.max_parallel = max(self.max_parallel, self.in_request)
        try:
            self.clock.now += .03
            self.calls.append((kind, payload, reply, timeout))
            self.events.append(('request', kind))
            if kind == self.fail_kind:
                raise TimeoutError('reply lost')
            if kind == maintenance.CAPS:
                return self.capability
            if kind == maintenance.VOLTAGE_CAPS:
                return self.voltage_capability
            if kind == maintenance.ENTER:
                self.cookie, = struct.unpack('<I', payload)
                return bytes((kind, 0))
            cookie = self.cookie + int(self.bad_cookie)
            if kind == maintenance.VOLTAGE_READ:
                request_cookie, sid = struct.unpack('<IB', payload)
                assert request_cookie == self.cookie
                row = self.servos.get(sid)
                outcome = 2 if self.voltage_error and row else 0 if row else 1
                return maintenance.VOLTAGE_ROW.pack(cookie, sid, outcome,
                    self.voltage_error if row else 0, row.get('voltage', 74) if outcome == 0 else 0)
            if kind == maintenance.READ:
                request_cookie, sid = struct.unpack('<IB', payload)
                assert request_cookie == self.cookie
                row = self.servos.get(sid)
                outcome = 2 if self.error and row else 0 if row else 1
                return maintenance.ROW.pack(cookie, sid, outcome, self.error if row else 0,
                                            row['position'] if row else 0xffff,
                                            row['torque'] if row else 0xff)
            if kind == maintenance.POLL:
                if self.center:
                    sid, target, low, high = self.center
                    return maintenance.ROW.pack(cookie, sid, 2, 0, target + self.center_delta,
                                                self.center_torque)
                return maintenance.ROW.pack(cookie, 255, 0, 0, 0xffff, 0xff)
            if kind == maintenance.RENAME:
                request_cookie, old, new = struct.unpack('<IBB', payload)
                assert request_cookie == self.cookie
                self.servos[new] = self.servos.pop(old)
            if kind == maintenance.CENTER:
                request_cookie, *self.center = struct.unpack('<IBHHH', payload)
                assert request_cookie == self.cookie
            if kind == maintenance.RELEASE:
                request_cookie, sid = struct.unpack('<IB', payload)
                self.servos[sid]['torque'] = 0
            return bytes((kind, 0))
        finally:
            self.in_request -= 1

    def close(self):
        self.closes += 1
        return copy.deepcopy(self.proof)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.transport = Transport(self.clock)
        self.factory_calls = 0
        self.blocked = None
        self.config = calibration()
        self.mapping = []
        self.faults = []

        def factory():
            self.factory_calls += 1
            return self.transport

        def changed(old, new):
            self.mapping.append((old, new))
            self.transport.events.append(('mapping', old, new))

        self.controller = maintenance.MaintenanceController(
            lock=threading.RLock(), transport_factory=factory, gate=lambda: self.blocked,
            calibration=lambda: self.config, identity_changed=changed,
            fault_callback=self.faults.append, clock=self.clock)

    def tearDown(self):
        self.transport.proof = dict(serial_closed=True, process_exited=True)
        self.controller.close()

    def test_review_requires_every_mapped_servo_and_closure_before_callback(self):
        self.blocked = 'maintenance fault held'
        self.transport.servos = {sid: dict(position=2048, torque=0) for sid in maintenance.IDS}
        reviewed = []
        self.controller._review_gate = lambda: None
        self.controller._reviewed_callback = lambda rows, proof: reviewed.append((rows, proof))
        self.controller.action({'action': 'review'})
        self.wait(lambda: bool(reviewed))
        rows, proof = reviewed[0]
        self.assertEqual([row['id'] for row in rows], list(maintenance.IDS))
        self.assertTrue(all(row['online'] and row['error'] == 0 and row['torque'] == 0 for row in rows))
        self.assertTrue(proof['serial_closed'] and proof['process_exited'])
        kinds = [call[0] for call in self.transport.calls]
        self.assertEqual(kinds[-1], maintenance.EXIT)
        self.assertFalse(set(kinds) & {maintenance.CENTER, maintenance.RENAME, maintenance.RELEASE})
        self.assertFalse(self.controller.owns_connection)

    def test_review_failure_or_unconfirmed_closure_never_calls_resolution(self):
        for failure in ('missing', 'torque', 'fault', 'closure'):
            with self.subTest(failure=failure):
                transport = Transport(Clock())
                transport.servos = {sid: dict(position=2048, torque=0) for sid in maintenance.IDS}
                if failure == 'missing': transport.servos.pop(31)
                if failure == 'torque': transport.servos[31]['torque'] = 1
                if failure == 'fault': transport.error = 1
                if failure == 'closure': transport.proof['serial_closed'] = False
                reviewed = []
                controller = maintenance.MaintenanceController(lock=threading.RLock(),
                    transport_factory=lambda: transport, gate=lambda: 'held', calibration=calibration,
                    identity_changed=lambda *args: None, review_gate=lambda: None,
                    reviewed_callback=lambda *args: reviewed.append(args), clock=transport.clock)
                controller.action({'action': 'review'})
                controller._thread.join(2)
                self.assertFalse(controller._thread.is_alive())
                self.assertEqual(reviewed, [])
                self.assertEqual(controller.owns_connection, failure == 'closure')
                self.assertFalse({call[0] for call in transport.calls} & {maintenance.CENTER, maintenance.RENAME})
                transport.proof['serial_closed'] = True
                controller.close()

    def test_review_gate_is_required_and_cannot_bypass_an_identity_hold(self):
        with self.assertRaisesRegex(ValueError, '未配置'): self.controller.action({'action': 'review'})
        self.controller._review_gate = lambda: '编号映射待核对'
        self.controller._reviewed_callback = lambda *args: self.fail('resolution forbidden')
        with self.assertRaisesRegex(ValueError, '编号'): self.controller.action({'action': 'review'})
        self.assertEqual(self.factory_calls, 0)

    def test_center_rejection_names_the_failed_step_and_retains_fault(self):
        self.controller.action({'action': 'enter'})
        self.wait(lambda: self.controller.snapshot()['connected'])
        original = self.transport.request
        def fail_center(kind, *args):
            if kind == maintenance.CENTER: raise ValueError('Maintenance request rejected: ACK error 18')
            return original(kind, *args)
        self.transport.request = fail_center
        self.controller.action({'action': 'center', 'id': 10, 'target': 2020, 'support_confirmed': True})
        self.wait(lambda: bool(self.controller.snapshot()['error']))
        self.assertIn('保持目标位置读回', self.controller.snapshot()['error'])
        self.assertIn('步骤码 18', self.controller.snapshot()['error'])
        self.assertTrue(self.faults)

    def test_center_evidence_is_explained_without_retry_or_clearing_fault(self):
        from maintenance_diagnostics import decode_center_failure
        from test_maintenance_diagnostics import failure_ack
        self.enter()
        self.transport.center_diagnostic = decode_center_failure(
            failure_ack(found=True, outcome=1, actual=1, sid=10), 10)
        original = self.transport.request
        def fail_center(kind, *args):
            if kind == maintenance.CENTER:
                self.transport.calls.append((kind, *args))
                raise ValueError('Maintenance request rejected: ACK error 19')
            return original(kind, *args)
        self.transport.request = fail_center
        result = self.run_action('center', id=10, target=2020, support_confirmed=True)
        self.assertIn('值 1，要求 0', result['error'])
        self.assertIn('原始应答已保存', result['error'])
        self.assertEqual(self.kinds().count(maintenance.CENTER), 1)
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(self.controller.owns_connection)

    def wait(self, predicate, seconds=2):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                self.fail('fake worker did not finish: ' + repr(self.controller.snapshot()))
            time.sleep(.002)

    def enter(self):
        self.controller.action({'action': 'enter'})
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertTrue(self.controller.snapshot()['connected'])

    def run_action(self, action, **data):
        self.controller.action(dict(action=action, **data))
        self.wait(lambda: not self.controller.snapshot()['busy'])
        return self.controller.snapshot()

    def kinds(self):
        return [call[0] for call in self.transport.calls]

    def test_voltage_batch_connects_once_reads_fixed_ids_and_preserves_unknowns(self):
        original = copy.deepcopy(self.config)
        result = self.run_action('voltages')
        self.assertTrue(result['voltage_supported'])
        rows = result['voltage_results']
        self.assertEqual([row['id'] for row in rows], list(maintenance.IDS))
        self.assertEqual((rows[0]['raw'], rows[0]['voltage_v']), (74, 7.4))
        self.assertTrue(all(row['age_ms'] >= 0 for row in rows))
        self.assertTrue(all(row['voltage_v'] is None and row['raw'] is None for row in rows[1:]))
        self.assertEqual(self.kinds().count(maintenance.ENTER), 1)
        self.assertEqual(self.kinds().count(maintenance.VOLTAGE_READ), 15)
        self.assertFalse(set(self.kinds()) & {maintenance.CENTER, maintenance.RENAME, maintenance.RELEASE})
        self.assertEqual(self.config, original)
        self.assertEqual(self.faults, [])
        self.clock.now += 30
        self.assertGreater(self.controller.snapshot()['voltage_results'][0]['age_ms'], 30000)

    def test_voltage_unsupported_does_not_break_legacy_maintenance_or_read_register(self):
        self.transport.voltage_capability = bytes((maintenance.VOLTAGE_CAPS, 1))
        result = self.run_action('voltages')
        self.assertIs(result['voltage_supported'], False)
        self.assertTrue(result['connected'])
        self.assertEqual(result['voltage_results'], [])
        self.assertNotIn(maintenance.VOLTAGE_READ, self.kinds())
        self.assertIn('需要更新固件', result['message'])
        self.assertEqual(self.faults, [])
        self.assertTrue(self.run_action('read', id=10)['servo']['online'])

    def test_voltage_error_retains_failure_record_and_never_fakes_zero(self):
        self.transport.voltage_error = 1
        result = self.run_action('voltages')
        row = result['voltage_results'][0]
        self.assertEqual((row['state'], row['error'], row['raw'], row['voltage_v']), (2, 1, None, None))
        self.assertIn('电压应答异常', result['error'])
        self.assertEqual(self.kinds().count(maintenance.VOLTAGE_READ), 1)
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(self.controller.owns_connection)

    def test_voltage_requests_cannot_bypass_existing_gate_or_accept_bad_identity(self):
        self.blocked = '运控正在使用连接'
        with self.assertRaisesRegex(maintenance.MaintenanceError, '运控正在'):
            self.controller.action({'action': 'voltages'})
        self.assertEqual(self.factory_calls, 0)
        self.blocked = None
        self.enter()
        self.transport.bad_cookie = True
        result = self.run_action('voltages')
        self.assertEqual(result['voltage_results'], [])
        self.assertIn('会话、编号', result['error'])

    def test_constructor_snapshot_do_not_construct_transport(self):
        self.assertFalse(self.controller.owns_connection)
        self.assertEqual(self.factory_calls, 0)
        self.assertIsNone(self.controller.snapshot()['compatible'])
        self.assertFalse(self.transport.calls)

    def test_gate_rejects_before_factory_or_capability_request(self):
        self.blocked = 'voltage unsafe hold'
        with self.assertRaisesRegex(ValueError, 'voltage'):
            self.controller.action({'action': 'enter'})
        self.assertEqual(self.factory_calls, 0)
        self.assertFalse(self.transport.calls)
        self.assertFalse(self.controller.owns_connection)

    def test_unsupported_capability_never_enters_or_sends_write(self):
        self.transport.capability = b'old firmware'
        self.run_action('enter')
        self.assertEqual(self.kinds(), [maintenance.CAPS])
        self.assertFalse(self.controller.snapshot()['compatible'])
        self.assertFalse(self.controller.owns_connection)
        self.assertFalse(self.faults)

    def test_explicit_unsupported_caps_ack_is_distinct_from_network_error(self):
        original = self.transport.request

        def unsupported_caps(kind, payload, reply, timeout):
            original(kind, payload, reply, timeout)
            raise ValueError('Maintenance unsupported by connected firmware: ACK error 1')

        self.transport.request = unsupported_caps
        result = self.run_action('enter')
        self.assertIs(result['compatible'], False)
        self.assertIn('更新固件', result['error'])
        self.assertEqual(self.kinds(), [maintenance.CAPS])
        self.assertFalse(self.faults)
        self.transport.request = original
        self.transport.fail_kind = maintenance.CAPS
        result = self.run_action('enter')
        self.assertIsNone(result['compatible'])
        self.assertEqual(result['error'], 'reply lost')

    def test_cookie_mismatch_closes_and_marks_hardware_fault(self):
        self.transport.bad_cookie = True
        self.run_action('enter')
        self.assertIn('会话', self.controller.snapshot()['error'])
        self.assertFalse(self.controller.owns_connection)
        self.assertEqual(len(self.faults), 1)

    def test_enter_requires_new_nonce_and_accepts_idle_sentinels(self):
        self.enter()
        self.assertGreater(self.transport.cookie, 0)
        self.assertEqual(self.kinds()[:3], [maintenance.CAPS, maintenance.ENTER, maintenance.POLL])
        self.assertIsNone(self.controller.snapshot()['servo'])
        with self.assertRaises(ValueError):
            self.controller.action({'action': 'enter'})

    def test_read_arbitrary_id_and_no_reply_are_current_results(self):
        self.transport.servos[123] = dict(position=3210, torque=0)
        self.enter()
        result = self.run_action('read', id=123)
        self.assertEqual(result['servo']['position'], 3210)
        self.assertEqual(result['servo']['id'], 123)
        self.assertEqual(result['received_age_ms'], 0)
        result = self.run_action('read', id=124)
        self.assertFalse(result['servo']['online'])
        self.assertIsNone(result['servo']['position'])
        self.assertIsNone(result['servo']['torque'])
        self.assertTrue(result['connected'])

    def test_scan_full_id_range_with_serial_heartbeats(self):
        self.transport.servos[253] = dict(position=123, torque=0)
        self.enter()
        start = len(self.transport.calls)
        result = self.run_action('scan')
        reads = [struct.unpack('<IB', payload)[1] for kind, payload, _, _
                 in self.transport.calls[start:] if kind == maintenance.READ]
        self.assertEqual(reads, list(range(254)))
        self.assertEqual([r['id'] for r in result['results']], [10, 253])
        kinds = self.kinds()[start:]
        self.assertIn(maintenance.POLL, kinds[1:-1])
        self.assertEqual(self.transport.max_parallel, 1)
        self.assertEqual(result['progress'], 100)

    def test_closed_one_click_scan_enters_once_then_scans_with_one_owner(self):
        result = self.run_action('scan')
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(self.kinds().count(maintenance.CAPS), 1)
        self.assertEqual(self.kinds().count(maintenance.ENTER), 1)
        self.assertEqual(self.kinds().count(maintenance.READ), 254)
        self.assertLess(self.kinds().index(maintenance.ENTER), self.kinds().index(maintenance.READ))
        self.assertEqual([row['id'] for row in result['results']], [10])
        self.assertTrue(result['connected'])
        self.assertTrue(result['owns_connection'])
        self.run_action('scan')
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(self.kinds().count(maintenance.ENTER), 1)

    def test_closed_scan_obeys_fault_gate_before_factory(self):
        self.blocked = 'unsafe voltage hold'
        with self.assertRaisesRegex(ValueError, 'unsafe'):
            self.controller.action({'action': 'scan'})
        self.assertEqual(self.factory_calls, 0)
        self.assertFalse(self.transport.calls)

    def test_exit_during_one_click_scan_capability_check_prevents_scan(self):
        original = self.transport.request
        ready, proceed = threading.Event(), threading.Event()

        def held_caps(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.CAPS:
                ready.set()
                proceed.wait(1.)
            return result

        self.transport.request = held_caps
        self.controller.action({'action': 'scan'})
        self.assertTrue(ready.wait(1.))
        self.controller.action({'action': 'exit'})
        proceed.set()
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertEqual(self.kinds(), [maintenance.CAPS])
        self.assertFalse(self.controller.owns_connection)

    def test_exit_during_one_click_scan_enter_reply_prevents_scan(self):
        original = self.transport.request
        ready, proceed = threading.Event(), threading.Event()

        def held_enter(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.ENTER:
                ready.set()
                proceed.wait(1.)
            return result

        self.transport.request = held_enter
        self.controller.action({'action': 'scan'})
        self.assertTrue(ready.wait(1.))
        self.controller.action({'action': 'exit'})
        proceed.set()
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertNotIn(maintenance.READ, self.kinds())
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)
        self.assertFalse(self.controller.owns_connection)

    def test_scan_cannot_restart_an_unproven_closing_owner(self):
        self.enter()
        self.transport.proof = dict(serial_closed=None, process_exited=True)
        self.run_action('exit')
        with self.assertRaises(ValueError):
            self.controller.action({'action': 'scan'})
        self.assertEqual(self.factory_calls, 1)

    def test_rename_requires_confirmations_and_single_servo(self):
        self.enter()
        with self.assertRaises(ValueError):
            self.controller.action(dict(action='rename', id=10, new_id=20,
                                        isolated_confirmed=False, mapping_acknowledged=True))
        self.transport.servos[11] = dict(position=2000, torque=0)
        self.run_action('rename', id=10, new_id=20, isolated_confirmed=True, mapping_acknowledged=True)
        self.assertNotIn(maintenance.RENAME, self.kinds())
        self.assertFalse(self.mapping)

    def test_rename_marks_mapping_before_single_write_and_verifies_new_id(self):
        self.enter()
        result = self.run_action('rename', id=10, new_id=20,
                                 isolated_confirmed=True, mapping_acknowledged=True)
        self.assertEqual(self.mapping, [(10, 20)])
        self.assertEqual(self.kinds().count(maintenance.RENAME), 1)
        self.assertLess(self.transport.events.index(('mapping', 10, 20)),
                        self.transport.events.index(('request', maintenance.RENAME)))
        self.assertTrue(result['identity_changed'])
        self.assertEqual(result['servo']['id'], 20)
        self.assertEqual(self.config, calibration())
        with self.assertRaisesRegex(ValueError, '映射'):
            self.controller.action(dict(action='center', id=20, support_confirmed=True))
        self.assertEqual(self.run_action('read', id=20)['servo']['id'], 20)

    def test_lost_rename_reply_does_not_retry_or_restore_mapping(self):
        self.enter()
        self.transport.fail_kind = maintenance.RENAME
        result = self.run_action('rename', id=10, new_id=20,
                                 isolated_confirmed=True, mapping_acknowledged=True)
        self.assertEqual(self.kinds().count(maintenance.RENAME), 1)
        self.assertTrue(result['identity_changed'])
        self.assertEqual(self.mapping, [(10, 20)])
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(self.controller.owns_connection)

    def test_center_defaults_to_saved_zero_for_one_joint(self):
        self.config['joints'][0]['zero'] = 2088
        self.enter()
        result = self.run_action('center', id=10, support_confirmed=True)
        packet, = [payload for kind, payload, _, _ in self.transport.calls if kind == maintenance.CENTER]
        self.assertEqual(struct.unpack('<IBHHH', packet), (self.transport.cookie, 10, 2088, 1600, 2400))
        self.assertEqual(result['servo']['position'], 2088)
        self.assertEqual(result['servo']['torque'], 0)
        self.assertNotIn(4, self.kinds())  # No all-joint ARM or TARGET.
        self.assertNotIn(5, self.kinds())
        with self.assertRaises(ValueError):
            self.controller.action(dict(action='center', id=10, minimum=0, support_confirmed=True))

    def test_custom_center_target_uses_saved_bounds_without_changing_calibration(self):
        self.enter()
        before = copy.deepcopy(self.config)
        result = self.run_action('center', id=10, target=2188, support_confirmed=True)
        packet, = [payload for kind, payload, _, _ in self.transport.calls if kind == maintenance.CENTER]
        self.assertEqual(struct.unpack('<IBHHH', packet), (self.transport.cookie, 10, 2188, 1600, 2400))
        self.assertEqual(result['servo']['position'], 2188)
        self.assertIn('2188', result['message'])
        self.assertEqual(self.config, before)

    def test_custom_center_target_rejects_invalid_type_and_outside_saved_range(self):
        self.enter()
        before = copy.deepcopy(self.config)
        for target in (True, 2000., '2000', None, -1, 4096, 1599, 2401):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.controller.action(dict(action='center', id=10, target=target, support_confirmed=True))
        self.assertNotIn(maintenance.CENTER, self.kinds())
        self.assertEqual(self.config, before)

    def test_custom_center_checks_completion_against_requested_target(self):
        self.enter()
        # Fake feedback lands at old zero 2000, far from the requested 2188.
        self.transport.center_delta = -188
        result = self.run_action('center', id=10, target=2188, support_confirmed=True)
        self.assertIn('终点', result['error'])
        self.assertEqual(len(self.faults), 1)
        self.assertEqual(self.kinds().count(maintenance.CENTER), 1)
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)

    def test_custom_center_target_keeps_calibration_and_current_position_gates(self):
        self.enter()
        self.config['joints'][0]['direction_confirmed'] = False
        with self.assertRaisesRegex(ValueError, '方向'):
            self.controller.action(dict(action='center', id=10, target=2188, support_confirmed=True))
        self.config['joints'][0]['direction_confirmed'] = True
        self.transport.servos[10]['position'] = 3000
        result = self.run_action('center', id=10, target=2188, support_confirmed=True)
        self.assertIn('当前位置', result['error'])
        self.assertNotIn(maintenance.CENTER, self.kinds())

    def test_center_rejects_unconfirmed_or_invalid_default_stance(self):
        self.enter()
        self.config['joints'][0]['direction_confirmed'] = False
        with self.assertRaisesRegex(ValueError, '方向'):
            self.controller.action(dict(action='center', id=10, support_confirmed=True))
        self.config['joints'][2].update(min=1950, max=2050)
        with self.assertRaisesRegex(ValueError, '默认站姿'):
            self.controller.action(dict(action='center', id=12, support_confirmed=True))
        self.assertNotIn(maintenance.CENTER, self.kinds())

    def test_center_rejects_bad_final_torque_and_closes(self):
        self.enter()
        self.transport.center_torque = 1
        result = self.run_action('center', id=10, support_confirmed=True)
        self.assertIn('释放扭矩', result['error'])
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(self.controller.owns_connection)
        self.assertEqual(self.kinds().count(maintenance.CENTER), 1)
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)

    def test_voltage_fault_closes_and_reports_once(self):
        self.enter()
        self.transport.error = 1
        result = self.run_action('read', id=10)
        self.assertIn('0x01', result['error'])
        self.assertIsNone(result['servo']['position'])
        self.assertEqual(len(self.faults), 1)

    def test_missing_close_proof_keeps_owner_until_actual_close(self):
        self.enter()
        self.transport.proof = dict(serial_closed=True, process_exited=False)
        self.run_action('exit')
        self.assertTrue(self.controller.owns_connection)
        self.assertEqual(self.controller.snapshot()['phase'], 'closing')
        with self.assertRaises(ValueError):
            self.controller.action({'action': 'enter'})
        self.transport.proof = dict(serial_closed=True, process_exited=True)
        self.run_action('exit')
        self.assertFalse(self.controller.owns_connection)

    def test_exit_allowed_after_external_gate_blocks(self):
        self.enter()
        self.blocked = 'new safety fault'
        self.run_action('exit')
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)
        self.assertFalse(self.controller.owns_connection)

    def test_partial_factory_error_preserves_unknown_close_owner(self):
        self.transport.proof = dict(serial_closed=None, process_exited=True)

        def partial_factory():
            error = ValueError('SSH setup failed after launch')
            error.transport = self.transport
            error.close_report = self.transport.proof
            raise error

        self.controller._factory = partial_factory
        result = self.run_action('enter')
        self.assertTrue(result['owns_connection'])
        self.assertIsNone(result['close_proof']['serial_closed'])
        self.assertFalse(self.transport.calls)
        self.assertGreater(self.transport.closes, 0)

    def test_factory_exception_without_proof_never_assumes_not_opened(self):
        def unproven_factory():
            raise OSError('unknown creation outcome')

        self.controller._factory = unproven_factory
        result = self.run_action('enter')
        self.assertTrue(result['owns_connection'])
        self.assertIsNone(result['close_proof']['serial_closed'])
        self.assertFalse(result['close_proof']['process_exited'])
        # No surviving real connection exists in this fake; attach its proven
        # fake close object so tearDown can finish without weakening the owner.
        self.controller._transport = self.transport

    def test_failed_mapping_persistence_prevents_rename_write(self):
        self.enter()

        def failed_record(old, new):
            raise OSError('disk write failed')

        self.controller._identity_changed = failed_record
        result = self.run_action('rename', id=10, new_id=20,
                                 isolated_confirmed=True, mapping_acknowledged=True)
        self.assertTrue(result['identity_changed'])
        self.assertNotIn(maintenance.RENAME, self.kinds())
        self.assertEqual(self.config, calibration())

    def test_exit_during_capability_check_does_not_enter(self):
        original_request = self.transport.request
        ready, proceed = threading.Event(), threading.Event()

        def held_request(kind, payload, reply, timeout):
            result = original_request(kind, payload, reply, timeout)
            if kind == maintenance.CAPS:
                ready.set()
                proceed.wait(1.)
            return result

        self.transport.request = held_request
        self.controller.action({'action': 'enter'})
        self.assertTrue(ready.wait(1.))
        self.controller.action({'action': 'exit'})
        proceed.set()
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertNotIn(maintenance.ENTER, self.kinds())
        self.assertFalse(self.controller.owns_connection)

    def test_selected_torque_enabled_blocks_center_and_records_fault(self):
        self.enter()
        self.transport.servos[10]['torque'] = 1
        result = self.run_action('center', id=10, support_confirmed=True)
        self.assertIn('扭矩', result['error'])
        self.assertNotIn(maintenance.CENTER, self.kinds())
        self.assertEqual(len(self.faults), 1)

    def test_exit_during_center_preflight_does_not_submit_motion(self):
        self.enter()
        ready, proceed = threading.Event(), threading.Event()
        original = self.transport.request

        def delayed_read(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.READ:
                ready.set()
                proceed.wait(1.)
            return result

        self.transport.request = delayed_read
        self.controller.action(dict(action='center', id=10, support_confirmed=True))
        self.assertTrue(ready.wait(1.))
        self.controller.action({'action': 'exit'})
        proceed.set()
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertNotIn(maintenance.CENTER, self.kinds())
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)

    def test_exit_during_final_rename_preflight_does_not_write_or_invalidate(self):
        self.enter()
        ready, proceed = threading.Event(), threading.Event()
        original = self.transport.request
        selected_reads = []

        def delayed_read(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.READ and struct.unpack('<IB', payload)[1] == 10:
                selected_reads.append(10)
                if len(selected_reads) == 3:  # initial, scan, final preflight
                    ready.set()
                    proceed.wait(1.)
            return result

        self.transport.request = delayed_read
        self.controller.action(dict(action='rename', id=10, new_id=20,
                                    isolated_confirmed=True, mapping_acknowledged=True))
        self.assertTrue(ready.wait(1.))
        self.controller.action({'action': 'exit'})
        proceed.set()
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertNotIn(maintenance.RENAME, self.kinds())
        self.assertFalse(self.mapping)
        self.assertFalse(self.controller.snapshot()['identity_changed'])

    def test_exit_does_not_wait_for_network_after_center_submission(self):
        self.enter()
        submitted, proceed, exited = threading.Event(), threading.Event(), threading.Event()
        original = self.transport.request

        def delayed_center(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.CENTER:
                submitted.set()
                proceed.wait(1.)
            return result

        self.transport.request = delayed_center
        self.controller.action(dict(action='center', id=10, support_confirmed=True))
        self.assertTrue(submitted.wait(1.))

        def exit_now():
            self.controller.action({'action': 'exit'})
            exited.set()

        request = threading.Thread(target=exit_now)
        request.start()
        try:
            self.assertTrue(exited.wait(.2), 'exit was blocked by network request')
        finally:
            proceed.set()
            request.join(timeout=1.)
        self.wait(lambda: not self.controller.snapshot()['busy'])
        self.assertEqual(self.kinds().count(maintenance.CENTER), 1)
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)

    def test_failed_exit_is_not_retried_and_original_fault_is_preserved(self):
        self.enter()
        self.transport.center_torque = 1
        self.transport.fail_kind = maintenance.EXIT
        result = self.run_action('center', id=10, support_confirmed=True)
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)
        self.assertIn('释放扭矩', result['error'])
        self.assertIn('退出维护未确认', result['message'])
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(result['owns_connection'])

    def test_closed_transport_fault_does_not_send_exit_or_reopen(self):
        self.enter()
        original = self.transport.request

        def closed_read(kind, payload, reply, timeout):
            result = original(kind, payload, reply, timeout)
            if kind == maintenance.READ:
                self.transport.close_report = self.transport.proof
                raise TimeoutError('transport already closed after wire timeout')
            return result

        self.transport.request = closed_read
        result = self.run_action('read', id=10)
        self.assertNotIn(maintenance.EXIT, self.kinds())
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(result['owns_connection'])

    def test_normal_exit_lost_reply_is_attempted_once(self):
        self.enter()
        self.transport.fail_kind = maintenance.EXIT
        result = self.run_action('exit')
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)
        self.assertEqual(len(self.faults), 1)
        self.assertFalse(result['owns_connection'])

    def test_exit_fault_after_local_range_refusal_still_records_hardware_hold(self):
        self.enter()
        self.transport.servos[10]['position'] = 3000
        self.transport.fail_kind = maintenance.EXIT
        result = self.run_action('center', id=10, support_confirmed=True)
        self.assertNotIn(maintenance.CENTER, self.kinds())
        self.assertIn('安全范围', result['error'])
        self.assertEqual(self.faults, ['reply lost'])
        self.assertEqual(self.kinds().count(maintenance.EXIT), 1)


if __name__ == '__main__':
    unittest.main()
