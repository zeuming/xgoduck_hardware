"""Pure FaultDetails protocol/closure checks; hardware and I/O are mocked."""
import ast
import copy
import struct
import types
import unittest
from unittest import mock

import feedback_fault_probe as probe
import feedback_stream


def runtime():
    ns = {}
    exec(compile(probe.runtime_source(), '<fault-details-test-definitions>', 'exec'), ns)
    return ns


def bank(ns, *, attempted=32767, found=32767, seen=32767,
         latest=None, valid=None, outcomes=None, times=None):
    return ns['FAULT_BANK'].pack(attempted, found, seen,
        *(latest if latest is not None else [0] * 15),
        *(valid if valid is not None else [0] * 15),
        *(outcomes if outcomes is not None else [1] * 15),
        *(times if times is not None else [990] * 15))


def payload(ns, *, version=1, count=15, snapshot=1000, errors=0,
            online=32767, fault=0, torque=0, present_faults=0,
            torque_faults=0, armed=0, cookie=0, present=None, torque_bank=None):
    return (ns['FAULT_HEADER'].pack(version, count, snapshot, errors, online,
        fault, torque, present_faults, torque_faults, armed, cookie)
        + (present if present is not None else bank(ns))
        + (torque_bank if torque_bank is not None else bank(ns)))


class FaultProbeTests(unittest.TestCase):
    def setUp(self):
        self.ns = runtime()
        self.Error = self.ns['ProbeError']
        self.expected = {'generation': 23913, 'serial': {'path': '/dev/ttyUSB0'}}

    def test_exact_wire_layout_and_raw_device_errors(self):
        ns = self.ns
        self.assertEqual((ns['FAULT_HEADER'].size, ns['FAULT_BANK'].size), (25, 111))
        latest = [0x82] + [0] * 14
        b = bank(ns, latest=latest, valid=latest, outcomes=[2] + [1] * 14)
        value = ns['_decode_fault_details'](payload(ns, present=b, errors=0xffffffff,
            present_faults=1, torque_faults=2, fault=3))
        self.assertEqual(len(payload(ns)), 247)
        self.assertEqual(value['present_bank']['latest_attempt_error'][0], 0x82)
        self.assertEqual(value['servo_errors'], 0xffffffff)
        self.assertEqual(value['fault_mask'], 3)
        self.assertFalse(value['motion_authorized'])
        self.assertFalse(value['fault_cause_determined'])
        self.assertNotIn('positions', value)

    def test_all_outcomes_and_retained_old_error_are_accepted(self):
        ns = self.ns
        empty = bank(ns, attempted=0, found=0, seen=0, outcomes=[0] * 15, times=[0] * 15)
        self.assertEqual(ns['_decode_fault_details'](payload(ns, present=empty))['present_bank']['latest_outcome'], [0] * 15)
        for outcome in range(1, 8):
            latest = [0x80] + [0] * 14 if outcome == 2 else [0] * 15
            valid = latest if outcome in (1, 2, 7) else [0x40] + [0] * 14
            b = bank(ns, found=32767 if outcome in (1, 2, 7) else 0,
                latest=latest, valid=valid, outcomes=[outcome] * 15)
            # outcome 2 requires nonzero for each found slot.
            if outcome == 2:
                b = bank(ns, latest=[0x80] * 15, valid=[0x80] * 15, outcomes=[2] * 15)
            with self.subTest(outcome=outcome):
                value = ns['_decode_fault_details'](payload(ns, present=b))
                self.assertEqual(value['present_bank']['latest_outcome'][0], outcome)

    def test_timestamp_wrap_zero_seen_and_no_freshness_assumption(self):
        ns = self.ns
        b = bank(ns, times=[0, 0xfffffff0, 0x90000000] + [990] * 12)
        value = ns['_decode_fault_details'](payload(ns, snapshot=16, present=b))
        self.assertEqual(value['present_bank']['last_valid_age_us_mod32'][:3], [16, 32, 0x70000010])
        self.assertFalse(value['last_valid_age_is_freshness_proof'])
        b = bank(ns, attempted=32767, found=0, seen=0,
                 outcomes=[6] * 15, times=[0] * 15)
        self.assertEqual(ns['_decode_fault_details'](payload(ns, present=b))['present_bank']['last_valid_age_us_mod32'], [None] * 15)

    def test_header_fields_and_lengths_rejected(self):
        cases = ({'version': 2}, {'count': 14}, {'armed': 2}, {'online': 32768},
                 {'torque': 32768}, {'fault': 1}, {'present_faults': 32768})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(self.Error):
                self.ns['_decode_fault_details'](payload(self.ns, **changes))
        for body in (b'', payload(self.ns)[:-1], payload(self.ns) + b'\0'):
            with self.assertRaisesRegex(self.Error, 'length'):
                self.ns['_decode_fault_details'](body)

    def test_bank_masks_enum_and_history_invariants_rejected(self):
        ns = self.ns
        cases = ({'attempted': 32768}, {'found': 1, 'seen': 0},
            {'attempted': 0, 'found': 0, 'seen': 1},
            {'outcomes': [8] * 15}, {'outcomes': [0] * 15},
            {'found': 0}, {'outcomes': [2] * 15},
            {'latest': [1] * 15}, {'valid': [1] * 15},
            {'outcomes': [6] * 15},
            {'found': 0, 'outcomes': [6] * 15, 'latest': [1] * 15},
            {'attempted': 0, 'found': 0, 'seen': 0, 'outcomes': [0] * 15},
            {'found': 0, 'seen': 0, 'outcomes': [6] * 15, 'valid': [1] * 15})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(self.Error):
                ns['_decode_fault_details'](payload(ns, present=bank(ns, **changes)))

    def test_original_frame_stays_hello_state_only_and_diag_is_empty_separate(self):
        ns = self.ns
        for forbidden in (3, 4, 5, 6, 7):
            with self.assertRaises(self.Error):
                ns['_frame'](forbidden, 2)
        frame = ns['_fault_frame'](2)
        self.assertEqual(ns['HEADER'].unpack(frame[:-2]), (b'XD', 1, 7, 0, 2))
        self.assertEqual(struct.unpack('<H', frame[-2:])[0], ns['_crc16'](frame[:-2]))
        for seq in (0, -1, 0x100000000, True, '2'):
            with self.assertRaises(self.Error): ns['_fault_frame'](seq)

    def test_strict_exchange_header_crc_noise_and_timeout(self):
        ns = self.ns
        body = payload(ns)
        good = ns['HEADER'].pack(b'XD', 1, 0x87, 247, 2)
        sent = []
        ns['_write_once'] = lambda fd, frame, deadline, metadata: sent.append(frame)
        def exchange(header=good, crc=None, noise=None, failure=None):
            tail = body + (struct.pack('<H', ns['_crc16'](header + body)) if crc is None else crc)
            ns['_read_exact'] = mock.Mock(side_effect=failure if failure else [header, tail])
            metadata = {'request_types_attempted': [], 'reply_types_received': []}
            with mock.patch.object(ns['select'], 'select', side_effect=noise or [( [], [], []), ([], [], [])]):
                return ns['_fault_exchange'](91, 2, ns['time'].monotonic() + 1, metadata)
        self.assertEqual(exchange(), body)
        self.assertEqual(sent[-1], ns['_fault_frame'](2))
        for fields in ((b'YY', 1, 0x87, 247, 2), (b'XD', 2, 0x87, 247, 2),
                       (b'XD', 1, 0x82, 247, 2), (b'XD', 1, 0x87, 246, 2),
                       (b'XD', 1, 0x87, 247, 3)):
            with self.assertRaisesRegex(self.Error, 'header'):
                exchange(header=ns['HEADER'].pack(*fields))
        with self.assertRaisesRegex(self.Error, 'CRC'): exchange(crc=b'\0\0')
        before = len(sent)
        with self.assertRaisesRegex(self.Error, 'before'): exchange(noise=[([91], [], [])])
        self.assertEqual(len(sent), before)
        with self.assertRaisesRegex(self.Error, 'trailing'):
            exchange(noise=[([], [], []), ([91], [], [])])
        with self.assertRaisesRegex(self.Error, 'timed out'):
            exchange(failure=self.Error('Serial reply timed out'))

    def fake_run(self, *, changes=None, health_failure=None, holder_stage=None,
                 failure=None, end=False, control_failure=None, modem_changes=False,
                 parent_gone=False, open_failure=False, emit_failure=False, health_delay=0,
                 discovered_path=None, discovery_failure=None):
        ns = self.ns
        events, requests, opened, closed = [], [], [], []
        clock, health_count, holder_count, modem_count, parent_count = [0.], [0], [0], [0], [0]
        binding = mock.Mock(side_effect=self.Error(discovery_failure) if discovery_failure else None)
        class Control:
            def __init__(self, fd): pass
            def poll(self):
                if control_failure: raise ns['ProbeError'](control_failure)
                return end
        def health(expected, fd=None):
            self.assertEqual(binding.call_count, 1)
            health_count[0] += 1
            clock[0] += health_delay
            if health_count[0] == health_failure: raise self.Error('new SD/USB/kernel fault')
            return {'historical_faults_cleared': False}
        def holders(*args):
            holder_count[0] += 1
            return [{'pid': 34, 'fd': '8'}] if holder_count[0] == holder_stage else []
        def modem(fd):
            modem_count[0] += 1
            return {'dtr': False, 'rts': modem_changes and modem_count[0] > 1}
        def getparent():
            parent_count[0] += 1
            return 1 if parent_gone and parent_count[0] > 1 else 42
        def hello(fd, kind, sequence, deadline, metadata):
            requests.append((kind, sequence))
            metadata['request_types_attempted'].append(kind)
            metadata['reply_types_received'].append(0x81)
            self.assertLessEqual(deadline - clock[0], .20)
            return b'XGODUCK_MOTION_1'
        def diag(fd, sequence, deadline, metadata):
            requests.append((7, sequence))
            metadata['request_types_attempted'].append(7)
            if failure: raise self.Error(failure)
            metadata['reply_types_received'].append(0x87)
            return payload(ns, **(changes or {}))
        def emit(kind, expected, **values):
            if emit_failure and kind == 'diagnostic': raise BrokenPipeError('stdout stopped')
            events.append(copy.deepcopy({'kind': kind, 'generation': expected['generation'], **values}))
        def open_uart(*args):
            opened.append(args)
            if open_failure: raise OSError('UART unavailable')
            return 91
        ns.update(ControlChannel=Control, health_snapshot=health, _holders=holders,
                  bind_serial=binding, serial_path=lambda expected: discovered_path or expected['serial']['path'],
                  serial_proof=lambda *args: (types.SimpleNamespace(st_rdev=188), {}),
                  _modem_bits=modem, _raw_settings=lambda fd: {'hupcl': False},
                  _exchange=hello, _fault_exchange=diag, emit=emit)
        with mock.patch.object(ns['os'], 'getppid', side_effect=getparent), \
             mock.patch.object(ns['os'], 'open', side_effect=open_uart), \
             mock.patch.object(ns['os'], 'close', side_effect=lambda fd: closed.append(fd)), \
             mock.patch.object(ns['os'], 'set_blocking'), \
             mock.patch.object(ns['fcntl'], 'flock'), mock.patch.object(ns['fcntl'], 'ioctl'), \
             mock.patch.object(ns['signal'], 'signal'), \
             mock.patch.object(ns['time'], 'monotonic', side_effect=lambda: clock[0]):
            result = ns['run_fault_probe'](self.expected)
        return result, events, requests, opened, closed, health_count[0]

    def test_discovery_binds_before_health_and_opens_the_bound_serial_path(self):
        self.expected['discover_serial'] = True
        result, events, requests, opened, closed, _ = self.fake_run(discovered_path='/dev/ttyUSB2')
        self.assertEqual(result, 0)
        self.assertEqual(opened[0][0], '/dev/ttyUSB2')
        self.assertEqual(requests, [(1, 1), (7, 2)])
        self.assertEqual(closed, [91])
        self.assertTrue(events[-1]['serial_closed'])
        self.ns = runtime()
        result, events, requests, opened, closed, _ = self.fake_run(discovery_failure='CH340 identity ambiguous')
        self.assertEqual(result, 20)
        self.assertEqual((requests, opened, closed), ([], [], []))
        self.assertTrue(events[-1]['serial_closed'])

    def test_safe_single_open_hello_diag_health_close(self):
        result, events, requests, opened, closed, checks = self.fake_run()
        self.assertEqual(result, 0)
        self.assertEqual(requests, [(1, 1), (7, 2)])
        self.assertEqual(len(opened), 1)
        self.assertEqual(opened[0][0], '/dev/ttyUSB0')
        self.assertEqual(closed, [91])
        self.assertEqual(checks, 5)
        self.assertEqual([e['kind'] for e in events], ['ready', 'diagnostic', 'closed'])
        self.assertTrue(events[-1]['normal'])
        self.assertTrue(events[-1]['serial_closed'])
        self.assertFalse(events[-1]['servo_motion_authorized'])
        self.assertEqual(events[-1]['query']['state_records'], 0)

    def test_four_unsafe_flags_are_diagnostic_only_and_always_close(self):
        for changes in ({'armed': 1}, {'cookie': 12}, {'torque': 32767},
                        {'fault': 1, 'present_faults': 1}):
            with self.subTest(changes=changes):
                self.ns = runtime()
                result, events, requests, opened, closed, _ = self.fake_run(changes=changes)
                self.assertEqual((result, requests, len(opened), closed), (0, [(1, 1), (7, 2)], 1, [91]))
                self.assertEqual([e['kind'] for e in events], ['ready', 'diagnostic', 'closed'])
                value = events[1]['diagnostic']
                self.assertTrue(value['unsafe_flags_present'])
                self.assertFalse(value['motion_authorized'])
                self.assertFalse(value['fault_cause_determined'])
                self.assertEqual(events[-1]['query']['last_diagnostic'], value)

    def test_faults_no_retry_and_all_close_paths(self):
        cases = ({'failure': 'reply timed out'}, {'failure': 'CRC mismatch'},
            {'failure': 'reply header mismatch'}, {'changes': {'armed': 2}},
            {'health_failure': 3}, {'health_failure': 4}, {'health_failure': 5},
            {'holder_stage': 2}, {'modem_changes': True}, {'emit_failure': True},
            {'open_failure': True})
        for changes in cases:
            with self.subTest(changes=changes):
                self.ns = runtime()
                result, events, requests, opened, closed, _ = self.fake_run(**changes)
                self.assertEqual(result, 20)
                self.assertEqual(len(opened), 1)
                self.assertEqual(closed, [] if changes.get('open_failure') else [91])
                self.assertTrue(events[-1]['serial_closed'])
                self.assertFalse(events[-1]['normal'])
                self.assertFalse(any(e['kind'] == 'state' for e in events))
                self.assertLessEqual(len(requests), 2)
                self.assertTrue(all(kind in (1, 7) for kind, sequence in requests))

    def test_preopen_stops_and_control_parent_budget_guard(self):
        for changes in ({'health_failure': 1}, {'health_failure': 2}, {'holder_stage': 1},
                        {'control_failure': 'stdin EOF'}, {'parent_gone': True},
                        {'health_delay': 4}):
            with self.subTest(changes=changes):
                self.ns = runtime()
                result, events, requests, opened, closed, _ = self.fake_run(**changes)
                self.assertEqual((result, requests, opened, closed), (20, [], [], []))
                self.assertTrue(events[-1]['serial_closed'])
        self.ns = runtime()
        result, events, requests, opened, closed, _ = self.fake_run(end=True)
        self.assertEqual((result, requests, opened, closed), (0, [], [], []))
        self.assertEqual([e['kind'] for e in events], ['closed'])

    def test_postquery_health_failure_preserves_flags_without_diagnostic_success(self):
        result, events, requests, opened, closed, _ = self.fake_run(
            changes={'fault': 32767, 'torque_faults': 32767}, health_failure=5)
        self.assertEqual(result, 20)
        self.assertEqual([e['kind'] for e in events], ['ready', 'fault', 'closed'])
        self.assertEqual(events[1]['query']['last_diagnostic']['fault_mask'], 32767)
        self.assertEqual(closed, [91])

    def test_builder_retains_pinned_guards_and_replaces_only_invocation(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-reboot-reviewed-20261004-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.REBOOT_PROFILE_SHA256)
        expected = {key: copy.deepcopy(profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
        expected['generation'] = 3921
        original = feedback_stream.remote_source(expected, reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.REBOOT_PROFILE_SHA256)
        generated = probe.remote_fault_source(expected, reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.REBOOT_PROFILE_SHA256)
        old_tree, tree = ast.parse(original), ast.parse(generated)
        self.assertEqual(ast.dump(tree.body[-1], include_attributes=False),
            ast.dump(ast.parse('raise SystemExit(run_fault_probe(EXPECTED))').body[0], include_attributes=False))
        new_defs = {n.name: n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
        for node in old_tree.body:
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                self.assertEqual(ast.dump(node, include_attributes=False),
                    ast.dump(new_defs[node.name], include_attributes=False))
        compile(generated, '<generated-fault-probe>', 'exec')
        self.assertEqual(generated.count(repr(profile['kernel_prefix'])), 1)
        with self.assertRaises(ValueError):
            probe.remote_fault_source(expected, reviewed_profile_path=path,
                reviewed_profile_sha256='0' * 64)


if __name__ == '__main__':
    unittest.main()
