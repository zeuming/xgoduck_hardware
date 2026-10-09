"""Pure local maintenance transport tests; no SSH, HTTP, serial, or device use."""
import ast
import base64
import copy
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest import mock

import maintenance_transport as transport
from transport_config import TransportConfig


PROFILE = transport.reader.EVIDENCE_DIR / 'feedback-returned-esp32-reviewed-20261004-01.json'
PROFILE_SHA = transport.reader.RETURNED_ESP32_PROFILE_SHA256


def fixture():
    profile = transport.reader.load_reviewed_profile(PROFILE, PROFILE_SHA)
    expected = {key: copy.deepcopy(profile[key]) for key in
                ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
    expected['generation'] = 42
    return transport.prepare_source(expected, PROFILE, PROFILE_SHA)


def ready(expected):
    return {'kind': 'ready', 'scope': 'maintenance', 'generation': expected['generation'],
            'allowed_requests': list(transport.ALLOWED_REPLIES),
            'identity': dict(expected, kernel_log=expected['kernel_prefix']),
            'query': {'serial_open_attempts': 1, 'exclusive': True, 'automatic_retry': False,
                      'board_files_written_by_transport': False}}


def closed(generation=42, serial=True):
    return {'kind': 'closed', 'generation': generation, 'serial_closed': serial, 'normal': True,
            'query': {'serial_open_attempts': 1, 'automatic_retry': False,
                      'board_files_written_by_transport': False}}


def response(kind=0x88, payload=b'XGODUCK_MAINT_1', sequence=1, generation=42):
    return {'kind': 'response', 'generation': generation, 'sequence': sequence,
            'type': kind, 'payload': base64.b64encode(payload).decode()}


class FakeChannel:
    def __init__(self, records=(), *, auto_close=True, exited=True):
        self.records = list(records)
        self.sent = []
        self.auto_close = auto_close
        self.exited = exited
        self.finished = 0
        self.observed = 0
    def send(self, value, timeout):
        self.sent.append(copy.deepcopy(value))
        if value.get('kind') == 'close' and self.auto_close:
            self.records.append(closed(value['generation']))
    def receive(self, deadline):
        if not self.records: raise TimeoutError('fake bounded timeout')
        value = self.records.pop(0)
        if isinstance(value, BaseException): raise value
        return value
    def finish(self):
        self.finished += 1
        return {'process_exited': self.exited, 'ssh_exit_code': 0 if self.exited else None, 'stderr': ''}
    def observe_exit(self):
        self.observed += 1
        return {'process_exited': self.exited, 'ssh_exit_code': 0 if self.exited else None, 'stderr': ''}


class TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected, cls.source = fixture()
        cls.namespace = {}
        exec(compile(transport._identity_runtime(), '<test-runtime>', 'exec'), cls.namespace)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name).resolve()
        self.directory.chmod(0o700)

    def tearDown(self):
        self.temp.cleanup()

    def connect(self, records=(), **options):
        channel = FakeChannel([ready(self.expected), *records], **options)
        instance = transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
            Path('/unused-known-hosts'), {}, channel_factory=lambda *args: channel)
        return instance, channel

    def test_source_has_no_feedback_loop_or_nonmaintenance_frame_builder(self):
        tree = ast.parse(self.source)
        functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
        self.assertNotIn('run_reader', functions)
        self.assertNotIn('_exchange', functions)
        self.assertNotIn('_frame', functions)
        self.assertIn('health_snapshot', functions)
        self.assertIn('run_maintenance', functions)
        self.assertLess(len(self.source), 262144)

    def test_caps_and_cookie_request_sequences_and_real_close_proof(self):
        instance, channel = self.connect([response(), response(128, b'\x09\0', 2)])
        self.assertEqual(instance.request(8, b'', 136, .3), b'XGODUCK_MAINT_1')
        self.assertEqual(instance.request(9, struct.pack('<I', 123), 128, .3), b'\x09\0')
        proof = instance.close()
        self.assertTrue(proof['serial_closed'])
        self.assertTrue(proof['process_exited'])
        self.assertEqual([x['sequence'] for x in channel.sent if 'sequence' in x], [1, 2])
        self.assertEqual(channel.sent[-1], {'generation': 42, 'kind': 'close'})
        self.assertEqual(instance.close(), proof)
        self.assertEqual(channel.finished, 1)

    def test_exited_child_does_not_prove_serial_closed(self):
        instance, _ = self.connect(auto_close=False)
        proof = instance.close()
        self.assertTrue(proof['process_exited'])
        self.assertIsNone(proof['serial_closed'])

    def test_serial_closed_does_not_prove_child_exit(self):
        instance, _ = self.connect(exited=False)
        proof = instance.close()
        self.assertTrue(proof['serial_closed'])
        self.assertFalse(proof['process_exited'])

    def test_explicit_close_can_observe_late_child_exit_without_remote_close_replay(self):
        instance, channel = self.connect(exited=False)
        first = instance.close()
        self.assertTrue(first['serial_closed'])
        self.assertFalse(first['process_exited'])
        self.assertEqual(len(channel.sent), 1)
        channel.exited = True
        latest = instance.close()
        self.assertTrue(latest['serial_closed'])
        self.assertTrue(latest['process_exited'])
        self.assertEqual(latest['ssh_exit_code'], 0)
        self.assertEqual(channel.finished, 1)
        self.assertEqual(channel.observed, 1)
        self.assertEqual(len(channel.sent), 1)
        self.assertEqual(json.loads((self.directory/'report.json').read_text()), latest)
        self.assertEqual(instance.close(), latest)
        self.assertEqual(channel.observed, 1)

    def test_late_child_exit_never_invents_missing_serial_proof(self):
        instance, channel = self.connect(exited=False, auto_close=False)
        first = instance.close()
        self.assertIsNone(first['serial_closed'])
        self.assertFalse(first['process_exited'])
        channel.exited = True
        latest = instance.close()
        self.assertTrue(latest['process_exited'])
        self.assertIsNone(latest['serial_closed'])
        self.assertIsNone(latest['remote_closed_report'])
        self.assertEqual(channel.finished, 1)
        self.assertEqual(len(channel.sent), 1)

    def test_caps_unsupported_ack_closes_and_is_not_retried(self):
        instance, channel = self.connect([response(128, b'\x08\x05')])
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            instance.request(8, b'', 136, .3)
        self.assertEqual(sum('sequence' in value for value in channel.sent), 1)
        self.assertTrue(instance.close()['serial_closed'])
        with self.assertRaisesRegex(ValueError, 'no automatic reopen'):
            instance.request(8, b'', 136, .3)

    def test_timeout_after_rename_never_resends_write(self):
        instance, channel = self.connect([TimeoutError('response lost')])
        with self.assertRaisesRegex(TimeoutError, 'lost'):
            instance.request(11, struct.pack('<IBB', 123, 1, 10), 128, 1.8)
        self.assertEqual([value['type'] for value in channel.sent if 'type' in value], [11])
        self.assertTrue(instance.close()['serial_closed'])

    def test_wrong_sequence_closes(self):
        instance, _ = self.connect([response(sequence=2)])
        with self.assertRaisesRegex(ValueError, 'Mismatched'):
            instance.request(8, b'', 136, .3)
        self.assertTrue(instance.close()['serial_closed'])

    def test_wrong_generation_cannot_supply_close_proof(self):
        instance, channel = self.connect([response(generation=43)], auto_close=False)
        channel.records.append(closed(generation=43))
        with self.assertRaisesRegex(ValueError, 'generation'):
            instance.request(8, b'', 136, .3)
        self.assertIsNone(instance.close()['serial_closed'])

    def test_zero_cookie_rejected_before_serial_transaction(self):
        instance, channel = self.connect()
        with self.assertRaisesRegex(ValueError, 'contract'):
            instance.request(9, b'\0'*4, 128, .3)
        self.assertFalse(any('sequence' in value for value in channel.sent))

    def test_nonmaintenance_type_rejected_without_command(self):
        instance, channel = self.connect()
        with self.assertRaises(ValueError): instance.request(3, b'', 128, .3)
        self.assertFalse(any('sequence' in value for value in channel.sent))

    def test_startup_failure_keeps_owner_and_unknown_closure(self):
        invalid = ready(self.expected)
        invalid['identity']['boot_id'] = 'other-boot'
        channel = FakeChannel([invalid], auto_close=False)
        with self.assertRaises(transport.MaintenanceTransportError) as caught:
            transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
                Path('/unused'), {}, channel_factory=lambda *args: channel)
        self.assertIsNotNone(caught.exception.transport)
        self.assertIsNone(caught.exception.close_report['serial_closed'])
        self.assertTrue(caught.exception.close_report['process_exited'])

    def test_pure_factory_builder_error_proves_no_child_was_started(self):
        with mock.patch.object(transport, 'prepare_source', side_effect=ValueError('bad profile')):
            with self.assertRaises(transport.MaintenancePreflightError) as caught:
                transport.make_transport_factory(self.expected, PROFILE, PROFILE_SHA,
                    self.directory/'known_hosts', self.directory)
        proof = caught.exception.close_report
        self.assertTrue(proof['never_opened'])
        self.assertTrue(proof['serial_closed'])
        self.assertTrue(proof['process_exited'])
        self.assertFalse(proof['ssh_started'])
        self.assertIsNone(caught.exception.transport)

    def test_preflight_bad_known_hosts_or_directory_never_calls_channel(self):
        channel_factory = mock.Mock(side_effect=AssertionError('Channel must not start'))
        factory = transport.make_transport_factory(self.expected, PROFILE, PROFILE_SHA,
            self.directory/'missing-known-hosts', self.directory, channel_factory=channel_factory)
        with self.assertRaises(transport.MaintenancePreflightError) as caught:
            factory()
        self.assertTrue(caught.exception.close_report['never_opened'])
        self.directory.chmod(0o755)
        with self.assertRaises(transport.MaintenancePreflightError) as caught:
            factory()
        self.assertTrue(caught.exception.close_report['never_opened'])
        self.directory.chmod(0o700)
        channel_factory.assert_not_called()

    def test_route_failure_before_popen_has_never_opened_proof(self):
        run = mock.Mock(return_value=mock.Mock(returncode=1, stdout='no route'))
        popen = mock.Mock(side_effect=AssertionError('SSH must not start'))
        with self.assertRaises(transport.MaintenancePreflightError) as caught:
            transport._ProcessChannel(self.source, self.directory, Path('/unused'), {},
                                      run=run, popen=popen)
        self.assertTrue(caught.exception.close_report['never_opened'])
        popen.assert_not_called()

    def test_wifi_process_uses_explicit_host_route_and_canonical_key_alias(self):
        from test_feedback_connection_check import FakeProcess
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        process = FakeProcess()
        self.addCleanup(process.clean)
        run = mock.Mock(return_value=subprocess.CompletedProcess([], 0, stdout='interface: en0\n'))
        popen = mock.Mock(return_value=process)
        parser = mock.Mock(side_effect=AssertionError('Frozen USB parser must remain unchanged'))
        writer = mock.Mock()
        helper = {'parse_route': parser, 'reader_bootstrap': lambda size: 'pass',
                  'write_pipe_once': writer}
        channel = transport._ProcessChannel(self.source, self.directory, Path('/private/known_hosts'),
                    helper, run=run, popen=popen, transport_config=wifi)
        channel.finish()
        run.assert_called_once_with(['route', '-n', 'get', '192.168.1.11'],
                                    capture_output=True, text=True, timeout=2)
        popen.assert_called_once()
        command = popen.call_args.args[0]
        for value in ('root@192.168.1.11', 'HostKeyAlias=192.168.128.10',
                      'StrictHostKeyChecking=yes', 'UserKnownHostsFile=/private/known_hosts',
                      'GlobalKnownHostsFile=/dev/null', 'ControlMaster=no', 'ConnectionAttempts=1'):
            self.assertIn(value, command)
        parser.assert_not_called()
        writer.assert_called_once()

    def test_wifi_wrong_route_is_never_opened_and_does_not_start_ssh(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        run = mock.Mock(return_value=subprocess.CompletedProcess([], 0, stdout='interface: en5\n'))
        popen = mock.Mock(side_effect=AssertionError('No SSH'))
        with self.assertRaisesRegex(transport.MaintenancePreflightError, 'Wi-Fi route') as caught:
            transport._ProcessChannel(self.source, self.directory, Path('/unused'), {},
                                      run=run, popen=popen, transport_config=wifi)
        self.assertTrue(caught.exception.close_report['never_opened'])
        popen.assert_not_called()

    def test_explicit_wifi_config_reaches_channel_and_is_recorded(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        channel = FakeChannel([ready(self.expected)])
        factory = mock.Mock(return_value=channel)
        instance = transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
            Path('/unused-known-hosts'), {}, channel_factory=factory, transport_config=wifi)
        self.assertEqual(factory.call_args.kwargs, {'transport_config': wifi})
        self.assertEqual(json.loads((self.directory/'transport-config.json').read_text()), wifi.as_record())
        self.assertTrue(instance.close()['serial_closed'])

    def test_exception_during_popen_cannot_claim_never_opened(self):
        run = mock.Mock(return_value=mock.Mock(returncode=0, stdout='reviewed route'))
        popen = mock.Mock(side_effect=OSError('indeterminate Popen failure'))
        helper = {'parse_route': lambda text: 'en5', 'reader_bootstrap': lambda size: 'pass'}
        def channel_factory(source, directory, known_hosts, ignored):
            return transport._ProcessChannel(source, directory, known_hosts, helper, run=run, popen=popen)
        with self.assertRaises(transport.MaintenanceTransportError) as caught:
            transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
                Path('/unused'), {}, channel_factory=channel_factory)
        popen.assert_called_once()
        self.assertIsNone(caught.exception.close_report['serial_closed'])
        self.assertFalse(caught.exception.close_report['process_exited'])
        self.assertIsNot(caught.exception.close_report.get('never_opened'), True)

    def test_route_failure_wrapped_by_transport_retains_proved_no_owner(self):
        def channel_factory(*args):
            raise transport.MaintenancePreflightError('reviewed route unavailable')
        with self.assertRaises(transport.MaintenanceTransportError) as caught:
            transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
                Path('/unused'), {}, channel_factory=channel_factory)
        proof = caught.exception.close_report
        self.assertTrue(proof['never_opened'])
        self.assertTrue(proof['serial_closed'])
        self.assertTrue(proof['process_exited'])

    def test_remote_request_strict_lengths_cookie_sequence_and_allowlist(self):
        validate = self.namespace['maintenance_request']
        value = {'generation': 42, 'sequence': 1, 'type': 11,
                 'payload': base64.b64encode(struct.pack('<IBB', 123, 1, 10)).decode(),
                 'reply_type': 128, 'timeout': 1.8}
        self.assertEqual(validate(value, 42, 1), struct.pack('<IBB', 123, 1, 10))
        for change in ({'type': 3}, {'sequence': 2}, {'generation': 3}, {'payload': 'AA=='},
                       {'timeout': float('nan')}, {'reply_type': 0x88},
                       {'payload': base64.b64encode(b'\0'*6).decode()}):
            with self.subTest(change=change), self.assertRaises(self.namespace['ProbeError']):
                validate(dict(value, **change), 42, 1)

    def test_voltage_extension_requests_preserve_strict_cookie_and_fixed_length(self):
        validate = self.namespace['maintenance_request']
        caps = dict(generation=42, sequence=1, type=17, payload='', reply_type=0x91, timeout=.3)
        self.assertEqual(validate(caps, 42, 1), b'')
        raw = struct.pack('<IB', 123, 10)
        read = dict(caps, type=18, reply_type=0x92, payload=base64.b64encode(raw).decode())
        self.assertEqual(validate(read, 42, 1), raw)
        for change in ({'reply_type':0x91}, {'payload':''},
                       {'payload':base64.b64encode(struct.pack('<IB',0,10)).decode()},
                       {'payload':base64.b64encode(raw+b'\x3e').decode()}):
            with self.subTest(change=change), self.assertRaises(self.namespace['ProbeError']):
                validate(dict(read, **change), 42, 1)

    def test_optional_voltage_unsupported_ack_is_preserved_without_closing_owner(self):
        raw = b'\x11\x01'
        instance, channel = self.connect([response(128, raw)])
        self.assertEqual(instance.request(17, b'', 0x91, .3), raw)
        self.assertIsNone(instance.error)
        self.assertEqual(channel.finished, 0)
        request = dict(type=17, sequence=1, reply_type=0x91, timeout=.3)
        result, writes = self.wire_exchange(self.frame(kind=128, payload=raw), request)
        self.assertEqual(result, (128, raw))
        self.assertEqual(len(writes), 1)
        instance.close()
        for raw in (b'\x11\x02', b'\x12\x01', b'\x11\0'):
            with self.subTest(raw=raw), self.assertRaises(self.namespace['ProbeError']):
                self.wire_exchange(self.frame(kind=128, payload=raw), request)

    def test_voltage_response_crc_and_sequence_are_checked_before_return(self):
        raw = struct.pack('<IBBBB', 123, 10, 0, 0, 74)
        instance, channel = self.connect([response(0x92, raw)])
        self.assertEqual(instance.request(18, struct.pack('<IB',123,10), 0x92, .3), raw)
        instance.close()
        request = dict(type=18, sequence=1, reply_type=0x92, timeout=.3)
        good = self.frame(kind=0x92, payload=raw)
        result, writes = self.wire_exchange(good, request, struct.pack('<IB',123,10))
        self.assertEqual(result,(0x92,raw)); self.assertEqual(len(writes),1)
        for packet in (good[:-1]+bytes([good[-1]^1]), self.frame(kind=0x92,sequence=2,payload=raw)):
            with self.subTest(packet=packet), self.assertRaises(self.namespace['ProbeError']):
                self.wire_exchange(packet, request, struct.pack('<IB',123,10))

    def wire_exchange(self, packet, value=None, payload=b''):
        ns = dict(self.namespace)
        value = value or {'type': 8, 'sequence': 1, 'reply_type': 136, 'timeout': .3}
        reads = [packet[:ns['HEADER'].size], packet[ns['HEADER'].size:]]
        writes = []
        ns['_read_exact'] = lambda *args: reads.pop(0)
        ns['_write_once'] = lambda fd, packet, deadline, meta: writes.append(packet)
        ns['select'] = mock.Mock(select=mock.Mock(return_value=([], [], [])))
        # Bind the function to this independent fake-I/O namespace.
        exec(compile(transport.REMOTE, '<fake-wire>', 'exec'), ns)
        return ns['maintenance_exchange'](1, value, payload, ns['time'].monotonic()+2., {}), writes

    def test_center_read_failure_evidence_is_preserved_and_owner_closes_without_retry(self):
        from test_maintenance_diagnostics import failure_ack
        raw = failure_ack(found=True, outcome=1, actual=1)
        instance, channel = self.connect([response(128, raw)])
        with self.assertRaisesRegex(ValueError, 'ACK error 19'):
            instance.request(12, struct.pack('<IBHHH', 123, 31, 2205, 1054, 2879), 128, .3)
        self.assertEqual(instance.center_diagnostic['actual'], 1)
        self.assertEqual(instance.close_report['center_diagnostic']['id'], 31)
        self.assertTrue(instance.close_report['serial_closed'])
        self.assertTrue(instance.close_report['process_exited'])
        self.assertEqual([v['type'] for v in channel.sent if 'type' in v], [12])
        persisted = json.loads((self.directory / 'report.json').read_text())
        self.assertEqual(persisted['center_diagnostic'], instance.center_diagnostic)

    def test_remote_extended_ack_validates_stage_id_and_crc_before_forwarding(self):
        from test_maintenance_diagnostics import failure_ack
        request = {'type': 16, 'sequence': 1, 'reply_type': 128, 'timeout': .3}
        payload = struct.pack('<IB', 123, 31)
        good = failure_ack(kind=16)
        result, writes = self.wire_exchange(self.frame(kind=128, payload=good), request, payload)
        self.assertEqual(result, (128, good))
        self.assertEqual(len(writes), 1)
        for bad in (failure_ack(kind=16, sid=30), failure_ack(kind=12), failure_ack(kind=16, stage=21)):
            with self.subTest(bad=bad), self.assertRaises((ValueError, self.namespace['ProbeError'])):
                self.wire_exchange(self.frame(kind=128, payload=bad), request, payload)

    def frame(self, kind=136, sequence=1, payload=b'XGODUCK_MAINT_1'):
        header = self.namespace['HEADER'].pack(b'XD', 1, kind, len(payload), sequence)
        body = header + payload
        return body + struct.pack('<H', self.namespace['_crc16'](body))

    def test_remote_crc_header_and_sequence_fail_without_retry(self):
        good = self.frame()
        result, writes = self.wire_exchange(good)
        self.assertEqual(result, (136, b'XGODUCK_MAINT_1'))
        self.assertEqual(len(writes), 1)
        for packet in (good[:-1]+bytes([good[-1] ^ 1]), self.frame(sequence=2), self.frame(kind=129)):
            with self.subTest(packet=packet), self.assertRaises(self.namespace['ProbeError']):
                self.wire_exchange(packet)

    def test_remote_ack_error_is_explicit_unsupported(self):
        with self.assertRaisesRegex(self.namespace['ProbeError'], 'unsupported'):
            self.wire_exchange(self.frame(kind=128, payload=b'\x08\x01'))

    def test_remote_owner_refusal_never_opens_serial(self):
        ns = dict(self.namespace)
        emitted = []
        ns['os'] = mock.Mock(geteuid=lambda: 0, getppid=lambda: 100, getpid=lambda: 101)
        ns['signal'] = mock.Mock(SIGTERM=15, SIGHUP=1, SIGINT=2)
        ns['sys'] = mock.Mock()
        ns['health_snapshot'] = mock.Mock(return_value={})
        ns['serial_proof'] = mock.Mock(return_value=(mock.Mock(st_rdev=5), {}))
        ns['_holders'] = mock.Mock(return_value=[{'pid': 11, 'fd': '2'}])
        ns['emit'] = lambda kind, expected, **values: emitted.append(dict(kind=kind, **values))
        exec(compile(transport.REMOTE, '<fake-remote>', 'exec'), ns)
        self.assertEqual(ns['run_maintenance'](self.expected), 20)
        ns['os'].open.assert_not_called()
        self.assertEqual([item['kind'] for item in emitted], ['fault', 'closed'])
        self.assertTrue(emitted[-1]['serial_closed'])
        self.assertEqual(emitted[-1]['query']['serial_open_attempts'], 0)

    def test_remote_health_runs_before_and_after_each_command_and_close_never_sends(self):
        ns = dict(self.namespace)
        events, emitted = [], []
        incoming = [json.dumps({'generation': 42, 'sequence': 1, 'type': 8,
                    'payload': '', 'reply_type': 136, 'timeout': .3}).encode()+b'\n',
                    b'{"generation":42,"kind":"close"}\n']
        fake_os = mock.Mock(geteuid=lambda: 0, getppid=lambda: 100, getpid=lambda: 101)
        fake_os.open.return_value = 55
        fake_os.read.side_effect = lambda *args: incoming.pop(0)
        for name in ('O_RDWR','O_NOCTTY','O_NONBLOCK','O_CLOEXEC','O_NOFOLLOW'):
            setattr(fake_os, name, 1)
        ns['os'] = fake_os
        ns['signal'] = mock.Mock(SIGTERM=15, SIGHUP=1, SIGINT=2)
        ns['sys'] = mock.Mock()
        ns['sys'].stdin.fileno.return_value = 0
        ns['sys'].stdout.fileno.return_value = 1
        ns['fcntl'] = mock.Mock(LOCK_EX=1, LOCK_NB=2)
        ns['termios'] = mock.Mock(TIOCEXCL=3)
        ns['select'] = mock.Mock(select=mock.Mock(return_value=([0], [], [])))
        ns['health_snapshot'] = lambda *args: (events.append('health') or {})
        ns['serial_proof'] = mock.Mock(return_value=(mock.Mock(st_rdev=5), {}))
        ns['_holders'] = mock.Mock(return_value=[])
        ns['_modem_bits'] = lambda *args: {'dtr': False, 'rts': False}
        ns['_raw_settings'] = lambda *args: {'baud': 921600}
        ns['emit'] = lambda kind, expected, **values: emitted.append(dict(kind=kind, **values))
        exec(compile(transport.REMOTE, '<fake-remote>', 'exec'), ns)
        ns['maintenance_exchange'] = lambda *args: (events.append('exchange') or (136, b'XGODUCK_MAINT_1'))
        self.assertEqual(ns['run_maintenance'](self.expected), 0)
        self.assertEqual(events, ['health','health','health','health','exchange','health'])
        self.assertEqual([item['kind'] for item in emitted], ['ready','response','closed'])
        fake_os.open.assert_called_once()
        fake_os.close.assert_called_once_with(55)
        self.assertTrue(emitted[-1]['serial_closed'])

    def test_factory_is_pure_until_call_and_prevents_duplicate_owner(self):
        calls = []
        def channel_factory(source, *args):
            tree = ast.parse(source)
            expected = next(ast.literal_eval(node.value) for node in tree.body
                            if isinstance(node, ast.Assign)
                            and any(isinstance(t, ast.Name) and t.id == 'EXPECTED' for t in node.targets))
            channel = FakeChannel([ready(expected)], auto_close=False)
            calls.append(channel)
            return channel
        with mock.patch.object(transport.subprocess, 'Popen', side_effect=AssertionError('No subprocess')):
            factory = transport.make_transport_factory(self.expected, PROFILE, PROFILE_SHA,
                transport.reader.EVIDENCE_DIR/'known_hosts', self.directory,
                channel_factory=channel_factory)
            self.assertEqual(calls, [])
            instance = factory()
            self.assertEqual(len(calls), 1)
            with self.assertRaisesRegex(transport.MaintenanceTransportError, 'closure proof'):
                factory()
            instance.close()
            with self.assertRaisesRegex(transport.MaintenanceTransportError, 'closure proof'):
                factory()
            self.assertEqual(len(calls), 1)

    def test_wifi_factory_is_local_until_call_and_preserves_owner_guard(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        calls = []
        def channel_factory(source, *args, **kwargs):
            tree = ast.parse(source)
            expected = next(ast.literal_eval(node.value) for node in tree.body
                            if isinstance(node, ast.Assign)
                            and any(isinstance(t, ast.Name) and t.id == 'EXPECTED' for t in node.targets))
            self.assertEqual(kwargs, {'transport_config': wifi})
            channel = FakeChannel([ready(expected)], auto_close=False)
            calls.append(channel)
            return channel
        with mock.patch.object(transport.subprocess, 'Popen', side_effect=AssertionError('No subprocess')):
            factory = transport.make_transport_factory(self.expected, PROFILE, PROFILE_SHA,
                transport.reader.EVIDENCE_DIR/'known_hosts', self.directory,
                channel_factory=channel_factory, transport_config=wifi)
            self.assertEqual(calls, [])
            instance = factory()
            self.assertEqual(len(calls), 1)
            with self.assertRaisesRegex(transport.MaintenanceTransportError, 'closure proof'):
                factory()
            instance.close()
            with self.assertRaisesRegex(transport.MaintenanceTransportError, 'closure proof'):
                factory()
            self.assertEqual(len(calls), 1)

    def test_new_kernel_storage_event_refuses_startup(self):
        bad = ready(self.expected)
        bad['identity']['kernel_log'] += '\n[ 99999.00] mmcblk1: error -110 transferring data\n'
        channel = FakeChannel([bad])
        with self.assertRaisesRegex(transport.MaintenanceTransportError, 'New kernel fault'):
            transport.SSHMaintenanceTransport(self.expected, self.source, self.directory,
                Path('/unused'), {}, channel_factory=lambda *args: channel)
        self.assertFalse(any('sequence' in value for value in channel.sent))


if __name__ == '__main__':
    unittest.main()
