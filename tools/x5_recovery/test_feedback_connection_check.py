"""Pure local metadata/reconciliation contracts. No SSH or device access."""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import types
import unittest
from unittest import mock

import feedback_connection_check as check
from transport_config import TransportConfig

PROFILE = check.reader.EVIDENCE_DIR / 'feedback-returned-esp32-reviewed-20261004-01.json'
PROFILE_SHA = check.reader.RETURNED_ESP32_PROFILE_SHA256


def fixture():
    profile = check.reader.load_reviewed_profile(PROFILE, PROFILE_SHA)
    expected = {key: copy.deepcopy(profile[key]) for key in
                ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')}
    expected['generation'] = 42
    return check.prepare_source(expected, PROFILE, PROFILE_SHA)


def remote_proof(expected):
    namespace = {}
    exec(compile(check.runtime_source(), '<test-definitions>', 'exec'), namespace)
    identity = {key: copy.deepcopy(expected[key]) for key in
                ('boot_id', 'cid', 'root_mount_line', 'model', 'serial')}
    identity['swaps'] = 'Filename\tType\tSize\tUsed\tPriority'
    identity.update(namespace['validate_kernel'](expected['kernel_prefix'], expected))
    return dict(kind='quiescent', generation=42, serial_path='/dev/ttyUSB0', serial_holders=[],
                identity_checked=True, guards_passed=True, observation_count=2,
                serial_node={'device': 1, 'inode': 100, 'rdev': 48128},
                identity=identity, serial_open_attempts=0, serial_bytes_written=0,
                board_files_written=False, automatic_retry=False, elapsed_seconds=.25)


class ConnectionCheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected, cls.source = fixture()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name).resolve()
        self.directory.chmod(0o700)
        self.known_hosts = self.directory / 'known_hosts'
        check.private_new(self.known_hosts, 'fake-known-hosts\n')
        self.helper = {'known_hosts_fingerprint': lambda value: check.FINGERPRINT}
        self.calls = []
        self.runner_options = []

    def tearDown(self):
        self.temp.cleanup()

    def execute(self, value=None, *, output=None, stderr=b'', status=0, exited=True,
                transport_config=None):
        if value is None:
            value = remote_proof(self.expected)
        if output is None:
            output = (json.dumps(value, allow_nan=False) + '\n').encode()
        def runner(*args, **kwargs):
            self.calls.append(args)
            self.runner_options.append(kwargs)
            return output, stderr, status, exited
        with mock.patch.object(check, 'helpers', return_value=self.helper):
            return check.check_connection(self.expected, self.known_hosts, self.directory,
                                          PROFILE, PROFILE_SHA, runner=runner,
                                          transport_config=transport_config)

    def test_wifi_config_reaches_one_runner_without_changing_remote_identity_proof(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        proof = self.execute(transport_config=wifi)
        self.assertEqual(self.runner_options, [{'transport_config': wifi}])
        self.assertEqual(self.calls[0][0], self.source)
        self.assertEqual(proof['identity']['boot_id'], self.expected['boot_id'])
        self.assertEqual(proof['identity']['cid'], self.expected['cid'])
        self.assertEqual(json.loads((self.directory/'transport-config.json').read_text()), wifi.as_record())
        self.assertEqual(proof['serial_open_attempts'], 0)

    def test_wifi_does_not_bypass_the_original_known_host_pin(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        self.helper['known_hosts_fingerprint'] = lambda value: 'SHA256:different-key'
        with self.assertRaisesRegex(ValueError, 'Pinned SSH host identity changed'):
            self.execute(transport_config=wifi)
        self.assertEqual(self.calls, [])
        self.assertFalse((self.directory/'proof.json').exists())

    def test_runtime_has_no_uart_open_configure_request_or_feedback_loop(self):
        tree = ast.parse(self.source)
        functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
        self.assertIn('health_snapshot', functions)
        self.assertIn('_holders', functions)
        self.assertIn('check_connection_metadata', functions)
        self.assertFalse({'run_reader', 'run_rx_capture', '_exchange', '_frame', '_read_exact',
                          '_write_once', '_raw_settings', '_modem_bits'} & functions)
        calls = [ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)]
        self.assertNotIn('os.open', calls)
        self.assertNotIn('fcntl.ioctl', calls)
        self.assertNotIn('fcntl.flock', calls)
        self.assertNotIn('os.kill', calls)
        self.assertLess(len(self.source), 262144)

    def test_completed_check_observes_single_zero_exit_and_persists_evidence(self):
        proof = self.execute()
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(proof['ssh_exit_code'], 0)
        self.assertTrue(proof['ssh_process_exited'])
        self.assertEqual(proof['serial_open_attempts'], 0)
        self.assertEqual(proof['source_sha256'], hashlib.sha256(self.source).hexdigest())
        self.assertEqual(check.validate_proof(proof, self.expected), proof)
        self.assertEqual(json.loads((self.directory / 'proof.json').read_text()), proof)
        self.assertFalse('serial_closed' in proof)

    def test_linux_uart_receipt_uses_linux_encoding_on_every_host(self):
        value = remote_proof(self.expected)
        value['serial_node'] = {'device': 5, 'inode': 640, 'rdev': 48128}
        with mock.patch.object(check.os, 'major', side_effect=AssertionError('host encoding used')), \
                mock.patch.object(check.os, 'minor', side_effect=AssertionError('host encoding used')):
            self.assertEqual(check._validate_remote(value, self.expected), value)

    def test_persistent_guard_receipt_requires_exact_authority_match(self):
        value = remote_proof(self.expected)
        receipt = dict(boot_id=self.expected['boot_id'],
            kernel_sha256=value['identity']['kernel_sha256'],cause_known=False,automatic_retry=False)
        source = check.reader.runtime_source() + '''
_prior_test_kernel = validate_kernel
def validate_kernel(kernel, expected):
    result = _prior_test_kernel(kernel, expected)
    result['reviewed_usb_return'] = TEST_REVIEWED_RECEIPT
    return result
''' + '\nTEST_REVIEWED_RECEIPT = ' + repr(receipt)
        value['identity']['reviewed_usb_return'] = receipt
        self.assertEqual(check._validate_remote(value,self.expected,validation_source=source),value)
        for change in (
            lambda s:s['identity']['reviewed_usb_return'].update(automatic_retry=True),
            lambda s:s['identity']['reviewed_usb_return'].update(kernel_sha256='0'*64),
            lambda s:s['identity'].pop('reviewed_usb_return'),
            lambda s:s['identity'].update(unrecognized_extension=True),
        ):
            changed=copy.deepcopy(value);change(changed)
            with self.assertRaises(ValueError):check._validate_remote(changed,self.expected,validation_source=source)
        with self.assertRaises(ValueError):check._validate_remote(value,self.expected)

    def test_wrong_linux_uart_number_or_invalid_node_never_passes(self):
        for field, invalid in (('rdev', 48129), ('rdev', 187 << 8), ('rdev', 188 << 24),
                               ('rdev', True), ('rdev', -1), ('rdev', (1 << 64) + 48128),
                               ('inode', 0), ('device', True)):
            with self.subTest(field=field, invalid=invalid):
                value = remote_proof(self.expected)
                value['serial_node'][field] = invalid
                with self.assertRaisesRegex(ValueError, 'UART node proof'):
                    check._validate_remote(value, self.expected)

    def test_fault_keeps_guard_reason_and_never_returns_proof(self):
        reason = 'Retained kernel history changed or lost its reviewed prefix'
        with self.assertRaisesRegex(ValueError, 'Retained kernel history'):
            self.execute({'kind': 'fault', 'generation': 42, 'reason': reason}, status=20)
        self.assertFalse((self.directory / 'proof.json').exists())
        self.assertEqual(len(self.calls), 1)

    def test_nonzero_ssh_exit_cannot_turn_good_record_into_proof(self):
        with self.assertRaisesRegex(ValueError, 'did not exit normally'):
            self.execute(status=255)

    def test_live_child_cannot_turn_good_record_into_proof(self):
        with self.assertRaisesRegex(ValueError, 'did not exit normally'):
            self.execute(exited=False)

    def test_stderr_is_not_ignored(self):
        with self.assertRaisesRegex(ValueError, 'reported stderr'):
            self.execute(stderr=b'SSH closed unexpectedly')

    def test_lost_ssh_without_output_remains_a_transport_failure_not_a_fake_guard_fault(self):
        for output in (b'', b'{"kind":'):
            with self.subTest(output=output), tempfile.TemporaryDirectory() as local:
                self.directory = Path(local).resolve()
                self.directory.chmod(0o700)
                with self.assertRaisesRegex(ValueError, 'SSH transport failed'):
                    self.execute(output=output, status=255)
                self.assertFalse((self.directory / 'proof.json').exists())

    def test_remote_host_key_failure_is_an_identity_fault_not_a_reconnect_suggestion(self):
        with self.assertRaisesRegex(ValueError, 'Pinned SSH host identity changed'):
            self.execute(output=b'', status=255,
                         stderr=b'WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!\nHost key verification failed.\n')
        self.assertFalse((self.directory / 'proof.json').exists())

    def test_requires_full_exact_single_record(self):
        for output in (b'', b'{}', b'{}\n{}\n', b'{}\npartial'):
            with self.subTest(output=output), tempfile.TemporaryDirectory() as local:
                self.directory = Path(local).resolve()
                self.directory.chmod(0o700)
                with self.assertRaisesRegex(ValueError, 'exactly one complete'):
                    self.execute(output=output)

    def test_duplicate_json_keys_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON'):
            self.execute(output=b'{"kind":"quiescent","kind":"fault"}\n')

    def test_wrong_generation_or_fake_observation_or_holder_rejected(self):
        for key, value in (('generation', True), ('generation', 43),
                           ('serial_holders', [{'pid': 6, 'fd': '3'}]),
                           ('observation_count', 1), ('observation_count', True),
                           ('serial_open_attempts', 1), ('serial_bytes_written', 1),
                           ('board_files_written', True), ('automatic_retry', True),
                           ('elapsed_seconds', .01), ('identity_checked', 1)):
            with self.subTest(key=key, value=value), tempfile.TemporaryDirectory() as local:
                self.directory = Path(local).resolve()
                self.directory.chmod(0o700)
                proof = remote_proof(self.expected)
                proof[key] = value
                with self.assertRaises(ValueError):
                    self.execute(proof)

    def test_extra_field_and_serial_closed_forgery_rejected(self):
        value = remote_proof(self.expected)
        value['serial_closed'] = True
        with self.assertRaisesRegex(ValueError, 'schema'):
            self.execute(value)

    def test_other_boot_card_model_serial_or_root_rejected(self):
        for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial'):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as local:
                self.directory = Path(local).resolve()
                self.directory.chmod(0o700)
                value = remote_proof(self.expected)
                value['identity'][key] = 'other'
                with self.assertRaisesRegex(ValueError, 'identity differs'):
                    self.execute(value)

    def test_missing_or_new_fault_kernel_never_learns_new_baseline(self):
        for kernel in ('[ 1000.0] ordinary missing old kernel\n',
                       self.expected['kernel_prefix'] + '\n[ 99999.0] mmcblk1: error -110\n'):
            with self.subTest(kernel=kernel[-80:]), tempfile.TemporaryDirectory() as local:
                self.directory = Path(local).resolve()
                self.directory.chmod(0o700)
                value = remote_proof(self.expected)
                value['identity']['kernel_log'] = kernel
                with self.assertRaises((ValueError, RuntimeError)):
                    self.execute(value)

    def test_kernel_hash_or_historical_clear_forgery_rejected(self):
        value = remote_proof(self.expected)
        value['identity']['kernel_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'kernel evidence'):
            self.execute(value)

    def test_wrong_source_hash_rejected(self):
        proof = dict(remote_proof(self.expected), ssh_exit_code=0,
                     ssh_process_exited=True, source_sha256='0' * 64)
        with self.assertRaisesRegex(ValueError, 'source proof differs'):
            check.validate_proof(proof, self.expected)

    def test_bad_known_host_blocks_before_one_ssh_invocation(self):
        self.helper['known_hosts_fingerprint'] = lambda value: 'wrong'
        with self.assertRaisesRegex(ValueError, 'Pinned SSH'):
            self.execute()
        self.assertEqual(self.calls, [])

    def test_private_directory_required_before_ssh(self):
        self.directory.chmod(0o755)
        with self.assertRaisesRegex(ValueError, 'private owned directory'):
            self.execute()
        self.assertEqual(self.calls, [])


class RemoteMetadataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected, cls.source = fixture()

    def run_remote(self, *, holders=([], []), nodes=None, health_error=None):
        namespace = {}
        exec(compile(check.runtime_source(), '<test-remote>', 'exec'), namespace)
        events, health_calls, holder_calls = [], [], []
        health = remote_proof(self.expected)['identity']
        node = types.SimpleNamespace(st_dev=1, st_ino=100, st_rdev=48128)
        node_values = list(nodes or [node, node, node])
        owner_values = list(holders)
        def snapshot(expected):
            health_calls.append(copy.deepcopy(expected))
            if health_error:
                raise namespace['ProbeError'](health_error)
            return copy.deepcopy(health)
        def serial(expected):
            return node_values.pop(0), copy.deepcopy(expected['serial'])
        def owners(rdev, deadline):
            holder_calls.append((rdev, deadline))
            return owner_values.pop(0)
        namespace['health_snapshot'] = snapshot
        namespace['serial_proof'] = serial
        namespace['_holders'] = owners
        namespace['emit'] = lambda kind, expected, **values: events.append(
            dict(kind=kind, generation=expected['generation'], **values))
        with mock.patch.object(namespace['os'], 'geteuid', return_value=0), \
             mock.patch.object(namespace['os'], 'getppid', return_value=100), \
             mock.patch.object(namespace['os'], 'set_blocking'), \
             mock.patch.object(namespace['os'], 'open', side_effect=AssertionError('UART must never open')), \
             mock.patch.object(namespace['signal'], 'signal'), \
             mock.patch.object(namespace['signal'], 'setitimer'):
            result = namespace['check_connection_metadata'](self.expected)
        return result, events, health_calls, holder_calls

    def test_success_requires_two_real_health_and_holder_inventories(self):
        status, events, health, holders = self.run_remote()
        self.assertEqual(status, 0)
        self.assertEqual(len(health), 2)
        self.assertEqual(len(holders), 2)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['kind'], 'quiescent')
        check._validate_remote(events[0], self.expected)

    def test_first_holder_stops_without_retry_or_later_health(self):
        status, events, health, holders = self.run_remote(holders=([{'pid': 77, 'fd': '4'}],))
        self.assertEqual(status, 20)
        self.assertEqual(len(health), 1)
        self.assertEqual(len(holders), 1)
        self.assertEqual(events[0]['kind'], 'fault')
        self.assertIn('UART already has an open holder', events[0]['reason'])

    def test_holder_race_second_inventory_stops(self):
        status, events, health, holders = self.run_remote(holders=([], [{'pid': 77, 'fd': '4'}]))
        self.assertEqual(status, 20)
        self.assertEqual(len(health), 2)
        self.assertEqual(len(holders), 2)
        self.assertEqual(events[0]['kind'], 'fault')

    def test_node_replacement_stops_even_without_holders(self):
        old = types.SimpleNamespace(st_dev=1, st_ino=100, st_rdev=48128)
        other = types.SimpleNamespace(st_dev=1, st_ino=101, st_rdev=48128)
        status, events, _, holders = self.run_remote(nodes=[old, other])
        self.assertEqual(status, 20)
        self.assertEqual(len(holders), 1)
        self.assertIn('node changed', events[0]['reason'])

    def test_kernel_or_identity_error_passes_raw_guard_reason(self):
        reason = 'Early kernel history is unavailable'
        status, events, _, holders = self.run_remote(health_error=reason)
        self.assertEqual(status, 20)
        self.assertEqual(holders, [])
        self.assertEqual(events[0]['reason'], reason)


class FakeProcess:
    """Real local pipes only; no process, SSH, network or serial is launched."""
    def __init__(self, stdout=b'{}\n', stderr=b'', status=0):
        stdin_read, stdin_write = os.pipe()
        self.stdin_read = stdin_read
        self.stdin = os.fdopen(stdin_write, 'wb', buffering=0)
        self.stdout = self.pipe(stdout)
        self.stderr = self.pipe(stderr)
        self.status, self.returncode = status, None
        self.waits = []
        self.terminations = self.kills = 0

    @staticmethod
    def pipe(value):
        reading, writing = os.pipe()
        os.write(writing, value)
        os.close(writing)
        return os.fdopen(reading, 'rb', buffering=0)

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.waits.append((timeout, self.stdin.closed))
        self.returncode = self.status if self.returncode is None else self.returncode
        return self.returncode

    def terminate(self):
        self.terminations += 1
        self.returncode = -15

    def kill(self):
        self.kills += 1
        self.returncode = -9

    def clean(self):
        for stream in (self.stdin, self.stdout, self.stderr):
            stream.close()
        os.close(self.stdin_read)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name).resolve()
        self.directory.chmod(0o700)
        self.commands, self.routes, self.writes = [], [], []
        self.process = FakeProcess()
        self.helper = dict(parse_route=lambda value: 'en5',
                           reader_bootstrap=lambda length: 'read exactly ' + str(length),
                           write_pipe_once=lambda *args: self.writes.append(args))

    def tearDown(self):
        self.process.clean()
        self.temp.cleanup()

    def popen(self, command, **options):
        self.commands.append((command, options))
        return self.process

    def route(self, command, **options):
        self.routes.append((command, options))
        return subprocess.CompletedProcess(command, 0, stdout='reviewed USB route', stderr='')

    def execute(self, **options):
        return check._run_ssh(b'local source only', self.directory, Path('/private/known_hosts'),
                             self.helper, run=self.route, popen=self.popen, **options)

    def source_failure(self, *, stdout=b'', stderr=b'', status=255, error=None):
        """Expose a failed source send while preserving the child's real pipes."""
        self.process.stdout.close()
        self.process.stderr.close()
        # A complete identity proof includes the reviewed kernel prefix and
        # can exceed an OS pipe's capacity before the fake reader starts.
        # Feed a real local pipe concurrently rather than blocking setup.
        reading, writing = os.pipe()
        def write_output():
            offset = 0
            try:
                while offset < len(stdout):
                    offset += os.write(writing, stdout[offset:])
            except BrokenPipeError:
                pass
            finally:
                os.close(writing)
        writer = threading.Thread(target=write_output, daemon=True)
        writer.start()
        self.addCleanup(writer.join, .5)
        self.process.stdout = os.fdopen(reading, 'rb', buffering=0)
        self.process.stderr = FakeProcess.pipe(stderr)
        self.process.status = status
        self.process.returncode = status
        failure = error or BrokenPipeError(32, 'Broken pipe')
        def fail_once(*args):
            self.writes.append(args)
            raise failure
        self.helper['write_pipe_once'] = fail_once

    def assert_failure_evidence(self, stdout, stderr):
        self.assertEqual((self.directory / 'stdout.jsonl').read_bytes(), stdout)
        if stderr:
            self.assertEqual((self.directory / 'stderr.txt').read_bytes(), stderr)
        self.assertTrue((self.directory / 'transport-failure.json').is_file())
        self.assertFalse((self.directory / 'proof.json').exists())
        self.assertEqual(len(self.commands), 1)
        self.assertEqual(len(self.writes), 1)
        self.assertFalse((self.directory / 'askpass.sh').exists())

    def test_broken_source_pipe_preserves_early_ssh_authentication_error_and_exit(self):
        stderr = b'root@192.168.128.10: Permission denied (password).\r\n'
        self.source_failure(stderr=stderr)
        with self.assertRaisesRegex(ValueError, 'Connection-check SSH transport failed.*Permission denied'):
            self.execute()
        self.assert_failure_evidence(b'', stderr)
        self.assertEqual(self.process.returncode, 255)
        self.assertEqual(self.process.terminations, 0)
        self.assertTrue(self.process.stdin.closed)

    def test_failed_source_send_cannot_grant_proof_even_with_forged_complete_success(self):
        expected, _ = fixture()
        stdout = (json.dumps(remote_proof(expected), allow_nan=False) + '\n').encode()
        self.source_failure(stdout=stdout, status=0)
        with self.assertRaisesRegex(ValueError, 'Connection-check SSH transport failed'):
            self.execute()
        self.assert_failure_evidence(stdout, b'')
        self.assertEqual(self.process.returncode, 0)

    def test_broken_source_pipe_preserves_host_key_failure_as_identity_error(self):
        stderr = (b'WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!\n'
                  b'Host key verification failed.\n')
        self.source_failure(stderr=stderr)
        with self.assertRaisesRegex(ValueError, 'Pinned SSH host identity changed'):
            self.execute()
        self.assert_failure_evidence(b'', stderr)

    def test_source_send_timeout_drains_child_failure_once_without_retry(self):
        stderr = b'ssh: connect to host 192.168.128.10 port 22: Operation timed out\n'
        self.source_failure(stderr=stderr, error=TimeoutError('source send deadline expired'))
        with self.assertRaisesRegex(ValueError, 'Connection-check SSH transport failed.*timed out'):
            self.execute()
        self.assert_failure_evidence(b'', stderr)
        self.assertEqual(self.process.returncode, 255)
        self.assertEqual(self.process.terminations, 0)

    def test_exact_one_pinned_ssh_and_full_pipe_eof_before_exit(self):
        output, stderr, status, exited = self.execute()
        self.assertEqual((output, stderr, status, exited), (b'{}\n', b'', 0, True))
        self.assertEqual(len(self.commands), 1)
        self.assertEqual(len(self.routes), 1)
        self.assertEqual(len(self.writes), 1)
        command, options = self.commands[0]
        self.assertIn('StrictHostKeyChecking=yes', command)
        self.assertIn('UserKnownHostsFile=/private/known_hosts', command)
        self.assertIn('ConnectionAttempts=1', command)
        self.assertIn('ControlMaster=no', command)
        self.assertIn('root@' + check.TARGET, command)
        self.assertIn('HostKeyAlias=192.168.128.10', command)
        self.assertIn('python3 -I -B -S', command[-1])
        self.assertTrue(options['start_new_session'])
        self.assertEqual(self.process.waits[0][1], False)
        self.assertEqual(self.process.terminations, 0)
        self.assertFalse((self.directory / 'askpass.sh').exists())

    def test_wifi_one_pinned_ssh_uses_explicit_route_and_original_host_key_alias(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        def route(command, **options):
            self.routes.append((command, options))
            return subprocess.CompletedProcess(command, 0, stdout='interface: en0\n')
        self.helper['parse_route'] = mock.Mock(side_effect=AssertionError('Do not weaken USB parser'))
        result = check._run_ssh(b'local source only', self.directory, Path('/private/known_hosts'),
                               self.helper, run=route, popen=self.popen, transport_config=wifi)
        self.assertEqual(result, (b'{}\n', b'', 0, True))
        self.assertEqual(self.routes[0][0], ['route', '-n', 'get', '192.168.1.11'])
        self.assertEqual(len(self.commands), 1)
        command = self.commands[0][0]
        for item in ('root@192.168.1.11', 'HostKeyAlias=192.168.128.10',
                     'StrictHostKeyChecking=yes', 'UserKnownHostsFile=/private/known_hosts',
                     'GlobalKnownHostsFile=/dev/null', 'ControlMaster=no', 'ConnectionAttempts=1'):
            self.assertIn(item, command)
        self.helper['parse_route'].assert_not_called()

    def test_wifi_wrong_interface_stops_before_child_or_source(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        route = mock.Mock(return_value=subprocess.CompletedProcess([], 0, stdout='interface: en5\n'))
        with self.assertRaisesRegex(ValueError, 'Wi-Fi route'):
            check._run_ssh(b'source', self.directory, Path('/private/known_hosts'), self.helper,
                           run=route, popen=self.popen, transport_config=wifi)
        self.assertEqual(self.commands, [])
        self.assertEqual(self.writes, [])
        self.assertFalse((self.directory/'askpass.sh').exists())

    def test_nonzero_real_process_status_is_preserved(self):
        self.process.status = 255
        self.assertEqual(self.execute()[2:], (255, True))
        self.assertEqual(len(self.commands), 1)

    def test_bad_route_never_creates_ssh_child(self):
        self.helper['parse_route'] = lambda value: 'en0'
        with self.assertRaisesRegex(ValueError, 'Reviewed USB route'):
            self.execute()
        self.assertEqual(self.commands, [])
        self.assertEqual(self.writes, [])

    def test_deadline_ends_only_own_local_child_without_retry(self):
        values = iter((0., 10., 20.))
        with self.assertRaisesRegex(TimeoutError, 'deadline expired'):
            self.execute(clock=lambda: next(values))
        self.assertEqual(len(self.commands), 1)
        self.assertEqual(self.process.terminations, 1)
        self.assertEqual(self.process.kills, 0)
        self.assertFalse((self.directory / 'askpass.sh').exists())

    def test_output_bound_stops_without_discarding_large_record(self):
        with mock.patch.object(check, 'MAX_OUTPUT', 2):
            with self.assertRaisesRegex(ValueError, 'bounded size'):
                self.execute()
        self.assertEqual(len(self.commands), 1)
        self.assertEqual(self.process.terminations, 1)


if __name__ == '__main__':
    unittest.main()
