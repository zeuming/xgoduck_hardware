"""Synthetic host integration only: no sockets, SSH processes, or UART."""
import contextlib
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock

import feedback_calibration_server as host
from test_feedback_calibration_server import config
from transport_config import TransportConfig


class ServerWifiTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.known = self.directory / 'known_hosts'
        self.known.write_text('private-fixture')
        self.wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        self.session = host.CalibrationSession(config(), self.directory, 7,
                                               transport_config=self.wifi)

    def preflight(self, route):
        helper = SimpleNamespace(
            EXPECTED_FINGERPRINT='pinned',
            known_hosts_fingerprint=lambda text: 'pinned',
            parse_route=lambda text: 'en5')
        spec = SimpleNamespace(loader=SimpleNamespace(exec_module=lambda value: None))
        with mock.patch.object(host.importlib.util, 'spec_from_file_location', return_value=spec), \
                mock.patch.object(host.importlib.util, 'module_from_spec', return_value=helper), \
                mock.patch.object(host.subprocess, 'run', return_value=route) as runner, \
                mock.patch.object(host.subprocess, 'Popen', side_effect=AssertionError('No SSH')):
            result = host.reader_launch_preflight(self.session, b'local fixture', self.known,
                                                  self.directory, 7)
            return result, runner

    def test_reader_selects_wifi_and_pins_original_host_identity_without_starting_ssh(self):
        (command, _, _), runner = self.preflight(SimpleNamespace(
            returncode=0, stdout='   interface: en0\n'))
        self.assertIn('root@192.168.1.11', command)
        self.assertNotIn('root@192.168.128.10', command)
        self.assertIn('HostKeyAlias=192.168.128.10', command)
        self.assertIn('StrictHostKeyChecking=yes', command)
        self.assertIn('ControlMaster=no', command)
        runner.assert_called_once_with(['route', '-n', 'get', '192.168.1.11'],
                                      capture_output=True, text=True, timeout=2)
        record = json.loads((self.directory / 'hardware-attempt.json').read_text())
        self.assertEqual(record['network'], dict(kind='wifi', host='192.168.1.11', interface='en0'))
        self.assertEqual(record['requests'], ['HELLO', 'STATE'])

    def test_reader_rejects_usb_interface_before_source_or_attempt_record(self):
        with self.assertRaisesRegex(ValueError, 'Wi-Fi route'):
            self.preflight(SimpleNamespace(returncode=0, stdout='interface: en5\n'))
        self.assertFalse((self.directory / 'hardware-attempt.json').exists())
        self.assertFalse((self.directory / 'remote-source.py').exists())

    def test_snapshot_and_motion_use_same_target_without_device_access(self):
        snapshot = self.session.snapshot()
        self.assertEqual(snapshot['connection_target'], dict(
            kind='wifi', host='192.168.1.11', page_url='http://127.0.0.1:8767/'))
        with mock.patch('motion_workbench.MotionWorkbench') as controller:
            self.session.configure_motion({}, None, None, self.known)
            self.assertEqual(controller.call_args.kwargs['transport_config'], self.wifi)

    def test_network_diagnostic_does_not_ask_wifi_user_to_replug_usb(self):
        text = host.connection_check_error_text('Connection-check SSH transport failed: Host is down',
                                                self.wifi)
        self.assertIn('192.168.1.11', text)
        self.assertIn('Wi-Fi', text)
        self.assertNotIn('USB', text)
        # Changing the path must not relabel an identity fault as a timeout.
        trust = host.connection_check_error_text('Pinned SSH host identity changed: changed', self.wifi)
        self.assertIn('身份', trust)
        self.assertNotIn('同一网络', trust)

    def test_invalid_cli_or_uncertain_motion_auto_start_fails_before_any_service(self):
        base = ['feedback_calibration_server', '--run-dir', str(self.directory / 'unused')]
        cases = [
            ['--x5-network', 'wifi'],
            ['--x5-network', 'wifi', '--x5-host', '192.168.1.11:8767'],
            ['--x5-network', 'wifi', '--x5-host', '192.168.1.11', '--x5-interface', 'en5'],
            ['--x5-host', '192.168.1.11'],
            ['--resume-motion-run', str(self.directory / 'motion-unknown')],
            ['--start-paused', '--reconcile-motion-at-start'],
            ['--start-paused', '--resume-run', str(self.directory),
             '--resume-motion-run', str(self.directory / 'motion-unknown'), '--deploy-motion'],
        ]
        for options in cases:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), \
                    mock.patch.object(sys, 'argv', base + options), \
                    mock.patch.object(host, 'ThreadingHTTPServer', side_effect=AssertionError('No sockets')), \
                    mock.patch.object(host.subprocess, 'Popen', side_effect=AssertionError('No SSH')):
                with self.assertRaises(SystemExit) as error:
                    host.main()
                self.assertEqual(error.exception.code, 2)
                self.assertFalse((self.directory / 'unused').exists())


if __name__ == '__main__':
    unittest.main()
