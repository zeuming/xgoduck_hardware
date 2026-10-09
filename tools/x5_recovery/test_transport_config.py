"""Pure local transport selection and route contracts; never probes a network."""
from dataclasses import FrozenInstanceError
import subprocess
import unittest
from unittest import mock

from transport_config import CANONICAL_HOST, DEFAULT_TRANSPORT, TransportConfig, resolve_transport


class TransportConfigTests(unittest.TestCase):
    def test_default_remains_exact_reviewed_usb(self):
        self.assertIs(resolve_transport(), DEFAULT_TRANSPORT)
        self.assertEqual(DEFAULT_TRANSPORT.route_command(), ['route', '-n', 'get', CANONICAL_HOST])
        self.assertEqual(DEFAULT_TRANSPORT.target, 'root@' + CANONICAL_HOST)
        parser = mock.Mock(return_value='en5')
        self.assertEqual(DEFAULT_TRANSPORT.validate_route('legacy route', {'parse_route': parser}), 'en5')
        parser.assert_called_once_with('legacy route')
        self.assertEqual(DEFAULT_TRANSPORT.ssh_options(),
                         ['-p', '22', '-o', 'HostKeyAlias=192.168.128.10'])

    def test_wifi_explicit_private_address_retains_canonical_key(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        self.assertIs(resolve_transport(wifi), wifi)
        self.assertEqual(wifi.target, 'root@192.168.1.11')
        self.assertEqual(wifi.route_command(), ['route', '-n', 'get', '192.168.1.11'])
        self.assertEqual(wifi.ssh_options(), DEFAULT_TRANSPORT.ssh_options())
        frozen_usb_parser = mock.Mock(side_effect=AssertionError('USB parser must remain USB-only'))
        self.assertEqual(wifi.validate_route('   route to: 192.168.1.11\n  interface: en0\n',
                                             {'parse_route': frozen_usb_parser}), 'en0')
        frozen_usb_parser.assert_not_called()
        for host in ('10.0.0.11', '172.16.0.11', '172.31.255.10'):
            self.assertEqual(TransportConfig(kind='wifi', host=host, interface='en0').host, host)

    def test_config_immutable_and_usb_cannot_be_relabelled(self):
        with self.assertRaises(FrozenInstanceError):
            DEFAULT_TRANSPORT.host = '192.168.1.11'
        for changes in ({'host': '192.168.1.11'}, {'interface': 'en0'}, {'kind': 'other'},
                        {'user': 'sunrise'}, {'port': 2222}, {'port': True},
                        {'host_key_alias': '192.168.1.11'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TransportConfig(**changes)
        for invalid in ({}, 'wifi', True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                resolve_transport(invalid)

    def test_wifi_rejects_hostname_options_loopback_public_and_other_interfaces(self):
        for host in ('', 'localhost', '127.0.0.1', '0.0.0.0', '169.254.1.11', '8.8.8.8',
                     '100.64.0.11', '224.0.0.1', '::1', '192.168.001.11',
                     '192.168.1.11 -o StrictHostKeyChecking=no', CANONICAL_HOST):
            with self.subTest(host=host), self.assertRaises(ValueError):
                TransportConfig(kind='wifi', host=host, interface='en0')
        for interface in ('en5', 'en1', 'utun0', '', 'en0\n'):
            with self.subTest(interface=interface), self.assertRaises(ValueError):
                TransportConfig(kind='wifi', host='192.168.1.11', interface=interface)

    def test_wifi_requires_exact_one_interface_and_successful_route(self):
        wifi = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        for raw in ('', 'interface: en5\n', 'interface: utun0\n',
                    'interface: en0\ninterface: en0\n', 'interface: en0\ninterface: en5\n',
                    'interface: en0 extra\n', 'interface: en0\0\n', 'x'*16385, None):
            with self.subTest(raw=str(raw)[:80]), self.assertRaisesRegex(ValueError, 'Wi-Fi route'):
                wifi.validate_route(raw)
        route = subprocess.CompletedProcess([], 1, stdout='interface: en0\n')
        with self.assertRaisesRegex(ValueError, 'Wi-Fi route'):
            wifi.verify_route(route)
        self.assertEqual(wifi.verify_route(subprocess.CompletedProcess([], 0, stdout='interface: en0\n')), 'en0')

    def test_wifi_does_not_weaken_usb_route_validation(self):
        parser = mock.Mock(side_effect=ValueError('Route is not the verified en5 USB flash link'))
        with self.assertRaisesRegex(ValueError, 'verified en5'):
            DEFAULT_TRANSPORT.validate_route('interface: en0\n', {'parse_route': parser})
        parser.assert_called_once()
        with self.assertRaisesRegex(ValueError, 'USB route'):
            DEFAULT_TRANSPORT.verify_route(subprocess.CompletedProcess([], 1, stdout='interface: en5\n'), {})


if __name__ == '__main__':
    unittest.main()
