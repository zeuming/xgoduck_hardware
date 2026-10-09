"""Pure USB discovery contracts: no SSH, UART, HTTP or physical motion."""
import copy
from pathlib import Path
import stat
import types
import unittest
from unittest import mock

import feedback_stream as reader
import feedback_connection_check as check
from test_feedback_connection_check import fixture, remote_proof


class SerialDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.ns = {}
        exec(reader.runtime_source(), self.ns)
        self.expected = {'generation': 1, 'serial': copy.deepcopy(reader.RECONNECT_SERIAL),
                         'discover_serial': True}
        self.actual = dict(reader.RECONNECT_SERIAL, path='/dev/ttyUSB1', sysfs_name='1-1.4')
        self.info = types.SimpleNamespace(st_dev=1, st_ino=100, st_rdev=48129, st_mode=stat.S_IFCHR)
        self.calls = []
        self.nodes = [Path('/sys/class/tty/ttyUSB1')]
        def inspect(path):
            self.calls.append(path)
            return self.info, copy.deepcopy(self.actual)
        self.ns['_usb_serial_identity'] = inspect
        self.glob = mock.patch.object(Path, 'glob', side_effect=lambda pattern: list(self.nodes))
        self.glob.start()
        self.addCleanup(self.glob.stop)

    def test_unique_new_port_is_selected_then_frozen(self):
        self.assertEqual(self.ns['bind_serial'](self.expected), self.actual)
        self.assertEqual(self.ns['serial_path'](self.expected), '/dev/ttyUSB1')
        self.nodes = [Path('/sys/class/tty/ttyUSB9')]
        self.ns['bind_serial'](self.expected)
        self.assertEqual(self.calls, ['/dev/ttyUSB1'] * 3)
        self.assertEqual(self.expected['serial'], reader.RECONNECT_SERIAL)

    def test_fixed_mode_does_not_enumerate(self):
        self.expected.pop('discover_serial')
        self.assertEqual(self.ns['bind_serial'](self.expected), reader.RECONNECT_SERIAL)
        self.assertEqual(self.calls, [])

    def test_missing_and_ambiguous_nodes_refuse_without_binding(self):
        for nodes, reason in [([], 'not found'),
                              ([Path('ttyUSB0'), Path('ttyUSB1')], 'Multiple matching')]:
            with self.subTest(reason=reason):
                self.nodes = nodes
                with self.assertRaisesRegex(self.ns['ProbeError'], reason):
                    self.ns['bind_serial'](self.expected)
                self.assertEqual(self.ns['_SERIAL_BINDINGS'], {})

    def test_wrong_usb_identity_is_not_selected(self):
        for field, value in [('vid', '10c4'), ('pid', 'ea60'), ('serial_number', 'other')]:
            with self.subTest(field=field):
                self.actual = dict(reader.RECONNECT_SERIAL, **{field: value})
                with self.assertRaisesRegex(self.ns['ProbeError'], 'not found'):
                    self.ns['bind_serial'](self.expected)

    def test_enumeration_is_bounded(self):
        self.nodes = [Path('ttyUSB'+str(i)) for i in range(17)]
        with self.assertRaisesRegex(self.ns['ProbeError'], 'Too many'):
            self.ns['bind_serial'](self.expected)
        self.assertEqual(self.calls, [])

    def test_live_node_or_usb_change_never_rediscovers(self):
        self.ns['bind_serial'](self.expected)
        for field in ('st_dev', 'st_ino', 'st_rdev'):
            old = getattr(self.info, field)
            setattr(self.info, field, old+1)
            with self.assertRaisesRegex(self.ns['ProbeError'], 'changed'):
                self.ns['bind_serial'](self.expected)
            setattr(self.info, field, old)
        self.actual['sysfs_name'] = '1-1.5'
        with self.assertRaisesRegex(self.ns['ProbeError'], 'changed'):
            self.ns['serial_proof'](self.expected)
        self.assertTrue(all(path == '/dev/ttyUSB1' for path in self.calls))

    def test_open_descriptor_must_match_frozen_node(self):
        self.ns['bind_serial'](self.expected)
        opened = types.SimpleNamespace(**vars(self.info))
        with mock.patch.object(self.ns['os'], 'fstat', return_value=opened):
            self.ns['serial_proof'](self.expected, 10)
            opened.st_ino += 1
            with self.assertRaisesRegex(self.ns['ProbeError'], 'Opened UART'):
                self.ns['serial_proof'](self.expected, 10)

    def test_host_receipt_accepts_only_bounded_matching_identity(self):
        self.assertTrue(reader.serial_receipt_matches(self.actual, self.expected))
        for change in [{'path':'/dev/ttyUSB256'}, {'path':'/dev/ttyUSB01'},
                       {'path':'/tmp/ttyUSB1'}, {'sysfs_name':'../1-1.4'},
                       {'vid':'10c4'}, {'serial_number':'other'}, {'extra':True}]:
            with self.subTest(change=change):
                self.assertFalse(reader.serial_receipt_matches(dict(self.actual, **change), self.expected))
        self.expected.pop('discover_serial')
        self.assertFalse(reader.serial_receipt_matches(self.actual, self.expected))
        self.assertTrue(reader.serial_receipt_matches(reader.RECONNECT_SERIAL, self.expected))

    def test_quiescence_receipt_accepts_dynamic_minor_keeps_board_and_node_pins(self):
        expected, _ = fixture()
        expected['discover_serial'] = True
        value = remote_proof(expected)
        value['identity']['serial'] = self.actual
        value['serial_path'] = self.actual['path']
        value['serial_node']['rdev'] = 48129
        self.assertEqual(check._validate_remote(value, expected), value)
        for section, key, value2 in [('serial_node','rdev',48128),
                                     ('identity','cid','0'*32),
                                     ('identity','boot_id','0'*36)]:
            changed = copy.deepcopy(value)
            changed[section][key] = value2
            with self.assertRaises(ValueError):
                check._validate_remote(changed, expected)

    def test_all_runtime_openers_use_bound_path(self):
        import maintenance_transport, motion_workbench
        self.assertIn('bind_serial(expected)', maintenance_transport.REMOTE)
        self.assertIn('os.open(serial_path(expected)', maintenance_transport.REMOTE)
        self.assertIn('bind_serial(EXPECTED)', motion_workbench.WORKER)
        self.assertIn('port=serial_path(EXPECTED)', motion_workbench.WORKER)
        self.assertIn('Selected USB serial device does not run', motion_workbench.WORKER)
        self.assertNotIn("os.open(expected['serial']['path']", reader.RUNTIME)

    def test_port_move_profile_retains_history_and_rejects_any_new_fault(self):
        path = reader.EVIDENCE_DIR / 'feedback-usb-port-move-reviewed-20261005-01.json'
        profile = reader.load_reviewed_profile(path, reader.USB_PORT_MOVE_ESP32_PROFILE_SHA256)
        self.assertEqual(profile['historical_faults'][:-4], reader.ARM_WRITE_REPLY_ESP32_HISTORICAL_FAULTS)
        expected = {key: copy.deepcopy(profile[key]) for key in
                    ('boot_id','cid','root_mount_line','model','serial','kernel_prefix','historical_faults')}
        expected.update(generation=1, discover_serial=True)
        _, prepared = reader._source_expected(expected, path, reader.USB_PORT_MOVE_ESP32_PROFILE_SHA256)
        self.ns['validate_kernel'](profile['kernel_prefix'], prepared)
        for line in ['[ 90000.0] usb 1-1.4: USB disconnect, device number 14',
                     '[ 90000.0] mmc1: error -5']:
            with self.assertRaises(self.ns['ProbeError']):
                self.ns['validate_kernel'](profile['kernel_prefix'] + line + '\n', prepared)
        expected.pop('discover_serial')
        with self.assertRaisesRegex(ValueError, 'requires explicit ESP32 discovery'):
            reader._source_expected(expected, path, reader.USB_PORT_MOVE_ESP32_PROFILE_SHA256)
        for field in ['prior_faults_cleared', 'observed_serial', 'reviewed_usb_events']:
            changed = copy.deepcopy(profile)
            changed['review']['usb_port_move_review'].pop(field)
            with self.assertRaises(ValueError):
                reader._validate_reviewed_profile(changed)

    def test_maintenance_ready_accepts_selected_port_and_keeps_other_identity_pins(self):
        import maintenance_transport as mt
        from test_maintenance_transport import fixture as maint_fixture, ready, FakeChannel
        import tempfile
        expected, source = maint_fixture()
        expected['discover_serial'] = True
        value = ready(expected)
        value['identity']['serial'] = self.actual
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            channel = FakeChannel([value])
            transport = mt.SSHMaintenanceTransport(expected, source, path, path/'known_hosts', {},
                channel_factory=lambda *args: channel)
            self.assertEqual(transport.ready['identity']['serial'], self.actual)
            transport.close()


if __name__ == '__main__':
    unittest.main()
