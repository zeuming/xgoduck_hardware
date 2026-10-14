"""CPU-only regression for an offline startup with an externally owned UART."""
import unittest
from test_deploy_native_workbench import NativeConnectionFactoryTests

class OfflineUARTReconnectTests(unittest.TestCase):
    factory_fixture = NativeConnectionFactoryTests.factory_fixture

    def offline(self, ns):
        ns['deployment']['startup_mode'] = 'metadata_only_fault_page'
        ns['pin']['serial_node_posix'] = None
        ns['pin']['startup_fault'] = dict(hardware_gate_passed=False, serial_open_attempts=0)

    def test_external_owner_is_reported_before_missing_pin_and_not_consumed(self):
        with self.factory_fixture() as (ns, model, _):
            self.offline(ns); model.holders = [{'pid': 123}]
            with self.assertRaisesRegex(ValueError, 'already has an owner'):
                ns['connection_guard_factory'](usb_recovery=True)
            self.assertIsNone(ns['uart_identity'])
            self.assertTrue(ns['initial_connection_gate_pending'])
            self.assertEqual(model.holders, [{'pid': 123}])
            model.holders = []
            gate = ns['connection_guard_factory'](usb_recovery=True)
            self.assertFalse(ns['initial_connection_gate_pending'])
            gate['ram_health']()
            model.inode += 1
            with self.assertRaises(ValueError): gate['ram_health']()

    def test_missing_pin_needs_explicit_recovery_and_offline_proof(self):
        for case in ('initial', 'ordinary', 'strict', 'missing_proof'):
            with self.subTest(case=case), self.factory_fixture() as (ns, model, _):
                self.offline(ns)
                kwargs = dict(usb_recovery=True)
                if case == 'initial': kwargs['initial'] = True
                if case == 'ordinary': kwargs = {}
                if case == 'strict': ns['deployment']['startup_mode'] = 'strict'
                if case == 'missing_proof': ns['pin'].pop('startup_fault')
                with self.assertRaisesRegex(ValueError, 'initial UART'):
                    ns['connection_guard_factory'](**kwargs)
                self.assertIsNone(ns['uart_identity'])

    def test_failed_health_and_late_owner_never_publish_missing_baseline(self):
        for case in ('health', 'late_owner', 'calibration'):
            with self.subTest(case=case), self.factory_fixture() as (ns, model, _):
                self.offline(ns)
                if case == 'health': model.health_error = 'kernel fault'
                elif case == 'calibration': (ns['root']/'policy/calibration.json').write_bytes(b'{}')
                else:
                    original = ns['native_transports'].isolated_holders
                    def scan(*args):
                        value = original(*args)
                        return [{'pid': 123}] if len(model.isolated_scans) > 1 else value
                    ns['native_transports'].isolated_holders = scan
                with self.assertRaises(ValueError): ns['connection_guard_factory'](usb_recovery=True)
                self.assertIsNone(ns['uart_identity'])
                self.assertTrue(ns['initial_connection_gate_pending'])

if __name__ == '__main__': unittest.main()
