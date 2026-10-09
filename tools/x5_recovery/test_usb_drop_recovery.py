"""Pure source/profile checks; never contact the board or open a UART."""
import copy
import unittest

import feedback_calibration_server as host
import feedback_connection_check as connection
import feedback_stream as reader


class UsbDropRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.path = reader.EVIDENCE_DIR / 'feedback-usb-drop-reviewed-20261005-01.json'
        self.sha = reader.COLD_USB_DROP_PROFILE_SHA256
        self.profile = reader.load_reviewed_profile(self.path, self.sha)
        self.expected = {key: copy.deepcopy(self.profile[key]) for key in
                         ('boot_id', 'cid', 'root_mount_line', 'model', 'serial',
                          'kernel_prefix', 'historical_faults')}
        self.expected.update(generation=42, discover_serial=True)

    def test_reviewed_usb_incident_remains_in_history_and_every_new_fault_still_stops(self):
        expected, source = connection.prepare_source(self.expected, self.path, self.sha)
        namespace = {}
        exec(compile(connection.runtime_source(), '<pure-usb-review>', 'exec'), namespace)
        proof = namespace['validate_kernel'](expected['kernel_prefix'], expected)
        self.assertEqual(proof['historical_faults'], self.profile['historical_faults'])
        self.assertEqual(len(proof['historical_faults']), 3)
        self.assertIn('USB disconnect', proof['historical_faults'][-1]['line'])
        for tail in ('[40000.1] mmc1: error -5\n',
                     '[40000.2] usb 1-1.4: USB disconnect, device number 4\n'):
            with self.subTest(tail=tail), self.assertRaises(namespace['ProbeError']):
                namespace['validate_kernel'](expected['kernel_prefix'] + tail, expected)
        self.assertLess(len(source), 262144)

    def test_review_cannot_drop_faults_or_replace_identity_or_usb_evidence(self):
        changes = ({'historical_faults': self.profile['historical_faults'][:2]},
                   {'boot_id': 'unreviewed-boot'}, {'cid': 'other-card'})
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                reader._validate_reviewed_profile(dict(self.profile, **change))
        changed = copy.deepcopy(self.profile)
        changed['review'].pop('metadata_capture_sha256')
        with self.assertRaises(ValueError):
            reader._validate_reviewed_profile(changed)

    def test_local_startup_accepts_exact_reviewed_board_without_changing_calibration_or_allowing_another_boot(self):
        for boot in (host.REVIEWED_BOOT_ID, host.REBOOT_BOOT_ID, reader.COLD_RECONNECT_BOOT_ID):
            expected = dict(self.expected, boot_id=boot)
            updated, _ = host.reviewed_expectations(expected, self.path, self.sha)
            self.assertEqual(updated['boot_id'], reader.COLD_RECONNECT_BOOT_ID)
            self.assertEqual(updated['historical_faults'], self.profile['historical_faults'])
            self.assertEqual(expected['boot_id'], boot)
        with self.assertRaises(ValueError):
            host.reviewed_expectations(dict(self.expected, boot_id='other'), self.path, self.sha)


if __name__ == '__main__':
    unittest.main()
