"""Offline checks for the explicitly reviewed Wi-Fi cold boot.

Read local evidence and exercise source generation and pure guards only. No
reader, SSH process, HTTP server, serial port or device probe is started.
"""
import ast
import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import feedback_calibration_server as host
import feedback_stream as stream


class WifiRebootReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-wifi-reboot-reviewed-20261006-01.json'
        cls.digest = stream.WIFI_RECONNECT_PROFILE_SHA256
        cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
        cls.namespace = {}
        exec(compile(stream.runtime_source(), '<offline-reader-definitions>', 'exec'),
             cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self, profile=None):
        profile = self.profile if profile is None else profile
        return {**{key: copy.deepcopy(profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial',
            'kernel_prefix', 'historical_faults')},
            'generation': 61006, 'discover_serial': True}

    def emitted(self):
        source = stream.remote_source(self.expected(), reviewed_profile_path=self.path,
                                      reviewed_profile_sha256=self.digest)
        assignment = next(node for node in ast.parse(source).body
                          if isinstance(node, ast.Assign) and any(
                              isinstance(target, ast.Name) and target.id == 'EXPECTED'
                              for target in node.targets))
        return ast.literal_eval(assignment.value)

    def test_artifact_uses_exact_stable_raw_kernel_and_retains_first_refusal(self):
        review = self.profile['review']
        first_path = Path(review['passive_metadata_path'])
        metadata_path = Path(review['metadata_capture_path'])
        self.assertEqual(hashlib.sha256(first_path.read_bytes()).hexdigest(),
                         review['passive_metadata_sha256'])
        self.assertEqual(hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
                         review['metadata_capture_sha256'])
        first = json.loads(first_path.read_bytes())
        metadata = json.loads(metadata_path.read_bytes())
        self.assertEqual(metadata['observations'][0], metadata['observations'][1])
        self.assertIs(metadata['stable_metadata'], True)
        self.assertEqual(self.profile['kernel_prefix'],
                         metadata['observations'][0]['kernel_log'])
        self.assertEqual(self.profile['kernel_prefix'].count('\r'), 30)
        self.assertEqual(len(self.profile['kernel_prefix'].splitlines()), 924)
        self.assertEqual(self.profile['kernel_prefix'].replace('\r\n', '\n'),
                         first['kernel_capture']['kernel_log'])
        self.assertNotEqual(self.profile['kernel_prefix_sha256'],
                            first['kernel_capture']['kernel_log_sha256'])
        self.assertEqual(first['identity_validation_errors'], ['Board boot identity changed'])
        self.assertEqual(review['passive_exit_code'], 20)
        self.assertIs(review['prior_boot_pin_refusal_retained'], True)

    def test_prior_artifact_and_all_three_older_boot_fault_lists_remain_exact(self):
        review = self.profile['review']
        prior = stream.load_reviewed_profile(Path(review['prior_reviewed_profile_path']),
                                            stream.COLD_USB_DROP_PROFILE_SHA256)
        self.assertEqual(review['prior_review'], prior['review'])
        self.assertEqual(review['prior_boot_historical_faults'], prior['historical_faults'])
        self.assertEqual(review['prior_boot_historical_faults'],
                         stream.COLD_USB_DROP_HISTORICAL_FAULTS)
        older = review['earlier_boot_historical_faults']
        self.assertEqual(older[0]['boot_id'], stream.REVIEWED_BOOT_ID)
        self.assertEqual(older[0]['historical_faults'], stream.RECONNECT2_HISTORICAL_FAULTS)
        self.assertEqual(older[1]['boot_id'], stream.REBOOT_BOOT_ID)
        self.assertEqual(older[1]['historical_faults'],
                         stream.USB_PORT_MOVE_ESP32_HISTORICAL_FAULTS)
        self.assertEqual([len(x['historical_faults']) for x in older], [8, 13])
        self.assertEqual(len(review['prior_boot_historical_faults']), 3)
        for key in ('prior_boot_faults_cleared', 'filesystem_repaired',
                    'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared',
                    'unexpected_reboot_root_cause_confirmed'):
            self.assertIs(review[key], False)

    def test_source_requires_fixed_artifact_and_explicit_discovery(self):
        expected = self.expected()
        with self.assertRaises(ValueError):
            stream.remote_source(expected)
        without_discovery = {key: value for key, value in expected.items()
                             if key != 'discover_serial'}
        with self.assertRaises(ValueError):
            stream.remote_source(without_discovery, reviewed_profile_path=self.path,
                                 reviewed_profile_sha256=self.digest)
        emitted = self.emitted()
        self.assertEqual(emitted['boot_id'], stream.WIFI_RECONNECT_BOOT_ID)
        self.assertEqual(emitted['reviewed_reconnect_proof'], stream.WIFI_RECONNECT_PROOF)
        self.assertIs(emitted['discover_serial'], True)
        proof = self.namespace['validate_kernel'](self.profile['kernel_prefix'], emitted)
        self.assertEqual(proof['historical_faults'], stream.WIFI_RECONNECT_HISTORICAL_FAULTS)
        self.assertIs(proof['historical_faults_cleared'], False)

    def test_new_faults_fail_closed_and_reviewed_artifact_keeps_raw_newlines(self):
        emitted = self.emitted()
        baseline = self.profile['kernel_prefix']
        # The fixed artifact binds raw bytes; runtime classification deliberately
        # compares complete splitlines, while other consumers use startswith.
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile({**self.profile,
                'kernel_prefix': baseline.replace('\r\n', '\n')})
        for log in (baseline.split('\n', 1)[1],
                    baseline + '[ 1300.0] mmc1: error -5\n',
                    baseline + '[ 1300.1] usb 1-1.4: USB disconnect, device number 3\n',
                    baseline + '[ 1300.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                    baseline + self.profile['historical_faults'][0]['line'] + '\n'):
            with self.subTest(log_tail=log[-120:]), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](log, emitted)
        with self.assertRaises(self.Error):
            self.namespace['validate_kernel'](baseline,
                {**emitted, 'boot_id': stream.COLD_RECONNECT_BOOT_ID})

    def test_edited_identity_history_review_or_scope_cannot_expand_allowances(self):
        changes = [
            {'boot_id': '00000000-0000-4000-8000-000000000001'},
            {'cid': '0' * 32}, {'historical_faults': []},
            {'serial': {**self.profile['serial'], 'sysfs_name': '1-1.4'}},
            {'allowed_requests': ['HELLO', 'STATE', 'ARM']},
            {'review': {**self.profile['review'], 'prior_boot_historical_faults': []}},
            {'review': {**self.profile['review'], 'raw_kernel_cr_count': 0}},
            {'review': {**self.profile['review'], 'serial_holders': [123]}},
            {'review': {**self.profile['review'], 'filesystem_repaired': True}},
        ]
        changes += [{key: True} for key in stream.PROFILE_FALSE_FLAGS]
        changes += [{key: False} for key in stream.PROFILE_TRUE_FLAGS]
        for change in changes:
            with self.subTest(change=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile({**self.profile, **change})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'edited-profile.json'
            path.write_text(json.dumps({**self.profile, 'servo_motion_authorized': True}))
            path.chmod(0o600)
            with self.assertRaises(ValueError):
                stream.load_reviewed_profile(path, self.digest)
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.COLD_USB_DROP_PROFILE_SHA256)

    def test_host_switches_only_exact_prior_boots_and_retains_card_serial_and_generation(self):
        self.assertEqual(host.WIFI_RECONNECT_BOOT_ID, stream.WIFI_RECONNECT_BOOT_ID)
        self.assertEqual(host.WIFI_RECONNECT_PROFILE_SHA256, self.digest)
        expected = self.expected()
        with mock.patch.object(host, 'RECOVERY', stream.EVIDENCE_DIR):
            for boot in (host.REVIEWED_BOOT_ID, host.REBOOT_BOOT_ID,
                         host.COLD_RECONNECT_BOOT_ID, host.WIFI_RECONNECT_BOOT_ID):
                actual, raw = host.reviewed_expectations({**expected, 'boot_id': boot},
                                                        self.path, self.digest)
                self.assertEqual(actual['boot_id'], host.WIFI_RECONNECT_BOOT_ID)
                self.assertEqual(raw, self.path.read_bytes())
                for key in ('cid', 'root_mount_line', 'model', 'serial',
                            'generation', 'discover_serial'):
                    self.assertEqual(actual[key], expected[key])
                self.assertEqual(actual['historical_faults'], self.profile['historical_faults'])
            for change in ({'boot_id': 'another-origin'}, {'cid': '0' * 32},
                           {'model': 'another-board'},
                           {'serial': {**expected['serial'], 'vid': 'ffff'}},
                           {'root_mount_line': expected['root_mount_line'].replace('rw,relatime', 'ro,relatime')}):
                with self.subTest(change=next(iter(change))), self.assertRaises(ValueError):
                    host.reviewed_expectations({**expected, **change}, self.path, self.digest)

    def test_no_automatic_baseline_selection_or_motion_scope(self):
        prior = stream.load_reviewed_profile(
            stream.EVIDENCE_DIR / 'feedback-usb-drop-reviewed-20261005-01.json',
            stream.COLD_USB_DROP_PROFILE_SHA256)
        original = self.expected(prior)
        actual, raw = host.reviewed_expectations(original, None, None)
        self.assertEqual(actual, original)
        self.assertIsNone(raw)
        self.assertEqual(actual['boot_id'], stream.COLD_RECONNECT_BOOT_ID)
        self.assertEqual(self.profile['scope'], stream.RECONNECT_SCOPE)
        self.assertEqual(self.profile['allowed_requests'], ['HELLO', 'STATE'])
        for key in stream.PROFILE_FALSE_FLAGS:
            self.assertIs(self.profile[key], False)


if __name__ == '__main__':
    unittest.main()
