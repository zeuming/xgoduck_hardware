"""Offline fixed evidence for a USB return without a human cable reconnect."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import feedback_stream as stream


class NativeUsbReturnReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-native-uncommanded-usb-return-reviewed-20261006-01.json'
        cls.digest = stream.NATIVE_USB_RETURN_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.prior = stream.load_reviewed_profile(
                Path(cls.profile['review']['prior_reviewed_profile_path']),
                stream.INTENTIONAL_POWERCYCLE_PROFILE_SHA256)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<pure-native-usb-guards>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self, profile=None):
        profile = self.profile if profile is None else profile
        return {**{key: copy.deepcopy(profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100608, 'discover_serial': True}

    def emitted(self, *, prior=False):
        profile = self.prior if prior else self.profile
        path = Path(self.profile['review']['prior_reviewed_profile_path']) if prior else self.path
        digest = stream.INTENTIONAL_POWERCYCLE_PROFILE_SHA256 if prior else self.digest
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            source = stream.remote_source(self.expected(profile), reviewed_profile_path=path,
                                          reviewed_profile_sha256=digest)
        compile(source, '<unexecuted-explicit-native-usb-reader>', 'exec')
        assignment = next(node for node in ast.parse(source).body if isinstance(node, ast.Assign)
                          and any(isinstance(target, ast.Name) and target.id == 'EXPECTED'
                                  for target in node.targets))
        return ast.literal_eval(assignment.value)

    def test_exact_raw_metadata_with_no_human_usb_reconnect_or_uart_access(self):
        review = self.profile['review']
        for prefix in ('metadata_result', 'metadata_source', 'metadata_ssh_exit', 'blocked_diagnostic'):
            data = Path(review[prefix + '_path']).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), review[prefix + '_sha256'])
        evidence = json.loads(Path(review['metadata_result_path']).read_bytes())
        self.assertEqual('native_usb_return_metadata_only', evidence['kind'])
        self.assertIs(evidence['stable_metadata'], True)
        self.assertEqual(evidence['observations'][0], evidence['observations'][1])
        observation = evidence['observations'][0]
        raw = self.profile['kernel_prefix']
        self.assertEqual(observation['kernel_log'], raw)
        self.assertEqual('446afca6b270b70792f41981099aba26345335c056530f404dc6990a3e2166c7',
                         hashlib.sha256(raw.encode()).hexdigest())
        self.assertEqual((934, 65227, 30), (len(raw.splitlines()), len(raw.encode()), raw.count('\r')))
        self.assertTrue(raw.startswith(self.prior['kernel_prefix']))
        self.assertEqual(raw[len(self.prior['kernel_prefix']):].splitlines(), review['new_kernel_tail_lines'])
        self.assertEqual(10, len(review['new_kernel_tail_lines']))
        self.assertIn('disabled by hub (EMI?)', review['new_kernel_tail_lines'][0])
        self.assertEqual({'device': 5, 'inode': 640, 'rdev': 48128}, observation['serial_node'])
        self.assertEqual([], observation['serial_holders'])
        self.assertEqual('4', observation['usb_device_number'])
        self.assertEqual('1-1.3', self.profile['serial']['sysfs_name'])
        self.assertEqual('1-1.4', observation['serial']['sysfs_name'])
        self.assertEqual(observation['serial'], review['observed_serial'])
        self.assertIn('shared:189', observation['config_mount'][0])
        self.assertEqual(0, evidence['serial_open_attempts'])
        self.assertEqual(0, evidence['serial_bytes_written'])
        self.assertIs(evidence['board_files_written'], False)
        self.assertIs(evidence['guard_permission_granted'], False)
        self.assertEqual({'ssh_exit_code': 0, 'ssh_process_exited': True, 'stderr': ''},
                         json.loads(Path(review['metadata_ssh_exit_path']).read_bytes()))
        self.assertIs(self.profile['user_confirmed_reconnect'], False)
        self.assertIs(review['user_confirmed_reconnect'], False)
        self.assertIs(review['user_reported_com_not_touched'], True)
        self.assertIs(review['user_reported_servo_battery_disconnected'], True)
        self.assertIs(review['physical_reconnect_not_claimed'], True)
        self.assertNotIn('native_processes', review)

    def test_all_five_prior_boot_histories_and_new_unresolved_fault_retained(self):
        review = self.profile['review']
        self.assertEqual(self.prior['review']['all_prior_boot_historical_faults'],
                         review['all_prior_boot_historical_faults'])
        history = review['all_prior_boot_historical_faults']
        self.assertEqual(5, len(history))
        self.assertEqual(30, sum(len(row['historical_faults']) for row in history))
        self.assertEqual(30, review['all_prior_fault_events_count'])
        self.assertEqual([4, 2, 3, 13, 8], [len(row['historical_faults']) for row in history])
        for row in history:
            prior = stream.load_reviewed_profile(Path(row['reviewed_profile_path']), row['reviewed_profile_sha256'])
            self.assertEqual(prior['historical_faults'], row['historical_faults'])
            self.assertIs(row['prior_faults_cleared'], False)
        self.assertEqual(self.prior['historical_faults'], self.profile['historical_faults'][:2])
        self.assertEqual(3, review['current_boot_historical_faults_count'])
        self.assertEqual('[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
                         self.profile['historical_faults'][-1]['line'])
        self.assertEqual(self.profile['historical_faults'][-1], review['new_unresolved_usb_event'])
        old_review_bytes = json.dumps(self.prior['review'], sort_keys=True, ensure_ascii=False,
                                     separators=(',', ':'), allow_nan=False).encode()
        self.assertEqual(hashlib.sha256(old_review_bytes).hexdigest(), review['prior_review_sha256'])
        self.assertIs(review['prior_review_retained_by_fixed_private_artifact'], True)
        self.assertLess(len(json.dumps(review, allow_nan=False).encode()), 32768)
        for field in ('new_usb_fault_cause_confirmed', 'usb_link_repaired', 'emi_confirmed',
                      'mcu_reset_confirmed', 'electrical_safety_confirmed', 'wiring_verified',
                      'powered_servo_health_certified', 'prior_boot_faults_cleared',
                      'current_boot_faults_cleared', 'previous_usb_faults_cleared',
                      'filesystem_repaired', 'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared',
                      'prior_imu_feedback_fault_cleared', 'servo_arm_authorized',
                      'servo_motion_authorized', 'automatic_reconnect_authorized'):
            self.assertIs(review[field], False)

    def test_false_reconnect_is_only_valid_for_the_new_immutable_hash(self):
        stream._validate_reviewed_profile(self.profile)
        for value in (True, 0, 1, None):
            with self.subTest(new_value=value), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, user_confirmed_reconnect=value))
        stream._validate_reviewed_profile(self.prior)
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.prior, user_confirmed_reconnect=False))
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile, wifi_warning_reviewed_as_specific_old_driver_event=False))
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.INTENTIONAL_POWERCYCLE_PROFILE_SHA256)
        with tempfile.TemporaryDirectory() as directory:
            rewritten = Path(directory) / 'rewritten-profile.json'
            rewritten.write_bytes(json.dumps(self.profile, sort_keys=True, ensure_ascii=False).encode())
            rewritten.chmod(0o600)
            with self.assertRaises(ValueError): stream.load_reviewed_profile(rewritten, self.digest)

    def test_discovery_is_required_and_scope_has_no_motion_or_automatic_reconnect(self):
        with self.assertRaises(ValueError): stream.remote_source(self.expected())
        expected = self.expected(); expected.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(expected, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        emitted = self.emitted()
        self.assertEqual(stream.NATIVE_USB_RETURN_PROOF, emitted['reviewed_reconnect_proof'])
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        self.assertEqual('feedback_only_reviewed_reconnect', self.profile['scope'])
        for key in stream.PROFILE_FALSE_FLAGS: self.assertIs(self.profile[key], False)
        proof = self.namespace['validate_kernel'](self.profile['kernel_prefix'], emitted)
        self.assertEqual(self.profile['historical_faults'], proof['historical_faults'])
        self.assertIs(proof['historical_faults_cleared'], False)

    def test_old_profile_still_refuses_this_usb_event_and_new_profile_refuses_future_faults(self):
        raw = self.profile['kernel_prefix']
        with self.assertRaises(self.Error): self.namespace['validate_kernel'](raw, self.emitted(prior=True))
        emitted = self.emitted()
        for tail in ('[ 3000.0] usb 1-1.4: USB disconnect, device number 4\n',
                     '[ 3000.1] mmc1: error -5\n',
                     '[ 3000.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                     self.profile['historical_faults'][-1]['line'] + '\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](raw + tail, emitted)
        with self.assertRaises(self.Error): self.namespace['validate_kernel'](raw.split('\n', 1)[1], emitted)
        proof = self.namespace['validate_kernel'](raw + '[ 3000.3] harmless message\n', emitted)
        self.assertIs(proof['historical_faults_cleared'], False)

    def test_fixed_review_identity_raw_history_and_no_motion_fields_cannot_be_edited(self):
        review = self.profile['review']
        changes = [dict(boot_id=stream.IMU_RECOVERY_USB_BOOT_ID), dict(historical_faults=[]),
                   dict(allowed_requests=['HELLO', 'STATE', 'ARM']),
                   dict(kernel_prefix=self.profile['kernel_prefix'].replace('\r\n', '\n'))]
        changes.extend({flag: True} for flag in stream.PROFILE_FALSE_FLAGS)
        changes.extend({'review': dict(review, **change)} for change in (
            dict(user_reported_com_not_touched=False), dict(user_reported_servo_battery_disconnected=False),
            dict(user_confirmed_reconnect=True), dict(all_prior_boot_historical_faults=[]),
            dict(new_usb_fault_cause_confirmed=True), dict(new_unresolved_usb_event={}),
            dict(electrical_safety_confirmed=True), dict(observed_serial_node={}),
            dict(metadata_result_sha256='0' * 64)))
        for change in changes:
            with self.subTest(change=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))


if __name__ == '__main__':
    unittest.main()
