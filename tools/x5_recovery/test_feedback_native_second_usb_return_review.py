"""Exact second USB return evidence; no hardware, generic exemption or retry."""
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


class NativeSecondUsbReturnReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-native-uncommanded-usb-return-reviewed-20261006-02.json'
        cls.digest = stream.NATIVE_USB_SECOND_RETURN_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.prior_path = Path(cls.profile['review']['prior_reviewed_profile_path'])
            cls.prior = stream.load_reviewed_profile(cls.prior_path, stream.NATIVE_USB_RETURN_PROFILE_SHA256)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<pure-second-usb-guards>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self, profile=None):
        profile = self.profile if profile is None else profile
        return {**{key: copy.deepcopy(profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100612, 'discover_serial': True}

    def emitted(self, prior=False):
        profile, path, digest = ((self.prior, self.prior_path, stream.NATIVE_USB_RETURN_PROFILE_SHA256)
                                if prior else (self.profile, self.path, self.digest))
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            source = stream.remote_source(self.expected(profile), reviewed_profile_path=path,
                                          reviewed_profile_sha256=digest)
        compile(source, '<unexecuted-second-usb-reader>', 'exec')
        node = next(node for node in ast.parse(source).body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        return ast.literal_eval(node.value)

    def test_three_exact_observations_raw_history_and_current_return_identity(self):
        review = self.profile['review']
        for prefix in ('metadata_result', 'metadata_source', 'metadata_ssh_exit'):
            raw = Path(review[prefix + '_path']).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), review[prefix + '_sha256'])
        evidence = json.loads(Path(review['metadata_result_path']).read_bytes())
        self.assertEqual('current_usb_metadata_only', evidence['kind'])
        self.assertIs(evidence['stable_metadata'], True)
        self.assertEqual(3, len(evidence['observations']))
        self.assertTrue(all(row == evidence['observations'][0] for row in evidence['observations']))
        self.assertEqual(.3, review['metadata_fixed_observation_interval_seconds'])
        self.assertEqual(.6, review['metadata_fixed_observation_span_seconds'])
        self.assertIn('if index<2:time.sleep(.3)', Path(review['metadata_source_path']).read_text())
        observation = evidence['observations'][0]
        kernel = self.profile['kernel_prefix']
        self.assertEqual(observation['kernel_log'], kernel)
        self.assertEqual('5b4a13362166637c829f9dc7bb9dd11225b0e0ce82f3a10ed387dabc149d82aa',
                         hashlib.sha256(kernel.encode()).hexdigest())
        self.assertEqual((944, 65925, 30), (len(kernel.splitlines()), len(kernel.encode()), kernel.count('\r')))
        self.assertTrue(kernel.startswith(self.prior['kernel_prefix']))
        self.assertEqual(934, len(self.prior['kernel_prefix'].splitlines()))
        tail = kernel[len(self.prior['kernel_prefix']):].splitlines()
        self.assertEqual(10, len(tail)); self.assertEqual(tail, review['new_kernel_tail_lines'])
        self.assertEqual('[ 3469.508801] usb 1-1-port4: disabled by hub (EMI?), re-enabling...', tail[0])
        self.assertEqual('[ 3469.515263] usb 1-1.4: USB disconnect, device number 4', tail[1])
        self.assertEqual('[ 3470.086493] usb 1-1.4: ch341-uart converter now attached to ttyUSB0', tail[-1])
        self.assertEqual({'device': 5, 'inode': 651, 'rdev': 48128}, observation['serial_node'])
        self.assertEqual('5', observation['usb_device_number'])
        self.assertEqual([], observation['serial_holders'])
        self.assertEqual('1-1.4', observation['serial']['sysfs_name'])
        self.assertEqual('1-1.3', self.profile['serial']['sysfs_name'])
        self.assertEqual(observation['serial'], review['observed_serial'])
        for field in ('config_mount', 'shm_mount'):
            self.assertEqual(observation[field], review['observed_' + field])
            self.assertEqual(observation[field], self.prior['review']['observed_' + field])
        self.assertEqual(0, evidence['serial_open_attempts']); self.assertEqual(0, evidence['serial_bytes_written'])
        self.assertIs(evidence['board_files_written'], False); self.assertIs(evidence['guard_permission_granted'], False)
        self.assertEqual({'ssh_exit_code': 0, 'process_exited': True, 'stderr': ''},
                         json.loads(Path(review['metadata_ssh_exit_path']).read_bytes()))
        self.assertIs(review['current_servo_battery_disconnected'], True)
        self.assertIs(review['physical_reconnect_not_claimed'], True)
        self.assertIs(self.profile['user_confirmed_reconnect'], False)

    def test_first_return_and_all_prior_boot_history_remain_unresolved(self):
        review = self.profile['review']
        self.assertEqual(self.prior['historical_faults'], self.profile['historical_faults'][:3])
        self.assertEqual(4, len(self.profile['historical_faults']))
        self.assertEqual(4, review['current_boot_historical_faults_count'])
        self.assertEqual(self.prior['historical_faults'], review['prior_current_boot_historical_faults'])
        self.assertEqual(self.profile['historical_faults'][-1], review['new_unresolved_usb_event'])
        history = review['all_prior_boot_historical_faults']
        self.assertEqual(history, self.prior['review']['all_prior_boot_historical_faults'])
        self.assertEqual(5, len(history)); self.assertEqual(30, review['all_prior_fault_events_count'])
        self.assertEqual([4, 2, 3, 13, 8], [len(row['historical_faults']) for row in history])
        for row in history:
            prior = stream.load_reviewed_profile(Path(row['reviewed_profile_path']), row['reviewed_profile_sha256'])
            self.assertEqual(prior['historical_faults'], row['historical_faults'])
            self.assertIs(row['prior_faults_cleared'], False)
        previous_review = json.dumps(self.prior['review'], sort_keys=True, ensure_ascii=False,
                                     separators=(',', ':'), allow_nan=False).encode()
        self.assertEqual(hashlib.sha256(previous_review).hexdigest(), review['prior_review_sha256'])
        self.assertEqual(stream.NATIVE_USB_RETURN_PROFILE_SHA256, review['prior_reviewed_profile_sha256'])
        self.assertIs(review['prior_review_retained_by_fixed_private_artifact'], True)
        self.assertLess(len(json.dumps(review, allow_nan=False).encode()), 32768)
        for field in ('new_usb_fault_cause_confirmed', 'usb_link_repaired', 'emi_confirmed',
                      'mcu_reset_confirmed', 'electrical_safety_confirmed', 'wiring_verified',
                      'powered_servo_health_certified', 'prior_boot_faults_cleared',
                      'current_boot_faults_cleared', 'previous_usb_faults_cleared', 'filesystem_repaired',
                      'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared', 'prior_imu_feedback_fault_cleared',
                      'servo_arm_authorized', 'servo_motion_authorized', 'automatic_reconnect_authorized',
                      'generic_kernel_exemption_authorized', 'dynamic_baseline_learning_authorized'):
            self.assertIs(review[field], False)

    def test_first_variant_keeps_rejecting_second_event_and_retains_actual_refusal(self):
        with self.assertRaises(self.Error) as refusal:
            self.namespace['validate_kernel'](self.profile['kernel_prefix'], self.emitted(prior=True))
        self.assertEqual(str(refusal.exception), self.profile['review']['prior_strict_refusal_retained'])
        first_proof = self.namespace['validate_kernel'](self.prior['kernel_prefix'], self.emitted(prior=True))
        self.assertIs(first_proof['historical_faults_cleared'], False)
        proof = self.namespace['validate_kernel'](self.profile['kernel_prefix'], self.emitted())
        self.assertEqual(self.profile['historical_faults'], proof['historical_faults'])
        self.assertIs(proof['historical_faults_cleared'], False)

    def test_only_fixed_hash_scope_and_discovery_are_allowed(self):
        emitted = self.emitted()
        self.assertEqual(stream.NATIVE_USB_SECOND_RETURN_PROOF, emitted['reviewed_reconnect_proof'])
        self.assertEqual('feedback_only_reviewed_reconnect', self.profile['scope'])
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        for key in stream.PROFILE_FALSE_FLAGS: self.assertIs(self.profile[key], False)
        with self.assertRaises(ValueError): stream.remote_source(self.expected())
        expected = self.expected(); expected.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(expected, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        for value in (True, 0, 1, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, user_confirmed_reconnect=value))
        with self.assertRaises(ValueError): stream.load_reviewed_profile(self.path, stream.NATIVE_USB_RETURN_PROFILE_SHA256)
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / 'edited.json'
            changed.write_bytes(json.dumps(self.profile, sort_keys=True).encode()); changed.chmod(0o600)
            with self.assertRaises(ValueError): stream.load_reviewed_profile(changed, self.digest)

    def test_new_fault_tail_and_changed_or_missing_raw_prefix_still_refused(self):
        expected = self.emitted(); kernel = self.profile['kernel_prefix']
        for tail in ('[ 5000.0] usb 1-1.4: USB disconnect, device number 5\n',
                     '[ 5000.1] mmc1: error -5\n',
                     '[ 5000.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                     self.profile['historical_faults'][-1]['line'] + '\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](kernel + tail, expected)
        with self.assertRaises(self.Error): self.namespace['validate_kernel'](kernel.split('\n', 1)[1], expected)
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile, kernel_prefix=kernel.replace('\r\n', '\n')))
        metadata = dict(self.profile, swaps='Filename\tType\tSize\tUsed\tPriority')
        for change in ({'boot_id': stream.IMU_RECOVERY_USB_BOOT_ID}, {'cid': '0' * 32},
                       {'model': 'another board'}, {'root_mount_line': 'another mount'}):
            with self.subTest(field=next(iter(change))), self.assertRaises(self.Error):
                self.namespace['validate_os'](dict(metadata, **change), expected)

    def test_no_edit_can_remove_history_claim_health_or_authorize_movement(self):
        changes = [dict(historical_faults=[]), dict(allowed_requests=['HELLO', 'STATE', 'ARM'])]
        changes.extend({field: True} for field in stream.PROFILE_FALSE_FLAGS)
        changes.extend({'review': dict(self.profile['review'], **change)} for change in (
            dict(new_usb_fault_cause_confirmed=True), dict(electrical_safety_confirmed=True),
            dict(current_servo_battery_disconnected=False), dict(user_confirmed_reconnect=True),
            dict(prior_current_boot_historical_faults=[]), dict(all_prior_boot_historical_faults=[]),
            dict(prior_strict_refusal_retained=''), dict(new_kernel_tail_lines=[]),
            dict(observed_serial_node={}), dict(metadata_result_sha256='0' * 64),
            dict(dynamic_baseline_learning_authorized=True)))
        for change in changes:
            with self.subTest(field=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))


if __name__ == '__main__':
    unittest.main()
