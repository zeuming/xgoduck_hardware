"""Fixed third USB return and bounded log eviction; pure, no hardware calls."""
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


class NativeThirdUsbReturnReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-native-uncommanded-usb-return-reviewed-20261006-03.json'
        cls.digest = stream.NATIVE_USB_THIRD_RETURN_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.prior_path = Path(cls.profile['review']['prior_reviewed_profile_path'])
            cls.prior = stream.load_reviewed_profile(cls.prior_path, stream.NATIVE_USB_SECOND_RETURN_PROFILE_SHA256)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<pure-third-usb-guards>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self, profile=None):
        profile = self.profile if profile is None else profile
        return {**{key: copy.deepcopy(profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100614, 'discover_serial': True}

    def emitted(self, prior=False):
        profile, path, digest = ((self.prior, self.prior_path, stream.NATIVE_USB_SECOND_RETURN_PROFILE_SHA256)
                                if prior else (self.profile, self.path, self.digest))
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            source = stream.remote_source(self.expected(profile), reviewed_profile_path=path,
                                          reviewed_profile_sha256=digest)
        compile(source, '<unexecuted-third-usb-reader>', 'exec')
        node = next(node for node in ast.parse(source).body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        return ast.literal_eval(node.value)

    def test_exact_raw_eviction_preserves_nonfault_lines_and_byte_overlap(self):
        review = self.profile['review']
        old = self.prior['kernel_prefix']
        kernel = self.profile['kernel_prefix']
        raw_lines = old.splitlines(keepends=True)
        evicted = ''.join(raw_lines[:36])
        retained = ''.join(raw_lines[36:])
        self.assertEqual(review['evicted_nonfault_lines'], evicted.splitlines())
        self.assertEqual((36, 2803), (len(evicted.splitlines()), len(evicted.encode())))
        self.assertEqual('e1233623953949e4624372d776fcf93a62e49fd4430f7c9f1f1792f7eb164e9f',
                         hashlib.sha256(evicted.encode()).hexdigest())
        self.assertEqual(hashlib.sha256(evicted.encode()).hexdigest(), review['evicted_nonfault_raw_prefix_sha256'])
        self.assertEqual(0, review['evicted_fault_lines_count'])
        for line in evicted.splitlines():
            self.assertIsNone(self.namespace['classify_feedback_log'](line))
        self.assertEqual((908, 63122), (len(retained.splitlines()), len(retained.encode())))
        self.assertEqual(hashlib.sha256(retained.encode()).hexdigest(), review['retained_prior_raw_suffix_sha256'])
        self.assertTrue(kernel.encode().startswith(retained.encode()))
        self.assertEqual(old.splitlines()[36:], kernel.splitlines()[:908])
        self.assertEqual((968, 67313, 30), (len(kernel.splitlines()), len(kernel.encode()), kernel.count('\r')))
        self.assertEqual('e8ff7b0b3a9456eb1a7c37fe1b73a2e8f10c49f874ae4c589788e07ecc924d48',
                         hashlib.sha256(kernel.encode()).hexdigest())
        self.assertEqual('[    0.000000] software IO TLB: area num 8.', kernel.splitlines()[0])
        self.assertIs(review['current_original_kernel_prefix_complete'], False)
        self.assertIs(review['current_early_timestamp_retained'], True)
        self.assertIs(review['original_full_kernel_retained_by_fixed_private_artifact'], True)

    def test_three_exact_metadata_observations_and_six_complete_returns(self):
        review = self.profile['review']
        for prefix in ('metadata_result', 'metadata_source', 'metadata_ssh_exit', 'metadata_comparison'):
            raw = Path(review[prefix + '_path']).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), review[prefix + '_sha256'])
        evidence = json.loads(Path(review['metadata_result_path']).read_bytes())
        self.assertEqual('native_pose_refusal_metadata', evidence['kind'])
        self.assertIs(evidence['stable_metadata'], True)
        self.assertEqual(3, len(evidence['observations']))
        self.assertTrue(all(row == evidence['observations'][0] for row in evidence['observations']))
        self.assertEqual((.3, .6), (review['metadata_fixed_observation_interval_seconds'],
                                  review['metadata_fixed_observation_span_seconds']))
        self.assertIn('if index<2: time.sleep(.30)', Path(review['metadata_source_path']).read_text())
        observation = evidence['observations'][0]
        self.assertEqual(self.profile['kernel_prefix'], observation['kernel_log'])
        self.assertEqual({'device': 5, 'inode': 717, 'rdev': 48128}, observation['serial_node'])
        self.assertEqual('11', observation['usb_device_number'])
        self.assertEqual([], observation['serial_holders'])
        self.assertEqual('1-1.4', observation['serial']['sysfs_name'])
        self.assertEqual('1-1.3', self.profile['serial']['sysfs_name'])
        self.assertEqual(observation['serial'], review['observed_serial'])
        for field in ('config_mount', 'shm_mount'):
            self.assertEqual(observation[field], review['observed_' + field])
            self.assertEqual(observation[field], self.prior['review']['observed_' + field])
        for field in ('serial_open_attempts', 'serial_bytes_written'):
            self.assertEqual(0, evidence[field])
        for field in ('remote_files_written', 'motor_start_attempted', 'automatic_retry'):
            self.assertIs(evidence[field], False)
        self.assertEqual({'ssh_exit_code': 0, 'ssh_process_exited': True, 'stderr': ''},
                         json.loads(Path(review['metadata_ssh_exit_path']).read_bytes()))
        retained = ''.join(self.prior['kernel_prefix'].splitlines(keepends=True)[36:])
        tail = self.profile['kernel_prefix'][len(retained):].splitlines()
        self.assertEqual(60, len(tail))
        self.assertEqual(tail, review['new_kernel_tail_lines'])
        self.assertEqual(6, review['new_completed_usb_cycles'])
        for cycle in range(6):
            lines = tail[cycle * 10:(cycle + 1) * 10]
            self.assertIn('disabled by hub (EMI?), re-enabling...', lines[0])
            self.assertIn('USB disconnect, device number ' + str(5 + cycle), lines[1])
            self.assertIn('new full-speed USB device number ' + str(6 + cycle), lines[4])
            self.assertIn('idVendor=1a86, idProduct=7523', lines[5])
            self.assertIn('Product: USB Serial', lines[7])
            self.assertIn('ch341-uart converter now attached to ttyUSB0', lines[9])
        self.assertEqual('[ 7150.450579] usb 1-1.4: ch341-uart converter now attached to ttyUSB0', tail[-1])
        self.assertIs(review['current_servo_battery_reported_disconnected'], True)
        self.assertIs(review['physical_reconnect_not_claimed'], True)
        self.assertIs(self.profile['user_confirmed_reconnect'], False)

    def test_prior_faults_and_full_artifact_history_are_preserved_unresolved(self):
        review = self.profile['review']
        self.assertEqual(self.prior['historical_faults'], self.profile['historical_faults'][:4])
        self.assertEqual(10, len(self.profile['historical_faults']))
        self.assertEqual(10, review['current_boot_historical_faults_count'])
        self.assertEqual(4, review['prior_current_boot_fault_events_count'])
        self.assertEqual(self.prior['historical_faults'], review['prior_current_boot_historical_faults'])
        self.assertEqual(self.profile['historical_faults'][4:], review['new_unresolved_usb_events'])
        classified = [{'line': line, 'reason': reason} for line in self.profile['kernel_prefix'].splitlines()
                      if (reason := self.namespace['classify_feedback_log'](line))]
        self.assertEqual(self.profile['historical_faults'], classified)
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
        self.assertEqual(stream.NATIVE_USB_SECOND_RETURN_PROFILE_SHA256, review['prior_reviewed_profile_sha256'])
        self.assertIs(review['prior_review_retained_by_fixed_private_artifact'], True)
        self.assertLess(len(json.dumps(review, allow_nan=False).encode()), 32768)
        for field in ('new_usb_fault_cause_confirmed', 'usb_link_repaired', 'emi_confirmed',
                      'mcu_reset_confirmed', 'electrical_safety_confirmed', 'wiring_verified',
                      'powered_servo_health_certified', 'prior_boot_faults_cleared',
                      'current_boot_faults_cleared', 'previous_usb_faults_cleared', 'filesystem_repaired',
                      'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared', 'prior_imu_feedback_fault_cleared',
                      'servo_arm_authorized', 'servo_motion_authorized', 'automatic_reconnect_authorized',
                      'generic_kernel_exemption_authorized', 'generic_prefix_loss_exemption_authorized',
                      'dynamic_baseline_learning_authorized'):
            self.assertIs(review[field], False)

    def test_old_profile_still_refuses_this_history_and_retains_failed_http_check(self):
        review = self.profile['review']
        for prefix in ('failed_http_imu_proof', 'failed_http_imu_request', 'failed_http_imu_final_status'):
            self.assertEqual(hashlib.sha256(Path(review[prefix + '_path']).read_bytes()).hexdigest(),
                             review[prefix + '_sha256'])
        failure = json.loads(Path(review['failed_http_imu_proof_path']).read_bytes())
        self.assertEqual('Retained kernel history changed or lost its reviewed prefix', failure['technical_reason'])
        self.assertEqual(failure['technical_reason'], review['prior_strict_refusal_retained'])
        self.assertIs(failure['motor_start_attempted'], False)
        self.assertIs(failure['calibration_save_attempted'], False)
        self.assertIs(failure['feedback_left_active'], False)
        with self.assertRaises(self.Error) as refusal:
            self.namespace['validate_kernel'](self.profile['kernel_prefix'], self.emitted(prior=True))
        self.assertEqual(str(refusal.exception), failure['technical_reason'])
        old_proof = self.namespace['validate_kernel'](self.prior['kernel_prefix'], self.emitted(prior=True))
        self.assertIs(old_proof['historical_faults_cleared'], False)
        proof = self.namespace['validate_kernel'](self.profile['kernel_prefix'], self.emitted())
        self.assertEqual(self.profile['historical_faults'], proof['historical_faults'])
        self.assertIs(proof['historical_faults_cleared'], False)

    def test_only_fixed_hash_feedback_scope_and_discovery_are_allowed(self):
        self.assertEqual(stream.NATIVE_USB_THIRD_RETURN_PROOF, self.emitted()['reviewed_reconnect_proof'])
        self.assertEqual('feedback_only_reviewed_reconnect', self.profile['scope'])
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        for key in stream.PROFILE_FALSE_FLAGS:
            self.assertIs(self.profile[key], False)
        with self.assertRaises(ValueError):
            stream.remote_source(self.expected())
        expected = self.expected(); expected.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(expected, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        for value in (True, 0, 1, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, user_confirmed_reconnect=value))
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.NATIVE_USB_SECOND_RETURN_PROFILE_SHA256)
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / 'edited.json'
            changed.write_bytes(json.dumps(self.profile, sort_keys=True).encode()); changed.chmod(0o600)
            with self.assertRaises(ValueError):
                stream.load_reviewed_profile(changed, self.digest)

    def test_future_fault_and_further_eviction_or_raw_normalization_still_refused(self):
        expected = self.emitted(); kernel = self.profile['kernel_prefix']
        for tail in ('[ 8000.0] usb 1-1.4: USB disconnect, device number 11\n',
                     '[ 8000.1] mmc1: error -5\n',
                     '[ 8000.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                     self.profile['historical_faults'][-1]['line'] + '\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](kernel + tail, expected)
        with self.assertRaises(self.Error):
            self.namespace['validate_kernel'](kernel.split('\n', 1)[1], expected)
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile, kernel_prefix=kernel.replace('\r\n', '\n')))
        metadata = dict(self.profile, swaps='Filename\tType\tSize\tUsed\tPriority')
        for change in ({'boot_id': stream.IMU_RECOVERY_USB_BOOT_ID}, {'cid': '0' * 32},
                       {'model': 'another board'}, {'root_mount_line': 'another mount'}):
            with self.subTest(field=next(iter(change))), self.assertRaises(self.Error):
                self.namespace['validate_os'](dict(metadata, **change), expected)

    def test_evidence_edits_cannot_remove_faults_claim_health_or_authorize_movement(self):
        changes = [dict(historical_faults=[]), dict(allowed_requests=['HELLO', 'STATE', 'ARM'])]
        changes.extend({field: True} for field in stream.PROFILE_FALSE_FLAGS)
        changes.extend({'review': dict(self.profile['review'], **change)} for change in (
            dict(new_usb_fault_cause_confirmed=True), dict(electrical_safety_confirmed=True),
            dict(current_servo_battery_reported_disconnected=False), dict(user_confirmed_reconnect=True),
            dict(prior_current_boot_historical_faults=[]), dict(all_prior_boot_historical_faults=[]),
            dict(prior_strict_refusal_retained=''), dict(new_kernel_tail_lines=[]),
            dict(evicted_nonfault_lines=[]), dict(evicted_fault_lines_count=1),
            dict(retained_prior_raw_suffix_sha256='0' * 64), dict(observed_serial_node={}),
            dict(metadata_result_sha256='0' * 64), dict(failed_http_imu_proof_sha256='0' * 64),
            dict(generic_prefix_loss_exemption_authorized=True), dict(dynamic_baseline_learning_authorized=True)))
        for change in changes:
            with self.subTest(field=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))


if __name__ == '__main__':
    unittest.main()
