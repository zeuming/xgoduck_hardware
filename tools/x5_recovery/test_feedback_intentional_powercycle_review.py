"""Offline evidence and fail-closed guards for an explicit intentional reboot."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import unittest
from unittest import mock

import feedback_stream as stream


class IntentionalPowercycleReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-intentional-powercycle-reviewed-20261006-01.json'
        cls.digest = stream.INTENTIONAL_POWERCYCLE_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<pure-powercycle-guards>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self):
        return {**{key: copy.deepcopy(self.profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100605, 'discover_serial': True}

    def emitted(self):
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            source = stream.remote_source(self.expected(), reviewed_profile_path=self.path,
                                          reviewed_profile_sha256=self.digest)
        node = next(node for node in ast.parse(source).body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        compile(source, '<unexecuted-explicit-powercycle-reader>', 'exec')
        return ast.literal_eval(node.value)

    def test_complete_raw_kernel_stable_inventory_and_user_powercycle_context(self):
        review = self.profile['review']
        for prefix in ('metadata_result', 'metadata_source', 'metadata_ssh_exit'):
            data = Path(review[prefix + '_path']).read_bytes()
            self.assertEqual(review[prefix + '_sha256'], hashlib.sha256(data).hexdigest())
        evidence = json.loads(Path(review['metadata_result_path']).read_bytes())
        self.assertIs(evidence['stable_metadata'], True)
        self.assertEqual(evidence['observations'][0], evidence['observations'][1])
        observation = evidence['observations'][0]; kernel = self.profile['kernel_prefix']
        self.assertEqual(kernel, observation['kernel_log'])
        self.assertEqual('31cadf6fbc9e50742b35f18f4731be25f25c7b76a22f7613878c6d9e408f66a8',
                         hashlib.sha256(kernel.encode()).hexdigest())
        self.assertEqual((64529, 924, 30), (len(kernel.encode()), len(kernel.splitlines()), kernel.count('\r')))
        self.assertTrue(kernel.startswith('[    0.000000] Booting Linux'))
        self.assertEqual(['boot_id changed'], observation['identity_validation_errors'])
        self.assertEqual({'device': 5, 'inode': 307, 'rdev': 48128}, observation['serial_node'])
        self.assertEqual([], observation['serial_holders']); self.assertEqual('3', observation['usb_device_number'])
        self.assertIn('shared:189', observation['config_mount'][0])
        for field in ('native_processes', 'native_ram_directories', 'listeners_8767'):
            self.assertEqual([], evidence[field]); self.assertEqual([], review[field])
        self.assertEqual(0, evidence['serial_open_attempts']); self.assertEqual(0, evidence['serial_bytes_written'])
        self.assertIs(evidence['remote_files_written'], False)
        self.assertEqual({'process_exited': True, 'ssh_exit_code': 0, 'stderr': ''},
                         json.loads(Path(review['metadata_ssh_exit_path']).read_bytes()))
        for field in ('user_confirmed_intentional_powercycle', 'user_confirmed_x5_power_restored',
                      'user_confirmed_servo_battery_disconnected'):
            self.assertIs(review[field], True)

    def test_all_old_boot_fault_events_and_complete_review_survive_without_larger_review_bound(self):
        review = self.profile['review']
        previous = stream.load_reviewed_profile(Path(review['prior_reviewed_profile_path']),
                                                stream.IMU_RECOVERY_USB_PROFILE_SHA256)
        self.assertEqual(previous['historical_faults'], review['prior_boot_historical_faults'])
        self.assertEqual(previous['boot_id'], review['prior_boot_id'])
        raw_review = json.dumps(previous['review'], sort_keys=True, ensure_ascii=False,
                                separators=(',', ':'), allow_nan=False).encode()
        self.assertEqual(hashlib.sha256(raw_review).hexdigest(), review['prior_review_sha256'])
        self.assertIs(review['prior_review_retained_by_fixed_private_artifact'], True)
        self.assertLess(len(json.dumps(review, allow_nan=False).encode()), 32768)
        history = review['all_prior_boot_historical_faults']
        self.assertEqual(5, len(history)); self.assertEqual(30, review['all_prior_fault_events_count'])
        self.assertEqual([4, 2, 3, 13, 8], [len(row['historical_faults']) for row in history])
        for row in history:
            prior = stream.load_reviewed_profile(Path(row['reviewed_profile_path']), row['reviewed_profile_sha256'])
            self.assertEqual(prior['boot_id'], row['boot_id'])
            self.assertEqual(prior['historical_faults'], row['historical_faults'])
            self.assertIs(row['prior_faults_cleared'], False)
        for field in ('prior_boot_faults_cleared', 'previous_usb_faults_cleared', 'filesystem_repaired',
                      'prior_unexpected_reboot_root_cause_confirmed', 'electrical_safety_confirmed',
                      'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared', 'prior_imu_feedback_fault_cleared',
                      'powered_servo_health_certified', 'servo_motion_authorized', 'automatic_reconnect_authorized'):
            self.assertIs(review[field], False)

    def test_exact_profile_and_discovery_required_without_auto_new_boot_learning(self):
        with self.assertRaises(ValueError): stream.remote_source(self.expected())
        without_discovery = self.expected(); without_discovery.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(without_discovery, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        emitted = self.emitted()
        self.assertEqual(stream.INTENTIONAL_POWERCYCLE_BOOT_ID, emitted['boot_id'])
        self.assertEqual(stream.INTENTIONAL_POWERCYCLE_PROOF, emitted['reviewed_reconnect_proof'])
        proof = self.namespace['validate_kernel'](self.profile['kernel_prefix'], emitted)
        self.assertEqual(stream.INTENTIONAL_POWERCYCLE_HISTORICAL_FAULTS, proof['historical_faults'])
        self.assertIs(proof['historical_faults_cleared'], False)
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        for key in stream.PROFILE_FALSE_FLAGS: self.assertIs(self.profile[key], False)

    def test_new_faults_missing_history_raw_rewrite_or_any_identity_change_refused(self):
        emitted = self.emitted(); kernel = self.profile['kernel_prefix']
        for log in (kernel + '[ 60.0] usb 1-1.4: USB disconnect, device number 3\n',
                    kernel + '[ 60.1] mmc1: error -5\n',
                    kernel + '[ 60.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                    kernel + self.profile['historical_faults'][0]['line'] + '\n', kernel.split('\n', 1)[1]):
            with self.subTest(tail=log[-110:]), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](log, emitted)
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile, kernel_prefix=kernel.replace('\r\n', '\n')))
        metadata = {key: self.profile[key] for key in ('boot_id', 'cid', 'root_mount_line', 'model')}
        metadata['swaps'] = 'Filename\tType\tSize\tUsed\tPriority'
        self.namespace['validate_os'](metadata, emitted)
        for change in ({'boot_id': stream.IMU_RECOVERY_USB_BOOT_ID}, {'cid': '0' * 32},
                       {'model': 'another board'}, {'root_mount_line': 'another mount'}):
            with self.subTest(key=next(iter(change))), self.assertRaises(self.Error):
                self.namespace['validate_os'](dict(metadata, **change), emitted)

    def test_fixed_scope_powercycle_context_and_all_history_cannot_be_edited(self):
        changes = ({'boot_id': stream.IMU_RECOVERY_USB_BOOT_ID}, {'historical_faults': []},
                   {'allowed_requests': ['HELLO', 'STATE', 'ARM']},
                   {'review': dict(self.profile['review'], user_confirmed_intentional_powercycle=False)},
                   {'review': dict(self.profile['review'], user_confirmed_servo_battery_disconnected=False)},
                   {'review': dict(self.profile['review'], all_prior_boot_historical_faults=[])},
                   {'review': dict(self.profile['review'], prior_review_sha256='0' * 64)},
                   {'review': dict(self.profile['review'], electrical_safety_confirmed=True)})
        for change in changes:
            with self.subTest(key=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.IMU_RECOVERY_USB_PROFILE_SHA256)


if __name__ == '__main__':
    unittest.main()
