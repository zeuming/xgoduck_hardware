"""Pure saved-evidence tests for an exact same-boot battery-off USB return."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import unittest
from unittest import mock

import feedback_stream as stream


class ImuRecoveryUsbReturnReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-imu-recovery-usb-return-reviewed-20261006-01.json'
        cls.digest = stream.IMU_RECOVERY_USB_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No device process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<offline-usb-guard-definitions>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self):
        return {**{key: copy.deepcopy(self.profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100604, 'discover_serial': True}

    def emitted(self):
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No device process')):
            source = stream.remote_source(self.expected(), reviewed_profile_path=self.path,
                                          reviewed_profile_sha256=self.digest)
        tree = ast.parse(source)
        node = next(node for node in tree.body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        compile(source, '<prepared-unlaunched-feedback>', 'exec')
        return ast.literal_eval(node.value)

    def test_full_raw_kernel_four_stable_observations_exact_ownership_and_window(self):
        review = self.profile['review']; observations = []
        for capture in review['metadata_captures']:
            for prefix in ('result', 'source', 'ssh_exit'):
                data = Path(capture[prefix + '_path']).read_bytes()
                self.assertEqual(capture[prefix + '_sha256'], hashlib.sha256(data).hexdigest())
            result = json.loads(Path(capture['result_path']).read_bytes())
            self.assertIs(result['stable_metadata'], True)
            self.assertEqual(0, result['serial_open_attempts'])
            self.assertEqual(0, result['serial_bytes_written'])
            self.assertIs(result['remote_files_written'], False)
            self.assertIs(result['motion_authorized'], False)
            observations.extend(result['observations'])
            self.assertEqual({'process_exited': True, 'ssh_exit_code': 0, 'stderr': ''},
                             json.loads(Path(capture['ssh_exit_path']).read_bytes()))
        self.assertEqual(4, len(observations))
        self.assertTrue(all(observation == observations[0] for observation in observations))
        one = observations[0]; kernel = self.profile['kernel_prefix']
        self.assertEqual(kernel, one['kernel_log'])
        self.assertEqual(65925, len(kernel.encode()))
        self.assertEqual(944, len(kernel.splitlines()))
        self.assertEqual(30, kernel.count('\r'))
        self.assertEqual('01d3a85c796ce8bb7ba0987ca06e0cad3db4304fefbd76e3731ad2f6fd4f0e9f',
                         hashlib.sha256(kernel.encode()).hexdigest())
        self.assertEqual({'device': 5, 'inode': 651, 'rdev': 48128}, one['serial_node'])
        self.assertEqual([], one['serial_holders']); self.assertEqual('5', one['usb_device_number'])
        window_data = Path(review['window_proof_path']).read_bytes()
        self.assertEqual(review['window_proof_sha256'], hashlib.sha256(window_data).hexdigest())
        self.assertEqual(review['window_proof'], json.loads(window_data))
        self.assertAlmostEqual(96.47230386734009, review['elapsed_seconds_from_capture_file_times'])
        self.assertIs(review['servo_battery_disconnected'], True)
        self.assertIs(review['user_touched_wires'], True)
        self.assertIs(review['user_com_movement_unknown'], True)

    def test_exact_prior_prefix_full_history_and_two_disconnects_remain_unresolved(self):
        review = self.profile['review']
        prior = stream.load_reviewed_profile(Path(review['prior_reviewed_profile_path']),
                                             stream.IMU_RECOVERY_RECONNECT_PROFILE_SHA256)
        self.assertEqual(prior['boot_id'], self.profile['boot_id'])
        self.assertEqual(prior['review'], review['prior_review'])
        self.assertTrue(self.profile['kernel_prefix'].startswith(prior['kernel_prefix']))
        self.assertEqual(924, review['retained_prior_line_count'])
        self.assertEqual(self.profile['kernel_prefix'].splitlines()[924:], review['new_kernel_lines'])
        self.assertEqual(20, len(review['new_kernel_lines']))
        self.assertEqual(prior['historical_faults'], self.profile['historical_faults'][:2])
        self.assertEqual(review['reviewed_new_usb_events'], self.profile['historical_faults'][2:])
        self.assertEqual(4, len(self.profile['historical_faults']))
        self.assertTrue(any('EMI?' in line for line in review['new_kernel_lines']))
        refusal = Path(review['quiescence_fault_refusal_path']).read_bytes()
        self.assertEqual(review['quiescence_fault_refusal_sha256'], hashlib.sha256(refusal).hexdigest())
        fault = json.loads(refusal)
        self.assertEqual('fault', fault['kind'])
        self.assertIn('"new_fault_count":2', fault['reason'])
        self.assertIs(review['quiescence_new_fault_refusal_retained'], True)
        for key in ('previous_usb_faults_cleared', 'prior_boot_faults_cleared', 'filesystem_repaired',
                    'usb_disconnect_electrical_cause_confirmed', 'electrical_safety_confirmed',
                    'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared', 'prior_imu_feedback_fault_cleared',
                    'native_service_deployment_authorized', 'servo_motion_authorized', 'automatic_reconnect_authorized'):
            self.assertIs(review[key], False)

    def test_only_explicit_feedback_scope_and_future_new_faults_still_fail_closed(self):
        with self.assertRaises(ValueError): stream.remote_source(self.expected())
        without_discovery = self.expected(); without_discovery.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(without_discovery, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        emitted = self.emitted(); kernel = self.profile['kernel_prefix']
        self.assertEqual(stream.IMU_RECOVERY_USB_PROOF, emitted['reviewed_reconnect_proof'])
        proof = self.namespace['validate_kernel'](kernel, emitted)
        self.assertEqual(self.profile['historical_faults'], proof['historical_faults'])
        self.assertIs(proof['historical_faults_cleared'], False)
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        for key in stream.PROFILE_FALSE_FLAGS: self.assertIs(self.profile[key], False)
        for log in (kernel + '[ 490.0] usb 1-1.4: USB disconnect, device number 5\n',
                    kernel + '[ 490.1] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                    kernel + '[ 490.2] mmc1: error -5\n', kernel.split('\n', 1)[1]):
            with self.subTest(tail=log[-100:]), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](log, emitted)

    def test_identity_scope_fixed_review_and_old_artifact_cannot_be_substituted(self):
        changes = ({'boot_id': stream.WIFI_RECONNECT_BOOT_ID}, {'cid': '0' * 32},
                   {'historical_faults': self.profile['historical_faults'][:2]},
                   {'allowed_requests': ['HELLO', 'STATE', 'STOP']},
                   {'review': dict(self.profile['review'], previous_usb_faults_cleared=True)},
                   {'review': dict(self.profile['review'], user_com_movement_unknown=False)},
                   {'review': dict(self.profile['review'], servo_battery_disconnected=False)},
                   {'review': dict(self.profile['review'], usb_disconnect_electrical_cause_confirmed=True)},
                   {'review': dict(self.profile['review'], prior_review={})})
        for change in changes:
            with self.subTest(key=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.IMU_RECOVERY_RECONNECT_PROFILE_SHA256)
        # Raw CRLF bytes are deliberately bound, independent of runtime splitlines comparison.
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile,
                kernel_prefix=self.profile['kernel_prefix'].replace('\r\n', '\n')))


if __name__ == '__main__':
    unittest.main()
