"""Offline verification of one manually reviewed IMU-recovery cold boot.

Only saved evidence and pure validators run; no device, reader or SSH starts.
"""
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


class ImuRecoveryBootReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = stream.EVIDENCE_DIR / 'feedback-imu-recovery-reboot-reviewed-20261006-01.json'
        cls.digest = stream.IMU_RECOVERY_RECONNECT_PROFILE_SHA256
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            cls.profile = stream.load_reviewed_profile(cls.path, cls.digest)
            cls.namespace = {}
            exec(compile(stream.runtime_source(), '<offline-guard-definitions>', 'exec'), cls.namespace)
        cls.Error = cls.namespace['ProbeError']

    def expected(self):
        return {**{key: copy.deepcopy(self.profile[key]) for key in (
            'boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'kernel_prefix', 'historical_faults')},
            'generation': 6100603, 'discover_serial': True}

    def emitted(self):
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No hardware process')):
            source = stream.remote_source(self.expected(), reviewed_profile_path=self.path,
                                          reviewed_profile_sha256=self.digest)
        tree = ast.parse(source)
        node = next(node for node in tree.body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        compile(source, '<prepared-not-executed-reader>', 'exec')
        return ast.literal_eval(node.value)

    def test_profile_keeps_exact_full_raw_kernel_and_two_stable_observations(self):
        review = self.profile['review']
        raw = Path(review['metadata_result_path']).read_bytes()
        self.assertEqual(review['metadata_result_sha256'], hashlib.sha256(raw).hexdigest())
        evidence = json.loads(raw)
        self.assertEqual(evidence['observations'][0], evidence['observations'][1])
        self.assertIs(evidence['stable_metadata'], True)
        observation = evidence['observations'][0]
        kernel = self.profile['kernel_prefix']
        self.assertEqual(observation['kernel_log'], kernel)
        self.assertEqual(64529, len(kernel.encode()))
        self.assertEqual(924, len(kernel.splitlines()))
        self.assertEqual(30, kernel.count('\r'))
        self.assertTrue(kernel.startswith('[    0.000000] Booting Linux'))
        self.assertEqual('a0cc0912ebd982ad206611c1015b30749a15b069eafe515d46a10f9df93a1959',
                         hashlib.sha256(kernel.encode()).hexdigest())
        self.assertEqual(['boot_id changed'], observation['identity_validation_errors'])
        self.assertEqual([], observation['serial_holders'])
        self.assertEqual({'device': 5, 'inode': 308, 'rdev': 48128}, observation['serial_node'])
        self.assertEqual(observation['config_mount'], review['observed_config_mount'])
        self.assertIn('shared:224', review['observed_config_mount'][0])
        self.assertEqual({'process_exited': True, 'ssh_exit_code': 0, 'stderr': ''},
                         json.loads(Path(review['metadata_ssh_exit_path']).read_bytes()))

    def test_previous_full_review_history_and_first_refusal_are_preserved(self):
        review = self.profile['review']
        old = stream.load_reviewed_profile(Path(review['prior_reviewed_profile_path']),
                                           stream.WIFI_RECONNECT_PROFILE_SHA256)
        self.assertEqual(old['review'], review['prior_review'])
        self.assertEqual(old['historical_faults'], review['prior_boot_historical_faults'])
        self.assertEqual(old['boot_id'], review['prior_boot_id'])
        for key in ('first_refusal_result', 'first_refusal_ssh_exit'):
            raw = Path(review[key + '_path']).read_bytes()
            self.assertEqual(review[key + '_sha256'], hashlib.sha256(raw).hexdigest())
        first = json.loads(Path(review['first_refusal_result_path']).read_bytes())
        self.assertEqual('fault', first['kind'])
        self.assertEqual('Reviewed boot/root/card/model identity changed', first['reason'])
        self.assertEqual(20, json.loads(Path(review['first_refusal_ssh_exit_path']).read_bytes())['ssh_exit_code'])
        self.assertIs(review['prior_boot_pin_refusal_retained'], True)
        for key in ('prior_boot_faults_cleared', 'filesystem_repaired', 'unexpected_reboot_root_cause_confirmed',
                    'prior_imu_feedback_fault_cleared', 'unsafe_feedback_hold_cleared', 'servo_fault_hold_cleared',
                    'prior_motion_occupancy_cleared', 'servo_motion_authorized', 'automatic_reconnect_authorized'):
            self.assertIs(review[key], False)
        # The previous Wi-Fi baseline is still independently supported.
        self.assertEqual('f2070288-4492-4ddc-9594-ad00640d01b7', stream.WIFI_RECONNECT_BOOT_ID)
        self.assertEqual('c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                         stream.WIFI_RECONNECT_PROFILE_SHA256)

    def test_explicit_profile_and_discovery_are_required_and_guard_scope_is_feedback_only(self):
        with self.assertRaises(ValueError):
            stream.remote_source(self.expected())
        expected = self.expected(); expected.pop('discover_serial')
        with self.assertRaises(ValueError):
            stream.remote_source(expected, reviewed_profile_path=self.path, reviewed_profile_sha256=self.digest)
        emitted = self.emitted()
        self.assertEqual(stream.IMU_RECOVERY_RECONNECT_BOOT_ID, emitted['boot_id'])
        self.assertEqual(stream.IMU_RECOVERY_RECONNECT_PROOF, emitted['reviewed_reconnect_proof'])
        result = self.namespace['validate_kernel'](self.profile['kernel_prefix'], emitted)
        self.assertEqual(stream.IMU_RECOVERY_RECONNECT_HISTORICAL_FAULTS, result['historical_faults'])
        self.assertIs(result['historical_faults_cleared'], False)
        self.assertEqual(['HELLO', 'STATE'], self.profile['allowed_requests'])
        for key in stream.PROFILE_FALSE_FLAGS:
            self.assertIs(self.profile[key], False)

    def test_new_faults_early_history_loss_raw_changes_and_identity_changes_fail_closed(self):
        emitted = self.emitted(); baseline = self.profile['kernel_prefix']
        for log in (baseline.split('\n', 1)[1], baseline + '[ 30.0] mmc1: error -5\n',
                    baseline + '[ 30.1] usb 1-1.4: USB disconnect, device number 3\n',
                    baseline + '[ 30.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                    baseline + self.profile['historical_faults'][0]['line'] + '\n'):
            with self.subTest(tail=log[-120:]), self.assertRaises(self.Error):
                self.namespace['validate_kernel'](log, emitted)
        with self.assertRaises(ValueError):
            stream._validate_reviewed_profile(dict(self.profile, kernel_prefix=baseline.replace('\r\n', '\n')))
        metadata = {key: self.profile[key] for key in ('boot_id', 'cid', 'root_mount_line', 'model')}
        metadata['swaps'] = 'Filename\tType\tSize\tUsed\tPriority'
        self.namespace['validate_os'](metadata, emitted)
        for change in ({'boot_id': stream.WIFI_RECONNECT_BOOT_ID}, {'cid': '0' * 32},
                       {'model': 'another board'}, {'root_mount_line': 'another mount'}):
            with self.subTest(key=next(iter(change))), self.assertRaises(self.Error):
                self.namespace['validate_os'](dict(metadata, **change), emitted)

    def test_edited_fixed_review_identity_scope_or_artifact_never_expands_allowances(self):
        changes = ({'boot_id': stream.WIFI_RECONNECT_BOOT_ID}, {'historical_faults': []},
                   {'allowed_requests': ['HELLO', 'STATE', 'ARM']},
                   {'review': dict(self.profile['review'], unexpected_reboot_root_cause_confirmed=True)},
                   {'review': dict(self.profile['review'], prior_review={})},
                   {'review': dict(self.profile['review'], observed_config_mount=[])},
                   {'review': dict(self.profile['review'], first_refusal_ssh_exit_code=0)})
        for change in changes:
            with self.subTest(key=next(iter(change))), self.assertRaises(ValueError):
                stream._validate_reviewed_profile(dict(self.profile, **change))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'edited.json'
            path.write_text(json.dumps(dict(self.profile, servo_motion_authorized=True)))
            path.chmod(0o600)
            with self.assertRaises(ValueError):
                stream.load_reviewed_profile(path, self.digest)
        with self.assertRaises(ValueError):
            stream.load_reviewed_profile(self.path, stream.WIFI_RECONNECT_PROFILE_SHA256)


if __name__ == '__main__':
    unittest.main()
