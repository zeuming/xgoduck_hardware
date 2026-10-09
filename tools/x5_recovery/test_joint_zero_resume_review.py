#!/usr/bin/env python3
"""Independent, local-only tests for selected-joint zero provenance on restart."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import feedback_calibration_server as host


def calibration():
    ids = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
    return dict(version=1, mechanical_reference_confirmed=True,
                joints=[dict(id=sid, zero=2000 + index, sign=-1,
                             min=1500, max=2500, direction_confirmed=True)
                        for index, sid in enumerate(ids)],
                imu=dict(axes=['-z', '-x', 'y'], axes_confirmed=True,
                         calibrated=True, bias_dps=[.1, -.2, .3]))


def digest(entry):
    payload = {key: value for key, value in entry.items() if key != 'sha256'}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def receipt(before, sid, zero, *, previous=None, sequence=1):
    after = copy.deepcopy(before)
    joint = next(item for item in after['joints'] if item['id'] == sid)
    joint.update(zero=zero, min=None, max=None, direction_confirmed=False)
    entry = dict(sequence=sequence, id=sid, generation=7,
                 previous_sha256=previous, before=copy.deepcopy(before), after=after)
    entry['sha256'] = digest(entry)
    return entry


def history(original, entries):
    return dict(version=1,
                original_zeros=[dict(id=item['id'], zero=item['zero'])
                                for item in original['joints']],
                entries=copy.deepcopy(entries))


class ZeroResumeReviewTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.run = self.root / 'feedback-calibration-zero-review'
        self.run.mkdir(mode=0o700)
        self.original = calibration()
        self.first = receipt(self.original, 12, 1770)
        self.ledger = history(self.original, [self.first])
        self.receipt_path = self.run / 'joint-zero-provenance.json'
        self.recovery = mock.patch.object(host, 'RECOVERY', self.root)
        self.recovery.start()
        self.hardware = mock.patch.object(host.subprocess, 'Popen',
                                         side_effect=AssertionError('No hardware in resume tests'))
        self.hardware.start()

    def tearDown(self):
        self.hardware.stop()
        self.recovery.stop()
        self.temporary.cleanup()

    def write(self, current=None, ledger=None):
        current = self.first['after'] if current is None else current
        host.atomic_calibration(self.run, current)
        value = self.ledger if ledger is None else ledger
        self.receipt_path.write_text(json.dumps(value, ensure_ascii=False) + '\n')
        self.receipt_path.chmod(0o600)

    def load(self):
        return host.load_resume_calibration(self.run, self.original,
                                            include_zero_provenance=True)

    def test_one_selected_joint_change_survives_restart_with_complete_other_data(self):
        self.write()
        current, raw, ledger = self.load()
        self.assertEqual(current, self.first['after'])
        self.assertEqual(json.loads(raw), current)
        self.assertEqual(ledger, self.ledger)
        self.assertEqual(current['joints'][:2], self.original['joints'][:2])
        self.assertEqual(current['joints'][3:], self.original['joints'][3:])
        self.assertEqual(current['imu'], self.original['imu'])
        self.assertEqual(current['joints'][2], dict(self.original['joints'][2],
                         zero=1770, min=None, max=None, direction_confirmed=False))

    def test_multiple_explicit_zero_saves_form_a_restartable_chain(self):
        second = receipt(self.first['after'], 13, 1780,
                         previous=self.first['sha256'], sequence=2)
        self.write(second['after'], history(self.original, [self.first, second]))
        current, _, ledger = self.load()
        self.assertEqual(current, second['after'])
        self.assertEqual(len(ledger['entries']), 2)

    def test_same_zero_recapture_can_invalidate_only_its_selected_direction_and_range(self):
        entry = receipt(self.original, 12, self.original['joints'][2]['zero'])
        self.write(entry['after'], history(self.original, [entry]))
        current, _, ledger = self.load()
        self.assertEqual(current, entry['after'])
        self.assertEqual(len(ledger['entries']), 1)

    def test_authorized_zero_allows_later_ordinary_direction_and_imu_calibration(self):
        current = copy.deepcopy(self.first['after'])
        current['joints'][2].update(sign=1, min=1600, max=2200, direction_confirmed=True)
        current['imu']['bias_dps'] = [.2, -.1, .4]
        self.write(current)
        resumed, _, _ = self.load()
        self.assertEqual(resumed, current)

    def test_missing_provenance_cannot_authorize_changed_zero(self):
        host.atomic_calibration(self.run, self.first['after'])
        with self.assertRaises(ValueError):
            self.load()

    def test_interrupted_terminal_intent_keeps_the_previous_zero_and_trims_only_tail(self):
        self.write(self.original)
        current, _, normalized = self.load()
        self.assertEqual(current, self.original)
        self.assertEqual(normalized['entries'], [])
        self.assertEqual(len(json.loads(self.receipt_path.read_text())['entries']), 1)

    def test_interrupted_second_intent_retains_the_first_committed_authorization(self):
        second = receipt(self.first['after'], 13, 1780,
                         previous=self.first['sha256'], sequence=2)
        self.write(self.first['after'], history(self.original, [self.first, second]))
        _, _, normalized = self.load()
        self.assertEqual(normalized['entries'], [self.first])

    def test_two_uncommitted_intents_cannot_be_trimmed_to_original(self):
        second = receipt(self.first['after'], 13, 1780,
                         previous=self.first['sha256'], sequence=2)
        self.write(self.original, history(self.original, [self.first, second]))
        with self.assertRaises(ValueError):
            self.load()

    def test_recomputed_hash_cannot_authorize_another_joint_change(self):
        changed = copy.deepcopy(self.first)
        changed['after']['joints'][0]['zero'] += 1
        changed['sha256'] = digest(changed)
        self.write(changed['after'], history(self.original, [changed]))
        with self.assertRaises(ValueError):
            self.load()

    def test_recomputed_hash_cannot_authorize_imu_or_global_reference_change(self):
        for target in ('imu', 'reference'):
            with self.subTest(target=target):
                changed = copy.deepcopy(self.first)
                if target == 'imu':
                    changed['after']['imu']['bias_dps'][0] += .1
                else:
                    changed['after']['mechanical_reference_confirmed'] = False
                changed['sha256'] = digest(changed)
                self.write(changed['after'], history(self.original, [changed]))
                with self.assertRaises(ValueError):
                    self.load()

    def test_recomputed_hash_cannot_change_selected_sign(self):
        changed = copy.deepcopy(self.first)
        changed['after']['joints'][2]['sign'] = 1
        changed['sha256'] = digest(changed)
        self.write(changed['after'], history(self.original, [changed]))
        with self.assertRaises(ValueError):
            self.load()

    def test_recomputed_hash_cannot_erase_selected_zero(self):
        changed = copy.deepcopy(self.first)
        changed['after']['joints'][2]['zero'] = None
        changed['sha256'] = digest(changed)
        self.write(changed['after'], history(self.original, [changed]))
        with self.assertRaises(ValueError):
            self.load()

    def test_original_anchor_cannot_be_replaced_by_the_changed_zero(self):
        ledger = copy.deepcopy(self.ledger)
        ledger['original_zeros'][2]['zero'] = 1770
        self.write(ledger=ledger)
        with self.assertRaises(ValueError):
            self.load()

    def test_sequence_generation_and_previous_digest_fail_closed(self):
        for key, value in (('sequence', True), ('generation', True), ('generation', 0),
                           ('generation', 0x80000000), ('id', True),
                           ('previous_sha256', '0' * 64)):
            with self.subTest(key=key, value=value):
                changed = copy.deepcopy(self.first)
                changed[key] = value
                changed['sha256'] = digest(changed)
                self.write(ledger=history(self.original, [changed]))
                with self.assertRaises(ValueError):
                    self.load()

    def test_hash_corruption_unknown_fields_and_duplicate_json_fields_are_rejected(self):
        self.write()
        ledger = copy.deepcopy(self.ledger)
        ledger['entries'][0]['sha256'] = '0' * 64
        self.write(ledger=ledger)
        with self.assertRaises(ValueError):
            self.load()
        ledger = copy.deepcopy(self.ledger)
        ledger['authorized'] = True
        self.write(ledger=ledger)
        with self.assertRaises(ValueError):
            self.load()
        self.write()
        raw = self.receipt_path.read_text().replace('"version": 1', '"version": 1, "version": 1', 1)
        self.receipt_path.write_text(raw)
        with self.assertRaises(ValueError):
            self.load()

    def test_unrelated_latest_zero_is_never_accepted(self):
        current = copy.deepcopy(self.first['after'])
        current['joints'][3]['zero'] += 1
        self.write(current)
        with self.assertRaises(ValueError):
            self.load()

    def test_provenance_file_must_be_private_and_nonlinked(self):
        self.write()
        self.receipt_path.chmod(0o644)
        with self.assertRaises(ValueError):
            self.load()
        self.receipt_path.chmod(0o600)
        linked = self.run / 'linked-receipt.json'
        os.link(self.receipt_path, linked)
        with self.assertRaises(ValueError):
            self.load()
        linked.unlink()
        self.receipt_path.rename(linked)
        self.receipt_path.symlink_to(linked)
        with self.assertRaises(OSError):
            self.load()


class ZeroCaptureReviewTests(unittest.TestCase):
    """Independent safety checks use only synthetic feedback and temporary files."""
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.now, self.sample, self.sequence = 20., 1000000, 0
        self.original = calibration()
        host.atomic_calibration(self.directory, self.original)
        self.original_bytes = (self.directory / 'calibration-latest.json').read_bytes()
        self.session = host.CalibrationSession(self.original, self.directory, 7, lambda: self.now)
        self.hardware = mock.patch.object(host.subprocess, 'Popen',
                                         side_effect=AssertionError('No hardware in capture tests'))
        self.hardware.start()
        self.session.accept(dict(kind='ready', generation=7, emitted_monotonic=self.now,
                                 reader_elapsed_seconds=0.))
        self.feed()

    def tearDown(self):
        self.hardware.stop()
        self.temporary.cleanup()

    def feed(self, raw=1770, **changes):
        self.now += .02
        self.sample = (self.sample + 20000) & 0xffffffff
        self.sequence += 1
        positions = [item['zero'] for item in self.session.config['joints']]
        positions[2] = raw
        state = dict(sample_us=self.sample, imu_us=(self.sample - 1000) & 0xffffffff,
                     online_mask=32767, fault_mask=0, torque_mask=0,
                     armed=False, arm_cookie=0, imu_ok=True, last_error=0,
                     positions=positions, accel_ms2=[0., 0., 9.81], gyro_dps=[0., 0., 0.])
        state.update(changes)
        self.session.accept(dict(kind='state', generation=7, sequence=self.sequence,
                                 received_monotonic=self.now,
                                 reader_elapsed_seconds=self.now - self.session.started, state=state))

    def action(self, action, **changes):
        body = dict(generation=7, id=12, action=action,
                    support_confirmed=True, zero_reference_confirmed=True)
        body.update(changes)
        return self.session.joint_zero_action(body)

    def candidate(self):
        self.action('capture')
        for _ in range(20):
            self.feed()
        self.assertEqual(self.session.joint_zero['stage'], 'ready')
        self.assertEqual(self.session.joint_zero['candidate_zero'], 1770)

    def unchanged(self):
        self.assertEqual((self.directory / 'calibration-latest.json').read_bytes(), self.original_bytes)
        self.assertFalse((self.directory / 'joint-zero-provenance.json').exists())

    def test_capture_is_preview_only_and_one_frame_cannot_complete_it(self):
        self.action('capture')
        self.feed()
        self.assertEqual(self.session.joint_zero['stage'], 'sampling')
        self.assertIsNone(self.session.joint_zero['candidate_zero'])
        self.unchanged()
        for _ in range(19):
            self.feed()
        self.assertEqual(self.session.joint_zero['stage'], 'ready')
        self.unchanged()

    def test_candidate_cannot_be_saved_after_other_calibration_changes(self):
        self.candidate()
        self.session.config['imu']['bias_dps'][0] = .2
        with self.assertRaises(ValueError):
            self.action('save')
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        self.unchanged()

    def test_encoder_motion_invalidates_ready_candidate_before_save(self):
        self.candidate()
        self.feed(raw=1780)
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        with self.assertRaises(ValueError):
            self.action('save')
        self.unchanged()

    def test_current_voltage_fault_invalidates_candidate_and_leaves_files_untouched(self):
        self.candidate()
        # run_reader catches a rejected STATE and faults this same session.
        with self.assertRaises(ValueError) as rejected:
            self.feed(last_error=1, fault_mask=32767)
        self.session.fault(str(rejected.exception))
        self.assertEqual(self.session.status, 'fault')
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        with self.assertRaises(ValueError):
            self.action('save')
        self.unchanged()

    def test_stale_feedback_and_changed_generation_cannot_commit_candidate(self):
        self.candidate()
        with self.assertRaises(ValueError):
            self.action('save', generation=6)
        self.now += .5
        with self.assertRaises(ValueError):
            self.action('save')
        self.unchanged()

    def test_imu_only_recheck_or_old_unsafe_hold_cannot_start_zero_capture(self):
        self.session.feedback_scope = 'imu'
        with self.assertRaises(ValueError):
            self.action('capture')
        self.session.feedback_scope = None
        self.session.recovery_recheck = dict(message='old hold', technical_reason='unsafe')
        with self.assertRaises(ValueError):
            self.action('capture')
        self.unchanged()

    def test_successful_save_invalidates_only_selected_range_and_new_zero_can_resume(self):
        self.candidate()
        self.action('save')
        expected = copy.deepcopy(self.original)
        expected['joints'][2].update(zero=1770, min=None, max=None, direction_confirmed=False)
        self.assertEqual(self.session.config, expected)
        self.assertEqual(self.session.original_zeros, host.joint_calibration.zero_signature(expected))
        ledger = json.loads((self.directory / 'joint-zero-provenance.json').read_bytes())
        self.assertEqual(host.validated_zero_provenance(ledger, self.original, expected), ledger)


if __name__ == '__main__':
    unittest.main()
