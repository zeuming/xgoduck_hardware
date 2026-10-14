#!/usr/bin/env python3
"""Selected zero recapture tests: fake feedback and temporary Mac files only."""
import copy
from email.message import Message
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import argparse,sys
parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);args,remaining=parser.parse_known_args()
sys.argv=[sys.argv[0]]+remaining
sys.path[:0]=[str(args.candidate/'service'),str(Path(__file__).resolve().parents[1]/'x5_recovery')]
import feedback_calibration_server as host
import test_feedback_calibration_server as fixtures


class SelectedZeroTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.clock = fixtures.Clock()
        self.original = fixtures.config()
        self.original['mechanical_reference_confirmed'] = True
        self.original['imu'].update(axes=['-z', '-x', 'y'], axes_confirmed=True,
                                    calibrated=True, bias_dps=[.1, -.2, .3])
        for index in (0, 1):
            joint = self.original['joints'][index]
            joint.update(direction_confirmed=True, min=joint['zero'] - 200,
                         max=joint['zero'] + 200)
        host.atomic_calibration(self.directory, self.original)
        self.session = host.CalibrationSession(self.original, self.directory, 7, self.clock)
        self.session.process = fixtures.FakeProcess()
        self.controls = []
        self.control_writer = mock.patch.object(host, 'write_pipe_once',
                                                side_effect=lambda *args: self.controls.append(args))
        self.control_writer.start()
        self.forbid_process = mock.patch.object(host.subprocess, 'Popen',
                                               side_effect=AssertionError('No hardware in pure tests'))
        self.forbid_process.start()
        self.sequence, self.sample = 1, 1000000
        self.session.accept(dict(kind='ready', generation=7, emitted_monotonic=self.clock(),
                                 reader_elapsed_seconds=0.))
        self.feed(1800)

    def tearDown(self):
        self.forbid_process.stop()
        self.control_writer.stop()
        self.temporary.cleanup()

    def body(self, action, sid=12, **changes):
        body = dict(generation=self.session.generation, id=sid, action=action,
                    support_confirmed=True, zero_reference_confirmed=True)
        body.update(changes)
        return body

    def feed(self, raw=1800, sid=12, advance=20000, imu_advance=None):
        previous_imu = self.session.feedback['imu_us'] if self.session.feedback else self.sample - 1000
        self.sample = (self.sample + advance) & 0xffffffff
        self.clock.now += max(20000, advance) / 1000000
        self.sequence += 1
        state = dict(sample_us=self.sample, imu_us=(self.sample - 1000) & 0xffffffff,
                     online_mask=32767, fault_mask=0, torque_mask=0, armed=False,
                     arm_cookie=0, imu_ok=True, last_error=0, accel_ms2=[0., 0., 9.81],
                     gyro_dps=[0., 0., 0.], positions=[joint['zero'] for joint in self.session.config['joints']])
        if imu_advance is not None:
            state['imu_us'] = (previous_imu + imu_advance) & 0xffffffff
        state['positions'][host.joint_calibration.IDS.index(sid)] = raw
        self.session.accept(dict(kind='state', generation=self.session.generation, sequence=self.sequence,
                                 state=state, received_monotonic=self.clock(),
                                 reader_elapsed_seconds=self.clock() - self.session.started))

    def capture(self, raw=1800, sid=12):
        self.feed(raw, sid)
        self.session.joint_zero_action(self.body('capture', sid))
        for _ in range(20):
            self.feed(raw, sid)
        self.assertEqual(self.session.joint_zero['stage'], 'ready')

    def saved(self):
        return host.strict_json((self.directory / 'calibration-latest.json').read_bytes())

    def test_capture_is_preview_only_and_save_changes_only_selected_zero_and_invalidates_selected_limits(self):
        before = copy.deepcopy(self.session.config)
        self.capture(1815)
        preview = self.session.snapshot()['joint_zero']
        self.assertEqual(preview['id'], 12)
        self.assertEqual(preview['old_zero'], before['joints'][2]['zero'])
        self.assertEqual(preview['candidate_zero'], 1815)
        self.assertEqual(preview['sample_count'], 1)
        self.assertEqual(self.saved(), before)
        self.assertEqual(self.session.config, before)
        self.assertEqual(self.controls, [])
        original_audit = copy.deepcopy(self.session.original_joints)
        result = self.session.joint_zero_action(self.body('save'))
        expected = copy.deepcopy(before)
        expected['joints'][2].update(zero=1815, min=None, max=None, direction_confirmed=False)
        self.assertEqual(self.saved(), expected)
        self.assertEqual(self.session.config, expected)
        self.assertEqual(self.session.original_joints, original_audit)
        self.assertEqual(self.session.original_zeros, host.joint_calibration.zero_signature(expected))
        self.assertEqual(result['joint_zero']['stage'], 'saved')
        self.assertEqual(result['session']['status'], 'active')
        self.assertEqual(self.controls, [])
        self.assertEqual(len(list(self.directory.glob('joint-zero-authorized-*.json'))), 1)

    def test_explicit_capture_uses_one_advancing_sample_without_automatic_save(self):
        self.session.joint_zero_action(self.body('capture'))
        self.feed(1800)
        self.assertEqual(self.session.joint_zero['stage'],'ready')
        self.assertEqual(self.session.joint_zero['candidate_zero'],1800)
        self.assertEqual(self.session.joint_zero['sample_count'],1)
        self.assertEqual(self.saved(),self.original)

    def test_repeated_imu_sample_never_counts_as_new_zero_proof(self):
        self.session.joint_zero_action(self.body('capture'))
        for _ in range(4):
            self.feed(1800, imu_advance=0)
        self.assertEqual(self.session.joint_zero['sample_count'], 0)
        self.assertEqual(self.session.joint_zero['stage'], 'sampling')
        self.assertEqual(self.saved(), self.original)

    def test_unstable_capture_cross_wrap_gap_and_clock_reset_fail_without_config_change(self):
        for mode in ('wrap', 'gap', 'reset'):
            with self.subTest(mode=mode):
                self.session.joint_zero_action(self.body('cancel'))
                self.feed(1800)
                self.session.joint_zero_action(self.body('capture'))
                if mode == 'moving':
                    for index in range(20):
                        self.feed(1800 + (index % 2) * 8)
                elif mode == 'wrap':
                    self.feed(4000)
                elif mode == 'gap':
                    self.feed(1800, advance=150001)
                else:
                    self.feed(1800, advance=-1)
                self.assertEqual(self.session.joint_zero['stage'], 'failed')
                self.assertIsNone(self.session.joint_zero['candidate_zero'])
                self.assertEqual(self.saved(), self.original)

    def test_save_still_requires_fresh_feedback_and_explicit_confirmation(self):
        self.capture()
        self.clock.now += .501
        with self.assertRaisesRegex(ValueError,'过期'):
            self.session.joint_zero_action(self.body('save'))
        self.assertEqual(self.saved(),self.original)

    def test_later_movement_does_not_change_captured_preview_or_autosave(self):
        self.capture()
        self.feed(1807)
        self.assertEqual(self.session.joint_zero['stage'],'ready')
        self.assertEqual(self.session.joint_zero['candidate_zero'],1800)
        self.assertEqual(self.saved(),self.original)
        self.session.joint_zero_action(self.body('cancel'))
        self.assertEqual(self.saved(),self.original)

    def test_atomic_save_failure_preserves_memory_file_and_retry_reuses_verified_proposal(self):
        self.capture()
        with mock.patch.object(host, 'atomic_calibration', side_effect=OSError('disk refused')):
            with self.assertRaisesRegex(ValueError, '旧校准保持不变'):
                self.session.joint_zero_action(self.body('save'))
        self.assertEqual(self.saved(), self.original)
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.session.joint_zero['stage'], 'ready')
        self.assertEqual(host.validated_zero_provenance(self.session.zero_provenance,
                                                        self.original, self.original)['entries'], [])
        self.session.joint_zero_action(self.body('save'))
        self.assertEqual(self.saved()['joints'][2]['zero'], 1800)
        self.assertEqual(len(self.session.zero_provenance['entries']), 1)
        self.assertEqual(len(list(self.directory.glob('joint-zero-authorized-*.json'))), 2)

    def test_receipt_failure_never_commits_zero(self):
        self.capture()
        with mock.patch.object(host, 'atomic_zero_provenance', side_effect=OSError('receipt refused')):
            with self.assertRaisesRegex(ValueError, '旧校准保持不变'):
                self.session.joint_zero_action(self.body('save'))
        self.assertEqual(self.saved(), self.original)
        self.assertEqual(self.session.config, self.original)
        self.assertEqual(self.session.original_zeros, host.joint_calibration.zero_signature(self.original))

    def test_cancel_fault_close_and_generation_change_discard_only_draft(self):
        self.capture()
        self.session.joint_zero_action(self.body('cancel', support_confirmed=False,
                                                   zero_reference_confirmed=False))
        self.assertEqual(self.session.joint_zero['stage'], 'cancelled')
        self.assertIsNone(self.session.zero_collection)
        self.capture()
        self.session.fault('test stopped')
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        self.assertIsNone(self.session.zero_collection)
        self.assertEqual(self.saved(), self.original)

    def test_close_and_new_recovery_generation_cannot_save_the_old_candidate(self):
        self.capture()
        self.session.accept(dict(kind='closed', generation=7, serial_closed=True, normal=False,
                                 reason='test closed'))
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        self.assertIsNone(self.session.zero_collection)
        self.session.process.returncode = 255
        self.session.connection_done.set()
        self.session.recovery_launcher = mock.Mock()
        with mock.patch.object(host.threading, 'Thread'):
            self.session.recover_action({'generation': 7})
        self.assertEqual(self.session.joint_zero['stage'], 'idle')
        self.assertNotEqual(self.session.generation, 7)
        with self.assertRaises(ValueError):
            self.session.joint_zero_action(self.body('save', generation=7))
        self.assertEqual(self.saved(), self.original)

    def test_rezero_completed_joint_retains_sign_and_other_completed_records(self):
        self.session.config['joints'][1]['sign'] = -1
        host.atomic_calibration(self.directory, self.session.config)
        before = copy.deepcopy(self.session.config)
        self.capture(1850, sid=11)
        self.session.joint_zero_action(self.body('save', sid=11))
        selected = self.saved()['joints'][1]
        self.assertEqual(selected['sign'], -1)
        self.assertEqual(selected['zero'], 1850)
        self.assertFalse(selected['direction_confirmed'])
        self.assertIsNone(selected['min'])
        self.assertIsNone(selected['max'])
        for index, joint in enumerate(self.saved()['joints']):
            if index != 1:
                self.assertEqual(joint, before['joints'][index])
        self.assertEqual(self.saved()['imu'], before['imu'])

    def test_config_change_after_capture_and_different_selected_id_block_save(self):
        self.capture()
        with self.assertRaises(ValueError):
            self.session.joint_zero_action(self.body('save', 13))
        self.session.config['imu']['bias_dps'][0] += .01
        with self.assertRaisesRegex(ValueError, '配置已改变'):
            self.session.joint_zero_action(self.body('save'))
        self.assertEqual(self.session.joint_zero['stage'], 'failed')
        self.assertEqual(self.saved(), self.original)

    def test_consecutive_explicit_recaptures_build_a_selected_change_chain(self):
        self.capture(1800)
        self.session.joint_zero_action(self.body('save'))
        self.capture(1700, sid=13)
        self.session.joint_zero_action(self.body('save', sid=13))
        ledger = host.strict_json((self.directory / 'joint-zero-provenance.json').read_bytes())
        validated = host.validated_zero_provenance(ledger, self.original, self.saved())
        self.assertEqual([entry['id'] for entry in validated['entries']], [12, 13])
        self.assertEqual(validated['entries'][1]['previous_sha256'], validated['entries'][0]['sha256'])
        self.assertEqual(self.saved()['joints'][:2], self.original['joints'][:2])

    def test_request_body_generation_id_and_confirmations_are_strict(self):
        valid = self.body('capture')
        for changes in (dict(id=True), dict(id=99), dict(generation=True), dict(generation=6),
                        dict(action='zero'), dict(zero=1), dict(support_confirmed='yes'),
                        dict(zero_reference_confirmed=1), dict(support_confirmed=False),
                        dict(zero_reference_confirmed=False)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.session.joint_zero_action(dict(valid, **changes))
        self.assertEqual(self.saved(), self.original)

    def test_unsafe_partial_feedback_imu_scope_and_maintenance_holds_block_capture(self):
        valid = copy.deepcopy(self.session.feedback)
        for fields in (dict(online_mask=0), dict(online_mask=1), dict(fault_mask=1),
                       dict(torque_mask=1), dict(armed=True), dict(arm_cookie=1),
                       dict(imu_ok=False), dict(last_error=1), dict(last_error=None)):
            self.session.feedback = dict(valid, **fields)
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.session.joint_zero_action(self.body('capture'))
        self.session.feedback = valid
        for attribute, value in (('feedback_scope', 'imu'), ('maintenance_mapping_changed', True),
                                 ('maintenance_fault_hold', True),
                                 ('recovery_recheck', dict(message='held', technical_reason=host.UNSAFE_FEEDBACK_REASON))):
            previous = getattr(self.session, attribute)
            setattr(self.session, attribute, value)
            with self.subTest(attribute=attribute), self.assertRaises(ValueError):
                self.session.joint_zero_action(self.body('capture'))
            setattr(self.session, attribute, previous)
        self.assertEqual(self.saved(), self.original)

    def test_ordinary_draft_and_zero_draft_cannot_overlap(self):
        self.session.joint_action(dict(generation=7, id=12, action='direction-start',
                                       support_confirmed=True, positive_confirmed=True))
        with self.assertRaisesRegex(ValueError, '取消'):
            self.session.joint_zero_action(self.body('capture'))
        self.session.joint_action(dict(generation=7, id=12, action='cancel',
                                       support_confirmed=False, positive_confirmed=False))
        self.capture()
        with self.assertRaisesRegex(ValueError, '零点预览'):
            self.session.joint_action(dict(generation=7, id=12, action='direction-start',
                                           support_confirmed=True, positive_confirmed=True))
        with self.assertRaisesRegex(ValueError, '其他关节'):
            self.session.joint_zero_action(self.body('cancel', 13))

    def test_same_id_capture_can_be_saved_then_followed_by_normal_direction_range_save(self):
        self.capture(1800)
        self.session.joint_zero_action(self.body('save'))
        for action, raw in (('direction-start', 1800), ('direction-end', 1750),
                            ('range-a', 1450), ('range-b', 2250)):
            self.feed(raw)
            self.session.joint_action(dict(generation=7, id=12, action=action,
                                           support_confirmed=True, positive_confirmed=True))
            for _ in range(20):
                self.feed(raw)
        self.session.joint_action(dict(generation=7, id=12, action='save',
                                       support_confirmed=True, positive_confirmed=True))
        self.assertEqual(self.saved()['joints'][2]['zero'], 1800)
        self.assertTrue(self.saved()['joints'][2]['direction_confirmed'])
        self.assertEqual(self.saved()['joints'][:2], self.original['joints'][:2])

    def test_handler_uses_existing_origin_token_gate_and_dispatches_only_validated_body(self):
        handler = host.handler_for(self.session, 8767)
        instance = handler.__new__(handler)
        instance.path = '/api/joint-zero'
        payload = json.dumps(self.body('capture')).encode()
        instance.headers = Message()
        for key, value in dict(Host='127.0.0.1:8767', Origin='http://127.0.0.1:8767',
                              **{'X-CSRF-Token': self.session.token, 'Content-Type': 'application/json',
                                 'Content-Length': str(len(payload))}).items():
            instance.headers[key] = value
        instance.rfile = io.BytesIO(payload)
        replies = []
        instance.reply = lambda code, value: replies.append((code, value))
        instance.do_POST()
        self.assertEqual(replies[-1][0], 200)
        self.assertEqual(replies[-1][1]['joint_zero']['stage'], 'sampling')
        instance.headers.replace_header('Origin', 'http://evil.invalid')
        instance.rfile = io.BytesIO(payload)
        instance.do_POST()
        self.assertEqual(replies[-1][0], 403)
        instance.headers.replace_header('Origin', 'http://127.0.0.1:8767')
        instance.headers.replace_header('X-CSRF-Token', 'incorrect')
        instance.rfile = io.BytesIO(payload)
        instance.do_POST()
        self.assertEqual(replies[-1][0], 403)


if __name__ == '__main__':
    unittest.main()
