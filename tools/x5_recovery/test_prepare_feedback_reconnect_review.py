import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest import mock

import prepare_feedback_reconnect_review as review


class ReconnectProposalTests(unittest.TestCase):
    def setUp(self):
        base = review.feedback_stream.EVIDENCE_DIR
        self.prior_raw = (base / 'feedback-reboot-reviewed-20261004-01.json').read_bytes()
        self.prior = json.loads(self.prior_raw)
        self.result = json.loads((base / 'postflash-passive-after-reboot-20261004-01/result.json').read_bytes())
        self.connection = json.loads((base / 'postflash-passive-after-reboot-20261004-01/connection.json').read_bytes())
        # Normalize the pure success fixture; a separate test exercises the
        # actual capture tool's retained original-boot refusal.
        self.result.update(status='metadata_captured_requires_local_review', stop_reason=None)
        self.result['diagnostic_reply'].update(status='metadata_captured_requires_local_review',
                                               stop_reason=None, identity_validation_errors=[])
        self.connection.update(status='metadata_captured_requires_local_review', ssh_exit_code=0)
        self.event = '[ 9999.123456] usb 1-1.3: USB disconnect, device number 7'
        self.tail = [self.event, '[10001.100000] usb 1-1.3: new full-speed USB device number 8 using xhci-hcd',
                     '[10001.300000] usb 1-1.3: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64',
                     '[10001.400000] usb 1-1.3: ch341-uart converter now attached to ttyUSB0']
        self.set_log(self.prior['kernel_prefix'] + '\n'.join(self.tail) + '\n')

    def set_log(self, log):
        original, _ = review.classifiers()
        kernel = self.result['diagnostic_reply']['kernel_capture']
        kernel.update(kernel_log=log, kernel_log_sha256=hashlib.sha256(log.encode()).hexdigest(),
                      kernel_line_count=len(log.splitlines()), kernel_faults=[{'line': line, 'reason': reason}
                          for line in log.splitlines() if (reason := original(line))])

    def inspect(self):
        return review.inspect_snapshot(self.prior_raw, json.dumps(self.result).encode(),
                                       json.dumps(self.connection).encode())

    def test_proposal_requires_exact_review_and_preserves_history_identity_and_no_permissions(self):
        inspection = self.inspect()
        self.assertEqual(inspection['new_faults'], [{'line': self.event, 'reason': review.USB_REASON}])
        self.assertFalse(inspection['old_boot_pin_refusal_retained'])
        for lines in ([], [self.event + ' extra'], [self.event, self.event]):
            with self.assertRaises(ValueError):
                review.proposed_profile(inspection, lines, reviewed_at='2026-10-04', human_context='COM returned')
        raw = review.proposed_profile(inspection, [self.event], reviewed_at='2026-10-04', human_context='COM returned')
        profile = json.loads(raw)
        for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial', 'allowed_requests'):
            self.assertEqual(profile[key], self.prior[key])
        self.assertEqual(profile['historical_faults'][:2], self.prior['historical_faults'])
        for key in review.feedback_stream.PROFILE_FALSE_FLAGS:
            self.assertIs(profile[key], False)
        for key, value in self.prior['review'].items():
            self.assertEqual(profile['review'][key], value)
        self.assertFalse(profile['review']['same_boot_usb_reconnect_review']['unsafe_feedback_hold_cleared'])
        self.assertEqual(self.prior_raw, json.dumps(self.prior, ensure_ascii=False, indent=2).encode() + b'\n')

    def test_original_capture_old_boot_pin_refusal_is_retained_but_actual_same_boot_must_match(self):
        reply = self.result['diagnostic_reply']
        reply.update(status='stopped', stop_reason='Board boot identity changed',
                     identity_validation_errors=['Board boot identity changed'])
        self.result.update(status='stopped', stop_reason='Board boot identity changed')
        self.connection.update(status='stopped', ssh_exit_code=20)
        self.assertTrue(self.inspect()['old_boot_pin_refusal_retained'])
        reply['boot_id'] = '00000000-0000-0000-0000-000000000000'
        with self.assertRaisesRegex(ValueError, 'Same-boot'):
            self.inspect()

    def test_no_generic_usb_reset_other_topology_extra_disconnect_wifi_or_storage_allowance(self):
        variants = [self.event.replace('1-1.3', '1-1'), self.event + ' I/O error',
                    '[ 9999.123456] usb 1-1.3: reset full-speed USB device number 7 using xhci-hcd',
                    '[ 9999.123456] mmcblk1: I/O error',
                    '[ 9999.123456] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff']
        for line in variants:
            self.set_log(self.prior['kernel_prefix'] + line + '\n')
            with self.subTest(line=line), self.assertRaises(ValueError):
                self.inspect()
        self.set_log(self.prior['kernel_prefix'] + self.event + '\n' + self.event + '\n')
        with self.assertRaisesRegex(ValueError, 'Exactly one'):
            self.inspect()

    def test_complete_prefix_byte_hash_and_recorded_classifier_cannot_be_mutated(self):
        self.set_log(self.prior['kernel_prefix'][1:] + self.event + '\n')
        with self.assertRaisesRegex(ValueError, 'prefix'):
            self.inspect()
        self.set_log(self.prior['kernel_prefix'] + self.event + '\n')
        self.result['diagnostic_reply']['kernel_capture']['kernel_faults'].pop()
        with self.assertRaisesRegex(ValueError, 'classifier'):
            self.inspect()
        with self.assertRaises(ValueError):
            review.inspect_snapshot(self.prior_raw + b' ', b'{}', b'{}')

    def test_identity_scope_connection_stderr_mismatches_fail(self):
        mutations = [lambda r, c: r['diagnostic_reply']['root_card'].update(cid='0' * 32),
                     lambda r, c: r['diagnostic_reply']['root'].update(line='other root'),
                     lambda r, c: r['diagnostic_reply']['usb_devices'].append({'sysfs_name': '1-1.3'}),
                     lambda r, c: r['diagnostic_reply']['serial_sysfs_metadata'][0].update(device_number='188:1'),
                     lambda r, c: r['diagnostic_reply'].update(serial_opened=True),
                     lambda r, c: c.update(host_key_fingerprint='other'),
                     lambda r, c: c.update(ssh_process_closed=False),
                     lambda r, c: c.update(ssh_sessions_attempted=2)]
        for mutate in mutations:
            r, c = copy.deepcopy(self.result), copy.deepcopy(self.connection)
            mutate(r, c)
            with self.assertRaises(ValueError):
                review.inspect_snapshot(self.prior_raw, json.dumps(r).encode(), json.dumps(c).encode())
        with self.assertRaises(ValueError):
            review.inspect_snapshot(self.prior_raw, json.dumps(self.result).encode(),
                                    json.dumps(self.connection).encode(), b'warning')

    def test_cli_reader_rejects_device_and_outside_paths_before_any_open(self):
        for path in (Path('/dev/cu.usbserial-1110'), Path('/tmp/result.json')):
            with mock.patch.object(review.os, 'open') as opened:
                with self.assertRaises(ValueError):
                    review.read_private_local(path)
                opened.assert_not_called()


if __name__ == '__main__':
    unittest.main()
