import struct
import unittest

from maintenance_diagnostics import decode_center_failure, describe_center_failure


def failure_ack(*, kind=12, stage=19, sid=31, found=False, invalid=False,
                outcome=6, actual=None, received=b''):
    raw = bytearray(101)
    raw[:6] = bytes((kind, stage, 1, sid, 42 if stage == 18 else 40, 2 if stage == 18 else 1))
    struct.pack_into('<H', raw, 6, 2185 if stage == 18 else 1 if stage == 21 else 0)
    raw[8:13] = bytes((found, invalid, outcome, 0, raw[5] if found else 0))
    if found:
        raw[13:13 + raw[5]] = actual.to_bytes(raw[5], 'little')
    struct.pack_into('<III', raw, 19, 1502, 1, 1)
    struct.pack_into('<H', raw, 33, len(received))
    raw[36] = min(len(received), 48)
    raw[53:53 + raw[36]] = received[:48]
    return bytes(raw)


class DiagnosticTests(unittest.TestCase):
    def test_echo_only_timeout_is_distinct_from_bad_reply_and_wrong_value(self):
        echo = bytes.fromhex('ffff1f04022801b1')
        value = decode_center_failure(failure_ack(received=echo), 31)
        self.assertFalse(value['found'])
        self.assertEqual(value['received_hex'], echo.hex())
        self.assertIn('超时', describe_center_failure(value))
        value = decode_center_failure(failure_ack(invalid=True, received=echo + b'\xff'), 31)
        self.assertIn('不完整', describe_center_failure(value))
        value = decode_center_failure(failure_ack(found=True, outcome=1, actual=1), 31)
        self.assertEqual(value['actual'], 1)
        self.assertIn('值 1，要求 0', describe_center_failure(value))

    def test_noise_before_valid_zero_is_not_a_success(self):
        value = decode_center_failure(failure_ack(found=True, invalid=True, outcome=1, actual=0), 31)
        self.assertTrue(value['invalid_reply_seen'])
        self.assertIn('有效应答值 0', describe_center_failure(value))
        with self.assertRaisesRegex(ValueError, 'Successful read'):
            decode_center_failure(failure_ack(found=True, outcome=1, actual=0), 31)

    def test_bounded_capture_reports_truncation_without_optimistic_decode(self):
        received = bytes(range(70))
        value = decode_center_failure(failure_ack(invalid=True, received=received), 31)
        self.assertTrue(value['received_truncated'])
        self.assertEqual(value['received_count'], 70)
        self.assertEqual(bytes.fromhex(value['received_hex']), received[:48])

    def test_rejects_wrong_id_bad_version_count_padding_and_inconsistent_status(self):
        good = failure_ack()
        for offset, byte in ((0, 11), (1, 20), (2, 2), (3, 30), (4, 42),
                             (5, 2), (6, 1), (8, 2), (9, 2), (10, 1),
                             (12, 1), (35, 1), (36, 1), (53, 1)):
            raw = bytearray(good); raw[offset] = byte
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                decode_center_failure(bytes(raw), 31)
        for raw in (good[:-1], good + b'\0', list(good)):
            with self.assertRaises(ValueError): decode_center_failure(raw, 31)

    def test_check_mode_cannot_claim_it_attempted_to_enable_torque(self):
        value = decode_center_failure(failure_ack(kind=16), 31)
        self.assertEqual(value['request_type'], 16)
        with self.assertRaisesRegex(ValueError, 'cannot reach explicit torque enable'):
            decode_center_failure(failure_ack(kind=16, stage=21), 31)


if __name__ == '__main__':
    unittest.main()
