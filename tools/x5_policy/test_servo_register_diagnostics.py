"""Pure offline schema checks; no UART, transport, runtime or network imports."""
import ast
from pathlib import Path
import struct
import unittest

import servo_register_diagnostics as registers


def packet(sid=11):
    raw = bytearray(202)
    raw[:4] = bytes([1, sid, registers.MOTOR_IDS.index(sid), 8])
    struct.pack_into('<HH', raw, 4, 2002, 2030)
    struct.pack_into('<6I', raw, 10, 0, 0, 1000, 1002, 1000, 1002)
    struct.pack_into('<4H', raw, 34, 0x7fff, 0, 0x7fff, 0)
    for i, (address, size) in enumerate(registers.REGISTER_READ_BLOCKS):
        offset = 42 + i * 20
        raw[offset:offset + 2] = bytes([address, size])
        # High bits deliberately set, including register40=255 and goal65535.
        struct.pack_into('<IIBBBB6s', raw, offset + 2, 1020 + i * 50, 1050 + i * 50,
                         1, 0, size, 5, b'\xff' * size + b'\0' * (6 - size))
    return raw


class RegisterDiagnosticsTests(unittest.TestCase):
    def decode(self, raw, sid=11):
        return registers.decode_register_read_details(0x97, bytes(raw), expected_id=sid)

    def rejected(self, raw, kind=0x97, sid=11):
        value = bytes(raw)
        with self.assertRaises(registers.RegisterDiagnosticError) as caught:
            registers.decode_register_read_details(kind, value, expected_id=sid)
        self.assertEqual(caught.exception.raw_payload_hex, value.hex())
        self.assertTrue(caught.exception.invalid)

    def test_all_mapped_ids_fixed_order_and_exact_raw_high_bits_are_preserved(self):
        for sid in registers.MOTOR_IDS:
            raw = packet(sid)
            result = self.decode(raw, sid)
            self.assertTrue(result['valid'])
            self.assertFalse(result['invalid'])
            self.assertEqual(result['index'], registers.MOTOR_IDS.index(sid))
            self.assertEqual((result['request_type'], result['reply_type'], result['block_count']), (23, 0x97, 8))
            self.assertEqual(result['raw_payload_hex'], bytes(raw).hex())
            self.assertEqual((result['mcu_desired_ticks'], result['mcu_goals_ticks']), (2002, 2030))
            self.assertEqual([(r['address'], r['requested_size']) for r in result['reads']],
                             list(registers.REGISTER_READ_BLOCKS))
            for read in result['reads']:
                size = read['requested_size']
                self.assertEqual(read['raw_bytes'], [255] * size)
                self.assertEqual(read['raw_words_le'], [65535] * (size // 2))
                self.assertEqual(read['reply_data_hex'], 'ff' * size)
                self.assertEqual(read['raw_data_hex'], 'ff' * size + '00' * (6 - size))
                self.assertEqual(read['elapsed_us'], 30)
                self.assertTrue(read['valid'])
                self.assertNotIn('position_ticks', read)
                self.assertNotIn('volts', read)
                self.assertNotIn('torque', read)

    def test_module_remains_pure_and_result_has_no_freshness_or_motion_authority(self):
        tree = ast.parse(Path(registers.__file__).read_text())
        imports = [n.names[0].name for n in ast.walk(tree) if isinstance(n, ast.Import)]
        self.assertEqual(imports, ['struct'])
        self.assertFalse(any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree)))
        result = self.decode(packet())
        for key in ('feedback_updated', 'sensor_freshness_established', 'motion_authorized'):
            self.assertIs(result[key], False)
        self.assertNotIn('age_ms', result)
        self.assertNotIn('release_confirmed', result)

    def test_reply_type_version_size_count_id_index_and_mcu_bounds(self):
        good = packet()
        for kind in (23, 0x96, 0x80, True, '151'):
            self.rejected(good, kind=kind)
        for sid in (10, 9, 255, True, '11'):
            self.rejected(good, sid=sid)
        for raw in (good[:-1], good + b'\0', b''):
            self.rejected(raw)
        for offset, value in ((0, 0), (0, 2), (1, 255), (2, 0), (2, 15), (3, 0), (3, 7), (3, 9)):
            raw = good.copy(); raw[offset] = value; self.rejected(raw)
        for offset in (4, 6):
            raw = good.copy(); struct.pack_into('<H', raw, offset, 4096); self.rejected(raw)
        for raw in (good, memoryview(good), None):
            with self.assertRaises(registers.RegisterDiagnosticError):
                registers.decode_register_read_details(0x97, raw, expected_id=11)

    def test_every_fixed_register_address_and_length_is_required_in_order(self):
        for i, (address, size) in enumerate(registers.REGISTER_READ_BLOCKS):
            offset = 42 + i * 20
            for field, value in ((0, address ^ 1), (1, size - 1), (1, size + 1)):
                raw = packet(); raw[offset + field] = value; self.rejected(raw)
        raw = packet(); raw[42:62], raw[62:82] = raw[62:82], raw[42:62]
        self.rejected(raw)

    def test_off_only_masks_and_unchanged_sensor_timestamps(self):
        for offset in (8, 9, 10, 14):
            raw = packet(); raw[offset] = 1; self.rejected(raw)
        for offset in (34, 36, 38, 40):
            raw = packet(); struct.pack_into('<H', raw, offset, 0x8000); self.rejected(raw)
        for offset, value in ((34, 0x3fff), (36, 1), (40, 1)):
            raw = packet(); struct.pack_into('<H', raw, offset, value); self.rejected(raw)
        for offset in (26, 30):
            raw = packet(); struct.pack_into('<I', raw, offset, 1010); self.rejected(raw)

    def test_natural_mask_expiry_and_microsecond_wrap_do_not_promote_freshness(self):
        raw = packet()
        struct.pack_into('<4H', raw, 34, 0x7fff, 0, 0, 0)
        struct.pack_into('<4I', raw, 18, 0xfffffff0, 0xfffffff4, 0xfffffff0, 0xfffffff4)
        for i in range(8):
            struct.pack_into('<II', raw, 44 + i * 20, 0xfffffff0, 0x10)
        result = self.decode(raw)
        self.assertEqual(result['torque_known_after'], 0)
        self.assertEqual(result['sample_after_us'], 0xfffffff0)
        self.assertTrue(all(read['elapsed_us'] == 32 for read in result['reads']))
        self.assertFalse(result['sensor_freshness_established'])

    def test_standard_reply_validity_is_only_found_error_and_exact_length(self):
        for i, (_, size) in enumerate(registers.REGISTER_READ_BLOCKS):
            raw = packet(); offset = 42 + i * 20
            raw[offset + 13] = 7  # valid normal reply can follow invalid traffic
            read = self.decode(raw)['reads'][i]
            self.assertTrue(read['valid'])
            self.assertTrue(read['invalid_reply_seen'])
            # Zero and high-bit-filled raw data are both structurally valid.
            raw[offset + 14:offset + 20] = b'\0' * 6
            self.assertTrue(self.decode(raw)['reads'][i]['valid'])

    def test_device_error_keeps_raw_zero_or_requested_bytes_without_validity(self):
        for i, (_, requested) in enumerate(registers.REGISTER_READ_BLOCKS):
            offset = 42 + i * 20
            for size in (0, requested):
                raw = packet(); raw[offset + 10:offset + 14] = bytes([2, 4, size, 3])
                raw[offset + 14:offset + 20] = b'\xfe' * size + b'\0' * (6 - size)
                result = self.decode(raw); read = result['reads'][i]
                self.assertEqual((read['outcome'], read['device_error'], read['actual_size']), (2, 4, size))
                self.assertTrue(read['found'])
                self.assertEqual(read['raw_bytes'], [254] * size)
                self.assertFalse(read['valid'])
                self.assertFalse(result['valid'])

    def test_each_real_read_failure_remains_evidence_without_invented_data(self):
        for i in range(8):
            for outcome in (3, 4, 5, 6):
                raw = packet(); offset = 42 + i * 20
                raw[offset + 10:offset + 14] = bytes([outcome, 0, 0, 2])
                raw[offset + 14:offset + 20] = b'\0' * 6
                result = self.decode(raw); read = result['reads'][i]
                self.assertEqual(read['outcome'], outcome)
                self.assertFalse(read['found'])
                self.assertTrue(read['invalid_reply_seen'])
                self.assertEqual(read['raw_bytes'], [])
                self.assertEqual(read['raw_words_le'], [])
                self.assertFalse(result['valid'])

    def test_contradictory_outcomes_flags_lengths_and_padding_rejected_per_block(self):
        for i, (_, requested) in enumerate(registers.REGISTER_READ_BLOCKS):
            offset = 42 + i * 20
            for field, value in ((10, 0), (10, 7), (10, 255), (11, 1),
                                 (12, requested - 1), (12, requested + 1),
                                 (13, 0), (13, 1), (13, 4), (13, 0x85)):
                raw = packet(); raw[offset + field] = value; self.rejected(raw)
            if requested < 6:
                raw = packet(); raw[offset + 14 + requested] = 1; self.rejected(raw)
            for field, value in ((10, 1), (11, 0), (12, requested + 1), (13, 0), (13, 5)):
                raw = packet(); raw[offset + 10:offset + 14] = bytes([2, 4, 0, 1])
                raw[offset + 14:offset + 20] = b'\0' * 6
                raw[offset + field] = value; self.rejected(raw)
            if requested > 1:
                raw = packet(); raw[offset + 10:offset + 14] = bytes([2, 4, 1, 1])
                raw[offset + 14:offset + 20] = b'\x01' + b'\0' * 5; self.rejected(raw)
            for field, value in ((11, 1), (12, 1), (13, 1), (13, 4), (14, 1)):
                raw = packet(); raw[offset + 10:offset + 14] = bytes([6, 0, 0, 0])
                raw[offset + 14:offset + 20] = b'\0' * 6
                raw[offset + field] = value; self.rejected(raw)

    def test_sequence_wrapper_rejects_old_ack_wrong_type_and_noninteger_sequence(self):
        raw = bytes(packet())
        for sequence in (0, 123, 0xffffffff):
            result = registers.decode_register_read_details_frame((0x97, sequence, raw),
                expected_id=11, expected_sequence=sequence)
            self.assertEqual(result['sequence'], sequence)
        for frame, expected in (((0x97, 122, raw), 123), ((0x80, 123, raw), 123),
                                ((0x96, 123, raw), 123), ((0x97, True, raw), 1),
                                ((0x97, 123, raw), True), ((0x97, -1, raw), -1),
                                ((0x97, 2 ** 32, raw), 2 ** 32), ([0x97, 123, raw], 123),
                                ((0x97, 123), 123), (None, 123)):
            with self.subTest(frame_type=type(frame), expected=expected), self.assertRaises(registers.RegisterDiagnosticError):
                registers.decode_register_read_details_frame(frame, expected_id=11, expected_sequence=expected)


if __name__ == '__main__':
    unittest.main()
