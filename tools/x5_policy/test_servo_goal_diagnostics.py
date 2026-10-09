"""Pure offline GoalReadDetails schema tests; no transport/device imports."""
import struct
import unittest

import servo_goal_diagnostics as goal


def packet(sid=11):
    raw = bytearray(78)
    raw[:4] = bytes([1, sid, goal.MOTOR_IDS.index(sid), 0])
    struct.pack_into('<HH', raw, 4, 2002, 2030)
    struct.pack_into('<6I', raw, 10, 0, 0, 1000, 1002, 1000, 1002)
    struct.pack_into('<4H', raw, 34, 0x7fff, 0, 0x7fff, 0)
    struct.pack_into('<IIBBBB6s', raw, 42, 1020, 1050, 1, 0, 2, 5, b'\xd2\x07\0\0\0\0')
    struct.pack_into('<IIBBBB6s', raw, 60, 1060, 1100, 1, 0, 6, 5, b'\0\x08\x40\x80\x01\x04')
    return raw


class GoalDiagnosticsTests(unittest.TestCase):
    def decode(self, raw, sid=11):
        return goal.decode_goal_read_details(goal.GOAL_READ_DETAILS_REPLY, bytes(raw), expected_id=sid)

    def rejected(self, raw, **kwargs):
        value = bytes(raw)
        with self.assertRaises(goal.GoalDiagnosticError) as caught:
            goal.decode_goal_read_details(kwargs.get('kind', 0x96), value,
                                          expected_id=kwargs.get('expected_id', 11))
        self.assertEqual(caught.exception.raw_payload_hex, value.hex())
        self.assertTrue(caught.exception.invalid)

    def test_exact_normal_reply_and_all_mapped_ids(self):
        for sid in goal.MOTOR_IDS:
            raw = packet(sid)
            result = self.decode(raw, sid)
            self.assertTrue(result['valid'])
            self.assertFalse(result['invalid'])
            self.assertEqual(result['index'], goal.MOTOR_IDS.index(sid))
            self.assertEqual((result['request_type'], result['reply_type']), (22, 0x96))
            self.assertEqual(result['mcu_desired_ticks'], 2002)
            self.assertEqual(result['mcu_goals_ticks'], 2030)
            self.assertEqual(result['goal_read']['position_ticks'], 2002)
            present = result['present_read']
            self.assertEqual(present['position_ticks'], 2048)
            self.assertEqual((present['raw_speed_word'], present['raw_load_word']), (0x8040, 0x0401))
            self.assertEqual(present['raw_data_hex'], '000840800104')
            self.assertEqual(result['raw_payload_hex'], bytes(raw).hex())
            self.assertFalse(result['feedback_updated'])
            self.assertFalse(result['sensor_freshness_established'])
            self.assertFalse(result['motion_authorized'])

    def test_type_version_length_reserved_id_index_and_mcu_bounds_rejected(self):
        good = packet()
        for kind in (22, 0x95, 0x80, True, '150'):
            with self.subTest(kind=kind): self.rejected(good, kind=kind)
        for expected in (10, 9, 15, 255, True, '11'):
            with self.subTest(expected=expected): self.rejected(good, expected_id=expected)
        for raw in (good[:-1], good + b'\0', b''):
            self.rejected(raw)
        for offset, value in ((0, 0), (0, 2), (1, 255), (2, 0), (2, 15), (3, 1)):
            raw = good.copy(); raw[offset] = value; self.rejected(raw)
        for offset in (4, 6):
            raw = good.copy(); struct.pack_into('<H', raw, offset, 4096); self.rejected(raw)
        with self.assertRaises(goal.GoalDiagnosticError):
            goal.decode_goal_read_details(0x96, good, expected_id=11)  # mutable input is not a wire payload

    def test_off_only_armed_cookie_and_mask_bits_are_strict(self):
        for offset in (8, 9, 10, 14):
            raw = packet(); raw[offset] = 1; self.rejected(raw)
        for offset in (34, 36, 38, 40):
            raw = packet(); struct.pack_into('<H', raw, offset, 0x8000); self.rejected(raw)
        for offset, value in ((34, 0x3fff), (36, 1), (40, 1)):
            raw = packet(); struct.pack_into('<H', raw, offset, value); self.rejected(raw)
        for offset in (26, 30):
            raw = packet(); struct.pack_into('<I', raw, offset, 1010); self.rejected(raw)

    def test_natural_mask_expiry_and_timestamp_wrap_never_create_freshness(self):
        raw = packet()
        struct.pack_into('<4H', raw, 34, 0x7fff, 0, 0x3fff, 0)
        struct.pack_into('<4I', raw, 18, 0xfffffff0, 0xfffffff4, 0xfffffff0, 0xfffffff4)
        struct.pack_into('<II', raw, 42, 0xfffffff0, 0x10)
        result = self.decode(raw)
        self.assertEqual(result['torque_known_before'], 0x7fff)
        self.assertEqual(result['torque_known_after'], 0x3fff)
        self.assertEqual(result['sample_after_us'], 0xfffffff0)
        self.assertEqual(result['goal_read']['elapsed_us'], 32)
        self.assertFalse(result['sensor_freshness_established'])
        self.assertNotIn('age_ms', result)
        self.assertNotIn('release_confirmed', result)

    def test_out_of_range_register_words_stay_raw_with_original_normal_outcome(self):
        for offset in (42, 60):
            for value in (4096, 65535):
                raw = packet(); struct.pack_into('<H', raw, offset + 12, value)
                raw[offset + 11] = 3  # found + previous invalid traffic, registerValid false
                result = self.decode(raw)
                read = result['goal_read' if offset == 42 else 'present_read']
                self.assertEqual(read['outcome'], 1)
                self.assertEqual(read['raw_position_word'], value)
                self.assertEqual(read['raw_data_hex'], bytes(raw[offset+12:offset+18]).hex())
                self.assertIsNone(read['position_ticks'])
                self.assertTrue(read['invalid'])
                self.assertTrue(read['invalid_reply_seen'])
                self.assertFalse(result['valid'])

    def test_position_endpoints_and_independent_invalid_traffic_flag(self):
        for offset in (42, 60):
            for position in (0, 4095):
                raw = packet(); struct.pack_into('<H', raw, offset+12, position)
                raw[offset+11] |= 2
                read = self.decode(raw)['goal_read' if offset == 42 else 'present_read']
                self.assertEqual(read['position_ticks'], position)
                self.assertTrue(read['valid'])
                self.assertTrue(read['invalid_reply_seen'])

    def test_device_error_reply_retains_zero_or_fixed_size_raw_without_normal_value(self):
        for offset, requested in ((42, 2), (60, 6)):
            for size in (0, requested):
                raw = packet(); raw[offset+8:offset+12] = bytes([2, 4, size, 3])
                if size == 0: raw[offset+12:offset+18] = b'\0'*6
                read = self.decode(raw)['goal_read' if offset == 42 else 'present_read']
                self.assertEqual((read['outcome'], read['device_error'], read['actual_size']), (2, 4, size))
                self.assertTrue(read['found'])
                self.assertIsNone(read['position_ticks'])
                self.assertFalse(read['valid'])
                self.assertEqual(read['reply_data_hex'], bytes(raw[offset+12:offset+12+size]).hex())

    def test_failed_reads_preserve_outcome_and_invalid_seen_without_position(self):
        for offset in (42, 60):
            for outcome in (3, 4, 5, 6):
                raw = packet(); raw[offset+8:offset+12] = bytes([outcome, 0, 0, 2])
                raw[offset+12:offset+18] = b'\0'*6
                read = self.decode(raw)['goal_read' if offset == 42 else 'present_read']
                self.assertEqual(read['outcome'], outcome)
                self.assertIsNone(read['position_ticks'])
                self.assertFalse(read['found'])
                self.assertTrue(read['invalid_reply_seen'])
                self.assertTrue(read['invalid'])

    def test_outcome_flags_sizes_padding_and_register_validity_must_correspond(self):
        for offset, requested in ((42, 2), (60, 6)):
            for field, value in ((8, 0), (8, 7), (8, 255), (9, 1),
                                 (10, requested-1), (10, requested+1), (11, 0), (11, 1), (11, 0x85)):
                raw = packet(); raw[offset+field] = value; self.rejected(raw)
            raw = packet(); struct.pack_into('<H', raw, offset+12, 4096); self.rejected(raw)
            for field, value in ((8, 1), (9, 0), (10, 1), (11, 0), (11, 5)):
                raw = packet(); raw[offset+8:offset+12] = bytes([2, 4, 0, 1]); raw[offset+12:offset+18] = b'\0'*6
                raw[offset+field] = value; self.rejected(raw)
            for field, value in ((9, 1), (10, 1), (11, 1), (11, 4), (12, 1)):
                raw = packet(); raw[offset+8:offset+12] = bytes([6, 0, 0, 0]); raw[offset+12:offset+18] = b'\0'*6
                raw[offset+field] = value; self.rejected(raw)
        raw = packet(); raw[56] = 1; self.rejected(raw)  # goal READ only has two data bytes

    def test_frame_wrapper_requires_original_sequence_not_ack_or_old_reply(self):
        raw = bytes(packet())
        value = goal.decode_goal_read_details_frame((0x96, 123, raw), expected_id=11, expected_sequence=123)
        self.assertEqual(value['sequence'], 123)
        for frame, expected in (((0x96, 122, raw), 123), ((0x80, 123, raw), 123),
                                ((0x96, True, raw), 1), ((0x96, 123, raw), True),
                                ((0x96, -1, raw), -1), ((0x96, 2**32, raw), 2**32),
                                ([0x96, 123, raw], 123), ((0x96, 123), 123)):
            with self.subTest(frame_type=type(frame), expected=expected), self.assertRaises(goal.GoalDiagnosticError):
                goal.decode_goal_read_details_frame(frame, expected_id=11, expected_sequence=expected)


if __name__ == '__main__':
    unittest.main()
