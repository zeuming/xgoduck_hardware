"""Disarmed postflash probe acceptance tests; never open hardware."""
import unittest

from motion import HELLO, STATE, STOP
from probe_motion import run_probe, signed_us_delta
from test_motion import FakeClock, FakeTransport


class ProbeTests(unittest.TestCase):
    def run_fake(self, kind='healthy'):
        clock = FakeClock()
        begun = clock()

        class Transport(FakeTransport):
            def request(self, command, payload=b''):
                elapsed = clock() - begun
                if command == HELLO and kind == 'wrong_protocol':
                    self.requests.append((command, payload))
                    return b'OTHER_FIRMWARE'
                if command == STATE:
                    self.online = 0 if kind == 'startup_offline' and elapsed < 1.5 else 0x7fff
                    self.fault = 1 if kind == 'late_fault' and elapsed > 8 else 0
                    self.freeze_sample = kind == 'frozen'
                    self.acc = [0, 0, 0] if kind == 'invalid_imu' else [0, 0, 9.8]
                    self.configured = kind == 'configured'
                    self.imu_age_us = -5000 if kind == 'newer_imu' else 0
                    if kind == 'missing_servo': self.online &= ~1
                    if kind == 'stale_imu': self.imu_age_us = 100000
                return super().request(command, payload)

        transport = Transport('fake')
        result = run_probe(transport, clock=clock, sleep=clock.advance)
        self.assertTrue(transport.closed)
        self.assertEqual(transport.requests[0][0], STOP)
        self.assertEqual(transport.requests[-1][0], STOP)
        self.assertTrue(set(command for command, _ in transport.requests).issubset({STOP, HELLO, STATE}))
        self.assertFalse(result['arm_sent'])
        self.assertFalse(result['config_sent'])
        self.assertFalse(result['targets_sent'])
        return result

    def test_healthy_link_passes_without_any_arm_configuration_or_target(self):
        result = self.run_fake()
        self.assertTrue(result['summary']['passed'])
        self.assertGreaterEqual(result['summary']['final_continuous_healthy_seconds'], 3)
        self.assertEqual(result['summary']['observed_online_ids'], result['expected_ids'])
        self.assertEqual(result['summary']['observed_torque_ids'], [])
        self.assertGreater(result['summary']['observed_unique_servo_state_hz'], 0)
        self.assertEqual(len(result['records'][0]['motors']), 15)

    def test_startup_settle_does_not_count_initial_offline_as_final_failure(self):
        result = self.run_fake('startup_offline')
        self.assertTrue(any(not record['healthy'] for record in result['records'] if not record['settled']))
        self.assertTrue(result['summary']['passed'])
        self.assertEqual(result['summary']['unhealthy_after_settle_count'], 0)

    def test_late_fault_fails_final_continuous_healthy_gate(self):
        result = self.run_fake('late_fault')
        self.assertFalse(result['summary']['passed'])
        self.assertIn('servo_fault', result['summary']['last_record_reasons'])
        self.assertEqual(result['summary']['observed_fault_ids'], [10])

    def test_frozen_timestamp_bad_imu_missing_servo_and_old_config_fail(self):
        for kind in ('frozen', 'invalid_imu', 'missing_servo', 'configured', 'stale_imu'):
            with self.subTest(kind=kind):
                result = self.run_fake(kind)
                self.assertFalse(result['summary']['passed'])
                self.assertGreater(result['summary']['unhealthy_after_settle_count'], 0)

    def test_exact_protocol_is_required_and_stop_still_runs_on_failure(self):
        result = self.run_fake('wrong_protocol')
        self.assertFalse(result['summary']['passed'])
        self.assertFalse(result['summary']['hello_verified'])
        self.assertEqual(result['summary']['valid_state_count'], 0)

    def test_newer_imu_and_uint32_timestamp_rollover_are_not_false_stale(self):
        result = self.run_fake('newer_imu')
        self.assertTrue(result['summary']['passed'])
        self.assertEqual(signed_us_delta(20, 0xfffffff0), 36)
        self.assertEqual(signed_us_delta(1000, 2000), -1000)


if __name__ == '__main__':
    unittest.main()
