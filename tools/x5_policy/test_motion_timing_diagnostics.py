"""Read-only timing evidence with a fake UART; no serial device is opened."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from motion import (MotionController, MotionError, STATE, TARGET,
                    TIMING_HISTORY_LIMIT)
from test_motion import FakeClock, FakePolicy, FakeTransport, ready_config


class TimingDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.config_path = Path(self.tmp.name) / 'calibration.json'
        self.config_path.write_text(json.dumps(ready_config()))
        self.clock = FakeClock()
        self.transports = []
        owner = self

        class TimedUART(FakeTransport):
            def __init__(self, port):
                super().__init__(port)
                self.seq = 0
                self.delays = {STATE: .004, TARGET: .003}
                self.fail_kind = None
                owner.transports.append(self)

            def request(self, kind, payload=b''):
                self.seq += 1
                owner.clock.advance(self.delays.get(kind, 0.))
                if kind == self.fail_kind:
                    raise MotionError('fake bounded request failure')
                return super().request(kind, payload)

        self.controller = MotionController({'walk': FakePolicy()}, self.config_path,
                                           transport_factory=TimedUART, clock=self.clock)

    def tearDown(self):
        self.tmp.cleanup()

    def connect(self):
        self.controller._connect_once()
        return self.transports[0]

    def test_empty_status_is_json_serializable_without_opening_transport(self):
        status = self.controller.status()
        self.assertEqual(self.transports, [])
        self.assertIsNone(status['sample_us'])
        self.assertIsNone(status['imu_us'])
        self.assertIsNone(status['state_request_sequence'])
        self.assertNotIn('state_sequence', status)
        for name in ('state_receive', 'state_samples', 'state_request', 'target_request', 'cycle'):
            self.assertEqual(status['timing'][name]['total_count'], 0)
            self.assertEqual(status['timing'][name]['window_count'], 0)
        json.dumps(status, allow_nan=False)

    def test_duplicate_sample_counts_receive_without_refreshing_sample_age(self):
        transport = self.connect()
        before = self.controller.status()
        received = self.controller._sample_received
        transport.freeze_sample = True
        self.clock.advance(.035)
        self.controller._cycle()
        after = self.controller.status()
        self.assertEqual(self.controller._sample_received, received)
        self.assertEqual(after['sample_us'], before['sample_us'])
        self.assertEqual(after['imu_us'], before['imu_us'])
        self.assertEqual(after['state_request_sequence'], before['state_request_sequence'] + 1)
        self.assertEqual(after['timing']['state_receive']['total_count'],
                         before['timing']['state_receive']['total_count'] + 1)
        self.assertEqual(after['timing']['state_samples']['total_count'],
                         before['timing']['state_samples']['total_count'])
        self.assertAlmostEqual(after['age_ms'] - before['age_ms'], 39., places=3)
        self.assertEqual(after['timing']['state_receive']['max_interval_ms'], 39.)
        self.assertEqual(after['timing']['state_samples']['last_received_age_ms'], after['age_ms'])

    def test_startup_and_cycle_requests_have_real_counts_and_raw_timestamps(self):
        transport = self.connect()
        status = self.controller.status()
        self.assertEqual(status['sample_us'], transport.sample_us)
        self.assertEqual(status['imu_us'], transport.sample_us)
        self.assertEqual(status['state_request_sequence'], transport.seq)
        self.assertEqual(status['timing']['state_request']['total_count'], 2)
        self.assertEqual(status['timing']['state_receive']['total_count'], 2)
        self.assertEqual(status['timing']['state_samples']['total_count'], 2)
        self.assertEqual(status['timing']['state_request']['max_ms'], 4.)
        self.controller.start_mode('hold', support_confirmed=True)
        before = self.controller.status()['timing']
        self.controller._cycle()
        status = self.controller.status()
        timing = status['timing']
        self.assertEqual(timing['state_request']['total_count'], before['state_request']['total_count'] + 1)
        self.assertEqual(timing['target_request']['total_count'], 1)
        self.assertEqual(timing['target_request']['last_ms'], 3.)
        self.assertEqual(timing['cycle']['total_count'], 1)
        self.assertEqual(timing['cycle']['last_ms'], 7.)
        self.assertEqual(timing['cycle']['total_failed'], 0)
        self.assertEqual(status['state_request_sequence'], transport.seq - 1)

    def test_request_and_cycle_failures_are_counted_without_retry(self):
        transport = self.connect()
        self.controller.start_mode('hold', support_confirmed=True)
        before_seq = transport.seq
        transport.fail_kind = TARGET
        with self.assertRaisesRegex(MotionError, 'fake bounded request failure'):
            self.controller._cycle()
        timing = self.controller.status()['timing']
        self.assertEqual(transport.seq, before_seq + 2, 'one STATE and one TARGET attempt only')
        self.assertEqual(timing['target_request']['total_count'], 1)
        self.assertEqual(timing['target_request']['total_failed'], 1)
        self.assertEqual(timing['target_request']['window_failed'], 1)
        self.assertEqual(timing['target_request']['last_ms'], 3.)
        self.assertEqual(timing['cycle']['total_failed'], 1)
        self.assertEqual(timing['cycle']['last_ms'], 7.)

    def test_all_histories_are_bounded_and_totals_survive_eviction(self):
        transport = self.connect()
        self.controller.start_mode('hold', support_confirmed=True)
        transport.delays = {STATE: .00001, TARGET: .00001}
        before = self.controller.status()['timing']
        count = TIMING_HISTORY_LIMIT + 40
        for _ in range(count):
            self.controller._cycle()
        timing = self.controller.status()['timing']
        self.assertEqual(len(self.controller._state_receive_times), TIMING_HISTORY_LIMIT)
        self.assertEqual(len(self.controller._state_times), TIMING_HISTORY_LIMIT)
        for name, records in self.controller._timing_records.items():
            self.assertEqual(len(records), TIMING_HISTORY_LIMIT)
            self.assertEqual(records.maxlen, TIMING_HISTORY_LIMIT)
            self.assertEqual(timing[name]['total_count'], before[name]['total_count'] + count)
            self.assertTrue(timing[name]['window_truncated'])
        for name in ('state_receive', 'state_samples'):
            self.assertEqual(timing[name]['total_count'], before[name]['total_count'] + count)
            self.assertEqual(timing[name]['window_count'], TIMING_HISTORY_LIMIT)
            self.assertTrue(timing[name]['window_truncated'])

    def test_window_includes_a_gap_ending_now_and_expires_without_new_events(self):
        self.connect()
        self.clock.advance(1.2)
        self.controller._cycle()
        timing = self.controller.status()['timing']
        self.assertEqual(timing['state_receive']['window_count'], 1)
        self.assertEqual(timing['state_receive']['max_interval_ms'], 1204.)
        self.assertEqual(timing['state_request']['window_count'], 1)
        self.clock.advance(1.01)
        expired = self.controller.status()['timing']
        for name in ('state_receive', 'state_samples', 'state_request', 'target_request', 'cycle'):
            self.assertEqual(expired[name]['window_count'], 0)
        self.assertEqual(expired['state_receive']['total_count'], timing['state_receive']['total_count'])
        self.assertIsNone(expired['state_receive']['max_interval_ms'])
        self.assertIsNone(expired['state_request']['max_ms'])
        self.assertEqual(expired['state_request']['last_ms'], 4.)

    def test_status_printing_does_not_change_evidence_or_issue_requests(self):
        transport = self.connect()
        before = self.controller.status()
        state_before = copy.deepcopy(self.controller._state)
        history_before = copy.deepcopy(self.controller._timing_records)
        request_before = list(transport.requests)
        received_before = self.controller._sample_received
        with mock.patch.object(transport, 'request', side_effect=AssertionError('status must not touch UART')):
            for _ in range(5):
                printed = json.dumps(self.controller.status(), allow_nan=False)
                self.assertEqual(json.loads(printed)['timing'], before['timing'])
        self.assertEqual(self.controller._state, state_before)
        self.assertEqual(self.controller._timing_records, history_before)
        self.assertEqual(transport.requests, request_before)
        self.assertEqual(self.controller._sample_received, received_before)
        self.assertEqual(self.controller.status()['age_ms'], before['age_ms'])
        changed = self.controller.status()
        changed['timing']['cycle']['total_count'] = 999
        self.assertNotEqual(self.controller.status()['timing']['cycle']['total_count'], 999)

    def test_missing_transport_sequence_is_reported_as_unknown(self):
        transport = self.connect()
        del transport.seq
        with mock.patch.object(transport, 'request', side_effect=lambda kind, payload=b'':
                               FakeTransport.request(transport, kind, payload)):
            self.controller._cycle()
        self.assertIsNone(self.controller.status()['state_request_sequence'])


if __name__ == '__main__':
    unittest.main()
