"""Passive collector lifecycle and timing; no serial or robot access."""
import gc
import unittest
from unittest import mock

from host_stall_evidence import HostLoopEvidence


class HostEvidenceTests(unittest.TestCase):
    def test_observer_does_not_change_collection_policy_and_removes_itself(self):
        enabled, thresholds, callbacks = gc.isenabled(), gc.get_threshold(), list(gc.callbacks)
        evidence = HostLoopEvidence()
        evidence.start()
        try:
            gc.collect(0)
            self.assertTrue(evidence.snapshot()['garbage_collections'])
            self.assertEqual((gc.isenabled(), gc.get_threshold()), (enabled, thresholds))
        finally:
            evidence.close()
        self.assertEqual(gc.callbacks, callbacks)

    def test_records_300ms_host_gap_separately_from_requested_wait_and_cpu(self):
        now, cpu = [0.], [0.]
        evidence = HostLoopEvidence(lambda: now[0], lambda: cpu[0])
        evidence.begin_cycle('walk', 3)
        now[0], cpu[0] = .012, .002
        evidence.end_cycle()
        evidence.waiting(.008)
        now[0] = .320
        evidence.begin_cycle('walk', 3)
        row = evidence.snapshot()['current']
        self.assertAlmostEqual(row['between_cycles_ms'], 308)
        self.assertAlmostEqual(row['wake_lateness_ms'], 300)
        self.assertAlmostEqual(evidence.cycles[0]['thread_cpu_ms'], 2)

    def test_gc_pause_has_its_own_interval_and_generation(self):
        now = [1.]
        evidence = HostLoopEvidence(lambda: now[0], lambda: now[0])
        evidence._on_gc('start', {'generation': 2})
        now[0] += .18
        evidence._on_gc('stop', {'generation': 2, 'collected': 7, 'uncollectable': 0})
        row = evidence.snapshot()['garbage_collections'][0]
        self.assertAlmostEqual(row['ended_monotonic']-row['began_monotonic'], .18)
        self.assertEqual((row['generation'], row['collected']), (2, 7))

    def test_history_is_bounded_and_snapshot_cannot_mutate_recording(self):
        evidence = HostLoopEvidence()
        for _ in range(200):
            evidence.begin_cycle('off', 1)
            evidence.end_cycle()
            evidence._on_gc('start', {'generation': 0})
            evidence._on_gc('stop', {'generation': 0})
        snapshot = evidence.snapshot()
        self.assertEqual((len(snapshot['cycles']), len(snapshot['garbage_collections'])), (150, 16))
        snapshot['cycles'][0]['mode'] = 'changed'
        self.assertEqual(evidence.snapshot()['cycles'][0]['mode'], 'off')

    def test_collection_during_snapshot_copy_cannot_mutate_the_iterator(self):
        evidence = HostLoopEvidence()
        evidence._on_gc('start', {})
        evidence._on_gc('stop', {})
        copying = False
        def copying_dict(*args, **kwargs):
            nonlocal copying
            if not copying:
                copying = True
                evidence._on_gc('start', {})
                evidence._on_gc('stop', {})
                copying = False
            return dict(*args, **kwargs)
        with mock.patch('host_stall_evidence.dict', copying_dict, create=True):
            snapshot = evidence.snapshot()
        self.assertEqual(len(snapshot['garbage_collections']), 1)
        self.assertGreater(len(evidence.collections), 1)


if __name__ == '__main__':
    unittest.main()
