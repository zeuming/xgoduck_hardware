"""GC policy tests; no device, network, model inference or motor commands."""
import gc
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest import mock

import native_runtime as app


class GCWindowTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.backend = types.SimpleNamespace(
            get_freeze_count=mock.Mock(return_value=0),
            collect=mock.Mock(side_effect=lambda: self.events.append('collect')),
            freeze=mock.Mock(side_effect=lambda: self.events.append('freeze')),
            unfreeze=mock.Mock(side_effect=lambda: self.events.append('unfreeze')),
            isenabled=mock.Mock(return_value=True))
        self.patch = mock.patch.object(app, 'gc', self.backend)
        self.patch.start()
        self.window = app.ConnectionGCWindow()
        self.addCleanup(self.patch.stop)
        self.addCleanup(lambda: self.window.finish(serial_closed=True))

    def test_prepare_then_unfreeze_only_after_exact_closure_proof(self):
        self.window.prepare()
        self.assertEqual(self.events, ['collect', 'freeze'])
        for proof in (False, None, 1, 'closed'):
            self.assertFalse(self.window.finish(serial_closed=proof))
            self.assertTrue(self.window.active)
            self.backend.unfreeze.assert_not_called()
        self.assertTrue(self.window.finish(serial_closed=True))
        self.assertFalse(self.window.active)
        self.assertEqual(self.events, ['collect', 'freeze', 'unfreeze'])

    def test_repeated_prepare_does_not_collect_during_connection(self):
        self.window.prepare()
        self.window.prepare()
        self.backend.collect.assert_called_once_with()
        self.backend.freeze.assert_called_once_with()

    def test_existing_external_permanent_generation_is_never_unfrozen(self):
        self.backend.get_freeze_count.return_value = 10
        with self.assertRaisesRegex(ValueError, '永久代所有者'):
            self.window.prepare()
        self.assertFalse(self.window.active)
        self.window.finish(serial_closed=True)
        self.assertFalse(self.events)

    def test_second_owner_cannot_change_live_connection_heap(self):
        self.window.prepare()
        other = app.ConnectionGCWindow()
        with self.assertRaisesRegex(ValueError, '永久代所有者'):
            other.prepare()
        other.finish(serial_closed=True)
        self.backend.unfreeze.assert_not_called()

    def test_failed_idle_collection_does_not_leave_active_window(self):
        self.backend.collect.side_effect = MemoryError('injected idle collection failure')
        with self.assertRaises(MemoryError):
            self.window.prepare()
        self.assertFalse(self.window.active)
        self.backend.freeze.assert_not_called()
        self.assertIsNone(app.ConnectionGCWindow._owner)

    def test_reconnect_prepares_a_new_heap_after_unfreeze(self):
        self.window.prepare()
        self.window.finish(serial_closed=True)
        self.window.prepare()
        self.assertEqual(self.events, ['collect', 'freeze', 'unfreeze', 'collect', 'freeze'])

    def test_real_collector_still_reclaims_new_cycles_and_restores_heap(self):
        # Isolate process-wide freezing from the unittest runner's own heap.
        source = '''
import gc, weakref
from native_runtime import ConnectionGCWindow
enabled, thresholds = gc.isenabled(), gc.get_threshold()
assert gc.get_freeze_count() == 0
window = ConnectionGCWindow()
window.prepare()
assert gc.get_freeze_count() > 0
assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
class Cell: pass
for _ in range(100):
    value = Cell(); value.cycle = value
    reference = weakref.ref(value)
    del value
    gc.collect(2)
    assert reference() is None
window.finish(serial_closed=True)
assert gc.get_freeze_count() == 0
assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
'''
        result = subprocess.run([sys.executable, '-B', '-c', source],
            cwd=Path(__file__).resolve().parent, text=True, capture_output=True,
            timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
