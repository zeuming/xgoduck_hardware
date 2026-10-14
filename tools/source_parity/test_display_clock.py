"""CPU-only regressions for locked capture/slow formatting display timestamps."""
import argparse
import json
from pathlib import Path
import sys
import threading
from unittest import mock
import unittest

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'policy'),str(a.candidate/'service'),
              str(Path(__file__).resolve().parents[1]/'x5_policy')]
import motion
from status_snapshot import capture_display_view
import native_runtime
from test_motion import FakeClock,FakePolicy,ready_config

class DisplayClockTests(unittest.TestCase):
 def test_detached_render_time_cannot_age_or_refresh_live_controller(self):
  clock=FakeClock()
  c=motion.MotionController({name:FakePolicy(name) for name in motion.MODEL_MODES},clock=clock)
  c._config=ready_config()
  with c._lock: view=capture_display_view(c)
  captured=view._clock();clock.advance(.7)
  self.assertEqual(view._clock(),captured)
  self.assertAlmostEqual(c._clock()-captured,.7)
  self.assertIsNone(view._transport)
  self.assertIsNot(view._lock,c._lock)

 def runtime(self,controller):
  r=object.__new__(native_runtime.NativeRuntime)
  r.lock=threading.RLock();r._controller=controller;r.closed_proof=None
  r._hardware_snapshot=None;r._hardware_generation_floor=0
  r._hardware_snapshot_at=None;r._hardware_version=0;r._connection_sequence=1
  return r

 def test_lock_and_format_delays_are_counted_once(self):
  now=[100.]
  class Controller:
   def status(self):
    now[0]+=.4 # waiting for the control lock, new state arrives in this interval
    captured=now[0]
    now[0]+=.2 # format outside lock
    return dict(generation=1,last_action=None,age_ms=20.,imu_age_ms=25.,
                status_captured_monotonic=captured)
  c=Controller();r=self.runtime(c)
  with mock.patch.object(native_runtime.time,'monotonic',lambda:now[0]):
   r._publish_hardware(c)
  self.assertAlmostEqual(r._hardware_snapshot_at,100.4)
  self.assertAlmostEqual(r._hardware_snapshot['age_ms']+(now[0]-r._hardware_snapshot_at)*1000,220.)
  # No timestamp reset to publication end: the 200ms formatting delay remains.
  self.assertNotEqual(r._hardware_snapshot_at,now[0])

 def test_future_nonfinite_boolean_capture_rejected(self):
  c=object();r=self.runtime(c)
  for value in (float('nan'),float('inf'),True,101.):
   with self.subTest(value=value),mock.patch.object(native_runtime.time,'monotonic',lambda:100.):
    with self.assertRaises(ValueError):
     r._publish_hardware(c,dict(generation=1,age_ms=0.,status_captured_monotonic=value))
  self.assertIsNone(r._hardware_snapshot)

 def test_legacy_status_remains_conservative_and_generation_order_preserved(self):
  c=object();r=self.runtime(c)
  with mock.patch.object(native_runtime.time,'monotonic',lambda:100.):
   r._publish_hardware(c,dict(generation=2,age_ms=10.))
   r._publish_hardware(c,dict(generation=1,age_ms=0.,status_captured_monotonic=99.))
  self.assertEqual(r._hardware_snapshot['generation'],2)
  self.assertEqual(r._hardware_snapshot_at,100.)

suite=unittest.defaultTestLoader.loadTestsFromTestCase(DisplayClockTests)
result=unittest.TextTestRunner(verbosity=2).run(suite)
a.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),
 failures=len(result.failures),passed=result.wasSuccessful(),hardware_access=False,
 deployment=False),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
