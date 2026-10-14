"""Packaged native controller integration; all devices and network are fake."""
import argparse,json,sys,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'service'),str(Path(__file__).resolve().parents[1]/'x5_recovery')]
sys.path.append(str(Path(__file__).resolve().parents[1]/'x5_policy'))
from test_native_gamepad import GamepadTests,MonitorTests
class OfficialGamepadTests(GamepadTests):
 def test_held_stick_never_repeats_forward(self):
  self.request();self.mode('walk');self.event(1,7,1);self.event(2,1,-32767)
  self.pad.poll_once();self.pad.poll_once()
  self.assertEqual(len(self.calls('command')),1);self.assertEqual(self.calls('command')[0]['forward'],.4)
  self.assertEqual(self.calls('forward-trial'),[])
 def test_forward_uses_existing_rejection_without_retry(self):
  self.request();self.mode('walk');self.runtime.failure='feedback stale'
  self.event(1,7,1);self.event(2,1,-32767);self.pad.poll_once();self.pad.poll_once()
  self.assertIn('feedback stale',self.pad.message);self.assertEqual(len(self.calls('command')),1)
  self.assertEqual(self.calls('stop'),[])
 def test_r1_release_and_stick_neutral_cancel_without_torque_release(self):
  for release in ((1,7,0),(2,1,0)):
   with self.subTest(release=release):
    self.setUp();self.request();self.mode('walk');self.event(1,7,1);self.event(2,1,-32767);self.pad.poll_once()
    self.event(*release);self.pad.poll_once()
    self.assertEqual(len(self.calls('command')),2);self.assertEqual(self.calls('command')[-1]['forward'],0)
    self.assertEqual(self.calls('stop'),[])
 def test_local_hold_heartbeat_does_not_depend_on_browser(self):
  self.request();self.pad.poll_once();self.pad.heartbeat_once()
  self.assertEqual(len(self.calls('heartbeat')),1)
 def test_all_three_axes_match_official_envelope(self):
  self.request();self.mode('walk');self.event(1,7,1)
  for n in (0,1,2):self.event(2,n,-32767)
  self.pad.poll_once();c=self.calls('command')[-1]
  self.assertEqual([c[k] for k in ('forward','side','yaw')],[.4,.3,1.])
suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in (OfficialGamepadTests,MonitorTests)])
r=unittest.TextTestRunner(verbosity=2).run(suite)
a.output.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,hardware_access=False,errors=[str(e) for e in r.errors+r.failures]),indent=2)+'\n')
raise SystemExit(not r.wasSuccessful())
