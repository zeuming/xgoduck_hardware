"""Packaged native controller integration; all devices and network are fake."""
import argparse,json,sys,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'host/service'),str(Path(__file__).resolve().parents[1]/'x5_recovery')]
from test_native_gamepad import GamepadTests,MonitorTests
class OfficialGamepadTests(GamepadTests):
 def test_local_hold_heartbeat_does_not_depend_on_browser(self):
  self.request();self.pad.poll_once();self.pad.heartbeat_once()
  self.assertEqual(len(self.calls('heartbeat')),1)
suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in (OfficialGamepadTests,MonitorTests)])
r=unittest.TextTestRunner(verbosity=2).run(suite)
a.output.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,hardware_access=False,errors=[str(e) for e in r.errors+r.failures]),indent=2)+'\n')
raise SystemExit(not r.wasSuccessful())
