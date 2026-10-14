"""Exercise actual API validation for the explicit stop-only action."""
from pathlib import Path
import unittest,tempfile,json,sys,types,copy
P=Path(__file__).resolve().parents[2]/'output/duck_sim/stop-recovery-20261012'
sys.path[:0]=[str(P/'candidate/host/service')]
from test_native_workbench import Runtime,SOURCE,CLIENT
import native_workbench as web
class ReleaseWeb(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  Path(self.tmp.name,'index.html').write_text(SOURCE.read_text());self.r=Runtime()
  self.r.motion.update(phase='fault',owns_connection=False,hardware=None,closed_proof=dict(serial_closed=True,release_confirmed=False))
  def release():
   self.r.calls.append(('release-torque',));self.r.motion['closed_proof']['release_confirmed']=True;return self.r.snapshot()
  self.r.release_torque=release
  self.w=web.NativeWorkbench(self.r,web_root=self.tmp.name,authentication_required=False)
 def body(self):return dict(action='release-torque',page_generation=self.w.generation,control_id=CLIENT)
 def test_explicit_release_routes_to_stop_only_without_connect_or_model(self):
  self.w.motion_action(self.body(),live=True);self.assertEqual(self.r.calls,[('release-torque',)])
  self.assertEqual(self.w.last_operation['state'],'complete')
 def test_stale_page_cannot_release(self):
  b=self.body();b['page_generation']-=1
  with self.assertRaises(ValueError):self.w.motion_action(b,live=True)
  self.assertFalse(self.r.calls)
 def test_release_cannot_accept_motion_parameters(self):
  b=self.body();b['mode']='walk'
  with self.assertRaises(ValueError):self.w.motion_action(b,live=True)
  self.assertFalse(self.r.calls)
 def test_maintenance_owner_prevents_takeover(self):
  self.w.calibration_service=types.SimpleNamespace(session=types.SimpleNamespace(recovery_pending=False,source_transmitting=False,maintenance=types.SimpleNamespace(owns_connection=True)))
  with self.assertRaisesRegex(ValueError,'不会抢占'):self.w.motion_action(self.body(),live=True)
  self.assertFalse(self.r.calls)
if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ReleaseWeb))
 (P/'release-web-tests.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=[str(e) for e in result.errors+result.failures]),indent=2)+'\n')
 raise SystemExit(not result.wasSuccessful())
