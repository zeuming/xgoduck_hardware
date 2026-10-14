"""Separate HTTP process freeze/flood tests with fake controller only."""
from pathlib import Path
import sys,inspect,textwrap,json,unittest,time,signal
P=Path(__file__).resolve().parents[2]/'output/duck_sim/command-view-20261012'
sys.path[:0]=[str(P/'candidate/host/service'),str(Path(__file__).resolve().parents[1]/'x5_recovery')]
import test_native_process as original
source=textwrap.dedent(inspect.getsource(original.ProcessTests.setUp))
source=source.replace("'system_health_panel.html'):","'system_health_panel.html', 'native_command_runtime.py', 'command_view_page.py', 'command_view_ui.js', 'official_runtime_ui.js'):")
namespace=dict(vars(original));exec(source,namespace)
class CommandProcess(original.ProcessTests):
 setUp=namespace['setUp']
 def test_started_command_survives_frozen_http_process(self):
  self.runtime.motion['owns_connection']=True
  code,result=self.request('/api/motion',self.body('start',mode='walk'))
  self.assertEqual(code,200,result)
  self.child.send_signal(signal.SIGSTOP)
  try:
   before=self.bridge.command_runtime.snapshot()['heartbeat_count']
   time.sleep(.65)
   after=self.bridge.command_runtime.snapshot()['heartbeat_count']
   self.assertGreaterEqual(after-before,3)
   self.assertEqual(self.runtime.motion['hardware']['mode'],'walk')
   self.assertEqual(self.bridge.control_monitor.snapshot()['sources']['browser']['heartbeat_accepted'],0)
   self.assertFalse(any(v[1].get('action')=='stop' for v in self.runtime.calls))
  finally:self.child.send_signal(signal.SIGCONT)
  code,result=self.request('/api/motion',self.body('stop',stop_source='button'))
  self.assertEqual(code,200,result)
  before=self.bridge.command_runtime.snapshot()['heartbeat_count'];time.sleep(.25)
  self.assertEqual(self.bridge.command_runtime.snapshot()['heartbeat_count'],before)
  self.assertEqual(self.runtime.motion['hardware']['mode'],'off')

names=['test_started_command_survives_frozen_http_process',
 'test_gamepad_endpoint_is_authenticated_and_does_not_call_controller',
 'test_health_reads_published_files_without_control_commands',
 'test_local_gamepad_works_while_http_process_is_paused',
 'test_status_request_flood_does_not_supply_or_block_gamepad_heartbeat',
 'test_monitor_file_failure_does_not_add_a_control_stop']
suite=unittest.TestSuite(CommandProcess(name) for name in names)
r=unittest.TextTestRunner(verbosity=2).run(suite)
(P/'process-tests.json').write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,hardware_access=False,errors=[str(x) for x in r.errors+r.failures]),indent=2)+'\n')
raise SystemExit(not r.wasSuccessful())
