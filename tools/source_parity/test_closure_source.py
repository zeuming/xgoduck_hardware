"""Actual packaged STOP/close regressions, memory UART only."""
import argparse, ast, json, struct, sys, tempfile, threading, time, unittest
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'policy'),str(a.candidate/'service'),
              str(Path(__file__).resolve().parents[1]/'x5_policy')]
import motion, native_runtime
from test_motion import FakeTransport,FakePolicy,FakeClock,ready_config

# Use the existing source-wire fake, never import its executable test runner.
tree=ast.parse((Path(__file__).parent/'test_source_host.py').read_text())
node=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Transport')
ns=dict(FakeTransport=FakeTransport,motion=motion,struct=struct)
exec(compile(ast.Module(body=[node],type_ignores=[]),'<memory transport>','exec'),ns)
Transport=ns['Transport']
tree=ast.parse((a.candidate/'service/worker-classes.py').read_text())
node=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='OneShotController')
ns=dict(motion=motion,time=time,request_counts={})
exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual native controller>','exec'),ns)
Controller=ns['OneShotController']

class ClosureTests(unittest.TestCase):
 def setUp(self):
  tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
  path=Path(tmp.name)/'config.json';path.write_text(json.dumps(ready_config()))
  self.clock=FakeClock();self.t=Transport('memory-only')
  self.c=Controller({n:FakePolicy(n) for n in motion.MODEL_MODES},path,
    transport_factory=lambda _:self.t,clock=self.clock)
  self.c._connect_once()

 def finish(self):
  c=self.c;r=object.__new__(native_runtime.NativeRuntime)
  # This fixture bypasses __init__; supply the current runtime's inactive GC
  # owner without preparing/freezing the test process or changing production.
  if hasattr(native_runtime, 'ConnectionGCWindow'):
   r._gc_window = native_runtime.ConnectionGCWindow()
  r.lock=r._lifecycle=threading.RLock();r._controller=c;r.closed_proof=None
  r._connection_sequence=1;r._hardware_version=0;r._hardware_generation_floor=0
  r._generation_floor=0;r.last_operation={};r._flush_action_evidence=lambda:None
  r._record=lambda *args:None;r._persist=lambda:None
  r._serial_closed=lambda controller,namespace:controller._transport is None
  r._finish(c,dict(exit_event=threading.Event(),serial_open_attempts=1,
    request_counts={}), 'original range error', fault=True)
  return r.closed_proof

 def test_measured_off_survives_fault_disconnect_and_live_flag_stays_false(self):
  c=self.c;c.start_mode('walk',expected_generation=c._generation)
  c.stop();record=c.release_verification()
  self.assertTrue(record['verification']['release_confirmed'])
  self.assertGreaterEqual(record['verification']['distinct_off_samples'],2)
  c._disconnect('original range error')
  self.assertFalse(c.status()['release_confirmed'])
  before=len(self.t.requests);proof=self.finish()
  self.assertEqual(len(self.t.requests),before) # closed controller does no I/O
  self.assertTrue(proof['serial_closed']);self.assertTrue(proof['release_confirmed'])
  self.assertEqual(proof['release_verification'],record)
  self.assertEqual(proof['reason'],'original range error')
  self.assertFalse(proof['after']['release_confirmed']) # not fresh live feedback

 def test_previous_off_cannot_confirm_later_arm_even_when_ack_is_lost(self):
  c=self.c;c.stop();self.assertIsNotNone(c.release_verification())
  original=self.t.request
  def lost(kind,payload=b''):
   value=original(kind,payload)
   if kind==motion.ARM:raise TimeoutError('lost ARM reply')
   return value
  self.t.request=lost
  with self.assertRaises(TimeoutError):c._request(motion.ARM,struct.pack('<I',123))
  self.assertTrue(self.t.armed);self.assertIsNone(c.release_verification())
  c._disconnect('lost reply');self.assertFalse(self.finish()['release_confirmed'])

 def test_target_attempt_invalidates_previous_off_before_validation_or_ack(self):
  for kind in (motion.TARGET,motion.SOURCE_TARGET):
   with self.subTest(kind=kind):
    c=self.c;c.stop();self.assertIsNotNone(c.release_verification())
    with self.assertRaises(Exception):c._request(kind,b'invalid target')
    self.assertIsNone(c.release_verification())

 def test_missing_torque_readback_cannot_make_transaction_proof(self):
  from stop_release import ReleaseUnconfirmed
  c=self.c;c.stop();self.t.off_known=0
  with self.assertRaises(motion.MotionError):
   with c._lock:c._stop_locked();c._service_stop_locked()
  self.assertIsNone(c.release_verification())
  c._disconnect('OFF unknown');self.assertFalse(self.finish()['release_confirmed'])

 def test_no_stop_transaction_does_not_infer_release_from_cached_state(self):
  self.c._disconnect('no STOP transaction')
  self.assertFalse(self.finish()['release_confirmed'])

 def test_new_connection_invalidates_old_transaction(self):
  c=self.c;c.stop();self.assertIsNotNone(c.release_verification())
  c._disconnect('closed');c._connect_once()
  self.assertIsNone(c.release_verification())

 def test_source_velocity_scale_both_filtered_observation_and_display(self):
  c=self.c;self.t.velocity=list(range(-7,8));c._last_sample=None
  c._filter=motion.ImuFilter() # first sample, without prior smoothing history
  c._accept_state_locked(c._read_state())
  expected=[v*50*360/4096*j['sign'] for v,j in zip(self.t.velocity,c._config['joints'])]
  np.testing.assert_allclose(c._filtered.vels,expected,atol=1e-12,rtol=0)
  np.testing.assert_allclose([x['velocity_deg_s'] for x in c.status()['rawdata']],expected,atol=1e-12,rtol=0)

result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ClosureTests))
a.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),
 failures=len(result.failures),passed=result.wasSuccessful(),hardware_access=False,
 deployment=False),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
