"""Exact submitted inputs, requested targets and saturated wire targets; memory transport only."""
import argparse, ast, json, math, struct, sys, tempfile, time, unittest
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'policy'),str(Path(__file__).resolve().parents[1]/'x5_policy')]
import motion
from test_motion import FakeTransport,FakePolicy,FakeClock,ready_config

def load_class(path,name,namespace):
 tree=ast.parse(path.read_text());node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
 exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
 return namespace[name]

Transport=load_class(Path(__file__).with_name('test_source_host.py'),'Transport',dict(FakeTransport=FakeTransport,motion=motion,struct=struct))
from policy_worker import PolicyWorkerError
Worker=load_class(Path(__file__).parents[1]/'x5_policy/test_async_policy.py','DeferredWorker',dict(np=np,PolicyWorkerError=PolicyWorkerError))

class EvidenceTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  path=Path(self.tmp.name)/'cal.json';path.write_text(json.dumps(ready_config()))
  self.clock=FakeClock();self.t=Transport('memory-only');self.t.gyro=[0.,0.,0.]
  self.sessions={n:FakePolicy(n) for n in motion.MODEL_MODES}
  self.c=motion.MotionController(self.sessions,path,transport_factory=lambda _:self.t,clock=self.clock)
  self.c._connect_once();self.c.start_mode('walk',expected_generation=self.c._generation)
 def step(self):
  self.clock.advance(.02);self.c._accept_state_locked(self.c._read_state())
  self.c.heartbeat(expected_generation=self.c._generation);self.c._control_once()
 def test_sync_tensor_action_ema_and_transport_are_exact(self):
  self.sessions['walk'].output[:]=.05;self.step();row=self.c._policy_trace[-1]
  np.testing.assert_array_equal(row['submitted_observation'],self.sessions['walk'].observation)
  np.testing.assert_array_equal(row['raw_action'],self.c._previous_action)
  np.testing.assert_array_equal(row['EMA'],self.c._smooth_action)
  np.testing.assert_array_equal(row['requested_target_deg'],self.c._last_action)
  self.assertTrue(row['target_write_attempted']);self.assertTrue(row['target_write_completed'])
  self.assertFalse(row['motor_execution_confirmed'])
  packet=next(v for k,v in reversed(self.t.requests) if k==motion.SOURCE_TARGET)
  self.assertEqual(row['transmitted_ticks'],list(struct.unpack_from('<I15H',packet)[1:]))
 def test_async_evidence_uses_submitted_not_latest_sensor_state(self):
  worker=Worker();self.c._policy_worker=worker;self.step()
  job=self.c._policy_job;obs=worker.requests[0][1].copy();sample=job['observation_sample']
  self.t.positions[2]+=10;worker.complete(1,.02);self.step()
  row=self.c._policy_trace[-1]
  np.testing.assert_array_equal(row['submitted_observation'],obs)
  self.assertEqual(row['observation_sample_us'],sample)
  self.assertNotEqual(row['commit_sample_us'],sample)
  self.assertEqual(row['submission_monotonic'],job['submission_monotonic'])
 def test_range_saturation_retains_requested_and_sent_policy(self):
  self.sessions['walk'].output[:]=10.
  before=sum(k==motion.SOURCE_TARGET for k,_ in self.t.requests)
  self.step()
  row=self.c._policy_trace[-1]
  self.assertTrue(row['target_write_attempted']);self.assertTrue(row['target_write_completed'])
  np.testing.assert_array_equal(row['submitted_observation'],self.sessions['walk'].observation)
  np.testing.assert_array_equal(row['raw_action'],self.c._previous_action)
  np.testing.assert_array_equal(row['requested_target_deg'],self.c._last_policy_target)
  self.assertEqual(before+1,sum(k==motion.SOURCE_TARGET for k,_ in self.t.requests))
  self.assertTrue(row['target_range_projection'])
  for value,joint in zip(row['transmitted_ticks'],self.c._config['joints']):
   self.assertTrue(joint['min'] <= value <= joint['max'])
  self.assertTrue(self.t.armed);self.assertEqual(self.c._mode,'walk')
 def test_stopped_or_obsolete_job_never_claims_committed_evidence(self):
  worker=Worker();self.c._policy_worker=worker;self.step()
  self.c.command(.1,0,0);worker.complete(1);self.step()
  self.assertFalse(self.c._policy_trace);self.assertIn(1,worker.discarded)
  self.c.stop();worker.complete(2);self.step();self.assertFalse(self.c._policy_trace)
 def test_detached_tensor_and_bounded_history(self):
  self.step();row=self.c._policy_trace[-1];old=row['submitted_observation'][0][:]
  self.sessions['walk'].observation.fill(999)
  self.assertEqual(row['submitted_observation'][0],old)
  for _ in range(3100):self.step()
  self.assertEqual(len(self.c._policy_trace),3000)
  self.c._stop_locked('bounded evidence');trial=self.c.completed_trial_traces()[-1]
  self.assertTrue(trial['policy_frames_truncated']);self.assertEqual(trial['total_policy_frames'],3101)
 def test_evidence_does_not_change_targets_or_recurrence(self):
  # Disable only the evidence sink on the paired second controller.
  path=Path(self.tmp.name)/'cal.json';t=Transport('memory-only');t.gyro=[0.,0.,0.]
  clock=FakeClock();sessions={n:FakePolicy(n) for n in motion.MODEL_MODES}
  other=motion.MotionController(sessions,path,transport_factory=lambda _:t,clock=clock)
  other._connect_once();other.start_mode('walk',expected_generation=other._generation)
  other._record_policy_frame_locked=lambda *args:None
  for i in range(120):
   value=.03*math.sin(i/7)
   self.sessions['walk'].output[:]=value;sessions['walk'].output[:]=value
   self.step();clock.advance(.02);other._accept_state_locked(other._read_state())
   other.heartbeat(expected_generation=other._generation);other._control_once()
   np.testing.assert_array_equal(self.c._last_executed_target,other._last_executed_target)
   np.testing.assert_array_equal(self.c._previous_action,other._previous_action)
   np.testing.assert_array_equal(self.c._smooth_action,other._smooth_action)
  wire=lambda transport:[v[4:] for k,v in transport.requests if k==motion.SOURCE_TARGET]
  self.assertEqual(wire(self.t),wire(t))

result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(EvidenceTests))
a.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),passed=result.wasSuccessful(),hardware_access=False),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
