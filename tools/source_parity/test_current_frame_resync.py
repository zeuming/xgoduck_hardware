"""Original-budget current-frame resynchronization, real IPC/ONNX and memory UART."""
import argparse,ast,hashlib,json,socket,struct,sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path[:0]=[str(a.candidate/'policy'),str(Path(__file__).parents[1]/'x5_policy')]
import motion,policy_worker
from test_motion import FakeTransport,FakePolicy,FakeClock,ready_config
from policy_worker import PolicyWorker,PolicyWorkerError

def cls(path,name,ns):
 node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name==name)
 exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns);return ns[name]
Transport=cls(Path(__file__).with_name('test_source_host.py'),'Transport',dict(FakeTransport=FakeTransport,motion=motion,struct=struct))

class ReadyWorker:
 def __init__(self):self.seq=0;self.results={};self.requests=[];self.discarded=[];self.value=.03;self.waits=[]
 def submit(self,name,obs):
  self.seq+=1;self.requests.append(obs.copy());self.results[self.seq]=(np.full((1,14),self.value,np.float32),.5);return self.seq
 def take(self,request):return self.results.pop(request,None)
 def wait_for_result(self,request,timeout):self.waits.append(timeout);return self.take(request)
 def discard(self,request):self.discarded.append(request)
 def status(self):return dict(alive=True)

class BudgetTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  path=Path(self.tmp.name)/'cal.json';path.write_text(json.dumps(ready_config()))
  self.clock=FakeClock();self.t=Transport('memory-only');self.t.gyro=[0.,0.,0.]
  self.sessions={n:FakePolicy(n) for n in motion.MODEL_MODES}
  self.c=motion.MotionController(self.sessions,path,transport_factory=lambda _:self.t,clock=self.clock)
  self.c._connect_once();self.c.start_mode('walk',expected_generation=self.c._generation)
  self.base=len(self.t.requests)
 def targets(self):return [payload for kind,payload in self.t.requests[self.base:] if kind==motion.SOURCE_TARGET]
 def step(self,budget=.02):
  self.clock.advance(.02);self.c.heartbeat(expected_generation=self.c._generation)
  self.c._policy_cycle_deadline=self.clock()+budget
  try:self.c._cycle()
  finally:self.c._policy_cycle_deadline=None
 def test_current_sample_result_transmitted_same_cycle_no_prefetch(self):
  w=ReadyWorker();self.c._policy_worker=w;self.step()
  self.assertEqual(len(self.targets()),1);self.assertEqual(w.seq,1);self.assertIsNone(self.c._policy_job)
  row=self.c._policy_trace[-1];self.assertEqual(row['observation_sample_us'],row['commit_sample_us'])
  self.assertGreater(w.waits[0],0);self.assertLessEqual(w.waits[0],motion.PERIOD+1e-12)
 def test_one_overrun_recovers_current_state_next_cycle_and_committed_history(self):
  w=ReadyWorker();count=0
  def deferred(request,timeout):
   nonlocal count
   count+=1
   return None if count==1 else w.take(request)
  w.wait_for_result=deferred;self.c._policy_worker=w
  self.step();self.assertFalse(self.targets())
  for i in range(1,12):
   self.step();self.assertEqual(len(self.targets()),i)
   self.assertIsNone(self.c._policy_job)
   if i>=2:
    row=self.c._policy_trace[-1]
    self.assertEqual(row['observation_sample_us'],row['commit_sample_us'])
  self.assertEqual(w.seq,11)
 def test_repeated_overruns_never_relabel_old_sample_or_commit_twice(self):
  w=ReadyWorker();attempts={}
  def deferred(request,timeout):
   attempts[request]=attempts.get(request,0)+1
   return None if attempts[request]==1 else w.take(request)
  w.wait_for_result=deferred;self.c._policy_worker=w
  for i in range(12):self.step()
  self.assertEqual(len(self.targets()),6);self.assertEqual(w.seq,6)
  self.assertTrue(all(r['observation_sample_us']<r['commit_sample_us'] for r in self.c._policy_trace))
  self.assertIsNone(self.c._policy_job)
 def test_no_cycle_budget_preserves_nonblocking_callers(self):
  w=ReadyWorker();self.c._policy_worker=w;self.c._cycle()
  self.assertFalse(self.targets());self.assertFalse(w.waits);self.assertIsNotNone(self.c._policy_job)
 def test_read_cost_and_old_imu_age_are_subtracted(self):
  w=ReadyWorker();self.c._policy_worker=w;original=self.t.request
  def slow_read(kind,payload=b''):
   if kind==motion.SOURCE_STATE:self.clock.advance(.013)
   return original(kind,payload)
  self.t.request=slow_read;self.step()
  self.assertAlmostEqual(w.waits[-1],.007,places=10)
  self.t.imu_age_us=int((motion.FRESH_SECONDS-.003)*1e6);self.step()
  self.assertLessEqual(w.waits[-1],.0030001)
 def test_stop_during_readiness_wait_does_not_block_and_discards_target(self):
  w=ReadyWorker();entered=threading.Event();release=threading.Event()
  def waiting(request,timeout):entered.set();release.wait(1);return w.take(request)
  w.wait_for_result=waiting;self.c._policy_worker=w;errors=[]
  def run():
   try:self.step()
   except BaseException as e:errors.append(repr(e))
  t=threading.Thread(target=run);t.start();self.assertTrue(entered.wait(1))
  began=time.monotonic();self.c.stop();stop_elapsed=time.monotonic()-began
  self.assertFalse(self.t.armed);self.assertLess(stop_elapsed,.1)
  release.set();t.join(1);self.assertFalse(t.is_alive());self.assertFalse(errors)
  self.assertFalse(self.targets());np.testing.assert_array_equal(self.c._previous_action,0)
 def test_context_change_back_during_wait_never_sends_old_action(self):
  w=ReadyWorker()
  def change(request,timeout):self.c.command(.1,0,0);self.c.command(0,0,0);return w.take(request)
  w.wait_for_result=change;self.c._policy_worker=w;self.step();self.assertFalse(self.targets())
  np.testing.assert_array_equal(self.c._previous_action,0)
 def test_expired_result_is_not_freshened_or_committed(self):
  w=ReadyWorker()
  def late(request,timeout):self.clock.advance(motion.FRESH_SECONDS);return w.take(request)
  w.wait_for_result=late;self.c._policy_worker=w;self.step()
  self.assertFalse(self.targets());np.testing.assert_array_equal(self.c._previous_action,0)
  self.assertGreater(self.c._late_policy_frames,0)
 def test_range_failure_has_no_target_and_retains_failed_frame(self):
  w=ReadyWorker();w.value=10;self.c._policy_worker=w
  with self.assertRaises(motion.MotionError):self.step()
  self.assertFalse(self.targets());self.assertFalse(self.c._policy_trace[-1]['target_write_attempted'])
 def test_120_cycles_same_inputs_have_exact_sync_wire_and_recurrence(self):
  path=Path(self.tmp.name)/'cal.json';other_t=Transport('memory-only');other_t.gyro=[0.,0.,0.]
  clock=FakeClock();sessions={n:FakePolicy(n) for n in motion.MODEL_MODES}
  other=motion.MotionController(sessions,path,transport_factory=lambda _:other_t,clock=clock)
  other._connect_once();other.start_mode('walk',expected_generation=other._generation)
  w=ReadyWorker();self.c._policy_worker=w
  for i in range(120):
   w.value=float(.04*np.sin(i/9));sessions['walk'].output[:]=w.value
   self.step();clock.advance(.02);other.heartbeat(expected_generation=other._generation);other._cycle()
   np.testing.assert_array_equal(w.requests[-1],sessions['walk'].observation)
   np.testing.assert_array_equal(self.c._previous_action,other._previous_action)
   np.testing.assert_array_equal(self.c._smooth_action,other._smooth_action)
  other_wire=[v[4:] for k,v in other_t.requests if k==motion.SOURCE_TARGET]
  self.assertEqual([v[4:] for v in self.targets()],other_wire);self.assertEqual(len(self.targets()),120)
 def test_real_onnx_worker_memory_uart_same_cycle_and_exact_tensor(self):
  import onnxruntime as ort
  models=Path(__file__).parents[2]/'output/xgoduck_runtime_upstream/python'
  digest=hashlib.sha256((models/'xgoduck_walk.onnx').read_bytes()).hexdigest()
  w=PolicyWorker(models,dict(walk=digest));self.addCleanup(w.close)
  session=ort.InferenceSession(str(models/'xgoduck_walk.onnx'),providers=['CPUExecutionProvider'])
  self.c._policy_worker=w;self.c.sessions['walk']=session
  self.c._policy_profiles['walk']=motion.policy_profile(session)
  rows=[]
  for _ in range(20):
   # Generous already-defined freshness is NOT a new wait: the cycle stays20ms.
   self.step()
   if self.c._policy_trace:
    row=self.c._policy_trace[-1];obs=np.asarray(row['submitted_observation'],np.float32)
    raw=session.run(None,{session.get_inputs()[0].name:obs})[0].reshape(14)
    np.testing.assert_array_equal(raw,np.asarray(row['raw_action'],np.float32))
    rows.append(dict(sample=row['observation_sample_us'],commit=row['commit_sample_us']))
  self.assertEqual(len(self.targets()),20)
  self.assertTrue(all(r['sample']==r['commit'] for r in rows));self.assertEqual(w.submitted,20)
  self.assertEqual(w.completed,20);self.assertIsNone(self.c._policy_job)

class IPCTests(unittest.TestCase):
 def setUp(self):
  self.w=object.__new__(PolicyWorker);self.w._lock=threading.Lock();self.w._pending=1
  self.w._abandoned=self.w._closed=False;self.w.completed=self.w.discarded=0
  self.w.process=type('Process',(),{'poll':lambda _:None})();self.w._socket,self.peer=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
  self.w._socket.setblocking(False);self.addCleanup(self.peer.close);self.addCleanup(self.w._socket.close)
 def send(self):self.peer.send(policy_worker._encode(dict(id=1,action=[[.01]*14],compute_ms=.5)))
 def test_timeout_never_submits_or_consumes_later_result(self):
  began=time.monotonic();self.assertIsNone(self.w.wait_for_result(1,.01));elapsed=time.monotonic()-began
  self.assertGreaterEqual(elapsed,.008);self.assertLess(elapsed,.1);self.assertEqual(self.w._pending,1)
  self.send();self.assertIsNotNone(self.w.take(1));self.assertEqual(self.w.completed,1)
 def test_select_wait_releases_worker_lock_and_cancellation_stays_authoritative(self):
  result=[]
  def wait():
   try:result.append(self.w.wait_for_result(1,.05))
   except PolicyWorkerError:result.append('cancelled')
  t=threading.Thread(target=wait);t.start();time.sleep(.002)
  self.w.discard(1);self.send();t.join(.2)
  self.assertFalse(t.is_alive());self.assertEqual(result,['cancelled']);self.assertEqual(self.w.completed,0)
 def test_interrupted_wait_does_not_reset_deadline(self):
  original=policy_worker.select.select;timeouts=[]
  def interrupted(read,write,error,timeout):
   timeouts.append(timeout)
   if len(timeouts)==1:time.sleep(.006);raise InterruptedError()
   return original(read,write,error,timeout)
  with patch.object(policy_worker.select,'select',interrupted):self.assertIsNone(self.w.wait_for_result(1,.015))
  self.assertEqual(len(timeouts),2);self.assertLess(timeouts[1],timeouts[0]-.004)
 def test_worker_death_is_explicit_not_sync_fallback(self):
  self.w.process.poll=lambda:1
  with self.assertRaises(PolicyWorkerError):self.w.wait_for_result(1,.02)

class SchedulerBudgetTests(unittest.TestCase):
 def run_loop(self,costs,late=0,fail=False):
  tree=ast.parse((a.candidate/'service/worker-classes.py').read_text())
  node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='OneShotController')
  run=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name=='_run')
  ns=dict(motion=motion,exit_event=threading.Event(),health_error=None)
  exec(compile(ast.Module(body=[run],type_ignores=[]),'<actual packaged _run>','exec'),ns)
  class Loop:
   def __init__(self):
    self.now=0.;self._clock=lambda:self.now;self._lock=threading.RLock();self.rows=[];self.stops=[]
    owner=self
    class Closed:
     def is_set(self):return len(owner.rows)>=len(costs)
     def wait(self,amount):owner.now+=amount+late
    self._closed=Closed()
   def _connect_once(self):pass
   def _cycle(self):
    self.rows.append(dict(begin=self.now,deadline=self._policy_cycle_deadline))
    self.now+=costs[len(self.rows)-1]
    if fail:raise RuntimeError('injected cycle failure')
   def _stop_locked(self,why):self.stops.append(why)
   def _service_stop_locked(self):pass
   def _disconnect(self,why):self.reason=why
  loop=Loop();ns['_run'](loop);return loop
 def test_every_budget_with_skipped_slots_is_at_most_original_period(self):
  loop=self.run_loop([.001,.025,.045,.001]*5,late=.004)
  self.assertTrue(all(0<r['deadline']-r['begin']<=motion.PERIOD+1e-12 for r in loop.rows))
  self.assertIsNone(loop._policy_cycle_deadline)
 def test_exception_clears_budget_and_keeps_existing_stop_path(self):
  loop=self.run_loop([.001]*2,fail=True)
  self.assertIsNone(loop._policy_cycle_deadline);self.assertEqual(loop.reason,'injected cycle failure')
  self.assertIn('injected cycle failure',loop.stops)

suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(BudgetTests),unittest.defaultTestLoader.loadTestsFromTestCase(IPCTests),unittest.defaultTestLoader.loadTestsFromTestCase(SchedulerBudgetTests)])
result=unittest.TextTestRunner(verbosity=2).run(suite)
a.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),passed=result.wasSuccessful(),hardware_access=False,real_CPU_ONNX=True,scope='actual packaged mixin/controller/IPC with memory UART; not physical50Hz or walking'),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
