"""Run the packaged loop body against a clock, never against hardware."""
import argparse,ast,json,threading,types,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()

def loop_method(directory):
 tree=ast.parse((directory/'service/worker-classes.py').read_text())
 cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='OneShotController')
 run=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_run')
 ns=dict(motion=types.SimpleNamespace(PERIOD=.02,MotionError=RuntimeError),
         exit_event=threading.Event(),health_error=None)
 exec(compile(ast.Module(body=[run],type_ignores=[]),'<actual packaged _run>','exec'),ns)
 return ns['_run'],ns

class Clock:
 def __init__(self):self.now=0.
 def __call__(self):return self.now

class Closed:
 def __init__(self,owner,late):self.owner=owner;self.late=late;self.waits=[]
 def is_set(self):return len(self.owner.cycles)>=100
 def wait(self,amount):
  self.waits.append(amount);self.owner.clock.now+=amount+self.late

class Loop:
 def __init__(self,method,late=0.,cost=.001):
  self.clock=self._clock=Clock();self.cycles=[];self.stops=[]
  self._lock=threading.RLock();self._closed=Closed(self,late);self.cost=cost
  self._run=types.MethodType(method,self)
 def _connect_once(self):pass
 def _cycle(self):
  self.cycles.append(self.clock.now)
  cost=self.cost[len(self.cycles)-1] if isinstance(self.cost,list) else self.cost
  self.clock.now+=cost
 def _stop_locked(self,reason):self.stops.append(reason)
 def _service_stop_locked(self):pass
 def _disconnect(self,reason):self.disconnected=reason

new,namespace=loop_method(a.candidate);old,_=loop_method(a.baseline)

class DeadlineTests(unittest.TestCase):
 def setUp(self):namespace['exit_event'].clear()
 def test_no_late_wakeup_same_50hz(self):
  for method in (new,old):
   c=Loop(method);c._run()
   self.assertAlmostEqual(c.cycles[-1],1.98,places=10)
   self.assertTrue(c.stops);self.assertTrue(c.disconnected)
 def test_late_wakeup_does_not_accumulate(self):
  after=Loop(new,late=.004);before=Loop(old,late=.004)
  after._run();before._run()
  self.assertAlmostEqual(after.cycles[-1],1.984,places=10)
  self.assertAlmostEqual(before.cycles[-1],2.376,places=10)
 def test_arbitrary_overruns_match_pinned_source_formula(self):
  costs=[.001,.025,.045,.006]*25
  c=Loop(new,late=.002,cost=costs);c._run()
  now=deadline=0.;expected=[]
  for cost in costs:
   expected.append(now);now+=cost;deadline+=.02
   if deadline<=now:deadline+=(int((now-deadline)/.02)+1)*.02
   else:now=deadline+.002
  self.assertEqual(len(c.cycles),len(expected))
  for observed,reference in zip(c.cycles,expected):
   self.assertAlmostEqual(observed,reference,places=12)
  self.assertTrue(all(b>a for a,b in zip(c.cycles,c.cycles[1:])))
  self.assertEqual(len(c.cycles),100) # no replayed catch-up outputs
 def test_health_fault_keeps_stop_disconnect_path(self):
  namespace['health_error']='injected health error'
  try:
   c=Loop(new);c._run();self.assertFalse(c.cycles)
   self.assertEqual(c.disconnected,'injected health error')
   self.assertIn('injected health error',c.stops)
  finally:namespace['health_error']=None

result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DeadlineTests))
a.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),
 passed=result.wasSuccessful(),hardware_access=False,scope='actual packaged scheduler; simulated clock, not actual motor50Hz'),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
