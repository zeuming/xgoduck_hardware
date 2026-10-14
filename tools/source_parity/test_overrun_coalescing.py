"""Execute actual host and pinned upstream loop ASTs against a deterministic clock.

No serial, device or model inference. This distinguishes the original upstream
unconditional final wait from the old host's else-only wait; previous tests
incorrectly treated those two algorithms as identical.
"""
import argparse
import ast
import json
from pathlib import Path
import threading
import types
import unittest

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--baseline',type=Path,required=True);p.add_argument('--upstream',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);args=p.parse_args()


def method(path,cls_name,method_name,namespace):
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name==cls_name)
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==method_name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[method_name]


class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now


class Closed:
    def __init__(self,owner):self.owner=owner;self.waits=[]
    def is_set(self):return len(self.owner.starts)>=100
    def wait(self,amount):
        self.waits.append(amount)
        if amount>0:self.owner.clock.now+=amount+self.owner.wakeup_late


class Runner:
    def __init__(self,kind,costs,late=0.):
        self.clock=self._clock=Clock();self.starts=[];self.costs=costs
        self._closed=self.stop_event=Closed(self);self.wakeup_late=late
        self._lock=self.lock=threading.RLock();self.stops=[];self.loop_missed=0
        ns=dict(time=types.SimpleNamespace(monotonic=self.clock),CONTROL_PERIOD=.02,
                motion=types.SimpleNamespace(PERIOD=.02,MotionError=RuntimeError),
                exit_event=threading.Event(),health_error=None)
        if kind=='upstream':file=args.upstream;cls='Controller';name='run'
        else:file=(args.candidate if kind=='candidate' else args.baseline)/'service/worker-classes.py';cls='OneShotController';name='_run'
        self.run=types.MethodType(method(file,cls,name,ns),self)
    def _connect_once(self):pass
    def _keep_warm_idle(self):pass
    def _cycle(self):
        self.starts.append(self.clock.now)
        self.clock.now+=self.costs[len(self.starts)-1]
    step=_cycle
    def _stop_locked(self,why):self.stops.append(why)
    def _service_stop_locked(self):pass
    def _disconnect(self,why):self.closed_reason=why


evidence={}


class OverrunTests(unittest.TestCase):
    def run_costs(self,name,costs,late=0.):
        out={}
        for kind in ('baseline','upstream','candidate'):
            r=Runner(kind,costs,late);r.run()
            out[kind]=r
        evidence[name]={kind:dict(starts=r.starts,waits=r._closed.waits,
            mean_interval_ms=1000*(r.starts[-1]-r.starts[0])/99,cycles=len(r.starts)) for kind,r in out.items()}
        return out

    def test_healthy_50hz_and_no_parameter_change(self):
        out=self.run_costs('healthy',[.001]*100)
        for r in out.values():self.assertAlmostEqual(r.starts[-1],1.98,places=10)

    def test_old_host_and_official_are_not_identical_after_overrun(self):
        out=self.run_costs('21ms_control',[.021]*100)
        self.assertAlmostEqual(out['upstream'].starts[1],.04)
        self.assertAlmostEqual(out['baseline'].starts[1],.021)
        self.assertAlmostEqual(out['baseline'].starts[2],.060)
        self.assertAlmostEqual(out['baseline'].starts[3],.081)
        for index,t in enumerate(out['candidate'].starts):self.assertAlmostEqual(t,index*.021,places=10)
        self.assertLess(evidence['21ms_control']['candidate']['mean_interval_ms'],evidence['21ms_control']['baseline']['mean_interval_ms'])

    def test_slight_overrun_followed_by_fast_cycle_has_no_extra_empty_slot(self):
        out=self.run_costs('alternating21_10ms',[.021,.010]*50)
        self.assertAlmostEqual(out['baseline'].starts[2],.060)
        self.assertAlmostEqual(out['candidate'].starts[2],.041)
        self.assertTrue(all(b-a>=.02-1e-12 for a,b in zip(out['candidate'].starts,out['candidate'].starts[1:])))

    def test_wakeup_lateness_and_large_overrun_never_replay_old_cycles(self):
        out=self.run_costs('mixed_late',[.001,.025,.045,.006]*25,late=.002)
        c=out['candidate'];self.assertEqual(len(c.starts),100)
        self.assertTrue(all(b-a>=.02-1e-12 for a,b in zip(c.starts,c.starts[1:])))
        self.assertTrue(c.stops);self.assertTrue(c.closed_reason)


result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(OverrunTests))
args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
    failures=len(result.failures),errors=len(result.errors),hardware_access=False,
    physical_50hz_verified=False,evidence=evidence),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
