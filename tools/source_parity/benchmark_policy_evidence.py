"""Bounded in-memory evidence overhead; parse method without importing control."""
import argparse,ast,collections,json,time,types
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);a=p.parse_args()
tree=ast.parse(a.source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MotionController')
fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_record_policy_frame_locked')
ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual detached evidence sink>','exec'),ns)
c=types.SimpleNamespace(_policy_trace=collections.deque(maxlen=3000),_policy_trace_total=0,_last_sample=100,_clock=time.monotonic)
record=types.MethodType(ns[fn.name],c)
f=dict(generation=2,mode='walk',selected='walk',observation_sample=90,
 observation_received=0.,submission_monotonic=0.,submitted_observation=np.zeros((1,61),np.float32),
 previous=np.zeros(14,np.float32),smooth=np.zeros(14,np.float32))
action=np.zeros(14,np.float32);target=[0.]*15;samples=[]
start=time.monotonic();cpu=time.thread_time()
for i in range(10000):
 if time.monotonic()-start>15:raise TimeoutError('Evidence overhead benchmark budget reached')
 began=time.perf_counter();record(f,target,action,action,1.);samples.append((time.perf_counter()-began)*1e6)
elapsed=time.monotonic()-start;cpu_elapsed=time.thread_time()-cpu
print(json.dumps(dict(calls=len(samples),ring_rows=len(c._policy_trace),mean_us=float(np.mean(samples)),p95_us=float(np.quantile(samples,.95)),max_us=max(samples),thread_cpu_us_per_frame=cpu_elapsed/len(samples)*1e6,wall_seconds=elapsed,hardware_access=False,scope='Actual evidence sink only, not full control-loop frequency or hardware timing')))
