"""Actual packaged OneShot._run + official ONNX + fake UART with 10ms STATE.

A CPU/IPC/scheduler harness, not measured X5 UART, stability or motor performance.
"""
import argparse,ast,hashlib,json,struct,sys,tempfile,threading,time
from pathlib import Path
import numpy as np
import onnxruntime as ort
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--models-dir',type=Path);p.add_argument('--execution',choices=('worker','synchronous'),default='worker');p.add_argument('--cycles',type=int,default=50);p.add_argument('--state-delay-ms',type=float,default=10.);a=p.parse_args()
if not 10<=a.cycles<=1000 or not 0<=a.state_delay_ms<=100:raise ValueError('Bounded memory-only benchmark required')
sys.path[:0]=[str(a.candidate/'policy'),str(a.candidate/'service'),str(Path(__file__).parents[1]/'x5_policy')]
import os
def deny_device(event,args):
 if event=='open' and args and isinstance(args[0],(str,bytes)):
  path=os.path.realpath(os.fsdecode(args[0]))
  if path.startswith('/dev/') and path!='/dev/null' and not path.startswith('/dev/shm/'):
   raise PermissionError('Isolated loop harness cannot open devices')
sys.addaudithook(deny_device)
import motion
from policy_worker import PolicyWorker
from test_motion import FakeTransport,ready_config

def load(path,name,ns):
 node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name==name)
 exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns);return ns[name]
Transport=load(Path(__file__).with_name('test_source_host.py'),'Transport',dict(FakeTransport=FakeTransport,motion=motion,struct=struct))
ns=dict(motion=motion,time=time,request_counts={},exit_event=threading.Event(),health_error=None)
Controller=load(a.candidate/'service/worker-classes.py','OneShotController',ns)
models=a.models_dir or Path(__file__).parents[2]/'output/xgoduck_runtime_upstream/python'
model=models/'xgoduck_walk.onnx';digest=hashlib.sha256(model.read_bytes()).hexdigest()
options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
sessions={n:ort.InferenceSession(str(models/('xgoduck_'+n+'.onnx')),sess_options=options,providers=['CPUExecutionProvider']) for n in ('walk','getup')}
hashes={n:hashlib.sha256((models/('xgoduck_'+n+'.onnx')).read_bytes()).hexdigest() for n in sessions}
class InstrumentedWorker(PolicyWorker):
 def __init__(self,*args,**kwargs):
  self.waits=[];super().__init__(*args,**kwargs)
 def wait_for_result(self,request,timeout):
  begin=time.monotonic();cpu=time.thread_time()
  try:
   result=super().wait_for_result(request,timeout)
   self.waits.append(dict(request=request,budget_ms=timeout*1000,wall_ms=(time.monotonic()-begin)*1000,cpu_ms=(time.thread_time()-cpu)*1000,ready=result is not None))
   return result
  except BaseException as error:
   self.waits.append(dict(request=request,budget_ms=timeout*1000,error=str(error)))
   raise
class Wire(Transport):
 def request(self,kind,payload=b''):
  if kind==motion.SOURCE_STATE:time.sleep(a.state_delay_ms/1000)
  return super().request(kind,payload)
class Loop(Controller):
 def _connect_once(self):
  super()._connect_once();self.start_mode('walk',expected_generation=self._generation)
  self.lease_stop=threading.Event()
  def lease():
   while not self.lease_stop.wait(.1):
    try:self.heartbeat(expected_generation=self._generation,minimal=True)
    except motion.MotionError as error:
     # The finite loop closes the generation before the lease thread joins.
     # Record unexpected live errors; do not print a benign closure race as a
     # background exception, or silently discard errors while still running.
     if not self._closed.is_set():self.lease_errors.append(str(error))
     return
  self.lease_thread=threading.Thread(target=lease,daemon=True);self.lease_thread.start()
 def _cycle(self):
  begin=time.monotonic();deadline=getattr(self,'_policy_cycle_deadline',None)
  super()._cycle();self.cycles.append(dict(begin=begin,end=time.monotonic(),deadline=deadline,phases=list(getattr(self,'_last_production_control_phases',()))))
  if len(self.cycles)>=a.cycles:self._closed.set()
with tempfile.TemporaryDirectory() as tmp:
 path=Path(tmp)/'cal.json';path.write_text(json.dumps(ready_config()))
 worker=InstrumentedWorker(models,hashes) if a.execution=='worker' else None;wire=Wire('memory-only');wire.gyro=[0.,0.,0.]
 c=Loop(sessions,path,transport_factory=lambda _:wire,clock=time.monotonic)
 c._policy_worker=worker;c.cycles=[];c.lease_errors=[]
 try:
  began=time.monotonic();c._run();wall=time.monotonic()-began
  traces=c.completed_trial_traces();rows=[row for trial in traces for row in trial.get('policy_frames',[])]
  age_ms=[(row['target_write_started_monotonic']-row['submission_monotonic'])*1000 for row in rows if 'target_write_started_monotonic' in row]
  summary=dict(hardware_access=False,hardware_UART=False,real_CPU_ONNX=True,simulated_STATE_delay_ms=a.state_delay_ms,heartbeat_thread_period_ms=100,heartbeat_minimal=True,lease_errors=list(c.lease_errors),
   candidate=str(a.candidate),model_sha256=digest,cycles=c.cycles,wall_seconds=wall,
   total_targets=sum(k==motion.SOURCE_TARGET for k,_ in wire.requests),policy_frames=len(rows),
   observation_equals_commit_samples=sum(row['observation_sample_us']==row['commit_sample_us'] for row in rows),
   policy_input_to_write_start_ms=age_ms,rows=rows,
   terminal_error=getattr(c,'_terminal_error',None),mode=c._mode,transport_closed=wire.closed,
   fake_actual_off=not wire.armed and wire.torque==0,release_verification=c.release_verification(),
   execution=a.execution,worker=worker.status() if worker else None,worker_waits=worker.waits if worker else [],scope='Mac actual packaged control/IPC loop and fake 10ms serial; NOT measured X5 Hz or physical walk')
  a.output.write_text(json.dumps(summary,indent=2)+'\n')
  assert summary['terminal_error'] is None and summary['transport_closed'] and summary['fake_actual_off']
  assert not c.lease_errors and len(c.cycles)==a.cycles and len(rows)>=a.cycles-5
 finally:
  if hasattr(c,'lease_stop'):c.lease_stop.set();c.lease_thread.join(1)
  if worker:worker.close()
