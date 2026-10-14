"""Native decoder and real ONNX validation with a simulated UART, never hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time


def scheduler_delay(elapsed, *, period=.02, scheduler='native'):
    """Match MotionController._run, including its minimum 1 ms yield."""
    return max(.001 if scheduler == 'native' else 0., period-elapsed)


def timing_summary(rows):
    """Report injected wait separately from controller CPU and scheduling wait."""
    if not rows:
        return {}
    fields = ('cycle_wall_ms', 'cycle_cpu_ms', 'transport_wall_ms',
              'injected_wait_ms', 'transport_excess_ms', 'outside_transport_ms')
    return {field: sum(row[field] for row in rows)/len(rows) for field in fields}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--models',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=('decode','stress'),default='stress')
    parser.add_argument('--transport-timing',type=Path,
        help='Recorded last-stop transport timings; replay durations without opening UART')
    parser.add_argument('--scheduler',choices=('native','legacy'),default='native',
        help='native reproduces the deployed minimum 1 ms cycle yield; legacy compares prior reports')
    parser.add_argument('--paired-feedback',action='store_true',
        help='Decode complete 238-byte v2 voltage and sample-bound goal packets in an ideal fake plant')
    args=parser.parse_args()
    sys.path[:0]=[str(args.source.resolve()/'policy'),str(Path(__file__).resolve().parent)]
    def deny_hardware(event,values):
        if event=='open' and values and isinstance(values[0],(str,bytes)):
            path=os.path.realpath(os.fsdecode(values[0]))
            if path.startswith('/dev/') and path!='/dev/null' and not path.startswith('/dev/shm/'):
                raise PermissionError('Validation cannot open robot devices')
    sys.addaudithook(deny_hardware)
    sys.setswitchinterval(.001)  # Match the deployed supervisor.
    import numpy as np
    import onnxruntime as ort
    import motion
    from rl_core import State
    from test_motion import FakeTransport,ready_config
    if args.paired_feedback:
        from causal_test_transport import PairedFakeTransport
        FakeTransport=PairedFakeTransport
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
    session=ort.InferenceSession(str(args.models/'xgoduck_walk.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
    model_hash=hashlib.sha256((args.models/'xgoduck_walk.onnx').read_bytes()).hexdigest()
    latency_rows = {}
    if args.transport_timing:
        recorded = json.loads(args.transport_timing.read_text())['transport']['requests']
        for row in recorded:
            if row.get('error') is None and row['kind'] in (motion.STATE, motion.VOLTAGE_STATE, motion.TARGET):
                kind = motion.STATE if row['kind'] == motion.VOLTAGE_STATE else row['kind']
                latency_rows.setdefault(kind, []).append((row['elapsed_ms'] + row.get('identity_proof_ms', 0)) / 1000)
        if set(latency_rows) != {motion.STATE, motion.TARGET} or any(
                not np.isfinite(x) or not 0 <= x < .16 for values in latency_rows.values() for x in values):
            raise ValueError('Timing replay requires successful bounded STATE and TARGET durations')
    with tempfile.TemporaryDirectory() as tmp:
        config=Path(tmp)/'calibration.json';config.write_text(json.dumps(ready_config()))
        if args.mode=='decode':
            c=motion.MotionController({'walk':session},config,allow_fixed_head_walk=True)
            random=np.random.default_rng(917)
            rows=[]
            previous=smooth=np.zeros(14,np.float32)
            for i in range(100):
                st=State([0,0,9.8],[0,0,0],(np.asarray(c._stand_deg)+random.normal(0,2,15)).tolist(),
                    random.normal(0,1,15).tolist(),[.01,-.01,-.9999],[.01,-.02,.03])
                target,action,smooth,_=c._infer_policy_target('walk',st,previous,np.array([.01,0,0],np.float32),smooth,0.)
                rows.append(dict(target=list(target),raw=action.tolist(),smooth=smooth.tolist(),diagnostics=target.diagnostics))
                previous=action
            report=dict(model_sha256=model_hash,rows=rows,hardware_access=False)
        else:
            from policy_worker import PolicyWorker
            class WatchdogTransport(FakeTransport):
                def __init__(self,port):
                    super().__init__(port);self.last_goal=None;self.watchdog_stops=0;self.samples=[];self.targets=[]
                    self.delay_cursor = {}
                    self.measurements = []
                    self.sample_origin = time.monotonic()
                def request(self,kind,payload=b''):
                    began=time.monotonic()
                    delays = latency_rows.get(motion.STATE if kind==motion.VOLTAGE_STATE else kind)
                    injected=0.
                    if delays:
                        index = self.delay_cursor.get(kind, 0)
                        injected=delays[index % len(delays)]
                        time.sleep(injected)
                        self.delay_cursor[kind] = index + 1
                    now=time.monotonic()
                    if self.armed and self.last_goal is not None and now-self.last_goal>=.250:
                        self.armed=False;self.torque=0;self.cookie=0;self.watchdog_stops+=1
                    if kind in (motion.STATE,motion.VOLTAGE_STATE):
                        # The old fake advanced 20 ms even when a real cycle
                        # needed 30 ms. That gave filters an artificial MCU clock.
                        self.sample_us=int((now-self.sample_origin)*1e6)&0xffffffff
                        self.freeze_sample=True
                    answer=super().request(kind,payload)
                    if kind in (motion.STATE,motion.VOLTAGE_STATE):self.samples.append(now)
                    if kind in (motion.ARM,motion.TARGET):self.last_goal=now
                    if kind==motion.TARGET:self.targets.append(now)
                    self.measurements.append(dict(kind=kind,injected_wait_ms=injected*1000,
                        transport_wall_ms=(time.monotonic()-began)*1000))
                    return answer
            transport=WatchdogTransport('fake-only')
            worker=PolicyWorker(args.models,{'walk':model_hash})
            c=motion.MotionController({'walk':session},config,transport_factory=lambda port:transport,
                allow_fixed_head_walk=True)
            c.attach_policy_worker(worker)
            done=threading.Event();readers=[];display_child=None
            try:
                c._connect_once();c.capture_head_clearance(clearance_confirmed=True,expected_generation=c._generation)
                c.start_mode('hold',support_confirmed=True)
                # Advance only the HOLD interpolation timer in a simulated plant.
                c._hold_begin-=3.1
                for _ in range(3):c._cycle();time.sleep(.02)
                c.start_mode('walk',support_confirmed=True)
                # Actual code/ONNX, idealised plant; this cannot prove balance.
                snapshot=Path(tmp)/'display.json'
                def reader():
                    while not done.is_set():
                        temporary=snapshot.with_suffix('.tmp')
                        temporary.write_text(json.dumps(c.status(),allow_nan=False))
                        os.replace(temporary,snapshot)
                        done.wait(.2)  # Production publication cadence.
                cases=[]
                def cycles(seconds):
                    end=time.monotonic()+seconds
                    rows=[]
                    while time.monotonic()<end:
                        # Production uses a small heartbeat receipt. Formatting
                        # full status here falsely adds one display dump at 50 Hz.
                        start=time.monotonic();cpu=time.thread_time();n=len(transport.measurements)
                        c.heartbeat(minimal=True);c._cycle()
                        elapsed=time.monotonic()-start
                        requests=transport.measurements[n:]
                        transport_wall=sum(row['transport_wall_ms'] for row in requests)
                        injected=sum(row['injected_wait_ms'] for row in requests)
                        rows.append(dict(cycle_wall_ms=elapsed*1000,
                            cycle_cpu_ms=(time.thread_time()-cpu)*1000,
                            transport_wall_ms=transport_wall,injected_wait_ms=injected,
                            transport_excess_ms=transport_wall-injected,
                            outside_transport_ms=elapsed*1000-transport_wall))
                        if c._mode!='walk':raise AssertionError('Fake plant unexpectedly stopped: '+str(c._error))
                        time.sleep(scheduler_delay(time.monotonic()-start,scheduler=args.scheduler))
                    return rows
                def segment(name,seconds):
                    n=len(transport.samples);nt=len(transport.targets)
                    rows=cycles(seconds);samples=transport.samples[n:];targets=transport.targets[nt:]
                    wall=[row['cycle_wall_ms'] for row in rows]
                    gaps=[(b-a)*1000 for a,b in zip(samples,samples[1:])]
                    cases.append(dict(name=name,duration_s=seconds,samples=len(samples),targets=len(targets),
                        max_sample_gap_ms=max(gaps,default=0),max_control_cycle_ms=max(wall),
                        mean_target_hz=(len(targets)-1)/(targets[-1]-targets[0]) if len(targets)>1 else None,
                        mean_control_cycle_ms=float(np.mean(wall)),
                        p95_control_cycle_ms=float(np.percentile(wall,95)),
                        cost_breakdown_mean_ms=timing_summary(rows),
                        max_target_gap_ms=max([(b-a)*1000 for a,b in zip(targets,targets[1:])],default=0)))
                segment('no_display_readers',3.)
                snapshot.write_text(json.dumps(c.status(),allow_nan=False))
                display_child=subprocess.Popen([sys.executable,'-I','-B',str(Path(__file__).resolve()),
                    '--display-worker',str(snapshot),str(Path(tmp)/'http-proof.json')])
                t=threading.Thread(target=reader);t.start();readers.append(t)
                segment('cached_http_process_eight_readers',3.)
                # Stop only our inference child, never a robot service/process.
                os.kill(worker.process.pid,signal.SIGSTOP)
                before=len(transport.samples);history=c._previous_action.copy()
                cycles(.12)
                during=len(transport.samples)-before
                os.kill(worker.process.pid,signal.SIGCONT)
                segment('after_120ms_inference_pause',1.)
                if during<4:raise AssertionError('Inference pause blocked sensor polling')
                if c._late_policy_frames<1:raise AssertionError('Expired inference was not rejected')
                segment('recovered',1.)
                # Process exit must be explicit and must not fall back to the
                # ONNX session in the control process or auto-restart the child.
                worker.process.terminate();worker.process.wait(timeout=2.)
                failed=None
                for _ in range(3):
                    try:c._cycle()
                    except motion.MotionError as error:
                        failed=str(error);break
                    time.sleep(.02)
                if failed is None:raise AssertionError('Worker death did not surface')
                c.stop()
                display_child.wait(timeout=15.)
                http_proof=json.loads((Path(tmp)/'http-proof.json').read_text())
                if http_proof['requests']<100:raise AssertionError('HTTP pressure did not run')
                if http_proof['errors']:raise AssertionError('HTTP reader errors invalidate display pressure proof')
                report=dict(model_sha256=model_hash,hardware_access=False,plant='idealized_fake_UART',
                    cases=cases,http_pressure=http_proof,samples_during_120ms_inference_pause=during,
                    discarded_frames=c._late_policy_frames,worker_exit_error=failed,
                    torque_released=not transport.armed,worker=worker.status(),
                    watchdog_stops=transport.watchdog_stops,
                    transport_timing_sha256=hashlib.sha256(args.transport_timing.read_bytes()).hexdigest() if args.transport_timing else None,
                    timing_replay='recorded request durations; ideal joint following; no hardware' if latency_rows else 'zero-latency fake UART',
                    scheduler=args.scheduler,minimum_cycle_yield_ms=1 if args.scheduler=='native' else 0,
                    fake_feedback_protocol=2 if args.paired_feedback else 0,
                    fake_feedback_payload_bytes=238 if args.paired_feedback else 106,
                    sample_clock='wall-monotonic (not fixed 20 ms per request)',
                    host=dict(platform=sys.platform,python=sys.version.split()[0]),
                    replay_limitations=['Historical request time includes then-current transport parsing and host scheduling.',
                        'Sleeping replays historical wall time, not current wire/device execution time.',
                        'Independent per-kind sequences preserve marginal delays, not original cross-request correlation.',
                        ('Full v2 voltage/sample-bound goal fields decoded; UART I/O and MCU scan cost are simulated.'
                         if args.paired_feedback else
                         'Fake STATE omits voltage/motion-evidence extensions; their full decoding cost is not measured.'),
                        'No MCU/servo model, force, contact, or balance dynamics are simulated.'],
                    scope='Software timing only; does not validate balance or gait')
            finally:
                done.set()
                for reader_thread in readers:reader_thread.join(timeout=2.)
                c.close();worker.close()
                if display_child is not None and display_child.poll() is None:
                    display_child.terminate();display_child.wait(timeout=2.)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='rows'},ensure_ascii=False))

def display_worker(snapshot, proof):
    # Like the deployed HTTP child, requests only read a cached publication;
    # they have no controller object, IPC command channel, or serial handle.
    import http.server
    import urllib.request
    path=Path(snapshot);counts=[0];errors=[];lock=threading.Lock()
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            body=path.read_bytes()
            self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers()
            self.wfile.write(body)
            with lock:counts[0]+=1
        def log_message(self,*args):pass
    class Server(http.server.ThreadingHTTPServer):
        request_queue_size=32
    server=Server(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url='http://127.0.0.1:%d/'%server.server_port
    deadline=time.monotonic()+6.
    def get():
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while time.monotonic()<deadline:
            try:
                with opener.open(url,timeout=2.) as response:json.loads(response.read())
            except Exception as error:
                with lock:errors.append(type(error).__name__)
                return
            time.sleep(.002)
    readers=[threading.Thread(target=get) for _ in range(8)]
    for reader in readers:reader.start()
    for reader in readers:reader.join()
    server.shutdown();server.server_close()
    Path(proof).write_text(json.dumps(dict(requests=counts[0],readers=8,
        source='cached_snapshot_only',controller_calls=0,errors=errors)))

if __name__=='__main__':
    if len(sys.argv)==4 and sys.argv[1]=='--display-worker':display_worker(*sys.argv[2:])
    else:main()
