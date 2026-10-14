"""Explicitly authorized fixed-robot checks with independent timed STOP.

Run on X5. No calibration changes, automatic reconnect, or motion retry.
Display polling is observation only; acceptance uses controller trace after UART
closure, distinct from a free-standing or floor-walking acceptance test.
"""
import argparse,importlib.util,json,os,subprocess,sys,time,urllib.error
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--authorized-fixed-robot',action='store_true',required=True);p.add_argument('--mode',choices=('hold','walk'),required=True);p.add_argument('--forward',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not(a.forward and a.mode!='walk')
a.output.mkdir(mode=0o700)
sp=importlib.util.spec_from_file_location('api','/dev/shm/duck-source-api.py');api=importlib.util.module_from_spec(sp);sp.loader.exec_module(api)
def emit(v):
 print(json.dumps(api.clean(v),ensure_ascii=False),flush=True)
 with (a.output/'events.jsonl').open('a') as f:f.write(json.dumps(api.clean(v),ensure_ascii=False)+'\n')
generation=None
# Keep opaque session and CSRF data inside the HTTP client only.
def action(name,**extra):
 global generation
 s=api.status();h=s['motion'].get('hardware') or {}
 if isinstance(h.get('generation'),int):generation=h['generation']
 body=dict(action=name,page_generation=s['native_access']['page_generation'],control_id=api.client,**extra)
 if isinstance(generation,int):body['generation']=generation
 import urllib.request
 req=urllib.request.Request('http://127.0.0.1:8767/api/motion',data=json.dumps(body).encode(),headers={'Content-Type':'application/json','X-CSRF-Token':s['token'],'Origin':'http://127.0.0.1:8767'})
 try:r=json.load(api.c.open(req,timeout=20))
 except urllib.error.HTTPError as e:
  emit(dict(event=name,error=e.read().decode(),status=e.code));raise
 g=r.get('generation',((r.get('motion') or {}).get('hardware') or {}).get('generation'))
 if isinstance(g,int):generation=g
 emit(dict(event=name,generation=g,receipt={k:r.get(k) for k in ('mode','connected','release_confirmed','action_receipt_protocol')}))
 return r

def sample(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  time.sleep(.2);s=api.status();m=s['motion'];h=m.get('hardware')
  emit(dict(event='display',phase=m['phase'],error=m['error'],hardware=api.clean(h),snapshot_compact=True))
  if m['phase']=='fault' or m['error']:raise RuntimeError('Runtime fault: '+str(m['error']))

watch_source="import runpy,time;time.sleep(10);a=runpy.run_path('/dev/shm/duck-source-api.py');a['action']('stop',stop_source='button');a['action']('disconnect')"
log=(a.output/'watchdog.log').open('x');watch=subprocess.Popen([sys.executable,'-c',watch_source],stdout=log,stderr=log,start_new_session=True)
released=False;failure=None;conn=None
def closed_release(value):
 m=value.get('motion') or {}
 proof=m.get('closed_proof') or {}
 return proof if proof.get('serial_closed') is True and proof.get('release_confirmed') is True else None
try:
 if api.status()['motion']['owns_connection']:
  assert action('stop',stop_source='button')['release_confirmed'] is True
  action('disconnect')
 r=action('connect');conn=r['motion']['connection_sequence'];emit(dict(event='connection',sequence=conn))
 action('start',mode=a.mode,support_confirmed=True);sample(2 if a.forward else 3)
 if a.forward:
  action('command',forward=.3,side=0.,yaw=0.);sample(2)
except BaseException as e:failure=str(e)
finally:
 try:
  receipt=action('stop',stop_source='button')
  proof=closed_release(receipt)
  released=receipt.get('release_confirmed') is True or proof is not None
  if released:
   if proof is None:
    disconnected=action('disconnect');m=disconnected['motion'];proof=m.get('closed_proof') or {}
   assert proof['serial_closed'] and proof['release_confirmed']
   (a.output/'closed-proof.json').write_text(json.dumps(api.clean(proof),ensure_ascii=False,indent=2)+'\n')
   watch.terminate();watch.wait(timeout=3)
 except BaseException as e:failure=(failure+'; ' if failure else '')+'Closure: '+str(e)
 log.close()
# Read only allowlisted controller trials after actual UART closure.
trials=[]
if released and conn is not None:
 pid=int(subprocess.check_output(['systemctl','show','xgoduck-workbench','--property=MainPID','--value']))
 roots=[]
 for proc in Path('/proc').iterdir():
  if proc.name.isdecimal():
   try:
    if int((proc/'stat').read_text().rsplit(')',1)[1].split()[1])==pid:roots.append(Path(os.path.realpath(proc/'cwd')))
   except FileNotFoundError:pass
 assert len(roots)==1
 for path in sorted((roots[0]/'native-evidence').glob('connection-%d-trial-*.json'%conn)):
  raw=path.read_bytes();d=json.loads(raw);assert d.get('hardware_walking_verified') is False
  (a.output/path.name).write_bytes(raw);trials.append(d)
frames=[x for d in trials for x in d['frames']];valid=[x for x in frames if x['mode']==a.mode and not x['error']]
unique=len({x['sample_us'] for x in valid});targets=len(valid)
summary=dict(mode=a.mode,forward_requested=a.forward,released=released,exception=failure,target_frames=targets,unique_position_samples=unique,completed_trials=len(trials),free_standing_verified=False,floor_walking_verified=False,parameters_changed=False)
summary['fixed_control_passed']=released and failure is None and unique>=10 and targets>=10 and all(not d.get('reason') for d in trials)
if a.forward:summary['forward_control_frames']=sum(x['command'][0]>.29 for x in valid);summary['fixed_control_passed'] &= summary['forward_control_frames']>=10
(a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');emit(dict(event='summary',**summary))
raise SystemExit(not summary['fixed_control_passed'])
