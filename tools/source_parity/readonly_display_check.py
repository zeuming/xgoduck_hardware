"""X5 read-only display verification, no ARM or position target commands."""
import contextlib,io,json,runpy,subprocess,time
from pathlib import Path
api=runpy.run_path('/dev/shm/duck-source-api.py')
def action(name,**kw):
 with contextlib.redirect_stdout(io.StringIO()):return api['action'](name,**kw)
rows=[];failure=None;closed=None
watch=subprocess.Popen(['python3','-c',"import contextlib,io,runpy,time;time.sleep(10);a=runpy.run_path('/dev/shm/duck-source-api.py');f=io.StringIO();\nwith contextlib.redirect_stdout(f): a['action']('stop',stop_source='button');a['action']('disconnect')"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
try:
 assert not api['status']()['motion']['owns_connection']
 action('connect')
 for i in range(12):
  time.sleep(.2);s=api['status']();m=s['motion'];h=m.get('hardware') or {}
  rows.append(dict(phase=m['phase'],sample_us=h.get('sample_us'),age_ms=h.get('age_ms'),
   imu_age_ms=h.get('imu_age_ms'),captured=h.get('status_captured_monotonic'),
   armed=h.get('armed'),mode=h.get('mode'),rates=h.get('rates'),
   publication_version=m.get('hardware_publication_version'),sample_age_ms=m.get('sample_age_ms'),
   last_age_ms=(m.get('last_hardware') or {}).get('age_ms')))
  if h: assert h['mode']=='off' and h['armed'] is False
 assert sum(x['sample_us'] is not None for x in rows)>=8, 'Fewer than 8/12 visible fresh samples'
except BaseException as e:failure=repr(e)
finally:
 try:
  stopped=action('stop',stop_source='button')
  assert stopped.get('release_confirmed') or (stopped.get('motion') or {}).get('closed_proof',{}).get('release_confirmed')
  closed=action('disconnect')['motion']['closed_proof']
  assert closed['serial_closed'] and closed['release_confirmed']
  assert closed['request_counts'].get('4',0)==0 and closed['request_counts'].get('31',0)==0
  watch.terminate();watch.wait(timeout=3)
 except BaseException as e:failure=(failure+'; ' if failure else '')+str(e)
result=dict(passed=failure is None,error=failure,samples=rows,
 closed_proof={k:closed.get(k) for k in ('serial_closed','release_confirmed','request_counts')} if closed else None,
 no_ARM_or_TARGET=True if closed and not closed['request_counts'].get('4',0) and not closed['request_counts'].get('31',0) else False)
print(json.dumps(result,ensure_ascii=False))
raise SystemExit(failure is not None)
