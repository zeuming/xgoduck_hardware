"""Read-only trace metrics: distinguish slower gait from delayed start/standing."""
import hashlib,json,math,statistics
from pathlib import Path

def summarize(path):
 path=Path(path);rows=[json.loads(s) for s in path.read_text().splitlines()]
 active=[r for r in rows if r['command_m_s']>0]
 if not active:return {'frames':len(rows),'no_forward':True}
 starts={s:[] for s in ('left','right')};spans={s:[] for s in starts}
 for side,other in [('left','right'),('right','left')]:
  run=[]
  for r in active+[None]:
   valid=r is not None and r['tilt_deg']<15 and r['feet'][side]['vertical_load_n']<.001 and r['feet'][side]['mesh_min_height_m']>=.003 and r['feet'][other]['vertical_load_n']>=1
   if valid and (not run or 0<r['time_s']-run[-1]['time_s']<=.020001):run.append(r)
   else:
    if run and run[-1]['time_s']-run[0]['time_s']>=.06-1e-9:starts[side].append(run[0]['time_s']);spans[side].append(run[-1]['time_s']-run[0]['time_s'])
    run=[r] if valid else []
 onsets=[t for ts in starts.values() for t in ts];first=min(onsets) if onsets else None
 periods=[b-a for ts in starts.values() for a,b in zip(ts,ts[1:])]
 steady=[r for r in active if first is not None and r['time_s']>=first]
 ids=[10,11,12,13,14,30,31,32,33,20,21,22,23,24];metrics=[]
 if len(steady)>1:
  for i,id in enumerate(ids):
   values=[r['processed_joint_rad'][i] for r in steady];deltas=[abs(b-a)*180/math.pi for a,b in zip(values,values[1:])]
   metrics.append(dict(id=id,goal_mean_abs_deg_s=sum(deltas)/(steady[-1]['time_s']-steady[0]['time_s']),goal_peak_to_peak_deg=(max(values)-min(values))*180/math.pi))
 return dict(trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),frames=len(rows),first_qualified_lift_after_command_s=first-active[0]['time_s'] if first else None,qualified_onsets=starts,qualified_spans=spans,median_samefoot_period_s=statistics.median(periods) if periods else None,active_after_first_lift_goal=sorted(metrics,key=lambda x:x['goal_mean_abs_deg_s'],reverse=True),scope='Descriptive actual trace, not a stability/plant acceptance or shape-retiming proof')

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('traces',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 assert not a.output.exists();a.output.write_text(json.dumps({str(p):summarize(p) for p in a.traces},indent=2))
