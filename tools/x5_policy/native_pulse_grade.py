"""Timestamp-based short-pulse observation gate; no physical safety certificate."""
import math

def grade_native(rows, hz):
 dt=1/hz;air=[False,False];lift_start=[None,None];land_start=[None,None];cycles=[0,0];sequence=[];last=None
 for r in rows:
  continuous=last is not None and 0<r['time_s']-last<=dt+1e-6
  for i,side in enumerate(('left','right')):
   foot=r['feet'][side];other=r['feet'][('right','left')[i]]
   lift=r['command_m_s']>0 and r['tilt_deg']<15 and foot['mesh_min_height_m']>=.003 and foot['vertical_load_n']<.001 and other['vertical_load_n']>=1
   lift_start[i]=(lift_start[i] if continuous and lift_start[i] is not None else r['time_s']) if lift else None
   if lift and r['time_s']-lift_start[i]>=.06-1e-9 and not air[i]:air[i]=True;sequence.append(side)
   land=foot['mesh_min_height_m']<.001 and foot['vertical_load_n']>=1
   land_start[i]=(land_start[i] if continuous and land_start[i] is not None else r['time_s']) if land else None
   if air[i] and land and r['time_s']-land_start[i]>=.06-1e-9:cycles[i]+=1;air[i]=False
  last=r['time_s']
 complete=bool(rows) and rows[-1]['task_complete'];safe=bool(rows) and all(r['tilt_deg']<15 and not r['failed_terms'] for r in rows)
 zero=[r for r in rows if r['time_s']<=5];start=rows[0]['root_position'] if rows else [0,0,0];heading=rows[0]['heading_rad'] if rows else 0
 zero_drift=max((math.hypot(r['root_position'][0]-start[0],r['root_position'][1]-start[1]) for r in zero),default=float('inf'))
 net=((rows[-1]['root_position'][0]-start[0])*math.cos(heading)+(rows[-1]['root_position'][1]-start[1])*math.sin(heading)) if rows else 0
 tail=rows[-hz:];drift=math.hypot(tail[-1]['root_position'][0]-tail[0]['root_position'][0],tail[-1]['root_position'][1]-tail[0]['root_position'][1]) if len(tail)==hz else float('inf')
 support=sum(all(r['feet'][s]['vertical_load_n']>=.1 and r['feet'][s]['mesh_min_height_m']<.0015 for s in ('left','right')) for r in tail)
 recovered=complete and len(tail)==hz and support>=math.ceil(.9*hz) and drift<=.005 and all(r['command_m_s']==0 and r['tilt_deg']<15 for r in tail)
 alternating=len(sequence)>=2 and all(a!=b for a,b in zip(sequence,sequence[1:]))
 return dict(accepted=complete and safe and zero_drift<=.015 and min(cycles)>=1 and alternating and net>=.02 and recovered,complete=complete,safe=safe,frames=len(rows),max_tilt_deg=max((r['tilt_deg'] for r in rows),default=None),zero_drift_m=zero_drift,completed_lift_land_cycles=cycles,lift_sequence=sequence,alternating=alternating,net_forward_m=net,recovered=recovered,recovery_drift_m=drift,final_supported_frames=support,hardware_verified=False,criterion='one complete lift/landing each side, alternating,3mm whole sole and opposing support>=1N, lift/landing samples span>=60ms; landing may finish in zero recovery; net>=2cm; 5s zero and final1s support')
