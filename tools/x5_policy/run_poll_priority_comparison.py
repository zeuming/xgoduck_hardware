"""Local fake-UART/real-ONNX matched benchmark; loopback HTTP pressure only."""
import json,subprocess,time
from pathlib import Path


def main():
 root=Path(__file__).resolve().parents[2];b=root/'output/duck_sim/overnight-20261011';out=b/'poll-priority-v5-comparison-v2';out.mkdir(exist_ok=False)
 py=root/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/bin/python';rows=[];start=time.monotonic()
 report=dict(hardware_access=False,host='local_Mac_CPU_not_X5',completed=False,maximum_wall_seconds=220,runs=rows)
 def save():(out/'progress.json').write_text(json.dumps(report,indent=2)+'\n')
 save()
 try:
  for name,source,mode in [('v4-both','host-deferred-v4','both'),('v5-both','host-poll-priority-v5','both'),('v5-none','host-poll-priority-v5','none'),('v4-none','host-deferred-v4','none')]:
   cmd=[str(py),'tools/x5_policy/validate_serial_owner.py','--source',str(b/source),'--output',str(out/f'{name}.json'),'--benchmark','--models','output/x5_policy_bundle/models','--transport-timing',str(b/'verified-delay-schedule.json'),'--delay-mode',mode,'--warmup-seconds','3']
   row=dict(name=name,command=cmd);rows.append(row);save()
   with (out/f'{name}.log').open('w') as log:r=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=min(55,220-(time.monotonic()-start)))
   row['exit_code']=r.returncode;save()
   if r.returncode:raise RuntimeError(f'{name} failed')
   data=json.loads((out/f'{name}.json').read_text());row.update(hz=[v['target_write_hz'] for v in data['cases']],sample_hz=[v['sample_receive_hz'] for v in data['cases']],min_gap_ms=[v['min_target_gap_ms'] for v in data['cases']]);print(json.dumps(row),flush=True);save()
  report['completed']=True
 except Exception as error:report['error']=str(error);raise
 finally:report['wall_seconds']=time.monotonic()-start;save()

if __name__=='__main__':main()
