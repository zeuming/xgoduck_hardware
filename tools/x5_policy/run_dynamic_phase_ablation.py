"""Bounded paired phase-only simulation diagnosis; no real hardware access."""
import hashlib,json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'
PARENT=BASE/'dynamic-step-search-contact-v1/search.json'
PY=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/bin/python'
TASK=ROOT/'tools/x5_policy/diagnose_dynamic_step_phase.py'
OUT=BASE/'dynamic-step-phase-ablation-v1'

def main():
 OUT.mkdir(exist_ok=False)
 parent=json.loads(PARENT.read_text())
 assert parent['completed'] and parent['source_hashes_unchanged']
 assert not parent['best']['single_cycle_qualified']
 (OUT/'parent-search.json').write_bytes(PARENT.read_bytes())
 params=OUT/'parameters.json'
 params.write_text(json.dumps(dict(parameters=parent['best']['parameters'],source=PARENT.as_posix(),source_sha256=hashlib.sha256(PARENT.read_bytes()).hexdigest(),selection='highest score unsuccessful unloading attempt, not qualified gait'),indent=2)+'\n')
 report=dict(hardware_access=False,completed=False,maximum_wall_seconds=360,parameter_amplitudes_frozen=True,seed=0,batch=32,delays_seconds=[0.,-.3,-.15,.15,.3,.6,1.],runs=[])
 def save():(OUT/'plan.json').write_text(json.dumps(report,indent=2)+'\n')
 save();start=time.monotonic()
 for i,delay in enumerate(report['delays_seconds']):
  left=360-(time.monotonic()-start)
  if left<40:report['budget_stop']=True;break
  out=OUT/f'phase-{i:02d}'
  cmd=[str(PY),str(TASK),'--candidate',str(ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'),'--run',str(BASE/'kl-checked-ppo-extended'),'--basis',str(BASE/'dynamic-step-basis-v1/basis.json'),'--output',str(out),'--batch','32','--generations','1','--wall-seconds','70','--parameters',str(params),'--lift-delay-seconds',str(delay)]
  run=dict(delay_seconds=delay,output=str(out),command=cmd,started_wall_seconds=time.monotonic()-start);report['runs'].append(run);save()
  try:
   with (OUT/f'phase-{i:02d}.log').open('w') as log:r=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=min(85,left))
   run['exit_code']=r.returncode
   if r.returncode:raise RuntimeError(f'Phase {delay} process failed')
   data=json.loads((out/'search.json').read_text());run['completed']=data['completed'];run['best']=data.get('best')
   run['qualified_count']=sum(g['qualified_count'] for g in data['generations'])
   if not data['completed']:raise RuntimeError('Bounded trial incomplete')
  except Exception as error:
   run['error']=str(error);save();raise
  save();print(json.dumps(dict(delay=delay,qualified=run['qualified_count'],best_clearance_m=run['best']['maximum_swing_clearance_m'],wall=time.monotonic()-start)),flush=True)
 report['completed']=len(report['runs'])==len(report['delays_seconds']);report['wall_seconds']=time.monotonic()-start;save()

if __name__=='__main__':main()
