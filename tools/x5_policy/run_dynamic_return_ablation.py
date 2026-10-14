"""Bounded paired offline preparation-return / swing-lift timing study."""
import hashlib,json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'
PY=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/bin/python'


def main():
 out=BASE/'dynamic-step-return-ablation-v1';out.mkdir(exist_ok=False)
 parameters=BASE/'dynamic-step-phase-ablation-v1/parameters.json'
 report=dict(completed=False,hardware_access=False,maximum_wall_seconds=300,parameter_path=str(parameters),parameter_sha256=hashlib.sha256(parameters.read_bytes()).hexdigest(),seed=0,batch=32,return_fractions=[None,0.,.5,1.,1.5],runs=[])
 (out/'source.py').write_bytes(Path(__file__).read_bytes())
 def save():(out/'plan.json').write_text(json.dumps(report,indent=2)+'\n')
 save();started=time.monotonic()
 try:
  for idx,frac in enumerate(report['return_fractions']):
   left=300-(time.monotonic()-started)
   if left<40:report['budget_stop']=True;break
   target=out/f'case-{idx:02d}'
   cmd=[str(PY),'tools/x5_policy/diagnose_dynamic_step_return.py','--candidate','output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json','--run',str(BASE/'kl-checked-ppo-extended'),'--basis',str(BASE/'dynamic-step-basis-v1/basis.json'),'--output',str(target),'--batch','32','--generations','1','--wall-seconds','75','--parameters',str(parameters)]
   if frac is not None:cmd.extend(['--return-at-lift-fraction',str(frac)])
   rec=dict(return_fraction=frac,output=str(target),command=cmd);report['runs'].append(rec);save()
   with (out/f'case-{idx:02d}.log').open('w') as log:r=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=min(85,left))
   rec['exit_code']=r.returncode
   if r.returncode:raise RuntimeError(f'Case {frac} failed')
   s=json.loads((target/'search.json').read_text());rec['completed']=s['completed'];rec['best']=s['best'];rec['qualified_count']=s['generations'][0]['qualified_count'];save()
   if not s['completed']:raise RuntimeError('Trial incomplete')
   print(json.dumps(dict(fraction=frac,qualified=rec['qualified_count'],best=rec['best'],wall_seconds=time.monotonic()-started)),flush=True)
  report['completed']=len(report['runs'])==len(report['return_fractions'])
 except Exception as e:report['error']=str(e);raise
 finally:report['wall_seconds']=time.monotonic()-started;save()

if __name__=='__main__':main()
