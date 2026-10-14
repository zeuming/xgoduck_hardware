"""Bounded CPU-only official-model cadence/sensor-delay comparison. No training."""
import argparse,hashlib,json,subprocess,time,os
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--calibration',type=Path,required=True)
p.add_argument('--speed',type=float,default=.3)
p.add_argument('--diagonal-only',action='store_true',help='Compare only 50Hz/current sensor vs40Hz/one-step older sensor')
a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
root=Path(__file__).resolve().parents[2]
python=root/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/bin/python'
evaluator=root/'tools/x5_policy/evaluate_official_causal_factors.py'
began=time.monotonic();report=dict(pid=os.getpid(),hardware_access=False,training=False,
 evaluator_sha256=hashlib.sha256(evaluator.read_bytes()).hexdigest(),
 calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest(),
 maximum_total_wall_seconds=360,cases=[],complete=False)
report.update(speed_m_s=a.speed,diagonal_only=a.diagonal_only)
def save():
 report['wall_seconds']=time.monotonic()-began
 (a.output/'progress.json').write_text(json.dumps(report,indent=2)+'\n')
save()
try:
 for seed in (0,3):
  for hz,lag in (((50,0),(40,1)) if a.diagonal_only else ((50,0),(50,1),(40,0),(40,1))):
   if time.monotonic()-began>260:raise TimeoutError('Do not start another case near total budget')
   name=f'hz{hz}-lag{lag}-seed{seed}';row=dict(name=name,seed=seed,hz=hz,lag=lag,state='running')
   report['cases'].append(row);save()
   cmd=[str(python),str(evaluator),'--output',str(a.output/name),'--seed',str(seed),
    '--speed',str(a.speed),'--factors','gains,ema,ranges','--official-runtime',
    '--head-range-projection','--quantize-goals','--saved-calibration',str(a.calibration),
    '--control-hz',str(hz),'--sensor-lag-steps',str(lag)]
   with (a.output/(name+'.log')).open('x') as log:
    result=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=100)
   row.update(state='complete' if result.returncode==0 else 'failed',returncode=result.returncode);save()
   if result.returncode:raise RuntimeError(name+' failed; inspect log before continuing')
 report['complete']=True
finally:save()
