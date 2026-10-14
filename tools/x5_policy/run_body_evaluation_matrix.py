"""Sequential independent matched-seed evaluation; no training/device actions."""
import argparse,hashlib,json,subprocess,sys,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--layout',choices=('official','pilot','snapshot64'),default='official')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',type=Path)
    p.add_argument('--modes',default='walk,getup,pick,roulade')
    p.add_argument('--seeds',default='0,3',help='Explicit independent evaluation seeds, default0,3.')
    p.add_argument('--evaluator',type=Path,default=Path(__file__).with_name('evaluate_current_body_policy.py'),
                   help='Pin the evaluator from the same archived bundle as a training experiment.')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    seeds=[int(s) for s in a.seeds.split(',')]
    if not seeds or len(set(seeds))!=len(seeds) or any(s<0 for s in seeds):
        raise ValueError('Select unique nonnegative evaluation seeds')
    a.evaluator=a.evaluator.resolve()
    evaluator_sha=hashlib.sha256(a.evaluator.read_bytes()).hexdigest()
    modes=a.modes.split(',')
    if any(m not in ('walk','getup','pick','roulade') for m in modes) or len(set(modes))!=len(modes):
        raise ValueError('Select unique known task modes')
    expected_cases=sum(6*len(seeds) if m=='walk' else 3*len(seeds) for m in modes)
    start=time.monotonic();rows=[]
    def save():
        tmp=a.output/'matrix.tmp';tmp.write_text(json.dumps({'cases':rows,'wall_seconds':time.monotonic()-start,
            'hardware_verified':False,'complete':len(rows)==expected_cases,'expected_cases':expected_cases,
            'evaluator':str(a.evaluator),'evaluator_sha256':evaluator_sha,'seeds':seeds},indent=2));tmp.replace(a.output/'matrix.json')
    for mode in modes:
        if a.layout=='official':model=a.models/f'xgoduck_{mode}.onnx'
        elif a.layout=='pilot':model=a.models/mode/'actor.onnx'
        else:model=a.models/mode/'snapshot-00064.onnx'
        if not model.is_file():
            rows.append({'mode':mode,'error':'model_missing','model':str(model)});save();continue
        for body in ('reference','rear-mid','rear-high'):
            for seed in seeds:
                for speed in ((0.,.3) if mode=='walk' else (0.,)):
                    key=f'{mode}-{body}-seed{seed}-speed{speed}'
                    out=a.output/key
                    cmd=[sys.executable,str(a.evaluator),
                        '--contract',str(a.contract),'--model',str(model),'--mode',mode,'--body',body,
                        '--seed',str(seed),'--walk-speed',str(speed),'--output',str(out),'--wall-seconds','240']
                    with (a.output/(key+'.log')).open('x') as log:
                        try:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=260)
                        except subprocess.TimeoutExpired:
                            rows.append({'key':key,'mode':mode,'error':'evaluation_timeout'});save();continue
                    report=json.loads((out/'gate.json').read_text()) if (out/'gate.json').exists() else {}
                    row={'key':key,'mode':mode,'body':body,'seed':seed,'speed':speed,'exit':r.returncode,
                        'accepted':report.get('simulation_accepted',False),'gate':report.get('gate'),
                        'model_sha256':report.get('model_sha256'),'error':report.get('error')}
                    if a.baseline:
                        base_path=a.baseline/key/'gate.json'
                        if not base_path.is_file():row['baseline_missing']=True
                        else:
                            base=json.loads(base_path.read_text())
                            row['paired_initial_state_exact']=base.get('initial_fingerprint')==report.get('initial_fingerprint') and bool(report.get('initial_fingerprint'))
                            row['baseline_accepted']=base.get('simulation_accepted',False)
                            row['regressed']=row['baseline_accepted'] and not row['accepted']
                    rows.append(row);save()
                    print(json.dumps(row),flush=True)
    save()


if __name__=='__main__':main()
