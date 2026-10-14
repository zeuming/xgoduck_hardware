"""Bounded balanced-domain follow-up to the diagnosed fixed-domain regressions."""
import argparse,datetime,json,os,subprocess,sys,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.bundle=a.bundle.resolve();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic();state={'scope':__doc__,'pid':os.getpid(),'tasks':{},'hardware_access':False,
        'converged':False,'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'domains':['reference','rear-mid','rear-high'],'actor_initialization':'officialONNX; freshcritic/Adam/KL'}
    def save(stage):
        state.update(stage=stage,wall_seconds=time.monotonic()-start)
        t=a.output/'batch.tmp';t.write_text(json.dumps(state,indent=2));t.replace(a.output/'batch.json')
    save('starting')
    for mode in ('getup','pick','roulade'):
        out=a.output/mode
        cmd=[sys.executable,str(a.bundle/'tools/train_current_body_policy.py'),'--contract',str(a.bundle/'inputs/contract.json'),
            '--mode',mode,'--domains','reference,rear-mid,rear-high','--output',str(out),'--device','cuda:0',
            '--num-envs','256','--iterations','1024','--steps','24','--epochs','5','--snapshot-every','64',
            '--wall-seconds','2400','--max-transitions','18874368']
        state['current_mode']=mode;save('training_'+mode);print('START_TASK '+mode,flush=True)
        with (a.output/(mode+'.log')).open('x') as log:
            try:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=2580)
            except subprocess.TimeoutExpired:
                state['tasks'][mode]={'exit':'outer_timeout'};save('task_timeout');continue
        report=json.loads((out/'run.json').read_text()) if (out/'run.json').exists() else {}
        state['tasks'][mode]={k:report.get(k) for k in ('stage','iterations','transitions','export','error')}
        state['tasks'][mode]['exit']=r.returncode;save('finished_'+mode)
        print('END_TASK '+mode+' '+json.dumps(state['tasks'][mode]),flush=True)
    state.pop('current_mode',None);save('complete_needs_retrieval_shutdown_and_evaluation')


if __name__=='__main__':main()
