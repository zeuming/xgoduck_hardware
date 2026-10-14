"""Bounded test-only runner for the explicitly staged X5 temporary directory."""
import argparse,hashlib,json,subprocess,tarfile,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',type=Path,required=True);a=p.parse_args()
    stage=a.stage.resolve()
    if stage.parent!=Path('/tmp') or not stage.name.startswith('xgoduck-poll-v5-'):
        raise ValueError('Only this task temporary staging directory is permitted')
    archive=stage/'poll-priority-v5-x5-allowlist.tar.gz'
    if hashlib.sha256(archive.read_bytes()).hexdigest()!='19c7fd002efce7190baa3e1867730fcd1472f45c04c84b47617f66c69fa5479a':
        raise ValueError('Archive changed')
    with tarfile.open(archive) as t:
        for member in t.getmembers():
            if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
                raise ValueError('Unexpected archive member')
        t.extractall(stage)
    manifest=json.loads((stage/'manifest.json').read_text())
    v4=json.loads((stage/'deferred-v4-x5-staging.json').read_text());base=Path(v4['remote_directory'])
    for directory,m in ((stage,manifest),(base,v4)):
        for name,digest in m['files'].items():
            if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:
                raise ValueError('Source mismatch: '+name)
    out=stage/'results';out.mkdir(exist_ok=False)
    python=Path('/tmp/xgoduck-io-owner-tMNvXMOy/venv/bin/python')
    model='/dev/shm/xgoduck-native-woodhdai/policy/models'
    start=time.monotonic();rows=[]
    report=dict(completed=False,hardware_access=False,deployed=False,host='X5',maximum_wall_seconds=180,
        stage=str(stage),source_manifest=manifest,baseline_manifest=v4,runs=rows)
    def save():(out/'progress.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    try:
        jobs=[]
        for tool in ('validate_serial_owner','validate_deferred_control','validate_poll_priority'):
            jobs.append((tool,[str(python),str(stage/'tools'/f'{tool}.py'),'--source',str(stage/'candidate'),'--output',str(out/f'{tool}.json')]))
        for name,directory,mode,ledger in (
                ('v4-none',base/'candidate','none',False),('v5-none',stage/'candidate','none',False),
                ('v4-both',base/'candidate','both',False),('v5-both',stage/'candidate','both',False),
                ('v5-none-waits',stage/'candidate','none',True),('v5-both-waits',stage/'candidate','both',True)):
            tool='diagnose_scheduler_waits.py' if ledger else 'validate_serial_owner.py'
            command=[str(python),str(stage/'tools'/tool),'--source',str(directory),'--output',str(out/f'{name}.json'),
                '--benchmark','--models',model,'--transport-timing',str(stage/'timing.json'),'--delay-mode',mode,
                '--warmup-seconds','3','--segment-seconds','3']
            if ledger:command+=['--ledger-dir',str(out/name)]
            jobs.append((name,command))
        for name,command in jobs:
            if time.monotonic()-start>=180:raise TimeoutError('Overall budget exhausted')
            row=dict(name=name,command=command);rows.append(row);save()
            with (out/f'{name}.log').open('w') as log:
                proc=subprocess.run(command,cwd=stage,stdout=log,stderr=subprocess.STDOUT,
                    timeout=min(35,180-(time.monotonic()-start)))
            row['exit_code']=proc.returncode;save()
            if proc.returncode:raise RuntimeError(name+' failed')
            result=json.loads((out/f'{name}.json').read_text())
            if 'cases' in result:row['hz']=[c['target_write_hz'] for c in result['cases']]
            else:row.update(tests=result['tests'],passed=result['passed'])
            print(json.dumps(row),flush=True);save()
        report['completed']=True
    except Exception as error:report['error']=str(error);raise
    finally:report['wall_seconds']=time.monotonic()-start;save()


if __name__=='__main__':main()
