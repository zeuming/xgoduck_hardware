"""On-X5, bounded isolated CPU loop; no deployment or real UART access."""
from pathlib import Path
import argparse,json,hashlib,tarfile,subprocess,os,time
p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--dependencies',type=Path,required=True);p.add_argument('--candidate-execution',choices=('worker','synchronous'),default='worker');a=p.parse_args()
root=a.output;root.mkdir(exist_ok=False)
with tarfile.open(a.archive) as tar:
 for m in tar:
  assert m.isfile() and not m.name.startswith('/') and '..' not in Path(m.name).parts
  target=root/m.name;target.parent.mkdir(exist_ok=True,parents=True);target.write_bytes(tar.extractfile(m).read())
manifest=json.loads((root/'manifest.json').read_text())
assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in manifest['files'].items())
native=a.dependencies;model=native/'policy/models/xgoduck_walk.onnx'
assert hashlib.sha256(model.read_bytes()).hexdigest()=='5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f'
began=time.monotonic()
for name in ('baseline','candidate'):
 assert time.monotonic()-began<25,'No new test near overall deadline'
 (root/name/'site').symlink_to(native/'site',target_is_directory=True)
 owners=[]
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:
   for fd in (proc/'fd').iterdir():
    try:
     if os.readlink(fd)=='/dev/ttyUSB0':owners.append(int(proc.name));break
    except OSError:pass
  except OSError:pass
 assert not owners,'Real UART active; defer isolated CPU test'
 env=dict(os.environ,PYTHONPATH=str(native/'site'))
 with (root/(name+'.log')).open('x') as log:
  code=subprocess.run(['python3',str(root/'tools/source_parity/benchmark_cycle_budget_loop.py'),'--candidate',str(root/name),'--models-dir',str(native/'policy/models'),'--output',str(root/(name+'.json')),'--execution',a.candidate_execution if name=='candidate' else 'worker'],env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=20).returncode
 print(json.dumps(dict(case=name,returncode=code,source_hashes_verified=len(manifest['files']),real_UART_owners=owners)),flush=True)
 if code:raise RuntimeError('Inspect isolated test log before any followup')
checks={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.suffix in ('.json','.log')}
(root/'completed.json').write_text(json.dumps(dict(completed=True,recorded_unix=time.time(),real_UART=False,actuation=False,wall_seconds=time.monotonic()-began,hashes=checks),indent=2)+'\n')
