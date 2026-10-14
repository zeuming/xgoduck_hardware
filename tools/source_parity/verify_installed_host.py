"""Read-only installed-source/closed-UART proof. Never opens a serial device."""
import argparse,base64,hashlib,json,shlex,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--host',required=True)
p.add_argument('--ssh-master',required=True);a=p.parse_args()
sha=lambda b:hashlib.sha256(b).hexdigest()
expected={x.relative_to(a.candidate).as_posix():sha(x.read_bytes())
 for folder in ('policy','service') for x in (a.candidate/folder).glob('*.py')}
upstream=Path(__file__).resolve().parents[2]/'output/xgoduck_runtime_upstream/python'
expected.update({'policy/models/'+x.name:sha(x.read_bytes()) for x in upstream.glob('xgoduck_*.onnx')})
code='''from pathlib import Path
import json,hashlib,tarfile,os,time,contextlib,io,runpy
root=Path('/opt/xgoduck-workbench');sha=lambda b:hashlib.sha256(b).hexdigest()
raw=(root/'installation.json').read_bytes();installation=json.loads(raw)
outer={n:sha((root/n).read_bytes())==h for n,h in installation['files'].items()}
with tarfile.open(root/'native-bundle.tar.gz') as t:
 actual={m.name:sha(t.extractfile(m).read()) for m in t if m.isfile() and m.name in EXPECTED}
checks={n:actual.get(n)==h for n,h in EXPECTED.items()}
owners=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  for fd in (p/'fd').iterdir():
   try:
    if os.readlink(fd)=='/dev/ttyUSB0':owners.append(int(p.name));break
   except OSError:pass
 except OSError:pass
with contextlib.redirect_stdout(io.StringIO()):
 api=runpy.run_path('/dev/shm/duck-source-api.py');status=api['status']()
motion=status.get('motion') or {}
print(json.dumps(dict(recorded_unix=time.time(),seal=sha(raw),outer_checks=outer,
 source_hash_checks=checks,calibration_sha256=sha(Path('/var/lib/xgoduck-workbench/calibration-state.json').read_bytes()),
 motion={k:motion.get(k) for k in ('phase','owns_connection','error','connection_sequence','closed_proof')},uart_owners=owners)))
'''
code='EXPECTED='+repr(expected)+'\n'+code
command='python3 -c '+shlex.quote('import base64;exec(base64.b64decode('+repr(base64.b64encode(code.encode()).decode())+'))')
x=json.loads(subprocess.check_output(['ssh','-S',a.ssh_master,'-o','BatchMode=yes',a.host,command],timeout=20,text=True))
x['passed']=all(x['outer_checks'].values()) and all(x['source_hash_checks'].values()) and not x['uart_owners'] and not x['motion']['owns_connection'] and x['calibration_sha256']=='d15e4ba548f6257d90c56a9848a53324e8ea7fa8c712d3dedf70ced646ce18da'
x.update(no_UART_opened_by_check=True,scope='Installed hashes and no UART owner; metadata startup is not a fresh hardware measurement.',actual_OFF_before_restart='../readiness-wait/closure-after-recovery-v2.json')
with a.output.open('x') as f:f.write(json.dumps(x,indent=2)+'\n')
print(json.dumps(dict(passed=x['passed'],seal=x['seal'],source_files=len(expected),uart_owners=x['uart_owners'])))
raise SystemExit(not x['passed'])
