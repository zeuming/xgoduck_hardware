"""Stage and dry-run a reviewed update; never executes install or opens UART."""
from pathlib import Path
import argparse,subprocess,base64,shlex,json,hashlib
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();out=a.root/'install';proof=json.loads((out/'preparation.json').read_text());raw=(out/'host.tar.gz').read_bytes();assert hashlib.sha256(raw).hexdigest()==proof['host_sha256']
source='SIZE='+repr(len(raw))+'\nSHA='+repr(proof['host_sha256'])+'\nSTAGE='+repr(proof['stage'])+'\n'+'''import sys,hashlib,tarfile,io,json
from pathlib import Path
raw=sys.stdin.buffer.read(SIZE+1);assert len(raw)==SIZE and hashlib.sha256(raw).hexdigest()==SHA
root=Path(STAGE);assert not root.exists();root.mkdir(mode=0o700)
with tarfile.open(fileobj=io.BytesIO(raw)) as t:
 for m in t:
  assert m.isfile() and not m.name.startswith('/') and '..' not in Path(m.name).parts
  p=root/m.name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(t.extractfile(m).read());p.chmod(0o600)
print(json.dumps(dict(staged=True,installed=False,serial_opened=False)))
'''
ssh=['ssh','-S','/tmp/duck-runtime-restore-20261012.sock','-o','BatchMode=yes','root@192.168.1.11']
code='import base64;exec(base64.b64decode('+repr(base64.b64encode(source.encode()).decode())+'))'
r=subprocess.run(ssh+['python3 -c '+shlex.quote(code)],input=raw,capture_output=True,timeout=25);(out/'staging.json').write_bytes(r.stdout);assert r.returncode==0,r.stderr.decode()
r=subprocess.run(ssh+['python3 '+proof['stage']+'/install.py'],capture_output=True,timeout=25)
(out/'host-dry-run.json').write_bytes(r.stdout);(out/'host-dry-run.err').write_bytes(r.stderr);assert r.returncode==0,r.stderr.decode();print(r.stdout.decode())
