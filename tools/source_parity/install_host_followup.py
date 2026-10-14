"""Install one hash-pinned host followup after verified firmware/OFF closure."""
import argparse,base64,hashlib,json,shlex,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--install',type=Path,required=True);p.add_argument('--host',required=True);p.add_argument('--ssh-master',required=True);p.add_argument('--authorized-fixed-robot',action='store_true',required=True);p.add_argument('--host-only-verified-firmware',type=Path);a=p.parse_args()
r=json.loads((a.install/'preparation.json').read_text())
if a.host_only_verified_firmware:
 assert r['old_firmware_sha256']==r['firmware_sha256'], 'Host-only update cannot change firmware'
 flash=json.loads(a.host_only_verified_firmware.read_text())
 assert flash['app_sha256']==r['firmware_sha256'], 'Prior full readback must match unchanged image'
else: flash=json.loads((a.install/'flash-execution/report.json').read_text())
assert flash['readback_verified'] and flash['serial_closed'] and flash['calibration_unchanged']
b=a.install/'host.tar.gz';assert hashlib.sha256(b.read_bytes()).hexdigest()==r['host_sha256']
remote=r['stage']+'.tar.gz';subprocess.run(['scp','-o','ControlPath='+a.ssh_master,str(b),a.host+':'+remote],check=True)
script='''import hashlib,io,tarfile,os,subprocess
from pathlib import Path
stage=Path(STAGE);raw=Path(REMOTE).read_bytes();assert hashlib.sha256(raw).hexdigest()==SHA
stage.mkdir(mode=0o700)
with tarfile.open(fileobj=io.BytesIO(raw)) as t:
 for x in t.getmembers():
  assert x.isfile() and not Path(x.name).is_absolute() and '..' not in Path(x.name).parts
  p=stage/x.name;p.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
  fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
  with os.fdopen(fd,'wb') as s:s.write(t.extractfile(x).read())
subprocess.run(['python3',str(stage/'install.py')],check=True)
subprocess.run(['python3',str(stage/'install.py'),'--install'],check=True)
'''
script='STAGE='+repr(r['stage'])+'\nREMOTE='+repr(remote)+'\nSHA='+repr(r['host_sha256'])+'\n'+script
cmd='python3 -c '+shlex.quote('import base64;exec(compile(base64.b64decode('+repr(base64.b64encode(script.encode()).decode())+'),"<pinned host followup>","exec"))')
with (a.install/'host-install-result.jsonl').open('x') as log:subprocess.run(['ssh','-S',a.ssh_master,'-o','BatchMode=yes',a.host,cmd],stdout=log,stderr=subprocess.STDOUT,timeout=90,check=True)
print('Paired host followup installed; log preserved')
