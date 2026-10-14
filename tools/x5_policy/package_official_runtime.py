"""Prepare reviewed idle-install/firmware update artifacts. No network or UART."""
import argparse,ast,hashlib,io,json,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();r=a.root
old=ROOT/'output/duck_sim/official-execution-20261012/install-20261012'
sha=lambda b:hashlib.sha256(b).hexdigest()
baseline=json.loads((r/'baseline.json').read_text());candidate=r/'candidate';manifest=json.loads((candidate/'manifest.json').read_text())
for n,h in manifest['changes'].items():assert sha((candidate/n).read_bytes())==h,(n,'candidate changed')
assert json.loads((r/'host-checks-v6.json').read_text())['passed']
assert json.loads((r/'gamepad-checks-v6.json').read_text())['passed']
assert json.loads((r/'wire-replay-result.json').read_text())['passed']
assert json.loads((r/'ui-check-result.json').read_text())['passed']
assert json.loads((r/'esp32-build-v4-result.json').read_text())['exit_code']==0
stage='/dev/shm/xgoduck-official-runtime-20261012-v6'
out=r/'install';out.mkdir(exist_ok=False)
files={n.removeprefix('host/'):(candidate/n).read_bytes() for n in manifest['changes'] if n.startswith('host/')}
s=(old/'reviewed-host-install.py').read_text()
for n in ast.parse(s).body:
 if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name):
  key=n.targets[0].id
  replacements={'STAGE':'STAGE=Path('+repr(stage)+')','EXPECTED_SEAL':'EXPECTED_SEAL='+repr(baseline['seal']),'EXPECTED_BOOT':'EXPECTED_BOOT='+repr(baseline['boot']),'FILES':'FILES='+repr(set(files))}
  if key in replacements:s=s.replace(ast.get_source_segment(s,n),replacements[key],1)
for n in ast.walk(ast.parse(s)):
 if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='expected_policy':
  expected={k:v for k,v in baseline['files'].items() if k.startswith('policy/') and k.endswith('.py')}
  s=s.replace(ast.get_source_segment(s,n),'expected_policy='+repr(expected),1);break
s=s.replace('Enable capability checked official policy goal execution and original HOME/EMA decode; preserve installed scheduling, calibration, stance transition and guards','Restore official normal motion and gains; preserve saved calibration, explicit head range projection, ownership, STOP and stale-state rejection')
compile(s,'<reviewed idle host install>','exec')
files['hashes.json']=(json.dumps({n:sha(b) for n,b in files.items()},indent=2)+'\n').encode();files['install.py']=s.encode()
def pack(values):
 raw=io.BytesIO()
 with tarfile.open(fileobj=raw,mode='w:gz') as t:
  for n,b in sorted(values.items()):
   m=tarfile.TarInfo(n);m.size=len(b);m.mode=0o600;t.addfile(m,io.BytesIO(b))
 return raw.getvalue()
(out/'host.tar.gz').write_bytes(pack(files));(out/'reviewed-host-install.py').write_text(s)
# Retain tested backup/readback/update protocol, repin old/new app and board seal.
s=(old/'reviewed-flash-source.py').read_text();constants={}
for n in ast.parse(s).body:
 if isinstance(n,ast.Import):start=n.lineno;break
 constants[n.targets[0].id]=ast.literal_eval(n.value)
body=''.join(s.splitlines(True)[start-1:]);fw=(candidate/'firmware/.pio/build/esp32s3_motion/firmware.bin').read_bytes()
constants.update(BOOT=baseline['boot'],INSTALL_SEAL=baseline['seal'],CAL_SHA256=baseline['calibration_sha'],OLD_SIZE=constants['APP_SIZE'],OLD_SHA256=constants['APP_SHA256'],APP_SIZE=len(fw),APP_SHA256=sha(fw))
assert len(fw)<constants['REGION_SIZE']
fwfiles={}
with tarfile.open(old/'reviewed-flash-bundle.tar.gz') as t:
 for m in t:
  b=t.extractfile(m).read();assert sha(b)==constants['FILES'][m.name]['sha256']
  if m.name!='firmware.bin':fwfiles[m.name]=b
fwfiles['firmware.bin']=fw;raw=pack(fwfiles)
constants.update(SIZE=len(raw),BUNDLE_SHA=sha(raw),FILES={n:dict(size=len(b),sha256=sha(b)) for n,b in fwfiles.items()})
source='\n'.join(k+' = '+repr(v) for k,v in constants.items())+'\n'+body
compile(source,'<reviewed firmware install>','exec')
(out/'reviewed-flash-source.py').write_text(source);(out/'reviewed-flash-bundle.tar.gz').write_bytes(raw)
report=dict(stage=stage,baseline_seal=baseline['seal'],host_members=list(files),host_sha256=sha((out/'host.tar.gz').read_bytes()),flash_source_sha256=sha(source.encode()),flash_bundle_sha256=sha(raw),firmware_sha256=sha(fw),firmware_size=len(fw),old_firmware_sha256=constants['OLD_SHA256'],flashed=False,installed=False,needs_current_support_confirmation=True,automatic_motion=False,calibration_sha256=baseline['calibration_sha'])
(out/'preparation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
