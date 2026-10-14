"""Repin existing reviewed backup/readback and idle host installers for voltage fix.

Offline only. Live pins must be captured again immediately before execution.
"""
import argparse,ast,hashlib,io,json,tarfile
from pathlib import Path
p=argparse.ArgumentParser()
for n in ('baseline','candidate','output','flash-parent','host-parent'):p.add_argument('--'+n,type=Path,required=True)
for n in ('boot','seal','calibration-sha256'):p.add_argument('--'+n,required=True)
p.add_argument('--calibration-revision',type=int,required=True);a=p.parse_args()
a.output.mkdir(exist_ok=False)
sha=lambda b:hashlib.sha256(b).hexdigest()
def pack(files):
 s=io.BytesIO()
 with tarfile.open(fileobj=s,mode='w:gz') as t:
  for n,b in sorted(files.items()):
   i=tarfile.TarInfo(n);i.size=len(b);i.mode=0o600;t.addfile(i,io.BytesIO(b))
 return s.getvalue()
def assigns(source,values):
 for node in reversed(list(ast.walk(ast.parse(source)))):
  if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id in values:
   source=source.replace(ast.get_source_segment(source,node),node.targets[0].id+' = '+repr(values[node.targets[0].id]),1)
 return source
source=(a.flash_parent/'reviewed-flash-source.py').read_text();const={}
for node in ast.parse(source).body:
 if not isinstance(node,ast.Assign):start=node.lineno;break
 const[node.targets[0].id]=ast.literal_eval(node.value)
body=''.join(source.splitlines(True)[start-1:]);image=(a.candidate/'firmware/.pio/build/esp32s3_motion/firmware.bin').read_bytes();files={}
with tarfile.open(a.flash_parent/'reviewed-flash-bundle.tar.gz') as t:
 for i in t:
  b=t.extractfile(i).read();assert sha(b)==const['FILES'][i.name]['sha256']
  if i.name!='firmware.bin':files[i.name]=b
files['firmware.bin']=image;bundle=pack(files)
const.update(BOOT=a.boot,INSTALL_SEAL=a.seal,CAL_SHA256=a.calibration_sha256,OLD_SIZE=const['APP_SIZE'],OLD_SHA256=const['APP_SHA256'],APP_SIZE=len(image),APP_SHA256=sha(image),REGION_SIZE=max(const['REGION_SIZE'],(len(image)+4095)//4096*4096),SIZE=len(bundle),BUNDLE_SHA=sha(bundle),FILES={n:dict(size=len(b),sha256=sha(b)) for n,b in files.items()})
source='\n'.join(k+' = '+repr(v) for k,v in const.items())+'\n'+body
source=source.replace('http://192.168.1.11:8767','http://127.0.0.1:8767')
compile(source,'<voltage flash>','exec')
(a.output/'reviewed-flash-source.py').write_text(source);(a.output/'reviewed-flash-bundle.tar.gz').write_bytes(bundle)
changed={n:(a.candidate/n).read_bytes() for n in ('policy/motion.py','service/worker-classes.py')}
stage='/dev/shm/duck-live-voltage-install-20261012'
s=(a.host_parent/'reviewed-host-install.py').read_text()
s=assigns(s,dict(EXPECTED_SEAL=a.seal,EXPECTED_BOOT=a.boot,FILES=set(changed),expected_policy={f.relative_to(a.baseline).as_posix():sha(f.read_bytes()) for f in (a.baseline/'policy').glob('*.py')}))
# Keep existing deployment profile verbatim; only the two reviewed modules change.
begin=s.index('    # HEAD-PROFILE-20261012:');end=s.index('    changed=[n for n in members',begin)
s=s[:begin]+s[end:]
s=s.replace("FILES|{'service/deployment.json'}",'FILES')
s=s.replace("saved['revision']==37 and digest(cal_raw)=='eeec9b73d9806418f91f06b7675e97cf5f0f93e938d681a185d9410390625de4'",f"saved['revision']=={a.calibration_revision} and digest(cal_raw)=={a.calibration_sha256!r}")
s=s.replace('calibration_revision=37',f'calibration_revision={a.calibration_revision}')
for node in ast.parse(s).body:
 if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='STAGE':s=s.replace(ast.get_source_segment(s,node),'STAGE=Path('+repr(stage)+')',1);break
for node in ast.walk(ast.parse(s)):
 if isinstance(node,ast.keyword) and node.arg=='diagnostic' and isinstance(node.value,ast.Constant):s=s.replace(ast.get_source_segment(s,node.value),repr('Restore voltage via one grouped register56..62 scan and versioned source status; preserve model, gains, goal, calibration and stop logic'),1);break
s=s.replace('e1f275419b1b391fb2bcac36cbdcc7223d5cc94f0b20d211e265a7455e7f4feb',sha((a.baseline/'service/native_workbench.py').read_bytes()))
compile(s,'<voltage host>','exec');changed['hashes.json']=(json.dumps({n:sha(b) for n,b in changed.items()},indent=2)+'\n').encode();changed['install.py']=s.encode();host=pack(changed)
(a.output/'reviewed-host-install.py').write_text(s);(a.output/'host.tar.gz').write_bytes(host)
r=dict(stage=stage,baseline_seal=a.seal,boot=a.boot,calibration_sha256=a.calibration_sha256,flash_source_sha256=sha(source.encode()),flash_bundle_sha256=sha(bundle),host_sha256=sha(host),firmware_sha256=sha(image),firmware_size=len(image),old_firmware_sha256=const['OLD_SHA256'],installed=False,flashed=False)
(a.output/'preparation.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
