"""Repin reviewed paired installers for an explicit source followup; no hardware."""
import argparse,ast,hashlib,io,json,tarfile
from pathlib import Path
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
   old=ast.get_source_segment(source,node);source=source.replace(old,node.targets[0].id+' = '+repr(values[node.targets[0].id]),1)
 return source
p=argparse.ArgumentParser(description=__doc__)
for n in ('parent-install','parent-candidate','candidate','output'):p.add_argument('--'+n,type=Path,required=True)
p.add_argument('--seal',required=True);p.add_argument('--stage',required=True);p.add_argument('--reason',required=True);a=p.parse_args()
a.output.mkdir(exist_ok=False);old=a.parent_install
s=(old/'reviewed-flash-source.py').read_text();const={}
for node in ast.parse(s).body:
 if not isinstance(node,ast.Assign):start=node.lineno;break
 const[node.targets[0].id]=ast.literal_eval(node.value)
body=''.join(s.splitlines(True)[start-1:]);image=(a.candidate/'firmware/.pio/build/esp32s3_motion/firmware.bin').read_bytes()
files={}
with tarfile.open(old/'reviewed-flash-bundle.tar.gz') as t:
 for i in t:
  b=t.extractfile(i).read();assert sha(b)==const['FILES'][i.name]['sha256']
  if i.name!='firmware.bin':files[i.name]=b
files['firmware.bin']=image;bundle=pack(files)
const.update(INSTALL_SEAL=a.seal,OLD_SIZE=const['APP_SIZE'],OLD_SHA256=const['APP_SHA256'],APP_SIZE=len(image),APP_SHA256=sha(image),REGION_SIZE=max(const['REGION_SIZE'],(len(image)+4095)//4096*4096),SIZE=len(bundle),BUNDLE_SHA=sha(bundle),FILES={n:dict(size=len(b),sha256=sha(b)) for n,b in files.items()})
s='\n'.join(k+' = '+repr(v) for k,v in const.items())+'\n'+body;compile(s,'<repinned followup>','exec')
(a.output/'reviewed-flash-source.py').write_text(s);(a.output/'reviewed-flash-bundle.tar.gz').write_bytes(bundle)
changed={};diff=[]
for x in a.candidate.rglob('*'):
 if not x.is_file() or '.pio' in x.parts or '__pycache__' in x.parts:continue
 n=x.relative_to(a.candidate).as_posix();parent=a.parent_candidate/n
 if n=='source-candidate.json':continue # Offline reconstruction receipt, never a runtime module.
 if not parent.is_file() or x.read_bytes()!=parent.read_bytes():
  diff.append(dict(path=n,before_sha256=sha(parent.read_bytes()) if parent.is_file() else None,after_sha256=sha(x.read_bytes())))
  if not n.startswith('firmware/'):changed[n]=x.read_bytes()
source=(old/'reviewed-host-install.py').read_text();source=assigns(source,dict(EXPECTED_SEAL=a.seal,FILES=set(changed),expected_policy={x.relative_to(a.parent_candidate).as_posix():sha(x.read_bytes()) for x in (a.parent_candidate/'policy').glob('*.py')}))
for node in ast.walk(ast.parse(source)):
 if isinstance(node,ast.keyword) and node.arg=='diagnostic' and isinstance(node.value,ast.Constant) and isinstance(node.value.value,str):
  source=source.replace(ast.get_source_segment(source,node.value),repr(a.reason),1);break
# Path constructor is code, not a quoted string assignment.
for node in ast.parse(source).body:
 if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='STAGE':source=source.replace(ast.get_source_segment(source,node),'STAGE=Path('+repr(a.stage)+')',1);break
compile(source,'<repinned followup host>','exec')
changed['hashes.json']=(json.dumps({n:sha(b) for n,b in changed.items()},indent=2)+'\n').encode();changed['install.py']=source.encode()
host=pack(changed);(a.output/'host.tar.gz').write_bytes(host);(a.output/'reviewed-host-install.py').write_text(source)
report=dict(stage=a.stage,reason=a.reason,baseline_seal=a.seal,calibration_sha256=const['CAL_SHA256'],flash_source_sha256=sha(s.encode()),flash_bundle_sha256=sha(bundle),host_sha256=sha(host),firmware_sha256=sha(image),firmware_size=len(image),old_firmware_sha256=const['OLD_SHA256'],changes=diff,installed=False,flashed=False)
(a.output/'preparation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='changes'}))
