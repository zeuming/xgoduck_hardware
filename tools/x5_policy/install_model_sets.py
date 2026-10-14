"""Build/install a hash-pinned host-only overlay while the X5 is idle.

Preserves the current bundle, launchers, calibration, firmware and controller
parameters. A changed baseline or active UART refuses the update. No robot
connect/start/ARM/TARGET command is sent. Package rollback preserves calibration.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import time


def sha(raw):return hashlib.sha256(raw).hexdigest()


REMOTE = r'''
from pathlib import Path
import hashlib,io,json,os,subprocess,tarfile,time,urllib.request,http.cookiejar
root=Path('/opt/xgoduck-workbench');state=Path('/var/lib/xgoduck-workbench/calibration-state.json')
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==PLAN['boot_id'], 'X5 rebooted'
assert digest(root/'installation.json')==PLAN['old_seal'], 'Installation changed'
assert digest(state)==PLAN['calibration_sha256'], 'Saved calibration changed'
old=json.loads((root/'installation.json').read_text())
assert all(digest(root/n)==h for n,h in old['files'].items()), 'Installed file changed'
def owners():
 found=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   for fd in (p/'fd').iterdir():
    try:
     target=os.readlink(fd)
     if target.startswith(('/dev/ttyUSB','/dev/ttyACM')):found.append(int(p.name));break
    except OSError:pass
  except OSError:pass
 return found
def status():
 client=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
 with client.open('http://127.0.0.1:8767/',timeout=3) as f:f.read(4*1024*1024)
 with client.open('http://127.0.0.1:8767/api/status',timeout=3) as f:return json.load(f)
def idle(value):
 assert value['motion']['owns_connection'] is False, 'Motion connection active'
 assert value['maintenance']['owns_connection'] is False and value['maintenance']['busy'] is False, 'Maintenance active'
 assert value['session']['status'] in ('closed','fault') and not value['session']['recovery_pending'], 'Calibration session active'
 assert all(not (value.get(n) or {}).get('busy') for n in ('wizard','joint_wizard','joint_zero')), 'Wizard active'
 assert not owners(), 'UART owner exists'
idle(status())
archive=Path(REMOTE_ARCHIVE);assert digest(archive)==PLAN['installation_archive_sha256']
stage=Path(PLAN['remote_stage']);stage.mkdir(mode=0o700)
with tarfile.open(archive) as t:
 assert len(t.getmembers())==len(PLAN['files']) and {m.name for m in t}==set(PLAN['files'])
 for m in t:
  assert m.isfile() and '/' not in m.name
  data=t.extractfile(m).read();assert hashlib.sha256(data).hexdigest()==PLAN['files'][m.name]
  fd=os.open(stage/m.name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
  with os.fdopen(fd,'wb') as f:f.write(data);f.flush();os.fsync(f.fileno())
idle(status());assert digest(state)==PLAN['calibration_sha256']
backup=root.with_name('xgoduck-workbench-before-modelsets-'+str(time.monotonic_ns()))
swapped=False
try:
 subprocess.run(['systemctl','stop','xgoduck-workbench.service'],check=True,timeout=25)
 assert not owners(), 'UART unexpectedly open after service stop'
 assert digest(state)==PLAN['calibration_sha256'], 'Calibration changed before swap'
 os.rename(root,backup);os.rename(stage,root);swapped=True
 subprocess.run(['systemctl','start','xgoduck-workbench.service'],check=True,timeout=25)
 deadline=time.monotonic()+45;latest=None
 while time.monotonic()<deadline:
  try:
   latest=status()
   if latest.get('motion',{}).get('available_model_sets')==['original','trained']:break
  except Exception:pass
  time.sleep(.5)
 else:raise RuntimeError('Updated model catalog was not published')
 idle(latest)
 assert latest['motion']['model_set']=='original'
 assert digest(state)==PLAN['calibration_sha256'], 'Calibration changed after restart'
 new=json.loads((root/'installation.json').read_text())
 assert all(digest(root/n)==h for n,h in new['files'].items())
 print(json.dumps({'installed':True,'backup':str(backup),'new_seal':digest(root/'installation.json'),
   'calibration_unchanged':True,'uart_owners':owners(),'motion_connection_owned':False,
   'model_set':latest['motion']['model_set'],'available_model_sets':latest['motion']['available_model_sets'],
   'model_hashes':latest['motion']['deployment']['model_sets'],'automatic_motor_start':False,'firmware_changed':False}))
except BaseException:
 if swapped:
  # A user may explicitly connect after the new page becomes available. Do
  # not turn a post-install observation mismatch into an unexpected STOP.
  if owners():
   print(json.dumps({'installation_committed':True,'rollback_deferred':'A new UART owner appeared; leave operator connection untouched','backup':str(backup)}),flush=True)
   raise
  subprocess.run(['systemctl','stop','xgoduck-workbench.service'],timeout=25)
  failed=root.with_name('xgoduck-workbench-failed-modelsets-'+str(time.monotonic_ns()))
  os.rename(root,failed);os.rename(backup,root)
  subprocess.run(['systemctl','start','xgoduck-workbench.service'],timeout=25)
 else:
  if not root.exists() and backup.exists():os.rename(backup,root)
  subprocess.run(['systemctl','start','xgoduck-workbench.service'],timeout=25)
 raise
'''


def build(baseline,candidate,output):
    output.mkdir(parents=True,exist_ok=False)
    inventory=json.loads((baseline/'inventory.json').read_text())
    manifest=json.loads((baseline/'deployment.json').read_text())
    prep=json.loads((candidate/'model-sets-preparation.json').read_text())
    with tarfile.open(baseline/'native-bundle.tar.gz') as t:
        contents={m.name:t.extractfile(m).read() for m in t if m.isfile()}
    assert set(contents)==set(manifest['files'])
    for name,value in contents.items():assert sha(value)==manifest['files'][name]['sha256']
    for name,digest in prep['changed_files'].items():
        value=(candidate/name).read_bytes();assert sha(value)==digest
        contents[name]=value
    # Files outside the declared overlay remain byte-identical, including the
    # active saved calibration reference and all four official ONNX assets.
    manifest['files']={n:dict(size_bytes=len(v),sha256=sha(v)) for n,v in contents.items()}
    manifest['required_source_sha256'].update({n:sha(contents[n]) for n in prep['changed_files'] if n.endswith(('.py','.js'))})
    declaration=json.loads(contents['service/deployment.json'])
    manifest['model_sets']=declaration['model_sets'];manifest['default_model_set']='original'
    raw=io.BytesIO()
    with tarfile.open(fileobj=raw,mode='w:gz') as t:
        for name,value in contents.items():
            m=tarfile.TarInfo(name);m.mode=0o600;m.size=len(value);t.addfile(m,io.BytesIO(value))
    bundle=raw.getvalue()
    manifest.update(archive_sha256=sha(bundle),archive_size=len(bundle),member_bytes=sum(len(v) for v in contents.values()))
    files={n:(baseline/n).read_bytes() for n in ('native_service.py','persistent_boot.py')}
    files.update({'native-bundle.tar.gz':bundle,'deployment.json':json.dumps(manifest,ensure_ascii=False,sort_keys=True).encode()})
    files['installation.json']=json.dumps({'schema_version':1,'files':{n:sha(v) for n,v in files.items()}},sort_keys=True).encode()
    raw=io.BytesIO()
    with tarfile.open(fileobj=raw,mode='w:gz') as t:
        for name,value in files.items():
            m=tarfile.TarInfo(name);m.mode=0o600;m.size=len(value);t.addfile(m,io.BytesIO(value))
    archive=raw.getvalue();(output/'installation.tar.gz').write_bytes(archive)
    suffix=str(time.time_ns())
    plan=dict(old_seal=inventory['seal'],boot_id=inventory['boot_id'],calibration_sha256=inventory['calibration_sha256'],
        files={n:sha(v) for n,v in files.items()},installation_archive_sha256=sha(archive),
        remote_stage='/opt/.duck-modelsets-'+suffix,changed_files=prep['changed_files'],
        automatic_motor_start=False,firmware_changed=False,controller_parameters_changed=False)
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    return plan


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--install',action='store_true')
    p.add_argument('--prepared',action='store_true',help='Reuse a verified archive after a transfer-only failure; remote seal still checked')
    p.add_argument('--host');p.add_argument('--ssh-master');a=p.parse_args()
    if a.prepared:
        plan=json.loads((a.output/'plan.json').read_text())
        assert sha((a.output/'installation.tar.gz').read_bytes())==plan['installation_archive_sha256']
        assert all(sha((a.candidate/n).read_bytes())==h for n,h in plan['changed_files'].items())
        assert not (a.output/'install-result.jsonl').exists(), 'Inspect the prior installation result before another execution'
    else:
        plan=build(a.baseline,a.candidate,a.output)
    if a.install:
        if not a.host or not a.ssh_master:raise ValueError('Explicit SSH target and existing master required')
        remote_archive=plan['remote_stage']+'.tar.gz'
        subprocess.run(['scp','-q','-o','ControlPath='+a.ssh_master,str(a.output/'installation.tar.gz'),a.host+':'+remote_archive],check=True,timeout=90)
        source='PLAN='+repr(plan)+'\nREMOTE_ARCHIVE='+repr(remote_archive)+'\n'+REMOTE
        (a.output/'install-source.py').write_text(source)
        with (a.output/'install-result.jsonl').open('x') as log:
            subprocess.run(['ssh','-S',a.ssh_master,'-o','BatchMode=yes',a.host,'python3','-'],input=source.encode(),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
        print('Installed host overlay; result and rollback reference saved. No motor action requested.')
    else:print(json.dumps({'prepared':True,'plan':str(a.output/'plan.json')}))
