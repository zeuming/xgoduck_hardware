#!/usr/bin/env python3
"""Install the current native workbench on its pinned X5; never start motion."""
import hashlib
import argparse
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools/x5_recovery'),str(ROOT/'tools/x5_policy')]
import deploy_native_workbench as deploy
import feedback_stream
import maintenance_transport as transport
from transport_config import TransportConfig
import restart_native_workbench as restart

UNIT='''[Unit]
Description=Xgoduck native workbench on port 8767
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 -I -B /opt/xgoduck-workbench/native_service.py
Environment=PYTHONDONTWRITEBYTECODE=1
KillMode=mixed
TimeoutStopSec=20
Restart=on-failure
RestartSec=5
UMask=0077

[Install]
WantedBy=multi-user.target
'''

REMOTE=r'''
import hashlib,io,json,os,pathlib,select,socket,stat,subprocess,sys,tarfile,tempfile,time
p=pathlib.Path
if os.geteuid()!=0 or p('/proc/sys/kernel/random/boot_id').read_text().strip()!=BOOT:
    raise ValueError('Current reviewed X5 boot changed')
if p('/sys/block/mmcblk1/device/cid').read_text().strip()!='035344534b33324785d55cb687017c00':
    raise ValueError('X5 card differs')
if p('/proc/device-tree/model').read_bytes().rstrip(b'\0').decode()!='D-Robotics RDK X5 V1.0':
    raise ValueError('X5 model differs')
target=p('/opt/xgoduck-workbench');unit=p('/etc/systemd/system/xgoduck-workbench.service')
updating=target.exists()
if target.is_symlink() or unit.is_symlink():raise ValueError('Installation links refused')
if updating:
    if OLD_SEAL is None or hashlib.sha256((target/'installation.json').read_bytes()).hexdigest()!=OLD_SEAL:
        raise ValueError('Existing installation seal differs')
    old=json.loads((target/'installation.json').read_text())
    for name,digest in old['files'].items():
        if hashlib.sha256((target/name).read_bytes()).hexdigest()!=digest:raise ValueError('Existing installation changed')
    if unit.read_text()!=UNIT:raise ValueError('Existing service unit differs')
elif unit.exists() or OLD_SEAL is not None:raise ValueError('Installation state differs')
for path in ('/opt','/etc/systemd/system','/var/lib'):
    if os.path.realpath(path)!=path:raise ValueError('Installation parent is not canonical')
if not updating:
    probe=socket.socket()
    try:probe.bind(('192.168.1.11',8767))
    finally:probe.close()
print(json.dumps(dict(kind='transfer_ready',bytes=SIZE)),flush=True)
raw=bytearray();deadline=time.monotonic()+120
while len(raw)<SIZE:
    if time.monotonic()>deadline:raise ValueError('Installation transfer timed out')
    if not select.select([sys.stdin.fileno()],[],[],.2)[0]:continue
    data=os.read(sys.stdin.fileno(),min(65536,SIZE-len(raw)))
    if not data:raise ValueError('Installation transfer ended early')
    raw.extend(data)
if hashlib.sha256(raw).hexdigest()!=SHA:raise ValueError('Installation transfer checksum changed')
directory=p(tempfile.mkdtemp(prefix='.xgoduck-workbench-',dir='/opt'));directory.chmod(0o700)
with tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz') as archive:
    members=archive.getmembers()
    if {item.name for item in members}!=set(FILES) or len(members)!=len(FILES):raise ValueError('Installation member set differs')
    for item in members:
        if not item.isfile() or '/' in item.name or item.size!=FILES[item.name]['size']:raise ValueError('Installation file differs')
        data=archive.extractfile(item).read()
        if hashlib.sha256(data).hexdigest()!=FILES[item.name]['sha256']:raise ValueError('Installation file checksum differs')
        fd=os.open(directory/item.name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
        with os.fdopen(fd,'wb') as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
if updating:
    # Updating the web package must not release an active robot or interrupt
    # a calibration operation. The caller must first close its UART owners.
    import http.cookiejar,urllib.request
    client=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    with client.open('http://192.168.1.11:8767/',timeout=10) as response:response.read(2097153)
    with client.open('http://192.168.1.11:8767/api/status',timeout=10) as response:
        status=json.loads(response.read(2097153))
    if (status.get('motion',{}).get('owns_connection') is not False
            or status.get('maintenance',{}).get('owns_connection') is True
            or status.get('maintenance',{}).get('busy') is True
            or status.get('session',{}).get('status') not in ('closed','fault')
            or status.get('session',{}).get('recovery_pending') is True
            or any(status.get(key,{}).get('busy') is True for key in ('wizard','joint_wizard','joint_zero'))):
        raise ValueError('Close active UART owners and operations before updating the workbench')
    subprocess.run(['systemctl','stop','xgoduck-workbench.service'],check=True,timeout=25,capture_output=True)
    os.rename(target,target.with_name('xgoduck-workbench-previous-'+str(time.monotonic_ns())))
os.rename(directory,target)
if not updating:
    fd=os.open(unit,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644)
    with os.fdopen(fd,'w') as stream:stream.write(UNIT);stream.flush();os.fsync(stream.fileno())
subprocess.run(['systemctl','daemon-reload'],check=True,timeout=15,capture_output=True)
subprocess.run(['systemctl','enable','--now','xgoduck-workbench.service'],check=True,timeout=90,capture_output=True)
print(json.dumps(dict(kind='installed',service='xgoduck-workbench.service',enabled=True,automatic_motor_start=False)),flush=True)
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--update',action='store_true');args=parser.parse_args()
    base=restart.BASE
    old_seal=None
    if args.update:
        previous=max((path for path in base.glob('native-service-install-*') if (path/'result.json').exists()),key=lambda path:path.stat().st_mtime)
        with tarfile.open(previous/'installation.tar.gz','r:gz') as archive:
            old_seal=hashlib.sha256(archive.extractfile('installation.json').read()).hexdigest()
    inventory=max(base.glob('native-persistent-inventory-*'),key=lambda path:path.stat().st_mtime)
    current=json.loads((inventory/'inventory.json').read_text())
    proof=max(base.glob('imu-independent-hold-live-*'),key=lambda path:path.stat().st_mtime)
    snapshot=json.loads((proof/'closed-final.json').read_text())
    cfg=restart.load(restart.SETUP/'calibration-latest.json')
    if snapshot['calibration']!=cfg:raise ValueError('Saved calibration differs from installation seed')
    expected=restart.load(restart.SETUP/'expected-preparation.json')
    expected['generation']=int(time.time())
    config=TransportConfig(kind='wifi',host='192.168.1.11',interface='en0')
    bundle,manifest=deploy.build_bundle(snapshot,cfg,restart.load(restart.SETUP/'joint-zero-provenance.json'),
        expected=expected,reviewed_profile_path=base/'feedback-native-service-restart-reviewed-20261006-01.json',
        reviewed_profile_sha256=feedback_stream.NATIVE_SERVICE_RESTART_PROFILE_SHA256,
        transport_config=config,offline_startup=True,authentication_required=False)
    files={'native-bundle.tar.gz':bundle,'deployment.json':json.dumps(manifest,ensure_ascii=False).encode()}
    for name in ('native_service.py','persistent_boot.py'):files[name]=(ROOT/'tools/x5_policy'/name).read_bytes()
    files['installation.json']=json.dumps(dict(schema_version=1,files={name:hashlib.sha256(data).hexdigest() for name,data in files.items()})).encode()
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
        for name,data in files.items():
            item=tarfile.TarInfo(name);item.size=len(data);item.mode=0o600;archive.addfile(item,io.BytesIO(data))
    payload=buffer.getvalue()
    directory=Path(tempfile.mkdtemp(prefix='native-service-install-',dir=base))
    transport.private_new(directory/'installation.tar.gz',payload)
    transport.private_new(directory/'deployment.json',json.dumps(manifest,ensure_ascii=False)+'\n')
    source=('BOOT='+repr(current['boot_id'])+'\nOLD_SEAL='+repr(old_seal)+'\nSIZE='+repr(len(payload))+'\nSHA='+repr(hashlib.sha256(payload).hexdigest())+'\nFILES='+repr({name:dict(size=len(data),sha256=hashlib.sha256(data).hexdigest()) for name,data in files.items()})+'\nUNIT='+repr(UNIT)+'\n'+REMOTE).encode()
    transport.private_new(directory/'remote-source.py',source)
    channel=transport._ProcessChannel(source,directory,base/'known_hosts',transport.helpers(),transport_config=config)
    try:
        ready=channel.receive(time.monotonic()+15)
        if ready!=dict(kind='transfer_ready',bytes=len(payload)):raise ValueError(ready)
        transport.helpers()['write_pipe_once'](channel.process.stdin.fileno(),payload,120)
        result=channel.receive(time.monotonic()+120)
        if result.get('kind')!='installed':raise ValueError(result)
        channel.process.stdin.close()
        try:extra=channel.receive(time.monotonic()+5)
        except EOFError:extra=None
        if extra is not None or channel.stderr:raise ValueError('Unexpected installation output')
        if channel.process.wait(timeout=5)!=0:raise ValueError('Installation SSH failed')
        transport.private_new(directory/'result.json',json.dumps(result)+'\n')
        print(json.dumps(dict(installed=True,directory=str(directory),url='http://192.168.1.11:8767/',command='systemctl restart xgoduck-workbench',automatic_motor_start=False)))
    finally:channel.finish()


if __name__=='__main__':main()
