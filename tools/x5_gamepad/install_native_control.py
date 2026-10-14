#!/usr/bin/env python3
"""Merge reviewed gamepad files on X5; preserve all deployed model bytes.

Secrets and complete installation backups stay on the board. Preparation never
opens hardware. Installation requires an idle workbench and no UART owners.
"""
import argparse
import fcntl
import hashlib
import http.cookiejar
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import tarfile
import tempfile
import time
import urllib.request

ROOT=Path('/opt/xgoduck-workbench')
STAGE=Path('/dev/shm/xgoduck-native-gamepad-stage')
EXPECTED_SEAL='5d3324d08ff9c76d2063004322fd64dbdcaf58c6c3b9ab6800055cae0062a0f7'
EXPECTED_BOOT='9ae4dfe7-8d3a-41af-ad29-1c1e49675408'
FILES={'native_process.py','native_gamepad.py','gamepad_web.py','gamepad_panel.html','native_workbench.py'}


def digest(raw): return hashlib.sha256(raw).hexdigest()


def unpack(raw):
    values={}
    with tarfile.open(fileobj=io.BytesIO(raw)) as t:
        for member in t.getmembers():
            p=Path(member.name)
            if not member.isfile() or p.is_absolute() or '..' in p.parts or member.name in values:
                raise ValueError('Unexpected bundle entry')
            values[member.name]=t.extractfile(member).read()
    return values


def pack(values):
    out=io.BytesIO()
    with tarfile.open(fileobj=out,mode='w:gz') as t:
        for name,data in sorted(values.items()):
            entry=tarfile.TarInfo(name);entry.size=len(data);entry.mode=0o600
            t.addfile(entry,io.BytesIO(data))
    return out.getvalue()


def status():
    c=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    c.open('http://127.0.0.1:8767/',timeout=3).read()
    return json.load(c.open('http://127.0.0.1:8767/api/status',timeout=3))


def closed_unarmed_connect_failure(motion):
    """A closed, never-armed connection probe may lack position readback.

    This permits an idle software update only; it does not claim servo release
    or clear the hardware fault. No ARM/target/maintenance writes are allowed.
    """
    proof=motion.get('closed_proof') or {}
    operation=motion.get('last_operation') or {}
    counts=proof.get('request_counts')
    if (motion.get('phase')!='fault' or operation.get('action')!='connect'
            or operation.get('state')!='failed' or proof.get('serial_closed') is not True
            or type(counts) is not dict or not counts
            or any(k not in {'1','3','6','19','20','24','25','27','30','32'}
                   or type(v) is not int or v<0 for k,v in counts.items())):
        return False
    return all(type(proof.get(key)) is dict and proof[key].get('mode')=='off'
               and proof[key].get('armed') is False and proof[key].get('torque_ids')==[]
               for key in ('before','after'))


def closed():
    value=status();motion=value['motion'];proof=motion.get('closed_proof') or {}
    assert motion['owns_connection'] is False
    assert proof.get('serial_closed') is True and (proof.get('no_uart_opened') is True
        or proof.get('release_confirmed') is True or closed_unarmed_connect_failure(motion))
    assert value['session']['status'] in ('closed','fault')
    assert not value['session'].get('recovery_pending')
    assert not value['maintenance'].get('owns_connection')
    assert not any(value.get(k,{}).get('busy') for k in ('wizard','joint_wizard','joint_zero','maintenance'))
    return value


def no_uart_owner():
    uart=Path('/dev/ttyUSB0').stat().st_rdev
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal(): continue
        try: fds=list((proc/'fd').iterdir())
        except (FileNotFoundError,ProcessLookupError): continue
        for fd in fds:
            try: node=fd.stat()
            except (FileNotFoundError,ProcessLookupError): continue
            if stat.S_ISCHR(node.st_mode) and node.st_rdev==uart:
                raise ValueError('UART still owned; update not allowed')


def command(*args): subprocess.run(args,check=True,timeout=30,stdout=subprocess.PIPE,stderr=subprocess.PIPE)


def main():
    global EXPECTED_SEAL, EXPECTED_BOOT, STAGE, FILES
    parser=argparse.ArgumentParser();parser.add_argument('--install',action='store_true')
    parser.add_argument('--expected-seal',default=EXPECTED_SEAL)
    parser.add_argument('--expected-boot',default=EXPECTED_BOOT)
    parser.add_argument('--stage',default=str(STAGE))
    parser.add_argument('--file',action='append',choices=sorted(FILES))
    parser.add_argument('--calibration-revision',type=int,default=24)
    parser.add_argument('--calibration-sha256',default='1bdb21e8e3ac0f5efd02b64ac444944fb4879fa34111b00e6336afc8d6fbe98c')
    args=parser.parse_args()
    EXPECTED_SEAL,EXPECTED_BOOT,STAGE=args.expected_seal,args.expected_boot,Path(args.stage)
    if args.file:FILES=set(args.file)
    assert STAGE.is_relative_to('/dev/shm') and STAGE.resolve()==STAGE
    assert os.geteuid()==0
    assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==EXPECTED_BOOT
    assert Path('/proc/device-tree/model').read_bytes().rstrip(b'\0')==b'D-Robotics RDK X5 V1.0'
    assert Path('/sys/block/mmcblk1/device/cid').read_text().strip()=='035344534b33324785d55cb687017c00'
    assert not ROOT.is_symlink() and not STAGE.is_symlink()
    seal_raw=(ROOT/'installation.json').read_bytes();assert digest(seal_raw)==EXPECTED_SEAL,'Deployment changed since review'
    seal=json.loads(seal_raw)
    assert all('/' not in n and n not in ('.','..') for n in seal['files'])
    outer={n:(ROOT/n).read_bytes() for n in seal['files']}
    assert all(digest(outer[n])==sha for n,sha in seal['files'].items())
    manifest=json.loads(outer['deployment.json']);assert digest(outer['native-bundle.tar.gz'])==manifest['archive_sha256']
    members=unpack(outer['native-bundle.tar.gz'])
    assert set(members)==set(manifest['files'])
    assert all(digest(members[n])==v['sha256'] and len(members[n])==v['size_bytes'] for n,v in manifest['files'].items())
    hashes=json.loads((STAGE/'hashes.json').read_text());assert set(hashes)==FILES
    original=dict(members)
    for name,sha in hashes.items():
        data=(STAGE/name).read_bytes();assert digest(data)==sha
        if name.endswith('.py'): compile(data,name,'exec')
        members['service/'+name]=data
    changed=[n for n in members if members[n]!=original.get(n)]
    assert set(changed)=={'service/'+n for n in FILES}
    assert all(members[n]==b for n,b in original.items() if n not in changed)
    assert members['service/sim_web.py']==original['service/sim_web.py']
    assert b'sim_web.route' in members['service/native_process.py']
    assert json.loads(members['service/deployment.json'])['automatic_motor_start'] is False
    raw=pack(members)
    manifest.update(archive_sha256=digest(raw),archive_size=len(raw),member_bytes=sum(map(len,members.values())),
        files={n:dict(size_bytes=len(b),sha256=digest(b)) for n,b in members.items()})
    manifest['required_source_sha256'].update({n:digest(members[n]) for n in changed})
    outer['native-bundle.tar.gz']=raw
    outer['deployment.json']=(json.dumps(manifest,sort_keys=True,ensure_ascii=False,indent=2)+'\n').encode()
    outer['installation.json']=(json.dumps(dict(schema_version=seal['schema_version'],
        files={n:digest(b) for n,b in outer.items()}),sort_keys=True,indent=2)+'\n').encode()
    cal=Path('/var/lib/xgoduck-workbench/calibration-state.json');cal_raw=cal.read_bytes();saved=json.loads(cal_raw)
    assert saved['revision']==args.calibration_revision and saved['calibration_sha256']==args.calibration_sha256
    report=dict(installed=False,old_seal=EXPECTED_SEAL,new_seal=digest(outer['installation.json']),
        changed_members=changed,policy_models_calibration_unchanged=True,sim_gateway_preserved=True,
        automatic_motion_start=False,control_source='x5_local',physical_motion_tested=False)
    if not args.install:
        print(json.dumps(report));return
    closed();no_uart_owner()
    prepared=Path(tempfile.mkdtemp(prefix='xgoduck-gamepad-stage-',dir='/opt'));prepared.chmod(0o700)
    for n,b in outer.items():
        fd=os.open(prepared/n,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
        with os.fdopen(fd,'wb') as f: f.write(b);f.flush();os.fsync(f.fileno())
    # Recheck immediately before stopping the idle service. Never disconnect
    # hardware or stop a running trial to make an update possible.
    closed();no_uart_owner()
    assert digest((ROOT/'installation.json').read_bytes())==EXPECTED_SEAL and cal.read_bytes()==cal_raw
    command('systemctl','stop','xgoduck-workbench.service')
    lock=os.open('/run/xgoduck-workbench.lock',os.O_RDWR|os.O_CREAT,0o600)
    backup=ROOT.with_name('xgoduck-workbench-before-native-gamepad-'+str(time.monotonic_ns()))
    switched=False
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        no_uart_owner()
        assert digest((ROOT/'installation.json').read_bytes())==EXPECTED_SEAL and cal.read_bytes()==cal_raw
        os.rename(ROOT,backup);switched=True;os.rename(prepared,ROOT)
    finally:
        fcntl.flock(lock,fcntl.LOCK_UN);os.close(lock)
        if not switched: command('systemctl','start','xgoduck-workbench.service')
    try:
        command('systemctl','start','xgoduck-workbench.service')
        deadline=time.monotonic()+35
        while True:
            try:
                s=closed();pad=s['motion']['gamepad_control']
                assert pad['source']=='x5_local' and pad['active'] is False
                assert s['motion']['closed_proof']['no_uart_opened'] is True
                assert s['calibration']==saved['calibration']
                break
            except (OSError,KeyError,ValueError,AssertionError):
                if time.monotonic()>=deadline: raise
                time.sleep(.3)
        no_uart_owner();assert cal.read_bytes()==cal_raw
    except BaseException:
        command('systemctl','stop','xgoduck-workbench.service');no_uart_owner()
        if ROOT.exists():os.rename(ROOT,ROOT.with_name('xgoduck-gamepad-failed-'+str(time.monotonic_ns())))
        os.rename(backup,ROOT);command('systemctl','start','xgoduck-workbench.service');raise
    report.update(installed=True,backup=str(backup),no_UART_opened=True,calibration_revision=args.calibration_revision)
    (STAGE/'installation-result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__': main()
