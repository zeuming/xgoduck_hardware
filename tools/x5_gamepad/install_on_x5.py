#!/usr/bin/env python3
"""Install staged gamepad input on the reviewed X5 without touching UART.

Run over SSH with stdin. The existing installed model/policy/calibration bytes
and network settings are preserved. Review/verify the stage hashes separately.
"""
import hashlib
import argparse
import http.cookiejar
import io
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tarfile
import time
import urllib.request

STAGE = Path('/dev/shm/xgoduck-gamepad-stage-20261010')
INSTALL = Path('/opt/xgoduck-workbench')
BACKUP = Path('/opt/xgoduck-workbench-before-gamepad-20261010')
INPUT = Path('/opt/xgoduck-gamepad')
EXPECTED_BOOT = '9ae4dfe7-8d3a-41af-ad29-1c1e49675408'
EXPECTED_SEAL = '0419b6226bdde7a1b3dbed399c3f5c49a8001abe2ceda6dd9eaf81425be62343'
EXPECTED_PROCESS = '561c65b61beaf1108aad0c298ef80ea195527d98955c8cf1d1526d3ea2e8fcf8'


def digest(raw): return hashlib.sha256(raw).hexdigest()


def unpack(raw):
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as archive:
        values = {}
        for item in archive.getmembers():
            path = Path(item.name)
            if not item.isfile() or path.is_absolute() or '..' in path.parts or item.name in values:
                raise ValueError('Unexpected package member')
            values[item.name] = archive.extractfile(item).read()
        return values


def pack(values):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as archive:
        for name, raw in sorted(values.items()):
            info = tarfile.TarInfo(name)
            info.size, info.mode, info.mtime = len(raw), 0o600, 0
            archive.addfile(info, io.BytesIO(raw))
    return buffer.getvalue()


def status():
    client = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    client.open('http://192.168.1.11:8767/', timeout=3).read()
    return json.load(client.open('http://192.168.1.11:8767/api/status', timeout=3))


def closed():
    value = status()
    motion, maintenance, session = (value.get(k, {}) for k in ('motion', 'maintenance', 'session'))
    if (motion.get('owns_connection') is not False or maintenance.get('owns_connection') is True
            or maintenance.get('busy') is True or session.get('status') not in ('closed','fault')
            or session.get('recovery_pending') is True
            or any(value.get(k,{}).get('busy') is True for k in ('wizard','joint_wizard','joint_zero'))):
        raise ValueError('Close workbench operations before installing gamepad input')


def command(*args):
    subprocess.run(args, check=True, timeout=30, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def main():
    global EXPECTED_SEAL, BACKUP
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-seal', default=EXPECTED_SEAL)
    parser.add_argument('--backup-suffix', default='20261010')
    args = parser.parse_args()
    if not args.backup_suffix.isalnum():
        raise ValueError('Backup suffix must be alphanumeric')
    EXPECTED_SEAL = args.expected_seal
    BACKUP = Path('/opt/xgoduck-workbench-before-gamepad-' + args.backup_suffix)
    if os.geteuid()!=0 or Path('/proc/sys/kernel/random/boot_id').read_text().strip()!=EXPECTED_BOOT:
        raise ValueError('Reviewed X5 boot changed')
    if Path('/sys/block/mmcblk1/device/cid').read_text().strip()!='035344534b33324785d55cb687017c00':
        raise ValueError('Reviewed X5 card changed')
    if Path('/proc/device-tree/model').read_bytes().rstrip(b'\0')!=b'D-Robotics RDK X5 V1.0':
        raise ValueError('Reviewed X5 model changed')
    if BACKUP.exists() or INSTALL.is_symlink() or STAGE.is_symlink() or INPUT.is_symlink():
        raise ValueError('Existing upgrade or linked directory requires review')
    hashes=json.loads((STAGE/'hashes.json').read_text())
    wanted={'reader.py','xgoduck-gamepad.service','native_process.py','native_gamepad.py','gamepad_web.py','gamepad_panel.html'}
    if set(hashes)!=wanted:raise ValueError('Unexpected staged file set')
    staged={name:(STAGE/name).read_bytes() for name in wanted}
    for name, raw in staged.items():
        if digest(raw)!=hashes[name]:raise ValueError('Staged checksum changed: '+name)
        if name.endswith('.py'):compile(raw,name,'exec')
    existing_input = INPUT.exists()
    if existing_input and (INPUT/'reader.py').read_bytes() != staged['reader.py']:
        raise ValueError('Existing input reader differs')
    unit=Path('/etc/systemd/system/xgoduck-gamepad.service')
    if unit.exists() and unit.read_bytes() != staged['xgoduck-gamepad.service']:
        raise ValueError('Existing input service differs')
    seal_raw=(INSTALL/'installation.json').read_bytes()
    if digest(seal_raw)!=EXPECTED_SEAL:raise ValueError('Installation changed since review')
    seal=json.loads(seal_raw)
    outer={name:(INSTALL/name).read_bytes() for name in seal['files']}
    for name,sha in seal['files'].items():
        if digest(outer[name])!=sha:raise ValueError('Installed checksum differs: '+name)
    manifest=json.loads(outer['deployment.json'])
    if digest(outer['native-bundle.tar.gz'])!=manifest['archive_sha256']:
        raise ValueError('Installed archive differs')
    members=unpack(outer['native-bundle.tar.gz'])
    for name,info in manifest['files'].items():
        if digest(members[name])!=info['sha256'] or len(members[name])!=info['size_bytes']:
            raise ValueError('Installed member differs: '+name)
    if digest(members['service/native_process.py'])!=EXPECTED_PROCESS:
        raise ValueError('Installed process code changed')
    deployment=json.loads(members['service/deployment.json'])
    if deployment['startup_mode']!='metadata_only_fault_page' or deployment['automatic_motor_start'] is not False:
        raise ValueError('Upgrade requires an installation with no automatic hardware connection')
    before=dict(members)
    for name in ('native_process.py','native_gamepad.py','gamepad_web.py','gamepad_panel.html'):
        members['service/'+name]=staged[name]
    changed=[n for n in members if members[n]!=before.get(n)]
    if set(changed)!={'service/native_process.py','service/native_gamepad.py','service/gamepad_web.py','service/gamepad_panel.html'}:
        raise ValueError('Unexpected workbench changes')
    for name in before:
        if name not in changed and before[name]!=members[name]:raise ValueError('Unrelated member changed')
    raw=pack(members)
    manifest.update(archive_sha256=digest(raw),archive_size=len(raw),member_bytes=sum(map(len,members.values())),
        files={n:dict(size_bytes=len(b),sha256=digest(b)) for n,b in members.items()})
    manifest['required_source_sha256'].update({n:digest(members[n]) for n in changed})
    outer.update({'native-bundle.tar.gz':raw,'deployment.json':json.dumps(manifest,ensure_ascii=False,sort_keys=True,indent=2).encode()})
    new_seal=json.dumps(dict(schema_version=seal['schema_version'],files={n:digest(b) for n,b in outer.items()}),sort_keys=True,indent=2).encode()
    outer['installation.json']=new_seal
    prepared=Path('/opt/.xgoduck-gamepad-workbench-stage')
    if prepared.exists():
        if prepared.is_symlink() or set(p.name for p in prepared.iterdir()) != set(outer):
            raise ValueError('Existing prepared package differs')
        for name, raw in outer.items():
            if (prepared/name).is_symlink() or (prepared/name).read_bytes() != raw:
                raise ValueError('Existing prepared package differs: ' + name)
    else:
        prepared.mkdir(mode=0o700)
        for name,raw in outer.items():
            path=prepared/name;path.write_bytes(raw);path.chmod(0o600)
    closed()
    # No operation below touches any UART node or signals its unrelated owner.
    command('systemctl','stop','xgoduck-workbench.service')
    try:
        os.rename(INSTALL,BACKUP)
        os.rename(prepared,INSTALL)
        if not existing_input:
            INPUT.mkdir(mode=0o700)
            (INPUT/'reader.py').write_bytes(staged['reader.py']);(INPUT/'reader.py').chmod(0o600)
        if not unit.exists():
            unit.write_bytes(staged['xgoduck-gamepad.service']);unit.chmod(0o644)
        command('systemctl','daemon-reload')
        command('systemctl','enable','--now','xgoduck-gamepad.service')
        command('systemctl','start','xgoduck-workbench.service')
        deadline=time.monotonic()+40
        while True:
            try:
                closed();break
            except (OSError,ValueError):
                if time.monotonic()>=deadline:raise
                time.sleep(.5)
    except BaseException:
        command('systemctl','stop','xgoduck-workbench.service')
        if BACKUP.exists():
            if INSTALL.exists():os.rename(INSTALL,Path('/opt/xgoduck-workbench-gamepad-failed-'+str(time.monotonic_ns())))
            os.rename(BACKUP,INSTALL)
        command('systemctl','start','xgoduck-workbench.service')
        raise
    report=dict(installed=True,old_seal=EXPECTED_SEAL,new_seal=digest(new_seal),changed_members=changed,
        input_service='xgoduck-gamepad.service',workbench_url='http://192.168.1.11:8767/#motion-models',
        policy_calibration_firmware_unchanged=True,automatic_motor_start=False,backup=str(BACKUP))
    Path('/var/lib/xgoduck-gamepad-installation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
