"""Apply a hash-bound additive joint-follow update to an idle X5 installation.

The stage contains reviewed native edits, four viewer files and review.json.
Default is validation only. No connection or motion API is ever invoked.
"""
import argparse
import fcntl
import hashlib
import http.cookiejar
import io
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tarfile
import tempfile
import time
import urllib.request

ROOT = Path('/opt/xgoduck-workbench')
SIM = Path('/opt/xgoduck-sim/app/tools/duck_sim')
CAL = Path('/var/lib/xgoduck-workbench/calibration-state.json')
NATIVE = {'service/native_runtime.py', 'service/native_workbench.py', 'policy/joint_follow.py'}
VIEWER = {'index.html', 'app.js', 'style.css', 'joint_follow.js'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def unpack(raw):
    files = {}
    with tarfile.open(fileobj=io.BytesIO(raw)) as archive:
        for member in archive:
            path = Path(member.name)
            if not member.isfile() or path.is_absolute() or '..' in path.parts or member.name in files:
                raise ValueError('Invalid bundle member')
            files[member.name] = archive.extractfile(member).read()
    return files


def pack(files):
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode='w:gz') as archive:
        for name, data in sorted(files.items()):
            entry = tarfile.TarInfo(name)
            entry.size, entry.mode = len(data), 0o600
            archive.addfile(entry, io.BytesIO(data))
    return out.getvalue()


def atomic(path, data):
    fd, name = tempfile.mkstemp(prefix='.'+path.name+'-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name): os.unlink(name)


def closed():
    client = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    client.open('http://127.0.0.1:8767/', timeout=3).read()
    value = json.load(client.open('http://127.0.0.1:8767/api/status', timeout=3))
    motion = value['motion']; proof = motion.get('closed_proof') or {}
    assert motion['owns_connection'] is False and proof.get('serial_closed') is True
    assert proof.get('no_uart_opened') is True or proof.get('release_confirmed') is True
    assert not motion.get('gamepad_control', {}).get('active')
    assert value['session']['status'] in ('closed', 'fault')
    assert not value['session'].get('recovery_pending')
    assert not value['maintenance'].get('owns_connection')
    assert not any(value.get(k, {}).get('busy') for k in ('wizard','joint_wizard','joint_zero','maintenance'))
    return value


def no_uart_owner():
    devices = list(Path('/dev').glob('ttyUSB*')) + list(Path('/dev').glob('ttyACM*'))
    numbers = {p.stat().st_rdev for p in devices if p.is_char_device()}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal(): continue
        try: descriptors = list((proc/'fd').iterdir())
        except (FileNotFoundError, ProcessLookupError): continue
        for fd in descriptors:
            try: node = fd.stat()
            except (FileNotFoundError, ProcessLookupError): continue
            if stat.S_ISCHR(node.st_mode) and node.st_rdev in numbers:
                raise ValueError('UART is owned; leave current service untouched')


def service(action):
    subprocess.run(['systemctl', action, 'xgoduck-workbench.service'], check=True,
                   timeout=30, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args(); stage = args.stage
    review = json.loads((stage/'review.json').read_text())
    identity = review['identity']
    assert os.geteuid() == 0 and not ROOT.is_symlink() and not SIM.is_symlink() and not stage.is_symlink()
    assert Path('/proc/sys/kernel/random/boot_id').read_text().strip() == identity['boot']
    assert Path('/proc/device-tree/model').read_bytes().rstrip(b'\0').decode() == identity['model']
    assert Path('/sys/block/mmcblk1/device/cid').read_text().strip() == identity['cid']
    original_seal = (ROOT/'installation.json').read_bytes()
    assert sha(original_seal) == identity['seal'], 'Installation changed since retrieval'
    seal = json.loads(original_seal)
    assert all('/' not in n and n not in ('.','..') for n in seal['files'])
    outer = {name:(ROOT/name).read_bytes() for name in seal['files']}
    assert all(sha(outer[n]) == h for n,h in seal['files'].items())
    manifest = json.loads(outer['deployment.json'])
    assert sha(outer['native-bundle.tar.gz']) == manifest['archive_sha256']
    members = unpack(outer['native-bundle.tar.gz'])
    assert set(members) == set(manifest['files'])
    assert all(sha(members[n]) == v['sha256'] and len(members[n]) == v['size_bytes'] for n,v in manifest['files'].items())
    assert set(review['native_after']) == NATIVE and set(review['viewer_after']) == VIEWER
    assert set(review['viewer_before']) == VIEWER
    original_members = dict(members)
    for name, digest in review['native_after'].items():
        data = (stage/'native'/name).read_bytes()
        assert sha(data) == digest
        compile(data, name, 'exec'); members[name] = data
    assert {n for n in members if members[n] != original_members.get(n)} == NATIVE
    startup = json.loads(members['service/deployment.json'])
    assert startup['startup_mode'] == 'metadata_only_fault_page'
    assert startup['automatic_motor_start'] is False and startup['automatic_reconnect'] is False
    assert b'auto_connect=not offline_startup' in members['service/bootstrap.py']
    old_viewer = {name:(SIM/name).read_bytes() if (SIM/name).exists() else None for name in VIEWER}
    assert all((sha(data) if data is not None else None) == review['viewer_before'][n] for n,data in old_viewer.items())
    new_viewer = {n:(stage/'viewer'/n).read_bytes() for n in VIEWER}
    assert all(sha(data) == review['viewer_after'][n] for n,data in new_viewer.items())
    cal = CAL.read_bytes(); assert sha(cal) == identity['calibration_sha256']
    archive = pack(members)
    manifest.update(archive_sha256=sha(archive), archive_size=len(archive),
        member_bytes=sum(map(len,members.values())),
        files={n:dict(size_bytes=len(b),sha256=sha(b)) for n,b in members.items()})
    manifest['required_source_sha256'].update({n:sha(members[n]) for n in NATIVE})
    outer['native-bundle.tar.gz'] = archive
    outer['deployment.json'] = (json.dumps(manifest,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()
    outer['installation.json'] = (json.dumps(dict(schema_version=seal['schema_version'],
        files={n:sha(b) for n,b in outer.items()}),sort_keys=True,indent=2)+'\n').encode()
    closed(); no_uart_owner()
    report = dict(installed=False, old_seal=sha(original_seal), new_seal=sha(outer['installation.json']),
        native_files=sorted(NATIVE), viewer_files=sorted(VIEWER), calibration_sha256=sha(cal),
        calibration_revision=identity['calibration_revision'], hardware_opened=False, physical_motion_tested=False)
    if not args.install:
        print(json.dumps(report)); return
    prepared = Path(tempfile.mkdtemp(prefix='xgoduck-joint-follow-stage-',dir='/opt'))
    for n,b in outer.items(): atomic(prepared/n,b)
    backup = Path('/opt/xgoduck-workbench-before-joint-follow-'+str(time.monotonic_ns()))
    ui_backup = backup.with_name(backup.name+'-viewer'); ui_backup.mkdir(mode=0o700)
    for n,b in old_viewer.items():
        if b is not None: atomic(ui_backup/n,b)
    atomic(ui_backup/'before.json',(json.dumps(review['viewer_before'])+'\n').encode())
    closed(); no_uart_owner()
    assert (ROOT/'installation.json').read_bytes() == original_seal and CAL.read_bytes() == cal
    service('stop')
    lock = os.open('/run/xgoduck-workbench.lock',os.O_RDWR|os.O_CREAT,0o600)
    switched = False
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB); no_uart_owner()
        assert (ROOT/'installation.json').read_bytes() == original_seal and CAL.read_bytes() == cal
        os.rename(ROOT,backup)
        try: os.rename(prepared,ROOT)
        except BaseException:
            os.rename(backup,ROOT); raise
        switched = True
    finally:
        fcntl.flock(lock,fcntl.LOCK_UN); os.close(lock)
        if not switched: service('start')
    try:
        service('start'); deadline=time.monotonic()+35
        while True:
            try:
                state=closed()
                assert state['native_access']['joint_follow_protocol'] == 1
                assert state['motion']['closed_proof']['no_uart_opened'] is True
                assert state['calibration'] == json.loads(cal)['calibration']
                break
            except (OSError, KeyError, ValueError, AssertionError):
                if time.monotonic() >= deadline: raise
                time.sleep(.25)
        no_uart_owner(); assert CAL.read_bytes() == cal
        for name,data in new_viewer.items(): atomic(SIM/name,data)
        assert all((SIM/n).read_bytes() == b for n,b in new_viewer.items())
    except BaseException:
        service('stop')
        failed = ROOT.with_name('xgoduck-workbench-failed-joint-follow-'+str(time.monotonic_ns()))
        os.rename(ROOT,failed); os.rename(backup,ROOT)
        for name,data in old_viewer.items():
            if data is None: (SIM/name).unlink(missing_ok=True)
            else: atomic(SIM/name,data)
        service('start'); raise
    report.update(installed=True,backup=str(backup),viewer_backup=str(ui_backup),no_uart_opened=True)
    atomic(stage/'installation-result.json',(json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(report))


if __name__ == '__main__': main()
