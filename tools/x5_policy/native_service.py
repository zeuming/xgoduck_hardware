#!/usr/bin/env python3
"""Installed X5 launcher: immutable package on disk, fresh runtime in RAM."""
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import signal
import shutil
import socket
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import http.cookiejar
import urllib.request
import zipfile

INSTALL = Path('/opt/xgoduck-workbench')
STATE = Path('/var/lib/xgoduck-workbench')


def private_json(path, value):
    fd = os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump(value,stream,ensure_ascii=False,allow_nan=False);stream.write('\n')


def remember_holds(snapshot, path):
    """Persist recovery holds only when changed; never persist browser tokens."""
    maintenance=snapshot.get('maintenance') or {};holds={}
    if maintenance.get('fault_hold') is True:
        holds['maintenance-fault-hold.json']=dict(reason=str(maintenance.get('error') or '上次维护故障待复查')[:512],requires_feedback_review=True,restored_from_snapshot=True)
    if maintenance.get('identity_changed') is True:
        holds['maintenance-identity-hold.json']=dict(reason='舵机编号映射待核对',requires_physical_mapping_review=True,restored_from_snapshot=True)
    value=dict(native_recovery_holds=holds)
    if path.exists() and json.loads(path.read_text())==value:return
    temporary=path.with_name('.recovery-'+str(time.monotonic_ns())+'.json')
    private_json(temporary,value);os.replace(temporary,path)


def safe_name(name):
    path=PurePosixPath(name)
    if not name or path.is_absolute() or '..' in path.parts or str(path)!=name:
        raise ValueError('Unsafe package member')
    return path


def unpack(raw, manifest, root):
    if hashlib.sha256(raw).hexdigest()!=manifest['archive_sha256']:
        raise ValueError('Installed archive checksum changed')
    with tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz') as archive:
        items=archive.getmembers()
        if len(items)!=len(manifest['files']) or len({item.name for item in items})!=len(items):
            raise ValueError('Installed archive member set changed')
        for item in items:
            safe_name(item.name);info=manifest['files'].get(item.name)
            if not item.isfile() or not info or item.size!=info['size_bytes']:
                raise ValueError('Installed archive member type/size changed')
            data=archive.extractfile(item).read()
            if hashlib.sha256(data).hexdigest()!=info['sha256']:
                raise ValueError('Installed archive member checksum changed')
            path=root/item.name;path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
            path.write_bytes(data);path.chmod(0o600)
    for wheel in (root/'wheels').glob('*.whl'):
        with zipfile.ZipFile(wheel) as archive:
            for item in archive.infolist():
                safe_name(item.filename.rstrip('/'))
                if stat.S_ISLNK(item.external_attr>>16):raise ValueError('Wheel links are forbidden')
                target=root/'site'/item.filename
                if item.is_dir():target.mkdir(mode=0o700,parents=True,exist_ok=True);continue
                target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
                target.write_bytes(archive.read(item));target.chmod(0o600)


def capture(namespace):
    mounts=namespace['bounded_read']('/proc/self/mountinfo',262144).splitlines()
    roots=[line for line in mounts if len(line.split())>4 and line.split()[4]=='/']
    if len(roots)!=1:raise ValueError('Root mount is not unique')
    return dict(boot_id=namespace['bounded_read']('/proc/sys/kernel/random/boot_id',128),
        cid=namespace['bounded_read']('/sys/block/mmcblk1/device/cid',128),root_mount_line=roots[0],
        model=namespace['bounded_read']('/proc/device-tree/model',4096,binary=True).rstrip(b'\0').decode(),
        swaps=namespace['bounded_read']('/proc/swaps',16384),kernel_log=namespace['bounded_dmesg'](),mounts=mounts)


def _closed_after_stop(root, stop_requested_ns):
    """Accept only a new private shutdown proof, never an old periodic packet."""
    if stop_requested_ns is None:
        return False
    path=root/'native-evidence/live-state.json'
    if path.resolve(strict=True)!=path:
        return False
    descriptor=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(descriptor,'rb') as stream:
        node=os.fstat(stream.fileno())
        if (not stat.S_ISREG(node.st_mode) or node.st_uid!=0 or node.st_mode&0o077
                or node.st_nlink!=1 or not 0<node.st_size<=2*1024*1024
                or node.st_mtime_ns<stop_requested_ns):
            return False
        raw=stream.read(2*1024*1024+1)
    if len(raw)>2*1024*1024:
        return False
    snapshot=json.loads(raw)
    proof=snapshot.get('closed_proof') or {}
    return (snapshot.get('phase') in ('closed','fault')
        and snapshot.get('owns_connection') is False
        and snapshot.get('exit_reconciled') is True
        and proof.get('serial_closed') is True
        and (proof.get('release_confirmed') is True or
             proof.get('no_uart_opened') is True
             and type(proof.get('serial_open_attempts')) is int and proof['serial_open_attempts']==0))


def cleanup_dependencies(root, identity, lifecycle):
    """Reclaim this invocation's rebuildable dependencies; keep every audit file.

    A live or unexpectedly exited child retains RAM. Signal exit codes alone
    do not prove that its finally blocks closed UART. Unknown roots and changed
    root/subtree links are never followed or removed.
    """
    removed=[]
    try:
        def check_root():
            node=root.lstat()
            if (root.resolve(strict=True)!=root or not stat.S_ISDIR(node.st_mode)
                    or node.st_uid!=0 or stat.S_IMODE(node.st_mode)!=0o700
                    or (node.st_dev,node.st_ino)!=identity):
                raise ValueError('Owned RAM directory identity changed')
        check_root()
        child=lifecycle.get('child')
        if child is not None:
            code=child.poll()
            if code is None:
                return dict(cleaned=False,removed=removed,reason='Created child is still running')
            if not (lifecycle.get('stopping') is True and code==0):
                if (lifecycle.get('stopping') is not True or code not in (-15,-2,130)
                        or not _closed_after_stop(root,lifecycle.get('stop_requested_ns'))):
                    return dict(cleaned=False,removed=removed,reason='No current controlled shutdown proof')
        paths=[]
        for name in ('site','wheels'):
            path=root/name
            try:node=path.lstat()
            except FileNotFoundError:continue
            if (not stat.S_ISDIR(node.st_mode) or path.resolve(strict=True)!=path
                    or node.st_uid!=0 or stat.S_IMODE(node.st_mode)!=0o700
                    or (node.st_dev,node.st_ino)!=lifecycle.get('dependency_nodes',{}).get(name)):
                raise ValueError('Dependency subtree is not an owned directory: '+name)
            paths.append((path,(node.st_dev,node.st_ino)))
        for path,identity_at_check in paths:
            check_root()
            node=path.lstat()
            if not stat.S_ISDIR(node.st_mode) or (node.st_dev,node.st_ino)!=identity_at_check:
                raise ValueError('Dependency subtree identity changed: '+path.name)
            shutil.rmtree(path)
            removed.append(path.name)
        return dict(cleaned=True,removed=removed,reason=None)
    except Exception as error:
        # Retention/cleanup diagnostics must not replace the startup failure.
        return dict(cleaned=False,removed=removed,reason=str(error))


def main():
    if os.geteuid()!=0:raise ValueError('Run this installed service as root')
    # Serialize starts independently of the serial port. A duplicate invocation
    # cannot terminate an existing controller or remove its runtime directory.
    lock=os.open('/run/xgoduck-workbench.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    seal=json.loads((INSTALL/'installation.json').read_text())
    for name,info in seal['files'].items():
        path=INSTALL/name;node=path.lstat();raw=path.read_bytes()
        if not stat.S_ISREG(node.st_mode) or node.st_uid!=0 or node.st_mode&0o077 or hashlib.sha256(raw).hexdigest()!=info:
            raise ValueError('Installed file changed: '+name)
    manifest=json.loads((INSTALL/'deployment.json').read_text())
    root=Path(tempfile.mkdtemp(prefix='xgoduck-native-',dir='/dev/shm'))
    node=root.lstat()
    lifecycle=dict(child=None,stopping=False,stop_requested_ns=None,dependency_nodes={})
    try:
        root.chmod(0o700)
        # Pin these new empty roots before unpacking can fail partway through.
        for name in ('site','wheels'):
            path=root/name;path.mkdir(mode=0o700)
            dependency=path.lstat()
            lifecycle['dependency_nodes'][name]=(dependency.st_dev,dependency.st_ino)
        return run_owned(root,manifest,lifecycle)
    finally:
        cleanup=cleanup_dependencies(root,(node.st_dev,node.st_ino),lifecycle)
        if not cleanup['cleaned']:
            print('Native RAM dependencies retained: '+str(cleanup['reason']),file=sys.stderr,flush=True)


def run_owned(root, manifest, lifecycle):
    unpack((INSTALL/'native-bundle.tar.gz').read_bytes(),manifest,root)
    guard_path=root/'service/guard-runtime.py'
    guard_path.write_bytes(guard_path.read_bytes()+b'\n'+(INSTALL/'persistent_boot.py').read_bytes())
    namespace={};exec(compile(guard_path.read_bytes(),'<installed-native-guards>','exec'),namespace)
    metadata=capture(namespace)
    expected=namespace['boot_expected'](json.loads((root/'service/expected.json').read_text()),metadata,namespace['classify_feedback_log'])
    expected['generation']=int(time.time())%0x7ffffffe+1
    namespace['validate_os'](metadata,expected)
    config=[line for line in metadata['mounts'] if len(line.split())>4 and line.split()[2]=='179:1']
    shm=[line for line in metadata['mounts'] if len(line.split())>4 and line.split()[4]=='/dev/shm']
    if len(config)!=1 or len(shm)!=1 or config[0].split()[4]!='/boot/config' or 'noexec' in shm[0].split()[5].split(','):
        raise ValueError('CONFIG/RAM mounts changed')
    serial=None;node=None;reasons=[]
    try:namespace['validate_kernel'](metadata['kernel_log'],expected)
    except Exception as error:reasons.append(str(error))
    try:
        namespace['bind_serial'](expected);node,identity=namespace['serial_proof'](expected)
        holders=namespace['_holders'](node.st_rdev,time.monotonic()+.2)
        if holders:raise ValueError('UART already has an owner')
        serial=dict(identity=identity,node=dict(device=node.st_dev,inode=node.st_ino,rdev=node.st_rdev),holders=[])
    except Exception as error:reasons.append(str(error));node=None
    reason=('硬件检查未通过；尚未启动舵机操作和采集：'+'；'.join(reasons)) if reasons else '已明确选择仅启动诊断网页；尚未执行完整硬件连接检查，尚未启动舵机操作和采集。'
    startup=dict(schema_version=1,kind='native_offline_startup_fault',reason=reason[:16384],
        core_metadata={k:metadata[k] for k in ('boot_id','cid','model','root_mount_line','swaps')},
        kernel_log=metadata['kernel_log'],kernel_sha256=hashlib.sha256(metadata['kernel_log'].encode()).hexdigest(),
        kernel_error='；'.join(reasons) or None,serial_metadata=serial,serial_error=None,
        hardware_gate_passed=False,uart_granted=False,serial_open_attempts=0,serial_bytes_written=0,
        board_files_written=False,automatic_retry=False)
    deployment=json.loads((root/'service/deployment.json').read_text())
    deployment.update(generation=expected['generation'],config_mount_review=config)
    (root/'service/expected.json').write_text(json.dumps(expected,ensure_ascii=False)+'\n')
    (root/'service/deployment.json').write_text(json.dumps(deployment,ensure_ascii=False)+'\n')
    recovery_path=STATE/'recovery-holds.json'
    if recovery_path.exists():
        recovery_info=recovery_path.lstat()
        if not stat.S_ISREG(recovery_info.st_mode) or recovery_info.st_uid!=0 or recovery_info.st_mode&0o077:
            raise ValueError('Persistent recovery holds are not private')
        recovery=json.loads(recovery_path.read_text())
        if set(recovery)!={'native_recovery_holds'} or set(recovery['native_recovery_holds'])-{'maintenance-fault-hold.json','maintenance-identity-hold.json'}:
            raise ValueError('Persistent recovery holds changed shape')
        snapshot_path=root/'service/startup-snapshot.json';snapshot=json.loads(snapshot_path.read_text())
        snapshot['native_recovery_holds']=recovery['native_recovery_holds'];snapshot_path.write_text(json.dumps(snapshot,ensure_ascii=False)+'\n')
    files={name:dict(size_bytes=(root/name).stat().st_size,sha256=hashlib.sha256((root/name).read_bytes()).hexdigest()) for name in manifest['files']}
    info=root.stat()
    private_json(root/'native-pin.json',dict(files=files,ram_directory_node=[info.st_dev,info.st_ino],
        shm_mount=shm,serial_node_posix=None if node is None else dict(dev=node.st_dev,ino=node.st_ino,rdev=node.st_rdev),startup_fault=startup))
    STATE.mkdir(mode=0o700,exist_ok=True)
    audit=STATE/'boot-records';audit.mkdir(mode=0o700,exist_ok=True)
    private_json(audit/(metadata['boot_id']+'-'+str(time.monotonic_ns())+'.json'),dict(startup=startup,ram_directory=str(root),prior_faults_cleared=False,motor_start_attempted=False))
    # Wi-Fi address may arrive after network-online.target. Wait for binding;
    # never retry or start any hardware action here.
    deadline=time.monotonic()+60
    while True:
        probe=socket.socket()
        try:probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);probe.bind((deployment['bind_host'],deployment['port']));break
        except OSError as error:
            if error.errno!=99 or time.monotonic()>=deadline:raise
            time.sleep(1)
        finally:probe.close()
    child=subprocess.Popen([sys.executable,'-I','-B',str(root/'service/bootstrap.py')],stdin=subprocess.DEVNULL,cwd=root)
    lifecycle['child']=child
    stopping=False
    def stop(*args):
        nonlocal stopping
        if not stopping:lifecycle['stop_requested_ns']=time.time_ns()
        stopping=True
        lifecycle['stopping']=True
        if child.poll() is None:child.terminate()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    print('X5 工作台启动：http://192.168.1.11:8767/',flush=True)
    client=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    origin='http://'+deployment['bind_host']+':'+str(deployment['port'])
    connected=False
    while child.poll() is None:
        try:
            if not connected:client.open(origin+'/',timeout=1).read();connected=True
            snapshot=json.load(client.open(origin+'/api/live-status',timeout=1))
            remember_holds(snapshot,recovery_path)
        except (OSError,ValueError):connected=False
        time.sleep(1)
    raise SystemExit(0 if stopping and child.returncode in (0,-2,-15,130) else child.returncode)


if __name__=='__main__':
    main()
