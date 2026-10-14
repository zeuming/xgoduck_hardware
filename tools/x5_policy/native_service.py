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
    metadata = dict(boot_id=namespace['bounded_read']('/proc/sys/kernel/random/boot_id',128),
        cid=namespace['bounded_read']('/sys/block/mmcblk1/device/cid',128),root_mount_line=roots[0],
        model=namespace['bounded_read']('/proc/device-tree/model',4096,binary=True).rstrip(b'\0').decode(),
        swaps=namespace['bounded_read']('/proc/swaps',16384),kernel_log=namespace['bounded_dmesg'](),mounts=mounts)
    namespace['check_platform'](metadata)
    metadata['kernel_log'] = restore_archived_kernel(metadata, namespace['restore_kernel_history'])
    return metadata


def kernel_boot_anchor(kernel, model):
    """Recognize the two observed complete X5 early-boot header forms."""
    import re
    first = kernel.splitlines()[:2]
    version_model_start = (kernel.endswith('\n') and len(first) == 2
        and re.fullmatch(r'\[    0\.000000\] Linux version [0-9]+\.[0-9]+(?:\.[0-9]+)?[^ \t\r\n]* .+', first[0]) is not None
        and first[1] == '[    0.000000] Machine model: ' + model)
    return (kernel.startswith('[    0.000000] Booting Linux on physical CPU ')
            or version_model_start)


def read_boot_kernel_journal(boot_id):
    """Read bounded current-boot kernel source records without opening UART."""
    import re
    import select
    if re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', boot_id) is None:
        raise ValueError('Kernel journal boot identity is invalid')
    fields='MESSAGE,_SOURCE_MONOTONIC_TIMESTAMP,_BOOT_ID,_TRANSPORT,SYSLOG_IDENTIFIER,SYSLOG_PID'
    child=subprocess.Popen(['journalctl','--quiet','--no-pager','--boot='+boot_id.replace('-',''),
        '_TRANSPORT=kernel','--output=json','--all','--lines=4096','--output-fields='+fields],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,close_fds=True)
    buffers={child.stdout.fileno():bytearray(),child.stderr.fileno():bytearray()}
    active=set(buffers);deadline=time.monotonic()+2.
    try:
        while active:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise ValueError('Same-boot kernel journal read exceeded its deadline')
            ready,_,exceptional=select.select(list(active),[],list(active),remaining)
            if exceptional:raise ValueError('Same-boot kernel journal pipe failed')
            for descriptor in ready:
                data=os.read(descriptor,65536)
                if not data:active.remove(descriptor);continue
                buffers[descriptor].extend(data)
                maximum=1048576 if descriptor==child.stdout.fileno() else 8192
                if len(buffers[descriptor])>maximum:
                    raise ValueError('Same-boot kernel journal exceeds its bounded size')
        code=child.wait(timeout=max(.001,deadline-time.monotonic()))
        raw=bytes(buffers[child.stdout.fileno()]);error=bytes(buffers[child.stderr.fileno()])
        if code or error or not raw:
            raise ValueError('Same-boot kernel journal is unavailable or reported stderr')
        return raw
    finally:
        if child.poll() is None:child.kill();child.wait(timeout=.3)
        child.stdout.close();child.stderr.close()


def restore_journal_kernel(metadata, raw, restore):
    """Recover only an evicted prefix across 64 exact timestamped records."""
    import base64
    import gzip
    import re
    if type(raw) is not bytes or not 0<len(raw)<=1048576 or not raw.endswith(b'\n'):
        raise ValueError('Same-boot kernel journal is incomplete or exceeds its bound')
    current=metadata['kernel_log']
    if type(current) is not str or not current.endswith('\n'):
        raise ValueError('Current kernel log is incomplete')
    entries=raw.decode('utf-8','strict').splitlines()
    if not 1<=len(entries)<=4096:raise ValueError('Kernel journal record count is invalid')
    rendered=[];previous=-1
    for entry in entries:
        record=json.loads(entry)
        if (type(record) is not dict or record.get('_BOOT_ID')!=metadata['boot_id'].replace('-','')
                or record.get('_TRANSPORT')!='kernel'):
            raise ValueError('Kernel journal contains another boot or a non-kernel record')
        source=record.get('_SOURCE_MONOTONIC_TIMESTAMP')
        if type(source) is not str or re.fullmatch(r'[0-9]{1,16}',source) is None:
            raise ValueError('Kernel journal lacks a valid source monotonic timestamp')
        timestamp=int(source)
        if timestamp<previous:raise ValueError('Kernel journal source timestamps regressed')
        previous=timestamp
        message=record.get('MESSAGE')
        if type(message) is list and all(type(value) is int and 0<=value<=255 for value in message):
            message=bytes(message).decode('utf-8','strict')
        if type(message) is not str:
            raise ValueError('Kernel journal message encoding is unsupported')
        message=''.join('\\x'+format(ord(char),'02x')
            if (ord(char)<32 and char not in '\t\r\n') or ord(char)==127 else char for char in message)
        identifier,pid=record.get('SYSLOG_IDENTIFIER'),record.get('SYSLOG_PID')
        if identifier is not None and identifier!='kernel':
            if (type(identifier) is not str or re.fullmatch(r'[A-Za-z0-9_./-]+',identifier) is None
                    or pid is not None and (type(pid) is not str or re.fullmatch(r'[0-9]+',pid) is None)):
                raise ValueError('Kernel journal syslog prefix is invalid')
            message=identifier+('['+pid+']' if pid is not None else '')+': '+message
        prefix=f'[{timestamp//1000000:5d}.{timestamp%1000000:06d}] '
        rendered.append(prefix+message.replace('\n','\n'+' '*len(prefix))+'\n' if message else '\n')
    journal=''.join(rendered)
    if len(journal.encode())>262144 or not kernel_boot_anchor(journal,metadata['model']):
        raise ValueError('Same-boot kernel journal lacks complete bounded early boot history')
    lines,observed=journal.splitlines(),current.splitlines()
    if not observed:raise ValueError('Current kernel log is empty')
    matches=[]
    for offset,line in enumerate(lines):
        if line!=observed[0]:continue
        count=0
        while count<min(len(lines)-offset,len(observed)) and lines[offset+count]==observed[count]:
            count+=1
        timestamped=sum(re.match(r'^\[\s*[0-9]+\.[0-9]+\]',line) is not None
                        for line in observed[:count])
        if timestamped>=64:matches.append((offset,count,timestamped))
    if len(matches)!=1:raise ValueError('Kernel journal has no unique 64-record exact dmesg overlap')
    offset,count,timestamped=matches[0]
    # Journald can omit its own later /dev/kmsg messages. Its only contribution
    # is the evicted prefix, proven by the contiguous overlap above. Existing
    # dmesg continuations remain unchanged and covered by the exact baseline.
    result=''.join(journal.splitlines(keepends=True)[:offset])+current
    checked=restore(current,result)
    if len(result.encode())>262144 or result.splitlines()!=checked.splitlines():
        raise ValueError('Kernel journal reconstruction differs from its exact overlap proof')
    pack=lambda value:base64.b64encode(gzip.compress(value,mtime=0)).decode('ascii')
    proof=dict(source='same_boot_kernel_journal',boot_id=metadata['boot_id'],
        source_timestamp_field='_SOURCE_MONOTONIC_TIMESTAMP',journal_records=len(entries),
        overlap_lines=count,overlap_timestamped_records=timestamped,recovered_prefix_lines=offset,
        journal_sha256=hashlib.sha256(raw).hexdigest(),journal_raw_gzip_base64=pack(raw),
        dmesg_sha256=hashlib.sha256(current.encode()).hexdigest(),dmesg_raw_gzip_base64=pack(current.encode()),
        reconstructed_sha256=hashlib.sha256(result.encode()).hexdigest(),hardware_gate_passed=False)
    return result,proof


def restore_archived_kernel(metadata, restore):
    """Keep this boot's previously saved evidence across service restarts."""
    directory=STATE/'boot-records'
    histories=[]
    if directory.exists():
        node=directory.lstat()
        if not stat.S_ISDIR(node.st_mode) or node.st_uid!=0 or node.st_mode&0o077:
            raise ValueError('Boot history directory is not private')
        paths=sorted(directory.glob(metadata['boot_id']+'-*.json'))
        if len(paths)>64:raise ValueError('Too many boot history archives')
        for path in paths:
            fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
            with os.fdopen(fd,'rb') as stream:
                node=os.fstat(stream.fileno())
                if (not stat.S_ISREG(node.st_mode) or node.st_uid!=0 or node.st_mode&0o077
                        or node.st_nlink!=1 or not 0<node.st_size<=2*1024*1024):
                    raise ValueError('Boot history archive is not a private regular file')
                record=json.loads(stream.read(2*1024*1024+1))['startup']
            if any(record['core_metadata'][key]!=metadata[key] for key in ('boot_id','cid','model','root_mount_line')):
                raise ValueError('Boot history archive belongs to another platform or boot')
            history=record['kernel_log']
            if hashlib.sha256(history.encode()).hexdigest()!=record['kernel_sha256']:
                raise ValueError('Boot history archive checksum changed')
            # Reconstruction joins records with LF, while original dmesg can
            # contain CRLF printk records. Verify each original hash first,
            # then normalize only this terminator for cross-archive equality.
            # Keep every record and leave the archived bytes untouched.
            histories.append(history.replace('\r\n', '\n'))
    kernel=metadata['kernel_log']
    if histories:
        # A later retained history must extend the earlier one byte-for-byte.
        histories.sort(key=len)
        for before,after in zip(histories,histories[1:]):
            if not after.startswith(before):raise ValueError('Retained boot archives disagree')
        return restore(kernel,histories[-1])
    if not kernel_boot_anchor(kernel,metadata['model']):
        raw=read_boot_kernel_journal(metadata['boot_id'])
        kernel,proof=restore_journal_kernel(metadata,raw,restore)
        metadata['kernel_journal_recovery']=proof
    return kernel


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
    usb_history_review = None
    if reasons and serial is not None:
        # Reuse the exact reconnect state machine without importing transports
        # or opening UART during startup. Retain the proof in the boot record.
        import ast, copy, re, types
        tree = ast.parse((root/'service/native_transports.py').read_text())
        function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                        and item.name == 'usb_recovery_proof')
        scope = dict(re=re, copy=copy, hashlib=hashlib, Path=Path,
                     reader=types.SimpleNamespace(MAX_KERNEL_BYTES=2*1024*1024))
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<startup-usb-review>', 'exec'), scope)
        try:
            usb_history_review = scope['usb_recovery_proof'](metadata['kernel_log'], expected,
                identity, namespace['validate_kernel'])
            if usb_history_review is not None:
                reasons = []
        except Exception:
            pass
    reason=('硬件检查未通过；尚未启动舵机操作和采集：'+'；'.join(reasons)) if reasons else '已明确选择仅启动诊断网页；尚未执行完整硬件连接检查，尚未启动舵机操作和采集。'
    startup=dict(schema_version=1,kind='native_offline_startup_fault',reason=reason[:16384],
        core_metadata={k:metadata[k] for k in ('boot_id','cid','model','root_mount_line','swaps')},
        kernel_log=metadata['kernel_log'],kernel_sha256=hashlib.sha256(metadata['kernel_log'].encode()).hexdigest(),
        kernel_journal_recovery=metadata.get('kernel_journal_recovery'),
        kernel_error='；'.join(reasons) or None,serial_metadata=serial,serial_error=None,
        usb_history_review=usb_history_review,
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
    # Loaded from the checksum-verified bundle. Sampling is confined to the
    # launcher process, outside the controller and HTTP request paths.
    health_scope={'__name__':'x5_health_launcher','__file__':str(root/'service/system_health.py')}
    exec(compile((root/'service/system_health.py').read_bytes(),health_scope['__file__'],'exec'),health_scope)
    health=health_scope['HealthMonitor'](root,STATE/'system-health')
    child=subprocess.Popen([sys.executable,'-I','-B',str(root/'service/bootstrap.py')],stdin=subprocess.DEVNULL,cwd=root)
    lifecycle['child']=child
    try:
        health.start()
    except Exception as error:
        print('X5 health monitor unavailable: '+type(error).__name__,file=sys.stderr,flush=True)
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
    origin='http://127.0.0.1:'+str(deployment['port'])
    connected=False
    try:
        while child.poll() is None:
            try:
                if not connected:client.open(origin+'/',timeout=1).read();connected=True
                snapshot=json.load(client.open(origin+'/api/live-status',timeout=1))
                remember_holds(snapshot,recovery_path)
            except (OSError,ValueError):connected=False
            time.sleep(1)
    finally:
        health.close()
    raise SystemExit(0 if stopping and child.returncode in (0,-2,-15,130) else child.returncode)


if __name__=='__main__':
    main()
