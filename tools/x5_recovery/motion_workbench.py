"""X5 RAM deployment and single-owner motion RPC for the unified workbench.

Construction/import is local only. An explicit connect deploys verified models
and the saved calibration to tmpfs, then opens one UART. Starts are separate
generation-bound requests; neither deployment nor navigation enables motors.
"""
from __future__ import annotations

import copy
import hashlib
import inspect
import io
import json
import math
import os
from pathlib import Path
import re
import secrets
import tarfile
import threading
import time

import calibration_restore
import feedback_connection_check
import feedback_stream
import maintenance_transport
import run_ram_model_web as archive_tools
from transport_config import resolve_transport

ROOT = Path(__file__).resolve().parents[2]
OLD_BUNDLE = ROOT / 'output/x5_recovery/new-board-20261003T114727Z/prepared/minimal-motion-calibration-ram-bundle.tar.gz'
OLD_MANIFEST = OLD_BUNDLE.with_name('minimal-motion-calibration-ram-bundle.manifest.json')
OLD_SHA = '032e0ab8b8fc2bc5c6e5e6d819c7de9819196313b9787b1b464a691f7347597e'
# The replacement board's system CONFIG mount was inspected independently.
# This motion capability preserves it, unlike the old card's unmounted-CONFIG
# calibration capability. Every runtime check pins this exact mount; the model
# audit denies persistent writes. Neither old runner nor its review is changed.
CONFIG_MOUNT_REVIEW = ['425 21 179:1 / /boot/config rw,relatime shared:220 - vfat /dev/mmcblk1p1 rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro']
# The same CONFIG partition was independently read after the reviewed cold boot.
# Mount propagation group numbers belong to a boot, not to persistent storage.
# Two metadata captures after the reviewed Wi-Fi reboot confirmed this exact row.
COLD_CONFIG_MOUNT_REVIEW = ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat /dev/mmcblk1p1 rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro']
# Two metadata observations after the IMU recovery update retained the same
# card/root and this exact CONFIG row in the explicitly reviewed new boot.
IMU_RECOVERY_CONFIG_MOUNT_REVIEW = ['425 21 179:1 / /boot/config rw,relatime shared:224 - vfat /dev/mmcblk1p1 rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro']
# Two read-only observations after the user's intentional X5 power cycle
# confirmed this CONFIG row on the same card. The boot binding stays explicit.
INTENTIONAL_POWERCYCLE_CONFIG_MOUNT_REVIEW = ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat /dev/mmcblk1p1 rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro']
STOP_SOURCES = ('button', 'window-blur', 'page-hidden', 'pagehide', 'workspace-leave')
MODEL_HASHES = {
    'walk': '5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f',
    'getup': 'c414f2bd84b0a79b89f0ba6997135eb45e4ecd9df9271ea989c237864027b2a3',
    'pick': 'f34e990dda785e9a8c53890bdcd0f7f1d457592691fd3703f3776b48fb2d14b8',
    'roulade': '8adb1e6bc3f5eef751ce4edbf940f6d5a25fdec3ec4a798e1d9ca135c8248e97',
}


def calibration_bytes(config):
    calibration_restore.validate_calibration_snapshot(config)
    return (json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()


def config_mount_for_boot(boot_id):
    if boot_id == feedback_stream.NATIVE_SERVICE_RESTART_BOOT_ID:
        # Three metadata observations pinned the same CONFIG row after this boot.
        return list(COLD_CONFIG_MOUNT_REVIEW)
    if boot_id == feedback_stream.INTENTIONAL_POWERCYCLE_BOOT_ID:
        return list(INTENTIONAL_POWERCYCLE_CONFIG_MOUNT_REVIEW)
    if boot_id == feedback_stream.IMU_RECOVERY_RECONNECT_BOOT_ID:
        return list(IMU_RECOVERY_CONFIG_MOUNT_REVIEW)
    if boot_id in (feedback_stream.COLD_RECONNECT_BOOT_ID, feedback_stream.WIFI_RECONNECT_BOOT_ID):
        return list(COLD_CONFIG_MOUNT_REVIEW)
    if boot_id in (feedback_stream.REVIEWED_BOOT_ID, feedback_stream.REBOOT_BOOT_ID):
        return list(CONFIG_MOUNT_REVIEW)
    raise ValueError('运控部署的启动身份尚未核对')


def prepare_bundle(config, *, boot_id=feedback_stream.REBOOT_BOOT_ID):
    """Reuse only digest-verified wheels; package current controller and models."""
    config_mount_review = config_mount_for_boot(boot_id)
    old = OLD_BUNDLE.read_bytes()
    if len(old) != 39209311 or hashlib.sha256(old).hexdigest() != OLD_SHA:
        raise ValueError('固定的 X5 运行依赖校验失败')
    manifest = json.loads(OLD_MANIFEST.read_text())
    files = {f['archive_path']: {k: f[k] for k in ('size_bytes', 'sha256')} for f in manifest['files']}
    members = archive_tools.inspect_tar(old, files)
    members = {name: data for name, data in members.items() if name.startswith('wheels/')}
    for name in ('motion.py', 'rl_core.py', 'infer.py'):
        members['policy/' + name] = (ROOT / 'tools/x5_policy' / name).read_bytes()
    models = ROOT / 'output/x5_policy_bundle/models'
    members['policy/models/manifest.json'] = (models / 'manifest.json').read_bytes()
    for name, digest in MODEL_HASHES.items():
        data = (models / f'xgoduck_{name}.onnx').read_bytes()
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError('运控模型摘要不符：' + name)
        members[f'policy/models/xgoduck_{name}.onnx'] = data
    members['policy/calibration.json'] = calibration_bytes(config)
    wheel_bytes = archive_tools.inspect_wheels(members)
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as target:
        for name, data in sorted(members.items()):
            entry = tarfile.TarInfo(name)
            entry.size, entry.mode = len(data), 0o600
            target.addfile(entry, io.BytesIO(data))
    data = buffer.getvalue()
    return data, dict(archive_sha256=hashlib.sha256(data).hexdigest(), archive_size=len(data),
                     scope='explicit_web_motion', automatic_motor_start=False,
                     config_mount_review=config_mount_review, persistent_model_writes=False,
                     member_bytes=sum(map(len, members.values())), wheel_bytes=wheel_bytes,
                     calibration_sha256=hashlib.sha256(members['policy/calibration.json']).hexdigest(),
                     model_hashes=MODEL_HASHES,
                     files={name: dict(size_bytes=len(value), sha256=hashlib.sha256(value).hexdigest())
                            for name, value in members.items()})


# This worker uses the existing policy math/controller, with a one-shot serial
# lifecycle and proved STOP. All model/dependency/config writes are within tmpfs.
WORKER = r'''
import base64, ctypes, io, platform, shutil, tarfile, tempfile, zipfile
archive_bytes = None
ram_directory = None
controller = None
request_counts = {}
serial_open_attempts = 0
opened_transports = []
out_lock = threading.Lock()
exit_event = threading.Event()
health_error = None
parent_pid = os.getppid()
ram_identity = None
last_status = 0.
def output(kind, **values):
    with out_lock:
        print(json.dumps(dict(kind=kind, deployment_generation=EXPECTED['generation'], **values),
                         ensure_ascii=False, allow_nan=False, separators=(',', ':')), flush=True)
def ram_health():
    global ram_identity
    proof = health_snapshot(EXPECTED)
    mount_lines = bounded_read('/proc/self/mountinfo', 262144).splitlines()
    entries = [line.split() for line in mount_lines]
    mount = [f for f in entries if len(f)>6 and f[4]=='/dev/shm']
    if len(mount)!=1: raise ProbeError('RAM mount is not unique')
    row=mount[0]; sep=row.index('-',6)
    if row[sep+1]!='tmpfs' or 'rw' not in row[5].split(',') or any(x in row[5].split(',')+row[sep+3].split(',') for x in ('noexec','ro')):
        raise ProbeError('RAM must be writable executable tmpfs')
    config_mounts=[line for line in mount_lines if len(line.split())>4 and line.split()[2]=='179:1']
    if config_mounts!=DEPLOY['config_mount_review']: raise ProbeError('Reviewed system CONFIG mount changed')
    st=os.stat('/dev/shm'); identity=(row[0],row[2],st.st_dev,st.st_ino)
    if ram_identity is None: ram_identity=identity
    elif identity!=ram_identity: raise ProbeError('RAM mount changed')
    if os.getppid()!=parent_pid: raise ProbeError('Motion SSH parent disappeared')
    return proof
def cancelled(signum, frame): exit_event.set()
for sig in (signal.SIGTERM,signal.SIGHUP,signal.SIGINT): signal.signal(sig,cancelled)
# Parent death stops the controller. The ESP32 independently has its target watchdog.
libc=ctypes.CDLL(None, use_errno=True)
if libc.prctl(1,signal.SIGTERM,0,0,0)!=0: raise ProbeError('Parent death guard unavailable')
def unpack_bundle(data):
    members=inspect_tar(data, DEPLOY['files'])
    if inspect_wheels(members)!=DEPLOY['wheel_bytes']: raise ProbeError('Wheel budget differs')
    for name,value in members.items():
        target=Path(ram_directory)/name
        target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
        with target.open('xb') as stream: stream.write(value)
        target.chmod(0o600)
    for name,value in members.items():
        if not name.startswith('wheels/'): continue
        with zipfile.ZipFile(io.BytesIO(value)) as wheel:
            for item in wheel.infolist():
                relative=safe_relative_path(item.filename)
                target=Path(ram_directory)/'site'/relative
                if item.is_dir(): target.mkdir(mode=0o700,parents=True,exist_ok=True); continue
                target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
                with wheel.open(item) as source, target.open('xb') as stream:
                    shutil.copyfileobj(source,stream,65536)
                target.chmod(0o600)
def run_motion():
    global ram_directory, controller, health_error
    serial_closed=False
    try:
        bind_serial(EXPECTED)
        if platform.machine()!='aarch64' or sys.version_info[:2]!=(3,10): raise ProbeError('Runtime requires aarch64 Python 3.10')
        proof=ram_health()
        required=DEPLOY['archive_size']+DEPLOY['member_bytes']+DEPLOY['wheel_bytes']+128*1024*1024
        shm=os.statvfs('/dev/shm')
        available=int(next(line.split()[1] for line in bounded_read('/proc/meminfo').splitlines() if line.startswith('MemAvailable:')))*1024
        if shm.f_bavail*shm.f_frsize<required or available<required+256*1024*1024: raise ProbeError('Insufficient RAM for motion models')
        ram_directory=tempfile.mkdtemp(prefix='xgoduck-motion-',dir='/dev/shm'); os.chmod(ram_directory,0o700)
        output('transfer_ready', archive_bytes=DEPLOY['archive_size'], identity=proof, ram_directory=ram_directory)
        raw=bytearray(); deadline=time.monotonic()+120
        while len(raw)<DEPLOY['archive_size']:
            if exit_event.is_set() or time.monotonic()>deadline: raise ProbeError('Transfer interrupted')
            if not select.select([sys.stdin.fileno()],[],[],.2)[0]: continue
            chunk=os.read(sys.stdin.fileno(),min(65536,DEPLOY['archive_size']-len(raw)))
            if not chunk: raise ProbeError('Transfer ended early')
            raw.extend(chunk)
        if hashlib.sha256(raw).hexdigest()!=DEPLOY['archive_sha256']: raise ProbeError('Archive digest differs')
        ram_health(); unpack_bundle(bytes(raw)); del raw
        os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',
                          TMPDIR=ram_directory,XDG_CACHE_HOME=ram_directory)
        # Deny writes outside this private tmpfs or the one reviewed UART.
        def audit(event,args):
            if event=='open' and args:
                path=args[0]; mode=args[1] if len(args)>1 else None; flags=args[2] if len(args)>2 else 0
                writes=(isinstance(mode,str) and any(c in mode for c in 'wax+')) or (isinstance(flags,int) and flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
                if isinstance(path,(str,bytes)):
                    path=os.path.abspath(os.fsdecode(path))
                    if path.startswith('/dev/mmcblk'): raise PermissionError('Model cannot open storage block devices')
                    if path.startswith(('/dev/tty','/dev/serial','/dev/bus/usb')) and path!=serial_path(EXPECTED): raise PermissionError('Unreviewed device')
                    if writes and path!=serial_path(EXPECTED) and not path.startswith(ram_directory+'/'): raise PermissionError('Motion writes are RAM only')
        sys.addaudithook(audit)
        sys.path[:0]=[ram_directory+'/site',ram_directory+'/policy']
        import infer, motion
        sessions={name:infer.session(name) for name in DEPLOY['model_hashes']}
        for s in sessions.values():
            obs=infer.np.zeros((1,61),infer.np.float32); obs[0,5]=-1; infer.infer(s,obs)
        class ReviewedTransport(motion.SerialTransport):
            def open(self):
                global serial_open_attempts
                ram_health(); node,_=serial_proof(EXPECTED)
                if _holders(node.st_rdev,time.monotonic()+.2): raise ProbeError('UART already has an owner')
                serial_open_attempts += 1; opened_transports.append(self)
                super().open()
                try:
                    serial_proof(EXPECTED,self.serial.fileno())
                    fcntl.flock(self.serial.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                    fcntl.ioctl(self.serial.fileno(),termios.TIOCEXCL)
                    owners=[x for x in _holders(node.st_rdev,time.monotonic()+.2) if x['pid']!=os.getpid()]
                    if owners: raise ProbeError('Another UART owner appeared')
                    time.sleep(.75)
                    ram_health()
                    if EXPECTED.get('discover_serial') is True:
                        request_counts[str(motion.HELLO)]=request_counts.get(str(motion.HELLO),0)+1
                        if super().request(motion.HELLO)!=b'XGODUCK_MOTION_1':
                            raise ProbeError('Selected USB serial device does not run the ESP32 motion protocol')
                except BaseException: self.close(); raise
            def request(self,kind,payload=b''):
                if kind not in (motion.HELLO,motion.STATE,motion.CONFIG,motion.ARM,motion.TARGET,motion.STOP,motion.VOLTAGE_STATE,motion.TELEMETRY_CAPS): raise ProbeError('Unapproved motion request')
                if health_error and kind not in (motion.STOP,motion.STATE,motion.VOLTAGE_STATE): raise ProbeError(health_error)
                proof_began=time.monotonic()
                hot=kind in (motion.STATE,motion.VOLTAGE_STATE,motion.TARGET,motion.STOP)
                check=globals().get('serial_request_proof',serial_proof) if hot else serial_proof
                check(EXPECTED,self.serial.fileno())
                proof_ms=(time.monotonic()-proof_began)*1000
                request_counts[str(kind)]=request_counts.get(str(kind),0)+1
                try: return super().request(kind,payload)
                finally:
                    timings=getattr(self,'request_timings',None)
                    if timings and timings[-1]['kind']==kind:
                        timings[-1]['identity_proof_ms']=round(proof_ms,3)
                        timings[-1]['identity_proof_scope']='pinned-open-uart' if check is not serial_proof else 'full-usb-identity'
        class OneShotController(motion.MotionController):
            def stop(self, source=None):
                with self._lock:
                    self._requested_stop_source=source
                    try: return super().stop()
                    finally: self._requested_stop_source=None
            def _stop_locked(self, reason=None):
                # Capture the triggering feedback before base STOP clears the
                # mode/target. This is RAM-only evidence, with no extra UART
                # request or output before torque release.
                before=super().status() if self._mode!='off' else None
                timing=getattr(self._transport,'timing_diagnostics',None)
                transport_diagnostic=timing() if before is not None and callable(timing) else None
                super()._stop_locked(reason)
                if before is not None:
                    previous=getattr(self,'_last_stop_diagnostic',None)
                    self._last_stop_diagnostic=dict(sequence=(previous or {}).get('sequence',0)+1,
                        reason=reason, source=getattr(self,'_requested_stop_source',None) if reason is None else None,
                        recorded_monotonic=self._clock(), before=before,
                        transport=transport_diagnostic)
            def _service_stop_locked(self):
                if self._pending_stop and self._connected:
                    self._request(motion.STOP)
                    state=self._read_state()
                    self._accept_state_locked(state)
                    if state['online_mask']!=motion.ALL_MASK or state['fault_mask'] or state['armed'] or state['cookie'] or state['torque_mask']:
                        message='停止已发送，但全部舵机扭矩关闭尚未确认'
                        if self._error:
                            message=self._error if message in self._error else self._error+'；'+message
                        diagnostic=getattr(self,'_last_stop_diagnostic',None)
                        if diagnostic is not None and 'verification_failure' not in diagnostic:
                            # Published diagnostics are immutable. A status
                            # reader may be copying the previous value without
                            # holding the controller lock.
                            self._last_stop_diagnostic=dict(diagnostic,
                                verification_failure=dict(reason=message,
                                    recorded_monotonic=self._clock(), after=super().status()))
                        raise motion.MotionError(message)
                    self._pending_stop=False
            def _run(self):
                failure = None
                try:
                    self._connect_once()
                    while not self._closed.is_set() and not exit_event.is_set():
                        began=self._clock()
                        if health_error: raise motion.MotionError(health_error)
                        self._cycle()
                        self._closed.wait(max(.001,motion.PERIOD-(self._clock()-began)))
                except BaseException as error:
                    failure = str(error) or type(error).__name__
                    self._terminal_error = failure
                    with self._lock:
                        self._stop_locked(failure)
                        try: self._service_stop_locked()
                        except BaseException: pass
                    self._disconnect(error)
                    # The RPC process must finish and prove UART closure when
                    # its sole control thread ends. Staying alive here used to
                    # publish "ready" and acknowledge STOP without a UART.
                    exit_event.set()
                finally:
                    if self._closed.is_set() or exit_event.is_set():
                        with self._lock:
                            self._stop_locked(failure)
                            try: self._service_stop_locked()
                            except BaseException: pass
                        self._disconnect(failure or '运控连接已结束')
            def status(self):
                with self._lock:
                    result=super().status()
                    try: self._motion_gate_locked('hold' if self._mode=='off' else self._mode); reason=None
                    except Exception as error: reason=str(error)
                    result['start_blocked_reason']=reason
                    result['model_start_blocked_reasons']={}
                    for name in motion.MODEL_MODES:
                        try: self._model_gate_locked(name); reason=None
                        except Exception as error: reason=str(error)
                        result['model_start_blocked_reasons'][name]=reason
                    result['release_confirmed']=bool(self._connected and self._state and
                        self._state['online_mask']==motion.ALL_MASK and not self._state['fault_mask'] and not self._state['armed'] and
                        not self._state['torque_mask'] and not self._state['cookie'] and not self._pending_stop)
                    result['imu_ok']=bool(self._state and self._state['imu_ok'])
                    result['request_counts']=dict(request_counts)
                    diagnostic=getattr(self,'_last_stop_diagnostic',None)
                # Historical packets can be much larger than live feedback.
                # Never hold up STATE/TARGET/STOP while copying that history.
                compact=globals().get('native_compact_stop_diagnostic')
                if callable(compact):
                    # Native publication/START must not copy historical UART
                    # arrays while the control loop is running in this process.
                    result['last_stop_diagnostic']=compact(diagnostic)
                    if result['last_stop_diagnostic'] is not None:
                        result['last_stop_diagnostic']['details_deferred_until_UART_closed']=True
                else:
                    result['last_stop_diagnostic']=motion.copy.deepcopy(diagnostic)
                return result
        controller=OneShotController(sessions,port=serial_path(EXPECTED),config_path=Path(ram_directory)/'policy/calibration.json',transport_factory=ReviewedTransport)
        if not controller._calibrated_locked(): raise ProbeError('Saved calibration is incomplete')
        # One explicit open, STOP/CONFIG/STATE only at deployment. No ARM.
        controller.start()
        deadline=time.monotonic()+5
        while not controller.status()['connected']:
            if not controller._thread.is_alive(): raise ProbeError(controller.status()['error'] or 'Motion connection failed')
            if time.monotonic()>deadline: raise ProbeError('Motion connection deadline exceeded')
            time.sleep(.02)
        output('ready',host='X5',runtime='CPUExecutionProvider',ram_directory=ram_directory,
               calibration_sha256=DEPLOY['calibration_sha256'],model_hashes=DEPLOY['model_hashes'],hardware=controller.status(),request_counts=dict(request_counts),motor_start_attempted=False)
        def watcher():
            global health_error
            last_health=time.monotonic()
            while not exit_event.wait(.1):
                if time.monotonic()-last_health>=1:
                    try: ram_health()
                    except BaseException as error: health_error=str(error)
                    last_health=time.monotonic()
                output('status',hardware=controller.status())
        threading.Thread(target=watcher,daemon=True).start()
        input_buffer=bytearray()
        while not exit_event.is_set():
            if not select.select([sys.stdin.fileno()],[],[],.1)[0]: continue
            chunk=os.read(sys.stdin.fileno(),8192)
            if not chunk: break
            input_buffer.extend(chunk)
            if len(input_buffer)>16384: raise ProbeError('Motion command exceeds bound')
            while b'\n' in input_buffer:
                line,_,tail=input_buffer.partition(b'\n'); input_buffer=bytearray(tail)
                request=json.loads(line); rid=request.get('request_id'); action=request.get('action')
                try:
                    if action=='close': exit_event.set(); break
                    if action=='stop':
                        before = controller.status()
                        if not before['connected']:
                            raise ProbeError(before.get('error') or '运控串口已关闭，请重新连接并载入模型')
                        result=controller.stop(source=request.get('stop_source'))
                        if result.get('release_confirmed') is not True:
                            raise ProbeError(result.get('error') or '停止已发送，但未回读确认全部扭矩关闭')
                    else:
                        if health_error: raise ProbeError(health_error)
                        if type(request.get('generation')) is not int or request['generation']!=controller.status()['generation']: raise ProbeError('Motion state changed; refresh before starting')
                        if action=='heartbeat': result=controller.heartbeat(background=request.get('background') is True)
                        elif action=='start': result=controller.start_mode(request.get('mode'),support_confirmed=request.get('support_confirmed') is True,expected_generation=request['generation'],preview_model=request.get('preview_model','walk'),behavior_confirmed=request.get('behavior_confirmed') is True)
                        elif action=='command': result=controller.command(request.get('forward'),request.get('side'),request.get('yaw'))
                        else: raise ProbeError('Unknown motion command')
                    output('reply',request_id=rid,hardware=result,error=None)
                except Exception as error: output('reply',request_id=rid,hardware=controller.status(),error=str(error))
        if getattr(controller, '_terminal_error', None):
            raise ProbeError(controller._terminal_error)
        return 0
    except BaseException as error:
        output('fault',reason=str(error) or type(error).__name__,
               last_stop_diagnostic=None if controller is None else controller.status().get('last_stop_diagnostic'))
        return 20
    finally:
        exit_event.set()
        if controller is not None:
            release=controller.stop().get('release_confirmed') is True
            controller.close()
            serial_closed=controller._transport is None and not controller._connected and not controller._thread.is_alive() and all(t.serial is None for t in opened_transports)
            stopped=controller.status()
        else: serial_closed=True; stopped=None; release=False
        # Cleanup is limited to our exclusive tmpfs directory, after UART close.
        if ram_directory is not None and serial_closed:
            try: shutil.rmtree(ram_directory)
            except BaseException: pass
        output('closed',serial_closed=serial_closed,release_confirmed=release,hardware=stopped,
               serial_open_attempts=serial_open_attempts,request_counts=dict(request_counts),automatic_retry=False)
raise SystemExit(run_motion())
'''


def prepare_source(expected, profile, profile_sha, deployment):
    _, canonical = feedback_stream._source_expected(expected, profile, profile_sha)
    definitions = feedback_stream.runtime_source()
    functions = '\n\n'.join(inspect.getsource(f) for f in (
        archive_tools.safe_relative_path, archive_tools.inspect_tar, archive_tools.inspect_wheels))
    source = (definitions + '\nimport threading, hashlib\nfrom pathlib import PurePosixPath\n' + functions +
              '\nMAX_EXPANDED_WHEEL_BYTES = 167772160\nEXPECTED = ' + repr(canonical) +
              '\nDEPLOY = ' + repr(deployment) + '\n' + WORKER)
    compile(source, '<x5-motion-workbench>', 'exec')
    return canonical, source.encode()


def _uncertain_motion_evidence(source_directory, expected, config):
    """Read one private, exited deployment; this grants no UART or motor proof."""
    source = feedback_connection_check._private_directory(source_directory)
    if not re.fullmatch(r'motion-[0-9a-f]{16}', source.name):
        raise ValueError('恢复运控必须指定完整的私有 motion 记录目录')
    for name in ('closed.json', 'disconnect-reconciled.json'):
        try:
            (source / name).lstat()
        except FileNotFoundError:
            continue
        raise ValueError('旧运控已有关闭或释放记录，不能恢复为未确认占用')
    limits = {'ready.json': 65536, 'transfer-result.json': 3 * feedback_stream.MAX_KERNEL_BYTES,
              'live-state.json': 262144, 'transport-exit.json': 16384, 'deployment.json': 65536}
    raw = {name: maintenance_transport.private_read(source / name, limit)
           for name, limit in limits.items()}
    values = {name: maintenance_transport.strict_json(data) for name, data in raw.items()}
    if any(type(value) is not dict for value in values.values()):
        raise ValueError('旧运控恢复记录格式不完整')
    ready, transfer, live, exited, deployment = (values[name] for name in limits)
    generation = ready.get('deployment_generation')
    ram = ready.get('ram_directory')
    if (set(ready) != {'kind', 'deployment_generation', 'host', 'runtime', 'ram_directory',
                      'calibration_sha256', 'model_hashes', 'hardware', 'request_counts', 'motor_start_attempted'}
            or ready['kind'] != 'ready' or type(generation) is not int or not 0 < generation < 0x80000000
            or ready['host'] != 'X5' or ready['runtime'] != 'CPUExecutionProvider'
            or type(ram) is not str or re.fullmatch(r'/dev/shm/xgoduck-motion-[A-Za-z0-9_-]+', ram) is None
            or ready['motor_start_attempted'] is not False
            or type(ready['hardware']) is not dict or ready['hardware'].get('connected') is not True
            or type(ready['hardware'].get('generation')) is not int or ready['hardware']['generation'] <= 0
            or type(ready['request_counts']) is not dict
            or any(type(k) is not str or not k.isdecimal() or type(v) is not int or v < 0
                   for k, v in ready['request_counts'].items())):
        raise ValueError('旧运控 ready 身份或代次不匹配')
    if (set(transfer) != {'kind', 'deployment_generation', 'archive_bytes', 'identity', 'ram_directory'}
            or transfer['kind'] != 'transfer_ready' or type(transfer['deployment_generation']) is not int
            or transfer['deployment_generation'] != generation or transfer['ram_directory'] != ram):
        raise ValueError('旧运控传输记录身份或代次不匹配')
    identity = transfer['identity']
    if type(identity) is not dict or set(identity) != feedback_connection_check.IDENTITY_FIELDS:
        raise ValueError('旧运控板卡身份记录不完整')
    for key in ('boot_id', 'cid', 'root_mount_line', 'model'):
        if type(identity[key]) is not type(expected[key]) or identity[key] != expected[key]:
            raise ValueError('旧运控板卡身份不匹配：' + key)
    if not feedback_stream.serial_receipt_matches(identity['serial'], expected):
        raise ValueError('旧运控板卡身份不匹配：serial')
    if (type(identity['kernel_log']) is not str
            or len(identity['kernel_log'].encode()) > feedback_stream.MAX_KERNEL_BYTES
            or type(identity['swaps']) is not str or len(identity['swaps'].encode()) > 16384):
        raise ValueError('旧运控系统记录超过限制')
    # Only pure definitions are evaluated locally; no metadata collector runs.
    namespace = {}
    exec(compile(feedback_stream.runtime_source(), '<restored-motion-validators>', 'exec'), namespace)
    namespace['validate_os'](identity, expected)
    kernel = namespace['validate_kernel'](identity['kernel_log'], expected)
    for key, actual in kernel.items():
        if type(identity[key]) is not type(actual) or identity[key] != actual:
            raise ValueError('旧运控内核记录不匹配：' + key)
    calibration = calibration_bytes(config)
    calibration_sha = hashlib.sha256(calibration).hexdigest()
    if (set(deployment) != {'archive_sha256', 'archive_size', 'scope', 'automatic_motor_start',
                           'config_mount_review', 'persistent_model_writes', 'member_bytes', 'wheel_bytes',
                           'calibration_sha256', 'model_hashes', 'files', 'started_monotonic'}
            or deployment['scope'] != 'explicit_web_motion' or deployment['automatic_motor_start'] is not False
            or deployment['persistent_model_writes'] is not False
            or deployment['config_mount_review'] != config_mount_for_boot(expected['boot_id'])
            or deployment['model_hashes'] != MODEL_HASHES or ready['model_hashes'] != MODEL_HASHES
            or deployment['calibration_sha256'] != calibration_sha or ready['calibration_sha256'] != calibration_sha
            or type(transfer['archive_bytes']) is not int
            or any(type(deployment[key]) is not int or deployment[key] <= 0
                   for key in ('archive_size', 'member_bytes', 'wheel_bytes'))
            or transfer['archive_bytes'] != deployment['archive_size']
            or type(deployment['archive_sha256']) is not str
            or re.fullmatch(r'[0-9a-f]{64}', deployment['archive_sha256']) is None
            or type(deployment['started_monotonic']) not in (int, float)
            or not math.isfinite(deployment['started_monotonic']) or deployment['started_monotonic'] < 0):
        raise ValueError('旧运控部署的模型、校准或存储范围不匹配')
    files = deployment['files']
    if (type(files) is not dict or not files
            or any(type(name) is not str or type(row) is not dict or set(row) != {'size_bytes', 'sha256'}
                   or type(row['size_bytes']) is not int or row['size_bytes'] <= 0
                   or type(row['sha256']) is not str or re.fullmatch(r'[0-9a-f]{64}', row['sha256']) is None
                   for name, row in files.items())
            or files.get('policy/calibration.json') != dict(size_bytes=len(calibration), sha256=calibration_sha)
            or any(files.get(f'policy/models/xgoduck_{name}.onnx', {}).get('sha256') != digest
                   for name, digest in MODEL_HASHES.items())):
        raise ValueError('旧运控部署成员摘要不匹配')
    exit_fields = {'process_exited', 'ssh_exit_code', 'stderr', 'original_error', 'reader_error',
                   'closed_record_received', 'remote_closure_inferred', 'automatic_retry'}
    if (not exit_fields <= set(exited) or set(exited) - exit_fields - {'cleanup_error'}
            or exited['process_exited'] is not True or type(exited['ssh_exit_code']) is not int
            or not -127 <= exited['ssh_exit_code'] <= 255
            or exited['closed_record_received'] is not False or exited['remote_closure_inferred'] is not False
            or exited['automatic_retry'] is not False or type(exited['stderr']) is not str
            or len(exited['stderr'].encode()) > 16384
            or type(exited['original_error']) is not str or not 0 < len(exited['original_error']) <= 16384
            or exited['reader_error'] is not None and (type(exited['reader_error']) is not str
                                                     or len(exited['reader_error']) > 16384)
            or 'cleanup_error' in exited and (type(exited['cleanup_error']) is not str
                                             or len(exited['cleanup_error']) > 16384)):
        raise ValueError('旧运控缺少严格的 SSH 实际退出记录')
    if (set(live) != {'recorded_monotonic', 'phase', 'error', 'host', 'ram_only', 'owns_connection',
                     'feedback_age_ms', 'hardware', 'deployment', 'closed_proof', 'last_operation',
                     'transport_exit', 'exit_reconciled'}
            or live['phase'] != 'fault' or live['owns_connection'] is not True or live['host'] != 'X5'
            or live['ram_only'] is not True or live['exit_reconciled'] is not False
            or live['closed_proof'] is not None or live['hardware'] is not None or live['feedback_age_ms'] is not None
            or live['error'] != exited['original_error']
            or json.dumps(live['transport_exit'], sort_keys=True) != json.dumps(exited, sort_keys=True)
            or json.dumps(live['deployment'], sort_keys=True) != json.dumps(deployment, sort_keys=True)
            or type(live['recorded_monotonic']) not in (int, float)
            or not math.isfinite(live['recorded_monotonic'])
            or live['recorded_monotonic'] < deployment['started_monotonic']):
        raise ValueError('旧运控未确认占用状态与部署或退出记录不匹配')
    return source, raw, values, generation


def _source_motion_review(source_directory):
    """Bind the old receipt to its own fixed parent review, across service boots."""
    source = feedback_connection_check._private_directory(source_directory)
    parent = feedback_connection_check._private_directory(source.parent)
    raw_expected = maintenance_transport.private_read(parent / 'expectations.json', 2 * 1024 * 1024)
    raw_record = maintenance_transport.private_read(parent / 'reviewed-baseline-record.json', 16384)
    raw_copy = maintenance_transport.private_read(parent / 'reviewed-baseline.json', feedback_stream.MAX_PROFILE_BYTES)
    expected, record = (maintenance_transport.strict_json(raw) for raw in (raw_expected, raw_record))
    fields = {'source_path', 'sha256', 'copy_sha256', 'applied_fields', 'boot_generation_changed',
              'original_boot_id', 'reviewed_boot_id', 'board_identity_changed', 'calibration_changed'}
    if (type(expected) is not dict or type(record) is not dict or set(record) != fields
            or type(record['source_path']) is not str or type(record['sha256']) is not str
            or re.fullmatch(r'[0-9a-f]{64}', record['sha256']) is None
            or record['copy_sha256'] != record['sha256']
            or record['board_identity_changed'] is not False or record['calibration_changed'] is not False
            or record['original_boot_id'] != feedback_stream.REVIEWED_BOOT_ID
            or record['reviewed_boot_id'] != expected.get('boot_id')
            or type(record['boot_generation_changed']) is not bool
            or record['boot_generation_changed'] != (record['original_boot_id'] != record['reviewed_boot_id'])
            or record['applied_fields'] != ['kernel_prefix', 'historical_faults'] + (
                ['boot_id'] if record['boot_generation_changed'] else [])):
        raise ValueError('旧运控父运行的固定基线来源记录不匹配')
    profile_path = Path(record['source_path'])
    if (not profile_path.is_absolute() or profile_path.parent.resolve() != feedback_stream.EVIDENCE_DIR
            or profile_path.is_symlink()):
        raise ValueError('旧运控基线必须来自固定恢复证据目录')
    raw_profile = maintenance_transport.private_read(profile_path, feedback_stream.MAX_PROFILE_BYTES)
    if raw_copy != raw_profile or hashlib.sha256(raw_copy).hexdigest() != record['sha256']:
        raise ValueError('旧运控基线副本或来源摘要不匹配')
    # The loader and source preparation both accept only reviewed fixed variants.
    feedback_stream.load_reviewed_profile(profile_path, record['sha256'])
    _, reviewed_expected = feedback_stream._source_expected(expected, profile_path, record['sha256'])
    return reviewed_expected, dict(source_parent=str(parent), source_profile_path=str(profile_path),
                                  source_profile_sha256=record['sha256'],
                                  source_expectations_sha256=hashlib.sha256(raw_expected).hexdigest(),
                                  source_review_record_sha256=hashlib.sha256(raw_record).hexdigest())


class MotionWorkbench:
    def __init__(self, expected, profile, profile_sha, known_hosts, directory, *, transport_config=None):
        self.expected, self.profile, self.profile_sha = expected, profile, profile_sha
        self.known_hosts, self.directory = known_hosts, directory
        self.transport_config = resolve_transport(transport_config)
        self.lock = threading.RLock()
        self.io_lock = threading.Lock()
        self.phase, self.error = 'idle', None
        self.channel, self.hardware, self.deployment = None, None, None
        self.updated_at = None
        self.pending = {}
        self.closed_report = None
        self.attempt = None
        self.remote_generation = None
        self.uncertain_owner = False
        self.last_persisted = 0.
        self.last_operation = None
        self.control_owner = None
        self.transport_exit = None
        self._restored_exit = None
        self.exit_reconciled = False
        self.reader_done = threading.Event()
        self.reader_done.set()

    def _owns_connection(self):
        return (self.uncertain_owner or self.channel is not None and not self.exit_reconciled and
                (self.closed_report is None or self.closed_report.get('ssh_exit_code') is None))

    def _pipe_exited(self):
        process = getattr(self.channel, 'process', None)
        if process is not None:
            return process.poll() is not None
        receipt = self._restored_exit
        return (self.channel is None and receipt is not None and self.uncertain_owner
                and not self.exit_reconciled and self.closed_report is None
                and type(self.remote_generation) is int
                and self.remote_generation == receipt['generation']
                and json.dumps(self.transport_exit, sort_keys=True) == json.dumps(receipt['transport_exit'], sort_keys=True))

    def restore_uncertain_motion(self, source_directory, config):
        """Restore only an ownership hold; explicit metadata disconnect clears it."""
        with self.io_lock, self.lock:
            if (self.phase != 'idle' or self.channel is not None or self.attempt is not None
                    or self._owns_connection() or not self.reader_done.is_set()):
                raise ValueError('运控恢复只能在未打开连接的服务初始化时执行')
            local_expected = copy.deepcopy(self.expected)
            local_expected.pop('reviewed_reconnect_proof', None)
            # Profile/source validation is local only, as at normal deployment.
            _, reviewed_expected = feedback_stream._source_expected(local_expected, self.profile, self.profile_sha)
            old_expected, old_review = _source_motion_review(source_directory)
            for key in ('cid', 'root_mount_line', 'model', 'serial'):
                if old_expected[key] != reviewed_expected[key]:
                    raise ValueError('旧运控与当前审核基线的板卡身份不匹配：' + key)
            source, raw, values, generation = _uncertain_motion_evidence(source_directory, old_expected, config)
            if not feedback_stream.serial_receipt_matches(values['transfer-result.json']['identity']['serial'], reviewed_expected):
                raise ValueError('旧运控串口身份与当前审核规则不匹配')
            parent = feedback_connection_check._private_directory(self.directory)
            attempt = parent / ('motion-' + secrets.token_hex(8))
            attempt.mkdir(mode=0o700)
            for name, data in raw.items():
                maintenance_transport.private_new(attempt / name, data)
            provenance = dict(source_directory=str(source), deployment_generation=generation,
                              source_sha256={name: hashlib.sha256(data).hexdigest() for name, data in raw.items()},
                              old_review=old_review, old_boot_id=old_expected['boot_id'],
                              current_reviewed_boot_id=reviewed_expected['boot_id'],
                              current_profile_sha256=self.profile_sha,
                              restored_uncertain_owner=True, serial_open_attempts=0,
                              motor_stop_confirmed=False, old_remote_closure_inferred=False, automatic_retry=False)
            maintenance_transport.private_new(attempt / 'resume-source.json',
                                              json.dumps(provenance, ensure_ascii=False, allow_nan=False) + '\n')
            self.attempt, self.remote_generation = attempt, generation
            self.deployment = copy.deepcopy(values['deployment.json'])
            self.transport_exit = copy.deepcopy(values['transport-exit.json'])
            self._restored_exit = dict(generation=generation, transport_exit=copy.deepcopy(self.transport_exit))
            self.phase, self.error = 'fault', self.transport_exit['original_error']
            self.uncertain_owner, self.exit_reconciled = True, False
            self.hardware, self.updated_at, self.closed_report, self.control_owner = None, None, None, None
            self.reader_done.set()
            return self.snapshot()

    def record(self, name, value, replace=False):
        if self.attempt is None:
            return
        raw = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        target = self.attempt / name
        if replace:
            temp = self.attempt / ('.state-' + secrets.token_hex(8))
            maintenance_transport.private_new(temp, raw)
            os.replace(temp, target)
        else:
            maintenance_transport.private_new(target, raw)

    def snapshot(self):
        with self.lock:
            age = None if self.updated_at is None else (time.monotonic()-self.updated_at)*1000
            return dict(phase=self.phase,error=self.error,host='X5',ram_only=True,
                        owns_connection=self._owns_connection(),
                        feedback_age_ms=age,hardware=copy.deepcopy(self.hardware) if age is not None and age<500 else None,
                        deployment=copy.deepcopy(self.deployment),closed_proof=copy.deepcopy(self.closed_report),
                        last_operation=copy.deepcopy(self.last_operation),
                        transport_exit=copy.deepcopy(self.transport_exit),
                        exit_reconciled=self.exit_reconciled)

    def begin_connect(self, config, before_connect):
        with self.lock:
            if self.phase=='deploying': raise ValueError('运控正在部署，请等待')
            if self._owns_connection() or not self.reader_done.is_set():
                raise ValueError('请先确认当前运控连接已关闭')
            self.phase,self.error='deploying',None
            self.channel,self.closed_report,self.hardware,self.deployment=None,None,None,None
            self.updated_at,self.remote_generation=None,None
            self.last_operation=None
            self.control_owner=None
            self.transport_exit=None
            self._restored_exit=None
            self.exit_reconciled=False
        def deploy():
            try:
                before_connect()
                bundle,deployment=prepare_bundle(config, boot_id=self.expected['boot_id'])
                deployment['started_monotonic']=time.monotonic()
                expected=copy.deepcopy(self.expected); expected['generation']=secrets.randbelow(0x7ffffffe)+1
                prepared,source=prepare_source(expected,self.profile,self.profile_sha,deployment)
                attempt=Path(self.directory)/('motion-'+secrets.token_hex(8)); attempt.mkdir(mode=0o700)
                self.attempt = attempt
                maintenance_transport.private_new(attempt/'deployment.json',json.dumps(deployment,indent=2)+'\n')
                maintenance_transport.private_new(attempt/'remote-source.py',source)
                helper=maintenance_transport.helpers()
                if helper['known_hosts_fingerprint'](Path(self.known_hosts).read_text())!=maintenance_transport.FINGERPRINT: raise ValueError('X5 SSH 身份变化')
                self.uncertain_owner = True
                channel=maintenance_transport._ProcessChannel(source,attempt,Path(self.known_hosts),helper,
                    transport_config=self.transport_config)
                with self.lock:
                    self.channel,self.deployment,self.attempt=channel,deployment,attempt
                    self.remote_generation = prepared['generation']
                    self.closed_report=None
                ready=channel.receive(time.monotonic()+15)
                self.record('transfer-result.json',ready)
                if ready.get('kind')!='transfer_ready' or ready.get('archive_bytes')!=len(bundle): raise ValueError(ready.get('reason') or 'X5 未确认部署传输')
                helper['write_pipe_once'](channel.process.stdin.fileno(),bundle,120)
                ready=channel.receive(time.monotonic()+40)
                self.record('model-ready-result.json',ready)
                if ready.get('kind')!='ready' or ready.get('calibration_sha256')!=deployment['calibration_sha256'] or ready.get('model_hashes')!=MODEL_HASHES: raise ValueError(ready.get('reason') or '模型部署未获确认')
                maintenance_transport.private_new(attempt/'ready.json',json.dumps(ready,ensure_ascii=False)+'\n')
                with self.lock:
                    self.hardware,self.updated_at=ready['hardware'],time.monotonic()
                    self.phase='ready'
                    self.reader_done.clear()
                threading.Thread(target=self._read,name='x5-motion-rpc',daemon=True).start()
            except BaseException as error:
                self.reader_done.set()
                with self.lock: self.error=str(error); self.phase='fault'
                self.record('failure.json',dict(error=str(error),motor_start_attempted=False))
                print(json.dumps(dict(motion_deployment='failed',error=str(error)),ensure_ascii=False),flush=True)
                if isinstance(error, maintenance_transport.MaintenancePreflightError): self.uncertain_owner=False
                if self.channel is not None:
                    try:
                        deadline=time.monotonic()+2
                        while time.monotonic()<deadline:
                            value=self.channel.receive(deadline)
                            if value.get('kind')=='closed' and value.get('deployment_generation')==self.remote_generation:
                                self.closed_report=value
                                break
                    except BaseException: pass
                    observed=self.channel.finish()
                    self.transport_exit=dict(observed or {},original_error=self.error,
                        closed_record_received=self.closed_report is not None,
                        remote_closure_inferred=False,automatic_retry=False)
                    self.record('transport-exit.json',self.transport_exit,replace=True)
                    if self.closed_report is not None:
                        self.closed_report['ssh_exit_code']=self.channel.process.poll()
                        self.uncertain_owner = self.closed_report.get('serial_closed') is not True
                        self.record('closed.json',self.closed_report)
        threading.Thread(target=deploy,name='x5-motion-deploy',daemon=True).start()

    def _read(self):
        channel = self.channel
        reader_error = None
        last_stop_sequence = 0
        stop_verification_failure_recorded = False
        try:
            while True:
                value=channel.receive(time.monotonic()+5)
                with self.lock:
                    if value.get('deployment_generation') != self.remote_generation:
                        raise ValueError('运控消息的部署代次不匹配')
                    if value.get('hardware') is not None:
                        previous=(self.hardware or {}).get('generation',-1)
                        if value['hardware'].get('generation',-1)>=previous:
                            self.hardware,self.updated_at=value['hardware'],time.monotonic()
                    diagnostic=value.get('last_stop_diagnostic') or (value.get('hardware') or {}).get('last_stop_diagnostic')
                    if isinstance(diagnostic,dict) and type(diagnostic.get('sequence')) is int:
                        sequence=diagnostic['sequence']
                        failed='verification_failure' in diagnostic
                        if sequence>last_stop_sequence or sequence==last_stop_sequence and sequence>0 and failed and not stop_verification_failure_recorded:
                            self.record('last-stop-diagnostic.json',diagnostic,replace=True)
                            last_stop_sequence,stop_verification_failure_recorded=sequence,failed
                    if value['kind']=='reply':
                        pending=self.pending.get(value.get('request_id'))
                        if pending: pending['value']=value; pending['event'].set()
                    elif value['kind']=='fault':
                        self.error=self.error or value.get('reason'); self.phase='fault'
                        self.record('remote-fault.json',value,replace=True)
                    elif value['kind']=='closed':
                        self.closed_report=value; self.phase='closed'
                        self.uncertain_owner=value.get('serial_closed') is not True
                        self.record('live-state.json',dict(recorded_monotonic=time.monotonic(),**self.snapshot()),replace=True)
                        break
                    if time.monotonic()-self.last_persisted>=1:
                        self.record('live-state.json',dict(recorded_monotonic=time.monotonic(),**self.snapshot()),replace=True)
                        self.last_persisted=time.monotonic()
            channel.process.wait(timeout=2)
            with self.lock: self.closed_report['ssh_exit_code']=channel.process.returncode
            self.record('live-state.json',dict(recorded_monotonic=time.monotonic(),**self.snapshot()),replace=True)
            maintenance_transport.private_new(self.attempt/'closed.json',json.dumps(self.closed_report,ensure_ascii=False)+'\n')
        except BaseException as error:
            reader_error = str(error) or type(error).__name__
            with self.lock:
                self.phase='fault'; self.error=self.error or str(error) or type(error).__name__
                self.hardware,self.updated_at=None,None
                self.uncertain_owner = self.closed_report is None or self.closed_report.get('serial_closed') is not True
        finally:
            # Always retain the actual SSH exit and its bounded stderr. A local
            # EOF is not a remote UART CLOSED record and cannot release ownership.
            try:
                try:
                    observed=channel.finish()
                except BaseException as error:
                    process=getattr(channel,'process',None)
                    observed=dict(process_exited=process is not None and process.poll() is not None,
                        ssh_exit_code=None if process is None else process.poll(),
                        stderr=bytes(getattr(channel,'stderr',b'')).decode('utf-8','replace')[-4096:],
                        cleanup_error=str(error))
                with self.lock:
                    self.transport_exit=dict(observed or {}, original_error=self.error,
                        reader_error=reader_error,
                        closed_record_received=self.closed_report is not None,
                        remote_closure_inferred=False, automatic_retry=False)
                    self.record('transport-exit.json',self.transport_exit,replace=True)
                    for pending in self.pending.values():
                        if not pending['event'].is_set():
                            pending['value']=dict(error=self.error or '运控连接已结束',hardware=None)
                            pending['event'].set()
                    self.record('live-state.json',dict(recorded_monotonic=time.monotonic(),**self.snapshot()),replace=True)
            finally:
                self.reader_done.set()

    def _reconcile_exited_connection(self):
        """One explicit disconnect checks metadata; never reconstruct CLOSED."""
        if not self._pipe_exited() or not self.reader_done.wait(2):
            raise ValueError('后台连接仍在关闭，尚不能重新检查；请等待关闭结果')
        directory=Path(self.directory)/('motion-disconnect-check-'+secrets.token_hex(8))
        directory.mkdir(mode=0o700)
        expected=copy.deepcopy(self.expected)
        expected['generation']=self.remote_generation
        try:
            proof=feedback_connection_check.check_connection(expected,Path(self.known_hosts),directory,
                reviewed_profile_path=self.profile,reviewed_profile_sha256=self.profile_sha,
                transport_config=self.transport_config)
            feedback_connection_check.validate_proof(proof, feedback_connection_check.prepare_source(
                expected,self.profile,self.profile_sha)[0])
            with self.lock:
                if not self._pipe_exited(): raise ValueError('原 SSH 进程退出证明已变化')
                self.record('disconnect-reconciled.json',dict(
                    proof_directory=str(directory),generation=self.remote_generation,
                    old_closed_record_received=self.closed_report is not None,
                    old_remote_closure_inferred=False,serial_open_attempts=0,
                    motor_stop_confirmed=False,automatic_retry=False),replace=True)
                self.exit_reconciled=True
                self.uncertain_owner=False
                self.phase='closed'
                self.hardware,self.updated_at=None,None
                return self.snapshot()
        except BaseException as error:
            maintenance_transport.private_new(directory/'reconciliation-failed.json',
                json.dumps(dict(error=str(error),serial_open_attempts=0,automatic_retry=False))+'\n')
            raise ValueError('连接检查未通过，原退出原因保留：'+str(error)) from error

    def _validate_control_locked(self, body):
        action=body.get('action')
        client=body.get('control_id')
        valid=type(client) is str and len(client)==32 and all(c in '0123456789abcdef' for c in client)
        if action=='start':
            if not valid: raise ValueError('运控页面版本已更新，请刷新后再启动')
            return
        if action=='stop' and body.get('stop_source')=='button': return
        if action not in ('heartbeat','command','stop'): return
        if action=='stop' and body.get('stop_source')=='window-blur':
            raise ValueError('窗口失焦不再停止运动；请刷新旧运控页面')
        owner=self.control_owner
        if (not valid or owner is None or client!=owner['id'] or
                body.get('generation')!=owner['generation'] or
                (self.hardware or {}).get('generation')!=owner['generation']):
            raise ValueError('该页面没有启动当前动作；旧页面请求未执行，请刷新或返回启动动作的页面')
        if action=='stop' and (self.hardware or {}).get('mode')=='hold' and (
                body.get('stop_source')=='page-hidden' or (self.hardware or {}).get('hold_control_protocol')==2):
            raise ValueError('固定站姿由 X5 独立保持；请使用停止按钮结束站姿')

    def action(self, body):
        action=body.get('action')
        if action not in ('start','command','heartbeat','stop','disconnect'): raise ValueError('未知运控操作')
        if 'stop_source' in body and (action!='stop' or body['stop_source'] not in STOP_SOURCES):
            raise ValueError('运控停止来源无效')
        if 'background' in body and (action!='heartbeat' or type(body['background']) is not bool):
            raise ValueError('运控后台心跳参数无效')
        with self.io_lock:
            if self._pipe_exited():
                if action=='disconnect':
                    return self._reconcile_exited_connection()
                raise ValueError('运控后台连接已结束，指令未发送。请点击「检查断开并释放连接」；原原因：'+
                                 (self.error or '后台进程已退出，尚未取得完整原因'))
            with self.lock:
                if self.phase!='ready' and action not in ('disconnect','stop'): raise ValueError(self.error or '运控尚未连接')
                if self.channel is None or self.closed_report is not None: raise ValueError('运控连接已关闭')
                self._validate_control_locked(body)
                rid=secrets.token_hex(12); pending=dict(event=threading.Event(),value=None); self.pending[rid]=pending
                audited=action in ('start','stop','disconnect')
                if audited:
                    self.last_operation=dict(request_id=rid,action=action,mode=body.get('mode'),
                        control_id=body.get('control_id'),
                        stop_source=body.get('stop_source'),
                        generation=body.get('generation'),support_confirmed=body.get('support_confirmed') is True,
                        behavior_confirmed=body.get('behavior_confirmed') is True,preview_model=body.get('preview_model','walk'),
                        state='pending',error=None,recorded_monotonic=time.monotonic(),
                        before=copy.deepcopy(self.hardware))
                    self.record('operation-'+rid+'.json',self.last_operation)
            request=dict(body,request_id=rid)
            if action=='disconnect': request['action']='close'
            try:
                self.channel.send(request,1.)
                if action=='disconnect':
                    deadline=time.monotonic()+4
                    while time.monotonic()<deadline:
                        with self.lock:
                            if self.closed_report is not None and self.closed_report.get('ssh_exit_code') is not None:
                                self.last_operation.update(state='complete',after=copy.deepcopy(self.closed_report))
                                self.record('operation-'+rid+'.json',self.last_operation,replace=True)
                                return self.snapshot()
                        time.sleep(.02)
                    raise ValueError('运控断开未收到串口关闭证明')
                if not pending['event'].wait(2): raise ValueError('运控应答超时，指令不会重发；控制器将按超时停止')
                result=pending['value']
                if action=='start' and not result.get('error'):
                    hardware=result.get('hardware') or {}
                    if hardware.get('connected') and hardware.get('mode')==body.get('mode'):
                        with self.lock:
                            self.control_owner=dict(id=body['control_id'],generation=hardware['generation'])
                elif action=='stop' and not result.get('error'):
                    with self.lock: self.control_owner=None
                if audited:
                    with self.lock:
                        self.last_operation.update(state='failed' if result.get('error') else 'complete',
                            error=result.get('error'),after=copy.deepcopy(result.get('hardware')))
                        self.record('operation-'+rid+'.json',self.last_operation,replace=True)
                if result.get('error'): raise ValueError(result['error'])
                return self.snapshot()
            except Exception as error:
                if audited:
                    with self.lock:
                        self.last_operation.update(state='failed',error=str(error))
                        self.record('operation-'+rid+'.json',self.last_operation,replace=True)
                if isinstance(error,(OSError,EOFError)):
                    with self.lock:
                        self.phase='fault'
                        self.error=self.error or str(error)
                        self.hardware,self.updated_at=None,None
                    raise ValueError('运控连接管道已结束，指令不会重发。请检查断开并重新连接；原原因：'+
                                     self.error) from error
                raise
            finally:
                with self.lock: self.pending.pop(rid,None)

    def close(self):
        if self.channel is not None and self.closed_report is None:
            if self._pipe_exited():
                # Service shutdown may reap its own child, but never launches
                # a new metadata SSH check on the operator's behalf.
                self.channel.finish()
                return
            try: self.action(dict(action='disconnect'))
            except Exception:
                self.channel.finish()
