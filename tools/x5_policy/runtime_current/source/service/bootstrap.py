
import hashlib, json, os, stat, sys, time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def load(name):
    def pairs(items):
        result={}
        for key,value in items:
            if key in result: raise ValueError('Duplicate native JSON field')
            result[key]=value
        return result
    return json.loads((root/name).read_text(),object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite native JSON')))
pin=load('native-pin.json')
for name,info in pin['files'].items():
    data=(root/name).read_bytes()
    if len(data)!=info['size_bytes'] or hashlib.sha256(data).hexdigest()!=info['sha256']:
        raise ValueError('Native bundle member changed: '+name)
deployment=load('service/deployment.json')
expected=load('service/expected.json')
sys.path[:0]=[str(root/'site'),str(root/'policy'),str(root/'service')]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',
    TMPDIR=str(root),XDG_CACHE_HOME=str(root),PATH='/usr/sbin:/usr/bin:/sbin:/bin')
# Metadata checks have a 200 ms absolute inventory budget. Frequent status
# requests must yield promptly when the guard thread resumes after filesystem
# calls; the default 5 ms GIL interval can accumulate across these calls.
# This changes scheduling fairness, never a check deadline or its result.
sys.setswitchinterval(.001)
import native_calibration_store as calibration_storage,native_transports,native_process,policy_worker
uart=None
uart_identity=None
immutable_files={str(root/name) for name in pin['files'] if name!='policy/calibration.json'}
def audit(event,args):
    if event=='open' and args:
        path=args[0];mode=args[1] if len(args)>1 else None;flags=args[2] if len(args)>2 else 0
        if isinstance(path,(str,bytes)):
            path=os.path.abspath(os.fsdecode(path));resolved=os.path.realpath(path)
            if resolved.startswith('/dev/mmcblk') or path=='/dev/root':
                raise PermissionError('Native service cannot open storage block devices')
            try:
                if stat.S_ISBLK(os.stat(path).st_mode): raise PermissionError('Native block-device open denied')
            except FileNotFoundError: pass
            if (path.startswith(('/dev/tty','/dev/serial','/dev/bus/usb')) or
                    resolved.startswith(('/dev/tty','/dev/serial','/dev/bus/usb'))) and path!=uart:
                raise PermissionError('Unreviewed native device')
            if path==uart:
                node=os.stat(path,follow_symlinks=False)
                if (not stat.S_ISCHR(node.st_mode) or resolved!=path or
                        (node.st_dev,node.st_ino,node.st_rdev)!=uart_identity):
                    raise PermissionError('Explicit native UART identity changed')
            writes=(isinstance(mode,str) and any(c in mode for c in 'wax+')) or (
                isinstance(flags,int) and flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
            if writes and (path in immutable_files or resolved.startswith(str(root/'site')+'/')):
                raise PermissionError('Native bundle code, dependencies and models are immutable')
            if writes and path!=uart and not (resolved==path and resolved.startswith(str(root)+'/')):
                if not calibration_storage.allows_persistent_mutation(path,'write'):
                    raise PermissionError('Native writes require RAM or an explicit private calibration save')
    elif event=='subprocess.Popen':
        if len(args)<2 or (len(args)>2 and args[2] is not None) or (len(args)>3 and args[3] is not None):
            raise PermissionError('Native child environment or working directory differs')
        if not (args[0]=='dmesg' and list(args[1])==['dmesg','--color=never']) and not (
                native_transports.allowed_child_command(args[0],args[1])) and not (
                native_process.allowed_web_command(args[0],args[1],root)) and not (
                policy_worker.allowed_worker_command(args[0],args[1],root)):
            raise PermissionError('Native service may spawn only its reviewed bounded local child')
    elif event in ('os.mkdir','os.remove','os.rename','os.rmdir','os.chmod','os.chown','os.truncate','os.utime'):
        operation={'os.mkdir':'mkdir','os.remove':'remove','os.rename':'rename'}.get(event)
        paths=args[:2] if event=='os.rename' else args[:1]
        for value in paths:
            if not isinstance(value,(str,bytes)):
                raise PermissionError('Native mutation requires an exact path')
            path=os.path.abspath(os.fsdecode(value));resolved=os.path.realpath(path)
            if path in immutable_files or resolved.startswith(str(root/'site')+'/'):
                raise PermissionError('Native immutable bundle mutation denied')
            if not (path==resolved and resolved.startswith(str(root)+'/')) and not (
                    operation and calibration_storage.allows_persistent_mutation(path,operation)):
                raise PermissionError('Native path mutation is outside its authorized calibration/RAM scope')
        if event=='os.rename' and len(args)>2 and any(fd not in (None,-1) for fd in args[2:]):
            raise PermissionError('Native mutation through directory descriptors denied')
    elif event in ('os.link','os.symlink'):
        raise PermissionError('Native link creation denied')
    elif event in ('os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):
        raise PermissionError('Native external process launch denied')
sys.addaudithook(audit)
guard_source=(root/'service/guard-runtime.py').read_text()
guard_code=compile(guard_source,'<native-guards>','exec')
# Parse/compile immutable helper definitions before model-loaded HTTP threads.
# Every later inventory is still fresh and retains its original .2 deadline.
native_transports.prepare_holder_runtime(guard_source)
calibration_reload_fault=None
seed_calibration=load('policy/calibration.json')
seed_provenance=load('service/joint-zero-provenance.json')
if deployment.get('persistent_calibration_directory')!='/var/lib/xgoduck-workbench' or (
        deployment.get('source_calibration_sha256')!=deployment['calibration_sha256']):
    raise ValueError('Native persistent calibration source or directory differs')
store=calibration_storage.NativeCalibrationStore('/var/lib/xgoduck-workbench',seed_calibration,
    seed_provenance,source_sha256=deployment['source_calibration_sha256'])
loaded_calibration=store.load()
authority=calibration_storage.RAMCalibrationAuthority(root,deployment['calibration_sha256'])
if loaded_calibration['persistent']:
    deployment['calibration_sha256']=authority.publish(calibration_storage.calibration_bytes(loaded_calibration['calibration']))
calibration=loaded_calibration['calibration']
zero_provenance=loaded_calibration['zero_provenance']
def publish_calibration(raw):
    if calibration_reload_fault is not None: raise ValueError(calibration_reload_fault)
    value=authority.publish(raw)
    deployment['calibration_sha256']=value
    return value
initial_connection_gate_pending=True
last_completed_ram_health=None
native_usb_recovery=None
def connection_guard_factory(initial=False, usb_recovery=False):
    global uart,uart_identity,initial_connection_gate_pending,last_completed_ram_health,native_usb_recovery
    # Each explicit connection gets independent function globals and serial
    # binding cache. Never clear the cache underneath a running controller.
    namespace={}
    exec(guard_code,namespace)
    # The reviewed scan releases the GIL on every proc entry. Run that same
    # bounded scan outside this threaded, model-loaded parent; do not renew its
    # caller's absolute deadline or change scans inside generated child source.
    namespace['_holders']=lambda rdev,deadline: native_transports.isolated_holders(
        guard_source,rdev,deadline,root/'holder-evidence')
    if calibration_reload_fault is not None: raise namespace['ProbeError'](calibration_reload_fault)
    connection_expected=json.loads(json.dumps(expected,allow_nan=False))
    namespace['EXPECTED']=connection_expected
    namespace['bind_serial'](connection_expected)
    connection_path=namespace['serial_path'](connection_expected)
    node,_=namespace['serial_proof'](connection_expected)
    connection_node=(node.st_dev,node.st_ino,node.st_rdev)
    recovery=globals().get('native_usb_recovery')
    if usb_recovery:
        recovery=native_transports.prepare_usb_recovery(namespace,connection_expected) or recovery
    connection_source=guard_source
    if recovery is not None:
        connection_source=native_transports.install_usb_recovery(namespace,guard_source,recovery)
    # Ownership is actionable even when offline startup captured no UART pin.
    if namespace['_holders'](node.st_rdev,time.monotonic()+.20):
        raise namespace['ProbeError']('UART already has an owner')
    # A metadata-only startup may lack a node because another process owned it.
    # Only an explicit recovery check may establish that missing baseline. The
    # board, kernel, RAM, calibration and final owner checks below still run;
    # no UART allowance is published until every check succeeds.
    missing_offline_pin = (not initial and usb_recovery
        and deployment.get('startup_mode') == 'metadata_only_fault_page'
        and pin.get('serial_node_posix') is None
        and isinstance(pin.get('startup_fault'), dict)
        and pin['startup_fault'].get('hardware_gate_passed') is False
        and pin['startup_fault'].get('serial_open_attempts') == 0)
    if (initial or initial_connection_gate_pending and not (usb_recovery and recovery is not None)
            and not missing_offline_pin) and (connection_path!=deployment['serial_path_review'] or
            dict(zip(('dev','ino','rdev'),connection_node))!=pin['serial_node_posix']):
        raise namespace['ProbeError']('Native initial UART differs from quiescence review')
    def ram_health():
        # Retain this connection's exact snapshot. Later USB faults still fail;
        # only an explicit reconnect can reconcile a complete unplug/replug.
        result=namespace['health_snapshot'](connection_expected)
        actual,_=namespace['serial_proof'](connection_expected)
        if ((actual.st_dev,actual.st_ino,actual.st_rdev)!=connection_node or
                namespace['serial_path'](connection_expected)!=connection_path):
            raise namespace['ProbeError']('Native connected UART identity changed')
        mounts=namespace['bounded_read']('/proc/self/mountinfo',262144).splitlines()
        config=[line for line in mounts if len(line.split())>4 and line.split()[2]=='179:1']
        if config!=deployment['config_mount_review']: raise namespace['ProbeError']('Native CONFIG mount changed')
        shm=[line for line in mounts if len(line.split())>4 and line.split()[4]=='/dev/shm']
        if shm!=pin['shm_mount']: raise namespace['ProbeError']('Native RAM mount changed')
        info=root.stat()
        if [info.st_dev,info.st_ino]!=pin['ram_directory_node'] or root.is_symlink():
            raise namespace['ProbeError']('Native RAM directory changed')
        if calibration_reload_fault is not None: raise namespace['ProbeError'](calibration_reload_fault)
        if authority.verify()!=deployment['calibration_sha256']:
            raise namespace['ProbeError']('Native calibration changed without reviewed reload')
        return result
    namespace['ram_health']=ram_health
    namespace['serial_request_proof']=native_transports.pinned_uart_checker(
        connection_path,connection_node,resolve_path=namespace['serial_path'],
        error_type=namespace['ProbeError'])
    namespace['connection_guard_factory']=connection_guard_factory
    namespace['usb_recovery_guard_factory']=lambda:connection_guard_factory(usb_recovery=True)
    namespace['publish_calibration']=publish_calibration
    namespace['NATIVE_RUNTIME_SOURCE']=connection_source
    namespace['NATIVE_RAM_GUARD']=dict(root_path=str(root),root_node=pin['ram_directory_node'],
        shm_mount=pin['shm_mount'],config_mount=deployment['config_mount_review'],**authority.child_spec())
    ram_health()
    if usb_recovery and recovery is not None:
        # Observe stability again before publishing metadata reconciliation.
        # These checks do not open UART or grant servo readiness.
        for _ in range(2):
            time.sleep(.10)
            ram_health()
            if namespace['bounded_dmesg']()!=recovery['kernel_prefix']:
                raise namespace['ProbeError']('检查期间 USB 日志变化，请等待连接稳定后再检查。')
    actual,_=namespace['serial_proof'](connection_expected)
    if namespace['_holders'](actual.st_rdev,time.monotonic()+.20):
        raise namespace['ProbeError']('UART acquired an owner during explicit connection check')
    # Only a completed explicit gate updates the audit allowance. All later
    # health/transport checks retain this connection's captured path and inode.
    uart,uart_identity=connection_path,connection_node
    if not initial: initial_connection_gate_pending=False
    # Retain the completed connection's live identity/RAM checks, never the
    # owner inventory. A checked calibration reader may own UART while saving.
    last_completed_ram_health=ram_health
    if usb_recovery and recovery is not None:
        directory=root/'usb-recovery-evidence'
        directory.mkdir(mode=0o700,exist_ok=True)
        receipt=directory/('recovered-'+str(time.monotonic_ns())+'.json')
        descriptor=os.open(receipt,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
        with os.fdopen(descriptor,'w') as stream:
            json.dump(dict(proof=recovery,serial_node=list(connection_node),
                observation_count=3,serial_open_attempts=0,motion_resumed=False),stream,ensure_ascii=False)
        native_usb_recovery=recovery
    return namespace
startup_fault=pin.get('startup_fault')
offline_startup=deployment.get('startup_mode')=='metadata_only_fault_page'
if offline_startup:
    if (type(startup_fault) is not dict or startup_fault.get('hardware_gate_passed') is not False
            or startup_fault.get('serial_open_attempts')!=0
            or type(startup_fault.get('reason')) is not str or not 0<len(startup_fault['reason'].encode())<=16384):
        raise ValueError('Explicit offline startup lacks its actual metadata fault proof')
    # The observed serial metadata is an identity pin, never a health approval.
    # No audit UART allowance is granted and no strict hardware gate is run.
    serial=startup_fault.get('serial_metadata')
    if serial is not None:
        deployment['serial_path_review']=serial['identity']['path']
        deployment['serial_node_review']=serial['node']
    namespace={}
    exec(guard_code,namespace)
    namespace['EXPECTED']=json.loads(json.dumps(expected,allow_nan=False))
    namespace['connection_guard_factory']=connection_guard_factory
    namespace['usb_recovery_guard_factory']=lambda:connection_guard_factory(usb_recovery=True)
    def retained_ram_health():
        health=last_completed_ram_health
        if not callable(health):
            raise namespace['ProbeError'](startup_fault['reason'])
        return health()
    namespace['ram_health']=retained_ram_health
    namespace['publish_calibration']=publish_calibration
    namespace['NATIVE_RUNTIME_SOURCE']=guard_source
else:
    if deployment.get('startup_mode')!='strict' or startup_fault is not None:
        raise ValueError('Strict native startup mode or fault proof differs')
    namespace=connection_guard_factory(initial=True)
import native_runtime,native_workbench,native_calibration
# 本机适配 HEAD-PROFILE-20261012：官方 controller.head 初始为全零；
# 这里从带哈希校验的安装配置注入 head_command（官方61维输入的51:55）。
# 本次 profile 的目标角度来自用户确认独立站立30秒的旧记录，而非猜测重心。
# 只选旧头颈姿态这一项；KP6/KD20、腿HOME、原ONNX/EMA、当前校准与范围
# 保持现状。完整出处和输入弧度值在 service/deployment.json 的
# walk_head_command_provenance / walk_head_command_rad 中；缺省仍为零。
# 读取配置只做元数据初始化，不自动连接UART或启动动作。
runtime=native_runtime.NativeRuntime(namespace,str(root),deployment,calibration,
    (root/'service/worker-classes.py').read_text(),
    startup_fault=startup_fault['reason'] if offline_startup else None,
    allow_fixed_head_walk=True, hold_hip_pitch_bias_deg=0,
    walk_head_command_rad=deployment.get('walk_head_command_rad'))
def calibration_writer(config,provenance):
    global calibration_reload_fault,loaded_calibration
    # Called under the wizard session lock, sometimes from its reader thread.
    # Acquiring HTTP io_lock here would invert the connect/end-reader lock order.
    runtime.prepare_calibration_reload(config)
    warning=store.commit(config,provenance)
    try:
        loaded_calibration=store.load()
        runtime.reload_calibration(config,provenance)
    except BaseException as error:
        calibration_reload_fault='校准已持久保存，但 RAM 运控载入失败；运动已锁定，需重启工作台：'+str(error)
        return calibration_reload_fault+(' '+warning if warning else '')
    return warning
transports=native_transports.NativeLocalTransports(connection_guard_factory,root,
    recovery_guard_factory=lambda:connection_guard_factory(usb_recovery=True))
session_directory=root/'calibration-session'
session_directory.mkdir(mode=0o700,exist_ok=False)
for name,hold in load('service/startup-snapshot.json').get('native_recovery_holds',{}).items():
    if name not in ('maintenance-fault-hold.json','maintenance-identity-hold.json') or type(hold) is not dict:
        raise ValueError('Invalid restored native maintenance hold')
    descriptor=os.open(session_directory/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
    with os.fdopen(descriptor,'w') as stream: json.dump(hold,stream,ensure_ascii=False)
calibration_service=native_calibration.NativeCalibrationService(calibration,session_directory,
    expected['generation'],reader_launcher=transports.launch_reader,
    connection_checker=transports.check_connection,calibration_writer=calibration_writer,
    zero_provenance=zero_provenance,host='192.168.1.11',
    startup_fault=startup_fault['reason'] if offline_startup else None)
calibration_service.session.configure_maintenance(transports.maintenance_factory,hold_directory=session_directory)
def service_ready(info):
    # Observe real runtime state after the actual HTTP listener was bound.
    motion=runtime.snapshot();hardware=motion.get('hardware')
    if (type(info) is not dict or info.get('pid')!=os.getpid() or info.get('port')!=deployment['port']
            or info.get('address')!=deployment['bind_host']):
        raise ValueError('Native bound listener identity differs')
    if info.get('auth_required') is not deployment['auth_required']:
        raise ValueError('Native authentication mode differs from explicit deployment')
    for key in ('listener_fd','listener_inode'):
        if type(info.get(key)) is not int or info[key]<1: raise ValueError('Native listener proof is missing')
    if hardware is not None and type(hardware) is not dict: raise ValueError('Native hardware schema differs')
    closed=motion.get('closed_proof')
    if closed is not None and type(closed) is not dict: raise ValueError('Native closure schema differs')
    import motion as policy_motion
    for evidence in (hardware,closed):
        counts=(evidence or {}).get('request_counts',{})
        if counts.get(str(policy_motion.ARM),0) or counts.get(str(policy_motion.TARGET),0):
            raise ValueError('Native startup unexpectedly sent motor commands')
    ready=dict(schema_version=1,kind='native_service_ready',deployment_generation=expected['generation'],
        pid=os.getpid(),address=deployment['bind_host'],port=deployment['port'],
        listener_fd=info['listener_fd'],listener_inode=info['listener_inode'],auth_required=deployment['auth_required'],
        web_ready=True,controller_ready=bool(motion.get('phase')=='ready' and hardware and
            hardware.get('connected') and hardware.get('mode')=='off' and hardware.get('release_confirmed')),
        ram_directory=str(root),ram_directory_node=pin['ram_directory_node'],
        calibration_sha256=deployment['calibration_sha256'],model_hashes=deployment['model_hashes'],
        calibration_store={key:loaded_calibration[key] for key in ('source_calibration_sha256','revision',
            'calibration_sha256','zero_provenance_sha256','persistent')},
        motor_start_attempted=False,automatic_reconnect=False,motion=motion)
    ready['startup_fault']=(dict(schema_version=1,kind='native_offline_startup_fault',
        reason=startup_fault['reason'],proof_sha256=hashlib.sha256(json.dumps(startup_fault,
            sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest(),
        hardware_gate_passed=False,serial_open_attempts=0,automatic_retry=False)
        if offline_startup else None)
    path=root/'service-ready.json'
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump(ready,stream,ensure_ascii=False,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
native_process.serve(runtime,web_root=root/'web',secret_file=root/'service/access-secret.json',
    host=deployment['bind_host'],port=deployment['port'],snapshot_template=root/'service/startup-snapshot.json',
    auto_connect=not offline_startup,ready_callback=service_ready,authentication_required=deployment['auth_required'],
    calibration_service=calibration_service)
