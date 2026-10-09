"""Pure preparation of the explicitly authorized, detached X5 RAM workbench.

This module never connects to SSH, a network, or a device. The caller supplies a
completed strict quiescence proof and launches the returned source once through
the existing pinned SSH transport. Its remote parent never opens UART and only
reports detached readiness after its own child has actually bound the service.
"""
from __future__ import annotations

import ast
import copy
import hashlib
import inspect
import io
import json
from pathlib import Path
import re
import secrets
import tarfile

import calibration_restore
import feedback_connection_check
import feedback_stream
import motion_workbench
import run_ram_model_web as archive_tools
from transport_config import resolve_transport

HERE = Path(__file__).resolve().parent
SERVICE_PORT = 8767
MAX_SNAPSHOT_BYTES = 1024 * 1024
SOURCE_CALIBRATION_SHA256 = '97e023a45254280fda4ea12189eaf0b37253ed8b4a7cba465794dd92e5db89a1'
PERSISTENT_CALIBRATION_DIRECTORY = '/var/lib/xgoduck-workbench'
CALIBRATION_MODULES = ('feedback_calibration_server', 'calibration_restore', 'imu_wizard',
    'joint_calibration', 'servo_maintenance', 'maintenance_diagnostics', 'maintenance_transport',
    'run_feedback_fault_details', 'feedback_fault_probe', 'feedback_stream',
    'feedback_connection_check', 'transport_config')


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False,
                       separators=(',', ':')) + '\n').encode('utf-8')


def _classes_source():
    tree = ast.parse(motion_workbench.WORKER)
    classes = []
    for name in ('ReviewedTransport', 'OneShotController'):
        matches = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == name]
        if len(matches) != 1:
            raise ValueError('Reviewed motion worker class is not unique: ' + name)
        classes.append(ast.unparse(matches[0]))
    return '\n\n'.join(classes) + '\n'


def _canonical_expected(expected, reviewed_profile_path, reviewed_profile_sha256):
    value = copy.deepcopy(expected)
    marker = value.pop('reviewed_reconnect_proof', None) if type(value) is dict else None
    _, canonical = feedback_stream._source_expected(value, reviewed_profile_path, reviewed_profile_sha256)
    if marker is not None and marker != canonical.get('reviewed_reconnect_proof'):
        raise ValueError('Native supplied reconnect marker differs from its exact reviewed profile')
    return canonical


def _snapshot_template(snapshot, calibration):
    if type(snapshot) is not dict:
        raise ValueError('A captured workbench schema template is required')
    result = copy.deepcopy(snapshot)
    maintenance = snapshot.get('maintenance') or {}
    holds = {}
    if maintenance.get('fault_hold') is True:
        holds['maintenance-fault-hold.json'] = dict(
            reason=str(maintenance.get('error') or maintenance.get('blocked_reason') or '原维护故障待复查')[:512],
            generation=snapshot.get('session',{}).get('generation'),
            requires_feedback_review=True, restored_from_snapshot=True)
    if maintenance.get('identity_changed') is True:
        holds['maintenance-identity-hold.json'] = dict(reason='原舵机编号映射待核对',
            requires_physical_mapping_review=True, restored_from_snapshot=True)
    result['native_recovery_holds'] = holds
    # Saved calibration is real; old feedback and ownership tokens are not
    # valid observations of the independent service.
    result.pop('token', None)
    result.update(calibration=copy.deepcopy(calibration), feedback=None, body_imu=None,
                  motion=None, maintenance=None)
    session = result.get('session')
    if type(session) is not dict:
        raise ValueError('Workbench schema template lacks session fields')
    session.update(status='closed', recovery_allowed=False, recovery_pending=False,
                   recheck_allowed=False, received_age_ms=None, closed_proof=None,
                   fault_feedback=None, fault_ids=None, torque_ids=None)
    result['native_schema_template'] = True
    if len(_json_bytes(result)) > MAX_SNAPSHOT_BYTES:
        raise ValueError('Workbench schema template exceeds its bounded size')
    return result


def _validate_zero_provenance(value, calibration):
    # Validate the exact chain locally using the existing pure validator without
    # importing the Mac server (and its scientific libraries).
    path = HERE / 'feedback_calibration_server.py'
    tree = ast.parse(path.read_text())
    names = ('_zero_rows', '_zero_receipt_digest', 'validated_zero_provenance')
    nodes = [next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
             for name in names]
    import joint_calibration
    namespace = dict(copy=copy, hashlib=hashlib, json=json,
                     calibration_restore=calibration_restore, joint_calibration=joint_calibration)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<native-zero-provenance>', 'exec'), namespace)
    if type(value) is not dict or not isinstance(value.get('original_zeros'), list):
        raise ValueError('Audited zero provenance is required')
    original = copy.deepcopy(calibration)
    if len(value['original_zeros']) != 15:
        raise ValueError('Zero provenance must preserve 15 original zeros')
    for row, joint in zip(value['original_zeros'], original['joints']):
        if type(row) is not dict or row.get('id') != joint['id']:
            raise ValueError('Zero provenance original joint order changed')
        joint['zero'] = row.get('zero')
    return namespace['validated_zero_provenance'](value, original, calibration)


def build_bundle(snapshot, calibration, zero_provenance, *, expected,
                 reviewed_profile_path, reviewed_profile_sha256, quiescence_proof=None,
                 transport_config=None, access_password=None, authentication_required=True,
                 offline_startup=False):
    """Build locally, preserving current models/calibration and all exact guards."""
    transport = resolve_transport(transport_config)
    if transport.kind != 'wifi' or transport.host != '192.168.1.11':
        raise ValueError('Native workbench requires the explicit reviewed Wi-Fi transport')
    if type(authentication_required) is not bool:
        raise ValueError('Native authentication mode must be an explicit boolean')
    if type(offline_startup) is not bool:
        raise ValueError('Native offline startup must be an explicit boolean')
    canonical = _canonical_expected(expected, reviewed_profile_path, reviewed_profile_sha256)
    if offline_startup and quiescence_proof is not None:
        raise ValueError('Offline startup must not reuse a hardware-health/quiescence proof')
    proof = None if offline_startup else feedback_connection_check.validate_proof(quiescence_proof, canonical)
    calibration_restore.validate_calibration_snapshot(calibration)
    if hashlib.sha256(motion_workbench.calibration_bytes(calibration)).hexdigest() != SOURCE_CALIBRATION_SHA256:
        raise ValueError('Native initial calibration differs from its verified saved source')
    provenance = _validate_zero_provenance(zero_provenance, calibration)
    password = secrets.token_urlsafe(32) if access_password is None else access_password
    if type(password) is not str or not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', password):
        raise ValueError('Private native access password must contain 32 to 128 random-safe characters')
    archive, base = motion_workbench.prepare_bundle(calibration, boot_id=canonical['boot_id'])
    members = archive_tools.inspect_tar(archive, base['files'])
    runtime = dict(schema_version=1, scope='native_ram_workbench',
                   generation=canonical['generation'], bind_host=transport.host, port=SERVICE_PORT,
                   calibration_sha256=base['calibration_sha256'], model_hashes=base['model_hashes'],
                   config_mount_review=base['config_mount_review'],
                   serial_node_review=None if proof is None else proof['serial_node'],
                   serial_path_review=None if proof is None else proof['serial_path'],
                   startup_mode='metadata_only_fault_page' if offline_startup else 'strict',
                   auth_required=authentication_required,
                   full_workbench=True, persistent_calibration_directory=PERSISTENT_CALIBRATION_DIRECTORY,
                   source_calibration_sha256=SOURCE_CALIBRATION_SHA256,
                   source_zero_provenance_sha256=hashlib.sha256(_json_bytes(provenance)[:-1]).hexdigest(),
                   automatic_motor_start=False,
                   persistent_model_writes=False, automatic_reconnect=False)
    additions = {
        'service/native_runtime.py': (HERE.parent / 'x5_policy/native_runtime.py').read_bytes(),
        'service/native_workbench.py': (HERE.parent / 'x5_policy/native_workbench.py').read_bytes(),
        'service/native_process.py': (HERE.parent / 'x5_policy/native_process.py').read_bytes(),
        'service/guard-runtime.py': feedback_stream.runtime_source().encode('utf-8'),
        'service/worker-classes.py': _classes_source().encode('utf-8'),
        'service/expected.json': _json_bytes(canonical),
        'service/deployment.json': _json_bytes(runtime),
        'service/startup-snapshot.json': _json_bytes(_snapshot_template(snapshot, calibration)),
        'service/joint-zero-provenance.json': _json_bytes(provenance),
        'service/access-secret.json': _json_bytes({'password': password}),
        'service/bootstrap.py': CHILD_BOOTSTRAP.encode('utf-8'),
        'web/index.html': (HERE / 'feedback_calibration.html').read_bytes(),
    }
    for name in CALIBRATION_MODULES:
        additions['service/' + name + '.py'] = (HERE / (name + '.py')).read_bytes()
    for name in ('native_calibration', 'native_transports', 'native_calibration_store'):
        additions['service/' + name + '.py'] = (HERE.parent / 'x5_policy' / (name + '.py')).read_bytes()
    for name in ('zero-reference.png', 'positive-direction.png'):
        additions['web/assets/' + name] = (HERE.parent / 'x5_policy/assets' / name).read_bytes()
    for name, source in additions.items():
        if name.endswith('.py'):
            compile(source, '<' + name + '>', 'exec')
    members.update(additions)
    wheel_bytes = archive_tools.inspect_wheels(members)
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as target:
        for name, data in sorted(members.items()):
            item = tarfile.TarInfo(name)
            item.size, item.mode, item.mtime = len(data), 0o600, 0
            target.addfile(item, io.BytesIO(data))
    data = buffer.getvalue()
    deployment = dict(runtime, archive_sha256=hashlib.sha256(data).hexdigest(), archive_size=len(data),
        member_bytes=sum(map(len, members.values())), wheel_bytes=wheel_bytes,
        files={name: dict(size_bytes=len(value), sha256=hashlib.sha256(value).hexdigest())
               for name, value in members.items()},
        transport_config=transport.as_record(), quiescence_proof=proof,
        required_source_sha256={name: hashlib.sha256(additions[name]).hexdigest()
                                for name in additions if name.endswith('.py')})
    return data, deployment


CHILD_BOOTSTRAP = r'''
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
import native_calibration_store as calibration_storage,native_transports,native_process
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
                native_process.allowed_web_command(args[0],args[1],root)):
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
    # Model loading occurs between the bootstrap review and the runtime's
    # first connect. That first gate still requires the quiescence identity;
    # the bootstrap-only initial check does not consume it.
    if (initial or initial_connection_gate_pending and not (usb_recovery and recovery is not None)) and (connection_path!=deployment['serial_path_review'] or
            dict(zip(('dev','ino','rdev'),connection_node))!=pin['serial_node_posix']):
        raise namespace['ProbeError']('Native initial UART differs from quiescence review')
    if namespace['_holders'](node.st_rdev,time.monotonic()+.20):
        raise namespace['ProbeError']('UART already has an owner')
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
# This deployment selects a fixed HOLD-only hip-pitch candidate, relative to
# the existing paired stance: ID12 -= 2 degrees, ID22 += 2; ankles stay fixed.
# Captured head targets, model HOME, calibration, interpolation and all existing
# feedback/tilt/actual/target limits remain unchanged. Every live model handoff
# is blocked until separately reviewed; this is not a zero-speed balance build.
runtime=native_runtime.NativeRuntime(namespace,str(root),deployment,calibration,
    (root/'service/worker-classes.py').read_text(),
    startup_fault=startup_fault['reason'] if offline_startup else None,
    allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=-2.0)
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
    zero_provenance=zero_provenance,host=deployment['bind_host'],
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
'''


REMOTE_PARENT = r'''
import io, platform, shutil, socket, tarfile, tempfile, zipfile
ram_directory=None
child=None
detached=False
startup_fault=None
def output(kind,**values):
    print(json.dumps(dict(kind=kind,deployment_generation=EXPECTED['generation'],**values),
        ensure_ascii=False,allow_nan=False,separators=(',',':')),flush=True)
def offline_parent_metadata():
    global startup_fault
    mounts=bounded_read('/proc/self/mountinfo',262144).splitlines()
    roots=[line for line in mounts if len(line.split())>4 and line.split()[4]=='/']
    if len(roots)!=1: raise ProbeError('Offline native root mount is not unique')
    core=dict(boot_id=bounded_read('/proc/sys/kernel/random/boot_id',128),
        cid=bounded_read('/sys/block/mmcblk1/device/cid',128),root_mount_line=roots[0],
        model=bounded_read('/proc/device-tree/model',4096,binary=True).rstrip(b'\0').decode('utf-8','strict'),
        swaps=bounded_read('/proc/swaps',16384))
    validate_os(core,EXPECTED)
    card=os.stat('/dev/mmcblk1p2',follow_symlinks=False)
    if not stat.S_ISBLK(card.st_mode) or (os.major(card.st_rdev),os.minor(card.st_rdev))!=(179,2):
        raise ProbeError('Offline native canonical root block-node identity changed')
    if bounded_read('/sys/block/mmcblk1/device/type',32)!='SD':
        raise ProbeError('Offline native root card is not SD')
    kernel=bounded_dmesg()
    kernel_error=None
    try: validate_kernel(kernel,EXPECTED)
    except ProbeError as error: kernel_error=(str(error) or type(error).__name__)[:8192]
    serial_metadata=None;serial_error=None;node=None
    try:
        bind_serial(EXPECTED)
        node,identity=serial_proof(EXPECTED)
        owners=_holders(node.st_rdev,time.monotonic()+.20)
        serial_metadata=dict(identity=identity,node={'device':node.st_dev,'inode':node.st_ino,'rdev':node.st_rdev},
            holders=owners)
    except (OSError,ProbeError) as error:
        serial_error=(str(error) or type(error).__name__)[:8192]
        node=None
    reasons=[value for value in (kernel_error,serial_error) if value]
    reason=('硬件检查未通过；尚未启动舵机操作和采集：'+'；'.join(reasons)) if reasons else (
        '已明确选择仅启动诊断网页；尚未执行完整硬件连接检查，尚未启动舵机操作和采集。')
    startup_fault=dict(schema_version=1,kind='native_offline_startup_fault',
        reason=reason.encode()[:16384].decode('utf-8','ignore'),
        core_metadata=core,kernel_log=kernel,kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),
        kernel_error=kernel_error,serial_metadata=serial_metadata,serial_error=serial_error,
        hardware_gate_passed=False,uart_granted=False,serial_open_attempts=0,serial_bytes_written=0,
        board_files_written=False,automatic_retry=False)
    return node,mounts
def parent_health():
    if DEPLOY['startup_mode']=='metadata_only_fault_page':
        node,mounts=offline_parent_metadata()
    else:
        proof=health_snapshot(EXPECTED)
        node,_=serial_proof(EXPECTED)
        actual={'device':node.st_dev,'inode':node.st_ino,'rdev':node.st_rdev}
        if actual!=DEPLOY['serial_node_review']: raise ProbeError('Native deployment UART node changed')
        if serial_path(EXPECTED)!=DEPLOY['serial_path_review']: raise ProbeError('Native deployment UART path changed')
        if _holders(node.st_rdev,time.monotonic()+.20): raise ProbeError('UART still has an owner; native deployment refused')
        mounts=bounded_read('/proc/self/mountinfo',262144).splitlines()
    config=[line for line in mounts if len(line.split())>4 and line.split()[2]=='179:1']
    if config!=DEPLOY['config_mount_review']: raise ProbeError('Native deployment CONFIG mount changed')
    shm=[line for line in mounts if len(line.split())>4 and line.split()[4]=='/dev/shm']
    if len(shm)!=1: raise ProbeError('Native RAM mount is not unique')
    fields=shm[0].split();sep=fields.index('-',6)
    if fields[sep+1]!='tmpfs' or 'rw' not in fields[5].split(',') or 'noexec' in fields[5].split(','):
        raise ProbeError('Native service requires writable executable tmpfs')
    return node,shm
def probe_native_address(host,port):
    # Match HTTPServer's SO_REUSEADDR behavior so a gracefully closed server's
    # TIME_WAIT connections do not reject its replacement. No SO_REUSEPORT:
    # an actual listening server still owns the address exclusively.
    probe=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        probe.bind((host,port))
    finally:
        probe.close()
def run_native_deployment():
    global ram_directory,child,detached
    try:
        if os.geteuid()!=0 or platform.machine()!='aarch64' or sys.version_info[:2]!=(3,10):
            raise ProbeError('Native deployment requires reviewed root aarch64 Python 3.10')
        if DEPLOY['startup_mode']=='strict': bind_serial(EXPECTED)
        node,shm=parent_health()
        needed=DEPLOY['archive_size']+DEPLOY['member_bytes']+DEPLOY['wheel_bytes']+128*1024*1024
        available=int(next(line.split()[1] for line in bounded_read('/proc/meminfo').splitlines() if line.startswith('MemAvailable:')))*1024
        if os.statvfs('/dev/shm').f_bavail*os.statvfs('/dev/shm').f_frsize<needed or available<needed+256*1024*1024:
            raise ProbeError('Insufficient RAM for independent native workbench')
        # Binding is a preflight only. The child must bind its own real socket.
        probe_native_address(DEPLOY['bind_host'],DEPLOY['port'])
        ram_directory=tempfile.mkdtemp(prefix='xgoduck-native-',dir='/dev/shm');os.chmod(ram_directory,0o700)
        output('transfer_ready',archive_bytes=DEPLOY['archive_size'],ram_directory=ram_directory,
            parent_serial_open_attempts=0,board_files_written=False,motor_start_attempted=False)
        raw=bytearray();deadline=time.monotonic()+120;last_health=0.
        while len(raw)<DEPLOY['archive_size']:
            if time.monotonic()>deadline: raise ProbeError('Native archive transfer deadline exceeded')
            if time.monotonic()-last_health>.5: parent_health();last_health=time.monotonic()
            if not select.select([sys.stdin.fileno()],[],[],.2)[0]: continue
            chunk=os.read(sys.stdin.fileno(),min(65536,DEPLOY['archive_size']-len(raw)))
            if not chunk: raise ProbeError('Native archive transfer ended early')
            raw.extend(chunk)
        if hashlib.sha256(raw).hexdigest()!=DEPLOY['archive_sha256']: raise ProbeError('Native archive digest differs')
        node,shm=parent_health()
        members=inspect_tar(bytes(raw),DEPLOY['files']);del raw
        if inspect_wheels(members)!=DEPLOY['wheel_bytes']: raise ProbeError('Native wheel budget differs')
        for name,info in DEPLOY['required_source_sha256'].items():
            if hashlib.sha256(members[name]).hexdigest()!=info: raise ProbeError('Native reviewed source differs')
        for name,data in members.items():
            path=Path(ram_directory)/name;path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
            with path.open('xb') as stream:stream.write(data)
            path.chmod(0o600)
        for name,data in members.items():
            if not name.startswith('wheels/'): continue
            with zipfile.ZipFile(io.BytesIO(data)) as wheel:
                for item in wheel.infolist():
                    target=Path(ram_directory)/'site'/safe_relative_path(item.filename)
                    if item.is_dir():target.mkdir(mode=0o700,parents=True,exist_ok=True);continue
                    target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
                    with wheel.open(item) as source,target.open('xb') as stream:shutil.copyfileobj(source,stream,65536)
                    target.chmod(0o600)
        node,shm=parent_health()
        info=os.stat(ram_directory)
        pin=dict(files=DEPLOY['files'],ram_directory_node=[info.st_dev,info.st_ino],shm_mount=shm,
            serial_node_posix=None if node is None else {'dev':node.st_dev,'ino':node.st_ino,'rdev':node.st_rdev},
            startup_fault=startup_fault)
        with (Path(ram_directory)/'native-pin.json').open('x') as stream:json.dump(pin,stream,allow_nan=False)
        (Path(ram_directory)/'native-pin.json').chmod(0o600)
        logs=[os.fdopen(os.open(Path(ram_directory)/name,
            os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600),'wb')
            for name in ('stdout.log','stderr.log')]
        try:
            child=subprocess.Popen([sys.executable,'-I','-B',str(Path(ram_directory)/'service/bootstrap.py')],
                stdin=subprocess.DEVNULL,stdout=logs[0],stderr=logs[1],close_fds=True,start_new_session=True,
                cwd=ram_directory,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        finally:
            for stream in logs:stream.close()
        deadline=time.monotonic()+60
        ready_path=Path(ram_directory)/'service-ready.json'
        while time.monotonic()<deadline:
            if child.poll() is not None: raise ProbeError('Native service child exited before readiness')
            if ready_path.exists():
                descriptor=os.open(ready_path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
                with os.fdopen(descriptor,'rb') as stream:
                    file=os.fstat(stream.fileno())
                    if not stat.S_ISREG(file.st_mode) or file.st_uid!=0 or stat.S_IMODE(file.st_mode)!=0o600 or file.st_nlink!=1 or not 0<=file.st_size<=1048576:
                        raise ProbeError('Native ready file is not private and bounded')
                    payload=stream.read(1048577)
                # The callback's exclusive file may still be completing its one write.
                if not payload.endswith(b'\n'):time.sleep(.02);continue
                try:ready=json.loads(payload)
                except json.JSONDecodeError:time.sleep(.02);continue
                validate_ready(ready,child.pid,ram_directory,DEPLOY,EXPECTED)
                if startup_fault is not None:
                    actual_hash=hashlib.sha256(json.dumps(startup_fault,sort_keys=True,allow_nan=False,
                        separators=(',',':')).encode()).hexdigest()
                    if ready['startup_fault']['proof_sha256']!=actual_hash:
                        raise ProbeError('Native offline fault proof differs from actual parent metadata')
                if ready['ram_directory_node']!=pin['ram_directory_node']:
                    raise ProbeError('Native ready RAM node differs')
                link=os.readlink('/proc/'+str(child.pid)+'/fd/'+str(ready['listener_fd']))
                if link!='socket:['+str(ready['listener_inode'])+']':raise ProbeError('Native live listener inode differs')
                if child.poll() is not None:raise ProbeError('Native child exited after readiness')
                detached=True
                output('detached_ready',**{k:v for k,v in ready.items() if k not in ('kind','deployment_generation')},
                    ssh_parent_may_exit=True,persistent_files_written=False)
                return 0
            time.sleep(.05)
        raise ProbeError('Native child readiness deadline exceeded')
    except BaseException as error:
        output('fault',reason=str(error),ram_directory=ram_directory,parent_serial_open_attempts=0,
            child_closure_unknown=child is not None,motor_start_attempted=False,detached_ready=False)
        return 20
    finally:
        # A failed deployment may terminate only the child it created. A
        # successful detached child remains alive independently of SSH.
        if child is not None and not detached and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Keep RAM and uncertainty; never kill an unproved live UART owner.
                output('cleanup_incomplete',reason='Native child did not exit after termination',ram_directory=ram_directory)
raise SystemExit(run_native_deployment())
'''


def validate_ready(value, pid, ram_directory, deployment, expected):
    fields = {'schema_version', 'kind', 'deployment_generation', 'pid', 'address', 'port',
              'listener_fd', 'listener_inode', 'auth_required', 'web_ready', 'controller_ready',
              'ram_directory', 'ram_directory_node', 'calibration_sha256', 'model_hashes',
              'motor_start_attempted', 'automatic_reconnect', 'motion', 'calibration_store','startup_fault'}
    if type(value) is not dict or set(value) != fields:
        raise ValueError('Native readiness schema differs')
    if (value['schema_version'] != 1 or type(value['schema_version']) is not int
            or value['kind'] != 'native_service_ready'
            or type(value['deployment_generation']) is not int
            or value['deployment_generation'] != expected['generation']
            or type(value['pid']) is not int or value['pid'] != pid
            or value['address'] != deployment['bind_host'] or type(value['port']) is not int
            or value['port'] != SERVICE_PORT or value['ram_directory'] != str(ram_directory)
            or type(value['auth_required']) is not bool
            or value['auth_required'] is not deployment['auth_required'] or value['web_ready'] is not True
            or type(value['controller_ready']) is not bool or value['motor_start_attempted'] is not False
            or value['automatic_reconnect'] is not False
            or value['model_hashes'] != deployment['model_hashes']
            or type(value['motion']) is not dict):
        raise ValueError('Native readiness identity or safety proof differs')
    startup=value['startup_fault']
    if deployment.get('startup_mode')=='metadata_only_fault_page':
        fields={'schema_version','kind','reason','proof_sha256','hardware_gate_passed',
                'serial_open_attempts','automatic_retry'}
        closed=value['motion'].get('closed_proof')
        if (type(startup) is not dict or set(startup)!=fields or type(startup['schema_version']) is not int
                or startup['schema_version']!=1 or startup['kind']!='native_offline_startup_fault'
                or type(startup['reason']) is not str or not 0<len(startup['reason'].encode())<=16384
                or type(startup['proof_sha256']) is not str or re.fullmatch(r'[0-9a-f]{64}',startup['proof_sha256']) is None
                or startup['hardware_gate_passed'] is not False or startup['automatic_retry'] is not False
                or type(startup['serial_open_attempts']) is not int or startup['serial_open_attempts']!=0
                or value['controller_ready'] is not False or value['motion'].get('hardware') is not None
                or value['motion'].get('phase')!='fault' or value['motion'].get('owns_connection') is not False
                or value['motion'].get('startup_fault')!=startup['reason']
                or type(closed) is not dict or closed.get('no_uart_opened') is not True
                or closed.get('serial_closed') is not True or closed.get('release_confirmed') is not False
                or closed.get('serial_open_attempts')!=0 or closed.get('request_counts')!={}):
            raise ValueError('Native offline readiness must remain an actual fault without UART or health approval')
    elif deployment.get('startup_mode')!='strict' or startup is not None:
        raise ValueError('Native startup mode or fault proof differs')
    receipt = value['calibration_store']
    if (type(receipt) is not dict or set(receipt) != {'source_calibration_sha256','revision',
            'calibration_sha256','zero_provenance_sha256','persistent'}
            or receipt['source_calibration_sha256'] != deployment['source_calibration_sha256']
            or type(receipt['revision']) is not int or not 0 <= receipt['revision'] <= 0x7fffffff
            or type(receipt['persistent']) is not bool or receipt['persistent'] != (receipt['revision'] > 0)
            or any(type(receipt[key]) is not str or re.fullmatch(r'[0-9a-f]{64}',receipt[key]) is None
                   for key in ('calibration_sha256','zero_provenance_sha256'))
            or receipt['calibration_sha256'] != value['calibration_sha256']
            or (not receipt['persistent'] and (receipt['calibration_sha256'] != deployment['calibration_sha256']
                or receipt['zero_provenance_sha256'] != deployment['source_zero_provenance_sha256']))):
        raise ValueError('Native readiness calibration store receipt differs from validated source')
    for key in ('listener_fd', 'listener_inode'):
        if type(value[key]) is not int or value[key] < 1:
            raise ValueError('Native readiness listener proof differs')
    node = value['ram_directory_node']
    if type(node) is not list or len(node) != 2 or any(type(n) is not int or n < 1 for n in node):
        raise ValueError('Native readiness RAM node proof differs')
    hardware = value['motion'].get('hardware')
    if hardware is not None and type(hardware) is not dict:
        raise ValueError('Native readiness hardware schema differs')
    actual = bool(value['motion'].get('phase') == 'ready' and hardware
                  and hardware.get('connected') and hardware.get('mode') == 'off'
                  and hardware.get('release_confirmed'))
    if value['controller_ready'] != actual:
        raise ValueError('Native readiness cannot invent controller readiness')
    return copy.deepcopy(value)


def remote_source(expected, deployment, *, reviewed_profile_path,
                  reviewed_profile_sha256, transport_config=None):
    """Create the one-shot deployment source; do not launch any process."""
    transport = resolve_transport(transport_config)
    if transport.kind != 'wifi' or transport.as_record() != deployment.get('transport_config'):
        raise ValueError('Native deployment transport differs from its prepared bundle')
    canonical = _canonical_expected(expected, reviewed_profile_path, reviewed_profile_sha256)
    mode=deployment.get('startup_mode')
    if mode not in ('strict','metadata_only_fault_page'):
        raise ValueError('Native startup mode requires an explicit reviewed choice')
    if mode=='metadata_only_fault_page':
        if (deployment.get('quiescence_proof') is not None or deployment.get('serial_node_review') is not None
                or deployment.get('serial_path_review') is not None):
            raise ValueError('Native offline deployment must not grant a hardware proof')
        proof=None
    else:
        proof = feedback_connection_check.validate_proof(deployment['quiescence_proof'], canonical)
    if (deployment.get('generation') != canonical['generation']
            or (proof is not None and deployment.get('serial_node_review') != proof['serial_node'])
            or (proof is not None and deployment.get('serial_path_review') != proof['serial_path'])
            or deployment.get('bind_host') != transport.host or deployment.get('port') != SERVICE_PORT
            or deployment.get('automatic_motor_start') is not False
            or deployment.get('full_workbench') is not True
            or deployment.get('persistent_calibration_directory') != PERSISTENT_CALIBRATION_DIRECTORY
            or deployment.get('source_calibration_sha256') != SOURCE_CALIBRATION_SHA256
            or deployment.get('calibration_sha256') != SOURCE_CALIBRATION_SHA256
            or type(deployment.get('auth_required')) is not bool
            or deployment.get('automatic_reconnect') is not False):
        raise ValueError('Native deployment differs from its reviewed generation/node')
    functions = '\n\n'.join(inspect.getsource(f) for f in
        (archive_tools.safe_relative_path, archive_tools.inspect_tar, archive_tools.inspect_wheels, validate_ready))
    source = (feedback_stream.runtime_source() + '\nimport copy\nfrom pathlib import PurePosixPath\n' +
        'MAX_EXPANDED_WHEEL_BYTES = 167772160\nSERVICE_PORT = 8767\n' + functions +
        '\nEXPECTED = ' + repr(canonical) + '\nDEPLOY = ' + repr(deployment) + '\n' + REMOTE_PARENT)
    compile(source, '<native-workbench-deployment>', 'exec')
    return source.encode('utf-8')
