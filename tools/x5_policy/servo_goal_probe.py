"""Build one guarded OFF-only mapped-servo probe (default ID11), without UART.

The generated script is for the existing pinned remote runner. It makes one
HELLO, bounded STATE queries, exactly one GoalReadDetails request, then STATE.
It never sends STOP, CONFIG, ARM, TARGET or a maintenance request.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import tarfile
import time

from servo_goal_diagnostics import MOTOR_IDS, decode_goal_read_details_frame


def adapt_helpers(source, namespace, *, probe_id=11):
    """Narrowly extend the pinned original helpers in a private namespace."""
    if type(probe_id) is not int or probe_id not in MOTOR_IDS:
        raise ValueError('Probe ID must be one mapped motor ID')
    tree = ast.parse(source)
    definitions = {}
    for name in ('_frame', '_exchange'):
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
        if len(nodes) != 1:
            raise ValueError('Missing or duplicate reviewed helper: ' + name)
        definitions[name] = ast.get_source_segment(source, nodes[0])
    frame = definitions['_frame']
    exchange = definitions['_exchange']
    changes = (
        ('def _frame(kind, sequence):', 'def _frame(kind, sequence, payload=b""):'),
        ('if kind not in (1, 2):', 'if not ((kind in (1, 2) and payload == b"") or (kind == 22 and payload == '+repr(bytes([probe_id]))+')):'),
        ('HEADER.pack(b"XD", 1, kind, 0, sequence)', 'HEADER.pack(b"XD", 1, kind, len(payload), sequence) + payload'),
    )
    for before, after in changes:
        if frame.count(before) != 1:
            raise ValueError('Reviewed frame helper shape changed')
        frame = frame.replace(before, after)
    changes = (
        ('def _exchange(fd, kind, sequence, total_deadline, metadata):',
         'def _exchange(fd, kind, sequence, total_deadline, metadata, *, payload=b""):'),
        ('_frame(kind, sequence)', '_frame(kind, sequence, payload)'),
        ('(0x81, 16) if kind == 1 else (0x82, STATE_PACKET.size)',
         '{1: (0x81, 16), 2: (0x82, STATE_PACKET.size), 22: (0x96, 78)}[kind]'),
    )
    for before, after in changes:
        if exchange.count(before) != 1:
            raise ValueError('Reviewed exchange helper shape changed')
        exchange = exchange.replace(before, after)
    exec(compile(frame + '\n\n' + exchange, '<OFF-only-goal-exchange>', 'exec'), namespace)


def checked_off_state(namespace, payload):
    state = namespace['_decode_state'](payload)
    if (state['armed'] is not False or state['arm_cookie'] or state['torque_mask']
            or state['fault_mask'] or state['last_error'] or state['imu_ok'] is not True
            or any(not math.isfinite(x) for x in state['accel_ms2'] + state['gyro_dps'])):
        raise ValueError('Probe requires healthy OFF feedback')
    return state


def run_probe(namespace, expected, guard_check, *, clock=time.monotonic, sleeper=time.sleep, probe_id=11):
    """One fd, one diagnostic attempt, bounded readiness, always close."""
    if type(probe_id) is not int or probe_id not in MOTOR_IDS:
        raise ValueError('Probe ID must be one mapped motor ID')
    import fcntl
    import termios
    started = clock()
    fd = None
    result = dict(kind='off_servo_goal_probe', id=probe_id, success=False, serial_closed=False,
                  correlated_with_previous_hold=False, mcu_reset_possible=True,
                  feedback_updated=False, sensor_freshness_established=False,
                  modem_lines_actively_changed=False, hupcl_actively_changed=False,
                  motion_authorized=False, diagnostic_attempts=0, states=[],
                  request_timing=[],
                  query=dict(request_types_attempted=[], reply_types_received=[], request_bytes_written=0))
    metadata = result['query']
    def exchange(kind, sequence, deadline, **options):
        timing = dict(kind=kind, sequence=sequence, started_monotonic=clock())
        result['request_timing'].append(timing)
        try:
            return namespace['_exchange'](fd, kind, sequence, deadline, metadata, **options)
        finally:
            timing['ended_monotonic'] = clock()
    def health():
        return {key: value for key, value in guard_check(fd).items() if key != 'kernel_log'}
    try:
        namespace['bind_serial'](expected)
        guard_check()
        node, _ = namespace['serial_proof'](expected)
        if namespace['_holders'](node.st_rdev, clock() + .2):
            raise ValueError('UART already has an owner')
        guard_check()
        result['open_started_monotonic'] = clock()
        fd = os.open(namespace['serial_path'](expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        result['opened_monotonic'] = clock()
        namespace['serial_proof'](expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        if any(not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))
               for owner in namespace['_holders'](node.st_rdev, clock() + .2)):
            raise ValueError('Another UART owner appeared')
        if termios.tcgetattr(fd)[2] & termios.HUPCL:
            raise ValueError('HUPCL is set; refusing to change modem lifecycle')
        result['modem_before_raw'] = namespace['_modem_bits'](fd)
        result['serial_settings'] = namespace['_raw_settings'](fd)
        result['modem_after_raw'] = namespace['_modem_bits'](fd)
        if (termios.tcgetattr(fd)[2] & termios.HUPCL
                or result['modem_before_raw'] != result['modem_after_raw']):
            raise ValueError('UART raw setup changed modem lines or HUPCL')
        result['health_before'] = health()
        hello_started = clock()
        namespace['_hello_feedback'](fd, metadata)
        result['request_timing'].append(dict(kind=1, sequence=1, started_monotonic=hello_started,
                                             ended_monotonic=clock()))
        sequence = 1
        deadline = clock() + 2.
        previous = None
        while clock() < deadline:
            sequence += 1
            payload = exchange(2, sequence, min(deadline, clock() + .4))
            if clock() >= deadline:
                raise ValueError('OFF feedback readiness deadline expired')
            state = checked_off_state(namespace, payload)
            result['states'].append(dict(sequence=sequence, received_monotonic=clock(), state=state))
            if previous is not None:
                deltas = [(state[key] - previous[key]) & 0xffffffff for key in ('sample_us', 'imu_us')]
                if any(delta >= 0x80000000 or delta > 500000 for delta in deltas):
                    raise ValueError('STATE clocks reversed/reset or jumped')
                if state['online_mask'] == 0x7fff and all(deltas):
                    break
            previous = state
            sleeper(min(.025, max(0., deadline - clock())))
        else:
            raise ValueError('Healthy complete OFF feedback did not advance within 2 seconds')
        result['health_pre_diagnostic'] = health()
        sequence += 1
        result['diagnostic_attempts'] = 1
        payload = exchange(22, sequence, clock() + .4, payload=bytes([probe_id]))
        result['diagnostic_raw_payload_hex'] = payload.hex()
        result['diagnostic'] = decode_goal_read_details_frame((0x96, sequence, payload),
                                                             expected_id=probe_id, expected_sequence=sequence)
        sequence += 1
        payload = exchange(2, sequence, clock() + .4)
        after = checked_off_state(namespace, payload)
        result['post_state'] = dict(sequence=sequence, received_monotonic=clock(), state=after)
        diagnostic = result['diagnostic']
        for key, diag_key in (('sample_us', 'sample_before_us'), ('imu_us', 'imu_before_us')):
            first = (diagnostic[diag_key] - state[key]) & 0xffffffff
            second = (after[key] - diagnostic[diag_key]) & 0xffffffff
            if first >= 0x80000000 or second >= 0x80000000 or first + second > 500000:
                raise ValueError('Diagnostic clocks do not correlate with surrounding STATE')
        if after['online_mask'] != 0x7fff:
            raise ValueError('Post-diagnostic STATE is not fully online')
        result['health_after'] = health()
        result['success'] = diagnostic['valid']
        if not result['success']:
            result['error'] = 'Register READ evidence was invalid; original outcome retained'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
        result['error_type'] = type(error).__name__
        if getattr(error, 'raw_payload_hex', None) is not None:
            result['rejected_raw_payload_hex'] = error.raw_payload_hex
        result['success'] = False
    finally:
        if fd is not None:
            try:
                result['close_started_monotonic'] = clock()
                os.close(fd)
                result['closed_monotonic'] = clock()
                result['serial_closed'] = True
            except BaseException as error:
                result['close_error'] = str(error) or type(error).__name__
                result['success'] = False
        else:
            result['serial_closed'] = True
        result['elapsed_seconds'] = clock() - started
    return result


REMOTE_SETUP = r'''
import ast,fcntl,hashlib,json,math,os,stat,subprocess,sys,termios,time
from pathlib import Path
sys.dont_write_bytecode=True
def digest(data):return hashlib.sha256(data).hexdigest()
def read_private(path,limit=1048576):
    node=path.lstat()
    if not stat.S_ISREG(node.st_mode) or node.st_uid!=0 or node.st_mode&0o077 or node.st_nlink!=1 or node.st_size>limit:
        raise ValueError('Private reviewed file metadata differs: '+str(path))
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
    with os.fdopen(fd,'rb') as stream:
        opened=os.fstat(stream.fileno());data=stream.read(limit+1);ended=os.fstat(stream.fileno())
    identity=lambda n:(n.st_dev,n.st_ino,n.st_size,n.st_mtime_ns,n.st_ctime_ns)
    if len(data)>limit or not identity(node)==identity(opened)==identity(ended)==identity(path.lstat()):
        raise ValueError('Reviewed file changed during read: '+str(path))
    return data
def prepare_namespace():
    if os.geteuid()!=0:raise ValueError('Pinned root-owned probe requires root')
    installation=Path('/opt/xgoduck-workbench')
    if digest(read_private(installation/'installation.json'))!=PINS['installation_seal']:raise ValueError('Installation seal changed')
    for name,sha in PINS['installation_files'].items():
        if digest(read_private(installation/name,536870912))!=sha:raise ValueError('Installed file changed: '+name)
    pid=int(subprocess.run(['systemctl','show','xgoduck-workbench','--property=MainPID','--value'],check=True,capture_output=True,text=True,timeout=2).stdout)
    if pid<=1:raise ValueError('Native service parent is unavailable')
    processes=list(Path('/proc').iterdir())
    if len(processes)>4096:raise ValueError('Process inventory exceeded bound')
    children=[]
    for process in processes:
        if not process.name.isdecimal():continue
        try:
            if int((process/'stat').read_text().rsplit(')',1)[1].split()[1])==pid:children.append(process)
        except FileNotFoundError:pass
    if len(children)!=1:raise ValueError('Expected exactly one native child')
    root=Path(os.path.realpath(children[0]/'cwd'))
    root_node=root.lstat()
    if root.parent!=Path('/dev/shm') or not root.name.startswith('xgoduck-native-') or not stat.S_ISDIR(root_node.st_mode) or root_node.st_uid!=0 or root_node.st_mode&0o077:raise ValueError('Native RAM root differs')
    pin=json.loads(read_private(root/'native-pin.json'))
    if pin['ram_directory_node']!=[root_node.st_dev,root_node.st_ino] or set(pin['files'])!=set(PINS['bundle_files']):raise ValueError('Native RAM manifest differs')
    mutable={'service/expected.json','service/deployment.json','service/startup-snapshot.json','policy/calibration.json'}
    for name,info in pin['files'].items():
        data=read_private(root/name,536870912)
        if name=='policy/calibration.json':
            if digest(data)!=PINS['live_calibration_sha256']:raise ValueError('Live calibration differs from reviewed closure')
        elif len(data)!=info['size_bytes'] or digest(data)!=info['sha256']:raise ValueError('Native pinned file changed: '+name)
        target=PINS['guard_sha'] if name=='service/guard-runtime.py' else PINS['bundle_files'][name]['sha256']
        if name not in mutable and digest(data)!=target:raise ValueError('Native source/model differs from installation: '+name)
    guard=read_private(root/'service/guard-runtime.py').decode()
    namespace={};exec(compile(guard,'<installed-OFF-diagnostic-guard>','exec'),namespace)
    expected=json.loads(read_private(root/'service/expected.json'))
    if expected['boot_id']!=PINS['boot_id'] or expected['cid']!=PINS['cid'] or expected['model']!='D-Robotics RDK X5 V1.0':raise ValueError('Boot/card/model changed')
    sys.path.insert(0,str(root/'service'))
    import native_transports
    namespace['bind_serial'](expected)
    recovery=native_transports.prepare_usb_recovery(namespace,expected)
    native_transports.install_usb_recovery(namespace,guard,recovery)
    node,_=namespace['serial_proof'](expected)
    serial_node=(node.st_dev,node.st_ino,node.st_rdev)
    serial_path=namespace['serial_path'](expected)
    deployment=json.loads(read_private(root/'service/deployment.json'))
    calibration=read_private(root/'policy/calibration.json')
    if digest(calibration)!=PINS['live_calibration_sha256']:raise ValueError('Live calibration changed')
    def health(fd=None):
        if Path(os.path.realpath(children[0]/'cwd'))!=root or int((children[0]/'stat').read_text().rsplit(')',1)[1].split()[1])!=pid:raise ValueError('Native process identity changed')
        if digest(read_private(installation/'installation.json'))!=PINS['installation_seal']:raise ValueError('Installation seal changed')
        if root.lstat().st_ino!=root_node.st_ino or root.lstat().st_dev!=root_node.st_dev:raise ValueError('RAM root changed')
        if digest(read_private(Path('/var/lib/xgoduck-workbench/calibration-state.json')))!=PINS['persistent_calibration_sha256']:raise ValueError('Persistent calibration changed')
        if read_private(root/'policy/calibration.json')!=calibration:raise ValueError('Live calibration bytes changed')
        proof=namespace['health_snapshot'](expected,fd)
        current,_=namespace['serial_proof'](expected,fd)
        if (current.st_dev,current.st_ino,current.st_rdev)!=serial_node or namespace['serial_path'](expected)!=serial_path:raise ValueError('Pinned UART changed')
        mounts=namespace['bounded_read']('/proc/self/mountinfo',262144).splitlines()
        if [x for x in mounts if len(x.split())>4 and x.split()[4]=='/dev/shm']!=pin['shm_mount']:raise ValueError('RAM mount changed')
        if [x for x in mounts if len(x.split())>4 and x.split()[2]=='179:1']!=deployment['config_mount_review']:raise ValueError('CONFIG mount changed')
        return proof
    adapt_helpers(guard,namespace)
    return namespace,expected,health
try:
    namespace,expected,health=prepare_namespace()
    result=run_probe(namespace,expected,health)
except BaseException as error:
    result=dict(kind='off_servo_goal_probe',success=False,error=str(error) or type(error).__name__,error_type=type(error).__name__,serial_closed=True,diagnostic_attempts=0,motion_authorized=False,correlated_with_previous_hold=False)
print(json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':')),flush=True)
raise SystemExit(0 if result.get('success') and result.get('serial_closed') else 20)
'''


def build_remote_source(installation_archive, closure_evidence, persistent_calibration_sha256, *, boot_id, cid, probe_id=11):
    """Generate reviewable source only; no network, process or UART operations."""
    if type(probe_id) is not int or probe_id not in MOTOR_IDS:
        raise ValueError('Probe ID must be one mapped motor ID')
    import io
    closure = json.loads(Path(closure_evidence).read_text())
    motion = closure['motion']
    proof = motion['closed_proof']
    if (motion['phase'] != 'closed' or motion['owns_connection'] is not False
            or motion.get('exit_reconciled') is not True
            or proof['release_confirmed'] is not True or proof['serial_closed'] is not True
            or proof['stop_error'] is not None or motion.get('control_owner') is not None):
        raise ValueError('Native closure evidence does not prove released, closed UART')
    if (closure['session']['status'] != 'closed' or closure['maintenance']['phase'] != 'closed'
            or closure['maintenance']['owns_connection'] is not False
            or any(closure[name].get('busy') for name in ('wizard', 'joint_wizard', 'joint_zero'))):
        raise ValueError('Calibration/maintenance has not closed')
    if len(persistent_calibration_sha256) != 64 or any(c not in '0123456789abcdef' for c in persistent_calibration_sha256):
        raise ValueError('Persistent calibration hash is required')
    with tarfile.open(installation_archive, 'r:gz') as archive:
        seal_raw = archive.extractfile('installation.json').read()
        seal = json.loads(seal_raw)
        installed = {name: archive.extractfile(name).read() for name in seal['files']}
    if any(hashlib.sha256(raw).hexdigest() != seal['files'][name] for name, raw in installed.items()):
        raise ValueError('Local installation archive seal differs')
    deployment = json.loads(installed['deployment.json'])
    with tarfile.open(fileobj=io.BytesIO(installed['native-bundle.tar.gz']), mode='r:gz') as bundle:
        guard = bundle.extractfile('service/guard-runtime.py').read() + b'\n' + installed['persistent_boot.py']
    pins = dict(installation_seal=hashlib.sha256(seal_raw).hexdigest(), installation_files=seal['files'],
                bundle_files=deployment['files'], guard_sha=hashlib.sha256(guard).hexdigest(),
                boot_id=boot_id, cid=cid, persistent_calibration_sha256=persistent_calibration_sha256,
                live_calibration_sha256=motion['deployment']['calibration_sha256'],
                closure_evidence_sha256=hashlib.sha256(Path(closure_evidence).read_bytes()).hexdigest())
    # Check exact helper transformation during local generation, without I/O.
    adapt_helpers(guard.decode(), {}, probe_id=probe_id)
    decoder = Path(__file__).with_name('servo_goal_diagnostics.py').read_text()
    definitions = '\n\n'.join(inspect.getsource(fn) for fn in (adapt_helpers, checked_off_state, run_probe))
    setup = REMOTE_SETUP.replace('adapt_helpers(guard,namespace)',
        'adapt_helpers(guard,namespace,probe_id='+repr(probe_id)+')').replace(
        'result=run_probe(namespace,expected,health)',
        'result=run_probe(namespace,expected,health,probe_id='+repr(probe_id)+')')
    source = decoder + '\nimport ast,math,os,time\nPINS=' + repr(pins) + '\n' + definitions + '\n' + setup
    compile(source, '<generated-OFF-goal-probe>', 'exec')
    return source


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--installation-archive', required=True)
    parser.add_argument('--closure-evidence', required=True)
    parser.add_argument('--persistent-calibration-sha256', required=True)
    parser.add_argument('--boot-id', required=True)
    parser.add_argument('--cid', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--id',type=int,choices=MOTOR_IDS,default=11)
    args = parser.parse_args()
    source = build_remote_source(args.installation_archive, args.closure_evidence,
                                 args.persistent_calibration_sha256, boot_id=args.boot_id, cid=args.cid,probe_id=args.id)
    Path(args.output).write_text(source)
    print(json.dumps(dict(source_path=str(Path(args.output).resolve()), sha256=hashlib.sha256(source.encode()).hexdigest(),
                          generated_only=True, uart_opened=False)))
