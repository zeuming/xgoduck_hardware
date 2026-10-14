"""Prepare an explicitly battery-disconnected variant of a pinned installer.

Local packaging only. The user's physical disconnect confirmation replaces
the inapplicable powered-servo OFF-readback prerequisite, not device identity,
UART ownership, MCU-off state, backup, one-write or readback verification.
No absent servo reply is promoted to measured torque OFF.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

GATE = '''
def battery_disconnected_gate(namespace, expected):
    # Human confirmation is the physical power evidence; UART only establishes
    # that the controller is unarmed. No servo torque measurement is claimed.
    port = RawPort(namespace['serial_path'](expected))
    proof = dict(battery_disconnected=True,
        battery_state_source='explicit user confirmation 2026-10-14',
        fresh_all_torque_registers_known_off=None,
        motion_commands_sent=False, read_request_types=[], samples=[])
    try:
        namespace['serial_proof'](expected, port.fileno())
        fcntl.flock(port.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.ioctl(port.fileno(),termios.TIOCEXCL)
        before=namespace['_modem_bits'](port.fileno())
        namespace['_raw_settings'](port.fileno())
        assert namespace['_modem_bits'](port.fileno())==before
        node,_=namespace['serial_proof'](expected,port.fileno())
        assert not [x for x in namespace['_holders'](node.st_rdev,time.monotonic()+.2) if x['pid']!=os.getpid()]
        for seq in range(1,4):
            raw,ms=exchange(port,2,seq)
            row=state(raw)
            assert not row['armed'] and not row['arm_cookie'], 'Active MCU; flash not allowed'
            proof['read_request_types'].append(2)
            proof['samples'].append(dict(row,request_ms=ms))
            time.sleep(.1)
        namespace['health_snapshot'](expected,port.fileno())
        proof['passed']=True
    finally:
        port.close()
        proof['serial_closed']=not port.is_open
    return proof
'''


def prepare(source, destination):
    report=json.loads((source/'preparation.json').read_text())
    src=(source/'reviewed-flash-source.py').read_text()
    sha=lambda b:hashlib.sha256(b).hexdigest()
    assert sha(src.encode())==report['flash_source_sha256']
    assert sha((source/'reviewed-flash-bundle.tar.gz').read_bytes())==report['flash_bundle_sha256']
    before=" proof=powered_off_gate(namespace,expected)\n phase('powered_off_gate_passed',powered_off_proof=proof)"
    assert src.count(before)==1
    src=src.replace(before," exec("+repr(GATE)+",globals())\n proof=battery_disconnected_gate(namespace,expected)\n phase('battery_disconnected_gate_passed',battery_disconnected_proof=proof)")
    compile(src,'<battery-disconnected-flash>','exec')
    tree=ast.parse(src)
    assert sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='_write_app_once' for n in ast.walk(tree))==1
    destination.mkdir(exist_ok=False)
    for name in ('reviewed-flash-bundle.tar.gz','reviewed-host-install.py','host.tar.gz'):
        shutil.copyfile(source/name,destination/name)
    (destination/'reviewed-flash-source.py').write_text(src)
    report.update(flash_source_sha256=sha(src.encode()),battery_disconnected_confirmed=True,
        battery_confirmation='User answered 电池已断开 on 2026-10-14',
        preflash_proof='User-confirmed physical power removal plus MCU unarmed; NOT measured 15-servo OFF')
    (destination/'preparation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('firmware_sha256','flash_source_sha256','battery_disconnected_confirmed')}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared-install',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--user-confirmed-servo-battery-disconnected',action='store_true',required=True)
    a=p.parse_args()
    prepare(a.prepared_install,a.output)
