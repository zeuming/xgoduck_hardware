"""Run one pinned firmware update after explicit fixed-robot authorization.

The remote script proves actual torque OFF before resetting the MCU. It emits
a complete flash backup which this process fsyncs before authorizing the write.
No retries, motion commands, bootloader/partition writes or calibration edits.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import threading
import time


def private(path, data):
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'wb') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install',type=Path,required=True)
    parser.add_argument('--ssh-master',required=True)
    parser.add_argument('--host',required=True)
    parser.add_argument('--authorized-fixed-robot',action='store_true',required=True)
    args=parser.parse_args()
    expected=json.loads((args.install/'preparation.json').read_text())
    source=(args.install/'reviewed-flash-source.py').read_bytes()
    payload=(args.install/'reviewed-flash-bundle.tar.gz').read_bytes()
    digest=lambda data:hashlib.sha256(data).hexdigest()
    assert digest(source)==expected['flash_source_sha256']
    assert digest(payload)==expected['flash_bundle_sha256']
    evidence=args.install/'flash-execution';evidence.mkdir(mode=0o700)
    command="import base64;exec(compile(base64.b64decode("+repr(base64.b64encode(source).decode())+"),'<reviewed-source-parity-flash>','exec'))"
    process=subprocess.Popen(['ssh','-S',args.ssh_master,'-o','BatchMode=yes',args.host,
                              'python3 -u -c '+shlex.quote(command)],
                             stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    incoming=queue.Queue();errors=[]
    def reader():
        for line in process.stdout:incoming.put(line)
        incoming.put(None)
    def err_reader():errors.append(process.stderr.read())
    threading.Thread(target=reader,daemon=True).start()
    threading.Thread(target=err_reader,daemon=True).start()
    end=time.monotonic()+225;records=[];closed=None
    try:
        first=incoming.get(timeout=20)
        if first is None:raise RuntimeError('Remote ended before transfer readiness')
        record=json.loads(first)
        assert record['phase']=='transfer_ready' and record['bytes']==len(payload)
        process.stdin.write(payload);process.stdin.flush()
        print(json.dumps(record),flush=True)
        while time.monotonic()<end:
            line=incoming.get(timeout=min(55,max(.1,end-time.monotonic())))
            if line is None:break
            record=json.loads(line)
            if record['phase']=='backup_ready':
                backup=base64.b64decode(record.pop('backup_base64'),validate=True)
                partition=base64.b64decode(record.pop('partition_base64'),validate=True)
                assert len(backup)==record['backup_bytes'] and digest(backup)==record['backup_sha256']
                private(evidence/'original-app-region.bin',backup)
                private(evidence/'original-partition-sector.bin',partition)
                assert digest((evidence/'original-app-region.bin').read_bytes())==record['backup_sha256']
                process.stdin.write((json.dumps({'backup_persisted_sha256':record['backup_sha256']})+'\n').encode())
                process.stdin.flush()
            if record['phase']=='verified_app_started':
                boot=base64.b64decode(record.pop('boot_uart_base64'),validate=True)
                assert len(boot)==record['boot_uart_bytes']
                private(evidence/'verified-app-boot-uart.bin',boot)
            records.append(record)
            private(evidence/('record-%03d.json'%len(records)),(json.dumps(record,indent=2)+'\n').encode())
            print(json.dumps({k:v for k,v in record.items() if k not in ('report','loader_log')}),flush=True)
            if record['phase']=='closed':closed=record;break
        process.stdin.close();process.wait(timeout=10)
        if closed is None:raise RuntimeError('No actual update closure report')
        report=closed['report']
        private(evidence/'report.json',(json.dumps(report,indent=2)+'\n').encode())
        assert process.returncode==0 and report['readback_verified'] and report['serial_closed'] and report['calibration_unchanged']
        print(json.dumps({'phase':'complete','readback_verified':True,'serial_closed':True}),flush=True)
    finally:
        if process.poll() is None:
            process.stdin.close() if not process.stdin.closed else None
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:process.terminate()
        private(evidence/'stderr.txt',b''.join(errors))


if __name__=='__main__':main()
