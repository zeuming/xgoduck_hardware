"""One explicit SSH/X5 application update with durable Mac backup and readback.

Importing this file does not connect, open UART or write firmware. The caller
must confirm disconnected servo power. Uses the pinned current X5 boot/card,
the installed USB/kernel guards and one exclusive CH340 open. No motion
command, flash retry, whole-chip erase, partition or bootloader write exists.
"""
from __future__ import annotations
import argparse
import ast
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tarfile
import tempfile
import time

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/x5_recovery/replacement-board-20261004T011959+0800'
PACKAGE=ROOT/'output/esp32_motion/firmware-first-stop-diagnostics-20261008-01'
EXPECTED_APP='ac4673e728c909c6109b281019bb26b121646ecec5f7dec39d204e58071423ad'
EXPECTED_OLD='db035fc0dad10c16d858930d411a4b655993b8c4f352b4189b43be18a1a5b91c'
EXPECTED_BOOT='76f869e0-d564-4d44-8c8e-201be3d7a6a1'
EXPECTED_CID='035344534b33324785d55cb687017c00'

REMOTE=r'''
import base64,contextlib,fcntl,hashlib,io,json,os,select,signal,stat,struct,subprocess,sys,tarfile,tempfile,time
from pathlib import Path
out=sys.stdout
def emit(value):print(json.dumps(value,separators=(',',':')),file=out,flush=True)
def digest(value):return hashlib.sha256(value).hexdigest()
_sha=digest
def private(path,value):
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
 with os.fdopen(fd,'wb') as f:f.write(value);f.flush();os.fsync(f.fileno())
def read_input(count,deadline):
 value=bytearray()
 while len(value)<count:
  if time.monotonic()>deadline:raise TimeoutError('Flash input deadline expired')
  if not select.select([sys.stdin.fileno()],[],[],.2)[0]:continue
  data=os.read(sys.stdin.fileno(),min(count-len(value),65536))
  if not data:raise ValueError('Flash input closed')
  value.extend(data)
 return bytes(value)
def read_confirmation(deadline):
 value=bytearray()
 while b'\n' not in value:
  value.extend(read_input(1,deadline))
  if len(value)>256:raise ValueError('Backup confirmation exceeds bound')
 return json.loads(value)
assert os.geteuid()==0
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==BOOT
assert Path('/sys/block/mmcblk1/device/cid').read_text().strip()==CID
assert Path('/proc/device-tree/model').read_bytes().rstrip(b'\0')==b'D-Robotics RDK X5 V1.0'
installation=Path('/opt/xgoduck-workbench')
assert digest((installation/'installation.json').read_bytes())==INSTALL_SEAL
seal=json.loads((installation/'installation.json').read_text())
for name,sha in seal['files'].items():assert digest((installation/name).read_bytes())==sha
pid=int(subprocess.run(['systemctl','show','xgoduck-workbench','--property=MainPID','--value'],check=True,capture_output=True,text=True).stdout)
children=[]
processes=[p for p in Path('/proc').iterdir() if p.name.isdecimal()]
assert len(processes)<4096
for process in processes:
 try:
  fields=(process/'stat').read_text().rsplit(')',1)[1].split()
  if int(fields[1])==pid:children.append(process.name)
 except FileNotFoundError:pass
assert len(children)==1
root=Path(os.path.realpath('/proc/'+children[0]+'/cwd'))
assert root.parent==Path('/dev/shm') and root.name.startswith('xgoduck-native-')
pin=json.loads((root/'native-pin.json').read_text())
guard=(root/'service/guard-runtime.py').read_bytes()
assert digest(guard)==pin['files']['service/guard-runtime.py']['sha256']
namespace={};exec(compile(guard,'<installed-flash-guard>','exec'),namespace)
expected=json.loads((root/'service/expected.json').read_text())
assert expected['boot_id']==BOOT and expected['cid']==CID
namespace['bind_serial'](expected)
sys.path.insert(0,str(root/'service'));import native_transports
recovery=native_transports.prepare_usb_recovery(namespace,expected)
native_transports.install_usb_recovery(namespace,'',recovery)
namespace['health_snapshot'](expected)
node,_=namespace['serial_proof'](expected)
assert not namespace['_holders'](node.st_rdev,time.monotonic()+.2)
emit({'phase':'transfer_ready','bytes':SIZE,'motor_motion_authorized':False})
raw=read_input(SIZE,time.monotonic()+30);assert digest(raw)==BUNDLE_SHA
directory=Path(tempfile.mkdtemp(prefix='xgoduck-flash-',dir='/dev/shm'));directory.chmod(0o700)
with tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz') as archive:
 members=archive.getmembers()
 assert len(members)==len(FILES) and {m.name for m in members}==set(FILES)
 for item in members:
  assert item.isfile() and not item.name.startswith('/') and '..' not in Path(item.name).parts
  info=FILES[item.name];assert item.size==info['size']
  data=archive.extractfile(item).read();assert digest(data)==info['sha256']
  target=directory/item.name;target.parent.mkdir(mode=0o700,parents=True,exist_ok=True);private(target,data)
sys.path[:0]=[str(directory/'site')]
# Use an explicit empty esptool configuration rather than host retry settings.
private(directory/'esptool.cfg',b'[esptool]\n')
os.environ['ESPTOOL_CFGFILE']=str(directory/'esptool.cfg')
import serial,esptool
from esptool import cmds,loader as loader_module
from esptool.targets.esp32s3 import ESP32S3ROM
from esptool.reset import UnixTightReset,HardReset
assert esptool.__version__=='4.9.0' and serial.VERSION=='3.5'
assert Path(esptool.__file__).resolve().parent==directory/'site/esptool'
loader_module.SYNC_TIMEOUT=.5
exec(HELPERS,globals())
assert digest((directory/'firmware.bin').read_bytes())==APP_SHA256
assert len((directory/'firmware.bin').read_bytes())==APP_SIZE
stream=None;stopped=False;success=False;report={'phase':'prepared','write_attempted':False,'readback_verified':False,'serial_closed':False,'automatic_motor_start':False,'ram_directory':str(directory)}
log=open(directory/'loader.log','w',encoding='utf-8')
def phase(name,**extra):
 report.update(phase=name,**{k:v for k,v in extra.items() if not k.endswith('_base64')})
 emit(dict(phase=name,**extra))
def timeout(*_):raise TimeoutError('Single flash attempt exceeded 180 seconds')
signal.signal(signal.SIGALRM,timeout);signal.setitimer(signal.ITIMER_REAL,180)
try:
 # Check the web owner immediately before quiescing it. Keeping the service
 # stopped prevents any tab from taking the serial port during the update.
 import http.cookiejar,urllib.request
 client=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
 client.open('http://192.168.1.11:8767/',timeout=5).read(2097153)
 snapshot=json.load(client.open('http://192.168.1.11:8767/api/status',timeout=5))
 assert snapshot['motion']['owns_connection'] is False and snapshot['maintenance']['owns_connection'] is False
 assert snapshot['session']['status'] in ('closed','fault') and not snapshot['session']['recovery_pending']
 assert not any(snapshot.get(k,{}).get('busy') for k in ('maintenance','wizard','joint_wizard','joint_zero'))
 subprocess.run(['systemctl','stop','xgoduck-workbench'],check=True,capture_output=True,timeout=25);stopped=True
 namespace['health_snapshot'](expected)
 node,_=namespace['serial_proof'](expected)
 assert not namespace['_holders'](node.st_rdev,time.monotonic()+.2)
 stream=serial.Serial(port=None,baudrate=115200,timeout=3,write_timeout=3,exclusive=True)
 stream.dtr=False;stream.rts=False;stream.port=namespace['serial_path'](expected)
 phase('opening_single_loader');stream.open()
 namespace['serial_proof'](expected,stream.fileno());fcntl.ioctl(stream.fileno(),__import__('termios').TIOCEXCL)
 assert not [x for x in namespace['_holders'](node.st_rdev,time.monotonic()+.2) if x['pid']!=os.getpid()]
 with contextlib.redirect_stdout(log):
  loader=_single_loader_class(ESP32S3ROM,UnixTightReset)(stream,baud=115200,trace_enabled=False)
  loader.connect(mode='default_reset',attempts=1,detecting=False)
  assert not loader.secure_download_mode and loader.get_chip_id()==9 and loader.CHIP_NAME=='ESP32-S3'
  mac=':'.join('%02x'%x for x in loader.read_mac());assert mac=='44:bd:8d:6a:6c:00'
  assert not loader.get_secure_boot_enabled() and not loader.get_flash_encryption_enabled()
  stub=loader.run_stub();assert stub._port is stream and stub.IS_STUB
  flash_id=stub.flash_id();assert cmds.DETECTED_FLASH_SIZES.get(flash_id>>16)=='16MB'
  stub.flash_set_parameters(16*1024*1024);stub.change_baud(460800)
 namespace['health_snapshot'](expected,stream.fileno())
 phase('controller_verified',mac=mac,chip='ESP32-S3',flash_id=flash_id)
 with contextlib.redirect_stdout(log):partition=stub.read_flash(0x8000,SECTOR_SIZE)
 _check_partition(partition);private(directory/'original-partition-sector.bin',partition)
 phase('backing_up_app')
 with contextlib.redirect_stdout(log):backup=stub.read_flash(APP_OFFSET,REGION_SIZE)
 assert len(backup)==REGION_SIZE and digest(backup[:OLD_SIZE])==OLD_SHA256
 private(directory/'original-app-region.bin',backup)
 # The Mac must fsync the exact backup before the first erase/write. EOF,
 # malformed confirmation or a lost SSH session cannot grant this step.
 phase('backup_ready',backup_sha256=digest(backup),backup_bytes=len(backup),backup_base64=base64.b64encode(backup).decode(),partition_base64=base64.b64encode(partition).decode())
 confirmation=read_confirmation(time.monotonic()+20)
 assert confirmation=={'backup_persisted_sha256':digest(backup)}
 namespace['health_snapshot'](expected,stream.fileno())
 image=(directory/'firmware.bin').read_bytes();payload=image+backup[APP_SIZE:]
 phase('writing_app',write_attempted=True,application_size=APP_SIZE,write_extent_end=APP_OFFSET+REGION_SIZE)
 with contextlib.redirect_stdout(log):_write_app_once(stub,payload)
 phase('reading_back_app')
 with contextlib.redirect_stdout(log):readback=stub.read_flash(APP_OFFSET,REGION_SIZE)
 assert len(readback)==REGION_SIZE and readback==payload and digest(readback[:APP_SIZE])==APP_SHA256
 namespace['health_snapshot'](expected,stream.fileno())
 phase('verified',readback_verified=True,app_sha256=APP_SHA256,region_sha256=digest(readback),preserved_tail_bytes=REGION_SIZE-APP_SIZE)
 with contextlib.redirect_stdout(log):HardReset(stream).reset()
 report['app_start_requested']=True
 # Boot ROM prints at 115200 after this explicit verified reset. Archive its
 # finite output on this same owned descriptor before a strict motion reader
 # takes over at 921600; never discard bytes in a regular motion connection.
 stream.baudrate=115200;stream.timeout=.05
 boot_output=bytearray();boot_deadline=time.monotonic()+2.5
 while time.monotonic()<boot_deadline:
  boot_output.extend(stream.read(min(512,max(1,stream.in_waiting))))
  if len(boot_output)>8192:raise ValueError('Unexpected continuous boot UART output')
 private(directory/'verified-app-boot-uart.bin',bytes(boot_output))
 phase('verified_app_started',boot_uart_bytes=len(boot_output),boot_uart_base64=base64.b64encode(boot_output).decode())
 success=True
except BaseException as error:
 report.update(phase='failed',error=str(error) or type(error).__name__);emit(report)
finally:
 signal.setitimer(signal.ITIMER_REAL,0)
 if stream is not None:stream.close()
 report['serial_closed']=stream is None or not stream.is_open
 if stopped:
  restart=subprocess.run(['systemctl','start','xgoduck-workbench'],capture_output=True,text=True,timeout=30)
  report['workbench_restart_ok']=restart.returncode==0
  if restart.returncode:report['workbench_restart_error']=restart.stderr[-2000:]
 log.close()
 private(directory/'report.json',(json.dumps(report,indent=2)+'\n').encode())
 emit(dict(phase='closed',report=report,loader_log=(directory/'loader.log').read_text()[-16000:]))
raise SystemExit(0 if success and report['serial_closed'] and report.get('workbench_restart_ok') else 20)
'''

def private(path,data):
    descriptor=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
    with os.fdopen(descriptor,'wb') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())

def bundle():
    files={}
    for prefix,source in [('site/esptool',Path('/tmp/xgoduck-pio-core/packages/tool-esptoolpy/esptool')),
                          ('site/intelhex',Path('/tmp/xgoduck-pio-core/packages/tool-esptoolpy/_contrib/intelhex')),
                          ('site/serial',Path('/tmp/xgoduck-timing-pio/lib/python3.12/site-packages/serial'))]:
        for path in sorted(source.rglob('*')):
            if path.is_file() and not path.is_symlink() and '__pycache__' not in path.parts and path.suffix in ('.py','.json'):
                files[prefix+'/'+path.relative_to(source).as_posix()]=path.read_bytes()
    files['firmware.bin']=(PACKAGE/'firmware.bin').read_bytes()
    assert hashlib.sha256(files['firmware.bin']).hexdigest()==EXPECTED_APP
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
        for name,data in files.items():
            item=tarfile.TarInfo(name);item.size=len(data);item.mode=0o600
            archive.addfile(item,io.BytesIO(data))
    return buffer.getvalue(),{name:dict(size=len(data),sha256=hashlib.sha256(data).hexdigest()) for name,data in files.items()}

def prepare_source(payload,files):
    # Reuse exact checked partition/bounded block writer and one-open reset
    # implementation from the already-reviewed, verified Mac updater.
    previous=(ROOT/'output/esp32_motion/stand-feedback-timing-20261008/update_app_once.py').read_text()
    wanted={'_single_loader_class','_check_partition','_write_app_once'}
    helpers='\n\n'.join(ast.get_source_segment(previous,node) for node in ast.parse(previous).body
                         if isinstance(node,ast.FunctionDef) and node.name in wanted)
    assert len([n for n in ast.parse(helpers).body if isinstance(n,ast.FunctionDef)])==3
    install=max((p for p in BASE.glob('native-service-install-*') if (p/'result.json').exists()),key=lambda p:p.stat().st_mtime)
    with tarfile.open(install/'installation.tar.gz','r:gz') as archive:
        seal=hashlib.sha256(archive.extractfile('installation.json').read()).hexdigest()
    app_size=(PACKAGE/'firmware.bin').stat().st_size
    constants=dict(BOOT=EXPECTED_BOOT,CID=EXPECTED_CID,INSTALL_SEAL=seal,SIZE=len(payload),
        BUNDLE_SHA=hashlib.sha256(payload).hexdigest(),FILES=files,HELPERS=helpers,
        APP_OFFSET=0x10000,APP_SIZE=app_size,REGION_SIZE=(app_size+4095)//4096*4096,SECTOR_SIZE=4096,
        APP_SHA256=EXPECTED_APP,OLD_SIZE=288656,OLD_SHA256=EXPECTED_OLD,
        PARTITION_SHA256='13daf80a09a7a4ee743883aed6779c6833ec4880709ce967a5799c0bd38c06e0')
    source='\n'.join(name+' = '+repr(value) for name,value in constants.items())+'\n'+REMOTE
    compile(source,'<x5-single-flash>','exec')
    return source.encode()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motors-power-disconnected',action='store_true',required=True)
    args=parser.parse_args()
    sys.path[:0]=[str(ROOT/'tools/x5_recovery'),str(ROOT/'tools/x5_policy')]
    import maintenance_transport as transport
    from transport_config import TransportConfig
    payload,files=bundle();source=prepare_source(payload,files)
    directory=Path(tempfile.mkdtemp(prefix='ssh-flash-first-stop-',dir=BASE));directory.chmod(0o700)
    private(directory/'source.py',source);private(directory/'bundle.tar.gz',payload)
    print(json.dumps({'evidence':str(directory),'phase':'prepared','bytes':len(payload)}),flush=True)
    channel=transport._ProcessChannel(source,directory,BASE/'known_hosts',transport.helpers(),
        transport_config=TransportConfig(kind='wifi',host='192.168.1.11',interface='en0'))
    records=[];closed=None
    try:
        deadline=time.monotonic()+210
        first=channel.receive(time.monotonic()+15)
        assert first.get('phase')=='transfer_ready' and first['bytes']==len(payload)
        transport.helpers()['write_pipe_once'](channel.process.stdin.fileno(),payload,30)
        while time.monotonic()<deadline:
            record=channel.receive(min(deadline,time.monotonic()+55))
            if record.get('phase')=='backup_ready':
                backup=base64.b64decode(record.pop('backup_base64'),validate=True)
                partition=base64.b64decode(record.pop('partition_base64'),validate=True)
                assert len(backup)==record['backup_bytes'] and hashlib.sha256(backup).hexdigest()==record['backup_sha256']
                private(directory/'original-app-region.bin',backup)
                private(directory/'original-partition-sector.bin',partition)
                assert hashlib.sha256((directory/'original-app-region.bin').read_bytes()).hexdigest()==record['backup_sha256']
                channel.send({'backup_persisted_sha256':record['backup_sha256']},5)
            if record.get('phase')=='verified_app_started':
                boot=base64.b64decode(record.pop('boot_uart_base64'),validate=True)
                assert len(boot)==record['boot_uart_bytes']
                private(directory/'verified-app-boot-uart.bin',boot)
            records.append(record)
            private(directory/('record-%03d.json'%len(records)),(json.dumps(record,indent=2)+'\n').encode())
            print(json.dumps({k:v for k,v in record.items() if k not in ('loader_log','report')}),flush=True)
            if record.get('phase')=='closed':closed=record;break
        if closed is None:raise TimeoutError('No actual flash closure report')
        channel.process.stdin.close();channel.process.wait(timeout=5)
        private(directory/'stderr.txt',bytes(channel.stderr))
        report=closed['report'];private(directory/'report.json',(json.dumps(report,indent=2)+'\n').encode())
        print(json.dumps({'evidence':str(directory),'phase':'complete' if report.get('readback_verified') and channel.process.returncode==0 else 'failed','readback_verified':report.get('readback_verified'),'serial_closed':report['serial_closed'],'service_restarted':report.get('workbench_restart_ok')}),flush=True)
        if channel.process.returncode!=0:raise RuntimeError(report.get('error') or 'X5 flash failed')
    except BaseException as error:
        private(directory/'failure.json',(json.dumps({'error':str(error),'type':type(error).__name__},indent=2)+'\n').encode())
        private(directory/'stderr-at-failure.txt',bytes(channel.stderr))
        raise
    finally:
        channel.finish()

if __name__=='__main__':main()
