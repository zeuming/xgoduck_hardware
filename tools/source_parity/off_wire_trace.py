"""Authorized fixed-robot OFF communication trace; no ARM or TARGET.

Uses the installed pinned guard, current host baud and production frame parser.
HELLO and STOP establish OFF observation. Stops on the first failed transaction.
"""
def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--authorized-fixed-robot',action='store_true',required=True)
    parser.parse_args()
    import sys,os,time,json,hashlib,fcntl,termios,subprocess
    from pathlib import Path
    pid=int(subprocess.check_output(["systemctl","show","xgoduck-workbench","--property=MainPID","--value"]))
    children=[]
    for p in Path("/proc").iterdir():
     if p.name.isdecimal():
      try:
       if int((p/"stat").read_text().rsplit(")",1)[1].split()[1])==pid:children.append(p)
      except FileNotFoundError:pass
    assert len(children)==1
    root=Path(os.path.realpath(children[0]/"cwd"));pin=json.loads((root/"native-pin.json").read_text());g=(root/"service/guard-runtime.py").read_bytes();assert hashlib.sha256(g).hexdigest()==pin["files"]["service/guard-runtime.py"]["sha256"]
    ns={};exec(compile(g,"<pinned-guard>","exec"),ns);expected=json.loads((root/"service/expected.json").read_text());ns["bind_serial"](expected)
    sys.path[:0]=[str(root/"policy"),str(root/"service")]
    import native_transports
    recovery=native_transports.prepare_usb_recovery(ns,expected);native_transports.install_usb_recovery(ns,"",recovery)
    ns["health_snapshot"](expected);node,_=ns["serial_proof"](expected);assert not ns["_holders"](node.st_rdev,time.monotonic()+.2)
    from production_motion import SerialTransport
    trace=[]
    class Port:
     def __init__(self,p):self.p=p
     def __getattr__(self,k):return getattr(self.p,k)
     def read(self,n):
      b=self.p.read(n)
      if b:trace.append(dict(at=time.monotonic(),rx=b.hex()))
      return b
    transport=SerialTransport(ns["serial_path"](expected));transport.open();fd=transport.serial.fileno();ns["serial_proof"](expected,fd);fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.ioctl(fd,termios.TIOCEXCL);transport.serial=Port(transport.serial);time.sleep(.75)
    try:
     for kind in (1,6,30):
      result=transport.request(kind);print(json.dumps(dict(event="setup",kind=kind,response=result.hex())),flush=True)
     for i in range(150):
      trace.clear();began=time.monotonic()
      try:
       r=transport.request(32);print(json.dumps(dict(event="sample",index=i,sequence=transport.seq,elapsed_ms=(time.monotonic()-began)*1000,length=len(r),trace=trace)),flush=True)
      except Exception as e:
       print(json.dumps(dict(event="failure",index=i,sequence=transport.seq,error=str(e),parser_buffer=transport.parser.buffer.hex(),trace=trace,timing=transport.timing_diagnostics())),flush=True);raise
      time.sleep(.02)
    finally:
     transport.close();print(json.dumps(dict(event="closed",serial_closed=True,no_ARM_or_TARGET_sent=True)),flush=True)

if __name__=='__main__':main()
