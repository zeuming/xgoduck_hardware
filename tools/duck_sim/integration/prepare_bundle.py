"""Build an X5-local copy without changing the developer's running sim."""
from pathlib import Path
import io,tarfile
ROOT=Path(__file__).resolve().parents[3]
values={}
for relative in ['tools/duck_sim','output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck','output/xgoduck_rl_reference/vendor/bam/bam']:
 for p in (ROOT/relative).rglob('*'):
  if p.is_file() and not any(x in p.parts for x in ['.venv','__pycache__','.git','integration']):values[str(p.relative_to(ROOT))]=p.read_bytes()
for relative in ['tools/x5_policy/rl_core.py','tools/x5_policy/audit_policy_sim.py','output/x5_policy_bundle/models/xgoduck_walk.onnx','output/x5_gait_trial/20261009/forward-immediate-collapse-v8/closed-state-and-traces.json']:
 p=ROOT/relative
 if p.exists(): values[relative]=p.read_bytes()
# Relative assets preserve both a standalone root and the /sim/ mount.
n='tools/duck_sim/index.html';s=values[n].decode().replace('href="/style.css"','href="./style.css"').replace('"/vendor/','"./vendor/').replace('src="/app.js"','src="./app.js"')
s=s.replace('<nav aria-label="工作模式">','<a href="/" target="_blank" rel="noopener" style="color:inherit">实机工作台 ↗</a><nav aria-label="工作模式">')
values[n]=s.encode()
n='tools/duck_sim/app.js';s=values[n].decode().replace("fetch(url,", "fetch('.'+url,").replace("loader.loadAsync('/assets/'", "loader.loadAsync('./assets/'")
s=s.replace("const options=params?", "const csrf=params?(await (await fetch('/api/live-status',{credentials:'same-origin'})).json()).token:null;\n    const options=params?")
s=s.replace("headers:{'Content-Type':'application/json'}", "headers:{'Content-Type':'application/json','X-CSRF-Token':csrf}")
values[n]=s.encode()
n='tools/duck_sim/live.py';s=values[n].decode();a=s.index("            command = ['ssh'");b=s.index('            self.process = subprocess.Popen',a)
s=s[:a]+"            command = ['/usr/bin/python3', '-u', '-']\n"+s[b:];values[n]=s.encode()
for n,b in values.items():
 if n.endswith('.py'):compile(b,n,'exec')
buf=io.BytesIO()
with tarfile.open(fileobj=buf,mode='w:gz') as t:
 for n,b in sorted(values.items()):
  m=tarfile.TarInfo(n);m.size=len(b);m.mode=0o600;t.addfile(m,io.BytesIO(b))
out=ROOT/'output/duck_sim/8767-integration';out.mkdir(parents=True,exist_ok=True)
(out/'sim.tar.gz').write_bytes(buf.getvalue());print(out/'sim.tar.gz',len(buf.getvalue()))
