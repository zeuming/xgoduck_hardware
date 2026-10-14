import pathlib,tarfile,io,sys,subprocess
root=pathlib.Path('/opt/xgoduck-sim/app');root.mkdir(exist_ok=True)
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz') as t:
 for m in t:
  p=pathlib.PurePosixPath(m.name)
  assert m.isfile() and not p.is_absolute() and '..' not in p.parts
  f=root/p;f.parent.mkdir(parents=True,exist_ok=True);f.write_bytes(t.extractfile(m).read());f.chmod(0o600)
unit='''[Unit]
Description=XGO Duck simulation backend
After=network.target
[Service]
Type=simple
WorkingDirectory=/opt/xgoduck-sim/app
Environment=PYTHONPATH=/opt/xgoduck-sim/deps
Environment=OMP_NUM_THREADS=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=MUJOCO_GL=egl
ExecStart=/usr/bin/python3 -B /opt/xgoduck-sim/app/tools/duck_sim/server.py --port 8788
Restart=on-failure
RestartSec=5
Nice=10
CPUQuota=100%
MemoryMax=1G
[Install]
WantedBy=multi-user.target
'''
pathlib.Path('/etc/systemd/system/xgoduck-sim.service').write_text(unit)
subprocess.run(['systemctl','daemon-reload'],check=True)
subprocess.run(['systemctl','enable','--now','xgoduck-sim.service'],check=True)
print('simulation backend installed; no live-feedback connection requested')
