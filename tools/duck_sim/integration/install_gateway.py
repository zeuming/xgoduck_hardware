import pathlib,subprocess,tarfile,io,json,hashlib,shutil,time,os,stat
p=pathlib.Path('/opt/xgoduck-workbench');sha=lambda b:hashlib.sha256(b).hexdigest()
# Refuse restart if a process currently owns the servo UART.
uart=pathlib.Path('/dev/ttyUSB0')
if uart.exists():
 rdev=uart.stat().st_rdev
 for proc in pathlib.Path('/proc').iterdir():
  if not proc.name.isdecimal():continue
  try:fds=list((proc/'fd').iterdir())
  except OSError:continue
  for fd in fds:
   try:s=fd.stat()
   except OSError:continue

   if stat.S_ISCHR(s.st_mode) and s.st_rdev==rdev:
    assert 'xgoduck-workbench.service' not in (proc/'cgroup').read_text(),'Workbench owns UART; refusing restart'
    print('External UART owner preserved:',proc.name)
import urllib.request,http.cookiejar
client=urllib.request.build_opener(urllib.request.ProxyHandler({}),urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
client.open('http://127.0.0.1:8767/',timeout=3).read()
state=json.load(client.open('http://127.0.0.1:8767/api/status',timeout=3))
assert state['motion'].get('owns_connection') is False,'Workbench controls UART; postpone deployment'
assert state.get('session',{}).get('status') in ('closed','fault'),'Calibration session active; postpone deployment'
assert not state.get('maintenance',{}).get('busy'),'Maintenance active'
seal=json.loads((p/'installation.json').read_text())
for n,h in seal['files'].items():assert sha((p/n).read_bytes())==h,n
manifest=json.loads((p/'deployment.json').read_text());assert manifest['bind_host']=='0.0.0.0'
raw=(p/'native-bundle.tar.gz').read_bytes();assert sha(raw)==manifest['archive_sha256']
with tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz') as t:
 members=t.getmembers();data={m.name:t.extractfile(m).read() for m in members}
changed=['service/native_process.py','service/sim_web.py']
n='service/native_process.py'
source=data[n].decode()
assert 'import sim_web' not in source
source=source.replace('import native_workbench as web\n','import native_workbench as web\nimport sim_web\n',1)
assert 'self.page = log_toolbar(self.page)' in source
source=source.replace('self.page = log_toolbar(self.page)','self.page = sim_web.toolbar(log_toolbar(self.page))',1)
needle='''    class Handler(web.handler_for(frontend)):
        def do_GET(self):'''
assert source.count(needle)==1
source=source.replace(needle,'''    class Handler(web.handler_for(frontend)):
        def do_POST(self):
            if sim_web.route(self, frontend, web, post=True): return
            return super().do_POST()
        def do_GET(self):
            if sim_web.route(self, frontend, web): return''',1)
data[n]=source.encode()
data['service/sim_web.py']=b'"""Bounded same-origin gateway to the X5-local simulation service."""\nimport http.client\nimport secrets\nimport threading\n\n_slots = threading.BoundedSemaphore(6)\nGET_API = {\'/api/model\', \'/api/replays\', \'/api/live\'}\nPOST_API = {\'/api/simulate\', \'/api/live/connect\', \'/api/live/disconnect\'}\n\n\ndef toolbar(page):\n    marker = \'<nav aria-label="\xe9\x80\x89\xe6\x8b\xa9\xe5\xb7\xa5\xe4\xbd\x9c\xe5\x8f\xb0\xe5\x8a\x9f\xe8\x83\xbd">\'.encode()\n    if page.count(marker) != 1:\n        raise ValueError(\'Workbench navigation changed\')\n    link = \'<a class="nav-item" href="/sim/" target="_blank" rel="noopener"><span class="nav-icon">\xe2\x97\x87</span><span>\xe4\xbb\xbf\xe7\x9c\x9f\xe5\xb7\xa5\xe4\xbd\x9c\xe5\x8f\xb0 \xe2\x86\x97</span></a>\'\n    return page.replace(marker, marker + link.encode(), 1)\n\n\ndef route(handler, frontend, web, *, post=False):\n    if not (handler.path == \'/sim\' or handler.path.startswith(\'/sim/\')):\n        return False\n    acquired = False\n    connection = None\n    try:\n        size = handler.framing(post=post)\n        path = handler.path[4:] or \'/\'\n        if path == \'/\' and not handler.path.endswith(\'/\'):\n            handler.send_response(302); handler.send_header(\'Location\', \'/sim/\')\n            handler.send_header(\'Content-Length\', \'0\'); handler.end_headers()\n            return True\n        cookie = None\n        if not post and path == \'/\' and not frontend.authentication_required:\n            token, created = frontend.anonymous_session(handler.headers.get(\'Cookie\'))\n            if created: cookie = web.session_cookie(token)\n        elif not frontend.authenticated(handler.headers.get(\'Cookie\')):\n            handler.reply(401, dict(error=\'\xe8\xaf\xb7\xe5\x85\x88\xe6\x89\x93\xe5\xbc\x80\xe5\xb7\xa5\xe4\xbd\x9c\xe5\x8f\xb0\xe9\xa6\x96\xe9\xa1\xb5\xe5\xbb\xba\xe7\xab\x8b\xe4\xbc\x9a\xe8\xaf\x9d\xef\xbc\x8c\xe5\x86\x8d\xe6\x89\x93\xe5\xbc\x80\xe4\xbb\xbf\xe7\x9c\x9f\xe5\xb7\xa5\xe4\xbd\x9c\xe5\x8f\xb0\xe3\x80\x82\'))\n            return True\n        if post:\n            if path not in POST_API:\n                handler.reply(404, dict(error=\'\xe6\x9c\xaa\xe7\x9f\xa5\xe4\xbb\xbf\xe7\x9c\x9f\xe6\x93\x8d\xe4\xbd\x9c\')); return True\n            if not secrets.compare_digest(handler.headers.get(\'X-CSRF-Token\', \'\').encode(), frontend.token.encode()):\n                handler.reply(403, dict(error=\'\xe9\xa1\xb5\xe9\x9d\xa2\xe4\xbc\x9a\xe8\xaf\x9d\xe5\xb7\xb2\xe5\x8f\x98\xe5\x8c\x96\xef\xbc\x8c\xe8\xaf\xb7\xe5\x88\xb7\xe6\x96\xb0\')); return True\n        elif (path not in GET_API and path != \'/\' and\n              not (path.endswith((\'.js\', \'.css\', \'.stl\', \'.txt\')) and \'..\' not in path and \'%\' not in path)):\n            handler.reply(404, dict(error=\'\xe6\x9c\xaa\xe7\x9f\xa5\xe4\xbb\xbf\xe7\x9c\x9f\xe8\xb5\x84\xe6\xba\x90\')); return True\n        acquired = _slots.acquire(blocking=False)\n        if not acquired:\n            handler.reply(503, dict(error=\'\xe4\xbb\xbf\xe7\x9c\x9f\xe8\xaf\xb7\xe6\xb1\x82\xe7\xb9\x81\xe5\xbf\x99\xef\xbc\x8c\xe8\xaf\xb7\xe7\xa8\x8d\xe5\x90\x8e\xe9\x87\x8d\xe8\xaf\x95\')); return True\n        body = handler.rfile.read(size) if post else None\n        if post and len(body) != size: raise ValueError(\'\xe8\xaf\xb7\xe6\xb1\x82\xe6\xad\xa3\xe6\x96\x87\xe4\xb8\x8d\xe5\xae\x8c\xe6\x95\xb4\')\n        connection = http.client.HTTPConnection(\'127.0.0.1\', 8788, timeout=45 if path == \'/api/simulate\' else 5)\n        connection.request(\'POST\' if post else \'GET\', path, body=body,\n                           headers={\'Content-Type\': \'application/json\'} if post else {})\n        response = connection.getresponse()\n        payload = response.read(16 * 1024 * 1024 + 1)\n        if len(payload) > 16 * 1024 * 1024: raise ValueError(\'\xe4\xbb\xbf\xe7\x9c\x9f\xe5\x93\x8d\xe5\xba\x94\xe8\xbf\x87\xe5\xa4\xa7\')\n        handler.reply(response.status, payload, kind=response.getheader(\'Content-Type\', \'application/octet-stream\'), cookie=cookie)\n    except (OSError, ValueError, http.client.HTTPException):\n        handler.reply(503, dict(error=\'\xe4\xbb\xbf\xe7\x9c\x9f\xe6\x9c\x8d\xe5\x8a\xa1\xe6\x9a\x82\xe4\xb8\x8d\xe5\x8f\xaf\xe7\x94\xa8\xef\xbc\x8c\xe8\xaf\xb7\xe7\xa8\x8d\xe5\x90\x8e\xe9\x87\x8d\xe8\xaf\x95\xe3\x80\x82\'))\n    finally:\n        if connection: connection.close()\n        if acquired: _slots.release()\n    return True\n'
for n in changed:
 compile(data[n],n,'exec')
 manifest['required_source_sha256'][n]=sha(data[n])
 manifest['files'][n]=dict(size_bytes=len(data[n]),sha256=sha(data[n]))
if not any(m.name=='service/sim_web.py' for m in members):
 m=tarfile.TarInfo('service/sim_web.py');m.mode=0o600;members.append(m)
buf=io.BytesIO()
with tarfile.open(fileobj=buf,mode='w:gz') as t:
 for m in members:m.size=len(data[m.name]);t.addfile(m,io.BytesIO(data[m.name]))
archive=buf.getvalue();manifest.update(bind_host='0.0.0.0',archive_sha256=sha(archive),archive_size=len(archive),member_bytes=sum(map(len,data.values())))
# Replace only old address in outer transport metadata, preserving all guards.
# Preserve all existing network and transport settings.
new={'native-bundle.tar.gz':archive,'deployment.json':(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode()}
for n,b in new.items():seal['files'][n]=sha(b)
new['installation.json']=(json.dumps(seal,indent=2)+'\n').encode()
cal=pathlib.Path('/var/lib/xgoduck-workbench/calibration-state.json');before=sha(cal.read_bytes())
backup=pathlib.Path('/opt/xgoduck-workbench-before-sim-'+str(time.time_ns()));shutil.copytree(p,backup)
subprocess.run(['systemctl','stop','xgoduck-workbench'],check=True,timeout=15)
for n,b in new.items():
 temp=p/(n+'.wifi-new');temp.write_bytes(b);temp.chmod(0o600);os.replace(temp,p/n)
subprocess.run(['systemctl','start','xgoduck-workbench'],check=True,timeout=10)
assert sha(cal.read_bytes())==before
print(json.dumps({'backup':str(backup),'changed_members':changed,'calibration_unchanged':True,'bind_host':manifest['bind_host'],'motor_commands_sent':0}))
