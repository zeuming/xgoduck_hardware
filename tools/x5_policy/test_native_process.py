"""Real separate-process HTTP/IPC tests, with fake hardware only."""
import copy
import ast
import inspect
from concurrent.futures import ThreadPoolExecutor
import http.cookiejar
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
import urllib.error
import urllib.request

import native_process as app
import native_workbench as web
from test_native_workbench import Runtime, SOURCE, CLIENT
from test_native_full_workbench import Calibration


def copy_service(source, destination):
    for name in ('native_process.py', 'native_workbench.py', 'gamepad_web.py',
                 'gamepad_panel.html', 'native_gamepad.py', 'system_health.py',
                 'system_health_panel.html', 'native_command_runtime.py',
                 'command_view_page.py', 'command_view_ui.js', 'official_runtime_ui.js',
                 'sim_web.py'):
        file = source / name
        if file.is_file():
            shutil.copyfile(file, destination / name)


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / 'service').mkdir()
        (self.root / 'web').mkdir()
        copy_service(Path(app.__file__).parent, self.root / 'service')
        shutil.copyfile(SOURCE, self.root / 'web/index.html')
        self.runtime = Runtime()
        self.runtime.motion['owns_connection'] = False
        self.calibration = Calibration()
        self.authority = web.NativeWorkbench(self.runtime, web_root=self.root / 'web',
            authentication_required=False, calibration_service=self.calibration)
        self.store = app.SnapshotFile(self.root / 'web-state.json')
        self.parents, self.children = {}, {}
        for lane in app.LANES:
            self.parents[lane], self.children[lane] = socket.socketpair()
        self.bridge = app.ControllerBridge(self.authority, self.store, self.parents)
        self.bridge.start(commands=False)
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(16)
        self.url = 'http://127.0.0.1:' + str(self.listener.getsockname()[1])
        descriptors = [self.listener.fileno()] + [self.children[lane].fileno() for lane in app.LANES]
        self.child = subprocess.Popen([sys.executable, '-I', '-B',
            str(self.root / 'service/native_process.py'), '--frontend'] + list(map(str, descriptors)),
            pass_fds=descriptors, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for sock in self.children.values():
            sock.close()
        self.parents['command'].settimeout(5)
        self.ready = app.receive(self.parents['command'], app.MAX_COMMAND)
        self.parents['command'].settimeout(None)
        self.bridge.start_commands()
        self.client = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        self.request('/')

    def test_gamepad_endpoint_is_authenticated_and_does_not_call_controller(self):
        with mock.patch.object(self.authority, 'snapshot', side_effect=AssertionError('Gamepad read controller')):
            code, result = self.request('/api/gamepad')
            self.assertEqual(code, 200)
            self.assertIs(type(result['connected']), bool)
            self.assertEqual(self.runtime.calls, [])
        self.assertEqual(self.request('/api/gamepad', client=urllib.request.build_opener())[0], 401)
        self.assertEqual(self.request('/api/gamepad', headers={'Host': 'untrusted:8767'})[0], 409)
        code, page = self.request('/')
        self.assertIn(b'x5GamepadPanel', page)

    def test_health_reads_published_files_without_control_commands(self):
        import system_health
        system_health.atomic_json(self.root / 'system-health.json', dict(
            sampled_monotonic=time.monotonic(), state='no_limit_observed'))
        system_health.atomic_json(self.root / 'system-health-history.json', dict(samples=[{'cpu_celsius':70}]))
        with mock.patch.object(self.authority, 'snapshot', side_effect=AssertionError('Health read controller')):
            code, result = self.request('/api/live-status')
            self.assertEqual(code, 200)
            self.assertEqual(result['system_health']['state'], 'no_limit_observed')
            code, result = self.request('/api/system-health/history')
            self.assertEqual(code, 200)
            self.assertEqual(result['samples'][0]['cpu_celsius'], 70)
        self.assertEqual(self.request('/api/system-health/history', client=urllib.request.build_opener())[0], 401)
        self.assertEqual(self.request('/api/system-health/history', headers={'Host':'untrusted:8767'})[0], 409)
        self.assertEqual(self.runtime.calls, [])
        code, page = self.request('/')
        self.assertIn(b'x5HealthPanel', page)

    def test_local_gamepad_keeps_control_without_browser_requests(self):
        self.runtime.motion['owns_connection'] = True
        self.runtime.motion['hardware'].update(armed=True, hold_ready=True)
        self.assertEqual(self.request('/api/motion', self.body('start'))[0], 200)
        source = dict(connected=True, session='a'*32, sequence=0, events=[],
                      axes=[0,0,0,0,-32767,-32767,0,0], buttons=[0]*15)
        self.bridge.gamepad.reader = lambda: copy.deepcopy(source)
        code, result = self.request('/api/motion', self.body('gamepad-enable', support_confirmed=True))
        self.assertEqual(code, 200, result)
        self.assertTrue(result['motion']['gamepad_control']['active'])
        source.update(sequence=1, buttons=[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0], events=[
            dict(sequence=1, kind=1, number=3, value=1, age_ms=0)])
        # No HTTP requests are made during this wait. Fake motor actions and
        # heartbeats must originate in the independent local input worker.
        deadline = time.monotonic()+2
        while self.bridge.gamepad.heartbeat_count < 2 and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertGreaterEqual(self.bridge.gamepad.heartbeat_count, 2)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'walk')
        self.assertEqual(self.request('/api/motion', self.body('heartbeat'))[0], 409)
        source['connected'] = False
        deadline = time.monotonic()+2
        while self.bridge.gamepad.session is not None and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertIsNone(self.bridge.gamepad.session)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'hold')
        self.assertFalse(any(call[1].get('action') == 'stop' for call in self.runtime.calls))

    def local_gamepad_running(self):
        self.runtime.motion['owns_connection'] = True
        self.runtime.motion['hardware'].update(armed=True, hold_ready=True)
        self.assertEqual(self.request('/api/motion', self.body('start'))[0], 200)
        source = dict(connected=True, session='a'*32, sequence=0, events=[],
                      axes=[0,0,0,0,-32767,-32767,0,0], buttons=[0]*15)
        self.bridge.gamepad.reader = lambda: copy.deepcopy(source)
        self.assertEqual(self.request('/api/motion', self.body('gamepad-enable', support_confirmed=True))[0], 200)
        source.update(sequence=1, buttons=[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0], events=[
            dict(sequence=1, kind=1, number=3, value=1, age_ms=0)])
        deadline = time.monotonic()+2
        while self.bridge.gamepad.heartbeat_count < 2 and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertGreaterEqual(self.bridge.gamepad.heartbeat_count, 2)

    def test_web_command_and_mode_change_stay_available_after_gamepad_start(self):
        self.local_gamepad_running()
        code,result=self.request('/api/motion',self.body('command',forward=.1,side=0,yaw=0))
        self.assertEqual(code,200,result)
        code,result=self.request('/api/motion',self.body('start',mode='hold',support_confirmed=True))
        self.assertEqual(code,200,result)
        self.assertEqual(self.runtime.motion['hardware']['mode'],'hold')
        self.assertIsNotNone(self.bridge.gamepad.session)

    def test_local_gamepad_works_while_http_process_is_paused(self):
        self.local_gamepad_running()
        before = self.bridge.gamepad.heartbeat_count
        self.child.send_signal(signal.SIGSTOP)
        try:
            time.sleep(.55)
            monitor = self.bridge.control_monitor.snapshot()
            self.assertGreaterEqual(self.bridge.gamepad.heartbeat_count-before, 3)
            self.assertEqual(monitor['sources']['browser']['heartbeat_accepted'], 0)
            self.assertLess(monitor['sources']['x5_local']['maximum_heartbeat_gap_ms'], 400)
            self.assertEqual(self.runtime.motion['hardware']['mode'], 'walk')
            self.assertIsNone(self.bridge.failure)
        finally:
            self.child.send_signal(signal.SIGCONT)

    def test_status_request_flood_does_not_supply_or_block_gamepad_heartbeat(self):
        self.local_gamepad_running()
        before = self.bridge.gamepad.heartbeat_count
        deadline = time.monotonic()+.6
        def flood(_):
            count = 0
            while time.monotonic()<deadline:
                code, _ = self.request('/api/live-status')
                self.assertIn(code, (200, 503))
                count += 1
            return count
        with ThreadPoolExecutor(max_workers=8) as pool:
            total = sum(pool.map(flood, range(8)))
        self.assertGreater(total, 10)
        monitor = self.bridge.control_monitor.snapshot()
        self.assertGreater(self.bridge.gamepad.heartbeat_count, before)
        self.assertEqual(monitor['sources']['browser']['heartbeat_accepted'], 0)
        self.assertLess(monitor['sources']['x5_local']['maximum_heartbeat_gap_ms'], 400)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'walk')

    def test_monitor_file_failure_does_not_add_a_control_stop(self):
        with mock.patch.object(self.bridge.monitor_store, 'publish', side_effect=OSError('test disk full')):
            self.bridge.last_monitor_write = None
            self.bridge.publish()
            self.bridge.publish()
        self.assertIsNone(self.bridge.failure)
        value = self.store.read()['snapshot']['motion']['control_monitor']
        self.assertIn('test disk full', value['recording_error'])
        self.assertEqual(self.runtime.calls, [])

    def tearDown(self):
        self.bridge.close()
        self.child.terminate()
        self.child.wait(timeout=3)
        self.child.stderr.close()
        self.listener.close()
        for sock in self.parents.values():
            sock.close()
        self.tmp.cleanup()

    def request(self, path, body=None, *, client=None, headers=None):
        values = {'Host': '127.0.0.1:' + str(web.PORT)}
        if body is not None:
            values.update(Origin='http://' + values['Host'], **{'Content-Type': 'application/json',
                'X-CSRF-Token': self.authority.token})
        values.update(headers or {})
        request = urllib.request.Request(self.url + path,
            data=None if body is None else json.dumps(body).encode(), headers=values)
        try:
            response = (client or self.client).open(request, timeout=3)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            raw = response.read()
            return response.code, raw if path == '/' else json.loads(raw)

    def body(self, action, **changes):
        value = dict(action=action, control_id=CLIENT, page_generation=self.authority.generation,
                     generation=self.runtime.motion['hardware']['generation'])
        if action == 'start':
            value.update(mode='hold', support_confirmed=True)
        if action == 'stop':
            value['stop_source'] = 'button'
        value.update(changes)
        return value

    def test_child_is_distinct_and_status_flood_never_calls_controller(self):
        self.assertEqual(self.ready['pid'], self.child.pid)
        self.assertNotEqual(self.child.pid, os.getpid())
        # Freeze the publisher, then fail every possible controller read.
        self.bridge.stop.set()
        self.bridge.threads[0].join(1)
        with mock.patch.object(self.authority, 'snapshot', side_effect=AssertionError('HTTP read called controller')):
            with ThreadPoolExecutor(max_workers=8) as pool:
                results = list(pool.map(lambda _: self.request('/api/live-status'), range(48)))
            for code, snapshot in results:
                self.assertEqual(code, 200)
                self.assertEqual(snapshot['process_isolation']['web_pid'], self.child.pid)
                self.assertEqual(snapshot['process_isolation']['status_source'], 'published_snapshot')

    def test_auth_csrf_generation_ownership_and_receipts_remain_enforced(self):
        outsider = urllib.request.build_opener()
        self.assertEqual(self.request('/api/live-status', client=outsider)[0], 401)
        self.assertEqual(self.request('/api/motion', self.body('start'),
            headers={'X-CSRF-Token': 'old'})[0], 403)
        self.assertEqual(self.runtime.calls, [])
        self.assertEqual(self.request('/api/motion', self.body('start', page_generation=-1))[0], 409)
        code, receipt = self.request('/api/motion', self.body('start'))
        self.assertEqual(code, 200, receipt)
        self.assertEqual(receipt['action_receipt_protocol'], 1)
        self.assertEqual(receipt['mode'], 'hold')
        self.assertEqual(self.request('/api/motion', self.body('heartbeat'))[0], 200)
        self.assertEqual(self.request('/api/motion', self.body('heartbeat', control_id='2' * 32))[0], 409)
        code, receipt = self.request('/api/motion', self.body('stop'))
        self.assertEqual(code, 200, receipt)
        self.assertTrue(receipt['release_confirmed'])

    def test_delayed_heartbeat_reply_does_not_kill_bridge_or_block_stop(self):
        self.runtime.motion['owns_connection'] = True
        self.assertEqual(self.request('/api/motion', self.body('start'))[0], 200)
        original = self.authority.motion_action
        entered, finished, release = threading.Event(), threading.Event(), threading.Event()
        calls = []
        def delayed(body, **kwargs):
            if body.get('action') == 'heartbeat':
                calls.append(body)
                entered.set()
                release.wait(1)
                try:
                    return original(body, **kwargs)
                finally:
                    finished.set()
            return original(body, **kwargs)
        with mock.patch.object(self.authority, 'motion_action', side_effect=delayed):
            with ThreadPoolExecutor(max_workers=1) as pool:
                pending = pool.submit(self.request, '/api/motion', self.body('heartbeat'))
                self.assertTrue(entered.wait(1))
                code, result = pending.result(1)
                self.assertEqual(code, 409)
                self.assertEqual(self.request('/api/live-status')[0], 200)
                # The original RPC owns its lane until its exact reply drains.
                # A second heartbeat is rejected, never queued or resent.
                self.assertEqual(self.request('/api/motion', self.body('heartbeat'))[0], 409)
                self.assertEqual(len(calls), 1)
                self.assertEqual(self.request('/api/motion', self.body('stop'))[0], 200)
                release.set();self.assertTrue(finished.wait(1));time.sleep(.05)
                self.assertIsNone(self.bridge.failure)
                self.assertIn('正在核对原请求', result['error'])
        self.assertEqual(self.request('/api/motion', self.body('start'))[0], 200)
        self.assertEqual(self.request('/api/motion', self.body('heartbeat'))[0], 200)
        self.assertIsNone(self.bridge.failure)

    def test_calibration_download_and_routes_use_real_authority(self):
        code, calibration = self.request('/api/calibration')
        self.assertEqual(code, 200)
        self.assertEqual(calibration, {'actual_saved': True})
        for path in ('imu-wizard', 'joint-calibration', 'joint-zero', 'end-feedback',
                     'recover-feedback', 'recheck-feedback', 'check-connection', 'servo-maintenance'):
            body = dict(generation=19, action='capture', purpose='imu')
            code, value = self.request('/api/' + path, body)
            self.assertEqual(code, 200, value)
            self.assertEqual(self.calibration.calls[-1], ('/api/' + path, body))

    def test_stop_lane_runs_while_normal_command_waits_and_busy_is_not_queued(self):
        entered, release = threading.Event(), threading.Event()
        original = self.calibration.dispatch
        def slow(path, body):
            entered.set()
            if not release.wait(2):
                raise ValueError('test blocked')
            return original(path, body)
        self.calibration.dispatch = slow
        with ThreadPoolExecutor(max_workers=2) as pool:
            pending = pool.submit(self.request, '/api/end-feedback', dict(generation=19))
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.request('/api/end-feedback', dict(generation=19))[0], 409)
                code, stopped = self.request('/api/motion', self.body('stop'))
                self.assertEqual(code, 200, stopped)
                self.assertTrue(stopped['release_confirmed'])
                self.assertFalse(release.is_set())
            finally:
                release.set()
            self.assertEqual(pending.result()[0], 200)
        self.assertEqual(len(self.calibration.calls), 1)


class ProtocolTests(unittest.TestCase):
    def test_frontend_audit_allows_private_dev_shm_state_but_denies_devices_and_writes(self):
        tree = ast.parse(inspect.getsource(app.frontend_main))
        policy = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                      and node.name == 'frontend_audit')
        namespace = dict(root=Path('/dev/shm/isolated-test'), os=os, stat=stat)
        exec(compile(ast.Module(body=[policy], type_ignores=[]), '<actual-frontend-audit>', 'exec'), namespace)
        check = namespace['frontend_audit']
        with mock.patch.object(os, 'stat', return_value=mock.Mock(st_mode=stat.S_IFREG | 0o600)):
            check('open', ('/dev/shm/isolated-test/web-state.json', 'r', os.O_RDONLY))
            with self.assertRaises(PermissionError):
                check('open', ('/dev/shm/isolated-test/web-state.json', 'w', os.O_WRONLY))
            with self.assertRaises(PermissionError):
                check('open', ('/dev/ttyUSB0', 'r', os.O_RDONLY))
        with mock.patch.object(os, 'stat', return_value=mock.Mock(st_mode=stat.S_IFCHR | 0o600)):
            with self.assertRaises(PermissionError):
                check('open', ('/dev/shm/isolated-test/planted-device', 'r', os.O_RDONLY))

    def test_stale_and_misrouted_commands_never_execute(self):
        authority = mock.Mock()
        bridge = app.ControllerBridge(authority, None, {})
        request = dict(id='1' * 32, path='/api/motion', body={'action': 'start'},
                       principal='browser', deadline=time.monotonic() - 1)
        with self.assertRaisesRegex(ValueError, 'expired'):
            bridge.dispatch('command', request)
        request['deadline'] = time.monotonic() + 1
        with self.assertRaisesRegex(ValueError, 'lane'):
            bridge.dispatch('stop', request)
        authority.motion_action.assert_not_called()

    def test_snapshot_delay_never_becomes_fresh_feedback_or_lease(self):
        value = dict(motion=dict(hardware=dict(age_ms=10, imu_age_ms=20,
            heartbeat_remaining_ms=300), last_hardware=dict(age_ms=10)),
            hardware=dict(age_ms=10), body_imu={'old': True},
            session=dict(received_age_ms=30), feedback={'old': True})
        aged = app.age_snapshot(value, 200)
        self.assertEqual(aged['motion']['hardware']['age_ms'], 210)
        self.assertEqual(aged['motion']['hardware']['heartbeat_remaining_ms'], 100)
        stale = app.age_snapshot(value, 600)
        self.assertIsNone(stale['motion']['hardware'])
        self.assertIsNone(stale['feedback'])
        self.assertIsNone(stale['body_imu'])
        self.assertEqual(value['motion']['hardware']['age_ms'], 10)

    def test_child_allowance_is_exact(self):
        root = Path('/dev/shm/test-runtime')
        command = [sys.executable, '-I', '-B', str(root / 'service/native_process.py'),
                   '--frontend', '3', '4', '5', '6']
        self.assertTrue(app.allowed_web_command(sys.executable, command, root))
        for changed in (command + ['extra'], command[:-1] + ['3'],
                        command[:3] + ['/tmp/unreviewed.py'] + command[4:]):
            self.assertFalse(app.allowed_web_command(sys.executable, changed, root))

    def test_stale_imu_does_not_hide_fresh_connection_and_joint_feedback(self):
        value = dict(motion=dict(owns_connection=True, phase='ready',
            hardware=dict(connected=True, mode='off', generation=15,
                          armed=False, release_confirmed=True, age_ms=10, imu_age_ms=60000)),
            hardware=dict(age_ms=10), body_imu={'old': True}, session={})
        aged = app.age_snapshot(value, 30)
        hardware = aged['motion']['hardware']
        self.assertEqual(hardware['age_ms'], 40)
        self.assertEqual(hardware['imu_age_ms'], 60030)
        self.assertTrue(hardware['connected'])
        self.assertTrue(hardware['release_confirmed'])
        self.assertIsNone(aged['body_imu'])
        self.assertEqual(value['motion']['hardware']['imu_age_ms'], 60000)
        self.assertIsNotNone(value['body_imu'])
        self.assertIsNone(app.age_snapshot(value, 500)['motion']['hardware'])

    def test_missing_reply_eventually_closes_lane_and_never_retries(self):
        parent, child = socket.socketpair()
        sockets = {lane: parent for lane in app.LANES}
        client = app.CommandClient(sockets)
        try:
            with self.assertRaisesRegex(ValueError, '正在核对原请求'):
                client.call('/api/motion', {'action': 'heartbeat'}, 'browser')
            request = app.receive(child, app.MAX_COMMAND)
            self.assertEqual(request['body']['action'], 'heartbeat')
            child.settimeout(3)
            self.assertEqual(child.recv(1), b'')
            with self.assertRaisesRegex(ValueError, '不会重发'):
                client.call('/api/motion', {'action': 'heartbeat'}, 'browser')
        finally:
            parent.close()
            child.close()

    def test_partial_late_heartbeat_frame_is_drained_before_next_request(self):
        for split in (2, 12):
            with self.subTest(split=split):
                parent, child = socket.socketpair()
                child.settimeout(2)
                client = app.CommandClient({lane:parent for lane in app.LANES})
                requests, errors = [], []
                def peer():
                    try:
                        first = app.receive(child, app.MAX_COMMAND);requests.append(first)
                        raw = app.encode(dict(id=first['id'],ok=True,result={'reply':1}))
                        packet = app.struct.pack('!I',len(raw))+raw
                        child.sendall(packet[:split]);time.sleep(.22);child.sendall(packet[split:])
                        second = app.receive(child, app.MAX_COMMAND);requests.append(second)
                        app.send(child,dict(id=second['id'],ok=True,result={'reply':2}))
                    except Exception as error:
                        errors.append(error)
                thread = threading.Thread(target=peer);thread.start()
                try:
                    with self.assertRaisesRegex(ValueError,'正在核对原请求'):
                        client.call('/api/motion',{'action':'heartbeat'},'browser')
                    deadline = time.monotonic()+1
                    while client.locks['heartbeat'].locked() and time.monotonic()<deadline:
                        time.sleep(.005)
                    self.assertFalse(client.locks['heartbeat'].locked())
                    self.assertEqual(client.call('/api/motion',{'action':'heartbeat'},'browser'),{'reply':2})
                    thread.join(2);self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
                    self.assertEqual(len(requests),2);self.assertNotEqual(requests[0]['id'],requests[1]['id'])
                finally:
                    parent.close();child.close();thread.join(2)

    def test_wrong_late_reply_identity_cannot_confirm_another_heartbeat(self):
        parent, child = socket.socketpair();client=app.CommandClient({lane:parent for lane in app.LANES})
        def peer():
            app.receive(child,app.MAX_COMMAND);time.sleep(.22)
            app.send(child,dict(id='wrong-request',ok=True,result={'reply':1}))
        thread=threading.Thread(target=peer);thread.start()
        try:
            with self.assertRaisesRegex(ValueError,'正在核对原请求'):
                client.call('/api/motion',{'action':'heartbeat'},'browser')
            thread.join(2)
            deadline=time.monotonic()+1
            while client.locks['heartbeat'].locked() and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual(parent.fileno(),-1)
            with self.assertRaisesRegex(ValueError,'不会重发'):
                client.call('/api/motion',{'action':'heartbeat'},'browser')
        finally:
            parent.close();child.close();thread.join(2)


class SupervisorTests(unittest.TestCase):
    def test_actual_supervisor_starts_distinct_http_child_and_closes_runtime_on_child_death(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'service').mkdir()
            (root / 'web').mkdir()
            copy_service(Path(app.__file__).parent, root / 'service')
            shutil.copyfile(SOURCE, root / 'web/index.html')
            probe = socket.socket()
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
            probe.close()
            runtime = Runtime()
            runtime.motion['owns_connection'] = False
            calibration = Calibration()
            calibration.close = mock.Mock()
            ready, done = threading.Event(), threading.Event()
            infos, failures = [], []
            audit_active = {'value': True}
            def deny_new_null_open(event, arguments):
                if audit_active['value'] and event == 'open' and arguments and arguments[0] == os.devnull:
                    raise PermissionError('Reviewed controller cannot open /dev/null for writing')
            sys.addaudithook(deny_new_null_open)
            def callback(info):
                infos.append(info)
                ready.set()
            def run():
                try:
                    app.serve(runtime, web_root=root / 'web', host='127.0.0.1', port=port,
                        authentication_required=False, auto_connect=False,
                        calibration_service=calibration, ready_callback=callback)
                except BaseException as error:
                    failures.append(error)
                finally:
                    done.set()
            with mock.patch.object(web, 'HOST', '127.0.0.1'), mock.patch.object(web, 'PORT', port):
                thread = threading.Thread(target=run)
                thread.start()
                try:
                    self.assertTrue(ready.wait(5), failures)
                    info = infos[0]
                    self.assertNotEqual(info['web_pid'], info['pid'])
                    self.assertEqual(info['pid'], os.getpid())
                    os.kill(info['web_pid'], 15)
                    self.assertTrue(done.wait(5), 'controller did not handle HTTP child exit')
                finally:
                    audit_active['value'] = False
                    if infos and not done.is_set():
                        os.kill(infos[0]['web_pid'], 15)
                    thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(runtime.calls, [('close',)])
            calibration.close.assert_called_once()
            self.assertTrue(failures)
            self.assertIsInstance(failures[0], RuntimeError)


if __name__ == '__main__':
    unittest.main()
