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


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / 'service').mkdir()
        (self.root / 'web').mkdir()
        for name in ('native_process.py', 'native_workbench.py'):
            shutil.copyfile(Path(app.__file__).parent / name, self.root / 'service' / name)
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
        values = {'Host': web.HOST + ':' + str(web.PORT)}
        if body is not None:
            values.update(Origin=web.ORIGIN, **{'Content-Type': 'application/json',
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

    def test_timeout_closes_lane_and_never_retries(self):
        parent, child = socket.socketpair()
        sockets = {lane: parent for lane in app.LANES}
        client = app.CommandClient(sockets)
        try:
            with self.assertRaisesRegex(ValueError, '不会重发'):
                client.call('/api/motion', {'action': 'heartbeat'}, 'browser')
            request = app.receive(child, app.MAX_COMMAND)
            self.assertEqual(request['body']['action'], 'heartbeat')
            self.assertEqual(child.recv(1), b'')
            with self.assertRaisesRegex(ValueError, '不会重发'):
                client.call('/api/motion', {'action': 'heartbeat'}, 'browser')
        finally:
            parent.close()
            child.close()


class SupervisorTests(unittest.TestCase):
    def test_actual_supervisor_starts_distinct_http_child_and_closes_runtime_on_child_death(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'service').mkdir()
            (root / 'web').mkdir()
            for name in ('native_process.py', 'native_workbench.py'):
                shutil.copyfile(Path(app.__file__).parent / name, root / 'service' / name)
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
