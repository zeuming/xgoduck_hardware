"""Full native routing and connection ownership, with no network or UART."""
import copy
import shutil
import subprocess
from pathlib import Path
import tempfile
import types
import unittest

import native_workbench as web
import test_native_workbench as legacy


class Calibration:
    def __init__(self):
        self.calls = []
        self.before_acquire = lambda purpose: None
        self.end_failure = None
        self.session = types.SimpleNamespace(maintenance=None, maintenance_mapping_changed=False,
            maintenance_fault_hold=None, recovery_recheck=None, recovery_pending=False,
            source_transmitting=False)

    def snapshot(self, *, compact=False, include_motion=True):
        return dict(session=dict(generation=19, status='closed', recovery_allowed=True),
            calibration=dict(actual_saved=True), feedback=None,
            wizard=dict(stage='idle'), maintenance=dict(owns_connection=False))

    def dispatch(self, path, body):
        self.calls.append((path, copy.deepcopy(body)))

    def end_for_motion(self):
        self.calls.append(('end-for-motion',))
        if self.end_failure:
            raise ValueError(self.end_failure)


class FullWorkbenchTests(unittest.TestCase):
    request = legacy.NativeWorkbenchTests.request
    body = legacy.NativeWorkbenchTests.body

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        (self.root / 'index.html').write_bytes(legacy.SOURCE.read_bytes())
        self.runtime = legacy.Runtime()
        self.runtime.motion['owns_connection'] = False
        self.calibration = Calibration()
        self.workbench = web.NativeWorkbench(self.runtime, web_root=self.root,
            password=legacy.PASSWORD, calibration_service=self.calibration)
        self.cookie = self.workbench.login(legacy.PASSWORD, '192.168.1.20')

    def tearDown(self):
        self.directory.cleanup()

    def test_actual_full_page_forward_controls_and_existing_calibration(self):
        page=self.root/'full-native.html';page.write_bytes(self.workbench.page)
        for script in ('test_bounded_forward_ui.js','test_native_full_workbench_ui.js'):
            result=subprocess.run([shutil.which('node'),str(Path(__file__).with_name(script)),str(page)],capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_full_snapshot_keeps_actual_acquisition_and_stable_motion_generation(self):
        value = self.workbench.snapshot()
        self.assertEqual(value['session']['generation'], 19)
        self.assertEqual(value['native_access']['page_generation'], self.workbench.generation)
        self.assertTrue(value['native_access']['full_workbench'])
        self.assertEqual(value['calibration'], {'actual_saved': True})
        self.assertNotIn(b'canAction=canCancel=', self.workbench.page)

    def test_motion_startup_history_does_not_replace_the_actual_calibration_session(self):
        self.runtime.motion['startup_fault'] = 'earlier USB disconnect; no motion UART opened'
        value = self.workbench.snapshot()
        self.assertEqual(value['session']['status'], 'closed')
        self.assertTrue(value['session']['recovery_allowed'])
        self.assertNotIn('error', value['session'])
        self.assertIsNone(value['feedback'])
        self.assertEqual(value['calibration'], {'actual_saved': True})
        self.assertTrue(value['native_access']['full_workbench'])

    def test_all_original_calibration_routes_dispatch_with_csrf(self):
        for path in ('imu-wizard', 'joint-calibration', 'joint-zero', 'end-feedback',
                     'recover-feedback', 'recheck-feedback', 'check-connection', 'servo-maintenance'):
            with self.subTest(path=path):
                body = dict(generation=19, action='capture', purpose='imu')
                code, value, _ = self.request('POST', '/api/' + path, body)
                self.assertEqual(code, 200, value)
                self.assertEqual(self.calibration.calls[-1], ('/api/' + path, body))
                before = len(self.calibration.calls)
                self.assertEqual(self.request('POST', '/api/' + path, body,
                    changes={'X-CSRF-Token': 'old-page'})[0], 403)
                self.assertEqual(len(self.calibration.calls), before)

    def test_calibration_action_response_uses_live_projection_and_full_status_stays_explicit(self):
        calls = []
        original = self.calibration.snapshot
        def snapshot(*, compact=False, include_motion=True):
            calls.append((compact, include_motion))
            return original(compact=compact, include_motion=include_motion)
        self.calibration.snapshot = snapshot
        code, value, _ = self.request('POST', '/api/joint-calibration', dict(generation=19, action='direction-end'))
        self.assertEqual(code, 200, value)
        self.assertEqual(calls, [(True, False)])
        calls.clear()
        self.assertEqual(self.request('GET', '/api/status')[0], 200)
        self.assertEqual(calls, [(False, False)])

    def test_owned_motion_blocks_acquisition_but_cleanup_remains_available(self):
        self.runtime.motion['owns_connection'] = True
        for path in ('recover-feedback', 'imu-wizard', 'joint-zero', 'joint-calibration'):
            self.assertEqual(self.request('POST', '/api/' + path, dict(generation=19))[0], 409)
        self.assertEqual(self.calibration.calls, [])
        with self.assertRaisesRegex(ValueError, '运控正在使用'):
            self.calibration.before_acquire('imu')
        for path, body in (('/api/end-feedback', dict(generation=19)),
                           ('/api/servo-maintenance', dict(action='exit', generation=19))):
            self.assertEqual(self.request('POST', path, body)[0], 200)

    def test_connect_waits_for_actual_acquisition_close_and_refuses_holds(self):
        self.calibration.end_failure = '旧读取尚未实际关闭'
        self.assertEqual(self.request('POST', '/api/motion', self.body('connect'))[0], 409)
        self.assertEqual(self.runtime.calls, [])
        self.calibration.end_failure = None
        for field, value in (('maintenance_mapping_changed', True),
                             ('maintenance_fault_hold', {'fault': True}),
                             ('recovery_recheck', {'category': 'trust'}),
                             ('recovery_pending', True), ('source_transmitting', True)):
            setattr(self.calibration.session, field, value)
            self.assertEqual(self.request('POST', '/api/motion', self.body('connect'))[0], 409)
            self.assertEqual(self.runtime.calls, [])
            setattr(self.calibration.session, field, False if value is True else None)
        self.calibration.session.maintenance = types.SimpleNamespace(owns_connection=True)
        self.assertEqual(self.request('POST', '/api/motion', self.body('connect'))[0], 409)
        self.calibration.session.maintenance = None
        self.assertEqual(self.request('POST', '/api/motion', self.body('connect'))[0], 200)
        self.assertEqual(self.calibration.calls[-1], ('end-for-motion',))
        self.assertEqual(self.runtime.calls, [('connect',)])


if __name__ == '__main__':
    unittest.main()
