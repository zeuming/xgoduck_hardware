"""Pure CPU tests: no HTTP listeners, SSH, network, UART, or hardware."""
import copy
from email.message import Message
import io
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
import subprocess
import shutil
from unittest import mock

import native_workbench as web


SOURCE = Path(__file__).resolve().parents[1] / 'x5_recovery' / 'feedback_calibration.html'
PASSWORD = 'A' * 48
CLIENT = '1' * 32
OTHER = '2' * 32


class Runtime:
    def __init__(self):
        self.calibration = dict(joints=[dict(id=11, zero=2002)], imu=dict(axes=['-z', '-x', 'y']))
        self.motion = dict(phase='ready', owns_connection=True, error=None, feedback_age_ms=0,
                           connection_sequence=1,
                           models_loaded=list(web.MODEL_NAMES),
                           deployment=dict(archive_sha256='actual', started_monotonic=12),
                           hardware=dict(connected=True, generation=5, mode='off',
                                         imu=dict(gravity=[0, 0, -1]), release_confirmed=True))
        self.calls = []
        self.failure = None

    def snapshot(self, *, compact=False):
        return copy.deepcopy(self.motion)

    def control_context(self):
        return dict(phase=self.motion['phase'], owns_connection=self.motion['owns_connection'],
                    connection_sequence=self.motion['connection_sequence'],
                    hardware=copy.deepcopy(self.motion['hardware']))

    def action_receipt(self, body):
        self.calls.append(('receipt', copy.deepcopy(body)))
        if self.failure:
            raise ValueError(self.failure)
        h = self.motion['hardware']
        if body['action'] != 'forward-trial':
            h['generation'] += 1
            h['mode'] = body['mode'] if body['action'] == 'start' else 'off'
        value = dict(action_receipt_protocol=1, action=body['action'],
                    connection_sequence=self.motion['connection_sequence'],
                    generation=h['generation'], mode=h['mode'], connected=h['connected'],
                    release_confirmed=h['mode'] == 'off')
        if body['action'] == 'capture-head-clearance':
            value['captured_head_deg'] = {30: -4., 31: -17., 32: -10., 33: -3.}
            h['hold_head_deg'] = copy.deepcopy(value['captured_head_deg'])
        return value

    def connect(self):
        self.calls.append(('connect',))
        if self.failure:
            self.motion.update(phase='fault', error=self.failure, hardware=None)
            raise ValueError(self.failure)
        self.motion['hardware'].update(mode='off', generation=self.motion['hardware']['generation'] + 1)
        self.motion['connection_sequence'] += 1
        return self.snapshot()

    def disconnect(self):
        self.calls.append(('disconnect',))
        self.motion.update(phase='closed', owns_connection=False, hardware=None)
        return self.snapshot()

    def action(self, body):
        self.calls.append(('action', copy.deepcopy(body)))
        if self.failure:
            raise ValueError(self.failure)
        if body['action'] in ('start', 'stop'):
            hardware = self.motion['hardware']
            hardware['generation'] += 1
            hardware['mode'] = body['mode'] if body['action'] == 'start' else 'off'
        if body['action'] == 'heartbeat':
            return dict(heartbeat_accepted=True, generation=self.motion['hardware']['generation'])
        return self.snapshot()

    def close(self):
        self.calls.append(('close',))


class NativeWorkbenchTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        (self.root / 'index.html').write_bytes(SOURCE.read_bytes())
        (self.root / 'assets').mkdir()
        (self.root / 'assets' / 'zero-reference.png').write_bytes(b'png-data')
        (self.root / 'assets' / 'positive-direction.png').write_bytes(b'png-data')
        self.runtime = Runtime()
        self.now = 100.
        self.workbench = web.NativeWorkbench(self.runtime, web_root=self.root, password=PASSWORD,
                                             clock=lambda: self.now)
        self.cookie = self.workbench.login(PASSWORD, '192.168.1.20')

    def tearDown(self):
        self.directory.cleanup()

    def test_forward_trial_requires_same_owner_generation_confirmation_and_fixed_payload(self):
        self.workbench.motion_action(self.body('start', mode='walk', support_confirmed=True))
        owner=copy.deepcopy(self.workbench.control_owner)
        before=len(self.runtime.calls)
        for changes in ({'support_confirmed':False},{'support_confirmed':1},
                        {'control_id':OTHER},{'generation':-1},{'forward':.1},
                        {'duration':30},{'peak':.2}):
            body=self.body('forward-trial',support_confirmed=True);body.update(changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.workbench.motion_action(body)
        with self.assertRaises(ValueError):
            self.workbench.motion_action(self.body('forward-trial',support_confirmed=True),principal='another-session')
        self.assertEqual(len(self.runtime.calls),before)
        result=self.workbench.motion_action(self.body('forward-trial',support_confirmed=True),live=True)
        self.assertEqual(result['action_receipt_protocol'],1)
        self.assertEqual(result['mode'],'walk')
        self.assertEqual(result['control_owner']['generation'],owner['generation'])
        self.assertEqual(self.workbench.control_owner,owner)
        self.assertEqual(self.runtime.calls[-1][0],'receipt')

    def test_head_capture_has_explicit_confirmation_and_generation_without_control_claim(self):
        for changes in ({}, {'clearance_confirmed':False}, {'clearance_confirmed':1},
                        {'clearance_confirmed':True,'generation':-1},
                        {'clearance_confirmed':True,'angles':[0,0,0,0]}):
            with self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('capture-head-clearance', **changes))
        before = len(self.runtime.calls)
        self.workbench.motion_action(self.body('capture-head-clearance', clearance_confirmed=True))
        self.assertEqual(len(self.runtime.calls), before + 1)
        self.assertIsNone(self.workbench.control_owner)
        self.assertEqual(self.workbench.last_operation['action'], 'capture-head-clearance')

    def body(self, action='start', **changes):
        value = dict(action=action, page_generation=self.workbench.generation,
                     generation=self.runtime.motion['hardware']['generation'], control_id=CLIENT)
        if action == 'start':
            value.update(mode='hold', support_confirmed=True)
        if action == 'stop':
            value['stop_source'] = 'button'
        value.update(changes)
        return value

    def request(self, method, path, body=None, *, changes=None, duplicate=None, logged_in=True, local_host='192.168.1.11'):
        raw = b'' if body is None else body if isinstance(body, bytes) else json.dumps(body).encode()
        headers = {'Host': local_host + ':' + str(web.PORT)}
        if logged_in:
            headers['Cookie'] = web.COOKIE + '=' + self.cookie
        if method == 'POST':
            headers.update(Origin='http://' + headers['Host'], **{'X-CSRF-Token': self.workbench.token,
                           'Content-Type': 'application/json', 'Content-Length': str(len(raw))})
        headers.update(changes or {})
        handler = object.__new__(web.handler_for(self.workbench))
        handler.headers = Message()
        for key, value in headers.items():
            if value is not None:
                handler.headers[key] = value
        if duplicate:
            handler.headers[duplicate[0]] = duplicate[1]
        handler.connection = mock.Mock()
        handler.connection.getsockname.return_value = (local_host, web.PORT)
        handler.path = path
        handler.rfile = io.BytesIO(raw)
        handler.client_address = ('192.168.1.20', 54321)
        replies = []
        handler.reply = lambda code, value, **options: replies.append((code, value, options))
        getattr(handler, 'do_' + method)()
        self.assertEqual(len(replies), 1)
        return replies[0]

    def test_snapshot_discards_all_template_live_state_and_credentials(self):
        template = dict(session=dict(generation=1, status='active'), token='old',
                        calibration=dict(stale=True), hardware=dict(armed=True), feedback=dict(stale=True),
                        body_imu=dict(stale=True), motion=dict(hardware=dict(armed=True)),
                        wizard=dict(stage='sampling'), maintenance=dict(owns_connection=True))
        self.workbench.template = template
        value = self.workbench.snapshot()
        self.assertEqual(value['session']['generation'], self.workbench.generation)
        self.assertEqual(value['session']['status'], 'closed')
        self.assertEqual(value['calibration'], self.runtime.calibration)
        self.assertEqual(value['hardware'], self.runtime.motion['hardware'])
        self.assertIsNone(value['feedback'])
        self.assertEqual(value['body_imu'], self.runtime.motion['hardware']['imu'])
        self.assertEqual(value['motion']['deployment']['archive_sha256'], 'actual')
        self.assertFalse(value['maintenance']['owns_connection'])
        self.assertEqual(value['wizard']['stage'], 'idle')
        self.assertNotIn(PASSWORD, json.dumps(value))
        self.assertNotIn(self.cookie, json.dumps(value))

    def test_unauthenticated_login_page_and_no_status_or_assets(self):
        code, html, _ = self.request('GET', '/', logged_in=False)
        self.assertEqual(code, 200)
        self.assertIn(self.workbench.login_token.encode(), html)
        self.assertNotIn(PASSWORD.encode(), html)
        for path in ('/api/status', '/api/live-status', '/api/calibration', '/assets/zero-reference.png'):
            with self.subTest(path=path):
                self.assertEqual(self.request('GET', path, logged_in=False)[0], 401)

    def anonymous_workbench(self):
        self.workbench = web.NativeWorkbench(self.runtime, web_root=self.root,
                                             authentication_required=False, clock=lambda: self.now)
        self.cookie = None

    def anonymous_home(self):
        code, page, options = self.request('GET', '/', logged_in=False)
        self.assertEqual(code, 200)
        self.cookie = options['cookie'].split('=', 1)[1].split(';', 1)[0]
        return page, options

    def test_anonymous_home_opens_without_password_and_establishes_private_browser_session(self):
        self.anonymous_workbench()
        page, options = self.anonymous_home()
        self.assertIn('浏览器控制会话'.encode(), page)
        self.assertNotIn('口令登录'.encode(), page)
        self.assertNotIn(b'id="password"', page)
        self.assertNotIn(self.workbench.login_token.encode(), page)
        self.assertIn('HttpOnly', options['cookie'])
        self.assertIn('SameSite=Strict', options['cookie'])
        code, value, _ = self.request('GET', '/api/status')
        self.assertEqual(code, 200)
        self.assertFalse(value['native_access']['authentication_required'])
        self.assertNotIn(self.cookie, json.dumps(value))
        self.assertEqual(value['token'], self.workbench.token)
        existing_cookie = self.cookie
        code, _, options = self.request('GET', '/')
        self.assertEqual(code, 200)
        self.assertIsNone(options['cookie'])
        self.assertEqual(self.workbench.principal(web.COOKIE + '=' + existing_cookie), existing_cookie)
        self.assertEqual(self.runtime.calls, [])

    def test_anonymous_api_without_session_requests_home_refresh_and_cannot_create_control_cookie(self):
        self.anonymous_workbench()
        for path in ('/api/status', '/api/calibration', '/assets/zero-reference.png'):
            with self.subTest(path=path):
                code, value, _ = self.request('GET', path, logged_in=False)
                self.assertEqual(code, 401)
                self.assertIn('刷新工作台首页', value['error'])
        self.assertEqual(self.request('GET', '/login', logged_in=False)[0], 404)
        self.assertEqual(self.request('GET', '/', logged_in=False, changes={'Host': 'evil.invalid'})[0], 403)
        self.assertEqual(self.workbench.cookies, {})

    def test_anonymous_controls_still_require_same_origin_csrf_and_browser_session(self):
        self.anonymous_workbench()
        self.anonymous_home()
        for changes in ({'Host': 'evil.invalid'}, {'Origin': 'http://evil.invalid'},
                        {'X-CSRF-Token': 'wrong'}, {'Cookie': 'not-a-session'}):
            with self.subTest(changes=changes):
                self.assertNotEqual(self.request('POST', '/api/motion', self.body(), changes=changes)[0], 200)
        self.assertEqual(self.runtime.calls, [])
        self.assertEqual(self.request('POST', '/api/motion', self.body())[0], 200)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'hold')
        self.assertEqual(self.request('POST', '/api/joint-zero', dict(action='capture'))[0], 403)
        self.assertEqual(self.request('POST', '/api/login', dict(password=PASSWORD), logged_in=False)[0], 403)

    def test_anonymous_other_browser_cannot_spoof_owner_but_can_press_emergency_stop(self):
        self.anonymous_workbench()
        self.anonymous_home()
        first_cookie = self.cookie
        self.assertEqual(self.request('POST', '/api/motion', self.body())[0], 200)
        owner = self.workbench.snapshot()['motion']['control_owner']
        self.anonymous_home()
        self.assertNotEqual(first_cookie, self.cookie)
        for body in (self.body('heartbeat', control_id=owner['id']),
                     self.body('start', control_id=owner['id']),
                     self.body('stop', control_id=owner['id'], stop_source='pagehide')):
            with self.subTest(action=body['action']):
                self.assertEqual(self.request('POST', '/api/motion', body)[0], 409)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'hold')
        self.assertEqual(self.request('POST', '/api/motion', self.body('stop', control_id=OTHER, generation=0))[0], 200)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'off')
        self.assertIsNone(self.workbench.snapshot()['motion']['control_owner'])

    def test_anonymous_session_is_renewed_after_expiry_without_password(self):
        self.anonymous_workbench()
        self.anonymous_home()
        previous = self.cookie
        self.now += web.COOKIE_SECONDS + 1
        code, page, options = self.request('GET', '/')
        self.assertEqual(code, 200)
        self.assertIn('浏览器控制会话'.encode(), page)
        self.assertNotIn(previous, options['cookie'])
        self.assertFalse(self.workbench.authenticated(web.COOKIE + '=' + previous))

    def test_session_churn_stays_bounded_without_blocking_home_or_evicting_owner(self):
        self.anonymous_workbench()
        self.anonymous_home()
        owner_cookie = self.cookie
        start = self.body()
        self.assertEqual(self.request('POST', '/api/motion', start)[0], 200)
        owner = copy.deepcopy(self.workbench.control_owner)
        for _ in range(web.MAX_SESSIONS * 2):
            self.anonymous_home()
        self.assertEqual(len(self.workbench.cookies), web.MAX_SESSIONS)
        self.assertEqual(self.workbench.control_owner, owner)
        self.assertTrue(self.workbench.authenticated(web.COOKIE + '=' + owner_cookie))
        self.assertEqual(self.runtime.calls, [('receipt', start)])
        self.assertEqual(self.request('POST', '/api/motion', self.body())[0], 409)
        self.cookie = owner_cookie
        self.assertEqual(self.request('GET', '/api/live-status')[0], 200)
        self.assertEqual(self.request('POST', '/api/motion', self.body('heartbeat'))[0], 200)

    def test_session_recency_preserves_active_reader_and_does_not_extend_expiry(self):
        self.anonymous_workbench()
        self.anonymous_home()
        active_cookie = self.cookie
        expiry = self.workbench.cookies[active_cookie]
        self.anonymous_home()
        abandoned_cookie = self.cookie
        for _ in range(web.MAX_SESSIONS - 2):
            self.anonymous_home()
        self.cookie = active_cookie
        self.now += 1
        self.assertEqual(self.request('GET', '/api/live-status')[0], 200)
        self.anonymous_home()
        self.assertIn(active_cookie, self.workbench.cookies)
        self.assertNotIn(abandoned_cookie, self.workbench.cookies)
        self.assertEqual(self.workbench.cookies[active_cookie], expiry)
        self.cookie = active_cookie
        count = len(self.workbench.cookies)
        for _ in range(100):
            code, _, options = self.request('GET', '/')
            self.assertEqual(code, 200)
            self.assertIsNone(options['cookie'])
        self.assertEqual(len(self.workbench.cookies), count)

    def test_login_same_origin_token_and_private_cookie(self):
        code, value, options = self.request('POST', '/api/login', dict(password=PASSWORD), logged_in=False,
                                           changes={'X-CSRF-Token': self.workbench.login_token})
        self.assertEqual(code, 200)
        self.assertEqual(value, dict(logged_in=True))
        self.assertIn('HttpOnly', options['cookie'])
        self.assertIn('SameSite=Strict', options['cookie'])
        self.assertNotIn(PASSWORD, options['cookie'])
        for changes in ({'Origin': 'http://evil.invalid'}, {'X-CSRF-Token': 'bad'}, {'Host': 'localhost:8767'}):
            with self.subTest(changes=changes):
                self.assertNotEqual(self.request('POST', '/api/login', dict(password=PASSWORD),
                                                logged_in=False, changes=changes)[0], 200)

    def test_password_failure_rate_bound_and_cookie_expiry(self):
        for _ in range(5):
            with self.assertRaisesRegex(ValueError, '口令错误'):
                self.workbench.login('wrong', '192.168.1.99')
        with self.assertRaisesRegex(ValueError, '次数过多'):
            self.workbench.login(PASSWORD, '192.168.1.99')
        self.assertTrue(self.workbench.authenticated(web.COOKIE + '=' + self.cookie))
        self.assertFalse(self.workbench.authenticated(web.COOKIE + '=' + self.cookie + '; ' + web.COOKIE + '=bad'))
        self.now += web.COOKIE_SECONDS + 1
        self.assertFalse(self.workbench.authenticated(web.COOKIE + '=' + self.cookie))

    def test_authenticated_relative_assets_and_calibration_export(self):
        for path in ('/api/status', '/api/calibration', '/assets/zero-reference.png', '/positive-direction.png'):
            with self.subTest(path=path):
                self.assertEqual(self.request('GET', path)[0], 200)
        self.assertTrue(self.request('GET', '/api/calibration')[2]['download'])
        self.assertEqual(self.request('GET', '/assets/../../etc/passwd')[0], 404)

    def test_header_origin_csrf_and_framing_reject_without_runtime_call(self):
        changes = [dict(Host='192.168.1.11'), dict(Origin='null'), {'X-CSRF-Token': 'wrong'}, {'X-CSRF-Token': '\u00ff'},
                   {'Transfer-Encoding': 'chunked'}, {'Expect': '100-continue'},
                   {'Content-Length': '-1'}, {'Content-Length': '4097'},
                   {'Content-Length': '01'}, {'Content-Length': '1,1'},
                   {'Content-Type': 'text/plain'}, {'X-Extra': 'line\r\ninjected: yes'},
                   {'X-Extra': 'x' * web.MAX_HEADERS}]
        for change in changes:
            with self.subTest(change=list(change)):
                self.assertNotEqual(self.request('POST', '/api/motion', self.body(), changes=change)[0], 200)
        for name in ('Host', 'Origin', 'Content-Length', 'X-CSRF-Token'):
            with self.subTest(duplicate=name):
                self.assertNotEqual(self.request('POST', '/api/motion', self.body(), duplicate=(name, 'x'))[0], 200)
        self.assertEqual(self.runtime.calls, [])

    def test_read_only_post_paths_always_rejected(self):
        for path in ('/api/imu-wizard', '/api/joint-zero', '/api/joint-calibration',
                     '/api/servo-maintenance', '/api/recover-feedback', '/api/recheck-feedback',
                     '/api/check-connection', '/api/end-feedback'):
            with self.subTest(path=path):
                code, value, _ = self.request('POST', path, self.body())
                self.assertEqual(code, 403)
                self.assertEqual(value['error'], web.READ_ONLY)
        self.assertEqual(self.runtime.calls, [])

    def test_json_duplicate_nonfinite_partial_and_wrong_shapes_rejected(self):
        for raw in (b'{"action":"start","action":"stop"}', b'{"x":NaN}', b'[]', b'null', b'"start"'):
            with self.subTest(raw=raw):
                self.assertNotEqual(self.request('POST', '/api/motion', raw)[0], 200)
        self.assertNotEqual(self.request('POST', '/api/motion', b'{}', changes={'Content-Length': '9'})[0], 200)
        self.assertEqual(self.runtime.calls, [])

    def test_start_claims_owner_and_stale_generation_or_tab_cannot_renew(self):
        result = self.workbench.motion_action(self.body())
        generation = result['motion']['hardware']['generation']
        self.assertEqual(result['motion']['control_owner'], dict(id=CLIENT, generation=generation))
        before = len(self.runtime.calls)
        for changes in (dict(control_id=OTHER), dict(generation=generation - 1), dict(page_generation=1)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('heartbeat', **changes))
        self.assertEqual(len(self.runtime.calls), before)
        self.workbench.motion_action(self.body('heartbeat', background=True))
        self.assertTrue(self.runtime.calls[-1][1]['background'])

    def test_heartbeat_ack_does_not_serialize_status_or_calibration(self):
        started = self.workbench.motion_action(self.body(mode='walk'), principal='first-cookie')
        generation = started['motion']['hardware']['generation']
        # Slow or unavailable display serialization must not hold the page
        # lease response; owner and generation checks still precede renewal.
        with (mock.patch.object(self.workbench, 'snapshot', side_effect=AssertionError('full web snapshot')),
              mock.patch.object(self.runtime, 'snapshot', side_effect=AssertionError('full runtime snapshot'))):
            accepted = self.workbench.motion_action(self.body('heartbeat', generation=generation),
                                                   principal='first-cookie')
            self.assertEqual(accepted, dict(heartbeat_accepted=True,
                page_generation=self.workbench.generation, generation=generation))
            with self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('heartbeat', generation=generation),
                                            principal='other-cookie')

    def test_live_start_and_stop_receipts_skip_all_full_snapshots(self):
        with (mock.patch.object(self.workbench, 'snapshot', side_effect=AssertionError('full web snapshot')),
              mock.patch.object(self.runtime, 'snapshot', side_effect=AssertionError('full runtime snapshot'))):
            started = self.workbench.motion_action(self.body(mode='walk'), principal='first-cookie', live=True)
            self.assertEqual(started['action_receipt_protocol'], 1)
            self.assertEqual(started['mode'], 'walk')
            self.assertEqual(started['control_owner'], dict(id=CLIENT, generation=started['generation']))
            self.assertNotIn('hardware', started)
            self.assertNotIn('token', started)
            self.assertLess(len(json.dumps(started)), 1024)
            stopped = self.workbench.motion_action(self.body('stop'), principal='other-cookie', live=True)
            self.assertEqual(stopped['mode'], 'off')
            self.assertTrue(stopped['release_confirmed'])
            self.assertIsNone(stopped['control_owner'])
        self.assertIsNone(self.workbench.control_owner)

    def test_live_stop_receipt_requires_actual_release_proof(self):
        original = self.runtime.action_receipt
        def unconfirmed(body):
            value = original(body)
            value['release_confirmed'] = False
            return value
        self.runtime.action_receipt = unconfirmed
        with self.assertRaisesRegex(ValueError, '旧确认未采用'):
            self.workbench.motion_action(self.body('stop'), live=True)
        self.assertEqual(self.workbench.last_operation['state'], 'failed')

    def test_live_capture_receipt_avoids_full_snapshots_and_clears_old_owner(self):
        self.workbench.control_owner = dict(id=OTHER, generation=4, principal='old-cookie')
        with (mock.patch.object(self.workbench, 'snapshot', side_effect=AssertionError('full web snapshot')),
              mock.patch.object(self.runtime, 'snapshot', side_effect=AssertionError('full runtime snapshot'))):
            value = self.workbench.motion_action(self.body('capture-head-clearance', clearance_confirmed=True),
                                                principal='current-cookie', live=True)
        self.assertEqual(value['action'], 'capture-head-clearance')
        self.assertEqual(value['mode'], 'off')
        self.assertTrue(value['release_confirmed'])
        self.assertEqual(value['captured_head_deg'], {30: -4., 31: -17., 32: -10., 33: -3.})
        self.assertIsNone(value['control_owner'])
        self.assertIsNone(self.workbench.control_owner)
        self.assertLess(len(json.dumps(value)), 1024)
        self.assertEqual(self.runtime.calls[-1][0], 'receipt')

    def test_live_capture_receipt_rejects_incomplete_nonfinite_or_unreleased_records(self):
        original = self.runtime.action_receipt
        for changes in ({'captured_head_deg': {30: 1., 31: 2., 32: 3.}},
                        {'captured_head_deg': {30: float('nan'), 31: 2., 32: 3., 33: 4.}},
                        {'captured_head_deg': {30: True, 31: 2., 32: 3., 33: 4.}},
                        {'release_confirmed': False}, {'mode': 'hold'}):
            def invalid(body):
                return dict(original(body), **changes)
            self.runtime.action_receipt = invalid
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, '旧确认未采用'):
                self.workbench.motion_action(self.body('capture-head-clearance', clearance_confirmed=True), live=True)
            self.assertIsNone(self.workbench.control_owner)

    def test_live_capture_rejects_active_mode_before_runtime_action(self):
        self.runtime.motion['hardware']['mode'] = 'hold'
        with self.assertRaisesRegex(ValueError, '先停止运动'):
            self.workbench.motion_action(self.body('capture-head-clearance', clearance_confirmed=True), live=True)
        self.assertFalse(self.runtime.calls)

    def test_live_snapshot_requests_source_compact_and_projects_operation_before_copy(self):
        self.runtime.motion['last_operation'] = dict(action='stop', after={'full_runtime_evidence': [5, 6]})
        self.workbench.last_operation = dict(action='stop', state='complete', error=None,
                                            before={'deep_audit': [1, 2]},
                                            after={'connected': True, 'generation': 7, 'deep_audit': [3, 4]})
        with mock.patch.object(self.runtime, 'snapshot', wraps=self.runtime.snapshot) as provider:
            live = self.workbench.snapshot(live=True)
            provider.assert_called_once_with(compact=True)
        self.assertEqual(live['motion']['last_operation'], dict(action='stop', state='complete', error=None,
                                                               after=dict(connected=True, generation=7)))
        self.assertIs(live['hardware'], live['motion']['hardware'])
        with mock.patch.object(self.runtime, 'snapshot', wraps=self.runtime.snapshot) as provider:
            full = self.workbench.snapshot()
            provider.assert_called_once_with()
        self.assertEqual(full['motion']['last_operation']['before'], {'deep_audit': [1, 2]})
        self.assertEqual(full['motion']['runtime_last_operation']['after'], {'full_runtime_evidence': [5, 6]})
        self.assertNotIn('runtime_last_operation', live['motion'])
        full['motion']['last_operation']['before']['deep_audit'].append(9)
        self.assertEqual(self.workbench.last_operation['before']['deep_audit'], [1, 2])

    def test_receipt_rejects_connected_or_release_context_that_changed_after_runtime_ack(self):
        original = self.runtime.control_context
        for action, changes in (('stop', {'connected': False}),
                                ('capture-head-clearance', {'release_confirmed': False})):
            calls = []
            def changed():
                value = original()
                calls.append(1)
                if len(calls) > 1:
                    value['hardware'].update(changes)
                return value
            self.runtime.control_context = changed
            extras = {'clearance_confirmed': True} if action == 'capture-head-clearance' else {}
            with self.subTest(action=action), self.assertRaisesRegex(ValueError, '旧确认未采用'):
                self.workbench.motion_action(self.body(action, **extras), live=True)

    def test_heartbeat_bypasses_busy_io_lock_without_bypassing_owner(self):
        self.workbench.motion_action(self.body(mode='walk'), principal='first-cookie')
        locked, release = threading.Event(), threading.Event()
        def hold_lock():
            with self.workbench.io_lock:
                locked.set()
                release.wait(2)
        thread = threading.Thread(target=hold_lock)
        thread.start()
        replies, failures = [], []
        def heartbeat():
            try:
                replies.append(self.workbench.motion_action(self.body('heartbeat'), principal='first-cookie'))
            except BaseException as error:
                failures.append(error)
        heartbeat_thread = threading.Thread(target=heartbeat)
        try:
            self.assertTrue(locked.wait(1))
            heartbeat_thread.start()
            heartbeat_thread.join(.5)
            self.assertFalse(heartbeat_thread.is_alive(), 'heartbeat waited behind an unrelated IO lock')
            self.assertEqual(failures, [])
            self.assertTrue(replies[0]['heartbeat_accepted'])
            self.assertFalse(release.is_set())
            with self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('heartbeat'), principal='other-cookie')
        finally:
            release.set()
            thread.join(2)
            heartbeat_thread.join(2)

    def test_late_live_start_receipt_cannot_restore_owner_after_stop(self):
        entered, release = threading.Event(), threading.Event()
        original = self.runtime.action_receipt
        errors = []
        def slow_receipt(body):
            result = original(body)
            if body['action'] == 'start':
                entered.set()
                if not release.wait(2):
                    raise ValueError('test did not release start')
            return result
        self.runtime.action_receipt = slow_receipt
        body = self.body(mode='walk')
        def start():
            try:
                self.workbench.motion_action(body, principal='first-cookie', live=True)
            except ValueError as error:
                errors.append(str(error))
        thread = threading.Thread(target=start)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            stopped = self.workbench.motion_action(self.body('stop'), principal='other-cookie', live=True)
            self.assertTrue(stopped['release_confirmed'])
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsNone(self.workbench.control_owner)
        self.assertEqual(self.workbench.last_operation['action'], 'stop')

    def test_live_receipt_from_previous_connection_is_rejected(self):
        original = self.runtime.action_receipt
        def reconnected(body):
            result = original(body)
            self.runtime.motion['connection_sequence'] += 1
            return result
        self.runtime.action_receipt = reconnected
        with self.assertRaisesRegex(ValueError, '旧确认未采用'):
            self.workbench.motion_action(self.body(mode='walk'), live=True)
        self.assertIsNone(self.workbench.control_owner)

    def test_late_live_capture_receipt_cannot_replace_newer_stop(self):
        entered, release = threading.Event(), threading.Event()
        original = self.runtime.action_receipt
        errors = []
        def slow_receipt(body):
            result = original(body)
            if body['action'] == 'capture-head-clearance':
                entered.set()
                if not release.wait(2):
                    raise ValueError('test did not release capture')
            return result
        self.runtime.action_receipt = slow_receipt
        body = self.body('capture-head-clearance', clearance_confirmed=True)
        def capture():
            try:
                self.workbench.motion_action(body, principal='first-cookie', live=True)
            except ValueError as error:
                errors.append(str(error))
        thread = threading.Thread(target=capture)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            stopped = self.workbench.motion_action(self.body('stop'), principal='other-cookie', live=True)
            self.assertTrue(stopped['release_confirmed'])
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsNone(self.workbench.control_owner)
        self.assertEqual(self.workbench.last_operation['action'], 'stop')

    def test_blur_and_hidden_hold_do_not_stop_but_owner_pagehide_does(self):
        self.workbench.motion_action(self.body())
        for source in ('window-blur', 'page-hidden'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('stop', stop_source=source))
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'hold')
        self.workbench.motion_action(self.body('stop', stop_source='pagehide'))
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'off')
        self.assertIsNone(self.workbench.control_owner)

    def test_independent_stance_rejects_all_automatic_stops_from_old_pages(self):
        self.runtime.motion['hardware']['hold_control_protocol']=2
        self.workbench.motion_action(self.body())
        before=len(self.runtime.calls)
        for source in ('window-blur','page-hidden','pagehide','workspace-leave'):
            with self.subTest(source=source), self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('stop',stop_source=source))
        self.assertEqual(len(self.runtime.calls),before)
        self.assertEqual(self.runtime.motion['hardware']['mode'],'hold')
        self.workbench.motion_action(self.body('stop',control_id=OTHER,generation=0))
        self.assertEqual(self.runtime.motion['hardware']['mode'],'off')

    def test_hidden_balance_stops_and_other_authenticated_device_can_emergency_stop(self):
        self.workbench.motion_action(self.body(mode='walk'))
        self.workbench.motion_action(self.body('stop', stop_source='page-hidden'))
        self.workbench.motion_action(self.body(mode='hold'))
        self.workbench.motion_action(self.body('stop', control_id=OTHER, generation=0))
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'off')

    def test_live_independent_hold_context_rejects_automatic_stop_without_snapshot(self):
        self.runtime.motion['hardware']['hold_control_protocol'] = 2
        started = self.workbench.motion_action(self.body(), live=True)
        before = len(self.runtime.calls)
        with mock.patch.object(self.runtime, 'snapshot', side_effect=AssertionError('full snapshot')):
            for source in ('page-hidden', 'pagehide', 'workspace-leave'):
                with self.subTest(source=source), self.assertRaisesRegex(ValueError, '独立保持'):
                    self.workbench.motion_action(self.body('stop', generation=started['generation'],
                                                          stop_source=source), live=True)
        self.assertEqual(len(self.runtime.calls), before)
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'hold')

    def test_another_login_cannot_forge_control_owner_or_take_active_motion(self):
        self.workbench.motion_action(self.body(), principal='first-cookie')
        before = len(self.runtime.calls)
        for action in ('heartbeat', 'start'):
            with self.subTest(action=action), self.assertRaisesRegex(ValueError, '另一页面|未启动'):
                self.workbench.motion_action(self.body(action), principal='second-cookie')
        with self.assertRaises(ValueError):
            self.workbench.motion_action(self.body('stop', stop_source='pagehide'), principal='second-cookie')
        self.assertEqual(len(self.runtime.calls), before)
        self.assertNotIn('first-cookie', json.dumps(self.workbench.snapshot()))
        self.workbench.motion_action(self.body('stop'), principal='second-cookie')
        self.assertEqual(self.runtime.motion['hardware']['mode'], 'off')

    def test_stop_interrupts_pending_start_and_late_start_cannot_restore_owner(self):
        entered, release = threading.Event(), threading.Event()
        original = self.runtime.action
        errors = []
        def slow_action(body):
            result = original(body)
            if body['action'] == 'start':
                entered.set()
                if not release.wait(2):
                    raise ValueError('test did not release start')
            return result
        self.runtime.action = slow_action
        def start():
            try:
                self.workbench.motion_action(self.body(), principal='first-cookie')
            except BaseException as error:
                errors.append(error)
        thread = threading.Thread(target=start)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            # This would deadlock until timeout if STOP shared the start io_lock.
            stopped = self.workbench.motion_action(self.body('stop'), principal='second-cookie')
            self.assertEqual(stopped['motion']['hardware']['mode'], 'off')
        finally:
            release.set()
            thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        snapshot = self.workbench.snapshot()
        self.assertIsNone(snapshot['motion']['control_owner'])
        self.assertEqual(snapshot['motion']['last_operation']['action'], 'stop')
        self.assertEqual(snapshot['motion']['last_operation']['state'], 'complete')

    def test_field_and_numeric_types_rejected_before_runtime(self):
        for changes in (dict(control_id='invalid'), dict(generation=True), dict(page_generation=True),
                        dict(mode='unknown'), dict(support_confirmed=False), dict(support_confirmed=1),
                        dict(mode='getup', behavior_confirmed=False), dict(preview_model='unknown'), dict(extra=True)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workbench.motion_action(self.body(**changes))
        self.workbench.motion_action(self.body(mode='walk'))
        before = len(self.runtime.calls)
        for number in (float('nan'), float('inf'), True, '0', .21):
            with self.subTest(number=number), self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('command', forward=number, side=0, yaw=0))
        self.assertEqual(len(self.runtime.calls), before)

    def test_runtime_failure_is_not_retried_and_audit_keeps_error(self):
        self.runtime.failure = 'real controller fault'
        with self.assertRaisesRegex(ValueError, 'real controller fault'):
            self.workbench.motion_action(self.body())
        self.assertEqual(len(self.runtime.calls), 1)
        self.assertEqual(self.workbench.snapshot()['motion']['last_operation']['state'], 'failed')
        self.workbench.initial_connect()
        self.assertEqual(self.runtime.calls[-1], ('connect',))
        self.assertEqual(self.workbench.snapshot()['motion']['phase'], 'fault')
        self.assertEqual(self.workbench.snapshot()['motion']['last_operation']['error'], 'real controller fault')

    def test_delayed_hold_display_dispatches_once_to_actual_runtime_gate(self):
        body = self.body()
        self.runtime.motion['last_hardware'] = self.runtime.motion.pop('hardware')
        self.runtime.motion['hardware'] = None
        self.runtime.motion['last_hardware']['age_ms'] = 850
        self.runtime.failure = 'live feedback is older than 100ms'
        with self.assertRaisesRegex(ValueError, 'live feedback is older than 100ms'):
            self.workbench.motion_action(body)
        self.assertEqual(self.runtime.calls, [('action', body)])
        self.assertIsNone(self.workbench.control_owner)
        self.assertEqual(self.workbench.last_operation['state'], 'failed')
        # This fallback is not available to dynamic actions or a dead owner.
        for changes in (dict(mode='walk'), dict(generation=4)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workbench.motion_action(dict(body, **changes))
        self.runtime.motion['owns_connection'] = False
        with self.assertRaises(ValueError):
            self.workbench.motion_action(body)
        self.assertEqual(len(self.runtime.calls), 1)

    def test_close_revokes_auth_and_closes_only_supplied_runtime(self):
        self.workbench.close()
        self.assertFalse(self.workbench.authenticated(web.COOKIE + '=' + self.cookie))
        self.assertEqual(self.runtime.calls, [('close',)])
        with self.assertRaisesRegex(ValueError, '已关闭'):
            self.workbench.motion_action(self.body())

    def test_private_credential_mode_symlink_and_shape(self):
        path = self.root / 'access-secret.json'
        path.write_text(json.dumps(dict(password=PASSWORD)))
        path.chmod(0o600)
        self.assertEqual(web.private_secret(path), PASSWORD)
        path.chmod(0o644)
        with self.assertRaises(ValueError):
            web.private_secret(path)
        path.chmod(0o600)
        link = self.root / 'link.json'
        link.symlink_to(path)
        with self.assertRaises(OSError):
            web.private_secret(link)
        path.write_text(json.dumps(dict(password='weak')))
        with self.assertRaises(ValueError):
            web.private_secret(path)

    def test_ui_preserves_original_and_disables_calibration_and_maintenance(self):
        source = SOURCE.read_text()
        page = web.native_page(source).decode()
        self.assertIn('canAction=canCancel=canEndFeedback=canRecoverFeedback=canMaintenance=', page)
        self.assertIn('canJointAction=canJointCancel=canZeroAction=()=>false', page)
        self.assertIn("recheckMotionFeedback=async()=>{if(canMotionRecheck())return runMotion('connect');}", page)
        self.assertIn('function motionHeartbeat()', page)
        self.assertIn("mode==='hold'", page)
        self.assertIn("stop_source:'page-hidden'", page)
        self.assertIn("stop_source:'pagehide'", page)
        self.assertIn('已保存校准 · 只读', page)
        self.assertEqual(page.replace(web.READ_ONLY_SCRIPT + '\n', ''),
                         source.replace("fetch('/api/status',", "fetch('/api/live-status',"))

    def test_live_status_route_preserves_current_hardware_and_requires_authentication(self):
        code, live, _ = self.request('GET', '/api/live-status')
        self.assertEqual(code, 200)
        self.assertEqual(live['motion']['hardware'], self.runtime.motion['hardware'])
        live['motion']['hardware']['generation'] = -1
        self.assertNotEqual(self.runtime.motion['hardware']['generation'], -1)
        self.assertEqual(self.runtime.calls, [])

    def test_actual_native_unified_page_in_existing_vm_dom_harness(self):
        ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
        self.runtime.calibration = dict(version=1, mechanical_reference_confirmed=True,
            joints=[dict(id=sid, zero=2048, sign=1, min=1000, max=3500, direction_confirmed=True) for sid in ids],
            imu=dict(axes=['-z', '-x', 'y'], axes_confirmed=True, calibrated=True, bias_dps=[0, 0, 0]))
        self.runtime.motion['deployment'].update(model_hashes={name: 'a' * 64 for name in web.MODEL_NAMES},
                                                  calibration_sha256='actual-calibration')
        hardware = self.runtime.motion['hardware']
        hardware.update(age_ms=0, imu_age_ms=0, imu_ok=True, calibrated=True,
                        policy_metadata_protocol=1, target_validation_protocol=1, behavior_protocol=1,
                        stand_target_deg=[0] * 15, hold_ready=False, online_ids=ids, torque_ids=[], fault_ids=[],
                        rates=dict(state_hz=47, control_hz=0),
                        rawdata=[dict(id=sid, position=2048) for sid in ids],
                        start_blocked_reason=None, model_start_blocked_reasons={})
        for authentication_required in (True, False):
            with self.subTest(authentication_required=authentication_required):
                workbench = web.NativeWorkbench(self.runtime, web_root=self.root, password=PASSWORD,
                                                authentication_required=authentication_required,
                                                clock=lambda: self.now)
                page = self.root / 'native-page.html'
                page.write_bytes(workbench.page)
                fixture = self.root / 'native-fixture.json'
                fixture.write_text(json.dumps(workbench.snapshot()))
                result = subprocess.run([shutil.which('node'), str(Path(__file__).with_name('test_native_workbench_ui.js')),
                                          str(page), str(fixture)], capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('checks passed', result.stdout)

    def test_bounded_threads_reject_immediately_without_dispatch(self):
        server = object.__new__(web.BoundedHTTPServer)
        server.slots = threading.BoundedSemaphore(1)
        server.slots.acquire()
        connection = mock.Mock()
        server.shutdown_request = mock.Mock()
        with mock.patch.object(web.ThreadingHTTPServer, 'process_request') as dispatch:
            server.process_request(connection, ('192.168.1.20', 1))
            dispatch.assert_not_called()
        self.assertIn(b'503', connection.sendall.call_args.args[0])
        server.shutdown_request.assert_called_once_with(connection)

    def test_serve_rejects_other_host_and_disk_secret_before_socket(self):
        with mock.patch.object(web, 'BoundedHTTPServer', side_effect=AssertionError('no sockets')):
            with self.assertRaises(ValueError):
                web.serve(self.runtime, web_root=self.root, secret_file=self.root / 'secret', host='0.0.0.0')
            with self.assertRaisesRegex(ValueError, 'RAM'):
                web.serve(self.runtime, web_root=self.root, secret_file=self.root / 'secret')

    def test_serve_connect_fault_keeps_bound_web_ready_and_closes_on_exit(self):
        self.runtime.failure = 'real connection fault'
        socket = mock.Mock()
        socket.fileno.return_value = 10
        server = mock.Mock(socket=socket, server_address=(web.HOST, web.PORT))
        info = []
        with mock.patch.object(web, 'private_secret', return_value=PASSWORD), \
             mock.patch.object(web, 'BoundedHTTPServer', return_value=server), \
             mock.patch.object(web.os, 'fstat', return_value=mock.Mock(st_ino=101)), \
             mock.patch.object(web.signal, 'signal', return_value=object()):
            web.serve(self.runtime, web_root=self.root, secret_file='/dev/shm/private/access-secret.json',
                      ready_callback=lambda value: info.append(value))
        server.serve_forever.assert_called_once()
        self.assertEqual(info[0]['address'], web.HOST)
        self.assertEqual(info[0]['listener_inode'], 101)
        self.assertEqual(info[0]['motion']['phase'], 'fault')
        self.assertEqual(self.runtime.calls, [('connect',), ('close',)])

    def test_anonymous_serve_never_reads_secret_and_reports_actual_access_mode(self):
        socket = mock.Mock()
        socket.fileno.return_value = 10
        server = mock.Mock(socket=socket, server_address=(web.HOST, web.PORT))
        info = []
        with mock.patch.object(web, 'private_secret', side_effect=AssertionError('no password reads')), \
             mock.patch.object(web, 'BoundedHTTPServer', return_value=server), \
             mock.patch.object(web.os, 'fstat', return_value=mock.Mock(st_ino=101)), \
             mock.patch.object(web.signal, 'signal', return_value=object()):
            web.serve(self.runtime, web_root=self.root, authentication_required=False,
                      ready_callback=lambda value: info.append(value))
        server.serve_forever.assert_called_once()
        self.assertFalse(info[0]['auth_required'])
        self.assertEqual(self.runtime.calls, [('connect',), ('close',)])

    def test_serve_controlled_signals_close_resources_and_exit_zero(self):
        for signum in (web.signal.SIGTERM, web.signal.SIGINT):
            with self.subTest(signum=signum):
                registered, cleanup = {}, []
                previous = object()
                def register(signal_number, handler):
                    registered[signal_number] = handler
                    return previous
                workbench = mock.Mock()
                workbench.close.side_effect = lambda: cleanup.append('workbench')
                server = mock.Mock(server_address=(web.HOST, web.PORT))
                server.socket.fileno.return_value = 10
                server.server_close.side_effect = lambda: cleanup.append('server')
                server.serve_forever.side_effect = lambda **_: registered[signum](signum, None)
                with (mock.patch.object(web, 'NativeWorkbench', return_value=workbench),
                      mock.patch.object(web, 'BoundedHTTPServer', return_value=server),
                      mock.patch.object(web.os, 'fstat', return_value=mock.Mock(st_ino=101)),
                      mock.patch.object(web.signal, 'signal', side_effect=register) as signals,
                      self.assertRaises(SystemExit) as stopped):
                    web.serve(self.runtime, web_root=self.root, authentication_required=False,
                              auto_connect=False)
                self.assertEqual(stopped.exception.code, 0)
                self.assertEqual(cleanup, ['server', 'workbench'])
                server.server_close.assert_called_once_with()
                workbench.close.assert_called_once_with()
                self.assertEqual(signals.call_args_list[-2:], [
                    mock.call(web.signal.SIGTERM, previous), mock.call(web.signal.SIGINT, previous)])

    def test_serve_cleanup_failure_overrides_controlled_zero_exit(self):
        registered = {}
        def register(signum, handler):
            registered[signum] = handler
            return object()
        workbench = mock.Mock()
        workbench.close.side_effect = RuntimeError('UART closure not confirmed')
        server = mock.Mock(server_address=(web.HOST, web.PORT))
        server.socket.fileno.return_value = 10
        server.serve_forever.side_effect = lambda **_: registered[web.signal.SIGTERM](web.signal.SIGTERM, None)
        with (mock.patch.object(web, 'NativeWorkbench', return_value=workbench),
              mock.patch.object(web, 'BoundedHTTPServer', return_value=server),
              mock.patch.object(web.os, 'fstat', return_value=mock.Mock(st_ino=101)),
              mock.patch.object(web.signal, 'signal', side_effect=register),
              self.assertRaisesRegex(RuntimeError, 'UART closure not confirmed')):
            web.serve(self.runtime, web_root=self.root, authentication_required=False, auto_connect=False)
        server.server_close.assert_called_once_with()
        workbench.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
