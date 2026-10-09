"""Motion HTTP routing, authorization and cancellation without hardware/models."""
import http.client
import json
import threading
import unittest
from http.server import ThreadingHTTPServer

from web_app import Workbench, make_handler


class FakeMotion:
    def __init__(self):
        self.mode = 'off'
        self.calibrated = False
        self.generation = 7
        self.calls = []
        self.fail_command = False

    def status(self):
        return dict(connected=True, mode=self.mode, generation=self.generation,
                    calibrated=self.calibrated, motor_output=self.mode in ('hold', 'walk'),
                    armed=self.mode in ('hold', 'walk'), joints=[], rawdata=[], online_ids=[],
                    torque_ids=[], fault_ids=[], imu={})

    def start_mode(self, mode, support_confirmed=False, *, expected_generation=None):
        if expected_generation != self.generation:
            raise ValueError('旧的启动请求已取消')
        if mode not in ('shadow', 'hold', 'walk'):
            raise ValueError('不支持此动作')
        if mode != 'shadow' and not self.calibrated:
            raise ValueError('校准尚未完成')
        self.calls.append(('start', mode, support_confirmed, expected_generation))
        self.mode = mode
        self.generation += 1
        return self.status()

    def stop(self):
        self.calls.append(('stop',))
        self.mode = 'off'
        self.generation += 1
        return self.status()

    def heartbeat(self):
        self.calls.append(('heartbeat',))
        return self.status()

    def command(self, *values):
        self.calls.append(('command', *values))
        if self.fail_command:
            raise ValueError('指令发送失败')
        return self.status()

    def capture(self, sid, field):
        self.calls.append(('capture', sid, field))
        return self.status()

    def configure_joint(self, sid, sign, direction_confirmed):
        self.calls.append(('joint', sid, sign, direction_confirmed))
        return self.status()

    def configure_reference(self, confirmed):
        self.calls.append(('reference', confirmed))
        return self.status()

    def set_imu_axes(self, axes, axes_confirmed=False):
        self.calls.append(('axes', axes, axes_confirmed))
        return self.status()

    def calibrate_imu(self):
        self.calls.append(('imu-calibrate',))
        return self.status()


class FakeWorkbench:
    token = 'test-motion-token'
    motion_status = Workbench.motion_status
    motion_action = Workbench.motion_action

    def __init__(self):
        self.lock = threading.Lock()
        self.motion = FakeMotion()
        self.calls = []

    def status(self):
        return dict(token=self.token, hardware=self.motion_status())

    def predict(self, body):
        self.calls.append('infer')
        return {'motor_output': False}

    def measure(self):
        self.calls.append('benchmark')
        return {'motor_output': False}


class MotionRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workbench = FakeWorkbench()
        handler = make_handler(cls.workbench)
        handler.log_message = lambda *args: None
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def setUp(self):
        self.workbench.motion = FakeMotion()
        self.workbench.calls.clear()

    def request(self, path, body=None, *, method='POST', headers=None):
        defaults = dict(Host=f'127.0.0.1:{self.port}', **{'Content-Type': 'application/json',
                        'X-Session-Token': self.workbench.token})
        defaults.update(headers or {})
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        try:
            connection.request(method, path, json.dumps(body or {}), defaults)
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def start_body(self, mode='hold', **extra):
        body = dict(mode=mode, generation=self.workbench.motion.generation,
                    support_confirmed=True, zero_reference_confirmed=True)
        body.update(extra)
        return body

    def test_status_and_disabled_motion_are_explicit(self):
        code, status = self.request('/api/motion/status', method='GET')
        self.assertEqual(code, 200)
        self.assertTrue(status['connected'])
        self.workbench.motion = None
        code, status = self.request('/api/motion/status', method='GET')
        self.assertEqual(code, 200)
        self.assertFalse(status['connected'])
        for path in ('stop', 'heartbeat', 'capture', 'joint', 'reference', 'imu-axes',
                     'imu-calibrate', 'start', 'command'):
            code, data = self.request('/api/motion/' + path)
            self.assertEqual(code, 400, path)
            self.assertIn('未启用', data['error'])

    def test_motion_routes_require_token_host_and_origin(self):
        for path in ('/api/motion/stop', '/api/motion/heartbeat', '/api/motion/start',
                     '/api/motion/capture'):
            for headers in ({'X-Session-Token': ''}, {'X-Session-Token': 'wrong'},
                            {'Origin': 'https://attacker.example'}, {'Host': 'attacker.example'}):
                self.assertEqual(self.request(path, self.start_body(), headers=headers)[0], 403)
        self.assertEqual(self.workbench.motion.calls, [])
        self.assertEqual(self.request('/api/motion/status', method='GET',
                                     headers={'Host': 'attacker.example'})[0], 403)

    def test_shadow_can_read_before_calibration_without_support_confirmation(self):
        self.assertFalse(self.workbench.motion.calibrated)
        code, status = self.request('/api/motion/start',
                                    dict(mode='shadow', generation=self.workbench.motion.generation))
        self.assertEqual(code, 200)
        self.assertEqual(status['mode'], 'shadow')
        self.assertFalse(status['motor_output'])
        self.assertEqual(self.workbench.motion.calls, [('start', 'shadow', False, 7)])

    def test_start_requires_strict_support_reference_and_generation(self):
        self.workbench.motion.calibrated = True
        invalid = [dict(support_confirmed=False), dict(support_confirmed=1),
                   dict(support_confirmed='true'), dict(zero_reference_confirmed=False),
                   dict(zero_reference_confirmed=1), dict(generation=True),
                   dict(generation='7'), dict(generation=None), dict(forward=True),
                   dict(forward=.21), dict(yaw=float('nan'))]
        for extra in invalid:
            with self.subTest(extra=extra):
                self.assertEqual(self.request('/api/motion/start', self.start_body(**extra))[0], 400)
        self.assertEqual(self.workbench.motion.calls, [])
        self.workbench.motion.calibrated = False
        code, data = self.request('/api/motion/start', self.start_body())
        self.assertEqual(code, 400)
        self.assertIn('校准', data['error'])
        self.assertEqual(self.workbench.motion.calls, [])

    def test_stop_and_heartbeat_bypass_busy_model_lock(self):
        self.workbench.motion.calibrated = True
        self.assertEqual(self.request('/api/motion/start', self.start_body())[0], 200)
        with self.workbench.lock:
            self.assertEqual(self.request('/api/infer')[0], 409)
            self.assertEqual(self.request('/api/motion/heartbeat')[0], 200)
            code, status = self.request('/api/motion/stop')
            self.assertEqual(code, 200)
            self.assertEqual(status['mode'], 'off')
            self.assertFalse(status['motor_output'])
        self.assertEqual(self.workbench.motion.calls[-2:], [('heartbeat',), ('stop',)])

    def test_stop_invalidates_old_start_and_unsupported_model_actions(self):
        self.workbench.motion.calibrated = True
        old_start = self.start_body()
        self.assertEqual(self.request('/api/motion/stop')[0], 200)
        self.assertEqual(self.request('/api/motion/start', old_start)[0], 400)
        self.assertEqual(self.workbench.motion.calls, [('stop',)])
        for mode in ('getup', 'pick', 'roulade'):
            self.assertEqual(self.request('/api/motion/start', self.start_body(mode))[0], 400)
        self.assertEqual(self.workbench.motion.calls, [('stop',)])

    def test_calibration_route_args_are_strict_and_pass_through(self):
        for path, body in (('capture', {'id': True, 'field': 'zero'}),
                           ('joint', {'id': 10, 'sign': -1, 'direction_confirmed': 1}),
                           ('joint', {'id': 10, 'sign': True, 'direction_confirmed': True}),
                           ('reference', {'confirmed': 1}),
                           ('imu-axes', {'axes': ['x', 'y', 'z'], 'axes_confirmed': 1})):
            self.assertEqual(self.request('/api/motion/' + path, body)[0], 400)
        self.assertEqual(self.workbench.motion.calls, [])
        routes = [('capture', {'id': 10, 'field': 'zero'}),
                  ('joint', {'id': 10, 'sign': -1, 'direction_confirmed': True}),
                  ('reference', {'confirmed': True}),
                  ('imu-axes', {'axes': ['x', 'y', 'z'], 'axes_confirmed': True}),
                  ('imu-calibrate', {})]
        for path, body in routes:
            self.assertEqual(self.request('/api/motion/' + path, body)[0], 200)
        self.assertEqual(self.workbench.motion.calls,
                         [('capture', 10, 'zero'), ('joint', 10, -1, True), ('reference', True),
                          ('axes', ['x', 'y', 'z'], True), ('imu-calibrate',)])

    def test_benchmark_refuses_active_motion_and_walk_command_failure_stops(self):
        self.workbench.motion.mode = 'hold'
        self.assertEqual(self.request('/api/benchmark')[0], 400)
        self.assertEqual(self.workbench.calls, [])
        self.workbench.motion.mode = 'off'
        self.assertEqual(self.request('/api/benchmark')[0], 200)
        self.workbench.motion.calibrated = True
        self.workbench.motion.fail_command = True
        self.assertEqual(self.request('/api/motion/start', self.start_body('walk', forward=.1))[0], 400)
        self.assertEqual(self.workbench.motion.mode, 'off')
        self.assertEqual(self.workbench.motion.calls[-1], ('stop',))


if __name__ == '__main__':
    unittest.main()
