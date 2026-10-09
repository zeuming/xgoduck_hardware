"""Workbench boundaries and HTTP isolation, without ONNX models or hardware."""
import http.client
import json
import math
import threading
import unittest
from http.server import ThreadingHTTPServer

import numpy as np
from web_app import make_handler, number, synthetic_observation


class ObservationTests(unittest.TestCase):
    def test_numeric_boundaries_and_rejected_values(self):
        for value in (-.4, 0, .4):
            self.assertEqual(number({'forward': value}, 'forward', -.4, .4), value)
        for value in (-.4001, .4001, True, False, None, '0', [], {}, 10**1000,
                      float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                number({'forward': value}, 'forward', -.4, .4)
        self.assertEqual(number({}, 'forward', -.4, .4), 0)

    def test_level_home_and_walk_command(self):
        name, obs = synthetic_observation({'forward': .2, 'side': -.1, 'yaw': .5})
        self.assertEqual(name, 'walk')
        self.assertEqual(obs.shape, (1, 61))
        self.assertEqual(obs.dtype, np.float32)
        expected = np.zeros((1, 61), np.float32)
        expected[0, 5] = -1
        expected[0, 48:51] = [.2, -.1, .5]
        np.testing.assert_array_equal(obs, expected)

    def test_attitude_and_nonwalk_commands(self):
        _, obs = synthetic_observation({'roll': 30, 'pitch': -20})
        roll, pitch = math.radians(30), math.radians(-20)
        np.testing.assert_allclose(obs[0, 3:6],
            [math.sin(pitch), -math.sin(roll)*math.cos(pitch),
             -math.cos(roll)*math.cos(pitch)], atol=1e-7)
        self.assertAlmostEqual(float(np.linalg.norm(obs[0, 3:6])), 1, places=6)
        for model in ('getup', 'roulade'):
            _, obs = synthetic_observation({'model': model, 'forward': .4, 'yaw': 1})
            np.testing.assert_array_equal(obs[0, 48:51], [0, 0, 0])
        _, obs = synthetic_observation({'model': 'pick', 'phase': .25})
        np.testing.assert_allclose(obs[0, 48:51], [0, 1, 0], atol=1e-7)
        for body in ({'model': 'fly'}, {'roll': 31}, {'pitch': -31}, {'phase': 1.1},
                     {'side': -.31}, {'yaw': 1.1}):
            with self.subTest(body=body), self.assertRaises(ValueError):
                synthetic_observation(body)


class FakeWorkbench:
    token = 'test-session-token'

    def __init__(self):
        self.lock = threading.Lock()
        self.calls = []

    def status(self):
        return {'token': self.token, 'hardware': {'motor_output': False}}

    def motion_status(self):
        return {'connected': False, 'mode': 'off', 'calibrated': False, 'motor_output': False}

    def motion_action(self, path, body):
        raise ValueError('真机接口尚未启用')

    def predict(self, body):
        name, obs = synthetic_observation(body)
        self.calls.append('infer')
        return {'model': name, 'observation': obs.tolist(), 'motor_output': False}

    def measure(self):
        self.calls.append('benchmark')
        return {'test': 'synthetic_observation_cpu_inference_only'}


class HTTPTests(unittest.TestCase):
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
        self.workbench.calls.clear()

    def request(self, path, method='POST', body=b'{}', headers=None):
        defaults = {'Host': f'127.0.0.1:{self.port}', 'Content-Type': 'application/json',
                    'X-Session-Token': self.workbench.token}
        defaults.update(headers or {})
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        try:
            conn.request(method, path, body=body, headers=defaults)
            response = conn.getresponse()
            return response.status, dict(response.getheaders()), json.loads(response.read())
        finally:
            conn.close()

    def test_status_and_valid_requests(self):
        code, headers, data = self.request('/api/status', 'GET')
        self.assertEqual(code, 200)
        self.assertEqual(data['token'], self.workbench.token)
        self.assertEqual(headers['X-Frame-Options'], 'DENY')
        self.assertEqual(headers['Cache-Control'], 'no-store')
        for path in ('/api/infer', '/api/benchmark'):
            self.assertEqual(self.request(path)[0], 200)
        self.assertEqual(self.workbench.calls, ['infer', 'benchmark'])

    def test_host_origin_token_and_absent_hardware_routes(self):
        for headers in ({'Host': 'attacker.example'}, {'Origin': 'https://attacker.example'},
                        {'X-Session-Token': ''}, {'X-Session-Token': 'wrong'},
                        {'X-Session-Token': '\u00e9'}):
            self.assertEqual(self.request('/api/infer', headers=headers)[0], 403)
        self.assertEqual(self.request('/api/status', 'GET', headers={'Host': 'attacker.example'})[0], 403)
        for path in ('/api/motors', '/api/calibrate', '/api/hardware', '/api/infer?motor=1'):
            self.assertEqual(self.request(path)[0], 404)
        self.assertEqual(self.workbench.calls, [])

    def test_malformed_requests_and_busy_lock(self):
        for body, headers, code in [(b'[]', {}, 400), (b'{', {}, 400),
                (b'{"forward":NaN}', {}, 400), (b'{}', {'Content-Type': 'text/plain'}, 415),
                (b'', {}, 413), (b' '*4097, {}, 413),
                (b'{}', {'Content-Length': 'invalid'}, 400)]:
            self.assertEqual(self.request('/api/infer', body=body, headers=headers)[0], code)
        with self.workbench.lock:
            self.assertEqual(self.request('/api/infer')[0], 409)
        self.assertEqual(self.workbench.calls, [])


if __name__ == '__main__':
    unittest.main()
