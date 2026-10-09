#!/usr/bin/env python3
"""Loopback model workbench with optional calibrated, supervised motor control."""
import argparse
import json
import math
import platform
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import onnxruntime as ort
from infer import NAMES, infer, session
from rl_core import DEFAULT_DEG, HOME_DEG, POLICY_IDS, State, build_obs

HERE = Path(__file__).resolve().parent
LABELS = dict(walk='行走', getup='起身', pick='抓取', roulade='翻滚')
ROLES = ['腿部旋转', '髋侧摆', '髋俯仰', '膝关节', '踝关节',
         '颈俯仰', '头俯仰', '头旋转', '头侧摆',
         '腿部旋转', '髋侧摆', '髋俯仰', '膝关节', '踝关节']


def number(body, key, low, high):
    value = body.get(key, 0)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} 必须是数字')
    try:
        value = float(value)
    except OverflowError:
        raise ValueError(f'{key} 超出数字范围') from None
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{key} 范围是 {low} 到 {high}')
    return value


def synthetic_observation(body):
    name = body.get('model', 'walk')
    if name not in NAMES:
        raise ValueError('未知模型')
    forward = number(body, 'forward', -.4, .4)
    side = number(body, 'side', -.3, .3)
    yaw = number(body, 'yaw', -1, 1)
    roll = math.radians(number(body, 'roll', -30, 30))
    pitch = math.radians(number(body, 'pitch', -30, 30))
    phase = number(body, 'phase', 0, 1)
    gravity = [math.sin(pitch), -math.sin(roll) * math.cos(pitch),
               -math.cos(roll) * math.cos(pitch)]
    state = State(acc=[0]*3, gyro=[0]*3, angles=list(DEFAULT_DEG), vels=[0]*15,
                  grav=gravity, gyro_rad=[0]*3)
    command = [forward, side, yaw] if name == 'walk' else [0]*3
    if name == 'pick':
        command = [math.cos(2*math.pi*phase), math.sin(2*math.pi*phase), 0]
    observation = build_obs(state, np.zeros(14, np.float32),
                            np.asarray(command, np.float32), np.zeros(4, np.float32))
    return name, observation.reshape(1, 61)


class Workbench:
    def __init__(self, motion_port=None):
        self.token = secrets.token_urlsafe(32)
        self.sessions = {}
        self.errors = {}
        self.lock = threading.Lock()
        for name in NAMES:
            try:
                s = session(name)
                _, obs = synthetic_observation({'model': name})
                for _ in range(12):
                    infer(s, obs)
                self.sessions[name] = s
            except Exception as exc:
                self.errors[name] = str(exc)
        try:
            self.benchmark = json.loads((HERE/'benchmark.json').read_text())
        except (OSError, ValueError):
            self.benchmark = None
        self.motion = None
        if motion_port:
            from motion import MotionController
            self.motion = MotionController(self.sessions, port=motion_port,
                                           config_path=HERE/'calibration.json')
            self.motion.start()

    def status(self):
        return dict(app='xgoduck-policy-workbench', online=True, token=self.token,
                    device=dict(hostname=platform.node(), architecture=platform.machine(),
                                python=platform.python_version(), onnxruntime=ort.__version__),
                    models=[dict(id=n, name=LABELS[n], ready=n in self.sessions,
                                 error=self.errors.get(n), input=[1,61], output=[1,14]) for n in NAMES],
                    benchmark=self.benchmark,
                    hardware=(self.motion_status() if self.motion else
                              dict(calibrated=False, live_feedback=False, motor_output=False)))

    def motion_status(self):
        if self.motion:
            return self.motion.status()
        return dict(connected=False, mode='off', generation=0, calibrated=False, motor_output=False,
                    armed=False, joints=[], rawdata=[], online_ids=[], torque_ids=[], fault_ids=[],
                    mechanical_reference_confirmed=False, hold_ready=False,
                    imu=dict(axes=['x','y','z'], axes_confirmed=False, calibrated=False,
                             calibrating=False, progress=0),
                    error='真机接口尚未启用；需要更新 ESP32 控制固件。')

    def motion_action(self, path, body):
        if not self.motion:
            raise ValueError('真机接口尚未启用；请先更新控制固件')
        m = self.motion
        if path == '/api/motion/stop':
            return m.stop()
        if path == '/api/motion/heartbeat':
            return m.heartbeat()
        if path == '/api/motion/capture':
            sid=body.get('id')
            if type(sid) is not int:
                raise ValueError('关节 ID 必须是整数')
            return m.capture(sid, body.get('field'))
        if path == '/api/motion/joint':
            sid, sign = body.get('id'), body.get('sign')
            if type(sid) is not int or type(sign) is not int or type(body.get('direction_confirmed')) is not bool:
                raise ValueError('关节参数格式错误')
            return m.configure_joint(sid, sign, body['direction_confirmed'])
        if path == '/api/motion/reference':
            if type(body.get('confirmed')) is not bool:
                raise ValueError('需要明确确认零姿态参考')
            return m.configure_reference(body['confirmed'])
        if path == '/api/motion/imu-axes':
            if type(body.get('axes_confirmed')) is not bool:
                raise ValueError('需要明确确认 IMU 轴向')
            return m.set_imu_axes(body.get('axes'), axes_confirmed=body['axes_confirmed'])
        if path == '/api/motion/imu-calibrate':
            return m.calibrate_imu()
        if path == '/api/motion/start':
            mode=body.get('mode')
            if type(body.get('generation')) is not int:
                raise ValueError('请刷新真机状态后启动')
            if mode != 'shadow' and (body.get('support_confirmed') is not True or body.get('zero_reference_confirmed') is not True):
                raise ValueError('请先支撑机身并确认关节零姿态')
            commands=(number(body,'forward',-.2,.2),number(body,'side',-.1,.1),number(body,'yaw',-.5,.5))
            result=m.start_mode(mode, support_confirmed=body.get('support_confirmed') is True,
                                expected_generation=body['generation'])
            if mode == 'walk':
                try:
                    result=m.command(*commands)
                except Exception:
                    m.stop()
                    raise
            return result
        if path == '/api/motion/command':
            return m.command(number(body,'forward',-.2,.2),number(body,'side',-.1,.1),number(body,'yaw',-.5,.5))
        raise ValueError('未知真机操作')

    def predict(self, body):
        name, obs = synthetic_observation(body)
        if name not in self.sessions:
            raise ValueError('模型未加载：' + self.errors.get(name, name))
        start = time.perf_counter_ns()
        action = infer(self.sessions[name], obs).reshape(14)
        elapsed = (time.perf_counter_ns()-start)/1e6
        offsets = np.rad2deg(action)
        actions = [dict(id=sid, role=ROLES[i], radians=float(action[i]),
                        degrees_relative_home=float(offsets[i]),
                        target_degrees=float(HOME_DEG[i]+offsets[i])) for i, sid in enumerate(POLICY_IDS)]
        return dict(model=name, elapsed_ms=elapsed, observation=obs.tolist(), actions=actions,
                    source='synthetic', motor_output=False, previous_action='zero')

    def measure(self):
        results = {}
        for name, s in self.sessions.items():
            _, obs = synthetic_observation({'model': name})
            for _ in range(50):
                infer(s, obs)
            elapsed = []
            for _ in range(1000):
                start = time.perf_counter_ns()
                infer(s, obs)
                elapsed.append((time.perf_counter_ns()-start)/1e6)
            results[name] = dict(mean_ms=float(np.mean(elapsed)), p95_ms=float(np.percentile(elapsed,95)),
                                 p99_ms=float(np.percentile(elapsed,99)), max_ms=max(elapsed),
                                 iterations=1000, finite=True, provider=s.get_providers())
        if not results:
            raise ValueError('没有可测试的模型')
        result = dict(test='synthetic_observation_cpu_inference_only', onnxruntime=ort.__version__,
                      models=results)
        self.benchmark = result
        (HERE/'benchmark.json').write_text(json.dumps(result,indent=2)+'\n')
        return result


def make_handler(workbench):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, body, content_type='application/json; charset=utf-8'):
            data = body if isinstance(body, bytes) else json.dumps(body, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(data)

        def valid_host(self):
            port = self.server.server_address[1]
            return self.headers.get('Host') in (f'127.0.0.1:{port}', f'localhost:{port}')

        def do_GET(self):
            if not self.valid_host():
                return self.reply(403, {'error':'仅允许本机访问'})
            if self.path == '/api/status':
                return self.reply(200, workbench.status())
            if self.path == '/api/motion/status':
                return self.reply(200, workbench.motion_status())
            if self.path == '/zero-reference.png':
                return self.reply(200, (HERE/'assets/zero-reference.png').read_bytes(), 'image/png')
            if self.path == '/positive-direction.png':
                return self.reply(200, (HERE/'assets/positive-direction.png').read_bytes(), 'image/png')
            if self.path == '/':
                return self.reply(200, (HERE/'index.html').read_bytes(), 'text/html; charset=utf-8')
            self.reply(404, {'error':'找不到页面'})

        def do_POST(self):
            if not self.valid_host():
                return self.reply(403, {'error':'仅允许本机访问'})
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                return self.reply(403, {'error':'来源不匹配'})
            if not secrets.compare_digest(self.headers.get('X-Session-Token', '').encode('utf-8'),
                                          workbench.token.encode('ascii')):
                return self.reply(403, {'error':'请刷新页面后重试'})
            motion_paths = ('/api/motion/stop','/api/motion/heartbeat','/api/motion/capture',
                            '/api/motion/joint','/api/motion/reference','/api/motion/imu-axes',
                            '/api/motion/imu-calibrate','/api/motion/start','/api/motion/command')
            if self.path not in ('/api/infer','/api/benchmark', *motion_paths):
                return self.reply(404, {'error':'未知测试'})
            if self.headers.get('Content-Type','').split(';')[0] != 'application/json':
                return self.reply(415, {'error':'需要 JSON 请求'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    return self.reply(413, {'error':'请求长度无效'})
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError('请求必须是对象')
            except (ValueError, TypeError):
                return self.reply(400, {'error':'请求格式错误'})
            # Stop and heartbeat must not wait behind model benchmarks/calibration.
            if self.path in ('/api/motion/stop','/api/motion/heartbeat'):
                try:
                    return self.reply(200, workbench.motion_action(self.path,body))
                except ValueError as exc:
                    return self.reply(400, {'error':str(exc)})
            if not workbench.lock.acquire(blocking=False):
                return self.reply(409, {'error':'测试正在进行，请稍后重试'})
            try:
                if self.path in motion_paths:
                    result = workbench.motion_action(self.path,body)
                elif self.path == '/api/infer':
                    result = workbench.predict(body)
                else:
                    if workbench.motion_status().get('mode') != 'off':
                        raise ValueError('请先停止真机控制，再测量性能')
                    result = workbench.measure()
                self.reply(200, result)
            except (ValueError, TypeError) as exc:
                self.reply(400, {'error':str(exc)})
            except Exception:
                self.reply(500, {'error':'测试失败，请查看 X5 服务日志'})
            finally:
                workbench.lock.release()

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--motion-port', help='已更新控制固件的 ESP32 COM 串口，例如 /dev/ttyUSB0')
    args = parser.parse_args()
    workbench = Workbench(motion_port=args.motion_port)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(workbench))
    print(f'X5 模型测试：http://127.0.0.1:{args.port}/', flush=True)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if workbench.motion:
            workbench.motion.close()
        server.server_close()


if __name__ == '__main__':
    main()
