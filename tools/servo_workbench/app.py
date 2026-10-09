#!/usr/bin/env python3
"""Local-only HD1910 workbench; opening the page never moves or renumbers a servo."""
import argparse
import copy
import json
import math
from pathlib import Path
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from serial.tools import list_ports
from protocol import Bus, BusError, NoReply, ServoFault, integer

HERE = Path(__file__).resolve().parent
BAUDS = (1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400)


class Cancelled(Exception):
    pass


class Controller:
    def __init__(self, bus_factory=Bus, port_source=list_ports.comports, monitor=True,
                 registry_path=None, transport='direct', bridge_port=''):
        self.bus_factory, self.port_source = bus_factory, port_source
        self.transport, self.bridge_port = transport, bridge_port
        self.lock = threading.RLock()
        self.bus_lock = threading.Lock()
        self.cancel = threading.Event()
        self.closed = threading.Event()
        self.worker = None
        self.registry_path = Path(registry_path) if registry_path else None
        self.roster_cursor = 0
        self.state = dict(ports=[], port='', selected_port=bridge_port, baud=1000000,
                          sid=10 if transport == 'esp32' else 1,
                          transport=transport,
                          connection_label='X5 → ESP32 → URT-2' if transport == 'esp32' else 'URT-2',
                          host_baud=115200 if transport == 'esp32' else 1000000,
                          servo_baud=1000000,
                          adapter=False, connected=False, servo=None, busy=False,
                          action='', progress=0, found=[], message='正在查找 URT-2…',
                          severity='info', logs=[], last_seen=None,
                          scan_results=[], scan_status='idle', scan_finished=None, known_servos=[])
        if self.registry_path and self.registry_path.exists():
            try:
                ids = json.loads(self.registry_path.read_text())
                if not isinstance(ids, list):
                    raise ValueError('编号记录格式错误')
                for sid in ids:
                    integer(sid, 0, 253, '已记录 ID')
                self.state['known_servos'] = [self.roster_row(sid) for sid in sorted(set(ids))]
            except (OSError, ValueError) as exc:
                self.log(f'无法加载编号记录：{exc}', 'error')
        if monitor:
            threading.Thread(target=self.monitor, daemon=True).start()

    def update(self, **kw):
        with self.lock:
            self.state.update(kw)

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.state)
            result['server_now'] = time.time()
            return result

    def log(self, text, level='info'):
        with self.lock:
            self.state['logs'].insert(0, dict(time=time.strftime('%H:%M:%S'), at=time.time(),
                                            text=text, level=level))
            del self.state['logs'][60:]
            self.state.update(message=text, severity=level)

    @staticmethod
    def roster_row(sid):
        return dict(id=sid, status='unknown', checked_at=None, last_seen=None, error='')

    def save_registry(self):
        if self.registry_path:
            try:
                temp = self.registry_path.with_suffix('.tmp')
                temp.write_text(json.dumps([r['id'] for r in self.state['known_servos']]))
                temp.replace(self.registry_path)
            except OSError as exc:
                self.log(f'编号记录保存失败：{exc}', 'error')

    def record_presence(self, sid, status, error=''):
        with self.lock:
            rows = self.state['known_servos']
            row = next((r for r in rows if r['id'] == sid), None)
            if row is None:
                if status not in ('online', 'fault'):
                    return
                row = self.roster_row(sid)
                rows.append(row)
                rows.sort(key=lambda r: r['id'])
                self.save_registry()
            row.update(status=status, checked_at=time.time(), error=error)
            if status in ('online', 'fault'):
                row['last_seen'] = row['checked_at']

    def presence_error(self, sid, exc):
        status = 'fault' if isinstance(exc, ServoFault) else 'offline' if isinstance(exc, NoReply) else 'unknown'
        self.record_presence(sid, status, str(exc))

    def reset_presence(self, reason):
        with self.lock:
            for row in self.state['known_servos']:
                row.update(status='unknown', checked_at=None, error=reason)

    def renamed(self, old, new):
        with self.lock:
            self.state['known_servos'] = [r for r in self.state['known_servos'] if r['id'] != old]
            self.record_presence(new, 'online')
            self.save_registry()
            self.state['sid'] = new

    def poll_known(self, bus, selected):
        ids = [r['id'] for r in self.snapshot()['known_servos'] if r['id'] != selected]
        if not ids:
            return
        start = self.roster_cursor % len(ids)
        batch = (ids[start:] + ids[:start])[:8]
        self.roster_cursor = (start + len(batch)) % len(ids)
        for sid in batch:
            if self.snapshot()['busy'] or self.closed.is_set():
                break
            try:
                bus.identify(sid, timeout=.04)
                self.record_presence(sid, 'online')
            except Exception as exc:
                self.presence_error(sid, exc)

    def inventory(self):
        ports = [dict(device=p.device, label=p.description or p.device,
                      recommended=(p.device == self.bridge_port if self.transport == 'esp32'
                                   else p.vid == 0x1A86 and p.pid == 0x55D3))
                 for p in self.port_source()
                 if 'Bluetooth' not in p.device and 'debug-console' not in p.device
                 and (self.transport != 'esp32' or p.device == self.bridge_port)]
        s = self.snapshot()
        chosen = s['selected_port']
        if not chosen:
            chosen = next((p['device'] for p in ports if p['recommended']), '')
        adapter = any(p['device'] == chosen for p in ports)
        self.update(ports=ports, port=chosen, adapter=adapter)
        if not adapter:
            self.update(connected=False, servo=None)
            self.reset_presence('URT-2 未连接，无法检测')
        return chosen if adapter else ''

    def open_bus(self):
        port = self.inventory()
        if not port:
            if self.transport == 'esp32':
                raise BusError(f'X5 未找到 ESP32 COM 串口 {self.bridge_port}，请检查 USB 连接')
            raise BusError('未找到 URT-2，请连接 USB，或在连接设置中选择串口')
        try:
            return self.bus_factory(port, self.snapshot()['baud'])
        except Exception:
            self.reset_presence('串口无法打开，无法检测')
            raise

    def remember(self, data):
        self.update(servo=data, connected=True, last_seen=time.time())
        self.record_presence(data['id'], 'online')

    def probe(self):
        bus = self.open_bus()
        sid = self.snapshot()['sid']
        try:
            data = bus.snapshot(sid)
            self.remember(data)
            return data
        except Exception as exc:
            self.presence_error(sid, exc)
            raise
        finally:
            try:
                self.poll_known(bus, sid)
            finally:
                bus.close()

    def monitor(self):
        while not self.closed.is_set():
            if not self.snapshot()['busy'] and self.bus_lock.acquire(blocking=False):
                try:
                    if not self.snapshot()['busy']:
                        was_connected = self.snapshot()['connected']
                        try:
                            self.probe()
                            if not was_connected:
                                self.log(f"已连接舵机 ID {self.snapshot()['sid']}", 'success')
                        except Exception as exc:
                            self.update(connected=False, servo=None)
                            if was_connected:
                                self.log(f'连接中断：{exc}', 'error')
                finally:
                    self.bus_lock.release()
            self.closed.wait(1.2)

    def configure(self, data):
        sid = integer(data.get('sid'), 0, 253, '查询 ID')
        baud = integer(data.get('baud'), 1, 1000000, '波特率')
        if baud not in BAUDS:
            raise ValueError('不支持该波特率')
        if self.transport == 'esp32' and baud != 1000000:
            raise ValueError('ESP32 桥接模式的舵机总线固定为 1 Mbps；COM 使用 115200 bps')
        port = data.get('port', '')
        if self.transport == 'esp32' and port != self.bridge_port:
            raise ValueError('ESP32 桥接模式只能使用启动时指定的 COM 串口')
        if not isinstance(port, str) or (port and port not in [p.device for p in self.port_source()]):
            raise ValueError('串口不存在，请刷新连接')
        with self.lock:
            if self.state['busy']:
                raise BusError('有操作正在进行，请等待完成')
            connection_changed = port != self.state['selected_port'] or baud != self.state['baud']
            self.state.update(selected_port=port, baud=baud, servo_baud=baud, sid=sid,
                              host_baud=115200 if self.transport == 'esp32' else baud,
                              connected=False, servo=None)
            if connection_changed:
                self.state.update(found=[], scan_results=[], scan_status='idle', scan_finished=None)
                self.reset_presence('连接设置已变化，等待检测')
        self.start('read', {})

    def start(self, action, data):
        if action not in ('read', 'position', 'scan', 'move', 'id', 'release'):
            raise ValueError('未知操作')
        if action == 'move':
            integer(data.get('target'), 0, 4095, '目标位置')
        if action == 'id':
            integer(data.get('new_id'), 0, 253, '新 ID')
        with self.lock:
            if self.state['busy']:
                raise BusError('有操作正在进行，请等待完成')
            if action in ('move', 'id', 'release'):
                if not self.state['connected']:
                    raise BusError('请先检测并连接舵机')
                if data.get('sid') != self.state['sid']:
                    raise BusError('当前 ID 已变化，请刷新后重试')
            self.state.update(busy=True, action=action, progress=0)
            if action == 'scan':
                self.state.update(found=[], scan_results=[], scan_status='running', scan_finished=None)
            self.cancel.clear()
            self.worker = threading.Thread(target=self.run_job, args=(action, data), daemon=True)
            self.worker.start()

    def stop(self):
        s = self.snapshot()
        if s['busy'] and s['action'] in ('move', 'scan'):
            self.cancel.set()
        elif s['busy']:
            raise BusError('当前操作正在写入参数，请等待完成')
        else:
            self.start('release', dict(sid=s['sid']))

    def check_cancel(self):
        if self.cancel.is_set() or self.closed.is_set():
            raise Cancelled('操作已停止')

    def scan(self, bus, include_positions=False):
        found = []
        results = []
        self.update(found=[])
        if include_positions:
            self.update(scan_results=[], scan_status='running', scan_finished=None)
        for sid in range(254):
            self.check_cancel()
            try:
                bus.identify(sid, timeout=.025)
            except NoReply:
                self.record_presence(sid, 'offline', '没有有效响应')
                self.update(progress=round((sid + 1) / 254 * 100))
                continue
            except ServoFault as exc:
                self.presence_error(sid, exc)
                found.append(sid)
                if include_positions:
                    results.append(dict(id=sid, position=None, error=str(exc)))
            else:
                self.record_presence(sid, 'online')
                found.append(sid)
                if include_positions:
                    try:
                        result = bus.snapshot(sid)
                        result['error'] = ''
                    except BusError as exc:
                        self.presence_error(sid, exc)
                        result = dict(id=sid, position=None, error=str(exc))
                    results.append(result)
            if include_positions:
                self.update(scan_results=copy.deepcopy(results))
            self.update(progress=round((sid + 1) / 254 * 100), found=found.copy())
        if include_positions:
            self.update(scan_status='complete', scan_finished=time.time())
        return found

    def move(self, bus, sid, target):
        initial = bus.snapshot(sid)
        self.remember(initial)
        start = initial['position']
        if initial['torque'] not in (0, 1):
            raise BusError('舵机扭矩模式异常，未发送运动指令')
        touched = False
        try:
            self.check_cancel()
            touched = True
            bus.goal(sid, start)
            if int.from_bytes(bus.read(sid, 42, 2), 'little') != start:
                raise BusError('初始目标位置回读不一致')
            if initial['torque'] == 0:
                self.check_cancel()
                bus.write(sid, 40, [1])
                if bus.read(sid, 40, 1) != b'\x01':
                    raise BusError('未能启用舵机扭矩')
            steps = max(1, math.ceil(abs(target - start) / 8))
            previous = start
            for i in range(1, steps + 1):
                self.check_cancel()
                goal = round(start + (target - start) * i / steps)
                bus.goal(sid, goal)
                self.cancel.wait(.035)
                self.check_cancel()
                feedback = bus.snapshot(sid)
                actual = feedback['position']
                self.remember(feedback)
                if abs(actual - goal) > 64 or abs(actual - previous) > 96:
                    raise BusError('位置未能跟随目标，可能供电不足或机构受阻')
                if feedback['torque'] != 1:
                    raise BusError('运动中扭矩关闭，可能发生了掉电或重启')
                previous = actual
                self.update(progress=round(i / steps * 100))
            self.cancel.wait(.3)
            self.check_cancel()
            samples = [bus.snapshot(sid) for _ in range(3)]
            self.remember(samples[-1])
            if any(abs(s['position'] - target) > 12 or s['torque'] != 1 for s in samples):
                raise BusError('最终位置未通过连续三次验证')
            self.log(f"回中完成：目标 {target}，实际 {samples[-1]['position']}；保持当前位置", 'success')
        except Exception as exc:
            if touched:
                try:
                    bus.release(sid)
                except Exception as read_exc:
                    self.presence_error(sid, read_exc)
                    self.update(connected=False, servo=None)
                    raise BusError(f'{exc}；无法确认停止，请立即断开舵机电源') from exc
            raise

    def run_job(self, action, data):
        with self.bus_lock:
            bus = None
            try:
                bus = self.open_bus()
                sid = self.snapshot()['sid']
                if action in ('read', 'position'):
                    feedback = bus.snapshot(sid)
                    self.remember(feedback)
                    if action == 'position':
                        self.log(f"位置读取成功：ID {sid}，当前位置 {feedback['position']}", 'success')
                    else:
                        self.log(f'读取成功：舵机 ID {sid}', 'success')
                elif action == 'scan':
                    self.log('正在扫描 ID 0–253，并读取每只舵机的位置…')
                    found = self.scan(bus, include_positions=True)
                    if found:
                        selected = sid if sid in found else found[0]
                        self.update(sid=selected)
                        try:
                            self.remember(bus.snapshot(selected))
                        except BusError as exc:
                            self.presence_error(selected, exc)
                            self.update(connected=False, servo=None)
                        rows = self.snapshot()['scan_results']
                        summary = '；'.join(f"ID {r['id']}：位置 {r['position'] if r['position'] is not None else '读取失败'}" for r in rows[:10])
                        self.log(f'扫描完成，发现 {len(found)} 个 ID。' + summary, 'success')
                    else:
                        self.update(connected=False, servo=None)
                        self.log('未发现舵机，请检查 TTL 接线、供电和波特率', 'error')
                elif action == 'move':
                    self.log(f"正在缓慢回中到 {data['target']}…")
                    self.move(bus, sid, data['target'])
                elif action == 'release':
                    bus.release(sid)
                    self.remember(bus.snapshot(sid))
                    self.log('已释放舵机扭矩，输出轴不再保持位置', 'success')
                elif action == 'id':
                    if data['new_id'] == sid:
                        raise ValueError('新 ID 与当前 ID 相同')
                    self.log(f"正在检查总线，将选中的 ID {sid} 改为 {data['new_id']}…")
                    found = self.scan(bus, include_positions=True)
                    if sid not in found:
                        raise BusError(f'选中的 ID {sid} 没有响应，未修改 ID')
                    if data['new_id'] in found:
                        raise BusError(f"ID {data['new_id']} 已有舵机响应，未修改")
                    self.check_cancel()
                    result = bus.change_id(sid, data['new_id'], lambda new: self.renamed(sid, new))
                    self.remember(result)
                    rows = [r for r in self.snapshot()['scan_results'] if r['id'] != sid]
                    rows.append(dict(result, error=''))
                    self.update(found=sorted(result['id'] if i == sid else i for i in found),
                                scan_results=sorted(rows, key=lambda r: r['id']))
                    self.log(f"ID 已从 {sid} 改为 {result['id']}，当前读回及参数锁定通过，扭矩已释放。断电保存尚待验证：请将同一只舵机断电重连，再点击一键扫描。", 'info')
            except Cancelled:
                if action == 'scan':
                    self.update(scan_status='stopped')
                self.log('操作已停止' + ('，已释放舵机扭矩' if action == 'move' else ''), 'info')
            except Exception as exc:
                if action == 'scan':
                    self.update(scan_status='error')
                self.log(str(exc), 'error')
                try:
                    if bus:
                        self.remember(bus.snapshot(self.snapshot()['sid']))
                except Exception as read_exc:
                    self.presence_error(self.snapshot()['sid'], read_exc)
                    self.update(connected=False, servo=None)
            finally:
                if bus:
                    bus.close()
                self.update(busy=False, action='')

    def close(self):
        self.closed.set()
        self.cancel.set()
        if self.worker:
            self.worker.join(timeout=3)


def create_server(controller, port):
    token = secrets.token_urlsafe(32)
    allowed_hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}
    origins = {'http://' + h for h in allowed_hosts}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, status, body, content_type='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            self.wfile.write(body)

        def authorized(self, mutation=False):
            if self.headers.get('Host') not in allowed_hosts:
                self.respond(403, dict(error='仅允许本机访问'))
                return False
            if self.headers.get('Origin') and self.headers['Origin'] not in origins:
                self.respond(403, dict(error='拒绝跨站请求'))
                return False
            if mutation and not secrets.compare_digest(self.headers.get('X-Workbench-Token', ''), token):
                self.respond(403, dict(error='页面令牌无效，请刷新页面'))
                return False
            return True

        def do_GET(self):
            if not self.authorized():
                return
            if self.path == '/':
                html = (HERE / 'index.html').read_text().replace('__TOKEN__', token)
                self.respond(200, html.encode(), 'text/html; charset=utf-8')
            elif self.path == '/api/status':
                self.respond(200, controller.snapshot())
            elif self.path == '/favicon.ico':
                self.respond(204, b'')
            else:
                self.respond(404, dict(error='未找到页面'))

        def do_POST(self):
            if not self.authorized(True):
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    raise ValueError('请求大小不合法')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('需要 JSON 请求')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('需要 JSON 对象')
                if self.path == '/api/config':
                    controller.configure(data)
                elif self.path == '/api/stop':
                    controller.stop()
                elif self.path in ('/api/read', '/api/position', '/api/scan', '/api/move', '/api/id', '/api/release'):
                    controller.start(self.path.rsplit('/', 1)[1], data)
                else:
                    self.respond(404, dict(error='未知操作'))
                    return
                self.respond(202, dict(ok=True))
            except (ValueError, BusError) as exc:
                self.respond(400, dict(error=str(exc)))
            except Exception as exc:
                self.respond(500, dict(error=str(exc)))

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description='HD1910 本地舵机工作台')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--esp32-bridge', action='store_true',
                        help='Use the X5-connected ESP32 workbench bridge firmware')
    parser.add_argument('--bridge-port', default='/dev/ttyUSB0')
    args = parser.parse_args()
    sessions = None
    if args.esp32_bridge:
        from esp_bridge import BridgeBus, EspSessionManager
        sessions = EspSessionManager()
        controller = Controller(bus_factory=lambda port, baud: BridgeBus(sessions.get(port), baud),
                                registry_path=HERE / 'known_servos.json', transport='esp32',
                                bridge_port=args.bridge_port)
    else:
        controller = Controller(registry_path=HERE / 'known_servos.json')
    try:
        server = create_server(controller, args.port)
        print(f'舵机工作台：http://127.0.0.1:{args.port}', flush=True)
        print('自动检测仅发送读取指令；运动和修改 ID 需点击页面按钮。Ctrl+C 退出。', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        if sessions:
            sessions.close()


if __name__ == '__main__':
    main()
