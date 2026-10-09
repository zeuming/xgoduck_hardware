"""Separate HTTP frontend from the sole hardware/controller process.

HTTP GETs read an atomically replaced RAM snapshot, never controller RPC. Three
inherited socket pairs carry commands, heartbeats and STOP independently. No
motion command is retried; expired/busy requests are rejected before dispatch.
The controller publishes at a fixed cadence regardless of browser traffic.
"""
from __future__ import annotations

import copy
import base64
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time
import types
import re
from collections import OrderedDict

if __name__ == '__main__':
    # Isolated Python deliberately omits the script directory from sys.path.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
import native_workbench as web

STATE_PERIOD = .2
MAX_PACKET = web.MAX_RESPONSE
MAX_COMMAND = 8192
LANES = ('command', 'heartbeat', 'stop')


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).encode()


def receive(sock, limit):
    def exact(size):
        result = bytearray()
        while len(result) < size:
            part = sock.recv(size - len(result))
            if not part:
                raise EOFError('Controller IPC closed')
            result.extend(part)
        return bytes(result)
    size = struct.unpack('!I', exact(4))[0]
    if not 0 < size <= limit:
        raise ValueError('Controller IPC packet exceeds bound')
    return web.strict_json(exact(size))


def send(sock, value, limit=MAX_PACKET):
    raw = encode(value)
    if not 0 < len(raw) <= limit:
        raise ValueError('Controller IPC packet exceeds bound')
    sock.sendall(struct.pack('!I', len(raw)) + raw)


class SnapshotFile:
    """One immutable RAM record; readers cannot backpressure the publisher."""
    def __init__(self, path):
        self.path = Path(path)

    def publish(self, value):
        raw = encode(value)
        if len(raw) > MAX_PACKET:
            raise ValueError('Controller display snapshot exceeds bound')
        temporary = self.path.with_name('.state-' + secrets.token_hex(12))
        try:
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                         os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, 'wb') as stream:
                stream.write(raw)
            os.replace(temporary, self.path)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    def read(self):
        fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid()
                    or info.st_mode & 0o077 or info.st_nlink not in (0, 1)
                    or not 0 < info.st_size <= MAX_PACKET):
                raise ValueError('Controller display snapshot is not private/bounded')
            raw = stream.read(MAX_PACKET + 1)
        if len(raw) > MAX_PACKET:
            raise ValueError('Controller display snapshot exceeds bound')
        value = web.strict_json(raw)
        if (type(value) is not dict or value.get('protocol') != 1
                or type(value.get('published_at')) not in (int, float)
                or not math.isfinite(value['published_at'])
                or type(value.get('snapshot')) is not dict):
            raise ValueError('Controller display snapshot schema changed')
        return value


def age_snapshot(snapshot, elapsed_ms):
    """Add publication delay to live ages; never age historical evidence."""
    value = copy.deepcopy(snapshot)
    def age(record):
        if type(record) is not dict:
            return
        for key in ('age_ms', 'imu_age_ms', 'received_age_ms', 'feedback_age_ms', 'sample_age_ms'):
            if type(record.get(key)) in (int, float):
                record[key] += elapsed_ms
        if type(record.get('heartbeat_remaining_ms')) in (int, float):
            record['heartbeat_remaining_ms'] = max(0, record['heartbeat_remaining_ms'] - elapsed_ms)
        for row in record.get('motor_voltages', []):
            if type(row.get('age_ms')) in (int, float):
                row['age_ms'] += elapsed_ms
                # Existing freshness may expire, never turn true on a GET.
                row['fresh'] = bool(row.get('fresh') and row['age_ms'] <= 1000)
                if not row['fresh']:
                    row['condition'] = 'unknown'
    motion = value.get('motion') or {}
    for record in (motion, motion.get('hardware'), motion.get('last_hardware'),
                   value.get('hardware'), value.get('session'), value.get('feedback')):
        age(record)
    hardware = motion.get('hardware')
    if hardware and max(hardware.get('age_ms') or 0, hardware.get('imu_age_ms') or 0) >= 500:
        motion['hardware'] = None
        value['hardware'] = None
        value['body_imu'] = None
    session = value.get('session') or {}
    if (session.get('received_age_ms') or 0) >= 500:
        value['feedback'] = None
        if motion.get('hardware') is None:
            value['body_imu'] = None
    value['process_isolation'] = dict(protocol=1, controller_pid=os.getppid(),
        web_pid=os.getpid(), snapshot_age_ms=round(elapsed_ms, 2),
        state_period_ms=STATE_PERIOD * 1000,
        status_source='published_snapshot', independent_command_lanes=list(LANES))
    return value


class CommandClient:
    def __init__(self, sockets):
        self.sockets = sockets
        self.locks = {name: threading.Lock() for name in LANES}

    def call(self, path, body, principal):
        action = body.get('action') if type(body) is dict else None
        lane = action if path == '/api/motion' and action in ('heartbeat', 'stop') else 'command'
        if not self.locks[lane].acquire(blocking=False):
            raise ValueError('当前操作尚未完成；请求未排队、未重发')
        sock = self.sockets[lane]
        timeout = .15 if lane == 'heartbeat' else 10.
        try:
            request_id = secrets.token_hex(16)
            sock.settimeout(timeout)
            try:
                send(sock, dict(id=request_id, path=path, body=body, principal=principal,
                                deadline=time.monotonic() + timeout), MAX_COMMAND)
                reply = receive(sock, MAX_PACKET)
                if type(reply) is not dict or reply.get('id') != request_id or type(reply.get('ok')) is not bool:
                    raise ValueError('Controller IPC response identity changed')
            except ValueError:
                sock.close()
                raise
            if reply.get('ok') is not True:
                raise ValueError(reply.get('error') or '控制进程拒绝操作')
            return reply['result']
        except (OSError, EOFError) as error:
            # A late reply must never confirm a later operation. This lane is
            # permanently closed until the service is explicitly restarted.
            sock.close()
            raise ValueError('控制进程通信中断，执行结果未确认；操作不会重发') from error
        finally:
            self.locks[lane].release()


class ControllerBridge:
    def __init__(self, workbench, store, sockets):
        self.workbench, self.store, self.sockets = workbench, store, sockets
        self.stop = threading.Event()
        self.failure = None
        self.threads = []

    def publish(self):
        began = time.monotonic()
        live = self.workbench.snapshot(live=True)
        maintenance = live.get('maintenance') or {}
        session = live.get('session') or {}
        idle = (not live['motion'].get('owns_connection')
                and not maintenance.get('owns_connection')
                and session.get('status') in ('closed', 'fault'))
        # Large historical diagnostics are generated only after hardware
        # connections close, never in response to /api/status while moving.
        full = self.workbench.snapshot() if idle else None
        self.store.publish(dict(protocol=1, published_at=began,
            metadata=dict(token=self.workbench.token, generation=self.workbench.generation,
                          authentication_required=self.workbench.authentication_required,
                          full_workbench=self.workbench.calibration_service is not None),
            snapshot=live, full_snapshot=full))

    def dispatch(self, lane, request):
        if (type(request) is not dict or set(request) != {'id', 'path', 'body', 'principal', 'deadline'}
                or type(request['id']) is not str or len(request['id']) != 32
                or type(request['principal']) is not str or not 1 <= len(request['principal']) <= 128
                or type(request['deadline']) not in (int, float)
                or not math.isfinite(request['deadline'])
                or not 0 < request['deadline'] - time.monotonic() <= 10.1):
            raise ValueError('IPC request invalid or expired; not executed')
        path, body = request['path'], request['body']
        action = body.get('action') if type(body) is dict else None
        expected_lane = action if path == '/api/motion' and action in ('heartbeat', 'stop') else 'command'
        if lane != expected_lane:
            raise ValueError('IPC command lane differs; not executed')
        if path == '/api/motion':
            return self.workbench.motion_action(body, principal=request['principal'], live=True)
        if path not in ('/api/imu-wizard', '/api/joint-calibration', '/api/joint-zero',
                        '/api/end-feedback', '/api/recover-feedback', '/api/recheck-feedback',
                        '/api/check-connection', '/api/servo-maintenance'):
            raise ValueError('Unknown controller IPC operation')
        return self.workbench.calibration_action(path, body, principal=request['principal'])

    def _commands(self, lane):
        sock = self.sockets[lane]
        try:
            while not self.stop.is_set():
                request = receive(sock, MAX_COMMAND)
                try:
                    result = self.dispatch(lane, request)
                    response = dict(id=request.get('id'), ok=True, result=result)
                except Exception as error:
                    response = dict(id=request.get('id') if type(request) is dict else None,
                                    ok=False, error=str(error)[:2048])
                sock.settimeout(.2)
                send(sock, response)
                sock.settimeout(None)
        except (OSError, EOFError, ValueError) as error:
            if not self.stop.is_set():
                self.failure = str(error)
        finally:
            sock.close()

    def _publish(self):
        while not self.stop.wait(STATE_PERIOD):
            try:
                self.publish()
            except Exception as error:
                self.failure = 'Controller state publication failed: ' + str(error)
                return

    def start(self, *, commands=True):
        self.publish()
        self.threads.append(threading.Thread(target=self._publish, daemon=True))
        self.threads[-1].start()
        if commands:
            self.start_commands()

    def start_commands(self):
        for name in LANES:
            thread = threading.Thread(target=self._commands, args=(name,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def close(self):
        self.stop.set()
        for sock in self.sockets.values():
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        for thread in self.threads:
            thread.join(timeout=.5)


class FrontendWorkbench(web.NativeWorkbench):
    """All cookies, HTTP, rendering, JSON and GET projections live here."""
    def __init__(self, store, client, *, web_root, password):
        self.store, self.client = store, client
        packet = store.read()
        metadata = packet['metadata']
        super().__init__(types.SimpleNamespace(), web_root=web_root, password=password,
                         authentication_required=metadata['authentication_required'])
        self.token, self.generation = metadata['token'], metadata['generation']
        self.calibration_service = True if metadata['full_workbench'] else None
        # The handler's calibration download also reads the publication only.
        self.runtime = CalibrationView(self)
        self.page = web.native_page((self.web_root / 'index.html').read_text(),
            authentication_required=self.authentication_required, full_workbench=metadata['full_workbench'])
        self.logs = LogReader(self.web_root.parent / 'native-evidence')
        self.page = log_toolbar(self.page)

    def snapshot(self, *, live=False):
        packet = self.store.read()
        snapshot = packet['snapshot'] if live else packet.get('full_snapshot') or packet['snapshot']
        result = age_snapshot(snapshot, max(0., time.monotonic() - packet['published_at']) * 1000)
        if not live and packet.get('full_snapshot') is None:
            result['diagnostics_deferred_until_disconnected'] = True
        return result

    def motion_action(self, body, *, principal=None, live=False):
        result = self.client.call('/api/motion', body, principal)
        # Preserve the browser owner during bounded cookie eviction. The
        # authoritative ownership/generation checks remain in the controller.
        with self.lock:
            if body.get('action') == 'start' and result.get('control_owner'):
                self.control_owner = dict(result['control_owner'], principal=principal)
            elif body.get('action') in ('stop', 'disconnect', 'connect', 'capture-head-clearance'):
                self.control_owner = None
        return result

    def calibration_action(self, path, body, *, principal=None):
        return self.client.call(path, body, principal)

    def close(self):
        with self.lock:
            self.closed = True
            self.cookies.clear()


class CalibrationView:
    def __init__(self, frontend):
        self.frontend = frontend

    @property
    def calibration(self):
        return self.frontend.snapshot(live=True)['calibration']


def allowed_web_command(executable, command, root):
    """Exact immutable frontend entry point and inherited descriptors only."""
    return (executable == sys.executable and isinstance(command, (list, tuple))
        and len(command) == 9 and list(command[:4]) == [sys.executable, '-I', '-B',
            str(Path(root) / 'service/native_process.py')]
        and command[4] == '--frontend'
        and all(type(item) is str and item.isdecimal() and 3 <= int(item) <= 65535
                for item in command[5:]) and len(set(command[5:])) == 4)


def serve(runtime, *, web_root, secret_file=None, host=web.HOST, port=web.PORT,
          snapshot_template=None, auto_connect=True, ready_callback=None,
          authentication_required=True, calibration_service=None):
    """Supervisor/controller stays foreground; only the child accepts HTTP."""
    if host != web.HOST or type(port) is not int or port != web.PORT:
        raise ValueError('仅允许部署时审查的 X5 Wi-Fi 地址与端口')
    if type(authentication_required) is not bool:
        raise ValueError('访问口令模式无效')
    root = Path(web_root).resolve().parent
    if authentication_required and (secret_file is None or
            not Path(secret_file).resolve().is_relative_to('/dev/shm')):
        raise ValueError('原生工作台访问口令必须保存在 RAM 中')
    password = web.private_secret(secret_file) if authentication_required else None
    authority = web.NativeWorkbench(runtime, web_root=web_root, password=password,
        authentication_required=authentication_required, snapshot_template=snapshot_template,
        calibration_service=calibration_service)
    store = SnapshotFile(root / 'web-state.json')
    parent, child_sockets = {}, {}
    listener = socket.socket()
    child = bridge = None
    saved = {}
    try:
        if threading.current_thread() is threading.main_thread():
            def shutdown(_signum, _frame):
                raise SystemExit(0)
            for signum in (signal.SIGTERM, signal.SIGINT):
                saved[signum] = signal.signal(signum, shutdown)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
        listener.listen(16)
        if auto_connect:
            authority.initial_connect()
        for name in LANES:
            parent[name], child_sockets[name] = socket.socketpair()
        bridge = ControllerBridge(authority, store, parent)
        bridge.start(commands=False)
        fds = [listener.fileno()] + [child_sockets[name].fileno() for name in LANES]
        command = [sys.executable, '-I', '-B', str(root / 'service/native_process.py'),
                   '--frontend'] + [str(fd) for fd in fds]
        # Bootstrap already provides a closed-input descriptor. Opening a new
        # /dev/null here would violate the controller's reviewed write scope.
        child = subprocess.Popen(command, pass_fds=tuple(fds))
        for sock in child_sockets.values():
            sock.close()
        parent['command'].settimeout(10.)
        info = receive(parent['command'], MAX_COMMAND)
        parent['command'].settimeout(None)
        if (type(info) is not dict or info.get('kind') != 'web-ready'
                or info.get('pid') != child.pid or info.get('port') != port
                or info.get('address') != host or info.get('auth_required') is not authentication_required
                or info.get('listener_inode') != os.fstat(listener.fileno()).st_ino):
            raise ValueError('Separate HTTP child listener proof differs')
        bridge.start_commands()
        if ready_callback:
            ready_callback(dict(pid=os.getpid(), address=host, port=port,
                listener_fd=listener.fileno(), listener_inode=os.fstat(listener.fileno()).st_ino,
                auth_required=authentication_required, web_pid=child.pid, motion=runtime.snapshot()))
        while child.poll() is None:
            if bridge.failure:
                raise RuntimeError(bridge.failure)
            time.sleep(.05)
        raise RuntimeError('HTTP child exited; hardware controller is closing')
    finally:
        # Stop accepting new commands before closing the sole UART owner.
        if bridge:
            bridge.close()
        try:
            authority.close()
            if bridge:
                bridge.publish()
        finally:
            if child is not None and child.poll() is None:
                child.terminate()
                child.wait(timeout=5)
            listener.close()
            for sock in child_sockets.values():
                sock.close()
            for signum, previous in saved.items():
                signal.signal(signum, previous)


def frontend_main(arguments):
    if len(arguments) != 5 or arguments[0] != '--frontend':
        raise ValueError('Invalid inherited frontend descriptors')
    root = Path(__file__).resolve().parents[1]
    listener = socket.socket(fileno=int(arguments[1]))
    sockets = {name: socket.socket(fileno=int(fd)) for name, fd in zip(LANES, arguments[2:])}
    # This process has no runtime, model or serial imports and may never open
    # a device. All hardware operations require authenticated controller IPC.
    def frontend_audit(event, args):
        if event == 'open' and args and isinstance(args[0], (str, bytes)):
            path = os.path.realpath(os.fsdecode(args[0]))
            if path.startswith('/dev/') and not path.startswith(str(root) + '/'):
                raise PermissionError('HTTP frontend cannot open devices')
            try:
                node = os.stat(path)
                if stat.S_ISCHR(node.st_mode) or stat.S_ISBLK(node.st_mode):
                    raise PermissionError('HTTP frontend cannot open devices')
            except FileNotFoundError:
                pass
            mode = args[1] if len(args) > 1 else None
            flags = args[2] if len(args) > 2 else 0
            if ((isinstance(mode, str) and any(c in mode for c in 'wax+'))
                    or isinstance(flags, int) and flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC)):
                raise PermissionError('HTTP frontend cannot write files')
        if event in ('subprocess.Popen', 'os.system', 'os.exec', 'os.posix_spawn', 'os.fork'):
            raise PermissionError('HTTP frontend cannot launch children')
        if event in ('os.mkdir', 'os.remove', 'os.rename', 'os.rmdir', 'os.chmod',
                     'os.chown', 'os.truncate', 'os.utime', 'os.link', 'os.symlink'):
            raise PermissionError('HTTP frontend cannot mutate files')
    # Ordinary UI/JSON CPU work yields to the controller under contention.
    os.nice(5)
    sys.addaudithook(frontend_audit)
    store = SnapshotFile(root / 'web-state.json')
    metadata = store.read()['metadata']
    password = web.private_secret(root / 'service/access-secret.json') if metadata['authentication_required'] else None
    frontend = FrontendWorkbench(store, CommandClient(sockets), web_root=root / 'web', password=password)
    server = web.BoundedHTTPServer(listener.getsockname(), frontend_handler_for(frontend), listener=listener)
    send(sockets['command'], dict(kind='web-ready', pid=os.getpid(),
         address=server.server_address[0], port=server.server_address[1],
         listener_inode=os.fstat(listener.fileno()).st_ino,
         auth_required=frontend.authentication_required), MAX_COMMAND)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        frontend.close()
        server.server_close()
        for sock in sockets.values():
            sock.close()


class LogReader:
    """Read existing evidence in the HTTP process; no control RPC or writes.

    Directory scans are cached and bounded. Each page reads at most eight
    bounded files, and only returns a whitelist of summary fields.
    """
    PAGE_SIZE = 8
    MAX_FILES = 4096
    MAX_FILE_BYTES = 256 * 1024
    INDEX_SECONDS = 3.
    NAME = re.compile(r'(?:operation-[0-9a-f]{16}|connection-[1-9][0-9]{0,9}-(?:ready|closed)|calibration-reload)\.json\Z')

    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.Lock()
        self.index_at = -math.inf
        self.index = []
        self.truncated = False
        self.cache = OrderedDict()

    @staticmethod
    def cursor(key):
        return base64.urlsafe_b64encode(encode(list(key))).decode().rstrip('=')

    @classmethod
    def decode_cursor(cls, value):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,240}', value):
            raise ValueError('日志分页标记无效')
        try:
            key = web.strict_json(base64.b64decode(value + '=' * (-len(value) % 4), altchars=b'-_', validate=True))
        except (ValueError, UnicodeError) as error:
            raise ValueError('日志分页标记无效') from error
        if (type(key) is not list or len(key) != 2 or type(key[0]) is not int or key[0] < 0
                or type(key[1]) is not str or cls.NAME.fullmatch(key[1]) is None):
            raise ValueError('日志分页标记无效')
        return tuple(key)

    def _index(self):
        now = time.monotonic()
        if now - self.index_at < self.INDEX_SECONDS:
            return
        rows, truncated = [], False
        try:
            # An evidence directory is a fixed private RAM directory, never
            # a browser-selected path or a link to a different filesystem.
            info = self.directory.lstat()
            if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid()
                    or info.st_mode & 0o077 or self.directory.resolve() != self.directory):
                raise ValueError('日志目录无效')
            with os.scandir(self.directory) as entries:
                for count, entry in enumerate(entries):
                    if count >= self.MAX_FILES or time.monotonic() - now > .05:
                        truncated = True
                        break
                    if self.NAME.fullmatch(entry.name) is None:
                        continue
                    node = entry.stat(follow_symlinks=False)
                    if (stat.S_ISREG(node.st_mode) and node.st_uid == os.geteuid()
                            and not node.st_mode & 0o077 and node.st_nlink == 1):
                        rows.append((node.st_mtime_ns, entry.name, node.st_size, node.st_ino))
        except FileNotFoundError:
            pass
        self.index = sorted(rows, reverse=True)
        self.index_at, self.truncated = now, truncated

    @staticmethod
    def _text(value):
        if type(value) is not str:
            return ''
        return re.sub(r'[\x00-\x1f\x7f]', ' ', value)[:300]

    def _read(self, row):
        stamp, name, size, inode = row
        key = tuple(row)
        if key in self.cache:
            self.cache.move_to_end(key)
            return dict(self.cache[key])
        value = dict(id=self.cursor((stamp, name)),
                     time=datetime.fromtimestamp(stamp / 1e9, timezone.utc).isoformat(),
                     event='运行记录', state='', mode='', message='', level='info')
        if size > self.MAX_FILE_BYTES:
            value.update(level='warning', message='记录较大，已跳过详情读取。')
        else:
            fd = os.open(self.directory / name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(fd, 'rb') as stream:
                node = os.fstat(stream.fileno())
                if (not stat.S_ISREG(node.st_mode) or node.st_uid != os.geteuid()
                        or node.st_mode & 0o077 or node.st_nlink != 1
                        or (node.st_ino, node.st_mtime_ns, node.st_size) != (inode, stamp, size)):
                    raise ValueError('日志记录已变化，请刷新')
                record = web.strict_json(stream.read(self.MAX_FILE_BYTES + 1))
            if type(record) is not dict:
                raise ValueError('日志记录格式无效')
            after = record.get('after') if type(record.get('after')) is dict else {}
            action = record.get('action')
            events = {'start': '启动动作', 'stop': '停止动作', 'connect': '连接运控',
                      'disconnect': '断开运控', 'capture-head-clearance': '记录头颈姿态'}
            if name.startswith('connection-'):
                value['event'] = '连接就绪' if name.endswith('-ready.json') else '连接关闭'
            elif name == 'calibration-reload.json':
                value['event'] = '载入校准'
            else:
                value['event'] = events.get(action, '运控操作')
            value['state'] = self._text(record.get('state'))
            value['mode'] = self._text(record.get('mode') or after.get('mode'))
            value['message'] = self._text(record.get('error') or record.get('reason'))
            if record.get('state') == 'failed' or record.get('error'):
                value['level'] = 'error'
        self.cache[key] = value
        while len(self.cache) > 128:
            self.cache.popitem(last=False)
        return dict(value)

    def page(self, cursor=None):
        before = self.decode_cursor(cursor) if cursor else None
        if not self.lock.acquire(blocking=False):
            raise ValueError('日志正在读取，请稍后刷新')
        try:
            self._index()
            rows = [row for row in self.index if before is None or row[:2] < before]
            items = []
            for row in rows[:self.PAGE_SIZE]:
                try:
                    items.append(self._read(row))
                except (OSError, ValueError, UnicodeError):
                    items.append(dict(id=self.cursor(row[:2]), time=None, event='运行记录',
                        state='', mode='', level='warning', message='记录已变化或不可读，请刷新。'))
            return dict(items=items, page_size=self.PAGE_SIZE,
                next_cursor=self.cursor(rows[self.PAGE_SIZE - 1][:2]) if len(rows) > self.PAGE_SIZE else None,
                index_truncated=self.truncated, source='existing_controller_evidence')
        finally:
            self.lock.release()


def frontend_handler_for(frontend):
    class Handler(web.handler_for(frontend)):
        def do_GET(self):
            if not self.path.startswith('/api/logs/'):
                return super().do_GET()
            try:
                self.framing()
                if not frontend.authenticated(self.headers.get('Cookie')):
                    return self.reply(401, dict(error=web.access_message(frontend)))
                if self.path == '/api/logs/latest':
                    cursor = None
                elif self.path.startswith('/api/logs/before/'):
                    cursor = self.path.removeprefix('/api/logs/before/')
                    if not cursor:
                        raise ValueError('日志分页标记无效')
                else:
                    return self.reply(404, dict(error='找不到日志页面'))
                return self.reply(200, frontend.logs.page(cursor))
            except (OSError, ValueError, UnicodeError) as error:
                return self.reply(409, dict(error=str(error)))
    return Handler


LOG_TOOLBAR = r'''
<style>
.workspace main[data-runtime-logs="true"]>:not(#runtimeLogPanel):not(.hero){display:none!important}
.workspace main[data-runtime-logs="true"]>.hero .hero-actions,.workspace main[data-runtime-logs="true"] #x5ConnectionTarget{display:none!important}
.runtime-log-panel{width:100%;min-height:420px;display:flex;flex-direction:column}
.runtime-log-header{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:16px;padding:20px 22px;border-bottom:1px solid var(--line)}
.runtime-log-header h2{font-size:16px;margin:0}.runtime-log-note{margin:5px 0 0;font-size:12px;color:var(--muted)}
.runtime-log-actions{display:flex;align-items:center;flex-wrap:wrap;gap:8px}.runtime-log-actions .small{min-width:44px;text-align:center}
.runtime-log-columns,.runtime-log-list li{display:grid;grid-template-columns:140px 140px minmax(0,1fr);gap:20px}
.runtime-log-columns{padding:12px 22px;background:#f6f8f2;color:var(--muted);font-size:11px;border-bottom:1px solid var(--line)}
.runtime-log-list{list-style:none;margin:0;padding:0 22px;flex:1}.runtime-log-list li{padding:20px 0;border-bottom:1px solid var(--line);font-size:13px;overflow-wrap:anywhere;align-items:start}.runtime-log-list li:last-child{border-bottom:0}
.runtime-log-list strong{font-weight:600}.runtime-log-list time{font:12px/1.8 var(--mono,monospace);color:var(--muted)}
.runtime-log-list p{margin:0;white-space:pre-wrap;line-height:1.8}.runtime-log-list [data-level="error"] strong{color:var(--red)}
.runtime-log-empty{padding:70px 22px;text-align:center;flex:1;color:var(--muted)}.runtime-log-empty strong{display:block;color:var(--ink);font-size:16px;margin-bottom:8px}.runtime-log-empty p{font-size:13px}
.runtime-log-status{font-size:12px;color:var(--muted);padding:14px 22px;margin:0;border-top:1px solid var(--line)}
@media(max-width:850px){.runtime-log-columns,.runtime-log-list li{grid-template-columns:125px 110px minmax(0,1fr);gap:12px}}
@media(max-width:680px){.runtime-log-header{padding:18px 16px}.runtime-log-actions{width:100%}.runtime-log-columns{display:none}.runtime-log-list{padding:0 16px}.runtime-log-list li{grid-template-columns:minmax(0,1fr) auto;gap:8px}.runtime-log-list time{grid-column:2;grid-row:1}.runtime-log-list strong{grid-column:1;grid-row:1}.runtime-log-list p{grid-column:1/-1}.runtime-log-panel{min-height:380px}.runtime-log-status{padding:14px 16px}}
</style>
<section id="runtimeLogPanel" class="card runtime-log-panel" aria-labelledby="runtimeLogTitle" hidden>
<div class="runtime-log-header"><div><h2 id="runtimeLogTitle">最近记录</h2><p class="runtime-log-note">每页 8 条 · 最新页每 5 秒刷新 · 北京时间</p></div><div class="runtime-log-actions"><button class="button" id="runtimeLogLatest" type="button">刷新最新</button><button class="button" id="runtimeLogNewer" type="button" disabled>← 较新</button><span id="runtimeLogPage" class="small">第1页</span><button class="button" id="runtimeLogOlder" type="button" disabled>较早 →</button></div></div>
<div class="runtime-log-columns" aria-hidden="true"><span>时间</span><span>事件</span><span>内容</span></div>
<ol id="runtimeLogList" class="runtime-log-list" aria-live="polite"></ol>
<div id="runtimeLogEmpty" class="runtime-log-empty" hidden><strong>暂无运行日志</strong><p>新运行记录产生后，会在这里自动显示。</p></div>
<p id="runtimeLogStatus" class="runtime-log-status" role="status">选择运行日志后读取。</p>
</section>
'''

LOG_SCRIPT = r'''
(()=>{
  const $=id=>document.getElementById(id),main=document.querySelector('.workspace main');
  const button=$('runtimeLogToggle'),panel=$('runtimeLogPanel'),status=$('runtimeLogStatus');
  if(!main||!button||typeof workspaceRoutes==='undefined')return;
  workspaceRoutes.push({key:'runtime-logs',nav:'runtimeLogToggle',title:'运行日志',subtitle:'按时间查看运行记录，最新记录显示在最前面。',sections:['runtimeLogPanel'],instructionTitle:'',instructionCopy:''});
  let opened=false,busy=false,controller=null,epoch=0,timer=null,next=null,history=[null],page=0;
  const format=new Intl.DateTimeFormat('zh-CN',{timeZone:'Asia/Shanghai',month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
  function controls(){
    $('runtimeLogLatest').disabled=busy;$('runtimeLogNewer').disabled=busy||page===0;$('runtimeLogOlder').disabled=busy||!next;
    $('runtimeLogPage').textContent='第'+(page+1)+'页';panel.setAttribute('aria-busy',String(busy));
  }
  function cancel(){epoch++;controller?.abort();controller=null;busy=false;clearTimeout(timer);timer=null;controls();}
  function schedule(){clearTimeout(timer);timer=null;if(opened&&!document.hidden&&page===0)timer=setTimeout(()=>load(0),5000);}
  async function load(target){
    if(!opened||document.hidden||busy)return;
    clearTimeout(timer);const current=++epoch,active=new AbortController();controller=active;busy=true;controls();status.textContent='正在读取…';
    const timeout=setTimeout(()=>active.abort(),2000),cursor=target===0?null:history[target];
    try{
      const response=await fetch(cursor?'/api/logs/before/'+encodeURIComponent(cursor):'/api/logs/latest',{credentials:'same-origin',cache:'no-store',signal:active.signal});
      const result=await response.json();if(!response.ok)throw new Error(result.error||'日志读取失败');
      if(current!==epoch||!opened)return;
      if(!Array.isArray(result.items)||result.items.length>8)throw new Error('日志格式无效');
      const nodes=result.items.map(item=>{
        const li=document.createElement('li');li.dataset.level=item.level||'info';
        const time=document.createElement('time'),title=document.createElement('strong'),message=document.createElement('p');
        time.textContent=item.time&&Number.isFinite(Date.parse(item.time))?format.format(new Date(item.time)):'—';title.textContent=item.event||'运行记录';
        message.textContent=[item.mode,item.state==='complete'?'完成':item.state==='failed'?'失败':item.state,item.message].filter(Boolean).join(' · ');li.append(time,title,message);return li;
      });
      $('runtimeLogList').replaceChildren(...nodes);$('runtimeLogEmpty').hidden=nodes.length!==0;page=target;next=result.next_cursor||null;if(target===0)history=[null];
      status.textContent=result.index_truncated?'本次扫描已达上限，部分记录可能未显示。':nodes.length?'已显示'+nodes.length+'条'+(page>0?' · 历史页暂停刷新':''):'暂时没有运行记录。';
    }catch(error){if(current===epoch&&opened)status.textContent=error.name==='AbortError'?'日志读取超时，可点击刷新。':error.message;}
    finally{clearTimeout(timeout);if(current===epoch){busy=false;controller=null;controls();schedule();}}
  }
  // Use the established workspace navigation, including its motion leave guard.
  // Only the log view starts file GETs; leaving it aborts and stops its timer.
  const originalNavigation=renderWorkspaceNavigation;
  renderWorkspaceNavigation=function(){
    originalNavigation();const visible=workspaceView==='runtime-logs';main.dataset.runtimeLogs=String(visible);
    if(visible===opened)return;
    opened=visible;
    if(!opened){cancel();return;}
    history=[null];page=0;next=null;load(0);
  };
  button.onclick=()=>selectWorkspace('#runtime-logs');
  $('runtimeLogLatest').addEventListener('click',()=>load(0));
  $('runtimeLogOlder').addEventListener('click',()=>{if(next&&!busy){history[page+1]=next;load(page+1);}});
  $('runtimeLogNewer').addEventListener('click',()=>{if(page>0&&!busy)load(page-1);});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)cancel();else if(opened){if(page===0)load(0);else controls();}});
  window.addEventListener('pagehide',cancel);
  selectWorkspace(globalThis.location.hash==='#runtime-logs'?'#runtime-logs':'#'+workspaceView);
})();
'''


def log_toolbar(page):
    nav = '<nav aria-label="选择工作台功能">'.encode()
    if any(page.count(marker) != 1 for marker in (nav, b'</main>', b'</body>')):
        raise ValueError('Native workspace page structure changed')
    link = '<a class="nav-item" id="runtimeLogToggle" href="#runtime-logs" aria-controls="runtimeLogPanel"><span class="nav-icon" aria-hidden="true">\u25a4</span><span>运行日志</span></a>'
    page = page.replace(nav, nav + link.encode(), 1)
    page = page.replace(b'</main>', LOG_TOOLBAR.encode() + b'</main>', 1)
    return page.replace(b'</body>', ('<script>' + LOG_SCRIPT + '</script></body>').encode(), 1)


if __name__ == '__main__':
    frontend_main(sys.argv[1:])
