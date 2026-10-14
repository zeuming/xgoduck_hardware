"""CPU-only ONNX child, single in-flight request, bounded nonblocking control IPC.

The child inherits only its socket. It has no controller, transport, calibration,
ARM cookie or command authority. Cancellation discards results, never replays them.
"""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
if (__package__ is None and len(sys.argv) == 4 and sys.argv[1] == "--worker"):
    site = Path(__file__).resolve().parents[1] / "site"
    if site.is_dir(): sys.path.insert(0, str(site))
import numpy as np

MAX_PACKET = 8192
NAMES = ('walk', 'getup', 'pick', 'roulade')


class PolicyWorkerError(RuntimeError):
    pass


def _encode(value):
    data = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
    if len(data) >= MAX_PACKET:
        raise PolicyWorkerError('Inference IPC packet exceeds limit')
    return data


def _decode(data):
    if not data or len(data) >= MAX_PACKET:
        raise PolicyWorkerError('Inference IPC packet invalid')
    value = json.loads(data)
    if not isinstance(value, dict):
        raise PolicyWorkerError('Inference IPC object required')
    return value


class PolicyWorker:
    """Create/warm before UART acquisition. Never respawn while controlling."""
    def __init__(self, models_dir, hashes, *, startup_timeout=30.):
        self._lock = threading.Lock()
        self._pending = None
        self._abandoned = False
        self._sequence = 0
        self._closed = False
        self.submitted = self.completed = self.discarded = 0
        self.last_compute_ms = None
        self._socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        command = [sys.executable, '-I', '-B', str(Path(__file__).resolve()),
                   '--worker', str(child.fileno()), str(Path(models_dir).resolve())]
        try:
            self.process = subprocess.Popen(command, pass_fds=(child.fileno(),),
                close_fds=True)
            child.close()
            self._socket.send(_encode(dict(hashes=hashes)))
            deadline = time.monotonic() + startup_timeout
            while True:
                if self.process.poll() is not None:
                    raise PolicyWorkerError('Inference process failed during CPU-only startup')
                try:
                    ready = _decode(self._socket.recv(MAX_PACKET))
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise PolicyWorkerError('Inference process startup deadline exceeded')
                    time.sleep(.005)
            if ready.get('ready') is not True or ready.get('models') != sorted(hashes):
                raise PolicyWorkerError('Inference model readiness differs')
            self.models = frozenset(hashes)
        except BaseException:
            child.close()
            self.close()
            raise

    def _alive(self):
        if self._closed or self.process.poll() is not None:
            raise PolicyWorkerError('独立推理进程已退出；不会自动重启或恢复动作')

    def _receive(self):
        try:
            message = _decode(self._socket.recv(MAX_PACKET))
        except BlockingIOError:
            return None
        if message.get('id') != self._pending:
            raise PolicyWorkerError('Inference result request ID mismatch')
        self._pending = None
        if message.get('error'):
            raise PolicyWorkerError('Inference process rejected observation or model output')
        action = np.asarray(message.get('action'), dtype=np.float32)
        ms = message.get('compute_ms')
        if action.shape != (1, 14) or not np.isfinite(action).all() or type(ms) not in (int, float) or not math.isfinite(ms) or ms < 0:
            raise PolicyWorkerError('Inference result invalid')
        self.completed += 1
        self.last_compute_ms = float(ms)
        return action, float(ms)

    def submit(self, selected, observation):
        # A preflight caller must never make the feedback loop wait on its lock.
        if not self._lock.acquire(blocking=False):
            return None
        try:
            self._alive()
            if self._pending is not None:
                if not self._abandoned or self._receive() is None:
                    return None
            if selected not in self.models:
                raise PolicyWorkerError('Requested model was not verified at startup')
            obs = np.asarray(observation, dtype=np.float32)
            if obs.shape != (1, 61) or not np.isfinite(obs).all():
                raise PolicyWorkerError('Inference observation invalid')
            request_id = self._sequence + 1
            try:
                self._socket.send(_encode(dict(id=request_id, model=selected, obs=obs.tolist())))
            except BlockingIOError:
                return None
            self._sequence = self._pending = request_id
            self._abandoned = False
            self.submitted += 1
            return request_id
        finally:
            self._lock.release()

    def take(self, request_id):
        if not self._lock.acquire(blocking=False):
            return None
        try:
            self._alive()
            if self._pending != request_id or self._abandoned:
                raise PolicyWorkerError('Inference request no longer belongs to caller')
            return self._receive()
        finally:
            self._lock.release()

    def discard(self, request_id):
        # No I/O or wait: Python assignment under this short lock only.
        # Called outside control locks; take/submit never block on this lock.
        with self._lock:
            if self._pending == request_id and not self._abandoned:
                self._abandoned = True
                self.discarded += 1

    def run(self, selected, observation, timeout=.1):
        """Preflight only, outside control lock; sampling continues meanwhile."""
        request_id = self.submit(selected, observation)
        if request_id is None:
            raise PolicyWorkerError('推理进程仍在处理上一请求，请等待后重新启动')
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                result = self.take(request_id)
                if result is not None:
                    return result
                time.sleep(.001)
            raise PolicyWorkerError('启动前独立推理超时；未启动模型动作')
        finally:
            self.discard(request_id)

    def status(self):
        return dict(protocol=1, backend='onnx_subprocess', pid=self.process.pid,
            alive=not self._closed and self.process.poll() is None,
            submitted=self.submitted, completed=self.completed, discarded=self.discarded,
            in_flight=self._pending is not None, last_compute_ms=self.last_compute_ms,
            queue_capacity=1, serial_access=False)

    def close(self):
        self._closed = True
        process = getattr(self, 'process', None)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.)
        self._socket.close()


def allowed_worker_command(executable, command, root):
    root = Path(root)
    return (executable == sys.executable and isinstance(command, (tuple, list))
        and len(command) == 7 and list(command[:5]) == [sys.executable, '-I', '-B',
            str(root / 'policy/policy_worker.py'), '--worker']
        and type(command[5]) is str and command[5].isdecimal() and 3 <= int(command[5]) <= 65535
        and command[6] == str(root / 'policy/models'))


def _worker(fd, models_dir):
    # Defence in depth for Python-level accidental device/subprocess access.
    # Not an OS sandbox claim: native ORT is part of the trusted CPU runtime.
    def deny_hardware(event, args):
        if event == 'open' and args and isinstance(args[0], (str, bytes)):
            path = os.path.realpath(os.fsdecode(args[0]))
            if path.startswith('/dev/') and path != '/dev/null' and not path.startswith('/dev/shm/'):
                raise PermissionError('CPU worker cannot open devices')
        if event in ('subprocess.Popen', 'os.system', 'os.exec', 'os.fork', 'os.posix_spawn'):
            raise PermissionError('CPU worker cannot launch another process')
    sys.addaudithook(deny_hardware)
    parent_pid = os.getppid()
    if parent_pid == 1:
        return
    channel = socket.socket(fileno=fd)
    startup = _decode(channel.recv(MAX_PACKET))
    hashes = startup.get('hashes')
    if not isinstance(hashes, dict) or not hashes or set(hashes) - set(NAMES):
        raise ValueError('Model hash allowlist invalid')
    import onnxruntime as ort
    sessions = {}
    for name, digest in hashes.items():
        path = Path(models_dir) / ('xgoduck_' + name + '.onnx')
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Model hash mismatch')
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        session = ort.InferenceSession(str(path), sess_options=opts, providers=['CPUExecutionProvider'])
        if session.get_providers() != ['CPUExecutionProvider'] or len(session.get_inputs()) != 1 or session.get_inputs()[0].shape != [1, 61] or len(session.get_outputs()) != 1 or session.get_outputs()[0].shape != [1, 14]:
            raise ValueError('Model schema/provider invalid')
        for _ in range(5):
            session.run(None, {session.get_inputs()[0].name: np.zeros((1, 61), np.float32)})
        sessions[name] = session
    channel.send(_encode(dict(ready=True, models=sorted(hashes))))
    channel.settimeout(.25)
    while os.getppid() == parent_pid:
        try:
            request = _decode(channel.recv(MAX_PACKET))
        except socket.timeout:
            continue
        request_id = request.get('id')
        try:
            if type(request_id) is not int or request_id < 1:
                raise ValueError('Request ID invalid')
            session = sessions[request['model']]
            obs = np.asarray(request['obs'], dtype=np.float32)
            if obs.shape != (1, 61) or not np.isfinite(obs).all():
                raise ValueError('Observation invalid')
            began = time.perf_counter()
            result = np.asarray(session.run(None, {session.get_inputs()[0].name: obs})[0])
            elapsed = (time.perf_counter() - began) * 1000
            if result.shape != (1, 14) or not np.isfinite(result).all():
                raise ValueError('Action invalid')
            channel.send(_encode(dict(id=request_id, action=result.tolist(), compute_ms=elapsed)))
        except Exception:
            channel.send(_encode(dict(id=request_id, error=True)))


if __name__ == '__main__':
    if len(sys.argv) != 4 or sys.argv[1] != '--worker':
        raise SystemExit('Only the private CPU worker entry point is supported')
    _worker(int(sys.argv[2]), sys.argv[3])
