"""Actual transport regression and CPU comparison using socket pairs, no UART."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import sys
import termios
import threading
import time
import unittest

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--baseline', type=Path, required=True)
p.add_argument('--candidate', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
assert not a.output.exists()
sys.path.insert(0, str(a.candidate/'policy'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


old = load('transport_before_wait', a.baseline/'policy/production_motion.py')
new = load('transport_after_wait', a.candidate/'policy/production_motion.py')


class SerialSocket:
    def __init__(self, sock):
        self.sock, self.reads = sock, 0
        sock.setblocking(False)

    def fileno(self):
        return self.sock.fileno()

    @property
    def in_waiting(self):
        return struct.unpack('I', fcntl.ioctl(self.fileno(), termios.FIONREAD, struct.pack('I', 0)))[0]

    def read(self, size):
        self.reads += 1
        try:
            return self.sock.recv(size)
        except BlockingIOError:
            return b''


def pair(module, timeout=.16):
    left, right = socket.socketpair()
    t = module.SerialTransport('socket-only', timeout=timeout)
    t.serial, t._posix_write = SerialSocket(left), True
    return t, left, right


def responder(peer, make_frames, delay=0.):
    errors, commands = [], []

    def run():
        try:
            parser = new.FrameParser()
            while not commands:
                commands.extend(parser.feed(peer.recv(1024)))
            time.sleep(delay)
            for frame in make_frames(commands[0]):
                peer.sendall(frame)
                time.sleep(.001)
        except BaseException as error:
            errors.append(str(error))
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, errors, commands


class WaitTests(unittest.TestCase):
    def check(self, make_frames, expected=b'position', delay=0.):
        t,l,r = pair(new)
        thread,errors,commands = responder(r,make_frames,delay)
        try:
            self.assertEqual(t.request(32), expected)
            thread.join(1.)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors)
            self.assertEqual(len(commands),1)
            self.assertEqual(commands[0][0],32)
        finally:
            l.close(); r.close()

    def test_delayed_reply(self):
        self.check(lambda c:[new.encode_frame(0xa0,c[1],b'position')], delay=.03)

    def test_bytewise_fragmented_reply(self):
        self.check(lambda c:[bytes([b]) for b in new.encode_frame(0xa0,c[1],b'position')])

    def test_bad_crc_stale_sequence_and_noise_not_accepted(self):
        def frames(c):
            good = new.encode_frame(0xa0,c[1],b'position')
            bad = bytearray(good); bad[-1] ^= 1
            return [b'noise',bytes(bad),new.encode_frame(0xa0,c[1]+1,b'wrong'),good]
        self.check(frames)

    def test_timeout_sends_exactly_once(self):
        t,l,r = pair(new,timeout=.025)
        try:
            began = time.monotonic()
            with self.assertRaisesRegex(new.MotionError,'响应超时'):
                t.request(32)
            elapsed = time.monotonic()-began
            commands = new.FrameParser().feed(r.recv(1024))
            self.assertEqual(len(commands),1)
            self.assertGreaterEqual(elapsed,.025)
            self.assertLess(elapsed,.25)
            self.assertEqual(t.seq,1)
        finally:
            l.close();r.close()

    def test_async_target_remains_send_only(self):
        t,l,r = pair(new)
        try:
            self.assertEqual(t.request(31,b'target'),b'')
            self.assertEqual(t.serial.reads,0)
            self.assertEqual(new.FrameParser().feed(r.recv(1024)),[(31,1,b'target')])
        finally:
            l.close();r.close()

    def test_error_ack_still_raises(self):
        t,l,r = pair(new)
        thread,errors,commands = responder(r,lambda c:[new.encode_frame(new.ACK,c[1],bytes([32,2]))])
        try:
            with self.assertRaisesRegex(new.MotionError,'错误码 2'):
                t.request(32)
            thread.join(1.)
            self.assertFalse(errors)
            self.assertEqual(len(commands),1)
        finally:
            l.close();r.close()


def benchmark(module):
    rows=[]
    for _ in range(4):
        t,l,r = pair(module)
        thread,errors,commands=responder(r,lambda c:[module.encode_frame(0xa0,c[1],b'position')],.06)
        try:
            began,cpu=time.monotonic(),time.thread_time()
            assert t.request(32)==b'position'
            rows.append(dict(wall_ms=(time.monotonic()-began)*1000,
                requesting_thread_cpu_ms=(time.thread_time()-cpu)*1000,read_calls=t.serial.reads))
            thread.join(1.)
            assert not errors and len(commands)==1 and not thread.is_alive()
        finally:
            l.close();r.close()
    return rows


result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(WaitTests))
bench=dict(before=benchmark(old),after=benchmark(new)) if result.wasSuccessful() else {}
report=dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=len(result.errors),
    failures=len(result.failures),benchmark=bench,hardware_access=False,deployment=False,
    pid=os.getpid(),platform=sys.platform,
    baseline_sha256=hashlib.sha256((a.baseline/'policy/production_motion.py').read_bytes()).hexdigest(),
    candidate_sha256=hashlib.sha256((a.candidate/'policy/production_motion.py').read_bytes()).hexdigest(),
    distinction='Socket readiness proves protocol and empty-read CPU behavior. It does not prove actual UART latency, 50Hz, or walking.')
a.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
raise SystemExit(not result.wasSuccessful())
