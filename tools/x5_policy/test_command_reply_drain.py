"""Real IPC framing with slow lifecycle replies, never robot hardware."""
import socket
import threading
import time
import unittest

import native_process as app


class CommandDrainTests(unittest.TestCase):
    def setUp(self):
        self.parents, self.children = {}, {}
        for lane in app.LANES:
            self.parents[lane], self.children[lane] = socket.socketpair()
            self.children[lane].settimeout(1)
            self.addCleanup(self.parents[lane].close)
            self.addCleanup(self.children[lane].close)
        self.client = app.CommandClient(self.parents)
        self.client.CALL_SECONDS = .05
        self.client.REPLY_DRAIN_SECONDS = .6

    def wait_drained(self, lane='command'):
        deadline = time.monotonic()+1
        while self.client.locks[lane].locked() and time.monotonic() < deadline:
            time.sleep(.002)
        self.assertFalse(self.client.locks[lane].locked())

    def test_slow_disconnect_partial_reply_cannot_kill_peer_or_confirm_next_call(self):
        requests, errors = [], []
        peer = self.children['command']
        def work():
            try:
                first = app.receive(peer, app.MAX_COMMAND)
                requests.append(first)
                raw = app.encode(dict(id=first['id'], ok=True, result={'old': True}))
                packet = app.struct.pack('!I', len(raw))+raw
                peer.sendall(packet[:2])
                time.sleep(.1)
                peer.sendall(packet[2:])
                second = app.receive(peer, app.MAX_COMMAND)
                requests.append(second)
                app.send(peer, dict(id=second['id'], ok=True, result={'new': True}))
            except Exception as error:
                errors.append(error)
        thread = threading.Thread(target=work)
        thread.start()
        try:
            began = time.monotonic()
            with self.assertRaisesRegex(ValueError, '执行结果尚未确认'):
                self.client.call('/api/motion', {'action': 'disconnect'}, 'browser')
            self.assertLess(requests[0]['deadline']-began, .08)
            with self.assertRaisesRegex(ValueError, '未排队、未重发'):
                self.client.call('/api/motion', {'action': 'connect'}, 'browser')
            self.wait_drained()
            self.assertEqual(self.client.call('/api/motion', {'action': 'connect'}, 'browser'), {'new': True})
            thread.join(1)
            self.assertEqual(errors, [])
            self.assertEqual(len(requests), 2)
            self.assertNotEqual(requests[0]['id'], requests[1]['id'])
        finally:
            thread.join(1)

    def test_slow_command_keeps_stop_lane_available(self):
        def slow():
            request = app.receive(self.children['command'], app.MAX_COMMAND)
            time.sleep(.1)
            app.send(self.children['command'], dict(id=request['id'], ok=True, result={}))
        def stop():
            request = app.receive(self.children['stop'], app.MAX_COMMAND)
            app.send(self.children['stop'], dict(id=request['id'], ok=True, result={'stopped': True}))
        threads = [threading.Thread(target=slow), threading.Thread(target=stop)]
        for thread in threads:
            thread.start()
        with self.assertRaisesRegex(ValueError, '回复延迟'):
            self.client.call('/api/motion', {'action': 'disconnect'}, 'browser')
        self.assertEqual(self.client.call('/api/motion', {'action': 'stop'}, 'browser'), {'stopped': True})
        for thread in threads:
            thread.join(1)
        self.wait_drained()

    def test_wrong_late_identity_closes_only_its_lane(self):
        def peer():
            app.receive(self.children['command'], app.MAX_COMMAND)
            time.sleep(.1)
            app.send(self.children['command'], dict(id='wrong', ok=True, result={}))
        thread = threading.Thread(target=peer)
        thread.start()
        with self.assertRaisesRegex(ValueError, '回复延迟'):
            self.client.call('/api/motion', {'action': 'disconnect'}, 'browser')
        thread.join(1)
        self.wait_drained()
        self.assertEqual(self.parents['command'].fileno(), -1)
        self.assertNotEqual(self.parents['stop'].fileno(), -1)
        with self.assertRaisesRegex(ValueError, '不会重发'):
            self.client.call('/api/motion', {'action': 'disconnect'}, 'browser')

    def test_missing_reply_is_bounded_without_retransmission(self):
        self.client.REPLY_DRAIN_SECONDS = .1
        with self.assertRaisesRegex(ValueError, '回复延迟'):
            self.client.call('/api/motion', {'action': 'disconnect'}, 'browser')
        request = app.receive(self.children['command'], app.MAX_COMMAND)
        self.assertEqual(request['body']['action'], 'disconnect')
        self.wait_drained()
        self.assertEqual(self.children['command'].recv(1), b'')


if __name__ == '__main__':
    unittest.main()
