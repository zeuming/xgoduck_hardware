"""Protocol/transport tests using only in-memory COM and servo simulators."""
import json
import threading
import time
import unittest

from esp_bridge import (BridgeBus, BridgeError, EspSession, EspSessionManager,
                        MAX_LINE_BYTES, MAX_TRANSACTION_BYTES, PREFIX, WriteUncertain)
from protocol import BusError, NoReply, ServoFault, encode
from test_workbench import FakeSerial


def wire_line(value):
    return (json.dumps(value) + '\n').encode()


class FakeCom:
    def __init__(self, factory, **options):
        self.factory = factory
        self.options = options
        self.port = None
        self.dtr = True
        self.rts = True
        self.is_open = False
        self.buffer = bytearray()
        self.commands = []
        self.read_after = 0
        self.fragment = 7
        self.before_reply = None
        self.drop_reply = False
        self.no_ack = False
        self.compatible = True
        self.close_count = 0

    def open(self):
        assert self.dtr is False and self.rts is False
        assert self.options['port'] is None
        assert self.options['baudrate'] == 115200
        self.is_open = True

    def close(self):
        self.close_count += 1
        self.is_open = False

    def reset_input_buffer(self):
        self.buffer.clear()

    @property
    def in_waiting(self):
        if time.monotonic() < self.read_after:
            return 0
        return min(self.fragment, len(self.buffer))

    def read(self, size):
        if time.monotonic() < self.read_after or not self.buffer:
            time.sleep(.001)
            return b''
        data = bytes(self.buffer[:min(size, self.fragment)])
        del self.buffer[:len(data)]
        return data

    def write(self, command):
        assert self.is_open
        self.commands.append(command)
        if command == b'INFO\n':
            self.buffer.extend(b'ESP-ROM:esp32s3-20210327\r\nnot json\n')
            self.buffer.extend(wire_line(dict(prefix=PREFIX, command='INFO', protocol=1,
                                             event='reply', firmware='0.2.0',
                                             capabilities=dict(workbench_xfer=self.compatible))))
        else:
            name, session, sequence, packet = command.decode().strip().split()
            assert name == 'XFER'
            packet = bytes.fromhex(packet)
            self.factory.servo.reset_input_buffer()
            self.factory.servo.write(packet)
            raw = bytes(self.factory.servo.buffer)
            if self.no_ack and packet[4] == 3:
                raw = b''
            reply = dict(prefix=PREFIX, command='XFER', session=session,
                         seq=int(sequence), ok=True, rx_hex=raw.hex(), error='')
            if self.before_reply:
                self.before_reply(self, packet, reply)
            if not self.drop_reply:
                self.buffer.extend(wire_line(reply))
        return len(command)


class FakeFactory:
    def __init__(self, **servo_options):
        self.servo = FakeSerial(**servo_options)
        self.ports = []
        self.configure = lambda port: None

    def __call__(self, **options):
        port = FakeCom(self, **options)
        self.configure(port)
        self.ports.append(port)
        return port


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.factory = FakeFactory()
        self.manager = EspSessionManager(serial_factory=self.factory, startup_delay=0, timeout=.4)
        self.session = self.manager.get('simulated-COM')
        self.bus = BridgeBus(self.session)
        self.addCleanup(self.manager.close)

    def test_lazy_open_and_persistent_port_across_bus_leases(self):
        self.assertEqual(self.factory.ports, [])
        self.assertIs(self.session, self.manager.get('simulated-COM'))
        self.assertEqual(self.bus.snapshot(1), dict(id=1, position=2040, torque=0))
        self.bus.close()
        second = BridgeBus(self.session)
        self.assertEqual(second.snapshot(1)['position'], 2040)
        second.close()
        self.assertEqual(len(self.factory.ports), 1)
        port = self.factory.ports[0]
        self.assertTrue(port.is_open)
        self.assertEqual(port.commands.count(b'INFO\n'), 1)
        self.assertTrue(all(p[4] == 2 for p in self.factory.servo.writes))
        self.manager.close()
        self.assertFalse(port.is_open)

    def test_fragmented_json_noise_and_stale_identifiers(self):
        def before_reply(port, packet, reply):
            wrong = encode(1, 0, [99]).hex()
            port.buffer.extend(b'boot console message\n{incomplete json}\n')
            port.buffer.extend(wire_line(dict(reply, seq=reply['seq'] - 1, rx_hex=wrong)))
            port.buffer.extend(wire_line(dict(reply, session='0' * 16, rx_hex=wrong)))
            port.buffer.extend(wire_line(dict(reply, seq=True, rx_hex=wrong)))
        self.factory.configure = lambda p: setattr(p, 'before_reply', before_reply)
        self.assertEqual(self.bus.identify(1), 1)
        self.assertEqual(self.bus.identify(1), 1)

    def test_scan_timeout_does_not_become_json_deadline(self):
        def delay_reply(port, packet, reply):
            port.read_after = time.monotonic() + .06
        self.factory.configure = lambda p: setattr(p, 'before_reply', delay_reply)
        self.assertEqual(self.bus.identify(1, timeout=.025), 1)

    def test_old_read_only_firmware_rejected_without_servo_command(self):
        self.factory.configure = lambda p: setattr(p, 'compatible', False)
        with self.assertRaisesRegex(BridgeError, 'workbench_xfer'):
            self.bus.identify(1)
        self.assertEqual(self.factory.servo.writes, [])
        self.assertFalse(self.factory.ports[0].is_open)

    def test_absent_servo_does_not_disconnect_esp(self):
        with self.assertRaises(NoReply):
            self.bus.identify(10)
        self.assertTrue(self.factory.ports[0].is_open)
        self.assertEqual(self.bus.identify(1), 1)
        self.assertEqual(len(self.factory.ports), 1)

    def test_no_ack_write_sent_once_and_readback_still_works(self):
        self.factory.configure = lambda p: setattr(p, 'no_ack', True)
        self.assertIsNone(self.bus.write(1, 40, [1]))
        self.assertEqual(self.bus.read(1, 40, 1), b'\x01')
        writes = [p for p in self.factory.servo.writes if p[4] == 3]
        self.assertEqual(len(writes), 1)
        self.assertTrue(self.factory.ports[0].is_open)

    def test_lost_write_reply_is_uncertain_not_retried_and_read_reconnects(self):
        def lose_write(port, packet, reply):
            if packet[4] == 3:
                port.drop_reply = True
        self.factory.configure = lambda p: setattr(p, 'before_reply', lose_write)
        with self.assertRaisesRegex(WriteUncertain, '结果不确定.*未自动重发'):
            self.bus.goal(1, 2047)
        self.assertEqual(len(self.factory.ports), 1)
        first = self.factory.ports[0]
        self.assertFalse(first.is_open)
        self.assertEqual(len(self.factory.servo.writes), 1)
        self.factory.configure = lambda p: None
        self.assertEqual(self.bus.read(1, 42, 2), (2047).to_bytes(2, 'little'))
        second = self.factory.ports[1]
        self.assertEqual(second.commands[0], b'INFO\n')
        old = first.commands[1].split()
        new = second.commands[1].split()
        self.assertEqual(old[1], new[1])  # Nonce and monotonic seq survive reconnect.
        self.assertGreater(int(new[2]), int(old[2]))
        self.assertEqual(len([p for p in self.factory.servo.writes if p[4] == 3]), 1)

    def test_host_timeout_is_not_mistaken_for_unoccupied_new_id(self):
        self.factory.configure = lambda p: setattr(p, 'drop_reply', True)
        with self.assertRaises(BridgeError) as caught:
            self.bus.identify(10, timeout=.025)
        self.assertNotIsInstance(caught.exception, NoReply)
        self.assertFalse(self.factory.ports[0].is_open)

    def test_id_change_preserves_release_unlock_write_readback_lock_order(self):
        self.factory.servo.mem[1][40] = 1
        self.factory.configure = lambda p: setattr(p, 'no_ack', True)
        renamed = []
        result = self.bus.change_id(1, 10, renamed.append)
        self.assertEqual(result, dict(id=10, position=2040, torque=0))
        self.assertEqual(renamed, [10])
        writes = [(p[2], p[5], list(p[6:-1])) for p in self.factory.servo.writes if p[4] == 3]
        self.assertEqual(writes, [(1, 40, [0]), (1, 55, [0]), (1, 5, [10]), (10, 55, [1])])
        self.factory.servo.power_cycle()
        self.assertEqual(self.bus.identify(10), 10)

    def test_id_change_accepts_ack_using_new_id(self):
        self.assertEqual(self.bus.change_id(1, 10)['id'], 10)
        self.assertEqual(len([p for p in self.factory.servo.writes if p[4] == 3 and p[5] == 5]), 1)

    def test_new_id_collision_never_writes(self):
        self.factory.servo = FakeSerial(ids=(1, 10))
        with self.assertRaisesRegex(BusError, '已有舵机'):
            self.bus.change_id(1, 10)
        self.assertTrue(all(p[4] == 2 for p in self.factory.servo.writes))

    def test_echo_ignored_bad_checksum_rejected_and_servo_fault_preserved(self):
        self.factory.servo.echo = True
        self.assertEqual(self.bus.identify(1), 1)
        self.factory.servo.corrupt = True
        with self.assertRaises(NoReply):
            self.bus.identify(1)
        self.factory.servo.corrupt = False
        self.factory.servo.errors[1] = 1
        with self.assertRaises(ServoFault) as caught:
            self.bus.identify(1)
        self.assertEqual(caught.exception.code, 1)

    def test_short_fault_response_uses_shared_protocol_validation(self):
        def short_fault(port, packet, reply):
            reply['rx_hex'] = encode(packet[2], 1, []).hex()
        self.factory.configure = lambda p: setattr(p, 'before_reply', short_fault)
        with self.assertRaises(ServoFault):
            self.bus.identify(1)

    def test_bad_requests_rejected_before_opening_com(self):
        invalid = [encode(1, 1, []), encode(1, 3, [40, 128]),
                   encode(1, 3, [55, 2]), encode(1, 3, [5, 254]),
                   encode(1, 2, [1, 1]), encode(1, 3, [42, 0, 16, 0, 0, 0, 0]),
                   encode(1, 3, [42, 0, 8, 1, 0, 0, 0])]
        for packet in invalid:
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                self.session.transfer(packet)
        with self.assertRaises(ValueError):
            self.bus.identify(254)
        with self.assertRaises(ValueError):
            BridgeBus(self.session, 115200)
        self.assertEqual(self.factory.ports, [])

    def test_oversized_line_and_malformed_rx_close_connection(self):
        def oversized(port, packet, reply):
            port.fragment = 1024
            port.buffer.extend(b'x' * (MAX_LINE_BYTES + 1))
            port.drop_reply = True
        self.factory.configure = lambda p: setattr(p, 'before_reply', oversized)
        with self.assertRaisesRegex(BridgeError, '长度限制'):
            self.bus.identify(1)
        self.assertFalse(self.factory.ports[0].is_open)
        def bad_hex(port, packet, reply):
            reply['rx_hex'] = 'ff ff'
        self.factory.configure = lambda p: setattr(p, 'before_reply', bad_hex)
        with self.assertRaisesRegex(BridgeError, '格式不合法'):
            self.bus.identify(1)
        self.assertFalse(self.factory.ports[1].is_open)

    def test_reboot_mid_write_is_uncertain(self):
        def reboot(port, packet, reply):
            port.buffer.extend(wire_line(dict(prefix=PREFIX, command='INFO', event='boot')))
        self.factory.configure = lambda p: setattr(p, 'before_reply', reboot)
        with self.assertRaisesRegex(WriteUncertain, '重启'):
            self.bus.write(1, 40, [0])
        self.assertFalse(self.factory.ports[0].is_open)

    def test_serial_disconnect_after_write_never_retries(self):
        def disconnected(port, packet, reply):
            raise OSError('simulated USB removal after transmit')
        self.factory.configure = lambda p: setattr(p, 'before_reply', disconnected)
        with self.assertRaises(WriteUncertain):
            self.bus.write(1, 40, [0])
        self.assertEqual(len(self.factory.servo.writes), 1)
        self.assertFalse(self.factory.ports[0].is_open)

    def test_firmware_error_is_not_a_servo_no_reply(self):
        def rejected(port, packet, reply):
            reply.update(ok=False, rx_hex='', error='session_capacity_exhausted',
                         tx_sent=False, uncertain=False)
        self.factory.configure = lambda p: setattr(p, 'before_reply', rejected)
        with self.assertRaisesRegex(BridgeError, 'session_capacity_exhausted') as caught:
            self.bus.identify(1)
        self.assertNotIsInstance(caught.exception, NoReply)
        self.assertFalse(self.factory.ports[0].is_open)

    def test_endless_short_noise_is_bounded(self):
        def noise(port, packet, reply):
            port.fragment = 1024
            port.buffer.extend(b'{}\n' * (MAX_TRANSACTION_BYTES // 3 + 10))
        self.factory.configure = lambda p: setattr(p, 'before_reply', noise)
        with self.assertRaisesRegex(BridgeError, '数据超出长度限制'):
            self.bus.identify(1)
        self.assertFalse(self.factory.ports[0].is_open)

    def test_concurrent_buses_share_serialized_transactions(self):
        values, failures = [], []
        def identify():
            try:
                values.append(BridgeBus(self.session).identify(1))
            except Exception as exc:
                failures.append(exc)
        workers = [threading.Thread(target=identify) for _ in range(8)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(2)
        self.assertFalse(any(worker.is_alive() for worker in workers))
        self.assertEqual(failures, [])
        self.assertEqual(values, [1] * 8)
        self.assertEqual(len(self.factory.ports), 1)
        requests = [c.split() for c in self.factory.ports[0].commands if c.startswith(b'XFER ')]
        self.assertEqual([int(r[2]) for r in requests], list(range(1, 9)))
        self.assertEqual(len({r[1] for r in requests}), 1)


if __name__ == '__main__':
    unittest.main()
