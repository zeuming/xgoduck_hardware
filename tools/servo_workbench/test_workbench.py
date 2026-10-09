"""Run with an interpreter containing pyserial. Never opens real hardware."""
import json
import threading
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import socket
import tempfile
from pathlib import Path

from protocol import Bus, BusError, NoReply, ServoFault, encode, integer
from app import Controller, Cancelled, create_server


class FakeSerial:
    def __init__(self, ids=(1,), echo=False, corrupt=False):
        self.mem = {i: bytearray(80) for i in ids}
        for sid, mem in self.mem.items():
            mem[5] = sid
            mem[55] = 1
            mem[56:58] = (2040).to_bytes(2, 'little')
        self.buffer = bytearray()
        # Model HLS-style EPROM gating separately from volatile readback.
        self.persisted_ids = {id(mem): sid for sid, mem in self.mem.items()}
        self.writes = []
        self.echo, self.corrupt = echo, corrupt
        self.follow = True
        self.drop_position = False
        self.errors = {}

    @property
    def in_waiting(self):
        return min(len(self.buffer), 3)  # Fragment frames like serial transport can.

    def reset_input_buffer(self):
        self.buffer.clear()

    def flush(self):
        pass

    def close(self):
        pass

    def power_cycle(self):
        memories = list(self.mem.values())
        self.mem = {}
        for mem in memories:
            sid = self.persisted_ids[id(mem)]
            mem[5], mem[40], mem[55] = sid, 0, 1
            self.mem[sid] = mem
        self.buffer.clear()

    def read(self, n):
        data = bytes(self.buffer[:n])
        del self.buffer[:n]
        return data

    def write(self, packet):
        self.writes.append(packet)
        assert sum(packet[2:]) & 255 == 255
        if self.echo:
            self.buffer.extend(packet)
        sid, op, params = packet[2], packet[4], packet[5:-1]
        if sid not in self.mem:
            return
        mem = self.mem[sid]
        payload = b''
        if op == 2:
            addr, length = params
            if self.drop_position and addr == 56:
                return
            payload = bytes(mem[addr:addr + length])
        elif op == 3:
            addr, data = params[0], params[1:]
            if addr == 5 and mem[55] == 0 and mem[40] == 0:
                self.persisted_ids[id(mem)] = data[0]
            mem[addr:addr + len(data)] = data
            if addr == 5:
                del self.mem[sid]
                sid = data[0]
                self.mem[sid] = mem
            if addr == 42 and self.follow and mem[40] == 1:
                mem[56:58] = data[:2]
        else:
            raise AssertionError('Unexpected instruction')
        frame = bytes([sid, len(payload) + 2, self.errors.get(sid, 0)]) + payload
        checksum = (~sum(frame)) & 255
        if self.corrupt:
            checksum ^= 1
        self.buffer.extend(b'\xff\xff' + frame + bytes([checksum]))


class ProtocolTests(unittest.TestCase):
    def bus(self, **kwargs):
        transport = FakeSerial(**kwargs)
        return Bus('', transport=transport), transport

    def test_read_with_echo_and_fragmented_response(self):
        bus, fake = self.bus(echo=True)
        self.assertEqual(bus.snapshot(1), dict(id=1, position=2040, torque=0))
        self.assertTrue(all(p[4] == 2 for p in fake.writes))

    def test_invalid_checksum_rejected(self):
        bus, _ = self.bus(corrupt=True)
        with self.assertRaises(NoReply):
            bus.read(1, 5, 1, timeout=.005)

    def test_short_valid_fault_reply_is_fault_not_offline(self):
        bus, fake = self.bus()
        fake.write = lambda packet: fake.buffer.extend(encode(packet[2], 1, []))
        with self.assertRaises(ServoFault) as fault:
            bus.read(1, 56, 6, timeout=.01)
        self.assertEqual(fault.exception.code, 1)

    def test_no_broadcast_or_invalid_input(self):
        for sid in (-1, 254, 255, True, 1.5):
            with self.assertRaises(ValueError):
                encode(sid, 2, [5, 1])
        for n in (-1, 4096, '2047', True):
            with self.assertRaises(ValueError):
                integer(n, 0, 4095, '位置')

    def test_change_id_unlock_write_relock_and_new_endpoint(self):
        bus, fake = self.bus()
        result = bus.change_id(1, 10)
        self.assertEqual(result['id'], 10)
        self.assertEqual(fake.mem[10][55], 1)
        self.assertNotIn(1, fake.mem)
        modifications = [(p[2], p[5], list(p[6:-1])) for p in fake.writes if p[4] == 3]
        self.assertEqual(modifications, [(1, 40, [0]), (1, 55, [0]), (1, 5, [10]), (10, 55, [1])])

    def test_volatile_id_readback_is_not_proof_of_persistence(self):
        bus, fake = self.bus()
        fake.mem[1][40] = 1
        # Reproduce the old sequence: valid ID and lock reads, but no torque off.
        bus.write(1, 55, [0])
        bus.write(1, 5, [10])
        bus.write(10, 55, [1])
        self.assertEqual(bus.identify(10), 10)
        self.assertEqual(bus.read(10, 55, 1), b'\x01')
        fake.power_cycle()
        self.assertEqual(bus.identify(1), 1)

    def test_id_change_from_holding_state_survives_simulated_power_cycle(self):
        bus, fake = self.bus()
        fake.mem[1][40] = 1
        self.assertEqual(bus.change_id(1, 10)['torque'], 0)
        fake.power_cycle()
        self.assertEqual(bus.identify(10), 10)
        self.assertNotIn(1, fake.mem)

    def test_id_change_aborts_before_unlock_if_torque_cannot_be_disabled(self):
        bus, fake = self.bus()
        fake.mem[1][40] = 1
        original = bus.write
        def ignore_torque_write(sid, addr, data):
            if addr != 40:
                return original(sid, addr, data)
        with patch.object(bus, 'write', side_effect=ignore_torque_write):
            with self.assertRaises(BusError):
                bus.change_id(1, 10)
        self.assertEqual(fake.mem[1][55], 1)
        self.assertFalse(any(p[4] == 3 and p[5] in (5, 55) for p in fake.writes))

    def test_id_collision_does_not_write(self):
        bus, fake = self.bus(ids=(1, 10))
        with self.assertRaises(BusError):
            bus.change_id(1, 10)
        self.assertTrue(all(p[4] == 2 for p in fake.writes))

    def test_zero_is_valid_id(self):
        bus, fake = self.bus()
        self.assertEqual(bus.change_id(1, 0)['id'], 0)
        self.assertEqual(fake.mem[0][55], 1)

    def test_failed_id_verification_relocks_old(self):
        bus, fake = self.bus()
        real_write = bus.write
        def ignore_id_write(sid, addr, data):
            if addr != 5:
                return real_write(sid, addr, data)
        with patch.object(bus, 'write', side_effect=ignore_id_write):
            with self.assertRaises(NoReply):
                bus.change_id(1, 10)
        self.assertEqual(fake.mem[1][55], 1)


class BridgeConfigurationTests(unittest.TestCase):
    def controller(self):
        from types import SimpleNamespace
        ports = [SimpleNamespace(device=p, description='ESP32 COM', vid=0x1A86, pid=0x7523)
                 for p in ('/dev/ttyUSB0', '/dev/ttyUSB1')]
        return Controller(monitor=False, transport='esp32', bridge_port='/dev/ttyUSB0',
                          port_source=lambda: ports)

    def test_bridge_uses_only_explicit_com_and_separates_baud_rates(self):
        c = self.controller()
        self.assertEqual(c.inventory(), '/dev/ttyUSB0')
        state = c.snapshot()
        self.assertEqual([p['device'] for p in state['ports']], ['/dev/ttyUSB0'])
        self.assertEqual(state['host_baud'], 115200)
        self.assertEqual(state['servo_baud'], 1000000)
        self.assertEqual(state['sid'], 10)

    def test_bridge_rejects_other_baud_and_ports_before_starting_action(self):
        c = self.controller()
        with patch.object(c, 'start') as start:
            for port, baud in [('/dev/ttyUSB0', 115200), ('/dev/ttyUSB1', 1000000)]:
                with self.assertRaises(ValueError):
                    c.configure(dict(port=port, baud=baud, sid=11))
            start.assert_not_called()
            c.configure(dict(port='/dev/ttyUSB0', baud=1000000, sid=11))
            start.assert_called_once_with('read', {})


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.bus = Bus('', transport=self.fake)
        self.c = Controller(monitor=False)

    def test_center_moves_and_reads_back_without_eeprom_writes(self):
        self.c.move(self.bus, 1, 2047)
        self.assertEqual(self.c.snapshot()['servo']['position'], 2047)
        self.assertEqual(self.fake.mem[1][40], 1)
        self.assertTrue(all(p[5] in (40, 42) for p in self.fake.writes if p[4] == 3))

    def test_position_action_reads_without_moving_or_changing_torque(self):
        with patch.object(self.c, 'open_bus', return_value=self.bus):
            self.c.start('position', {})
            self.c.worker.join(timeout=2)
        state = self.c.snapshot()
        self.assertFalse(state['busy'])
        self.assertEqual(state['servo']['position'], 2040)
        self.assertIn('当前位置 2040', state['message'])
        self.assertTrue(self.fake.writes)
        self.assertTrue(all(packet[4] == 2 for packet in self.fake.writes))
        self.assertEqual(self.fake.mem[1][40], 0)

    def test_stalled_motion_releases_torque(self):
        self.fake.follow = False
        with self.assertRaisesRegex(BusError, '位置未能跟随'):
            self.c.move(self.bus, 1, 2200)
        self.assertEqual(self.fake.mem[1][40], 0)

    def test_cancel_during_motion_releases(self):
        original = self.bus.goal
        def goal_and_cancel(sid, target):
            original(sid, target)
            if target != 2040:
                self.c.cancel.set()
        with patch.object(self.bus, 'goal', side_effect=goal_and_cancel):
            with self.assertRaises(Cancelled):
                self.c.move(self.bus, 1, 2047)
        self.assertEqual(self.fake.mem[1][40], 0)

    def test_disconnect_during_motion_attempts_release(self):
        original = self.bus.goal
        def drop_on_movement(sid, target):
            original(sid, target)
            if target != 2040:
                self.fake.drop_position = True
        with patch.object(self.bus, 'goal', side_effect=drop_on_movement):
            with self.assertRaises(NoReply):
                self.c.move(self.bus, 1, 2047)
        self.assertEqual(self.fake.mem[1][40], 0)

    def test_refuse_unexpected_torque_mode(self):
        self.fake.mem[1][40] = 128
        with self.assertRaises(BusError):
            self.c.move(self.bus, 1, 2047)
        self.assertTrue(all(p[4] == 2 for p in self.fake.writes))

    def test_action_validation_and_busy_rejection(self):
        self.c.update(connected=True, sid=1)
        with self.assertRaises(ValueError):
            self.c.start('move', dict(sid=1, target=4096))
        with self.assertRaises(ValueError):
            self.c.start('id', dict(sid=1, new_id=254))
        with self.assertRaises(BusError):
            self.c.start('id', dict(sid=2, new_id=10))
        with self.assertRaises(BusError):
            self.c.start('move', dict(sid=2, target=2047))
        self.c.update(busy=True)
        with self.assertRaises(BusError):
            self.c.start('read', {})


class ScanTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial(ids=(1, 10))
        self.fake.mem[10][56:58] = (3100).to_bytes(2, 'little')
        self.bus = Bus('', transport=self.fake)
        self.c = Controller(monitor=False)
        original = self.bus.identify
        def fast_identify(sid, timeout=.12):
            if sid not in self.fake.mem:
                raise NoReply('not present')
            return original(sid, timeout)
        self.bus.identify = fast_identify

    def test_fault_is_online_and_scan_continues_to_other_ids(self):
        self.fake.errors[1] = 1
        self.c.update(connected=True)
        with patch.object(self.c, 'open_bus', return_value=self.bus):
            self.c.start('scan', {})
            self.c.worker.join(timeout=3)
        state = self.c.snapshot()
        self.assertEqual(state['found'], [1, 10])
        self.assertEqual(state['scan_status'], 'complete')
        self.assertEqual([r['status'] for r in state['known_servos']], ['fault', 'online'])
        self.assertIn('0x01', state['scan_results'][0]['error'])
        self.assertEqual(state['scan_results'][1]['position'], 3100)
        self.assertTrue(all(p[4] == 2 for p in self.fake.writes))

    def test_known_id_offline_and_reconnected_without_changing_selection(self):
        self.c.scan(self.bus)
        mem = self.fake.mem.pop(10)
        self.c.poll_known(self.bus, 1)
        self.assertEqual(self.c.snapshot()['known_servos'][1]['status'], 'offline')
        self.fake.mem[10] = mem
        self.c.poll_known(self.bus, 1)
        self.assertEqual(self.c.snapshot()['known_servos'][1]['status'], 'online')
        self.assertEqual(self.c.snapshot()['sid'], 1)
        self.assertTrue(all(p[4] == 2 for p in self.fake.writes))

    def test_registry_persists_ids_but_never_stale_online_status(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'known.json'
            c = Controller(monitor=False, registry_path=path)
            c.record_presence(10, 'online')
            c.record_presence(11, 'fault', '0x01')
            c.renamed(10, 20)
            restored = Controller(monitor=False, registry_path=path)
            rows = restored.snapshot()['known_servos']
            self.assertEqual([r['id'] for r in rows], [11, 20])
            self.assertEqual([r['status'] for r in rows], ['unknown', 'unknown'])

    def test_adapter_disconnect_is_unknown_not_stale_online(self):
        c = Controller(monitor=False, port_source=lambda: [])
        c.record_presence(10, 'online')
        c.inventory()
        row = c.snapshot()['known_servos'][0]
        self.assertEqual(row['status'], 'unknown')
        self.assertIsNone(row['checked_at'])

    def test_serial_open_failure_clears_stale_online_status(self):
        c = Controller(monitor=False, bus_factory=lambda *args: (_ for _ in ()).throw(OSError('busy')))
        c.record_presence(10, 'online')
        with patch.object(c, 'inventory', return_value='fake-port'):
            with self.assertRaises(OSError):
                c.open_bus()
        self.assertEqual(c.snapshot()['known_servos'][0]['status'], 'unknown')

    def test_rename_selected_servo_on_multi_servo_bus(self):
        self.fake.mem[1][40] = 1
        other_before = bytes(self.fake.mem[1])
        self.c.update(sid=10, connected=True)
        with patch.object(self.c, 'open_bus', return_value=self.bus):
            self.c.start('id', dict(sid=10, new_id=20))
            self.c.worker.join(timeout=3)
        state = self.c.snapshot()
        self.assertFalse(state['busy'])
        self.assertEqual(state['sid'], 20)
        self.assertEqual(state['servo']['id'], 20)
        self.assertEqual(state['found'], [1, 20])
        self.assertEqual([r['id'] for r in state['scan_results']], [1, 20])
        self.assertEqual(bytes(self.fake.mem[1]), other_before)
        self.assertTrue(all(p[2] in (10, 20) for p in self.fake.writes if p[4] == 3))
        self.fake.power_cycle()
        self.assertEqual(sorted(self.fake.mem), [1, 20])

    def test_rename_to_occupied_id_is_blocked_without_writes(self):
        self.c.update(sid=1, connected=True)
        with patch.object(self.c, 'open_bus', return_value=self.bus):
            self.c.start('id', dict(sid=1, new_id=10))
            self.c.worker.join(timeout=3)
        state = self.c.snapshot()
        self.assertFalse(state['busy'])
        self.assertEqual(state['sid'], 1)
        self.assertIn('已有舵机响应', state['message'])
        self.assertTrue(all(p[4] == 2 for p in self.fake.writes))

    def test_missing_selected_servo_is_blocked_without_writes(self):
        self.c.update(sid=11, connected=True)
        with patch.object(self.c, 'open_bus', return_value=self.bus):
            self.c.start('id', dict(sid=11, new_id=20))
            self.c.worker.join(timeout=3)
        self.assertIn('选中的 ID 11 没有响应', self.c.snapshot()['message'])
        self.assertTrue(all(p[4] == 2 for p in self.fake.writes))

    def test_scan_reads_each_id_and_position_and_never_writes(self):
        self.assertEqual(self.c.scan(self.bus, include_positions=True), [1, 10])
        state = self.c.snapshot()
        self.assertEqual([(r['id'], r['position']) for r in state['scan_results']], [(1, 2040), (10, 3100)])
        self.assertEqual(state['scan_status'], 'complete')
        self.assertIsNotNone(state['scan_finished'])
        self.assertTrue(all(packet[4] == 2 for packet in self.fake.writes))
        self.fake.mem.clear()
        self.assertEqual(self.c.scan(self.bus, include_positions=True), [])
        self.assertEqual(self.c.snapshot()['scan_results'], [])

    def test_position_failure_does_not_hide_other_servo(self):
        original = self.bus.snapshot
        def fail_one(sid):
            if sid == 1:
                raise NoReply('position unavailable')
            return original(sid)
        with patch.object(self.bus, 'snapshot', side_effect=fail_one):
            self.c.scan(self.bus, include_positions=True)
        rows = self.c.snapshot()['scan_results']
        self.assertIsNone(rows[0]['position'])
        self.assertEqual(rows[0]['error'], 'position unavailable')
        self.assertEqual(rows[1]['position'], 3100)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller(monitor=False)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            self.port = sock.getsockname()[1]
        self.server = create_server(self.c, self.port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.c.close()

    def test_page_and_status_do_not_trigger_jobs(self):
        with urlopen(self.base) as r:
            page = r.read().decode()
        self.assertIn('舵机工作台', page)
        self.assertNotIn('__TOKEN__', page)
        with urlopen(self.base + '/api/status') as r:
            self.assertFalse(json.load(r)['busy'])
        self.assertIsNone(self.c.worker)

    def test_unauthorized_write_and_foreign_host_rejected(self):
        req = Request(self.base + '/api/move', data=b'{"target":2047}', headers={'Content-Type': 'application/json'})
        with self.assertRaises(HTTPError) as result:
            urlopen(req)
        self.assertEqual(result.exception.code, 403)
        req = Request(self.base + '/api/status', headers={'Host': 'attacker.example'})
        with self.assertRaises(HTTPError) as result:
            urlopen(req)
        self.assertEqual(result.exception.code, 403)


if __name__ == '__main__':
    unittest.main()
