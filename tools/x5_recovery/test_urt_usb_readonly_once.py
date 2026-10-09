import contextlib
import io
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import urt_usb_readonly_once as tool


def port(device='/dev/cu.urt2', vid=0x1a86, pid=0x55d3):
    return SimpleNamespace(device=device, vid=vid, pid=pid, serial_number='urt', location='test')


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, delay):
        self.now += delay


class FakeSerial:
    def __init__(self, *, fault=None, no_reply_at=None, torque=0, corrupt=None, echo=True,
                 close_error=False, unsolicited=False, wrong_id=False, wrong_length=False,
                 short_write=False, **settings):
        self.settings, self.port = settings, None
        self.fault, self.no_reply_at, self.torque = fault, no_reply_at, torque
        self.wrong_id, self.wrong_length, self.short_write = wrong_id, wrong_length, short_write
        self.corrupt, self.echo, self.close_error = corrupt, echo, close_error
        self.is_open = False
        self.buffer = bytearray(b'unknown' if unsolicited else b'')
        self.writes, self.open_count, self.close_count = [], 0, 0

    @property
    def in_waiting(self):
        return len(self.buffer)

    def open(self):
        self.open_count += 1
        self.is_open = True

    def close(self):
        self.close_count += 1
        if self.close_error:
            raise OSError('close failed')
        self.is_open = False

    def fileno(self):
        return 7

    def write(self, packet):
        self.writes.append(packet)
        sid, address, size = packet[2], packet[5], packet[6]
        if self.short_write:
            return len(packet) - 1
        if (sid, address) == self.no_reply_at:
            return len(packet)
        if address == 5:
            payload = bytes((sid,))
        elif address == 56:
            payload = (2000 + sid).to_bytes(2, 'little') + b'\x00' * 4
        else:
            payload = bytes((self.torque,))
        error = 3 if (sid, address) == self.fault else 0
        if error:
            payload = b''  # Valid short status error.
        if self.wrong_length:
            payload += b'\x00\x00'
        body = bytes((sid + 1 if self.wrong_id else sid, len(payload) + 2, error)) + payload
        reply = b'\xff\xff' + body + bytes(((~sum(body)) & 255,))
        if (sid, address) == self.corrupt:
            reply = reply[:-1] + bytes((reply[-1] ^ 1,))
        self.buffer.extend((packet if self.echo else b'') + reply)
        return len(packet)

    def read(self, count):
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def flush(self):
        raise AssertionError('flush must never be called')

    def reset_input_buffer(self):
        raise AssertionError('input bytes must never be flushed')


class OneShotTests(unittest.TestCase):
    def run_fake(self, **options):
        clock = Clock()
        made = []
        def factory(**settings):
            raw = FakeSerial(**options, **settings)
            made.append(raw)
            return raw
        result = tool.run_once('/dev/cu.urt2', physical_isolation_confirmed=True,
            ports=[port()], serial_factory=factory, clock=clock, sleep=clock.sleep,
            ownership_guard=SimpleNamespace(before_open=lambda: None, after_open=lambda raw: None,
                                            proof={'fake_guard': True}))
        return result, made[0]

    def test_default_never_imports_serial_or_opens_devices(self):
        original_import = __import__
        def guarded_import(name, *args, **kwargs):
            if name == 'serial' or name.startswith('serial.'):
                raise AssertionError('Default must not load pyserial')
            return original_import(name, *args, **kwargs)
        output = io.StringIO()
        with patch('builtins.__import__', guarded_import), contextlib.redirect_stdout(output):
            self.assertEqual(tool.main([]), 0)
        self.assertIn('prepared_only', output.getvalue())

    def test_success_exact_45_reads_one_open_and_actual_close(self):
        result, raw = self.run_fake()
        self.assertTrue(result['all_15_healthy_torque_off'])
        self.assertTrue(result['serial_closed'])
        self.assertEqual((raw.open_count, raw.close_count), (1, 1))
        self.assertEqual(len(raw.writes), 45)
        self.assertTrue(all(p[4] == 2 and len(p) == 8 and sum(p[2:]) & 255 == 255 for p in raw.writes))
        self.assertEqual([(p[2], p[5], p[6]) for p in raw.writes],
                         [(sid, addr, size) for sid in tool.IDS for addr, size in tool.READS])
        self.assertTrue(all(r['torque'] == 0 for r in result['results']))
        self.assertEqual(raw.settings['timeout'], 0)
        self.assertEqual(raw.settings['write_timeout'], .05)

    def test_fault_stops_immediately_keeps_rest_unknown(self):
        result, raw = self.run_fake(fault=(10, 56))
        self.assertEqual(len(raw.writes), 2)
        self.assertEqual(result['results'][0]['device_error'], 3)
        self.assertTrue(all(r['status'] == 'unknown' for r in result['results'][1:]))
        self.assertTrue(result['serial_closed'])
        self.assertFalse(result['all_15_healthy_torque_off'])

    def test_timeout_has_no_retry_and_no_current_torque_claim(self):
        result, raw = self.run_fake(no_reply_at=(10, 40))
        self.assertEqual(len(raw.writes), 3)
        self.assertEqual(result['results'][0]['status'], 'timeout')
        self.assertNotIn('torque', result['results'][0])
        self.assertLessEqual(result['elapsed_seconds'], .120001)
        self.assertTrue(result['serial_closed'])

    def test_nonzero_or_invalid_torque_never_sends_disable(self):
        for value in (1, 255):
            with self.subTest(value=value):
                result, raw = self.run_fake(torque=value)
                self.assertEqual(len(raw.writes), 3)
                self.assertEqual(result['results'][0]['torque'], value)
                self.assertFalse(result['all_15_healthy_torque_off'])
                self.assertTrue(all(p[4] == 2 for p in raw.writes))

    def test_bad_checksum_or_unsolicited_input_closes_without_retry(self):
        result, raw = self.run_fake(corrupt=(10, 5))
        self.assertEqual(len(raw.writes), 1)
        self.assertIn('checksum', result['error'])
        self.assertTrue(result['serial_closed'])

    def test_wrong_id_length_and_short_write_stop_at_first_read(self):
        for option in ('wrong_id', 'wrong_length', 'short_write'):
            with self.subTest(option=option):
                result, raw = self.run_fake(**{option: True})
                self.assertEqual(len(raw.writes), 1)
                self.assertTrue(result['serial_closed'])
                self.assertFalse(result['all_15_healthy_torque_off'])
        result, raw = self.run_fake(unsolicited=True)
        self.assertEqual(raw.writes, [])
        self.assertIn('not flushed', result['error'])
        self.assertTrue(result['serial_closed'])

    def test_bad_ports_or_missing_physical_flag_never_construct_transport(self):
        for infos, physical in (([port(pid=0x7523)], True), ([port(), port()], True),
                                ([port(), port('/dev/cu.urt3')], True), ([port()], False)):
            with self.subTest(infos=infos, physical=physical):
                factory = unittest.mock.Mock(side_effect=AssertionError('Must refuse before constructing'))
                result = tool.run_once('/dev/cu.urt2', physical_isolation_confirmed=physical,
                                       ports=infos, serial_factory=factory)
                self.assertEqual(result['serial_open_attempts'], 0)
                factory.assert_not_called()

    def test_close_failure_never_grants_complete_health(self):
        result, raw = self.run_fake(close_error=True)
        self.assertTrue(result['query_completed'])
        self.assertFalse(result['serial_closed'])
        self.assertFalse(result['all_15_healthy_torque_off'])
        self.assertEqual(raw.close_count, 1)

    def test_encoder_rejects_all_non_whitelisted_commands(self):
        for args in ((1, 5, 1), (10, 42, 2), (10, 40, 2), (True, 5, 1), (10, 5, True)):
            with self.assertRaises(tool.ReadError):
                tool.read_frame(*args)

    def test_lsof_aliases_timeout_and_exclusive_owner_proof_without_devices(self):
        outputs = [SimpleNamespace(returncode=1, stdout='', stderr=''),
                   SimpleNamespace(returncode=0, stdout='p%d\nf7\n' % os.getpid(), stderr='')]
        calls, claims = [], []
        def runner(args, **kwargs):
            calls.append((args, kwargs))
            return outputs.pop(0)
        guard = tool.LocalOwnershipGuard('/dev/cu.urt2', runner=runner,
                                        claim_exclusive=lambda raw: claims.append(raw.fileno()))
        guard.before_open()
        guard.after_open(FakeSerial())
        self.assertEqual(claims, [7])
        self.assertTrue(guard.proof['tioc_exclusive_claimed'])
        self.assertEqual(calls[0][0][-2:], ['/dev/cu.urt2', '/dev/tty.urt2'])
        self.assertEqual(calls[0][1]['timeout'], 2.)

    def test_owned_or_unverifiable_port_refused_without_uart_open(self):
        for response in (SimpleNamespace(returncode=0, stdout='p123\nf9\n', stderr=''),
                         SimpleNamespace(returncode=1, stdout='', stderr='cannot inspect')):
            factory = unittest.mock.Mock(side_effect=AssertionError('Must not open'))
            guard = tool.LocalOwnershipGuard('/dev/cu.urt2', runner=lambda *a, **kw: response,
                                            claim_exclusive=lambda raw: None)
            result = tool.run_once('/dev/cu.urt2', physical_isolation_confirmed=True,
                                  ports=[port()], serial_factory=factory, ownership_guard=guard)
            factory.assert_not_called()
            self.assertEqual(result['serial_open_attempts'], 0)

    def test_owner_race_after_open_closes_before_any_read_request(self):
        outputs = [SimpleNamespace(returncode=1, stdout='', stderr=''),
                   SimpleNamespace(returncode=0, stdout='p123\nf9\n', stderr='')]
        guard = tool.LocalOwnershipGuard('/dev/cu.urt2', runner=lambda *a, **kw: outputs.pop(0),
                                        claim_exclusive=lambda raw: None)
        raw = FakeSerial()
        result = tool.run_once('/dev/cu.urt2', physical_isolation_confirmed=True,
                              ports=[port()], serial_factory=lambda **kwargs: raw,
                              ownership_guard=guard)
        self.assertEqual(raw.writes, [])
        self.assertTrue(result['serial_closed'])


if __name__ == '__main__':
    unittest.main()
