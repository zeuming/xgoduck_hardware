"""Pure Mac probe checks: no device enumeration/open, SSH or flashing."""
import copy
import struct
import types
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import feedback_fault_probe
import mac_feedback_fault_probe as mac
from test_feedback_fault_probe import payload


def runtime():
    ns = {}
    exec(compile(feedback_fault_probe.runtime_source(), '<mac-query-test-definitions>', 'exec'), ns)
    return ns


class MacProbeTests(unittest.TestCase):
    def setUp(self):
        self.ns = runtime()

    def test_query_exact_hello_diag_frames_and_preserves_dangerous_flags(self):
        ns = self.ns
        body = payload(ns, armed=1, cookie=27, fault=1, present_faults=1, torque=2)
        hello = b'XGODUCK_MOTION_1'
        h1 = ns['HEADER'].pack(b'XD', 1, 0x81, 16, 1)
        h2 = ns['HEADER'].pack(b'XD', 1, 0x87, 247, 2)
        ns['_read_exact'] = mock.Mock(side_effect=[h1,
            hello + struct.pack('<H', ns['_crc16'](h1 + hello)), h2,
            body + struct.pack('<H', ns['_crc16'](h2 + body))])
        frames = []
        ns['_write_once'] = lambda fd, frame, deadline, metadata: frames.append(frame)
        metadata = {'request_types_attempted': [], 'reply_types_received': [], 'request_bytes_written': 0}
        with mock.patch.object(ns['select'], 'select', return_value=([], [], [])):
            result = mac._query_once(91, ns, ns['time'].monotonic() + 4, metadata)
        self.assertEqual([ns['HEADER'].unpack(frame[:-2])[2:4] for frame in frames], [(1, 0), (7, 0)])
        self.assertEqual(metadata['request_types_attempted'], [1, 7])
        self.assertEqual(metadata['reply_types_received'], [0x81, 0x87])
        self.assertTrue(result['armed'])
        self.assertEqual((result['arm_cookie'], result['fault_mask'], result['torque_mask']), (27, 1, 2))
        self.assertFalse(result['motion_authorized'])

    def test_query_failure_is_not_retried_and_unsolicited_input_rejected(self):
        ns = self.ns
        metadata = {'request_types_attempted': [], 'reply_types_received': [], 'request_bytes_written': 0}
        ns['_write_once'] = mock.Mock()
        with mock.patch.object(ns['select'], 'select', return_value=([91], [], [])):
            with self.assertRaisesRegex(ns['ProbeError'], 'Unexpected serial input'):
                mac._query_once(91, ns, ns['time'].monotonic() + 4, metadata)
        ns['_write_once'].assert_not_called()
        ns['_exchange'] = mock.Mock(return_value=b'XGODUCK_MOTION_1')
        ns['_fault_exchange'] = mock.Mock(side_effect=ns['ProbeError']('Reply CRC mismatch'))
        with self.assertRaisesRegex(ns['ProbeError'], 'CRC'):
            mac._query_once(91, ns, ns['time'].monotonic() + 4, metadata)
        self.assertEqual(ns['_exchange'].call_count, 1)
        self.assertEqual(ns['_fault_exchange'].call_count, 1)

    def test_serial_class_disables_initial_flush_and_modem_writes(self):
        class Base:
            def _reset_input_buffer(self): raise AssertionError('flushed')
            def _update_dtr_state(self): raise AssertionError('DTR write')
            def _update_rts_state(self): raise AssertionError('RTS write')
        port = mac._no_flush_serial(Base)()
        port._reset_input_buffer()
        port._update_dtr_state()
        port._update_rts_state()

    def test_identity_exact_match_and_changes_rejected(self):
        value = dict(device=mac.PORT, vid=0x1a86, pid=0x7523, location='1-1.1', serial_number=None)
        self.assertEqual(mac._identity([types.SimpleNamespace(**value)])['port'], mac.PORT)
        for change in ({'vid': 0}, {'pid': 0}, {'location': '1-2'}, {'serial_number': 'unknown'}, {'device': mac.TTY_ALIAS}):
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                mac._identity([types.SimpleNamespace(**{**value, **change})])
        with self.assertRaises(RuntimeError): mac._identity([])

    def test_dual_alias_holders_strict_and_only_own_fd_after_open(self):
        own = mac.os.getpid()
        def result(code, output, error=b''):
            return types.SimpleNamespace(returncode=code, stdout=output, stderr=error)
        with mock.patch.object(mac.subprocess, 'run', return_value=result(1, b'')) as command:
            mac._holders(mac.time.monotonic() + 4)
            self.assertIn(mac.PORT, command.call_args.args[0])
            self.assertIn(mac.TTY_ALIAS, command.call_args.args[0])
            with self.assertRaises(RuntimeError): mac._holders(mac.time.monotonic() + 4, 91)
        with mock.patch.object(mac.subprocess, 'run', return_value=result(0, f'p{own}\nf91\n'.encode())):
            mac._holders(mac.time.monotonic() + 4, 91)
            with self.assertRaises(RuntimeError): mac._holders(mac.time.monotonic() + 4)
        for value in (result(0, b'p1\nf3\n'), result(0, b'p3\n'), result(0, b'bad\n'), result(1, b'', b'failed')):
            with mock.patch.object(mac.subprocess, 'run', return_value=value), self.assertRaises(RuntimeError):
                mac._holders(mac.time.monotonic() + 4, 91)

    def test_passive_boot_capture_preserves_bytes_and_never_writes(self):
        ns = self.ns
        with mock.patch.object(ns['select'], 'select', side_effect=[([91], [], []), ([], [], [])]), \
             mock.patch.object(mac.os, 'read', return_value=b'known boot\r\n'), \
             mock.patch.object(mac.os, 'write', side_effect=AssertionError('passive write')):
            raw, truncated = mac._capture_boot(91, ns, mac.time.monotonic() + 1.5)
        self.assertEqual(raw, b'known boot\r\n')
        self.assertFalse(truncated)
        with mock.patch.object(ns['select'], 'select', return_value=([], [], [])):
            self.assertEqual(mac._capture_boot(91, ns, mac.time.monotonic() + 1.5), (b'', False))
        with mock.patch.object(ns['select'], 'select', return_value=([91], [], [])), \
             mock.patch.object(mac.os, 'read', side_effect=[b'x' * 1024] * 8 + [b'y']):
            raw, truncated = mac._capture_boot(91, ns, mac.time.monotonic() + 1.5)
        self.assertTrue(truncated)
        self.assertEqual(len(raw), 8192)

    def test_passive_requires_battery_off_before_any_io(self):
        with mock.patch.object(mac.os, 'open', side_effect=AssertionError('device open')):
            with self.assertRaises(ValueError):
                mac.run_once(None, None, {}, 'powered-supported', passive_boot=True)

    def test_partial_passive_bytes_survive_later_failure(self):
        evidence = {}
        with mock.patch.object(self.ns['select'], 'select', side_effect=[([91], [], []), ([], [], [91])]), \
             mock.patch.object(mac.os, 'read', return_value=b'partial boot'):
            with self.assertRaises(RuntimeError):
                mac._capture_boot(91, self.ns, mac.time.monotonic() + 1.5, evidence)
        self.assertEqual(bytes.fromhex(evidence['raw_boot_hex']), b'partial boot')

    def test_single_open_and_real_close_on_success_or_crc_failure(self):
        ports = types.SimpleNamespace(comports=lambda: [types.SimpleNamespace(
            device=mac.PORT, vid=0x1a86, pid=0x7523, location='1-1.1', serial_number=None)])
        for failure in (False, True):
            opens, closes = [], []
            class Serial:
                BAUDRATE_CONSTANTS = {}
                def __init__(self, **kwargs):
                    self.is_open, self.fd = False, None
                    self.kwargs = kwargs
                    self.baudrate = kwargs['baudrate']
                def _set_special_baudrate(self, baudrate): pass
                def open(self):
                    opens.append(self.kwargs)
                    self.is_open, self.fd = True, 91
                def fileno(self): return self.fd
                def close(self):
                    closes.append(self.fd)
                    self.is_open, self.fd = False, None
            node = types.SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3,
                st_uid=mac.os.getuid(), st_mode=mac.stat.S_IFCHR | 0o600)
            ns = runtime()
            ns['_modem_bits'] = lambda fd: {'dtr': False, 'rts': False}
            value = ns['_decode_fault_details'](payload(ns, torque=32767))
            with mock.patch.object(mac, '_holders'), \
                 mock.patch.object(mac.os, 'stat', return_value=node), \
                 mock.patch.object(mac.os, 'fstat', return_value=node), \
                 mock.patch.object(mac.fcntl, 'ioctl'), \
                 mock.patch.object(mac.termios, 'tcgetattr', return_value=[0, 0, 0, 0, 0, 0, []]), \
                 mock.patch.object(mac.termios, 'tcsetattr'), \
                 mock.patch.object(mac.signal, 'getsignal'), \
                 mock.patch.object(mac.signal, 'getitimer', return_value=(0., 0.)), \
                 mock.patch.object(mac.signal, 'signal'), mock.patch.object(mac.signal, 'setitimer'), \
                 mock.patch.object(mac, '_query_once', side_effect=RuntimeError('CRC mismatch') if failure else None,
                                   return_value=value):
                report = mac.run_once(types.SimpleNamespace(Serial=Serial), ports, ns, 'battery-off')
            self.assertEqual(len(opens), 1)
            self.assertEqual(opens[0]['baudrate'], 921600)
            self.assertEqual(closes, [91])
            self.assertTrue(report['serial_closed'])
            self.assertEqual(report['query_completed'], not failure)
            self.assertFalse(report['x5_accessed'])
            self.assertFalse(report['servo_motion_authorized'])

    def test_darwin_hupcl_speed_restore_precedes_query_and_standard_baud_skips(self):
        ports = types.SimpleNamespace(comports=lambda: [types.SimpleNamespace(
            device=mac.PORT, vid=0x1a86, pid=0x7523, location='1-1.1', serial_number=None)])
        wire_baud, operations = [None], []
        class Serial:
            BAUDRATE_CONSTANTS = {}
            def __init__(self, **kwargs):
                self.baudrate = kwargs['baudrate']
                self.is_open, self.fd = False, None
            def open(self):
                self.is_open, self.fd = True, 91
                wire_baud[0] = self.baudrate
            def fileno(self): return self.fd
            def _set_special_baudrate(self, baudrate):
                operations.append(('special', baudrate))
                wire_baud[0] = baudrate
            def close(self): self.is_open, self.fd = False, None
        node = types.SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3,
            st_uid=mac.os.getuid(), st_mode=mac.stat.S_IFCHR | 0o600)
        def setattrs(*args):
            operations.append(('tcsetattr', wire_baud[0]))
            if wire_baud[0] == 921600: wire_baud[0] = 38400
        def query(*args):
            operations.append(('query', wire_baud[0]))
            self.assertEqual(wire_baud[0], 921600)
            return {'motion_authorized': False}
        termios = types.SimpleNamespace(HUPCL=1024, TIOCEXCL=123, TCSANOW=0,
            tcgetattr=lambda fd: [0, 0, 0, 0, 0, 0, []], tcsetattr=setattrs)
        ns = runtime()
        ns['_modem_bits'] = lambda fd: {'dtr': False, 'rts': False}
        with mock.patch.object(mac, 'termios', termios), \
             mock.patch.object(mac.sys, 'platform', 'darwin'), \
             mock.patch.object(mac, '_holders'), \
             mock.patch.object(mac.os, 'stat', return_value=node), mock.patch.object(mac.os, 'fstat', return_value=node), \
             mock.patch.object(mac.fcntl, 'ioctl'), \
             mock.patch.object(mac.signal, 'getsignal'), mock.patch.object(mac.signal, 'getitimer', return_value=(0., 0.)), \
             mock.patch.object(mac.signal, 'signal'), mock.patch.object(mac.signal, 'setitimer'), \
             mock.patch.object(mac, '_query_once', side_effect=query), \
             mock.patch.object(mac, '_capture_boot', return_value=(b'', False)):
            report = mac.run_once(types.SimpleNamespace(Serial=Serial), ports, ns, 'battery-off')
            self.assertTrue(report['query_completed'])
            self.assertTrue(report['custom_baud_restored_after_hupcl'])
            self.assertEqual(operations, [('tcsetattr', 921600), ('special', 921600), ('query', 921600)])
            operations.clear()
            report = mac.run_once(types.SimpleNamespace(Serial=Serial), ports, ns, 'battery-off', passive_boot=True)
            self.assertTrue(report['query_completed'])
            self.assertFalse(report['custom_baud_restored_after_hupcl'])
            self.assertEqual(operations, [('tcsetattr', 115200)])

    def test_explicit_restart_clear_pulse_release_readback_no_uart_write(self):
        ns = self.ns
        raw, writes = [6], []
        def modem(fd):
            return {'raw': raw[0], 'dtr': bool(raw[0] & mac.termios.TIOCM_DTR),
                    'rts': bool(raw[0] & mac.termios.TIOCM_RTS)}
        def ioctl(fd, command, packed):
            self.assertEqual(command, mac.termios.TIOCMSET)
            raw[0] = struct.unpack('I', packed)[0]
            writes.append(raw[0])
        ns['_modem_bits'] = modem
        report = {'modem_pulsed': False, 'reset_completed': False}
        with mock.patch.object(mac.fcntl, 'ioctl', side_effect=ioctl), \
             mock.patch.object(mac.time, 'sleep') as pause, \
             mock.patch.object(mac.os, 'write', side_effect=AssertionError('UART write')):
            mac._restart_to_app(91, ns, report)
        self.assertEqual(writes, [0, mac.termios.TIOCM_RTS, 0])
        pause.assert_called_once_with(.10)
        self.assertTrue(report['modem_pulsed'])
        self.assertTrue(report['reset_completed'])
        self.assertEqual([s['stage'] for s in report['reset_steps']],
                         ['clear_dtr_rts', 'assert_rts', 'release_rts'])
        self.assertEqual((raw[0] & mac.termios.TIOCM_DTR, raw[0] & mac.termios.TIOCM_RTS), (0, 0))

    def test_interrupted_reset_releases_once_without_replaying_pulse(self):
        ns = self.ns
        raw, writes = [6], []
        ns['_modem_bits'] = lambda fd: {'raw': raw[0],
            'dtr': bool(raw[0] & mac.termios.TIOCM_DTR), 'rts': bool(raw[0] & mac.termios.TIOCM_RTS)}
        def ioctl(fd, command, packed):
            raw[0] = struct.unpack('I', packed)[0]
            writes.append(raw[0])
        report = {'modem_pulsed': False, 'reset_completed': False}
        with mock.patch.object(mac.fcntl, 'ioctl', side_effect=ioctl), \
             mock.patch.object(mac.time, 'sleep', side_effect=RuntimeError('interrupted')):
            with self.assertRaisesRegex(RuntimeError, 'interrupted'):
                mac._restart_to_app(91, ns, report)
        self.assertEqual(writes, [0, mac.termios.TIOCM_RTS, 0])
        self.assertEqual(report['reset_steps'][-1]['stage'], 'cleanup_release_rts')
        self.assertFalse(report['reset_completed'])

    def test_restart_requires_real_verified_device_report_and_battery_off(self):
        proof = dict(app_sha256=mac.VERIFIED_APP_SHA, mac=mac.VERIFIED_MAC,
                     readback_verified=True, prior_serial_closed=True)
        for phase, value in (('powered-supported', proof), ('battery-off', {})):
            with mock.patch.object(mac.os, 'open', side_effect=AssertionError('opened device')):
                with self.assertRaises(ValueError):
                    mac.restart_boot_capture(None, None, {}, phase, value)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'report.json'
            value = dict(phase='complete', port=mac.PORT, app_sha256=mac.VERIFIED_APP_SHA,
                mac=mac.VERIFIED_MAC, chip='ESP32-S3', readback_verified=True,
                serial_closed=True, verified_region_sha256='f' * 64, expected_region_sha256='f' * 64)
            path.write_text(json.dumps(value))
            path.chmod(0o600)
            with mock.patch.object(mac, 'VERIFIED_FLASH_REPORT', path):
                self.assertEqual(mac._verified_app_report(path)['app_sha256'], mac.VERIFIED_APP_SHA)
                for field, wrong in (('mac', 'wrong'), ('app_sha256', 'wrong'),
                                     ('readback_verified', False), ('serial_closed', False)):
                    path.write_text(json.dumps({**value, field: wrong}))
                    with self.assertRaises(RuntimeError): mac._verified_app_report(path)

    def test_same_open_restart_capture_then_correct_baud_single_query_rejects_line_change(self):
        ports = types.SimpleNamespace(comports=lambda: [types.SimpleNamespace(
            device=mac.PORT, vid=0x1a86, pid=0x7523, location='1-1.1', serial_number=None)])
        proof = dict(app_sha256=mac.VERIFIED_APP_SHA, mac=mac.VERIFIED_MAC,
                     readback_verified=True, prior_serial_closed=True)
        for changed_line in (False, True):
            wire, bits, events, requests = [None], [6], [], []
            class Serial:
                BAUDRATE_CONSTANTS = {}
                def __init__(self, **kwargs):
                    self._baudrate = kwargs['baudrate']
                    self.is_open, self.fd = False, None
                @property
                def baudrate(self): return self._baudrate
                @baudrate.setter
                def baudrate(self, value):
                    self._baudrate = value
                    wire[0] = value
                    events.append(('baud', value))
                def open(self):
                    self.is_open, self.fd = True, 91
                    wire[0] = self.baudrate
                    events.append(('open', wire[0]))
                def fileno(self): return self.fd
                def _set_special_baudrate(self, value):
                    wire[0] = value
                    events.append(('special', value))
                    if changed_line: bits[0] = 2
                def close(self):
                    events.append(('close', self.fd))
                    self.is_open, self.fd = False, None
            node = types.SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3,
                st_uid=mac.os.getuid(), st_mode=mac.stat.S_IFCHR | 0o600)
            def ioctl(fd, command, packed=None):
                if command == 999:
                    bits[0] = struct.unpack('I', packed)[0]
                    events.append(('modem', bits[0]))
            def attrs(*args):
                events.append(('tcsetattr', wire[0]))
                if wire[0] == 921600: wire[0] = 38400
            def capture(*args):
                self.assertEqual(wire[0], 115200)
                events.append(('capture', 115200))
                return b'SPI_FAST_FLASH_BOOT entry\r\n', False
            ns = runtime()
            ns['_modem_bits'] = lambda fd: dict(raw=bits[0], dtr=bool(bits[0] & 2), rts=bool(bits[0] & 4))
            def hello(fd, kind, sequence, deadline, metadata):
                self.assertEqual((fd, wire[0], bits[0]), (91, 921600, 0))
                self.assertIn(('capture', 115200), events)
                requests.append((kind, sequence))
                metadata['request_types_attempted'].append(kind)
                metadata['reply_types_received'].append(0x81)
                metadata['request_bytes_written'] += 12
                return b'XGODUCK_MOTION_1'
            def diag(fd, sequence, deadline, metadata):
                self.assertEqual((fd, wire[0], bits[0]), (91, 921600, 0))
                requests.append((7, sequence))
                metadata['request_types_attempted'].append(7)
                metadata['reply_types_received'].append(0x87)
                metadata['request_bytes_written'] += 12
                return payload(ns, online=0, fault=32767, torque_faults=32767)
            ns['_exchange'], ns['_fault_exchange'] = hello, diag
            termios = types.SimpleNamespace(HUPCL=1024, TIOCEXCL=123, TCSANOW=0,
                TIOCM_DTR=2, TIOCM_RTS=4, TIOCMSET=999,
                tcgetattr=lambda fd: [0, 0, 0, 0, 0, 0, []], tcsetattr=attrs)
            with mock.patch.object(mac, 'termios', termios), \
                 mock.patch.object(mac.sys, 'platform', 'darwin'), mock.patch.object(mac, '_holders'), \
                 mock.patch.object(mac.os, 'stat', return_value=node), mock.patch.object(mac.os, 'fstat', return_value=node), \
                 mock.patch.object(mac.fcntl, 'ioctl', side_effect=ioctl), mock.patch.object(mac.time, 'sleep'), \
                 mock.patch.object(mac.signal, 'getsignal'), mock.patch.object(mac.signal, 'getitimer', return_value=(0., 0.)), \
                 mock.patch.object(mac.signal, 'signal'), mock.patch.object(mac.signal, 'setitimer'), \
                 mock.patch.object(mac, '_capture_boot', side_effect=capture):
                report = mac.restart_boot_capture(types.SimpleNamespace(Serial=Serial), ports, ns,
                    'battery-off', proof, query_after_restart=True)
            self.assertEqual([item for item in events if item[0] == 'open'], [('open', 115200)])
            self.assertEqual([item for item in events if item[0] == 'close'], [('close', 91)])
            self.assertEqual([item for item in events if item[0] == 'modem'], [('modem', 0), ('modem', 4), ('modem', 0)])
            self.assertLess(events.index(('capture', 115200)), events.index(('baud', 921600)))
            self.assertEqual(requests, [] if changed_line else [(1, 1), (7, 2)])
            self.assertEqual(report['query_completed'], not changed_line)
            self.assertEqual(report['application_protocol_responded'], not changed_line)
            self.assertTrue(report['serial_closed'])
            self.assertFalse(report['application_ready_proved'])
            if not changed_line:
                self.assertEqual(report['diagnostic']['fault_mask'], 32767)
                self.assertEqual(report['request_bytes_written'], 24)
                self.assertFalse(report['diagnostic']['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
