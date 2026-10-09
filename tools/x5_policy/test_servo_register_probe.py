"""Mock transport failure tests; no hardware/network/device opens."""
import os
from pathlib import Path
import struct
import sys
import termios
import types
import unittest
from unittest import mock

import servo_register_probe as probe


def wire_packet(sid, tick):
    raw = bytearray(202)
    ids = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
    raw[:4] = bytes([1, sid, ids.index(sid), 8])
    struct.pack_into('<HH', raw, 4, 2000, 2000)
    struct.pack_into('<4I', raw, 18, tick, tick, tick, tick)
    struct.pack_into('<4H', raw, 34, 0x7fff, 0, 0x7fff, 0)
    for index, (address, size) in enumerate(((0,6),(16,2),(33,1),(40,1),(42,6),(48,4),(62,2),(69,2))):
        offset = 42 + 20 * index
        raw[offset:offset+2] = bytes([address, size])
        struct.pack_into('<II', raw, offset+2, tick+index*1500, tick+index*1500+1000)
        raw[offset+10:offset+14] = bytes([1, 0, size, 5])
        raw[offset+14:offset+14+size] = bytes([0xff]) * size
    return bytes(raw)


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.now, self.tick = 100., 1000000
        self.calls, self.diag_times = [], []
        self.ns = dict(bind_serial=lambda expected: None,
            serial_proof=lambda expected, fd=None: (types.SimpleNamespace(st_rdev=12), {}),
            serial_path=lambda expected: '/dev/ttyUSB0', _holders=lambda *args: [],
            _modem_bits=lambda fd: dict(dtr=False, rts=False),
            _raw_settings=lambda fd: dict(baud=921600, hupcl=False),
            _decode_state=lambda data: data, _hello_feedback=lambda fd, metadata: self.calls.append((1, 1, None)),
            _exchange=self.exchange)
        self.guard = mock.Mock(return_value=dict(boot_id='same'))
    def state(self):
        self.tick += 20000
        return dict(sample_us=self.tick, imu_us=self.tick, online_mask=0x7fff, fault_mask=0,
                    torque_mask=0, armed=False, arm_cookie=0, last_error=0, imu_ok=True,
                    accel_ms2=[0,0,9.8], gyro_dps=[0,0,0])
    def exchange(self, fd, kind, sequence, deadline, metadata, **options):
        self.calls.append((kind, sequence, options.get('payload')))
        self.assertLessEqual(deadline-self.now, .400001)
        if kind == 23:
            sid = options['payload'][0]
            self.diag_times.append((sid, self.now))
            self.now += .35  # A slow reply must not defeat the MCU 500ms quota.
            return wire_packet(sid, self.tick)
        self.assertEqual(kind, 2)
        self.assertEqual(options, {})
        return self.state()
    def sleep(self, seconds):
        self.now += seconds
    def execute(self, *, hupcl=False, close_error=None, probe_ids=probe.PROBE_IDS):
        attrs = [0,0,termios.HUPCL if hupcl else 0,0,0,0,[]]
        with (mock.patch.object(probe.os, 'open', return_value=91) as opened,
              mock.patch.object(probe.os, 'close', side_effect=close_error) as closed,
              mock.patch('fcntl.flock'), mock.patch('fcntl.ioctl'),
              mock.patch('termios.tcgetattr', return_value=attrs)):
            result = probe.run_probe(self.ns, {}, self.guard, clock=lambda:self.now, sleeper=self.sleep,
                                     probe_ids=probe_ids)
        return result, opened, closed
    def test_four_distinct_single_reads_and_real_close_without_control_writes(self):
        result, opened, closed = self.execute()
        self.assertTrue(result['success'], result)
        self.assertTrue(result['serial_closed'])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['correlated_with_previous_hold'])
        self.assertEqual(result['diagnostic_attempts'], 4)
        self.assertEqual([sid for sid,_ in self.diag_times], [11,12,21,22])
        self.assertTrue(all(b[1]-a[1] >= .89999 for a,b in zip(self.diag_times,self.diag_times[1:])))
        self.assertEqual(set(k for k,_,_ in self.calls), {1,2,23})
        opened.assert_called_once()
        closed.assert_called_once_with(91)
    def test_hupcl_owner_and_modem_changes_refuse_before_protocol(self):
        result, _, closed = self.execute(hupcl=True)
        self.assertFalse(result['success'])
        self.assertEqual(self.calls, [])
        closed.assert_called_once()
        self.ns['_holders'] = lambda *args:[dict(pid=123,fd='4')]
        result, opened, closed = self.execute()
        self.assertFalse(result['success'])
        opened.assert_not_called()
        closed.assert_not_called()
        self.ns['_holders'] = lambda *args:[]
        self.ns['_modem_bits'] = mock.Mock(side_effect=[dict(dtr=False),dict(dtr=True)])
        result, *_ = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(self.calls, [])
    def test_first_timeout_and_second_read_failure_stop_without_retry(self):
        original = self.ns['_exchange']
        def fail(fd, kind, *args, **kwargs):
            if kind == 23:
                self.calls.append((kind,args[0],kwargs.get('payload')))
                raise TimeoutError('single diagnostic failed')
            return original(fd,kind,*args,**kwargs)
        self.ns['_exchange'] = fail
        result, _, closed = self.execute()
        self.assertEqual(result['diagnostic_attempts'], 1)
        self.assertEqual(result['error_type'], 'TimeoutError')
        closed.assert_called_once()
        self.calls.clear()
        def invalid(fd, kind, *args, **kwargs):
            raw = original(fd,kind,*args,**kwargs)
            if kind == 23 and kwargs['payload'] == bytes([12]):
                data = bytearray(raw)
                data[52:56] = bytes([6,0,0,0])
                data[56:62] = bytes(6)
                return bytes(data)
            return raw
        self.ns['_exchange'] = invalid
        result, _, closed = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['diagnostic_attempts'], 2)
        self.assertEqual([d['id'] for d in result['diagnostics']], [11,12])
        self.assertIn('raw_payload_hex',result['diagnostics'][1])
        closed.assert_called_once()
    def test_unsafe_post_state_cannot_proceed_to_next_id(self):
        original = self.ns['_decode_state']
        self.ns['_decode_state'] = lambda raw: dict(original(raw), torque_mask=1 if self.diag_times else 0)
        result, _, closed = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['diagnostic_attempts'],1)
        closed.assert_called_once()
    def test_readiness_deadline_and_close_failure_do_not_claim_success(self):
        self.ns['_decode_state'] = lambda raw:dict(raw,online_mask=0)
        result, _, closed = self.execute()
        self.assertFalse(result['success'])
        self.assertEqual(result['diagnostic_attempts'],0)
        self.assertLessEqual(result['elapsed_seconds'],2.000001)
        self.ns['_decode_state'] = lambda raw:raw
        result, _, closed = self.execute(close_error=OSError('close failed'))
        self.assertFalse(result['success'])
        self.assertFalse(result['serial_closed'])
        self.assertEqual(result['close_error'],'close failed')
    def test_allowlist_sequence_crc_and_unmodified_original_helper(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'x5_recovery'))
        import feedback_stream
        source, namespace = feedback_stream.runtime_source(), {}
        exec(source, namespace)
        probe.adapt_helpers(source,namespace)
        for kind,payload in ((3,b''),(22,b'\x0b'),(23,b'\x0a'),(23,b''),(1,b'\x0b')):
            with self.assertRaises(namespace['ProbeError']):namespace['_frame'](kind,5,payload)
        for change in ('none','sequence','type','crc'):
            raw = wire_packet(11,1000000)
            header=namespace['HEADER'].pack(b'XD',1,0x96 if change=='type' else 0x97,202,4 if change=='sequence' else 5)
            tail=raw+struct.pack('<H',namespace['_crc16'](header+raw)^(1 if change=='crc' else 0))
            namespace['_read_exact']=mock.Mock(side_effect=[header,tail])
            namespace['_write_once']=mock.Mock()
            with mock.patch.object(namespace['select'],'select',return_value=([],[],[])):
                call=lambda:namespace['_exchange'](91,23,5,probe.time.monotonic()+.4,
                    dict(request_types_attempted=[],reply_types_received=[],request_bytes_written=0),payload=b'\x0b')
                if change=='none':self.assertEqual(call(),raw)
                else:
                    with self.assertRaises(namespace['ProbeError']):call()

    def test_explicit_head_ids_are_the_only_diagnostic_payloads_and_close_once(self):
        result, opened, closed = self.execute(probe_ids=(30,31,32,33))
        self.assertTrue(result['success'], result)
        self.assertEqual([sid for sid,_ in self.diag_times], [30,31,32,33])
        self.assertEqual(result['ids'], [30,31,32,33])
        self.assertEqual(set(kind for kind,_,_ in self.calls), {1,2,23})
        opened.assert_called_once();closed.assert_called_once_with(91)
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'x5_recovery'))
        import feedback_stream
        source, namespace = feedback_stream.runtime_source(), {}
        exec(source, namespace)
        probe.adapt_helpers(source, namespace, probe_ids=(30,31,32,33))
        for sid in (30,31,32,33):namespace['_frame'](23,5,bytes([sid]))
        for kind,payload in ((23,b'\x0b'),(23,b'\x22'),(4,b''),(5,b'\x21'),(1,b'\x21')):
            with self.assertRaises(namespace['ProbeError']):namespace['_frame'](kind,5,payload)
        for ids in ((),(33,33),(True,),(35,),[33],(10,11,12,13,14)):
            with self.assertRaises(ValueError):probe.checked_probe_ids(ids)


if __name__ == '__main__':
    unittest.main()
