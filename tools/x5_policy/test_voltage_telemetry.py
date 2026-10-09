"""Hardware-free live voltage protocol, compatibility and source-age tests."""
import struct
import unittest

import motion
from test_motion import FakeTransport
import test_motion as fixtures


def extension(raw=71, seen=motion.ALL_MASK, valid=motion.ALL_MASK, age=200000):
    return motion.VOLTAGE_EXTENSION.pack(1, seen, valid,
        *[raw if seen & (1 << i) else 0 for i in range(15)],
        *[age if seen & (1 << i) else 0xffffffff for i in range(15)])


class VoltageTransport(FakeTransport):
    voltage = extension()
    def negotiate_voltage(self):
        self.requests.append((motion.TELEMETRY_CAPS, b''))
        return True
    def request(self, kind, payload=b''):
        if kind == motion.VOLTAGE_STATE:
            base = super().request(motion.STATE)
            self.requests[-1] = (kind, payload)
            return base + self.voltage
        return super().request(kind, payload)


class VoltageProtocolTests(unittest.TestCase):
    def base(self):
        return FakeTransport('fake').request(motion.STATE)

    def test_legacy_106_bytes_and_versioned_186_bytes(self):
        base = self.base()
        self.assertEqual(len(base), 106)
        self.assertIsNone(motion.decode_state(base)['voltage'])
        extended = base + extension()
        self.assertEqual(len(extended), 186)
        decoded = motion.decode_state(extended)
        self.assertEqual(decoded['voltage']['raw'], [71]*15)
        self.assertEqual(decoded['voltage']['ages_us'], [200000]*15)
        for key, value in motion.decode_state(base).items():
            if key != 'voltage': self.assertEqual(decoded[key], value)

    def test_rejects_bad_masks_versions_lengths_and_impossible_ages(self):
        good = bytearray(self.base() + extension())
        bads = [good[:-1], good+b'\x00']
        for offset, value in [(106,2),(107,0),(110,0x80),(111,0)]:
            bad = good.copy();bad[offset] = value;bads.append(bad)
        for age in (1000001,0xffffffff):
            bad = good.copy();struct.pack_into('<I',bad,126,age);bads.append(bad)
        for bad in bads:
            with self.subTest(payload=bytes(bad)[106:113]), self.assertRaises(motion.MotionError):
                motion.decode_state(bad)
        unknown = motion.decode_state(self.base()+extension(seen=0,valid=0))['voltage']
        self.assertEqual(unknown['raw'], [0]*15)
        stale = motion.decode_state(self.base()+extension(valid=0,age=2000000))['voltage']
        self.assertEqual(stale['raw'], [71]*15)

    def stream(self, payload, *, response_kind=motion.TELEMETRY_CAPS_REPLY):
        class Stream:
            in_waiting = 0
            def __init__(self): self.writes=[]
            def write(self, data):
                _,_,kind,_,seq = motion.HEADER.unpack_from(data);self.writes.append(kind)
                value = bytes([kind,0]) if kind == motion.STOP else payload
                self.rx = motion.encode_frame(motion.ACK if kind == motion.STOP else response_kind,seq,value)
                self.in_waiting = len(self.rx);return len(data)
            def read(self, size):
                value,self.rx = self.rx,b'';self.in_waiting=0;return value
        transport=motion.SerialTransport('fake');transport.serial=Stream();return transport

    def test_capability_exact_legacy_refusal_only_and_no_motor_commands(self):
        supported=self.stream(b'XGODUCK_TELEMETRY_1')
        self.assertTrue(supported.negotiate_voltage())
        self.assertEqual(supported.serial.writes,[20])
        old=self.stream(bytes([20,1]),response_kind=motion.ACK)
        self.assertFalse(old.negotiate_voltage())
        self.assertEqual(old.serial.writes,[20,motion.STOP])
        for payload,kind in [(b'XGODUCK_TELEMETRY_2',0x94),(bytes([20,2]),motion.ACK),
                             (bytes([20,0]),motion.ACK),(b'',0x94)]:
            transport=self.stream(payload,response_kind=kind)
            if payload==b'' and kind==0x94:
                # A successful empty capability is malformed, not an old ACK.
                with self.assertRaises(motion.MotionError): transport.negotiate_voltage()
            else:
                with self.assertRaises(motion.MotionError): transport.negotiate_voltage()
            self.assertEqual(transport.serial.writes,[20])


class VoltageControllerTests(unittest.TestCase):
    def setUp(self):
        self.helper=fixtures.ControllerTests();self.helper.setUp()
        self.addCleanup(self.helper.tearDown)
        self.controller=self.helper.controller(ready=True)
        self.controller._factory=VoltageTransport
        self.controller._connect_once()
        self.transport=self.controller._transport

    def test_negotiated_stream_reports_voltage_without_arm_or_target(self):
        s=self.controller.status()
        self.assertTrue(s['voltage_supported'])
        self.assertEqual(s['voltage_range_v'],[4.,8.4])
        self.assertEqual([v['volts'] for v in s['motor_voltages']],[7.1]*15)
        self.assertTrue(all(v['fresh'] and v['condition']=='normal' for v in s['motor_voltages']))
        self.assertNotIn(motion.STATE,[k for k,_ in self.transport.requests])
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in self.transport.requests))

    def test_low_high_and_loss_preserve_actual_reading_and_age(self):
        for raw,condition in [(39,'low'),(85,'high'),(70,'normal')]:
            self.transport.voltage=extension(raw=raw)
            self.controller._cycle()
            row=self.controller.status()['motor_voltages'][0]
            self.assertEqual(row['volts'],raw/10)
            self.assertEqual(row['condition'],condition)
        self.helper.clock.advance(2)
        old=self.controller.status()['motor_voltages'][0]
        self.assertEqual(old['volts'],7.0)
        self.assertFalse(old['fresh']);self.assertEqual(old['condition'],'unknown')
        self.assertGreaterEqual(old['age_ms'],2200)
        self.transport.voltage=extension(raw=70,valid=0,age=2300000)
        self.controller._cycle();row=self.controller.status()['motor_voltages'][0]
        self.assertEqual(row['volts'],7.0);self.assertFalse(row['fresh'])
        self.transport.voltage=extension(seen=0,valid=0)
        self.controller._cycle();row=self.controller.status()['motor_voltages'][0]
        self.assertIsNone(row['volts']);self.assertIsNone(row['age_ms'])

    def test_voltage_does_not_refresh_frozen_position_or_safety_gate(self):
        self.transport.freeze_sample=True
        self.helper.clock.advance(.2);self.controller._cycle()
        self.assertTrue(self.controller.status()['motor_voltages'][0]['fresh'])
        with self.assertRaisesRegex(motion.MotionError,'反馈已过期'):
            self.controller.start_mode('hold',support_confirmed=True)
        self.assertNotIn(motion.ARM,[k for k,_ in self.transport.requests])

    def test_negotiated_short_reply_is_not_accepted_as_legacy(self):
        self.transport.voltage=b''
        with self.assertRaisesRegex(motion.MotionError,'协商后的'):
            self.controller._read_state()


if __name__=='__main__': unittest.main()
