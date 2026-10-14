import struct,unittest
import read_retry_diagnostics as d
class WireTests(unittest.TestCase):
    def test_motion_types_and_payloads_cannot_be_requested(self):
        for kind in (1,3,4,5,6,19,True,'26'):
            with self.assertRaises(ValueError):d.request(kind,1)
        for kind in (2,7,26):
            raw=d.request(kind,55);self.assertEqual(len(raw),12)
            self.assertEqual(d.HEADER.unpack(raw[:10]),(b'XD',1,kind,0,55))
        self.assertEqual(d.crc16(b'123456789'),0x29b1)
    def test_strict_frame_exchange_and_no_retry(self):
        class Port:
            in_waiting=0
            def __init__(self,raw):self.raw=raw;self.sent=[]
            def write(self,data):self.sent.append(data);return len(data)
            def read(self,n):result=self.raw[:min(n,3)];self.raw=self.raw[len(result):];return result
        payload=struct.pack('<4B6I',1,30,6,1,2,1,1,123,1514,150)
        head=d.HEADER.pack(b'XD',1,0x9a,28,1);wire=head+payload;wire+=struct.pack('<H',d.crc16(wire))
        port=Port(wire);self.assertEqual(d.exchange(port,26,1)[0],payload);self.assertEqual(len(port.sent),1)
        for raw in (wire[:-1]+bytes([wire[-1]^1]),bytes([wire[0]^1])+wire[1:]):
            port=Port(raw)
            with self.assertRaises(ValueError):d.exchange(port,26,1)
            self.assertEqual(len(port.sent),1)
        self.assertEqual(d.retry_details(payload)['recovered'],1)
    def test_state_layout_and_retry_validation(self):
        raw=bytearray(106);struct.pack_into('<IIHHHBB',raw,0,11,12,32767,0,0,0,1)
        struct.pack_into('<I',raw,100,123);raw[104]=4;raw[105]=1
        s=d.state(raw);self.assertEqual((s['arm_cookie'],s['last_error'],s['configured']),(123,4,True))
        self.assertEqual(s['online_mask'],32767);self.assertEqual(len(s['positions']),15)
        for raw in (b'',bytes(28),struct.pack('<4B6I',1,99,0,0,0,0,0,0,0,0)):
            with self.assertRaises(ValueError):d.retry_details(raw)
if __name__=='__main__':unittest.main()
