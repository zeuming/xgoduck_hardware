import argparse
import ast
import json
from pathlib import Path
import struct
import unittest

from arm_read_trace import decode_arm_read_trace, arm_read_trace_text, ARM_TRACE_SIZE


def records(path):
    data=path.read_bytes(); offset=0; rows=[]
    while offset<len(data):
        n=struct.unpack_from('<H',data,offset)[0]; offset+=2
        payload=data[offset:offset+n]; offset+=n
        count=struct.unpack_from('<I',data,offset)[0]; offset+=4
        bus=data[offset:offset+count]; offset+=count
        rows.append((payload,bus))
    assert offset==len(data) and len(rows)==11
    return rows


class Tests(unittest.TestCase):
    def test_wire_parity_and_strict_decode(self):
        for i, ((old,old_bus),(new,new_bus)) in enumerate(zip(BASELINE,CANDIDATE)):
            self.assertEqual(old_bus,new_bus, 'Changed servo transactions in case '+str(i))
            self.assertEqual(old[:2],new[:2])
            self.assertEqual(old[3:14],new[3:14])
            if i>=9:
                self.assertEqual(old,new);continue
            d=decode_arm_read_trace(new,sequence=2,cookie=123)
            self.assertEqual((d['id'],d['address'],d['stage']),(10,40,9))
            self.assertEqual(d['drained_count'],0)
            self.assertGreater(d['enable_to_request_us'],0)
            self.assertTrue(d['request_written'])
            self.assertIn('RX=',arm_read_trace_text(d))
            DETAILS.append(d)

    def test_distinct_failure_evidence(self):
        ds=[decode_arm_read_trace(row[0]) for row in CANDIDATE[:9]]
        self.assertEqual([ds[i]['received_count'] for i in (0,1,2,3,4,5,6,7,8)],
                         [0,8,15,15,0,0,3,0,70])
        self.assertEqual(ds[5]['pending_count'],15)
        self.assertGreater(ds[5]['elapsed_us'],1500)
        self.assertEqual(ds[4]['uart_errors_after']-ds[4]['uart_errors_before'],1)
        self.assertTrue(ds[2]['invalid_reply_seen'])
        self.assertTrue(ds[3]['invalid_reply_seen'])
        self.assertFalse(ds[1]['invalid_reply_seen'])
        self.assertTrue(ds[8]['received_truncated'])
        self.assertEqual(len(bytes.fromhex(ds[8]['received_hex'])),48)
        # A reply arriving after the deadline cannot be distinguished from
        # silence by evidence captured at exit; never pretend otherwise.
        self.assertEqual(ds[7]['pending_count'],0)

    def test_malformed_and_wrong_attempt_are_rejected(self):
        p=CANDIDATE[0][0]
        for n,value in [(2,1),(3,3),(4,15),(5,56),(22,128),(23,6)]:
            q=bytearray(p); q[n]=value
            with self.assertRaises(ValueError):decode_arm_read_trace(q)
        for q in [p[:-1],p+b'\0']:
            with self.assertRaises(ValueError):decode_arm_read_trace(q)
        with self.assertRaises(ValueError):decode_arm_read_trace(p,sequence=3)
        with self.assertRaises(ValueError):decode_arm_read_trace(p,cookie=124)

    def test_actual_host_decoder_and_transport(self):
        source=(HOST/'production_motion.py').read_text();tree=ast.parse(source)
        names=['MotionError','FrameParser','SerialTransport','crc16','encode_frame',
               'arm_failure_detail','target_stop_detail']
        body=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
        ns=dict(struct=struct,collections=__import__('collections'),time=__import__('time'),os=__import__('os'),
            binascii=__import__('binascii'),copy=__import__('copy'),
            decode_arm_read_trace=decode_arm_read_trace,arm_read_trace_text=arm_read_trace_text,ARM_TRACE_SIZE=ARM_TRACE_SIZE)
        # Reuse the real module's literal protocol constants without importing
        # policy workers, ONNX or opening any devices.
        for n in tree.body:
            if isinstance(n,(ast.Assign,ast.AnnAssign)):
                try:exec(compile(ast.Module(body=[n],type_ignores=[]),'constants','exec'),ns)
                except (NameError,TypeError):pass
        exec(compile(ast.Module(body=body,type_ignores=[]),'host','exec'),ns)
        for payload in [BASELINE[0][0],CANDIDATE[0][0],CANDIDATE[5][0]]:
            expected=ns['arm_failure_detail'](payload)
            self.assertIn('使能后扭矩检查',expected)
            transport=ns['SerialTransport']('fake')
            class Stream:
                rx=b''
                @property
                def in_waiting(self):return len(self.rx)
                def write(self,data):
                    _,_,_,_,seq=ns['HEADER'].unpack_from(data)
                    self.rx=ns['encode_frame'](ns['ACK'],seq,payload)
                    return len(data)
                def read(self,size):
                    data,self.rx=self.rx[:size],self.rx[size:];return data
            transport.serial=Stream();transport.seq=1
            with self.assertRaises(ns['MotionError']) as caught:
                transport.request(4,struct.pack('<I',123))
            self.assertIn(expected,str(caught.exception))
            self.assertEqual(transport.seq,2)
            if len(payload)==ARM_TRACE_SIZE:self.assertEqual(transport.last_arm_read_trace['arm_sequence'],2)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path)
    p.add_argument('host',type=Path);p.add_argument('result',type=Path);a=p.parse_args()
    BASELINE=records(a.baseline);CANDIDATE=records(a.candidate);HOST=a.host;DETAILS=[]
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Tests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    a.result.write_text(json.dumps(dict(passed=result.wasSuccessful(),scenario_count=11,
        servo_bus_packets_identical=True if result.wasSuccessful() else None,
        root_cause_proven_on_hardware=False,details=DETAILS),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
