"""Focused passive-calibration regressions; no network, UART or motor commands."""
from pathlib import Path
import os
import struct
import sys
import time
import types
import unittest

ROOT=Path(__file__).resolve().parents[2]
stage=os.environ.get('XGO_JOINT_CAPTURE_STAGE')
if stage:
    sys.path[:0]=[str(Path(stage)/'native/service'), str(Path(stage).parent/'before/service')]
sys.path.extend([str(ROOT/'tools/x5_recovery'),str(ROOT/'tools/source_parity')])
import native_transports
import feedback_stream
import native_calibration
import test_native_calibration as fixtures


class JointCaptureRegression(unittest.TestCase):
    def setUp(self):
        self.h=fixtures.NativeCalibrationTests('runTest')
        self.h.setUp()
        self.h.start('joints')
        self.s=self.h.session

    def tearDown(self):
        self.h.tearDown()

    def test_large_manual_swing_keeps_direction_and_captures_both_endpoints(self):
        for action,raw in [('direction-start',2000),('direction-end',2050),
                           ('range-a',100),('range-b',3900)]:
            self.h.capture_joint(action,raw,sid=22)
        w=self.s.joint_wizard.value
        self.assertEqual((w['range_a'],w['range_b']),(100,3900))
        self.assertTrue(w['ready_to_save'])
        self.assertEqual(self.h.commits,[])

    def test_unreadable_selected_position_preserves_steps_and_recovers(self):
        for action,raw in [('direction-start',2000),('direction-end',2050),('range-a',100)]:
            self.h.capture_joint(action,raw,sid=22)
        self.h.feed(online_mask=32767 ^ (1<<7),torque_known_mask=32767)
        w=self.s.joint_wizard.value
        self.assertEqual(self.s.status,'active')
        self.assertEqual((w['stage'],w['range_a'],w['inferred_sign']),('await-range-b',100,1))
        self.assertIsNone(w['error'])
        with self.assertRaisesRegex(ValueError,'暂时不可读'):
            self.s.joint_action(self.h.joint_body('range-b',22))
        self.h.positions[7]=3900
        self.h.feed(online_mask=32767,torque_known_mask=32767)
        self.h.capture_joint('range-b',3900,sid=22)
        self.assertTrue(self.s.joint_wizard.value['ready_to_save'])
        self.assertEqual(self.h.commits,[])

    def test_other_unreadable_joint_does_not_block_selected_joint(self):
        self.h.feed(online_mask=32767 ^ (1<<7),torque_known_mask=32767)
        self.s.joint_action(self.h.joint_body('direction-start',10))
        for _ in range(20):self.h.feed(online_mask=32767 ^ (1<<7),torque_known_mask=32767)
        self.assertEqual(self.s.joint_wizard.value['start_position'],2000)
        self.assertEqual(self.s.feedback['online_mask'],32767 ^ (1<<7))

    def test_source_state_reader_preserves_real_mask_and_independent_torque(self):
        ns={};exec(native_transports.joint_collection_source(feedback_stream.runtime_source()),ns)
        from test_feedback_stream import payload as state_payload
        # Production decoder receives the unchanged legacy prefix from Type32.
        payload=state_payload(ns,online=32767 ^ (1<<7))
        packet=payload+struct.pack('<HBB15I',32767,1,1,*([0]*15))
        calls=[]
        ns['_joint_exchange']=lambda fd,kind,seq,deadline,meta:(calls.append(kind) or packet)
        actual=ns['_exchange'](91,2,2,1.,{})
        state=ns['_decode_state'](actual)
        self.assertEqual(calls,[32])
        self.assertEqual(state['online_mask'],32767 ^ (1<<7))
        self.assertEqual(state['torque_known_mask'],32767)
        self.assertTrue(ns['StateTracker']().accept(state,20.))
        self.assertEqual(state['online_mask'],32767 ^ (1<<7))

    def test_real_faults_still_stop_reader(self):
        ns={};exec(native_transports.joint_collection_source(feedback_stream.runtime_source()),ns)
        state=self.h.state();state.update(online_mask=32639,torque_known_mask=32767)
        for changes in ({'torque_mask':1},{'fault_mask':1},
                        {'armed':True},{'arm_cookie':1},{'last_error':1},{'imu_ok':False}):
            with self.subTest(changes=changes),self.assertRaises(ns['ProbeError']):
                ns['StateTracker']().accept(dict(state,**changes),20.)

    def test_missing_torque_pauses_capture_without_clearing_steps(self):
        for action,raw in [('direction-start',2000),('direction-end',2050),('range-a',100)]:
            self.h.capture_joint(action,raw,sid=22)
        self.h.feed(online_mask=32767,torque_known_mask=32639)
        self.assertEqual(self.s.status,'active')
        self.assertEqual(self.s.joint_wizard.value['range_a'],100)
        with self.assertRaisesRegex(ValueError,'等待实际扭矩'):
            self.s.joint_action(self.h.joint_body('range-b',22))
        self.h.feed(online_mask=32767,torque_known_mask=32767)
        self.h.capture_joint('range-b',3900,sid=22)
        self.assertTrue(self.s.joint_wizard.value['ready_to_save'])

    def test_feedback_interface_is_initialized_without_motor_commands(self):
        ns={};exec(native_transports.joint_collection_source(feedback_stream.runtime_source()),ns)
        calls=[]
        ns['_joint_hello']=lambda fd,meta:calls.append(1)
        ns['_joint_exchange']=lambda fd,kind,seq,deadline,meta:(calls.append(kind) or bytes((30,0)))
        ns['_hello_feedback'](91,{})
        self.assertEqual(calls,[1,30])
        ns['_joint_exchange']=lambda *args:bytes((30,2))
        with self.assertRaises(ns['ProbeError']):ns['_hello_feedback'](91,{})

    def test_interface_initialization_decodes_actual_firmware_ack_frame(self):
        ns={};exec(native_transports.joint_collection_source(feedback_stream.runtime_source()),ns)
        header=ns['HEADER'].pack(b'XD',1,0x80,2,2)
        data=bytes((30,0));tail=data+struct.pack('<H',ns['_crc16'](header+data))
        reads=iter((header,tail));writes=[]
        ns['select']=types.SimpleNamespace(select=lambda *args:([],[],[]))
        ns['_read_exact']=lambda *args:next(reads)
        ns['_write_once']=lambda fd,raw,deadline,metadata:writes.append(raw)
        meta=dict(request_types_attempted=[],reply_types_received=[])
        self.assertEqual(ns['_joint_exchange'](91,30,2,time.monotonic()+1,meta),data)
        self.assertEqual(writes[0][3],30)
        self.assertEqual(meta['reply_types_received'],[0x80])


if __name__=='__main__':unittest.main()
