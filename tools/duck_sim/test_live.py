import binascii
import json
from pathlib import Path
import struct
import sys
import time
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'x5_policy'))
from live import LiveFeedback
from live_reader import IDS, PACKET, ReadPort, read_state, state_request
from live_reader import calibration_stamp, require_same_calibration


class FakeProcess:
    def poll(self):return None


class LiveTests(unittest.TestCase):
    def test_atomic_calibration_replace_ends_old_mapping_even_with_same_size(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'calibration.json';path.write_text('old')
            with patch('live_reader.CALIBRATION_PATH', path):
                stamp=calibration_stamp();require_same_calibration(stamp)
                new=path.with_suffix('.new');new.write_text('new');new.replace(path)
                with self.assertRaisesRegex(RuntimeError,'保存校准已更新'):
                    require_same_calibration(stamp)

    def test_wire_gate_allows_only_empty_state(self):
        port=ReadPort.__new__(ReadPort);port.fd=456
        with patch('live_reader.os.write',return_value=12) as write:
            self.assertEqual(port.write(state_request(1)),12)
            for kind in (1,3,4,5,6,9,12):
                raw=struct.pack('<2sBBHI',b'XD',1,kind,0,2)
                raw+=struct.pack('<H',binascii.crc_hqx(raw,0xffff))
                with self.assertRaises(RuntimeError):port.write(raw)
            with self.assertRaises(RuntimeError):port.write(state_request(2)[:-1]+b'\x00')
            self.assertEqual(write.call_count,1)

    def test_protocol_crc_sequence_and_joint_decode(self):
        payload=PACKET.pack(10000,10100,0x7fff,0,0,0,1,*([2048]*15),*([0]*15),0,0,9.8,0,0,0,0,0,1)
        def response(seq):
            frame=struct.pack('<2sBBHI',b'XD',1,0x82,len(payload),seq)+payload
            return frame+struct.pack('<H',binascii.crc_hqx(frame,0xffff))
        frames=[response(7),response(8)[:-1]+b'\x00',response(8)]
        class Stream:
            def write(self,data):self.sent=data;return len(data)
            def read(self,n):return frames.pop(0) if frames else b''
        stream=Stream();row=read_state(stream,8)
        self.assertEqual(stream.sent,state_request(8))
        self.assertEqual(row['positions'],[2048]*15)
        self.assertFalse(row['armed'])
        self.assertEqual(row['torque_mask'],0)

    def configured(self):
        f=LiveFeedback();f.process=FakeProcess()
        f.cfg={'joints':[{'id':sid,'zero':2048,'sign':-1 if i%2 else 1} for i,sid in enumerate(IDS)],
               'imu':{'axes':['x','y','z'],'bias_dps':[0,0,0],'axes_confirmed':True,'calibrated':True}}
        f.calibration_sha='test';f.revision=23;f._validate_calibration()
        return f

    def sample(self):
        return dict(type='state',sequence=1,sample_us=10000,remote_time=time.monotonic(),positions=[2058]*15,
                    online_mask=0x7fff,torque_mask=0,armed=False,imu_ok=True,acc=[0,0,9.8],gyro=[0,0,0],
                    sample_unchanged_ms=0,imu_unchanged_ms=0,uart_rtt_ms=4,fault_mask=0,firmware_error=0)

    def test_measured_angles_calibration_and_offline_freeze(self):
        f=self.configured();f._consume(self.sample());s=f.snapshot()
        self.assertTrue(s['fresh']);self.assertAlmostEqual(s['feedback']['q'][0],10*360/4095)
        self.assertAlmostEqual(s['feedback']['q'][1],-10*360/4095)
        self.assertEqual(s['feedback']['gravity'],[0,0,-1])
        f.received=time.monotonic()-1
        self.assertFalse(f.snapshot()['fresh'])
        f.process=None
        self.assertEqual(f.snapshot()['status'],'disconnected')

    def test_jaw_has_independent_measured_angle_and_missing_is_unknown(self):
        f=self.configured();row=self.sample();f._consume(row)
        before=f.snapshot()['feedback']['q'];self.assertEqual(len(before),15)
        row['positions'][14]+=200;f._consume(row);after=f.snapshot()['feedback']['q']
        self.assertEqual(before[:14],after[:14])
        self.assertAlmostEqual(after[14]-before[14],200*360/4095)
        row['online_mask'] &= ~(1<<14);f._consume(row);s=f.snapshot()
        self.assertTrue(s['fresh']);self.assertIsNone(s['feedback']['q'][14])
        f.process=None

    def test_missing_joints_imu_and_queued_frames_never_fresh(self):
        f=self.configured()
        row=self.sample();row['online_mask']=0x7ffe;f._consume(row)
        self.assertFalse(f.snapshot()['fresh'])
        row=self.sample();row['imu_ok']=False;f._consume(row)
        self.assertFalse(f.snapshot()['fresh'])
        row=self.sample();f._consume(row)
        row=self.sample();row['remote_time']-=2;f._consume(row)
        self.assertFalse(f.snapshot()['fresh'])
        f.process=None

    def test_formal_right_hip_sign_preserves_other_feedback(self):
        f=self.configured();joint=f.cfg['joints'][IDS.index(21)]
        joint.update(zero=2209,sign=1)
        row=self.sample();row['positions'][IDS.index(21)]=2541
        f._consume(row);old=f.snapshot()['feedback']['q']
        joint['sign']=-1;f.revision=24
        f._consume(row);new=f.snapshot()['feedback']
        self.assertEqual(new['calibration_revision'],24)
        self.assertAlmostEqual(new['q'][10],-old[10])
        for index in range(14):
            if index!=10:self.assertEqual(new['q'][index],old[index])
        row['positions'][IDS.index(21)]=2472;row['sample_us']+=40000
        f._consume(row);end=f.snapshot()['feedback']['q'][10]
        self.assertAlmostEqual(end-new['q'][10],69*360/4095)
        self.assertGreater(end,new['q'][10])
        f.process=None


if __name__=='__main__':unittest.main()
