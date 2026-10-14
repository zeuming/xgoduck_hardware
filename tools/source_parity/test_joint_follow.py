"""Exercise real controller control/packet paths with a fake UART only."""
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

import argparse,sys
parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
sys.path[:0]=[str(args.candidate/'policy'),str(Path(__file__).resolve().parents[1]/'x5_policy')]
from test_source_host import Transport as SourceTransport
sys.argv=[sys.argv[0]]
from joint_follow import LEASE_S, controller_class, validate_target
from motion import MotionController, MotionError, MOTOR_IDS, ALL_MASK, TARGET, ARM
from test_motion import FakeClock, FakeTransport, FakePolicy, ready_config

class FollowTransport(SourceTransport):
    def request(self,kind,payload=b''):
        if kind==TARGET and len(payload)==34:
            return FakeTransport.request(self,kind,payload)
        return super().request(kind,payload)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        path=Path(self.tmp.name)/'calibration.json'
        config=ready_config()
        config['joints'][6]['sign']=1
        path.write_text(json.dumps(config))
        self.clock=FakeClock();self.transport=FollowTransport('fake')
        cls=controller_class(MotionController, MotionError, MOTOR_IDS, ALL_MASK, TARGET)
        self.c=cls({'walk':FakePolicy()},path,transport_factory=lambda _:self.transport,clock=self.clock)
        self.c._connect_once()
        self.transport.positions=[2047+i*2 for i in range(15)]
        self.c._accept_state_locked(self.c._read_state())

    def tearDown(self):
        self.tmp.cleanup()

    def start(self):
        return self.c.start_joint_follow(expected_generation=self.c._generation,
            support_confirmed=True,clearance_confirmed=True)

    def tick(self, n=1):
        for _ in range(n):
            self.clock.advance(.02)
            self.c.heartbeat(expected_generation=self.c._generation)
            self.c._accept_state_locked(self.c._read_state())
            self.c._control_once()

    def test_start_captures_actual_pose_never_home(self):
        initial=list(self.transport.positions)
        status=self.start();self.tick()
        self.assertEqual(self.transport.positions,initial)
        self.assertTrue(status['joint_follow']['active'])
        self.assertFalse(status['hold_ready'])
        self.assertTrue(status['page_heartbeat_required'])
        self.assertEqual(sum(k==ARM for k,_ in self.transport.requests),1)

    def test_only_selected_joint_moves_slowly_using_saved_direction(self):
        self.start();initial=list(self.transport.positions)
        p=self.c.status()['joint_follow'];j=p['joints'][6]
        self.c.joint_target(expected_generation=self.c._generation,joint_id=21,
            angle_deg=j['target_deg']+3,sequence=1)
        self.tick(10)
        changes=[i for i,(a,b) in enumerate(zip(initial,self.transport.positions)) if a!=b]
        self.assertEqual(changes,[6]);self.assertGreater(self.transport.positions[6],initial[6])
        self.assertLessEqual((self.transport.positions[6]-initial[6])*360/4095,1.1)

    def test_switching_joint_freezes_previous_target(self):
        self.start();g=self.c._generation
        self.c.joint_target(expected_generation=g,joint_id=10,angle_deg=5,sequence=1)
        self.tick(5);last=self.transport.positions[0]
        self.c.joint_target(expected_generation=g,joint_id=21,angle_deg=3,sequence=2)
        self.tick(5);self.assertEqual(self.transport.positions[0],last)

    def test_speed_changes_rate_without_changing_angle_or_other_joints(self):
        self.start();g=self.c._generation;initial=list(self.transport.positions)
        mouth=self.c.status()['joint_follow']['joints'][-1]['target_deg']
        self.c.joint_target(expected_generation=g,joint_id=34,angle_deg=mouth+8,sequence=1,speed_deg_s=5)
        self.tick(5);slow=self.c.status()['joint_follow']['joints'][-1]['target_deg']
        self.assertAlmostEqual(slow-mouth,.5)
        self.c.joint_target(expected_generation=g,joint_id=34,angle_deg=mouth+8,sequence=2,speed_deg_s=50)
        self.tick(5);p=self.c.status()['joint_follow']
        self.assertAlmostEqual(p['joints'][-1]['target_deg']-slow,5)
        self.assertEqual(p['joints'][-1]['requested_deg'],mouth+8)
        self.assertEqual(p['speed_deg_s'],50)
        self.assertEqual(initial[:-1],self.transport.positions[:-1])
        # Older clients omitting speed keep the chosen session rate.
        self.c.joint_target(expected_generation=g,joint_id=34,angle_deg=mouth+7,sequence=3)
        self.assertEqual(self.c.status()['joint_follow']['speed_deg_s'],50)

    def test_speed_is_bounded_and_delayed_tick_cannot_jump(self):
        self.start();g=self.c._generation;before=self.c.status()['joint_follow']
        for speed in (0,51,True,'20',float('nan'),float('inf')):
            with self.subTest(speed=speed),self.assertRaises(ValueError):
                self.c.joint_target(expected_generation=g,joint_id=34,angle_deg=5,sequence=1,speed_deg_s=speed)
        self.assertEqual(self.c.status()['joint_follow'],before)
        start=self.c.status()['joint_follow']['joints'][-1]['target_deg']
        self.c.joint_target(expected_generation=g,joint_id=34,angle_deg=start+3,sequence=1,speed_deg_s=50)
        self.clock.advance(.3);self.c._accept_state_locked(self.c._read_state());self.c._control_once()
        self.assertAlmostEqual(self.c.status()['joint_follow']['joints'][-1]['target_deg']-start,2)

    def test_stale_out_of_range_and_nonfinite_requests_send_no_target(self):
        self.start();g=self.c._generation;before=len(self.transport.requests)
        for values in ((10,11,1),(10,float('nan'),1),(10,float('inf'),1),(True,0,1),(99,0,1),(10,0,True)):
            with self.subTest(values=values),self.assertRaises(ValueError):
                self.c.joint_target(expected_generation=g,joint_id=values[0],angle_deg=values[1],sequence=values[2])
        self.c.joint_target(expected_generation=g,joint_id=10,angle_deg=0,sequence=1)
        with self.assertRaises(ValueError):self.c.joint_target(expected_generation=g,joint_id=10,angle_deg=2,sequence=1)
        with self.assertRaises(ValueError):self.c.joint_target(expected_generation=g-1,joint_id=10,angle_deg=2,sequence=2)
        self.assertEqual(len(self.transport.requests),before)

    def test_expired_lease_cannot_revive_or_send(self):
        self.start();self.clock.advance(LEASE_S+.001)
        before=sum(k==TARGET for k,_ in self.transport.requests)
        with self.assertRaises(ValueError):self.c.heartbeat(expected_generation=self.c._generation)
        self.c._accept_state_locked(self.c._read_state())
        self.c._cycle()
        self.assertEqual(sum(k==TARGET for k,_ in self.transport.requests),before)
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.c._following())

    def test_mode_switch_and_calibration_change_rejected(self):
        self.start()
        with self.assertRaises(ValueError):self.c.start_mode('hold',True)
        self.c._config['joints'][0]['sign']*= -1
        with self.assertRaises(ValueError):self.c._control_once()

    def test_enable_requires_confirmations_and_off_state(self):
        for support,clearance in ((False,True),(True,False)):
            with self.assertRaises(ValueError):self.c.start_joint_follow(expected_generation=self.c._generation,
                support_confirmed=support,clearance_confirmed=clearance)
        self.assertFalse(any(k==ARM for k,_ in self.transport.requests))
        self.start()
        with self.assertRaises(ValueError):self.start()

    def test_saved_limits_intersect_excursion_even_at_endpoint(self):
        self.c._config['joints'][0]['min']=2047
        self.start()
        self.assertAlmostEqual(self.c.status()['joint_follow']['joints'][0]['max_deg'],0)
        with self.assertRaises(ValueError):self.c.joint_target(expected_generation=self.c._generation,
            joint_id=10,angle_deg=.1,sequence=1)

    def test_current_position_outside_saved_range_rejects_before_arm(self):
        index=MOTOR_IDS.index(30)
        self.c._config['joints'][index].update(zero=1977,sign=-1,min=1720,max=3729)
        self.transport.positions[index]=1603
        self.c._accept_state_locked(self.c._read_state())
        before=len(self.transport.requests)
        with self.assertRaisesRegex(MotionError,'ID 30.*1603.*1720'):
            self.start()
        self.assertFalse(self.c._following())
        self.assertFalse(any(kind in (ARM,TARGET) for kind,_ in self.transport.requests[before:]))


if __name__=='__main__':unittest.main()
