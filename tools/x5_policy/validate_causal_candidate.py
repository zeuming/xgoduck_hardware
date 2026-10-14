"""Exercise actual candidate start/forward/stop paths with full fake v2 data."""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,required=True)
args=parser.parse_args()
sys.path[:0]=[str(args.source.resolve()/'policy'),str(Path(__file__).resolve().parent)]
def deny_device(event,values):
    if event=='open' and values and isinstance(values[0],(str,bytes)):
        path=os.path.realpath(os.fsdecode(values[0]))
        if path.startswith('/dev/') and path!='/dev/null':
            raise PermissionError('Offline causal validation cannot open a device')
sys.addaudithook(deny_device)
import motion
from test_motion import FakeClock,FakePolicy,ready_config
from causal_test_transport import PairedFakeTransport
from attended_trial_limits import ZERO_WINDOW_S,ZERO_COMPLETE,PAGE_LOST_HOLD


class CandidateCausalityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.clock=FakeClock();self.transport=PairedFakeTransport('fake-only')
        config=Path(self.tmp.name)/'calibration.json';config.write_text(json.dumps(ready_config()))
        self.c=motion.MotionController({'walk':FakePolicy()},config,
            transport_factory=lambda _:self.transport,clock=self.clock,allow_fixed_head_walk=True)
        self.addCleanup(self.c.close)

    def hold(self):
        self.c._connect_once()
        self.c.capture_head_clearance(clearance_confirmed=True,expected_generation=self.c._generation)
        self.c.start_mode('hold',support_confirmed=True)
        self.clock.advance(3.1);self.c._cycle()
        self.clock.advance(.02);self.c._cycle()
        self.assertTrue(self.c.status()['hold_ready'])

    def walk(self):
        self.hold();self.c.start_mode('walk',support_confirmed=True)
        for _ in range(270):
            self.clock.advance(.02);self.c.heartbeat(minimal=True);self.c._cycle()
        self.assertTrue(self.c._walk_warmup_ready_locked())

    def test_old_protocol_is_rejected_before_config_arm_or_target(self):
        self.transport.protocol_version=1
        with self.assertRaisesRegex(motion.MotionError,'协议 v2'):
            self.c._connect_once()
        self.assertFalse(any(k in (motion.ARM,motion.TARGET,motion.CONFIG) for k,_ in self.transport.requests))
        self.assertTrue(self.transport.closed)

    def test_connect_and_status_are_off_no_automatic_action(self):
        self.c._connect_once()
        self.assertEqual(self.c._motion_evidence_supported,2)
        self.assertEqual(self.c._state_request_payload,b'\x02')
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in self.transport.requests))
        self.assertEqual(self.c._leg_following_status_locked()['status'],'inactive')

    def test_forward_uses_paired_target_and_rejects_unknown_without_unloading(self):
        self.walk()
        self.assertIsNone(self.c._forward_block_locked())
        self.transport.pair_valid=False;self.clock.advance(.02);self.c._cycle()
        self.assertIn('未形成有效配对',self.c._forward_block_locked())
        before=self.transport.stop_count
        with self.assertRaises(motion.MotionError):
            self.c.forward_trial(expected_generation=self.c._generation,support_confirmed=True)
        self.assertEqual(self.transport.stop_count,before)
        self.assertTrue(self.transport.armed)

    def test_page_loss_with_valid_pair_blends_to_hold_without_stop_or_rearm(self):
        self.walk();before=len(self.transport.requests);generation=self.c._generation
        # Feedback/TARGET cycles continue while only the page lease expires.
        for _ in range(22):
            self.clock.advance(.02);self.c._cycle()
            if self.c._mode != 'walk':break
        self.assertEqual(self.c._mode,'hold')
        self.assertTrue(self.transport.armed)
        self.assertEqual(self.c._generation,generation+1)
        self.assertEqual(self.c._last_trial_stop['reason'],PAGE_LOST_HOLD)
        self.assertFalse(any(k in (motion.STOP,motion.ARM) for k,_ in self.transport.requests[before:]))
        self.assertEqual(self.c._last_trial_stop['leg_following']['target_source'],
                         'MCU_sample_bound_transmitted_goal')

    def test_page_loss_with_unknown_pair_does_not_invent_recovery_evidence(self):
        self.walk();self.transport.pair_valid=False
        for _ in range(22):
            self.clock.advance(.02);self.c._cycle()
            if self.c._mode != 'walk':break
        self.assertEqual(self.c._mode,'off')
        self.assertFalse(self.transport.armed)
        self.assertIn('未形成有效配对',self.c._last_trial_stop['reason'])

    def test_normal_window_completion_keeps_torque_and_hold(self):
        self.walk();before=len(self.transport.requests)
        self.c._trial_began=self.clock()-ZERO_WINDOW_S
        self.c.heartbeat(minimal=True);self.clock.advance(.02);self.c._cycle()
        self.assertEqual(self.c._mode,'hold')
        self.assertTrue(self.transport.armed)
        self.assertEqual(self.c._last_trial_stop['reason'],ZERO_COMPLETE)
        self.assertFalse(any(k in (motion.STOP,motion.ARM) for k,_ in self.transport.requests[before:]))

    def test_measured_following_error_is_not_reported_as_missing_data(self):
        self.walk();goals=list(self.transport.paired_goals);goals[0]+=60
        self.transport.goal_override=goals;self.clock.advance(.02);self.c._cycle()
        following=self.c._leg_following_status_locked()
        self.assertEqual(following['status'],'tracking_error')
        self.assertIn('偏差',self.c._forward_block_locked())
        self.assertTrue(self.transport.armed)

    def test_stop_trace_retains_v2_causal_fields_without_cookie(self):
        self.walk();self.c.stop()
        trace=self.c.completed_trial_traces()[-1]
        frame=trace['frames'][-1]
        self.assertEqual(frame['MCU_evidence_protocol'],2)
        self.assertTrue(frame['MCU_pairing_valid'])
        self.assertIn('MCU_scan_start_us',frame)
        self.assertFalse(any('cookie' in key for key in frame))
        self.assertFalse(self.transport.armed)

    def test_capability_is_explicit_and_malformed_reply_never_downgrades(self):
        for capability,accepted in ((b'XGODUCK_MOTION_EVIDENCE_2',True),
                                    (b'XGODUCK_MOTION_EVIDENCE_1',False),
                                    (b'',False)):
            requests=[]
            transport=motion.SerialTransport('fake-only')
            def request(kind,payload=b''):
                requests.append((kind,payload));return capability
            transport.request=request
            if accepted:self.assertEqual(transport.negotiate_motion_evidence(),2)
            else:
                with self.assertRaises(motion.MotionError):transport.negotiate_motion_evidence()
            self.assertEqual(requests,[(motion.MOTION_EVIDENCE_CAPS,b'\x02')])

    def test_v2_length_and_version_are_not_silently_decoded_as_v1(self):
        packet=self.transport.request(motion.VOLTAGE_STATE,b'\x02')
        self.assertEqual(len(packet),238)
        self.assertEqual(motion.decode_state(packet)['motion_evidence']['protocol'],2)
        for broken in (packet[:-1],packet+b'\0',packet[:186]+bytes([1])+packet[187:]):
            with self.assertRaises(motion.MotionError):motion.decode_state(broken)


result=unittest.TextTestRunner(verbosity=2).run(unittest.TestLoader().loadTestsFromTestCase(CandidateCausalityTests))
raise SystemExit(not result.wasSuccessful())
