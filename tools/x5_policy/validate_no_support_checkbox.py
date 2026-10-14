"""Test the installed variant with fake I/O; never opens a hardware device."""
import argparse,json,os,sys,tempfile,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);args=p.parse_args()
sys.path[:0]=[str(args.source.resolve()/'policy'),str(Path(__file__).resolve().parent)]
def deny_device(event,values):
    if event=='open' and values and isinstance(values[0],(str,bytes)) and os.path.realpath(os.fsdecode(values[0])).startswith('/dev/'):
        raise PermissionError('Offline test cannot open devices')
sys.addaudithook(deny_device)
import motion
from test_motion import FakeClock,FakePolicy,FakeTransport,ready_config
class Transport(FakeTransport):
    def request(self,kind,payload=b''):
        if kind==24:self.requests.append((kind,payload));return bytes([24,0])
        if kind==motion.TARGET and len(payload)==35:
            assert payload[-1]==1
            result=super().request(kind,payload[:34]);self.requests[-1]=(kind,payload);return result
        return super().request(kind,payload)
class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.clock=FakeClock();self.t=Transport('fake-only');self.policy=FakePolicy()
        config=Path(self.tmp.name)/'calibration.json';config.write_text(json.dumps(ready_config()))
        self.c=motion.MotionController({'walk':self.policy},config,transport_factory=lambda _:self.t,clock=self.clock,allow_fixed_head_walk=True)
        self.addCleanup(self.c.close);self.c._connect_once()
    def hold(self):
        self.c.start_mode('hold',support_confirmed=False)
        self.clock.advance(3.1);self.c._cycle();self.clock.advance(.02);self.c._cycle()
        self.assertTrue(self.c.status()['hold_ready'])
    def test_connection_does_not_start_or_capture(self):
        self.assertFalse(self.c._require_head_clearance);self.assertIsNone(self.c._hold_head_deg)
        self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,_ in self.t.requests))
    def test_hold_without_capture_uses_existing_model_home(self):
        self.hold();self.assertEqual(list(self.c._fixed_stance_locked()),list(self.c._stand_deg))
        self.assertIsNone(self.c._hold_head_deg)
    def test_walk_without_capture(self):
        self.hold();self.c.start_mode('walk',support_confirmed=False)
        self.assertEqual(self.c._mode,'walk');self.assertIsNone(self.c._hold_head_deg)
    def test_old_capture_cannot_reenable_feature(self):
        before=len(self.t.requests)
        with self.assertRaisesRegex(motion.MotionError,'已移除'):self.c.capture_head_clearance(clearance_confirmed=True,expected_generation=self.c._generation)
        self.assertEqual(before,len(self.t.requests));self.assertFalse(self.c._require_head_clearance)
    def test_support_checkbox_no_longer_required(self):
        self.c.start_mode('hold')
        self.assertTrue(self.t.armed)
        self.assertEqual(self.c._mode,'hold')
    def test_stale_generation_still_rejected(self):
        with self.assertRaises(motion.MotionError):self.c.start_mode('hold',support_confirmed=False,expected_generation=-1)
        self.assertFalse(self.t.armed)
    def test_saved_head_target_range_still_enforced(self):
        self.c._config['joints'][10]['min']=self.c._config['joints'][10]['zero']-1
        with self.assertRaises(motion.MotionError):self.c.start_mode('hold',support_confirmed=False)
        self.assertFalse(self.t.armed)
    def test_expired_feedback_still_rejected(self):
        self.clock.advance(.2)
        with self.assertRaises(motion.MotionError):self.c.start_mode('hold',support_confirmed=False)
        self.assertFalse(self.t.armed)
    def test_stop_still_releases_torque(self):
        self.hold();self.c.stop();self.assertFalse(self.t.armed);self.assertEqual(self.c._mode,'off')
    def test_forward_uses_runtime_readiness_without_checkbox(self):
        self.hold();self.c.start_mode('walk')
        for _ in range(275):
            self.clock.advance(.02);self.c.heartbeat();self.c._cycle()
        reason=self.c._forward_block_locked()
        if reason:
            with self.assertRaises(motion.MotionError) as error:
                self.c.forward_trial(expected_generation=self.c._generation)
            self.assertEqual(str(error.exception),reason)
            self.assertNotIn('防跌准备',str(error.exception))
        else:
            self.c.forward_trial(expected_generation=self.c._generation)
            self.assertTrue(self.c._forward.used)
    def test_other_models_still_locked(self):
        for name in ('getup','pick','roulade'):
            with self.assertRaises(motion.MotionError):self.c.start_mode(name,support_confirmed=False,behavior_confirmed=True)
        self.assertFalse(self.t.armed)
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
sys.exit(not result.wasSuccessful())
