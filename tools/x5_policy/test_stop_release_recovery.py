"""Real installed-class regressions using fake UART only; no device access."""
from pathlib import Path
import sys,unittest,copy,json,time,types,ast,tempfile
from unittest import mock
P=Path(__file__).resolve().parents[2]/'output/duck_sim/stop-recovery-20261012'
sys.path[:0]=[str(P/'candidate/host/policy'),str(P/'candidate/host/service')]
from stop_release import confirm_release,ReleaseUnconfirmed
import motion,native_runtime
from test_native_runtime import NativeRuntimeTests as Fixture
from test_motion import FakeClock,FakeTransport,FakePolicy,ready_config
IDS=motion.MOTOR_IDS;ALL=motion.ALL_MASK

def state(sample=1,**values):
    return dict(dict(sample_us=sample,online_mask=ALL,fault_mask=0,torque_mask=0,armed=False,cookie=0),**values)

class Verification(unittest.TestCase):
    def run_case(self,values,stop_error=False,read_duration=0.):
        c=FakeClock();seen=[];i=0
        def stop():
            seen.append('STOP')
            if stop_error:raise RuntimeError('ACK lost')
        def read():
            nonlocal i
            seen.append('STATE');c.advance(read_duration)
            v=values[min(i,len(values)-1)];i+=1
            if isinstance(v,Exception):raise v
            return copy.deepcopy(v)
        try:return confirm_release(stop,read,IDS,clock=c,sleep=c.advance),seen
        except ReleaseUnconfirmed as e:e.sent=seen;raise
    def test_short_missing_reply_then_progressing_off_confirms(self):
        r,s=self.run_case([state(1,online_mask=ALL-3),state(2),state(3)])
        self.assertTrue(r['release_confirmed']);self.assertEqual(s.count('STOP'),1)
    def test_cached_zero_mask_cannot_confirm(self):
        with self.assertRaises(ReleaseUnconfirmed):self.run_case([state(1)])
    def test_missing_or_enabled_or_armed_cannot_confirm(self):
        for k,v in [('online_mask',ALL-1),('torque_mask',1),('armed',True),('cookie',5),('fault_mask',1)]:
            with self.subTest(k=k),self.assertRaises(ReleaseUnconfirmed):self.run_case([state(1,**{k:v}),state(2,**{k:v})])
    def test_old_response_cannot_confirm(self):
        with self.assertRaises(ReleaseUnconfirmed):self.run_case([state(1),state(2)],read_duration=.2)
    def test_ack_loss_requires_actual_off_feedback_and_never_retries_stop(self):
        r,s=self.run_case([state(1),state(2)],stop_error=True)
        self.assertFalse(r['stop_acknowledged']);self.assertTrue(r['release_confirmed']);self.assertEqual(s.count('STOP'),1)
    def test_read_error_and_sample_rewind_restart_proof(self):
        r,s=self.run_case([state(10),RuntimeError('reply missing'),state(8),state(7),state(9),state(10)])
        self.assertEqual(r['samples'],5);self.assertEqual(s.count('STOP'),1)
    def test_actual_worker_waits_for_later_valid_feedback_without_closing(self):
        source=(P/'candidate/host/service/worker-classes.py').read_text()
        node=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='OneShotController')
        ns=dict(motion=motion,time=time,request_counts={})
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<candidate OneShot>','exec'),ns)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'calibration.json';cfg=ready_config();cfg['imu']['bias_dps']=[0.,0.,0.];path.write_text(json.dumps(cfg))
            clock=FakeClock();t=FakeTransport('fake-only');t.gyro=[0.,0.,0.]
            c=ns['OneShotController']({n:FakePolicy(n) for n in motion.MODEL_MODES},path,transport_factory=lambda p:t,clock=clock)
            c._connect_once();c.start_mode('walk',expected_generation=c._generation)
            original=t.request;polls=0
            def request(kind,payload=b''):
                nonlocal polls
                if kind==motion.STATE:
                    polls+=1;t.online=ALL-3 if polls==1 else ALL;clock.advance(.02)
                return original(kind,payload)
            t.request=request;before=len(t.requests);c.stop(source='button')
            self.assertTrue(c.status()['release_confirmed']);self.assertTrue(c.status()['connected'])
            self.assertGreaterEqual(polls,3);self.assertEqual(sum(k==motion.STOP for k,p in t.requests[before:]),1)
            self.assertFalse(any(k in (motion.ARM,motion.TARGET) for k,p in t.requests[before:]))

    def test_counter_wrap_is_forward_progress(self):
        r,_=self.run_case([state(0xfffffff0),state(2)]);self.assertTrue(r['release_confirmed'])

class Recovery(unittest.TestCase):
    tearDown=Fixture.tearDown;create=Fixture.create
    def setUp(self):
        Fixture.setUp(self)
        patch=mock.patch('policy_worker.PolicyWorker', return_value=types.SimpleNamespace(close=lambda:None))
        patch.start();self.patches.append(patch)
    def build(self):
        return self.create((P/'candidate/host/service/worker-classes.py').read_text(),startup_fault='original ARM timeout ID10')
    def test_closed_fault_can_release_without_models_configuration_or_arm(self):
        r=self.build();s=r.release_torque();proof=s['closed_proof']
        self.assertTrue(proof['serial_closed']);self.assertTrue(proof['release_confirmed']);self.assertEqual(s['phase'],'closed')
        self.assertEqual(proof['prior_fault'],'original ARM timeout ID10')
        self.assertEqual({k for k,p in self.transports[0].requests},{motion.HELLO,motion.STOP,motion.STATE})
        self.assertEqual(sum(k==motion.STOP for k,p in self.transports[0].requests),1)
        self.assertFalse(r.connection_reserved());self.assertIsNone(self.transports[0].serial)
    def test_other_owner_prevents_any_open(self):
        r=self.build();self.holders=[dict(pid=123)]
        with self.assertRaisesRegex(ValueError,'owner'):r.release_torque()
        self.assertFalse(r.closed_proof['release_confirmed']);self.assertTrue(r.closed_proof['serial_closed']);self.assertFalse(self.transports)
    def test_guard_failure_never_falls_back(self):
        r=self.build();r.guard['usb_recovery_guard_factory']=mock.Mock(side_effect=ValueError('identity changed'))
        with self.assertRaisesRegex(ValueError,'identity changed'):r.release_torque()
        self.assertFalse(self.transports);self.assertEqual(self.factory_calls,0)
    def test_persistent_missing_feedback_stays_unconfirmed_and_closes(self):
        r=self.build();self.initial_feedback={'online':ALL-1}
        with self.assertRaisesRegex(ValueError,'未确认'):r.release_torque()
        self.assertFalse(r.closed_proof['release_confirmed']);self.assertTrue(r.closed_proof['serial_closed']);self.assertEqual(r.phase,'fault')
    def test_active_connection_not_interrupted(self):
        r=self.build()
        with mock.patch.object(r,'_owns_locked',return_value=True):
            with self.assertRaisesRegex(ValueError,'不会抢占'):r.release_torque()
        self.assertFalse(self.transports)
    def test_wire_off_once_then_confirm_no_start_and_close(self):
        r=self.build();r.release_torque();r.release_torque()
        self.assertEqual(len(self.transports),2)
        self.assertNotIn('previous_close_proof',r.closed_proof['previous_close_proof'])
        self.assertTrue(all(k not in (motion.ARM,motion.TARGET,motion.CONFIG) for t in self.transports for k,p in t.requests))

if __name__=='__main__':
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in (Verification,Recovery))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    (P/'release-tests.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=[str(e) for e in result.errors+result.failures],hardware_access=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
