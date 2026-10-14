"""Installed connection/STOP/save regression with fake UART and temporary files."""
import argparse
import copy
import json
from pathlib import Path
import struct
import sys
import threading
import types
import unittest
from unittest import mock

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
sys.path[:0] = [str(args.source / 'policy'), str(args.source / 'service')]
import motion
import native_workbench
from stop_release import confirm_release, ReleaseUnconfirmed
from test_motion import FakeClock, FakeTransport
from test_native_runtime import NativeRuntimeTests as Fixture

ALL, IDS = motion.ALL_MASK, motion.MOTOR_IDS
original_request = FakeTransport.request


def source_request(self, kind, payload=b''):
    if kind == motion.SOURCE_TARGET:
        original_request(self, motion.TARGET, payload)
        self.requests[-1] = (kind, payload)
        return b''
    if kind == motion.SOURCE_STATE:
        # Real decoder and controller; only the UART bytes are synthetic.
        raw = original_request(self, motion.STATE)
        self.requests[-1] = (kind, payload)
        known = getattr(self, 'torque_known', self.online)
        ages = [1000 if self.online & (1 << i) else 0xffffffff for i in range(15)]
        return raw + struct.pack('<HBB15I', known, 1, 1, *ages)
    return original_request(self, kind, payload)


class ReleaseProof(unittest.TestCase):
    def prove(self, **changes):
        clock = FakeClock()
        def sample():
            clock.advance(.02)
            return dict(dict(sample_us=int(clock.now*1e6), online_mask=ALL,
                torque_known_mask=ALL, fault_mask=0, torque_mask=0, armed=False, cookie=0), **changes)
        return confirm_release(lambda: None, sample, IDS, clock=clock, sleep=clock.advance)

    def test_missing_position_does_not_invalidate_independent_torque_readback(self):
        report = self.prove(online_mask=ALL & ~(1 << IDS.index(22)))
        self.assertTrue(report['release_confirmed'])
        self.assertEqual(report['last_state']['missing_ids'], [22])
        self.assertEqual(report['last_state']['torque_unknown_ids'], [])

    def test_unknown_enabled_faulted_or_armed_cannot_confirm(self):
        for values in (dict(torque_known_mask=ALL-1), dict(torque_mask=1),
                       dict(armed=True), dict(cookie=5), dict(fault_mask=1)):
            with self.subTest(values=values), self.assertRaises(ReleaseUnconfirmed):
                self.prove(**values)

    def test_legacy_reply_still_requires_every_servo_read(self):
        clock=FakeClock()
        def sample():
            clock.advance(.02)
            return dict(sample_us=int(clock.now*1e6), online_mask=ALL-1,
                        fault_mask=0, torque_mask=0, armed=False, cookie=0)
        with self.assertRaises(ReleaseUnconfirmed):
            confirm_release(lambda:None, sample, IDS, clock=clock, sleep=clock.advance)


class Loading(unittest.TestCase):
    create, tearDown = Fixture.create, Fixture.tearDown
    def setUp(self):
        Fixture.setUp(self)
        patch = mock.patch.object(FakeTransport, 'request', source_request)
        patch.start(); self.patches.append(patch)
        # No inference process or ONNX is needed for connection/configuration.
        patch = mock.patch('policy_worker.PolicyWorker', return_value=types.SimpleNamespace(close=lambda:None))
        patch.start(); self.patches.append(patch)
        self.classes=(args.source/'service/worker-classes.py').read_text()

    def assert_no_motor_start(self):
        self.assertFalse(any(k in (motion.ARM,motion.TARGET,motion.SOURCE_TARGET)
                             for t in self.transports for k,p in t.requests))

    def test_both_range_outliers_load_but_start_remains_blocked(self):
        positions=[2047]*15;positions[1]=99;positions[2]=4000
        self.initial_feedback=dict(positions=positions, torque_known=ALL)
        runtime=self.create(self.classes);loaded=runtime.connect()
        self.assertEqual(loaded['phase'],'ready')
        self.assertTrue(loaded['hardware']['release_confirmed'])
        self.assertEqual(runtime.connect()['connection_sequence'],loaded['connection_sequence'])
        with self.assertRaisesRegex(motion.MotionError,'ID 11 实际位置超出'):
            runtime.action(dict(action='start',mode='hold',generation=loaded['hardware']['generation'],support_confirmed=True))
        self.assertTrue(runtime.connection_reserved())
        self.assertEqual(len(self.transports),1)
        self.assert_no_motor_start()

    def test_missing_id22_stop_keeps_connection_and_other_boundaries_can_save(self):
        positions=[2047]*15;positions[1]=99
        self.initial_feedback=dict(positions=positions, torque_known=ALL,
            online=ALL & ~(1<<IDS.index(22)), imu_ok=False, imu_age_us=60000000)
        runtime=self.create(self.classes);loaded=runtime.connect()
        self.assertEqual(loaded['phase'],'ready')
        self.assertNotIn(22,loaded['hardware']['online_ids'])
        self.assertTrue(loaded['hardware']['release_confirmed'])
        stopped=runtime.action(dict(action='stop',stop_source='button'))['hardware']
        self.assertTrue(stopped['connected']);self.assertTrue(stopped['release_confirmed'])
        with self.assertRaises(motion.MotionError):
            runtime.action(dict(action='start',mode='hold',generation=stopped['generation'],support_confirmed=True))
        before=runtime.range_feedback_snapshot()
        self.assertGreater(before['hardware']['imu_age_ms'],500)
        self.assertLess(before['feedback_age_ms'],100)
        config=copy.deepcopy(self.calibration)
        writes=[]
        def save(value):
            writes.append(copy.deepcopy(value))
            (self.directory/'saved-boundaries.json').write_text(json.dumps(value))
        session=types.SimpleNamespace(config=config,lock=threading.RLock(),_write_calibration=save)
        wb=types.SimpleNamespace(runtime=runtime,calibration_service=types.SimpleNamespace(session=session),snapshot=lambda **kw:{})
        j=next(j for j in config['joints'] if j['id']==11)
        native_workbench.NativeWorkbench._expand_feedback_ranges(wb,
            dict(generation=before['hardware']['generation'],updates=[dict(id=11,position=99,min=j['min'],max=j['max'])]),before)
        self.assertEqual(len(writes),1)
        self.assertEqual(session.config['joints'][1]['min'],99)
        session.config['joints'][1]['min']=j['min']
        self.assertEqual(session.config,config)
        self.assertFalse(runtime.connection_reserved())
        self.assertTrue(runtime.closed_proof['serial_closed'])
        self.assertTrue(runtime.closed_proof['release_confirmed'])
        self.assert_no_motor_start()

    def test_unknown_torque_still_refuses_loading_and_never_starts(self):
        self.initial_feedback=dict(torque_known=ALL & ~(1<<IDS.index(22)))
        runtime=self.create(self.classes)
        with self.assertRaises(ValueError):runtime.connect()
        self.assertFalse(runtime.connection_reserved())
        self.assertFalse(runtime.closed_proof['release_confirmed'])
        self.assert_no_motor_start()

    def test_stop_from_fake_hold_confirms_torque_even_with_missing_position(self):
        self.initial_feedback=dict(torque_known=ALL)
        runtime=self.create(self.classes);loaded=runtime.connect()
        runtime.action(dict(action='start',mode='hold',generation=loaded['hardware']['generation'],support_confirmed=True))
        transport=self.transports[0]
        transport.online=ALL & ~(1<<IDS.index(22))
        began=len(transport.requests)
        stopped=runtime.action(dict(action='stop',stop_source='button'))['hardware']
        self.assertTrue(stopped['connected']);self.assertTrue(stopped['release_confirmed'])
        self.assertFalse(stopped['armed']);self.assertEqual(stopped['torque_ids'],[])
        requests=transport.requests[began:]
        self.assertEqual(sum(k==motion.STOP for k,p in requests),1)
        self.assertFalse(any(k in (motion.ARM,motion.TARGET,motion.SOURCE_TARGET) for k,p in requests))


if __name__=='__main__':
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in (ReleaseProof,Loading))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    args.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        errors=[str(e) for e in result.errors+result.failures],hardware_access=False),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
