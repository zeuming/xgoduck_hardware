"""Hardware-denied reference parity checks for the explicit official contract."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np
import validate_official_goal_execution as execution

base = execution.base
ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT/'output/xgoduck_runtime_upstream/python/policy.py'
spec = importlib.util.spec_from_file_location('pinned_reference_policy', REFERENCE)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)
from rl_core import State, MOTOR_IDS, POLICY_IDS, POLICY_INDEX
from candidate_controller import ORIGINAL_HOME


class ContractTests(unittest.TestCase):
    setUp = base.OwnerTests.setUp
    cleanup = base.OwnerTests.cleanup

    def test_metadata_home_and_observation_order_4096_states(self):
        np.testing.assert_array_equal(self.c._policy_profiles['walk']['home_rad'], ORIGINAL_HOME)
        self.assertFalse(self.c._policy_profiles['walk']['home_rad'].flags.writeable)
        rng = np.random.default_rng(1849)
        for _ in range(4096):
            angles = rng.uniform(-70, 70, 15).astype(np.float32)
            velocity = rng.uniform(-200, 200, 15).astype(np.float32)
            gyro, gravity = rng.normal(size=(2, 3))
            previous = rng.normal(size=14).astype(np.float32)
            command = rng.normal(size=3).astype(np.float32)
            head = rng.normal(size=4).astype(np.float32)
            state = State([0.]*3, [0.]*3, angles.tolist(), velocity.tolist(), gravity.tolist(), gyro.tolist())
            actual = self.c._prepare_policy_observation('walk',state,previous,command,0.,head_command=head)
            expected = np.zeros((1,61),np.float32)
            expected[0,:3]=gyro;expected[0,3:6]=gravity
            expected[0,6:20]=np.deg2rad(angles[POLICY_INDEX])-ORIGINAL_HOME
            expected[0,20:34]=np.deg2rad(velocity[POLICY_INDEX])
            expected[0,34:48]=previous;expected[0,48:51]=command;expected[0,51:55]=head
            np.testing.assert_array_equal(actual,expected)

    def test_sequential_filter_exactly_matches_reference_4096_frames(self):
        original=reference.Policy();original.session=base.FakePolicy()
        original.in_name='obs';original._outputs=['actions']
        original._feed={'obs':original._obs.reshape(1,61)}
        smooth=np.zeros(14,np.float32);previous=smooth.copy()
        rng=np.random.default_rng(6718)
        for _ in range(4096):
            raw=rng.uniform(-.8,.8,(1,14)).astype(np.float32)
            original.session.output=raw
            filtered=original.infer([0.,0.,-1.],[0.]*3,[0.]*15,[0.]*15)
            target,actual_raw,next_smooth,_=self.c._decode_policy_action('walk',raw,previous,smooth,0.,0.)
            np.testing.assert_array_equal(next_smooth,filtered)
            np.testing.assert_array_equal(actual_raw,raw[0])
            expected=np.zeros(15,np.float32)
            expected[POLICY_INDEX]=np.rad2deg(ORIGINAL_HOME+filtered)
            np.testing.assert_array_equal(target,expected)
            self.assertEqual(target.diagnostics['roll_decoder'],{})
            np.testing.assert_array_equal(previous,original._obs[34:48])
            previous=actual_raw;smooth=next_smooth

    def test_saved_range_conflict_is_not_silently_retargeted(self):
        cfg=json.loads((ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json').read_text())['candidate']
        self.c._config['joints']=[dict(id=j['id'],zero=j['zero_tick'],sign=j['sign'],min=j['min_tick'],max=j['max_tick']) for j in cfg['calibration']['joints']]
        raw=np.zeros((1,14),np.float32)
        raw[0,POLICY_IDS.index(14)]=np.deg2rad(49.7)-ORIGINAL_HOME[4]
        target,_,_,_=self.c._decode_policy_action('walk',raw,raw[0],raw[0],0.,0.)
        self.assertAlmostEqual(target[MOTOR_IDS.index(14)],49.7,places=4)
        with self.assertRaises(base.motion.MotionError):self.c._raw_targets_locked(target,mode='walk')
        self.assertEqual(self.c._target_rejection['id'],14)
        self.assertFalse(any(k==base.motion.TARGET for k,p in self.t.requests))

    def test_bad_output_and_history_are_rejected_and_status_is_explicit(self):
        for raw in (np.zeros((14,)),np.full((1,14),np.nan),np.full((1,14),np.inf)):
            with self.assertRaises(base.motion.MotionError):
                self.c._decode_policy_action('walk',raw,np.zeros(14),np.zeros(14),0.,0.)
        with self.assertRaises(base.motion.MotionError):
            self.c._decode_policy_action('walk',np.zeros((1,14)),np.zeros(14),np.full(14,np.nan),0.,0.)
        profile=self.c.status()['candidate_profile']
        self.assertEqual(profile['HOME_offsets_deg'],{})
        self.assertEqual(profile['s95_roll_scale'],1.)
        self.assertFalse(profile['additional_joint_retargeting'])
        self.assertFalse(profile['hardware_qualified'])


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ContractTests))
    report=dict(tests=result.testsRun,passed=result.wasSuccessful(),hardware_access=False,
        failures=[str(x) for x in result.failures+result.errors],source=str(base.a.source.resolve()),
        reference_path=str(REFERENCE),reference_sha256=hashlib.sha256(REFERENCE.read_bytes()).hexdigest(),
        compared_observations=4096,compared_filter_frames=4096,scope='Decoder/observation parity; not closed-loop walking or physical validation')
    base.a.output.write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
