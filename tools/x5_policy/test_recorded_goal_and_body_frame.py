"""Offline replay admission and virtual-frame contract checks."""
import copy
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from body_frame_pose_map import BodyFramePoseMap
from replay_recorded_wire_goals import wire_schedule
from rl_core import POLICY_IDS

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=ROOT/'output/duck_sim/source-parity-20261012/backward-causal-observation'


class GoalReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.trial=json.loads((EVIDENCE/'recent-completed/connection-1-trial-3.json').read_text())
        cls.cal=json.loads((EVIDENCE/'calibration-operator-33-v2.json').read_text())

    def test_only_actually_sent_goals_reach_replay(self):
        schedule,samples,end=wire_schedule(self.trial,self.cal,POLICY_IDS)
        self.assertEqual(len(schedule),62) # preceding HOLD plus61 accepted goals
        self.assertEqual(len(samples)%14,0)
        self.assertTrue(all(b['time_s']>a['time_s'] for a,b in zip(schedule,schedule[1:])))
        rejected=self.trial['policy_frames'][-1]
        self.assertFalse(rejected['target_write_completed'])
        self.assertIsNone(rejected.get('transmitted_ticks'))
        self.assertLess(schedule[-1]['time_s'],end)

    def test_stale_wrong_range_fixture_is_rejected(self):
        stale=copy.deepcopy(self.cal)
        row=next(j for j in stale['joints'] if j['id']==12)
        row['max']=2882
        with self.assertRaisesRegex(ValueError,'rejected goal'):
            wire_schedule(self.trial,stale,POLICY_IDS)

    def test_position_comparison_uses_each_joint_measurement_age(self):
        _,samples,_=wire_schedule(self.trial,self.cal,POLICY_IDS)
        row=self.trial['policy_frames'][1]
        p=row['input_feedback_provenance']
        p0=self.trial['policy_frames'][0]['input_feedback_provenance']
        all_ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
        for sid in POLICY_IDS:
            i=all_ids.index(sid)
            at=(p['state_sample_us']-p0['state_sample_us']-p['position_ages_us'][i])/1e6
            self.assertTrue(any(s['id']==sid and abs(s['time_s']-at)<1e-12 for s in samples))


class FrameContractTests(unittest.TestCase):
    def test_rigid_frame_vectors_match_world_measurement(self):
        rng=np.random.default_rng(22)
        virtual=Rotation.from_rotvec([.05,-.08,.11]).as_matrix()
        delta=Rotation.from_rotvec([0.,.12,0.]).as_matrix()
        physical=virtual@delta
        offset=rng.normal(size=14)*.03
        mapper=BodyFramePoseMap(delta,offset)
        obs=rng.normal(size=(1,61)).astype(np.float32)
        world_omega=np.array([.3,-.2,.1]); gravity=np.array([0.,0.,-1.])
        obs[0,:3]=physical.T@world_omega; obs[0,3:6]=physical.T@gravity
        original=obs.copy()
        expected=obs.copy(); expected[0,:3]=virtual.T@world_omega; expected[0,3:6]=virtual.T@gravity
        mapped=mapper.observations(obs)
        np.testing.assert_allclose(mapped[0,:6],expected[0,:6],atol=1e-7)
        np.testing.assert_array_equal(mapped[...,20:],obs[...,20:])
        np.testing.assert_array_equal(obs,original)
        np.testing.assert_allclose(mapper.goals(mapped[0,6:20]),obs[0,6:20],atol=1e-8)

    def test_velocity_derivative_and_raw_history_are_preserved(self):
        rng=np.random.default_rng(13)
        mapper=BodyFramePoseMap(Rotation.from_rotvec([0.,.12,0.]).as_matrix(),rng.normal(size=14)*.03)
        a=rng.normal(size=61); b=a.copy(); dt=1e-5
        b[6:20]+=dt*a[20:34]
        da=(mapper.observations(b)[6:20]-mapper.observations(a)[6:20])/dt
        np.testing.assert_allclose(da,a[20:34],atol=1e-10)
        np.testing.assert_array_equal(mapper.observations(a)[34:],a[34:])

    def test_identity_map_is_exact_and_reflection_is_rejected(self):
        obs=np.random.default_rng(1).normal(size=(1,61)).astype(np.float32)
        np.testing.assert_array_equal(BodyFramePoseMap(np.eye(3),np.zeros(14)).observations(obs),obs)
        with self.assertRaises(ValueError):
            BodyFramePoseMap(np.diag([1.,1.,-1.]),np.zeros(14))


if __name__=='__main__':unittest.main()
