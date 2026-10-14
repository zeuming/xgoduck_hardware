"""Check recorded-entry frame conventions and prohibit running-state injection."""
import copy
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np
import torch

from offline_recorded_entry import recorded_entry

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT/'output/duck_sim/source-parity-20261012/backward-causal-observation'
TRIAL = EVIDENCE/'recent-completed/connection-1-trial-3.json'
MJCF = ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'


class EntryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.candidate = json.loads((ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json').read_text())['candidate']

    def test_initialization_preserves_recorded_body_and_joint_velocity_frames(self):
        initialize, fixture = recorded_entry(TRIAL, self.candidate, MJCF)
        obs = np.asarray(json.loads(TRIAL.read_text())['policy_frames'][0]['submitted_observation'])[0]
        calls = {}
        asset = SimpleNamespace(joint_names=self.candidate['joint_names'],
            write_joint_state_to_sim=lambda q, v, **kw: calls.update(q=q, v=v),
            write_root_state_to_sim=lambda state, **kw: calls.update(state=state))
        env = SimpleNamespace(common_step_counter=0, num_envs=1, device='cpu',
            scene={'robot': asset})
        env.scene = type('Scene', (dict,), {})(env.scene)
        env.scene.env_origins = torch.zeros((1, 3))
        initialize(env, None)
        state = calls['state'][0].numpy()
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, state[3:7].astype(float))
        rotation = rotation.reshape(3, 3)
        np.testing.assert_allclose(rotation.T @ state[10:13], obs[:3], atol=1e-8)
        np.testing.assert_allclose(rotation.T @ [0., 0., -1.], obs[3:6], atol=2e-7)
        np.testing.assert_allclose(calls['q'][0].numpy(), obs[6:20]+self.candidate['home_rad'], atol=1e-7)
        np.testing.assert_allclose(calls['v'][0].numpy(), obs[20:34], atol=1e-7)
        self.assertEqual(min(fixture['foot_lowest_z_relative_to_ground_m']), 0.)

    def test_running_plant_is_rejected_before_any_state_write(self):
        initialize, _ = recorded_entry(TRIAL, self.candidate, MJCF)
        with self.assertRaisesRegex(RuntimeError, 'running plant'):
            initialize(SimpleNamespace(common_step_counter=1), None)

    def test_nonzero_action_history_cannot_be_silently_reset(self):
        trial = copy.deepcopy(json.loads(TRIAL.read_text()))
        trial['policy_frames'][0]['previous_EMA'][0] = .1
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'trial.json'
            path.write_text(json.dumps(trial))
            with self.assertRaisesRegex(ValueError, 'history'):
                recorded_entry(path, self.candidate, MJCF)


if __name__ == '__main__':
    unittest.main()
