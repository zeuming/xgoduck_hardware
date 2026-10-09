"""Checks independent offline FK against MuJoCo, including non-axis-aligned pivots."""
import unittest

import mujoco as mj
import numpy as np

from audit_pose import (axis_rotation, forward_kinematics, gravity_rotation,
                        snapshot_angles, snapshot_hardware)
from rl_core import POLICY_IDS


class PoseAuditTests(unittest.TestCase):
    def test_general_joint_axes_pivots_refs_and_ball_against_mujoco(self):
        model = mj.MjModel.from_xml_string('''<mujoco><compiler angle="radian"/>
          <worldbody><body name="root" pos=".1 .2 .3"><freejoint/>
            <geom type="sphere" size=".02" mass="1"/>
            <body name="hinge" pos=".02 .05 -.07" quat=".9 .1 .2 .3">
              <joint axis="1 2 -3" pos=".01 -.02 .03" ref=".21"/>
              <geom type="sphere" size=".02" mass=".1"/>
              <body name="slide" pos="-.03 .02 .04" quat=".8 -.2 .1 .4">
                <joint type="slide" axis="-2 3 1" ref="-.04"/>
                <geom type="sphere" size=".02" mass=".1"/>
                <body name="ball" pos=".01 -.06 .03" quat=".8 .3 -.1 .2">
                  <joint type="ball" pos=".02 .01 -.03"/>
                  <geom type="sphere" size=".02" mass=".1"/>
                </body>
              </body>
            </body>
          </body></worldbody></mujoco>''')
        data, rng = mj.MjData(model), np.random.default_rng(625)
        for _ in range(25):
            data.qpos[:] = model.qpos0
            data.qpos[:3] = rng.uniform(-.3, .3, 3)
            data.qpos[3:7] = rng.normal(size=4)
            data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
            data.qpos[7:9] = rng.uniform(-.5, .5, 2)
            data.qpos[9:13] = rng.normal(size=4)
            data.qpos[9:13] /= np.linalg.norm(data.qpos[9:13])
            mj.mj_forward(model, data)
            rotations, positions = forward_kinematics(model, data.qpos)
            np.testing.assert_allclose(rotations, data.xmat.reshape(-1, 3, 3), atol=1e-12)
            np.testing.assert_allclose(positions, data.xpos, atol=1e-12)

    def test_gravity_rotation_direction_and_antipodal_case(self):
        for roll, pitch in [(0, 0), (10, 0), (0, -10), (7, -13), (179, 0)]:
            rotation = axis_rotation([0, 1, 0], np.radians(pitch)) @ axis_rotation([1, 0, 0], np.radians(roll))
            gravity = np.einsum('ji,j->i', rotation, [0, 0, -1])
            result = gravity_rotation(gravity)
            np.testing.assert_allclose(np.einsum('ij,j->i', result, gravity), [0, 0, -1], atol=1e-12)
            np.testing.assert_allclose(result.T @ result, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(gravity_rotation([0, 0, 1]) @ [0, 0, 1], [0, 0, -1], atol=1e-12)

    def test_recompute_angles_rejects_mismatched_calibration(self):
        hardware = dict(joints=[dict(id=sid, zero=2000, sign=-1) for sid in POLICY_IDS],
                        rawdata=[dict(id=sid, position=2050, angle_deg=-50*360/4095) for sid in POLICY_IDS])
        snapshot = dict(motion=dict(hardware=hardware))
        section, found = snapshot_hardware(snapshot)
        self.assertEqual(section, 'motion.hardware')
        angles, rows = snapshot_angles(snapshot, found)
        self.assertEqual(len(angles), 14)
        self.assertEqual(len(rows), 14)
        hardware['joints'][0]['sign'] = 1
        with self.assertRaisesRegex(ValueError, 'disagree'):
            snapshot_angles(snapshot, found)


if __name__ == '__main__':
    unittest.main()
