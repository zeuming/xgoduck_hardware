"""Conditional initialization from a recorded policy sample, file-only.

This is not real-plant replay: encoder/CAD and IMU/body alignment are assumed.
Unmeasured base translation velocity and yaw are zero; ground height is chosen
from the lowest CAD foot vertex. Original startup physics and sensor noise stay
active. Only the initial reset is written, never a running trajectory.
"""
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from audit_pose import gravity_rotation, set_pose


def recorded_entry(path, candidate, mjcf):
    path, mjcf = Path(path), Path(mjcf)
    trial = json.loads(path.read_text())
    row = trial['policy_frames'][0]
    obs = np.asarray(row['submitted_observation'], dtype=float)
    if obs.shape != (1, 61) or not np.isfinite(obs).all():
        raise ValueError('Expected one finite recorded 61D policy observation')
    obs = obs[0]
    if np.any(obs[34:] != 0) or np.any(row['previous_EMA']):
        raise ValueError('This initializer requires an initial zero-command/action history')
    home = np.asarray(candidate['home_rad'])
    angles, velocities = obs[6:20]+home, obs[20:34]
    rotation = gravity_rotation(obs[3:6])
    np.testing.assert_allclose(rotation.T @ [0., 0., -1.],
                               obs[3:6]/np.linalg.norm(obs[3:6]), atol=1e-12)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(rotation).ravel())
    model = mujoco.MjModel.from_xml_path(str(mjcf.resolve()))
    data = mujoco.MjData(model)
    set_pose(model, data, dict(zip(candidate['joint_names'], angles)), rotation)
    lowest = []
    for name in ('left_foot_collision', 'right_foot_collision'):
        geom = model.geom(name).id
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start+count].astype(float)
        world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        lowest.append(float(world[:, 2].min()))
    root_z = float(data.qpos[2] - min(lowest))
    root_state = np.r_[0., 0., root_z, quaternion, np.zeros(3), rotation @ obs[:3]]
    provenance = dict(scope=__doc__, trial_path=str(path),
        implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        trial_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        mjcf_sha256=hashlib.sha256(mjcf.read_bytes()).hexdigest(),
        first_sample_us=row['observation_sample_us'],
        position_age_us=row['input_feedback_provenance']['position_ages_us'],
        joint_rad=angles.tolist(), joint_velocity_rad_s=velocities.tolist(),
        body_gravity=obs[3:6].tolist(), body_gyro_rad_s=obs[:3].tolist(),
        root_state_world=root_state.tolist(),
        foot_lowest_z_relative_to_ground_m=(np.asarray(lowest)-min(lowest)).tolist(),
        actor_input_is_not_forced_to_recording=True, measured_mass_distribution=False,
        unknown=['physical encoder-to-CAD zero/assembly', 'IMU-to-body alignment',
                 'support/contact and human forces', 'root linear velocity',
                 'BAM internal state and command/feedback delay history'])

    def initialize(env, env_ids):
        import torch
        if env.common_step_counter != 0:
            raise RuntimeError('Recorded entry cannot overwrite a running plant')
        asset = env.scene['robot']
        ids = (torch.arange(env.num_envs, device=env.device)
               if env_ids is None else env_ids)
        joints = torch.tensor([list(asset.joint_names).index(n)
                               for n in candidate['joint_names']], device=env.device)
        q = torch.tensor(angles, device=env.device, dtype=torch.float32).repeat(len(ids), 1)
        v = torch.tensor(velocities, device=env.device, dtype=torch.float32).repeat(len(ids), 1)
        state = torch.tensor(root_state, device=env.device,
                             dtype=torch.float32).repeat(len(ids), 1)
        state[:, :3] += env.scene.env_origins[ids]
        asset.write_joint_state_to_sim(q, v, joint_ids=joints, env_ids=ids)
        asset.write_root_state_to_sim(state, env_ids=ids)

    return initialize, provenance
