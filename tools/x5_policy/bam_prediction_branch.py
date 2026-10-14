"""Isolated multi-step BAM plant prediction; never writes its source environment.

This is an offline full-state teacher building block, not a robot controller.
Only the existing single-robot CPU diagnostic with zero interval pushes is
supported. Known-action validation supplies each tick's random state to replay
the same exogenous delay realization; this is not foresight available on hardware.
"""
import copy
import dataclasses


def clone_warp(value):
    import numpy as np
    import warp as wp
    if isinstance(value, wp.array):
        return wp.clone(value)
    if dataclasses.is_dataclass(value):
        result = copy.copy(value)
        for field in dataclasses.fields(value):
            setattr(result, field.name, clone_warp(getattr(value, field.name)))
        return result
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, (tuple, list)):
        return type(value)(clone_warp(v) for v in value)
    if isinstance(value, dict):
        return {k: clone_warp(v) for k, v in value.items()}
    return value


class BamPredictionBranch:
    def __init__(self, env):
        import torch
        import warp as wp
        from mjlab.sim.sim_data import WarpBridge
        if env.num_envs != 1 or str(env.device) != 'cpu':
            raise ValueError('Only one-world offline CPU branch is verified')
        robot = env.scene['robot']
        if len(robot.actuators) != 1:
            raise ValueError('Unexpected actuator topology')
        if torch.any(robot.data.encoder_bias != 0):
            raise ValueError('Encoder bias differs from candidate')
        source = robot.actuators[0]
        self.model = clone_warp(env.sim.wp_model)
        self.data = clone_warp(env.sim.wp_data)
        assert self.data.qpos.ptr != env.sim.wp_data.qpos.ptr
        assert self.model.dof_frictionloss.ptr != env.sim.wp_model.dof_frictionloss.ptr
        self.bam = copy.copy(source)
        for key, value in vars(source).items():
            if key in ('entity', '_data', '_mjwarp_model', '_bam_model'):
                continue
            if isinstance(value, torch.Tensor):
                setattr(self.bam, key, value.detach().clone())
        self.bam._delay_buffer = copy.deepcopy(source._delay_buffer)
        self.bam._bam_model = copy.deepcopy(source._bam_model)
        # Passive loggers attach an instance closure that references the source
        # motor. Drop that observer in the COPY and use the original class law.
        self.bam._bam_model.actuator.__dict__.pop('compute_control', None)
        self.bam._data = WarpBridge(self.data)
        self.bam._mjwarp_model = WarpBridge(self.model, nworld=1)
        self.bam.entity = None  # compute/apply_delay must not reach the source.
        self.term = copy.copy(env.action_manager.get_term('joint_pos'))
        for key, value in vars(self.term).items():
            if isinstance(value, torch.Tensor):
                setattr(self.term, key, value.detach().clone())
        self.term._entity = None  # only the original pure process_actions is used.
        self.qadr = robot.indexing.joint_q_adr.clone()
        self.vadr = robot.indexing.joint_v_adr.clone()
        self.target = robot.data.joint_pos_target.clone()
        self.vel_target = robot.data.joint_vel_target.clone()
        self.effort_target = robot.data.joint_effort_target.clone()
        self.decimation = env.cfg.decimation
        self.rng = torch.get_rng_state().clone()
        self._wp = wp
        self.root_body_id = robot.indexing.root_body_id
        self.gravity_vec = robot.data.gravity_vec_w.clone()
        self.feet = []
        for side in ('left', 'right'):
            import mujoco
            model = env.sim.mj_model
            gid = next(i for i in range(model.ngeom) if
                (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').split('/')[-1] == side+'_foot_collision')
            mid = model.geom_dataid[gid]
            start, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            self.feet.append((gid, model.mesh_vert[start:start+n].copy()))
        self.contacts = copy.copy(env.scene['feet_ground_contact'])
        self.contacts._slots = [copy.copy(v) for v in self.contacts._slots]
        for slot in self.contacts._slots:
            sensor = env.sim.mj_model.sensor(slot.sensor_name)
            start, dim = int(sensor.adr[0]), int(sensor.dim[0])
            slot.data_view = wp.to_torch(self.data.sensordata)[:, start:start+dim]
        self.contacts._data = self.bam._data
        self.collision = copy.copy(env.scene['self_collision'])
        self.collision._slots = [copy.copy(v) for v in self.collision._slots]
        for slot in self.collision._slots:
            sensor = env.sim.mj_model.sensor(slot.sensor_name)
            start, dim = int(sensor.adr[0]), int(sensor.dim[0])
            slot.data_view = wp.to_torch(self.data.sensordata)[:, start:start+dim]
        self.collision._data = self.bam._data

    def step(self, raw_action, random_state=None):
        import torch
        import mujoco_warp as mjwarp
        from mjlab.actuator.actuator import ActuatorCmd
        # Delay sampling uses an independent RNG stream, or the explicitly
        # provided known-delay replay stream. Restore caller RNG unconditionally.
        with torch.random.fork_rng(devices=[]):
            torch.set_rng_state(self.rng if random_state is None else random_state)
            self.term.process_actions(raw_action)
            for _ in range(self.decimation):
                q = self._wp.to_torch(self.data.qpos)[:, self.qadr]
                v = self._wp.to_torch(self.data.qvel)[:, self.vadr]
                actual = q[:, self.term._target_ids]
                self.term.range_failed |= (~torch.isfinite(actual)).any(-1) | (
                    (actual < self.term._low) | (actual > self.term._high)).any(-1)
                self.target[:, self.term._target_ids] = self.term._processed_actions
                ids = self.bam.target_ids
                cmd = ActuatorCmd(self.target[:, ids], self.vel_target[:, ids],
                                  self.effort_target[:, ids], q[:, ids], v[:, ids])
                cmd = self.bam.apply_delay(cmd)
                torque = self.bam.compute(cmd)
                self._wp.to_torch(self.data.ctrl)[:, self.bam.global_ctrl_ids] = torque
                mjwarp.step(self.model, self.data)
                self.bam.update(self.bam._dt)
            # Match ManagerBasedRlEnv's final post-integration forward exactly.
            mjwarp.forward(self.model, self.data)
            self.rng = torch.get_rng_state().clone()
        return self.snapshot()

    def snapshot(self):
        result = {key: getattr(self.data, key).numpy().copy() for key in
                  ('qpos', 'qvel', 'qacc', 'ctrl', 'qfrc_constraint', 'geom_xpos', 'geom_xmat')}
        for key, value in [('ema', self.term._ema), ('goal', self.term._goal),
                           ('range_failed', self.term.range_failed),
                           ('motor_torque', self.bam._prev_motor_torque),
                           ('motor_target', self.bam._bam_model.actuator.q_target_smooth)]:
            result[key] = self.bam._as_tensor(value).numpy().copy()
        return result

    def measure(self):
        import numpy as np
        import torch
        from mjlab.utils.lab_api.math import quat_apply_inverse
        qpos = self.data.qpos.numpy()[0]
        quat = self._wp.to_torch(self.data.xquat)[:, self.root_body_id]
        gravity = quat_apply_inverse(quat, self.gravity_vec)
        tilt = torch.rad2deg(torch.acos((-gravity[:,2]/gravity.norm(dim=1)).clamp(-1,1)))[0].item()
        heights = []; positions = []
        for gid, vertices in self.feet:
            pos = self.data.geom_xpos.numpy()[0, gid]
            rotation = self.data.geom_xmat.numpy()[0, gid].reshape(3, 3)
            heights.append(float((vertices@rotation.T+pos)[:, 2].min()))
            positions.append(pos.copy())
        contact = self.contacts._extract_sensor_data()
        collision = self.collision._extract_sensor_data().found
        actual = qpos[self.qadr.numpy()][self.term._target_ids.numpy()]
        return dict(height=np.asarray(heights), positions=np.asarray(positions),
            load=abs(contact.force.numpy()[0, :, 2]), contact=contact.found.numpy()[0].copy(),
            tilt=float(tilt), qpos=qpos.copy(), actual=actual.copy(),
            range_failed=bool(self.term.range_failed.any()), self_collision=bool((collision > 0).any()))
