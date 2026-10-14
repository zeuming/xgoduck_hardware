"""Independent float64 MuJoCo cross-check of the original BAM action chain.

This is an ENGINE cross-check, not a claim that changing dtype alone explains
differences. Motor/action math stays original Torch float32. No source writes.
"""
import copy
from types import SimpleNamespace
import numpy as np
from bam_prediction_branch import BamPredictionBranch


class NativeBamPredictionBranch(BamPredictionBranch):
    def __init__(self, env):
        import mujoco
        import mujoco_warp as mjwarp
        super().__init__(env)
        self.native_model = copy.copy(env.sim.mj_model)
        self.native_data = mujoco.MjData(self.native_model)
        m, d = self.native_model, self.native_data
        # These are the per-world physical fields varied by this candidate or
        # updated by BAM. Unmodified geometry/joint/actuator/sensor topology is
        # copied from the exact environment's compiled model, not a CAD reload.
        fields = ('body_mass','body_ipos','body_iquat','body_inertia','geom_friction',
                  'dof_armature','dof_frictionloss','dof_damping')
        self.physical_fields = {}
        for name in fields:
            v=getattr(self.model,name).numpy()[0]
            target=getattr(m,name)
            if np.shares_memory(target,getattr(env.sim.mj_model,name)):
                raise AssertionError(f'Native model copy shares source memory: {name}')
            if target.shape != v.shape:
                raise ValueError(f'Unsupported native physical field shape: {name}')
            target[:] = v
            self.physical_fields[name]=v.copy()
        m.opt.timestep=float(self.model.opt.timestep.numpy()[0])
        m.opt.solver=int(self.model.opt.solver)
        m.opt.cone=int(self.model.opt.cone)
        m.opt.integrator=int(self.model.opt.integrator)
        m.opt.iterations=100
        m.opt.ls_iterations=100
        m.opt.tolerance=1e-10
        m.opt.ls_tolerance=1e-4
        mujoco.mj_setConst(m,d)
        # Copy previous force/constraint history as well as positions/velocities.
        # BAM must see the same previous solve on the first substep.
        mjwarp.get_data_into(d,m,self.data,world_id=0)
        for name in ('qpos','qvel','qacc','qfrc_bias','qfrc_constraint','qfrc_actuator'):
            if not np.array_equal(getattr(d,name),getattr(self.data,name).numpy()[0]):
                raise AssertionError(f'Native initial history differs: {name}')
        probe=mujoco.MjData(m)
        mjwarp.get_data_into(probe,m,self.data,world_id=0)
        mujoco.mj_forward(m,probe)
        matrix=np.empty((m.nv,m.nv));mujoco.mj_fullM(m,probe,matrix)
        self.initial_consistency=dict(
            maximum_mass_matrix_error=float(abs(matrix-self.data.qM.numpy()[0,:m.nv,:m.nv]).max()),
            maximum_geometry_position_error=float(abs(probe.geom_xpos-self.data.geom_xpos.numpy()[0]).max()),
            maximum_bias_force_error=float(abs(probe.qfrc_bias-self.data.qfrc_bias.numpy()[0]).max()),
            body_mass_total_kg=float(m.body_mass.sum()),
            copied_force_history_exact=True)
        if self.initial_consistency['maximum_mass_matrix_error']>2e-7 or self.initial_consistency['maximum_geometry_position_error']>2e-7:
            raise AssertionError('Native reconstruction differs from effective plant')
        self.integration_records=[]

    def sync_bam_history(self):
        import torch
        d=self.native_data
        def f(a):return torch.from_numpy(np.asarray(a,dtype=np.float32).copy()[None,:])
        efc=SimpleNamespace(type=torch.from_numpy(d.efc_type.copy()[None,:]),
            id=torch.from_numpy(d.efc_id.copy()[None,:]),force=f(d.efc_force))
        self.bam._data=SimpleNamespace(efc=efc,nefc=torch.tensor([d.nefc]),
            qfrc_bias=f(d.qfrc_bias),qfrc_constraint=f(d.qfrc_constraint),
            qfrc_actuator=f(d.qfrc_actuator))

    def numeric_record(self):
        import mujoco
        m,d=self.native_model,self.native_data
        matrix=np.empty((m.nv,m.nv));mujoco.mj_fullM(m,d,matrix)
        return dict(residual=matrix@d.qacc-d.qfrc_smooth-d.qfrc_constraint,
            qacc=d.qacc.copy(),qfrc_constraint=d.qfrc_constraint.copy(),
            solver_niter=int(d.solver_niter[0]))

    def step(self, raw_action, random_state=None):
        import torch
        import mujoco
        from mjlab.actuator.actuator import ActuatorCmd
        m,d=self.native_model,self.native_data
        with torch.random.fork_rng(devices=[]):
            torch.set_rng_state(self.rng if random_state is None else random_state)
            self.term.process_actions(raw_action)
            for _ in range(self.decimation):
                self.sync_bam_history()
                q=torch.from_numpy(d.qpos[self.qadr.numpy()].astype(np.float32)[None,:])
                v=torch.from_numpy(d.qvel[self.vadr.numpy()].astype(np.float32)[None,:])
                actual=q[:,self.term._target_ids]
                self.term.range_failed |= (~torch.isfinite(actual)).any(-1) | (
                    (actual<self.term._low)|(actual>self.term._high)).any(-1)
                self.target[:,self.term._target_ids]=self.term._processed_actions
                ids=self.bam.target_ids
                cmd=ActuatorCmd(self.target[:,ids],self.vel_target[:,ids],
                    self.effort_target[:,ids],q[:,ids],v[:,ids])
                cmd=self.bam.apply_delay(cmd)
                torque=self.bam.compute(cmd)
                d.ctrl[self.bam.global_ctrl_ids.numpy()]=torque.numpy()[0]
                m.dof_frictionloss[:]=self.model.dof_frictionloss.numpy()[0]
                m.dof_damping[:]=self.model.dof_damping.numpy()[0]
                mujoco.mj_step(m,d)
                self.integration_records.append(self.numeric_record())
                self.bam.update(self.bam._dt)
            mujoco.mj_forward(m,d)
            self.rng=torch.get_rng_state().clone()
        return self.native_snapshot()

    def native_snapshot(self):
        d=self.native_data
        return dict(qpos=d.qpos.copy(),qvel=d.qvel.copy(),
            goal=self.term._goal.numpy()[0].copy(),ema=self.term._ema.numpy()[0].copy(),
            requested=self.term.requested_targets.numpy()[0].copy(),**self.numeric_record())

    def measure(self):
        # Reuse the same contact-sensor extraction with native sensor values.
        # These copies belong only to this predictor, never the original plant.
        for name in ('qpos','qvel','geom_xpos','geom_xmat','xquat','sensordata'):
            a=getattr(self.data,name)
            native=getattr(self.native_data,name)
            value=native.reshape(a.numpy().shape[1:]).astype(a.numpy().dtype)[None,...]
            a.assign(value)
        measurement=super().measure()
        # Foot geometry and actual joint positions use native float64 here.
        d=self.native_data
        measurement['height']=np.array([float((vertices.astype(float)@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid])[:,2].min())
            for gid,vertices in self.feet])
        measurement['qpos']=d.qpos.copy()
        measurement['actual']=d.qpos[self.qadr.numpy()][self.term._target_ids.numpy()].copy()
        return measurement
