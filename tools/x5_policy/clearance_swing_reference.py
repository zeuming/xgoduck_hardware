"""Offline full-state teacher: sole clearance without a fixed foot orientation.

This changes the geometric task, not any physical limit. The complete sole must
clear the reference plane. The minimum joint displacement solution may rotate
or translate the foot; those quantities are recorded, never assumed constant.
"""
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from nonlinear_swing_reference_precise import SwingReference as PreciseReference


class SwingReference(PreciseReference):
    def solve(self, root, actual, height, preload):
        root, actual, preload = (np.asarray(v, dtype=np.float64) for v in (root, actual, preload))
        idx = self.index
        low = np.maximum(self.lower[idx], self.lower[idx]-preload[idx])+1e-8
        high = np.minimum(self.upper[idx], self.upper[idx]-preload[idx])-1e-8
        if np.any(low >= high):
            return None, dict(success=False, reason='preload leaves no joint interval')
        self.place(root, actual)
        original_position = self.data.geom_xpos[self.gid].copy()
        original_rotation = self.data.geom_xmat[self.gid].reshape(3, 3).copy()
        desired = self.initial_minimum+height
        q = actual.copy()
        def geometry(values):
            q[idx] = values
            self.place(root, q)
            pos = self.data.geom_xpos[self.gid]
            rot = self.data.geom_xmat[self.gid].reshape(3, 3)
            return self.vertices@rot.T+pos
        q0 = actual[idx].copy()
        fit = minimize(lambda v: float(np.sum((v-q0)**2)),
            np.clip(q0, low, high), jac=lambda v: 2*(v-q0), bounds=list(zip(low, high)),
            constraints=[dict(type='ineq', fun=lambda v: (geometry(v)[:, 2]-desired)/.1)],
            method='SLSQP', options=dict(maxiter=80, ftol=1e-11))
        vertices = geometry(fit.x)
        minimum = float(vertices[:, 2].min())
        valid = bool(fit.success and minimum >= desired-1e-7)
        evidence = dict(success=valid, nfev=int(fit.nfev), iterations=int(fit.nit),
            solver_message=str(fit.message), configuration_rad=fit.x.tolist(),
            desired_minimum_sole_height_m=float(desired), achieved_minimum_sole_height_m=minimum,
            sole_clearance_error_m=float(minimum-desired),
            foot_translation_m=(self.data.geom_xpos[self.gid]-original_position).tolist(),
            foot_rotation_rad=Rotation.from_matrix(self.data.geom_xmat[self.gid].reshape(3, 3)@original_rotation.T).as_rotvec().tolist(),
            maximum_joint_change_rad=float(abs(fit.x-q0).max()),
            reference='minimum joint displacement, every sole vertex above initial sole minimum plus lift profile; free foot XY/orientation')
        return (fit.x+preload[idx] if valid else None), evidence
