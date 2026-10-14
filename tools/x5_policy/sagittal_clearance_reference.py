"""Offline clearance IK that preserves the separately planned lateral transfer.

Hip yaw/roll actuator requests come from the original preparation reference.
Only hip pitch, knee and ankle are optimized for whole-sole clearance. This
does not change bounds, gains or dynamics, and is not a hardware controller.
"""
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from nonlinear_swing_reference_precise import SwingReference as PreciseReference


class SwingReference(PreciseReference):
    def solve(self, root, actual, height, preload, target_request):
        root, actual, preload, target_request = (np.asarray(v, dtype=np.float64)
            for v in (root, actual, preload, target_request))
        idx = self.index
        low = np.maximum(self.lower[idx], self.lower[idx]-preload[idx])+1e-8
        high = np.minimum(self.upper[idx], self.upper[idx]-preload[idx])-1e-8
        # The kinematic configuration excludes the retained preload. Returning
        # it plus preload reproduces the original yaw/roll requests exactly.
        fixed = target_request[idx[:2]]-preload[idx[:2]]
        if np.any(low >= high) or np.any(fixed < low[:2]) or np.any(fixed > high[:2]):
            return None, dict(success=False, reason='fixed lateral reference outside original configuration/target interval')
        self.place(root, actual)
        original_position = self.data.geom_xpos[self.gid].copy()
        original_rotation = self.data.geom_xmat[self.gid].reshape(3,3).copy()
        q = actual.copy();q[idx[:2]] = fixed
        desired = self.initial_minimum+height
        def geometry(values):
            q[idx[2:]] = values
            self.place(root,q)
            return self.vertices@self.data.geom_xmat[self.gid].reshape(3,3).T+self.data.geom_xpos[self.gid]
        q0 = actual[idx[2:]].copy()
        fit = minimize(lambda v:float(np.sum((v-q0)**2)), np.clip(q0,low[2:],high[2:]),
            jac=lambda v:2*(v-q0),bounds=list(zip(low[2:],high[2:])),method='SLSQP',
            constraints=[dict(type='ineq',fun=lambda v:(geometry(v)[:,2]-desired)/.1)],
            options=dict(maxiter=80,ftol=1e-11))
        points = geometry(fit.x);minimum=float(points[:,2].min())
        config = np.r_[fixed,fit.x]
        valid=bool(fit.success and minimum>=desired-1e-7)
        e=dict(success=valid,nfev=int(fit.nfev),iterations=int(fit.nit),solver_message=str(fit.message),
            configuration_rad=config.tolist(),fixed_yaw_roll_request_rad=target_request[idx[:2]].tolist(),
            desired_minimum_sole_height_m=float(desired),achieved_minimum_sole_height_m=minimum,
            sole_clearance_error_m=float(minimum-desired),maximum_joint_change_rad=float(abs(config-actual[idx]).max()),
            foot_translation_m=(self.data.geom_xpos[self.gid]-original_position).tolist(),
            foot_rotation_rad=Rotation.from_matrix(self.data.geom_xmat[self.gid].reshape(3,3)@original_rotation.T).as_rotvec().tolist(),
            reference='Original yaw/roll transfer request retained; minimum pitch/knee/ankle change with every sole vertex above clearance plane')
        result=config+preload[idx]
        result[:2]=target_request[idx[:2]]
        return (result if valid else None),e
