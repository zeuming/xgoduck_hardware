"""Offline teacher: solve the swing sole pose using full simulated root state.

This uses ground-relative pose unavailable to the deployed proprioceptive
actor. It is NOT an executable real-robot controller or a trained policy.
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class SwingReference:
    def __init__(self,model_path,joint_names,joint_ids,side,root,actual,lower,upper):
        self.model=mujoco.MjModel.from_xml_path(str(model_path));self.data=mujoco.MjData(self.model)
        self.adr=np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in joint_names])
        self.index=np.array([i for i,sid in enumerate(joint_ids) if sid in range(10 if side=='left' else 20,15 if side=='left' else 25)])
        self.gid=self.model.geom(side+'_foot_collision').id
        self.lower=np.asarray(lower);self.upper=np.asarray(upper)
        self.previous=np.asarray(actual)[self.index].copy()
        self.place(root,actual);self.position=self.data.geom_xpos[self.gid].copy();self.rotation=self.data.geom_xmat[self.gid].reshape(3,3).copy()

    def place(self,root,q):
        self.data.qpos[:7]=root;self.data.qpos[self.adr]=q;mujoco.mj_forward(self.model,self.data)

    def solve(self,root,actual,height,preload):
        desired=self.position+np.array([0.,0.,height]);idx=self.index
        # Both desired configuration and resulting actuator target stay inside
        # the unchanged CAD/saved intersection. A fitting failure is rejected.
        low=np.maximum(self.lower[idx],self.lower[idx]-preload[idx])+1e-8
        high=np.minimum(self.upper[idx],self.upper[idx]-preload[idx])-1e-8
        if np.any(low>=high):return None,dict(success=False,reason='preload leaves no joint interval')
        q=np.asarray(actual).copy()
        def residual(values):
            q[idx]=values;self.place(root,q)
            position=self.data.geom_xpos[self.gid];rotation=self.data.geom_xmat[self.gid].reshape(3,3)
            orientation=Rotation.from_matrix(rotation@self.rotation.T).as_rotvec()
            return np.r_[(position-desired)/.1,orientation[:2]]
        fit=least_squares(residual,np.clip(self.previous,low+1e-10,high-1e-10),bounds=(low,high),max_nfev=35,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        error=residual(fit.x);position_error=float(np.linalg.norm(error[:3]*.1));orientation_error=float(np.linalg.norm(error[3:]))
        valid=bool(fit.success and position_error<=.0002 and orientation_error<=np.deg2rad(.2))
        evidence=dict(success=valid,nfev=fit.nfev,position_error_m=position_error,roll_pitch_error_rad=orientation_error,
            desired_height_increment_m=float(height),configuration_rad=fit.x.tolist(),cost=float(fit.cost))
        if not valid:return None,evidence
        self.previous=fit.x.copy();return fit.x+preload[idx],evidence
