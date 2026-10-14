"""Use float64 IK variables; simulator float32 measurements stay unchanged.

SciPy perturbs double-precision optimization variables for numerical Jacobians.
Storing those perturbations in a float32 temporary can erase the derivatives.
"""
import numpy as np
from nonlinear_swing_reference_current import SwingReference as CurrentSwingReference


class SwingReference(CurrentSwingReference):
    def solve(self,root,actual,height,preload):
        return super().solve(np.asarray(root,dtype=np.float64),np.asarray(actual,dtype=np.float64),height,np.asarray(preload,dtype=np.float64))
