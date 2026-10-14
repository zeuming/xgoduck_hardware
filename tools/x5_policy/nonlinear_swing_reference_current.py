"""Alternate offline teacher: current horizontal foot pose, commanded clearance.

Only a vertical-clearance reference is imposed. Initial-world XY/orientation
are not constrained as the body shifts. All CAD/saved bounds remain unchanged.
"""
import numpy as np
from nonlinear_swing_reference import SwingReference as InitialSwingReference


class SwingReference(InitialSwingReference):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        mid=int(self.model.geom_dataid[self.gid]);start=int(self.model.mesh_vertadr[mid]);n=int(self.model.mesh_vertnum[mid])
        self.vertices=np.asarray(self.model.mesh_vert[start:start+n],dtype=float)
        self.initial_minimum=float((self.vertices@self.rotation.T+self.position)[:,2].min())

    def solve(self,root,actual,height,preload):
        self.place(root,actual)
        pos=self.data.geom_xpos[self.gid].copy();rot=self.data.geom_xmat[self.gid].reshape(3,3).copy()
        minimum=float((self.vertices@rot.T+pos)[:,2].min())
        self.position=pos;self.rotation=rot;self.previous=np.asarray(actual)[self.index].copy()
        # Parent solves centre_z = current_z + delta. Orientation is maintained
        # for this local IK request, so this delta lifts every sole vertex.
        delta=self.initial_minimum+height-minimum
        target,evidence=super().solve(root,actual,delta,preload)
        evidence.update(reference='current XY and orientation, initial minimum sole height plus lift profile',desired_minimum_sole_height_m=float(self.initial_minimum+height),current_minimum_sole_height_m=minimum)
        return target,evidence
