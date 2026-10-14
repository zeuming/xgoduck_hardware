"""Timestamp-based kinematic extrapolation, with no model/gain/limit edits.

An explicit constant angular/joint velocity approximation. It is not a
freshness check, dynamics predictor or safety measurement. Callers retain the
original physical feedback and original stale-data/session/range interlocks.
"""
import numpy as np


def extrapolate(observation, joint_age_s, gravity_age_s):
    original=np.asarray(observation)
    if original.shape!=(1,61) or not np.isfinite(original).all():
        raise ValueError('One finite61D observation required')
    joint_age=np.broadcast_to(np.asarray(joint_age_s,dtype=float),(14,))
    if not np.isfinite(joint_age).all() or np.any(joint_age<0):
        raise ValueError('Finite nonnegative per-joint ages required')
    if not np.isfinite(gravity_age_s) or gravity_age_s<0:
        raise ValueError('Finite nonnegative gravity age required')
    if gravity_age_s==0 and np.all(joint_age==0):return original.copy()
    result=original.copy()
    result[0,6:20]=original[0,6:20]+original[0,20:34]*joint_age
    w=original[0,:3].astype(float)
    theta=np.linalg.norm(w)*gravity_age_s
    if theta!=0:
        axis=w/np.linalg.norm(w)
        gravity=original[0,3:6].astype(float)
        # A world-fixed vector in a body rotating at constant body angular
        # velocity: g(t+dt)=exp(-[omega]dt)g(t). Preserve measured magnitude.
        result[0,3:6]=(gravity*np.cos(theta)-np.cross(axis,gravity)*np.sin(theta)
            +axis*np.dot(axis,gravity)*(1-np.cos(theta)))
    return result
