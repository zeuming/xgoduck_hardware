"""Explicit file-only virtual-body mapping for a geometrically solved stance.

R_physical = R_virtual @ rotation, q_physical = q_virtual + joint_offset.
This is a controller hypothesis, not an IMU calibration or a physical model
identity: the rear payload inertia and contact dynamics are still different.
Real safety checks must always consume the original physical measurements.
"""
import numpy as np


class BodyFramePoseMap:
    def __init__(self, rotation, joint_offset):
        self.rotation = np.asarray(rotation, dtype=np.float64)
        self.offset = np.asarray(joint_offset, dtype=np.float64)
        if self.rotation.shape != (3, 3) or self.offset.shape != (14,):
            raise ValueError('Expected 3x3 rotation and14joint offsets')
        if not np.isfinite(self.rotation).all() or not np.isfinite(self.offset).all():
            raise ValueError('Nonfinite body-frame map')
        np.testing.assert_allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-12, rtol=0)
        if abs(np.linalg.det(self.rotation)-1) > 1e-12:
            raise ValueError('A proper rotation is required')

    def observations(self, physical):
        values = np.asarray(physical)
        if values.shape[-1] != 61:
            raise ValueError('Expected the original61D observation')
        result = values.copy()
        # Body vectors change basis; joint velocities retain the identity
        # derivative of a constant joint offset. Action/command history stays raw.
        result[..., :3] = values[..., :3] @ self.rotation.T
        result[..., 3:6] = values[..., 3:6] @ self.rotation.T
        result[..., 6:20] = values[..., 6:20] - self.offset
        return result

    def goals(self, virtual):
        values = np.asarray(virtual)
        if values.shape[-1] != 14:
            raise ValueError('Expected14policy joints')
        return values + self.offset
