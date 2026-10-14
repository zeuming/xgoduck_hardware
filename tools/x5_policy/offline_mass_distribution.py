"""Explicit hypothetical inertial changes for file-only policy comparisons.

No calibration, MJCF source file, controller or hardware parameter is written.
Uniform scaling is an assumption about the unmeasured mass distribution, not a
measurement. Relocation treats a specified part of the trunk mass as a point
initially at its COM, then moves it behind the hip-yaw midpoint at the same
height unless an explicit, unmeasured height hypothesis is supplied. The total
mass is preserved and the parallel-axis tensor is updated.
"""
import copy
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


def inertia_matrix(model, body_id):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body_id])
    rotation = rotation.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T


def parallel_axis(mass, displacement):
    d = np.asarray(displacement, dtype=float)
    return mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def relocate_point(mass, com, tensor, part_mass, destination):
    """Retain total mass and the background inertia, relocate one COM point."""
    com, destination = np.asarray(com, dtype=float), np.asarray(destination, dtype=float)
    tensor = np.asarray(tensor, dtype=float)
    if not 0 <= part_mass < mass or not np.isfinite([mass, part_mass]).all():
        raise ValueError('Relocated mass must be finite and smaller than trunk mass')
    if not np.isfinite(com).all() or not np.isfinite(destination).all():
        raise ValueError('Nonfinite COM or destination')
    new_com = com + part_mass / mass * (destination - com)
    new_tensor = (tensor + parallel_axis(mass - part_mass, com - new_com)
                  + parallel_axis(part_mass, destination - new_com))
    if np.linalg.eigvalsh(new_tensor).min() <= 0:
        raise ValueError('Invalid inertial tensor')
    return new_com, new_tensor


def set_inertial(body, mass, com, tensor):
    body.mass = mass
    body.ipos = com
    # Use the symmetric eigensolver directly: compiler fullinertia conversion
    # can lose off-diagonal precision. Preserve the strict reconstruction check.
    eigenvalues, rotation = np.linalg.eigh(tensor)
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    body.fullinertia = np.array([np.nan, 0., 0., 0., 0., 0.])
    body.inertia = eigenvalues
    body.iquat = quaternion
    body.explicitinertial = True


def scenario_spec(original_spec_fn, scenario_path):
    scenario_path = Path(scenario_path)
    scenario = json.loads(scenario_path.read_text())
    total, moved = scenario['total_mass_kg'], scenario['relocated_mass_kg']
    rear = scenario.get('rear_of_hip_m')
    height = scenario.get('height_above_original_trunk_com_m', 0.)
    if not np.isfinite(total) or total <= 0:
        raise ValueError('Invalid scenario total mass')
    if moved and (rear is None or not np.isfinite(rear) or rear < 0):
        raise ValueError('Specify a finite rear displacement')
    if not np.isfinite(height):
        raise ValueError('Specify a finite height hypothesis')
    evidence = dict(scenario=copy.deepcopy(scenario), scenario_sha256=hashlib.sha256(
        scenario_path.read_bytes()).hexdigest(), hardware_access=False,
        measured_mass_distribution=False, physics_application=None)

    def build():
        spec = original_spec_fn()
        before = spec.compile()
        scale = total / before.body_mass.sum()
        for i in range(1, before.nbody):
            body = spec.body(before.body(i).name)
            set_inertial(body, float(before.body_mass[i] * scale), before.body_ipos[i],
                         inertia_matrix(before, i) * scale)
        trunk_id = before.body('trunk_base').id
        old_mass = float(before.body_mass[trunk_id] * scale)
        old_com = before.body_ipos[trunk_id].copy()
        old_tensor = inertia_matrix(before, trunk_id) * scale
        hip_x = float(np.mean([spec.body(n).pos[0] for n in ('yaw2roll', 'bearing_roll')]))
        destination = old_com.copy()
        if moved:
            destination[0] = hip_x - rear
            destination[2] += height
        new_com, new_tensor = relocate_point(old_mass, old_com, old_tensor, moved, destination)
        set_inertial(spec.body('trunk_base'), old_mass, new_com, new_tensor)
        after = spec.compile()
        np.testing.assert_allclose(after.body_mass.sum(), total, atol=1e-12, rtol=0)
        np.testing.assert_allclose(after.body_mass, before.body_mass * scale, atol=1e-12, rtol=0)
        np.testing.assert_allclose(after.body_ipos[trunk_id], new_com, atol=1e-12, rtol=0)
        np.testing.assert_allclose(inertia_matrix(after, trunk_id), new_tensor, atol=1e-12, rtol=0)
        # Physical shape, joint geometry, reference angles and servo viewer fields
        # remain identical. The BAM configuration is outside this function.
        for field in ('body_pos', 'body_quat', 'jnt_pos', 'jnt_axis', 'jnt_range', 'qpos0',
                      'geom_pos', 'geom_quat', 'geom_size', 'geom_contype', 'geom_conaffinity',
                      'actuator_gainprm', 'actuator_biasprm', 'dof_armature', 'dof_damping'):
            np.testing.assert_array_equal(getattr(before, field), getattr(after, field))
        for i in range(1, before.nbody):
            if i != trunk_id:
                np.testing.assert_allclose(after.body_ipos[i], before.body_ipos[i], atol=1e-12, rtol=0)
                np.testing.assert_allclose(inertia_matrix(after, i), inertia_matrix(before, i) * scale,
                                           atol=1e-12, rtol=0)
        evidence['physics_application'] = dict(
            source_nominal_mass_kg=float(before.body_mass.sum()), total_nominal_mass_kg=float(after.body_mass.sum()),
            uniform_scale=float(scale), trunk_body_id=trunk_id, trunk_mass_kg=old_mass,
            hip_yaw_midpoint_x_m=hip_x, original_trunk_com_m=old_com.tolist(),
            relocated_point_m=destination.tolist(), new_trunk_com_m=new_com.tolist(),
            original_trunk_inertia_kg_m2=old_tensor.tolist(), new_trunk_inertia_kg_m2=new_tensor.tolist(),
            noninertial_fields_identical=True, tensor_reconstruction_verified=True,
            height_assumption=f'{height}m above original trunk COM, same lateral position; not measured from photo',
            relocation_assumption='specified existing point mass initially at trunk COM; not an identified board model')
        return spec

    return build, evidence
