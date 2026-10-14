"""Extra read-only diagnostic fields; calls the original raw-contact verifier.

Load the original measurement source with the existing audited bundle layout.
No root-state writes, reward edits, random draws, targets or simulator steps.
"""
import importlib.util
from pathlib import Path
import math

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'
spec=importlib.util.spec_from_file_location('original_raw_contact_measurement',SOURCE)
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)


def actual_plant_measurements(env):
    import torch
    import warp as wp
    from slow_gait_experiment import sole_heights
    result=original.actual_plant_measurements(env)
    masses=wp.to_torch(env.sim.wp_model.body_mass)
    if masses.ndim==1:masses=masses[None].expand(env.num_envs,-1)
    com=(masses[:,:,None]*env.sim.data.xipos).sum(1)/masses.sum(-1)[:,None]
    sole_heights(env)
    centers=torch.stack([env.sim.data.geom_xpos[:,gid] for gid,_ in env._slow_sole_geometry],dim=1)
    term=env.action_manager.get_term('joint_pos')
    goal=term._processed_actions[0]
    actual=term._entity.data.joint_pos[0,term._target_ids]
    result.update(actual_total_mass_kg=float(masses[0].sum()),whole_body_com_m=com[0].tolist(),
        foot_collision_centers_m=centers[0].tolist(),goal_rad=goal.tolist(),
        requested_rad=term.requested_targets[0].tolist() if hasattr(term,'requested_targets') else None,
        ema_rad=term._ema[0].tolist(),
        actual_joint_rad=actual.tolist(),goal_tracking_error_deg=((goal-actual)*180/math.pi).tolist())
    return result
