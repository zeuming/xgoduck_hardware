"""Known-command/delay replay through independent float64 physics and original BAM.

Changes physics engine as well as precision. This is a diagnostic cross-check,
not a selected replacement, a new walking controller or hardware validation.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path
import numpy as np


def main():
    os.environ.setdefault('MPLCONFIGDIR','/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from native_bam_prediction_branch import NativeBamPredictionBranch
    import diagnose_bam_control_tap as baseline
    out=Path(sys.argv[sys.argv.index('--output')+1]);count=0;branch=None;rows=[]
    old_step=ManagerBasedRlEnv.step;started=time.monotonic()
    def digest(env):
        motor=env.scene['robot'].actuators[0];term=env.action_manager.get_term('joint_pos')
        values=[env.sim.wp_data.qpos.numpy(),env.sim.wp_data.qvel.numpy(),
            env.sim.wp_model.dof_frictionloss.numpy(),env.sim.mj_model.body_mass,
            term._goal.numpy(),motor._prev_motor_torque.numpy(),torch.get_rng_state().numpy()]
        return hashlib.sha256(b''.join(v.tobytes() for v in values)).hexdigest()
    def observed(env, action):
        nonlocal count,branch
        if time.monotonic()-started>180:raise TimeoutError('180 second cross-engine diagnostic bound')
        if count>=250:
            before=digest(env)
            if branch is None:branch=NativeBamPredictionBranch(env)
            snapshot=branch.step(action.clone(),torch.get_rng_state().clone())
            measured=branch.measure()
            rows.append({k:np.asarray(v).copy() for k,v in {**snapshot,**measured}.items()})
            if digest(env)!=before:raise AssertionError('Native copy changed source plant/RNG')
        result=old_step(env,action);count+=1;return result
    ManagerBasedRlEnv.step=observed
    try:baseline.main()
    finally:ManagerBasedRlEnv.step=old_step
    arrays={k:np.stack([r[k] for r in rows]) for k in rows[0]}
    arrays.update({'substep_'+k:np.stack([r[k] for r in branch.integration_records])
        for k in branch.integration_records[0]})
    np.savez_compressed(out/'native-physics.npz',**arrays)
    base=Path(__file__).resolve().parents[2]/'output/duck_sim/overnight-20261011'
    original=np.load(base/'bam-tracking-control-tap-v2/generation-00.npz')
    current=np.load(out/'generation-00.npz')
    parity={k:bool(np.array_equal(original[k],current[k])) for k in original.files}
    assert all(parity.values())
    warp=np.load(base/'trajectory-convergence-v3/native10.npz')
    air=(arrays['height'][:,0]>=.003)&(arrays['load'][:,0]<.001)&(arrays['contact'][:,1]>0)
    report=dict(completed=True,scope=__doc__,hardware_access=False,walking_qualified=False,
        original_source_arrays_bitwise_equal=parity,frames=len(rows),initial_consistency=branch.initial_consistency,
        known_actions_and_delay_replay=True,source_plant_writes=0,original_BAM_and_action_methods=True,
        max_frame_force_balance_residual=float(abs(arrays['residual']).max()),
        max_substep_force_balance_residual=float(abs(arrays['substep_residual']).max()),
        frames_over_2e4_tolerance=int((abs(arrays['residual']).max(axis=1)>2e-4).sum()),
        substeps_over_2e4_tolerance=int((abs(arrays['substep_residual']).max(axis=1)>2e-4).sum()),
        max_swing_clearance_mm=float(arrays['height'][:,0].max()*1000),max_tilt_deg=float(arrays['tilt'].max()),
        air_frames=int(air.sum()),consecutive_air_pairs=int((air[1:]&air[:-1]).sum()),
        any_range_failed=bool(arrays['range_failed'].any()),any_self_collision=bool(arrays['self_collision'].any()),
        first_tilt_boundary_frame=next((i for i,v in enumerate(arrays['tilt']) if v>=15),None),
        maximum_difference_from_warp_native={k:float(abs(arrays[k]-warp[k]).max())
            for k in ('qpos','qvel','height','load','tilt','goal','ema','requested')},
        numerical_options=dict(iterations=branch.native_model.opt.iterations,
            tolerance=branch.native_model.opt.tolerance,ls_iterations=branch.native_model.opt.ls_iterations,
            ls_tolerance=branch.native_model.opt.ls_tolerance),
        wall_seconds=time.monotonic()-started,
        limitations=['Crosses engine implementation as well as numerical precision; not a dtype-only ablation.',
            'Native geometry and dynamics are float64; original motor and action math remain float32.',
            'Source raw commands and realized random-delay draws are known; this is not a future-control oracle.',
            'Contact and collision backend differences must be investigated before any model replacement.',
            'No physical gait or impossibility claim follows from this diagnostic.'])
    paths=[Path(__file__),Path(__file__).with_name('native_bam_prediction_branch.py'),
        Path(__file__).with_name('bam_prediction_branch.py'),Path(baseline.__file__)]
    report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    for p in paths:(out/p.name).write_bytes(p.read_bytes())
    (out/'native-physics-summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()
