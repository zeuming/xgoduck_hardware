"""Paired, open-loop numerical convergence on isolated BAM prediction copies.

All copies receive the source's raw actions and known per-tick delay RNG.
Only numerical solver options differ. Copies never write into the source.
This is neither a new controller nor a hardware/walking acceptance test.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    import mujoco_warp as mjwarp
    from mujoco_warp._src import solver
    from mjlab.envs import ManagerBasedRlEnv
    from bam_prediction_branch import BamPredictionBranch
    import diagnose_bam_control_tap as baseline

    output = Path(sys.argv[sys.argv.index('--output')+1])
    original_env_step = ManagerBasedRlEnv.step
    original_warp_step = mjwarp.step
    original_solve = solver._solve
    branches, samples, integrated, configs = {}, {}, {}, {}
    latest_solver = {}
    count = 0
    started = time.monotonic()

    def solve_tap(model, data, context):
        original_solve(model, data, context)
        if not any(data is b.data for b in branches.values()):
            return
        factor = float(model.stat.meaninertia.numpy()[0])*int(model.nv)
        cost, previous = float(context.cost.numpy()[0]), float(context.prev_cost.numpy()[0])
        latest_solver[id(data)] = dict(solver_cost=cost, solver_previous_cost=previous,
            solver_scaled_improvement=(previous-cost)/factor,
            solver_scaled_gradient=float(np.sqrt(context.grad_dot.numpy()[0]))/factor,
            solver_gradient=context.grad.numpy()[0].copy())

    def digest(env):
        motor = env.scene['robot'].actuators[0]
        term = env.action_manager.get_term('joint_pos')
        arrays = [env.sim.wp_data.qpos.numpy(), env.sim.wp_data.qvel.numpy(),
                  env.sim.wp_model.dof_frictionloss.numpy(), term._goal.numpy(),
                  motor._prev_motor_torque.numpy(), torch.get_rng_state().numpy(),
                  env.sim.wp_model.opt.tolerance.numpy(), env.sim.wp_model.opt.ls_tolerance.numpy()]
        return hashlib.sha256(b''.join(a.tobytes() for a in arrays)).hexdigest()

    def numeric_record(branch):
        data = branch.data
        nv = data.qvel.shape[1]
        matrix = data.qM.numpy()[0, :nv, :nv].astype(float)
        if not np.allclose(matrix, matrix.T, atol=2e-7):
            raise ValueError('Expected dense symmetric mass matrix')
        acc = data.qacc.numpy()[0].astype(float)
        smooth = data.qfrc_smooth.numpy()[0].astype(float)
        constraint = data.qfrc_constraint.numpy()[0].astype(float)
        load = abs(branch.contacts._extract_sensor_data().force.numpy()[0, :, 2])
        result = dict(residual=matrix@acc-smooth-constraint,
                    qacc=acc, qfrc_constraint=constraint, load=load,
                    solver_niter=data.solver_niter.numpy()[0].copy())
        result.update(latest_solver[id(data)])
        return result

    def observe(env, action):
        nonlocal count
        if time.monotonic()-started > 240:
            raise TimeoutError('240 second full-trajectory diagnostic bound')
        if count == 250:
            before = digest(env)
            for name in ('native10', 'cap100', 'strict100'):
                b = BamPredictionBranch(env)
                if name != 'native10':
                    b.model.opt.iterations = 100
                if name == 'strict100':
                    b.model.opt.tolerance.fill_(1e-8)
                    b.model.opt.ls_iterations = 100
                    b.model.opt.ls_tolerance.fill_(1e-4)
                branches[name] = b
                samples[name] = []
                integrated[name] = []
                configs[name] = dict(iterations=int(b.model.opt.iterations),
                    solver=int(b.model.opt.solver), cone=int(b.model.opt.cone),
                    integrator=int(b.model.opt.integrator),
                    ls_iterations=int(b.model.opt.ls_iterations),
                    tolerance=b.model.opt.tolerance.numpy().tolist(),
                    ls_tolerance=b.model.opt.ls_tolerance.numpy().tolist(),
                    dt=float(b.model.opt.timestep.numpy()[0]), decimation=b.decimation,
                    initial_qpos=b.data.qpos.numpy().tolist())
            if digest(env) != before:
                raise AssertionError('Copy construction/options changed original plant')
        if count >= 250:
            before = digest(env)
            rng = torch.get_rng_state().clone()
            for name, b in branches.items():
                def tap(model, data):
                    if model is not b.model or data is not b.data:
                        raise AssertionError('Numerical tap reached unexpected plant')
                    original_warp_step(model, data)
                    # Forward solved these forces before this substep's
                    # integration. This is the force sample used for impulse;
                    # final 20ms forward is recorded separately below.
                    integrated[name].append(numeric_record(b))
                mjwarp.step = tap
                try:
                    state = b.step(action.clone(), rng)
                finally:
                    mjwarp.step = original_warp_step
                measured = b.measure()
                row = numeric_record(b)
                row.update({k: np.asarray(v).copy() for k, v in measured.items()})
                row.update(goal=state['goal'][0], ema=state['ema'][0],
                           requested=b.term.requested_targets.numpy()[0].copy(),
                           qvel=state['qvel'][0])
                samples[name].append(row)
            if digest(env) != before:
                raise AssertionError('Prediction copies changed original source or RNG')
        result = original_env_step(env, action)
        count += 1
        return result

    ManagerBasedRlEnv.step = observe
    solver._solve = solve_tap
    try:
        baseline.main()
    finally:
        ManagerBasedRlEnv.step = original_env_step
        mjwarp.step = original_warp_step
        solver._solve = original_solve

    base = Path(__file__).resolve().parents[2]/'output/duck_sim/overnight-20261011/bam-tracking-control-tap-v2/generation-00.npz'
    old, actual = np.load(base), np.load(output/'generation-00.npz')
    parity = {k: bool(np.array_equal(old[k], actual[k])) for k in old.files}
    if not all(parity.values()):
        raise AssertionError('Instrumentation changed original source trace')
    arrays = {}
    for group, rows in samples.items():
        arrays[group] = {k: np.stack([r[k] for r in rows]) for k in rows[0]}
        arrays[group].update({'substep_'+k: np.stack([r[k] for r in integrated[group]])
                              for k in integrated[group][0]})
        np.savez_compressed(output/(group+'.npz'), **arrays[group])
    ref = arrays['native10']
    report = dict(completed=True, scope=__doc__, hardware_access=False,
        known_actions_and_delay_replay=True, source_arrays_bitwise_equal=parity,
        source_root_writes=0, controller_or_walking_acceptance=False,
        samples=len(samples['native10']), wall_seconds=time.monotonic()-started,
        balance_threshold=2e-4, configurations=configs, variants={},
        limitations=['Known-delay open-loop actions, not a future-state oracle.',
            'All copies omit zero-push callback coordinate roundoff as in validated branch.',
            'Numerical solver setting changes are isolated counterfactuals; no source replacement.',
            'Raw finite-step contact forces and timestep sums are not hardware measurements.'])
    for name, arr in arrays.items():
        residual = abs(arr['residual'])
        subres = abs(arr['substep_residual'])
        air = (arr['height'][:,0] >= .003) & (arr['load'][:,0] < .001) & (arr['contact'][:,1] > 0)
        report['variants'][name] = dict(
            max_force_balance_residual=float(residual.max()),
            max_root_force_balance_residual=float(residual[:,:3].max()),
            max_joint_torque_balance_residual=float(residual[:,6:].max()),
            frames_over_balance_threshold=int((residual.max(axis=1)>2e-4).sum()),
            substeps=len(integrated[name]),
            substeps_over_balance_threshold=int((subres.max(axis=1)>2e-4).sum()),
            max_substep_balance_residual=float(subres.max()),
            solver_niter_max=int(arr['solver_niter'].max()),
            substep_solver_niter_max=int(arr['substep_solver_niter'].max()),
            contact_impulse_ns=(arr['substep_load'].sum(axis=0)*configs[name]['dt']).tolist(),
            peak_swing_sole_clearance_mm=float(arr['height'][:,0].max()*1000),
            max_tilt_deg=float(arr['tilt'].max()),
            frames_qualified_air=int(air.sum()), consecutive_air_pairs=int((air[1:]&air[:-1]).sum()),
            first_tilt_boundary_frame=next((i for i,v in enumerate(arr['tilt']) if v>=15),None),
            any_range_failed=bool(arr['range_failed'].any()),
            any_self_collision=bool(arr['self_collision'].any()),
            difference_from_native={k: float(abs(arr[k].astype(float)-ref[k].astype(float)).max())
                for k in ('qpos','qvel','height','load','tilt','qfrc_constraint','goal','requested')})
        criteria = {}
        for prefix in ('', 'substep_'):
            tol = configs[name]['tolerance'][0]
            bad = abs(arr[prefix+'residual']).max(axis=1)>2e-4
            improved = arr[prefix+'solver_scaled_improvement'] < tol
            gradient = arr[prefix+'solver_scaled_gradient'] < tol
            cap = arr[prefix+'solver_niter'] >= configs[name]['iterations']
            criteria[prefix or 'frame_'] = dict(over_residual_threshold=int(bad.sum()),
                bad_but_cost_termination=int((bad & improved & ~gradient & ~cap).sum()),
                bad_with_iteration_cap=int((bad & cap).sum()),
                bad_with_gradient_termination=int((bad & gradient).sum()),
                negative_improvement=int((arr[prefix+'solver_scaled_improvement']<0).sum()),
                maximum_recorded_gradient_balance_difference=float(abs(
                    arr[prefix+'solver_gradient']-arr[prefix+'residual']).max()))
        report['variants'][name]['solver_termination_criteria'] = criteria
    report['native_vs_original_maximum_error'] = {k: float(abs(ref[k]-actual[k][:,0]).max())
        for k in ('height','load','tilt','actual')}
    source_files = [Path(__file__), Path(__file__).with_name('bam_prediction_branch.py'), Path(baseline.__file__)]
    report['source_sha256'] = {str(p): sha(p) for p in source_files+[base]}
    for p in source_files:
        (output/p.name).write_bytes(p.read_bytes())
    report['output_sha256'] = {name+'.npz': sha(output/(name+'.npz')) for name in arrays}
    (output/'convergence-summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False),flush=True)


if __name__ == '__main__':
    main()
