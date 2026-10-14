"""Offline source-duration trick -> zero-speed model workflow, one fixed plant.

This is a source-aligned workflow prototype, NOT the installed command service.
No plant state is written after its ordinary source reset. Model/EMA/raw-history
switches do not reset motor/BAM/delay, time, contacts or the physical root.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--production', type=Path, required=True)
    p.add_argument('--mode', choices=('getup', 'pick', 'roulade'), required=True)
    p.add_argument('--models', choices=('original', 'selected'), required=True)
    p.add_argument('--body', choices=('reference', 'rear-mid', 'rear-high'), required=True)
    p.add_argument('--seed', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--selection', type=Path, help='Explicit counterfactual actor set; never overwrite the selected index')
    p.add_argument('--termination-contract', choices=('source-task', 'source-runtime'), default='source-task')
    p.add_argument('--horizon', type=int, choices=(8, 12, 16), help='Evaluation horizon only, not a motion lease or protection threshold')
    p.add_argument('--control-hz', type=int, choices=(40, 50), default=50)
    p.add_argument('--sensor-lag-steps', type=int, choices=(0, 1), default=0)
    p.add_argument('--pick-phase-epoch', choices=('application', 'sensor'), default='application',
                   help='Source-derived phase-epoch control only; sensor uses the same sampled frame as the delayed sensor slots')
    p.add_argument('--wall-seconds', type=float, default=240.)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    root = a.root.resolve()
    # Pin the same archived physical adapter used by the accepted independent
    # matrices. The new harness changes workflow, not its reference plant.
    sys.path.insert(0, str(root/'gpu-bundle-v4/tools'))
    import numpy as np
    import torch
    import onnxruntime as ort
    import mujoco
    from mjlab.envs import ManagerBasedRlEnv
    from current_body_training import load_contract, configure, RuntimeDecoder, sha
    from current_body_fingerprint import initial_fingerprint
    from current_body_measurements import actual_plant_measurements

    sys.path.insert(0, str(a.production.resolve()/'policy'))
    from official_recovery import Recovery
    from rl_core import policy_profile
    contract = load_contract(root/'current-body-v2/inputs/contract.json')
    selection_path = a.selection if a.selection is not None else root/'selected-models-v1.json'
    selected = json.loads(selection_path.read_text())
    joints = json.loads(Path(contract['files']['calibration']['resolved_path']).read_text())['joints']
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    random.seed(a.seed)
    cfg, _, declaration = configure(contract, a.mode, a.body)
    cfg.decimation = round(1/a.control_hz/cfg.sim.mujoco.timestep)
    cfg.scene.num_envs = 1
    cfg.seed = a.seed
    cfg.auto_reset = False
    cfg.curriculum.clear()
    cfg.events.pop('push_robot', None)
    seconds = float(a.horizon) if a.horizon is not None else 12. if a.mode == 'getup' else 8.
    cfg.episode_length_s = seconds+.02
    if a.termination_contract == 'source-runtime':
        # A task's training fell_over term is not an MCU/host safety limit.
        # Source controller Recovery handles falls instead of terminating the
        # simulation at75deg. Keep source NaN guards and bounded horizon.
        cfg.terminations.pop('fell_over', None)
    if a.mode == 'roulade':
        cfg.events['set_roulade_state'].params.update(standing_prob=1., midroll_prob=0.)
    if a.mode == 'getup':
        # The source reset preparation is 2 seconds; recovery.py's actual
        # runtime default-pose stage is 1 second. Test that distinction
        # explicitly without changing the source event or physical reset.
        cfg.actions['joint_pos'].reset_freeze_seconds = 1.
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    sessions, models = {}, {}
    for mode in contract['models']:
        path = Path(contract['files'][mode]['resolved_path']) if a.models == 'original' else root/selected['models'][mode]['model']
        sessions[mode] = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
        profile = policy_profile(sessions[mode])
        assert np.array_equal(profile['home_rad'], contract['models'][mode]['home_rad'])
        models[mode] = dict(path=str(path), sha256=sha(path))
    report = dict(mode=a.mode, model_set=a.models, body=a.body, seed=a.seed,
                  hardware_access=False, installed_runtime_tested=False,
                  prototype_scope='Source trick duration -> walk/recovery, persistent physical plant; cached one-shot OFF path not replaced',
                  sensor_scope='Original source task sensor/noise/delay model persists across switches; not measured real IMU/filter/timing',
                  relative_home_remap='Float64 change of HOME basis for already sampled float32 relative-position slots; no resampling or physical write',
                  declaration=declaration, models=models, simulation_accepted=False,
                  source_runtime_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',
                  frozen_adapter_sha256=sha(root/'gpu-bundle-v4/tools/current_body_training.py'),
                  evaluator_sha256=sha(Path(__file__)),
                  selection_sha256=sha(selection_path),
                  termination_contract=a.termination_contract,
                  evaluation_horizon_s=seconds,
                  control_hz=a.control_hz, additional_actor_sensor_lag_steps=a.sensor_lag_steps,
                  pick_phase_epoch=a.pick_phase_epoch,
                  phase_epoch_scope='Single-factor command-clock control derived from cached async prefetch; not a complete worker/startup/serial emulator',
                  timing_scope='Fixed simulation control period and additional first34 actor-sensor lag; source Recovery reads current sensor sample, raw history remains current; PICK phase epoch is declared separately. Not measured X5 cadence.',
                  transitions=[], clock_resets=0, physical_resets_after_initial=0)
    env = None
    rows = []
    started = time.monotonic()
    try:
        env = ManagerBasedRlEnv(cfg, device='cpu', render_mode=None)
        obs, _ = env.reset(seed=a.seed)
        report['initial_fingerprint'] = initial_fingerprint(env)
        (a.output/'initialized.json').write_text(json.dumps(report, indent=2)+'\n')
        mouth_sites = [i for i in range(env.sim.mj_model.nsite)
                       if (mujoco.mj_id2name(env.sim.mj_model, mujoco.mjtObj.mjOBJ_SITE, i) or '').split('/')[-1] == 'mouth_tip']
        assert len(mouth_sites) == 1
        term = env.action_manager.get_term('joint_pos')
        active = a.mode
        previous = np.zeros(14, np.float32)
        sensor_history = []
        recovery = Recovery()
        if a.mode == 'getup':
            recovery.phase = 'default_pose'
            recovery.phase_since = 0.
        report['transitions'].append(dict(time_s=0., selected=active,
            phase=recovery.phase, reason='initial_source_reset'))
        with (a.output/'trace.jsonl').open('x') as stream:
            for step in range(round(seconds/env.step_dt)):
                if time.monotonic()-started > a.wall_seconds:
                    raise TimeoutError('workflow wall budget')
                now = step*env.step_dt
                rolling = a.mode == 'roulade' and now < 1.9
                picking = a.mode == 'pick' and now < 4.
                gravity = obs['actor'][0, 3:6].cpu().numpy()
                if not rolling:
                    recovery.update(gravity, now, True)
                wanted = 'roulade' if rolling else 'pick' if picking and recovery.phase == 'walk' else 'walk' if recovery.phase == 'walk' else 'getup'
                sampled_home = term.decoder.home[0:14].cpu().numpy().copy()
                sensor_history.append((obs['actor'][0, :34].cpu().numpy().copy(), sampled_home, now))
                if len(sensor_history) > a.sensor_lag_steps+1:
                    sensor_history.pop(0)
                if wanted != active:
                    report['transitions'].append(dict(time_s=now, selected=wanted,
                        phase=recovery.phase, reason='source_duration_or_recovery_transition'))
                    active = wanted
                    previous.fill(0)
                    # Source Policy.reset_action_filter clears both raw history
                    # and EMA. This changes only action state, never the plant.
                    term.decoder = RuntimeDecoder(contract['models'][active], joints, 1, 'cpu')
                frozen = recovery.phase == 'default_pose'
                raw = obs['actor'].cpu().numpy().copy()
                raw[0, :34], sampled_home, sensor_epoch = sensor_history[0]
                home = np.array(contract['models'][active]['home_rad'], np.float64)
                if not np.array_equal(sampled_home, home):
                    raw[0, 6:20] = (raw[0, 6:20].astype(np.float64)+sampled_home-home).astype(np.float32)
                raw[0, 34:48] = previous
                raw[0, 48:61] = 0.
                if active == 'pick':
                    phi = (now if a.pick_phase_epoch == 'application' else sensor_epoch)/4.
                    raw[0, 48:51] = [math.cos(2*math.pi*phi), math.sin(2*math.pi*phi), 0.]
                elif active == 'walk':
                    raw[0, 51:55] = contract['head_command_rad']
                action = np.zeros((1, 14), np.float32) if frozen else sessions[active].run(None, {sessions[active].get_inputs()[0].name: raw})[0]
                if frozen:
                    term.decoder.reset()
                obs, _, terminated, truncated, _ = env.step(torch.from_numpy(action))
                previous = action[0].copy()
                row = actual_plant_measurements(env)
                row.update(time_s=(step+1)*env.step_dt, selected=active,
                    phase=recovery.phase, actor_observation=raw[0].tolist(), raw_action=action[0].tolist(),
                    transmitted_ticks=term.decoder.ticks[0].tolist(),
                    mouth_tip_height_m=float(env.sim.data.site_xpos[0, mouth_sites[0], 2]),
                    failed_terms=[k for k in env.termination_manager.active_terms if k != 'time_out' and bool(env.termination_manager.get_term(k)[0])],
                    task_complete=step == round(seconds/env.step_dt)-1)
                rows.append(row)
                stream.write(json.dumps(row, allow_nan=False)+'\n')
                if bool(terminated[0]) or bool(truncated[0]):
                    break
        tail = rows[-round(1/env.step_dt):]
        supported = sum(all(r['feet'][s]['vertical_load_n'] >= .1 and r['feet'][s]['mesh_min_height_m'] < .0015 for s in ('left', 'right')) for r in tail)
        stable = len(tail) == round(1/env.step_dt) and all(r['tilt_deg'] < 15 and r['selected'] == 'walk' and not r['failed_terms'] for r in tail)
        drift = math.dist(tail[-1]['root_position'][:2], tail[0]['root_position'][:2]) if tail else float('inf')
        gate = dict(complete=bool(rows) and rows[-1]['task_complete'], final_zero_walk_stable=stable,
                    final_supported_frames=supported, final_drift_m=drift)
        gate['post_task_fall_recovery_started'] = any(r['phase'] != 'walk' for r in rows
            if r['time_s'] > (4. if a.mode == 'pick' else 1.9 if a.mode == 'roulade' else seconds))
        if a.mode == 'pick':
            gate['minimum_descent_mouth_height_m'] = min((r['mouth_tip_height_m'] for r in rows if .8 <= r['time_s'] <= 1.8), default=float('inf'))
            task = gate['minimum_descent_mouth_height_m'] <= .04
        elif a.mode == 'roulade':
            turn = 0.
            maximum = 0.
            previous_q = None
            for row in rows:
                q = np.array(row['root_quaternion_wxyz'], np.float64)
                if previous_q is not None:
                    w0, x0, y0, z0 = previous_q
                    w, x, y, z = q
                    dq = np.array([w0*w+x0*x+y0*y+z0*z, w0*x-x0*w-y0*z+z0*y,
                                   w0*y+x0*z-y0*w-z0*x, w0*z-x0*y+y0*x-z0*w])
                    if dq[0] < 0:
                        dq *= -1
                    norm = np.linalg.norm(dq[1:])
                    turn += dq[2]*(2*math.atan2(norm, dq[0])/norm if norm else 2.)
                    maximum = max(maximum, abs(turn))
                previous_q = q
            gate['maximum_quaternion_body_y_turn_rad'] = maximum
            task = maximum >= 1.5*math.pi
        else:
            task = any(r['selected'] == 'getup' and r['phase'] == 'getup' for r in rows)
        gate['accepted'] = bool(gate['complete'] and stable and supported >= .9*len(tail) and drift <= .005 and task)
        report.update(frames=len(rows), gate=gate, simulation_accepted=gate['accepted'], wall_seconds=time.monotonic()-started)
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        if env is not None:
            env.close()
        (a.output/'gate.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report.get(k) for k in ('mode', 'model_set', 'body', 'seed', 'simulation_accepted', 'gate', 'transitions', 'error')}))


if __name__ == '__main__':
    main()
