"""File-only open-loop replay of accepted operator wire goals and write times.

The initial recorded pose is conditional, not a physical parameter estimate.
Only goals are replayed: feedback/root states are never overwritten after reset.
The original BAM physics, gains and delay model are retained. The explicit
calibrated-physical hypothesis sets simulated encoder bias to zero, so recorded
decoded positions and goals mean CAD angles. Original random draws are consumed
before this declared coordinate assumption; no actual calibration is edited.
Outer EMA is already included in the recorded integer goals and must
not be applied a second time. Command times are rounded up to a physics substep.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


class ReplayCompleted(Exception):
    pass


def wire_schedule(trial, calibration, ids):
    rows = trial['policy_frames']
    start_host = rows[0]['observation_received_monotonic']
    start_sample = rows[0]['input_feedback_provenance']['state_sample_us']
    all_ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
    by_id = {v['id']: v for v in calibration['joints']}

    def angles(ticks):
        if len(ticks) != 15:
            raise ValueError('Expected all15 wire positions')
        values = []
        for sid in ids:
            c, tick = by_id[sid], ticks[all_ids.index(sid)]
            if not c['min'] <= tick <= c['max']:
                raise ValueError('Replay must not send a rejected goal')
            values.append((tick-c['zero'])*2*np.pi/4095*c['sign'])
        return values

    before = next(v for v in reversed(trial['frames']) if v['mode']=='hold')
    schedule = [dict(time_s=0., q=angles(before['attempted_target_ticks']),
                     ticks=before['attempted_target_ticks'], phase='preceding_hold')]
    for row in rows:
        if not row['target_write_completed']:
            continue
        schedule.append(dict(time_s=row['target_write_started_monotonic']-start_host,
            q=angles(row['transmitted_ticks']), ticks=row['transmitted_ticks'],
            phase='recorded_zero_goal'))
    if any(b['time_s'] <= a['time_s'] for a,b in zip(schedule,schedule[1:])):
        raise ValueError('Goal times must increase')
    samples = []
    for row in rows:
        provenance = row['input_feedback_provenance']
        if provenance['state_sample_us'] < start_sample:
            raise ValueError('This replay does not handle sample timestamp wrap')
        for sid in ids:
            j, c = all_ids.index(sid), by_id[sid]
            age = provenance['position_ages_us'][j]
            if age is None:
                continue
            # MCU per-ID measurement age matters; a packet timestamp is not
            # the timestamp of all retained positions in a partial scan.
            at = (provenance['state_sample_us']-start_sample-age)/1e6
            if at >= 0:
                samples.append(dict(id=sid,time_s=at,angle_rad=
                    (provenance['raw_positions'][j]-c['zero'])*2*np.pi/4095*c['sign']))
    return schedule, samples, rows[-1]['validation_monotonic']-start_host


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,required=True)
    a=p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve previous replay')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    import evaluate_official_causal_factors as baseline
    from rl_core import POLICY_IDS
    trial=json.loads(a.trial.read_text())
    schedule, real_samples, duration=wire_schedule(trial,json.loads(a.calibration.read_text()),POLICY_IDS)
    spec=importlib.util.spec_from_file_location('recorded_goal_measurement',ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py')
    measure=importlib.util.module_from_spec(spec);spec.loader.exec_module(measure)
    traces=[];applied=[];report={};started=time.monotonic()
    original_step=ManagerBasedRlEnv.step

    def replay(env, ignored_action):
        if env.common_step_counter != 0:
            raise RuntimeError('Replay driver must start from reset')
        term=env.action_manager.get_term('joint_pos')
        robot=env.scene['robot'];dt=float(env.physics_dt)
        report['source_randomized_encoder_bias_rad']=robot.data.encoder_bias.numpy().tolist()
        robot.data.encoder_bias.zero_()
        report['encoder_reference']='Explicit calibrated-physical CAD hypothesis: simulated bias zero; real zero/assembly remains unverified'
        report.update(initial_qpos_sha256=hashlib.sha256(env.sim.data.qpos.numpy().tobytes()).hexdigest(),
            initial_qvel_sha256=hashlib.sha256(env.sim.data.qvel.numpy().tobytes()).hexdigest(),
            initial_fingerprint=baseline.initial_fingerprint(env),physics_dt_s=dt,
            initial_measurement=measure.actual_plant_measurements(env))
        traces.append(dict(time_s=0.,actual_joint_rad=robot.data.joint_pos[0,term.target_ids].numpy().tolist(),
            **report['initial_measurement']))
        next_goal=0
        for k in range(int(np.ceil(duration/dt))):
            if time.monotonic()-started>120:
                raise TimeoutError('120s local replay budget')
            at=k*dt
            while next_goal<len(schedule) and schedule[next_goal]['time_s']<=at+1e-12:
                goal=schedule[next_goal]
                term._processed_actions=torch.tensor([goal['q']],dtype=torch.float32,device=env.device)
                applied.append(dict(index=next_goal,requested_time_s=goal['time_s'],
                                    applied_time_s=at,rounding_s=at-goal['time_s']))
                next_goal+=1
            env._sim_step_counter+=1
            env.action_manager.apply_action()
            env.scene.write_data_to_sim()
            env.sim.step()
            env.scene.update(dt=dt)
            env.sim.forward()
            row=measure.actual_plant_measurements(env)
            row.update(time_s=(k+1)*dt,
                actual_joint_rad=robot.data.joint_pos[0,term.target_ids].numpy().tolist(),
                processed_joint_rad=term._processed_actions[0].numpy().tolist(),
                active_goal_index=next_goal-1)
            traces.append(row)
            q=np.asarray(row['actual_joint_rad'])
            if row['tilt_deg']>=15 or np.any((q<term.low.numpy())|(q>term.high.numpy())):
                report['stopped_at_existing_boundary']=dict(time_s=row['time_s'],
                    tilt_deg=row['tilt_deg'],actual_range_failed=bool(np.any((q<term.low.numpy())|(q>term.high.numpy()))))
                break
        report['applied_goals']=applied
        raise ReplayCompleted()

    ManagerBasedRlEnv.step=replay
    previous_argv=sys.argv
    sys.argv=[str(baseline.__file__),'--output',str(a.output),'--seed',str(a.seed),
        '--speed','0','--factors','gains,ema,ranges','--official-runtime',
        '--quantize-goals','--head-range-projection','--saved-calibration',str(a.calibration),
        '--mass-distribution-scenario',str(a.scenario),'--recorded-entry-trial',str(a.trial)]
    try:
        baseline.main()
    except ReplayCompleted:
        report['complete']=True
    finally:
        ManagerBasedRlEnv.step=original_step;sys.argv=previous_argv
    times=np.asarray([r['time_s'] for r in traces]);actual=np.asarray([r['actual_joint_rad'] for r in traces])
    errors={}
    for sid in POLICY_IDS:
        selected=[r for r in real_samples if r['id']==sid and r['time_s']<=times[-1]]
        difference=np.array([np.interp(r['time_s'],times,actual[:,POLICY_IDS.index(sid)])-r['angle_rad'] for r in selected])
        errors[str(sid)]=dict(samples=len(selected),rmse_deg=float(np.degrees(np.sqrt(np.mean(difference**2)))) if len(selected) else None)
    report.update(scope=__doc__,hardware_access=False,production_changed=False,walking_verified=False,
        seed=a.seed,requested_duration_s=duration,simulated_duration_s=times[-1],
        accepted_recorded_goals=len(schedule)-1,rejected_recorded_goals_excluded=sum(not r['target_write_completed'] for r in trial['policy_frames']),
        max_goal_rounding_ms=1000*max(r['rounding_s'] for r in applied),
        compared_per_ID_read_time_errors=errors,max_tilt_deg=max(r['tilt_deg'] for r in traces),
        limitations=['Open-loop recorded commands; not a controller or gait acceptance.',
            'Unknown clock/transport offset and up-to-one5ms physics-step command rounding.',
            'Initial gravity/angles assumed physical; unknown real support/forces and BAM history.',
            'Recorded goals already include action EMA; no second EMA is applied.',
            'All original actuator substep apply/delay/BAM calls retained; no running root injection.'],
        source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
            (Path(__file__),Path(baseline.__file__),Path(__file__).with_name('offline_recorded_entry.py'),a.trial,a.calibration,a.scenario)},
        wall_seconds=time.monotonic()-started)
    (a.output/'goal-replay-trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in traces))
    (a.output/'goal-replay-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('complete','simulated_duration_s','max_tilt_deg','max_goal_rounding_ms','compared_per_ID_read_time_errors')},indent=2))


if __name__=='__main__':
    main()
