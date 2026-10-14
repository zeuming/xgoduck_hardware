"""Offline source/filter and exact cached async-job timestamp audit.

The worker is deliberately ready immediately. No serial, controller thread,
IPC process or plant is constructed. This checks program semantics only.
"""
from __future__ import annotations
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import types


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_module(name, path):
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--production', type=Path, required=True)
    p.add_argument('--upstream', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    sys.path.insert(0, str(a.production.resolve()/'policy'))
    import motion
    import rl_core as cached
    upstream_path = a.upstream/'python/rl_core.py'
    upstream = load_module('audit_upstream_rl_core', upstream_path)
    old_module = sys.modules['rl_core']
    try:
        sys.modules['rl_core'] = upstream
        proc = load_module('audit_upstream_proc', a.upstream/'python/proc.py')
    finally:
        sys.modules['rl_core'] = old_module
    def definition(path, name):
        return ast.dump(next(x for x in ast.parse(path.read_text()).body
                             if isinstance(x, (ast.ClassDef, ast.FunctionDef)) and x.name == name),
                        include_attributes=False)
    report = dict(hardware_access=False, actual_plant_constructed=False,
        production_verified_current=False, scope='Cached functions and synthetic timed inputs; not physical timing/root-cause proof',
        source_sha256={str(p): sha(p) for p in (
            upstream_path, a.upstream/'python/proc.py',
            a.production/'policy/rl_core.py', a.production/'policy/async_policy.py',
            a.production/'policy/production_motion.py')})
    report['filter'] = dict(
        imu_filter_class_ast_exact=definition(upstream_path, 'ImuFilter') == definition(a.production/'policy/rl_core.py', 'ImuFilter'),
        constants={k: dict(upstream=getattr(upstream, k), cached=getattr(cached, k))
                   for k in ('GYRO_ALPHA', 'VEL_ALPHA', 'GRAVITY_TAU', 'FEEDBACK_PERIOD')},
        axis_scope='After correct policy-body calibration: QMI already supplies gravity sign; LSM supplies specific force and native adapter negates it. This does not verify real mounting.')
    source_filter = proc.SignalFilter()
    native_filter = cached.ImuFilter()
    now = 10.
    previous = None
    maximum_error = dict(gravity=0., gyro_rad=0., velocity_deg_s=0.)
    for i in range(192):
        now += (.01, .02, .015, .04, .01, .03)[i % 6]
        force = [math.sin(i*.11), .3*math.cos(i*.07), 9.8+.4*math.sin(i*.03)]
        gyro = [12*math.sin(i*.09), 9*math.cos(i*.08), 2.]
        angles = [float(j)+i*.01 for j in range(15)]
        velocities = [math.sin(i*.13+j*.1)*30 for j in range(15)]
        source = source_filter.update([-v for v in force], gyro, angles, velocities, timestamp=now)
        dt = cached.FEEDBACK_PERIOD if previous is None else now-previous
        previous = now
        target = native_filter.update(cached.State([force[1], force[2], -force[0]],
            [-gyro[1], -gyro[2], gyro[0]], angles, velocities), dt)
        for name, x, y in (
            ('gravity', source['grav'], target.grav),
            ('gyro_rad', source['gyro_rad'], target.gyro_rad),
            ('velocity_deg_s', source['vels_deg'], target.vels)):
            maximum_error[name] = max(maximum_error[name], float(np.max(np.abs(np.array(x)-y))))
    assert all(v == 0 for v in maximum_error.values())
    report['filter'].update(synthetic_matching_clock_frames=192,
                            synthetic_body_frame_outputs_max_abs_error=maximum_error,
                            observation_model_difference='Training uses rotated true root gravity/angular velocity plus source noise/delay, not this accelerometer/gyro complementary filter and velocity EMA.')
    c = json.loads((a.root/'current-body-v2/inputs/contract.json').read_text())
    class ReadyWorker:
        def __init__(self):
            self.pending = {}
            self.submissions = []
            self.sequence = 0
        def submit(self, selected, obs):
            self.sequence += 1
            self.pending[self.sequence] = (np.full((1, 14), self.sequence/100., np.float32), 0.)
            self.submissions.append(dict(id=self.sequence, observation=obs.copy()))
            return self.sequence
        def take(self, request):
            return self.pending.pop(request)
        def discard(self, request):
            self.pending.pop(request, None)
    report['async_pick'] = {}
    for hz in (50, 40):
        time_now = [100.]
        ctrl = object.__new__(motion.MotionController)
        ctrl._clock = lambda: time_now[0]
        ctrl.sessions = {'pick': object()}
        ctrl._policy_profiles = {'pick': dict(home_rad=np.array(c['models']['pick']['home_rad']), scale=1.)}
        ctrl._policy_worker = ReadyWorker()
        ctrl._policy_job = None
        ctrl._late_policy_frames = 0
        previous_action = np.zeros(14, np.float32)
        smooth = np.zeros(14, np.float32)
        rows = []
        for tick in range(8):
            elapsed = tick/hz
            time_now[0] = 100.+elapsed
            state = cached.State([0., 0., 9.8], [0., 0., 0.], [0.]*15, [0.]*15,
                                 grav=[0., 0., -1.], gyro_rad=[float(tick), 0., 0.])
            frame = dict(generation=1, mode='pick', context=(1, 'pick', 0, None, 'walk'),
                selected='pick', st=state, previous=previous_action.copy(), command=np.zeros(3, np.float32),
                smooth=smooth.copy(), elapsed=elapsed, head_command=None,
                observation_received=time_now[0], observation_sample=round(elapsed*1e6))
            ready = ctrl._async_policy_step(frame, .15)
            if ready is None:
                assert tick == 0
                continue
            job, _, action, new_smooth, _ = ready
            submitted = job['submitted_observation'][0]
            np.testing.assert_array_equal(submitted[34:48], previous_action)
            current_phase = np.array([math.cos(2*math.pi*elapsed/4), math.sin(2*math.pi*elapsed/4)], np.float32)
            job_phase = np.array([math.cos(2*math.pi*job['elapsed']/4), math.sin(2*math.pi*job['elapsed']/4)], np.float32)
            np.testing.assert_array_equal(submitted[48:50], job_phase)
            assert submitted[0] == tick-1
            rows.append(dict(application_tick=tick, input_sensor_tick=int(submitted[0]),
                application_elapsed_s=elapsed, submitted_elapsed_s=job['elapsed'],
                input_age_ms=(time_now[0]-job['observation_received'])*1000,
                compute_ms=0., submitted_pick_phase=submitted[48:50].tolist(),
                application_time_pick_phase=current_phase.tolist(),
                maximum_pick_command_slot_difference=float(np.max(np.abs(current_phase-job_phase))),
                raw_history_matches_latest_committed=True))
            previous_action, smooth = action, new_smooth
            # Exact production_motion._async_control_body tail: next job uses
            # the latest sampled state with only the just-committed recurrence.
            frame.update(previous=action.copy(), smooth=new_smooth.copy())
            ctrl._submit_policy_frame(frame)
        report['async_pick'][str(hz)] = dict(worker_scope='Deterministic already-ready zero-compute fake; no IPC/serial',
            frames=rows, first_call_submits_without_consuming=True,
            maximum_command_slot_difference=max(x['maximum_pick_command_slot_difference'] for x in rows),
            inference_epoch_matches_sensor_epoch=True, history_matches_application_epoch=True)
    report['protocol_gap'] = dict(
        previous_stress='Delay first34 sensor slots; preserve application-time PICK phase and current raw history',
        cached_prefetch='Sensor slots AND PICK phase are from the submitted prior frame; raw history is the most recently committed action',
        interpretation='The sensor-only stress is valid as a declared counterfactual, but is not exact cached-PICK pipeline emulation. No physical failure is attributed to this audit.',
        next_control='If needed, compare only source-derived phase epoch while holding initial plant/sensor lag/history/timing fixed; do not search arbitrary phase offsets')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(filter=report['filter'], async_pick={k: {n:v for n,v in x.items() if n != 'frames'} for k,x in report['async_pick'].items()}, protocol_gap=report['protocol_gap'])))


if __name__ == '__main__':
    main()
