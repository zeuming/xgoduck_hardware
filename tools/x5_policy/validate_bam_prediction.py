"""Known-action, known-delay multi-step parity and source-isolation check."""
import hashlib
import json
import os
import sys
from pathlib import Path
import numpy as np


def main():
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/xgoduck-dynamics-mpl-cache')
    delay_mode = 'replay'
    if '--prediction-delay-mode' in sys.argv:
        i = sys.argv.index('--prediction-delay-mode')
        delay_mode = sys.argv[i+1]; del sys.argv[i:i+2]
    if delay_mode not in ('replay', 'sampled'):
        raise ValueError('Unsupported delay validation mode')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from bam_prediction_branch import BamPredictionBranch
    import diagnose_bam_control_tap as baseline
    output = Path(sys.argv[sys.argv.index('--output')+1])
    original = ManagerBasedRlEnv.step
    count = 0
    branch = None
    records = []
    def state_digest(env):
        bam = env.scene['robot'].actuators[0]
        term = env.action_manager.get_term('joint_pos')
        arrays = [env.sim.wp_data.qpos.numpy(), env.sim.wp_data.qvel.numpy(),
                  env.sim.wp_model.dof_frictionloss.numpy(), term._goal.numpy(),
                  bam._prev_motor_torque.numpy(), torch.get_rng_state().numpy()]
        return hashlib.sha256(b''.join(a.tobytes() for a in arrays)).hexdigest()
    def observed(env, action):
        nonlocal count, branch
        if count == 250:
            before = state_digest(env)
            branch = BamPredictionBranch(env)
            if state_digest(env) != before:
                raise AssertionError('Branch construction mutated source')
        predicted = None
        if 250 <= count < 350:
            before = state_digest(env)
            predicted = branch.step(action.clone(), torch.get_rng_state().clone() if delay_mode == 'replay' else None)
            measured = branch.measure()
            if state_digest(env) != before:
                raise AssertionError('Prediction mutated source or its RNG')
        result = original(env, action)
        if predicted is not None:
            bam = env.scene['robot'].actuators[0]
            term = env.action_manager.get_term('joint_pos')
            actual = {key: getattr(env.sim.wp_data, key).numpy().copy() for key in
                      ('qpos', 'qvel', 'qacc', 'ctrl', 'qfrc_constraint', 'geom_xpos', 'geom_xmat')}
            actual.update(ema=term._ema.numpy().copy(), goal=term._goal.numpy().copy(),
                          range_failed=term.range_failed.numpy().copy(),
                          motor_torque=bam._prev_motor_torque.numpy().copy(),
                          motor_target=bam._as_tensor(bam._bam_model.actuator.q_target_smooth).numpy().copy())
            records.append(dict(step=count-250,
                maximum_error={k:float(np.max(np.abs(predicted[k].astype(float)-v.astype(float)))) for k,v in actual.items()},
                bitwise_equal={k:bool(np.array_equal(predicted[k],v)) for k,v in actual.items()},
                predicted_measurement={k:np.asarray(v).tolist() for k,v in measured.items()}))
        count += 1
        return result
    ManagerBasedRlEnv.step = observed
    try:
        baseline.main()
    finally:
        ManagerBasedRlEnv.step = original
    old = Path(__file__).resolve().parents[2]/'output/duck_sim/overnight-20261011/bam-tracking-control-tap-v2/generation-00.npz'
    before = np.load(old); after = np.load(output/'generation-00.npz')
    parity = {k:bool(np.array_equal(before[k],after[k])) for k in before.files}
    sources = [Path(__file__), Path(__file__).with_name('bam_prediction_branch.py'), Path(baseline.__file__)]
    report = dict(completed=True,scope=__doc__,hardware_access=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        known_action_and_exogenous_random_replay=delay_mode == 'replay', delay_mode=delay_mode, horizon_s=len(records)*.02,
        source_arrays_bitwise_equal=parity, source_isolation_passed=all(parity.values()),
        maximum_error={k:max(r['maximum_error'][k] for r in records) for k in records[0]['maximum_error']},
        all_prediction_arrays_bitwise_equal=all(all(r['bitwise_equal'].values()) for r in records), records=records)
    report['measurement_maximum_error'] = {}
    for field in ('height', 'load', 'contact', 'tilt', 'actual', 'self_collision'):
        pv=np.asarray([r['predicted_measurement'][field] for r in records]).astype(float)
        av=after[field][:len(records),0].astype(float)
        report['measurement_maximum_error'][field]=float(abs(pv-av).max())
    report['replay_tolerance'] = dict(qpos=1e-5, geom_xpos=1e-6, ema=0., goal=0., motor_target=0.)
    report['known_delay_replay_passed'] = delay_mode == 'replay' and all(
        report['maximum_error'][k] <= v for k,v in report['replay_tolerance'].items())
    (output/'prediction-parity.json').write_text(json.dumps(report,indent=2)+'\n')
    for p in sources:(output/p.name).write_bytes(p.read_bytes())
    print(json.dumps({k:v for k,v in report.items() if k!='records'}),flush=True)
    if not all(parity.values()):raise AssertionError('Predictor changed source trajectory')
    if delay_mode=='replay' and not report['known_delay_replay_passed']:
        raise AssertionError('Known-delay replay exceeded declared numerical tolerance')


if __name__ == '__main__':main()
