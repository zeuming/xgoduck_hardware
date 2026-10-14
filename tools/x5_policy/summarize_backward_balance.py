"""Audit the declared rear-load comparisons without hardware or new simulation."""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
R=ROOT/'output/duck_sim/source-parity-20261012/backward-balance'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(name):
    folder=R/name
    return json.loads((folder/'summary.json').read_text()), [json.loads(x) for x in (folder/'trace.jsonl').read_text().splitlines()]


def main():
    folders=sorted(p.parent.name for p in R.glob('*/summary.json'))
    all_cases={name:load(name) for name in folders}
    cases=[]
    for name,(s,rows) in all_cases.items():
        gate=s['gate']
        cases.append(dict(name=name,seed=s['seed'],controller=s['controller'],speed_m_s=s['speed_m_s'],
            control_hz=s.get('control_hz',50),sensor_lag_steps=s.get('sensor_lag_steps',0),initial_hold_seconds=s.get('initial_hold_seconds',0),
            accepted=gate['accepted'],frames=s['frames'],max_tilt_deg=s['max_tilt_deg'],
            net_forward_m=gate['net_forward_m'],cycles=gate['completed_lift_land_cycles'],
            alternating=gate['alternating'],recovered=gate['recovered'],
            actual_ranges=s['all_actual_ranges'],transmitted_ranges=s['all_transmitted_ranges'],
            trace_sha256=sha(R/name/'trace.jsonl'),model_sha256=s['model_sha256'],
            initial_qpos_sha256=s['initial_qpos_sha256'],initial_qvel_sha256=s['initial_qvel_sha256'],
            effective_mass_kg=s['actual_body_properties'].get('total_mass_kg'),
            final_failed_terms=rows[-1]['failed_terms'] if rows else []))
    paired=[]
    for left,right in [('reference-hold-seed0','reference-actor-seed0'),
                       ('rear-high-envelope-hold-seed0','rear-high-envelope-actor-seed0'),
                       ('rear-original-forward-seed0','com-restored-forward-seed0'),
                       ('rear-high-forward-seed3','com-restored-forward-seed3')]:
        a,b=all_cases[left][0],all_cases[right][0]
        qsame=a['initial_qpos_sha256']==b['initial_qpos_sha256'] and a['initial_qvel_sha256']==b['initial_qvel_sha256']
        rngsame=a['initial_fingerprint']['torch_rng']==b['initial_fingerprint']['torch_rng']
        assert qsame and rngsame
        motor_keys=[k for k in a['initial_fingerprint'] if k.startswith(('motor.','bam.','bam_actuator.','delay.'))]
        differing_motor=[k for k in motor_keys if a['initial_fingerprint'][k]!=b['initial_fingerprint'][k]]
        assert not differing_motor
        paired.append(dict(left=left,right=right,initial_qpos_qvel_rng_identical=True,
            all_initial_fingerprints_identical=a['initial_fingerprint']==b['initial_fingerprint'],
            actuator_histories_and_parameters_identical=True,
            changed_factor='controller only' if 'hold' in left else 'same point mass fore-aft position; height retained'))
    before=all_cases['reference-actor-seed0'][1];after=all_cases['unchanged-default-regression'][1]
    fields=['root_position','root_quaternion_wxyz','actual_joint_rad','actual_joint_velocity_rad_s',
            'actor_observation','action','processed_joint_rad','adapter_requested_joint_rad','transmitted_joint_rad']
    for field in fields:
        np.testing.assert_array_equal([r[field] for r in before],[r[field] for r in after])
    source_model=ROOT/'output/xgoduck_runtime_upstream/python/models/xgoduck_walk.onnx'
    candidate=json.loads((ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json').read_text())['candidate']
    source_model=Path(candidate['source_model']['path'])
    import onnxruntime as ort
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    actor=ort.InferenceSession(str(source_model),options,providers=['CPUExecutionProvider'])
    delta=np.array(json.loads((R/'geometric-stance-v3.json').read_text())['stance_delta_rad'],dtype=np.float32)
    home=np.array(candidate['home_rad'],dtype=np.float32)
    mapped_checks=[]
    for name in ('rear-pose-map-zero-seed0','rear-pose-map-forward-seed0'):
        rows=all_cases[name][1];smooth=np.zeros(14,dtype=np.float32);previous=np.zeros(14,dtype=np.float32)
        for row in rows:
            raw=np.array(row['actor_observation'],dtype=np.float32)
            physical=np.array(row['physical_actor_observation'],dtype=np.float32)
            expected=physical.copy();expected[6:20]-=delta
            np.testing.assert_array_equal(raw,expected)
            np.testing.assert_array_equal(raw[34:48],previous)
            action=np.array(row['action'],dtype=np.float32)
            smooth*=.45;smooth+=action*.55
            np.testing.assert_array_equal(np.array(row['unbounded_transmitted_joint_rad'],dtype=np.float32),home+candidate['action_scale']*smooth+delta)
            previous=action
        sample_indices=np.unique(np.linspace(0,len(rows)-1,8).astype(int))
        for i in sample_indices:
            row=rows[i];raw=np.array([row['actor_observation']],dtype=np.float32)
            np.testing.assert_array_equal(actor.run(None,{actor.get_inputs()[0].name:raw})[0][0],np.array(row['action'],dtype=np.float32))
        mapped_checks.append(dict(case=name,frames=len(rows),mapping_gravity_gyro_velocity_and_history_verified=True,
            original_float32_ema_verified=True,official_onnx_sample_outputs_exact=len(sample_indices)))
    report=dict(hardware_access=False,production_changed=False,physical_walking_verified=False,
        purpose='Default backwardfall is primary; range fault is secondary',
        declared_load='220g EXISTING trunk point mass, hip-rear9cm,5cm above old trunk COM; unmeasured stress hypothesis. Total nominal871g; original DR retained.',
        prior_simulation_gap='Earlier71g/same-height actor-zero examples did not test fixed HOLD or identify the real robot.',
        cases=cases,paired=paired,unchanged_default_regression=dict(frames=700,exact_fields=fields),
        pose_map_contract_checks=mapped_checks,official_onnx_sha256=sha(source_model),
        findings=[
            'Rear/high fixed HOLD falls in both seeds; original actor zero remains stable in both. COM shift alone in this hypothesis does not reproduce the real zero-speed failure.',
            'Rear/high original forward fails seed0, passes seed3 at lower progress. Do not call every rear-loaded gait impossible.',
            'Calculated constant pose map passes one zero example but fails forward on actual saved head33 range; not approved for real deployment.',
            'Restoring fore-aft COM while keeping assumed mass and raised height passes HOLD, zero, and original-actor forward in two seeds.',
            'With restored COM,40Hz lag0 forward passes both seeds;40Hz lag1 forward fails both. No claim that this fixed delay is an exact real UART replay.',
            'Rear/high default HOLD0.10s then zero changes seed0 from stable zero to backward drift at50Hz, and large backward drift/range stop at40Hz lag1;seed3 still passes both. This is a sufficient simulated startup interaction, not a closed real root cause.',
            'Human requires rear mounting unchanged. COM relocation is only a causal counterfactual, not the selected remedy.',
            'Static geometric correction and simulated remount position are not measured physical compensation values.'
        ],remaining=['Develop and independently validate software adaptation with rear mounting unchanged; constant pose map has not passed walking',
                     'Identify actual rear mass/COM and mounted physical reference',
                     'Close real zero-speed failure cause with synchronized per-ID feedback and correct ownership',
                     'Verify real unsupported stand, zero and ground walking; no simulator result satisfies this gate'],
        operator_boundary='Do not touch active physical connection; no UART/ARM/TARGET/STOP/install/flash/X5 load',
        source_sha256=sha(__file__))
    (R/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(cases=len(cases),paired=len(paired),default_frames_exact=700,pose_map_frames=sum(x['frames'] for x in mapped_checks),physical_verified=False)))


if __name__=='__main__':
    main()
