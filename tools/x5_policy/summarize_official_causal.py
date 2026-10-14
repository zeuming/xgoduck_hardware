"""Summarize unchanged official-policy contrasts; never authorizes hardware."""
import json, hashlib
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'output/duck_sim/slow-gait-20261011'

def main():
    cases=json.loads((OUT/'official-causal-v3-matrix.json').read_text())
    controls={s['seed']:s for s in cases if not s['factors']}
    summary=[];pairings=[];envelopes=[]
    expected_model='5727b909d5ea1a0f9ff63bf4cea9707339dea6a522374c7a120da633605b923f'
    c=json.loads((ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json').read_text())['candidate']
    for s in cases:
        b=controls[s['seed']]
        assert s['model_sha256']==b['model_sha256']==expected_model
        assert s['initial_qpos_sha256']==b['initial_qpos_sha256']
        assert s['initial_qvel_sha256']==b['initial_qvel_sha256']
        assert s['actual_body_properties']==b['actual_body_properties']
        assert s['source_sha256']==b['source_sha256']
        h=s['initial_fingerprint'];bh=b['initial_fingerprint']
        changed=sorted(k for k in h.keys()|bh.keys() if h.get(k)!=bh.get(k))
        expected=['bam_actuator.kp','motor._base_kp','motor._prev_motor_torque'] if 'gains' in s['factors'] else []
        assert changed==expected,(s['factors'],s['seed'],changed)
        # Gains legitimately change the initial computed actuator torque, but
        # do not change qpos/qvel, physical randomization or random stream.
        pairings.append(dict(factors=s['factors'],seed=s['seed'],changed_initial_fields=changed,
            other_initial_fields_identical=True,hashed_fields=len(bh)))
        tag='-'.join(sorted(s['factors'],key=lambda x:['gains','ema','head','slew','ranges'].index(x))) if s['factors'] else 'control'
        folder=OUT/f'official-causal-v3-{tag}-seed{s["seed"]}'
        # Single factors and combined folders retain the runner's declared order.
        rows=[json.loads(line) for line in (folder/'trace.jsonl').read_text().splitlines()]
        first=next((r for r in rows if r['failed_terms']),None)
        summary.append(dict(factors=s['factors'],seed=s['seed'],gate=s['gate'],
            all_saved_ranges=s['all_saved_ranges'],hardware_eligible=False,
            first_failure=dict(time_s=first['time_s'],terms=first['failed_terms'],
                range_conflicts=first['saved_range_conflicts']) if first else None))
        if s['factors'] in ([],['ema','gains','head']):
            env=dict(factors=s['factors'],seed=s['seed'],joints=[])
            for i,sid in enumerate(c['policy_joint_ids']):
                entry=dict(id=sid,saved_min_deg=float(np.rad2deg(c['lower_rad'][i])),saved_max_deg=float(np.rad2deg(c['upper_rad'][i])))
                for key in ('adapter_requested_joint_rad','processed_joint_rad','actual_joint_rad'):
                    values=np.rad2deg([r[key][i] for r in rows]);entry[key]=dict(min_deg=float(values.min()),max_deg=float(values.max()),
                        outside_saved_frames=int(((values<entry['saved_min_deg'])|(values>entry['saved_max_deg'])).sum()))
                active=np.array([r['time_s']>5 and r['command_m_s']>0 for r in rows])
                velocity=np.abs(np.rad2deg([r['actual_joint_velocity_rad_s'][i] for r in rows]))[active]
                entry['actual_speed_abs_p95_deg_s']=float(np.quantile(velocity,.95))
                entry['actual_speed_abs_max_deg_s']=float(velocity.max())
                env['joints'].append(entry)
            envelopes.append(env)
    low=json.loads((OUT/'official-command015-v1-matrix.json').read_text())
    for s in low:
        b=next(x for x in cases if x['seed']==s['seed'] and x['factors']==s['factors'])
        assert s['initial_fingerprint']==b['initial_fingerprint']
        assert s['model_sha256']==expected_model
    result=dict(scope='Unchanged official ONNX, source-domain evidence only',model_sha256=expected_model,
        hardware_eligible=False,production_changed=False,source_reference_has_HOME_reset_corrections=True,
        zero_push=True,exact_original_training_checkpoint_not_proven=True,
        main_matrix=summary,pairings=pairings,envelopes=envelopes,
        command_015=[dict(factors=s['factors'],seed=s['seed'],gate=s['gate']) for s in low],
        preserved_limits=dict(tilt_deg=15,firmware_slew_ticks_s=600,left_ankle_saved_upper_deg=34.10989010989011),
        interpretation=['Source walking acceptance does not include current saved-range approval.',
          'The 600 cap causes failure in these paired simulations; physical causal proof remains separate.',
          'Saved endpoints are not established mechanical stops.',
          'Historical installed-source gains are not a fresh servo-register measurement.',
          'Current 0.15 m/s command is a separate no-step mechanism in these simulations.',
          'No result establishes retraining is necessary or the robot is impossible.'])
    (OUT/'official-causal-v3-evidence.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(cases=len(cases),paired_fields=len(controls[0]['initial_fingerprint']),
        command_cases=len(low),all_pairings_passed=True,hardware_verified=False)))

if __name__=='__main__':main()
