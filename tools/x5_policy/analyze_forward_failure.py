"""Paired saved-observation actor comparison; no environment or hardware run."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','run','output'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    from constrained_training_candidate import load_candidate,digest
    import onnxruntime as ort
    candidate=load_candidate(a.candidate)
    run=json.loads((a.run/'run.json').read_text())
    if digest(candidate)!=run['candidate_sha256']:raise ValueError('Candidate binding differs')
    if not run.get('checkpoint',{}).get('optimizer_full_state_reload_identical'):
        raise ValueError('Unverified checkpoint')
    models=dict(original=Path(candidate['source_model']['path']),
                bounded_baseline=Path(run['validation_baseline']['path']),
                trained=a.run/'unvalidated_actor.onnx')
    trace=a.run/'regression-gate/candidate.trace.jsonl'
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    sessions={name:ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
              for name,path in models.items()}
    actions={}
    for name,session in sessions.items():
        values=[]
        for row in rows:
            obs=np.asarray(row['actor_observation'],np.float32).reshape(1,61)
            values.append(np.asarray(session.run(None,{session.get_inputs()[0].name:obs})[0]).reshape(14))
        actions[name]=np.asarray(values)
    recorded=np.asarray([r['raw_action'] for r in rows])
    replay_error=float(np.max(np.abs(actions['trained']-recorded)))
    if replay_error>1e-6:raise ValueError('Saved actor output does not reproduce')
    factor=candidate['action_scale']*180/np.pi
    ids=candidate['policy_joint_ids'];legs=[i for i,sid in enumerate(ids) if sid not in (30,31,32,33)]
    first_boundary=next((i for i,r in enumerate(rows) if i>=250 and r['tilt_deg']>=15),len(rows))
    report=dict(hardware_access=False,environment_steps=0,optimizer_steps=0,
        source_sha256={str(path):sha(path) for path in [a.candidate,trace,a.run/'run.json',*models.values()]},
        candidate_sha256=digest(candidate),actor_replay_max_error=replay_error,
        first_forward_15_degree_boundary=None if first_boundary==len(rows) else
            dict(index=first_boundary,time_s=rows[first_boundary]['sim_time_s'],
                 after_forward_s=rows[first_boundary]['sim_time_s']-5.,tilt_deg=rows[first_boundary]['tilt_deg']),
        scope='same recorded observations; actor effects separated from closed-loop state drift; no counterfactual plant/gait proof',
        phases={})
    for phase,start,end in [('zero',0,250),('forward_before_boundary',250,first_boundary)]:
        if end<=start:continue
        delta=(actions['trained'][start:end]-actions['bounded_baseline'][start:end])*factor
        transform=(actions['bounded_baseline'][start:end]-actions['original'][start:end])*factor
        requested=np.asarray([r['requested_rad'] for r in rows[start:end]])*180/np.pi
        applied=np.asarray([r['applied_rad'] for r in rows[start:end]])*180/np.pi
        actual=np.asarray([r['actual_rad'] for r in rows[start:end]])*180/np.pi
        if requested.shape[1]!=14:raise ValueError('Unexpected policy joint layout')
        per_joint=[]
        for j in legs:
            per_joint.append(dict(id=ids[j],training_max_abs_same_obs_target_delta_deg=float(np.max(np.abs(delta[:,j]))),
                bounding_max_abs_same_obs_target_delta_deg=float(np.max(np.abs(transform[:,j]))),
                requested_span_deg=float(np.ptp(requested[:,j])),
                applied_span_deg=float(np.ptp(applied[:,j])),actual_span_deg=float(np.ptp(actual[:,j])),
                max_request_to_applied_deg=float(np.max(np.abs(requested[:,j]-applied[:,j]))),
                max_applied_to_actual_deg=float(np.max(np.abs(applied[:,j]-actual[:,j])))))
        report['phases'][phase]=dict(frames=end-start,joints=per_joint,
            both_contact_frames=sum(all(r['feet'][s]['contact_count']>0 for s in ('left','right')) for r in rows[start:end]),
            lift_frames={s:sum(r['feet'][s]['lifted'] for r in rows[start:end]) for s in ('left','right')},
            max_training_target_change_deg=float(np.max(np.abs(delta[:,legs]))),
            max_bounding_target_change_deg=float(np.max(np.abs(transform[:,legs]))))
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('actor_replay_max_error','first_forward_15_degree_boundary')}))


if __name__=='__main__':main()
