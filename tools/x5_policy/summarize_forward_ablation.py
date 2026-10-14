"""Reduce source-bound frozen traces; never equate reward or falls with walking."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reduce_case(case, trace):
    if sha(trace) != case['trace_sha256']:
        raise ValueError('Frozen trace hash differs')
    rows=[json.loads(s) for s in trace.read_text().splitlines()]
    zero=[r for r in rows if r['phase']=='zero']
    forward=[r for r in rows if r['phase']=='forward']
    eligible=[r for r in forward if r['tilt_deg']<15 and not any(r['range_violations'].values()) and not any(r['termination_terms'].values())]
    anchor=zero[-1]
    def displacement(row):
        dx=row['root_position'][0]-anchor['root_position'][0]
        dy=row['root_position'][1]-anchor['root_position'][1]
        angle=anchor['heading_rad']
        return dict(forward_m=dx*math.cos(angle)+dy*math.sin(angle), lateral_m=-dx*math.sin(angle)+dy*math.cos(angle))
    intervals=[]
    for side in ('left','right'):
        start=None
        for r in eligible:
            foot=r['feet'][side]
            if foot['lifted'] and start is None:
                start=r['index']
            elif start is not None and foot['contact_count']>0:
                intervals.append(dict(side=side,first_lift_index=start,landing_index=r['index']))
                start=None
    intervals.sort(key=lambda item:item['first_lift_index'])
    alternating_nonoverlap_pairs=sum(a['side']!=b['side'] and a['landing_index']<=b['first_lift_index'] for a,b in zip(intervals,intervals[1:]))
    return dict(seed=case['seed'],initial_state_sha256=case['initial_state_sha256'],
        zero_minimum_pass=case['zero_minimum_pass'],schedule_complete=case['full_command_schedule_completed'],
        forward_frames=len(forward),before_boundary_forward_frames=len(eligible),
        lift_frames={side:sum(r['feet'][side]['lifted'] for r in eligible) for side in ('left','right')},
        completed_lift_land_intervals=intervals,alternating_nonoverlap_pairs=alternating_nonoverlap_pairs,
        max_forward_tilt_deg=max(r['tilt_deg'] for r in forward),
        final_before_boundary_displacement=displacement(eligible[-1]) if eligible else None,
        completed_forward_displacement=displacement(forward[-1]) if len(forward)==100 else None,
        after_recovery_displacement=displacement(rows[-1]) if len(rows)==500 else None,
        maximum_request_to_goal_lag_deg=max(abs(q-g)*180/math.pi for r in eligible for q,g in zip(r['requested_rad'],r['goal_rad'])) if eligible else None,
        maximum_goal_to_actual_lag_deg=max(abs(g-q)*180/math.pi for r in eligible for g,q in zip(r['goal_rad'],r['actual_rad'])) if eligible else None,
        original_reward_mean_linear_tracking=case['phases']['forward']['mean_reward_rates']['track_linear_velocity'],
        walking_qualified=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    root=args.root
    plan_path=root/'forward-reward-ablation-plan.json'
    plan=json.loads(plan_path.read_text())
    result=dict(source_sha256={str(Path(__file__)):sha(__file__),str(plan_path):sha(plan_path)},
        experiment='matched original versus relative-speed reward; both freeze the same parent and train a forward residual',
        interpretation='independent original-task evaluation; cached reward is original, not the treatment training reward',
        hardware_access=False,walking_qualified=False,training_converged=False,arms={})
    for arm in ('original','relative-v1'):
        run_path=root/f'forward-residual-{arm}-128/run.json'
        diagnostic=root/f'forward-residual-{arm}-evaluation'
        summary_path=diagnostic/'summary.json'
        r=json.loads(run_path.read_text());s=json.loads(summary_path.read_text())
        if not s['completed']:raise ValueError('Frozen evaluation unfinished')
        result['source_sha256'].update({str(run_path):sha(run_path),str(summary_path):sha(summary_path)})
        result['arms'][arm]=dict(run_id=r['run_id'],export_sha256=r['export']['sha256'],
            actor_before_sha256=r['actor_state_before_sha256'],environment_steps=r['environment_steps'],
            iterations=r['iterations_completed'],optimizer_steps=r['optimizer_steps'],
            source=r['forward_experiment']['actor_initialization'],
            zero_mapping_audit=r['forward_zero_mapping_audit'],
            strict_zero_regression_pass=r['regression_gate']['new_zero_regression_pass'],
            max_admitted_mean_kl=r['checked_optimizer']['maximum_committed_kl'],
            physical_range_failures={k:r[k] for k in ('requested_target_invalid_count','applied_target_invalid_count','actual_joint_invalid_count')},
            training_phase_coverage=r['phase_coverage'],
            cases=[reduce_case(c,diagnostic/f"trained-seed-{c['seed']}.jsonl") for c in s['cases']])
    a,b=result['arms'].values()
    for key in ('actor_before_sha256','environment_steps','iterations','optimizer_steps','source'):
        if a[key]!=b[key]:raise ValueError('Matched ablation differs: '+key)
    if [(c['seed'],c['initial_state_sha256']) for c in a['cases']]!=[(c['seed'],c['initial_state_sha256']) for c in b['cases']]:
        raise ValueError('Independent initial states differ')
    result['matched_initial_actor_training_budget_and_evaluation_states']=True
    output=root/'forward-reward-ablation-summary.json'
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:dict(zero=v['strict_zero_regression_pass'],cases=v['cases']) for k,v in result['arms'].items()},indent=2))


if __name__=='__main__':main()
