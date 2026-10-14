#!/usr/bin/env python3
"""Independent CPU baseline/candidate zero-speed regression rejection gate.

Passing the 15-degree experimental observation boundary is only offline zero
evidence. It is not a validated fall threshold, gait acceptance, or hardware permit.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
EVALUATOR=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def judge_trace(report,rows,candidate_sha,model_sha):
    from stand_button_training_profile import PROFILE,PULSE_SHA,source_pulse
    if report.get('stage')!='smoke_completed':raise ValueError('Independent evaluation did not complete')
    if report['candidate_sha256']!=candidate_sha or report['evaluated_policy']['sha256']!=model_sha:
        raise ValueError('Evaluation provenance mismatch')
    if report['policy_diagnostic']['automatic_reset'] is not False:raise ValueError('Evaluation used automatic resets')
    if report['policy_diagnostic']['seed']!=0:raise ValueError('Evaluation seed differs from fixed baseline protocol')
    if report['button_command_source']['sha256']!=PULSE_SHA:raise ValueError('Evaluation command source mismatch')
    for key in ('warmup_s','duration_s','ramp_s','recovery_s','peak_m_s'):
        if report['button_command_source'][key]!=PROFILE[key]:raise ValueError('Command constants mismatch')
    if len(rows)!=report['steps_completed']:raise ValueError('Incomplete evaluation trace')
    if not 0<len(rows)<=500:raise ValueError('Evaluation trace exceeds fixed budget or is empty')
    pulse=source_pulse()
    for index,row in enumerate(rows):
        if len(row['actor_observation'])!=61:raise ValueError('Actor observation layout differs')
        if not all(math.isfinite(value) for value in row['actor_observation']):raise ValueError('Nonfinite actor observation')
        if not math.isfinite(row['sim_time_s']) or not math.isfinite(row['speed_m_s']):raise ValueError('Nonfinite trace command or time')
        if len(row['root_position'])!=3 or not all(math.isfinite(v) for v in row['root_position']):
            raise ValueError('Nonfinite or incomplete root position')
        speed=pulse.command_at(index*.02,0)[0] if index>=250 else 0.
        if row['index']!=index or abs(row['sim_time_s']-(index+1)*.02)>1e-8:
            raise ValueError('Trace timestamps are not contiguous 50Hz physics steps')
        if not math.isclose(row['speed_m_s'],speed,abs_tol=1e-7):raise ValueError('Trace command differs')
        if any(abs(a-b)>1e-7 for a,b in zip(row['actor_observation'][48:51],[speed,0,0])):
            raise ValueError('Actual actor observations used a different command')
        if not math.isfinite(row['tilt_deg']):raise ValueError('Nonfinite actual tilt')
    zero=rows[:250]
    causes=[]
    if len(zero)!=250:causes.append('five_second_zero_incomplete')
    if any(row['tilt_deg']>=15 for row in zero):causes.append('experimental_15_degree_boundary_crossed')
    if any(row['termination_terms'] for row in zero):causes.append('environment_termination')
    if any(row['violations'] for row in zero):causes.append('requested_applied_or_actual_range_violation')
    if any(row['contact_sensor_consistency_passed'] is not True for row in rows):
        raise ValueError('Sensor/raw physics comparison failed')
    forward=rows[250:350]
    initial=report['reset_actual_plant']['root_position']
    drift=math.hypot(zero[-1]['root_position'][0]-initial[0],zero[-1]['root_position'][1]-initial[1]) if zero else None
    if drift is not None and not math.isfinite(drift):raise ValueError('Nonfinite zero-phase displacement')
    return dict(zero_minimum_pass=not causes,rejection_reasons=causes,zero_steps=len(zero),
        zero_max_tilt_deg=max([r['tilt_deg'] for r in zero],default=None),
        zero_horizontal_drift_m=drift,
        steps=len(rows),phase_metrics=report['phase_metrics'],
        forward_necessary_evidence=dict(complete_100_frames=len(forward)==100,
            both_feet_lifted=bool(forward) and all(any(r['feet'][s]['lifted'] for r in forward) for s in ('left','right')),
            no_15_degree_crossing=bool(forward) and all(r['tilt_deg']<15 for r in forward),
            no_termination_or_range_failure=bool(forward) and all(not r['termination_terms'] and not r['violations'] for r in forward)),
        forward_gait_pass=False,forward_gait_status='not_accepted_by_this_zero_regression_gate',
        hardware_eligible=False)

def compare_zero_evidence(baseline,candidate):
    """Strict two-metric non-regression; no tolerance or whole-gait claim."""
    baseline_pass=baseline['zero_minimum_pass'];minimum=candidate['zero_minimum_pass']
    differences={key:candidate[key]-baseline[key] for key in ('zero_max_tilt_deg','zero_horizontal_drift_m')}
    no_increase=all(value<=0 for value in differences.values())
    regression_pass=baseline_pass and minimum and no_increase
    status=('baseline_not_established' if not baseline_pass else
            'candidate_rejected_zero_minimum' if not minimum else
            'rejected_margin_regression' if not no_increase else
            'two_metric_zero_regression_passed_gait_unaccepted')
    return dict(baseline_pass=baseline_pass,zero_minimum_pass=minimum,
        new_zero_regression_pass=regression_pass,status=status,
        relative_zero_metrics=dict(candidate_minus_baseline=differences,both_metrics_non_increasing=no_increase,
            rule='same deterministic seed and initial protocol; maximum zero-phase tilt and horizontal drift must each be <= baseline, with no expanded tolerance',
            scope='only these two zero-phase metrics; not all stability measures or walking acceptance'))

def run_gate(candidate_path,baseline,candidate_model,output):
    from constrained_training_candidate import load_candidate,digest
    from train_constrained_policy import install_offline_guards
    from stand_button_training_profile import PULSE_SOURCE,PULSE_SHA
    install_offline_guards()
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    candidate=load_candidate(candidate_path)
    profile_source=Path(__file__).with_name('stand_button_training_profile.py')
    guard_source=Path(__file__).with_name('train_constrained_policy.py')
    pins={str(p):sha(p) for p in (Path(__file__),EVALUATOR,PULSE_SOURCE,profile_source,guard_source,
        Path(candidate_path),Path(baseline),Path(candidate_model))}
    for name,path in [('gate-source.py',Path(__file__)),('evaluator-source.py',EVALUATOR),('button-source.py',PULSE_SOURCE),
                      ('profile-source.py',profile_source),('offline-guard-source.py',guard_source)]:
        (output/name).write_bytes(path.read_bytes())
    result=dict(candidate_sha256=digest(candidate),sources=pins,baseline_pass=False,
        zero_minimum_pass=False,new_zero_regression_pass=False,forward_gait_pass=False,walking_verified=False,
        hardware_eligible=False,training_converged=False,experimental_tilt_boundary_deg=15,
        scope='independent deterministic CPU zero regression; no hardware or gait qualification')
    try:
        for label,model in [('baseline',baseline),('candidate',candidate_model)]:
            destination=output/(label+'.json')
            bootstrap="import sys,runpy;sys.path.insert(0,sys.argv.pop(1));from train_constrained_policy import install_offline_guards;install_offline_guards();sys.argv.pop(0);runpy.run_path(sys.argv[0],run_name='__main__')"
            command=[sys.executable,'-c',bootstrap,str(ROOT/'tools/x5_policy'),str(EVALUATOR),
                '--candidate',str(Path(candidate_path).resolve()),'--old-policy-diagnostic','--button-pulse',
                '--policy-onnx',str(Path(model).resolve()),'--output',str(destination)]
            with (output/(label+'.log')).open('w') as log:
                completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=90)
            if completed.returncode:raise ValueError(label+' independent evaluation failed; inspect saved log')
            report=json.loads(destination.read_text())
            rows=[json.loads(line) for line in destination.with_suffix('.trace.jsonl').read_text().splitlines()]
            result[label]=judge_trace(report,rows,digest(candidate),sha(model))
        result.update(compare_zero_evidence(result['baseline'],result['candidate']))
        if any(sha(p)!=value for p,value in pins.items()):raise ValueError('Pinned sources changed during gate')
        load_candidate(candidate_path)
    except Exception as error:
        result.update(status='gate_failed_closed',error=str(error),new_zero_regression_pass=False)
    (output/'gate.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result

def rejudge_saved_evaluations(candidate_path,baseline,candidate_model,prior,output):
    """Reinterpret unchanged bound full traces after a gate-rule correction.

    No simulation/training occurs. Preserve old reports, pin every input byte,
    and state both the original evaluator and new decision-code provenance.
    """
    from constrained_training_candidate import load_candidate,digest
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    prior=Path(prior).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    candidate=load_candidate(candidate_path);old=json.loads((prior/'gate.json').read_text())
    recorded_evaluator=old['sources'][str(EVALUATOR)]
    if sha(prior/'evaluator-source.py')!=recorded_evaluator:raise ValueError('Archived evaluator differs from original gate binding')
    result=dict(candidate_sha256=digest(candidate),rejudgment_only=True,environment_steps=0,optimizer_steps=0,
        source_evaluation_directory=str(prior),original_gate_status=old['status'],
        original_evaluator_sha256=recorded_evaluator,current_gate_sha256=sha(__file__),
        input_sha256={},forward_gait_pass=False,walking_verified=False,hardware_eligible=False,
        experimental_tilt_boundary_deg=15,scope='corrected two-metric zero regression on unchanged complete original evaluation traces')
    for label,model in [('baseline',baseline),('candidate',candidate_model)]:
        report_path=prior/(label+'.json');trace=prior/(label+'.trace.jsonl')
        for path in (report_path,trace,Path(model)):
            result['input_sha256'][str(path)]=sha(path)
        report=json.loads(report_path.read_text());rows=[json.loads(line) for line in trace.read_text().splitlines()]
        if old[label]['steps']!=len(rows):raise ValueError('Trace length differs from bound historical gate')
        result[label]=judge_trace(report,rows,digest(candidate),sha(model))
    result.update(compare_zero_evidence(result['baseline'],result['candidate']))
    (output/'gate-source.py').write_bytes(Path(__file__).read_bytes())
    (output/'gate.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('candidate','baseline-onnx','candidate-onnx','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--rejudge-from',type=Path,help='Use and bind complete saved traces; no new physics or training')
    args=parser.parse_args()
    result=(rejudge_saved_evaluations(args.candidate,args.baseline_onnx,args.candidate_onnx,args.rejudge_from,args.output)
            if args.rejudge_from else run_gate(args.candidate,args.baseline_onnx,args.candidate_onnx,args.output))
    print(json.dumps({key:result[key] for key in ('status','baseline_pass','new_zero_regression_pass','forward_gait_pass','hardware_eligible')}))
    return 0 if result['new_zero_regression_pass'] else 2

if __name__=='__main__':raise SystemExit(main())
