"""Read completed offline traces; verify the PICK phase-epoch control pairing."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    root = args.root
    evaluator = root / 'pick-phase-epoch-control-v1/evaluator-v1.py'
    evaluator_sha = sha(evaluator)
    cases = []
    for hz in (50, 40):
        baseline_folder = ('source-workflow-50hz-lag1-factor-v1' if hz == 50
                           else 'source-workflow-timing40-lag1-v1')
        for model_set in ('original', 'selected'):
            candidate = root / f'pick-phase-epoch-control-v1/{hz}-{model_set}'
            baseline = root / baseline_folder / f'pick-rear-high-seed3-{model_set}'
            new = json.loads((candidate / 'gate.json').read_text())
            old = json.loads((baseline / 'gate.json').read_text())
            new_rows = [json.loads(line) for line in (candidate / 'trace.jsonl').open()]
            old_rows = [json.loads(line) for line in (baseline / 'trace.jsonl').open()]
            assert not new.get('error') and not old.get('error')
            assert new['evaluator_sha256'] == evaluator_sha
            assert new['initial_fingerprint'] == old['initial_fingerprint']
            for key in ('models', 'declaration', 'control_hz', 'additional_actor_sensor_lag_steps',
                        'termination_contract', 'evaluation_horizon_s', 'frozen_adapter_sha256'):
                assert new[key] == old[key], key
            assert new['pick_phase_epoch'] == 'sensor'
            assert old.get('pick_phase_epoch', 'application') == 'application'
            assert new_rows[0] == old_rows[0]
            # Before action-induced divergence, frame two differs ONLY in the
            # two phase slots. Subsequent physical divergence is not constrained.
            a, b = new_rows[1]['actor_observation'], old_rows[1]['actor_observation']
            changed = [i for i, (x, y) in enumerate(zip(a, b)) if x != y]
            assert changed == [48, 49], changed
            cases.append(dict(
                control_hz=hz, model_set=model_set, body='rear-high', seed=3,
                sensor_epoch_gate=str((candidate / 'gate.json').relative_to(root)),
                application_epoch_gate=str((baseline / 'gate.json').relative_to(root)),
                full_initial_fingerprint_exact=True,
                models_physics_and_evaluation_contract_exact=True,
                first_complete_trace_row_exact=True,
                second_actor_input_changed_slots=changed,
                second_sensor_epoch_phase=a[48:50], second_application_epoch_phase=b[48:50],
                sensor_epoch_frames=new['frames'], application_epoch_frames=old['frames'],
                sensor_epoch_result=new['gate'], application_epoch_result=old['gate'],
                last_failed_terms=new_rows[-1]['failed_terms'],
                all_later_trace_rows_equal_claimed=False,
                evidence_sha256={str(p.relative_to(root)): sha(p) for p in (
                    candidate/'gate.json', candidate/'trace.jsonl',
                    baseline/'gate.json', baseline/'trace.jsonl')}))
    report = dict(
        hardware_access=False, actual_controller_verified=False,
        scope='One source-derived command-clock factor; four completed offline trajectories, not full async worker/startup/serial emulation',
        immutable_evaluator=str(evaluator.relative_to(root)),
        immutable_evaluator_sha256=evaluator_sha,
        cases=cases, sensor_epoch_passed=sum(x['sensor_epoch_result']['accepted'] for x in cases),
        application_epoch_passed=sum(x['application_epoch_result']['accepted'] for x in cases),
        case_count=len(cases), source_audit='cached-input-pipeline-audit-v1.json',
        conclusion='Matching PICK phase to the delayed sensor epoch does not rescue any of the four tested failures. Physical input age and real failure cause remain unidentified.',
        report_wording_correction='Raw new gate timing_scope inherited the phrase commands remain current; pick_phase_epoch=sensor is authoritative. Only raw history remains current; PICK phase follows sensor epoch. Original evidence retained.',
        no_new_threshold_reward_model_or_real_calibration_changes=True,
        summary_script_sha256=sha(Path(__file__)))
    (root/'pick-phase-epoch-control-summary-v1.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('sensor_epoch_passed', 'application_epoch_passed', 'case_count', 'conclusion')}))


if __name__ == '__main__':
    main()
