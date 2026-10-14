"""Audit the completed, hypothetical same-total-mass offline comparisons."""
import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np


def main():
    root = Path(__file__).resolve().parents[2]
    out = root / 'output/duck_sim/source-parity-20261012/backpack-mass-distribution'
    scenarios = ['total871-reference', 'total871-relocated71-rear30', 'total871-relocated71-rear90']
    loaded, cases, pairs = {}, [], []
    for seed in (0, 3):
        for name in scenarios:
            path = out / f'{name}-seed{seed}-v2' / 'summary.json'
            row = json.loads(path.read_text())
            loaded[seed, name] = row
            assert row['complete'] and row['frames'] == 700 and row['official_runtime_simulation_accepted']
            cases.append(dict(seed=seed, scenario=name, frames=row['frames'], max_tilt_deg=row['max_tilt_deg'],
                zero_drift_mm=row['gate']['zero_drift_m']*1000,
                all_actual_ranges=row['all_actual_ranges'], all_transmitted_ranges=row['all_transmitted_ranges'],
                zero_simulation_accepted=row['gate']['accepted'],
                nominal_mass_g=1000*row['actual_body_properties']['nominal_total_mass_kg'],
                effective_randomized_mass_g=1000*row['actual_body_properties']['total_mass_kg'],
                source_model_sha256=row['model_sha256'], calibration_sha256=row['calibration_sha256'],
                summary_path=str(path.relative_to(root)), summary_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        base = loaded[seed, scenarios[0]]
        ref = base['initial_fingerprint']
        for name in scenarios[1:]:
            row = loaded[seed, name]
            fp = row['initial_fingerprint']
            prefixes = ('motor.', 'bam.', 'bam_actuator.', 'delay.', 'torch_rng')
            changed = [key for key in ref if key.startswith(prefixes) and ref[key] != fp[key]]
            assert not changed
            assert base['initial_qpos_sha256'] == row['initial_qpos_sha256']
            assert base['initial_qvel_sha256'] == row['initial_qvel_sha256']
            mb = np.array(base['actual_body_properties']['body_mass_kg'])
            mr = np.array(row['actual_body_properties']['body_mass_kg'])
            mass_rounding = np.spacing(np.maximum(mb, mr).astype(np.float32)).astype(float)
            mass_diff = np.abs(mb-mr)
            assert np.all(mass_diff <= mass_rounding)
            cb = np.array(base['actual_body_properties']['body_ipos_m'])
            cr = np.array(row['actual_body_properties']['body_ipos_m'])
            indices = np.flatnonzero(np.any(cb != cr, axis=1))
            assert len(indices) == 1
            index = int(indices[0])
            application = row['mass_distribution']['physics_application']
            old = np.array(application['original_trunk_com_m'])
            new = np.array(application['new_trunk_com_m'])
            # Two float32 COM fields include nominal rounding plus the same
            # random additive displacement. Bound their subtraction using the
            # ULPs of the operands, not the much smaller ULP of the difference.
            rounding = sum(np.abs(np.spacing(value.astype(np.float32)).astype(float))
                           for value in (cb[index], cr[index], old, new))
            assert np.all(np.abs((cr[index]-cb[index])-(new-old)) <= rounding)
            pairs.append(dict(seed=seed, scenario=name, initial_qpos_and_qvel_identical=True,
                random_state_BAM_gains_voltage_friction_and_delay_identical=True,
                changed_noninertial_motor_random_fields=changed, nontrunk_COM_fields_identical=True,
                actual_trunk_index_in_scene=index, nominal_total_mass_identical=True,
                maximum_effective_mass_difference_g=float(mass_diff.max()*1000),
                effective_mass_within_float32_one_ULP=True,
                effective_COM_difference_matches_declared_change_with_operand_rounding=True,
                COM_rounding_bound_m=rounding.tolist(),
                nominal_whole_robot_COM_translation_trunk_frame_mm=(
                    (new-old)*application['trunk_mass_kg']/.871*1000).tolist(),
                effective_trunk_COM_translation_mm=((cr[index]-cb[index])*1000).tolist(),
                effective_whole_COM_translation_along_shared_trunk_X_mm=float(
                    (cr[index, 0]-cb[index, 0])*mr[index]/mr.sum()*1000)))
    result = dict(updated_at=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        hardware_access=False, deployed=False, cases=cases, pairing_proofs=pairs,
        completed_cases=6, qualified_zero_cases=6,
        task='14s zero command; original ONNX/BAM/EMA.45/KP6, saved ranges+integer goals+existing head projection,50Hz/no pushes.',
        comparison='Same initial qpos/qvel, controls and RNG; declared uniform mass/inertia scaling, then trunk COM and parallel-axis inertia relocation.',
        limitations=[
            '871g is compile-time nominal; original startup mass randomization remains. Effective mass reported per seed.',
            '71g is unmeasured existing trunk mass in an example, not new mass or measured board weight.',
            'Destination height equals original trunk COM; real board height/full inertia/COM unknown.',
            'These cases do not reproduce backwardfall and do not identify or exclude real rear COM as a cause.',
            'Zero-loop stability is not fixed-HOLD free standing, walking or real-device acceptance.'],
        conclusion='All six zero loops remain stable under this declared hypothesis. No justified real software compensation amount or root cause follows.',
        summarizer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        production_recipe_sha256=hashlib.sha256((root/'tools/source_parity/current-changes.json').read_bytes()).hexdigest())
    (out/'summary-v2.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(completed_cases=6, accepted_zero_cases=6, paired_initial_conditions=True,
                         max_tilt_deg=max(row['max_tilt_deg'] for row in cases))))


if __name__ == '__main__':
    main()
