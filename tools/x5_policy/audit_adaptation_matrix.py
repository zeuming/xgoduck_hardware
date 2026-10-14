"""Small source-backed offline comparisons, never a parameter search or deployment.

Each declared comparison changes one factor. Every execution stage is checked
against the same saved constraints; no range expansion, target clipping or
range-ignoring continuation can make a candidate pass.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def source_comparisons():
    base = dict(decoder='metadata', hz=50, slew_ticks_s=0, action_alpha=.45,
                head_action_alpha=.45, head_source='captured', executor='fixed-head',
                physics_contract='legacy-cpu')
    rows = []

    def case(name, values, against, factor, source):
        row = dict(name=name, settings=values, comparison_to=against,
                   changed_factor=factor, source_basis=source)
        rows.append(row)
        return values

    case('legacy-runtime-filter', base, None, None,
         'Reproduce the prior offline baseline; metadata decoder, official hardware-runtime EMA .45')
    official = case('official-physics', dict(base, physics_contract='official-bam'),
         'legacy-runtime-filter', 'physics_contract',
         'Official BAM friction constraints and XGO named collision settings')
    case('official-physics-current-slew', dict(official, slew_ticks_s=600),
         'official-physics', 'goal_slew',
         '600 ticks/s is the installed firmware limit; training BAM max_velocity=100 effectively disables it')
    direct = case('official-physics-direct-actions', dict(official, action_alpha=0., head_action_alpha=0.),
         'official-physics', 'action_filter',
         'Current training JointPositionAction and generic official inference apply actor actions directly')
    full = case('official-physics-full-policy', dict(direct, executor='policy'),
         'official-physics-direct-actions', 'head_execution',
         'Official action contract controls all 14 policy joints; compare against runtime frozen head')
    case('official-physics-full-policy-home-head', dict(full, head_source='metadata-home'),
         'official-physics-full-policy', 'head_setpoint',
         'Selected model metadata HOME, zero head command offset; no invented posture or safe-range changes')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mass-kg', type=float, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    program = Path(__file__).with_name('audit_policy_sim.py')
    common = [sys.executable, str(program), '--snapshot', str(args.snapshot),
              '--initial-pose', 'snapshot', '--duration-s', '10',
              '--feedback-delay-ms', '20', '--execution-delay-ms', '20',
              '--mass-kg', str(args.mass_kg), '--kp', '6',
              '--zero-warmup-s', '5', '--pulse-s', '2', '--ramp-seconds', '.5',
              '--speeds', '0', '.2']
    rows = []
    for case in source_comparisons():
        values = case['settings']
        output = args.output/(case['name']+'.json')
        command = common + ['--decoder', values['decoder'], '--hz', str(values['hz']),
            '--slew-ticks-per-second', str(values['slew_ticks_s']),
            '--action-alpha', str(values['action_alpha']),
            '--head-action-alpha', str(values['head_action_alpha']),
            '--head-source', values['head_source'], '--executors', values['executor'],
            '--physics-contract', values['physics_contract'], '--output', str(output)]
        subprocess.run(command, check=True)
        for result in json.loads(output.read_text())['results']:
            rows.append(dict(case=case['name'], settings=values,
                comparison_to=case['comparison_to'], changed_factor=case['changed_factor'],
                source_basis=case['source_basis'], speed_m_s=result['speed_m_s'], ended=result['ended'],
                max_tilt_deg=result['max_tilt_deg'], displacement_m=result['displacement_m'],
                command_progress=result['command_progress'],
                assessment=result['assessment'], evidence_file=output.name))
    report = dict(kind='source_backed_offline_adaptation_comparisons', hardware_access=False,
        deployed=False, physical_walking_verified=False,
        snapshot_sha256=hashlib.sha256(args.snapshot.read_bytes()).hexdigest(),
        simulator_sha256=hashlib.sha256(program.read_bytes()).hexdigest(),
        fixed_assumptions=dict(mass_kg=args.mass_kg, mass_distribution='uniform CAD scaling; unmeasured',
            actuator_kp=6, actuator_kp_provenance='historical_uniform_simulation_hypothesis_not_installed_firmware',
            voltage_v=7.7, feedback_delay_ms=20, execution_delay_ms=20,
            initial_root_height_m=.14, initial_roll_pitch_deg=[0,0],
            observation='ideal simulated gravity',
            command='zero 5s, ramped forward 2s, zero 3s'),
        limits=['Source-backed single factors, not an exhaustive search for a passing parameter set.',
                '50 Hz/no slew are offline comparisons; neither claims the physical loop or firmware already matches.',
                'Exact exported-policy checkpoint/config and loaded physical plant remain unidentified.',
                'CPU MuJoCo/BAM differs from the GPU training environment.',
                'No screen pass authorizes physical motion or expands saved bounds.'], cases=rows)
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(cases=len(rows), standing_passed=sum(
        r['speed_m_s']==0 and r['assessment']['offline_screen_passed'] for r in rows),
        walking_passed=sum(r['speed_m_s']!=0 and r['assessment']['offline_screen_passed'] for r in rows),
        physical_walking_verified=False)))


if __name__ == '__main__':
    main()
