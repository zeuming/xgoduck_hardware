#!/usr/bin/env python3
"""Build an auditable model/plant contract from local immutable artifacts.

This command imports no hardware or controller modules and never authorizes
motion. Saved encoder endpoints are constraints, not measured mechanical stops.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import re
from pathlib import Path

from rl_core import (MOTOR_IDS, POLICY_IDS, POLICY_JOINT_NAMES,
                     POLICY_OBSERVATION_NAMES, policy_profile)


ROOT = Path(__file__).resolve().parents[2]
LAST_VERIFIED_GAIN_SOURCE = ROOT/'output/duck_sim/position-read-retry-candidate/firmware-source/firmware/main.cpp'
LAST_VERIFIED_INSTALL_RECEIPT = ROOT/'output/duck_sim/position-read-retry-install/installation-status.json'
LEGACY_CANDIDATE_ROOT = ROOT/'output/duck_sim/adaptation-20261011/candidate/policy'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numeric_assignment(path, name, *, radians_call=False):
    """Read known source literals without importing/executing reference code."""
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name
                                                for t in node.targets):
            value = node.value
            if radians_call:
                if (not isinstance(value, ast.Call) or len(value.args) != 1 or value.keywords
                        or not isinstance(value.func, ast.Attribute)
                        or not isinstance(value.func.value, ast.Name)
                        or value.func.value.id != 'math' or value.func.attr != 'radians'):
                    raise ValueError(f'{name} no longer has the audited literal radians form')
                value = value.args[0]
            if isinstance(value, ast.Constant) and type(value.value) in (int, float):
                return float(value.value)
    raise ValueError(f'Cannot establish source assignment {name}')


def source_gain_manifest(source=LAST_VERIFIED_GAIN_SOURCE, receipt=LAST_VERIFIED_INSTALL_RECEIPT):
    """Extract the reviewed C++ gain function's literals; fail on changed logic.

    The installation receipt is historical provenance. This never reads a servo
    register or infers that the current device still runs this application.
    """
    text = Path(source).read_text()
    match = re.search(r'uint8_t runKpForId\(uint8_t id\)\s*\{([^}]+)\}', text)
    if not match:
        raise ValueError('Missing reviewed runKpForId function')
    body = re.sub(r'//[^\n]*', '', match.group(1))
    parsed = re.fullmatch(r'\s*if\s*\(id\s*==\s*(\d+)\)\s*return\s+(\d+);\s*'
        r'return\s*\(id\s*==\s*(\d+)\s*\|\|\s*id\s*==\s*(\d+)\)\s*'
        r'\?\s*(\d+)\s*:\s*(\d+);\s*', body)
    setup = re.search(r'const uint8_t kp\s*=\s*(\d+),\s*kd\s*=\s*(\d+),\s*mouthKp\s*=\s*(\d+);', text)
    if not parsed or not setup:
        raise ValueError('Gain source logic changed; requires a source audit')
    high_id, high, a, b, paired, default = map(int, parsed.groups())
    base, kd, mouth = map(int, setup.groups())
    kp = {str(sid):high if sid == high_id else paired if sid in (a,b) else default for sid in MOTOR_IDS}
    if base != default or kp['34'] != mouth or any(not 0 < v <= 100 for v in kp.values()):
        raise ValueError('Gain setup and function literals are inconsistent')
    installed = json.loads(Path(receipt).read_text())
    if installed.get('firmware_installed') is not True or installed.get('readback_verified') is not True:
        raise ValueError('Historical receipt does not verify installation readback')
    return dict(schema_version=1, kind='source_declared_servo_gains',
        status='last_verified_installed_source_expected', current_servo_registers_measured=False,
        current_device_application_verified=False,
        kp_by_id=kp, kd_by_id={str(sid):kd for sid in MOTOR_IDS},
        source=dict(path=str(Path(source).resolve()), sha256=sha256(source), function='runKpForId/prepareRunGains'),
        installation_receipt=dict(path=str(Path(receipt).resolve()), sha256=sha256(receipt),
            application_sha256=installed['application_sha256'], readback_verified=True),
        limitation='Historical source/installation evidence, not a current register measurement; BAM does not identify the firmware KD register independently.')


def load_gain_manifest(path):
    manifest = json.loads(Path(path).read_text())
    kp = manifest.get('kp_by_id', {})
    if (manifest.get('kind') != 'source_declared_servo_gains'
            or set(kp) != {str(sid) for sid in MOTOR_IDS}
            or any(type(v) not in (int,float) or not math.isfinite(v) or not 0 < v <= 100 for v in kp.values())):
        raise ValueError('Gain manifest needs all 15 finite positive per-ID gains')
    source = manifest.get('source', {})
    receipt = manifest.get('installation_receipt', {})
    if sha256(source['path']) != source.get('sha256') or sha256(receipt['path']) != receipt.get('sha256'):
        raise ValueError('Gain source/receipt hash no longer matches manifest')
    if manifest != source_gain_manifest(Path(source['path']), Path(receipt['path'])):
        raise ValueError('Gain manifest values or provenance disagree with reviewed source')
    return manifest


def legacy_candidate_contract():
    """Describe and pin deployed-candidate math separately from ONNX metadata."""
    path = LEGACY_CANDIDATE_ROOT/'candidate_controller.py'
    tree = ast.parse(path.read_text())
    offsets = None
    values = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'HOME_OFFSETS_DEG' for t in node.targets):
            offsets = ast.literal_eval(node.value)
        if isinstance(node, ast.keyword) and node.arg in (
                's95_roll_scale', 'C1_exponential_threshold_deg', 'each_roll_saved_bound_interior_ticks'):
            values[node.arg] = ast.literal_eval(node.value)
    if offsets != {12:-1.,14:-1.,22:1.,24:1.} or values != {
            's95_roll_scale':.95, 'C1_exponential_threshold_deg':16.,
            'each_roll_saved_bound_interior_ticks':3}:
        raise ValueError('Legacy candidate transformations changed; source audit required')
    return dict(status='previously_deployed_candidate_transformations_not_hardware_qualified',
        source_files={name:dict(path=str((LEGACY_CANDIDATE_ROOT/name).resolve()),
            sha256=sha256(LEGACY_CANDIDATE_ROOT/name)) for name in (
                'candidate_controller.py','candidate_math.py','production_motion.py','rl_core.py')},
        home_offsets_deg=offsets,
        home_offset_scope='Changed walk HOME is used for both relative observations and decoded targets',
        roll_transform=dict(ids=[11,21], policy_indices=[1,10], scale=values['s95_roll_scale'],
            identity_center_deg=[-16,16], saved_bound_interior_ticks=3,
            tail='C1 monotone exponential approach to saved interior bounds', before_ema=True),
        execution_order=['metadata HOME plus declared offsets', 'raw ONNX actor action',
            'roll S95 scale and exponential angle map', 'EMA', 'degree targets',
            'captured fixed head override where runtime fixed-head walk is active', 'firmware goal slew'],
        raw_previous_action='original unmodified actor output',
        fixed_head_ids=[30,31,32,33], physical_effect_validated=False,
        metadata_decoder_is_actual_deployed_pipeline=False,
        simulator_selector='--decoder homeprime-roll95 --executors fixed-head --head-source captured',
        current_device_configuration_verified=False,
        note='Describes frozen prior deployment sources; this contract neither restores, removes nor approves these transformations.')


def saved_joint_contract(calibration):
    if calibration.get('version') != 1:
        raise ValueError('Calibration version must be 1')
    joints = calibration.get('joints')
    if not isinstance(joints, list) or [j.get('id') for j in joints] != MOTOR_IDS:
        raise ValueError('Need exactly 15 joints in MOTOR_IDS order')
    result = []
    for j in joints:
        if (any(type(j.get(k)) is not int for k in ('zero', 'sign', 'min', 'max'))
                or j['sign'] not in (-1, 1)
                or not 0 <= j['min'] <= j['zero'] <= j['max'] <= 4095
                or j['min'] == j['max']):
            raise ValueError(f'Invalid saved calibration for ID {j.get("id")}')
        lo, hi = sorted((tick-j['zero'])*360/4095*j['sign'] for tick in (j['min'], j['max']))
        result.append(dict(id=j['id'], policy_index=POLICY_IDS.index(j['id']) if j['id'] in POLICY_IDS else None,
            name=POLICY_JOINT_NAMES[POLICY_IDS.index(j['id'])] if j['id'] in POLICY_IDS else 'mouth',
            zero_tick=j['zero'], sign=j['sign'], min_tick=j['min'], max_tick=j['max'],
            min_deg=lo, max_deg=hi, kind='preserved_saved_constraint',
            physical_hard_stop_verified=False, collision_margin_verified=False))
    return result


def screen_evidence(reports, *, model_hash, calibration_hash):
    """Recompute screens; never trust a cached pass bit or unrelated artifacts."""
    from audit_policy_sim import assess_result
    evidence = []
    for path in reports:
        report = json.loads(Path(path).read_text())
        matching = (report.get('source_sha256', {}).get('model') == model_hash
                    and report.get('calibration_sha256') == calibration_hash)
        counterfactual = report.get('settings', {}).get('continue_after_range_violation', True)
        rows = []
        for result in report.get('results', []):
            assessment = assess_result(result)
            rows.append(dict(speed_m_s=result['speed_m_s'], executor=result.get('executor'),
                offline_screen_passed=bool(matching and not counterfactual and assessment['offline_screen_passed']),
                failures=assessment['failures']))
        evidence.append(dict(path=str(Path(path).resolve()), sha256=sha256(path),
            matching_model_and_calibration=matching,
            range_ignoring_counterfactual=bool(counterfactual), results=rows))
    return evidence


def build_contract(snapshot, session, *, model_path, reference_root, runtime_root,
                   mass_kg, reports=()):
    if not math.isfinite(mass_kg) or not 0 < mass_kg <= 5:
        raise ValueError('Mass must be finite and positive')
    calibration = snapshot['calibration']
    joints = saved_joint_contract(calibration)
    profile = policy_profile(session)
    model_hash = sha256(model_path)
    calibration_hash = hashlib.sha256(json.dumps(calibration, sort_keys=True,
        separators=(',', ':')).encode()).hexdigest()
    sources = {
        'training_robot': reference_root/'src/mjlab_microduck/robot/xgoduck_constants.py',
        'training_task': reference_root/'src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py',
        'training_bam': reference_root/'vendor/bam/bam/mjlab.py',
        'cpu_bam': reference_root/'vendor/bam/bam/mujoco.py',
        'actuator_parameters': reference_root/'src/mjlab_microduck/robot/xgoduck/params/1910_m6.json',
        'runtime_math': runtime_root/'python/rl_core.py',
        'runtime_policy': runtime_root/'python/policy.py',
    }
    evidence = screen_evidence(reports, model_hash=model_hash, calibration_hash=calibration_hash)
    gain_source = source_gain_manifest()
    candidate_math = legacy_candidate_contract()
    rows = [r for e in evidence for r in e['results']]
    walking_pass = any(r['speed_m_s'] != 0 and r['offline_screen_passed'] for r in rows)
    standing_pass = any(r['speed_m_s'] == 0 and r['offline_screen_passed'] for r in rows)
    coherent_candidate = any(
        any(r['speed_m_s'] == 0 and r['offline_screen_passed'] for r in e['results'])
        and any(r['speed_m_s'] != 0 and r['offline_screen_passed'] for r in e['results'])
        for e in evidence)
    gates = [
        dict(id='model_io_contract', status='passed', reason='ONNX tensor and metadata layout validated'),
        dict(id='training_provenance_or_new_reproducible_candidate', status='unverified',
             reason='Exact checkpoint/config for this export has not been established; a new candidate would need its own manifest'),
        dict(id='saved_range_standing_screen', status='passed' if standing_pass else 'not_passed',
             reason='Same model/calibration, all saved bounds, no counterfactual continuation'),
        dict(id='saved_range_supported_walking_screen', status='passed' if walking_pass else 'not_passed',
             reason='Completed supported landed lifts plus progress; all requested/applied/actual ranges respected'),
        dict(id='same_configuration_zero_and_walking', status='passed' if coherent_candidate else 'not_passed',
             reason='One report/settings must pass both zero and walking; separate configurations cannot be combined into a candidate'),
        dict(id='physical_frame_and_mechanical_margin', status='unverified',
             reason='Saved zeros/signs/endpoints are preserved constraints, not independently measured CAD frames or collision margins'),
        dict(id='plant_and_timing_identification', status='unverified',
             reason='Loaded motor response, voltage sag, mass distribution and live closed-loop timing require evidence'),
        dict(id='attended_physical_acceptance', status='not_run', reason='Offline tools cannot certify physical walking'),
    ]
    return dict(schema_version=1, kind='model_adaptation_contract',
        hardware_access=False, motor_commands=0, calibration_writes=0,
        deployment_authorized=False, physical_walking_verified=False,
        model=dict(path=str(Path(model_path).resolve()), sha256=model_hash,
            input_shape=[1, 61], output_shape=[1, 14], dtype='float32',
            joint_order=list(POLICY_JOINT_NAMES), observation_order=list(POLICY_OBSERVATION_NAMES),
            home_rad=profile['home_rad'].tolist(), action_scale=profile['scale'],
            previous_action='raw_actor_output', output_formula='home_rad + action_scale * action'),
        source_files={k:dict(path=str(v.resolve()), sha256=sha256(v)) for k,v in sources.items()},
        differing_home_sources=dict(model_metadata_leg_pitch_deg=math.degrees(profile['home_rad'][4]),
            official_runtime_leg_pitch_deg=numeric_assignment(sources['runtime_math'], 'q'),
            current_training_leg_pitch_deg=numeric_assignment(sources['training_robot'], '_DEG24', radians_call=True),
            exact_export_training_config_verified=False,
            interpretation='Different source versions; not permission to overwrite the model metadata HOME'),
        calibration=dict(sha256=calibration_hash, joints=joints,
            endpoint_interpretation='Saved operating constraints, not verified physical hard stops'),
        physical_robot=dict(total_mass_kg=mass_kg, total_mass_source='user_report',
            mass_distribution='unknown', center_of_mass='unmeasured', inertia='unmeasured',
            uniform_cad_scaling='offline hypothesis only', loaded_motor_parameters='unidentified',
            voltage_sag='unverified'),
        runtime_differences=dict(
            legacy_candidate_transformations=candidate_math,
            source_expected_gains=gain_source,
            previous_uniform_kp6=dict(status='historical_simulation_hypothesis',
                matches_last_verified_firmware_source=False,
                correction='The earlier audit incorrectly described uniform KP6 as physical firmware configuration.'),
            action_ema=dict(leg=numeric_assignment(sources['runtime_math'], 'ACTION_ALPHA'),
                head=numeric_assignment(sources['runtime_math'], 'ACTION_ALPHA_HEAD'),
                source='official hardware runtime; current training uses direct actions'),
            head_execution='must be explicit: full policy or fixed head; fixed head changes plant seen by learned policy',
            goal_slew='must be explicit; no equivalence assumed between 600 ticks/s and training max_velocity=100 rad/s',
            observation_filter='truth and native host IMU filter are distinct experimental settings'),
        evidence=evidence, release_gates=gates,
        physical_release_ready=False,
        interpretation='This contract tracks evidence, never authorizes deployment or robot movement.')


def held_head_training_candidate(contract):
    """A concrete adapter specification, explicitly not an implemented/trained policy."""
    policy = [j for j in contract['calibration']['joints'] if j['policy_index'] is not None]
    return dict(schema_version=1, kind='untrained_held_head_adapter_specification',
        implemented=False, trained=False, hardware_eligible=False,
        source_model_sha256=contract['model']['sha256'],
        calibration_sha256=contract['calibration']['sha256'],
        source_expected_actuator_gains=contract['runtime_differences']['source_expected_gains'],
        legacy_runtime_transforms_to_reconcile=contract['runtime_differences']['legacy_candidate_transformations'],
        task_reference='Mjlab-Velocity-Flat-XgoDuck',
        preserve_observation_layout=contract['model']['observation_order'],
        hold_joint_ids=[30, 31, 32, 33], actuated_policy_joint_ids=[j['id'] for j in policy if j['id'] < 30],
        retained_constraint_angles=[dict(id=j['id'], min_deg=j['min_deg'], max_deg=j['max_deg']) for j in policy],
        required_implementation=[
            'Apply held-head execution inside every training and evaluation physics rollout, not only after inference on hardware.',
            'Use one declared in-range head setpoint; verify physical head/body/harness clearance before selecting it for deployment.',
            'Include preserved saved encoder constraints in target violation diagnostics, episode handling and export acceptance.',
            'Treat clipped or projected execution as an explicit plant change; never count clipping alone as proof the raw policy respects bounds.',
            'Model measured action cadence, execution latency, motor response and voltage range; record any unmeasured assumptions.',
            'Export exact HOME, action scale, joint/observation order and source/checkpoint/config hashes with the resulting model.',
        ],
        unresolved_inputs=['safe head setpoint', 'loaded motor response', 'mass distribution and center of mass', 'validated control cadence'],
        acceptance=['zero warmup and recovery complete', 'no requested/applied/actual saved-range violation',
            'supported landed lifts on both feet and forward progress', 'repeatability across declared plant uncertainties',
            'closed-loop timing evidence', 'attended physical acceptance after offline gates'],
        note='A training specification is not a completed training run and provides no guarantee that training will succeed.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--model', type=Path, default=ROOT/'output/x5_policy_bundle/models/xgoduck_walk.onnx')
    parser.add_argument('--reference-root', type=Path, default=ROOT/'output/xgoduck_rl_reference')
    parser.add_argument('--runtime-root', type=Path, default=ROOT/'output/xgoduck_runtime_upstream')
    parser.add_argument('--mass-kg', type=float, required=True)
    parser.add_argument('--reports', type=Path, nargs='*', default=[])
    parser.add_argument('--output', type=Path, required=True, help='New output directory')
    args = parser.parse_args(argv)
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.model), sess_options=options, providers=['CPUExecutionProvider'])
    contract = build_contract(json.loads(args.snapshot.read_text()), session,
        model_path=args.model, reference_root=args.reference_root, runtime_root=args.runtime_root,
        mass_kg=args.mass_kg, reports=args.reports)
    contract['snapshot_artifact'] = dict(path=str(args.snapshot.resolve()), sha256=sha256(args.snapshot))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'contract.json').write_text(json.dumps(contract, indent=2, ensure_ascii=False)+'\n')
    (args.output/'held-head-training-candidate.json').write_text(json.dumps(
        held_head_training_candidate(contract), indent=2, ensure_ascii=False)+'\n')
    (args.output/'servo-gain-manifest.json').write_text(json.dumps(
        contract['runtime_differences']['source_expected_gains'], indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(args.output.resolve()), physical_release_ready=False,
                         release_gates=contract['release_gates']), ensure_ascii=False))


if __name__ == '__main__':
    main()
