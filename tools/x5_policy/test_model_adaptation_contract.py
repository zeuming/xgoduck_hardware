"""Regression tests for evidence admission and non-authorizing adaptation output."""
import copy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from audit_adaptation_matrix import source_comparisons
from model_adaptation_contract import (ROOT, build_contract, held_head_training_candidate,
                                      numeric_assignment, saved_joint_contract, screen_evidence,
                                      source_gain_manifest, load_gain_manifest, legacy_candidate_contract)
from rl_core import MOTOR_IDS, POLICY_JOINT_NAMES, POLICY_OBSERVATION_NAMES


def calibration():
    return dict(version=1, joints=[dict(id=i, zero=2048, min=1000, max=3000, sign=-1)
                                  for i in MOTOR_IDS])


def result():
    return dict(ended=dict(reason='duration'), max_tilt_deg=4., speed_m_s=.2,
        command_progress=dict(origin_at_s=5., direction_progress_m=.3), displacement_m=[.3, 0.],
        outputs={stage:dict(violating_samples=0) for stage in (
            'requested_execution_target', 'mcu_applied_goal', 'actual_joint')},
        actual_foot_motion=dict(feet={side:dict(contact_fraction=.5,
            qualifying_lift_intervals=[dict(completed=True, duration_s=.1, opposite_contact_s=.1)
                                      for _ in range(2)]) for side in ('left','right')}))


class EvidenceTests(unittest.TestCase):
    def test_endpoint_sign_is_preserved_not_called_a_mechanical_limit(self):
        cfg = calibration()
        before = copy.deepcopy(cfg)
        row = saved_joint_contract(cfg)[4]
        self.assertAlmostEqual(row['min_deg'], (3000-2048)*360/4095*-1)
        self.assertAlmostEqual(row['max_deg'], (1000-2048)*360/4095*-1)
        self.assertFalse(row['physical_hard_stop_verified'])
        self.assertFalse(row['collision_margin_verified'])
        self.assertEqual(cfg, before)
        cfg['joints'][0]['max'] = 4096
        with self.assertRaises(ValueError):
            saved_joint_contract(cfg)

    def test_evidence_rejects_other_model_calibration_counterfactual_and_cached_pass(self):
        doc = dict(source_sha256=dict(model='model'), calibration_sha256='calibration',
                   settings=dict(continue_after_range_violation=False), results=[result()])
        with TemporaryDirectory() as directory:
            path = Path(directory)/'report.json'
            def check(value):
                path.write_text(json.dumps(value))
                return screen_evidence([path], model_hash='model', calibration_hash='calibration')[0]['results'][0]['offline_screen_passed']
            self.assertTrue(check(doc))
            bad = copy.deepcopy(doc); bad['source_sha256']['model'] = 'other'
            self.assertFalse(check(bad))
            bad = copy.deepcopy(doc); bad['calibration_sha256'] = 'other'
            self.assertFalse(check(bad))
            bad = copy.deepcopy(doc); bad['settings']['continue_after_range_violation'] = True
            self.assertFalse(check(bad))
            bad = copy.deepcopy(doc); bad['settings'] = {}
            self.assertFalse(check(bad))
            bad = copy.deepcopy(doc)
            bad['results'][0]['outputs']['actual_joint']['violating_samples'] = 1
            bad['results'][0]['assessment'] = dict(offline_screen_passed=True)
            self.assertFalse(check(bad))

    def test_source_literals_are_read_without_executing_python(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'source.py'
            path.write_text('raise RuntimeError("must not run")\nq = 23.0\n_DEG24=math.radians(24)\n')
            self.assertEqual(numeric_assignment(path, 'q'), 23.)
            self.assertEqual(numeric_assignment(path, '_DEG24', radians_call=True), 24.)
            path.write_text('q = float("23")\n')
            with self.assertRaises(ValueError):
                numeric_assignment(path, 'q')

    def test_gain_manifest_values_come_from_verified_source_and_reject_tampering(self):
        manifest = source_gain_manifest()
        self.assertEqual(manifest['kp_by_id']['10'], 8)
        self.assertEqual(manifest['kp_by_id']['23'], 10)
        self.assertEqual(manifest['kp_by_id']['33'], 12)
        self.assertEqual(manifest['kp_by_id']['34'], 10)
        self.assertFalse(manifest['current_servo_registers_measured'])
        self.assertEqual(manifest['status'], 'last_verified_installed_source_expected')
        with TemporaryDirectory() as directory:
            path = Path(directory)/'gains.json'; path.write_text(json.dumps(manifest))
            self.assertEqual(load_gain_manifest(path), manifest)
            manifest['kp_by_id']['14'] = 6
            path.write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                load_gain_manifest(path)

    def test_source_gain_vector_reaches_bam_in_policy_order(self):
        import numpy as np
        from audit_policy_sim import parser, load_dependencies, make_sim
        with TemporaryDirectory() as directory:
            path = Path(directory)/'gains.json'; path.write_text(json.dumps(source_gain_manifest()))
            args = parser().parse_args(['--snapshot','unused-readonly-fixture','--kp-by-id',str(path)])
            args.bam_params = args.xml.parent/'params/1910_m6.json'
            mj, _, load, controller = load_dependencies(args.bam_root)
            model, data, bam, _, qi, *_ = make_sim(args, mj, load, controller)
            self.assertEqual(len(qi), 14)
            bam.actuator.q_target_smooth = np.zeros(14)
            volts = bam.actuator.compute_control(np.ones(14)*.001, np.zeros(14), np.zeros(14), .005)
            self.assertAlmostEqual(volts[8]/volts[0], 12/8)  # ID33 head-roll
            self.assertAlmostEqual(volts[12]/volts[0], 10/8)  # ID23 right-knee
            self.assertAlmostEqual(volts[9]/volts[0], 1.)  # ID20 is right yaw, not head

    def test_each_source_comparison_changes_only_its_declared_factor(self):
        rows = source_comparisons()
        self.assertEqual(len(rows), 6)
        by_name = {r['name']:r for r in rows}
        expected = dict(physics_contract={'physics_contract'}, goal_slew={'slew_ticks_s'},
            action_filter={'action_alpha','head_action_alpha'}, head_execution={'executor'},
            head_setpoint={'head_source'})
        for row in rows:
            if row['comparison_to'] is None:
                continue
            previous = by_name[row['comparison_to']]['settings']
            changed = {k for k,v in row['settings'].items() if v != previous[k]}
            self.assertEqual(changed, expected[row['changed_factor']])
            self.assertTrue(row['source_basis'])

    def test_deployed_candidate_math_is_not_mislabeled_as_metadata_only(self):
        candidate = legacy_candidate_contract()
        self.assertEqual(candidate['home_offsets_deg'], {12:-1.,14:-1.,22:1.,24:1.})
        self.assertEqual(candidate['roll_transform']['ids'], [11,21])
        self.assertEqual(candidate['roll_transform']['scale'], .95)
        self.assertFalse(candidate['metadata_decoder_is_actual_deployed_pipeline'])
        self.assertFalse(candidate['physical_effect_validated'])
        self.assertEqual(candidate['fixed_head_ids'], [30,31,32,33])

    @unittest.skipUnless((ROOT/'output/xgoduck_rl_reference').exists(), 'Audited reference sources unavailable')
    def test_contract_never_claims_physical_or_training_completion(self):
        meta = dict(joint_names=','.join(POLICY_JOINT_NAMES), observation_names=','.join(POLICY_OBSERVATION_NAMES),
                    default_joint_pos=','.join(['0']*14), action_scale='1')
        session = SimpleNamespace(
            get_inputs=lambda:[SimpleNamespace(shape=[1,61],type='tensor(float)')],
            get_outputs=lambda:[SimpleNamespace(shape=[1,14],type='tensor(float)')],
            get_modelmeta=lambda:SimpleNamespace(custom_metadata_map=meta))
        with TemporaryDirectory() as directory:
            model = Path(directory)/'model.onnx'; model.write_bytes(b'fixture model')
            contract = build_contract(dict(calibration=calibration()), session, model_path=model,
                reference_root=ROOT/'output/xgoduck_rl_reference',
                runtime_root=ROOT/'output/xgoduck_runtime_upstream', mass_kg=.871)
        self.assertFalse(contract['physical_release_ready'])
        self.assertFalse(contract['deployment_authorized'])
        self.assertEqual(contract['physical_robot']['mass_distribution'], 'unknown')
        candidate = held_head_training_candidate(contract)
        self.assertFalse(candidate['implemented'])
        self.assertFalse(candidate['trained'])
        self.assertFalse(candidate['hardware_eligible'])
        self.assertEqual(candidate['actuated_policy_joint_ids'], [10,11,12,13,14,20,21,22,23,24])


if __name__ == '__main__':
    unittest.main()
