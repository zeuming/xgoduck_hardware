"""CPU contract/action/CAD tests. These do not pretend to load mjlab/warp."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import sys
import unittest
import numpy as np

from constrained_training_candidate import (ROOT, HEAD_INDICES, CpuActionPlant,
    build_candidate, validate_candidate, save_candidate, load_candidate,
    cpu_structure_check, check_model_compatibility, check_runtime_head_setpoint,
    training_export_metadata, digest, configure_single_battery_gain_vector)

CONTRACT = ROOT/'output/duck_sim/repair-20261011/model/contract-reviewed/contract.json'
SNAPSHOT = ROOT/'output/duck_sim/baseline-repair/trace-snapshot.json'


class ConstrainedCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = build_candidate(CONTRACT,SNAPSHOT)

    def test_contract_roundtrip_pins_all_saved_constraints(self):
        original = SNAPSHOT.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'candidate.json'
            save_candidate(self.c,path)
            self.assertEqual(load_candidate(path),self.c)
            payload = json.loads(path.read_text())
            payload['candidate']['upper_rad'][4] += .1
            payload['candidate_sha256'] = digest(payload['candidate'])
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError,'saved range'):
                load_candidate(path)
        self.assertEqual(SNAPSHOT.read_bytes(),original)
        self.assertEqual(len(self.c['calibration']['joints']),15)

    def test_corrupt_nonfinite_timing_or_tick_resolution_is_rejected(self):
        for key,value in [('alpha',np.nan),('control_dt_s',np.inf),('goal_slew_ticks_s',np.inf),
                          ('goal_slew_ticks_s',np.nan),('control_dt_s',0),('ticks_per_turn',-4095),
                          ('ticks_per_turn',0),('ticks_per_turn',4096)]:
            with self.subTest(key=key,value=value):
                c = copy.deepcopy(self.c); c['action'][key] = value
                with self.assertRaisesRegex(ValueError,'Invalid EMA, period or slew'):
                    validate_candidate(c)

    def test_per_joint_kp_preserves_single_shared_battery_and_rejects_split(self):
        voltage = np.array([[7.5],[7.8]])
        drop = np.array([[.1],[.2]])
        load = np.arange(28,dtype=float).reshape(2,14)/100
        actuator = SimpleNamespace(target_names=self.c['joint_names'],_base_kp=5.,
            vin_tensor=voltage,vin_drop_gain=drop,_prev_motor_torque=load)
        expected_voltage = voltage-drop*np.abs(load).sum(axis=-1,keepdims=True)
        configure_single_battery_gain_vector([actuator],self.c)
        self.assertIs(actuator.vin_tensor,voltage)
        self.assertIs(actuator.vin_drop_gain,drop)
        self.assertIs(actuator._prev_motor_torque,load)
        # Same source battery formula still sums all 14 axes, including the head.
        actual_voltage = actuator.vin_tensor-actuator.vin_drop_gain*np.abs(actuator._prev_motor_torque).sum(axis=-1,keepdims=True)
        np.testing.assert_array_equal(actual_voltage,expected_voltage)
        np.testing.assert_array_equal(actuator._base_kp,[[8,8,8,8,8,8,8,8,12,8,8,8,10,8]])
        self.assertEqual((actuator._base_kp*np.ones((2,1))).shape,(2,14))
        # The earlier KP-group split must fail, even if groups share nominal volts.
        with self.assertRaisesRegex(ValueError,'one shared-battery'):
            configure_single_battery_gain_vector([actuator,actuator,actuator],self.c)
        actuator.target_names = self.c['joint_names'][:5]
        with self.assertRaisesRegex(ValueError,'all 14'):
            configure_single_battery_gain_vector([actuator],self.c)

    def test_root_placement_uses_sole_geometry_not_double_absolute_z(self):
        placement = self.c['reset_placement']
        self.assertEqual(placement['relative_reset_z_range_m'],[0.,0.])
        for value in placement['root_zero_sole_min_z_m'].values():
            self.assertGreaterEqual(value+placement['root_z_m'],.001-1e-10)
        self.assertAlmostEqual(min(placement['root_zero_sole_min_z_m'].values())+placement['root_z_m'],.001)
        self.assertGreater(placement['root_z_m'],.1)
        self.assertLess(placement['root_z_m'],.2)
        changed = copy.deepcopy(self.c); changed['reset_placement']['root_z_m'] *= 2
        with self.assertRaisesRegex(ValueError,'Grounded reset placement'):
            validate_candidate(changed)

    def test_source_hash_drift_is_rejected(self):
        for key in ('source_snapshot','adapter_source'):
            changed = copy.deepcopy(self.c)
            changed[key]['sha256'] = 'bad'
            with self.assertRaisesRegex(ValueError,'Pinned source changed'):
                validate_candidate(changed)

    def test_executing_copy_must_match_pinned_adapter_bytes(self):
        source = Path(self.c['adapter_source']['path'])
        with tempfile.TemporaryDirectory() as directory:
            copied = Path(directory)/'adapter_copy.py'
            name = 'constrained_training_candidate_copy_test'
            try:
                for changed in (False, True):
                    copied.write_bytes(source.read_bytes()+(b'\n# altered copied implementation\n' if changed else b''))
                    spec = importlib.util.spec_from_file_location(name,copied)
                    module = importlib.util.module_from_spec(spec)
                    sys.modules[name] = module
                    spec.loader.exec_module(module)
                    if changed:
                        with self.assertRaisesRegex(ValueError,'Executing adapter source differs'):
                            module.validate_candidate(self.c)
                    else:
                        self.assertEqual(module.validate_candidate(self.c),self.c)
            finally:
                sys.modules.pop(name,None)

    def test_full_policy_history_is_retained_head_applied_setpoint_is_fixed(self):
        plant = CpuActionPlant(self.c)
        raw = np.zeros((1,14)); raw[:,list(HEAD_INDICES)] = 100
        before = raw.copy()
        result = plant.process(raw)
        np.testing.assert_array_equal(raw,before)
        np.testing.assert_array_equal(result['raw'],before)
        np.testing.assert_allclose(result['applied'][0,list(HEAD_INDICES)],self.c['held_head_rad'])
        self.assertFalse(result['episode_failed'].any())
        np.testing.assert_allclose(self.c['head_command_rad'],np.array(self.c['held_head_rad'])-np.array(self.c['home_rad'])[list(HEAD_INDICES)])

    def test_outside_saved_request_fails_before_ema_and_holds_not_clips(self):
        plant = CpuActionPlant(self.c)
        previous = plant.goal.copy()
        raw = np.zeros((1,14))
        raw[0,4] = (self.c['upper_rad'][4]+.001-self.c['home_rad'][4])/self.c['action_scale']
        result = plant.process(raw)
        self.assertTrue(result['episode_failed'][0])
        self.assertGreater(result['requested'][0,4],self.c['upper_rad'][4])
        np.testing.assert_array_equal(result['applied'],previous)
        self.assertNotEqual(result['applied'][0,4],self.c['upper_rad'][4])
        # A subsequent legal command cannot clear the failed episode.
        self.assertTrue(plant.process(np.zeros((1,14)))['episode_failed'][0])

    def test_nonfinite_actor_output_fails_even_in_masked_head(self):
        for value in (np.nan,np.inf,-np.inf):
            plant = CpuActionPlant(self.c)
            raw = np.zeros((1,14)); raw[0,5] = value
            result = plant.process(raw)
            self.assertTrue(result['episode_failed'][0])
            self.assertTrue(np.isfinite(result['applied']).all())

    def test_bound_rejection_is_per_environment(self):
        plant = CpuActionPlant(self.c,2)
        raw = np.zeros((2,14)); raw[0,4] = 100
        result = plant.process(raw)
        np.testing.assert_array_equal(result['episode_failed'],[True,False])

    def test_in_range_slew_is_explicit_and_bounded(self):
        plant = CpuActionPlant(self.c)
        before = plant.goal.copy()
        raw = np.zeros((1,14)); raw[0,4] = .1
        result = plant.process(raw)
        limit = self.c['action']['goal_slew_ticks_s']*2*np.pi/4095*self.c['action']['control_dt_s']
        self.assertFalse(result['episode_failed'][0])
        self.assertAlmostEqual(result['applied'][0,4]-before[0,4],limit)
        self.assertLess(result['applied'][0,4],result['requested'][0,4])

    def test_zero_slew_setting_has_explicit_ema_only_behavior(self):
        c = copy.deepcopy(self.c); c['action']['goal_slew_ticks_s'] = 0
        plant = CpuActionPlant(c)
        raw = np.zeros((1,14)); raw[0,4] = .01
        result = plant.process(raw)
        self.assertAlmostEqual(result['applied'][0,4],c['home_rad'][4]+(1-c['action']['alpha'])*c['action_scale']*.01)

    def test_real_cad_mapping_has_different_joint_and_actuator_indices(self):
        report = cpu_structure_check(self.c)
        self.assertTrue(report['cad_compiled'])
        self.assertTrue(report['original_hard_limits_unchanged'])
        self.assertEqual(len(report['joints']),14)
        for row in report['joints']:
            self.assertNotEqual(row['cad_joint_index'],row['cad_actuator_index'])
        self.assertFalse(report['full_mjlab_environment_loaded'])

    def test_old_model_loads_but_cannot_claim_retraining(self):
        model = self.c['source_model']['path']
        result = check_model_compatibility(self.c,model,require_training_provenance=False)
        self.assertTrue(result['io_metadata_compatible'])
        self.assertFalse(result['matching_training_candidate_metadata'])
        with self.assertRaisesRegex(ValueError,'No matching constrained-training provenance'):
            check_model_compatibility(self.c,model)

    def test_arbitrary_runtime_head_capture_is_rejected(self):
        self.assertTrue(check_runtime_head_setpoint(self.c,self.c['held_head_rad']))
        held = list(self.c['held_head_rad']); held[0] += .01
        with self.assertRaisesRegex(ValueError,'Runtime head setpoint'):
            check_runtime_head_setpoint(self.c,held)
        metadata = training_export_metadata(self.c)
        self.assertEqual(json.loads(metadata['constrained_training_held_head_rad']),self.c['held_head_rad'])
        self.assertEqual(metadata['constrained_training_hardware_eligible'],'false')
        self.assertFalse(self.c['production_loader_integrated'])
        self.assertFalse(self.c['hardware_eligible'])

    def test_held_pose_cannot_be_rebound_under_pinned_snapshot(self):
        c = copy.deepcopy(self.c); c['held_head_rad'][0] += .01; c['head_command_rad'][0] += .01
        with self.assertRaises(ValueError):
            validate_candidate(c)


if __name__ == '__main__':
    unittest.main()
