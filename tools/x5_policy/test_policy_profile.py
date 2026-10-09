"""Policy contracts and math, with optional local ONNX CPU replay; no device I/O."""
import importlib.util
import math
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from motion import MotionController, MotionError
from rl_core import (DEFAULT_DEG, FIXED_STANCE_OVERRIDES, MOTOR_IDS, State, action_to_deg,
                     build_obs, policy_profile, policy_profile_from_metadata,
                     stance_from_home)
from test_motion import FakePolicy, ready_config


class ProfileTests(unittest.TestCase):
    def test_right_hip_yaw_uses_leg_filter_when_head_and_leg_rates_differ(self):
        policy = FakePolicy()
        policy.output.fill(.1)
        controller = MotionController({'walk': policy}, config_path='/__unused_profile_test__.json')
        stance = stance_from_home(policy_profile(policy)['home_rad'])
        state = State([0] * 3, [0] * 3, stance, [0] * 15)
        previous = np.linspace(-.2, .2, 14, dtype=np.float32)
        with mock.patch('motion.ACTION_ALPHA', .2), mock.patch('motion.ACTION_ALPHA_HEAD', .8):
            _, raw, smooth, _ = controller._infer_policy_target('walk', state, previous,
                np.zeros(3), np.zeros(14, np.float32), 0.)
        # Distinct rates expose a boundary that equal production rates hide.
        expected = np.full(14, .08)
        expected[5:9] = .02
        np.testing.assert_allclose(smooth, expected, atol=1e-8)
        np.testing.assert_array_equal(policy.observation[0, 34:48], previous)
        np.testing.assert_array_equal(raw, policy.output[0])
        self.assertFalse(controller.status()['connected'])

    def test_valid_profiles_keep_different_model_homes_and_readonly_arrays(self):
        walk, roll = policy_profile(FakePolicy()), policy_profile(FakePolicy('roulade'))
        self.assertAlmostEqual(walk['home_rad'][2], -.349)
        self.assertAlmostEqual(roll['home_rad'][2], -.419)
        self.assertEqual(walk['scale'], 1.)
        with self.assertRaises(ValueError):
            walk['home_rad'][2] = 0

    def test_missing_metadata_fields_never_have_defaults(self):
        for key in ('joint_names','observation_names','default_joint_pos','action_scale'):
            with self.subTest(key=key):
                metadata = FakePolicy().metadata.copy(); del metadata[key]
                with self.assertRaises(ValueError): policy_profile_from_metadata(metadata)

    def test_wrong_duplicate_or_extra_joint_names_are_rejected(self):
        names = FakePolicy().metadata['joint_names'].split(',')
        wrong = names.copy(); wrong[0], wrong[1] = wrong[1], wrong[0]
        duplicate = names.copy(); duplicate[1] = duplicate[0]
        for bad in (wrong,duplicate,names+['mouth']):
            metadata = FakePolicy().metadata.copy(); metadata['joint_names'] = ','.join(bad)
            with self.assertRaises(ValueError): policy_profile_from_metadata(metadata)

    def test_observation_order_must_match_the_exact_61_dimension_layout(self):
        metadata = FakePolicy().metadata.copy()
        names = metadata['observation_names'].split(','); names[0],names[1] = names[1],names[0]
        metadata['observation_names'] = ','.join(names)
        with self.assertRaises(ValueError): policy_profile_from_metadata(metadata)

    def test_home_requires_exactly_fourteen_finite_radian_values(self):
        for bad in ('0,'*13+'NaN','0,'*13+'Infinity','0,'*12+'0','0,'*14+'0','false'):
            metadata = FakePolicy().metadata.copy(); metadata['default_joint_pos'] = bad
            with self.assertRaises(ValueError): policy_profile_from_metadata(metadata)

    def test_scale_is_read_and_requires_a_positive_finite_value(self):
        metadata = FakePolicy().metadata.copy(); metadata['action_scale'] = '.5'
        self.assertEqual(policy_profile_from_metadata(metadata)['scale'], .5)
        for bad in ('0','-1','NaN','Infinity','false',True):
            metadata['action_scale'] = bad
            with self.assertRaises(ValueError): policy_profile_from_metadata(metadata)

    def test_shape_dtype_or_unreadable_metadata_rejects_the_session(self):
        for field,value in (('shape',[1,60]),('type','tensor(double)')):
            policy=FakePolicy()
            attrs={'shape':[1,61],'type':'tensor(float)'};attrs[field]=value
            policy.get_inputs=lambda attrs=attrs:[type('Input',(),attrs)()]
            with self.assertRaises(ValueError): policy_profile(policy)
        policy=FakePolicy();policy.get_outputs=lambda:[]
        with self.assertRaises(ValueError): policy_profile(policy)
        policy=FakePolicy();policy.get_modelmeta=lambda: (_ for _ in ()).throw(RuntimeError('missing'))
        with self.assertRaises(ValueError): policy_profile(policy)

    def test_matching_home_yields_zero_joint_observation_and_zero_action_reconstructs_that_pose(self):
        for name in ('walk','roulade'):
            profile=policy_profile(FakePolicy(name));stance=stance_from_home(profile['home_rad'])
            st=State([0]*3,[0]*3,stance,[0]*15)
            obs=build_obs(st,np.zeros(14),np.zeros(3),np.zeros(4),home_rad=profile['home_rad'])
            np.testing.assert_allclose(obs[6:20],0,atol=5e-8)
            target=action_to_deg(np.zeros(14),home_rad=profile['home_rad'],action_scale=profile['scale'])
            np.testing.assert_allclose(target,stance,atol=2e-6)
            self.assertEqual(target[MOTOR_IDS.index(34)],0.)

    def test_action_scale_affects_outputs_but_not_joint_observation_units(self):
        profile=policy_profile(FakePolicy());stance=stance_from_home(profile['home_rad'])
        st=State([0]*3,[0]*3,stance,[0]*15);st.angles[0]+=math.degrees(.1)
        obs=build_obs(st,np.zeros(14),np.zeros(3),np.zeros(4),home_rad=profile['home_rad'])
        self.assertAlmostEqual(float(obs[6]),.1,places=6)
        action=np.zeros(14);action[0]=.2
        target=action_to_deg(action,home_rad=profile['home_rad'],action_scale=.5)
        self.assertAlmostEqual(target[0],math.degrees(.1),places=5)

    def test_static_default_matches_verified_walk_metadata_in_motor_order(self):
        stance=stance_from_home(policy_profile(FakePolicy())['home_rad'])
        np.testing.assert_allclose(DEFAULT_DEG,stance,atol=1e-10)

    def test_invalid_controller_metadata_fails_before_any_transport_construction(self):
        calls=[];policy=FakePolicy();del policy.metadata['action_scale']
        with self.assertRaises(MotionError):
            MotionController({'walk':policy},config_path='/__unused_profile_test__.json',transport_factory=lambda port:calls.append(port))
        self.assertEqual(calls,[])

    def test_controller_inference_uses_selected_home_and_scale_and_exposes_walk_stance(self):
        walk,roll=FakePolicy(),FakePolicy('roulade');roll.metadata['action_scale']='.5';roll.output.fill(.2)
        controller=MotionController({'walk':walk,'roulade':roll},config_path='/__unused_profile_test__.json')
        stance=stance_from_home(policy_profile(walk)['home_rad']);st=State([0]*3,[0]*3,stance,[0]*15)
        target,action,smooth,_=controller._infer_policy_target('roulade',st,np.zeros(14),np.zeros(3),np.zeros(14),0.)
        self.assertAlmostEqual(float(roll.observation[0,8]),.07,places=6)
        expected=action_to_deg(np.full(14,.17),home_rad=policy_profile(roll)['home_rad'],action_scale=.5)
        np.testing.assert_allclose(target,expected,atol=2e-6)
        status=controller.status();self.assertEqual(status['policy_metadata_protocol'],1)
        fixed_stance=[FIXED_STANCE_OVERRIDES.get(sid,angle) for sid,angle in zip(MOTOR_IDS,stance)]
        np.testing.assert_allclose(status['stand_target_deg'],fixed_stance)
        np.testing.assert_allclose(controller._stand_deg,stance)
        self.assertEqual(status['policy_profiles']['roulade']['scale'],.5)
        self.assertFalse(status['connected'])


MODEL_DIR=Path(__file__).resolve().parents[2]/'output/x5_policy_bundle/models'

@unittest.skipUnless(importlib.util.find_spec('onnxruntime') is not None and MODEL_DIR.is_dir(),
                     'Local ONNX CPU runtime/models unavailable')
class RealModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import onnxruntime as ort
        opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
        cls.sessions={name:ort.InferenceSession(str(MODEL_DIR/f'xgoduck_{name}.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
                      for name in ('walk','getup','pick','roulade')}

    def test_actual_four_models_have_valid_contracts_and_distinct_roll_home(self):
        profiles={name:policy_profile(session) for name,session in self.sessions.items()}
        for name in ('getup','pick'):
            np.testing.assert_array_equal(profiles[name]['home_rad'],profiles['walk']['home_rad'])
        self.assertAlmostEqual(profiles['walk']['home_rad'][2],-.349)
        self.assertAlmostEqual(profiles['roulade']['home_rad'][2],-.419)
        self.assertTrue(all(p['scale']==1. for p in profiles.values()))

    def test_real_zero_command_walk_first_frame_respects_the_recorded_run15_ranges(self):
        # Recorded calibration from run 15. This checks only inference and
        # preflight limits; it does not claim unsupported free-standing balance.
        rows=[(10,2071,-1,1502,2694),(11,1945,-1,1607,2195),(12,2909,-1,1867,3850),
              (13,3313,-1,2223,3629),(14,2371,1,1983,3212),(20,2066,-1,1394,2749),
              (21,2209,1,1971,2561),(22,2131,-1,1469,3137),(23,1871,1,1556,2935),
              (24,1975,-1,1266,2754),(30,1995,-1,1626,2990),(31,2101,1,1054,2879),
              (32,3201,-1,2173,4076),(33,910,-1,724,969),(34,1296,-1,902,1296)]
        controller=MotionController(self.sessions,config_path='/__unused_profile_test__.json')
        controller._config=ready_config()
        for joint,(sid,zero,sign,lo,hi) in zip(controller._config['joints'],rows):
            self.assertEqual(joint['id'],sid);joint.update(zero=zero,sign=sign,min=lo,max=hi)
        st=State([0]*3,[0]*3,controller.status()['stand_target_deg'],[0]*15)
        target,action,smooth,_=controller._infer_policy_target('walk',st,np.zeros(14),np.zeros(3),np.zeros(14),0.)
        self.assertEqual(len(controller._raw_targets_locked(target,mode='walk')),15)
        self.assertTrue(np.isfinite(action).all());self.assertFalse(controller.status()['connected'])
        roll_target,*_=controller._infer_policy_target('roulade',st,np.zeros(14),np.zeros(3),np.zeros(14),0.)
        with self.assertRaisesRegex(MotionError,'目标超过已记录的机械范围'):
            controller._raw_targets_locked(roll_target,mode='roulade')


if __name__=='__main__':
    unittest.main()
