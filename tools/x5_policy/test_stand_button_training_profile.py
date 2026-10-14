import copy
import unittest
from pathlib import Path
import torch
from stand_button_training_profile import apply_task_profile,command_at_steps,source_pulse,PULSE_SHA,PROFILE
from validate_training_regression import judge_trace,compare_zero_evidence

class ProfileTests(unittest.TestCase):
    def test_scalar_frozen_equivalence_all_frames_and_edges(self):
        pulse=source_pulse();steps=torch.arange(1100)
        expected=torch.tensor([pulse.command_at(i*.02,0)[0] if i>=250 else 0. for i in range(1100)])
        torch.testing.assert_close(command_at_steps(steps,.02),expected,atol=1e-7,rtol=0)
    def test_upstream_default_is_noop(self):
        cfg=object()
        self.assertEqual(apply_task_profile(cfg),dict(name='upstream-velocity',modified=False))
    def test_unknown_profile_rejected(self):
        with self.assertRaises(ValueError):apply_task_profile(object(),'typo')

class GateTests(unittest.TestCase):
    def fixture(self):
        rows=[dict(index=i,sim_time_s=(i+1)*.02,speed_m_s=0.,actor_observation=[0.]*61,
            tilt_deg=1.,root_position=[0.,0.,0.],termination_terms=[],violations=[],contact_sensor_consistency_passed=True)
            for i in range(250)]
        report=dict(stage='smoke_completed',candidate_sha256='candidate',evaluated_policy=dict(sha256='policy'),
            policy_diagnostic=dict(automatic_reset=False,seed=0),button_command_source=dict(PROFILE,sha256=PULSE_SHA),
            steps_completed=250,phase_metrics={},reset_actual_plant=dict(root_position=[0.,0.,0.]))
        return report,rows
    def test_zero_pass_never_marks_walking_or_hardware(self):
        report,rows=self.fixture();result=judge_trace(report,rows,'candidate','policy')
        self.assertTrue(result['zero_minimum_pass']);self.assertFalse(result['forward_gait_pass'])
        self.assertFalse(result['hardware_eligible'])
    def test_degradation_and_experimental_boundary_reject(self):
        report,rows=self.fixture();rows[42]['tilt_deg']=15.
        self.assertFalse(judge_trace(report,rows,'candidate','policy')['zero_minimum_pass'])
        report['steps_completed']=57
        self.assertIn('five_second_zero_incomplete',judge_trace(report,rows[:57],'candidate','policy')['rejection_reasons'])
    def test_trace_command_tamper_rejected(self):
        report,rows=self.fixture();rows[0]['actor_observation'][48]=.1
        with self.assertRaises(ValueError):judge_trace(report,rows,'candidate','policy')
    def test_missing_observation_cannot_pass_as_zero(self):
        report,rows=self.fixture();rows[0]['actor_observation']=[]
        with self.assertRaises(ValueError):judge_trace(report,rows,'candidate','policy')
    def test_nonfinite_observation_and_time_rejected(self):
        for location in ('actor','time'):
            report,rows=self.fixture()
            if location=='actor':rows[0]['actor_observation'][48]=float('nan')
            else:rows[0]['sim_time_s']=float('nan')
            with self.assertRaises(ValueError):judge_trace(report,rows,'candidate','policy')
    def test_automatic_reset_and_identity_mismatch_rejected(self):
        report,rows=self.fixture()
        with self.assertRaises(ValueError):judge_trace(report,rows,'wrong','policy')
        report['policy_diagnostic']['automatic_reset']=True
        with self.assertRaises(ValueError):judge_trace(report,rows,'candidate','policy')
    def test_minimum_pass_does_not_hide_margin_regression(self):
        baseline=dict(zero_minimum_pass=True,zero_max_tilt_deg=1.,zero_horizontal_drift_m=.001)
        candidate=dict(zero_minimum_pass=True,zero_max_tilt_deg=11.,zero_horizontal_drift_m=.022)
        result=compare_zero_evidence(baseline,candidate)
        self.assertTrue(result['zero_minimum_pass']);self.assertFalse(result['new_zero_regression_pass'])
        self.assertEqual(result['status'],'rejected_margin_regression')
        self.assertTrue(compare_zero_evidence(baseline,baseline)['new_zero_regression_pass'])

if __name__=='__main__':unittest.main()
