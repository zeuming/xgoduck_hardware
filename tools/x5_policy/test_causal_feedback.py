"""No-I/O protocol/causality regression tests."""
import copy
import unittest
from causal_feedback import PAIRED_EXTENSION, decode_paired_extension, following_status


class CausalFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.state=dict(sample_us=100000, armed=True, cookie=123,
            online_mask=0x7fff, fault_mask=0, torque_mask=0x7fff, positions=[2000]*15)
        self.config=[dict(zero=2000,sign=1)]*15

    def evidence(self, **overrides):
        fields=dict(valid=1, seq=7, start=98000, cookie=123, goal_us=97000, goal_seq=8, goals=[2000]*15)
        fields.update(overrides)
        return PAIRED_EXTENSION.pack(2, fields['valid'],fields['seq'],fields['start'],
            fields['cookie'],fields['goal_us'],fields['goal_seq'],*fields['goals'])

    def status(self):
        return following_status(self.state,self.config,mode='walk',age_s=.02,
                                fresh_seconds=.1,expected_cookie=123)

    def test_sample_bound_target_is_used_instead_of_a_newer_live_target(self):
        self.state['motion_evidence']=decode_paired_extension(self.evidence(),self.state)
        self.state['unrelated_later_host_target']=[2060]*15
        self.assertTrue(self.status()['ready'])
        self.assertFalse(self.status()['servo_goal_register_readback'])

    def test_invalid_or_legacy_pair_never_falls_back_to_host_goal(self):
        for evidence in (None,dict(protocol=1,written_goals_raw=[2000]*15),
                         dict(protocol=2,pairing_valid=False)):
            self.state['motion_evidence']=evidence
            self.assertFalse(self.status()['ready'])
            self.assertEqual(self.status()['status'],'unknown')
            self.assertIsNone(self.status()['worst'])

    def test_future_goal_wrong_session_and_partial_scan_cannot_claim_valid(self):
        for args in (dict(goal_us=99000),dict(cookie=124),dict(start=100001)):
            with self.subTest(args=args),self.assertRaises(ValueError):
                decode_paired_extension(self.evidence(**args),self.state)
        for field,value in [('online_mask',0x7ffe),('fault_mask',1),('armed',False)]:
            state=dict(self.state,**{field:value})
            with self.subTest(field=field),self.assertRaises(ValueError):
                decode_paired_extension(self.evidence(),state)

    def test_invalid_pair_is_readable_but_never_authorizes(self):
        self.state['motion_evidence']=decode_paired_extension(
            self.evidence(valid=0,cookie=0,goal_us=100010),self.state)
        self.assertFalse(self.status()['ready'])

    def test_wrapping_clock_and_sequence_zero_are_valid(self):
        self.state['sample_us']=20
        evidence=decode_paired_extension(self.evidence(start=10,goal_us=0xfffffff0,
            seq=0,goal_seq=0),self.state)
        self.assertTrue(evidence['pairing_valid'])
        self.assertEqual(evidence['goal_write_sequence'],0)

    def test_threshold_is_unchanged_and_reports_measured_error(self):
        for ticks,ready in ((56,True),(57,False)):
            goals=[2000+ticks]+[2000]*14
            self.state['motion_evidence']=decode_paired_extension(self.evidence(goals=goals),self.state)
            status=self.status()
            self.assertEqual(status['ready'],ready)
            self.assertEqual(status['tolerance_deg'],5.)
            self.assertAlmostEqual(status['worst']['error_deg'],ticks*360/4095)

    def test_stale_changed_session_or_missing_torque_are_unknown(self):
        self.state['motion_evidence']=decode_paired_extension(self.evidence(),self.state)
        for age,cookie,torque in ((.1,123,0x7fff),(.02,124,0x7fff),(.02,123,0x7ffe)):
            state=dict(self.state,torque_mask=torque)
            status=following_status(state,self.config,mode='walk',age_s=age,
                fresh_seconds=.1,expected_cookie=cookie)
            self.assertFalse(status['ready'])
            self.assertEqual(status['status'],'unknown')

    def test_bad_version_length_position_and_validity_flag_are_rejected(self):
        good=self.evidence()
        for bad in (good[:-1],good+b'\0',bytes([1])+good[1:],good[:1]+bytes([2])+good[2:],
                    self.evidence(goals=[4096]+[2000]*14)):
            with self.subTest(payload=bad[:4]),self.assertRaises(ValueError):
                decode_paired_extension(bad,self.state)


if __name__=='__main__':unittest.main()
