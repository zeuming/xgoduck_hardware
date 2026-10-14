import copy
import unittest
from capture_joint_envelope import sample, display_sample


class ReadOnlyCaptureTests(unittest.TestCase):
    def state(self):
        return dict(mode='off', connected=True, armed=False, torque_ids=[],
                    release_confirmed=True, fault_ids=[], online_ids=[14], age_ms=20.,
                    sample_us=1234, rawdata=[dict(id=14, position=2200)])

    def test_valid_off_feedback_is_read_without_mutation(self):
        h = self.state();original = copy.deepcopy(h)
        self.assertEqual(sample(h,14)['position'],2200)
        self.assertEqual(h,original)

    def test_stale_feedback_is_not_recorded(self):
        for age in (500, 501, float('nan'), None):
            h=self.state();h['age_ms']=age
            self.assertIsNone(sample(h,14))

    def test_manual_display_cache_is_distinct_from_actuation_freshness(self):
        h=self.state();h['age_ms']=250
        self.assertEqual(sample(h,14)['position'],2200)

    def test_missing_display_packet_is_skipped_without_historical_fallback(self):
        self.assertIsNone(display_sample(dict(owns_connection=True, hardware=None,
                                             last_hardware=self.state()),14))
        for state in (dict(owns_connection=False),dict(owns_connection=True,error='offline')):
            with self.assertRaises(ValueError):display_sample(state,14)

    def test_motion_torque_fault_or_offline_never_counts_as_off_proof(self):
        for key,value in [('mode','hold'),('armed',True),('torque_ids',[14]),
                          ('release_confirmed',False),('fault_ids',[14]),('online_ids',[])]:
            h=self.state();h[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):sample(h,14)
        with self.assertRaises(ValueError):sample(None,14)


if __name__ == '__main__':
    unittest.main()
