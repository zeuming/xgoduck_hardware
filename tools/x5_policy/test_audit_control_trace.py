import unittest
from audit_control_trace import analyze


class TraceTests(unittest.TestCase):
    def trace(self):
        frame=dict(event='target-attempt',request_error=None,mode='hold',
            request_ended_monotonic=10.01,monotonic=10.,age_s=.001,
            MCU_written_goal_us=0xfffffff0,sample_us=0x10,
            actual_raw_ticks=[2050]*15,MCU_written_goal_ticks=[2048]*15,command=[0,0,0])
        return dict(motor_ids=list(range(15)),calibration=dict(joints=[
            dict(id=i,zero=2048,sign=-1,min=1024,max=3072) for i in range(15)]),frames=[frame])

    def test_wrap_and_encoder_sign_are_preserved(self):
        report,rows=analyze(self.trace())
        self.assertEqual(len(rows),15)
        self.assertAlmostEqual(rows[0]['goal_cache_age_at_scan_end_ms'],.032)
        self.assertAlmostEqual(rows[0]['error_deg'],-720/4095)
        self.assertFalse(report['suitable_for_actuator_identification'])

    def test_future_goal_is_not_compared_to_older_feedback(self):
        trace=self.trace();trace['frames'][0].update(sample_us=100,MCU_written_goal_us=200)
        report,rows=analyze(trace)
        self.assertEqual(rows,[])
        self.assertEqual(report['skipped_frames']['no_prior_goal_within_100ms'],1)

    def test_previous_cached_goal_is_used_instead_of_future_goal(self):
        trace=self.trace()
        first=trace['frames'][0]
        first.update(sample_us=1000,MCU_written_goal_us=1100)
        second=dict(first,sample_us=31000,MCU_written_goal_us=31100,
                    MCU_written_goal_ticks=[2100]*15,monotonic=10.03,request_ended_monotonic=10.04)
        trace['frames'].append(second)
        report,rows=analyze(trace)
        self.assertEqual(len(rows),15)
        self.assertEqual(rows[0]['goal_write_us'],1100)
        self.assertEqual(rows[0]['cached_goal_deg'],0.)

    def test_rejected_targets_and_unrelated_calibration_are_not_counted(self):
        trace=self.trace();trace['frames'][0]['request_error']='stop'
        self.assertEqual(analyze(trace)[1],[])
        trace=self.trace();trace['calibration']['joints'].reverse()
        with self.assertRaises(ValueError):analyze(trace)


if __name__=='__main__':unittest.main()
