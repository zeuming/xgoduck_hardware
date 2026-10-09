import copy
import unittest

from audit_trial_trace import summarize, WINDOW_END_REASON


def frame(at, mode, generation, event='target-attempt', error=None):
    return dict(monotonic=at, mode=mode, generation=generation, event=event,
        command=[0.,0.,0.], gravity=[0.,0.,-1.], age_s=.02,
        attempted_target_ticks=[2000]*15 if event=='target-attempt' else None,
        request_ended_monotonic=at+.005, request_error=error, stop_reason=None)


def trace():
    frames=[frame(0,'hold',1,'start'),frame(50,'hold',1),
            frame(51,'walk',2,'start'),frame(51.02,'walk',2),frame(60,'walk',2)]
    return dict(sequence=1,frames=frames,total_frames=5,retained_frames=5,
        truncated=False,terminal=frame(60.02,'walk',2,'stop'))


def continuous_trace():
    value=trace()
    value['frames']=value['frames'][:3]+[frame(51.02+i*.05,'walk',2) for i in range(600)]
    value.update(total_frames=len(value['frames']),retained_frames=len(value['frames']))
    value['terminal']['monotonic']=81.
    return value


class Tests(unittest.TestCase):
    def test_hold_time_does_not_qualify_model_balance(self):
        rows=summarize(trace())['mode_segments']
        self.assertEqual(rows[0]['duration_s'],50.)
        self.assertAlmostEqual(rows[1]['duration_s'],9.02)
        self.assertFalse(any(row['zero_model_30s_software_screen_passed'] for row in rows))

    def test_complete_zero_model_episode_is_only_a_software_screen(self):
        value=continuous_trace()
        result=summarize(value)
        self.assertTrue(result['mode_segments'][1]['zero_model_30s_software_screen_passed'])
        self.assertFalse(result['physical_independent_support_inferred'])
        self.assertFalse(result['walking_qualified'])

    def test_duration_does_not_hide_fault_or_nonzero_or_incomplete_trace(self):
        original=continuous_trace()
        changes=[('protective_stop',lambda d:d['terminal'].update(stop_reason='tilt')),
            ('TARGET_request_error',lambda d:d['frames'][-1].update(request_error='timeout')),
            ('nonzero_command_present',lambda d:d['frames'][-1].update(command=[.1,0.,0.])),
            ('trace_incomplete',lambda d:d.update(truncated=True)),
            ('feedback_not_fresh',lambda d:d['frames'][-1].update(age_s=.1))]
        for expected,change in changes:
            with self.subTest(expected=expected):
                value=copy.deepcopy(original);change(value)
                row=summarize(value)['mode_segments'][1]
                self.assertFalse(row['zero_model_30s_software_screen_passed'])
                self.assertIn(expected,row['failures'])

    def test_other_generation_terminal_never_extends_model_duration(self):
        value=trace();value['terminal'].update(monotonic=200,generation=3)
        row=summarize(value)['mode_segments'][1]
        self.assertFalse(row['matching_terminal'])
        self.assertEqual(row['duration_s'],9.)
        self.assertFalse(row['zero_model_30s_software_screen_passed'])

    def test_sparse_samples_do_not_prove_continuous_model_control(self):
        value=trace();value['terminal']['monotonic']=81.
        row=summarize(value)['mode_segments'][1]
        self.assertFalse(row['zero_model_30s_software_screen_passed'])
        self.assertIn('TARGET_coverage_gap_reached_400ms',row['failures'])

    def test_exact_scheduled_end_can_complete_the_30s_software_screen(self):
        value=continuous_trace();value['terminal']['stop_reason']=WINDOW_END_REASON
        row=summarize(value)['mode_segments'][1]
        self.assertTrue(row['scheduled_window_end'])
        self.assertTrue(row['zero_model_30s_software_screen_passed'])
        value['frames'][-1]['age_s']=.1
        self.assertFalse(summarize(value)['mode_segments'][1]['zero_model_30s_software_screen_passed'])

    def test_scheduled_end_text_cannot_qualify_a_short_or_faulted_episode(self):
        for reason in (WINDOW_END_REASON, WINDOW_END_REASON+' tilt'):
            value=continuous_trace();value['terminal'].update(monotonic=80.5,stop_reason=reason)
            row=summarize(value)['mode_segments'][1]
            self.assertFalse(row['scheduled_window_end'])
            self.assertIn('protective_stop',row['failures'])

    def test_invalid_gravity_and_time_cannot_be_clamped_into_a_pass(self):
        for key,value in (('gravity',[0.,0.,float('nan')]),('gravity',[0.,0.,-2.]),
                          ('monotonic',float('inf')),('age_s',float('nan'))):
            original=continuous_trace();original['frames'][-1][key]=value
            with self.assertRaises(ValueError):summarize(original)


if __name__=='__main__':unittest.main()
