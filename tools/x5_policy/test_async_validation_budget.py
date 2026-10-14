"""Offline validation must report the deployed scheduler, not optimistic rates."""
import unittest

from validate_async_policy import scheduler_delay, timing_summary


class ValidationBudgetTests(unittest.TestCase):
    def test_overrun_yields_like_the_native_control_loop(self):
        self.assertEqual(scheduler_delay(.027), .001)
        self.assertEqual(scheduler_delay(.027, scheduler='legacy'), 0)
        self.assertAlmostEqual(scheduler_delay(.004), .016)
        self.assertEqual(scheduler_delay(.0195), .001)

    def test_delay_cpu_and_other_wall_cost_are_not_conflated(self):
        # A 27 ms replay cycle does not imply 27 ms of controller CPU work.
        rows=[dict(cycle_wall_ms=27, cycle_cpu_ms=8, transport_wall_ms=19,
                   injected_wait_ms=18, transport_excess_ms=1,
                   outside_transport_ms=8),
              dict(cycle_wall_ms=25, cycle_cpu_ms=7, transport_wall_ms=18,
                   injected_wait_ms=17, transport_excess_ms=1,
                   outside_transport_ms=7)]
        result=timing_summary(rows)
        self.assertEqual(result['cycle_cpu_ms'],7.5)
        self.assertEqual(result['cycle_wall_ms'],26)
        self.assertEqual(result['injected_wait_ms']+result['transport_excess_ms']+
                         result['outside_transport_ms'], result['cycle_wall_ms'])
        self.assertEqual(timing_summary([]),{})


if __name__ == '__main__':
    unittest.main()
