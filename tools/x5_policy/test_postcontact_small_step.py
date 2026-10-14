import unittest
from types import SimpleNamespace
import torch
from postcontact_small_step import PostContactReward


class PostContact(unittest.TestCase):
    def setUp(self):
        self.term = PostContactReward(SimpleNamespace(step_dt=.02, num_envs=2, device='cpu'))
        self.step = 0

    def feed(self, height, load, active=(True, True), done=(False, False)):
        self.step += 1
        return self.term.compute(torch.tensor(height), torch.tensor(load),
                                 torch.tensor(active), torch.tensor([self.step] * 2),
                                 torch.tensor(done))

    def test_zero_and_post_reset_never_gain_gait_reward(self):
        for _ in range(12):
            r = self.feed([[.005, 0.], [.005, 0.]], [[0., 8.], [0., 8.]],
                          active=(False, True), done=(False, True))
            torch.testing.assert_close(r, torch.zeros(2))
        self.assertFalse(self.term.air_count.any())
        self.assertFalse(self.term.landing.armed.any())

    def test_low_drag_and_double_flight_do_not_earn_air_reward(self):
        for _ in range(12):
            self.feed([[.002, 0.], [.006, .006]], [[0., 8.], [0., 0.]])
        torch.testing.assert_close(self.term.last_terms['post_qualified_air'], torch.zeros(2))
        torch.testing.assert_close(self.term.last_terms['post_alternating_landing'], torch.zeros(2))

    def test_support_gap_breaks_qualified_duration(self):
        for _ in range(9):
            self.feed([[.005, 0.], [.005, 0.]], [[0., 8.], [0., 8.]])
        torch.testing.assert_close(self.term.last_terms['post_qualified_air'], torch.full((2,), 3.))
        self.feed([[.005, 0.], [.005, 0.]], [[0., 0.], [0., 8.]])
        self.assertEqual(float(self.term.last_terms['post_qualified_air'][0]), 0.)
        self.feed([[.005, 0.], [.005, 0.]], [[0., 8.], [0., 8.]])
        self.assertEqual(float(self.term.last_terms['post_qualified_air'][0]), 0.)

    def test_selective_reset_preserves_other_world(self):
        for _ in range(9):
            self.feed([[.005, 0.], [.005, 0.]], [[0., 8.], [0., 8.]])
        self.term.reset(torch.tensor([0]))
        self.feed([[.005, 0.], [.005, 0.]], [[0., 8.], [0., 8.]])
        torch.testing.assert_close(self.term.last_terms['post_qualified_air'], torch.tensor([0., 3.]))


if __name__ == '__main__':
    unittest.main()
