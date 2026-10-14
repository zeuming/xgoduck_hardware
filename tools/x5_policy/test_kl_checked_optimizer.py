import copy
import unittest
import torch
from kl_checked_optimizer import KlCheckedOptimizer


class CheckedOptimizerTests(unittest.TestCase):
    def setup_problem(self,bound,trials=16):
        actor=torch.nn.Parameter(torch.tensor([0.]))
        critic=torch.nn.Parameter(torch.tensor([0.]))
        opt=torch.optim.Adam([actor,critic],lr=.1)
        measure=lambda:dict(mean_kl=float((actor.detach()**2).sum()))
        guard=KlCheckedOptimizer(opt,[actor],[critic],measure,bound,trials)
        actor.grad=torch.tensor([-1.]);critic.grad=torch.tensor([-1.])
        return actor,critic,opt,guard

    def test_backtracking_restores_adam_moments_and_critic_trial_work(self):
        actor,critic,opt,g=self.setup_problem(.0007)
        opt.step()
        self.assertLessEqual(float(actor.detach()**2),.0007)
        self.assertAlmostEqual(float(actor.detach()),.025,places=6)
        self.assertAlmostEqual(float(critic.detach()),.1,places=6)
        self.assertEqual(g.trial_steps,3)
        for p in (actor,critic):
            self.assertEqual(int(opt.state[p]['step']),1)
            torch.testing.assert_close(opt.state[p]['exp_avg'],torch.tensor([-.1]))

    def test_unaccepted_actor_keeps_parameters_and_moments_unchanged(self):
        actor,critic,opt,g=self.setup_problem(1e-20,trials=2)
        opt.step()
        self.assertEqual(float(actor.detach()),0.)
        self.assertNotIn(actor,opt.state)
        self.assertAlmostEqual(float(critic.detach()),.1,places=6)
        self.assertEqual(g.actor_steps,0)
        self.assertEqual(g.critic_only_steps,1)
        self.assertEqual(int(opt.state[critic]['step']),1)

    def test_checkpoint_can_load_with_matching_groups(self):
        _,_,opt,g=self.setup_problem(.0007);opt.step()
        checkpoint=copy.deepcopy(opt.state_dict())
        _,_,other,_=self.setup_problem(.0007)
        other.load_state_dict(checkpoint)
        self.assertEqual([p['training_role'] for p in other.param_groups],['actor','critic'])
        self.assertEqual([p['lr'] for p in other.param_groups],[.025,.1])

    def test_preexisting_violation_does_not_take_another_step(self):
        actor,critic,opt,g=self.setup_problem(.0007)
        with torch.no_grad():actor.fill_(1.)
        with self.assertRaisesRegex(ValueError,'pre-step'):opt.step()
        self.assertEqual(float(critic.detach()),0.)
        self.assertFalse(opt.state)

    def test_next_rollout_recovers_first_useful_rate(self):
        actor,critic,opt,g=self.setup_problem(.0007)
        g.begin_rollout();opt.step()
        self.assertAlmostEqual(g.preferred_actor_lr,.025)
        opt.param_groups[0]['lr']=1e-12
        g.rollout_closed=True
        g.begin_rollout()
        self.assertAlmostEqual(opt.param_groups[0]['lr'],.0375)
        self.assertFalse(g.rollout_closed)

    def test_closed_rollout_updates_only_critic_until_new_rollout(self):
        actor,critic,opt,g=self.setup_problem(1e-20,trials=2)
        g.begin_rollout();opt.step()
        trials=g.trial_steps;opt.step()
        self.assertEqual(g.trial_steps,trials+1)
        self.assertEqual(float(actor.detach()),0.)
        self.assertEqual(int(opt.state[critic]['step']),2)

    def test_boundary_resume_matches_next_update_parameters_moments_and_backtracking(self):
        a,c,opt,g=self.setup_problem(.0007)
        g.begin_rollout();opt.step()
        state=g.continuation_state();adam=copy.deepcopy(opt.state_dict())
        aa,cc,other,h=self.setup_problem(.0007)
        with torch.no_grad():aa.copy_(a);cc.copy_(c)
        other.load_state_dict(adam);h.restore_continuation_state(state)
        # New rollout reference distributions, with identical next gradients.
        old=a.detach().clone();old2=aa.detach().clone()
        g.measure=lambda:dict(mean_kl=float(((a.detach()-old)**2).sum()))
        h.measure=lambda:dict(mean_kl=float(((aa.detach()-old2)**2).sum()))
        for guard,optimizer in ((g,opt),(h,other)):
            guard.begin_rollout();optimizer.step();optimizer.step()
        from train_constrained_policy import assert_state_identical
        assert_state_identical(opt.state_dict(),other.state_dict())
        assert_state_identical(g.continuation_state(),h.continuation_state())
        torch.testing.assert_close(a,aa,atol=0,rtol=0)
        torch.testing.assert_close(c,cc,atol=0,rtol=0)

    def test_invalid_continuation_is_rejected_before_mutation(self):
        _,_,_,g=self.setup_problem(.0007)
        original=g.continuation_state()
        for key,value in [('maximum_kl',1.),('preferred_actor_lr',float('nan')),
                          ('actor_steps',99),('schema_version',2)]:
            invalid=copy.deepcopy(original);invalid[key]=value
            with self.assertRaises(ValueError):g.restore_continuation_state(invalid)
            self.assertEqual(g.continuation_state(),original)


if __name__=='__main__':unittest.main()
