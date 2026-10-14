import copy
import unittest
import torch
from torch import nn
from residual_hold_experiment import discounted_prefix,HybridExport


class ReturnTests(unittest.TestCase):
    def test_terminal_reward_once_and_reset_rewards_excluded(self):
        r=torch.tensor([[1.,10.],[2.,20.],[100.,30.],[100.,40.]])
        d=torch.tensor([[0,0],[1,0],[0,0],[0,1]],dtype=torch.bool)
        actual,ended=discounted_prefix(r,d,.9)
        torch.testing.assert_close(actual,torch.tensor([1+.9*2,10+.9*20+.9**2*30+.9**3*40]))
        self.assertTrue(ended.all())
    def test_nonterminal_macro_discount_matches_micro_bellman_return(self):
        r=torch.arange(20.).reshape(10,2);d=torch.zeros(10,2,dtype=torch.bool);v=torch.tensor([2.,5.]);g=.99
        total,ended=discounted_prefix(r,d,g)
        expected=v
        for reward in reversed(r):expected=reward+g*expected
        torch.testing.assert_close(total+g**10*v,expected);self.assertFalse(ended.any())
    def test_explicit_update_flag_holds_policy_correction(self):
        class Lower(nn.Module):
            def forward(self,obs,residual):return residual+obs[:,:10]
        class Actor(nn.Module):
            def __init__(self):
                super().__init__();self.obs_normalizer=nn.Identity();self.mlp=nn.Linear(73,10)
        actor=Actor();model=HybridExport(Lower(),actor)
        obs=torch.randn(3,61);phase=torch.randn(3,2);held=torch.randn(3,10)
        actions,newheld=model(obs,phase,held,torch.tensor([[0.],[1.],[0.]]))
        torch.testing.assert_close(newheld[[0,2]],held[[0,2]],atol=0,rtol=0)
        expected=actor.mlp(torch.cat((obs,phase,held),dim=-1))
        torch.testing.assert_close(newheld[1],expected[1],atol=0,rtol=0)
        torch.testing.assert_close(actions,newheld+obs[:,:10])


if __name__=='__main__':unittest.main()
