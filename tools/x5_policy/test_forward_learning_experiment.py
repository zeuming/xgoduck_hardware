"""The new objective must not silently alter zero behavior or physical limits."""
import copy
import math
import unittest

import torch
from torch import nn

from forward_learning_experiment import ZeroPreservingResidual, relative_tracking


class ForwardExperimentTests(unittest.TestCase):
    def test_relative_reward_distinguishes_stationary_low_speed(self):
        cmd=torch.tensor([[0.,0.,0.],[.05,0.,0.],[.1,0.,0.],[.2,0.,0.]])
        reward=relative_tracking(cmd,torch.zeros_like(cmd),math.sqrt(.1))
        torch.testing.assert_close(reward,torch.tensor([1.,math.exp(-4),math.exp(-4),math.exp(-4)]))
        torch.testing.assert_close(relative_tracking(cmd,cmd,math.sqrt(.1)),torch.ones(4))

    def test_zero_objective_exact_upstream_and_vertical_error_retained(self):
        actual=torch.tensor([[.01,.02,.03],[.1,0.,.03]])
        zero=torch.zeros_like(actual)
        torch.testing.assert_close(relative_tracking(zero,actual,math.sqrt(.1)),
            torch.exp(-(actual**2).sum(1)/math.sqrt(.1)**2),atol=0,rtol=0)
        cmd=torch.tensor([[.05,0.,0.]])
        self.assertLess(float(relative_tracking(cmd,cmd+torch.tensor([[0.,0.,.01]]),math.sqrt(.1))),1.)

    def test_learning_forward_preserves_exact_zero_and_head_for_arbitrary_states(self):
        torch.manual_seed(412)
        mean=torch.randn(1,61);den=torch.rand(1,61)+.1
        base=nn.Sequential(nn.Linear(61,32),nn.ELU(),nn.Linear(32,14))
        base_state=copy.deepcopy(base.state_dict())
        mask=[1]*5+[0]*4+[1]*5
        model=ZeroPreservingResidual(base,mean,den,mask)
        raw=torch.randn(128,61);raw[:,48:51]=0
        zero=(raw-mean)/den
        fwd=raw.clone();fwd[:,48]=.2;fwd=(fwd-mean)/den
        torch.testing.assert_close(model(zero),base(zero),atol=0,rtol=0)
        torch.testing.assert_close(model(fwd),base(fwd),atol=0,rtol=0)
        opt=torch.optim.Adam(model.parameters(),lr=.01)
        for _ in range(4):
            opt.zero_grad();loss=(model(fwd)-1).square().mean();loss.backward();opt.step()
        self.assertGreater(float((model(fwd)-base(fwd)).detach().abs().max()),.01)
        torch.testing.assert_close(model(zero),base(zero),atol=0,rtol=0)
        torch.testing.assert_close(model(fwd)[:,5:9],base(fwd)[:,5:9],atol=0,rtol=0)
        for key,value in base_state.items():torch.testing.assert_close(value,base.state_dict()[key],atol=0,rtol=0)
        rebuilt=ZeroPreservingResidual(copy.deepcopy(base),mean,den,mask)
        rebuilt.load_state_dict(model.state_dict(),strict=True)
        torch.testing.assert_close(model(fwd),rebuilt(fwd),atol=0,rtol=0)


if __name__=='__main__':unittest.main()
