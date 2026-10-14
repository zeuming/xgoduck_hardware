"""Behavioral checks for the new offline phase interface and learning signal."""
import copy
import math
import unittest
import torch
from torch import nn
from slow_gait_experiment import schedule,swing_shape,dense_gait_values,PhaseNormalizer,StandingResidual,lateral_transfer_values


class SlowGaitTests(unittest.TestCase):
    def test_com_signal_rewards_correct_support_side_continuously(self):
        feet=torch.tensor([[.025,-.025]]).expand(3,-1)
        com=torch.tensor([.01,0.,-.01]);phase=torch.full((3,),.15)
        reward=lateral_transfer_values(com,feet,phase)
        self.assertTrue((torch.diff(reward)>0).all())
        reverse=lateral_transfer_values(com,feet,phase+.5)
        self.assertTrue((torch.diff(reverse)<0).all())
        # Global translation cannot change this geometric objective.
        torch.testing.assert_close(reward,lateral_transfer_values(com+1.,feet+1.,phase))

    def test_schedule_starts_and_finishes_at_zero_and_alternates(self):
        command,phase=schedule(torch.tensor([0,100,125,175,899,900,999]),.02)
        self.assertEqual(command[0],0);self.assertEqual(command[1],0)
        self.assertAlmostEqual(float(command[2]),.03,places=6)
        self.assertEqual(command[-1],0);self.assertEqual(command[-2],0)
        shape=swing_shape(torch.tensor([.15,.65,.40,.90]))
        torch.testing.assert_close(shape,torch.tensor([[1.,0.],[0.,1.],[0.,0.],[0.,0.]]),atol=1e-6,rtol=0)

    def test_true_supported_clearance_better_than_unloading_alone(self):
        phase=torch.full((4,),.15);weight=torch.full((4,),8.);tilt=torch.zeros(4)
        height=torch.tensor([[0.,0.],[0.,0.],[.004,0.],[.004,.004]])
        load=torch.tensor([[4.,4.],[0.,8.],[0.,8.],[0.,0.]])
        reward=dense_gait_values(height,load,phase,weight,tilt)
        self.assertLess(reward[0],reward[1]);self.assertLess(reward[1],reward[2])
        self.assertEqual(reward[3],0.)
        fallen=dense_gait_values(height,load,phase,weight,torch.full((4,),math.radians(16)))
        self.assertTrue((fallen<=0).all())

    def test_forward_learning_cannot_change_zero_or_frozen_base(self):
        torch.manual_seed(45);mean=torch.randn(1,61);den=torch.rand(1,61)+.1
        base=nn.Sequential(nn.Linear(61,32),nn.ELU(),nn.Linear(32,14))
        before=copy.deepcopy(base.state_dict());normalizer=PhaseNormalizer(mean,den)
        residual=StandingResidual(base,mean,den,[1]*5+[0]*4+[1]*5)
        zero=torch.randn(100,63);zero[:,48:51]=0.
        fwd=zero.clone();fwd[:,48]=.03
        expected=base(normalizer(zero)[:,:61]).detach()
        torch.testing.assert_close(residual(normalizer(fwd)),expected,atol=0,rtol=0)
        opt=torch.optim.Adam(residual.parameters(),lr=.001)
        for _ in range(5):
            opt.zero_grad();loss=(residual(normalizer(fwd))-1).square().mean();loss.backward();opt.step()
        torch.testing.assert_close(residual(normalizer(zero)),expected,atol=0,rtol=0)
        self.assertGreater(float((residual(normalizer(fwd))-expected).detach().abs().max()),.01)
        for k,v in before.items():torch.testing.assert_close(v,base.state_dict()[k],atol=0,rtol=0)


if __name__=='__main__':unittest.main()
