import unittest
from types import SimpleNamespace
import torch
from small_step_experiment import AlternatingLanding,height_values,progress_gate

class Rewards(unittest.TestCase):
    def test_command_gating_preserves_zero_and_penalizes_idle(self):
        x=progress_gate(torch.tensor([0.,.05,.05,.05,.05]),torch.tensor([0.,0.,-.1,.05,.1]))
        torch.testing.assert_close(x,torch.tensor([1.,.2,.2,1.,1.]))
    def test_airborne_and_loaded_lift_are_not_supported_clearance(self):
        height=torch.tensor([[.005,0.],[.005,.005],[.005,0.],[0.,0.]])
        load=torch.tensor([[0.,8.],[0.,0.],[4.,4.],[4.,4.]])
        good,_=height_values(height,load)
        torch.testing.assert_close(good,torch.tensor([1.,0.,0.,0.]))
    def test_full_landing_alternation_and_time_span(self):
        for dt in [.02,.025]:
            env=SimpleNamespace(step_dt=dt,device='cpu',num_envs=1)
            term=AlternatingLanding(None,env);step=0
            def feed(side=None,air=False,n=1):
                nonlocal step
                total=0.
                for _ in range(n):
                    step+=1;h=torch.zeros(1,2);f=torch.tensor([[4.,4.]])
                    if side is not None and air:h[0,side]=.004;f[0,side]=0.;f[0,1-side]=8.
                    total+=float(term.update(h,f,torch.tensor([True]),torch.tensor([step]))[0])*dt
                return total
            n=term.span_samples
            # Fewer samples spans less than60ms even if nominal sample count*dt=60ms.
            feed(0,True,n-1);self.assertEqual(feed(n=n),0.)
            feed(0,True,n);self.assertEqual(feed(n=n-1),0.);self.assertAlmostEqual(feed(),1.)
            feed(0,True,n);self.assertEqual(feed(n=n),0.)
            feed(1,True,n);self.assertAlmostEqual(feed(n=n),1.)
            term.reset();feed(1,True,n);self.assertAlmostEqual(feed(n=n),1.)
    def test_duplicate_evaluation_rejected(self):
        term=AlternatingLanding(None,SimpleNamespace(step_dt=.02,device='cpu',num_envs=1))
        args=(torch.zeros(1,2),torch.ones(1,2),torch.tensor([True]),torch.tensor([1]))
        term.update(*args)
        with self.assertRaises(ValueError):term.update(*args)
    def test_load_without_sole_landing_does_not_complete_step(self):
        term=AlternatingLanding(None,SimpleNamespace(step_dt=.02,device='cpu',num_envs=1))
        for step in range(1,5):term.update(torch.tensor([[.004,0.]]),torch.tensor([[0.,8.]]),torch.tensor([True]),torch.tensor([step]))
        for step in range(5,10):
            value=term.update(torch.tensor([[.002,0.]]),torch.tensor([[2.,6.]]),torch.tensor([True]),torch.tensor([step]))
            self.assertEqual(float(value[0]),0.)

if __name__=='__main__':unittest.main()
