"""Real Torch tests of bounded density, PPO contracts and deterministic export."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import onnxruntime as ort
import torch
from rsl_rl.modules.distribution import GaussianDistribution

from bounded_action_distribution import ArctanActionTransform, HomeAnchoredArctanGaussian, distribution_config


ROOT = Path(__file__).resolve().parents[2]
CANDIDATE = ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated/candidate.json'


class BoundedDistributionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.candidate = json.loads(CANDIDATE.read_text())['candidate']
        cls.config = distribution_config(cls.candidate)

    def distribution(self, sigma=1.):
        return HomeAnchoredArctanGaussian(self.config, GaussianDistribution(14, init_std=sigma, std_type='log'))

    def test_zero_and_first_derivative_identity_and_head_unchanged(self):
        transform = ArctanActionTransform(self.config)
        z = torch.zeros(2, 14, requires_grad=True)
        mapped = transform(z)
        torch.testing.assert_close(mapped, z, atol=0, rtol=0)
        mapped.sum().backward()
        torch.testing.assert_close(z.grad, torch.ones_like(z), atol=1e-6, rtol=0)
        values = torch.randn(50, 14)
        torch.testing.assert_close(transform(values)[:, ~transform.mask], values[:, ~transform.mask], atol=0, rtol=0)

    def test_actual_ranges_sigma_one_100k_no_endpoints_and_roundtrip(self):
        torch.manual_seed(0)
        transform = ArctanActionTransform(self.config)
        z = torch.randn(100000, 14)
        output = transform(z)
        recovered, valid = transform.inverse_and_support(output)
        self.assertTrue(bool(valid.all()))
        self.assertTrue(bool(torch.isfinite(recovered).all()))
        max_error = float((recovered - z).abs().max())
        self.assertLess(max_error, 4e-5)
        print('ARCTAN_ROUNDTRIP ' + json.dumps({'rows':len(z),'max_abs_error':max_error,'all_strictly_in_support':True}))

    def test_density_normalizes_in_narrow_left_ankle_interval(self):
        distribution = self.distribution(.35)
        nodes, weights = np.polynomial.legendre.leggauss(512)
        joint = self.candidate['policy_joint_ids'].index(14)
        low, high = self.config['lower_action'][joint], self.config['upper_action'][joint]
        actions = torch.zeros(512,14)
        actions[:,joint] = torch.tensor(low + (nodes+1)*(high-low)/2, dtype=torch.float32)
        distribution.update(torch.zeros(512,14))
        # At zero every other transform has derivative 1, hence known normal density.
        other_log_density = float(torch.distributions.Normal(0.,.35).log_prob(torch.tensor(0.))) * 13
        marginal = torch.exp(distribution.log_prob(actions) - other_log_density).detach().numpy()
        integral = np.sum(weights*marginal)*(high-low)/2
        self.assertAlmostEqual(float(integral),1.,places=4)

    def test_log_probability_includes_exact_jacobian_and_has_stable_gradients(self):
        distribution = self.distribution(.35)
        mean = torch.full((4,14),.1,requires_grad=True)
        distribution.update(mean)
        z = torch.linspace(-.4,.4,56).reshape(4,14)
        outputs = distribution.transform(z).detach()
        actual = distribution.log_prob(outputs)
        expected = (distribution._distribution.log_prob(z) - distribution.transform.log_abs_det_jacobian(z)).sum(-1)
        torch.testing.assert_close(actual,expected,atol=2e-5,rtol=1e-5)
        actual.sum().backward()
        self.assertTrue(bool(torch.isfinite(mean.grad).all()))
        self.assertTrue(bool(torch.isfinite(distribution.log_std_param.grad).all()))
        self.assertGreater(float(mean.grad.abs().sum()),0)
        # Independent central finite difference for one latent mean component.
        analytic = float(mean.grad[0,0]); eps=1e-3
        m0=mean.detach().clone(); m0[0,0]+=eps; distribution.update(m0); plus=float(distribution.log_prob(outputs).sum())
        m0[0,0]-=2*eps; distribution.update(m0); minus=float(distribution.log_prob(outputs).sum())
        self.assertAlmostEqual(analytic,(plus-minus)/(2*eps),delta=.004)

    def test_entropy_uses_transformed_density_with_pathwise_gradients(self):
        torch.manual_seed(12)
        distribution = self.distribution(.35)
        mean = torch.full((40000,14),.1,requires_grad=True)
        distribution.update(mean)
        entropy = distribution.entropy
        samples = distribution.sample()
        measured = -distribution.log_prob(samples).mean()
        self.assertAlmostEqual(float(entropy.mean()),float(measured),delta=.06)
        self.assertLess(float(entropy.mean()),float(distribution._distribution.entropy().sum(-1).mean()))
        entropy.mean().backward()
        self.assertTrue(bool(torch.isfinite(mean.grad).all()))
        self.assertTrue(bool(torch.isfinite(distribution.log_std_param.grad).all()))
        self.assertGreater(float(distribution.log_std_param.grad.abs().sum()),0)

    def test_same_params_ppo_ratio_one_kl_exact_and_cross_contract_rejected(self):
        distribution = self.distribution(.35)
        distribution.update(torch.zeros(16,14))
        action = distribution.sample(); old_log = distribution.log_prob(action).detach()
        old = tuple(v.detach().clone() for v in distribution.params)
        distribution.update(torch.zeros(16,14))
        torch.testing.assert_close((distribution.log_prob(action)-old_log).exp(),torch.ones(16),atol=0,rtol=0)
        distribution.update(torch.full((16,14),.1))
        new = distribution.params
        expected = torch.distributions.kl_divergence(torch.distributions.Normal(old[0],old[1]),torch.distributions.Normal(new[0],new[1])).sum(-1)
        torch.testing.assert_close(distribution.kl_divergence(old,new),expected,atol=0,rtol=0)
        corrupted = list(old); corrupted[2] = corrupted[2].clone(); corrupted[2][0,0]+=1
        with self.assertRaisesRegex(ValueError,'different fixed'):
            distribution.kl_divergence(tuple(corrupted),new)

    def test_outside_support_is_zero_density_without_clipping(self):
        distribution = self.distribution()
        distribution.update(torch.zeros(3,14))
        outputs = torch.zeros(3,14); outputs[0,4]=distribution.transform.upper[4]
        outputs[1,4]=distribution.transform.lower[4]-.1; outputs[2,4]=torch.nan
        log_prob=distribution.log_prob(outputs)
        self.assertTrue(bool(torch.isneginf(log_prob[:2]).all()))
        self.assertTrue(bool(torch.isnan(log_prob[2])))
        with self.assertRaisesRegex(ValueError,'Nonfinite'):
            distribution.update(torch.full((1,14),torch.inf))

    def test_checkpoint_rejects_different_bounds_even_with_same_keys(self):
        distribution = self.distribution(); saved=copy.deepcopy(distribution.state_dict())
        same=self.distribution(); same.load_state_dict(saved,strict=True)
        config=copy.deepcopy(self.config); config['upper_action'][0]+=.01
        different=HomeAnchoredArctanGaussian(config,GaussianDistribution(14,std_type='log'))
        before=different.transform.upper.clone()
        with self.assertRaisesRegex(RuntimeError,'fixed action transform'):
            different.load_state_dict(saved,strict=True)
        torch.testing.assert_close(before,different.transform.upper,atol=0,rtol=0)

    def test_onnx_export_matches_transform_and_propagates_nonfinite_inputs(self):
        transform=ArctanActionTransform(self.config)
        values=torch.randn(64,14); values[0,0]=torch.inf; values[1,4]=-torch.inf; values[2,11]=torch.nan
        expected=transform(values).numpy()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'transform.onnx'
            torch.onnx.export(transform,(torch.zeros(1,14),),str(path),input_names=['latent'],output_names=['action'],
                dynamic_axes={'latent':{0:'batch'},'action':{0:'batch'}},opset_version=18,dynamo=False)
            options=ort.SessionOptions(); options.intra_op_num_threads=1; options.inter_op_num_threads=1
            session=ort.InferenceSession(str(path),options,providers=['CPUExecutionProvider'])
            actual=session.run(None,{'latent':values.numpy()})[0]
        np.testing.assert_allclose(actual,expected,atol=1e-6,rtol=1e-6,equal_nan=True)
        self.assertTrue(np.isposinf(actual[0,0])); self.assertTrue(np.isneginf(actual[1,4])); self.assertTrue(np.isnan(actual[2,11]))


if __name__ == '__main__': unittest.main()
