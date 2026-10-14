"""Actual Torch/RSL/ORT tests; no environment, training job or hardware access."""
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import onnx
import torch
from tensordict import TensorDict
from rsl_rl.models import MLPModel

from onnx_actor_warmstart import FrozenOnnxNormalizer, warmstart_actor_from_onnx


ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'output/x5_policy_bundle/models/xgoduck_walk.onnx'
TRACE = ROOT / 'output/duck_sim/repair-20261011/model/training-adapter/before-contact-reader-fix-real-button-pulse-diagnostic.trace.jsonl'


def new_actor(hidden=(512, 256, 128)):
    obs = TensorDict({'actor': torch.zeros(2, 61)}, batch_size=[2])
    return MLPModel(obs, {'policy': ['actor']}, 'policy', 14, hidden_dims=hidden,
                    activation='elu', obs_normalization=True,
                    distribution_cfg={'class_name': 'GaussianDistribution', 'init_std': 1., 'std_type': 'scalar'})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class OnnxActorWarmstartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.source_sha = sha(MODEL)
        cls.observations = np.asarray([json.loads(line)['actor_observation'] for line in TRACE.read_text().splitlines()], dtype=np.float32)

    def import_actor(self, actor, **kwargs):
        return warmstart_actor_from_onnx(actor, MODEL, expected_sha256=self.source_sha, **kwargs)

    def test_actual_rsl_actor_matches_onnx_on_recorded_observations(self):
        actor = new_actor()
        distribution = copy.deepcopy(actor.distribution.state_dict())
        ids_before = {n: id(p) for n, p in actor.named_parameters()}
        result = self.import_actor(actor, parity_observations=self.observations)
        self.assertTrue(result['parity']['passed'])
        self.assertEqual(result['supplied_parity_observations'], 301)
        self.assertFalse(result['checkpoint_resume'])
        self.assertFalse(result['training_started'])
        self.assertEqual(ids_before, {n: id(p) for n, p in actor.named_parameters()})
        for name, value in actor.distribution.state_dict().items():
            torch.testing.assert_close(value, distribution[name], atol=0, rtol=0)
        print('WARMSTART_PARITY ' + json.dumps(result['parity']))

    def test_normalization_stays_exact_through_training_mode_updates(self):
        actor = new_actor(); self.import_actor(actor)
        self.assertIsInstance(actor.obs_normalizer, FrozenOnnxNormalizer)
        before = copy.deepcopy(actor.obs_normalizer.state_dict())
        actor.train()
        for offset in (-1e4, 0, 1e4):
            actor.update_normalization(TensorDict({'actor': torch.ones(4, 61) * offset}, batch_size=[4]))
        for name, value in actor.obs_normalizer.state_dict().items():
            torch.testing.assert_close(value, before[name], atol=0, rtol=0)

    def test_existing_optimizer_keeps_weights_and_receives_gradients(self):
        actor = new_actor()
        optimizer = torch.optim.Adam(actor.parameters(), lr=1e-5)
        self.assertEqual(len(optimizer.state), 0)
        self.import_actor(actor)
        before = actor.mlp[6].weight.detach().clone()
        observations = TensorDict({'actor': torch.tensor(self.observations[:8])}, batch_size=[8])
        loss = actor(observations).square().mean()
        loss.backward()
        self.assertTrue(torch.isfinite(actor.mlp[6].weight.grad).all())
        self.assertGreater(float(actor.mlp[6].weight.grad.abs().sum()), 0)
        optimizer.step()
        self.assertFalse(torch.equal(before, actor.mlp[6].weight))
        self.assertTrue(all(torch.isfinite(p).all() for p in actor.parameters()))
        self.assertTrue(optimizer.state)

    def test_strict_checkpoint_reload_requires_and_retains_frozen_normalizer(self):
        actor = new_actor(); self.import_actor(actor)
        # Represent changed learned weights without pretending to run PPO here.
        with torch.no_grad(): actor.mlp[6].bias.add_(.01)
        buf = io.BytesIO(); torch.save(actor.state_dict(), buf); buf.seek(0)
        saved = torch.load(buf, weights_only=True)
        other = new_actor()
        with self.assertRaises(RuntimeError): other.load_state_dict(saved, strict=True)
        self.import_actor(other)
        other.load_state_dict(saved, strict=True)
        observations = TensorDict({'actor': torch.tensor(self.observations[:3])}, batch_size=[3])
        torch.testing.assert_close(other(observations), actor(observations), atol=0, rtol=0)
        other.train(); before = copy.deepcopy(other.obs_normalizer.state_dict())
        other.update_normalization(observations)
        for name, value in other.obs_normalizer.state_dict().items():
            torch.testing.assert_close(value, before[name], atol=0, rtol=0)

    def test_bad_hash_is_rejected_without_actor_mutation(self):
        actor = new_actor(); before = copy.deepcopy(actor.state_dict())
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            warmstart_actor_from_onnx(actor, MODEL, expected_sha256='bad')
        for name, value in actor.state_dict().items():
            torch.testing.assert_close(value, before[name], atol=0, rtol=0)

    def test_wrong_actor_layout_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'shape'):
            self.import_actor(new_actor((256, 256, 128)))

    def test_changed_activation_graph_is_rejected_even_with_matching_hash(self):
        actor = new_actor(); before = copy.deepcopy(actor.state_dict())
        model = onnx.load(str(MODEL))
        model.graph.node[3].attribute[0].f = 2.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'other.onnx'; onnx.save(model, str(path))
            with self.assertRaisesRegex(ValueError, 'attributes'):
                warmstart_actor_from_onnx(actor, path, expected_sha256=sha(path))
        for name, value in actor.state_dict().items():
            torch.testing.assert_close(value, before[name], atol=0, rtol=0)

    def test_invalid_parity_input_is_rejected_before_mutation(self):
        actor = new_actor(); normalizer = actor.obs_normalizer
        with self.assertRaisesRegex(ValueError, 'Parity observations'):
            self.import_actor(actor, parity_observations=np.full((1, 61), np.nan))
        self.assertIs(actor.obs_normalizer, normalizer)


if __name__ == '__main__':
    unittest.main()
