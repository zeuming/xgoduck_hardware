#!/usr/bin/env python3
"""Import the audited deterministic ONNX actor into genuine RSL-RL 5 MLPModel.

This is actor-only initialization, never recovery of a training checkpoint.
The ONNX contains neither critic, optimizer, exploration standard deviation nor
normalizer observation count. Keep the exported affine normalization frozen;
inventing a running count would silently change the initialized policy.

For a checkpoint produced after this import, install the same importer on the
fresh actor BEFORE strict runner.load(). The distinct normalizer state keys
make a vanilla empirical-normalizer load fail rather than resume with drift.
No training, network or hardware operation is performed by this module.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path

import numpy as np
import onnx
from onnx import numpy_helper
import onnxruntime as ort
import torch
from tensordict import TensorDict


class FrozenOnnxNormalizer(torch.nn.Module):
    """Exact exported (x - mean) / denominator, also frozen in training mode."""

    def __init__(self, mean: torch.Tensor, denominator: torch.Tensor):
        super().__init__()
        self.register_buffer("onnx_mean", mean.detach().clone())
        self.register_buffer("onnx_denominator", denominator.detach().clone())
        self.register_buffer("frozen_onnx_contract_version", torch.tensor(1, dtype=torch.int64, device=mean.device))

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return (observations - self.onnx_mean) / self.onnx_denominator

    @torch.jit.unused
    def update(self, observations: torch.Tensor) -> None:
        # RSL-RL calls this explicitly during PPO, even after actor.train().
        del observations


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_audited_graph(path, expected_sha256):
    actual_sha = _sha256(path)
    if not expected_sha256 or actual_sha != expected_sha256:
        raise ValueError("ONNX SHA256 differs from the pinned source model")
    model = onnx.load_model(str(path), load_external_data=False)
    onnx.checker.check_model(model)
    if any(i.data_location == onnx.TensorProto.EXTERNAL for i in model.graph.initializer):
        raise ValueError("External ONNX weights are not supported by this importer")
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError("Expected one observation input and one action output")
    arrays = {i.name: numpy_helper.to_array(i).copy() for i in model.graph.initializer}
    shapes = {"obs_normalizer._mean": (1, 61), "onnx::Div_24": (1, 61)}
    for layer, shape in zip((0, 2, 4, 6), ((512, 61), (256, 512), (128, 256), (14, 128))):
        shapes[f"mlp.{layer}.weight"] = shape
        shapes[f"mlp.{layer}.bias"] = (shape[0],)
    if set(arrays) != set(shapes):
        raise ValueError("ONNX initializers differ from the audited actor-only layout")
    for name, shape in shapes.items():
        value = arrays[name]
        if value.dtype != np.float32 or value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f"Invalid ONNX tensor: {name}")
    if (arrays["onnx::Div_24"] <= 0).any():
        raise ValueError("ONNX normalization denominator must be positive")
    nodes = list(model.graph.node)
    if [n.op_type for n in nodes] != ["Sub", "Div", "Gemm", "Elu", "Gemm", "Elu", "Gemm", "Elu", "Gemm"]:
        raise ValueError("ONNX graph differs from the audited Sub/Div/ELU MLP")
    previous = model.graph.input[0].name
    for index, node in enumerate(nodes):
        if node.domain not in ("", "ai.onnx") or len(node.output) != 1:
            raise ValueError("Unsupported ONNX operator domain or output")
        attrs = {a.name: onnx.helper.get_attribute_value(a) for a in node.attribute}
        if index < 2:
            other = "obs_normalizer._mean" if index == 0 else "onnx::Div_24"
            valid = list(node.input) == [previous, other] and not attrs
        elif node.op_type == "Elu":
            valid = list(node.input) == [previous] and attrs.get("alpha", 1.0) == 1.0 and set(attrs) <= {"alpha"}
        else:
            layer = index - 2
            valid = (list(node.input) == [previous, f"mlp.{layer}.weight", f"mlp.{layer}.bias"]
                     and attrs.get("alpha", 1.0) == 1.0 and attrs.get("beta", 1.0) == 1.0
                     and attrs.get("transA", 0) == 0 and attrs.get("transB", 0) == 1
                     and set(attrs) <= {"alpha", "beta", "transA", "transB"})
        if not valid:
            raise ValueError(f"Unsupported ONNX data flow or attributes at node {index}")
        previous = node.output[0]
    if previous != model.graph.output[0].name:
        raise ValueError("ONNX action output differs from the audited MLP")
    return arrays, actual_sha


def _validate_actor(actor):
    from rsl_rl.models import MLPModel
    from rsl_rl.modules.distribution import GaussianDistribution
    if (type(actor) is not MLPModel or actor.is_recurrent or actor.obs_dim != 61
            or len(actor.obs_groups) != 1 or not actor.obs_normalization
            or not isinstance(actor.distribution, GaussianDistribution)):
        raise ValueError("Expected genuine normalized RSL-RL MLPModel with one 61D observation group and Gaussian distribution")
    layers = list(actor.mlp)
    if len(layers) != 7:
        raise ValueError("Actor MLP architecture differs from the audited ONNX")
    for index, layer in enumerate(layers):
        if index % 2:
            if type(layer) is not torch.nn.ELU or layer.alpha != 1.0:
                raise ValueError("Actor activation differs from the audited ELU")
        else:
            shape = ((512, 61), (256, 512), (128, 256), (14, 128))[index // 2]
            if (type(layer) is not torch.nn.Linear or layer.bias is None or tuple(layer.weight.shape) != shape
                    or layer.weight.dtype != torch.float32):
                raise ValueError("Actor MLP shape or dtype differs from the audited ONNX")


def _install(actor, arrays):
    parameter = next(actor.parameters())
    with torch.no_grad():
        for layer in (0, 2, 4, 6):
            actor.mlp[layer].weight.copy_(torch.as_tensor(arrays[f"mlp.{layer}.weight"], device=parameter.device))
            actor.mlp[layer].bias.copy_(torch.as_tensor(arrays[f"mlp.{layer}.bias"], device=parameter.device))
    actor.obs_normalizer = FrozenOnnxNormalizer(
        torch.as_tensor(arrays["obs_normalizer._mean"], device=parameter.device),
        torch.as_tensor(arrays["onnx::Div_24"], device=parameter.device))


def _parity(actor, path, observations):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
    # The upstream ONNX export fixes batch=1; do not edit its graph for this check.
    expected = np.concatenate([session.run(None, {session.get_inputs()[0].name: row[None]})[0] for row in observations])
    device = next(actor.parameters()).device
    with torch.no_grad():
        inputs = TensorDict({actor.obs_groups[0]: torch.as_tensor(observations, device=device)}, batch_size=[len(observations)])
        actual = actor(inputs, stochastic_output=False).cpu().numpy()
    if not np.isfinite(actual).all() or not np.allclose(actual, expected, atol=1e-5, rtol=1e-5):
        raise ValueError("Imported Torch actor fails ONNX inference equivalence")
    return dict(observations=len(observations), max_abs_error=float(np.max(np.abs(actual - expected))),
                atol=1e-5, rtol=1e-5, passed=True)


def warmstart_actor_from_onnx(actor, model_path, *, expected_sha256, parity_observations=None):
    """Initialize actor weights, preserving its existing std and parameter objects.

    Critic and optimizer are not arguments and cannot be restored by this API.
    Provide actual 61D observations for an extra parity check. Hash, graph,
    architecture and parity are checked on a copy before modifying the actor.
    Keep this frozen normalizer during subsequent PPO. Adapting its statistics
    would be a separate change, requiring an explicit policy continuity plan.
    """
    _validate_actor(actor)
    arrays, model_sha = _load_audited_graph(model_path, expected_sha256)
    mean, denominator = arrays["obs_normalizer._mean"], arrays["onnx::Div_24"]
    rng = np.random.default_rng(0)
    observations = np.concatenate([np.zeros((1, 61), np.float32), mean,
        mean + denominator * rng.normal(size=(32, 61)).astype(np.float32)])
    extra_rows = 0
    if parity_observations is not None:
        extra = np.asarray(parity_observations, dtype=np.float32)
        if extra.ndim != 2 or extra.shape[1] != 61 or not len(extra) or not np.isfinite(extra).all():
            raise ValueError("Parity observations must be finite nonempty N by 61")
        observations = np.concatenate([observations, extra])
        extra_rows = len(extra)
    trial = copy.deepcopy(actor)
    _install(trial, arrays)
    parity = _parity(trial, model_path, observations)
    _install(actor, arrays)
    return dict(kind="actor_only_onnx_warmstart", source_onnx_sha256=model_sha,
                normalization="frozen exact exported mean and denominator; update() is a no-op even in train mode",
                actor_parameter_objects_preserved=True, exploration_std="preserved from newly created actor, not imported",
                critic="not imported; new training runner initializes it", optimizer="not imported; new training runner initializes it",
                checkpoint_resume=False, training_started=False, parity=parity,
                supplied_parity_observations=extra_rows, checkpoint_reload="install same frozen normalizer via this importer before strict runner.load")
