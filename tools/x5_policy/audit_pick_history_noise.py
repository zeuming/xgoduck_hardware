"""Replay recorded sensors with paired action-history noise; no plant/training.

The three levels are the deterministic reference and the two already-used
training initializations (.02 and1). This is a fixed-sensor counterfactual,
not a sigma search, physical rollout, or proof of a better initialization.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import onnxruntime as ort
    from current_body_training import load_contract
    from onnx_actor_warmstart import _load_audited_graph, FrozenOnnxNormalizer
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    c = load_contract(a.root / 'current-body-v2/inputs/contract.json')
    model = c['files']['pick']['resolved_path']
    arrays, model_sha = _load_audited_graph(model, c['files']['pick']['sha256'])
    layers = [FrozenOnnxNormalizer(torch.from_numpy(arrays['obs_normalizer._mean']),
                                  torch.from_numpy(arrays['onnx::Div_24']))]
    for i in (0, 2, 4, 6):
        w = torch.from_numpy(arrays[f'mlp.{i}.weight'])
        layer = torch.nn.Linear(w.shape[1], w.shape[0])
        with torch.no_grad():
            layer.weight.copy_(w)
            layer.bias.copy_(torch.from_numpy(arrays[f'mlp.{i}.bias']))
        layers.append(layer)
        if i != 6:
            layers.append(torch.nn.ELU())
    network = torch.nn.Sequential(*layers).eval()
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = opts.inter_op_num_threads = 1
    session = ort.InferenceSession(model, opts, providers=['CPUExecutionProvider'])
    matrix = json.loads((a.root / 'original-matrix-v1/matrix.json').read_text())
    cases, archive = [], {}
    for case in matrix['cases']:
        if case['mode'] != 'pick':
            continue
        key = case['key']
        trace = a.root / 'original-matrix-v1' / key / 'trace.jsonl'
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        obs = np.asarray([row['actor_observation'] for row in rows], dtype=np.float32)
        baseline = np.asarray([row['raw_action'] for row in rows], dtype=np.float32)
        z = np.random.default_rng(case['seed']).standard_normal(baseline.shape).astype(np.float32)
        archive[key + '/observations'] = obs
        archive[key + '/paired_noise'] = z
        x = torch.from_numpy(obs[0]).requires_grad_(True)
        jac = torch.autograd.functional.jacobian(lambda t: network(t[None])[0], x)
        history_jac = jac[:, 34:48].detach().numpy().astype(np.float64)
        with torch.no_grad():
            expected = network(torch.from_numpy(obs)).numpy()
        np.testing.assert_allclose(expected, baseline, atol=1e-5, rtol=1e-5)
        result = dict(key=key, recorded_frames=len(rows), original_accepted=case['accepted'],
                      input_trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                      paired_noise_sha256=hashlib.sha256(z.tobytes()).hexdigest(),
                      local_history_jacobian_spectral_radius=float(max(abs(np.linalg.eigvals(history_jac)))),
                      local_history_jacobian_operator_norm=float(np.linalg.svd(history_jac, compute_uv=False)[0]),
                      scenarios=[])
        for sigma in (0., .02, 1.):
            history = obs[0, 34:48].copy()
            means, actions, differences = [], [], []
            nonfinite_at = None
            for step, recorded in enumerate(obs):
                v = recorded.copy()
                v[34:48] = history
                assert np.array_equal(v[:34], recorded[:34]) and np.array_equal(v[48:], recorded[48:])
                mean = session.run(None, {session.get_inputs()[0].name: v[None]})[0][0]
                action = mean + np.float32(sigma) * z[step]
                if not np.isfinite(action).all():
                    nonfinite_at = step
                    break
                means.append(mean.copy())
                actions.append(action.copy())
                differences.append(float(abs(mean - baseline[step]).max()))
                history = action
            means, actions = np.asarray(means), np.asarray(actions)
            if sigma == 0:
                np.testing.assert_allclose(means, baseline, atol=1e-5, rtol=1e-5)
            archive[key + f'/sigma{sigma}/mean'] = means
            archive[key + f'/sigma{sigma}/action'] = actions
            result['scenarios'].append(dict(sigma=sigma, simulated_plant=False,
                finite_frames=len(actions), nonfinite_at=nonfinite_at,
                maximum_mean_difference_from_expert=float(max(differences, default=0)),
                maximum_raw_action_abs=float(abs(actions).max()) if actions.size else None,
                mean_difference_first_8_frames=float(max(differences[:8], default=0))))
        cases.append(result)
    assert len(cases) == 6
    np.savez_compressed(a.output / 'counterfactual-arrays.npz', **archive)
    report = dict(scope=__doc__, created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  model_sha256=model_sha, analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  all_nonhistory_slots_fixed=True, same_standard_noise_per_case=True,
                  no_new_actor_parameters=True, source_model_modified=False, training_performed=False,
                  physics_performed=False, hardware_access=False, initialization_selected=False,
                  normalizer_history_denominator=arrays['onnx::Div_24'][0,34:48].tolist(),
                  inference_parity_passed=True, deterministic_history_replay_passed=True,
                  cases=cases, arrays_sha256=hashlib.sha256((a.output/'counterfactual-arrays.npz').read_bytes()).hexdigest(),
                  limitation='History-only local/fixed-sensor sensitivity cannot prove closed-loop instability, explain the learned failure, or select sigma. Actual plant feedback and learning remain absent.')
    (a.output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:report[k] for k in ['cases','inference_parity_passed','deterministic_history_replay_passed']}))


if __name__ == '__main__':
    main()
