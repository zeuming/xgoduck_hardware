#!/usr/bin/env python3
"""CPU policy inference only. Does not open a serial port or command motors."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import onnxruntime as ort
from rl_core import policy_profile

HERE = Path(__file__).resolve().parent
NAMES = ('walk', 'getup', 'pick', 'roulade')


def session(name):
    manifest = json.loads((HERE / 'models/manifest.json').read_text())
    path = HERE / 'models' / f'xgoduck_{name}.onnx'
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['models'][path.name]['sha256']:
        raise ValueError(f'Model hash mismatch: {path.name}')
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    opts.inter_op_num_threads = 1
    opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    s = ort.InferenceSession(str(path), sess_options=opts, providers=['CPUExecutionProvider'])
    if len(s.get_inputs()) != 1 or s.get_inputs()[0].shape != [1, 61]:
        raise ValueError('Expected one [1,61] observation input')
    if len(s.get_outputs()) != 1 or s.get_outputs()[0].shape != [1, 14]:
        raise ValueError('Expected one [1,14] action output')
    policy_profile(s)
    return s


def infer(s, obs):
    obs = np.asarray(obs, dtype=np.float32)
    if obs.shape != (1, 61) or not np.isfinite(obs).all():
        raise ValueError('Observation must be finite [1,61] float32')
    action = s.run(None, {s.get_inputs()[0].name: obs})[0]
    if action.shape != (1, 14) or not np.isfinite(action).all():
        raise ValueError('Invalid policy action')
    return action


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=NAMES, default='walk')
    parser.add_argument('--observation', type=Path, help='JSON array [1,61], already normalized to policy units/frame')
    parser.add_argument('--benchmark', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.benchmark:
        results = {}
        for name in NAMES:
            s = session(name)
            obs = np.zeros((1, 61), dtype=np.float32)
            obs[0, 5] = -1  # Synthetic level gravity, home pose, zero velocity.
            for _ in range(100):
                infer(s, obs)
            elapsed = []
            for _ in range(1000):
                start = time.perf_counter_ns()
                action = infer(s, obs)
                elapsed.append((time.perf_counter_ns() - start) / 1e6)
            results[name] = dict(provider=s.get_providers(), input=s.get_inputs()[0].shape,
                                 output=list(action.shape), iterations=1000,
                                 mean_ms=float(np.mean(elapsed)), p95_ms=float(np.percentile(elapsed, 95)),
                                 p99_ms=float(np.percentile(elapsed, 99)), max_ms=max(elapsed),
                                 finite=True)
        result = dict(test='synthetic_observation_cpu_inference_only', onnxruntime=ort.__version__, models=results)
    else:
        if args.observation is None:
            parser.error('Supply --observation or --benchmark')
        result = dict(model=args.model, action_radians_relative_to_home=infer(
            session(args.model), json.loads(args.observation.read_text())).tolist())
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
