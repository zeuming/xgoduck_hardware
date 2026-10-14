"""Regression check for the offline IK float32 finite-difference defect."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from nonlinear_swing_reference_precise import SwingReference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    fixture = json.loads(args.fixture.read_text())
    model = Path(fixture['model_path'])
    assert hashlib.sha256(model.read_bytes()).hexdigest() == fixture['model_sha256']
    results = []
    for side in ('left', 'right'):
        for height in (.001, .003):
            targets = []
            for dtype in (np.float32, np.float64):
                # Both inputs contain exactly the same measurement values.
                root = np.asarray(fixture['root'], dtype=np.float32).astype(dtype)
                actual = np.asarray(fixture['actual'], dtype=np.float32).astype(dtype)
                preload = np.zeros(14, dtype=dtype)
                saved = [x.copy() for x in (root, actual, preload)]
                teacher = SwingReference(model, fixture['joint_names'], fixture['joint_ids'],
                    side, root, actual, fixture['lower'], fixture['upper'])
                target, evidence = teacher.solve(root, actual, height, preload)
                assert evidence['success'], (side, height, dtype, evidence)
                assert evidence['position_error_m'] < 1e-6, evidence
                for before, after in zip(saved, (root, actual, preload)):
                    np.testing.assert_array_equal(before, after)
                targets.append(target)
                results.append(dict(side=side, height_m=height, input_dtype=str(dtype), **evidence))
            np.testing.assert_array_equal(*targets)
    report = dict(passed=True, cases=results, hardware_access=False,
        fixture_sha256=hashlib.sha256(args.fixture.read_bytes()).hexdigest(),
        claim='Same float32 measurements give identical double-precision IK results; no gait claim.')
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(passed=True, cases=len(results))))


if __name__ == '__main__':
    main()
