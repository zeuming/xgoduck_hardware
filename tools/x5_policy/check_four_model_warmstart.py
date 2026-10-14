"""Offline four-model actor import/reload check; performs no PPO or device I/O."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    from train_constrained_policy import install_offline_guards, assert_state_identical
    install_offline_guards()
    import numpy as np
    import torch
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel
    from onnx_actor_warmstart import warmstart_actor_from_onnx

    torch.set_num_threads(1)
    observation = TensorDict({'actor': torch.zeros(1, 61)}, batch_size=[1])

    def make_actor():
        return MLPModel(observation, {'actor': ['actor']}, 'actor', 14,
                        hidden_dims=[512, 256, 128], activation='elu',
                        obs_normalization=True,
                        distribution_cfg={'class_name': 'rsl_rl.modules.distribution:GaussianDistribution',
                                          'init_std': 0.02})

    report = {'scope': __doc__, 'hardware_access': False, 'training_started': False,
              'full_training_checkpoint_recovered': False, 'models': {}}
    try:
        for name in ('walk', 'getup', 'pick', 'roulade'):
            path = args.models / f'xgoduck_{name}.onnx'
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            actor = make_actor()
            imported = warmstart_actor_from_onnx(actor, path, expected_sha256=digest)
            saved = args.output / f'{name}-actor-import.pt'
            torch.save({'actor': actor.state_dict(), 'source_sha256': digest,
                        'kind': 'actor_import_only_not_adapted_policy'}, saved)
            again = make_actor()
            warmstart_actor_from_onnx(again, path, expected_sha256=digest)
            state = torch.load(saved, map_location='cpu', weights_only=True)
            again.load_state_dict(state['actor'], strict=True)
            assert_state_identical(actor.state_dict(), again.state_dict())
            before = actor.obs_normalizer(torch.zeros(3, 61)).detach().clone()
            actor.train()
            actor.obs_normalizer.update(torch.full((3, 61), 100.))
            torch.testing.assert_close(actor.obs_normalizer(torch.zeros(3, 61)),
                                       before, atol=0, rtol=0)
            assert np.isfinite(imported['parity']['max_abs_error'])
            report['models'][name] = {'source_sha256': digest, 'import': imported,
                                      'strict_actor_reload': True,
                                      'normalization_stayed_frozen': True,
                                      'adaptation_trained': False}
        report['passed'] = True
    except Exception as error:
        report.update(passed=False, error=repr(error))
        raise
    finally:
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'passed': report['passed'], 'models': list(report['models']),
                      'training_started': False, 'hardware_access': False}))


if __name__ == '__main__':
    main()
