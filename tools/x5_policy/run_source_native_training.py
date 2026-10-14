"""Bounded getup/pick experiment restoring the pinned source PPO settings.

The official exported actor/affine normalization is reused, with a fresh critic,
Adam and source trainable exploration distribution. This is not restoration of
an unavailable official training checkpoint. No real hardware access is used.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bundle = args.bundle.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    state = dict(scope=__doc__, pid=os.getpid(), tasks={},
                 started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                 domains=['reference', 'rear-mid', 'rear-high'],
                 learning_variant='source', source_observer='off', actor_step_admission=False,
                 initialization='official ONNX actor; fresh critic/Adam/source exploration',
                 checkpoint_resume=False, hardware_access=False, converged=False,
                 num_envs_per_domain=256, steps=24, iterations_limit=1024,
                 task_wall_seconds=2400, task_outer_seconds=2580,
                 maximum_transitions_per_model=18874368)

    def save(stage):
        state.update(stage=stage, wall_seconds=time.monotonic() - started)
        tmp = output / 'batch.tmp'
        tmp.write_text(json.dumps(state, indent=2))
        tmp.replace(output / 'batch.json')

    save('starting')
    for mode in ('getup', 'pick'):
        command = [sys.executable, str(bundle / 'tools/train_current_body_policy.py'),
                   '--contract', str(bundle / 'inputs/contract.json'),
                   '--mode', mode, '--domains', 'reference,rear-mid,rear-high',
                   '--output', str(output / mode), '--device', 'cuda:0',
                   '--num-envs', '256', '--iterations', '1024', '--steps', '24',
                   '--epochs', '5', '--snapshot-every', '64',
                   '--learning-variant', 'source', '--source-observer', 'off',
                   '--wall-seconds', '2400', '--max-transitions', '18874368']
        state['current_mode'] = mode
        save('training_' + mode)
        print('START_TASK ' + mode, flush=True)
        with (output / (mode + '.log')).open('x') as log:
            try:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=2580)
            except subprocess.TimeoutExpired:
                state['tasks'][mode] = {'exit': 'outer_timeout'}
                save('failed_requires_retrieval_and_shutdown')
                return
        report_path = output / mode / 'run.json'
        report = json.loads(report_path.read_text()) if report_path.exists() else {}
        state['tasks'][mode] = {k: report.get(k) for k in
                               ('stage', 'iterations', 'transitions', 'export', 'error')}
        state['tasks'][mode]['exit'] = result.returncode
        print('END_TASK ' + mode + ' ' + json.dumps(state['tasks'][mode]), flush=True)
        if result.returncode != 0 or report.get('error'):
            save('failed_requires_retrieval_and_shutdown')
            return
        save('finished_' + mode)
    state.pop('current_mode', None)
    save('complete_needs_retrieval_shutdown_and_evaluation')


if __name__ == '__main__':
    main()
