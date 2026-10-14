"""Prepare a narrowly pinned late-policy-result patch; no robot access."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('Pinned source differs: ' + old[:90])
    return source.replace(old, new, 1)


def prepare(source, destination):
    source, destination = Path(source), Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    for folder in ('policy', 'service'):
        shutil.copytree(source/folder, destination/folder)
    pinned = {
        'policy/production_motion.py': '8e735e32d957f9e76dcea2b87f5d3fa1a69b8f65fa1735a94ac2e3af0f447aa1',
        'policy/candidate_controller.py': '3fe2a3bcb7fb655657a00fbb6f050f840036d2d321b474d11fc10d717df5b586',
        'service/native_runtime.py': '8ab1715d00752036d833e2eabba375370267b07529031d98fe92a16c74815b0e',
    }
    for name, expected in pinned.items():
        if sha((source/name).read_bytes()) != expected:
            raise ValueError('Unexpected reviewed source: '+name)
    common = Path(__file__).with_name('motion.py').read_text()
    if 'AsyncPolicyMixin' in common:
        raise ValueError('Late-result-only patch superseded; use prepare_async_policy.py with inspected deployed sources')
    original = (source/'policy/production_motion.py').read_text()
    updated = once(original, 'class MotionError(ValueError):\n    pass',
        common[common.index('class MotionError(ValueError):'):common.index('class FeedbackExpired(MotionError):')]+
        'class FeedbackExpired(MotionError):\n    """No command may use this feedback; distinct from a device fault."""\n    pass')
    updated = once(updated, '        self._sample_received = None\n',
        '        self._sample_received = None\n        self._late_policy_frames = 0\n        self._last_late_policy_frame = None\n')
    updated = once(updated, "raise MotionError('舵机反馈已过期')", "raise FeedbackExpired('舵机反馈已过期')")
    anchor = "                if self._filtered is None:\n                    return\n"
    updated = once(updated, anchor, anchor+
        "                if mode == 'walk':\n                    self._require_feedback_locked()\n"
        "                observation_received = self._sample_received\n"
        "                observation_sample = self._last_sample\n")
    start = original.index('                _phase_mark("feedback_gate_begin")\n')
    end = original.index('                    _phase_mark("TARGET_request_begin")\n', start)
    new_start = common.index('            preview_error = None\n', common.index('    def _control_once(self):'))
    new_end = common.index("                self._request(TARGET,", new_start)
    block = ''.join('    '+line if line.strip() else line for line in common[new_start:new_end].splitlines(keepends=True))
    # Keep the existing bounded timing instrumentation around the same gate.
    block = once(block, '                try:\n                    self._require_feedback_locked()\n',
        '                try:\n                    _phase_mark("feedback_gate_begin")\n'
        '                    self._require_feedback_locked()\n'
        '                    _phase_mark("feedback_gate_end")\n')
    updated = once(updated, original[start:end], block)
    updated = once(updated, '                        timing=self._timing_summary_locked(),\n',
        '                        timing=self._timing_summary_locked(),\n'
        '                        policy_deadline=dict(discarded_frames=self._late_policy_frames,\n'
        '                            last_discard=copy.deepcopy(self._last_late_policy_frame),\n'
        '                            observation_limit_ms=FRESH_SECONDS*1000),\n')
    (destination/'policy/production_motion.py').write_text(updated)
    candidate = (source/'policy/candidate_controller.py').read_text()
    candidate = once(candidate, pinned['policy/production_motion.py'], sha(updated.encode()))
    (destination/'policy/candidate_controller.py').write_text(candidate)
    runtime = (source/'service/native_runtime.py').read_text()
    runtime = once(runtime, 'imu_us state_request_sequence timing rates',
                   'imu_us state_request_sequence timing policy_deadline rates')
    (destination/'service/native_runtime.py').write_text(runtime)
    hashes = {}
    for name in pinned:
        raw = (destination/name).read_bytes()
        compile(raw, name, 'exec')
        hashes[name] = sha(raw)
    (destination/'patch-manifest.json').write_text(json.dumps(dict(
        original=pinned, changed=hashes, hardware_access=False,
        model_bytes_changed=False, watchdogs_changed=False,
        source_template_sha256=sha(common.encode())), indent=2)+'\n')
    return hashes


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.destination), indent=2))
