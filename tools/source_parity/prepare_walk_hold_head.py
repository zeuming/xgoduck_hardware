"""Local HOLD-head/WALK comparison. Never deploys or opens hardware."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil


def prepare(baseline, destination):
    if destination.exists():
        raise FileExistsError('Preserve previous candidates; choose a new directory')
    shutil.copytree(baseline, destination,
                    ignore=shutil.ignore_patterns('__pycache__', '.pio'))
    path = destination / 'policy/motion.py'
    before = path.read_text()
    source = before

    def replace(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError(f'Unexpected baseline: {old[:80]}')
        source = source.replace(old, new)

    replace('"""Native transport adapter; official ONNX weights and action decoding unchanged.',
            '''"""Local uninstalled WALK-HOLD-HEAD-20261014 comparison.

The official actor and its decode remain unchanged; execution of its four
head outputs is replaced by the existing compensated HOLD head goals during
normal WALK, both at zero and nonzero velocity. This is a local adaptation,
not original-source behavior or proof of physical stability. The previous
implementation described below passed a head command only.

Native transport adapter; official ONNX weights and action decoding unchanged.''')
    replace('        self._head_projection = []',
            '        self._head_projection = []\n        self._walk_head_override = []')
    replace('''        # 仅作为官方 build_obs 的输入；WALK 不强制覆盖模型的头部输出。
        # 与历史“固定头部输出”的实现不同，不能把历史成功当作本版验收。''',
            '''        # 保留官方 build_obs 的头颈指令输入。此本地对照版本还在
        # _execution_target_locked 中让正常 WALK 执行相同 HOLD 头颈目标。
        # 模型原始输出/历史继续独立记录；尚未证明实机平衡或行走通过。''')
    replace("        raise MotionError('官方运行模式不固定头颈；不需要手动记录头颈位置')",
            "        raise MotionError('头颈目标来自已配置的站姿；此接口不重新采集头颈位置')")
    replace('''        result = list(policy_target)
        projections = []''', '''        result = list(policy_target)
        # WALK-HOLD-HEAD-20261014, compared with upstream 8cdbbd controller/
        # rl_core: upstream head_command is a policy input, not a guaranteed
        # joint target. The operator reports compensated HOLD stands, and asks
        # to retain that same compensation in zero and moving WALK.
        # Reuse the existing HOLD goals; do NOT add the offset a second time
        # to actor output. Only the four head execution goals are replaced.
        # `mode` is the selected model, so getup/default-recovery/pick/roulade
        # are unaffected. Original actor history, EMA, measured observations,
        # leg/mouth goals and the following saved-range clamp remain intact.
        self._walk_head_override = []
        if mode == 'walk' and any(self._walk_head_profile):
            held = self._fixed_stance_locked()
            for sid in (30, 31, 32, 33):
                index = MOTOR_IDS.index(sid)
                self._walk_head_override.append(dict(id=sid,
                    policy_deg=float(result[index]), hold_deg=float(held[index])))
                result[index] = held[index]
        projections = []''')
    replace("            target_range_projection=copy.deepcopy(self._target_projection),",
            "            head_pose_override=copy.deepcopy(self._walk_head_override),\n            target_range_projection=copy.deepcopy(self._target_projection),")
    replace("        result['walk_head_command_rad'] = list(self._walk_head_profile)",
            """        result['walk_head_command_rad'] = list(self._walk_head_profile)
        result['walk_head_execution'] = dict(
            mode='hold-pose' if any(self._walk_head_profile) else 'actor',
            last=copy.deepcopy(self._walk_head_override),
            scope='normal walk only; existing saved-range clamp still applies',
            physical_balance_verified=False)""")
    replace('            head_fixed=False, takeover_seconds=0, stance_transition_seconds=0,',
            '            head_fixed=bool(any(self._walk_head_profile)), takeover_seconds=0, stance_transition_seconds=0,')
    compile(source, str(path), 'exec')
    path.write_text(source)
    (destination.parent / 'review.patch').write_text(''.join(difflib.unified_diff(
        before.splitlines(True), source.splitlines(True),
        fromfile='baseline/policy/motion.py', tofile='candidate/policy/motion.py')))
    report = dict(installed=False, hardware_access=False,
                  baseline=str(baseline), changed_files=['policy/motion.py'],
                  baseline_sha256=hashlib.sha256(before.encode()).hexdigest(),
                  candidate_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  physical_balance_verified=False)
    for old in baseline.rglob('*'):
        if not old.is_file() or '__pycache__' in old.parts or '.pio' in old.parts:
            continue
        relative = old.relative_to(baseline)
        if relative.as_posix() != 'policy/motion.py':
            assert old.read_bytes() == (destination / relative).read_bytes(), relative
    (destination.parent / 'preparation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.baseline, args.output)
