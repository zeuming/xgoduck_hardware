"""Package explicitly selected weights/checkpoints and reviewed numeric evidence.

No installation, device connection, archive recursion or implicit file globbing.
Original models and saved calibration are included as reference inputs only.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--name', default='xgoduck-864g-candidates-v1')
    p.add_argument('--production', type=Path, required=True, help='Cached host code for no-I/O loader proof')
    a = p.parse_args()
    root = a.root.resolve()
    if not a.name.replace('-', '').isalnum():
        raise ValueError('Simple artifact name required')
    selected = json.loads((root/'selected-models-v1.json').read_text())
    contract = json.loads((root/'current-body-v2/inputs/contract.json').read_text())
    workflow = json.loads((root/'source-workflow-summary-v2.json').read_text())
    codec = json.loads((root/'selected-runtime-codec-audit-v2.json').read_text())
    assert codec['pure_codec_passed']
    assert workflow['complete'] and workflow['all_initial_fingerprints_exact']
    shutdown = json.loads((root/'shutdown-proof-v4.json').read_text())
    assert shutdown['verified_stopped'] and shutdown['all_member_hashes_exact']
    folder = root/'delivery'/a.name
    folder.mkdir(parents=True, exist_ok=False)
    members = []
    def copy(source, relative, expected=None):
        if expected is not None and digest(source) != expected:
            raise ValueError('Changed selected artifact: '+str(source))
        target = folder/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        members.append(dict(path=relative, sha256=digest(target), bytes=target.stat().st_size))
    def write(relative, text):
        target = folder/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
        members.append(dict(path=relative, sha256=digest(target), bytes=target.stat().st_size))
    for mode in ('walk', 'getup', 'pick', 'roulade'):
        info = selected['models'][mode]
        assert info['selected'] and info['simulation_accepted'] and not info['hardware_eligible']
        copy(root/info['model'], f'models/xgoduck_{mode}.onnx', info['sha256'])
        copy(root/info['checkpoint'], f'checkpoints/{mode}.pt', info['checkpoint_sha256'])
        copy(root/'current-body-v2/inputs'/contract['files'][mode]['path'],
             'reference-inputs/'+contract['files'][mode]['path'], contract['files'][mode]['sha256'])
    for key in ('calibration', 'body_reference', 'body_rear-mid', 'body_rear-high'):
        item = contract['files'][key]
        copy(root/'current-body-v2/inputs'/item['path'], 'reference-inputs/'+item['path'], item['sha256'])
    copy(root/'current-body-v2/inputs/contract.json', 'reference-inputs/contract.json', selected['contract_sha256'])
    model_manifest = dict(schema_version=1, candidate=True, hardware_eligible=False,
        models={f'xgoduck_{mode}.onnx': dict(sha256=info['sha256'],
            checkpoint_sha256=info['checkpoint_sha256'], input=[1, 61], output=[1, 14],
            selected_iteration=info['selected_iteration']) for mode, info in selected['models'].items()})
    write('models/manifest.json', json.dumps(model_manifest, indent=2)+'\n')
    # Exercise the actual cached infer.session loader including its manifest
    # hash check. No controller or serial path is imported or constructed.
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    sys.path.insert(0, str(a.production.resolve()/'policy'))
    import infer
    infer.HERE = folder
    trace_matrices = dict(walk='pilot128-walk-matrix-v1', getup='native512-getup-matrix-v1',
                          pick='warmstart1024-pick-matrix-v1', roulade='mixed512-roulade-matrix-v1')
    loader = dict(hardware_access=False, cached_loader_sha256=digest(a.production/'policy/infer.py'),
                  installed_service_tested=False, models={})
    for mode, matrix_name in trace_matrices.items():
        with (root/matrix_name/f'{mode}-reference-seed0-speed0.0/trace.jsonl').open() as stream:
            row = json.loads(next(stream))
        session = infer.session(mode)
        output = infer.infer(session, [row['actor_observation']])
        np.testing.assert_array_equal(output[0], np.array(row['raw_action'], np.float32))
        loader['models'][mode] = dict(manifest_hash_accepted=True, metadata_accepted=True,
            actual_recorded_input_output_exact=True, input_type=session.get_inputs()[0].type,
            input_shape=session.get_inputs()[0].shape, output_shape=session.get_outputs()[0].shape)
    write('evidence/cached-loader-proof.json', json.dumps(loader, indent=2)+'\n')
    evidence = ('selected-models-v1.json', 'walk-holdout-summary-v1.json',
                'getup512-holdout-summary-v1.json', 'pick512-holdout-summary-v1.json',
                'pick1024-holdout-summary-v1.json', 'pick-return-causal-summary-v1.json',
                'roll512-holdout-summary-v1.json', 'roll512-quaternion-audit-v1.json',
                'roll512-button-prefix-audit-v1.json', 'selected-runtime-codec-audit-v2.json',
                'source-workflow-summary-v1.json', 'source-workflow-summary-v2.json',
                'source-workflow-timing40-lag1-summary-v1.json', 'curriculum-parity-audit-v1.json',
                'source-workflow-timing-factor-summary-v1.json',
                'selected-checkpoint-onnx-identity-v2.json')
    for name in evidence:
        copy(root/name, 'evidence/'+name)
    shutdown_summary = dict(verified_stopped=True, observed_at=shutdown['created_at'],
        platform_status=shutdown['platform_stopped_read']['data']['status'],
        last_stopped_at=shutdown['platform_stopped_read']['data']['last_stopped_at'],
        all_member_hashes_exact=True, final_results_files=42, gpu_shortcheck_files=16,
        ssh_master_closed=True, hzm_started=False, source_proof_sha256=digest(root/'shutdown-proof-v4.json'))
    latest_read = json.loads((root/'platform-stopped-read-v5.json').read_text())
    assert latest_read['verified_stopped']
    shutdown_summary['later_stopped_read'] = dict(observed_at=latest_read['observed_at'],
        status=latest_read['platform_read']['data']['status'], source_sha256=digest(root/'platform-stopped-read-v5.json'))
    write('evidence/cloud-shutdown.json', json.dumps(shutdown_summary, indent=2)+'\n')
    workflow_rows = workflow['by_mode']
    timing = json.loads((root/'source-workflow-timing40-lag1-summary-v1.json').read_text())
    timing_table = '\n'.join(f"| {m} | {timing['by_mode'][m]['selected_passed']}/{timing['by_mode'][m]['cases']} | {timing['by_mode'][m]['original_passed']}/{timing['by_mode'][m]['cases']} |" for m in ('getup', 'pick', 'roulade'))
    table = '\n'.join(f"| {m} | {workflow_rows[m]['selected_passed']}/{workflow_rows[m]['cases']} | {workflow_rows[m]['original_passed']}/{workflow_rows[m]['cases']} |" for m in ('getup', 'pick', 'roulade'))
    readme = f'''# XGO Duck 864g 候选模型

这是一套已通过独立仿真的候选权重，**没有安装到机器人，没有完成真机验收**。
整机864g是用户实测值；背部安装保留。重心、惯量和刚性背包包络仍是假设范围，不能把仿真通过解释为真实后倒已经修复。

| 模型 | 新候选独立仿真 | 官方原模型同组 |
| --- | --- | --- |
| 行走 walk128 | 24/24 | 24/24 |
| 起身 getup512 | 12/12 | 9/12 |
| 俯身 pick1024 | 12/12 | 8/12 |
| 翻滚 roulade512 | 12/12 | 10/12 |

合计60/60与51/60，所有对照初始完整物理指纹一致。行走原模型同样通过，因此尚无行走训练收益或真实后倒复现的证据。
独立测试使用3种未实测质量分布、种子0/3/5/7、50Hz、原物理随机化、关闭推扰、修正HOME/reset、清除训练课程；不是完全未改的官方训练回放。
行走验真实整脚3mm连续低载荷抬落脚、左右交替、净前进与恢复；起身12个源跌落准备中11个释放倾角超过55度，另一个约52度；俯身验证4秒phase0与恢复，嘴舵机没有建模；翻滚四元数证明至少270度旋转并恢复。

## 连续动作补验

按官方1.9秒翻滚／4秒俯身后接回零速及Recovery流程；起身比较运行时1秒默认姿态准备。保持同一个物理世界、BAM、电机和延迟状态，不重置根状态。
每种流程3种分布、种子0/3，新旧模型成对：

| 流程 | 新候选 | 原模型 |
| --- | --- | --- |
{table}

该流程是离线原型，保留各原任务的传感器噪声与延迟模型，不是已安装工作台或真实IMU的验收。
缓存工作台的独立按钮在动作结束时STOP/OFF，而官方控制器接回行走／恢复；本包没有擅自修改这个现场行为。
30,000帧冻结输入重放中，新ONNX输出、raw14历史、EMA和整数舵机目标与缓存运行时纯解码一致；这不能证明真实闭环时序与传感器一致。

## 40Hz与额外一帧传感延迟

仅较大后移先验、种子0/3，两种模型集在同一组合条件下成对测试：

| 流程 | 新候选 | 原模型 |
| --- | --- | --- |
{timing_table}

这是单列的组合压力测试，不能据此把失败单独归因于频率或延迟；不能用50Hz通过掩盖这些失败，也不能称已适配真实X5执行链。训练机保持关闭。
分量对照进一步核对rear-high seed3的完整初态逐值一致：PICK候选在40Hz无额外延迟通过，在50Hz加一帧延迟失败；ROLL候选两个单因素均通过，组合失败。说明这里有传感延迟及组合敏感性，尚未识别真实机器人反馈年龄，不能直接宣称实机根因或盲改延迟补偿。

## 与官方训练的差异

- 四个模型均从官方ONNX actor和导出的冻结仿射归一化初始化，新建critic和Adam；不是恢复官方完整checkpoint。ONNX不包含官方学得的探索标准差。
- walk128使用历史小探索、额外KL验收的训练器；历史适配器误删head COM/head bias课程，这一例外保留并披露，不能称完整课程原样。
- roulade512使用历史小探索、额外KL验收的混合域训练；该任务没有上述课程误删。
- getup512使用原生PPO、可学习初始标准差1、adaptive LR、已纠正官方课程；额外观察器关闭。
- pick1024使用同一原生PPO，仅把初始探索标准差设为0.02并保持可学习；不是恢复出的官方值，也不是舵机限值。原生std1候选0/6、早128候选1/6均保留为失败证据。512独立测试12/12，但接回WALK128仅5/6；既有1024在12组独立和6组连续切换都通过，因此选1024，依据是切换闭环证据，不是轮数更多。固定PICK512前4秒轨迹，只换官方WALK也能修复该失败；这只定位仿真切换弱点，不是实机根因。
- 原奖励公式、网络、模型HOME、动作尺度、EMA0.45／翻滚0.15、raw14历史、编码器映射、BAM和CAD限位保留。范围外目标才投影到min+1/max−1，不改保存范围；无600限速、固定头输出或15度训练终止。
- 训练模型总重864g，原质量随机化保留；reference/mid/high是假设域，不是已测定的板卡质量和重心。

## 文件与使用边界

`models/`为四个新ONNX，float32[1,61]→[1,14]，HOME／关节顺序／动作尺度保留。
`checkpoints/`是对应完整选中快照，actor和归一化各10个张量与ONNX逐位一致。四份快照均含课程clock，mjlab继承实现会保存／恢复它；此前仅搜索训练入口而称旧快照缺clock的判断已撤回。新getup/pick另有native LR和next-iteration绑定，只支持明确源码／配置恢复；不能混用旧入口，任何一个都不是物理轨迹精确恢复。
`reference-inputs/`保存原ONNX及本轮数值输入用于复核，**不要覆盖机器当前校准**。
`evidence/`保存验收摘要与差异，`manifest.json`逐文件SHA256。所有原始轨迹、失败、训练日志和归档源码仍在工作目录。
官方runtime基准8cdbbd84710d856581982c9eaf0d5e2970666232；训练源码326d77a1122870bdefa2c36403937502c958e69c，具体原ONNX训练修订未知。

尚未完成真实COM／碰撞包络、IMU朝向与滤波／反馈新鲜度、实际执行周期、机械零位／松动、共享电池引发X5重启原因、嘴舵机的闭环验证。
本包不含安装器，不会连接串口、修改保护或自动启动动作。安装需先核对当前校准与运行时配套；本次没有生产部署授权。
5090训练结果已逐文件取回，平台已核实stopped，hzm本任务未使用。
'''
    write('README_CN.md', readme)
    manifest = dict(schema_version=1, created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        selected_models=selected['models'], files=members, excludes_manifest_self=True,
        nominal_mass_g=864, body_identified=False, hardware_verified=False,
        production_installed=False, installation_authorized=False,
        independent_simulation_passed=60, independent_simulation_cases=60,
        independent_original_passed=51, workflow_summary=workflow,
        archived_reproduction_sources={name: dict(path=str(root/name), sha256=digest(root/name))
            for name in ('gpu-bundle-v3.tar.gz', 'gpu-bundle-v4.tar.gz', 'gpu-bundle-v7.tar.gz', 'gpu-bundle-v8.tar.gz')})
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    archive = folder.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name in sorted([x['path'] for x in members]+['manifest.json']):
            z.write(folder/name, name)
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        assert set(z.namelist()) == {x['path'] for x in members}|{'manifest.json'}
        for row in members:
            assert hashlib.sha256(z.read(row['path'])).hexdigest() == row['sha256']
    proof = dict(path=str(archive), sha256=digest(archive), bytes=archive.stat().st_size,
        member_files=len(members)+1, all_member_hashes_exact=True,
        hardware_access=False, production_installed=False)
    archive.with_suffix('.zip.sha256').write_text(proof['sha256']+'  '+archive.name+'\n')
    version = a.name.rsplit('-', 1)[-1]
    (root/f'candidate-delivery-proof-{version}.json').write_text(json.dumps(proof, indent=2)+'\n')
    print(json.dumps(proof))


if __name__ == '__main__':
    main()
