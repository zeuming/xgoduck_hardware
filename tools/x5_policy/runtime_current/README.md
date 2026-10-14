# 2026-10-14 已安装运控源码快照

本目录保存该日期已核实安装的 RDK X5 主机代码及配套 ESP32 源码，供审阅、比较和离线检查。它不是稳定行走发布版，不会自动连接机器人。`source-manifest.json` 记录每份源码的 SHA256、安装标识、固件应用摘要和上游运行时提交。

## 目录和边界

- `source/policy`：实际部署的动作解码、观测处理、目标限幅和串口控制。
- `source/service`、`source/web`：命令、后台进程、手柄、校准和异步状态页面。
- `source/firmware`：配套 ESP32 固件及历史假硬件测试。
- `source/policy/models/manifest.json`：四个原始 ONNX 的固定上游版本与哈希。

源码字节保持安装快照原样，包括历史路径和诊断入口。设备校准、串口身份、访问密钥、设备部署配置、模型二进制和运行日志均未收录。因此这不是可直接搬到另一台机器运行的一键安装包；不能用别人的机械零点、方向、范围或 IMU 标定代替本机标定。

模型可单独下载并校验（不部署、不操作机器人）：

```sh
python3 tools/x5_policy/runtime_current/fetch_models.py \
  --destination tools/x5_policy/runtime_current/source/policy/models
```

## 当前行为

四个官方模型保持原样。使用同步舵机反馈、官方运行增益及恢复流程；没有历史600刻度/秒软限速。目标按操作者要求限制到保存范围，超界取下限+1或上限−1；启动和运行不会仅因实测位置超出手工保存范围而全身卸力。显式停止与通信看门狗仍保留。

显式 HOLD 保留历史选定的头颈姿态指令。正常 WALK 使用原模型头颈输出；此前覆盖成 HOLD 头颈目标的实验已经撤回。具体本机差异见 `source/policy/motion.py` 注释；安装实例的头颈指令属于设备配置，没有在此生成通用配置。

## 离线检查

Python 检查需要 `numpy`。以下测试使用合成校准、假时钟及假串口，不连接实机：

```sh
python3 tools/source_parity/test_stop_policy_repair.py \
  --candidate tools/x5_policy/runtime_current/source \
  --output /tmp/duck-stop-policy.json
python3 tools/source_parity/test_closure_source.py \
  --candidate tools/x5_policy/runtime_current/source \
  --output /tmp/duck-closure.json
python3 tools/source_parity/test_clamped_policy_evidence.py \
  --candidate tools/x5_policy/runtime_current/source \
  --output /tmp/duck-policy-evidence.json
```

`tools/source_parity` 中有些测试刻意绑定旧实验，不能将全部历史用例混跑为当前验收。离线回归、仿真、固定支撑测试、真实零速平衡和落地行走是不同证据；本次提交不宣称实机行走已解决，也没有再次部署或启动动作。
