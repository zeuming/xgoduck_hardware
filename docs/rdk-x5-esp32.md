# RDK X5 + ESP32 自定义适配

本适配使用 RDK X5、ESP32-S3 和 HD-1910 舵机，保留总线诊断、校准、策略执行与网页工作台源码。它仍在诊断承重跟随和步态问题，不是原项目已验证的 UNO Q 硬件路线，也没有通过稳定行走验收。

2026-10-14 更新：[当前安装源码快照](../tools/x5_policy/runtime_current/README.md) 保存已核实的主机与配套固件；[发布检查](rdk-x5-publication-checks-20261014.md) 记录本轮检查范围。旧 v8 与早期固件继续保留，不能当作当前安装入口。

## 源码入口

- `tools/servo_workbench`：组装前的舵机扫描、位置和 ID 工具。
- `tools/esp32_linkcheck`：ESP32 链路检查。
- `tools/esp32_motion`：早期运动固件与测试。
- `tools/esp32_motion/diagnostic_baseline`：包含运行增益读回与诊断遥测的后续固件源码基线；左踝 KP8、右膝 KP10、ID33 KP12，其余设置以源码为准。未纳入没有显示改善的左踝 KP10 试验。
- `tools/x5_policy`、`tools/x5_recovery`：策略、主机守护、校准、维护和打包工具。
- `tools/x5_policy/runtime_v8`：后续 v8 运控源码、网页和 CPU 测试快照，包含零速接管、受限前进和单关节跟随诊断。

以上 v8 快照保留当时的源文件布局与行为。后续已按操作者要求修改额外停止条件、目标限幅、心跳归属与官方恢复行为；具体当前行为以 `runtime_current` 为准，不能把历史规则当成当前规则。

## 可重建内容与本地状态

Git 忽略整个 `output/` 和 `tmp/`，以及虚拟环境和 PlatformIO 缓存。固件镜像、模型下载、安装包和日志应按构建/测试流程重新生成。`runtime_v8/source/policy/models/manifest.json` 保存上游模型仓库、固定提交和 SHA256；模型本身不上传，可用快照中的 `fetch_models.py` 从固定提交获取并校验。

设备校准、访问密钥、预期板卡身份和部署启动快照是每台机器的私有输入，不包含在源码快照中。不能把其他机器的零点、安全范围、方向或 IMU 安装校准直接用于实机。测试使用合成校准与假传输，不读取串口或运行模型动作。

## 离线检查

先安装 Python 依赖 `numpy`；网页测试还需要 Node.js。

```sh
python3 tools/x5_policy/runtime_v8/run_tests.py
sh tools/esp32_motion/diagnostic_baseline/tests/run.sh
```

固件构建使用 PlatformIO：

```sh
pio run -d tools/esp32_motion/diagnostic_baseline
```

该构建只生成文件，不烧录、不启动舵机。运行时快照不携带一键实机部署配置，安装需要经过已有打包、设备身份、校准和独占串口检查。离线测试通过不能替代现场硬件验收。
