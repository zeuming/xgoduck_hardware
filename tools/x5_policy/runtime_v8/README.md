# v8 运控源码与离线测试

从后续硬件排查版本提取的源码快照，避免依赖被忽略的 `output/` 工作副本。`source/policy`、`source/service` 和 `source/web` 保留原有源码布局。设备私有 JSON、依赖 wheel、模型二进制及测试日志未纳入。

在仓库根目录运行 `python3 tools/x5_policy/runtime_v8/run_tests.py`。需要 NumPy 与 Node.js；仅使用合成校准、假模型和假串口。详细范围、固件基线和限制见 [适配说明](../../../docs/rdk-x5-esp32.md)。

获取固定版本的模型：`python3 tools/x5_policy/runtime_v8/fetch_models.py --destination output/models-v8`。下载和 SHA256 校验不会连接机器人或部署模型。
