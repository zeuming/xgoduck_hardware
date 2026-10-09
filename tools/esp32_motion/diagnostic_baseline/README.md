# ESP32 诊断固件源码基线

此源码保存 c27 诊断版本的控制逻辑与测试，不包含固件二进制。保持左踝 KP8、右膝 KP10、ID33 KP12、其余增益及安全保护；未保留左踝 KP10 的未改善试验。平台、框架和编译器版本已固定。

在仓库根目录执行 `sh tools/esp32_motion/diagnostic_baseline/tests/run.sh` 运行纯 CPU 测试，或 `pio run -d tools/esp32_motion/diagnostic_baseline` 编译。编译和测试均不烧录或启动机器人。固件 SHA 会随构建环境、路径或源文件变化，应以每次构建与读回校验为准，不宣称重新构建后二进制必然逐字节相同。
