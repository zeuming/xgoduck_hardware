# ESP32-S3 链路诊断

用于 X5 经 ESP32-S3 的 COM 口读取 LSM6DSR 身份、扫描 HD1910 舵机 ID、读取位置和扭矩状态。`check_links.py` 和固件的 INFO、IMU、SCAN、READ 诊断命令不会改变舵机位置、ID、扭矩或校准设置。0.2 固件另有供舵机工作台使用的 XFER 命令，允许受限的设置写入；只有工作台操作按钮触发相应写入，固件启动不会自动发送舵机指令。

接线和命令边界见 [firmware/README.md](firmware/README.md)。X5 到 ESP32 使用 115200 bps；ESP32 到 URT-2 使用 GPIO17/TX、GPIO18/RX，1 Mbps；IMU 使用 GPIO8/SDA、GPIO9/SCL。

## 编译和检查

```sh
pio run -d tools/esp32_linkcheck
python3 tools/esp32_linkcheck/check_links.py --port /dev/ttyUSB0 --output linkcheck.json
```

客户端自动读取所有扫描到的 ID。也可用 `--skip-scan --read 10 11` 只检查指定 ID。需要安装 pyserial，诊断时串口只能由一个程序使用。

客户端退出码 0 表示命令采集完成。判断链路是否通过还要检查 IMU 的 `found`、扫描的 ID 集合，以及每个 `READ` 的 `ok`。故障应答、超时、缺失数据保留在 JSON 中。IMU 身份读取不代表连续采样已经启用。

## 原固件备份

本次设备已确认是 ESP32-S3、16MB 闪存，原程序为 xiaozhi 1.9.2。完整备份读取时已通过 esptool 的板端 MD5 校验，复制到笔记本后又核对大小与 SHA-256。原程序在默认重启后更新了 NVS，因此额外保留停止运行时的稳定快照，全部 16MB 再与板端 MD5 对照通过。备份位于 `output/esp32_linkcheck/backups/`，该目录已被 Git 忽略。

完整备份可能包含原程序的联网凭据，应保留在本地。恢复需要使用原来的完整备份文件，保留原镜像头参数；恢复会替换当前诊断程序，不能与串口诊断同时执行：

```sh
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 --baud 460800 write_flash --flash_mode keep --flash_size keep --flash_freq keep 0 esp32s3-original-stable-20261003.bin
```

固件文件、实际烧录地址和 SHA-256 记录在 `output/esp32_linkcheck/firmware/manifest.json`。

支持网页工作台的 0.2 固件另存于 `output/esp32_workbench/firmware/`。笔记本调试方式见 [舵机工作台 README](../servo_workbench/README.md)。启动工作台后，它会独占 ESP32 的 COM 串口；不要同时运行 `check_links.py`。

## 本次实测

2026-10-03 已通过通信检查：IMU 在 0x6B 返回 WHO_AM_I=0x6B；舵机 ID 10–14、20–24、30–34 全部正确返回 ID、位置和扭矩状态，故障码均为 0，UART 和协议解析错误计数均为 0。15 个扭矩读回值均为 0。扫描其余 239 个未使用 ID 的超时属于没有应答，不代表已发现舵机有故障。

完整数据见 `output/esp32_linkcheck/linkcheck-first.json`，摘要见 `result-summary.json`，烧录校验日志见 `flash.log`。X5 的系统时钟尚未设置，原始报告保留其 2000 年时间；摘要另用笔记本记录采集时间。此次检查未发送动作或设置写入，IMU 连续采样尚未配置。
