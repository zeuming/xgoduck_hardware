#!/usr/bin/env python3
"""Build a self-contained Linux HELLO/STATE feedback reader, without running it.

The reader never opens SSH, creates a board file, sends a motor-control command,
or retries an open or failed request. All calibration belongs on the Mac. The
old storage and motion guards are deliberately untouched; this reader grants no
motion capability and retains the reviewed historical faults as unresolved.

``remote_source(expected)`` returns Python source for an explicitly launched
Linux process. Required expected fields are boot_id, cid, root_mount_line,
model, serial, kernel_prefix, historical_faults, and generation. serial has
path=/dev/ttyUSB0, vid, pid, sysfs_name=1-1.3 and serial_number. generation is a
nonzero 31-bit integer. There is no absolute lifetime: the Mac ends the one held
read-only connection by sending the fixed stdin message END_FEEDBACK followed
by a newline. EOF, malformed control input, lost health or feedback still fail
closed. duration_seconds is obsolete and explicitly rejected.
stdout JSONL kinds: ready (identity/full kernel proof), state (decoded state),
fault (reason), closed (serial closure proof). Every record has generation.
After a CRC/header-validated STATE reply arrives, fault/closed
query.last_feedback retains its bounded scalar status before decoding/safety
checks. It proves neither full packet validity nor calibration acceptance;
rejected feedback never emits as state. Each STATE request clears the previous
diagnostic, so a timeout cannot borrow an earlier frame's status.
"""

from __future__ import annotations

import ast
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import uuid


EVIDENCE_DIR = (Path(__file__).resolve().parents[2] / "output/x5_recovery/"
                "replacement-board-20261004T011959+0800")
HELPER_NAMES = ("ProbeError", "_check_deadline", "_crc16", "_frame", "_holders",
                "_modem_bits", "_raw_settings", "_ready", "_read_exact",
                "_write_once", "_exchange", "_decode_state")
MAX_KERNEL_BYTES = 2 * 1024 * 1024
CONTROL_END = b"END_FEEDBACK\n"
REVIEWED_BOOT_ID = "dcf6255f-2afb-4262-8c96-4a00bc0d8cab"
REVIEWED_CID = "035344534b33324785d55cb687017c00"
REVIEWED_HISTORICAL_FAULTS = [
    {"line": "[    7.573470] systemd-journald[273]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.",
     "reason": "Journald reported a corrupt or uncleanly closed journal file"},
    {"line": "[   19.944418] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.",
     "reason": "Kernel reported an unclean FAT volume or filesystem fault"},
    {"line": "[ 3008.252350] usb 1-1.3: USB disconnect, device number 3",
     "reason": "Kernel reported a USB disconnect/reset/error"},
]
RECONNECT_PROFILE_SHA256 = "92f372030bf2b6c034f0b4a85c88c7efafb9f108c5158a534de03767797580da"
RECONNECT_KERNEL_SHA256 = "e75b1b38daef8e19dfcbf81af282b40e309e8fc02cd1c23342ef759f3ace04de"
RECONNECT_KERNEL_LINES = 962
RECONNECT_SCOPE = "feedback_only_reviewed_reconnect"
RECONNECT_ROOT = "21 1 179:2 / / rw,relatime shared:1 - ext4 /dev/root rw"
RECONNECT_MODEL = "D-Robotics RDK X5 V1.0"
RECONNECT_SERIAL = {"path": "/dev/ttyUSB0", "vid": "1a86", "pid": "7523",
                    "sysfs_name": "1-1.3", "serial_number": ""}
RECONNECT_HISTORICAL_FAULTS = REVIEWED_HISTORICAL_FAULTS + [
    {"line": "[ 8518.691466] usb 1-1: USB disconnect, device number 2",
     "reason": "Kernel reported a USB disconnect/reset/error"},
    {"line": "[ 8518.691521] usb 1-1.3: USB disconnect, device number 4",
     "reason": "Kernel reported a USB disconnect/reset/error"},
    {"line": "[ 8518.931957] usb 2-1: USB disconnect, device number 2",
     "reason": "Kernel reported a USB disconnect/reset/error"},
    {"line": "[ 8544.480306] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail",
     "reason": "Kernel reported an SD/MMC failure"},
]
RECONNECT_PROOF = {"scope": RECONNECT_SCOPE, "profile_sha256": RECONNECT_PROFILE_SHA256,
                   "kernel_prefix_sha256": RECONNECT_KERNEL_SHA256,
                   "kernel_line_count": RECONNECT_KERNEL_LINES}
RECONNECT2_PROFILE_SHA256 = "71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd"
RECONNECT2_KERNEL_SHA256 = "1754adca02d6739e910cae4e05e9a0b995a71fc54e802e464ce541bcaef72f48"
RECONNECT2_HISTORICAL_FAULTS = RECONNECT_HISTORICAL_FAULTS + [
    {"line": "[ 9890.432382] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff",
     "reason": "Feedback guard reported an AIC SDIO WiFi driver failure"},
]
RECONNECT2_PROOF = {"scope": RECONNECT_SCOPE, "profile_sha256": RECONNECT2_PROFILE_SHA256,
                    "kernel_prefix_sha256": RECONNECT2_KERNEL_SHA256, "kernel_line_count": 963}
REBOOT_PROFILE_SHA256 = "dd634b2e8dfe4432a70024254c6533f2f57df56043b0b9edf92793bcfa9fb529"
REBOOT_BOOT_ID = "8583b8f1-2d9d-4605-b34f-facbcf18f328"
REBOOT_KERNEL_SHA256 = "69769ba003f945aa7fc3e730b840cfb63acf312d75c2d464b45a956157e81372"
REBOOT_KERNEL_LINES = 922
REBOOT_HISTORICAL_FAULTS = [
    {"line": "[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.",
     "reason": "Journald reported a corrupt or uncleanly closed journal file"},
    {"line": "[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.",
     "reason": "Kernel reported an unclean FAT volume or filesystem fault"},
]
REBOOT_PROOF = {"scope": RECONNECT_SCOPE, "profile_sha256": REBOOT_PROFILE_SHA256,
                "kernel_prefix_sha256": REBOOT_KERNEL_SHA256, "kernel_line_count": REBOOT_KERNEL_LINES}
RETURNED_ESP32_PROFILE_SHA256 = "a6c3ce1cefbcf87f729accd48b549b01168d7addb5ba8f0cb2d4ce31478f309e"
RETURNED_ESP32_KERNEL_SHA256 = "2f71ae9a6ebdae673996e23a3b2eb29614639b3afb0569ddfc4af68cda12ae4d"
RETURNED_ESP32_KERNEL_LINES = 935
RETURNED_ESP32_HISTORICAL_FAULTS = REBOOT_HISTORICAL_FAULTS + [
    {"line": "[ 5127.470586] usb 1-1.3: USB disconnect, device number 3",
     "reason": "Kernel reported a USB disconnect/reset/error"},
]
RETURNED_ESP32_PROOF = {"scope": RECONNECT_SCOPE, "profile_sha256": RETURNED_ESP32_PROFILE_SHA256,
                        "kernel_prefix_sha256": RETURNED_ESP32_KERNEL_SHA256,
                        "kernel_line_count": RETURNED_ESP32_KERNEL_LINES}
MAINTENANCE_ESP32_PROFILE_SHA256 = 'ffa15f5e97683645565a364d754a30b170acac8012fdec1ffc0c6995d467f127'
MAINTENANCE_ESP32_KERNEL_SHA256 = 'fd6c0ba91c5b5ea92337ef83771d6e3ae4c556f260b072f1d4e25dab40aaeaeb'
MAINTENANCE_ESP32_KERNEL_LINES = 944
MAINTENANCE_ESP32_HISTORICAL_FAULTS = RETURNED_ESP32_HISTORICAL_FAULTS + [{'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
MAINTENANCE_ESP32_PROOF = {"scope": RECONNECT_SCOPE, "profile_sha256": MAINTENANCE_ESP32_PROFILE_SHA256,
                           "kernel_prefix_sha256": MAINTENANCE_ESP32_KERNEL_SHA256,
                           "kernel_line_count": MAINTENANCE_ESP32_KERNEL_LINES}
WRITE_ACK_ESP32_PROFILE_SHA256 = '7b69c8ee69665812eb2ccc76b0e202ab66df7039acdad0b80751ab5797d648b4'
WRITE_ACK_ESP32_KERNEL_SHA256 = 'f0b1550b4a9e4053b176ce07244eae2c9882dec3be318653692a76b6881ed675'
WRITE_ACK_ESP32_KERNEL_LINES = 953
WRITE_ACK_ESP32_HISTORICAL_FAULTS = MAINTENANCE_ESP32_HISTORICAL_FAULTS + [{'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
WRITE_ACK_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': WRITE_ACK_ESP32_PROFILE_SHA256,
                         'kernel_prefix_sha256': WRITE_ACK_ESP32_KERNEL_SHA256, 'kernel_line_count': 953}
CENTER_TRACE_ESP32_PROFILE_SHA256 = '069f4fc66898a5f2767c1881a5e6e2fc86434a31264a1eb43e5b41ee6f0d11d3'
CENTER_TRACE_ESP32_KERNEL_SHA256 = 'e0fe3dc00aa4481850bfd4d340697f2e2e89169b52e73a9f8174f5f1333c7781'
CENTER_TRACE_ESP32_KERNEL_LINES = 961
CENTER_TRACE_ESP32_HISTORICAL_FAULTS = WRITE_ACK_ESP32_HISTORICAL_FAULTS + [{'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
CENTER_TRACE_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': CENTER_TRACE_ESP32_PROFILE_SHA256,
                            'kernel_prefix_sha256': CENTER_TRACE_ESP32_KERNEL_SHA256, 'kernel_line_count': 961}
CENTER_HOLDING_ESP32_PROFILE_SHA256 = '3071e1cfab266ef8764eab9e856bc086a379425933c578ed884523b74d2da6b7'
CENTER_HOLDING_ESP32_KERNEL_SHA256 = '5912ce0ee02430818193adf16bff3ba76729aeba8689966180adebdc9c7e20b5'
CENTER_HOLDING_ESP32_KERNEL_LINES = 964
CENTER_HOLDING_ESP32_HISTORICAL_FAULTS = CENTER_TRACE_ESP32_HISTORICAL_FAULTS + [{'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
CENTER_HOLDING_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': CENTER_HOLDING_ESP32_PROFILE_SHA256, 'kernel_prefix_sha256': CENTER_HOLDING_ESP32_KERNEL_SHA256, 'kernel_line_count': 964}
ARM_FRESH_ESP32_PROFILE_SHA256 = '1ff3566694a6f2d50da3211e307cdb4e9ac9cc8e6f6b55e782636156686d232c'
ARM_FRESH_ESP32_KERNEL_SHA256 = 'a90b070111c45c7eb24ad355c24d92e1e0ea9e0c766e9b976910ae523b635dbd'
ARM_FRESH_ESP32_KERNEL_LINES = 963
ARM_FRESH_ESP32_HISTORICAL_FAULTS = CENTER_HOLDING_ESP32_HISTORICAL_FAULTS + [{'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
ARM_FRESH_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': ARM_FRESH_ESP32_PROFILE_SHA256, 'kernel_prefix_sha256': ARM_FRESH_ESP32_KERNEL_SHA256, 'kernel_line_count': ARM_FRESH_ESP32_KERNEL_LINES}
ARM_WRITE_REPLY_ESP32_PROFILE_SHA256 = '69d3c8d2f98de3a8e8a3243cae2601db1d77f336891b37fb57d7b10a733af07f'
ARM_WRITE_REPLY_ESP32_KERNEL_SHA256 = 'fdcfebeb2cbadba85645ed1ecde16c7fe3f9eed82bda6cad5f11d088b36ea0c4'
ARM_WRITE_REPLY_ESP32_KERNEL_LINES = 962
ARM_WRITE_REPLY_ESP32_HISTORICAL_FAULTS = ARM_FRESH_ESP32_HISTORICAL_FAULTS + [{'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
ARM_WRITE_REPLY_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, 'kernel_prefix_sha256': ARM_WRITE_REPLY_ESP32_KERNEL_SHA256, 'kernel_line_count': ARM_WRITE_REPLY_ESP32_KERNEL_LINES}
USB_PORT_MOVE_ESP32_PROFILE_SHA256 = '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c'
USB_PORT_MOVE_ESP32_KERNEL_SHA256 = 'b3bcd8f9d90c866295aace63831c8583516095c4d39fb18a42c34445629e53cf'
USB_PORT_MOVE_ESP32_KERNEL_LINES = 967
USB_PORT_MOVE_ESP32_HISTORICAL_FAULTS = ARM_WRITE_REPLY_ESP32_HISTORICAL_FAULTS + [{'line': '[82677.647322] usb 1-1.3: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82987.887775] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83012.205481] usb 1-1.4: USB disconnect, device number 12', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83245.141873] usb 1-1.4: USB disconnect, device number 13', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
USB_PORT_MOVE_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T04:02:02.275986+08:00', 'human_context': '用户确认将 ESP32 COM 线换到 X5 另一 USB 口，并明确要求修复。用户确认官方电源和闪联线已接、X5 已开机；按请求重新插好闪联线后 SSH 恢复。两次只读元数据确认同一板、卡、启动以及唯一 CH340，现位于 USB1-1.4。逐字比较保留旧931行，环形缓冲仅逐出31条非故障启动信息。新增36行仅 CH340 拔插与重枚举，新增4条精确 USB 断开事件。后三次断开原因尚未确认，作为未解决历史保留，不认定供电或接线已修复。此基线只允许用户重新发起连接的实时核对，不自动使能、不清除历史故障或安全锁。', 'prior_profile_sha256': '69d3c8d2f98de3a8e8a3243cae2601db1d77f336891b37fb57d7b10a733af07f', 'diagnostic_result_sha256': '56ec645b1afb7f36eb04d3f734f9b756578558a705f560ae154b7c7c2385929d', 'connection_sha256': '7871056f036d2a18085d75447cb1ddb82f29daa2e01cf9f8a7db75ce96ba6d4e', 'reviewed_usb_events': [{'line': '[82677.647322] usb 1-1.3: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82987.887775] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83012.205481] usb 1-1.4: USB disconnect, device number 12', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83245.141873] usb 1-1.4: USB disconnect, device number 13', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'evicted_nonfault_lines': ['[    0.000000] CPU features: kernel page table isolation disabled by kernel configuration', '[    0.000000] CPU features: detected: Qualcomm erratum 1009, or ARM erratum 1286807, 2441009', '[    0.000000] CPU features: detected: ARM errata 1165522, 1319367, or 1530923', '[    0.000000] alternatives: applying boot alternatives', '[    0.000000] Built 1 zonelists, mobility grouping on.  Total pages: 2048256', '[    0.000000] Kernel command line: console=tty1 console=ttyS0,115200 rootfstype=ext4 rw rootwait root=/dev/mmcblk1p2 mtdparts=spi7.0:0x700000@0x0(miniboot),0x180000@0x700000(ubootenv) hobotboot.reason=REBOOT_CMD', '[    0.000000] Dentry cache hash table entries: 1048576 (order: 11, 8388608 bytes, linear)', '[    0.000000] Inode-cache hash table entries: 524288 (order: 10, 4194304 bytes, linear)', '[    0.000000] mem auto-init: stack:off, heap alloc:off, heap free:off', '[    0.000000] software IO TLB: area num 8.', '[    0.000000] software IO TLB: mapped [mem 0x00000000fbfff000-0x00000000fffff000] (64MB)', '[    0.000000] Memory: 6856580K/8323072K available (14144K kernel code, 1042K rwdata, 4836K rodata, 2560K init, 3532K bss, 1075324K reserved, 391168K cma-reserved)', '[    0.000000] SLUB: HWalign=64, Order=0-3, MinObjects=0, CPUs=8, Nodes=1', '[    0.000000] rcu: Preemptible hierarchical RCU implementation.', '[    0.000000] rcu: \tRCU event tracing is enabled.', '[    0.000000] \tTrampoline variant of Tasks RCU enabled.', '[    0.000000] \tTracing variant of Tasks RCU enabled.', '[    0.000000] rcu: RCU calculated value of scheduler-enlistment delay is 10 jiffies.', '[    0.000000] NR_IRQS: 64, nr_irqs: 64, preallocated irqs: 0', '[    0.000000] GICv3: GIC: Using split EOI/Deactivate mode', '[    0.000000] GICv3: 256 SPIs implemented', '[    0.000000] GICv3: 0 Extended SPIs implemented', '[    0.000000] Root IRQ handler: gic_handle_irq', '[    0.000000] GICv3: GICv3 features: 16 PPIs', '[    0.000000] GICv3: CPU0: found redistributor 0 region 0:0x0000000030140000', '[    0.000000] rcu: srcu_init: Setting srcu_struct sizes based on contention.', '[    0.000000] arch_timer: cp15 timer(s) running at 24.00MHz (phys).', '[    0.000000] clocksource: arch_sys_counter: mask: 0xffffffffffffff max_cycles: 0x588fe9dc0, max_idle_ns: 440795202592 ns', '[    0.000000] sched_clock: 56 bits at 24MHz, resolution 41ns, wraps every 4398046511097ns', '[    0.000817] Console: colour dummy device 80x25', '[    0.001266] printk: console [tty1] enabled'], 'retained_prior_line_count': 931, 'previous_serial_reference': {'path': '/dev/ttyUSB0', 'vid': '1a86', 'pid': '7523', 'sysfs_name': '1-1.3', 'serial_number': ''}, 'observed_serial': {'path': '/dev/ttyUSB0', 'sysfs_name': '1-1.4', 'vid': '1a86', 'pid': '7523', 'serial_number': ''}, 'serial_discovery_required': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False, 'unexplained_disconnect_root_cause_confirmed': False}
USB_PORT_MOVE_ESP32_PROOF = {'scope': RECONNECT_SCOPE, 'profile_sha256': USB_PORT_MOVE_ESP32_PROFILE_SHA256, 'kernel_prefix_sha256': USB_PORT_MOVE_ESP32_KERNEL_SHA256, 'kernel_line_count': USB_PORT_MOVE_ESP32_KERNEL_LINES}
COLD_RECONNECT_BOOT_ID = 'acbc7d13-7515-4419-a55d-90cf267d74b0'
COLD_RECONNECT_PROFILE_SHA256 = 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0'
COLD_RECONNECT_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T07:43:24.369656+08:00', 'human_context': '用户称重后看到 USB 闪联路由错误，明确要求恢复使用，并确认 URT2 电池已接通。被动元数据和完整启动日志证明 X5 已冷启动；SSH 主机密钥、板卡型号、SD CID 和根挂载均与上一份核对记录一致，唯一 CH340 仍位于用户更换后的 USB1-1.4。此次启动日志保留最早启动行，共922行；完整分类仅有两条未正常卸载的文件系统警告，继续作为未解决故障保留。旧启动中的 USB 和 WiFi 故障通过旧基线摘要和完整故障列表保留，不认定已修复。本记录只允许一次显式只读复查；实际舵机状态仍须重新读取，不自动进入站姿或启动模型动作。', 'scope_detail': 'feedback-only exact same board/card cold-boot review; no motion or storage write allowance', 'diagnostic_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/result.json', 'diagnostic_result_sha256': '2056a55521d5c5382b071ce28fc12a4ea18e0641a0e8ad3dc4ea43afc27a8a51', 'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/connection.json', 'connection_sha256': '7ec305787122202ca623e9e238c5e211b55995a241b3b21f14adefd430531845', 'diagnostic_uart_opened': False, 'diagnostic_block_device_opened': False, 'diagnostic_board_files_written': False, 'prior_boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json', 'prior_reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c', 'prior_boot_historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82677.647322] usb 1-1.3: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82987.887775] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83012.205481] usb 1-1.4: USB disconnect, device number 12', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83245.141873] usb 1-1.4: USB disconnect, device number 13', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'prior_boot_faults_cleared': False, 'filesystem_repaired': False, 'board_card_identity_unchanged': True, 'early_kernel_history_complete': True, 'observed_boot_reason': 'COLD_BOOT', 'unexpected_reboot_root_cause_confirmed': False, 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg', 'observed_serial': {'path': '/dev/ttyUSB0', 'sysfs_name': '1-1.4', 'vid': '1a86', 'pid': '7523', 'serial_number': ''}, 'serial_discovery_required': True, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}
COLD_RECONNECT_HISTORICAL_FAULTS = [{'line': '[    7.379408] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.469697] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]
COLD_RECONNECT_PROOF = {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0', 'kernel_prefix_sha256': '4c55e0485f393e7487ce34c3b8867f10165516d89ff90e436649ee2651d14423', 'kernel_line_count': 922}
COLD_USB_DROP_PROFILE_SHA256 = 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9'
COLD_USB_DROP_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T18:09:55.119122+08:00', 'purpose': 'Explicit read-only recovery after a completed same-port ESP32 USB disconnect; no movement or storage writes', 'prior_profile_sha256': 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0', 'prior_review': {'reviewer': 'root', 'reviewed_at': '2026-10-05T07:43:24.369656+08:00', 'human_context': '用户称重后看到 USB 闪联路由错误，明确要求恢复使用，并确认 URT2 电池已接通。被动元数据和完整启动日志证明 X5 已冷启动；SSH 主机密钥、板卡型号、SD CID 和根挂载均与上一份核对记录一致，唯一 CH340 仍位于用户更换后的 USB1-1.4。此次启动日志保留最早启动行，共922行；完整分类仅有两条未正常卸载的文件系统警告，继续作为未解决故障保留。旧启动中的 USB 和 WiFi 故障通过旧基线摘要和完整故障列表保留，不认定已修复。本记录只允许一次显式只读复查；实际舵机状态仍须重新读取，不自动进入站姿或启动模型动作。', 'scope_detail': 'feedback-only exact same board/card cold-boot review; no motion or storage write allowance', 'diagnostic_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/result.json', 'diagnostic_result_sha256': '2056a55521d5c5382b071ce28fc12a4ea18e0641a0e8ad3dc4ea43afc27a8a51', 'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/connection.json', 'connection_sha256': '7ec305787122202ca623e9e238c5e211b55995a241b3b21f14adefd430531845', 'diagnostic_uart_opened': False, 'diagnostic_block_device_opened': False, 'diagnostic_board_files_written': False, 'prior_boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json', 'prior_reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c', 'prior_boot_historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82677.647322] usb 1-1.3: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82987.887775] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83012.205481] usb 1-1.4: USB disconnect, device number 12', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83245.141873] usb 1-1.4: USB disconnect, device number 13', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'prior_boot_faults_cleared': False, 'filesystem_repaired': False, 'board_card_identity_unchanged': True, 'early_kernel_history_complete': True, 'observed_boot_reason': 'COLD_BOOT', 'unexpected_reboot_root_cause_confirmed': False, 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg', 'observed_serial': {'path': '/dev/ttyUSB0', 'sysfs_name': '1-1.4', 'vid': '1a86', 'pid': '7523', 'serial_number': ''}, 'serial_discovery_required': True, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}, 'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-usb-disconnect-20261005-01/stdout.jsonl', 'metadata_capture_sha256': '9efd34b817e471f7d0bc0268d4103211d125404e971817ff250d29ba5e081e3d', 'ssh_exit_code': 0, 'ssh_process_exited': True, 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg', 'observed_serial': {'path': '/dev/ttyUSB0', 'sysfs_name': '1-1.4', 'vid': '1a86', 'pid': '7523', 'serial_number': ''}, 'serial_ownership_observations': 2, 'serial_holders': [], 'old_kernel_prefix_retained': True, 'new_kernel_lines': ['[36903.064378] usb 1-1-port4: disabled by hub (EMI?), re-enabling...', '[36903.070883] usb 1-1.4: USB disconnect, device number 3', '[36903.072460] ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0', '[36903.072647] ch341 1-1.4:1.0: device disconnected', '[36903.416608] usb 1-1.4: new full-speed USB device number 4 using xhci-hcd', '[36903.567826] usb 1-1.4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64', '[36903.567888] usb 1-1.4: New USB device strings: Mfr=0, Product=2, SerialNumber=0', '[36903.567927] usb 1-1.4: Product: USB Serial', '[36903.608493] ch341 1-1.4:1.0: ch341-uart converter detected', '[36903.610058] usb 1-1.4: ch341-uart converter now attached to ttyUSB0'], 'unresolved_historical_faults_retained': True, 'usb_disconnect_electrical_cause_confirmed': False, 'kernel_emi_hint_not_diagnosed_as_proven_interference': True, 'filesystem_repaired': False, 'servo_fault_hold_cleared': False, 'diagnostic_uart_opened': False, 'board_files_written': False, 'servo_motion_authorized': False}
COLD_USB_DROP_HISTORICAL_FAULTS = [{'line': '[    7.379408] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.469697] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[36903.070883] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
COLD_USB_DROP_PROOF = {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9', 'kernel_prefix_sha256': '05af493f614d06e30caa269c477bc455aeddee33f783bd8f769d2e714a3d598a', 'kernel_line_count': 932}
# Manually reviewed Wi-Fi cold boot: retain all prior artifacts and fault history.
WIFI_RECONNECT_BOOT_ID = 'f2070288-4492-4ddc-9594-ad00640d01b7'
WIFI_RECONNECT_PROFILE_SHA256 = 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0'
WIFI_RECONNECT_REQUIRED_REVIEW = {
    'reviewer': 'root',
    'reviewed_at': '2026-10-06T00:27:28+08:00',
    'human_context': ('用户明确要求日后通过 Wi-Fi 使用 X5；本地审查当前 Wi-Fi 被动记录及两次完整原始内核日志，确认该板卡发生冷启动，主机密钥、SD '
 'CID、根挂载、板卡型号与既有核对记录一致，唯一 CH340 仍为 USB1-1.4。两次元数据及 924 '
 '行日志逐字相同，串口无占用。本次只建立精确固定启动的只读反馈基线，不判定冷启动根因、不修复文件系统、不清除旧故障或运动占用。'),
    'scope_detail': ('feedback-only exact same board/card Wi-Fi cold-boot review; no motion or storage write '
 'allowance'),
    'prior_boot_id': COLD_RECONNECT_BOOT_ID,
    'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
    'prior_reviewed_profile_sha256': COLD_USB_DROP_PROFILE_SHA256,
    'prior_review': COLD_USB_DROP_REQUIRED_REVIEW,
    'prior_boot_historical_faults': COLD_USB_DROP_HISTORICAL_FAULTS,
    'earlier_boot_historical_faults': [
        {'boot_id': REVIEWED_BOOT_ID, 'reviewed_profile_sha256': RECONNECT2_PROFILE_SHA256,
         'historical_faults': RECONNECT2_HISTORICAL_FAULTS},
        {'boot_id': REBOOT_BOOT_ID, 'reviewed_profile_sha256': USB_PORT_MOVE_ESP32_PROFILE_SHA256,
         'historical_faults': USB_PORT_MOVE_ESP32_HISTORICAL_FAULTS},
    ],
    'passive_metadata_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/stdout.jsonl',
    'passive_metadata_sha256': '9e0fe674dbaa3a8314ef449441d21f4a9f494273f322e645251768f1f0d0b331',
    'passive_connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/connection.json',
    'passive_connection_sha256': '1971361ee38199b23ed3837147a8685491103660f3e4616c13bd27fd2c7ff549',
    'passive_exit_code': 20,
    'prior_boot_pin_refusal_retained': True,
    'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/stdout.jsonl',
    'metadata_capture_sha256': '77b881bc12c5c0405a331c92cbbc4f9dd27508c0d2d8ec21dcb3de8048607814',
    'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/connection.json',
    'connection_sha256': 'a9b9d87471824cc421b1b7e7021c76d75ee16261dbaa72b188912b6cac897eff',
    'ssh_exit_code': 0,
    'ssh_process_exited': True,
    'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
    'observed_transport': 'wifi',
    'observed_target_host': '192.168.1.11',
    'stable_metadata_observations': 2,
    'serial_holders': [],
    'observed_serial': {'path': '/dev/ttyUSB0',
 'sysfs_name': '1-1.4',
 'vid': '1a86',
 'pid': '7523',
 'serial_number': ''},
    'serial_discovery_required': True,
    'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat /dev/mmcblk1p1 '
 'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
    'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
    'raw_kernel_cr_count': 30,
    'raw_kernel_preserved': True,
    'passive_crlf_normalization_only': True,
    'early_kernel_history_complete': True,
    'observed_boot_reason': 'COLD_BOOT',
    'board_card_identity_unchanged': True,
    'unresolved_historical_faults_retained': True,
    'prior_boot_faults_cleared': False,
    'filesystem_repaired': False,
    'unexpected_reboot_root_cause_confirmed': False,
    'electrical_safety_confirmed': False,
    'wiring_verified': False,
    'storage_health_certified': False,
    'diagnostic_uart_opened': False,
    'diagnostic_block_device_opened': False,
    'diagnostic_board_files_written': False,
    'unsafe_feedback_hold_cleared': False,
    'servo_fault_hold_cleared': False,
    'servo_motion_authorized': False,
    'automatic_reconnect_authorized': False,
}
WIFI_RECONNECT_HISTORICAL_FAULTS = [{'line': '[    7.647480] systemd-journald[276]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.707257] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]
WIFI_RECONNECT_PROOF = {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0', 'kernel_prefix_sha256': '36c8abdbbd95399cafa57ecc5ef46d26326f96979bda4f4ddd3aad5901db0ada', 'kernel_line_count': 924}

# One explicitly reviewed same-card cold boot; this is never automatic discovery.
IMU_RECOVERY_RECONNECT_BOOT_ID = '4a15fef9-dcc3-4e59-84a7-e8b62a01e208'
IMU_RECOVERY_RECONNECT_PROFILE_SHA256 = '03fd2bde822b13a57223185d5832c64c83d69a9b69fd50930e8b09b1f66467f3'
IMU_RECOVERY_RECONNECT_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T02:15:00.115492+08:00',
 'human_context': '用户要求恢复原生网页和 IMU 显示。此前固定启动身份复查按原保护拒绝并实际 SSH exit20；另一次只读元数据确认该 X5 '
                  '已发生新冷启动。两次完整观察逐字一致，固定主机密钥、SD CID、根挂载、型号和唯一 USB1-1.4 CH340 保持原身份。独立完整内核分类仅有两条旧类 '
                  'journald/FAT 未正常卸载警告，首行启动历史完整。重启原因未证实，本次只手工加入这一精确 boot 的反馈基线，不自动学习新启动、不清除旧 '
                  'SD/USB/WiFi、IMU 或运动安全故障，不授权电动测试、修盘或持久写入。',
 'scope_detail': 'feedback-only exact same board/card IMU-recovery cold-boot review; no motion or '
                 'storage write allowance',
 'prior_boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
 'prior_reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
 'prior_review': {'reviewer': 'root',
                  'reviewed_at': '2026-10-06T00:27:28+08:00',
                  'human_context': '用户明确要求日后通过 Wi-Fi 使用 X5；本地审查当前 Wi-Fi '
                                   '被动记录及两次完整原始内核日志，确认该板卡发生冷启动，主机密钥、SD CID、根挂载、板卡型号与既有核对记录一致，唯一 '
                                   'CH340 仍为 USB1-1.4。两次元数据及 924 '
                                   '行日志逐字相同，串口无占用。本次只建立精确固定启动的只读反馈基线，不判定冷启动根因、不修复文件系统、不清除旧故障或运动占用。',
                  'scope_detail': 'feedback-only exact same board/card Wi-Fi cold-boot review; no '
                                  'motion or storage write allowance',
                  'prior_boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                  'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                  'prior_reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                  'prior_review': {'reviewer': 'root',
                                   'reviewed_at': '2026-10-05T18:09:55.119122+08:00',
                                   'purpose': 'Explicit read-only recovery after a completed '
                                              'same-port ESP32 USB disconnect; no movement or '
                                              'storage writes',
                                   'prior_profile_sha256': 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0',
                                   'prior_review': {'reviewer': 'root',
                                                    'reviewed_at': '2026-10-05T07:43:24.369656+08:00',
                                                    'human_context': '用户称重后看到 USB '
                                                                     '闪联路由错误，明确要求恢复使用，并确认 URT2 '
                                                                     '电池已接通。被动元数据和完整启动日志证明 X5 '
                                                                     '已冷启动；SSH 主机密钥、板卡型号、SD CID '
                                                                     '和根挂载均与上一份核对记录一致，唯一 CH340 '
                                                                     '仍位于用户更换后的 '
                                                                     'USB1-1.4。此次启动日志保留最早启动行，共922行；完整分类仅有两条未正常卸载的文件系统警告，继续作为未解决故障保留。旧启动中的 '
                                                                     'USB 和 WiFi '
                                                                     '故障通过旧基线摘要和完整故障列表保留，不认定已修复。本记录只允许一次显式只读复查；实际舵机状态仍须重新读取，不自动进入站姿或启动模型动作。',
                                                    'scope_detail': 'feedback-only exact same '
                                                                    'board/card cold-boot review; '
                                                                    'no motion or storage write '
                                                                    'allowance',
                                                    'diagnostic_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/result.json',
                                                    'diagnostic_result_sha256': '2056a55521d5c5382b071ce28fc12a4ea18e0641a0e8ad3dc4ea43afc27a8a51',
                                                    'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/connection.json',
                                                    'connection_sha256': '7ec305787122202ca623e9e238c5e211b55995a241b3b21f14adefd430531845',
                                                    'diagnostic_uart_opened': False,
                                                    'diagnostic_block_device_opened': False,
                                                    'diagnostic_board_files_written': False,
                                                    'prior_boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                                    'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                                    'prior_reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                                    'prior_boot_historical_faults': [{'line': '[    '
                                                                                              '7.293643] '
                                                                                              'systemd-journald[274]: '
                                                                                              'File '
                                                                                              '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                              'corrupted '
                                                                                              'or '
                                                                                              'uncleanly '
                                                                                              'shut '
                                                                                              'down, '
                                                                                              'renaming '
                                                                                              'and '
                                                                                              'replacing.',
                                                                                      'reason': 'Journald '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'corrupt '
                                                                                                'or '
                                                                                                'uncleanly '
                                                                                                'closed '
                                                                                                'journal '
                                                                                                'file'},
                                                                                     {'line': '[   '
                                                                                              '19.321881] '
                                                                                              'FAT-fs '
                                                                                              '(mmcblk1p1): '
                                                                                              'Volume '
                                                                                              'was '
                                                                                              'not '
                                                                                              'properly '
                                                                                              'unmounted. '
                                                                                              'Some '
                                                                                              'data '
                                                                                              'may '
                                                                                              'be '
                                                                                              'corrupt. '
                                                                                              'Please '
                                                                                              'run '
                                                                                              'fsck.',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'an '
                                                                                                'unclean '
                                                                                                'FAT '
                                                                                                'volume '
                                                                                                'or '
                                                                                                'filesystem '
                                                                                                'fault'},
                                                                                     {'line': '[ '
                                                                                              '5127.470586] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '3',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[68102.480531] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '4',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[69821.346890] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '5',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[71514.103758] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '6',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[73274.437850] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '7',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[78166.615277] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '8',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[79843.245789] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '9',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[82677.647322] '
                                                                                              'usb '
                                                                                              '1-1.3: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '10',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[82987.887775] '
                                                                                              'usb '
                                                                                              '1-1.4: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '11',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[83012.205481] '
                                                                                              'usb '
                                                                                              '1-1.4: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '12',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'},
                                                                                     {'line': '[83245.141873] '
                                                                                              'usb '
                                                                                              '1-1.4: '
                                                                                              'USB '
                                                                                              'disconnect, '
                                                                                              'device '
                                                                                              'number '
                                                                                              '13',
                                                                                      'reason': 'Kernel '
                                                                                                'reported '
                                                                                                'a '
                                                                                                'USB '
                                                                                                'disconnect/reset/error'}],
                                                    'prior_boot_faults_cleared': False,
                                                    'filesystem_repaired': False,
                                                    'board_card_identity_unchanged': True,
                                                    'early_kernel_history_complete': True,
                                                    'observed_boot_reason': 'COLD_BOOT',
                                                    'unexpected_reboot_root_cause_confirmed': False,
                                                    'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                                                    'observed_serial': {'path': '/dev/ttyUSB0',
                                                                        'sysfs_name': '1-1.4',
                                                                        'vid': '1a86',
                                                                        'pid': '7523',
                                                                        'serial_number': ''},
                                                    'serial_discovery_required': True,
                                                    'unsafe_feedback_hold_cleared': False,
                                                    'servo_motion_authorized': False},
                                   'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-usb-disconnect-20261005-01/stdout.jsonl',
                                   'metadata_capture_sha256': '9efd34b817e471f7d0bc0268d4103211d125404e971817ff250d29ba5e081e3d',
                                   'ssh_exit_code': 0,
                                   'ssh_process_exited': True,
                                   'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                                   'observed_serial': {'path': '/dev/ttyUSB0',
                                                       'sysfs_name': '1-1.4',
                                                       'vid': '1a86',
                                                       'pid': '7523',
                                                       'serial_number': ''},
                                   'serial_ownership_observations': 2,
                                   'serial_holders': [],
                                   'old_kernel_prefix_retained': True,
                                   'new_kernel_lines': ['[36903.064378] usb 1-1-port4: disabled by '
                                                        'hub (EMI?), re-enabling...',
                                                        '[36903.070883] usb 1-1.4: USB disconnect, '
                                                        'device number 3',
                                                        '[36903.072460] ch341-uart ttyUSB0: '
                                                        'ch341-uart converter now disconnected '
                                                        'from ttyUSB0',
                                                        '[36903.072647] ch341 1-1.4:1.0: device '
                                                        'disconnected',
                                                        '[36903.416608] usb 1-1.4: new full-speed '
                                                        'USB device number 4 using xhci-hcd',
                                                        '[36903.567826] usb 1-1.4: New USB device '
                                                        'found, idVendor=1a86, idProduct=7523, '
                                                        'bcdDevice= 2.64',
                                                        '[36903.567888] usb 1-1.4: New USB device '
                                                        'strings: Mfr=0, Product=2, SerialNumber=0',
                                                        '[36903.567927] usb 1-1.4: Product: USB '
                                                        'Serial',
                                                        '[36903.608493] ch341 1-1.4:1.0: '
                                                        'ch341-uart converter detected',
                                                        '[36903.610058] usb 1-1.4: ch341-uart '
                                                        'converter now attached to ttyUSB0'],
                                   'unresolved_historical_faults_retained': True,
                                   'usb_disconnect_electrical_cause_confirmed': False,
                                   'kernel_emi_hint_not_diagnosed_as_proven_interference': True,
                                   'filesystem_repaired': False,
                                   'servo_fault_hold_cleared': False,
                                   'diagnostic_uart_opened': False,
                                   'board_files_written': False,
                                   'servo_motion_authorized': False},
                  'prior_boot_historical_faults': [{'line': '[    7.379408] systemd-journald[274]: '
                                                            'File '
                                                            '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                            'corrupted or uncleanly shut down, '
                                                            'renaming and replacing.',
                                                    'reason': 'Journald reported a corrupt or '
                                                              'uncleanly closed journal file'},
                                                   {'line': '[   19.469697] FAT-fs (mmcblk1p1): '
                                                            'Volume was not properly unmounted. '
                                                            'Some data may be corrupt. Please run '
                                                            'fsck.',
                                                    'reason': 'Kernel reported an unclean FAT '
                                                              'volume or filesystem fault'},
                                                   {'line': '[36903.070883] usb 1-1.4: USB '
                                                            'disconnect, device number 3',
                                                    'reason': 'Kernel reported a USB '
                                                              'disconnect/reset/error'}],
                  'earlier_boot_historical_faults': [{'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                                      'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                                      'historical_faults': [{'line': '[    '
                                                                                     '7.573470] '
                                                                                     'systemd-journald[273]: '
                                                                                     'File '
                                                                                     '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                     'corrupted or '
                                                                                     'uncleanly '
                                                                                     'shut down, '
                                                                                     'renaming and '
                                                                                     'replacing.',
                                                                             'reason': 'Journald '
                                                                                       'reported a '
                                                                                       'corrupt or '
                                                                                       'uncleanly '
                                                                                       'closed '
                                                                                       'journal '
                                                                                       'file'},
                                                                            {'line': '[   '
                                                                                     '19.944418] '
                                                                                     'FAT-fs '
                                                                                     '(mmcblk1p1): '
                                                                                     'Volume was '
                                                                                     'not properly '
                                                                                     'unmounted. '
                                                                                     'Some data '
                                                                                     'may be '
                                                                                     'corrupt. '
                                                                                     'Please run '
                                                                                     'fsck.',
                                                                             'reason': 'Kernel '
                                                                                       'reported '
                                                                                       'an unclean '
                                                                                       'FAT volume '
                                                                                       'or '
                                                                                       'filesystem '
                                                                                       'fault'},
                                                                            {'line': '[ '
                                                                                     '3008.252350] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 3',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[ '
                                                                                     '8518.691466] '
                                                                                     'usb 1-1: USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 2',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[ '
                                                                                     '8518.691521] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 4',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[ '
                                                                                     '8518.931957] '
                                                                                     'usb 2-1: USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 2',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[ '
                                                                                     '8544.480306] '
                                                                                     'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                                     'sdio wakeup '
                                                                                     'fail',
                                                                             'reason': 'Kernel '
                                                                                       'reported '
                                                                                       'an SD/MMC '
                                                                                       'failure'},
                                                                            {'line': '[ '
                                                                                     '9890.432382] '
                                                                                     'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                                     'ret=-110, '
                                                                                     'intstatus=ff',
                                                                             'reason': 'Feedback '
                                                                                       'guard '
                                                                                       'reported '
                                                                                       'an AIC '
                                                                                       'SDIO WiFi '
                                                                                       'driver '
                                                                                       'failure'}]},
                                                     {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                                      'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                                      'historical_faults': [{'line': '[    '
                                                                                     '7.293643] '
                                                                                     'systemd-journald[274]: '
                                                                                     'File '
                                                                                     '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                     'corrupted or '
                                                                                     'uncleanly '
                                                                                     'shut down, '
                                                                                     'renaming and '
                                                                                     'replacing.',
                                                                             'reason': 'Journald '
                                                                                       'reported a '
                                                                                       'corrupt or '
                                                                                       'uncleanly '
                                                                                       'closed '
                                                                                       'journal '
                                                                                       'file'},
                                                                            {'line': '[   '
                                                                                     '19.321881] '
                                                                                     'FAT-fs '
                                                                                     '(mmcblk1p1): '
                                                                                     'Volume was '
                                                                                     'not properly '
                                                                                     'unmounted. '
                                                                                     'Some data '
                                                                                     'may be '
                                                                                     'corrupt. '
                                                                                     'Please run '
                                                                                     'fsck.',
                                                                             'reason': 'Kernel '
                                                                                       'reported '
                                                                                       'an unclean '
                                                                                       'FAT volume '
                                                                                       'or '
                                                                                       'filesystem '
                                                                                       'fault'},
                                                                            {'line': '[ '
                                                                                     '5127.470586] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 3',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[68102.480531] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 4',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[69821.346890] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 5',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[71514.103758] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 6',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[73274.437850] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 7',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[78166.615277] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 8',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[79843.245789] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 9',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[82677.647322] '
                                                                                     'usb 1-1.3: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 10',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[82987.887775] '
                                                                                     'usb 1-1.4: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 11',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[83012.205481] '
                                                                                     'usb 1-1.4: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 12',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'},
                                                                            {'line': '[83245.141873] '
                                                                                     'usb 1-1.4: '
                                                                                     'USB '
                                                                                     'disconnect, '
                                                                                     'device '
                                                                                     'number 13',
                                                                             'reason': 'Kernel '
                                                                                       'reported a '
                                                                                       'USB '
                                                                                       'disconnect/reset/error'}]}],
                  'passive_metadata_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/stdout.jsonl',
                  'passive_metadata_sha256': '9e0fe674dbaa3a8314ef449441d21f4a9f494273f322e645251768f1f0d0b331',
                  'passive_connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/connection.json',
                  'passive_connection_sha256': '1971361ee38199b23ed3837147a8685491103660f3e4616c13bd27fd2c7ff549',
                  'passive_exit_code': 20,
                  'prior_boot_pin_refusal_retained': True,
                  'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/stdout.jsonl',
                  'metadata_capture_sha256': '77b881bc12c5c0405a331c92cbbc4f9dd27508c0d2d8ec21dcb3de8048607814',
                  'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/connection.json',
                  'connection_sha256': 'a9b9d87471824cc421b1b7e7021c76d75ee16261dbaa72b188912b6cac897eff',
                  'ssh_exit_code': 0,
                  'ssh_process_exited': True,
                  'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                  'observed_transport': 'wifi',
                  'observed_target_host': '192.168.1.11',
                  'stable_metadata_observations': 2,
                  'serial_holders': [],
                  'observed_serial': {'path': '/dev/ttyUSB0',
                                      'sysfs_name': '1-1.4',
                                      'vid': '1a86',
                                      'pid': '7523',
                                      'serial_number': ''},
                  'serial_discovery_required': True,
                  'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - '
                                            'vfat /dev/mmcblk1p1 '
                                            'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
                  'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs '
                                         'tmpfs rw'],
                  'raw_kernel_cr_count': 30,
                  'raw_kernel_preserved': True,
                  'passive_crlf_normalization_only': True,
                  'early_kernel_history_complete': True,
                  'observed_boot_reason': 'COLD_BOOT',
                  'board_card_identity_unchanged': True,
                  'unresolved_historical_faults_retained': True,
                  'prior_boot_faults_cleared': False,
                  'filesystem_repaired': False,
                  'unexpected_reboot_root_cause_confirmed': False,
                  'electrical_safety_confirmed': False,
                  'wiring_verified': False,
                  'storage_health_certified': False,
                  'diagnostic_uart_opened': False,
                  'diagnostic_block_device_opened': False,
                  'diagnostic_board_files_written': False,
                  'unsafe_feedback_hold_cleared': False,
                  'servo_fault_hold_cleared': False,
                  'servo_motion_authorized': False,
                  'automatic_reconnect_authorized': False},
 'prior_boot_historical_faults': [{'line': '[    7.647480] systemd-journald[276]: File '
                                           '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                           'corrupted or uncleanly shut down, renaming and '
                                           'replacing.',
                                   'reason': 'Journald reported a corrupt or uncleanly closed '
                                             'journal file'},
                                  {'line': '[   19.707257] FAT-fs (mmcblk1p1): Volume was not '
                                           'properly unmounted. Some data may be corrupt. Please '
                                           'run fsck.',
                                   'reason': 'Kernel reported an unclean FAT volume or filesystem '
                                             'fault'}],
 'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/result.json',
 'metadata_result_sha256': '87fff7405bd3ffa180fc6f42cbc4fedd87bab9e152a78b80c073003e1fbc5e92',
 'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/remote-source.py',
 'metadata_source_sha256': '2de3adfcb317b475e295812865230772f6530546164257e3786be09bbfb5ba43',
 'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/ssh-exit.json',
 'metadata_ssh_exit_sha256': '7b427e67e8d7f3e52e67476052ad738dac66a5ff24053734733a0f52cd9e5b38',
 'ssh_exit_code': 0,
 'ssh_process_exited': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
 'first_refusal_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/current-metadata/result.json',
 'first_refusal_result_sha256': '307e456bac20b40b56cc9124935bcb152a1b197a47fc34ecde78242ef11eb57f',
 'first_refusal_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/current-metadata/ssh-exit.json',
 'first_refusal_ssh_exit_sha256': '0bfc6e86210f14d952966e1886f014840ee0996464b729ae207b0676e620f852',
 'first_refusal_reason': 'Reviewed boot/root/card/model identity changed',
 'first_refusal_ssh_exit_code': 20,
 'prior_boot_pin_refusal_retained': True,
 'observed_transport': 'wifi',
 'observed_target_host': '192.168.1.11',
 'stable_metadata_observations': 2,
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'serial_discovery_required': True,
 'serial_holders': [],
 'observed_serial_node': {'device': 5, 'inode': 308, 'rdev': 48128},
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:224 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'raw_kernel_byte_count': 64529,
 'raw_kernel_cr_count': 30,
 'raw_kernel_preserved': True,
 'early_kernel_history_complete': True,
 'observed_boot_reason': 'COLD_BOOT',
 'independent_full_kernel_review_passed': True,
 'board_card_identity_unchanged': True,
 'unresolved_historical_faults_retained': True,
 'prior_boot_faults_cleared': False,
 'filesystem_repaired': False,
 'unexpected_reboot_root_cause_confirmed': False,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'storage_health_certified': False,
 'diagnostic_uart_opened': False,
 'diagnostic_block_device_opened': False,
 'diagnostic_board_files_written': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'prior_motion_occupancy_cleared': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False}
IMU_RECOVERY_RECONNECT_HISTORICAL_FAULTS = [{'line': '[    7.746460] systemd-journald[274]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]
IMU_RECOVERY_RECONNECT_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': '03fd2bde822b13a57223185d5832c64c83d69a9b69fd50930e8b09b1f66467f3',
 'kernel_prefix_sha256': 'a0cc0912ebd982ad206611c1015b30749a15b069eafe515d46a10f9df93a1959',
 'kernel_line_count': 924}

# Exact same-boot returned USB observations; unrelated future faults remain refused.
IMU_RECOVERY_USB_BOOT_ID = '4a15fef9-dcc3-4e59-84a7-e8b62a01e208'
IMU_RECOVERY_USB_PROFILE_SHA256 = 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca'
IMU_RECOVERY_USB_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T02:23:24.709413+08:00',
 'human_context': '用户确认 URT2 舵机电池已断开且碰过线，但不确定是否移动 ESP32 COM。此前纯元数据复查按原保护拒绝本启动新增的两次 USB 断开；并未打开 '
                  'UART 或发送舵机指令。随后两次独立只读元数据捕获、共四份观察完全一致，捕获文件时间间隔 96.47230386734009 秒，原 924 '
                  '行完整启动历史逐字保留，新尾部仅 20 行两次同口 CH340 断开及返回。完整分类保留原两条文件系统警告及两条新 USB 断开为未解决事件；两次 hub 的 '
                  'EMI? 提示不是已证实电磁干扰或供电根因。稳定窗口不能证明电气安全、修复连接或清除故障。本记录仅允许下一次显式电池断开条件下的 HELLO/STATE '
                  '只读诊断，须再按真实结果决定部署，不自动重连、不使能舵机、不放宽任何新错误判定。',
 'scope_detail': 'feedback-only exact same-boot CH340 return after battery-off stable metadata; no '
                 'motion, service deployment or storage-write allowance',
 'prior_boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-reboot-reviewed-20261006-01.json',
 'prior_reviewed_profile_sha256': '03fd2bde822b13a57223185d5832c64c83d69a9b69fd50930e8b09b1f66467f3',
 'prior_review': {'reviewer': 'root',
                  'reviewed_at': '2026-10-06T02:15:00.115492+08:00',
                  'human_context': '用户要求恢复原生网页和 IMU 显示。此前固定启动身份复查按原保护拒绝并实际 SSH exit20；另一次只读元数据确认该 '
                                   'X5 已发生新冷启动。两次完整观察逐字一致，固定主机密钥、SD CID、根挂载、型号和唯一 USB1-1.4 CH340 '
                                   '保持原身份。独立完整内核分类仅有两条旧类 journald/FAT '
                                   '未正常卸载警告，首行启动历史完整。重启原因未证实，本次只手工加入这一精确 boot 的反馈基线，不自动学习新启动、不清除旧 '
                                   'SD/USB/WiFi、IMU 或运动安全故障，不授权电动测试、修盘或持久写入。',
                  'scope_detail': 'feedback-only exact same board/card IMU-recovery cold-boot '
                                  'review; no motion or storage write allowance',
                  'prior_boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
                  'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
                  'prior_reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                  'prior_review': {'reviewer': 'root',
                                   'reviewed_at': '2026-10-06T00:27:28+08:00',
                                   'human_context': '用户明确要求日后通过 Wi-Fi 使用 X5；本地审查当前 Wi-Fi '
                                                    '被动记录及两次完整原始内核日志，确认该板卡发生冷启动，主机密钥、SD '
                                                    'CID、根挂载、板卡型号与既有核对记录一致，唯一 CH340 仍为 '
                                                    'USB1-1.4。两次元数据及 924 '
                                                    '行日志逐字相同，串口无占用。本次只建立精确固定启动的只读反馈基线，不判定冷启动根因、不修复文件系统、不清除旧故障或运动占用。',
                                   'scope_detail': 'feedback-only exact same board/card Wi-Fi '
                                                   'cold-boot review; no motion or storage write '
                                                   'allowance',
                                   'prior_boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                                   'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                                   'prior_reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                                   'prior_review': {'reviewer': 'root',
                                                    'reviewed_at': '2026-10-05T18:09:55.119122+08:00',
                                                    'purpose': 'Explicit read-only recovery after '
                                                               'a completed same-port ESP32 USB '
                                                               'disconnect; no movement or storage '
                                                               'writes',
                                                    'prior_profile_sha256': 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0',
                                                    'prior_review': {'reviewer': 'root',
                                                                     'reviewed_at': '2026-10-05T07:43:24.369656+08:00',
                                                                     'human_context': '用户称重后看到 USB '
                                                                                      '闪联路由错误，明确要求恢复使用，并确认 '
                                                                                      'URT2 '
                                                                                      '电池已接通。被动元数据和完整启动日志证明 '
                                                                                      'X5 已冷启动；SSH '
                                                                                      '主机密钥、板卡型号、SD '
                                                                                      'CID '
                                                                                      '和根挂载均与上一份核对记录一致，唯一 '
                                                                                      'CH340 '
                                                                                      '仍位于用户更换后的 '
                                                                                      'USB1-1.4。此次启动日志保留最早启动行，共922行；完整分类仅有两条未正常卸载的文件系统警告，继续作为未解决故障保留。旧启动中的 '
                                                                                      'USB 和 WiFi '
                                                                                      '故障通过旧基线摘要和完整故障列表保留，不认定已修复。本记录只允许一次显式只读复查；实际舵机状态仍须重新读取，不自动进入站姿或启动模型动作。',
                                                                     'scope_detail': 'feedback-only '
                                                                                     'exact same '
                                                                                     'board/card '
                                                                                     'cold-boot '
                                                                                     'review; no '
                                                                                     'motion or '
                                                                                     'storage '
                                                                                     'write '
                                                                                     'allowance',
                                                                     'diagnostic_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/result.json',
                                                                     'diagnostic_result_sha256': '2056a55521d5c5382b071ce28fc12a4ea18e0641a0e8ad3dc4ea43afc27a8a51',
                                                                     'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-weight-reconnect-20261005-01/connection.json',
                                                                     'connection_sha256': '7ec305787122202ca623e9e238c5e211b55995a241b3b21f14adefd430531845',
                                                                     'diagnostic_uart_opened': False,
                                                                     'diagnostic_block_device_opened': False,
                                                                     'diagnostic_board_files_written': False,
                                                                     'prior_boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                                                     'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                                                     'prior_reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                                                     'prior_boot_historical_faults': [{'line': '[    '
                                                                                                               '7.293643] '
                                                                                                               'systemd-journald[274]: '
                                                                                                               'File '
                                                                                                               '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                                               'corrupted '
                                                                                                               'or '
                                                                                                               'uncleanly '
                                                                                                               'shut '
                                                                                                               'down, '
                                                                                                               'renaming '
                                                                                                               'and '
                                                                                                               'replacing.',
                                                                                                       'reason': 'Journald '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'corrupt '
                                                                                                                 'or '
                                                                                                                 'uncleanly '
                                                                                                                 'closed '
                                                                                                                 'journal '
                                                                                                                 'file'},
                                                                                                      {'line': '[   '
                                                                                                               '19.321881] '
                                                                                                               'FAT-fs '
                                                                                                               '(mmcblk1p1): '
                                                                                                               'Volume '
                                                                                                               'was '
                                                                                                               'not '
                                                                                                               'properly '
                                                                                                               'unmounted. '
                                                                                                               'Some '
                                                                                                               'data '
                                                                                                               'may '
                                                                                                               'be '
                                                                                                               'corrupt. '
                                                                                                               'Please '
                                                                                                               'run '
                                                                                                               'fsck.',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'an '
                                                                                                                 'unclean '
                                                                                                                 'FAT '
                                                                                                                 'volume '
                                                                                                                 'or '
                                                                                                                 'filesystem '
                                                                                                                 'fault'},
                                                                                                      {'line': '[ '
                                                                                                               '5127.470586] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '3',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[68102.480531] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '4',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[69821.346890] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '5',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[71514.103758] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '6',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[73274.437850] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '7',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[78166.615277] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '8',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[79843.245789] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '9',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[82677.647322] '
                                                                                                               'usb '
                                                                                                               '1-1.3: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '10',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[82987.887775] '
                                                                                                               'usb '
                                                                                                               '1-1.4: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '11',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[83012.205481] '
                                                                                                               'usb '
                                                                                                               '1-1.4: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '12',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'},
                                                                                                      {'line': '[83245.141873] '
                                                                                                               'usb '
                                                                                                               '1-1.4: '
                                                                                                               'USB '
                                                                                                               'disconnect, '
                                                                                                               'device '
                                                                                                               'number '
                                                                                                               '13',
                                                                                                       'reason': 'Kernel '
                                                                                                                 'reported '
                                                                                                                 'a '
                                                                                                                 'USB '
                                                                                                                 'disconnect/reset/error'}],
                                                                     'prior_boot_faults_cleared': False,
                                                                     'filesystem_repaired': False,
                                                                     'board_card_identity_unchanged': True,
                                                                     'early_kernel_history_complete': True,
                                                                     'observed_boot_reason': 'COLD_BOOT',
                                                                     'unexpected_reboot_root_cause_confirmed': False,
                                                                     'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                                                                     'observed_serial': {'path': '/dev/ttyUSB0',
                                                                                         'sysfs_name': '1-1.4',
                                                                                         'vid': '1a86',
                                                                                         'pid': '7523',
                                                                                         'serial_number': ''},
                                                                     'serial_discovery_required': True,
                                                                     'unsafe_feedback_hold_cleared': False,
                                                                     'servo_motion_authorized': False},
                                                    'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-usb-disconnect-20261005-01/stdout.jsonl',
                                                    'metadata_capture_sha256': '9efd34b817e471f7d0bc0268d4103211d125404e971817ff250d29ba5e081e3d',
                                                    'ssh_exit_code': 0,
                                                    'ssh_process_exited': True,
                                                    'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                                                    'observed_serial': {'path': '/dev/ttyUSB0',
                                                                        'sysfs_name': '1-1.4',
                                                                        'vid': '1a86',
                                                                        'pid': '7523',
                                                                        'serial_number': ''},
                                                    'serial_ownership_observations': 2,
                                                    'serial_holders': [],
                                                    'old_kernel_prefix_retained': True,
                                                    'new_kernel_lines': ['[36903.064378] usb '
                                                                         '1-1-port4: disabled by '
                                                                         'hub (EMI?), '
                                                                         're-enabling...',
                                                                         '[36903.070883] usb '
                                                                         '1-1.4: USB disconnect, '
                                                                         'device number 3',
                                                                         '[36903.072460] '
                                                                         'ch341-uart ttyUSB0: '
                                                                         'ch341-uart converter now '
                                                                         'disconnected from '
                                                                         'ttyUSB0',
                                                                         '[36903.072647] ch341 '
                                                                         '1-1.4:1.0: device '
                                                                         'disconnected',
                                                                         '[36903.416608] usb '
                                                                         '1-1.4: new full-speed '
                                                                         'USB device number 4 '
                                                                         'using xhci-hcd',
                                                                         '[36903.567826] usb '
                                                                         '1-1.4: New USB device '
                                                                         'found, idVendor=1a86, '
                                                                         'idProduct=7523, '
                                                                         'bcdDevice= 2.64',
                                                                         '[36903.567888] usb '
                                                                         '1-1.4: New USB device '
                                                                         'strings: Mfr=0, '
                                                                         'Product=2, '
                                                                         'SerialNumber=0',
                                                                         '[36903.567927] usb '
                                                                         '1-1.4: Product: USB '
                                                                         'Serial',
                                                                         '[36903.608493] ch341 '
                                                                         '1-1.4:1.0: ch341-uart '
                                                                         'converter detected',
                                                                         '[36903.610058] usb '
                                                                         '1-1.4: ch341-uart '
                                                                         'converter now attached '
                                                                         'to ttyUSB0'],
                                                    'unresolved_historical_faults_retained': True,
                                                    'usb_disconnect_electrical_cause_confirmed': False,
                                                    'kernel_emi_hint_not_diagnosed_as_proven_interference': True,
                                                    'filesystem_repaired': False,
                                                    'servo_fault_hold_cleared': False,
                                                    'diagnostic_uart_opened': False,
                                                    'board_files_written': False,
                                                    'servo_motion_authorized': False},
                                   'prior_boot_historical_faults': [{'line': '[    7.379408] '
                                                                             'systemd-journald[274]: '
                                                                             'File '
                                                                             '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                             'corrupted or '
                                                                             'uncleanly shut down, '
                                                                             'renaming and '
                                                                             'replacing.',
                                                                     'reason': 'Journald reported '
                                                                               'a corrupt or '
                                                                               'uncleanly closed '
                                                                               'journal file'},
                                                                    {'line': '[   19.469697] '
                                                                             'FAT-fs (mmcblk1p1): '
                                                                             'Volume was not '
                                                                             'properly unmounted. '
                                                                             'Some data may be '
                                                                             'corrupt. Please run '
                                                                             'fsck.',
                                                                     'reason': 'Kernel reported an '
                                                                               'unclean FAT volume '
                                                                               'or filesystem '
                                                                               'fault'},
                                                                    {'line': '[36903.070883] usb '
                                                                             '1-1.4: USB '
                                                                             'disconnect, device '
                                                                             'number 3',
                                                                     'reason': 'Kernel reported a '
                                                                               'USB '
                                                                               'disconnect/reset/error'}],
                                   'earlier_boot_historical_faults': [{'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                                                       'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                                                       'historical_faults': [{'line': '[    '
                                                                                                      '7.573470] '
                                                                                                      'systemd-journald[273]: '
                                                                                                      'File '
                                                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                                      'corrupted '
                                                                                                      'or '
                                                                                                      'uncleanly '
                                                                                                      'shut '
                                                                                                      'down, '
                                                                                                      'renaming '
                                                                                                      'and '
                                                                                                      'replacing.',
                                                                                              'reason': 'Journald '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'corrupt '
                                                                                                        'or '
                                                                                                        'uncleanly '
                                                                                                        'closed '
                                                                                                        'journal '
                                                                                                        'file'},
                                                                                             {'line': '[   '
                                                                                                      '19.944418] '
                                                                                                      'FAT-fs '
                                                                                                      '(mmcblk1p1): '
                                                                                                      'Volume '
                                                                                                      'was '
                                                                                                      'not '
                                                                                                      'properly '
                                                                                                      'unmounted. '
                                                                                                      'Some '
                                                                                                      'data '
                                                                                                      'may '
                                                                                                      'be '
                                                                                                      'corrupt. '
                                                                                                      'Please '
                                                                                                      'run '
                                                                                                      'fsck.',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'an '
                                                                                                        'unclean '
                                                                                                        'FAT '
                                                                                                        'volume '
                                                                                                        'or '
                                                                                                        'filesystem '
                                                                                                        'fault'},
                                                                                             {'line': '[ '
                                                                                                      '3008.252350] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '3',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[ '
                                                                                                      '8518.691466] '
                                                                                                      'usb '
                                                                                                      '1-1: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '2',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[ '
                                                                                                      '8518.691521] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '4',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[ '
                                                                                                      '8518.931957] '
                                                                                                      'usb '
                                                                                                      '2-1: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '2',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[ '
                                                                                                      '8544.480306] '
                                                                                                      'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                                                      'sdio '
                                                                                                      'wakeup '
                                                                                                      'fail',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'an '
                                                                                                        'SD/MMC '
                                                                                                        'failure'},
                                                                                             {'line': '[ '
                                                                                                      '9890.432382] '
                                                                                                      'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                                                      'ret=-110, '
                                                                                                      'intstatus=ff',
                                                                                              'reason': 'Feedback '
                                                                                                        'guard '
                                                                                                        'reported '
                                                                                                        'an '
                                                                                                        'AIC '
                                                                                                        'SDIO '
                                                                                                        'WiFi '
                                                                                                        'driver '
                                                                                                        'failure'}]},
                                                                      {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                                                       'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                                                       'historical_faults': [{'line': '[    '
                                                                                                      '7.293643] '
                                                                                                      'systemd-journald[274]: '
                                                                                                      'File '
                                                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                                                      'corrupted '
                                                                                                      'or '
                                                                                                      'uncleanly '
                                                                                                      'shut '
                                                                                                      'down, '
                                                                                                      'renaming '
                                                                                                      'and '
                                                                                                      'replacing.',
                                                                                              'reason': 'Journald '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'corrupt '
                                                                                                        'or '
                                                                                                        'uncleanly '
                                                                                                        'closed '
                                                                                                        'journal '
                                                                                                        'file'},
                                                                                             {'line': '[   '
                                                                                                      '19.321881] '
                                                                                                      'FAT-fs '
                                                                                                      '(mmcblk1p1): '
                                                                                                      'Volume '
                                                                                                      'was '
                                                                                                      'not '
                                                                                                      'properly '
                                                                                                      'unmounted. '
                                                                                                      'Some '
                                                                                                      'data '
                                                                                                      'may '
                                                                                                      'be '
                                                                                                      'corrupt. '
                                                                                                      'Please '
                                                                                                      'run '
                                                                                                      'fsck.',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'an '
                                                                                                        'unclean '
                                                                                                        'FAT '
                                                                                                        'volume '
                                                                                                        'or '
                                                                                                        'filesystem '
                                                                                                        'fault'},
                                                                                             {'line': '[ '
                                                                                                      '5127.470586] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '3',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[68102.480531] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '4',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[69821.346890] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '5',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[71514.103758] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '6',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[73274.437850] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '7',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[78166.615277] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '8',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[79843.245789] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '9',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[82677.647322] '
                                                                                                      'usb '
                                                                                                      '1-1.3: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '10',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[82987.887775] '
                                                                                                      'usb '
                                                                                                      '1-1.4: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '11',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[83012.205481] '
                                                                                                      'usb '
                                                                                                      '1-1.4: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '12',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'},
                                                                                             {'line': '[83245.141873] '
                                                                                                      'usb '
                                                                                                      '1-1.4: '
                                                                                                      'USB '
                                                                                                      'disconnect, '
                                                                                                      'device '
                                                                                                      'number '
                                                                                                      '13',
                                                                                              'reason': 'Kernel '
                                                                                                        'reported '
                                                                                                        'a '
                                                                                                        'USB '
                                                                                                        'disconnect/reset/error'}]}],
                                   'passive_metadata_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/stdout.jsonl',
                                   'passive_metadata_sha256': '9e0fe674dbaa3a8314ef449441d21f4a9f494273f322e645251768f1f0d0b331',
                                   'passive_connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/postflash-passive-wifi-20261006-01/connection.json',
                                   'passive_connection_sha256': '1971361ee38199b23ed3837147a8685491103660f3e4616c13bd27fd2c7ff549',
                                   'passive_exit_code': 20,
                                   'prior_boot_pin_refusal_retained': True,
                                   'metadata_capture_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/stdout.jsonl',
                                   'metadata_capture_sha256': '77b881bc12c5c0405a331c92cbbc4f9dd27508c0d2d8ec21dcb3de8048607814',
                                   'connection_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/metadata-wifi-mount-20261006-02/connection.json',
                                   'connection_sha256': 'a9b9d87471824cc421b1b7e7021c76d75ee16261dbaa72b188912b6cac897eff',
                                   'ssh_exit_code': 0,
                                   'ssh_process_exited': True,
                                   'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                                   'observed_transport': 'wifi',
                                   'observed_target_host': '192.168.1.11',
                                   'stable_metadata_observations': 2,
                                   'serial_holders': [],
                                   'observed_serial': {'path': '/dev/ttyUSB0',
                                                       'sysfs_name': '1-1.4',
                                                       'vid': '1a86',
                                                       'pid': '7523',
                                                       'serial_number': ''},
                                   'serial_discovery_required': True,
                                   'observed_config_mount': ['425 21 179:1 / /boot/config '
                                                             'rw,relatime shared:189 - vfat '
                                                             '/dev/mmcblk1p1 '
                                                             'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
                                   'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev '
                                                          'shared:3 - tmpfs tmpfs rw'],
                                   'raw_kernel_cr_count': 30,
                                   'raw_kernel_preserved': True,
                                   'passive_crlf_normalization_only': True,
                                   'early_kernel_history_complete': True,
                                   'observed_boot_reason': 'COLD_BOOT',
                                   'board_card_identity_unchanged': True,
                                   'unresolved_historical_faults_retained': True,
                                   'prior_boot_faults_cleared': False,
                                   'filesystem_repaired': False,
                                   'unexpected_reboot_root_cause_confirmed': False,
                                   'electrical_safety_confirmed': False,
                                   'wiring_verified': False,
                                   'storage_health_certified': False,
                                   'diagnostic_uart_opened': False,
                                   'diagnostic_block_device_opened': False,
                                   'diagnostic_board_files_written': False,
                                   'unsafe_feedback_hold_cleared': False,
                                   'servo_fault_hold_cleared': False,
                                   'servo_motion_authorized': False,
                                   'automatic_reconnect_authorized': False},
                  'prior_boot_historical_faults': [{'line': '[    7.647480] systemd-journald[276]: '
                                                            'File '
                                                            '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                            'corrupted or uncleanly shut down, '
                                                            'renaming and replacing.',
                                                    'reason': 'Journald reported a corrupt or '
                                                              'uncleanly closed journal file'},
                                                   {'line': '[   19.707257] FAT-fs (mmcblk1p1): '
                                                            'Volume was not properly unmounted. '
                                                            'Some data may be corrupt. Please run '
                                                            'fsck.',
                                                    'reason': 'Kernel reported an unclean FAT '
                                                              'volume or filesystem fault'}],
                  'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/result.json',
                  'metadata_result_sha256': '87fff7405bd3ffa180fc6f42cbc4fedd87bab9e152a78b80c073003e1fbc5e92',
                  'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/remote-source.py',
                  'metadata_source_sha256': '2de3adfcb317b475e295812865230772f6530546164257e3786be09bbfb5ba43',
                  'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/post-reboot-metadata/ssh-exit.json',
                  'metadata_ssh_exit_sha256': '7b427e67e8d7f3e52e67476052ad738dac66a5ff24053734733a0f52cd9e5b38',
                  'ssh_exit_code': 0,
                  'ssh_process_exited': True,
                  'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
                  'first_refusal_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/current-metadata/result.json',
                  'first_refusal_result_sha256': '307e456bac20b40b56cc9124935bcb152a1b197a47fc34ecde78242ef11eb57f',
                  'first_refusal_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/current-metadata/ssh-exit.json',
                  'first_refusal_ssh_exit_sha256': '0bfc6e86210f14d952966e1886f014840ee0996464b729ae207b0676e620f852',
                  'first_refusal_reason': 'Reviewed boot/root/card/model identity changed',
                  'first_refusal_ssh_exit_code': 20,
                  'prior_boot_pin_refusal_retained': True,
                  'observed_transport': 'wifi',
                  'observed_target_host': '192.168.1.11',
                  'stable_metadata_observations': 2,
                  'observed_serial': {'path': '/dev/ttyUSB0',
                                      'sysfs_name': '1-1.4',
                                      'vid': '1a86',
                                      'pid': '7523',
                                      'serial_number': ''},
                  'serial_discovery_required': True,
                  'serial_holders': [],
                  'observed_serial_node': {'device': 5, 'inode': 308, 'rdev': 48128},
                  'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:224 - '
                                            'vfat /dev/mmcblk1p1 '
                                            'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
                  'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs '
                                         'tmpfs rw'],
                  'raw_kernel_byte_count': 64529,
                  'raw_kernel_cr_count': 30,
                  'raw_kernel_preserved': True,
                  'early_kernel_history_complete': True,
                  'observed_boot_reason': 'COLD_BOOT',
                  'independent_full_kernel_review_passed': True,
                  'board_card_identity_unchanged': True,
                  'unresolved_historical_faults_retained': True,
                  'prior_boot_faults_cleared': False,
                  'filesystem_repaired': False,
                  'unexpected_reboot_root_cause_confirmed': False,
                  'electrical_safety_confirmed': False,
                  'wiring_verified': False,
                  'storage_health_certified': False,
                  'diagnostic_uart_opened': False,
                  'diagnostic_block_device_opened': False,
                  'diagnostic_board_files_written': False,
                  'unsafe_feedback_hold_cleared': False,
                  'servo_fault_hold_cleared': False,
                  'prior_imu_feedback_fault_cleared': False,
                  'prior_motion_occupancy_cleared': False,
                  'servo_motion_authorized': False,
                  'automatic_reconnect_authorized': False},
 'prior_boot_historical_faults': [{'line': '[    7.746460] systemd-journald[274]: File '
                                           '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                           'corrupted or uncleanly shut down, renaming and '
                                           'replacing.',
                                   'reason': 'Journald reported a corrupt or uncleanly closed '
                                             'journal file'},
                                  {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not '
                                           'properly unmounted. Some data may be corrupt. Please '
                                           'run fsck.',
                                   'reason': 'Kernel reported an unclean FAT volume or filesystem '
                                             'fault'}],
 'metadata_captures': [{'result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-baseline/result.json',
                        'result_sha256': '832a28df6194a48e580db1dd239d7a689214a403e5b4bcfa75b33801fcf811a1',
                        'source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-baseline/remote-source.py',
                        'source_sha256': '312b8ded87bde54cc42d0c3830a4fa4f0acd6c904d25d30b1c21b62e9583968c',
                        'ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-baseline/ssh-exit.json',
                        'ssh_exit_sha256': '7b427e67e8d7f3e52e67476052ad738dac66a5ff24053734733a0f52cd9e5b38',
                        'ssh_exit_code': 0,
                        'ssh_process_exited': True},
                       {'result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-after-window/result.json',
                        'result_sha256': '832a28df6194a48e580db1dd239d7a689214a403e5b4bcfa75b33801fcf811a1',
                        'source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-after-window/remote-source.py',
                        'source_sha256': '312b8ded87bde54cc42d0c3830a4fa4f0acd6c904d25d30b1c21b62e9583968c',
                        'ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-after-window/ssh-exit.json',
                        'ssh_exit_sha256': '7b427e67e8d7f3e52e67476052ad738dac66a5ff24053734733a0f52cd9e5b38',
                        'ssh_exit_code': 0,
                        'ssh_process_exited': True}],
 'window_proof_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-window-proof.json',
 'window_proof_sha256': 'ba84d801e19c60e272296ff0a09cf1e734e4ffed3ca5acdc361fde81910f5818',
 'window_proof': {'stable_baseline': True,
                  'stable_after_window': True,
                  'metadata_identical_across_window': True,
                  'elapsed_seconds_from_capture_file_times': 96.47230386734009,
                  'same_kernel_bytes': True,
                  'same_serial_node': True,
                  'same_usb_device_number': True,
                  'serial_node': {'device': 5, 'inode': 651, 'rdev': 48128},
                  'usb_device_number': '5',
                  'raw_kernel_sha256': '01d3a85c796ce8bb7ba0987ca06e0cad3db4304fefbd76e3731ad2f6fd4f0e9f',
                  'uart_open_attempts': 0,
                  'servo_battery_disconnected': True,
                  'previous_usb_faults_cleared': False,
                  'root_cause_confirmed': False},
 'battery_confirmation_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/usb-battery-off-confirmed.json',
 'battery_confirmation_sha256': 'ece61ee7c06a12b95bef4ae7960dfbf738fa67eb3f8a06551a5d60b57a1440c3',
 'servo_battery_disconnected': True,
 'user_touched_wires': True,
 'user_com_movement_unknown': True,
 'serial_ownership_observations': 4,
 'serial_holders': [],
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'observed_serial_node': {'device': 5, 'inode': 651, 'rdev': 48128},
 'observed_usb_device_number': '5',
 'metadata_identical_across_window': True,
 'elapsed_seconds_from_capture_file_times': 96.47230386734009,
 'prior_raw_kernel_exact_prefix_retained': True,
 'retained_prior_line_count': 924,
 'new_kernel_lines': ['[  348.410547] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                      '[  348.417038] usb 1-1.4: USB disconnect, device number 3',
                      '[  348.418586] ch341-uart ttyUSB0: ch341-uart converter now disconnected '
                      'from ttyUSB0',
                      '[  348.418770] ch341 1-1.4:1.0: device disconnected',
                      '[  348.761067] usb 1-1.4: new full-speed USB device number 4 using xhci-hcd',
                      '[  348.912181] usb 1-1.4: New USB device found, idVendor=1a86, '
                      'idProduct=7523, bcdDevice= 2.64',
                      '[  348.912244] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                      'SerialNumber=0',
                      '[  348.912281] usb 1-1.4: Product: USB Serial',
                      '[  348.954648] ch341 1-1.4:1.0: ch341-uart converter detected',
                      '[  348.956189] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                      '[  386.038723] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                      '[  386.045233] usb 1-1.4: USB disconnect, device number 4',
                      '[  386.046835] ch341-uart ttyUSB0: ch341-uart converter now disconnected '
                      'from ttyUSB0',
                      '[  386.047019] ch341 1-1.4:1.0: device disconnected',
                      '[  386.381141] usb 1-1.4: new full-speed USB device number 5 using xhci-hcd',
                      '[  386.532119] usb 1-1.4: New USB device found, idVendor=1a86, '
                      'idProduct=7523, bcdDevice= 2.64',
                      '[  386.532183] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                      'SerialNumber=0',
                      '[  386.532219] usb 1-1.4: Product: USB Serial',
                      '[  386.582857] ch341 1-1.4:1.0: ch341-uart converter detected',
                      '[  386.584630] usb 1-1.4: ch341-uart converter now attached to ttyUSB0'],
 'reviewed_new_usb_events': [{'line': '[  348.417038] usb 1-1.4: USB disconnect, device number 3',
                              'reason': 'Kernel reported a USB disconnect/reset/error'},
                             {'line': '[  386.045233] usb 1-1.4: USB disconnect, device number 4',
                              'reason': 'Kernel reported a USB disconnect/reset/error'}],
 'quiescence_fault_refusal_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-03/quiescence-check/stdout.jsonl',
 'quiescence_fault_refusal_sha256': '99300b06de1ab53b34d0d79b4608cfee5c85248374100b0075337c03414fe500',
 'quiescence_new_fault_refusal_retained': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
 'observed_transport': 'wifi',
 'observed_target_host': '192.168.1.11',
 'serial_discovery_required': True,
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:224 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'raw_kernel_byte_count': 65925,
 'raw_kernel_cr_count': 30,
 'raw_kernel_preserved': True,
 'early_kernel_history_complete': True,
 'board_card_identity_unchanged': True,
 'independent_full_kernel_review_passed': True,
 'unresolved_historical_faults_retained': True,
 'previous_usb_faults_cleared': False,
 'prior_boot_faults_cleared': False,
 'filesystem_repaired': False,
 'unexpected_reboot_root_cause_confirmed': False,
 'usb_disconnect_electrical_cause_confirmed': False,
 'kernel_emi_hint_not_diagnosed_as_proven_interference': True,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'storage_health_certified': False,
 'diagnostic_uart_opened': False,
 'diagnostic_block_device_opened': False,
 'diagnostic_board_files_written': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'prior_motion_occupancy_cleared': False,
 'native_service_deployment_authorized': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False}
IMU_RECOVERY_USB_HISTORICAL_FAULTS = [{'line': '[    7.746460] systemd-journald[274]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'},
 {'line': '[  348.417038] usb 1-1.4: USB disconnect, device number 3',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[  386.045233] usb 1-1.4: USB disconnect, device number 4',
  'reason': 'Kernel reported a USB disconnect/reset/error'}]
IMU_RECOVERY_USB_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
 'kernel_prefix_sha256': '01d3a85c796ce8bb7ba0987ca06e0cad3db4304fefbd76e3731ad2f6fd4f0e9f',
 'kernel_line_count': 944}

# Exact user-confirmed intentional power cycle; no automatic new-boot acceptance.
INTENTIONAL_POWERCYCLE_BOOT_ID = 'c24863fc-a8eb-455d-a4f6-54633b6483e3'
INTENTIONAL_POWERCYCLE_PROFILE_SHA256 = 'cbc644070503c2c5099cdd94fc692e4d2cabbfb870b1f6f57df24363e84e8143'
INTENTIONAL_POWERCYCLE_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T02:45:31.014217+08:00',
 'human_context': '用户明确确认刚才主动断开 X5 电源，现已重新接官方电源，URT2 舵机电池保持断开。两份只读完整元数据逐字一致，主机密钥、SD '
                  'CID、根挂载、板卡型号和唯一 CH340 保持原身份，当前为明确固定的新冷启动。完整原始 924 行含最早启动历史，完整分类仅两条旧类 '
                  'journald/FAT 未正常卸载警告；没有本启动 USB 断开、SD/SDIO '
                  '新故障。板端未发现原生服务进程、RAM目录或8767监听，串口无占用。这些元数据不证明带电舵机状态或电气安全。此记录只手工接纳这一精确 boot '
                  '的反馈基线，不自动学习、不清除旧未解决 SD/USB/WiFi/IMU 或运动故障，也不授权使能、运动或持久写入。',
 'scope_detail': 'feedback-only exact same-board/card user-confirmed '
                 'intentional-powercycle-20261006; no motion or storage-write allowance',
 'user_confirmed_intentional_powercycle': True,
 'user_confirmed_x5_power_restored': True,
 'user_confirmed_servo_battery_disconnected': True,
 'prior_boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-usb-return-reviewed-20261006-01.json',
 'prior_reviewed_profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
 'prior_review_sha256': '5b65447a1728f9ff35116b94e3b4efa909e6bbf1fe3b6d583dfbba961027d1ea',
 'prior_review_retained_by_fixed_private_artifact': True,
 'prior_boot_historical_faults': [{'line': '[    7.746460] systemd-journald[274]: File '
                                           '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                           'corrupted or uncleanly shut down, renaming and '
                                           'replacing.',
                                   'reason': 'Journald reported a corrupt or uncleanly closed '
                                             'journal file'},
                                  {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not '
                                           'properly unmounted. Some data may be corrupt. Please '
                                           'run fsck.',
                                   'reason': 'Kernel reported an unclean FAT volume or filesystem '
                                             'fault'},
                                  {'line': '[  348.417038] usb 1-1.4: USB disconnect, device '
                                           'number 3',
                                   'reason': 'Kernel reported a USB disconnect/reset/error'},
                                  {'line': '[  386.045233] usb 1-1.4: USB disconnect, device '
                                           'number 4',
                                   'reason': 'Kernel reported a USB disconnect/reset/error'}],
 'all_prior_boot_historical_faults': [{'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-usb-return-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
                                       'historical_faults': [{'line': '[    7.746460] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.748613] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[  348.417038] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[  386.045233] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                                       'historical_faults': [{'line': '[    7.647480] '
                                                                      'systemd-journald[276]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.707257] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                                       'historical_faults': [{'line': '[    7.379408] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.469697] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[36903.070883] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                       'historical_faults': [{'line': '[    7.293643] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.321881] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 5127.470586] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[68102.480531] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[69821.346890] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 5',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[71514.103758] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 6',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[73274.437850] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 7',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[78166.615277] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 8',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[79843.245789] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 9',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82677.647322] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 10',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82987.887775] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 11',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83012.205481] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 12',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83245.141873] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 13',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-reconnect-reviewed-20261004-02.json',
                                       'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                       'historical_faults': [{'line': '[    7.573470] '
                                                                      'systemd-journald[273]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.944418] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 3008.252350] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691466] usb 1-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691521] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.931957] usb 2-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8544.480306] '
                                                                      'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                      'sdio wakeup fail',
                                                              'reason': 'Kernel reported an SD/MMC '
                                                                        'failure'},
                                                             {'line': '[ 9890.432382] '
                                                                      'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                      'ret=-110, intstatus=ff',
                                                              'reason': 'Feedback guard reported '
                                                                        'an AIC SDIO WiFi driver '
                                                                        'failure'}],
                                       'prior_faults_cleared': False}],
 'all_prior_fault_events_count': 30,
 'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-04/current-boot-metadata/result.json',
 'metadata_result_sha256': '6aa0e518f76bc82f85f077b933b2467e933d353d53b1692fefa3584ab6903861',
 'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-04/current-boot-metadata/remote-source.py',
 'metadata_source_sha256': 'c08ceed1d1b75f0f11643eeb350ec267dc20a267212b2eab635e3d2910e6f46a',
 'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-04/current-boot-metadata/ssh-exit.json',
 'metadata_ssh_exit_sha256': '7b427e67e8d7f3e52e67476052ad738dac66a5ff24053734733a0f52cd9e5b38',
 'ssh_exit_code': 0,
 'ssh_process_exited': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg',
 'observed_transport': 'wifi',
 'observed_target_host': '192.168.1.11',
 'stable_metadata_observations': 2,
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'serial_discovery_required': True,
 'serial_holders': [],
 'observed_serial_node': {'device': 5, 'inode': 307, 'rdev': 48128},
 'observed_usb_device_number': '3',
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'native_processes': [],
 'native_ram_directories': [],
 'listeners_8767': [],
 'raw_kernel_byte_count': 64529,
 'raw_kernel_cr_count': 30,
 'raw_kernel_preserved': True,
 'early_kernel_history_complete': True,
 'observed_boot_reason': 'COLD_BOOT',
 'independent_full_kernel_review_passed': True,
 'board_card_identity_unchanged': True,
 'unresolved_historical_faults_retained': True,
 'prior_boot_faults_cleared': False,
 'previous_usb_faults_cleared': False,
 'filesystem_repaired': False,
 'prior_unexpected_reboot_root_cause_confirmed': False,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'storage_health_certified': False,
 'powered_servo_health_certified': False,
 'diagnostic_uart_opened': False,
 'diagnostic_block_device_opened': False,
 'diagnostic_board_files_written': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False}
INTENTIONAL_POWERCYCLE_HISTORICAL_FAULTS = [{'line': '[    7.385520] systemd-journald[275]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]
INTENTIONAL_POWERCYCLE_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': 'cbc644070503c2c5099cdd94fc692e4d2cabbfb870b1f6f57df24363e84e8143',
 'kernel_prefix_sha256': '31cadf6fbc9e50742b35f18f4731be25f25c7b76a22f7613878c6d9e408f66a8',
 'kernel_line_count': 924}

# Exact current-boot feedback review of an uncommanded USB loss/return.
# This immutable variant records no human physical reconnect and grants no motion.
NATIVE_USB_RETURN_BOOT_ID = 'c24863fc-a8eb-455d-a4f6-54633b6483e3'
NATIVE_USB_RETURN_PROFILE_SHA256 = '6cbc2bc51cb2a7321b4baa890a8198c1423a0c4ebb21cf7319e4c5e0c1bdc1d8'
NATIVE_USB_RETURN_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T03:35:19.290627+08:00',
 'human_context': '用户要求恢复 X5 原生工作台遗漏的采集与校准流程。用户明确表示未碰 COM 线、URT2 舵机电池仍断开；本次 USB 掉线不是用户确认的主动拔插。07 '
                  '的未打开串口诊断显示新 USB 故障；随后两次相隔 0.3 秒的只读元数据完全一致，当前启动、板卡、卡和根挂载未变。完整原始内核日志逐字保留前 924 '
                  '行，新增 10 行记载 hub disabled by hub (EMI?)、CH340 设备 3 断开及同端口设备 4 返回。EMI? '
                  '是内核提示，不证明电气干扰或 MCU 复位；掉线原因仍未确认。只为当前启动的一次显式反馈重连建立精确历史，不清除任何故障、不认证硬件健康。',
 'scope_detail': 'Fixed current-boot feedback-only reviewed reconnect; HELLO/STATE only. Root will '
                 'explicitly read IMU with servo battery disconnected. No RESET, ARM, TARGET, '
                 'automatic reconnect, fault clearance or motion authorization.',
 'user_reported_com_not_touched': True,
 'user_reported_servo_battery_disconnected': True,
 'user_confirmed_reconnect': False,
 'user_confirmed_intentional_usb_disconnect': False,
 'physical_reconnect_not_claimed': True,
 'explicit_feedback_read_task_authorization_retained': True,
 'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-07/usb-return-metadata/proof.json',
 'metadata_result_sha256': 'b8037cfeda1afdb63fc747e38f27db9667bd9f3845ff462848b6805c598b6baf',
 'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-07/usb-return-metadata/remote-source.py',
 'metadata_source_sha256': '382df57bc55423dceb412cfae4e085de5c80fed728a83502f372b74675921b4f',
 'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-07/usb-return-metadata/ssh-exit.json',
 'metadata_ssh_exit_sha256': 'a660b075a3347eeecd70f98f80be206e20298430268640d931127584947be519',
 'metadata_kind': 'native_usb_return_metadata_only',
 'metadata_generation': 1791228327,
 'metadata_observation_count': 2,
 'metadata_observations_identical': True,
 'metadata_fixed_observation_interval_seconds': 0.3,
 'metadata_serial_open_attempts': 0,
 'metadata_serial_bytes_written': 0,
 'metadata_board_files_written': False,
 'metadata_guard_permission_granted': False,
 'blocked_diagnostic_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-07/fault-blocked-diagnostics/proof.json',
 'blocked_diagnostic_sha256': '24d19868f7f9c4496424730875b5d75c0e5af8d3566a24242c8b5303c08754f8',
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'observed_serial_node': {'device': 5, 'inode': 640, 'rdev': 48128},
 'observed_serial_holders': [],
 'observed_usb_device_number': '4',
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'raw_kernel_prefix_sha256': '446afca6b270b70792f41981099aba26345335c056530f404dc6990a3e2166c7',
 'raw_kernel_bytes': 65227,
 'raw_kernel_line_count': 934,
 'raw_kernel_carriage_returns': 30,
 'raw_kernel_previous_prefix_retained': True,
 'new_kernel_tail_lines': ['[ 2860.546313] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
                           '[ 2860.554441] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 2860.554627] ch341 1-1.4:1.0: device disconnected',
                           '[ 2860.897793] usb 1-1.4: new full-speed USB device number 4 using '
                           'xhci-hcd',
                           '[ 2861.048920] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 2861.048982] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 2861.049019] usb 1-1.4: Product: USB Serial',
                           '[ 2861.091629] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 2861.094977] usb 1-1.4: ch341-uart converter now attached to '
                           'ttyUSB0'],
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-intentional-powercycle-reviewed-20261006-01.json',
 'prior_reviewed_profile_sha256': 'cbc644070503c2c5099cdd94fc692e4d2cabbfb870b1f6f57df24363e84e8143',
 'prior_review_sha256': '9fec21c22ed00253207ab66bf259528e7e33ecd3204481f77845aa8024e68eaa',
 'prior_review_retained_by_fixed_private_artifact': True,
 'prior_current_boot_historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File '
                                                   '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                   'corrupted or uncleanly shut down, renaming and '
                                                   'replacing.',
                                           'reason': 'Journald reported a corrupt or uncleanly '
                                                     'closed journal file'},
                                          {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was '
                                                   'not properly unmounted. Some data may be '
                                                   'corrupt. Please run fsck.',
                                           'reason': 'Kernel reported an unclean FAT volume or '
                                                     'filesystem fault'}],
 'prior_current_boot_fault_events_count': 2,
 'all_prior_boot_historical_faults': [{'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-usb-return-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
                                       'historical_faults': [{'line': '[    7.746460] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.748613] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[  348.417038] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[  386.045233] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                                       'historical_faults': [{'line': '[    7.647480] '
                                                                      'systemd-journald[276]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.707257] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                                       'historical_faults': [{'line': '[    7.379408] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.469697] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[36903.070883] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                       'historical_faults': [{'line': '[    7.293643] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.321881] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 5127.470586] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[68102.480531] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[69821.346890] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 5',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[71514.103758] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 6',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[73274.437850] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 7',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[78166.615277] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 8',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[79843.245789] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 9',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82677.647322] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 10',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82987.887775] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 11',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83012.205481] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 12',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83245.141873] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 13',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-reconnect-reviewed-20261004-02.json',
                                       'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                       'historical_faults': [{'line': '[    7.573470] '
                                                                      'systemd-journald[273]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.944418] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 3008.252350] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691466] usb 1-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691521] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.931957] usb 2-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8544.480306] '
                                                                      'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                      'sdio wakeup fail',
                                                              'reason': 'Kernel reported an SD/MMC '
                                                                        'failure'},
                                                             {'line': '[ 9890.432382] '
                                                                      'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                      'ret=-110, intstatus=ff',
                                                              'reason': 'Feedback guard reported '
                                                                        'an AIC SDIO WiFi driver '
                                                                        'failure'}],
                                       'prior_faults_cleared': False}],
 'all_prior_fault_events_count': 30,
 'current_boot_historical_faults_count': 3,
 'new_unresolved_usb_event': {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
                              'reason': 'Kernel reported a USB disconnect/reset/error'},
 'new_usb_fault_cause_confirmed': False,
 'usb_link_repaired': False,
 'emi_confirmed': False,
 'mcu_reset_confirmed': False,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'powered_servo_health_certified': False,
 'prior_boot_faults_cleared': False,
 'current_boot_faults_cleared': False,
 'previous_usb_faults_cleared': False,
 'filesystem_repaired': False,
 'prior_unexpected_reboot_root_cause_confirmed': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'servo_arm_authorized': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False,
 'board_card_identity_unchanged': True,
 'early_kernel_history_complete': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg'}
NATIVE_USB_RETURN_HISTORICAL_FAULTS = [{'line': '[    7.385520] systemd-journald[275]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'},
 {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
  'reason': 'Kernel reported a USB disconnect/reset/error'}]
NATIVE_USB_RETURN_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': '6cbc2bc51cb2a7321b4baa890a8198c1423a0c4ebb21cf7319e4c5e0c1bdc1d8',
 'kernel_prefix_sha256': '446afca6b270b70792f41981099aba26345335c056530f404dc6990a3e2166c7',
 'kernel_line_count': 934}

# Exact second uncommanded USB return; first review and all fault history remain retained.
NATIVE_USB_SECOND_RETURN_BOOT_ID = 'c24863fc-a8eb-455d-a4f6-54633b6483e3'
NATIVE_USB_SECOND_RETURN_PROFILE_SHA256 = 'a0e2322015e34108b9b879d749f16ef07dc69b3e42e43468d315e5e8e23298f4'
NATIVE_USB_SECOND_RETURN_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T04:08:17.541943+08:00',
 'human_context': '用户要求迁回遗漏的 X5 原生采集与校准功能。首次未主动触碰 COM 的 USB 断开及返回已有独立固定记录，本次仍不声明用户主动拔插。Root '
                  '复核第二次同端口 CH340 设备 4 断开并以设备 5 返回的精确事件；当前 URT2 电池保持断开。11 的只读元数据经实际 SSH exit0 '
                  '完成，三份相隔 0.3 秒、跨越 0.6 秒的完整观察完全一致，板卡、启动、SD CID、根挂载未变，唯一 CH340 1-1.4 /dev/ttyUSB0 '
                  '节点 inode651 无占用。完整原始内核日志逐字保留首次固定记录的前 934 行，新增 10 行构成第二次同口断开及返回，四项当前故障全部保留。hub 的 '
                  'EMI? 提示仍不证明干扰、复位或电气根因，稳定观察不认证硬件健康、修复连接或清除故障。只授权下一次显式电池断开条件下的 HELLO/STATE '
                  '反馈复查；不自动重连、不使能、不接受其他新故障。',
 'scope_detail': 'Fixed exact second same-boot CH340 return; feedback-only HELLO/STATE with servo '
                 'battery disconnected. No generic kernel exemption, dynamic baseline learning, '
                 'RESET, ARM, TARGET, automatic retry, fault clearance or motion authorization.',
 'user_confirmed_reconnect': False,
 'second_usb_disconnect_user_initiated_confirmed': False,
 'physical_reconnect_not_claimed': True,
 'prior_no_com_touch_report_retained': True,
 'current_servo_battery_disconnected': True,
 'explicit_feedback_read_task_authorization_retained': True,
 'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-11/current-usb-metadata-01/proof.json',
 'metadata_result_sha256': '00672f94391e9e1f130fc5d0424606e01956f6da0c81861263c5666eed3378b0',
 'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-11/current-usb-metadata-01/remote-source.py',
 'metadata_source_sha256': '49a2e120ea53b39fd6862d49586e507ffa0012877dda236756bbe67299472d25',
 'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-11/current-usb-metadata-01/ssh-exit.json',
 'metadata_ssh_exit_sha256': 'cb812f7baf443a28c230dd7a4bc3a9d50c1c8b2bd1c5d39a9cd5b4e50731499b',
 'metadata_kind': 'current_usb_metadata_only',
 'metadata_observation_count': 3,
 'metadata_observations_identical': True,
 'metadata_fixed_observation_interval_seconds': 0.3,
 'metadata_fixed_observation_span_seconds': 0.6,
 'metadata_serial_open_attempts': 0,
 'metadata_serial_bytes_written': 0,
 'metadata_board_files_written': False,
 'metadata_guard_permission_granted': False,
 'prior_strict_refusal_retained': 'New kernel fault; classifier result requires source review: '
                                  '{"new_fault_count":1,"faults":[{"source":"new_kernel_tail","kernel_line_number":936,"line":"[ '
                                  '3469.515263] usb 1-1.4: USB disconnect, device number '
                                  '4","line_truncated":false,"classifier_reason":"Kernel reported '
                                  'a USB '
                                  'disconnect/reset/error","context":[{"kernel_line_number":935,"line":"[ '
                                  '3469.508801] usb 1-1-port4: disabled by hub (EMI?), '
                                  're-enabling...","line_truncated":false},{"kernel_line_number":936,"line":"[ '
                                  '3469.515263] usb 1-1.4: USB disconnect, device number '
                                  '4","line_truncated":false},{"kernel_line_number":937,"line":"[ '
                                  '3469.521101] ch341-uart ttyUSB0: ch341-uart converter now '
                                  'disconnected from '
                                  'ttyUSB0","line_truncated":false},{"kernel_line_number":938,"line":"[ '
                                  '3469.521208] ch341 1-1.4:1.0: device '
                                  'disconnected","line_truncated":false}]}],"fault_details_truncated":false}',
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'observed_serial_node': {'device': 5, 'inode': 651, 'rdev': 48128},
 'observed_serial_holders': [],
 'observed_usb_device_number': '5',
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'raw_kernel_prefix_sha256': '5b4a13362166637c829f9dc7bb9dd11225b0e0ce82f3a10ed387dabc149d82aa',
 'raw_kernel_bytes': 65925,
 'raw_kernel_line_count': 944,
 'raw_kernel_carriage_returns': 30,
 'raw_kernel_previous_prefix_retained': True,
 'new_kernel_tail_lines': ['[ 3469.508801] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4',
                           '[ 3469.521101] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 3469.521208] ch341 1-1.4:1.0: device disconnected',
                           '[ 3469.868014] usb 1-1.4: new full-speed USB device number 5 using '
                           'xhci-hcd',
                           '[ 3470.019223] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 3470.019286] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 3470.019323] usb 1-1.4: Product: USB Serial',
                           '[ 3470.084845] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 3470.086493] usb 1-1.4: ch341-uart converter now attached to '
                           'ttyUSB0'],
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-native-uncommanded-usb-return-reviewed-20261006-01.json',
 'prior_reviewed_profile_sha256': '6cbc2bc51cb2a7321b4baa890a8198c1423a0c4ebb21cf7319e4c5e0c1bdc1d8',
 'prior_review_sha256': '0e92a0b8533ef2d83e0bbc293a6a1331046597e1d64a509c9a6cbdc0e2da876f',
 'prior_review_retained_by_fixed_private_artifact': True,
 'prior_current_boot_historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File '
                                                   '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                   'corrupted or uncleanly shut down, renaming and '
                                                   'replacing.',
                                           'reason': 'Journald reported a corrupt or uncleanly '
                                                     'closed journal file'},
                                          {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was '
                                                   'not properly unmounted. Some data may be '
                                                   'corrupt. Please run fsck.',
                                           'reason': 'Kernel reported an unclean FAT volume or '
                                                     'filesystem fault'},
                                          {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, '
                                                   'device number 3',
                                           'reason': 'Kernel reported a USB '
                                                     'disconnect/reset/error'}],
 'prior_current_boot_fault_events_count': 3,
 'all_prior_boot_historical_faults': [{'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-usb-return-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
                                       'historical_faults': [{'line': '[    7.746460] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.748613] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[  348.417038] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[  386.045233] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                                       'historical_faults': [{'line': '[    7.647480] '
                                                                      'systemd-journald[276]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.707257] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                                       'historical_faults': [{'line': '[    7.379408] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.469697] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[36903.070883] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                       'historical_faults': [{'line': '[    7.293643] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.321881] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 5127.470586] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[68102.480531] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[69821.346890] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 5',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[71514.103758] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 6',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[73274.437850] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 7',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[78166.615277] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 8',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[79843.245789] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 9',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82677.647322] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 10',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82987.887775] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 11',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83012.205481] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 12',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83245.141873] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 13',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-reconnect-reviewed-20261004-02.json',
                                       'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                       'historical_faults': [{'line': '[    7.573470] '
                                                                      'systemd-journald[273]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.944418] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 3008.252350] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691466] usb 1-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691521] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.931957] usb 2-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8544.480306] '
                                                                      'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                      'sdio wakeup fail',
                                                              'reason': 'Kernel reported an SD/MMC '
                                                                        'failure'},
                                                             {'line': '[ 9890.432382] '
                                                                      'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                      'ret=-110, intstatus=ff',
                                                              'reason': 'Feedback guard reported '
                                                                        'an AIC SDIO WiFi driver '
                                                                        'failure'}],
                                       'prior_faults_cleared': False}],
 'all_prior_fault_events_count': 30,
 'current_boot_historical_faults_count': 4,
 'new_unresolved_usb_event': {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4',
                              'reason': 'Kernel reported a USB disconnect/reset/error'},
 'new_usb_fault_cause_confirmed': False,
 'usb_link_repaired': False,
 'emi_confirmed': False,
 'mcu_reset_confirmed': False,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'powered_servo_health_certified': False,
 'prior_boot_faults_cleared': False,
 'current_boot_faults_cleared': False,
 'previous_usb_faults_cleared': False,
 'filesystem_repaired': False,
 'prior_unexpected_reboot_root_cause_confirmed': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'servo_arm_authorized': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False,
 'generic_kernel_exemption_authorized': False,
 'dynamic_baseline_learning_authorized': False,
 'board_card_identity_unchanged': True,
 'early_kernel_history_complete': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg'}
NATIVE_USB_SECOND_RETURN_HISTORICAL_FAULTS = [{'line': '[    7.385520] systemd-journald[275]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'},
 {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4',
  'reason': 'Kernel reported a USB disconnect/reset/error'}]
NATIVE_USB_SECOND_RETURN_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': 'a0e2322015e34108b9b879d749f16ef07dc69b3e42e43468d315e5e8e23298f4',
 'kernel_prefix_sha256': '5b4a13362166637c829f9dc7bb9dd11225b0e0ce82f3a10ed387dabc149d82aa',
 'kernel_line_count': 944}

# Exact third review: only these 36 nonfault evictions and six completed USB returns.
NATIVE_USB_THIRD_RETURN_BOOT_ID = 'c24863fc-a8eb-455d-a4f6-54633b6483e3'
NATIVE_USB_THIRD_RETURN_PROFILE_SHA256 = 'aa40ce44e09ba1bd677dc5c133d3442d4065a0050a653a03136aeea9bb219f3f'
NATIVE_USB_THIRD_RETURN_REQUIRED_REVIEW = {'reviewer': 'root',
 'reviewed_at': '2026-10-06T05:07:54.334153+08:00',
 'human_context': '用户要求保留在同一运控卡中查看电池断开时的实时 IMU 姿态；URT2 电池仍断开，之前未碰 COM 的报告保持，不声明这些掉线由用户主动拔插。部署 13 '
                  '的显式 purpose=imu HTTP 复查按原 02 固定保护拒绝，未打开 UART；Root 随后读取三份相隔 0.30 秒、跨越 0.60 '
                  '秒的完整当前元数据，实际 SSH exit0，三份观察逐字相同。同启动、SD CID、根挂载、板卡型号及唯一 USB1-1.4 CH340 身份未变，当前 '
                  '/dev/ttyUSB0 节点 inode717、USB设备号11、无占用。当前 968 行原始日志与 02 的第 37–944 行逐字重叠，共 908 '
                  '行；已消失的最初 36 行逐条核对均非故障，其完整原文和原 944 行文件通过固定 SHA 保留。当前日志仍从启动时间 0 '
                  '开始，但不声明旧完整开头仍在当前环形日志中。新增 60 行是六次设备 5→6→7→8→9→10→11 的同口 CH340 '
                  '断开及返回，完整十项当前故障全部保留未解决。Root 逐行复核全部 60 行；EMI? '
                  '仍仅内核提示，不证明电气根因、复位、修复或硬件健康。此记录只为精确当前原始日志建立一次显式 HELLO/STATE '
                  '只读复查，继续拒绝任何新错误、进一步历史丢失或身份改变，不使能、不自动重连、不授权运动。',
 'scope_detail': 'Exact fixed current-boot feedback-only review after 36 proven nonfault initial '
                 'lines disappeared and six completed CH340 returns. Battery-off HELLO/STATE only; '
                 'no generic prefix-loss exemption, dynamic baseline learning, RESET, ARM, TARGET, '
                 'retry, fault clearance or hardware-health certification.',
 'user_confirmed_reconnect': False,
 'physical_reconnect_not_claimed': True,
 'prior_no_com_touch_report_retained': True,
 'current_servo_battery_reported_disconnected': True,
 'explicit_feedback_read_task_authorization_retained': True,
 'metadata_result_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/current-kernel-metadata/stdout.jsonl',
 'metadata_result_sha256': 'f6f04d5a947510c5b2b0963cf485e9c228d2a94e0c1109021e9956ce40e6a4ba',
 'metadata_source_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/current-kernel-metadata/remote-source.py',
 'metadata_source_sha256': '6e5b31e3b1ebf483b6d1013f84c5e707d3ab30f9662096cdbeb4e91ead13f43e',
 'metadata_ssh_exit_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/current-kernel-metadata/ssh-exit.json',
 'metadata_ssh_exit_sha256': 'a660b075a3347eeecd70f98f80be206e20298430268640d931127584947be519',
 'metadata_comparison_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/current-kernel-metadata/comparison.json',
 'metadata_comparison_sha256': 'd5e817af741a4e7d756e20f3ee29c530c3ee4fad4376fafab729a2ecda280b3e',
 'metadata_kind': 'native_pose_refusal_metadata',
 'metadata_observation_count': 3,
 'metadata_observations_identical': True,
 'metadata_fixed_observation_interval_seconds': 0.3,
 'metadata_fixed_observation_span_seconds': 0.6,
 'metadata_serial_open_attempts': 0,
 'metadata_serial_bytes_written': 0,
 'metadata_remote_files_written': False,
 'metadata_motor_start_attempted': False,
 'metadata_automatic_retry': False,
 'failed_http_imu_proof_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/live-imu-pose-check/proof.json',
 'failed_http_imu_proof_sha256': '28618083558c47c64dab79b59b3d5b7978c1b3a3c33550a9dae7846a85ccf78b',
 'failed_http_imu_request_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/live-imu-pose-check/request.json',
 'failed_http_imu_request_sha256': '505b1bca728f54c20b28b67c03a8f3b7a378c84ec79772ab64fc4b5281702474',
 'failed_http_imu_final_status_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-13/live-imu-pose-check/status-01.json',
 'failed_http_imu_final_status_sha256': '2591df410e41919b55a3db725c8dd78eb5e63cb6c510caba238de618978e9118',
 'prior_strict_refusal_retained': 'Retained kernel history changed or lost its reviewed prefix',
 'observed_serial': {'path': '/dev/ttyUSB0',
                     'sysfs_name': '1-1.4',
                     'vid': '1a86',
                     'pid': '7523',
                     'serial_number': ''},
 'observed_serial_node': {'device': 5, 'inode': 717, 'rdev': 48128},
 'observed_serial_holders': [],
 'observed_usb_device_number': '11',
 'observed_config_mount': ['425 21 179:1 / /boot/config rw,relatime shared:189 - vfat '
                           '/dev/mmcblk1p1 '
                           'rw,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro'],
 'observed_shm_mount': ['26 22 0:22 / /dev/shm rw,nosuid,nodev shared:3 - tmpfs tmpfs rw'],
 'raw_kernel_prefix_sha256': 'e8ff7b0b3a9456eb1a7c37fe1b73a2e8f10c49f874ae4c589788e07ecc924d48',
 'raw_kernel_bytes': 67313,
 'raw_kernel_line_count': 968,
 'raw_kernel_carriage_returns': 30,
 'evicted_nonfault_lines': ['[    0.000000] Booting Linux on physical CPU 0x0000000000 '
                            '[0x412fd050]',
                            '[    0.000000] Linux version 6.1.83 (hobot@Dpx-PF4PF7V1) '
                            '(aarch64-none-linux-gnu-gcc (GNU Toolchain for the Arm Architecture '
                            '11.2-2022.02 (arm-11.14)) 11.2.1 20220111, GNU ld (GNU Toolchain for '
                            'the Arm Architecture 11.2-2022.02 (arm-11.14)) 2.37.20220122) #1 SMP '
                            'PREEMPT Wed Apr  8 21:09:59 CST 2026',
                            '[    0.000000] Machine model: D-Robotics RDK X5 V1.0',
                            '[    0.000000] Reserved memory: created CMA memory pool at '
                            '0x0000000088000000, size 382 MiB',
                            '[    0.000000] OF: reserved mem: initialized node linux,cma@88000000, '
                            'compatible id shared-dma-pool',
                            '[    0.000000] Zone ranges:',
                            '[    0.000000]   DMA      [mem 0x0000000084000000-0x00000000ffffffff]',
                            '[    0.000000]   DMA32    empty',
                            '[    0.000000]   Normal   [mem 0x0000000100000000-0x000000027fffffff]',
                            '[    0.000000] Movable zone start for each node',
                            '[    0.000000] Early memory node ranges',
                            '[    0.000000]   node   0: [mem '
                            '0x0000000084000000-0x000000009fe7efff]',
                            '[    0.000000]   node   0: [mem '
                            '0x000000009fe7f000-0x00000000a207ffff]',
                            '[    0.000000]   node   0: [mem '
                            '0x00000000a2080000-0x000000027fffffff]',
                            '[    0.000000] Initmem setup node 0 [mem '
                            '0x0000000084000000-0x000000027fffffff]',
                            '[    0.000000] On node 0, zone DMA: 16384 pages in unavailable ranges',
                            '[    0.000000] psci: probing for conduit method from DT.',
                            '[    0.000000] psci: PSCIv1.1 detected in firmware.',
                            '[    0.000000] psci: Using standard PSCI v0.2 function IDs',
                            '[    0.000000] psci: Trusted OS migration not required',
                            '[    0.000000] psci: SMC Calling Convention v1.2',
                            '[    0.000000] percpu: Embedded 23 pages/cpu s53880 r8192 d32136 '
                            'u94208',
                            '[    0.000000] pcpu-alloc: s53880 r8192 d32136 u94208 alloc=23*4096',
                            '[    0.000000] pcpu-alloc: [0] 0 [0] 1 [0] 2 [0] 3 [0] 4 [0] 5 [0] 6 '
                            '[0] 7 ',
                            '[    0.000000] Detected VIPT I-cache on CPU0',
                            '[    0.000000] CPU features: detected: GIC system register CPU '
                            'interface',
                            '[    0.000000] CPU features: detected: Virtualization Host Extensions',
                            '[    0.000000] CPU features: kernel page table isolation disabled by '
                            'kernel configuration',
                            '[    0.000000] CPU features: detected: Qualcomm erratum 1009, or ARM '
                            'erratum 1286807, 2441009',
                            '[    0.000000] CPU features: detected: ARM errata 1165522, 1319367, '
                            'or 1530923',
                            '[    0.000000] alternatives: applying boot alternatives',
                            '[    0.000000] Built 1 zonelists, mobility grouping on.  Total pages: '
                            '2048256',
                            '[    0.000000] Kernel command line: console=tty1 console=ttyS0,115200 '
                            'rootfstype=ext4 rw rootwait root=/dev/mmcblk1p2 '
                            'mtdparts=spi7.0:0x700000@0x0(miniboot),0x180000@0x700000(ubootenv) '
                            'hobotboot.reason=COLD_BOOT',
                            '[    0.000000] Dentry cache hash table entries: 1048576 (order: 11, '
                            '8388608 bytes, linear)',
                            '[    0.000000] Inode-cache hash table entries: 524288 (order: 10, '
                            '4194304 bytes, linear)',
                            '[    0.000000] mem auto-init: stack:off, heap alloc:off, heap '
                            'free:off'],
 'evicted_nonfault_line_count': 36,
 'evicted_nonfault_raw_prefix_sha256': 'e1233623953949e4624372d776fcf93a62e49fd4430f7c9f1f1792f7eb164e9f',
 'evicted_nonfault_raw_prefix_bytes': 2803,
 'evicted_fault_lines_count': 0,
 'retained_prior_line_count': 908,
 'retained_prior_raw_suffix_sha256': '2c09fe31394ac87dfb8b540717bccd6009cab4519d10b84c4d86f2a140feecfb',
 'retained_prior_raw_suffix_bytes': 63122,
 'new_kernel_tail_lines': ['[ 6976.866123] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5',
                           '[ 6976.878388] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 6976.878639] ch341 1-1.4:1.0: device disconnected',
                           '[ 6977.209225] usb 1-1.4: new full-speed USB device number 6 using '
                           'xhci-hcd',
                           '[ 6977.359990] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 6977.360011] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 6977.360018] usb 1-1.4: Product: USB Serial',
                           '[ 6977.409888] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 6977.411019] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                           '[ 7006.559143] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6',
                           '[ 7006.565904] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 7006.565951] ch341 1-1.4:1.0: device disconnected',
                           '[ 7006.909229] usb 1-1.4: new full-speed USB device number 7 using '
                           'xhci-hcd',
                           '[ 7007.060013] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 7007.060035] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 7007.060042] usb 1-1.4: Product: USB Serial',
                           '[ 7007.102881] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 7007.103500] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                           '[ 7019.613901] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7',
                           '[ 7019.622022] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 7019.622197] ch341 1-1.4:1.0: device disconnected',
                           '[ 7019.959221] usb 1-1.4: new full-speed USB device number 8 using '
                           'xhci-hcd',
                           '[ 7020.110007] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 7020.110031] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 7020.110040] usb 1-1.4: Product: USB Serial',
                           '[ 7020.157570] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 7020.158196] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                           '[ 7086.935084] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8',
                           '[ 7086.943205] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 7086.943381] ch341 1-1.4:1.0: device disconnected',
                           '[ 7087.279201] usb 1-1.4: new full-speed USB device number 9 using '
                           'xhci-hcd',
                           '[ 7087.430186] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 7087.430251] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 7087.430288] usb 1-1.4: Product: USB Serial',
                           '[ 7087.478751] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 7087.480383] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                           '[ 7089.494722] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9',
                           '[ 7089.501554] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 7089.501614] ch341 1-1.4:1.0: device disconnected',
                           '[ 7089.859178] usb 1-1.4: new full-speed USB device number 10 using '
                           'xhci-hcd',
                           '[ 7090.009924] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 7090.009945] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 7090.009953] usb 1-1.4: Product: USB Serial',
                           '[ 7090.070504] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 7090.071123] usb 1-1.4: ch341-uart converter now attached to ttyUSB0',
                           '[ 7149.904722] usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
                           '[ 7149.911226] usb 1-1.4: USB disconnect, device number 10',
                           '[ 7149.912845] ch341-uart ttyUSB0: ch341-uart converter now '
                           'disconnected from ttyUSB0',
                           '[ 7149.913022] ch341 1-1.4:1.0: device disconnected',
                           '[ 7150.249098] usb 1-1.4: new full-speed USB device number 11 using '
                           'xhci-hcd',
                           '[ 7150.400219] usb 1-1.4: New USB device found, idVendor=1a86, '
                           'idProduct=7523, bcdDevice= 2.64',
                           '[ 7150.400280] usb 1-1.4: New USB device strings: Mfr=0, Product=2, '
                           'SerialNumber=0',
                           '[ 7150.400319] usb 1-1.4: Product: USB Serial',
                           '[ 7150.448825] ch341 1-1.4:1.0: ch341-uart converter detected',
                           '[ 7150.450579] usb 1-1.4: ch341-uart converter now attached to '
                           'ttyUSB0'],
 'new_completed_usb_cycles': 6,
 'current_original_kernel_prefix_complete': False,
 'current_early_timestamp_retained': True,
 'original_full_kernel_retained_by_fixed_private_artifact': True,
 'prior_reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-native-uncommanded-usb-return-reviewed-20261006-02.json',
 'prior_reviewed_profile_sha256': 'a0e2322015e34108b9b879d749f16ef07dc69b3e42e43468d315e5e8e23298f4',
 'prior_review_sha256': 'e54b4cb7f54f00e3a3676b727354d600dfdc0aa6d432c7b8d8678a60d6b86933',
 'prior_review_retained_by_fixed_private_artifact': True,
 'prior_current_boot_historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File '
                                                   '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                   'corrupted or uncleanly shut down, renaming and '
                                                   'replacing.',
                                           'reason': 'Journald reported a corrupt or uncleanly '
                                                     'closed journal file'},
                                          {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was '
                                                   'not properly unmounted. Some data may be '
                                                   'corrupt. Please run fsck.',
                                           'reason': 'Kernel reported an unclean FAT volume or '
                                                     'filesystem fault'},
                                          {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, '
                                                   'device number 3',
                                           'reason': 'Kernel reported a USB '
                                                     'disconnect/reset/error'},
                                          {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, '
                                                   'device number 4',
                                           'reason': 'Kernel reported a USB '
                                                     'disconnect/reset/error'}],
 'prior_current_boot_fault_events_count': 4,
 'all_prior_boot_historical_faults': [{'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-imu-recovery-usb-return-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca',
                                       'historical_faults': [{'line': '[    7.746460] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.748613] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[  348.417038] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[  386.045233] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-wifi-reboot-reviewed-20261006-01.json',
                                       'reviewed_profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0',
                                       'historical_faults': [{'line': '[    7.647480] '
                                                                      'systemd-journald[276]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.707257] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-drop-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9',
                                       'historical_faults': [{'line': '[    7.379408] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.469697] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[36903.070883] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-usb-port-move-reviewed-20261005-01.json',
                                       'reviewed_profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c',
                                       'historical_faults': [{'line': '[    7.293643] '
                                                                      'systemd-journald[274]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.321881] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 5127.470586] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[68102.480531] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[69821.346890] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 5',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[71514.103758] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 6',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[73274.437850] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 7',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[78166.615277] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 8',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[79843.245789] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 9',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82677.647322] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 10',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[82987.887775] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 11',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83012.205481] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 12',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[83245.141873] usb 1-1.4: '
                                                                      'USB disconnect, device '
                                                                      'number 13',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'}],
                                       'prior_faults_cleared': False},
                                      {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab',
                                       'reviewed_profile_path': '/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/replacement-board-20261004T011959+0800/feedback-reconnect-reviewed-20261004-02.json',
                                       'reviewed_profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd',
                                       'historical_faults': [{'line': '[    7.573470] '
                                                                      'systemd-journald[273]: File '
                                                                      '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal '
                                                                      'corrupted or uncleanly shut '
                                                                      'down, renaming and '
                                                                      'replacing.',
                                                              'reason': 'Journald reported a '
                                                                        'corrupt or uncleanly '
                                                                        'closed journal file'},
                                                             {'line': '[   19.944418] FAT-fs '
                                                                      '(mmcblk1p1): Volume was not '
                                                                      'properly unmounted. Some '
                                                                      'data may be corrupt. Please '
                                                                      'run fsck.',
                                                              'reason': 'Kernel reported an '
                                                                        'unclean FAT volume or '
                                                                        'filesystem fault'},
                                                             {'line': '[ 3008.252350] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 3',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691466] usb 1-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.691521] usb 1-1.3: '
                                                                      'USB disconnect, device '
                                                                      'number 4',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8518.931957] usb 2-1: USB '
                                                                      'disconnect, device number 2',
                                                              'reason': 'Kernel reported a USB '
                                                                        'disconnect/reset/error'},
                                                             {'line': '[ 8544.480306] '
                                                                      'txrx_err:<aicwf_sdio_wakeup,1312>: '
                                                                      'sdio wakeup fail',
                                                              'reason': 'Kernel reported an SD/MMC '
                                                                        'failure'},
                                                             {'line': '[ 9890.432382] '
                                                                      'sdio_err:<aicwf_sdio_hal_irqhandler,2855>: '
                                                                      'ret=-110, intstatus=ff',
                                                              'reason': 'Feedback guard reported '
                                                                        'an AIC SDIO WiFi driver '
                                                                        'failure'}],
                                       'prior_faults_cleared': False}],
 'all_prior_fault_events_count': 30,
 'current_boot_historical_faults_count': 10,
 'new_unresolved_usb_events': [{'line': '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5',
                                'reason': 'Kernel reported a USB disconnect/reset/error'},
                               {'line': '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6',
                                'reason': 'Kernel reported a USB disconnect/reset/error'},
                               {'line': '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7',
                                'reason': 'Kernel reported a USB disconnect/reset/error'},
                               {'line': '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8',
                                'reason': 'Kernel reported a USB disconnect/reset/error'},
                               {'line': '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9',
                                'reason': 'Kernel reported a USB disconnect/reset/error'},
                               {'line': '[ 7149.911226] usb 1-1.4: USB disconnect, device number '
                                        '10',
                                'reason': 'Kernel reported a USB disconnect/reset/error'}],
 'new_usb_fault_cause_confirmed': False,
 'usb_link_repaired': False,
 'emi_confirmed': False,
 'mcu_reset_confirmed': False,
 'electrical_safety_confirmed': False,
 'wiring_verified': False,
 'powered_servo_health_certified': False,
 'prior_boot_faults_cleared': False,
 'current_boot_faults_cleared': False,
 'previous_usb_faults_cleared': False,
 'filesystem_repaired': False,
 'prior_unexpected_reboot_root_cause_confirmed': False,
 'unsafe_feedback_hold_cleared': False,
 'servo_fault_hold_cleared': False,
 'prior_imu_feedback_fault_cleared': False,
 'servo_arm_authorized': False,
 'servo_motion_authorized': False,
 'automatic_reconnect_authorized': False,
 'generic_kernel_exemption_authorized': False,
 'generic_prefix_loss_exemption_authorized': False,
 'dynamic_baseline_learning_authorized': False,
 'board_card_identity_unchanged': True,
 'ssh_host_fingerprint': 'SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg'}
NATIVE_USB_THIRD_RETURN_HISTORICAL_FAULTS = [{'line': '[    7.385520] systemd-journald[275]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'},
 {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7149.911226] usb 1-1.4: USB disconnect, device number 10',
  'reason': 'Kernel reported a USB disconnect/reset/error'}]
NATIVE_USB_THIRD_RETURN_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': 'aa40ce44e09ba1bd677dc5c133d3442d4065a0050a653a03136aeea9bb219f3f',
 'kernel_prefix_sha256': 'e8ff7b0b3a9456eb1a7c37fe1b73a2e8f10c49f874ae4c589788e07ecc924d48',
 'kernel_line_count': 968}

# One explicitly confirmed COM roundtrip for the servo-voltage firmware.
# Exact artifacts only; future log loss and USB errors still fail validation.
SERVO_VOLTAGE_RETURN_BOOT_ID = 'c24863fc-a8eb-455d-a4f6-54633b6483e3'
SERVO_VOLTAGE_RETURN_PROFILE_SHA256 = '1e4b12d7d8db2abe123ac5717d338a690b93da49ff32d6d2614ca481e8a12b94'
SERVO_VOLTAGE_RETURN_REVIEW_SHA256 = 'd4982915f6603dded89b09b665ac3bea3241d38c21a1d77ada8dd5fe67b3982b'
SERVO_VOLTAGE_RETURN_HISTORICAL_FAULTS = [{'line': '[    7.385520] systemd-journald[275]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'},
 {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 7149.911226] usb 1-1.4: USB disconnect, device number 10',
  'reason': 'Kernel reported a USB disconnect/reset/error'},
 {'line': '[ 9685.073183] usb 1-1.4: USB disconnect, device number 11',
  'reason': 'Kernel reported a USB disconnect/reset/error'}]
SERVO_VOLTAGE_RETURN_PROOF = {'scope': 'feedback_only_reviewed_reconnect',
 'profile_sha256': '1e4b12d7d8db2abe123ac5717d338a690b93da49ff32d6d2614ca481e8a12b94',
 'kernel_prefix_sha256': '486185a5e0943f77ea3bb9ff7407e025c8ced869fd4cd9270dcd4427861f250f',
 'kernel_line_count': 968}

# Exact boot reviewed for the explicitly requested native service restoration.
NATIVE_SERVICE_RESTART_BOOT_ID = '7d23364b-a37c-4ca9-a52f-56bb90547d88'
NATIVE_SERVICE_RESTART_PROFILE_SHA256 = 'e1e7d9acdbe6b72081378718bc080643bc855260a49e8e0bda71e1a1da4df07a'
NATIVE_SERVICE_RESTART_REVIEW_SHA256 = '058c69c2de4f2ed5923066c61fd1bbef0013418dea7d7c1ae6a2e23d096d7ae7'
NATIVE_SERVICE_RESTART_HISTORICAL_FAULTS = [{'line': '[    7.491990] systemd-journald[288]: File '
          '/var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly '
          'shut down, renaming and replacing.',
  'reason': 'Journald reported a corrupt or uncleanly closed journal file'},
 {'line': '[   19.724435] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be '
          'corrupt. Please run fsck.',
  'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]
NATIVE_SERVICE_RESTART_PROOF = {'kernel_line_count': 924,
 'kernel_prefix_sha256': '49dc785846d729879b1eb7bae1b52126509dff56df5b708f28a71b183098f2a8',
 'profile_sha256': 'e1e7d9acdbe6b72081378718bc080643bc855260a49e8e0bda71e1a1da4df07a',
 'scope': 'feedback_only_reviewed_reconnect'}
RECONNECT_VARIANTS = (
    {"boot_id": NATIVE_SERVICE_RESTART_BOOT_ID, "proof": NATIVE_SERVICE_RESTART_PROOF,
     "historical_faults": NATIVE_SERVICE_RESTART_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": SERVO_VOLTAGE_RETURN_BOOT_ID, "proof": SERVO_VOLTAGE_RETURN_PROOF,
     "historical_faults": SERVO_VOLTAGE_RETURN_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": NATIVE_USB_THIRD_RETURN_BOOT_ID, "proof": NATIVE_USB_THIRD_RETURN_PROOF,
     "historical_faults": NATIVE_USB_THIRD_RETURN_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": NATIVE_USB_SECOND_RETURN_BOOT_ID, "proof": NATIVE_USB_SECOND_RETURN_PROOF,
     "historical_faults": NATIVE_USB_SECOND_RETURN_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": NATIVE_USB_RETURN_BOOT_ID, "proof": NATIVE_USB_RETURN_PROOF,
     "historical_faults": NATIVE_USB_RETURN_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": INTENTIONAL_POWERCYCLE_BOOT_ID, "proof": INTENTIONAL_POWERCYCLE_PROOF,
     "historical_faults": INTENTIONAL_POWERCYCLE_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": IMU_RECOVERY_USB_BOOT_ID, "proof": IMU_RECOVERY_USB_PROOF,
     "historical_faults": IMU_RECOVERY_USB_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": IMU_RECOVERY_RECONNECT_BOOT_ID, "proof": IMU_RECOVERY_RECONNECT_PROOF,
     "historical_faults": IMU_RECOVERY_RECONNECT_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": WIFI_RECONNECT_BOOT_ID, "proof": WIFI_RECONNECT_PROOF,
     "historical_faults": WIFI_RECONNECT_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": COLD_RECONNECT_BOOT_ID, "proof": COLD_RECONNECT_PROOF,
     "historical_faults": COLD_RECONNECT_HISTORICAL_FAULTS, "requires_discovery": True},
    {"boot_id": COLD_RECONNECT_BOOT_ID, "proof": COLD_USB_DROP_PROOF,
     "historical_faults": COLD_USB_DROP_HISTORICAL_FAULTS, "requires_discovery": True},
    {'boot_id': REBOOT_BOOT_ID, 'proof': USB_PORT_MOVE_ESP32_PROOF, 'historical_faults': USB_PORT_MOVE_ESP32_HISTORICAL_FAULTS, 'requires_discovery': True},
    {"boot_id": REVIEWED_BOOT_ID, "proof": RECONNECT_PROOF,
     "historical_faults": RECONNECT_HISTORICAL_FAULTS},
    {"boot_id": REVIEWED_BOOT_ID, "proof": RECONNECT2_PROOF,
     "historical_faults": RECONNECT2_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": REBOOT_PROOF,
     "historical_faults": REBOOT_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": RETURNED_ESP32_PROOF,
     "historical_faults": RETURNED_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": MAINTENANCE_ESP32_PROOF,
     "historical_faults": MAINTENANCE_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": WRITE_ACK_ESP32_PROOF,
     "historical_faults": WRITE_ACK_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": CENTER_TRACE_ESP32_PROOF,
     "historical_faults": CENTER_TRACE_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": CENTER_HOLDING_ESP32_PROOF,
     "historical_faults": CENTER_HOLDING_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": ARM_FRESH_ESP32_PROOF,
     "historical_faults": ARM_FRESH_ESP32_HISTORICAL_FAULTS},
    {"boot_id": REBOOT_BOOT_ID, "proof": ARM_WRITE_REPLY_ESP32_PROOF,
     "historical_faults": ARM_WRITE_REPLY_ESP32_HISTORICAL_FAULTS},
)
RECONNECT2_NETWORK_EVIDENCE = {
    "original_reconnect_profile_sha256": RECONNECT_PROFILE_SHA256,
    "wifi_volatile_setting_result_sha256": "5b1e8576489d561d53f2dbfb0e81d8c1ba30eaa287c20cb87cccc39bcc73294e",
    "wifi_power_save_before": "on", "wifi_power_save_after": "off",
    "wifi_power_save_set_attempts": 1, "wifi_power_save_root_cause_confirmed": False,
    "uart_open_attempts_in_network_recovery": 0, "board_files_written_in_network_recovery": False,
    "network_before_after_kernel_identical": True, "new_faults_during_network_recovery": 0,
    "rx_capture_empty_quiet_closed": True, "original_serial_readable_cause_confirmed": False,
}
REBOOT_REQUIRED_REVIEW = {
    "reviewer": "root", "human_context": "重启好了",
    "diagnostic_result_sha256": "51e77b1db6e98015094142ce7090e41f0913b12cfca5f38d509704570be45d67",
    "diagnostic_uart_opened": False, "diagnostic_block_device_opened": False,
    "diagnostic_board_files_written": False, "prior_boot_id": REVIEWED_BOOT_ID,
    "prior_reviewed_profile_sha256": RECONNECT2_PROFILE_SHA256,
    "prior_boot_historical_faults": RECONNECT2_HISTORICAL_FAULTS,
    "prior_log_loss_result_sha256": "929298a5ee88738e31d6f1aa76e2f6602dbea34087e8f0abb124b0af3e478dbe",
    "prior_boot_faults_cleared": False, "filesystem_repaired": False,
    "wifi_settings_modified_in_this_update": False, "board_card_identity_unchanged": True,
    "early_kernel_history_complete": True,
    "ssh_host_fingerprint": "SHA256:HAFDy5A/XsCqVZka7dfGHJSVa09Oitm3vNHDQnnsryg",
}
RETURNED_ESP32_REQUIRED_REVIEW = {
    "reviewer": "root", "reviewed_at": "2026-10-04T06:20:39.961632+08:00",
    "human_context": ("用户确认COM已从X5移至笔记本且URT2电池仍断开；ESP32诊断应用已完整读回核验。"
                      "用户随后回复接好了，确认按要求COM已接回X5、电池保持断开。"
                      "本次完整日志逐字保留旧922行；新增13行仅正常网络连接与CH340同端口拔插，"
                      "Root核对精确USB1-1.3事件及VID/PID/tty身份，无新增存储错误。"
                      "只授权单次只读诊断，不解除unsafe暂停。"),
    "prior_profile_sha256": REBOOT_PROFILE_SHA256,
    "diagnostic_result_sha256": "e81a6265f6b5b42c31dcd9f1e32ac01fdba9ba20233210a68172a7c08182a8ee",
    "connection_sha256": "7564d492f792d31369be66d501a681746edccfc8f5d59b9ed2e3d64192fc8449",
    "reviewed_usb_events": RETURNED_ESP32_HISTORICAL_FAULTS[2:],
    "old_boot_pin_refusal_retained": True,
    "prior_faults_cleared": False, "filesystem_repaired": False,
    "unsafe_feedback_hold_cleared": False, "servo_motion_authorized": False,
}
MAINTENANCE_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-04T23:33:24.123056+08:00', 'human_context': '用户确认 COM 已接笔记本、电池已断开，ESP32 维护应用完成原固件备份与完整读回核验；随后用户确认 COM 已接回 X5。完整日志保留旧 935 行；新增 9 行仅 USB1-1.3 的一次主动拔出及同端口 CH340 返回。', 'prior_profile_sha256': 'a6c3ce1cefbcf87f729accd48b549b01168d7addb5ba8f0cb2d4ce31478f309e', 'diagnostic_result_sha256': 'b54c67df462f219135e4ce8f51a23ed90b0e48a8789ef44fd5de40ce1abfea00', 'connection_sha256': 'b284405e5f5059551f6481be5747c277a7c61fed783fce1394d990ee1869255f', 'verified_firmware_report_sha256': '925a1c487aeefd33fc0a7c605b6f19b8778a22ef43c56921d025ed999efe11ad', 'verified_firmware_app_sha256': 'bf0ce929a708da4e3d9a04e72a9d9cd4e3b2c0a10c22661c9c8ed7ce0844966b', 'reviewed_usb_events': [{'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}
WRITE_ACK_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-04T23:59:46.727950+08:00', 'human_context': '用户确认 COM 已接笔记本、电池已断开；回中写应答补丁已备份并完整读回核验。用户随后确认 COM 已接回 X5、电池已接通且机身托稳。完整日志保留旧 944 行；新增 9 行仅 USB1-1.3 主动拔出及同端口 CH340 返回。未放行任何运动，也未清除维护故障锁。', 'prior_profile_sha256': 'ffa15f5e97683645565a364d754a30b170acac8012fdec1ffc0c6995d467f127', 'diagnostic_result_sha256': 'c120505d6ede7a758394e77f12ffe334fd02822876e1f3a342ab6fbc7a2b2c06', 'connection_sha256': '9bcf824dc1dedd638838bb0794073beab3b1167e1b473ed3cbceebc747f30fe3', 'verified_firmware_report_sha256': 'dc8f69065d41b0c2002d1fd5b0cdfa3c429325fbea583ba7f210ae8f7525118b', 'verified_firmware_app_sha256': '487890bf8d65e697d0f578f9ad9a3cd28ea544d1bc5ce89c3f2d20318250ebac', 'reviewed_usb_events': [{'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}
MAX_PROFILE_BYTES = 128 * 1024
PROFILE_FALSE_FLAGS = ("storage_health_certified", "filesystem_warnings_cleared",
                       "servo_arm_authorized", "servo_motion_authorized",
                       "generic_storage_writes_authorized", "automatic_reconnect_authorized")
PROFILE_TRUE_FLAGS = ("user_confirmed_reconnect",
                      "wifi_warning_reviewed_as_specific_old_driver_event")


def _extract(path, names):
    """Copy definitions only: never import an evidence script or its main body."""
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    definitions = {node.name: ast.get_source_segment(source, node)
                   for node in tree.body
                   if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    if any(name not in definitions for name in names):
        raise ValueError("Missing reviewed helper definition")
    return "\n\n".join(definitions[name] for name in names)


def additional_feedback_fault(line):
    """Extra WiFi-driver stops for feedback only; the old classifier is intact.

    A source marker and an explicit failing result identify AIC driver errors.
    A negative buffer_cnt is also a conservative stop even when its line omits
    the driver name; that alone does not identify its source. Ordinary init,
    config diagnostics, bus-down text, and a retry count alone do not establish
    a failed transfer. No matched event is ever exempted.
    """
    negative_buffers = re.finditer(r"\bbuffer_cnt\s*[:=]\s*(-\d+)\b", line, re.I)
    if any(int(match.group(1)) < 0 for match in negative_buffers):
        return "Feedback guard reported a negative driver buffer count"
    if not re.search(r"\b(?:aicwf_[a-z0-9_]+|aicsdio|aic8800[a-z0-9_]*)\b", line, re.I):
        return None
    negatives = re.finditer(r"\b(?:ret(?:urn)?|rc|errno|err(?:or)?)\s*[:=]\s*(-\d+)\b",
                           line, re.I)
    negative_result = any(int(match.group(1)) < 0 for match in negatives)
    timeout = re.search(r"\b(?:time[ -]?out|timed[ -]?out|etimedout|eio|enodev|ecomm|eilseq|eremoteio)\b",
                        line, re.I)
    transfer_failure = re.search(
        r"\b(?:sdio[ _:-]*)?(?:read|write|wakeup|wake|transfer|irq)[ _:-]+(?:fail(?:ed|ure)?|error)\b",
        line, re.I)
    severity_failure = (re.search(r"\b(?:sdio_err|txrx_err)\s*:", line, re.I)
                        and re.search(r"\bfail(?:ed|ure)?\b", line, re.I))
    if negative_result or timeout or transfer_failure or severity_failure:
        return "Feedback guard reported an AIC SDIO WiFi driver failure"
    return None


CENTER_TRACE_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T00:28:27.450383+08:00', 'human_context': '用户按要求断开 URT2 电池并将 COM 接到笔记本，诊断应用已完成原固件备份和完整读回核验；随后用户确认 COM 已接回 X5、电池已接通、机身托稳。新增 9 行仅一次主动拔出及同端口 CH340 返回。日志环形缓冲仅丢弃最早一条非故障 Booting Linux 行；其余旧 952 行逐字保留，全部历史故障保留。此复查不放行运动、不清除维护故障锁。', 'prior_profile_sha256': '7b69c8ee69665812eb2ccc76b0e202ab66df7039acdad0b80751ab5797d648b4', 'diagnostic_result_sha256': '72d21c062f4dd28b7a57e932bfc4009a12e328e47ea4032ad41827881e74597e', 'connection_sha256': '7bd232e241c8ad45cf60a3ba45ea0c9a76b24ebe0e45c27f11a9cd5509847b7d', 'verified_firmware_report_sha256': '815798ae7319a1310d5cd3212382224d48d23fb950cfa6e19f9dc4f439873751', 'verified_firmware_app_sha256': 'f4cacbc8bd39fbc72801d3e1b6e8baedd5bd92b4e07c000fbe26e21174995ad4', 'reviewed_usb_events': [{'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'evicted_nonfault_lines': ['[    0.000000] Booting Linux on physical CPU 0x0000000000 [0x412fd050]'], 'retained_prior_line_count': 952, 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}

CENTER_HOLDING_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T01:02:14.216340+08:00', 'human_context': '用户确认 COM 已接回 X5，电池已接通。回中修复应用已备份旧应用并完整读回核验。新增九行仅此次主动 USB 拔出及相同 CH340 端口返回；日志环形缓冲逐出最早六条非故障启动信息，其余旧 955 行逐字保留，全部历史故障保留。该复查不放行运动、不清除维护故障锁。', 'prior_profile_sha256': '069f4fc66898a5f2767c1881a5e6e2fc86434a31264a1eb43e5b41ee6f0d11d3', 'diagnostic_result_sha256': '3d2341b703784dcf736310e71f34a8a50df4df863c11fcafaba437490205da53', 'connection_sha256': 'de3d928a3a8cda8a555702025a2c23e89adcf6d69183095fee616834361393af', 'verified_firmware_report_sha256': '212853e716ba59212bf4bb835c951a5b5ad5fe850209c6e295c591d3566596aa', 'verified_firmware_app_sha256': '025fc966456906c27dc725f5b7c3aa27bca7ba5236a973f28577c0c70c50c842', 'reviewed_usb_events': [{'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'evicted_nonfault_lines': ['[    0.000000] Linux version 6.1.83 (hobot@Dpx-PF4PF7V1) (aarch64-none-linux-gnu-gcc (GNU Toolchain for the Arm Architecture 11.2-2022.02 (arm-11.14)) 11.2.1 20220111, GNU ld (GNU Toolchain for the Arm Architecture 11.2-2022.02 (arm-11.14)) 2.37.20220122) #1 SMP PREEMPT Wed Apr  8 21:09:59 CST 2026', '[    0.000000] Machine model: D-Robotics RDK X5 V1.0', '[    0.000000] Reserved memory: created CMA memory pool at 0x0000000088000000, size 382 MiB', '[    0.000000] OF: reserved mem: initialized node linux,cma@88000000, compatible id shared-dma-pool', '[    0.000000] Zone ranges:', '[    0.000000]   DMA      [mem 0x0000000084000000-0x00000000ffffffff]'], 'retained_prior_line_count': 955, 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}

ARM_FRESH_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T02:25:49.368385+08:00', 'human_context': '用户按要求将 COM 移到笔记本、断开 URT2 电池。ARM 反馈更新补丁已备份旧应用并完整读回核验；用户随后确认接回 X5 并接通电池。新增九行仅本次主动拔出及同端口 CH340 返回。环形缓冲逐出最早十条非故障启动行，其余旧 954 行逐字保留，全部历史故障保留。该身份复查不自动使能或清除维护故障锁。', 'prior_profile_sha256': '3071e1cfab266ef8764eab9e856bc086a379425933c578ed884523b74d2da6b7', 'diagnostic_result_sha256': 'bd9c04c6ccc57d7526ba0354710db6473520bf8a0e14561a7a352bba571132b7', 'connection_sha256': 'c045a62a3dbd2b73b2bd5843b85e90c021ea5693d0d3c4bc44a33bfdd9795d8e', 'verified_firmware_report_sha256': '9510f98e8015caca779ff771bf5028cd3e03b7251cc917ec199206064f28ed91', 'verified_firmware_app_sha256': 'd101c589c6a2214523970dcf8e340025f8aa0ed1e5ff98ec6591803615baf1ab', 'reviewed_usb_events': [{'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'evicted_nonfault_lines': ['[    0.000000]   DMA32    empty', '[    0.000000]   Normal   [mem 0x0000000100000000-0x000000027fffffff]', '[    0.000000] Movable zone start for each node', '[    0.000000] Early memory node ranges', '[    0.000000]   node   0: [mem 0x0000000084000000-0x000000009fe7efff]', '[    0.000000]   node   0: [mem 0x000000009fe7f000-0x00000000a207ffff]', '[    0.000000]   node   0: [mem 0x00000000a2080000-0x000000027fffffff]', '[    0.000000] Initmem setup node 0 [mem 0x0000000084000000-0x000000027fffffff]', '[    0.000000] On node 0, zone DMA: 16384 pages in unavailable ranges', '[    0.000000] psci: probing for conduit method from DT.'], 'retained_prior_line_count': 954, 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}

ARM_WRITE_REPLY_ESP32_REQUIRED_REVIEW = {'reviewer': 'root', 'reviewed_at': '2026-10-05T02:49:06.981436+08:00', 'human_context': '用户确认 COM 已接回 X5，电池已接通，机身托稳。ARM 增益写应答修复应用已备份旧应用并完整读回核验。新增九行仅此次主动 USB 拔出及相同 CH340 端口返回；日志环形缓冲逐出最早十条非故障启动信息，其余旧 953 行逐字保留，全部历史故障保留。该身份复查不自动使能或清除维护故障锁。', 'prior_profile_sha256': '1ff3566694a6f2d50da3211e307cdb4e9ac9cc8e6f6b55e782636156686d232c', 'diagnostic_result_sha256': '24ae1dac7922190d847333f1fd0b47ef60548e450e14b38202dcc3b8796d4424', 'connection_sha256': '56b4bf344c387f73d8e126a4f333eb40eef5eb3149a801e595a867fb9cbc41a2', 'verified_firmware_report_sha256': '236e66438edc909b4f2b8fa434f07938e75e5f8855041dcadf4c8fd25f73170f', 'verified_firmware_app_sha256': 'de31bbafaacb978734cc5400f771aad918eb8571b4ee70f769df254307f810f3', 'reviewed_usb_events': [{'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'evicted_nonfault_lines': ['[    0.000000] psci: PSCIv1.1 detected in firmware.', '[    0.000000] psci: Using standard PSCI v0.2 function IDs', '[    0.000000] psci: Trusted OS migration not required', '[    0.000000] psci: SMC Calling Convention v1.2', '[    0.000000] percpu: Embedded 23 pages/cpu s53880 r8192 d32136 u94208', '[    0.000000] pcpu-alloc: s53880 r8192 d32136 u94208 alloc=23*4096', '[    0.000000] pcpu-alloc: [0] 0 [0] 1 [0] 2 [0] 3 [0] 4 [0] 5 [0] 6 [0] 7 ', '[    0.000000] Detected VIPT I-cache on CPU0', '[    0.000000] CPU features: detected: GIC system register CPU interface', '[    0.000000] CPU features: detected: Virtualization Host Extensions'], 'retained_prior_line_count': 953, 'old_boot_pin_refusal_retained': True, 'prior_faults_cleared': False, 'filesystem_repaired': False, 'unsafe_feedback_hold_cleared': False, 'servo_motion_authorized': False}

def _validate_reviewed_profile(profile, *, variant=None):
    """Bind a fixed artifact and boot; never accept an arbitrary new baseline."""
    fields = {"schema_version", "scope", "boot_id", "cid", "root_mount_line", "model",
              "serial", "kernel_prefix", "kernel_prefix_sha256", "kernel_line_count",
              "historical_faults", "allowed_requests", "review", *PROFILE_FALSE_FLAGS,
              *PROFILE_TRUE_FLAGS}
    if not isinstance(profile, dict) or set(profile) != fields:
        raise ValueError("Incomplete or unexpected reviewed reconnect profile fields")
    if variant is None:
        variant = next((item for item in RECONNECT_VARIANTS
                        if item["proof"]["kernel_prefix_sha256"] == profile["kernel_prefix_sha256"]), None)
    if variant is None:
        raise ValueError("No explicitly reviewed feedback reconnect variant matches this kernel")
    if (type(profile["schema_version"]) is not int or profile["schema_version"] != 1
            or profile["scope"] != RECONNECT_SCOPE
            or profile["boot_id"] != variant["boot_id"] or profile["cid"] != REVIEWED_CID
            or profile["root_mount_line"] != RECONNECT_ROOT or profile["model"] != RECONNECT_MODEL
            or profile["serial"] != RECONNECT_SERIAL
            or profile["allowed_requests"] != ["HELLO", "STATE"]
            or any(profile[key] is not False for key in PROFILE_FALSE_FLAGS)
            or any(profile[key] is not True for key in PROFILE_TRUE_FLAGS
                   if key != "user_confirmed_reconnect")
            # Only these immutable reviews record uncommanded USB returns;
            # every older artifact still requires a human physical reconnect.
            or profile["user_confirmed_reconnect"] is not
               (False if variant["proof"]["profile_sha256"] in
                (NATIVE_USB_RETURN_PROFILE_SHA256, NATIVE_USB_SECOND_RETURN_PROFILE_SHA256,
                 NATIVE_USB_THIRD_RETURN_PROFILE_SHA256, NATIVE_SERVICE_RESTART_PROFILE_SHA256) else True)):
        raise ValueError("Reviewed reconnect identity or feedback-only scope differs")
    prefix = profile["kernel_prefix"]
    proof = variant["proof"]
    if (not isinstance(prefix, str) or len(prefix.encode("utf-8")) > MAX_KERNEL_BYTES
            or hashlib.sha256(prefix.encode("utf-8")).hexdigest() != proof["kernel_prefix_sha256"]
            or profile["kernel_prefix_sha256"] != proof["kernel_prefix_sha256"]
            or type(profile["kernel_line_count"]) is not int
            or profile["kernel_line_count"] != proof["kernel_line_count"]
            or len(prefix.splitlines()) != proof["kernel_line_count"]
            or profile["historical_faults"] != variant["historical_faults"]):
        raise ValueError("Reviewed reconnect must retain its exact full kernel and unresolved events")
    if (not isinstance(profile["review"], dict)
            or len(json.dumps(profile["review"], allow_nan=False).encode("utf-8")) > 32768):
        raise ValueError("Reviewed reconnect evidence must be a bounded JSON object")
    if proof["profile_sha256"] == RECONNECT2_PROFILE_SHA256:
        for key, value in RECONNECT2_NETWORK_EVIDENCE.items():
            actual = profile["review"].get(key)
            if type(actual) is not type(value) or actual != value:
                raise ValueError("Second feedback review must retain its exact completed RAM network evidence: " + key)
    if proof["profile_sha256"] in (REBOOT_PROFILE_SHA256, RETURNED_ESP32_PROFILE_SHA256,
                                       MAINTENANCE_ESP32_PROFILE_SHA256, WRITE_ACK_ESP32_PROFILE_SHA256, CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        for key, value in REBOOT_REQUIRED_REVIEW.items():
            actual = profile["review"].get(key)
            if type(actual) is not type(value) or actual != value:
                raise ValueError("Reboot feedback review must retain its exact unresolved history and diagnostics: " + key)
    if proof["profile_sha256"] in (RETURNED_ESP32_PROFILE_SHA256, MAINTENANCE_ESP32_PROFILE_SHA256, WRITE_ACK_ESP32_PROFILE_SHA256, CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("same_boot_usb_reconnect_review")
        if (type(actual) is not dict
                or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(RETURNED_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("Returned ESP32 review must retain its exact same-boot evidence and unsafe hold")
    if proof["profile_sha256"] in (MAINTENANCE_ESP32_PROFILE_SHA256, WRITE_ACK_ESP32_PROFILE_SHA256, CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("maintenance_firmware_usb_reconnect_review")
        if (type(actual) is not dict
                or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(MAINTENANCE_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("Maintenance ESP32 return must retain its exact update and USB review")
    if proof["profile_sha256"] in (WRITE_ACK_ESP32_PROFILE_SHA256, CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("maintenance_write_ack_usb_reconnect_review")
        if (type(actual) is not dict or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(WRITE_ACK_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("Write ACK firmware return must retain its exact update and USB review")
    if proof["profile_sha256"] in (CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("maintenance_center_trace_usb_reconnect_review")
        if (type(actual) is not dict or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(CENTER_TRACE_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("Center trace firmware return must retain its exact update, USB and evicted nonfault line review")
    if proof["profile_sha256"] in (CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("maintenance_center_holding_usb_reconnect_review")
        if (type(actual) is not dict or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(CENTER_HOLDING_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("Center holding firmware return must retain its exact update, USB and six evicted nonfault line review")
    if proof["profile_sha256"] in (ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("motion_arm_fresh_feedback_usb_reconnect_review")
        if (type(actual) is not dict or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(ARM_FRESH_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("ARM feedback firmware return must retain its exact update, USB and ten evicted nonfault line review")
    if proof["profile_sha256"] in (ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256):
        actual = profile["review"].get("motion_arm_write_reply_usb_reconnect_review")
        if (type(actual) is not dict or json.dumps(actual, sort_keys=True, allow_nan=False) !=
                json.dumps(ARM_WRITE_REPLY_ESP32_REQUIRED_REVIEW, sort_keys=True, allow_nan=False)):
            raise ValueError("ARM write reply return must retain its exact update, USB and evicted nonfault history")
    if proof['profile_sha256'] == USB_PORT_MOVE_ESP32_PROFILE_SHA256:
        if profile['review'].get('usb_port_move_review') != USB_PORT_MOVE_ESP32_REQUIRED_REVIEW:
            raise ValueError('USB port move review differs from its exact retained history and physical identity')
    if proof['profile_sha256'] == COLD_RECONNECT_PROFILE_SHA256:
        if profile['review'] != COLD_RECONNECT_REQUIRED_REVIEW:
            raise ValueError('Cold boot review differs from its exact identity, diagnostics and unresolved history')
    if proof['profile_sha256'] == COLD_USB_DROP_PROFILE_SHA256:
        if profile['review'] != COLD_USB_DROP_REQUIRED_REVIEW:
            raise ValueError('USB drop review differs from its exact preserved kernel and stable same-device observations')
    if proof['profile_sha256'] == WIFI_RECONNECT_PROFILE_SHA256:
        if profile['review'] != WIFI_RECONNECT_REQUIRED_REVIEW:
            raise ValueError('Wi-Fi cold boot review differs from its exact raw kernel, identity and unresolved history')
    if proof['profile_sha256'] == IMU_RECOVERY_RECONNECT_PROFILE_SHA256:
        if profile['review'] != IMU_RECOVERY_RECONNECT_REQUIRED_REVIEW:
            raise ValueError('IMU recovery boot review differs from its exact raw kernel, first refusal and unresolved history')
    if proof['profile_sha256'] == IMU_RECOVERY_USB_PROFILE_SHA256:
        if profile['review'] != IMU_RECOVERY_USB_REQUIRED_REVIEW:
            raise ValueError('IMU recovery USB return review differs from its exact observations, unresolved events and battery-off scope')
    if proof['profile_sha256'] == INTENTIONAL_POWERCYCLE_PROFILE_SHA256:
        if profile['review'] != INTENTIONAL_POWERCYCLE_REQUIRED_REVIEW:
            raise ValueError('Intentional powercycle review differs from its exact user context, raw kernel and retained unresolved history')
    if proof['profile_sha256'] == NATIVE_USB_RETURN_PROFILE_SHA256:
        if profile['review'] != NATIVE_USB_RETURN_REQUIRED_REVIEW:
            raise ValueError('Uncommanded USB return review differs from its exact raw kernel, untouched COM context and unresolved history')
    if proof['profile_sha256'] == NATIVE_USB_SECOND_RETURN_PROFILE_SHA256:
        if profile['review'] != NATIVE_USB_SECOND_RETURN_REQUIRED_REVIEW:
            raise ValueError('Second uncommanded USB return review differs from its exact three observations, prior refusal and unresolved history')
    if proof['profile_sha256'] == SERVO_VOLTAGE_RETURN_PROFILE_SHA256:
        review_sha = hashlib.sha256(json.dumps(profile['review'], sort_keys=True,
            ensure_ascii=False, separators=(',', ':')).encode('utf-8')).hexdigest()
        if review_sha != SERVO_VOLTAGE_RETURN_REVIEW_SHA256:
            raise ValueError('Servo-voltage COM return review differs from its exact retained history and read-only firmware proof')
    if proof['profile_sha256'] == NATIVE_USB_THIRD_RETURN_PROFILE_SHA256:
        if profile['review'] != NATIVE_USB_THIRD_RETURN_REQUIRED_REVIEW:
            raise ValueError('Third uncommanded USB return review differs from its exact nonfault eviction, retained raw suffix and six unresolved returns')
    if proof['profile_sha256'] == NATIVE_SERVICE_RESTART_PROFILE_SHA256:
        review_sha = hashlib.sha256(json.dumps(profile['review'], sort_keys=True,
            ensure_ascii=False, separators=(',', ':')).encode('utf-8')).hexdigest()
        if review_sha != NATIVE_SERVICE_RESTART_REVIEW_SHA256:
            raise ValueError('Native service restart review differs from its exact current boot and retained unresolved history')
    return variant


def load_reviewed_profile(path, sha):
    """Read a private local artifact and validate an explicit fixed reconnect.

    This performs no SSH, subprocess, serial or hardware operation. The fixed
    byte hash prevents edited evidence from granting additional allowances.
    """
    variant = next((item for item in RECONNECT_VARIANTS if item["proof"]["profile_sha256"] == sha), None)
    if variant is None:
        raise ValueError("Only explicitly pinned reviewed feedback artifact hashes are supported")
    path = Path(path)
    if not path.is_absolute():
        raise ValueError("Reviewed reconnect profile path must be absolute")
    fd = None
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        before = os.fstat(fd)
        if (not stat.S_ISREG(before.st_mode) or stat.S_IMODE(before.st_mode) != 0o600
                or before.st_uid != os.geteuid() or before.st_nlink != 1
                or not 0 < before.st_size <= MAX_PROFILE_BYTES):
            raise ValueError("Reviewed reconnect profile must be a private, bounded, singly linked regular file")
        chunks = bytearray()
        while True:
            chunk = os.read(fd, min(65536, MAX_PROFILE_BYTES + 1 - len(chunks)))
            if not chunk:
                break
            chunks.extend(chunk)
            if len(chunks) > MAX_PROFILE_BYTES:
                raise ValueError("Reviewed reconnect profile exceeds its size bound")
        after = os.fstat(fd)
        signature = lambda info: (info.st_dev, info.st_ino, info.st_mode, info.st_uid,
                                  info.st_nlink, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
        if signature(before) != signature(after) or len(chunks) != after.st_size:
            raise ValueError("Reviewed reconnect profile changed while being read")
        if hashlib.sha256(chunks).hexdigest() != sha:
            raise ValueError("Reviewed reconnect profile bytes differ from their reviewed hash")
    except OSError as error:
        raise ValueError("Cannot safely read reviewed reconnect profile: " + str(error)) from error
    finally:
        if fd is not None:
            os.close(fd)

    def unique_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate reviewed profile JSON key")
            result[key] = value
        return result

    def finite_only(value):
        raise ValueError("Nonfinite reviewed profile JSON number: " + value)

    profile = json.loads(chunks.decode("utf-8", "strict"), object_pairs_hook=unique_keys,
                         parse_constant=finite_only)
    _validate_reviewed_profile(profile, variant=variant)
    # Use the original classifier definition without importing its executable
    # evidence script. Complete baseline classification is required here and
    # again at remote startup; no generic USB/SDIO exemption is introduced.
    namespace = {"re": re}
    exec(compile(_extract(EVIDENCE_DIR / "passive-snapshot.py", ("classify_log",)),
                 "<reviewed-profile-classifier>", "exec"), namespace)
    faults = [{"line": line, "reason": reason} for line in profile["kernel_prefix"].splitlines()
              if (reason := namespace["classify_log"](line) or additional_feedback_fault(line))]
    if faults != variant["historical_faults"]:
        raise ValueError("Reviewed reconnect full kernel classification differs from its exact events")
    return profile


def serial_receipt_matches(actual, expected):
    """Validate a reported USB identity without opening or enumerating devices."""
    reference = expected['serial']
    if expected.get('discover_serial') is not True:
        return actual == reference
    if (type(actual) is not dict or set(actual) != set(reference)
            or any(type(actual.get(key)) is not str for key in reference)):
        return False
    match = re.fullmatch(r'/dev/ttyUSB(0|[1-9][0-9]{0,2})', actual['path'])
    return (match is not None and int(match.group(1)) <= 255
            and re.fullmatch(r'[1-9][0-9]*-[1-9][0-9]*(?:\.[1-9][0-9]*)*', actual['sysfs_name']) is not None
            and all(actual[key] == reference[key] for key in ('vid', 'pid', 'serial_number')))


def validate_expected(expected, *, reviewed_profile=None):
    # Round-trip also rejects unserializable caller values and detaches objects.
    expected = json.loads(json.dumps(expected, allow_nan=False))
    required = {"boot_id", "cid", "root_mount_line", "model", "serial",
                "kernel_prefix", "historical_faults", "generation"}
    if not isinstance(expected, dict) or not required <= expected.keys():
        raise ValueError("Incomplete feedback identity/baseline")
    if "duration_seconds" in expected:
        raise ValueError("duration_seconds is obsolete; end feedback through the fixed stdin channel")
    if set(expected) - required - {"run_id", "discover_serial"}:
        raise ValueError("Unexpected feedback settings; control messages cannot be customized")
    if "discover_serial" in expected and expected["discover_serial"] is not True:
        raise ValueError("Serial discovery must be an explicit boolean capability")
    variant = _validate_reviewed_profile(reviewed_profile) if reviewed_profile is not None else None
    if variant is not None and variant.get('requires_discovery') and expected.get('discover_serial') is not True:
        raise ValueError('USB port move profile requires explicit ESP32 discovery')
    approved_boot = variant["boot_id"] if variant is not None else REVIEWED_BOOT_ID
    if str(uuid.UUID(expected["boot_id"])) != expected["boot_id"]:
        raise ValueError("Invalid boot identity")
    if expected["boot_id"] != approved_boot or expected["cid"] != REVIEWED_CID:
        raise ValueError("This reader supports only the explicitly reviewed replacement board/card/boot")
    if (not isinstance(expected["cid"], str)
            or not re.fullmatch(r"[0-9a-f]{32}", expected["cid"])
            or expected["cid"] == "fe34325344313647200000052e013c00"):
        raise ValueError("Invalid or previously failed card identity")
    if type(expected["generation"]) is not int or not 0 < expected["generation"] < 2**31:
        raise ValueError("generation must be a nonzero 31-bit integer")
    serial = expected["serial"]
    if (not isinstance(serial, dict)
            or set(serial) != {"path", "vid", "pid", "sysfs_name", "serial_number"}
            or serial["path"] != "/dev/ttyUSB0" or serial["sysfs_name"] != "1-1.3"
            or serial["vid"] != "1a86" or serial["pid"] != "7523"
            or not isinstance(serial["serial_number"], str)
            or len(serial["serial_number"]) > 128):
        raise ValueError("Only the reviewed ttyUSB0/1-1.3 CH340 identity is supported")
    for key in ("model", "root_mount_line", "kernel_prefix"):
        value = expected[key]
        maximum = MAX_KERNEL_BYTES if key == "kernel_prefix" else 4096
        if not isinstance(value, str) or not value or len(value.encode("utf-8")) > maximum:
            raise ValueError("Invalid bounded identity field: " + key)
    if "\n" in expected["model"] or "\n" in expected["root_mount_line"]:
        raise ValueError("Identity must be a single line")
    fields = expected["root_mount_line"].split()
    try:
        separator = fields.index("-", 6)
        valid_root = (fields[2] == "179:2" and fields[4] == "/"
                      and "rw" in fields[5].split(",") and "ro" not in fields[5].split(",")
                      and fields[separator + 1] == "ext4"
                      and fields[separator + 2] in ("/dev/root", "/dev/mmcblk1p2")
                      and "ro" not in fields[separator + 3].split(","))
    except (ValueError, IndexError):
        valid_root = False
    if not valid_root:
        raise ValueError("Reviewed root must be writable 179:2/ext4")
    faults = expected["historical_faults"]
    approved_faults = REVIEWED_HISTORICAL_FAULTS
    if reviewed_profile is not None:
        for key in ("boot_id", "cid", "root_mount_line", "model", "serial", "kernel_prefix",
                    "historical_faults"):
            if expected[key] != reviewed_profile[key]:
                raise ValueError("Expected feedback evidence differs from reviewed reconnect profile: " + key)
        approved_faults = variant["historical_faults"]
    if (not isinstance(faults, list) or len(faults) != len(approved_faults)
            or any(not isinstance(item, dict) or set(item) != {"line", "reason"}
                   or not isinstance(item["line"], str) or not isinstance(item["reason"], str)
                   for item in faults)):
        raise ValueError("Exact reviewed, unresolved historical faults are required")
    if faults != approved_faults:
        raise ValueError("Historical faults differ from the explicitly reviewed replacement-board events")
    return expected


RUNTIME = r'''
import base64, collections, errno, fcntl, hashlib, json, math, os, re, select, signal
import stat, struct, subprocess, sys, termios, time, functools
from pathlib import Path
HEADER = struct.Struct('<2sBBHI')
STATE_PACKET = struct.Struct('<IIHHHBB15H15h6fIBB')
IDS = (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
ALL_MASK = (1 << len(IDS)) - 1
REPLY_SECONDS = .20
MAX_KERNEL_BYTES = 2*1024*1024
PERIOD_SECONDS = .02
HEALTH_INTERVAL = .25
CONTROL_END = b'END_FEEDBACK\n'
__REVIEW_CONSTANTS__
__HELPERS__
__CLASSIFIER__
__FEEDBACK_CLASSIFIER__

def classify_feedback_log(line):
    # Classification depends only on the exact line and immutable source.
    # Reuse that pure result, never a kernel snapshot or an acceptance proof:
    # every inventory still reads and validates the full current history.
    return classify_log(line) or additional_feedback_fault(line)

classify_feedback_log = functools.lru_cache(maxsize=2048)(classify_feedback_log)

class ReaderStopped(ProbeError):
    pass

class FeedbackEnded(Exception):
    pass

class ControlChannel:
    """One fixed termination message; no calibration or motor commands."""
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()
        self.partial_started = None
        self.ended = False
    def poll(self):
        if self.ended:
            return True
        now = time.monotonic()
        if self.partial_started is not None and now-self.partial_started >= .20:
            raise ProbeError('Mac end-feedback control message did not complete within .20 seconds')
        readable, _, exceptional = select.select([self.fd], [], [self.fd], 0)
        if exceptional:
            raise ProbeError('Mac control pipe signaled an exception')
        if not readable:
            return False
        try:
            chunk = os.read(self.fd, 64-len(self.buffer))
        except BlockingIOError:
            return False
        if not chunk:
            raise ProbeError('Mac control stdin ended without an explicit END_FEEDBACK message')
        if self.partial_started is None:
            self.partial_started = now
        self.buffer.extend(chunk)
        if len(self.buffer) > 64 or not CONTROL_END.startswith(self.buffer):
            raise ProbeError('Malformed Mac control input; only END_FEEDBACK newline is allowed')
        if self.buffer == CONTROL_END:
            self.ended = True
            return True
        return False

def _signal_stop(signum, frame):
    raise ReaderStopped('Feedback reader received signal ' + str(signum))

def bounded_read(path, maximum=65536, binary=False):
    # Only proc/sys metadata is read here; no device, disk image or file writes.
    with open(path, 'rb') as stream:
        data = stream.read(maximum + 1)
    if len(data) > maximum:
        raise ProbeError('Metadata exceeds bounded size: ' + str(path))
    return data if binary else data.decode('utf-8', 'strict').strip()

def bounded_dmesg():
    child = subprocess.Popen(['dmesg', '--color=never'], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, close_fds=True)
    stdout_fd, stderr_fd = child.stdout.fileno(), child.stderr.fileno()
    buffers = {stdout_fd: bytearray(), stderr_fd: bytearray()}
    active = set(buffers)
    deadline = time.monotonic() + .30
    try:
        while active:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProbeError('Kernel log read exceeded its bounded deadline')
            ready, _, exceptional = select.select(list(active), [], list(active), remaining)
            if exceptional:
                raise ProbeError('Kernel log pipe failed')
            for fd in ready:
                chunk = os.read(fd, 65536)
                if not chunk:
                    active.remove(fd)
                    continue
                buffers[fd].extend(chunk)
                maximum = MAX_KERNEL_BYTES if fd == stdout_fd else 8192
                if len(buffers[fd]) > maximum:
                    raise ProbeError('Kernel log or stderr exceeded its bounded size')
        status = child.wait(timeout=max(.001, deadline-time.monotonic()))
        stdout = bytes(buffers[stdout_fd])
        stderr = bytes(buffers[stderr_fd])
        if status or stderr or not stdout.strip():
            raise ProbeError('Kernel log is unavailable or reported stderr')
        return stdout.decode('utf-8', 'strict')
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=.3)
        child.stdout.close()
        child.stderr.close()

def validate_kernel(kernel, expected):
    lines = kernel.splitlines()
    # Only the unchanged, completely classified baseline can be reused. The
    # cache key also binds every historical line/reason, so changing either the
    # expected prefix or its allowances requires full classification again.
    baseline_key = (expected['kernel_prefix'],
                    tuple((item['line'], item['reason']) for item in expected['historical_faults']),
                    json.dumps(expected.get('reviewed_reconnect_proof'), sort_keys=True, allow_nan=False),
                    expected['boot_id'], expected['cid'], expected['root_mount_line'], expected['model'],
                    json.dumps(expected['serial'], sort_keys=True, allow_nan=False))
    baseline = getattr(validate_kernel, '_classified_baseline', None)
    if baseline is None or baseline['key'] != baseline_key:
        prefix = expected['kernel_prefix'].splitlines()
    else:
        prefix = baseline['lines']
    if not prefix or lines[:len(prefix)] != prefix:
        raise ProbeError('Retained kernel history changed or lost its reviewed prefix')
    if baseline is None or baseline['key'] != baseline_key:
        proof = expected.get('reviewed_reconnect_proof')
        if proof is None:
            if (expected['historical_faults'] != REVIEWED_HISTORICAL_FAULTS
                or expected['boot_id'] != REVIEWED_BOOT_ID or expected['cid'] != REVIEWED_CID):
                raise ProbeError('Default feedback requires its original boot/card and three unresolved events')
        else:
            variant = next((item for item in RECONNECT_VARIANTS if item['proof'] == proof), None)
            if (variant is None or expected['historical_faults'] != variant['historical_faults']
                or expected['boot_id'] != variant['boot_id'] or expected['cid'] != REVIEWED_CID
                or expected['root_mount_line'] != RECONNECT_ROOT or expected['model'] != RECONNECT_MODEL
                or expected['serial'] != RECONNECT_SERIAL or len(prefix) != proof['kernel_line_count']
                or hashlib.sha256(expected['kernel_prefix'].encode('utf-8')).hexdigest() != proof['kernel_prefix_sha256']):
                raise ProbeError('Feedback-only reconnect proof differs from its fixed reviewed artifact')
        first = next((float(match.group(1)) for line in prefix
                      if (match := re.match(r'^\s*\[\s*(\d+(?:\.\d+)?)\]', line))), None)
        if first is None or first > 1:
            raise ProbeError('Early kernel history is unavailable')
        faults = [{'line': line, 'reason': reason} for line in prefix
                  if (reason := classify_feedback_log(line))]
        if faults != expected['historical_faults']:
            raise ProbeError('Reviewed baseline faults changed or are incomplete')
        baseline = {'key': baseline_key, 'lines': prefix, 'faults': faults}
        validate_kernel._classified_baseline = baseline
    # Matching the entire retained prefix above is mandatory on every check.
    # Even an identical historical warning replayed in the tail is a new fault.
    tail = lines[len(prefix):]
    new_faults = [{'line': line, 'reason': reason, 'tail_index': index}
                  for index, line in enumerate(tail) if (reason := classify_feedback_log(line))]
    if new_faults:
        # Classifier labels describe a conservative stop condition, not proof
        # that the TF card caused the message. Keep the kernel source visible
        # (for example USB topology or an aicwf SDIO driver), with bounded
        # neighboring lines, rather than requiring another hardware query.
        details = []
        for fault in new_faults[:4]:
            index = fault['tail_index']
            details.append({'source': 'new_kernel_tail',
                            'kernel_line_number': len(prefix)+index+1,
                            'line': fault['line'][:512],
                            'line_truncated': len(fault['line']) > 512,
                            'classifier_reason': fault['reason'][:256],
                            'context': [{'kernel_line_number': len(prefix)+neighbor+1,
                                         'line': tail[neighbor][:512],
                                         'line_truncated': len(tail[neighbor]) > 512}
                                        for neighbor in range(max(0,index-1), min(len(tail),index+3))]})
        evidence = {'new_fault_count': len(new_faults), 'faults': details,
                    'fault_details_truncated': len(new_faults) > 4}
        raise ProbeError('New kernel fault; classifier result requires source review: '
                         + json.dumps(evidence, ensure_ascii=False, separators=(',',':')),
                         {'new_kernel_fault_evidence': evidence})
    normalized = ('\n'.join(lines)+'\n').encode('utf-8')
    return {'kernel_log': kernel, 'kernel_sha256': hashlib.sha256(normalized).hexdigest(),
            'kernel_lines': len(lines), 'historical_faults': [dict(item) for item in baseline['faults']],
            'historical_faults_cleared': False}

_SERIAL_BINDINGS = {}

def _usb_serial_identity(path):
    match = re.fullmatch(r'/dev/ttyUSB(0|[1-9][0-9]{0,2})', str(path))
    if match is None or int(match.group(1)) > 255:
        raise ProbeError('Invalid CH340 serial device path')
    info = os.stat(path, follow_symlinks=False)
    if not stat.S_ISCHR(info.st_mode) or (os.major(info.st_rdev), os.minor(info.st_rdev)) != (188, int(match.group(1))):
        raise ProbeError('CH340 character node does not match its tty number')
    tty = (Path('/sys/class/tty') / Path(path).name / 'device').resolve(strict=True)
    usb = next((parent for parent in (tty, *tty.parents)
                if (parent/'idVendor').is_file() and (parent/'idProduct').is_file()), None)
    if usb is None or re.fullmatch(r'[1-9][0-9]*-[1-9][0-9]*(?:\.[1-9][0-9]*)*', usb.name) is None:
        raise ProbeError('CH340 USB identity unavailable')
    actual = {'path': str(path), 'sysfs_name': usb.name,
              'vid': bounded_read(usb/'idVendor', 32).lower(),
              'pid': bounded_read(usb/'idProduct', 32).lower(),
              'serial_number': bounded_read(usb/'serial', 256) if (usb/'serial').is_file() else ''}
    return info, actual

def bind_serial(expected):
    """Discover once at explicit connection entry; never switch a live UART."""
    if expected.get('discover_serial') is not True:
        return expected['serial']
    key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
    if key in _SERIAL_BINDINGS:
        serial_proof(expected)
        return dict(_SERIAL_BINDINGS[key]['identity'])
    nodes = sorted(Path('/sys/class/tty').glob('ttyUSB*'))
    if len(nodes) > 16:
        raise ProbeError('Too many USB serial devices; select ESP32 explicitly')
    candidates = []
    for node in nodes:
        try:
            info, actual = _usb_serial_identity('/dev/' + node.name)
        except (OSError, ProbeError):
            continue
        if all(actual[field] == expected['serial'][field] for field in ('vid','pid','serial_number')):
            candidates.append((info, actual))
    if not candidates:
        raise ProbeError('ESP32 USB serial device not found; check its COM cable and X5 USB connection')
    if len(candidates) != 1:
        raise ProbeError('Multiple matching CH340 USB serial devices; cannot identify ESP32 uniquely')
    info, actual = candidates[0]
    _SERIAL_BINDINGS[key] = {'identity': actual,
                            'node': (info.st_dev, info.st_ino, info.st_rdev)}
    serial_proof(expected)
    return dict(actual)

def serial_identity(expected):
    if expected.get('discover_serial') is not True:
        return expected['serial']
    key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
    if key not in _SERIAL_BINDINGS:
        raise ProbeError('ESP32 serial discovery has not completed')
    return _SERIAL_BINDINGS[key]['identity']

def serial_path(expected):
    return serial_identity(expected)['path']

def serial_proof(expected, fd=None):
    identity = serial_identity(expected)
    if expected.get('discover_serial') is True:
        info, actual = _usb_serial_identity(identity['path'])
        key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
        if ((info.st_dev, info.st_ino, info.st_rdev) != _SERIAL_BINDINGS[key]['node']
                or actual != identity):
            raise ProbeError('Selected ESP32 USB serial device changed; reconnect explicitly')
        if fd is not None:
            opened = os.fstat(fd)
            if (opened.st_dev, opened.st_ino, opened.st_rdev, stat.S_IFMT(opened.st_mode)) != (
                    info.st_dev, info.st_ino, info.st_rdev, stat.S_IFCHR):
                raise ProbeError('Opened UART no longer matches its physical node')
        return info, dict(actual)
    info = os.stat(identity['path'], follow_symlinks=False)
    if not stat.S_ISCHR(info.st_mode) or (os.major(info.st_rdev), os.minor(info.st_rdev)) != (188, 0):
        raise ProbeError('Reviewed ttyUSB0 character node changed')
    tty = Path('/sys/class/tty/ttyUSB0/device').resolve(strict=True)
    usb = next((parent for parent in (tty, *tty.parents)
                if (parent/'idVendor').is_file() and (parent/'idProduct').is_file()), None)
    fixed_usb = Path('/sys/bus/usb/devices/1-1.3').resolve(strict=True)
    if usb is None or usb != fixed_usb or usb.name != identity['sysfs_name']:
        raise ProbeError('Serial USB physical topology changed')
    actual = {'path': identity['path'], 'sysfs_name': usb.name,
              'vid': bounded_read(usb/'idVendor', 32).lower(),
              'pid': bounded_read(usb/'idProduct', 32).lower(),
              'serial_number': bounded_read(usb/'serial', 256) if (usb/'serial').is_file() else ''}
    if actual != identity:
        raise ProbeError('Serial USB VID/PID/serial identity changed')
    if fd is not None:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino, opened.st_rdev, stat.S_IFMT(opened.st_mode)) != (
                info.st_dev, info.st_ino, info.st_rdev, stat.S_IFCHR):
            raise ProbeError('Opened UART no longer matches its physical node')
    return info, actual

def validate_os(metadata, expected):
    if (metadata['boot_id'] != expected['boot_id'] or metadata['cid'] != expected['cid']
            or metadata['root_mount_line'] != expected['root_mount_line']
            or metadata['model'] != expected['model']):
        raise ProbeError('Reviewed boot/root/card/model identity changed')
    swaps = metadata['swaps'].splitlines()
    if not swaps or swaps[0].split() != ['Filename','Type','Size','Used','Priority'] or len(swaps) != 1:
        raise ProbeError('Swap is active or its inventory is malformed')
    root = metadata['root_mount_line'].split()
    try:
        separator = root.index('-', 6)
        writable = ('rw' in root[5].split(',') and 'ro' not in root[5].split(',')
                    and 'ro' not in root[separator+3].split(','))
    except (ValueError, IndexError):
        writable = False
    if not writable:
        raise ProbeError('Root filesystem is no longer writable')

def health_snapshot(expected, fd=None):
    mountinfo = bounded_read('/proc/self/mountinfo', 262144)
    roots = [line for line in mountinfo.splitlines()
             if len(line.split()) > 4 and line.split()[4] == '/']
    if len(roots) != 1:
        raise ProbeError('Root mount is not unique')
    metadata = {'boot_id': bounded_read('/proc/sys/kernel/random/boot_id', 128),
                'cid': bounded_read('/sys/block/mmcblk1/device/cid', 128),
                'root_mount_line': roots[0],
                'model': bounded_read('/proc/device-tree/model', 4096, binary=True).rstrip(b'\0').decode('utf-8','strict'),
                'swaps': bounded_read('/proc/swaps', 16384)}
    validate_os(metadata, expected)
    node = os.stat('/dev/mmcblk1p2', follow_symlinks=False)
    if not stat.S_ISBLK(node.st_mode) or (os.major(node.st_rdev), os.minor(node.st_rdev)) != (179,2):
        raise ProbeError('Canonical root block-node identity changed')
    if bounded_read('/sys/block/mmcblk1/device/type', 32) != 'SD':
        raise ProbeError('Root card is no longer an SD card')
    _, metadata['serial'] = serial_proof(expected, fd)
    metadata.update(validate_kernel(bounded_dmesg(), expected))
    return metadata

class StateTracker:
    def __init__(self):
        self.previous = None
        self.sample_progress = self.imu_progress = None
    def accept(self, state, now):
        if state['armed'] is not False or state['arm_cookie'] or state['fault_mask'] or state['torque_mask']:
            raise ProbeError('Unsafe armed/cookie/fault/torque feedback; reader will close')
        if state['online_mask'] not in (0, ALL_MASK):
            raise ProbeError('Partial servo communication; reader will close')
        if state['last_error'] or state['imu_ok'] is not True:
            raise ProbeError('Firmware error or invalid IMU feedback')
        if any(not math.isfinite(value) for value in state['accel_ms2'] + state['gyro_dps']):
            raise ProbeError('Non-finite IMU data')
        age = (state['sample_us'] - state['imu_us']) & 0xffffffff
        if 500000 < age < 0x80000000:
            raise ProbeError('IMU sample is stale relative to servo sample time')
        if age >= 0x80000000 and ((state['imu_us']-state['sample_us']) & 0xffffffff) > 500000:
            raise ProbeError('IMU and servo timestamps are inconsistent')
        if self.previous is None:
            self.previous = state
            self.sample_progress = self.imu_progress = now
            return True
        advances = []
        for key, clock_key in (('sample_us','sample_progress'), ('imu_us','imu_progress')):
            delta = (state[key]-self.previous[key]) & 0xffffffff
            if delta >= 0x80000000 or delta > 500000:
                raise ProbeError('Feedback sample time reversed/reset or jumped')
            if delta:
                setattr(self, clock_key, now)
            elif now-getattr(self, clock_key) >= .5:
                raise ProbeError('Feedback timestamp stopped advancing')
            advances.append(delta > 0)
        # A read near a 20-ms MCU boundary may repeat one sample. Do not emit it
        # as fresh data or collect it for calibration; bound a frozen source.
        if not all(advances):
            return False
        self.previous = state
        return True

def emit(kind, expected, **values):
    event = {'kind': kind, 'generation': expected['generation'], **values}
    payload = (json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(',',':'))+'\n').encode('utf-8')
    if len(payload) > 3*MAX_KERNEL_BYTES:
        raise ProbeError('JSON output exceeds its bounded size')
    deadline = time.monotonic()+.20
    offset = 0
    while offset < len(payload):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise ProbeError('Mac stdout pipe stopped accepting feedback')
        _, writable, exceptional = select.select([], [sys.stdout.fileno()], [sys.stdout.fileno()], remaining)
        if exceptional:
            raise ProbeError('Mac stdout pipe failed')
        if writable:
            try:
                count = os.write(sys.stdout.fileno(), payload[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise ProbeError('Mac stdout pipe closed')
            offset += count

def _feedback_summary(payload, sequence):
    # _exchange verified header/sequence/CRC. This independent scalar-only
    # diagnostic does not change _decode_state or StateTracker acceptance.
    # Never include positions/vectors, even when their values are malformed.
    if len(payload) != STATE_PACKET.size:
        return None, 'state_length_mismatch'
    values = STATE_PACKET.unpack(payload)
    sample_us, imu_us, online, fault, torque, armed, imu_ok = values[:7]
    cookie, error = values[43:45]
    if ((online | fault | torque) & ~ALL_MASK or armed not in (0,1)
            or imu_ok not in (0,1) or error not in range(6)):
        return None, 'invalid_scalar_fields'
    return {'sequence': sequence, 'sample_us': sample_us, 'imu_us': imu_us,
            'online_mask': online, 'fault_mask': fault, 'torque_mask': torque,
            'armed': bool(armed), 'arm_cookie': cookie, 'imu_ok': bool(imu_ok),
            'last_error': error}, None

def _query_summary(metadata):
    summary = {**metadata, 'request_types_attempted': list(metadata['request_types_attempted']),
               'reply_types_received': list(metadata['reply_types_received'])}
    if 'last_feedback' in metadata:
        summary['last_feedback'] = dict(metadata['last_feedback'])
    return summary

def _hello_feedback(fd, metadata):
    try:
        reply = _exchange(fd, 1, 1, time.monotonic()+REPLY_SECONDS, metadata)
    except ProbeError as error:
        if str(error) == 'Response deadline expired':
            raise ProbeError('ESP32 handshake response timed out') from error
        raise
    if reply != b'XGODUCK_MOTION_1':
        raise ProbeError('Unknown motion firmware identity')

def run_reader(expected):
    # An explicitly launched process gets one UART open, one HELLO, and a
    # STATE stream until its fixed end message. Every I/O and health operation
    # stays bounded. There is no STOP, reset, flushing or reconnect path.
    started = time.monotonic()
    parent = os.getppid()
    fd = None
    closed = False
    reason = 'Feedback reader stopped before normal termination'
    normal = False
    health_watch = None
    metadata = {'request_types_attempted': collections.deque(maxlen=4),
                'reply_types_received': collections.deque(maxlen=4),
                'request_bytes_written': 0, 'state_requests': 0,
                'state_records': 0, 'serial_open_attempts': 0,
                'exclusive': False, 'serial_closed': False,
                'board_files_written_by_reader': False,
                'mcu_reset_possible': True, 'electrical_safety_verified': False,
                'modem_lines_actively_changed': False,
                'firmware_binary_hash_verified': False,
                'firmware_may_autonomously_write_disable_or_imu_config': True,
                'servo_motion_authorized': False, 'storage_health_certified': False}
    tracker = StateTracker()
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        os.set_blocking(sys.stdout.fileno(), False)
        os.set_blocking(sys.stdin.fileno(), False)
        control = ControlChannel(sys.stdin.fileno())
        def check_control():
            if os.getppid() != parent:
                raise ProbeError('Reader parent process disappeared')
            if control.poll():
                raise FeedbackEnded('Feedback ended by Mac')
        check_control()
        bind_serial(expected)
        proof = health_snapshot(expected)
        node, _ = serial_proof(expected)
        owners = _holders(node.st_rdev, time.monotonic()+.20)
        if owners:
            raise ProbeError('UART already has an open holder')
        # The /proc owner scan is bounded, and health is checked again directly
        # before the sole open rather than relying on an earlier snapshot.
        proof = health_snapshot(expected)
        check_control()
        metadata['serial_open_attempts'] += 1
        fd = os.open(serial_path(expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        owners = [owner for owner in _holders(node.st_rdev, time.monotonic()+.20)
                  if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]
        if owners:
            raise ProbeError('Another UART holder appeared while opening')
        metadata['modem_before_raw'] = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        metadata['modem_after_raw'] = _modem_bits(fd)
        if any(metadata['modem_after_raw'][line] != metadata['modem_before_raw'][line]
               for line in ('dtr','rts')):
            raise ProbeError('Observed modem signals changed during UART configuration')
        proof = health_snapshot(expected, fd)
        check_control()
        # Do not flush stale input, swallow boot logs, pulse DTR/RTS or retry.
        _hello_feedback(fd, metadata)
        proof = health_snapshot(expected, fd)
        check_control()
        emit('ready', expected, identity=proof, protocol_identity='XGODUCK_MOTION_1',
             query=_query_summary(metadata), online_requirement='zero_or_all_15',
             allowed_requests=['HELLO','STATE'], emitted_monotonic=time.monotonic(),
             reader_elapsed_seconds=time.monotonic()-started)
        last_health = time.monotonic()
        last_health_started = last_health
        # Native X5 supplies an independently monitored health worker. The
        # legacy reader keeps its synchronous health path unchanged.
        health_factory = globals().get('reader_health_watcher')
        if health_factory is not None:
            health_watch = health_factory(expected, fd)
        sequence = 1
        while True:
            tick = time.monotonic()
            check_control()
            if health_watch is not None:
                last_health = health_watch.check()
            elif tick-last_health_started > .5:
                raise ProbeError('Storage/USB health checks lost their timing guarantee')
            if health_watch is None and tick-last_health_started >= HEALTH_INTERVAL:
                last_health_started = tick
                health_snapshot(expected, fd)
                last_health = time.monotonic()
                if last_health-last_health_started > .5:
                    raise ProbeError('Storage/USB health check took too long')
                check_control()
            sequence += 1
            if sequence > 0xffffffff:
                raise ProbeError('Protocol sequence exhausted; reader will close')
            metadata.pop('last_feedback', None)
            metadata.pop('last_feedback_invalid', None)
            metadata['state_requests'] += 1
            payload = _exchange(fd, 2, sequence, time.monotonic()+REPLY_SECONDS, metadata)
            diagnostic, invalid = _feedback_summary(payload, sequence)
            if invalid is not None:
                metadata['last_feedback_invalid'] = invalid
            else:
                metadata['last_feedback'] = diagnostic
            state = _decode_state(payload)
            now = time.monotonic()
            accepted = tracker.accept(state, now)
            check_control()
            if accepted:
                metadata['state_records'] += 1
                age = (state['sample_us']-state['imu_us']) & 0xffffffff
                age_ms = age/1000 if age < 0x80000000 else 0.
                emit('state', expected, sequence=sequence, state=state,
                     received_monotonic=now, reader_elapsed_seconds=now-started,
                     age_ms=0., imu_age_ms=age_ms,
                     servo_power_observed='none_online' if state['online_mask']==0 else 'all_online',
                     health_age_ms=(now-last_health)*1000)
            delay = max(0., PERIOD_SECONDS-(time.monotonic()-tick))
            if delay:
                time.sleep(delay)
    except FeedbackEnded:
        reason = 'Feedback ended by Mac'
        normal = True
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason, query=_query_summary(metadata))
        except BaseException:
            pass  # The broken pipe itself is sufficient to end the session.
    finally:
        if health_watch is not None:
            try:
                health_watch.close()
            except BaseException as error:
                reason = 'Health worker close failed: '+str(error)
                normal = False
        if fd is not None:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason = 'Serial close failed: '+str(error)
                normal = False
        else:
            closed = True  # Proof means no descriptor was opened or it closed.
        metadata['serial_closed'] = closed
        try:
            emit('closed', expected, reason=reason, normal=normal,
                 serial_closed=closed, query=_query_summary(metadata),
                 elapsed_seconds=time.monotonic()-started)
        except BaseException:
            pass
    return 0 if normal and closed else 20

def run_rx_capture(expected):
    """One guarded diagnostic open: RX only, no request or recovery attempt."""
    started = time.monotonic()
    parent = os.getppid()
    fd = None
    closed = normal = truncated = False
    reason = 'RX capture stopped before completion'
    received = bytearray()
    metadata = {'request_types_attempted': collections.deque(maxlen=4),
                'reply_types_received': collections.deque(maxlen=4),
                'request_bytes_written': 0, 'serial_open_attempts': 0,
                'rx_bytes_read': 0, 'readonly_uart': True,
                'exclusive': False, 'serial_closed': False,
                'board_files_written_by_reader': False,
                'mcu_reset_possible': True, 'electrical_safety_verified': False,
                'modem_lines_actively_changed': False,
                'servo_motion_authorized': False, 'storage_health_certified': False}
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        output_fd = sys.stdout.fileno()
        os.set_blocking(output_fd, False)
        output_poll = select.poll()
        output_poll.register(output_fd, select.POLLOUT | select.POLLERR | select.POLLHUP | select.POLLNVAL)
        def liveness():
            if os.getppid() != parent:
                raise ProbeError('RX capture parent process disappeared')
            events = output_poll.poll(0)
            if any(mask & (select.POLLERR | select.POLLHUP | select.POLLNVAL) for _, mask in events):
                raise ProbeError('RX capture Mac stdout pipe closed')
            if not any(mask & select.POLLOUT for _, mask in events):
                raise ProbeError('RX capture Mac stdout pipe is blocked')
        liveness()
        bind_serial(expected)
        proof = health_snapshot(expected)
        node, _ = serial_proof(expected)
        if _holders(node.st_rdev, time.monotonic()+.20):
            raise ProbeError('UART already has an open holder')
        proof = health_snapshot(expected)
        liveness()
        metadata['serial_open_attempts'] += 1
        fd = os.open(serial_path(expected), os.O_RDONLY | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        others = [owner for owner in _holders(node.st_rdev, time.monotonic()+.20)
                  if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]
        if others:
            raise ProbeError('Another UART holder appeared while opening')
        metadata['modem_before_raw'] = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        metadata['modem_after_raw'] = _modem_bits(fd)
        if any(metadata['modem_after_raw'][line] != metadata['modem_before_raw'][line]
               for line in ('dtr','rts')):
            raise ProbeError('Observed modem signals changed during RX configuration')
        proof = health_snapshot(expected, fd)
        liveness()
        emit('rx_ready', expected, identity=proof, query=_query_summary(metadata),
             allowed_requests=[], capture_limits={'seconds': 1., 'bytes': 4096,
                                                   'quiet_seconds': .1, 'poll_seconds': .05})
        capture_started = last_rx = last_health_started = time.monotonic()
        deadline = capture_started + 1.
        while True:
            liveness()
            now = time.monotonic()
            if now >= deadline:
                reason = 'RX capture time limit reached'
                break
            if now-last_rx >= .1:
                reason = 'RX capture quiet interval reached'
                break
            if now-last_health_started > .5:
                raise ProbeError('RX capture lost its health timing guarantee')
            if now-last_health_started >= HEALTH_INTERVAL:
                last_health_started = now
                health_snapshot(expected, fd)
                if time.monotonic()-last_health_started > .5:
                    raise ProbeError('RX capture health check took too long')
                liveness()
                continue
            wait = min(.05, deadline-now, .1-(now-last_rx),
                       max(.001, HEALTH_INTERVAL-(now-last_health_started)))
            readable, _, exceptional = select.select([fd], [], [fd], wait)
            liveness()
            if exceptional:
                raise ProbeError('RX capture UART signaled an exception')
            if not readable:
                continue
            try:
                chunk = os.read(fd, 4096-len(received))
            except BlockingIOError:
                continue
            if not chunk:
                raise ProbeError('RX capture UART returned EOF')
            received.extend(chunk)
            metadata['rx_bytes_read'] = len(received)
            last_rx = time.monotonic()
            if len(received) >= 4096:
                truncated = True
                reason = 'RX capture byte limit reached; additional input was not consumed'
                break
        liveness()
        health_snapshot(expected, fd)
        liveness()
        normal = not truncated
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason, query=_query_summary(metadata))
        except BaseException:
            pass
    finally:
        if fd is not None:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason = 'Serial close failed: '+str(error)
                normal = False
        else:
            closed = True
        metadata['serial_closed'] = closed
        try:
            emit('rx_capture', expected, raw_base64=base64.b64encode(received).decode('ascii'),
                 sha256=hashlib.sha256(received).hexdigest(), count=len(received),
                 truncated=truncated, empty=not received, reason=reason,
                 elapsed_seconds=time.monotonic()-started, query=_query_summary(metadata))
            emit('closed', expected, reason=reason, normal=normal, serial_closed=closed,
                 query=_query_summary(metadata), elapsed_seconds=time.monotonic()-started)
        except BaseException:
            normal = False
    return 0 if normal and closed else 20
'''


def runtime_source():
    """Return definitions only, without expected data or a call to run_reader."""
    helpers = _extract(EVIDENCE_DIR / "feedback_probe.py", HELPER_NAMES)
    classifier = _extract(EVIDENCE_DIR / "passive-snapshot.py", ("classify_log",))
    feedback_classifier = _extract(Path(__file__), ("additional_feedback_fault",))
    constants = "\n".join(name + " = " + repr(globals()[name]) for name in (
        "REVIEWED_HISTORICAL_FAULTS", "REVIEWED_BOOT_ID", "REVIEWED_CID", "RECONNECT_PROOF",
        "RECONNECT_HISTORICAL_FAULTS", "RECONNECT_ROOT", "RECONNECT_MODEL", "RECONNECT_SERIAL",
        "RECONNECT_KERNEL_LINES", "RECONNECT_KERNEL_SHA256", "RECONNECT_VARIANTS"))
    return (RUNTIME.replace("__REVIEW_CONSTANTS__", constants, 1)
            .replace("__HELPERS__", helpers, 1).replace("__CLASSIFIER__", classifier, 1)
            .replace("__FEEDBACK_CLASSIFIER__", feedback_classifier, 1))


def _source_expected(expected, reviewed_profile_path, reviewed_profile_sha256):
    if (reviewed_profile_path is None) != (reviewed_profile_sha256 is None):
        raise ValueError("Reviewed reconnect profile path and hash must be supplied together")
    profile = (load_reviewed_profile(reviewed_profile_path, reviewed_profile_sha256)
               if reviewed_profile_path is not None else None)
    expected = validate_expected(expected, reviewed_profile=profile)
    if profile is not None:
        # Keep opaque review prose on the Mac; carry only the fixed provenance
        # marker. The complete kernel exists once in EXPECTED, never duplicated.
        expected["reviewed_reconnect_proof"] = dict(_validate_reviewed_profile(profile)["proof"])
    source = runtime_source()
    # Exercise pure validators before returning anything executable remotely.
    namespace = {}
    exec(compile(source, "<feedback-reader-definitions>", "exec"), namespace)
    namespace["validate_kernel"](expected["kernel_prefix"], expected)
    return source, expected


def remote_source(expected, *, reviewed_profile_path=None, reviewed_profile_sha256=None):
    """Validate the explicit evidence and generate code; no device side effects."""
    source, expected = _source_expected(expected, reviewed_profile_path, reviewed_profile_sha256)
    return source + "\nEXPECTED = " + repr(expected) + "\nraise SystemExit(run_reader(EXPECTED))\n"


def rx_capture_source(expected, *, reviewed_profile_path=None, reviewed_profile_sha256=None):
    """Generate one bounded RX-only diagnostic, without executing it locally."""
    source, expected = _source_expected(expected, reviewed_profile_path, reviewed_profile_sha256)
    return source + "\nEXPECTED = " + repr(expected) + "\nraise SystemExit(run_rx_capture(EXPECTED))\n"
