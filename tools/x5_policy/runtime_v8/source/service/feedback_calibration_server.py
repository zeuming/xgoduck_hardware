#!/usr/bin/env python3
"""Mac UI for a single real, feedback-only X5 session; calibration stays on Mac.

Importing this module never opens hardware or starts a server. The explicit
launcher sends a standard-library reader through one pinned SSH connection.
Calibration never commands motors. The separate, explicit motion workbench
deploys models in X5 RAM and takes ownership only after calibration closes.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import secrets
import select
import shlex
import signal
import stat
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import calibration_restore
import imu_wizard
import joint_calibration
import transport_config as network_transport

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
RECOVERY = WORKSPACE / 'output/x5_recovery/replacement-board-20261004T011959+0800'
SAVED = WORKSPACE / 'output/x5_recovery/new-board-20261003T114727Z/wifi-imu-calibration-web-20261003T152130Z/calibration-latest.json'
SAVED_SHA = '39e91daabbf9ebce5361e3ee400c2a0814c956425d41759996078621dededad0'
FRESH = .5
END_FEEDBACK = b'END_FEEDBACK\n'
REVIEWED_BOOT_ID = 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab'
REBOOT_BOOT_ID = '8583b8f1-2d9d-4605-b34f-facbcf18f328'
REBOOT_PROFILE_SHA256 = 'dd634b2e8dfe4432a70024254c6533f2f57df56043b0b9edf92793bcfa9fb529'
RETURNED_ESP32_PROFILE_SHA256 = 'a6c3ce1cefbcf87f729accd48b549b01168d7addb5ba8f0cb2d4ce31478f309e'
MAINTENANCE_ESP32_PROFILE_SHA256 = 'ffa15f5e97683645565a364d754a30b170acac8012fdec1ffc0c6995d467f127'
WRITE_ACK_ESP32_PROFILE_SHA256 = '7b69c8ee69665812eb2ccc76b0e202ab66df7039acdad0b80751ab5797d648b4'
CENTER_TRACE_ESP32_PROFILE_SHA256 = '069f4fc66898a5f2767c1881a5e6e2fc86434a31264a1eb43e5b41ee6f0d11d3'
CENTER_HOLDING_ESP32_PROFILE_SHA256 = '3071e1cfab266ef8764eab9e856bc086a379425933c578ed884523b74d2da6b7'
ARM_FRESH_ESP32_PROFILE_SHA256 = '1ff3566694a6f2d50da3211e307cdb4e9ac9cc8e6f6b55e782636156686d232c'
ARM_WRITE_REPLY_ESP32_PROFILE_SHA256 = '69d3c8d2f98de3a8e8a3243cae2601db1d77f336891b37fb57d7b10a733af07f'
USB_PORT_MOVE_ESP32_PROFILE_SHA256 = '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c'
COLD_RECONNECT_BOOT_ID = 'acbc7d13-7515-4419-a55d-90cf267d74b0'
COLD_RECONNECT_PROFILE_SHA256 = 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0'
COLD_USB_DROP_PROFILE_SHA256 = 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9'
WIFI_RECONNECT_BOOT_ID = 'f2070288-4492-4ddc-9594-ad00640d01b7'
WIFI_RECONNECT_PROFILE_SHA256 = 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0'
UNSAFE_FEEDBACK_REASON = 'Unsafe armed/cookie/fault/torque feedback; reader will close'
CLOSURE_REASONS = (
    '真机反馈连接已结束，未收到完整关闭报告',
    '结束反馈后未收到完整关闭证明，当前连接已停止。',
    '复查连接的串口关闭未获确认',
    '关闭页面服务时未收到完整反馈关闭证明，当前连接已停止。')


def is_unsafe_feedback(error):
    return str(error).startswith(('Unsafe armed/cookie/fault/torque feedback',
                                  'Firmware reports ARMED',
                                  'Disarmed firmware reports a nonzero arm cookie'))


def validated_fault_feedback(query):
    """Diagnostic scalars only; never accept this object as live feedback."""
    if type(query) is not dict or 'last_feedback' not in query:
        return None
    value = query['last_feedback']
    fields = {'sequence', 'sample_us', 'imu_us', 'online_mask', 'fault_mask', 'torque_mask',
              'armed', 'arm_cookie', 'imu_ok', 'last_error'}
    if type(value) is not dict or set(value) != fields:
        raise ValueError('故障诊断字段不完整或含额外字段')
    for key in ('sample_us', 'imu_us', 'arm_cookie'):
        if type(value[key]) is not int or not 0 <= value[key] <= 0xffffffff:
            raise ValueError('故障诊断计时或使能标记无效')
    for key in ('online_mask', 'fault_mask', 'torque_mask'):
        if type(value[key]) is not int or not 0 <= value[key] <= 32767:
            raise ValueError('故障诊断舵机标志无效')
    if (type(value['sequence']) is not int or not 2 <= value['sequence'] <= 0xffffffff
            or type(value['last_error']) is not int or not 0 <= value['last_error'] <= 5
            or type(value['armed']) is not bool or type(value['imu_ok']) is not bool):
        raise ValueError('故障诊断序号或状态无效')
    return copy.deepcopy(value)


def mask_ids(mask):
    return [sid for index, sid in enumerate(joint_calibration.IDS) if mask & (1 << index)]


def unsafe_feedback_text(value=None):
    if value is None:
        return '舵机安全反馈异常，采集已停止；触发值未保存，暂不能区分使能状态或故障标志。需核对并处理后恢复。'
    signals = []
    if value['fault_mask']:
        signals.append('故障标志 ID：' + '、'.join(map(str, mask_ids(value['fault_mask']))))
    if value['torque_mask']:
        signals.append('扭矩仍开启的舵机 ID：' + '、'.join(map(str, mask_ids(value['torque_mask']))))
    if value['armed']:
        signals.append('控制器报告已使能')
    if value['arm_cookie']:
        signals.append('控制器解锁标记异常（非零）')
    if not signals:
        signals.append('该诊断记录未包含触发此安全拒绝的非零标志，尚不能确定触发项')
    return '舵机安全反馈未通过，采集已停止；' + '；'.join(signals) + '。需检查实际反馈并处理后重新核对。'


def fault_text(error, fault_feedback=None, transport_config=None):
    detail = str(error)
    if detail in ('Route is not the verified en5 USB flash link', 'X5 USB 闪联路由不可用',
                  'Reviewed USB route is unavailable'):
        return 'X5 的 USB 网络连接未就绪，采集尚未启动；连接恢复后可再次点击继续采集。', detail
    if detail == 'ESP32 handshake response timed out':
        return 'X5 已连接，但 ESP32 未回应通信握手，采集尚未开始；需检查 ESP32 启动状态。', detail
    if detail.startswith(('Connection-check SSH transport failed:', 'Pinned SSH host identity changed:')):
        return connection_check_error_text(detail, transport_config), detail
    if detail.startswith('Reviewed Wi-Fi route'):
        return connection_check_error_text(detail, transport_config), detail
    if is_unsafe_feedback(detail):
        return unsafe_feedback_text(fault_feedback), detail
    if ('Early kernel history unavailable' in detail or
            'Early kernel history is unavailable' in detail or
            'Reviewed kernel prefix changed or lost' in detail or
            'Retained kernel history changed or lost its reviewed prefix' in detail):
        return '连接检查未通过：板端早期日志已缺失或改变，当前采集未启动。', detail
    if ('kernel fault' in detail.lower() or 'WiFi driver failure' in detail):
        return '板端报告新的异常，采集未启动或已停止。', detail
    return detail, detail


def recovery_recheck_error(error):
    """Keep acquisition locked; explicit metadata checks can be repeated."""
    detail = str(error)
    if is_unsafe_feedback(detail):
        return dict(message='需检查实际舵机反馈并处理后重新核对，当前连接禁止再次恢复；已核对日志不代表舵机当前状态安全。',
                    technical_reason=detail, category='servo')
    if detail in CLOSURE_REASONS:
        return dict(message='连接已中断，尚未确认原串口关闭。点击「检查连接并继续」，确认旧连接已释放后重新采集。',
                    technical_reason=detail, category='closure')
    check_trust = ('Reviewed UART node changed during connection check',
                   'Reviewed UART identity changed during connection check',
                   'Connection-check identity ', 'Connection-check kernel evidence ',
                   'Connection-check UART node proof ', 'Connection-check bounded metadata ',
                   'Connection-check generation, ownership, timing or scope ',
                   'Connection-check SSH exit or source proof ', 'Connection-check source proof ',
                   'Invalid connection-check proof schema', 'Invalid completed connection-check proof schema',
                   'Invalid bounded SSH connection-check result',
                   'Connection-check requires exactly one complete JSON record',
                   'Connection-check stdout/stderr exceeds its bounded size',
                   'Pinned SSH host identity changed',
                   'Connection check requires the pinned root SSH identity',
                   '连接检查证明无效或与当前会话不匹配')
    if detail.startswith(check_trust):
        return dict(message='连接检查的板卡身份或证据不匹配，需要重新核对后恢复；已保存校准保留。',
                    technical_reason=detail, category='trust')
    fixed = ('Retained kernel history changed or lost its reviewed prefix',
             'Early kernel history is unavailable', 'Early kernel history unavailable',
             'Reviewed kernel prefix changed or lost',
             'Reviewed baseline faults changed or are incomplete',
             'Reviewed boot/root/card/model identity changed',
             'X5 的 SSH 身份已变化',
             'Reviewed ttyUSB0 character node changed',
             'Serial USB physical topology changed',
             'Serial USB VID/PID/serial identity changed',
             'Opened UART no longer matches its physical node',
             'Canonical root block-node identity changed',
             'Swap is active or its inventory is malformed',
             'Root filesystem is no longer writable', 'Root mount is not unique',
             'Root card is no longer an SD card',
             'Kernel log is unavailable or reported stderr',
             'Kernel log read exceeded its bounded deadline',
             'Kernel log pipe failed', 'Kernel log or stderr exceeded its bounded size',
             '反馈会话标识不匹配')
    if detail in fixed or detail.startswith('New kernel fault; classifier result requires source review:'):
        return dict(message='需重新核对板端状态后恢复；当前固定日志基线或身份检查未通过。',
                    technical_reason=detail, category='trust')
    return None


def connection_check_error_text(error, transport_config=None):
    detail = str(error)
    prefix = 'New kernel fault; classifier result requires source review: '
    if detail.startswith(prefix):
        try:
            evidence = json.loads(detail[len(prefix):])
            faults = evidence['faults']
            if (faults and evidence.get('fault_details_truncated') is False
                    and all(item.get('classifier_reason') == 'Kernel reported a USB disconnect/reset/error'
                            for item in faults)):
                return 'X5 日志记录了 USB 断开或复位，连接尚未复查通过。可以再次点击检查；运动保持禁用。'
        except (ValueError, KeyError, TypeError, AttributeError):
            pass
    hold = recovery_recheck_error(error)
    if hold is not None:
        return hold['message']
    if 'Permission denied' in detail or 'Authentication failed' in detail:
        return 'X5 的 SSH 登录没有通过，采集未启动；需核对当前板子的登录用户或密码。'
    if 'Connection refused' in detail:
        return '已找到 X5，但 SSH 服务拒绝连接；采集未启动，请等待系统启动完成后再检查。'
    transport = network_transport.resolve_transport(transport_config)
    if transport.kind == 'wifi' and any(text in detail for text in (
            'Host is down', 'Broken pipe', 'SSH', 'timed out', 'deadline',
            'Network is unreachable', 'No route to host', 'Reviewed Wi-Fi route')):
        return ('X5 的 Wi-Fi 连接未完成（' + transport.host +
                '），采集未启动。请确认笔记本和 X5 在同一网络、X5 已开机且地址正确，再点击「检查连接并继续」。')
    if 'Host is down' in detail:
        return 'X5 的 USB 地址 192.168.128.10 没有响应，尚未读取舵机。请重新插好 USB 闪联数据线，再点击「检查连接并继续」。'
    if 'Broken pipe' in detail:
        return 'SSH 连接在发送检查程序时提前中断，板端检查尚未完成；请检查 X5 的 USB 数据连接。'
    if 'Connection-check SSH reported stderr' in detail:
        return 'X5 的 SSH 连接未完成，采集未启动。请检查 X5 供电和 USB 数据连接，再点击「检查连接并继续」。'
    if 'UART already has an open holder' in detail:
        return '旧串口仍被占用。请先关闭另一处串口读取或维护，再点击「检查连接并继续」。'
    if any(text in detail for text in ('timed out', 'deadline', 'SSH connection', 'SSH exited',
                                       'SSH process', 'Network is unreachable', 'No route to host',
                                       'Connection-check SSH transport failed')):
        return '连接 X5 未成功。请检查 X5 供电和 USB 数据连接，再点击「检查连接并继续」。'
    if any(text in detail for text in ('USB route', 'route to', 'direct USB', 'USB 闪联', 'en5')):
        return '未找到 X5 的 USB 数据连接。请确认数据线连接后，再点击「检查连接并继续」。'
    return detail


def _private_json_bytes(filename):
    """Bounded stable read of a private, nonlinked host-side JSON file."""
    descriptor = os.open(filename, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(descriptor)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or before.st_mode & 0o077 or before.st_nlink != 1
                or not 0 < before.st_size <= 1024 * 1024):
            raise ValueError('恢复校准文件不是有效私有文件')
        raw = os.read(descriptor, 1024 * 1024 + 1)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError('恢复校准文件在读取中发生变化')
        return raw
    finally:
        os.close(descriptor)


def _zero_rows(config):
    return [dict(id=sid, zero=zero) for sid, zero in joint_calibration.zero_signature(config)]


def _zero_receipt_digest(value):
    payload = {key: item for key, item in value.items() if key != 'sha256'}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False,
                                     separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def validated_zero_provenance(value, original, current):
    """Authorize only audited, selected-joint saves; never accept arbitrary zeros.

    The last intent is persisted before the calibration rename. A crash or failed
    save can therefore leave its exact before snapshot current. Only that one
    terminal intent may be trimmed; earlier history must form a complete chain.
    """
    if (type(value) is not dict or set(value) != {'version', 'original_zeros', 'entries'}
            or type(value['version']) is not int or value['version'] != 1
            or type(value['original_zeros']) is not list
            or value['original_zeros'] != _zero_rows(original)
            or any(type(item) is not dict or set(item) != {'id', 'zero'}
                   or type(item['id']) is not int or type(item['zero']) is not int
                   for item in value['original_zeros'])
            or type(value['entries']) is not list or len(value['entries']) > 256):
        raise ValueError('重新采集零点的授权记录无效')
    previous_sha, zeros = None, _zero_rows(original)
    for index, entry in enumerate(value['entries']):
        if (type(entry) is not dict or set(entry) != {
                'sequence', 'id', 'generation', 'previous_sha256', 'before', 'after', 'sha256'}
                or type(entry['sequence']) is not int or entry['sequence'] != index + 1
                or type(entry['id']) is not int or entry['id'] not in joint_calibration.IDS
                or type(entry['generation']) is not int or not 0 < entry['generation'] < 0x80000000
                or entry['previous_sha256'] != previous_sha
                or type(entry['sha256']) is not str or entry['sha256'] != _zero_receipt_digest(entry)):
            raise ValueError('重新采集零点的授权链不完整')
        before = calibration_restore.validate_calibration_snapshot(entry['before'])
        after = calibration_restore.validate_calibration_snapshot(entry['after'])
        if _zero_rows(before) != zeros:
            raise ValueError('重新采集零点的前后记录不连续')
        expected = copy.deepcopy(before)
        joint = expected['joints'][joint_calibration.IDS.index(entry['id'])]
        new_zero = joint_calibration.position(after['joints'][joint_calibration.IDS.index(entry['id'])]['zero'])
        joint.update(zero=new_zero,
                     direction_confirmed=False, min=None, max=None)
        if after != expected:
            raise ValueError('零点重采记录改变了其他关节或 IMU')
        zeros, previous_sha = _zero_rows(after), entry['sha256']
    normalized = copy.deepcopy(value)
    current_zeros = _zero_rows(current)
    if current_zeros != zeros:
        if not value['entries'] or current_zeros != _zero_rows(value['entries'][-1]['before']):
            raise ValueError('恢复校准改变了原始 15 个关节 ID 或零点，且无对应重采授权')
        normalized['entries'].pop()
    return normalized


def load_resume_calibration(path, original, *, include_zero_provenance=False):
    """Read only a private prior local run; never infer hardware closure."""
    path = Path(path)
    if (not path.is_absolute() or path.parent.resolve() != RECOVERY
            or not path.name.startswith('feedback-calibration-') or path.is_symlink()):
        raise ValueError('恢复来源必须是当前证据目录中的 feedback-calibration-* 私有目录')
    info = path.stat()
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
            or info.st_mode & 0o077):
        raise ValueError('恢复来源目录不是当前用户的私有目录')
    raw = _private_json_bytes(path / 'calibration-latest.json')
    config = calibration_restore.validate_calibration_snapshot(strict_json(raw))
    provenance = None
    receipt = path / 'joint-zero-provenance.json'
    if os.path.lexists(receipt):
        provenance = validated_zero_provenance(strict_json(_private_json_bytes(receipt)), original, config)
    elif joint_calibration.zero_signature(config) != joint_calibration.zero_signature(original):
        raise ValueError('恢复校准改变了原始 15 个关节 ID 或零点')
    if include_zero_provenance:
        return config, raw, provenance
    return config, raw


def load_stopped_session(path, expected):
    """Carry an exited transport's uncertainty across a local service reload.

    This is an audit of host files, not proof that the remote UART closed.
    In particular it never grants the never-attempted startup exception.
    """
    path = Path(path)
    info = path.stat()
    if (not path.is_absolute() or path.parent.resolve() != RECOVERY
            or not path.name.startswith('feedback-calibration-') or path.is_symlink()
            or not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
            or info.st_mode & 0o077):
        raise ValueError('停止会话来源不是有效私有采集目录')
    attempts = list(path.glob('feedback-attempt-*'))
    if len(attempts) > 256:
        raise ValueError('停止会话连接记录过多，不能自动核对')
    if os.path.lexists(path / 'connection-finished.json'):
        attempts.append(path)
    records = []
    for directory in attempts:
        item = directory.stat()
        if (directory.is_symlink() or not stat.S_ISDIR(item.st_mode)
                or item.st_uid != os.getuid() or item.st_mode & 0o077):
            raise ValueError('停止会话包含不安全连接目录')
        raw = _private_json_bytes(directory / 'connection-finished.json')
        finished = strict_json(raw)
        if (type(finished) is not dict or type(finished.get('generation')) is not int
                or not 0 < finished['generation'] < 0x80000000
                or type(finished.get('ssh_exit_code')) is not int):
            raise ValueError('上次本地连接退出证据无效，不能恢复页面')
        records.append((directory.joinpath('connection-finished.json').stat().st_mtime_ns,
                        directory, finished, raw))
    if not records:
        raise ValueError('停止会话缺少已退出连接记录')
    _, directory, finished, finished_raw = max(records, key=lambda item: item[0])
    evidence_raw = _private_json_bytes(directory / 'fault-evidence.json')
    evidence = strict_json(evidence_raw)
    source_expected = strict_json(_private_json_bytes(directory / 'expectations.json'))
    if (finished.get('session_status') != 'fault' or finished.get('closed_proof', 'missing') is not None
            or type(evidence) is not dict or evidence.get('generation') != finished['generation']
            or evidence.get('ssh_exit_code') != finished['ssh_exit_code']
            or evidence.get('closed_proof', 'missing') is not None
            or evidence.get('error') != finished.get('error')
            or evidence.get('error_details') != finished.get('error_details')
            or source_expected != dict(expected, generation=finished['generation'])):
        raise ValueError('停止会话故障、退出或板端身份记录不一致')
    details = finished.get('error_details')
    if (type(details) is not str or (details not in CLOSURE_REASONS
            and details != '连续 0.6 秒未收到新反馈，采集已暂停；已保存校准保持不变。')):
        raise ValueError('停止会话不是可核对旧连接的传输中断')
    # No trust/servo fault may disappear behind the transport's first error.
    if (any(item.name != 'fault-evidence.json' for item in directory.glob('fault-*.json'))
            or os.path.lexists(directory / 'launch-failure.json')):
        raise ValueError('停止会话另有板端故障报告，需要保留并单独核对')
    passed = strict_json(_private_json_bytes(directory / 'feedback-recheck-passed.json'))
    if (type(passed) is not dict or passed.get('generation') != finished['generation']
            or passed.get('purpose') not in ('imu', 'joints')
            or type(passed.get('frame_count')) is not int or passed['frame_count'] < 2
            or type(passed.get('span_ms')) not in (int, float) or passed['span_ms'] < 200):
        raise ValueError('停止会话缺少此前通过的用途反馈复查记录')
    final_hold = finished.get('recovery_recheck')
    if final_hold is not None and (type(final_hold) is not dict or final_hold.get('category') != 'closure'
            or final_hold != recovery_recheck_error(final_hold.get('technical_reason'))):
        raise ValueError('停止会话另有未解除的板端检查，不能当作单纯连接中断')
    pending_servo = finished.get('pending_servo_hold')
    if pending_servo is None and passed.get('servo_hold_cleared') is not True:
        candidate = passed.get('pending_servo_hold') or passed.get('previous_hold')
        if type(candidate) is dict and is_unsafe_feedback(candidate.get('technical_reason')):
            pending_servo = candidate
    if pending_servo is not None and (type(pending_servo) is not dict
            or not is_unsafe_feedback(pending_servo.get('technical_reason'))
            or pending_servo != recovery_recheck_error(pending_servo['technical_reason'])):
        raise ValueError('停止会话舵机故障锁记录不一致')
    return dict(generation=finished['generation'], ssh_exit_code=finished['ssh_exit_code'],
                error=finished['error'], error_details=details, fault_record=evidence,
                pending_servo_hold=pending_servo,
                purpose=passed['purpose'], source_run=str(path), source_attempt=directory.name,
                source_finished_sha256=hashlib.sha256(finished_raw).hexdigest(),
                source_fault_sha256=hashlib.sha256(evidence_raw).hexdigest(),
                local_transport_exited=True, remote_serial_closed=False,
                closure_not_inferred=True, hardware_started=False)


def held_diagnostic_message(errors, voltage_ids):
    """Historical error bytes are descriptive; never infer live electrical state."""
    if (voltage_ids == list(joint_calibration.IDS) and len(errors) == 15
            and all(item['present_error'] == item['torque_error'] == 1 for item in errors)):
        detail = '15个舵机均报 0x01 电压报警'
    elif voltage_ids:
        detail = '舵机 ID：' + '、'.join(map(str, voltage_ids)) + ' 报电压报警（错误字节 bit 0）'
    else:
        detail = '舵机诊断包含不安全标志，未核实具体原因'
    return ('历史诊断：' + detail + '；这不是实时读数，尚未重新检查。'
            '当前采集保持停止；需核对供电后复查。')


def validated_held_diagnostic(value):
    """Validate an independent historical summary, not a STATE packet."""
    fields = {'scope', 'source_sha256', 'source_name', 'generation', 'sample_us',
              'voltage_alarm_ids', 'device_errors', 'message'}
    if (type(value) is not dict or set(value) != fields
            or value['scope'] != 'historical_diagnostic'
            or type(value['source_sha256']) is not str
            or len(value['source_sha256']) != 64
            or any(c not in '0123456789abcdef' for c in value['source_sha256'])
            or type(value['source_name']) is not str or not value['source_name']
            or len(value['source_name']) > 255
            or Path(value['source_name']).name != value['source_name']
            or value['source_name'] in ('.', '..')
            or type(value['generation']) is not int or not 0 < value['generation'] < 0x80000000
            or type(value['sample_us']) is not int or not 0 <= value['sample_us'] <= 0xffffffff):
        raise ValueError('历史诊断摘要无效')
    errors = value['device_errors']
    if type(errors) is not list or len(errors) != len(joint_calibration.IDS):
        raise ValueError('历史诊断舵机列表无效')
    voltage_ids = []
    for sid, item in zip(joint_calibration.IDS, errors):
        if (type(item) is not dict or set(item) != {'id', 'present_error', 'torque_error'}
                or type(item['id']) is not int or item['id'] != sid):
            raise ValueError('历史诊断舵机 ID 无效')
        for name in ('present_error', 'torque_error'):
            error = item[name]
            if error is not None and (type(error) is not int or not 0 <= error <= 255):
                raise ValueError('历史诊断错误字节无效')
        if any(item[name] is not None and item[name] & 1 for name in ('present_error', 'torque_error')):
            voltage_ids.append(sid)
    if (type(value['voltage_alarm_ids']) is not list
            or any(type(sid) is not int for sid in value['voltage_alarm_ids'])
            or value['voltage_alarm_ids'] != voltage_ids
            or value['message'] != held_diagnostic_message(errors, voltage_ids)):
        raise ValueError('历史诊断结论与错误字节不符')
    return copy.deepcopy(value)


def load_held_diagnostic_report(path, sha256):
    """Pure local, pinned history review. No launcher, UART, or current proof."""
    import feedback_fault_probe
    import run_feedback_fault_details

    path = Path(path)
    if (not path.is_absolute() or type(sha256) is not str or len(sha256) != 64
            or any(c not in '0123456789abcdef' for c in sha256)):
        raise ValueError('历史诊断须使用绝对路径及固定 SHA-256')
    root = RECOVERY.resolve()
    try:
        relative = path.relative_to(root)
    except ValueError as error:
        raise ValueError('历史诊断必须位于当前私有证据目录') from error
    if not relative.parts or any(part in ('.', '..') for part in relative.parts):
        raise ValueError('历史诊断路径无效')
    for parent in (root, *[root.joinpath(*relative.parts[:index])
                          for index in range(1, len(relative.parts))]):
        info = parent.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o700):
            raise ValueError('历史诊断目录必须为私有目录且不能使用符号链接')
    raw = run_feedback_fault_details.private_read(path, 2 * 1024 * 1024)
    if hashlib.sha256(raw).hexdigest() != sha256:
        raise ValueError('历史诊断报告与固定摘要不符')
    report = run_feedback_fault_details.strict_json(raw)
    if (type(report) is not dict or report.get('query_completed') is not True
            or report.get('serial_closed') is not True or report.get('ssh_process_exited') is not True
            or type(report.get('ssh_exit_code')) is not int or report['ssh_exit_code'] != 0
            or report.get('error', 'missing') is not None or report.get('fault_report', 'missing') is not None
            or report.get('servo_motion_authorized') is not False
            or report.get('board_files_written_by_reader') is not False
            or report.get('calibration_written') is not False
            or type(report.get('generation')) is not int or not 0 < report['generation'] < 0x80000000):
        raise ValueError('历史诊断范围或关闭证据未通过')
    namespace = {}
    exec(compile(feedback_fault_probe.runtime_source(), '<historical-fault-definitions>', 'exec'), namespace)
    diagnostic = None
    for name, kind, requests, replies, count, closed in (
            ('ready_report', 'ready', [1], [129], 0, False),
            ('diagnostic_report', 'diagnostic', [1, 7], [129, 135], 0, False),
            ('remote_closed_report', 'closed', [1, 7], [129, 135], 1, True)):
        record = report.get(name)
        if (type(record) is not dict or record.get('kind') != kind
                or type(record.get('generation')) is not int or record['generation'] != report['generation']):
            raise ValueError('历史诊断记录代次或类型不符')
        query = record.get('query')
        if (type(query) is not dict or query.get('request_types_attempted') != requests
                or query.get('reply_types_received') != replies
                or any(type(v) is not int for v in query['request_types_attempted'] + query['reply_types_received'])
                or query.get('servo_motion_authorized') is not False
                or query.get('board_files_written_by_reader') is not False
                or type(query.get('state_records')) is not int or query['state_records'] != 0
                or type(query.get('serial_open_attempts')) is not int or query['serial_open_attempts'] != 1
                or query.get('exclusive') is not True or query.get('serial_closed') is not closed
                or type(query.get('request_bytes_written')) is not int
                or query['request_bytes_written'] != 12 * len(requests)
                or type(query.get('diagnostic_requests')) is not int
                or query['diagnostic_requests'] != (0 if kind == 'ready' else 1)
                or type(query.get('diagnostic_records')) is not int or query['diagnostic_records'] != count):
            raise ValueError('历史诊断请求或能力范围不符')
        if kind == 'ready' and (record.get('allowed_requests') != ['HELLO', 'FAULT_DETAILS']
                                or record.get('diagnostic_only') is not True):
            raise ValueError('历史诊断 HELLO 范围不符')
        if kind == 'diagnostic':
            if type(record.get('sequence')) is not int or record['sequence'] != 2:
                raise ValueError('历史诊断序号不符')
            try:
                diagnostic = run_feedback_fault_details.validated_diagnostic(record['diagnostic'], namespace)
            except (KeyError, TypeError, OverflowError, namespace['struct'].error, namespace['ProbeError']) as error:
                raise ValueError('历史诊断协议字段无效') from error
            if diagnostic['unsafe_flags_present'] is not True:
                raise ValueError('历史诊断未包含需保持停止的异常标志')
        if kind != 'ready':
            try:
                last = run_feedback_fault_details.validated_diagnostic(query['last_diagnostic'], namespace)
            except (KeyError, TypeError, OverflowError, namespace['struct'].error, namespace['ProbeError']) as error:
                raise ValueError('历史诊断关闭字段无效') from error
            if last != diagnostic:
                raise ValueError('历史诊断及关闭记录中的诊断不符')
        if kind == 'closed' and (record.get('normal') is not True or record.get('serial_closed') is not True
                                 or record.get('diagnostic_only') is not True
                                 or record.get('servo_motion_authorized') is not False):
            raise ValueError('历史诊断关闭范围不符')
    errors = []
    voltage_ids = []
    for index, sid in enumerate(joint_calibration.IDS):
        item = dict(id=sid)
        for name, output in (('present_bank', 'present_error'), ('torque_bank', 'torque_error')):
            bank = diagnostic[name]
            found = bank['latest_found_mask'] & (1 << index)
            valid = bank['latest_outcome'][index] in (1, 2, 7)
            item[output] = bank['latest_attempt_error'][index] if found and valid else None
        errors.append(item)
        if any(item[name] is not None and item[name] & 1 for name in ('present_error', 'torque_error')):
            voltage_ids.append(sid)
    summary = validated_held_diagnostic(dict(
        scope='historical_diagnostic', source_sha256=sha256, source_name=path.name,
        generation=report['generation'], sample_us=diagnostic['snapshot_us'],
        voltage_alarm_ids=voltage_ids, device_errors=errors,
        message=held_diagnostic_message(errors, voltage_ids)))
    return summary, raw


def reader_bootstrap(source_size):
    """Consume exactly the source bytes; leave the control channel unbuffered."""
    if type(source_size) is not int or not 0 < source_size <= 262144:
        raise ValueError('真机采集程序大小无效')
    return ("import os\n"
            "_source = bytearray()\n"
            f"while len(_source) < {source_size}:\n"
            f"    _chunk = os.read(0, {source_size} - len(_source))\n"
            "    if not _chunk: raise SystemExit('Feedback source ended early')\n"
            "    _source.extend(_chunk)\n"
            "exec(compile(bytes(_source), '<feedback-reader>', 'exec'))\n")


def write_pipe_once(descriptor, data, seconds):
    """Finish one byte sequence under a deadline; never replay written bytes."""
    offset, deadline = 0, time.monotonic() + seconds
    while offset < len(data):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('X5 反馈控制传送超时')
        _, writable, exceptional = select.select([], [descriptor], [descriptor], remaining)
        if exceptional:
            raise OSError('X5 反馈控制通道中断')
        if writable:
            try:
                count = os.write(descriptor, data[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise OSError('X5 反馈控制传送未完成')
            offset += count


def strict_json(raw):
    def constant(value):
        raise ValueError('非有限 JSON 数值')
    return json.loads(raw, object_pairs_hook=calibration_restore._restore_json_pairs,
                      parse_constant=constant)


def private_new(path, data):
    if isinstance(data, str):
        data = data.encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as output:
        output.write(data)
        output.flush()
        os.fsync(output.fileno())


def atomic_calibration(directory, config):
    calibration_restore.validate_calibration_snapshot(config)
    data = json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    temp = directory / ('.calibration-' + secrets.token_hex(8))
    private_new(temp, data)
    try:
        os.replace(temp, directory / 'calibration-latest.json')
        try:
            descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        except OSError:
            # Rename has committed already. Keep memory consistent with the
            # new file, while reporting that crash durability is uncertain.
            return '当前校准已写入，但持久化确认失败；请导出一份备份。'
        return None
    finally:
        temp.unlink(missing_ok=True)


def atomic_zero_provenance(directory, value):
    """Persist save intent before the calibration rename; host files only."""
    data = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if len(data.encode()) > 1024 * 1024:
        raise ValueError('零点授权记录超过可恢复的大小限制；当前零点未保存')
    temp = directory / ('.joint-zero-' + secrets.token_hex(8))
    private_new(temp, data)
    try:
        os.replace(temp, directory / 'joint-zero-provenance.json')
        descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temp.unlink(missing_ok=True)


def reviewed_expectations(expected, profile_path, profile_sha256, *, loader=None):
    """Apply only an explicitly reviewed, digest-bound kernel baseline."""
    if profile_path is None and profile_sha256 is None:
        return copy.deepcopy(expected), None
    if profile_path is None or profile_sha256 is None:
        raise ValueError('重连日志基线必须同时提供文件路径及 SHA256')
    path = Path(profile_path)
    if (not path.is_absolute() or path.parent.resolve() != RECOVERY
            or path.is_symlink() or type(profile_sha256) is not str
            or len(profile_sha256) != 64
            or any(c not in '0123456789abcdef' for c in profile_sha256)):
        raise ValueError('必须显式指定恢复证据目录中的私有基线文件及有效摘要')
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(descriptor)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or before.st_mode & 0o077 or before.st_nlink != 1
                or not 0 < before.st_size <= 2 * 1024 * 1024):
            raise ValueError('重连基线必须为当前用户的私有小文件')
        raw = os.read(descriptor, 2 * 1024 * 1024 + 1)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError('重连基线在读取过程中发生变化')
    finally:
        os.close(descriptor)
    if hashlib.sha256(raw).hexdigest() != profile_sha256:
        raise ValueError('重连基线摘要不匹配')
    if loader is None:
        import feedback_stream
        loader = feedback_stream.load_reviewed_profile
    profile = loader(path, profile_sha256)
    approved_reboot = (profile_sha256 in (REBOOT_PROFILE_SHA256, RETURNED_ESP32_PROFILE_SHA256,
                                        MAINTENANCE_ESP32_PROFILE_SHA256, WRITE_ACK_ESP32_PROFILE_SHA256, CENTER_TRACE_ESP32_PROFILE_SHA256, CENTER_HOLDING_ESP32_PROFILE_SHA256, ARM_FRESH_ESP32_PROFILE_SHA256, ARM_WRITE_REPLY_ESP32_PROFILE_SHA256, USB_PORT_MOVE_ESP32_PROFILE_SHA256)
                       and expected['boot_id'] == REVIEWED_BOOT_ID
                       and profile['boot_id'] == REBOOT_BOOT_ID)
    approved_reboot = approved_reboot or (
        profile_sha256 in (COLD_RECONNECT_PROFILE_SHA256, COLD_USB_DROP_PROFILE_SHA256)
        and expected['boot_id'] in (REVIEWED_BOOT_ID, REBOOT_BOOT_ID)
        and profile['boot_id'] == COLD_RECONNECT_BOOT_ID)
    approved_reboot = approved_reboot or (
        profile_sha256 == WIFI_RECONNECT_PROFILE_SHA256
        and expected['boot_id'] in (REVIEWED_BOOT_ID, REBOOT_BOOT_ID, COLD_RECONNECT_BOOT_ID)
        and profile['boot_id'] == WIFI_RECONNECT_BOOT_ID)
    for key in ('boot_id', 'cid', 'root_mount_line', 'model', 'serial'):
        if profile[key] != expected[key]:
            if key == 'boot_id' and approved_reboot:
                continue
            raise ValueError('重连基线改变了已核对的板卡身份：' + key)
    updated = copy.deepcopy(expected)
    updated['boot_id'] = profile['boot_id']
    updated['kernel_prefix'] = profile['kernel_prefix']
    updated['historical_faults'] = copy.deepcopy(profile['historical_faults'])
    return updated, raw


class CalibrationSession:
    def __init__(self, config, directory, generation, clock=time.monotonic, recovery_launcher=None,
                 zero_provenance=None, connection_checker=None, transport_config=None,
                 calibration_writer=None):
        self.lock = threading.RLock()
        self.transport_config = network_transport.resolve_transport(transport_config)
        self.clock = clock
        self.config = copy.deepcopy(calibration_restore.validate_calibration_snapshot(config))
        self.original_joints = copy.deepcopy(config['joints'])
        self.original_zeros = joint_calibration.zero_signature(config)
        self.zero_provenance = copy.deepcopy(zero_provenance) if zero_provenance is not None else dict(
            version=1, original_zeros=_zero_rows(config), entries=[])
        self.directory = directory
        self.trace_directory = directory
        self.recovery_launcher = recovery_launcher
        self.connection_checker = connection_checker
        self.calibration_writer = calibration_writer
        self.connection_check = dict(phase='idle', error=None, technical_reason=None, message=None)
        self._restored_transport_exit = None
        self._connection_reconciled_generation = None
        self.recovery_pending = False
        self.recovery_recheck = None
        self.pending_servo_hold = None
        self.feedback_recheck = None
        self.feedback_scope = None
        self._recheck_first = None
        self._recheck_last = None
        # Only start_initial(paused=True) grants this provenance. A failed
        # Popen can leave process=None after actually attempting hardware.
        self._paused_without_attempt = False
        # Only an observed failure before the SSH creation boundary grants
        # this exception. process=None after Popen is never sufficient.
        self._reader_preflight_failure_generation = None
        self.generation = generation
        self.token = secrets.token_hex(32)
        self.started = clock()
        self.status = 'waiting'
        self.error = None
        self.error_details = None
        self.message = None
        self.end_reason = None
        self.end_requested_at = None
        self.end_request_sent = False
        self.end_request_attempted = False
        self.end_delivery_error = None
        self.source_transmitting = False
        self.fault_at = None
        self.last_state = None
        self.last_watch_at = None
        self.watch_gap_ms = None
        self.fault_record = None
        self.fault_feedback = None
        self.fault_feedback_invalid = None
        self.held_diagnostic = None
        self.connection_done = threading.Event()
        self.feedback = None
        self.received_at = None
        self.ready = None
        self.clock_offset = None
        self.closed_proof = None
        self.process = None
        self.stderr = bytearray()
        self.collection = None
        self.poses = {}
        self.axes = None
        self.wizard = self._empty_wizard()
        self.joint_wizard = joint_calibration.JointWizard()
        self.joint_zero = self._empty_joint_zero()
        self.zero_collection = None
        self.maintenance = None
        self.maintenance_hold_directory = directory
        self.maintenance_mapping_changed = False
        self.maintenance_fault_hold = False
        self.maintenance_fault_reason = None
        self.maintenance_review_hold_sha = None
        self.motion = None

    def _write_calibration(self, config):
        """Commit through an injected owner, or preserve the original host store."""
        if self.calibration_writer is None:
            return atomic_calibration(self.directory, config)
        return self.calibration_writer(copy.deepcopy(config), copy.deepcopy(self.zero_provenance))

    def motion_reserved(self):
        if self.motion is None:
            return False
        state = self.motion.snapshot()
        return state['phase'] == 'deploying' or state['owns_connection']

    def configure_motion(self, expected, profile, profile_sha, known_hosts, *, transport_config=None):
        import motion_workbench
        transport = self.transport_config if transport_config is None else transport_config
        self.motion = motion_workbench.MotionWorkbench(expected, profile, profile_sha, known_hosts,
                                                       self.directory, transport_config=transport)

    def motion_action(self, body):
        if type(body) is not dict or self.motion is None:
            raise ValueError('当前服务尚未配置运控入口')
        allowed = {'action', 'page_generation', 'generation', 'mode', 'support_confirmed',
                   'preview_model', 'behavior_confirmed', 'forward', 'side', 'yaw', 'stop_source', 'background', 'control_id'}
        if set(body) - allowed or type(body.get('page_generation')) is not int or body['page_generation'] != self.generation:
            raise ValueError('运控请求的页面会话已变化，请刷新')
        if 'stop_source' in body:
            from motion_workbench import STOP_SOURCES
            if body.get('action') != 'stop' or body['stop_source'] not in STOP_SOURCES:
                raise ValueError('运控停止来源无效')
        if 'background' in body and (body.get('action') != 'heartbeat' or type(body['background']) is not bool):
            raise ValueError('运控后台心跳参数无效')
        if body.get('action') == 'connect':
            with self.lock:
                if self.maintenance_mapping_changed or self.maintenance_fault_hold or self.recovery_recheck:
                    raise ValueError('当前连接或舵机故障尚未复查通过，不能启动运控')
                if (joint_calibration.completed_count(self.config) != 15 or
                        self.config['mechanical_reference_confirmed'] is not True or
                        self.config['imu']['axes_confirmed'] is not True or self.config['imu']['calibrated'] is not True):
                    raise ValueError('请先完成 15 个关节和 IMU 校准')
                if self.collection is not None or self.joint_wizard.value.get('busy') or self.joint_zero['busy']:
                    raise ValueError('正在采样，请先完成或取消本次采样')
                config = copy.deepcopy(self.config)
            def before_connect():
                if self.maintenance is not None and self.maintenance.owns_connection:
                    self.maintenance.action({'action': 'exit'})
                    if self.maintenance.owns_connection:
                        raise ValueError('维护串口关闭尚未确认')
                with self.lock:
                    if self.status in ('waiting', 'active'):
                        self.request_end('manual')
                if not self.connection_done.wait(3):
                    raise ValueError('校准采集关闭尚未确认，运控未打开串口')
                with self.lock:
                    proof = self.closed_proof or {}
                    if self.status != 'closed' or (self.process is not None and (
                            self.process.poll() != 0 or proof.get('normal') is not True or proof.get('serial_closed') is not True)):
                        raise ValueError('校准连接缺少正常关闭证明，运控未打开串口')
                    if self.config != config:
                        raise ValueError('部署前校准发生变化，请重新连接运控')
            self.motion.begin_connect(config, before_connect)
        else:
            self.motion.action({k: v for k, v in body.items() if k != 'page_generation'})
        return self.snapshot()

    def configure_maintenance(self, transport_factory, *, hold_directory=None):
        """Attach a lazy single owner; constructing the page never connects."""
        import servo_maintenance
        with self.lock:
            if self.maintenance is not None:
                raise ValueError('舵机维护入口已配置')
            self.maintenance_hold_directory = Path(hold_directory or self.directory)
            self.maintenance_mapping_changed = os.path.lexists(
                self.maintenance_hold_directory / 'maintenance-identity-hold.json')
            self.maintenance_fault_hold = os.path.lexists(
                self.maintenance_hold_directory / 'maintenance-fault-hold.json')
            if self.maintenance_fault_hold:
                try:
                    held = json.loads((self.maintenance_hold_directory / 'maintenance-fault-hold.json').read_text())
                    self.maintenance_fault_reason = str(held['reason'])[:512]
                except (OSError, ValueError, KeyError, TypeError):
                    self.maintenance_fault_reason = '原维护故障记录无法读取，故障锁保留。'
            self.maintenance = servo_maintenance.MaintenanceController(
                lock=self.lock, transport_factory=transport_factory,
                gate=self.maintenance_blocked_reason, calibration=lambda: copy.deepcopy(self.config),
                identity_changed=self._maintenance_identity_changed,
                fault_callback=self._maintenance_fault, clock=self.clock,
                review_gate=self.maintenance_review_blocked_reason,
                reviewed_callback=self._maintenance_reviewed)

    def _maintenance_identity_changed(self, old, new):
        # Keep the calibration bytes for recovery, but never silently reassign
        # them to a servo whose bus address was modified (or may have been).
        with self.lock:
            self.maintenance_mapping_changed = True
            path = self.maintenance_hold_directory / 'maintenance-identity-hold.json'
            if not os.path.lexists(path):
                private_new(path, json.dumps(dict(
                    reason='servo_id_write_attempted', old_id=old, new_id=new,
                    generation=self.generation, calibration_preserved=True,
                    requires_physical_mapping_review=True), ensure_ascii=False) + '\n')

    def _maintenance_fault(self, error):
        with self.lock:
            self.maintenance_fault_hold = True
            path = self.maintenance_hold_directory / 'maintenance-fault-hold.json'
            if not os.path.lexists(path):
                self.maintenance_fault_reason = str(error)[:512]
                private_new(path, json.dumps(dict(
                    reason=str(error)[:512], generation=self.generation,
                    requires_feedback_review=True), ensure_ascii=False) + '\n')

    def _maintenance_reviewed(self, rows, proof):
        with self.lock:
            if (not self.maintenance_fault_hold or self.maintenance_mapping_changed
                    or self.maintenance is None or self.maintenance.owns_connection
                    or len(rows) != len(joint_calibration.IDS) or [row['id'] for row in rows] != list(joint_calibration.IDS)
                    or any(row.get('online') is not True or row.get('state') != 0
                           or row.get('error') != 0 or row.get('torque') != 0
                           or type(row.get('position')) is not int or not 0 <= row['position'] <= 4095
                           for row in rows)
                    or proof.get('serial_closed') is not True or proof.get('process_exited') is not True
                    or proof.get('ssh_exit_code', 0) != 0):
                raise ValueError('维护复查缺少全部舵机正常反馈或真实关闭证明，故障锁保留')
            path = self.maintenance_hold_directory / 'maintenance-fault-hold.json'
            raw = path.read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            if self.maintenance_review_hold_sha is None or digest != self.maintenance_review_hold_sha:
                raise ValueError('复查期间故障记录变化，故障锁保留')
            name = 'maintenance-fault-reviewed-' + secrets.token_hex(12)
            # Preserve the original fault bytes and a separate resolution audit.
            # No saved calibration is rewritten and no actuator is enabled.
            private_new(self.maintenance_hold_directory / (name + '.json'), json.dumps(dict(
                generation=self.generation, hold_sha256=digest, rows=rows, close_proof=proof,
                calibrated_config_sha256=hashlib.sha256(json.dumps(self.config, sort_keys=True,
                    separators=(',', ':'), ensure_ascii=False).encode()).hexdigest(),
                motion_attempted=False, id_write_attempted=False), ensure_ascii=False) + '\n')
            archive = self.maintenance_hold_directory / (name + '.hold.json')
            if os.path.lexists(archive): raise ValueError('维护故障存档已存在，故障锁保留')
            os.rename(path, archive)
            self.maintenance_fault_hold = False
            self.maintenance_fault_reason = None
            self.maintenance_review_hold_sha = None

    def maintenance_review_blocked_reason(self):
        with self.lock:
            if self.maintenance_mapping_changed:
                return '舵机编号映射待核对，反馈复查不能解除编号锁。'
            if not self.maintenance_fault_hold:
                return '当前没有需要复查的维护故障。'
            return self._maintenance_connection_blocked_reason()

    def maintenance_blocked_reason(self):
        with self.lock:
            if self.maintenance_fault_hold:
                return '上次维护出现故障，需核对实际反馈后重新启用。'
            return self._maintenance_connection_blocked_reason()

    def _maintenance_connection_blocked_reason(self):
        with self.lock:
            if self.motion_reserved():
                return '运控正在使用连接；先在「运控模型」停止并断开。'
            if self.recovery_recheck is not None:
                return self.recovery_recheck['message']
            if self.recovery_pending or self.status in ('waiting', 'active', 'stopping'):
                return '请先结束反馈采集，确认连接关闭后再进入舵机维护。'
            if self.status != 'closed':
                return self.error or '当前反馈故障尚未核对，维护保持停止。'
            if not self.connection_done.is_set() or self.source_transmitting:
                return '原采集连接尚在收尾。'
            if self.process is not None:
                proof = self.closed_proof or {}
                if (self.process.poll() != 0 or proof.get('normal') is not True
                        or proof.get('serial_closed') is not True):
                    return '原采集连接尚无完整关闭证明，不能切换维护。'
            if self.collection is not None or self.joint_wizard.value.get('busy') or self.joint_zero['busy']:
                return '采样尚未结束，不能同时维护舵机。'
            return None

    def maintenance_snapshot(self):
        if self.maintenance is None:
            return dict(phase='unavailable', connected=False, compatible=False,
                        owns_connection=False, busy=False,
                        blocked_reason='当前服务尚未加载维护接口，请更新服务。')
        result = self.maintenance.snapshot()
        result['blocked_reason'] = self.maintenance_blocked_reason()
        result['identity_changed'] = bool(result.get('identity_changed') or self.maintenance_mapping_changed)
        result['fault_hold'] = self.maintenance_fault_hold
        if self.maintenance_fault_hold and not result.get('error') and not result.get('busy'):
            result['error'] = self.maintenance_fault_reason
        review_reason = self.maintenance_review_blocked_reason()
        result['fault_review_allowed'] = (review_reason is None and not result['owns_connection']
                                          and not result['busy'])
        result['fault_review_blocked_reason'] = review_reason
        return result

    def maintenance_action(self, body):
        with self.lock:
            if (type(body) is not dict or type(body.get('generation')) is not int
                    or body['generation'] != self.generation):
                raise ValueError('维护请求的页面会话已变化，请刷新')
            if self.maintenance is None:
                raise ValueError('当前服务尚未配置维护接口')
            action = body.get('action')
            allowed = {'action', 'generation', 'id', 'new_id', 'target', 'support_confirmed',
                       'isolated_confirmed', 'mapping_acknowledged'}
            if set(body) - allowed:
                raise ValueError('维护请求包含不支持的字段')
            if action == 'center' and self.maintenance_mapping_changed:
                raise ValueError('编号曾修改，原关节映射待核对；暂不能按旧校准回中')
            if action == 'review':
                reason = self.maintenance_review_blocked_reason()
                if reason: raise ValueError(reason)
                if self.maintenance.owns_connection: raise ValueError('维护连接仍在占用或收尾')
                path = self.maintenance_hold_directory / 'maintenance-fault-hold.json'
                self.maintenance_review_hold_sha = hashlib.sha256(path.read_bytes()).hexdigest()
            request = {key: value for key, value in body.items() if key != 'generation'}
            self.maintenance.action(request)
            return self.snapshot()

    def _empty_wizard(self):
        return dict(stage='idle', expected_action='level', busy=False, progress=0.,
                    sample_count=0, message='保持 URT2 电池断开，固定机身后从第一步开始。', error=None)

    @staticmethod
    def _empty_joint_zero(stage='idle', message='选择关节，按零姿态摆稳后采集；预览不会修改已保存零点。', error=None):
        return dict(id=None, stage=stage, busy=False, old_zero=None, candidate_zero=None,
                    sample_count=0, progress=0., error=error, message=message)

    def _fail_joint_zero(self, error):
        if self.joint_zero['stage'] not in ('sampling', 'ready'):
            return
        self.zero_collection = None
        self.joint_zero.update(stage='failed', busy=False, candidate_zero=None,
                               error=str(error), message=str(error))

    def _joint_zero_pending(self):
        return self.joint_zero['stage'] in ('sampling', 'ready')

    def _audit_wizard(self, event, collection=None, error=None):
        """Few private Mac events; never store feedback, credentials, or tokens."""
        collection = collection or {}
        elapsed = self.clock() - collection.get('begin', self.clock())
        record = dict(event=event, generation=self.generation,
                      action=collection.get('action', self.wizard.get('expected_action')),
                      sample_count=len(collection.get('samples', [])),
                      elapsed_ms=round(max(0., elapsed) * 1000., 3),
                      error=None if error is None else str(error)[:512])
        descriptor = None
        try:
            descriptor = os.open(self.directory / 'wizard-events.jsonl',
                                 os.O_WRONLY | os.O_CREAT | os.O_APPEND |
                                 os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            info = os.fstat(descriptor)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                    or info.st_mode & 0o077 or info.st_nlink != 1):
                raise OSError('Unsafe Mac audit file')
            data = (json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n').encode()
            with os.fdopen(descriptor, 'wb') as output:
                descriptor = None
                output.write(data)
                output.flush()
                os.fsync(output.fileno())
        except OSError:
            warning = '笔记本步骤记录写入失败；当前校准结果请以页面及导出文件为准。'
            self.wizard['audit_warning'] = warning
            if warning not in self.wizard['message']:
                self.wizard['message'] += ' ' + warning
        finally:
            if descriptor is not None:
                os.close(descriptor)

    def _wizard_fail(self, error):
        collection = self.collection
        action = (self.collection or {}).get('action', self.wizard['expected_action'])
        self.collection = None
        self.wizard.update(stage='failed', expected_action=action, busy=False,
                           progress=0., error=str(error), message=str(error))
        if collection is not None:
            self._audit_wizard('fail', collection, error)

    def fault(self, error):
        with self.lock:
            recheck = recovery_recheck_error(error)
            # A later identity/kernel/closure failure outranks an old servo
            # hold, even when the original fault record is already frozen.
            ranks = {'servo': 1, 'closure': 2, 'trust': 3}
            current = self.recovery_recheck
            if recheck is not None and recheck.get('category') == 'servo':
                self.pending_servo_hold = copy.deepcopy(recheck)
            if recheck is not None and (current is None or
                    ranks.get(recheck.get('category'), 3) > ranks.get(current.get('category'), 3)
                    or (current.get('category') == 'closure'
                        and self._connection_reconciled_generation == self.generation
                        and recheck.get('category') == 'servo')):
                self.recovery_recheck = recheck
            if self.feedback_recheck is not None:
                self.feedback_recheck.update(phase='failed', error=str(error),
                                             message='本次反馈复查未通过；已保存校准保持不变。')
            if self.status != 'fault':
                self.status = 'fault'
                self.error, self.error_details = fault_text(error, self.fault_feedback, self.transport_config)
                self.fault_at = self.end_requested_at or self.clock()
                self.fault_record = dict(
                    generation=self.generation, error=self.error,
                    error_details=self.error_details, fault_monotonic=self.clock(),
                    received_at=self.received_at, last_state=copy.deepcopy(self.last_state),
                    watch_gap_ms=self.watch_gap_ms)
                if self.wizard['stage'] != 'complete':
                    self._wizard_fail(self.error)
                else:
                    self.message = '校准已保存；反馈关闭尚未确认。'
                self.feedback = None
                self.joint_wizard.fail(self.error)
                self._fail_joint_zero(self.error)
                self._send_end_once()

    def _send_end_once(self):
        """Called under the session lock; failure never permits a replay."""
        if self.end_request_attempted or self.source_transmitting:
            return
        self.end_request_attempted = True
        try:
            if (self.process is None or self.process.poll() is not None
                    or self.process.stdin is None or self.process.stdin.closed):
                raise OSError('反馈控制通道不可用')
            write_pipe_once(self.process.stdin.fileno(), END_FEEDBACK, .2)
            self.end_request_sent = True
        except (OSError, TimeoutError, ValueError) as error:
            self.end_delivery_error = str(error)

    def record_fault_feedback(self, message):
        query = message.get('query')
        if self.fault_feedback is None:
            try:
                self.fault_feedback = validated_fault_feedback(query)
            except ValueError:
                self.fault_feedback_invalid = 'invalid_host_diagnostic_fields'
            if type(query) is dict and query.get('last_feedback_invalid') in (
                    'state_length_mismatch', 'invalid_scalar_fields'):
                self.fault_feedback_invalid = query['last_feedback_invalid']
        if self.status == 'fault' and is_unsafe_feedback(self.error_details):
            # The raw first reason is immutable. Only its Chinese explanation
            # may gain verified diagnostics from the matching close report.
            self.error = unsafe_feedback_text(self.fault_feedback)

    def current_attempt(self, generation, process):
        return self.generation == generation and self.process is process

    def source_finished(self, generation, process):
        with self.lock:
            if not self.current_attempt(generation, process):
                return
            self.source_transmitting = False
            if self.status in ('stopping', 'fault'):
                self._send_end_once()
                if not self.end_request_sent:
                    self.fault('结束反馈请求未送达：' + str(self.end_delivery_error))

    def recovery_blocked_reason(self):
        if self.motion_reserved():
            return '运控正在使用连接；先在「运控模型」停止并断开。'
        if self.maintenance_mapping_changed:
            return '舵机编号曾修改或写入结果不确定，需先核对编号与关节映射；已保存校准保留。'
        if self.maintenance_fault_hold:
            return '上次维护出现故障，需核对实际反馈后恢复采集。'
        if self.maintenance is not None and self.maintenance.owns_connection:
            return '舵机维护占用连接；先退出维护并等待串口关闭证明。'
        if self.recovery_recheck is not None:
            return self.recovery_recheck['message']
        if self.feedback_recheck is not None:
            return '本次会话已按用途复查，请使用 IMU 或关节反馈复查继续采集。'
        if self.recovery_launcher is None:
            return '当前服务未配置手动恢复入口。'
        if self.recovery_pending:
            return '正在检查新的连接，请等待结果。'
        if self.status not in ('fault', 'closed'):
            return '请先结束当前反馈采集。'
        if not self.connection_done.is_set() or (self.process is not None and self.process.poll() is None):
            return '上次采集进程尚在收尾，请稍候。'
        return None

    def recheck_blocked_reason(self):
        """Eligibility for one user-requested, purpose-bound read-only check."""
        if self.motion_reserved():
            return '运控正在使用连接；先在「运控模型」停止并断开。'
        if self.maintenance_mapping_changed:
            return '舵机编号映射待核对，反馈复查不能解除编号锁。'
        if self.maintenance_fault_hold:
            return '维护故障尚未核对，反馈复查不能解除维护锁。'
        if self.maintenance is not None and self.maintenance.owns_connection:
            return '舵机维护仍占用连接，请先退出维护并等待关闭证明。'
        hold = self.recovery_recheck
        if hold is not None and not is_unsafe_feedback(hold['technical_reason']):
            return hold['message']
        if self.recovery_launcher is None:
            return '当前服务未配置反馈复查入口。'
        if self.recovery_pending:
            return '当前连接仍在运行或收尾，请先结束采集。'
        if self.status not in ('fault', 'closed'):
            return '请先结束当前采集，再按用途重新复查。'
        if not self.connection_done.is_set() or self.source_transmitting:
            return '上次采集进程尚在收尾，暂不能复查。'
        if self.process is None:
            if (not self._paused_without_attempt
                    and self._reader_preflight_failure_generation != self.generation):
                return '上次连接是否关闭尚未确认，不能再次启动。'
        else:
            proof = self.closed_proof or {}
            if (self.process.poll() is None or proof.get('kind') != 'closed'
                    or type(proof.get('generation')) is not int
                    or proof['generation'] != self.generation
                    or proof.get('serial_closed') is not True):
                return '原连接缺少匹配本次会话的串口关闭及进程退出证明。'
        # An arbitrary failure is not a licence to reuse this special path.
        # A previous purpose check may fail transiently (e.g. wrong power
        # state); it can be retried only after the full closure gate above.
        clean_end = (self.status == 'closed' and self.end_reason in ('manual', 'completed')
                     and self.end_request_sent and (self.closed_proof or {}).get('normal') is True
                     and self.process is not None and self.process.poll() == 0)
        if (hold is None and not self._paused_without_attempt and not clean_end
                and self.feedback_recheck is None
                and self._reader_preflight_failure_generation != self.generation):
            return '当前故障不属于可在页面复查的舵机反馈问题。'
        return None

    def connection_check_blocked_reason(self):
        """Retry read-only metadata checks without weakening acquisition gates."""
        if self.motion_reserved():
            return '运控正在使用连接；先在「运控模型」停止并断开。'
        if self.maintenance_mapping_changed:
            return '舵机编号映射待核对，连接检查不能解除编号锁。'
        if self.maintenance_fault_hold:
            return '维护故障尚未核对，连接检查不能解除维护锁。'
        if self.maintenance is not None and self.maintenance.owns_connection:
            return '舵机维护仍占用连接，请先退出维护。'
        if self.connection_checker is None or self.recovery_launcher is None:
            return '当前服务尚未配置连接检查入口。'
        hold = self.recovery_recheck
        if hold is None or hold.get('category') not in ('closure', 'trust'):
            return hold['message'] if hold else '当前无需检查旧连接，请使用采集入口。'
        if self.recovery_pending or self.connection_check['phase'] == 'checking':
            return '正在检查连接，请等待本次结果。'
        if self.status not in ('fault', 'closed') or not self.connection_done.is_set() or self.source_transmitting:
            return '上次采集进程尚在收尾，请稍候再检查连接。'
        if self.process is not None:
            if self.process.poll() is None:
                return '上次本地采集进程仍在运行，不能检查新连接。'
        elif (self._restored_transport_exit is None
                or self._restored_transport_exit.get('generation') != self.generation
                or self._restored_transport_exit.get('local_transport_exited') is not True
                or type(self._restored_transport_exit.get('ssh_exit_code')) is not int):
            return '上次本地连接退出尚未得到确认，不能检查新连接。'
        return None

    def check_connection_action(self, body):
        with self.lock:
            if (type(body) is not dict or set(body) != {'generation', 'purpose'}
                    or type(body['generation']) is not int or body['generation'] != self.generation
                    or body['purpose'] not in ('imu', 'joints')):
                raise ValueError('连接检查请求无效或页面会话已变化，请刷新')
            blocked = self.connection_check_blocked_reason()
            if blocked is not None:
                raise ValueError(blocked)
            generation, purpose = self.generation, body['purpose']
            directory = self.directory / ('connection-check-' + secrets.token_hex(8))
            try:
                directory.mkdir(mode=0o700, exist_ok=False)
                private_new(directory / 'request.json', json.dumps(dict(
                    generation=generation, purpose=purpose, previous_hold=self.recovery_recheck,
                    pending_servo_hold=self.pending_servo_hold,
                    previous_fault=self.fault_record, previous_closed_proof=self.closed_proof,
                    restored_transport_exit=self._restored_transport_exit,
                    calibration_preserved=True, explicit_user_request=True),
                    ensure_ascii=False, allow_nan=False) + '\n')
            except OSError as error:
                raise ValueError('笔记本连接检查记录保存失败，尚未连接 X5：' + str(error)) from error
            self.connection_check.update(phase='checking', error=None, technical_reason=None,
                message='正在检查板卡身份、日志和旧串口占用；通过后继续采集。')
            self.recovery_pending = True

            def check():
                try:
                    proof = self.connection_checker(generation, directory)
                    # The configured transport also validates full identity;
                    # this boundary forbids stale or capability-changing receipts.
                    if (type(proof) is not dict or proof.get('kind') != 'quiescent'
                            or type(proof.get('generation')) is not int or proof['generation'] != generation
                            or (not getattr(self, 'serial_discovery_allowed', False) and proof.get('serial_path') != '/dev/ttyUSB0')
                            or (getattr(self, 'serial_discovery_allowed', False) and not (
                                type(proof.get('serial_path')) is str and
                                re.fullmatch(r'/dev/ttyUSB(0|[1-9][0-9]{0,2})', proof['serial_path']) and
                                int(proof['serial_path'].removeprefix('/dev/ttyUSB')) <= 255))
                            or proof.get('serial_holders') != []
                            or proof.get('identity_checked') is not True or proof.get('guards_passed') is not True
                            or type(proof.get('observation_count')) is not int or proof['observation_count'] != 2
                            or type(proof.get('serial_open_attempts')) is not int or proof['serial_open_attempts'] != 0
                            or type(proof.get('serial_bytes_written')) is not int or proof['serial_bytes_written'] != 0
                            or proof.get('board_files_written') is not False or proof.get('automatic_retry') is not False
                            or type(proof.get('ssh_exit_code')) is not int or proof['ssh_exit_code'] != 0
                            or proof.get('ssh_process_exited') is not True):
                        raise ValueError('连接检查证明无效或与当前会话不匹配，采集保持停止。')
                    with self.lock:
                        if self.generation != generation:
                            raise ValueError('检查过程中页面会话已变化，不能继续采集。')
                        # Re-evaluate every local owner/hold after the probe.
                        self.recovery_pending = False
                        self.connection_check['phase'] = 'passed'
                        blocked = self.connection_check_blocked_reason()
                        if blocked is not None:
                            raise ValueError(blocked)
                        private_new(directory / 'reconciled.json', json.dumps(dict(
                            generation=generation, purpose=purpose, proof=proof,
                            previous_hold=self.recovery_recheck,
                            uart_closed_report_fabricated=False, calibration_preserved=True),
                            ensure_ascii=False, allow_nan=False) + '\n')
                        self.connection_check.update(error=None, technical_reason=None,
                            message='旧连接已释放，正在重新检查实时反馈。')
                        self._launch_feedback_attempt(purpose, connection_reconciled=proof)
                except BaseException as error:
                    with self.lock:
                        if self.generation == generation:
                            self.recovery_pending = False
                            newer = recovery_recheck_error(error)
                            if newer is not None and newer.get('category') == 'trust':
                                self.recovery_recheck = newer
                            self.connection_check.update(phase='failed', error=connection_check_error_text(error, self.transport_config),
                                technical_reason=str(error),
                                message='连接检查未通过，采集保持停止：' + connection_check_error_text(error, self.transport_config))
                            try:
                                private_new(directory / 'failed.json', json.dumps(dict(
                                    generation=generation, error=str(error),
                                    previous_fault=self.fault_record, calibration_preserved=True),
                                    ensure_ascii=False, allow_nan=False) + '\n')
                            except OSError:
                                self.connection_check['message'] = '连接检查记录保存失败；采集保持停止。'
            try:
                threading.Thread(target=check, daemon=True).start()
            except RuntimeError as error:
                self.recovery_pending = False
                self.connection_check.update(phase='failed', error='本地连接检查未启动。',
                                             technical_reason=str(error), message='本地连接检查未启动。')
            return self.snapshot()

    def restore_stopped(self, record):
        """Consume only load_stopped_session's audited local stopped record."""
        with self.lock:
            if (record.get('generation') != self.generation
                    or record.get('local_transport_exited') is not True
                    or record.get('remote_serial_closed') is not False
                    or record.get('closure_not_inferred') is not True
                    or type(record.get('ssh_exit_code')) is not int):
                raise ValueError('停止会话导入记录无效')
            self._paused_without_attempt = False
            self._restored_transport_exit = copy.deepcopy(record)
            self.status = 'fault'
            self.error, self.error_details = record['error'], record['error_details']
            self.fault_record = copy.deepcopy(record['fault_record'])
            self.recovery_recheck = recovery_recheck_error(CLOSURE_REASONS[0])
            self.pending_servo_hold = copy.deepcopy(record.get('pending_servo_hold'))
            self.message = '已保留上次中断及串口关闭未确认状态；点击「检查连接并继续」重新核对。'
            self.connection_done.set()
            if self.config['imu']['calibrated'] and self.config['imu']['axes_confirmed']:
                self.wizard.update(stage='complete', expected_action=None, progress=1.,
                                   message='IMU 校准已保存。')

    def start_initial(self, paused=False, hold_unsafe=False, held_diagnostic=None):
        if hold_unsafe and not paused:
            raise ValueError('安全反馈暂停只能用于暂停启动，不得启动硬件')
        if held_diagnostic is not None and (not paused or not hold_unsafe):
            raise ValueError('历史诊断仅可用于安全反馈暂停启动')
        history = None if held_diagnostic is None else validated_held_diagnostic(held_diagnostic)
        if paused:
            with self.lock:
                self._paused_without_attempt = True
                saved_imu = self.config['imu']['calibrated'] and self.config['imu']['axes_confirmed']
                self.status = 'closed' if saved_imu else 'fault'
                self.error = None if saved_imu else '上次反馈已中断，请点击恢复采集重新检查连接。'
                self.message = '页面已就绪；尚未启动新的硬件连接，已保存校准保持不变。'
                if saved_imu:
                    self.wizard.update(stage='complete', expected_action=None, progress=1.,
                                       message='IMU 校准已保存；恢复反馈后可继续采集关节方向与范围。')
                if hold_unsafe:
                    self.status = 'fault'
                    self.error, self.error_details = fault_text(UNSAFE_FEEDBACK_REASON)
                    self.recovery_recheck = recovery_recheck_error(UNSAFE_FEEDBACK_REASON)
                    self.pending_servo_hold = copy.deepcopy(self.recovery_recheck)
                    self.message = '已保存校准保持不变；本页尚未启动硬件，需另行核对实际舵机反馈。'
                    if history is not None:
                        self.held_diagnostic = history
                        self.error = history['message']
                        self.message = history['message'] + ' 已保存校准保持不变。'
                self.connection_done.set()
            return
        if self.recovery_launcher is None:
            raise ValueError('未配置反馈启动入口')
        # The persistent reader must not block creation of the web server.
        # Use the same single-attempt lifecycle as an explicit page request.
        with self.lock:
            self._launch_feedback_attempt()

    def recover_action(self, body):
        with self.lock:
            if (type(body) is not dict or set(body) != {'generation'}
                    or type(body['generation']) is not int or body['generation'] != self.generation):
                raise ValueError('恢复请求的校准会话已变化，请刷新页面')
            blocked = self.recovery_blocked_reason()
            if blocked is not None:
                raise ValueError(blocked)
            return self._launch_feedback_attempt()

    def recheck_feedback_action(self, body):
        with self.lock:
            if (type(body) is not dict or set(body) != {'generation', 'purpose'}
                    or type(body['generation']) is not int or body['generation'] != self.generation
                    or body['purpose'] not in ('imu', 'joints')):
                raise ValueError('反馈复查请求无效或页面会话已变化，请刷新')
            blocked = self.recheck_blocked_reason()
            if blocked is not None:
                raise ValueError(blocked)
            return self._launch_feedback_attempt(body['purpose'])

    def _launch_feedback_attempt(self, purpose=None, *, connection_reconciled=None):
        """One explicit request, one guarded reader; caller holds self.lock."""
        generation = (self.generation + secrets.randbelow(0x7ffffffe)) % 0x7ffffffe + 1
        if generation == self.generation:
            generation = self.generation % 0x7ffffffe + 1
        trace = self.directory / ('feedback-attempt-' + secrets.token_hex(8))
        try:
            trace.mkdir(mode=0o700, exist_ok=False)
        except OSError as error:
            raise ValueError('笔记本采集证据目录创建失败，未启动硬件连接：' + str(error)) from error
        if purpose is not None:
            try:
                private_new(trace / 'feedback-recheck-request.json', json.dumps(dict(
                    previous_generation=self.generation, generation=generation, purpose=purpose,
                    previous_hold=self.recovery_recheck, previous_fault=self.fault_record,
                    pending_servo_hold=self.pending_servo_hold,
                    previous_feedback_recheck=self.feedback_recheck,
                    previous_fault_feedback=self.fault_feedback, held_diagnostic=self.held_diagnostic,
                    previous_closed_proof=self.closed_proof,
                    connection_reconciled=connection_reconciled,
                    paused_without_attempt=self._paused_without_attempt,
                    calibration_preserved=True), ensure_ascii=False, allow_nan=False) + '\n')
            except OSError as error:
                raise ValueError('笔记本复查证据保存失败，未启动硬件：' + str(error)) from error
        self.feedback_scope = purpose
        self.feedback_recheck = (None if purpose is None else dict(
            purpose=purpose, phase='checking', frame_count=0, span_ms=0., error=None,
            previous_hold=copy.deepcopy(self.recovery_recheck),
            pending_servo_hold=copy.deepcopy(self.pending_servo_hold),
            connection_reconciled=copy.deepcopy(connection_reconciled),
            message='正在检查连接及当前反馈；至少持续 0.2 秒通过后才开放采集。'))
        self._recheck_first, self._recheck_last = None, None
        self._paused_without_attempt = False
        self._reader_preflight_failure_generation = None
        self.generation, self.token = generation, secrets.token_hex(32)
        self._connection_reconciled_generation = generation if connection_reconciled is not None else None
        self._restored_transport_exit = None
        self.trace_directory, self.recovery_pending = trace, True
        self.status, self.error, self.error_details = 'waiting', None, None
        self.started, self.message = self.clock(), '正在重新检查板卡身份和完整日志；检查通过后才开始反馈。'
        self.process, self.connection_done = None, threading.Event()
        self.feedback, self.received_at, self.ready, self.clock_offset = None, None, None, None
        self.closed_proof, self.stderr = None, bytearray()
        self.end_reason, self.end_requested_at = None, None
        self.end_request_sent, self.end_request_attempted = False, False
        self.end_delivery_error, self.fault_at, self.fault_record = None, None, None
        self.fault_feedback, self.fault_feedback_invalid = None, None
        self.source_transmitting = False
        self.last_state, self.last_watch_at, self.watch_gap_ms = None, None, None
        self.collection, self.poses, self.axes = None, {}, None
        self.joint_wizard.clear()
        self.joint_zero, self.zero_collection = self._empty_joint_zero(), None
        if self.wizard['stage'] != 'complete':
            self.wizard = self._empty_wizard()
        done = self.connection_done

        def launch():
            try:
                self.recovery_launcher(self, generation, trace)
            except BaseException as error:
                with self.lock:
                    if self.generation == generation:
                        self.fault(error)
                        try:
                            private_new(trace / 'launch-failure.json', json.dumps(dict(
                                generation=generation, error=self.error,
                                error_details=self.error_details,
                                ssh_exit_code=None if self.process is None else self.process.poll(),
                                closure_not_inferred=True), ensure_ascii=False) + '\n')
                            self.write_fault_evidence(trace, self.process)
                        except OSError:
                            self.message = '笔记本启动证据写入失败；采集保持停止，已保存校准保持不变。'
                        if self.process is None or self.process.poll() is not None:
                            done.set()
            finally:
                with self.lock:
                    if self.generation == generation:
                        self.recovery_pending = False

        try:
            threading.Thread(target=launch, daemon=True).start()
        except RuntimeError as error:
            self.recovery_pending = False
            self.fault('本地采集启动失败：' + str(error))
            done.set()
        return self.snapshot()

    def end_action(self, body):
        with self.lock:
            if (type(body) is not dict or set(body) != {'generation'}
                    or type(body['generation']) is not int or body['generation'] != self.generation):
                raise ValueError('结束请求的校准会话已变化，请刷新页面')
            return self.request_end('manual')

    def request_end(self, reason):
        """One fixed control record on this SSH stdin; never a MCU command."""
        if reason not in ('manual', 'completed'):
            raise ValueError('未知的结束原因')
        with self.lock:
            if self.end_reason is not None:
                return self.snapshot()
            if self.status in ('fault', 'closed'):
                raise ValueError('反馈会话已关闭')
            if reason == 'completed' and self.wizard['stage'] != 'complete':
                raise ValueError('四步校准尚未保存完成')
            self.end_reason, self.end_requested_at = reason, self.clock()
            if self.feedback_recheck is not None and self.feedback_recheck['phase'] == 'checking':
                self.feedback_recheck.update(phase='failed', error='user_cancelled',
                                             message='本次反馈复查已取消；已保存校准保持不变。')
            self.status, self.feedback = 'stopping', None
            self.joint_wizard.cancel()
            self._fail_joint_zero('采集会话已结束，未保存的零点预览已取消。')
            self.message = ('四步校准已保存，正在结束反馈。' if reason == 'completed'
                            else '正在手动结束反馈；已保存校准保持不变。')
            if reason == 'manual' and self.wizard['stage'] != 'complete':
                collection = self.collection
                previous_action = self.wizard.get('expected_action')
                self.collection, self.poses, self.axes = None, {}, None
                self.wizard.update(stage='cancelled', busy=False, progress=0.,
                                   message='本次采样已取消；已保存校准保持不变。', error=None)
                self._audit_wizard('cancel', collection or dict(action=previous_action))
            self._send_end_once()
            if not self.end_request_sent and not self.source_transmitting:
                self.fault('结束反馈请求未送达，当前连接已停止：' + str(self.end_delivery_error))
            return self.snapshot()

    def check_watchdog(self):
        """Only bounded startup, stale-data and requested-shutdown deadlines."""
        with self.lock:
            now = self.clock()
            self.watch_gap_ms = None if self.last_watch_at is None else max(0., now - self.last_watch_at) * 1000
            self.last_watch_at = now
            error = None
            if self.status == 'waiting' and now - self.started > 10:
                error = '真机反馈启动超时，当前连接已停止；已保存校准保持不变。'
            elif (self.status == 'active' and self.received_at is not None
                  and now - self.received_at > .6):
                error = '连续 0.6 秒未收到新反馈，采集已暂停；已保存校准保持不变。'
            elif (self.status == 'stopping' and self.end_requested_at is not None
                  and now - self.end_requested_at > 2):
                error = '结束反馈后未收到完整关闭证明，当前连接已停止。'
            if error is not None:
                self.fault(error)
            return self.status in ('fault', 'closed')

    def cleanup_due(self):
        with self.lock:
            return self.status == 'closed' or (self.status == 'fault' and
                    self.fault_at is not None and self.clock() - self.fault_at >= 2)

    def write_fault_evidence(self, directory, process):
        with self.lock:
            if self.fault_record is None:
                return
            record = copy.deepcopy(self.fault_record)
            record.update(end_request_attempted=self.end_request_attempted,
                          end_request_sent=self.end_request_sent,
                          end_delivery_error=self.end_delivery_error,
                          ssh_exit_code=None if process is None else process.poll(), closed_proof=self.closed_proof,
                          fault_feedback=copy.deepcopy(self.fault_feedback),
                          fault_feedback_invalid=self.fault_feedback_invalid,
                          fault_ids=None if self.fault_feedback is None else mask_ids(self.fault_feedback['fault_mask']),
                          torque_ids=None if self.fault_feedback is None else mask_ids(self.fault_feedback['torque_mask']),
                          collected_samples=self.wizard.get('sample_count'))
            private_new(directory / 'fault-evidence.json', json.dumps(record, ensure_ascii=False) + '\n')

    def confirm_closed(self, exit_code):
        """A UART closure report alone never proves that SSH exited cleanly."""
        with self.lock:
            proof = self.closed_proof or {}
            if self.status == 'fault':
                return self.snapshot()
            if (type(exit_code) is not int or exit_code != 0
                    or self.status != 'stopping' or self.end_reason is None
                    or not self.end_request_sent or proof.get('normal') is not True
                    or proof.get('serial_closed') is not True):
                self.fault('反馈采集进程退出未得到正常确认，关闭报告保持原样供核对。')
                return self.snapshot()
            self.status, self.error = 'closed', None
            self.message = ('四步校准已保存，反馈已正常结束。' if self.end_reason == 'completed'
                            else '反馈已手动结束；已保存校准保持不变。')
            return self.snapshot()

    def _require_imu(self, generation):
        if type(generation) is not int or generation != self.generation:
            raise ValueError('校准会话已变化，请刷新页面')
        if (self.status != 'active' or self.feedback is None or self.received_at is None
                or not 0 <= self.clock() - self.received_at < FRESH):
            raise ValueError('真机反馈不可用或已过期，采样已停止')
        s = self.feedback
        if (s['armed'] is not False or s['arm_cookie'] != 0 or s['torque_mask'] != 0
                or s['fault_mask'] != 0 or s['online_mask'] != 0 or s['imu_ok'] is not True):
            raise ValueError('IMU 校准要求 URT2 电池断开、未使能且 IMU 有效')
        return s

    def _require_joint_feedback(self):
        imu = self.config['imu']
        if imu['calibrated'] is not True or imu['axes_confirmed'] is not True:
            raise ValueError('请先完成并保存 IMU 校准')
        if (self.status != 'active' or self.feedback is None or self.received_at is None
                or not 0 <= self.clock() - self.received_at < FRESH):
            raise ValueError('真机反馈不可用或已过期，关节采样已停止')
        state = self.feedback
        if (state['online_mask'] != 32767 or state['fault_mask'] != 0 or state['torque_mask'] != 0
                or state['armed'] is not False or state['arm_cookie'] != 0 or state['imu_ok'] is not True
                or state.get('last_error', 0) != 0):
            raise ValueError('关节采样须 15 个舵机全部在线、实际扭矩关闭且未使能，IMU 有效')
        return state

    def _require_joint_zero_feedback(self):
        if self.feedback_scope == 'imu' or self.recovery_recheck is not None:
            raise ValueError('请先在机械零点页面复查带电舵机反馈，再重采零点')
        if self.maintenance_mapping_changed or self.maintenance_fault_hold:
            raise ValueError('舵机编号或维护故障待核对，零点采集保持停止')
        if self.maintenance is not None and self.maintenance.snapshot().get('owns_connection'):
            raise ValueError('舵机维护仍占用连接，请先退出维护')
        state = self._require_joint_feedback()
        if type(state.get('last_error')) is not int or state['last_error'] != 0:
            raise ValueError('零点采集须收到当前无故障反馈')
        imu_age = (state['sample_us'] - state['imu_us']) & 0xffffffff
        if 500000 < imu_age < 0x100000000 - 500000:
            raise ValueError('零点采集的 IMU 反馈已过期或计时不一致')
        return state

    def joint_zero_action(self, body):
        fields = {'generation', 'action', 'id', 'support_confirmed', 'zero_reference_confirmed'}
        if (type(body) is not dict or set(body) != fields
                or type(body['id']) is not int or body['id'] not in joint_calibration.IDS
                or type(body['support_confirmed']) is not bool
                or type(body['zero_reference_confirmed']) is not bool
                or body['action'] not in ('capture', 'save', 'cancel')):
            raise ValueError('零点重采请求字段无效')
        with self.lock:
            if type(body['generation']) is not int or body['generation'] != self.generation:
                raise ValueError('零点重采会话已变化，请刷新页面')
            action, sid = body['action'], body['id']
            if action == 'cancel':
                if self._joint_zero_pending() and self.joint_zero['id'] != sid:
                    raise ValueError('请取消当前关节的零点预览，再选择其他关节')
                self.joint_zero = self._empty_joint_zero('cancelled', '未保存的零点预览已取消；已保存校准保持不变。')
                self.zero_collection = None
                return self.snapshot()
            if body['support_confirmed'] is not True or body['zero_reference_confirmed'] is not True:
                raise ValueError('请确认机身由稳固支撑承重，所选关节已按零姿态图摆稳')
            if self.collection is not None or self.wizard['busy']:
                raise ValueError('IMU 正在采样，请先完成或取消')
            if self.joint_wizard.active():
                raise ValueError('请先取消未保存的方向与范围采集，再重新采集零点')
            state = self._require_joint_zero_feedback()
            raw = joint_calibration.position(state['positions'][joint_calibration.IDS.index(sid)])
            if action == 'capture':
                if self._joint_zero_pending():
                    raise ValueError('已有未保存的零点采样或预览，请先保存或取消')
                self.joint_zero = dict(self._empty_joint_zero(), id=sid, stage='sampling', busy=True,
                                       old_zero=self.config['joints'][joint_calibration.IDS.index(sid)]['zero'],
                                       message='正在只读采样，请保持该关节的零姿态静止；采完后先预览。')
                self.zero_collection = dict(generation=self.generation, begin=self.clock(), samples=[],
                                            first_sample_us=None, first_imu_us=None,
                                            last_sample_us=state['sample_us'], last_imu_us=state['imu_us'],
                                            last_position=raw, before=copy.deepcopy(self.config))
            else:
                draft, capture = self.joint_zero, self.zero_collection
                if (draft['id'] != sid or draft['stage'] != 'ready' or draft['busy']
                        or capture is None or capture['generation'] != self.generation):
                    raise ValueError('所选关节尚无稳定采集的零点预览')
                if self.config != capture['before']:
                    self._fail_joint_zero('采集后校准配置已改变，请重新采集零点')
                    raise ValueError(self.joint_zero['error'])
                if abs(raw - draft['candidate_zero']) > 6:
                    self._fail_joint_zero('关节已离开刚才采集的零姿态，未保存；摆稳后重新采集')
                    raise ValueError(self.joint_zero['error'])
                updated = copy.deepcopy(self.config)
                updated['joints'][joint_calibration.IDS.index(sid)].update(
                    zero=draft['candidate_zero'], direction_confirmed=False, min=None, max=None)
                # Normalize only a final uncommitted intent after a previous
                # failed save. Its immutable audit copy remains on the Mac.
                anchor = dict(joints=copy.deepcopy(self.config['joints']))
                for joint, row in zip(anchor['joints'], self.zero_provenance['original_zeros']):
                    joint['zero'] = row['zero']
                provenance = validated_zero_provenance(self.zero_provenance, anchor, self.config)
                entry = dict(sequence=len(provenance['entries']) + 1, id=sid, generation=self.generation,
                             previous_sha256=None if not provenance['entries'] else provenance['entries'][-1]['sha256'],
                             before=copy.deepcopy(self.config), after=copy.deepcopy(updated))
                entry['sha256'] = _zero_receipt_digest(entry)
                provenance['entries'].append(entry)
                validated_zero_provenance(provenance, anchor, updated)
                try:
                    private_new(self.directory / ('joint-zero-authorized-' + secrets.token_hex(8) + '.json'),
                                json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
                    atomic_zero_provenance(self.directory, provenance)
                    self.zero_provenance = provenance
                    warning = self._write_calibration(updated)
                except (OSError, ValueError) as error:
                    message = '零点保存失败，旧校准保持不变：' + str(error)
                    self.joint_zero.update(error=message, message=message)
                    raise ValueError(message) from error
                self.config = updated
                self.original_zeros = joint_calibration.zero_signature(updated)
                self.zero_collection = None
                self.joint_zero.update(stage='saved', busy=False, progress=1., error=None,
                                       message=warning or f'ID {sid} 新零点已保存；仅该关节的方向与范围需重新采集，其他校准保留。')
            return self.snapshot()

    def _tick_joint_zero(self):
        if not self._joint_zero_pending():
            return
        try:
            state = self._require_joint_zero_feedback()
            capture, draft = self.zero_collection, self.joint_zero
            if capture is None or capture['generation'] != self.generation:
                raise ValueError('零点采样会话已变化，请重新采集')
            raw = joint_calibration.position(state['positions'][joint_calibration.IDS.index(draft['id'])])
            sample_delta = (state['sample_us'] - capture['last_sample_us']) & 0xffffffff
            imu_delta = (state['imu_us'] - capture['last_imu_us']) & 0xffffffff
            if sample_delta >= 0x80000000 or imu_delta >= 0x80000000:
                raise ValueError('零点采样时钟倒退，本次未保存')
            if sample_delta > 100000 or imu_delta > 100000:
                raise ValueError('零点连续反馈中断超过 100 毫秒，本次未保存')
            if abs(raw - capture['last_position']) >= 2048:
                raise ValueError('编码器跨零点或大跳变，本次未保存')
            if not sample_delta or not imu_delta:
                if not sample_delta and raw != capture['last_position']:
                    raise ValueError('同一采样时间出现不同编码器值，本次未保存')
                return
            capture.update(last_sample_us=state['sample_us'], last_imu_us=state['imu_us'], last_position=raw)
            if draft['stage'] == 'ready':
                if abs(raw - draft['candidate_zero']) > 6:
                    raise ValueError('关节已离开刚才采集的零姿态，未保存；摆稳后重新采集')
                return
            elapsed = self.clock() - capture['begin']
            if not 0 <= elapsed <= 3:
                raise ValueError('零点采样超过 3 秒，本次未保存；请重新采集')
            capture['samples'].append(raw)
            if capture['first_sample_us'] is None:
                capture.update(first_sample_us=state['sample_us'], first_imu_us=state['imu_us'])
            span = min((state['sample_us'] - capture['first_sample_us']) & 0xffffffff,
                       (state['imu_us'] - capture['first_imu_us']) & 0xffffffff)
            draft.update(sample_count=len(capture['samples']),
                         progress=min(.99, len(capture['samples']) / 12., elapsed / .35, span / 350000.))
            if len(capture['samples']) < 12 or elapsed < .35 or span < 350000:
                return
            if max(capture['samples']) - min(capture['samples']) > 6:
                raise ValueError('零点采样时关节还在移动，未保存；放稳后重新采集')
            candidate = int(round(float(np.median(capture['samples']))))
            draft.update(stage='ready', busy=False, candidate_zero=candidate, progress=1.,
                         message='稳定读数已采集，仅为预览。核对旧零点与新零点后点击保存；勿移动关节。')
        except (ValueError, KeyError) as error:
            self._fail_joint_zero(error)

    def joint_action(self, body):
        if (type(body) is not dict or set(body) != {
                'generation', 'action', 'id', 'support_confirmed', 'positive_confirmed'}
                or type(body['id']) is not int or body['id'] not in joint_calibration.IDS
                or type(body['support_confirmed']) is not bool or type(body['positive_confirmed']) is not bool
                or body['action'] not in ('direction-start', 'direction-end', 'range-a', 'range-b', 'save', 'save-direction', 'cancel')):
            raise ValueError('关节采集请求字段无效')
        with self.lock:
            if type(body['generation']) is not int or body['generation'] != self.generation:
                raise ValueError('关节采集会话已变化，请刷新页面')
            action, sid = body['action'], body['id']
            if action == 'cancel':
                self.joint_wizard.cancel()
                return self.snapshot()
            if self._joint_zero_pending():
                raise ValueError('请先保存或取消未完成的零点预览，再采集方向与范围')
            if body['support_confirmed'] is not True:
                raise ValueError('请确认机身由稳固支撑承重')
            if action in ('direction-end', 'save', 'save-direction') and body['positive_confirmed'] is not True:
                raise ValueError('请确认实际轻摆方向与模型正方向图一致')
            if self.collection is not None or self.wizard['busy']:
                raise ValueError('IMU 正在采样，请先完成或取消')
            state = self._require_joint_feedback()
            if action in ('save', 'save-direction'):
                updated = (self.joint_wizard.proposed_direction_config(self.config, sid)
                           if action == 'save-direction' else self.joint_wizard.proposed_config(self.config, sid))
                if (joint_calibration.zero_signature(updated) != self.original_zeros
                        or updated['imu'] != self.config['imu']):
                    raise ValueError('原零点或 IMU 发生意外变化，本关节未保存')
                try:
                    warning = self._write_calibration(updated)
                except OSError as error:
                    message = '关节校准保存失败，旧校准保持不变：' + str(error)
                    self.joint_wizard.value.update(error=message, message=message)
                    raise ValueError(message) from error
                self.config = updated
                self.joint_wizard.saved(warning or ('本关节正方向已保存；沿用已保存安全范围，零点、IMU 及其他关节保留。'
                                                   if action == 'save-direction' else None))
            else:
                self.joint_wizard.start(action, sid, state, self.clock())
            return self.snapshot()

    def _tick_joint(self):
        if not self.joint_wizard.active():
            return
        try:
            state = self._require_joint_feedback()
            self.joint_wizard.tick(state, self.clock(), self.config)
        except (ValueError, KeyError) as error:
            self.joint_wizard.fail(error)

    def action(self, body):
        if type(body) is not dict or set(body) != {
                'action', 'generation', 'mounted_confirmed', 'support_confirmed'}:
            raise ValueError('校准请求字段无效')
        action = body['action']
        if action not in ('level', 'front-up', 'left-up', 'finish', 'cancel'):
            raise ValueError('未知校准步骤')
        with self.lock:
            if type(body['generation']) is not int or body['generation'] != self.generation:
                raise ValueError('校准会话已变化，请刷新页面')
            if action == 'cancel':
                collection = self.collection
                previous_action = self.wizard.get('expected_action')
                self.collection, self.poses, self.axes = None, {}, None
                self.wizard = self._empty_wizard()
                self._audit_wizard('cancel', collection or dict(action=previous_action))
                return self.snapshot()
            if body['mounted_confirmed'] is not True or body['support_confirmed'] is not True:
                raise ValueError('请确认 IMU 已固定、躯干由稳固支撑承重')
            state = self._require_imu(body['generation'])
            if self.joint_wizard.active():
                raise ValueError('请先取消未完成的关节采集，再采样 IMU')
            if self._joint_zero_pending():
                raise ValueError('请先保存或取消未完成的零点预览，再采样 IMU')
            if self.wizard['busy']:
                raise ValueError('正在采样，先保持当前姿态')
            if action == 'level':
                self.poses, self.axes = {}, None
            elif action != self.wizard['expected_action']:
                raise ValueError('请按顺序完成四步校准')
            self.collection = dict(action=action, begin=self.clock(), samples=[],
                                   last_imu_us=state['imu_us'], last_sample_us=state['sample_us'])
            self.wizard.update(stage=action, expected_action=action, busy=True,
                               progress=0., sample_count=0, error=None,
                               message='采样中，请保持静止，通常约 4 秒。')
            self._audit_wizard('start', self.collection)
            return self.snapshot()

    def _scoped_online_mask(self, purpose):
        return 0 if purpose == 'imu' else 32767

    def _accept_scoped_state(self, message, state, generated):
        """No samples or unlocks until fresh progress proves this purpose."""
        purpose = self.feedback_scope
        if purpose is None:
            return True
        expected_online = self._scoped_online_mask(purpose)
        if expected_online is not None and state['online_mask'] != expected_online:
            raise ValueError('IMU 复查要求 URT2 断电、舵机全部离线；请结束后按用途重新复查。'
                             if purpose == 'imu' else
                             '关节复查要求 15 个舵机全部在线；请核对供电后重新复查。')
        if type(state.get('last_error')) is not int or state['last_error'] != 0:
            raise ValueError('复查反馈缺少有效的无故障状态，采集保持停止。')
        sequence = message.get('sequence')
        if type(sequence) is not int or not 2 <= sequence <= 0xffffffff:
            raise ValueError('复查反馈序号无效')
        age = (state['sample_us'] - state['imu_us']) & 0xffffffff
        if 500000 < age < 0x100000000 - 500000:
            raise ValueError('复查 IMU 与舵机采样时间不一致或已过期')
        point = dict(sequence=sequence, sample_us=state['sample_us'], imu_us=state['imu_us'],
                     generated=generated)
        previous = self._recheck_last
        if previous is not None:
            if sequence <= previous['sequence'] or generated <= previous['generated']:
                raise ValueError('复查反馈序号或时间未向前推进')
            deltas = [(point[key] - previous[key]) & 0xffffffff for key in ('sample_us', 'imu_us')]
            if any(delta >= 0x80000000 for delta in deltas):
                raise ValueError('复查传感器采样时钟倒退')
            if 0 in deltas:
                return False  # A repeated sensor sample is not fresh proof.
        self._recheck_last = point
        record = self.feedback_recheck
        if record['phase'] == 'passed':
            return True
        if self._recheck_first is None:
            self._recheck_first = point
        first = self._recheck_first
        span = min(generated - first['generated'],
                   ((point['sample_us'] - first['sample_us']) & 0xffffffff) / 1e6,
                   ((point['imu_us'] - first['imu_us']) & 0xffffffff) / 1e6)
        record['frame_count'] += 1
        record['span_ms'] = round(span * 1000, 3)
        if record['frame_count'] < 2 or span + 1e-9 < .2:
            return False
        # Quiescence alone never opens collection or resolves a servo fault.
        # A closure hold needs that receipt AND this guarded fresh window.
        hold = self.recovery_recheck
        reconciled = bool(hold is not None and hold == record['previous_hold']
            and self._connection_reconciled_generation == self.generation
            and type(record.get('connection_reconciled')) is dict)
        closure_cleared = reconciled and hold.get('category') == 'closure'
        trust_cleared = reconciled and hold.get('category') == 'trust'
        servo_cleared = bool(purpose == 'joints' and (
            self.pending_servo_hold is not None or (hold is not None
                and is_unsafe_feedback(hold['technical_reason']))))
        if hold is not None and not is_unsafe_feedback(hold['technical_reason']) and not (closure_cleared or trust_cleared):
            raise ValueError('板端检查仍被阻断，不能解除反馈锁')
        private_new(self.trace_directory / 'feedback-recheck-passed.json', json.dumps(dict(
            generation=self.generation, purpose=purpose, frame_count=record['frame_count'],
            span_ms=record['span_ms'], first=first, last=point,
            previous_hold=record['previous_hold'], calibration_preserved=True,
            pending_servo_hold=copy.deepcopy(self.pending_servo_hold),
            servo_hold_cleared=servo_cleared, connection_hold_cleared=closure_cleared,
            trust_hold_cleared=trust_cleared,
            connection_reconciled=record.get('connection_reconciled')),
            ensure_ascii=False, allow_nan=False) + '\n')
        if servo_cleared:
            self.pending_servo_hold = None
        if servo_cleared or closure_cleared or trust_cleared:
            self.recovery_recheck = copy.deepcopy(self.pending_servo_hold)
        record.update(phase='passed', error=None,
                      message=('IMU 反馈复查通过；仅开放 IMU 采集，舵机故障历史未清除。'
                               if purpose == 'imu' else
                               '关节反馈复查通过；15 个舵机在线、扭矩关闭，已开放手动采集。'))
        self.message = record['message']
        return True

    def accept(self, message):
        with self.lock:
            if message.get('generation') != self.generation:
                raise ValueError('反馈会话标识不匹配')
            kind = message.get('kind')
            if kind == 'ready':
                if self.ready is not None or self.status not in ('waiting', 'stopping', 'fault'):
                    raise ValueError('出现重复的真机连接')
                emitted = message.get('emitted_monotonic')
                if type(emitted) not in (int, float) or not math.isfinite(emitted):
                    raise ValueError('缺少真机反馈时钟证明')
                self.clock_offset = self.clock() - emitted
                self.ready = message
            elif kind == 'state':
                if self.ready is None:
                    raise ValueError('未就绪或已关闭的会话收到数据')
                state = message['state']
                self._validate_state(state)
                generated = message.get('received_monotonic')
                if type(generated) not in (int, float) or not math.isfinite(generated):
                    raise ValueError('传感器反馈时钟无效')
                if self.status in ('stopping', 'fault', 'closed'):
                    return  # A STATE already in flight is never collected again.
                lag = self.clock() - (generated + self.clock_offset)
                self.last_state = dict(arrived_monotonic=self.clock(),
                                       received_monotonic=generated, lag_ms=lag * 1000,
                                       sequence=message.get('sequence'),
                                       sample_us=state['sample_us'], imu_us=state['imu_us'])
                if lag > .2 or lag < -.05:
                    raise ValueError('真机反馈发生积压或传输计时异常，采样已停止')
                if not self._accept_scoped_state(message, state, generated):
                    return
                self.feedback = copy.deepcopy(state)
                self.received_at = self.clock() - max(0., lag)
                self.status = 'active'
                self._tick_wizard()
                self._tick_joint()
                self._tick_joint_zero()
            elif kind == 'fault':
                self.record_fault_feedback(message)
                self.fault(message.get('reason') or message.get('error') or '真机采样故障')
            elif kind == 'closed':
                self.record_fault_feedback(message)
                self.closed_proof = message
                self.feedback = None
                self._fail_joint_zero('采集连接已关闭，未保存的零点预览已取消。')
                if self.feedback_recheck is not None and message.get('serial_closed') is not True:
                    self.fault('复查连接的串口关闭未获确认')
                if self.status != 'fault':
                    if (message.get('normal') is True and message.get('serial_closed') is True
                            and self.end_reason is not None and self.end_request_sent):
                        self.status, self.error = 'stopping', None
                        self.message = '已收到串口关闭报告，正在确认采集进程退出。'
                    else:
                        self.fault(message.get('reason') or message.get('error') or '反馈连接异常结束')
                if self.collection is not None:
                    self._wizard_fail('采样会话已结束，已保存校准保持不变')
            else:
                raise ValueError('未知反馈消息')

    @staticmethod
    def _validate_state(state):
        for key in ('sample_us', 'imu_us', 'online_mask', 'fault_mask', 'torque_mask', 'arm_cookie'):
            if type(state.get(key)) is not int or not 0 <= state[key] <= 0xffffffff:
                raise ValueError('反馈整数无效')
        if (state.get('armed') is not False or state.get('imu_ok') is not True
                or state['arm_cookie'] or state['fault_mask'] or state['torque_mask']
                or state['online_mask'] not in (0, 32767)):
            raise ValueError('真机使能、舵机或 IMU 状态不适合只读采样')
        for key in ('accel_ms2', 'gyro_dps'):
            values = state.get(key)
            if type(values) is not list or len(values) != 3 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError('IMU 向量无效')
        if (type(state.get('positions')) is not list or len(state['positions']) != 15
                or any(type(v) is not int or not 0 <= v <= 4095 for v in state['positions'])):
            raise ValueError('编码器反馈无效')

    def _tick_wizard(self):
        c = self.collection
        if c is None:
            return
        try:
            state = self._require_imu(self.generation)
            elapsed = self.clock() - c['begin']
            if not 0 <= elapsed <= 8:
                raise ValueError('本步采样超时，放稳机身后重新点击')
            di = (state['imu_us'] - c['last_imu_us']) & 0xffffffff
            ds = (state['sample_us'] - c['last_sample_us']) & 0xffffffff
            if di >= 0x80000000 or ds >= 0x80000000:
                raise ValueError('传感器计时倒退，请重新开始校准')
            if di == 0 or ds == 0:
                return
            if di >= 100000 or ds >= 100000:
                raise ValueError('连续数据中断超过 100 毫秒，本步未保存')
            c['last_imu_us'], c['last_sample_us'] = state['imu_us'], state['sample_us']
            c['samples'].append(state['accel_ms2'] + state['gyro_dps'])
            self.wizard.update(sample_count=len(c['samples']),
                               progress=min(.99, len(c['samples']) / 100., elapsed / 1.8))
            if len(c['samples']) < 100 or elapsed < 1.8:
                return
            batch = imu_wizard.wizard_validate_batch(c['samples'])
            action = c['action']
            if action == 'level':
                dominant = np.zeros(3)
                dominant[int(np.argmax(np.abs(batch['acc'])))] = 1.
                if imu_wizard.wizard_angle(np.abs(batch['acc']), dominant) > 8:
                    raise ValueError('IMU 明显歪斜；模块应平贴躯干，机身放平后重试')
                self.poses['level'] = batch
                next_action, text = 'front-up', '第一步完成。抬高鸭嘴那端 20–40°，放稳后点击第二步。'
            elif action == 'front-up':
                self.axes = imu_wizard.wizard_find_axes(self.poses['level']['acc'], batch['acc'])
                self.poses[action] = batch
                next_action, text = 'left-up', '前后方向已识别。回到水平，再抬高鸭子自己的左侧。'
            elif action == 'left-up':
                imu_wizard.wizard_verify_pose(action, batch['acc'], self.axes)
                self.poses[action] = batch
                next_action, text = 'finish', '左右方向验证通过。回到水平，放稳后点击完成校准。'
            else:
                imu_wizard.wizard_verify_pose('level', batch['acc'], self.axes)
                if imu_wizard.wizard_angle(batch['acc'], self.poses['level']['acc']) > 5:
                    raise ValueError('与第一步水平姿态差异较大；请重新放平')
                updated = copy.deepcopy(self.config)
                updated['imu'] = dict(axes=list(self.axes), axes_confirmed=True,
                                      calibrated=True, bias_dps=list(batch['gyro']))
                if joint_calibration.zero_signature(updated) != self.original_zeros:
                    raise ValueError('关节零点发生意外变化')
                warning = self._write_calibration(updated)
                self.config = updated
                self.collection = None
                self.wizard.update(stage='complete', expected_action=None, busy=False,
                                   progress=1., message=warning or 'IMU 校准已保存到笔记本，15 个零点保持不变。',
                                   durability_warning=warning, error=None)
                self._audit_wizard('success', c)
                self.request_end('completed')
                return
            self.collection = None
            self.wizard.update(stage=next_action, expected_action=next_action, busy=False,
                               progress=1., message=text, error=None)
            self._audit_wizard('success', c)
        except Exception as error:
            if self.wizard['stage'] == 'complete':
                self.fault('校准已保存，但反馈关闭未确认：' + str(error))
            else:
                self._wizard_fail(error)

    def snapshot(self, *, include_motion=True, compact=False):
        with self.lock:
            age = None if self.received_at is None else max(0., self.clock() - self.received_at)
            fresh = self.status == 'active' and age is not None and age < FRESH
            feedback = copy.deepcopy(self.feedback) if fresh else None
            body = None
            imu = self.config['imu']
            if feedback is not None and imu['axes_confirmed'] and imu['calibrated']:
                matrix = next(m for axes, m in imu_wizard.wizard_rotations() if axes == imu['axes'])
                acc = matrix @ np.array(feedback['accel_ms2'])
                gyro = matrix @ (np.array(feedback['gyro_dps']) - np.array(imu['bias_dps']))
                body = dict(accel_ms2=acc.tolist(), gyro_dps=gyro.tolist(),
                            pitch_deg=math.degrees(math.atan2(acc[0], acc[2])),
                            roll_deg=math.degrees(math.atan2(acc[1], acc[2])))
            blocked = self.recovery_blocked_reason()
            recheck_blocked = self.recheck_blocked_reason()
            connection_blocked = self.connection_check_blocked_reason()
            connection_check = copy.deepcopy(self.connection_check)
            connection_check.update(allowed=connection_blocked is None, blocked_reason=connection_blocked)
            # Reconnection proofs contain complete kernel history. The live
            # page needs their result, not a new copy of that history each tick.
            # Full snapshots keep the original evidence unchanged.
            feedback_recheck = self.feedback_recheck
            if compact and isinstance(feedback_recheck, dict):
                feedback_recheck = {key: value for key, value in feedback_recheck.items()
                                    if key != 'connection_reconciled'}
                if 'connection_reconciled' in self.feedback_recheck:
                    feedback_recheck['connection_reconciled'] = dict(details_available=True)
            return dict(session=dict(status=self.status, error=self.error, error_details=self.error_details,
                                     generation=self.generation,
                                     message=self.message, end_reason=self.end_reason,
                                     recovery_allowed=blocked is None,
                                     recovery_pending=self.recovery_pending,
                                     recovery_blocked_reason=blocked,
                                     recovery_recheck=copy.deepcopy(self.recovery_recheck),
                                     recheck_allowed=recheck_blocked is None,
                                     recheck_blocked_reason=recheck_blocked,
                                     feedback_recheck=copy.deepcopy(feedback_recheck),
                                     connection_check=connection_check,
                                     fault_feedback=copy.deepcopy(self.fault_feedback),
                                     fault_feedback_invalid=self.fault_feedback_invalid,
                                     held_diagnostic=copy.deepcopy(self.held_diagnostic),
                                     fault_ids=None if self.fault_feedback is None else mask_ids(self.fault_feedback['fault_mask']),
                                     torque_ids=None if self.fault_feedback is None else mask_ids(self.fault_feedback['torque_mask']),
                                     received_age_ms=None if age is None else age * 1000,
                                     closed_proof=self.closed_proof),
                        connection_target=dict(kind=self.transport_config.kind,
                                               host=self.transport_config.host,
                                               page_url='http://127.0.0.1:8767/'),
                        feedback=feedback, body_imu=body, calibration=copy.deepcopy(self.config),
                        wizard=copy.deepcopy(self.wizard), joint_wizard=self.joint_wizard.snapshot(self.config),
                        joint_zero=copy.deepcopy(self.joint_zero),
                        motion=None if self.motion is None or not include_motion else self.motion.snapshot(),
                        maintenance=self.maintenance_snapshot(), token=self.token)


def handler_for(session, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def _host_ok(self):
            return self.headers.get('Host') in (f'127.0.0.1:{port}', f'localhost:{port}')

        def reply(self, code, value, kind='application/json; charset=utf-8', download=False):
            data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            if download:
                self.send_header('Content-Disposition', 'attachment; filename="duck-calibration.json"')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self._host_ok():
                return self.reply(403, {'error': '仅允许本机访问'})
            if self.path == '/':
                return self.reply(200, (HERE / 'feedback_calibration.html').read_bytes(), 'text/html; charset=utf-8')
            if self.path == '/api/status':
                return self.reply(200, session.snapshot())
            if self.path == '/api/calibration':
                return self.reply(200, session.snapshot()['calibration'], download=True)
            name = self.path.removeprefix('/assets/').removeprefix('/')
            if name in ('zero-reference.png', 'positive-direction.png') and self.path in ('/' + name, '/assets/' + name):
                return self.reply(200, (HERE.parent / 'x5_policy/assets' / name).read_bytes(), 'image/png')
            return self.reply(404, {'error': '找不到页面'})

        def do_POST(self):
            if not self._host_ok() or self.headers.get('Origin') not in (
                    f'http://127.0.0.1:{port}', f'http://localhost:{port}'):
                return self.reply(403, {'error': '仅允许本机同源请求'})
            token = self.headers.get('X-CSRF-Token', '')
            if not secrets.compare_digest(token, session.token):
                return self.reply(403, {'error': '页面会话已变化，请刷新'})
            if self.path not in ('/api/imu-wizard', '/api/end-feedback', '/api/recover-feedback', '/api/recheck-feedback', '/api/check-connection', '/api/joint-calibration', '/api/joint-zero', '/api/servo-maintenance', '/api/motion'):
                return self.reply(403, {'error': '不支持的工作台操作'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096 or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('请求大小或类型无效')
                body = strict_json(self.rfile.read(size))
                action = {'/api/imu-wizard': session.action, '/api/end-feedback': session.end_action,
                          '/api/recover-feedback': session.recover_action,
                          '/api/recheck-feedback': session.recheck_feedback_action,
                          '/api/check-connection': session.check_connection_action,
                          '/api/joint-calibration': session.joint_action,
                          '/api/joint-zero': session.joint_zero_action,
                          '/api/servo-maintenance': session.maintenance_action,
                          '/api/motion': session.motion_action}[self.path]
                result = action(body)
            except (ValueError, KeyError) as error:
                return self.reply(409, {'error': str(error)})
            return self.reply(200, result)
    return Handler


def persist_reader_record(session, directory, line, first_state_saved):
    message = strict_json(line)
    if type(message) is not dict:
        raise ValueError('反馈消息不是 JSON 对象')
    # Preserve fault/closed reports even if applying them rejects this session.
    if message.get('kind') in ('ready', 'fault', 'closed'):
        private_new(directory / (message['kind'] + '-' + secrets.token_hex(4) + '.json'), line)
    was_collecting = session.status in ('waiting', 'active')
    session.accept(message)
    if message.get('kind') == 'state' and not first_state_saved and was_collecting:
        private_new(directory / 'first-state.json', line)
        first_state_saved = True
    return first_state_saved


def bounded_stop(process):
    if process is not None and process.poll() is None:
        try:
            process.terminate()
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def reader_launch_preflight(session, source, known_hosts, directory, generation, *, transport_config=None):
    """Local checks only; this function cannot create an SSH child."""
    helper_path = RECOVERY / 'run_postflash_passive.py'
    spec = importlib.util.spec_from_file_location('local_capture_helper', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    if helper.known_hosts_fingerprint(known_hosts.read_text()) != helper.EXPECTED_FINGERPRINT:
        raise ValueError('X5 的 SSH 身份已变化')
    transport = network_transport.resolve_transport(
        getattr(session, 'transport_config', None) if transport_config is None else transport_config)
    route = subprocess.run(transport.route_command(), capture_output=True, text=True, timeout=2)
    transport.verify_route(route, {'parse_route': helper.parse_route})
    private_new(directory / 'hardware-attempt.json', json.dumps(dict(
        source_sha256=hashlib.sha256(source).hexdigest(), generation=generation,
        port='discover_unique_ch340' if getattr(session, 'serial_discovery_allowed', False) else '/dev/ttyUSB0',
        network=dict(kind=transport.kind, host=transport.host, interface=transport.interface),
        requests=['HELLO', 'STATE'], no_reconnect=True,
        end_control='END_FEEDBACK', source_bytes=len(source))) + '\n')
    private_new(directory / 'remote-source.py', source)
    askpass = directory / 'askpass.sh'
    private_new(askpass, '#!/bin/sh\nexec /usr/bin/printenv X5_DIAGNOSTIC_PASSWORD\n')
    askpass.chmod(0o700)
    remote_command = 'exec python3 -I -B -S -c ' + shlex.quote(reader_bootstrap(len(source)))
    command = ['ssh', '-F', '/dev/null', '-T', '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1',
               '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=1', '-o', 'NumberOfPasswordPrompts=1',
               '-o', 'PreferredAuthentications=password', '-o', 'PubkeyAuthentication=no',
               '-o', 'StrictHostKeyChecking=yes', '-o', 'UserKnownHostsFile=' + str(known_hosts),
               '-o', 'GlobalKnownHostsFile=/dev/null', '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
               '-o', 'ControlPersist=no', '-o', 'LogLevel=ERROR',
               *transport.ssh_options(), transport.target, remote_command]
    environment = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force',
                       DISPLAY='codex-real-feedback', X5_DIAGNOSTIC_PASSWORD='root')
    return command, environment, askpass


def run_reader(session, source, known_hosts, directory, *, transport_config=None, launch_preflight=None):
    with session.lock:
        generation, done = session.generation, session.connection_done
        if session.process is not None:
            raise ValueError('上次采集进程尚未释放，未启动新连接')
        session._reader_preflight_failure_generation = None
    try:
        transport_options = {} if transport_config is None else dict(transport_config=transport_config)
        if launch_preflight is None:
            command, environment, askpass = reader_launch_preflight(
                session, source, known_hosts, directory, generation, **transport_options)
        else:
            command, environment, askpass = launch_preflight(
                session, source, known_hosts, directory, generation)
    except Exception as error:
        # No SSH call exists above this boundary. Retain this fact instead
        # of asking for a remote UART close report from a nonexistent child.
        with session.lock:
            if session.current_attempt(generation, None):
                private_new(directory / 'local-preflight-failure.json', json.dumps(dict(
                    generation=generation, error=str(error), ssh_start_attempted=False,
                    source_bytes_sent=0, automatic_retry=False), ensure_ascii=False) + '\n')
                session._reader_preflight_failure_generation = generation
                session.fault(error)
                session.message = '本次连接在笔记本检查阶段停止，未启动 SSH 或打开串口；已保存校准保留。'
                done.set()
        (directory / 'askpass.sh').unlink(missing_ok=True)
        raise
    process = None
    source_fully_sent = False
    try:
        with session.lock:
            if session.generation != generation or session.status != 'waiting':
                raise ValueError('本次启动已取消，未打开新的硬件连接')
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, env=environment, start_new_session=True)
            session.process = process
            descriptor = process.stdin.fileno()
            os.set_blocking(descriptor, False)
            session.source_transmitting = True
        # Keep UI snapshots responsive. An end/fault during this bounded
        # transfer changes session state immediately but defers the one END
        # record until the exact-N source bytes have finished.
        write_pipe_once(descriptor, source, 8)
        source_fully_sent = True
        session.source_finished(generation, process)
    except BaseException as error:
        # SSH can fail before the normal stderr/record readers start. Retain
        # its real exit and bounded output instead of exposing only EPIPE.
        # No local exit is treated as proof that the remote UART closed.
        early_stderr, early_stdout = bytearray(), bytearray()
        stdout_truncated = False
        cleanup_error = None
        streams = {}
        if process is not None:
            for name in ('stdout', 'stderr'):
                stream = getattr(process, name, None)
                if stream is not None and not stream.closed:
                    descriptor = stream.fileno()
                    os.set_blocking(descriptor, False)
                    streams[descriptor] = name

        def drain_early(seconds):
            nonlocal stdout_truncated
            deadline = time.monotonic() + seconds
            while streams and time.monotonic() < deadline:
                readable, _, _ = select.select(list(streams), [], [],
                                               max(0., deadline - time.monotonic()))
                for descriptor in readable:
                    try:
                        data = os.read(descriptor, 4096)
                    except BlockingIOError:
                        continue
                    if not data:
                        streams.pop(descriptor)
                    elif streams[descriptor] == 'stderr':
                        early_stderr.extend(data)
                        del early_stderr[:-4096]
                    else:
                        room = max(0, 262144 - len(early_stdout))
                        early_stdout.extend(data[:room])
                        stdout_truncated = stdout_truncated or len(data) > room

        cleanup_errors = []
        try:
            # A failed output drain must not skip stopping this exact child.
            # Fully transmitted fault/CLOSED records remain real evidence.
            for stage, action in (
                    ('first_drain', lambda: drain_early(1.)),
                    ('child_stop', lambda: bounded_stop(process)),
                    ('final_drain', lambda: drain_early(.5))):
                try:
                    action()
                except (OSError, ValueError, subprocess.TimeoutExpired) as stop_error:
                    cleanup_errors.append(stage + ': ' + str(stop_error))
            cleanup_error = '; '.join(cleanup_errors) or None
        finally:
            for name in ('stdin', 'stdout', 'stderr'):
                stream = None if process is None else getattr(process, name, None)
                if stream is not None and not stream.closed:
                    stream.close()
            askpass.unlink(missing_ok=True)

        exit_code = None if process is None else process.poll()
        stderr_text = bytes(early_stderr).decode('utf-8', 'replace')
        diagnostic = ('Connection-check SSH transport failed: '
                      + (stderr_text.strip() or str(error)) + '; exit ' + str(exit_code))
        if any(text in stderr_text.lower() for text in ('remote host identification has changed',
                                                        'host key verification failed')):
            diagnostic = 'Pinned SSH host identity changed: ' + stderr_text.strip() + '; exit ' + str(exit_code)
        with session.lock:
            owns_attempt = session.current_attempt(generation, process)
            if owns_attempt:
                session.source_transmitting = False
                session.stderr = early_stderr
                if source_fully_sent:
                    first_state_saved = False
                    for line in bytes(early_stdout).splitlines(keepends=True):
                        if not line.endswith(b'\n'):
                            session.fault('反馈消息过大或不完整')
                            break
                        try:
                            first_state_saved = persist_reader_record(
                                session, directory, line, first_state_saved)
                        except (ValueError, KeyError, OSError) as record_error:
                            session.fault(record_error)
                    if stdout_truncated:
                        session.fault('反馈消息过大或不完整')
                session.fault(diagnostic)
                if process is not None and session.closed_proof is None:
                    session.fault('真机反馈连接已结束，未收到完整关闭报告')
            # Save raw transport evidence even if the session generation
            # changed during transmission. Never attach it to the new owner.
            private_new(directory / 'source-transmission-failure.json', json.dumps(dict(
                generation=generation, source_bytes=len(source), source_fully_sent=source_fully_sent,
                error=str(error), transport_error=diagnostic, ssh_exit_code=exit_code,
                cleanup_error=cleanup_error, stdout_truncated=stdout_truncated,
                stdout=bytes(early_stdout).decode('utf-8', 'replace'), stderr=stderr_text,
                closed_proof=session.closed_proof if owns_attempt else None,
                closure_not_inferred=True), ensure_ascii=False) + '\n')
            if owns_attempt:
                private_new(directory / 'connection-finished.json', json.dumps(dict(
                    generation=generation, ssh_exit_code=exit_code, session_status=session.status,
                    error=session.error, error_details=session.error_details,
                    source_fully_sent=source_fully_sent, closed_proof=session.closed_proof,
                    end_reason=session.end_reason, recovery_recheck=session.recovery_recheck,
                    pending_servo_hold=session.pending_servo_hold, stderr=stderr_text)) + '\n')
                session.write_fault_evidence(directory, process)
                if process is None or exit_code is not None:
                    done.set()
        raise ValueError(diagnostic) from error

    stderr = bytearray()

    def owner():
        return session.current_attempt(generation, process)

    def stderr_reader():
        while True:
            data = process.stderr.read(1024)
            if not data:
                return
            with session.lock:
                stderr.extend(data)
                del stderr[:-4096]
                if owner():
                    session.stderr = stderr

    def watch():
        while process.poll() is None:
            time.sleep(.1)
            with session.lock:
                if not owner():
                    return
                session.check_watchdog()
                cleanup = session.cleanup_due()
            if cleanup:
                bounded_stop(process)
                return

    stderr_thread = threading.Thread(target=stderr_reader, daemon=True)
    watch_thread = threading.Thread(target=watch, daemon=True)

    def consume():
        first_state_saved = False
        try:
            while True:
                line = process.stdout.readline(262145)
                if not line:
                    break
                if len(line) > 262144 or not line.endswith(b'\n'):
                    raise ValueError('反馈消息过大或不完整')
                with session.lock:
                    if not owner():
                        return
                    try:
                        first_state_saved = persist_reader_record(session, directory, line, first_state_saved)
                    except (ValueError, KeyError, OSError) as error:
                        # Fault hides readings immediately. Keep consuming
                        # bounded records so an already queued close survives.
                        session.fault(error)
            exit_code = process.wait(timeout=2)
            with session.lock:
                if owner():
                    if session.closed_proof is None:
                        session.fault('真机反馈连接已结束，未收到完整关闭报告')
                    elif exit_code != 0:
                        session.fault('反馈连接退出状态异常，关闭报告保持原样供核对。')
                    elif session.status == 'stopping':
                        session.confirm_closed(exit_code)
        except BaseException as error:
            with session.lock:
                if owner():
                    session.fault(error)
        finally:
            # The watch thread allows at most two seconds for raw closure
            # evidence, then terminates/waits/kills this exact child only.
            watch_thread.join(timeout=6.5)
            if process.poll() is None:
                bounded_stop(process)
            stderr_thread.join(timeout=2)
            watch_thread.join(timeout=2)
            askpass.unlink(missing_ok=True)
            with session.lock:
                if owner():
                    try:
                        private_new(directory / 'connection-finished.json', json.dumps(dict(
                            generation=generation, ssh_exit_code=process.poll(), session_status=session.status,
                            error=session.error, error_details=session.error_details,
                            closed_proof=session.closed_proof, end_reason=session.end_reason,
                            recovery_recheck=session.recovery_recheck,
                            pending_servo_hold=session.pending_servo_hold,
                            stderr=bytes(stderr).decode('utf-8', 'replace'))) + '\n')
                        session.write_fault_evidence(directory, process)
                    except BaseException as error:
                        session.fault('笔记本关闭证据写入失败：' + str(error))
                    finally:
                        if process.poll() is not None:
                            process.stdin.close()
                        # A mere local child exit does not certify UART
                        # closure. The next explicit request repeats all guards.
                        if (process.poll() is not None and not stderr_thread.is_alive()
                                and not watch_thread.is_alive()):
                            done.set()

    stderr_thread.start()
    watch_thread.start()
    threading.Thread(target=consume, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--x5-network', choices=('usb', 'wifi'), default='usb',
                        help='X5 communication path; the page remains on localhost')
    parser.add_argument('--x5-host', help='Explicit X5 IPv4 address for Wi-Fi')
    parser.add_argument('--x5-interface', help='Pinned Mac route interface for this connection')
    parser.add_argument('--deploy-motion', action='store_true', help='Deploy X5 RAM models after a paused, calibrated startup; no motor start')
    parser.add_argument('--discover-esp32', action='store_true', help='Find the unique reviewed CH340 at explicit connection, then pin this session to its current port')
    parser.add_argument('--reviewed-baseline', type=Path)
    parser.add_argument('--reviewed-baseline-sha256')
    parser.add_argument('--start-paused', action='store_true',
                        help='Start only the local page; connect only after an explicit recovery click')
    parser.add_argument('--recheck-joints-at-start', action='store_true',
                        help='Explicit one-time read-only joint recheck after a paused restore; no motor start')
    parser.add_argument('--resume-run', type=Path,
                        help='Preserve the complete calibration from a private prior local run')
    parser.add_argument('--resume-stopped-run', type=Path,
                        help='Carry an audited exited feedback transport and missing UART closure into the paused page')
    parser.add_argument('--resume-motion-run', type=Path,
                        help='Carry one audited exited motion transport and its uncertain UART ownership into a paused page')
    parser.add_argument('--reconcile-motion-at-start', action='store_true',
                        help='Explicit one-shot metadata-only check of the restored exited motion; no UART or motor commands')
    parser.add_argument('--hold-unsafe-feedback', action='store_true',
                        help='Keep the manually known unsafe-feedback condition blocked; never connect')
    parser.add_argument('--held-diagnostic-report', type=Path,
                        help='Pinned private historical diagnostic for a held page; never live feedback')
    parser.add_argument('--held-diagnostic-sha256')
    args = parser.parse_args()
    if args.port != 8767:
        parser.error('Only the reviewed local port 8767 is allowed')
    if args.x5_network == 'wifi' and args.x5_host is None:
        parser.error('--x5-network wifi requires an explicit --x5-host')
    try:
        transport = network_transport.TransportConfig(
            kind=args.x5_network,
            host=args.x5_host or '192.168.128.10',
            interface=args.x5_interface or ('en0' if args.x5_network == 'wifi' else 'en5'))
    except (TypeError, ValueError) as error:
        parser.error(str(error))
    if (args.reviewed_baseline is None) != (args.reviewed_baseline_sha256 is None):
        parser.error('Provide both --reviewed-baseline and --reviewed-baseline-sha256')
    if args.resume_run is not None and not args.start_paused:
        parser.error('--resume-run requires --start-paused')
    if args.recheck_joints_at_start and (
            not args.start_paused or args.resume_run is None
            or args.resume_stopped_run is not None or args.resume_motion_run is not None or args.deploy_motion):
        parser.error('--recheck-joints-at-start requires a paused calibration restore with no uncertain transport or motion deployment')
    if args.resume_stopped_run is not None and (
            not args.start_paused or args.resume_run != args.resume_stopped_run
            or args.resume_motion_run is not None
            or args.hold_unsafe_feedback or args.held_diagnostic_report is not None):
        parser.error('--resume-stopped-run requires --start-paused and the same --resume-run, without replacement holds')
    if args.resume_motion_run is not None and (
            not args.start_paused or args.resume_run is None
            or args.resume_motion_run.absolute().parent != args.resume_run.absolute()
            or (args.deploy_motion and not args.reconcile_motion_at_start) or args.recheck_joints_at_start):
        parser.error('--resume-motion-run requires --start-paused and its parent --resume-run, without automatic hardware startup')
    if args.reconcile_motion_at_start and (args.resume_motion_run is None or not args.start_paused):
        parser.error('--reconcile-motion-at-start requires a paused restored motion session')
    if args.hold_unsafe_feedback and (not args.start_paused or args.resume_run is None):
        parser.error('--hold-unsafe-feedback requires --start-paused and --resume-run')
    if (args.held_diagnostic_report is None) != (args.held_diagnostic_sha256 is None):
        parser.error('Provide both --held-diagnostic-report and --held-diagnostic-sha256')
    if args.held_diagnostic_report is not None and (
            not args.start_paused or args.resume_run is None or not args.hold_unsafe_feedback):
        parser.error('Historical diagnostics require --start-paused, --resume-run and --hold-unsafe-feedback')
    held_diagnostic, held_raw = None, None
    if args.held_diagnostic_report is not None:
        held_diagnostic, held_raw = load_held_diagnostic_report(
            args.held_diagnostic_report, args.held_diagnostic_sha256)
    directory = args.run_dir.absolute()
    if directory.parent.resolve() != RECOVERY or not directory.name.startswith('feedback-calibration-'):
        parser.error('Use a fresh feedback-calibration-* directory in the replacement-board evidence folder')
    directory.mkdir(mode=0o700, exist_ok=False)
    private_new(directory / 'transport-config.json', json.dumps(dict(
        kind=transport.kind, host=transport.host, interface=transport.interface,
        user=transport.user, port=transport.port, host_key_alias=transport.host_key_alias,
        page_url='http://127.0.0.1:8767/', automatic_retry=False,
        automatic_transport_fallback=False), ensure_ascii=False, indent=2) + '\n')
    if held_raw is not None:
        private_new(directory / 'held-diagnostic-report.json', held_raw)
    raw = SAVED.read_bytes()
    if hashlib.sha256(raw).hexdigest() != SAVED_SHA:
        raise ValueError('已保存零点文件与核对摘要不符')
    config = strict_json(raw)
    calibration_restore.validate_calibration_snapshot(config)
    private_new(directory / 'calibration-original.json', raw)
    zero_provenance = None
    if args.resume_run is not None:
        config, resume_raw, zero_provenance = load_resume_calibration(
            args.resume_run, config, include_zero_provenance=True)
        private_new(directory / 'resume-source.json', json.dumps(dict(
            source_run=str(args.resume_run), source_sha256=hashlib.sha256(resume_raw).hexdigest(),
            zeros_equal_original=joint_calibration.zero_signature(config) ==
                                 joint_calibration.zero_signature(strict_json(raw)),
            explicit_zero_provenance=zero_provenance is not None, joints_preserved_from_resume=True,
            hardware_started=False, closure_not_inferred=True)) + '\n')
        if zero_provenance is not None:
            # The normalized ledger excludes at most one interrupted save;
            # retain the exact source journal separately for audit as well.
            private_new(directory / 'joint-zero-resume-source.json',
                        _private_json_bytes(args.resume_run / 'joint-zero-provenance.json'))
            atomic_zero_provenance(directory, zero_provenance)
    atomic_calibration(directory, config)
    result = strict_json((RECOVERY / 'postflash-powered-query-20261004-01/result.json').read_bytes())
    if result['status'] != 'communication_snapshot_complete':
        raise ValueError('通电通信复查尚未完成')
    previous = result['diagnostic_reply']
    generation = secrets.randbelow(0x7ffffffe) + 1
    expected = dict(boot_id=previous['before']['boot_id'], cid=previous['identities']['card']['cid'],
                    root_mount_line=previous['before']['root_mount'], model=previous['identities']['board_model'],
                    serial=dict(path='/dev/ttyUSB0', vid='1a86', pid='7523', sysfs_name='1-1.3', serial_number=''),
                    kernel_prefix=previous['after']['kernel_log'],
                    historical_faults=previous['after']['kernel_faults'], generation=generation)
    # Generation/identity are reviewed again on the X5, without certifying storage.
    if expected['boot_id'] != 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab' or expected['cid'] != '035344534b33324785d55cb687017c00':
        raise ValueError('板子或卡的身份不匹配')
    import feedback_stream
    original_boot = expected['boot_id']
    expected, profile_raw = reviewed_expectations(expected, args.reviewed_baseline,
                                                  args.reviewed_baseline_sha256)
    if args.discover_esp32:
        expected['discover_serial'] = True
    if profile_raw is not None:
        private_new(directory / 'reviewed-baseline.json', profile_raw)
        private_new(directory / 'reviewed-baseline-record.json', json.dumps(dict(
            source_path=str(args.reviewed_baseline), sha256=args.reviewed_baseline_sha256,
            copy_sha256=hashlib.sha256(profile_raw).hexdigest(),
            applied_fields=['kernel_prefix', 'historical_faults'] + (
                ['boot_id'] if expected['boot_id'] != original_boot else []),
            boot_generation_changed=expected['boot_id'] != original_boot,
            original_boot_id=original_boot, reviewed_boot_id=expected['boot_id'],
            board_identity_changed=False, calibration_changed=False)) + '\n')
    private_new(directory / 'expectations.json', json.dumps(expected, ensure_ascii=False, indent=2) + '\n')
    stopped_record = None
    if args.resume_stopped_run is not None:
        stopped_record = load_stopped_session(args.resume_stopped_run, expected)
        generation = stopped_record['generation']
        private_new(directory / 'stopped-session-source.json',
                    json.dumps(stopped_record, ensure_ascii=False, allow_nan=False) + '\n')

    def launch(current, attempt_generation, attempt_directory):
        attempt_expected = copy.deepcopy(expected)
        attempt_expected['generation'] = attempt_generation
        # Revalidate the same explicitly pinned artifact on every manual
        # request. This never learns a baseline from the current board logs.
        attempt_expected, _ = reviewed_expectations(attempt_expected, args.reviewed_baseline,
                                                    args.reviewed_baseline_sha256)
        if attempt_directory != directory:
            private_new(attempt_directory / 'expectations.json',
                        json.dumps(attempt_expected, ensure_ascii=False, indent=2) + '\n')
        kwargs = ({} if args.reviewed_baseline is None else dict(
            reviewed_profile_path=args.reviewed_baseline,
            reviewed_profile_sha256=args.reviewed_baseline_sha256))
        source = feedback_stream.remote_source(attempt_expected, **kwargs).encode()
        run_reader(current, source, RECOVERY / 'known_hosts', attempt_directory, transport_config=transport)

    def connection_checker(previous_generation, check_directory):
        import feedback_connection_check
        check_expected = copy.deepcopy(expected)
        check_expected['generation'] = previous_generation
        check_expected, _ = reviewed_expectations(check_expected, args.reviewed_baseline,
                                                  args.reviewed_baseline_sha256)
        return feedback_connection_check.check_connection(
            check_expected, RECOVERY / 'known_hosts', check_directory,
            reviewed_profile_path=args.reviewed_baseline,
            reviewed_profile_sha256=args.reviewed_baseline_sha256, transport_config=transport)

    session = CalibrationSession(config, directory, generation, recovery_launcher=launch,
                                 zero_provenance=zero_provenance, connection_checker=connection_checker,
                                 transport_config=transport)
    session.serial_discovery_allowed = args.discover_esp32
    import maintenance_transport
    def maintenance_factory():
        maintenance_expected = copy.deepcopy(expected)
        maintenance_expected['generation'] = secrets.randbelow(0x7ffffffe) + 1
        return maintenance_transport.make_transport_factory(
            maintenance_expected, args.reviewed_baseline, args.reviewed_baseline_sha256,
            RECOVERY / 'known_hosts', directory, transport_config=transport)()
    session.configure_maintenance(maintenance_factory, hold_directory=RECOVERY)
    session.configure_motion(expected, args.reviewed_baseline, args.reviewed_baseline_sha256,
                             RECOVERY / 'known_hosts', transport_config=transport)
    if args.resume_motion_run is not None:
        session.motion.restore_uncertain_motion(args.resume_motion_run, config)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(session, args.port))
    def stop(_number, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, stop)
    try:
        if stopped_record is not None:
            session.restore_stopped(stopped_record)
        else:
            session.start_initial(paused=args.start_paused, hold_unsafe=args.hold_unsafe_feedback,
                                  held_diagnostic=held_diagnostic)
        motion_reconciled = args.resume_motion_run is None
        if args.reconcile_motion_at_start:
            try:
                session.motion.action(dict(action='disconnect'))
                motion_reconciled = session.motion.snapshot()['exit_reconciled'] is True
            except ValueError as error:
                session.motion.error = str(error)
                private_new(directory / 'motion-reconciliation-refused.json', json.dumps(dict(
                    error=str(error), old_owner_hold_preserved=True,
                    serial_open_attempts=0, motor_start_attempted=False), ensure_ascii=False) + '\n')
        if args.hold_unsafe_feedback:
            private_new(directory / 'startup-hold.json', json.dumps(dict(
                source='explicit_operator_unsafe_feedback_hold', hardware_started=False,
                technical_reason=UNSAFE_FEEDBACK_REASON, fault_feedback=None,
                held_diagnostic=copy.deepcopy(held_diagnostic),
                held_diagnostic_source_sha256=None if held_diagnostic is None else held_diagnostic['source_sha256'],
                held_diagnostic_copy_sha256=None if held_raw is None else hashlib.sha256(held_raw).hexdigest(),
                    calibration_preserved=True, closure_not_inferred=True)) + '\n')
        if args.recheck_joints_at_start:
            # This explicit CLI operation follows the identical purpose,
            # closure and live-servo gates used by the page's recheck button.
            # Existing servo holds are cleared only by verified new feedback.
            session.recheck_feedback_action(dict(generation=session.generation, purpose='joints'))
        private_new(directory / 'startup-state.json',
                    json.dumps(session.snapshot(), ensure_ascii=False, allow_nan=False) + '\n')
        if args.deploy_motion:
            try:
                if not motion_reconciled:
                    raise ValueError('旧运控串口关闭尚未确认，未启动新的模型连接')
                if session.maintenance_fault_hold:
                    # Explicit deployment includes one actual feedback review.
                    # Only the existing reviewed_callback can archive the hold.
                    session.maintenance_action(dict(action='review', generation=session.generation))
                    deadline = time.monotonic() + 10
                    while session.maintenance.snapshot()['busy'] and time.monotonic() < deadline:
                        time.sleep(.05)
                    if session.maintenance_fault_hold or session.maintenance.owns_connection:
                        raise ValueError('舵机反馈复查未通过，原故障锁保留；请查看舵机工作台的实际原因')
                session.motion_action(dict(action='connect', page_generation=session.generation))
            except ValueError as error:
                session.motion.error, session.motion.phase = str(error), 'fault'
                private_new(directory / 'motion-startup-refused.json', json.dumps(dict(
                    error=str(error), hold_preserved=session.maintenance_fault_hold,
                    motor_start_attempted=False), ensure_ascii=False) + '\n')
        print(json.dumps(dict(url='http://127.0.0.1:8767/', scope='calibration_maintenance_and_explicit_x5_motion',
                              automatic_motion=False, start_paused=args.start_paused,
                              previous_transport_uncertainty_preserved=stopped_record is not None,
                              previous_motion_uncertainty_preserved=args.resume_motion_run is not None,
                              x5_network=transport.kind, x5_host=transport.host,
                              directory=str(directory))), flush=True)
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        if session.motion is not None:
            session.motion.close()
        if session.maintenance is not None:
            session.maintenance.close()
        if session.process is not None and session.process.poll() is None:
            if session.status in ('waiting', 'active'):
                session.request_end('manual')
            if session.status in ('stopping', 'fault') and not session.connection_done.wait(timeout=2):
                session.fault('关闭页面服务时未收到完整反馈关闭证明，当前连接已停止。')
        bounded_stop(session.process)
        if session.process is not None and session.process.stdin is not None:
            session.process.stdin.close()


if __name__ == '__main__':
    main()
