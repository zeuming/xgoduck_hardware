"""Explicit, digest-bound restoration of this robot's calibration into RAM."""

import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import re
import stat


def _restore_json_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('校准备份包含重复字段')
        result[key] = value
    return result


def validate_calibration_snapshot(cfg):
    if (type(cfg) is not dict or set(cfg) != {'version', 'mechanical_reference_confirmed', 'joints', 'imu'}
            or type(cfg['version']) is not int or cfg['version'] != 1
            or type(cfg['mechanical_reference_confirmed']) is not bool):
        raise ValueError('校准备份版本或字段无效')
    ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
    joints = cfg['joints']
    if type(joints) is not list or len(joints) != 15:
        raise ValueError('校准备份必须包含本机 15 个关节')
    for sid, joint in zip(ids, joints):
        if (type(joint) is not dict or set(joint) != {'id', 'zero', 'sign', 'min', 'max', 'direction_confirmed'}
                or type(joint['id']) is not int or joint['id'] != sid
                or type(joint['sign']) is not int or joint['sign'] not in (-1, 1)
                or type(joint['direction_confirmed']) is not bool):
            raise ValueError('校准备份关节顺序或方向无效')
        for key in ('zero', 'min', 'max'):
            value = joint[key]
            if value is not None and (type(value) is not int or not 0 <= value <= 4095):
                raise ValueError('校准备份编码器值无效')
        if joint['min'] is not None and joint['max'] is not None:
            if joint['min'] >= joint['max'] or (joint['zero'] is not None
                    and not joint['min'] <= joint['zero'] <= joint['max']):
                raise ValueError('校准备份关节安全范围无效')
    imu = cfg['imu']
    if (type(imu) is not dict or set(imu) != {'axes', 'axes_confirmed', 'calibrated', 'bias_dps'}
            or type(imu['axes_confirmed']) is not bool or type(imu['calibrated']) is not bool
            or imu['calibrated'] and not imu['axes_confirmed']):
        raise ValueError('校准备份 IMU 确认字段无效')
    axes = imu['axes']
    allowed = {'x', 'y', 'z', '-x', '-y', '-z'}
    if (type(axes) is not list or len(axes) != 3 or any(type(a) is not str or a not in allowed for a in axes)
            or len({a[-1] for a in axes}) != 3):
        raise ValueError('校准备份 IMU 轴向无效')
    permutation = ['xyz'.index(a[-1]) for a in axes]
    inversions = sum(permutation[i] > permutation[j] for i in range(3) for j in range(i + 1, 3))
    if (-1) ** (inversions + sum(a.startswith('-') for a in axes)) != 1:
        raise ValueError('校准备份 IMU 轴向必须为右手坐标系')
    bias = imu['bias_dps']
    if (type(bias) is not list or len(bias) != 3 or any(type(v) not in (int, float)
            or not math.isfinite(v) or abs(v) > 5 for v in bias)):
        raise ValueError('校准备份 IMU 偏置无效')
    return cfg


def validate_restore_record(record, *, boot_id, cid, serial_device):
    if (type(record) is not dict or set(record) != {'snapshot_utf8', 'sha256', 'boot_id', 'cid', 'serial_device'}
            or record['boot_id'] != boot_id or record['cid'] != cid or record['serial_device'] != serial_device):
        raise ValueError('校准备份与当前板子、SD 卡或串口不匹配')
    raw = record['snapshot_utf8']
    digest = record['sha256']
    if (type(raw) is not str or not 0 < len(raw.encode('utf-8')) <= 16384 or type(digest) is not str
            or not re.fullmatch(r'[0-9a-f]{64}', digest)
            or hashlib.sha256(raw.encode('utf-8')).hexdigest() != digest):
        raise ValueError('校准备份摘要无效')
    cfg = json.loads(raw, object_pairs_hook=_restore_json_pairs)
    return validate_calibration_snapshot(cfg)


def read_restore_record(path, expected_sha256, review):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError('必须显式指定本机私有校准备份的绝对路径')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid()
                or before.st_mode & 0o077 or not 0 < before.st_size <= 16384):
            raise ValueError('校准备份必须是当前用户的私有小文件')
        raw = os.read(fd, 16385)
        after = os.fstat(fd)
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError('读取期间校准备份发生变化')
    finally:
        os.close(fd)
    record = dict(snapshot_utf8=raw.decode('utf-8', 'strict'), sha256=expected_sha256,
                  boot_id=review['expected_boot_id'], cid=review['expected_cid'],
                  serial_device=review['serial_device'])
    validate_restore_record(record, boot_id=review['expected_boot_id'], cid=review['expected_cid'],
                            serial_device=review['serial_device'])
    return record


def runtime_source():
    return ('import hashlib, json, math, re\n' + '\n'.join(inspect.getsource(f) for f in
            (_restore_json_pairs, validate_calibration_snapshot, validate_restore_record)))
