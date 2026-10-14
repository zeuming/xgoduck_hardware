"""Private, atomic calibration/provenance storage and live RAM authorization.

The persistent store is written only by commit(), called from a checked user
save. Loading a deployment never initializes persistent files automatically.
Models, identity baselines, firmware and system settings are outside this API.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
import fcntl
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import stat
import threading
import time

import calibration_restore
from feedback_calibration_server import validated_zero_provenance

STORE_DIRECTORY = '/var/lib/xgoduck-workbench'
STATE_NAME = 'calibration-state.json'
MAX_STATE_BYTES = 4 * 1024 * 1024
MAX_CALIBRATION_BYTES = 16384
_WRITE_SCOPE = threading.local()


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('校准存储含重复 JSON 字段')
            result[key] = value
        return result
    def constant(_value):
        raise ValueError('校准存储含非有限数值')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)


def calibration_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()


def canonical_bytes(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def private_read(path, limit):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(descriptor)
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.geteuid()
                or stat.S_IMODE(before.st_mode) != 0o600 or before.st_nlink != 1
                or not 0 < before.st_size <= limit):
            raise ValueError('校准存储须为当前用户的私有普通文件')
        raw = bytearray()
        while len(raw) <= limit:
            part = os.read(descriptor, min(65536, limit + 1 - len(raw)))
            if not part:
                break
            raw.extend(part)
        after = os.fstat(descriptor)
        fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns')
        if len(raw) != before.st_size or any(getattr(before, key) != getattr(after, key) for key in fields):
            raise ValueError('校准存储在读取期间发生变化')
        return bytes(raw)
    finally:
        os.close(descriptor)


def allows_persistent_mutation(path, operation):
    """Bootstrap audit hook: authorize only the current commit's exact paths."""
    directory = getattr(_WRITE_SCOPE, 'directory', None)
    if directory is None:
        return False
    path = Path(os.path.abspath(os.fsdecode(path)))
    if operation == 'mkdir':
        return path == directory
    if path.parent != directory or path.is_symlink() or os.path.realpath(path) != str(path):
        return False
    temporary = re.fullmatch(r'\.calibration-state\.[0-9a-f]{32}\.json', path.name) is not None
    if operation in ('write', 'remove'):
        return temporary
    if operation == 'rename':
        return temporary or path.name == STATE_NAME
    return False


@contextmanager
def _write_scope(directory):
    previous = getattr(_WRITE_SCOPE, 'directory', None)
    _WRITE_SCOPE.directory = directory
    try:
        yield
    finally:
        _WRITE_SCOPE.directory = previous


class NativeCalibrationStore:
    def __init__(self, directory, initial_config, initial_zero_provenance, *, source_sha256):
        self.directory = Path(directory)
        if (not self.directory.is_absolute() or self.directory.is_symlink()
                or os.path.realpath(self.directory) != str(self.directory)):
            raise ValueError('校准存储目录必须是规范绝对路径')
        self.lock = threading.RLock()
        self.initial = copy.deepcopy(calibration_restore.validate_calibration_snapshot(initial_config))
        self.source_sha256 = source_sha256
        if type(source_sha256) is not str or source_sha256 != digest(calibration_bytes(self.initial)):
            raise ValueError('校准初始输入与已验证摘要不符')
        ledger = copy.deepcopy(initial_zero_provenance)
        if type(ledger) is not dict or type(ledger.get('original_zeros')) is not list or len(ledger['original_zeros']) != 15:
            raise ValueError('校准存储缺少原始零点来源')
        self.anchor = copy.deepcopy(self.initial)
        for joint, row in zip(self.anchor['joints'], ledger['original_zeros']):
            if type(row) is not dict or row.get('id') != joint['id']:
                raise ValueError('原始零点关节顺序已改变')
            joint['zero'] = row.get('zero')
        self.initial_provenance = self._validate_pair(self.initial, ledger)
        self.current = self._record(self.initial, self.initial_provenance, revision=0)
        self.persistent = False
        self.load()

    def _validate_pair(self, config, provenance):
        calibration_restore.validate_calibration_snapshot(config)
        normalized = validated_zero_provenance(provenance, self.anchor, config)
        if normalized != provenance:
            raise ValueError('零点来源含尚未提交的保存意图')
        initial = getattr(self, 'initial_provenance', None)
        if initial is not None and (normalized['original_zeros'] != initial['original_zeros'] or
                normalized['entries'][:len(initial['entries'])] != initial['entries']):
            raise ValueError('零点来源不能撤销部署前的授权记录')
        return copy.deepcopy(normalized)

    def _record(self, config, provenance, *, revision):
        return dict(schema_version=1, source_calibration_sha256=self.source_sha256,
                    revision=revision, calibration=copy.deepcopy(config), zero_provenance=copy.deepcopy(provenance),
                    calibration_sha256=digest(calibration_bytes(config)),
                    zero_provenance_sha256=digest(canonical_bytes(provenance)))

    def _validate_record(self, value):
        keys = {'schema_version', 'source_calibration_sha256', 'revision', 'calibration',
                'zero_provenance', 'calibration_sha256', 'zero_provenance_sha256'}
        if (type(value) is not dict or set(value) != keys or type(value['schema_version']) is not int
                or value['schema_version'] != 1 or value['source_calibration_sha256'] != self.source_sha256
                or type(value['revision']) is not int or not 1 <= value['revision'] <= 0x7fffffff):
            raise ValueError('持久校准版本、初始来源或保存序号无效')
        provenance = self._validate_pair(value['calibration'], value['zero_provenance'])
        expected = self._record(value['calibration'], provenance, revision=value['revision'])
        if value != expected:
            raise ValueError('持久校准或零点来源摘要不一致')
        return expected

    def _directory_present(self, *, create=False):
        if not os.path.lexists(self.directory):
            if not create:
                return False
            self.directory.mkdir(mode=0o700)
            parent = os.open(self.directory.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            try:
                os.fsync(parent)
            finally:
                os.close(parent)
        info = self.directory.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid()
                or stat.S_IMODE(info.st_mode) != 0o700 or self.directory.is_symlink()
                or os.path.realpath(self.directory) != str(self.directory)):
            raise ValueError('校准存储目录须由当前用户私有保存')
        return True

    def _disk_record(self):
        if not self._directory_present() or not os.path.lexists(self.directory / STATE_NAME):
            return None
        return self._validate_record(strict_json(private_read(self.directory / STATE_NAME, MAX_STATE_BYTES)))

    def load(self):
        with self.lock:
            record = self._disk_record()
            if record is not None:
                self.current, self.persistent = record, True
            elif self.persistent:
                raise ValueError('已保存的持久校准文件消失')
            result = copy.deepcopy(self.current)
            result['persistent'] = self.persistent
            return result

    def commit(self, config, zero_provenance):
        """Atomically commit a checked explicit save; return a durability warning."""
        provenance = self._validate_pair(config, zero_provenance)
        with self.lock, _write_scope(self.directory):
            self._directory_present(create=True)
            dirfd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            temporary = '.calibration-state.' + secrets.token_hex(16) + '.json'
            promoted = False
            try:
                fcntl.flock(dirfd, fcntl.LOCK_EX)
                current = self._disk_record()
                if (current is None) != (not self.persistent) or current is not None and current != self.current:
                    raise ValueError('校准存储被另一会话改变；本次保存未执行')
                value = self._record(config, provenance, revision=self.current['revision'] + 1)
                self._validate_record(value)
                # The reviewed calibration digest preserves its original JSON
                # field order. Sorting the enclosing record would recursively
                # reorder that config and change its byte digest on reload.
                raw = (json.dumps(value, ensure_ascii=False, separators=(',', ':'),
                                  allow_nan=False) + '\n').encode()
                if len(raw) > MAX_STATE_BYTES:
                    raise ValueError('持久校准和零点来源超过存储上限')
                descriptor = os.open(str(self.directory / temporary),
                                     os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                with os.fdopen(descriptor, 'wb') as stream:
                    if stream.write(raw) != len(raw):
                        raise OSError('持久校准写入不完整')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(str(self.directory / temporary), str(self.directory / STATE_NAME))
                promoted = True
                self.current, self.persistent = value, True
                try:
                    os.fsync(dirfd)
                except OSError:
                    return '校准已保存，但目录同步未完成；请保留本次导出的校准备份。'
                return None
            finally:
                if not promoted:
                    try:
                        (self.directory / temporary).unlink()
                    except FileNotFoundError:
                        pass
                fcntl.flock(dirfd, fcntl.LOCK_UN)
                os.close(dirfd)


def authority_mac(key, sha256, revision):
    return hmac.new(key, canonical_bytes(dict(sha256=sha256, revision=revision)), hashlib.sha256).hexdigest()


class RAMCalibrationAuthority:
    """Authorize exact checked calibration replacements across guarded children."""
    def __init__(self, root, initial_sha256):
        self.root = Path(root)
        if not self.root.is_absolute() or self.root.is_symlink() or self.root.resolve() != self.root:
            raise ValueError('RAM 校准目录无效')
        self.path = self.root / 'policy/calibration.json'
        self.authority_path = self.root / 'service/calibration-authority.json'
        self.lock_path = self.root / 'service/calibration-authority.lock'
        self.key = secrets.token_bytes(64)
        self.revision, self.sha256 = 0, initial_sha256
        self.lock = threading.RLock()
        raw = private_read(self.path, MAX_CALIBRATION_BYTES)
        calibration_restore.validate_calibration_snapshot(strict_json(raw))
        if digest(raw) != initial_sha256:
            raise ValueError('初始 RAM 校准摘要无效')
        descriptor = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self.lock_node = os.fstat(descriptor)
        os.close(descriptor)
        self._atomic(self.authority_path, canonical_bytes(self._record()) + b'\n')

    def _record(self):
        return dict(sha256=self.sha256, revision=self.revision,
                    mac=authority_mac(self.key, self.sha256, self.revision))

    @staticmethod
    def _atomic(path, raw):
        temporary = path.parent / ('.calibration-authority.' + secrets.token_hex(16) + '.json')
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        try:
            with os.fdopen(descriptor, 'wb') as stream:
                if stream.write(raw) != len(raw):
                    raise OSError('RAM 校准写入不完整')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    @contextmanager
    def _file_lock(self, exclusive):
        descriptor = os.open(self.lock_path, os.O_RDWR | os.O_NOFOLLOW)
        try:
            node = os.fstat(descriptor)
            if (not stat.S_ISREG(node.st_mode) or node.st_nlink != 1 or node.st_uid != os.geteuid()
                    or stat.S_IMODE(node.st_mode) != 0o600 or
                    (node.st_dev, node.st_ino) != (self.lock_node.st_dev, self.lock_node.st_ino)):
                raise ValueError('RAM 校准锁身份改变')
            deadline = time.monotonic() + .2
            operation = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
            while True:
                try:
                    fcntl.flock(descriptor, operation | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise ValueError('RAM 校准锁超时')
                    time.sleep(.002)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def _verify_locked(self):
        record = strict_json(private_read(self.authority_path, 1024))
        raw = private_read(self.path, MAX_CALIBRATION_BYTES)
        if record != self._record() or digest(raw) != self.sha256:
            raise ValueError('RAM 校准未经已验证保存流程授权')
        calibration_restore.validate_calibration_snapshot(strict_json(raw))
        return self.sha256

    def verify(self):
        with self.lock, self._file_lock(False):
            return self._verify_locked()

    def publish(self, raw):
        if type(raw) is not bytes or not 0 < len(raw) <= MAX_CALIBRATION_BYTES:
            raise ValueError('RAM 校准写入大小无效')
        calibration_restore.validate_calibration_snapshot(strict_json(raw))
        with self.lock, self._file_lock(True):
            self._verify_locked()
            previous_raw = private_read(self.path, MAX_CALIBRATION_BYTES)
            previous_sha, previous_revision = self.sha256, self.revision
            try:
                self._atomic(self.path, raw)
                self.sha256, self.revision = digest(raw), self.revision + 1
                self._atomic(self.authority_path, canonical_bytes(self._record()) + b'\n')
            except BaseException:
                self.sha256, self.revision = previous_sha, previous_revision
                self._atomic(self.path, previous_raw)
                self._atomic(self.authority_path, canonical_bytes(self._record()) + b'\n')
                raise
            return self.sha256

    def child_spec(self):
        with self.lock:
            return dict(calibration_path=str(self.path), calibration_sha256=self.sha256,
                        calibration_authority_path=str(self.authority_path), calibration_authority_key=self.key.hex(),
                        calibration_lock_path=str(self.lock_path), calibration_revision=self.revision)
