"""Executed over SSH on X5. The ONLY transmitted MCU command is empty STATE (2).

No motion runtime is imported. No HELLO, STOP, CONFIG, ARM, TARGET, EEPROM write,
reset or service change. The exclusive UART descriptor closes when SSH closes.
"""
import binascii
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import stat
import struct
import termios
import time

IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
PACKET = struct.Struct('<IIHHHBB15H15h6fIBB')
PORT = '/dev/ttyUSB0'


class ReadPort:
    """Same raw UART setup as the installed feedback reader; no modem setters."""
    def __init__(self):
        self.fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            before = fcntl.ioctl(self.fd, termios.TIOCMGET, struct.pack('I', 0))
            settings = termios.tcgetattr(self.fd)
            settings[0] = settings[1] = settings[3] = 0
            settings[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL | getattr(termios, 'CRTSCTS', 0))
            settings[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[4] = settings[5] = termios.B921600
            settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
            actual = termios.tcgetattr(self.fd)
            blocked = termios.PARENB | termios.CSTOPB | termios.HUPCL | getattr(termios, 'CRTSCTS', 0)
            if (actual[0] or actual[1] or actual[3] or actual[2] & blocked or
                    actual[2] & termios.CSIZE != termios.CS8 or
                    not actual[2] & termios.CREAD or not actual[2] & termios.CLOCAL or
                    actual[4] != termios.B921600 or actual[5] != termios.B921600 or
                    actual[6][termios.VMIN] != 0 or actual[6][termios.VTIME] != 0):
                raise RuntimeError('UART 配置回读不一致')
            after = fcntl.ioctl(self.fd, termios.TIOCMGET, struct.pack('I', 0))
            mask = termios.TIOCM_DTR | termios.TIOCM_RTS
            if (struct.unpack('I', before)[0] & mask) != (struct.unpack('I', after)[0] & mask):
                raise RuntimeError('串口控制线在配置期间发生变化')
        except BaseException:
            os.close(self.fd); raise

    def read(self, size):
        return os.read(self.fd, size) if select.select([self.fd], [], [], .005)[0] else b''

    def write(self, data):
        # Keep the wire write itself restricted, even if callers change later.
        if len(data) != 12 or data[:6] != b'XD\x01\x02\x00\x00' or binascii.crc_hqx(data[:-2],0xffff) != struct.unpack('<H',data[-2:])[0]:
            raise RuntimeError('只允许发送空载荷 STATE 请求')
        return os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


def state_request(sequence):
    header = struct.pack('<2sBBHI', b'XD', 1, 2, 0, sequence)
    return header + struct.pack('<H', binascii.crc_hqx(header, 0xffff))


def read_state(stream, sequence):
    request = state_request(sequence)
    if stream.write(request) != len(request):
        raise RuntimeError('只读 STATE 请求未完整发送')
    deadline = time.monotonic() + .3
    buffer = bytearray()
    while time.monotonic() < deadline:
        buffer.extend(stream.read(512))
        while len(buffer) >= 12:
            if buffer[:2] != b'XD':
                del buffer[0]; continue
            _, version, kind, length, seq = struct.unpack_from('<2sBBHI', buffer)
            if version != 1 or length > 256:
                del buffer[0]; continue
            size = length + 12
            if len(buffer) < size:
                break
            frame = bytes(buffer[:size]); del buffer[:size]
            if binascii.crc_hqx(frame[:-2], 0xffff) != struct.unpack_from('<H', frame, size-2)[0]:
                continue
            if seq != sequence:
                continue
            if kind != 0x82 or length != PACKET.size:
                raise RuntimeError('STATE 回复与已验证的运动固件协议不符')
            values = PACKET.unpack(frame[10:-2])
            sample, imu, online, fault, torque, armed, imu_ok = values[:7]
            pos, vel = list(values[7:22]), list(values[22:37])
            acc, gyro = list(values[37:40]), list(values[40:43])
            if ((online | fault | torque) & ~0x7fff or armed not in (0,1) or imu_ok not in (0,1)
                    or not all(math.isfinite(x) for x in acc+gyro)
                    or any(online & (1<<i) and not 0 <= p <= 4095 for i,p in enumerate(pos))):
                raise RuntimeError('STATE 字段无效')
            return dict(sample_us=sample, imu_us=imu, online_mask=online, fault_mask=fault,
                        torque_mask=torque, armed=bool(armed), imu_ok=bool(imu_ok),
                        positions=pos, velocities=vel, acc=acc, gyro=gyro, firmware_error=values[44])
    raise RuntimeError('读取 STATE 超时；请检查 ESP32 供电与 USB 连接')


CALIBRATION_PATH = Path('/var/lib/xgoduck-workbench/calibration-state.json')


def calibration_stamp():
    value = CALIBRATION_PATH.stat()
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)


def require_same_calibration(expected):
    if calibration_stamp() != expected:
        raise RuntimeError('保存校准已更新，实时反馈已结束；请重新连接以载入新校准')


def load_calibration():
    path = CALIBRATION_PATH
    record = json.loads(path.read_text())
    cfg = record['calibration']
    digest = hashlib.sha256((json.dumps(cfg, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()).hexdigest()
    if digest != record['calibration_sha256'] or [j['id'] for j in cfg['joints']] != IDS:
        raise RuntimeError('保存校准的摘要或关节顺序不匹配')
    for j in cfg['joints']:
        if type(j['zero']) is not int or not 0 <= j['zero'] <= 4095 or j['sign'] not in (-1,1):
            raise RuntimeError('保存关节零点或方向无效')
    return cfg, digest, record['revision']


def emit(row):
    print(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':')), flush=True)


def verify_port():
    node = os.stat(PORT, follow_symlinks=False)
    if not stat.S_ISCHR(node.st_mode):
        raise RuntimeError('反馈串口不是字符设备')
    device = Path('/sys/class/tty/ttyUSB0/device').resolve()
    usb = next((p for p in device.parents if (p/'idVendor').exists()), None)
    if usb is None or (usb/'idVendor').read_text().strip() != '1a86' or (usb/'idProduct').read_text().strip() != '7523':
        raise RuntimeError('串口不是预期的 CH340 USB 设备')
    for proc in Path('/proc').glob('[0-9]*/fd'):
        if int(proc.parent.name) == os.getpid():
            continue
        try:
            for fd in proc.iterdir():
                try:
                    info = fd.stat()
                    if stat.S_ISCHR(info.st_mode) and info.st_rdev == node.st_rdev:
                        raise RuntimeError('串口正由其他程序使用；请先在运控页主动断开连接')
                except FileNotFoundError:
                    pass
        except (FileNotFoundError, PermissionError):
            pass
    return node


def main():
    stream = None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit()))
    try:
        saved_stamp = calibration_stamp()
        cfg, digest, revision = load_calibration()
        require_same_calibration(saved_stamp)
        node = verify_port()
        stream = ReadPort()
        require_same_calibration(saved_stamp)
        emit(dict(type='calibration', calibration=cfg, sha256=digest, revision=revision, protocol='STATE only'))
        last_sample = last_imu = None
        last_sample_at = last_imu_at = time.monotonic()
        for sequence in range(1, 0xffffffff):
            began = time.monotonic()
            current = os.stat(PORT, follow_symlinks=False)
            if (current.st_dev,current.st_ino,current.st_rdev) != (node.st_dev,node.st_ino,node.st_rdev):
                raise RuntimeError('USB 设备已变化，请重新连接反馈')
            value = read_state(stream, sequence)
            now = time.monotonic()
            if value['sample_us'] != last_sample:
                last_sample, last_sample_at = value['sample_us'], now
            if value['imu_us'] != last_imu:
                last_imu, last_imu_at = value['imu_us'], now
            value.update(type='state', sequence=sequence, remote_time=now,
                         sample_unchanged_ms=(now-last_sample_at)*1000,
                         imu_unchanged_ms=(now-last_imu_at)*1000,
                         uart_rtt_ms=(now-began)*1000)
            require_same_calibration(saved_stamp)
            emit(value)
            time.sleep(max(0, 1/30 - (time.monotonic()-began)))
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    except Exception as exc:
        emit(dict(type='error', message=str(exc)))
    finally:
        if stream is not None:
            stream.close()


if __name__ == '__main__':
    main()
