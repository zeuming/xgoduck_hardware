"""Persistent X5 COM -> ESP32 -> URT-2 transport for the existing workbench.

The application owns an EspSessionManager for its lifetime, calls get(port) when
creating a BridgeBus, and calls manager.close() on shutdown. BridgeBus.close()
does not close COM. A lost transaction is never retried, especially a write.
No port is opened by constructing a session, manager, or bus.
"""
import json
import math
import secrets
import threading
import time

import serial

from protocol import Bus, BusError, NoReply, encode, integer


PREFIX = 'XGODUCK_LINKCHECK'
HOST_BAUD = 115200
SERVO_BAUD = 1000000
MAX_LINE_BYTES = 8192
MAX_TRANSACTION_BYTES = 32768
MAX_RX_BYTES = 512
READS = {(5, 1), (40, 1), (42, 2), (55, 1), (56, 6)}


class BridgeError(BusError):
    """Host/ESP transport or handshake failure, not an absent servo."""


class WriteUncertain(BridgeError):
    """The write may have reached the servo; never automatically resend it."""


def validate_packet(packet):
    """Allow only the exact single-ID operations used by protocol.Bus."""
    if not isinstance(packet, (bytes, bytearray)):
        raise ValueError('舵机数据包必须是字节串')
    packet = bytes(packet)
    if not 6 <= len(packet) <= 13 or packet[:2] != b'\xff\xff':
        raise ValueError('舵机数据包格式不合法')
    if packet[2] > 253 or packet[3] + 4 != len(packet) or sum(packet[2:]) & 255 != 255:
        raise ValueError('舵机数据包 ID、长度或校验和不合法')
    op, params = packet[4], packet[5:-1]
    if op == 2 and len(params) == 2 and tuple(params) in READS:
        return packet
    if op == 3 and params:
        address, data = params[0], params[1:]
        if address == 5 and len(data) == 1 and data[0] <= 253:
            return packet
        if address in (40, 55) and len(data) == 1 and data[0] in (0, 1):
            return packet
        if address == 42 and len(data) == 6 and int.from_bytes(data[:2], 'little') <= 4095 and data[2:] == b'\x00' * 4:
            return packet
    raise ValueError('桥接仅允许工作台使用的单 ID 读取、回中、扭矩与 ID 设置指令')


class EspSession:
    def __init__(self, port, *, serial_factory=serial.Serial, startup_delay=2.0, timeout=.6):
        if not isinstance(port, str) or not port:
            raise ValueError('需要指定 ESP32 COM 串口')
        if not math.isfinite(timeout) or not .4 <= timeout <= 10:
            raise ValueError('ESP 响应超时必须在 0.4–10 秒之间')
        if not math.isfinite(startup_delay) or not 0 <= startup_delay <= 10:
            raise ValueError('启动等待时间不合法')
        self.port = port
        self.timeout = timeout
        self.startup_delay = startup_delay
        self.serial_factory = serial_factory
        self._serial = None
        self._lock = threading.Lock()
        self._pending = bytearray()
        self._session = secrets.token_hex(8)
        self._seq = 0
        self.info = None

    def _disconnect(self):
        connection, self._serial = self._serial, None
        self._pending.clear()
        self.info = None
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass

    def close(self):
        with self._lock:
            self._disconnect()

    def _send(self, command):
        data = (command + '\n').encode('ascii')
        if self._serial.write(data) != len(data):
            raise BridgeError('ESP 串口未完整发送命令')
        # write() uses write_timeout; avoid an unbounded OS tcdrain()/flush().

    def _receive(self, matches, *, handshake=False):
        deadline = time.monotonic() + self.timeout
        received = len(self._pending)
        while time.monotonic() < deadline:
            while b'\n' in self._pending:
                line, _, rest = self._pending.partition(b'\n')
                self._pending = bytearray(rest)
                if len(line) > MAX_LINE_BYTES:
                    raise BridgeError('ESP 回复行超出长度限制')
                try:
                    result = json.loads(line.decode('utf-8'))
                except (UnicodeDecodeError, json.JSONDecodeError, RecursionError):
                    continue  # Boot messages and incomplete/unrelated console lines.
                if not isinstance(result, dict) or result.get('prefix') != PREFIX:
                    continue
                if not handshake and result.get('command') == 'INFO' and result.get('event') == 'boot':
                    raise BridgeError('ESP 在事务中重启，连接状态已失效')
                if matches(result):
                    return result
            if len(self._pending) > MAX_LINE_BYTES:
                raise BridgeError('ESP 回复行超出长度限制')
            raw = self._serial.read(min(self._serial.in_waiting or 1, 1024))
            received += len(raw)
            if received > MAX_TRANSACTION_BYTES:
                raise BridgeError('ESP 回复数据超出长度限制')
            self._pending.extend(raw)
        raise BridgeError('ESP 回复超时；连接已关闭，下次读取会重新握手')

    def _connect(self):
        if self._serial is not None and self._serial.is_open:
            return
        self._disconnect()
        connection = self.serial_factory(port=None, baudrate=HOST_BAUD, timeout=.02, write_timeout=.3)
        self._serial = connection
        connection.dtr = False
        connection.rts = False
        connection.port = self.port
        connection.open()
        time.sleep(self.startup_delay)
        connection.reset_input_buffer()
        self._send('INFO')
        info = self._receive(lambda r: r.get('command') == 'INFO' and r.get('event') == 'reply', handshake=True)
        capabilities = info.get('capabilities')
        if not isinstance(capabilities, dict) or capabilities.get('workbench_xfer') is not True or type(info.get('protocol')) is not int or info['protocol'] != 1:
            raise BridgeError('ESP 固件不支持工作台桥接；需要具有 workbench_xfer 的 0.2.0 或兼容固件')
        self.info = info

    def transfer(self, packet):
        """Send once and return raw servo RX bytes. No-ACK is b'', not link loss."""
        packet = validate_packet(packet)
        writing = packet[4] == 3
        attempted = False
        with self._lock:
            try:
                self._connect()  # Only INFO is sent during a connection/reconnection.
                if self._seq >= 0xffffffff:
                    raise BridgeError('ESP 会话序号已用尽，需要重新建立工作台会话')
                self._seq += 1
                sequence = self._seq
                session = self._session
                command = f'XFER {session} {sequence} {packet.hex()}'
                attempted = True
                self._send(command)
                result = self._receive(lambda r: r.get('command') == 'XFER' and
                                       r.get('session') == session and
                                       type(r.get('seq')) is int and r['seq'] == sequence)
                if result.get('ok') is not True:
                    reason = str(result.get('error', 'unknown'))[:200]
                    raise BridgeError(f'ESP 未完成事务：{reason}')
                value = result.get('rx_hex')
                if not isinstance(value, str) or len(value) > MAX_RX_BYTES * 2 or len(value) % 2 or any(c not in '0123456789abcdefABCDEF' for c in value):
                    raise BridgeError('ESP 返回的舵机数据格式不合法')
                return bytes.fromhex(value)
            except (OSError, serial.SerialException, BridgeError) as exc:
                self._disconnect()
                if writing and attempted:
                    raise WriteUncertain(f'{exc}；写入结果不确定，未自动重发，请先读取确认') from exc
                if isinstance(exc, BridgeError):
                    raise
                raise BridgeError(f'ESP 串口连接失败：{exc}') from exc


class EspSessionManager:
    """One lazy physical session per selected port for the application's lifetime."""
    def __init__(self, **session_options):
        self._sessions = {}
        self._lock = threading.Lock()
        self._options = session_options

    def get(self, port):
        with self._lock:
            if port not in self._sessions:
                self._sessions[port] = EspSession(port, **self._options)
            return self._sessions[port]

    def close(self):
        with self._lock:
            sessions, self._sessions = self._sessions, {}
            for session in sessions.values():
                session.close()


class _EndOfReply(Exception):
    pass


class _MemoryTransport:
    """Feed returned RX through the unchanged protocol.Bus frame validator."""
    def __init__(self, response):
        self.response = response
        self.buffer = bytearray()

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, _):
        self.buffer.extend(self.response)

    def flush(self):
        pass

    def read(self, size):
        if not self.buffer:
            raise _EndOfReply()
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        pass


class BridgeBus(Bus):
    def __init__(self, session, baud=SERVO_BAUD):
        if type(baud) is not int or baud != SERVO_BAUD:
            raise ValueError('ESP 桥接仅支持舵机总线 1,000,000 bps；COM 固定 115,200 bps')
        self.session = session
        self._closed = False

    def close(self):
        self._closed = True  # The application still owns the physical session.

    def exchange(self, sid, op, params, expected, timeout=.12, required=True):
        if self._closed:
            raise BridgeError('此工作台操作已经结束，请创建新的 BridgeBus')
        integer(expected, 0, 6, '响应长度')
        packet = validate_packet(encode(sid, op, params))
        if expected != (params[1] if op == 2 else 0):
            raise ValueError('响应长度与舵机指令不一致')
        response = self.session.transfer(packet)
        # timeout is a legacy direct-servo setting, never the COM/JSON deadline.
        parser = Bus('', transport=_MemoryTransport(response))
        try:
            return parser.exchange(sid, op, params, expected, timeout=.12, required=required)
        except _EndOfReply:
            if required:
                raise NoReply(f'ID {sid} 没有有效响应，请检查连接、ID 和供电') from None
            return None
