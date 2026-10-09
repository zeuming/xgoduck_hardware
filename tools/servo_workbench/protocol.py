"""Feetech HD1910 protocol. No broadcast or calibration commands.

Verified registers: official xgoduck_runtime_arduino, commit
8cdbbd84710d856581982c9eaf0d5e2970666232, sketch/{scs_bus,duck_config}.h.
EEPROM programming: Feetech FTServo_Arduino HLSCL.cpp unLockEprom:
disable torque BEFORE unlocking. A lock-register readback alone does not
prove that the nonvolatile write took effect.
"""
import time
import serial


class BusError(Exception):
    pass


class NoReply(BusError):
    pass


class ServoFault(BusError):
    def __init__(self, sid, code):
        self.sid, self.code = sid, code
        detail = '（电压异常）' if code == 1 else ''
        super().__init__(f'舵机 ID {sid} 返回故障码 0x{code:02X}{detail}，操作已停止')


def integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name}必须是 {low}–{high} 的整数')
    return value


def encode(sid, instruction, params):
    integer(sid, 0, 253, 'ID')
    body = bytes([sid, len(params) + 2, instruction, *params])
    return b'\xff\xff' + body + bytes([(~sum(body)) & 255])


class Bus:
    def __init__(self, port, baud=1000000, transport=None):
        self.serial = transport if transport is not None else serial.Serial(
            port, baud, timeout=.003, write_timeout=.3)

    def close(self):
        self.serial.close()

    def exchange(self, sid, op, params, expected, timeout=.12, required=True):
        tx = encode(sid, op, params)
        self.serial.reset_input_buffer()
        self.serial.write(tx)
        self.serial.flush()
        raw = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw.extend(self.serial.read(self.serial.in_waiting or 1))
            for i in range(max(0, len(raw) - 5)):
                if raw[i:i + 2] != b'\xff\xff':
                    continue
                size = raw[i + 3] + 4
                frame = bytes(raw[i:i + size])
                if len(frame) != size or frame == tx or frame[2] != sid:
                    continue
                if sum(frame[2:]) & 255 != 255:
                    continue
                if frame[4] and frame[3] in (2, expected + 2):
                    raise ServoFault(sid, frame[4])
                if frame[3] != expected + 2:
                    continue
                return frame[5:-1]
        if required:
            raise NoReply(f'ID {sid} 没有有效响应，请检查连接、ID 和供电')
        return None

    def read(self, sid, addr, size, timeout=.12):
        return self.exchange(sid, 2, [addr, size], size, timeout)

    def write(self, sid, addr, data):
        # An ID change may acknowledge under the new ID; verify via a subsequent read.
        return self.exchange(sid, 3, [addr, *data], 0, required=False)

    def identify(self, sid, timeout=.12):
        actual = self.read(sid, 5, 1, timeout)[0]
        if actual != sid:
            raise BusError('ID 回读不一致，请只连接一只舵机后重试')
        return actual

    def snapshot(self, sid):
        self.identify(sid)
        data = self.read(sid, 56, 6)
        position = int.from_bytes(data[:2], 'little')
        if not 0 <= position <= 4095:
            raise BusError('位置不在 0–4095 范围，本工具仅支持单圈位置模式')
        torque = self.read(sid, 40, 1)[0]
        return dict(id=sid, position=position, torque=torque)

    def goal(self, sid, position):
        integer(position, 0, 4095, '目标位置')
        self.write(sid, 42, [*position.to_bytes(2, 'little'), 0, 0, 0, 0])

    def release(self, sid):
        self.write(sid, 40, [0])
        if self.read(sid, 40, 1) != b'\x00':
            raise BusError('无法确认已释放，请断开舵机电源')

    def change_id(self, old, new, progress=lambda _: None):
        integer(new, 0, 253, '新 ID')
        if old == new:
            raise ValueError('新 ID 与当前 ID 相同')
        self.identify(old)
        try:
            self.identify(new, .04)
        except NoReply:
            pass
        else:
            raise BusError(f'ID {new} 已有舵机响应，未修改')
        # HLS/HTS's official unLockEprom releases torque first (unlike SMS_STS).
        # Keep it released after programming; do not move to a stale goal.
        self.release(old)
        unlocked = False
        try:
            self.write(old, 55, [0])
            unlocked = True
            if self.read(old, 55, 1) != b'\x00':
                raise BusError('参数区解锁未通过验证，未修改 ID')
            self.write(old, 5, [new])
            time.sleep(.05)
            self.identify(new)
            progress(new)  # Track actual new ID even if relocking later fails.
            self.write(new, 55, [1])
            if self.read(new, 55, 1) != b'\x01':
                raise BusError(f'ID 已改为 {new}，但参数区锁定未通过验证')
            return self.snapshot(new)
        except Exception:
            if unlocked:
                # Never roll back an uncertain ID write. Relock any reachable endpoint.
                for sid in (new, old):
                    try:
                        self.identify(sid, .04)
                        self.write(sid, 55, [1])
                    except Exception:
                        pass
            raise
