"""Pure decoder for bounded evidence attached to a failed CENTER ACK.

The identical decoder runs on the RAM transport and on the host. Evidence
never authorizes another request or changes any maintenance safety hold.
"""
import struct

WIRE_DECODER = r'''
def decode_center_failure(raw, expected_id):
    if (type(raw) is not bytes or len(raw) != 101 or raw[0] not in (12, 16)
            or raw[1] not in (18, 19, 21) or raw[2] != 1
            or type(expected_id) is not int or not 0 <= expected_id <= 253
            or raw[3] != expected_id):
        raise ValueError('Invalid CENTER diagnostic header')
    stage, sid, address, size = raw[1], raw[3], raw[4], raw[5]
    if raw[0] == 16 and stage == 21:
        raise ValueError('Current-position check cannot reach explicit torque enable verification')
    expected = struct.unpack_from('<H', raw, 6)[0]
    found, invalid, outcome, error, actual_size = raw[8:13]
    register = (42, 2) if stage == 18 else (40, 1)
    if ((address, size) != register or expected > 4095
            or stage == 19 and expected != 0 or stage == 21 and expected != 1
            or found not in (0, 1) or invalid not in (0, 1)
            or outcome not in range(1, 7) or actual_size > 6):
        raise ValueError('Invalid CENTER diagnostic register or flags')
    if found:
        if ((outcome == 1 and (error or actual_size != size))
                or (outcome == 2 and (not error or actual_size))
                or outcome not in (1, 2)):
            raise ValueError('Inconsistent CENTER diagnostic reply')
    elif outcome not in (3, 4, 5, 6) or error or actual_size:
        raise ValueError('Inconsistent CENTER diagnostic missing reply')
    data = raw[13:19]
    if any(data[actual_size:]):
        raise ValueError('Invalid CENTER diagnostic unused reply bytes')
    elapsed, before, after = struct.unpack_from('<III', raw, 19)
    drained, received = struct.unpack_from('<HH', raw, 31)
    drain_size, rx_size = raw[35:37]
    if (drain_size != min(drained, 16) or rx_size != min(received, 48)
            or any(raw[37 + drain_size:53]) or any(raw[53 + rx_size:101])):
        raise ValueError('Invalid CENTER diagnostic byte counts')
    actual = (int.from_bytes(data[:actual_size], 'little') if found and not error else None)
    if found and not error and not invalid and actual == expected:
        raise ValueError('Successful read cannot be CENTER failure evidence')
    return dict(version=1, request_type=raw[0], stage=stage, id=sid, address=address, requested_size=size,
        expected=expected, found=bool(found), invalid_reply_seen=bool(invalid),
        outcome=outcome, device_error=error, actual_size=actual_size, actual=actual,
        elapsed_us=elapsed, uart_errors_before=before, uart_errors_after=after,
        drained_count=drained, received_count=received,
        drained_hex=raw[37:37 + drain_size].hex(), received_hex=raw[53:53 + rx_size].hex(),
        drained_truncated=drained > 16, received_truncated=received > 48)
'''
exec(WIRE_DECODER)


def describe_center_failure(value):
    """Explain observed evidence without claiming an electrical root cause."""
    if value['device_error']:
        return f'舵机应答错误码 0x{value["device_error"]:02X}'
    if value['outcome'] == 5:
        return '此次读回出现 UART 硬件错误'
    if value['invalid_reply_seen']:
        detail = '此次读回收到损坏、错编号或不完整报文'
        if value['found']:
            detail += f'；有效应答值 {value["actual"]}，要求 {value["expected"]}'
        return detail
    if value['found']:
        return f'此次读回值 {value["actual"]}，要求 {value["expected"]}'
    if value['outcome'] == 3:
        return '发送查询前总线未进入静默状态'
    if value['outcome'] == 4:
        return '此次查询未完整发送到 UART'
    return f'此次查询超时，收到 {value["received_count"]} 字节，未收到有效应答'
