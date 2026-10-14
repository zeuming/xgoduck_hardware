"""Decode passive ARM failure evidence. This module never opens hardware."""
import struct

ARM_TRACE_SIZE = 134
IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)


def decode_arm_read_trace(payload, *, sequence=None, cookie=None):
    if (len(payload) != ARM_TRACE_SIZE or payload[0] != 4
            or payload[1] not in (2, 4) or payload[2] != 2):
        raise ValueError('ARM 原始读取记录格式无效')
    if (payload[3] not in (4, 9) or payload[4] not in IDS or payload[5] != 40
            or payload[6] not in (0, 1) or payload[8] > 6 or payload[9] not in range(8)
            or payload[22] & ~31 or payload[23] != 1):
        raise ValueError('ARM 原始读取记录字段无效')
    flags = payload[22]
    drained, received, pending = struct.unpack_from('<3H', payload, 24)
    (start, written, first, last, elapsed, errors_before, errors_after,
     enabled, request_sequence, request_cookie) = struct.unpack_from('<10I', payload, 30)
    if (bool(flags & 2) != (drained > 16) or bool(flags & 4) != (received > 48)
            or bool(flags & 16) != bool(received)
            or received and not flags & 8
            or payload[3] == 9 and not request_cookie
            or sequence is not None and request_sequence != sequence
            or cookie is not None and request_cookie != cookie):
        raise ValueError('ARM 原始读取记录关联校验失败')
    if flags & 8 and (written - start) & 0xffffffff > elapsed:
        raise ValueError('ARM 原始读取记录时间无效')
    if received and ((first - start) & 0xffffffff > elapsed
                     or (last - first) & 0xffffffff > elapsed):
        raise ValueError('ARM 原始读取记录接收时间无效')
    return dict(version=2, stage=payload[3], id=payload[4], address=40,
        outcome=payload[9], invalid_reply_seen=bool(flags & 1),
        request_written=bool(flags & 8), drained_count=drained, received_count=received,
        pending_count=pending, started_us=start, written_us=written,
        first_rx_us=first if received else None, last_rx_us=last if received else None,
        elapsed_us=elapsed, uart_errors_before=errors_before, uart_errors_after=errors_after,
        enable_completed_us=enabled if payload[3] == 9 else None,
        enable_to_request_us=(written-enabled) & 0xffffffff if payload[3] == 9 and flags & 8 else None,
        arm_sequence=request_sequence, arm_cookie=request_cookie,
        drained_hex=payload[70:70+min(drained, 16)].hex(),
        received_hex=payload[86:86+min(received, 48)].hex(),
        drained_truncated=bool(flags & 2), received_truncated=bool(flags & 4))


def arm_read_trace_text(record):
    text = (f'原始读取耗时 {record["elapsed_us"]} us；'
            f'读前清理 {record["drained_count"]} 字节，读取 {record["received_count"]} 字节，'
            f'退出时待处理 {record["pending_count"]} 字节；'
            f'UART 错误计数 {record["uart_errors_before"]}→{record["uart_errors_after"]}')
    if record['enable_to_request_us'] is not None:
        text += f'；使能广播发送完成至读请求提交 {record["enable_to_request_us"]} us'
    if record['invalid_reply_seen']:
        text += '；存在被解析器拒绝的字节'
    if not record['received_count'] and not record['pending_count']:
        text += '；退出时未观察到接收字节（不能据此确定供电故障）'
    elif record['pending_count']:
        text += '；退出时仍有未处理字节（不能据此确定到达时间）'
    text += '；RX=' + (record['received_hex'] or '空')
    if record['received_truncated']:
        text += '（前48字节）'
    return text
