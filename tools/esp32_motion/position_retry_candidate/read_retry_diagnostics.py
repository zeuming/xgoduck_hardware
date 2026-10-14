"""Read-only wire helpers; import never opens a device or starts motion."""
import math
import struct
import time

HEADER=struct.Struct('<2sBBHI')
STATE=struct.Struct('<IIHHHBB15H15h6fIBB')
IDS=(10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
REPLIES={2:(0x82,106),7:(0x87,247),26:(0x9a,28)}


def crc16(raw):
    value=0xffff
    for byte in raw:
        value^=byte<<8
        for _ in range(8):value=((value<<1)^0x1021 if value&0x8000 else value<<1)&0xffff
    return value


def request(kind,sequence):
    if type(kind) is not int or kind not in REPLIES or type(sequence) is not int or not 1<=sequence<=0xffffffff:
        raise ValueError('Only empty STATE, cached fault, and cached retry reads are allowed')
    raw=HEADER.pack(b'XD',1,kind,0,sequence)
    return raw+struct.pack('<H',crc16(raw))


def exchange(stream,kind,sequence):
    frame=request(kind,sequence)
    if stream.in_waiting:raise ValueError('Unexpected UART input before query; not discarded')
    began=time.monotonic();deadline=began+.2
    if stream.write(frame)!=len(frame):raise ValueError('Short host query write; not retried')
    def read_exact(size):
        raw=bytearray()
        while len(raw)<size:
            if time.monotonic()>deadline:raise TimeoutError('Read-only query deadline expired; not retried')
            raw.extend(stream.read(size-len(raw)))
        return bytes(raw)
    head=read_exact(HEADER.size)
    expected_kind,size=REPLIES[kind]
    if HEADER.unpack(head)!=(b'XD',1,expected_kind,size,sequence):raise ValueError('Read-only reply header differs')
    tail=read_exact(size+2)
    if struct.unpack('<H',tail[-2:])[0]!=crc16(head+tail[:-2]):raise ValueError('Read-only reply CRC differs')
    if stream.in_waiting:raise ValueError('Unexpected trailing UART bytes')
    return tail[:-2],(time.monotonic()-began)*1000


def state(raw):
    if len(raw)!=STATE.size:raise ValueError('Invalid STATE length')
    v=STATE.unpack(raw)
    if any(x not in (0,1) for x in (v[5],v[6],v[-1])) or any(x&~32767 for x in v[2:5]):raise ValueError('Invalid STATE flags')
    if not all(math.isfinite(x) for x in v[37:43]):raise ValueError('Non-finite IMU')
    return dict(sample_us=v[0],imu_us=v[1],online_mask=v[2],fault_mask=v[3],torque_mask=v[4],armed=bool(v[5]),imu_ok=bool(v[6]),positions=list(v[7:22]),arm_cookie=v[-3],last_error=v[-2],configured=bool(v[-1]))


def retry_details(raw):
    if len(raw)!=28:raise ValueError('Invalid retry diagnostic length')
    version,joint,first,last,attempts,recovered,failed,at,first_us,retry_us=struct.unpack('<4B6I',raw)
    if version!=1 or joint not in (*IDS,255) or first>7 or last>7:raise ValueError('Invalid retry diagnostic fields')
    return dict(version=version,last_id=joint,first_outcome=first,retry_outcome=last,attempts=attempts,recovered=recovered,failed=failed,at_us=at,first_elapsed_us=first_us,retry_elapsed_us=retry_us)
