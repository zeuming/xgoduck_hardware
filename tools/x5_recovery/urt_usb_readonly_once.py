#!/usr/bin/env python3
"""One explicit native URT2 read attempt; default only prints preparation.

No X5, SSH, HTTP, motion commands or calibration integration. Physical flags
record the caller's confirmation, not an electrical safety certification.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
READS = ((5, 1), (56, 6), (40, 1))
BAUD = 1000000
READ_SECONDS = .12
TOTAL_SECONDS = 8.
OUTPUT = Path(__file__).resolve().parents[2] / 'output'


class ReadError(Exception):
    pass


class DeviceFault(ReadError):
    def __init__(self, code):
        self.code = code
        super().__init__('Valid servo error reply: 0x%02x' % code)


def read_frame(sid, address, size):
    if (type(sid) is not int or sid not in IDS or type(address) is not int
            or type(size) is not int or (address, size) not in READS):
        raise ReadError('Only fixed-ID READ registers 5/56/40 are allowed')
    body = bytes((sid, 4, 2, address, size))
    return b'\xff\xff' + body + bytes(((~sum(body)) & 255,))


def selected_port(port, ports):
    if type(port) is not str or not port.startswith('/dev/'):
        raise ReadError('An explicit listed /dev/ port is required')
    same = [p for p in ports if p.device == port]
    urt = [p for p in ports if p.vid == 0x1a86 and p.pid == 0x55d3]
    if len(same) != 1 or len(urt) != 1 or same[0] is not urt[0]:
        raise ReadError('Selected port must uniquely identify CH343 URT2 1a86:55d3')
    p = same[0]
    return dict(device=p.device, vid=p.vid, pid=p.pid,
                serial_number=getattr(p, 'serial_number', None),
                location=getattr(p, 'location', None))


class ReadOnlyTransport:
    """Validate every outbound byte packet; no arbitrary UART write API."""
    def __init__(self, raw):
        self.raw = raw
        self.requests = []

    def send_read(self, sid, address, size):
        packet = read_frame(sid, address, size)
        self.requests.append(dict(id=sid, instruction=2, register=address, size=size))
        if self.raw.write(packet) != len(packet):
            raise ReadError('Incomplete READ UART write')
        return packet


def _claim_exclusive(raw):
    import fcntl
    import termios
    fcntl.ioctl(raw.fileno(), termios.TIOCEXCL)


class LocalOwnershipGuard:
    def __init__(self, port, *, runner=subprocess.run, claim_exclusive=_claim_exclusive):
        self.paths = [port]
        if port.startswith('/dev/cu.'):
            self.paths.append('/dev/tty.' + port[len('/dev/cu.'):])
        elif port.startswith('/dev/tty.'):
            self.paths.append('/dev/cu.' + port[len('/dev/tty.'):])
        self.runner, self.claim_exclusive = runner, claim_exclusive
        self.proof = {}

    def owners(self):
        result = self.runner(['/usr/sbin/lsof', '-nP', '-Fpf', '--', *self.paths],
                             capture_output=True, text=True, timeout=2.)
        if result.returncode == 1 and not result.stdout and not result.stderr:
            return []
        if result.returncode != 0 or result.stderr or not result.stdout:
            raise ReadError('Cannot verify UART ownership with lsof')
        owners, pid = [], None
        for line in result.stdout.splitlines():
            if re.fullmatch(r'p[1-9][0-9]*', line):
                pid = int(line[1:])
            elif re.fullmatch(r'f[0-9]+', line) and pid is not None:
                owners.append(dict(pid=pid, fd=int(line[1:])))
            else:
                raise ReadError('Unrecognized lsof ownership record')
        if not owners:
            raise ReadError('Incomplete lsof ownership record')
        return owners

    def before_open(self):
        owners = self.owners()
        self.proof['before_open_owners'] = owners
        if owners:
            raise ReadError('UART already has an open owner; no process was killed')

    def after_open(self, raw):
        self.claim_exclusive(raw)
        self.proof['tioc_exclusive_claimed'] = True
        owners = self.owners()
        self.proof['after_open_owners'] = owners
        if not owners or any(o != dict(pid=os.getpid(), fd=raw.fileno()) for o in owners):
            raise ReadError('UART ownership changed or cannot be verified after open')


def read_register(transport, sid, address, size, total_deadline, clock, sleep):
    deadline = min(total_deadline, clock() + READ_SECONDS)
    if clock() >= deadline:
        raise TimeoutError('Whole diagnostic deadline expired')
    if transport.raw.in_waiting:
        raise ReadError('Unexpected buffered input before READ; not flushed')
    packet = transport.send_read(sid, address, size)
    received = bytearray()
    while clock() < deadline:
        chunk = transport.raw.read(64)
        if clock() >= deadline:
            raise TimeoutError('READ deadline expired while receiving')
        received.extend(chunk)
        if len(received) > 128:
            raise ReadError('UART reply exceeds bounded buffer')
        while len(received) >= 4:
            if received[:2] != b'\xff\xff':
                raise ReadError('Invalid native reply header')
            length = received[3]
            if length not in (2, 4, size + 2):
                raise ReadError('Invalid native reply length')
            frame_size = length + 4
            if len(received) < frame_size:
                break
            frame = bytes(received[:frame_size])
            del received[:frame_size]
            if frame[2] != sid or sum(frame[2:]) & 255 != 255:
                raise ReadError('Native reply ID/checksum mismatch')
            if frame == packet:
                continue  # An exact READ echo is never a status reply.
            if length != size + 2 and not (length == 2 and frame[4] != 0):
                raise ReadError('Native reply payload length mismatch')
            if received:
                raise ReadError('Unexpected trailing native reply bytes')
            if frame[4]:
                raise DeviceFault(frame[4])
            return frame[5:-1]
        if not chunk:
            sleep(min(.001, max(0., deadline - clock())))
    raise TimeoutError('No valid reply before READ deadline')


def initial_report(port):
    return dict(port=port, baud=BAUD, read_seconds=READ_SECONDS,
        read_stage_deadline_seconds=TOTAL_SECONDS, serial_open_attempts=0, serial_opened=False,
        serial_closed=None, query_completed=False, all_15_healthy_torque_off=False,
        automatic_retry=False, servo_motion_authorized=False,
        electrical_safety_verified=False, calibration_written=False,
        unlocks_calibration_page=False, x5_accessed=False,
        open_and_close_have_hard_deadline=False,
        serial_open_may_change_modem_controls=True,
        results=[dict(id=sid, status='unknown', reads=[]) for sid in IDS],
        requests=[], error=None)


def run_once(port, *, physical_isolation_confirmed, ports, serial_factory,
             ownership_guard=None, clock=time.monotonic, sleep=time.sleep):
    report = initial_report(port)
    started = clock()
    query_started = query_ended = None
    raw = transport = None
    current = None
    try:
        if physical_isolation_confirmed is not True:
            raise ReadError('Explicit physical isolation/support confirmation required')
        report['identity'] = selected_port(port, ports)
        report['physical_isolation_confirmed_by_caller'] = True
        report['physical_confirmation_scope'] = ['ESP32 TX/RX/GND separated from URT2',
                                                'old battery isolated', 'body supported']
        if ownership_guard is None:
            raise ReadError('Explicit local UART ownership guard required')
        ownership_guard.before_open()
        raw = serial_factory(port=None, baudrate=BAUD, timeout=0,
                             write_timeout=.05, exclusive=True)
        raw.port = port
        report['serial_open_attempts'] = 1
        raw.open()
        report['serial_opened'] = True
        ownership_guard.after_open(raw)
        transport = ReadOnlyTransport(raw)
        query_started = clock()
        deadline = query_started + TOTAL_SECONDS
        for current in report['results']:
            for address, size in READS:
                current['stage'] = address
                payload = read_register(transport, current['id'], address, size,
                                        deadline, clock, sleep)
                current['reads'].append(dict(register=address, payload_hex=payload.hex(),
                                            elapsed_seconds=clock() - started))
                if address == 5 and payload != bytes((current['id'],)):
                    raise ReadError('Servo ID register mismatch')
                if address == 56:
                    position = int.from_bytes(payload[:2], 'little')
                    if position > 4095:
                        raise ReadError('Unsupported position outside 0..4095')
                    current['position'] = position
                if address == 40:
                    current['torque'] = payload[0]
                    if payload != b'\x00':
                        raise ReadError('Actual torque is nonzero or invalid; not disabled')
            current.update(status='healthy_torque_off', stage='complete')
        report['query_completed'] = True
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        if current is not None:
            current['status'] = ('device_fault' if isinstance(error, DeviceFault)
                                 else 'timeout' if isinstance(error, TimeoutError) else 'rejected')
            current['error'] = str(error)
            if isinstance(error, DeviceFault):
                current['device_error'] = error.code
    finally:
        query_ended = clock()
        if ownership_guard is not None:
            report['ownership_proof'] = ownership_guard.proof
        if transport is not None:
            report['requests'] = transport.requests
        if raw is not None:
            try:
                raw.close()
                report['serial_closed'] = raw.is_open is False
                if report['serial_closed'] is not True:
                    raise ReadError('Serial remained open after close')
            except Exception as error:
                report['serial_closed'] = False
                report['close_error'] = str(error)
        report['elapsed_seconds'] = clock() - started
        report['read_stage_elapsed_seconds'] = (None if query_started is None
                                               else query_ended - query_started)
    report['all_15_healthy_torque_off'] = (report['query_completed']
        and report['serial_closed'] is True and report['error'] is None)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--physical-isolation-confirmed', action='store_true',
        help='Human confirms ESP32/URT2 TX/RX/GND separated, old battery isolated, body supported')
    args = parser.parse_args(argv)
    if not args.execute_once:
        print(json.dumps(dict(status='prepared_only', hardware_accessed=False,
            required_flags=['--execute-once', '--port', '--run-dir', '--physical-isolation-confirmed'],
            note='URT2 USB native mode only; no motion or automatic 8767 unlock')))
        return 0
    if not args.port or not args.run_dir or not args.physical_isolation_confirmed:
        parser.error('Execute requires explicit port, fresh run directory and physical confirmation')
    directory = args.run_dir
    if (not directory.is_absolute() or not directory.parent.is_dir()
            or not directory.parent.resolve().is_relative_to(OUTPUT.resolve())):
        parser.error('Use a fresh absolute directory under workspace/output')
    os.umask(0o077)
    directory.mkdir(mode=0o700, exist_ok=False)
    report = initial_report(args.port)
    try:
        import serial
        from serial.tools import list_ports
        if serial.VERSION != '3.5':
            raise ReadError('Reviewed pyserial 3.5 required')
        class NoImplicitFlushSerial(serial.Serial):
            def _reset_input_buffer(self):
                pass  # Preserve unsolicited bytes for rejection, including at open.
        report = run_once(args.port, physical_isolation_confirmed=True,
                          ports=list(list_ports.comports()), serial_factory=NoImplicitFlushSerial,
                          ownership_guard=LocalOwnershipGuard(args.port))
    except Exception as error:
        report['error'] = str(error)
    fd = os.open(directory / 'report.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report['all_15_healthy_torque_off'] else 20


if __name__ == '__main__':
    raise SystemExit(main())
