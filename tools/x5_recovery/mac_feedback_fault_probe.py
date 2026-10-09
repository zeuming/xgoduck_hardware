#!/usr/bin/env python3
"""Explicit one-attempt Mac HELLO/FaultDetails query; no X5 or flashing.

The phase flag records the caller's real physical confirmation; it cannot
detect battery/support state. Import performs no enumeration, library loading,
device access, SSH or output write. Opening a USB UART can still reset an MCU;
this tool supplies no electrical or board-storage health certification.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import termios
import time

WORKSPACE = Path('/Users/lo/Documents/project/xgoduck_hardware')
OUTPUT = WORKSPACE / 'output/esp32_motion'
PORT = '/dev/cu.usbserial-1110'
TTY_ALIAS = '/dev/tty.usbserial-1110'
SERIAL_PACKAGE = Path('/tmp/xgoduck-pio-venv/lib/python3.12/site-packages/serial')
TOTAL_SECONDS = 4.0
VERIFIED_APP_SHA = 'e4b7fd872953c23ac97e0dc5d9a922946b2e5b2a4cfe805b185bcaa19a36b33b'
VERIFIED_MAC = '44:bd:8d:6a:6c:00'
VERIFIED_FLASH_REPORT = OUTPUT / 'fault-details-direct-mac-20261004-01/report.json'


def _libraries():
    import serial
    from serial.tools import list_ports
    if serial.__version__ != '3.5' or Path(serial.__file__).resolve().parent != SERIAL_PACKAGE.resolve():
        raise RuntimeError('Unreviewed PySerial version/path')
    import feedback_fault_probe
    namespace = {}
    exec(compile(feedback_fault_probe.runtime_source(), '<fault-probe-definitions>', 'exec'), namespace)
    return serial, list_ports, namespace


def _no_flush_serial(base):
    class ObservationalSerial(base):
        # PySerial 3.5 open normally flushes input and writes DTR/RTS. Avoid
        # those operations so unsolicited input is rejected by the protocol.
        def _reset_input_buffer(self):
            pass
        def _update_dtr_state(self):
            pass
        def _update_rts_state(self):
            pass
    return ObservationalSerial


def _identity(ports):
    matches = [item for item in ports if item.device == PORT]
    if (len(matches) != 1 or matches[0].vid != 0x1a86 or matches[0].pid != 0x7523
            or matches[0].location != '1-1.1' or matches[0].serial_number is not None):
        raise RuntimeError('Reviewed USB serial identity/location changed')
    return dict(port=PORT, tty_alias=TTY_ALIAS, vid=0x1a86, pid=0x7523,
                location='1-1.1', serial_number=None)


def _holders(deadline, own_fd=None):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError('Mac diagnostic deadline expired')
    result = subprocess.run(['/usr/sbin/lsof', '-nP', '-Fpf', PORT, TTY_ALIAS],
                            capture_output=True, timeout=min(1.0, remaining))
    if result.stderr or result.returncode not in (0, 1):
        raise RuntimeError('Serial/cu alias ownership check failed')
    if result.returncode == 1 and not result.stdout:
        if own_fd is not None:
            raise RuntimeError('Opened serial descriptor is absent from ownership inventory')
        return
    current_pid = None
    owners = []
    for line in result.stdout.decode('ascii', 'strict').splitlines():
        if line.startswith('p') and line[1:].isdigit():
            current_pid = int(line[1:])
        elif line.startswith('f') and line[1:].isdigit() and current_pid is not None:
            owners.append((current_pid, int(line[1:])))
        else:
            raise RuntimeError('Malformed serial/cu ownership inventory')
    if not owners or any(pair != (os.getpid(), own_fd) for pair in owners):
        raise RuntimeError('Serial/cu alias has an unexpected holder')


def _query_once(fd, namespace, deadline, metadata):
    namespace['_check_deadline'](deadline)
    if namespace['_exchange'](fd, 1, 1, min(deadline, time.monotonic() + .20), metadata) != b'XGODUCK_MOTION_1':
        raise RuntimeError('Unknown motion firmware identity')
    payload = namespace['_fault_exchange'](fd, 2, deadline, metadata)
    diagnostic = namespace['_decode_fault_details'](payload)
    metadata['last_diagnostic'] = diagnostic
    namespace['_check_deadline'](deadline)
    return diagnostic


def _restore_custom_baud(port):
    # Darwin stores PySerial's nonstandard 921600 speed via IOSSIOSPEED after
    # tcsetattr(B38400 placeholder). Our later HUPCL tcsetattr must be followed
    # by that same speed ioctl, before querying. It does not flush or set lines.
    if (port.baudrate == 921600 and not hasattr(termios, 'B921600')
            and 921600 not in port.BAUDRATE_CONSTANTS):
        if not sys.platform.startswith('darwin'):
            raise RuntimeError('Unsupported nonstandard baud restoration platform')
        port._set_special_baudrate(port.baudrate)
        return True
    return False


def _capture_boot(fd, namespace, deadline, evidence=None):
    data = bytearray()
    def retain():
        if evidence is not None:
            raw = bytes(data[:8192])
            evidence.update(raw_boot_hex=raw.hex(), raw_boot_bytes=len(raw),
                            raw_boot_sha256=hashlib.sha256(raw).hexdigest(),
                            raw_boot_truncated=len(data) > 8192)
    retain()
    until = deadline - .05  # Leave a bounded interval to close the descriptor.
    while time.monotonic() < until:
        readable, _, exceptional = namespace['select'].select(
            [fd], [], [fd], max(0., until - time.monotonic()))
        if exceptional:
            raise RuntimeError('Passive boot serial exception')
        if not readable:
            break
        chunk = os.read(fd, min(1024, 8193 - len(data)))
        if not chunk:
            raise RuntimeError('Passive boot serial stream ended')
        data.extend(chunk)
        retain()
        if len(data) > 8192:
            return bytes(data[:8192]), True
    return bytes(data), False


def run_once(serial_module, list_ports, namespace, phase, *, passive_boot=False):
    if phase not in ('battery-off', 'powered-supported'):
        raise ValueError('Unknown physical phase')
    if passive_boot and phase != 'battery-off':
        raise ValueError('Passive boot capture requires confirmed battery-off phase')
    started = time.monotonic()
    budget = 1.5 if passive_boot else TOTAL_SECONDS
    deadline = started + budget
    report = dict(phase=phase, physical_phase_confirmed_by_caller=True,
                  diagnostic_only=True, automatic_retry=False,
                  serial_open_attempts=0, serial_closed=None, diagnostic=None,
                  error=None, mcu_reset_possible=True, electrical_safety_verified=False,
                  x5_accessed=False, board_storage_health_certified=False,
                  calibration_written=False, servo_motion_authorized=False,
                  input_flushed=False, modem_pulsed=False)
    report['passive_boot_capture'] = passive_boot
    metadata = {'request_types_attempted': [], 'reply_types_received': [],
                'request_bytes_written': 0}
    port = None
    previous_handler = signal.getsignal(signal.SIGALRM)
    if signal.getitimer(signal.ITIMER_REAL) != (0., 0.):
        raise RuntimeError('Existing process alarm prevents exclusive diagnostic deadline')
    def timed_out(signum, frame):
        raise TimeoutError('Bounded Mac diagnostic deadline expired')
    signal.signal(signal.SIGALRM, timed_out)
    signal.setitimer(signal.ITIMER_REAL, budget)
    try:
        report['usb_identity'] = _identity(list_ports.comports())
        nodes = {name: os.stat(name, follow_symlinks=False) for name in (PORT, TTY_ALIAS)}
        if any(not stat.S_ISCHR(info.st_mode) or info.st_uid not in (0, os.getuid()) for info in nodes.values()):
            raise RuntimeError('Serial paths are not direct, trusted-owner character nodes')
        _holders(deadline)
        port = _no_flush_serial(serial_module.Serial)(port=None, baudrate=115200 if passive_boot else 921600,
            timeout=.20, write_timeout=.20, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = PORT
        report['serial_open_attempts'] = 1
        port.open()
        fd = port.fileno()
        fcntl.ioctl(fd, termios.TIOCEXCL)
        actual = os.fstat(fd)
        expected = nodes[PORT]
        fields = ('st_dev', 'st_ino', 'st_rdev', 'st_uid', 'st_mode')
        if any(getattr(actual, key) != getattr(expected, key) for key in fields):
            raise RuntimeError('Serial character node changed while opening')
        _identity(list_ports.comports())
        for name, original in nodes.items():
            current = os.stat(name, follow_symlinks=False)
            if any(getattr(current, key) != getattr(original, key) for key in fields):
                raise RuntimeError('Serial/cu alias node changed while opening')
        _holders(deadline, fd)
        report['modem_before_hupcl'] = namespace['_modem_bits'](fd)
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        if termios.tcgetattr(fd)[2] & termios.HUPCL:
            raise RuntimeError('UART HUPCL could not be cleared')
        report['custom_baud_restored_after_hupcl'] = _restore_custom_baud(port)
        report['modem_after_hupcl'] = namespace['_modem_bits'](fd)
        if any(report['modem_before_hupcl'][key] != report['modem_after_hupcl'][key] for key in ('dtr', 'rts')):
            raise RuntimeError('Observed modem lines changed during configuration')
        if passive_boot:
            raw, truncated = _capture_boot(fd, namespace, deadline, report)
            report.update(raw_boot_hex=raw.hex(), raw_boot_bytes=len(raw),
                          raw_boot_sha256=hashlib.sha256(raw).hexdigest(),
                          raw_boot_truncated=truncated)
            if truncated:
                raise RuntimeError('Passive boot data exceeds 8192 bytes; capture incomplete')
        else:
            report['diagnostic'] = _query_once(fd, namespace, deadline, metadata)
        # The diagnostic itself, including dangerous flags, grants no motion.
        report['query_completed'] = True
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['query_completed'] = False
    finally:
        try:
            if port is not None:
                port.close()
                report['serial_closed'] = (not port.is_open and getattr(port, 'fd', None) is None)
            else:
                report['serial_closed'] = True  # No port object/open was attempted.
        except BaseException as error:
            report['serial_closed'] = False
            report['close_error'] = str(error) or type(error).__name__
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
    report['query'] = metadata
    report['elapsed_seconds'] = time.monotonic() - started
    return report


def _verified_app_report(path):
    path = Path(path)
    if path != VERIFIED_FLASH_REPORT:
        raise RuntimeError('Unexpected verified application report path')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or info.st_nlink != 1
                or not 0 < info.st_size <= 65536):
            raise RuntimeError('Verified application report is not a private bounded file')
        data = stream.read(65537)
    value = json.loads(data)
    if (value.get('phase') != 'complete' or value.get('port') != PORT
            or value.get('app_sha256') != VERIFIED_APP_SHA or value.get('mac') != VERIFIED_MAC
            or value.get('chip') != 'ESP32-S3' or value.get('readback_verified') is not True
            or value.get('serial_closed') is not True
            or value.get('verified_region_sha256') != value.get('expected_region_sha256')
            or not isinstance(value.get('verified_region_sha256'), str)):
        raise RuntimeError('Application readback/device proof did not match')
    return dict(path=str(path), sha256=hashlib.sha256(data).hexdigest(),
                app_sha256=VERIFIED_APP_SHA, mac=VERIFIED_MAC,
                readback_verified=True, prior_serial_closed=True)


def _restart_to_app(fd, namespace, report):
    before = namespace['_modem_bits'](fd)
    cleared = before['raw'] & ~(termios.TIOCM_DTR | termios.TIOCM_RTS)
    report['reset_modem_before'] = before
    report['reset_steps'] = []
    pulse_attempted = release_attempted = False
    def apply(raw, stage, dtr, rts):
        fcntl.ioctl(fd, termios.TIOCMSET, namespace['struct'].pack('I', raw))
        observed = namespace['_modem_bits'](fd)
        report['reset_steps'].append(dict(stage=stage, requested_dtr=dtr,
            requested_rts=rts, observed=observed))
        if observed['dtr'] is not dtr or observed['rts'] is not rts:
            raise RuntimeError('Explicit reset modem readback disagrees at ' + stage)
    try:
        apply(cleared, 'clear_dtr_rts', False, False)
        report['modem_pulse_attempted'] = True
        pulse_attempted = True
        apply(cleared | termios.TIOCM_RTS, 'assert_rts', False, True)
        report['modem_pulsed'] = True
        time.sleep(.10)
        release_attempted = True
        apply(cleared, 'release_rts', False, False)
        report['reset_completed'] = True
    finally:
        # Release a partially applied pulse once; never replay the reset.
        if pulse_attempted and not release_attempted:
            release_attempted = True
            apply(cleared, 'cleanup_release_rts', False, False)


def restart_boot_capture(serial_module, list_ports, namespace, phase, verified_app, *, query_after_restart=False):
    """Explicit battery-off reset/capture, optionally one same-open query."""
    if phase != 'battery-off':
        raise ValueError('Application restart capture requires confirmed battery-off phase')
    if (verified_app.get('app_sha256') != VERIFIED_APP_SHA or verified_app.get('mac') != VERIFIED_MAC
            or verified_app.get('readback_verified') is not True
            or verified_app.get('prior_serial_closed') is not True):
        raise ValueError('Verified application/device proof is required before restart')
    started = time.monotonic()
    deadline = started + 4.0
    report = dict(phase=phase, physical_phase_confirmed_by_caller=True,
        restart_boot_capture=True, explicit_application_reset=True,
        restart_and_query=query_after_restart,
        verified_app=verified_app, automatic_retry=False, serial_open_attempts=0,
        serial_closed=None, diagnostic=None, query_completed=False,
        request_bytes_written=0, reset_completed=False, modem_pulsed=False,
        modem_pulse_attempted=False, mcu_reset_possible=True, input_flushed=False,
        x5_accessed=False, board_storage_health_certified=False,
        servo_motion_authorized=False, calibration_written=False,
        electrical_safety_verified=False, application_ready_proved=False,
        application_protocol_responded=False, error=None)
    port = None
    metadata = {'request_types_attempted': [], 'reply_types_received': [],
                'request_bytes_written': 0}
    old_handler = signal.getsignal(signal.SIGALRM)
    if signal.getitimer(signal.ITIMER_REAL) != (0., 0.):
        raise RuntimeError('Existing process alarm prevents exclusive restart deadline')
    def timed_out(signum, frame):
        raise TimeoutError('Explicit restart capture deadline expired')
    signal.signal(signal.SIGALRM, timed_out)
    signal.setitimer(signal.ITIMER_REAL, 4.0)
    try:
        report['usb_identity'] = _identity(list_ports.comports())
        nodes = {name: os.stat(name, follow_symlinks=False) for name in (PORT, TTY_ALIAS)}
        if any(not stat.S_ISCHR(info.st_mode) or info.st_uid not in (0, os.getuid()) for info in nodes.values()):
            raise RuntimeError('Serial paths are not direct, trusted-owner character nodes')
        _holders(deadline)
        port = _no_flush_serial(serial_module.Serial)(port=None, baudrate=115200,
            timeout=.20, write_timeout=.20, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = PORT
        report['serial_open_attempts'] = 1
        port.open()
        fd = port.fileno()
        fcntl.ioctl(fd, termios.TIOCEXCL)
        fields = ('st_dev', 'st_ino', 'st_rdev', 'st_uid', 'st_mode')
        actual = os.fstat(fd)
        if any(getattr(actual, key) != getattr(nodes[PORT], key) for key in fields):
            raise RuntimeError('Serial character node changed while opening')
        _identity(list_ports.comports())
        for name, before in nodes.items():
            now = os.stat(name, follow_symlinks=False)
            if any(getattr(now, key) != getattr(before, key) for key in fields):
                raise RuntimeError('Serial/cu alias node changed while opening')
        _holders(deadline, fd)
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        if termios.tcgetattr(fd)[2] & termios.HUPCL:
            raise RuntimeError('UART HUPCL could not be cleared')
        report['hupcl_cleared'] = True
        namespace['_check_deadline'](deadline)
        _restart_to_app(fd, namespace, report)
        raw, truncated = _capture_boot(fd, namespace, min(deadline, time.monotonic() + 1.5), report)
        report.update(raw_boot_hex=raw.hex(), raw_boot_bytes=len(raw),
            raw_boot_sha256=hashlib.sha256(raw).hexdigest(), raw_boot_truncated=truncated)
        if truncated:
            raise RuntimeError('Restart boot data exceeds 8192 bytes; capture incomplete')
        if query_after_restart:
            before_baud = namespace['_modem_bits'](fd)
            report['query_modem_before_baud'] = before_baud
            if before_baud['dtr'] is not False or before_baud['rts'] is not False:
                raise RuntimeError('Restart/query requires both modem lines released')
            namespace['_check_deadline'](deadline)
            # Keep this sole descriptor open. PySerial reconfiguration does not
            # flush; our subclass also suppresses any initial modem writes.
            port.baudrate = 921600
            attrs = termios.tcgetattr(fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            if termios.tcgetattr(fd)[2] & termios.HUPCL:
                raise RuntimeError('Query UART HUPCL could not be cleared')
            report['custom_baud_restored_after_hupcl'] = _restore_custom_baud(port)
            after_baud = namespace['_modem_bits'](fd)
            report['query_modem_after_baud'] = after_baud
            if after_baud['dtr'] is not False or after_baud['rts'] is not False:
                raise RuntimeError('Modem lines changed during same-open query configuration')
            report['diagnostic'] = _query_once(fd, namespace, deadline, metadata)
            report['application_protocol_responded'] = True
        # Default mode proves capture completion; query mode additionally proves
        # one protocol exchange. Neither declares hardware readiness or health.
        report['query_completed'] = True
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        try:
            if port is not None:
                port.close()
                report['serial_closed'] = not port.is_open and getattr(port, 'fd', None) is None
            else:
                report['serial_closed'] = True
        except BaseException as error:
            report['serial_closed'] = False
            report['close_error'] = str(error) or type(error).__name__
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)
    report['elapsed_seconds'] = time.monotonic() - started
    report['query'] = metadata
    report['request_bytes_written'] = metadata['request_bytes_written']
    return report


def _save_new(path, data):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        if stream.write(data) != len(data):
            raise RuntimeError('Incomplete local evidence write')
        stream.flush()
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--phase', choices=('battery-off', 'powered-supported'), required=True)
    parser.add_argument('--execute-once', action='store_true', required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--passive-boot-capture', action='store_true',
                        help='Separate battery-off 115200 RX-only boot record, at most 1.5 seconds/8192 bytes')
    modes.add_argument('--restart-boot-capture', action='store_true',
                       help='Explicit battery-off RTS application reset, then RX-only 115200 capture')
    modes.add_argument('--restart-and-query', action='store_true',
                       help='Explicit battery-off verified-app reset/capture, then one same-open HELLO/FaultDetails query')
    parser.add_argument('--verified-flash-report', type=Path)
    args = parser.parse_args()
    directory = args.run_dir
    if (not directory.is_absolute() or directory.parent.resolve() != OUTPUT.resolve()
            or not directory.name.startswith('diagnostic-')):
        parser.error('A fresh diagnostic-* directory in output/esp32_motion is required')
    os.umask(0o077)
    directory.mkdir(mode=0o700, exist_ok=False)
    try:
        proof = None
        if args.restart_boot_capture or args.restart_and_query:
            if args.phase != 'battery-off' or args.verified_flash_report is None:
                raise ValueError('Restart capture requires battery-off and --verified-flash-report')
            proof = _verified_app_report(args.verified_flash_report)
        elif args.verified_flash_report is not None:
            raise ValueError('Verified flash report is only used for explicit restart capture')
        serial_module, list_ports, namespace = _libraries()
        if args.restart_boot_capture or args.restart_and_query:
            report = restart_boot_capture(serial_module, list_ports, namespace, args.phase, proof,
                                          query_after_restart=args.restart_and_query)
        else:
            report = run_once(serial_module, list_ports, namespace, args.phase,
                              passive_boot=args.passive_boot_capture)
    except BaseException as error:
        # Libraries are loaded before enumeration/open; report startup failure
        # without fabricating a diagnostic or a board health claim.
        report = dict(phase=args.phase, passive_boot_capture=args.passive_boot_capture,
                      query_completed=False, serial_closed=None, error=str(error),
                      x5_accessed=False, servo_motion_authorized=False,
                      board_storage_health_certified=False, automatic_retry=False)
    if 'raw_boot_hex' in report:
        _save_new(directory / 'boot-uart.bin', bytes.fromhex(report.pop('raw_boot_hex')))
    _save_new(directory / 'report.json', (json.dumps(report, ensure_ascii=False,
        indent=2, allow_nan=False) + '\n').encode())
    print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if report['query_completed'] and report['serial_closed'] else 20


if __name__ == '__main__':
    raise SystemExit(main())
