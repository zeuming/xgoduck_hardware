#!/usr/bin/env python3
"""Read-only Linux joystick monitor. Never opens a motor/serial device."""
import argparse
import array
import fcntl
import json
import os
import select
import struct
import time


def describe(fd):
    name = bytearray(128)
    fcntl.ioctl(fd, 0x80806a13, name)
    axes, buttons = array.array('B', [0]), array.array('B', [0])
    fcntl.ioctl(fd, 0x80016a11, axes)
    fcntl.ioctl(fd, 0x80016a12, buttons)
    return dict(name=name.split(b'\0')[0].decode(errors='replace'),
                axis_count=axes[0], button_count=buttons[0])


def decode(raw):
    if len(raw) != 8:
        raise ValueError('Incomplete joystick event')
    timestamp, value, kind, number = struct.unpack('<IhBB', raw)
    if kind & 0x7f not in (1, 2):
        raise ValueError('Unknown joystick event')
    return dict(device_ms=timestamp, value=value, kind='button' if kind & 0x7f == 1 else 'axis',
                number=number, initial=bool(kind & 0x80))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/input/js0')
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        print(json.dumps(dict(device=args.device, **describe(fd))), flush=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            if select.select([fd], [], [], min(.5, max(0, deadline-time.monotonic())))[0]:
                raw = os.read(fd, 8)
                if not raw:
                    raise RuntimeError('Joystick disconnected')
                print(json.dumps(decode(raw)), flush=True)
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
