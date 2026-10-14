#!/usr/bin/env python3
"""Publish USB 0079:181c joystick input. No motor, UART or network access."""
import array
import fcntl
import json
import os
from pathlib import Path
import select
import struct
import time
import uuid

STATE = Path('/run/xgoduck-gamepad/input.json')
AXES = [0, 1, 2, 5, 9, 10, 16, 17]
BUTTONS = list(range(304, 319))


def decode(raw):
    if len(raw) != 8:
        raise ValueError('Incomplete input event')
    _, value, kind, number = struct.unpack('<IhBB', raw)
    initial, kind = bool(kind & 128), kind & 127
    if kind not in (1, 2) or number >= (15 if kind == 1 else 8):
        raise ValueError('Unexpected input event')
    if kind == 1 and value not in (0, 1):
        raise ValueError('Invalid button value')
    return kind, number, value, initial


def open_controller():
    candidates = []
    for node in Path('/sys/class/input').glob('js*'):
        identity = node / 'device/id'
        try:
            if ((identity / 'vendor').read_text().strip() == '0079'
                    and (identity / 'product').read_text().strip() == '181c'):
                candidates.append(node)
        except FileNotFoundError:
            continue
    if len(candidates) != 1:
        raise ValueError('Expected one USB 0079:181c controller')
    path = '/dev/input/' + candidates[0].name
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    try:
        axes, buttons = array.array('B', [0] * 64), array.array('H', [0] * 512)
        fcntl.ioctl(fd, 0x80406a32, axes)
        fcntl.ioctl(fd, 0x84006a34, buttons)
        ac, bc = array.array('B', [0]), array.array('B', [0])
        fcntl.ioctl(fd, 0x80016a11, ac)
        fcntl.ioctl(fd, 0x80016a12, bc)
        if ac[0] != 8 or bc[0] != 15 or axes[:8].tolist() != AXES or buttons[:15].tolist() != BUTTONS:
            raise ValueError('Controller mapping changed')
        opened, current = os.fstat(fd), os.stat(path)
        if (opened.st_dev, opened.st_ino, opened.st_rdev) != (current.st_dev, current.st_ino, current.st_rdev):
            raise ValueError('Controller changed during open')
        return fd, path
    except BaseException:
        os.close(fd)
        raise


def publish(value):
    temporary = STATE.with_suffix('.new')
    raw = json.dumps(value, separators=(',', ':'), allow_nan=False)
    with open(temporary, 'w') as stream:
        stream.write(raw)
    os.replace(temporary, STATE)


def main():
    STATE.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    state = dict(schema=1, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                 receiver='0079:181c', connected=False, session=None, sequence=0,
                 axes=[0] * 8, buttons=[0] * 15, events=[], error=None)
    fd = None
    retry_at = 0.
    while True:
        now = time.monotonic()
        try:
            if fd is None and now >= retry_at:
                fd, path = open_controller()
                state.update(connected=False, session=uuid.uuid4().hex, device=path,
                             axes=[0] * 8, buttons=[0] * 15, events=[], error=None)
                initialized = set()
            if fd is not None:
                if select.select([fd], [], [], .05)[0]:
                    # Bound draining so a noisy device cannot prevent publication.
                    for _ in range(256):
                        try:
                            raw = os.read(fd, 8)
                        except BlockingIOError:
                            break
                        kind, number, value, initial = decode(raw)
                        state['buttons' if kind == 1 else 'axes'][number] = value
                        if initial:
                            initialized.add((kind, number))
                        else:
                            state['sequence'] += 1
                            state['events'].append(dict(sequence=state['sequence'], at=time.monotonic(),
                                kind=kind, number=number, value=value))
                            state['events'] = state['events'][-64:]
                    state['connected'] = len(initialized) == 23
            else:
                time.sleep(.1)
        except (OSError, ValueError) as error:
            if fd is not None:
                os.close(fd)
            fd = None
            state.update(connected=False, events=[], error=str(error))
            retry_at = time.monotonic() + 1.
        state['published_at'] = time.monotonic()
        publish(state)


if __name__ == '__main__':
    main()
