"""Read bounded, fresh input snapshots and add a gamepad panel to native UI."""
import json
import math
import os
from pathlib import Path
import stat
import time

STATE = Path('/run/xgoduck-gamepad/input.json')
MAX_BYTES = 32768


def snapshot(path=STATE, *, now=None):
    age_ms = None
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as stream:
            node = os.fstat(stream.fileno())
            if not stat.S_ISREG(node.st_mode) or node.st_uid != os.getuid() or node.st_mode & 0o022:
                raise ValueError('Input snapshot is not an owned regular file')
            raw = stream.read(MAX_BYTES + 1)
        if len(raw) > MAX_BYTES:
            raise ValueError('Input snapshot too large')
        value = json.loads(raw)
        # The publisher atomically replaces this file. Timestamp AFTER the
        # read: sampling before open can compare a new publication against an
        # older time when this thread yields, falsely declaring it offline.
        now = time.monotonic() if now is None else now
        published = value['published_at']
        if type(published) not in (int, float) or not math.isfinite(published):
            raise ValueError('Invalid input publication timestamp')
        age_ms = round((now-published)*1000, 2)
        if published > now:
            raise ValueError('Input publication timestamp is in the future')
        if now-published > .5:
            raise ValueError('Input reader stale')
        if value['schema'] != 1 or value['receiver'] != '0079:181c':
            raise ValueError('Input receiver changed')
        if value['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise ValueError('Input reader boot changed')
        if type(value['connected']) is not bool or type(value['sequence']) is not int or value['sequence'] < 0:
            raise ValueError('Invalid input snapshot')
        session = value['session']
        if value['connected'] and (type(session) is not str or len(session) != 32
                                  or any(c not in '0123456789abcdef' for c in session)):
            raise ValueError('Invalid input session')
        for field, size, low, high in (('axes', 8, -32767, 32767), ('buttons', 15, 0, 1)):
            values = value[field]
            if type(values) is not list or len(values) != size or any(type(v) is not int or not low <= v <= high for v in values):
                raise ValueError('Invalid input values')
        if type(value['events']) is not list or len(value['events']) > 64:
            raise ValueError('Invalid input events')
        events, previous = [], -1
        for event in value['events']:
            kind, number, number_value, seq, at = (event[k] for k in ('kind','number','value','sequence','at'))
            if (type(kind) is not int or kind not in (1, 2) or type(number) is not int
                    or not 0 <= number < (15 if kind == 1 else 8)
                    or type(number_value) is not int
                    or not (0 <= number_value <= 1 if kind == 1 else -32767 <= number_value <= 32767)
                    or type(seq) is not int or not previous < seq <= value['sequence']
                    or type(at) not in (int,float) or not math.isfinite(at) or at > now):
                raise ValueError('Invalid input event')
            previous = seq
            events.append(dict(sequence=seq, kind=kind, number=number, value=number_value,
                               age_ms=round((now-at)*1000, 2)))
        return dict(connected=value['connected'], receiver=value['receiver'], session=value['session'],
                    sequence=value['sequence'], axes=value['axes'], buttons=value['buttons'],
                    age_ms=age_ms, events=events, error=value.get('error'))
    except (OSError, ValueError, KeyError, TypeError) as error:
        return dict(connected=False, error=str(error), age_ms=age_ms, events=[])


def toolbar(page):
    source = page.decode('utf-8') if isinstance(page, bytes) else page
    fragment = Path(__file__).with_name('gamepad_panel.html').read_text()
    if source.count('</body>') != 1:
        raise ValueError('Gamepad panel insertion point changed')
    return source.replace('</body>', fragment + '\n</body>').encode('utf-8')
