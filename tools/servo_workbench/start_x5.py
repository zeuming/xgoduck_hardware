#!/usr/bin/env python3
"""Start or reuse the loopback ESP32 workbench without interrupting a running job."""
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

HERE = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8765/api/status'


def running():
    try:
        with urlopen(URL, timeout=1) as response:
            state = json.load(response)
    except URLError:
        return False
    if state.get('transport') != 'esp32':
        raise RuntimeError('8765 已被其他服务占用，请先检查该服务')
    return True


def main():
    if not running():
        with (HERE / 'workbench.log').open('ab') as log:
            process = subprocess.Popen(
                [sys.executable, '-u', str(HERE / 'app.py'), '--esp32-bridge',
                 '--bridge-port', '/dev/ttyUSB0'], cwd=HERE, stdin=subprocess.DEVNULL,
                stdout=log, stderr=log, start_new_session=True)
        (HERE / 'workbench.pid').write_text(str(process.pid) + '\n')
        for _ in range(30):
            if running():
                break
            if process.poll() is not None:
                raise RuntimeError('网页服务启动失败，请查看 workbench.log')
            time.sleep(.1)
        else:
            raise RuntimeError('网页服务启动超时，请查看 workbench.log')
    print('X5 舵机工作台已就绪：http://127.0.0.1:8765/', flush=True)


if __name__ == '__main__':
    main()
