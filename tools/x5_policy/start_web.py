#!/usr/bin/env python3
"""Start or reuse the X5 loopback model web service."""
import json
from pathlib import Path
import subprocess
import time
from urllib.error import URLError
from urllib.request import urlopen

HERE = Path(__file__).resolve().parent
MOTION_PORT = HERE/'motion_port.txt'


def running():
    try:
        with urlopen('http://127.0.0.1:8767/api/status',timeout=1) as response:
            status = json.load(response)
    except URLError:
        return False
    if status.get('app') != 'xgoduck-policy-workbench':
        raise RuntimeError('8767 已被其他服务占用')
    return True


if not running():
    command = [str(HERE/'.venv/bin/python'), '-u', str(HERE/'web_app.py')]
    # Installed only after the motion firmware has been verified on the board.
    if MOTION_PORT.exists():
        motion_port = MOTION_PORT.read_text().strip()
        if motion_port != '/dev/ttyUSB0':
            raise RuntimeError('motion_port.txt 串口配置无效')
        command.extend(['--motion-port', motion_port])
    with (HERE/'web.log').open('ab') as log:
        process = subprocess.Popen(command,
                                   cwd=HERE, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                   start_new_session=True)
    (HERE/'web.pid').write_text(str(process.pid)+'\n')
    for _ in range(50):
        if running():
            break
        if process.poll() is not None:
            raise RuntimeError('网页启动失败，请检查 web.log')
        time.sleep(.1)
    else:
        raise RuntimeError('网页启动超时，请检查 web.log')
print('X5 模型测试服务已就绪',flush=True)
