"""Prepare an offline recovery from the exact current workbench bundle.

Keep grouped voltage opt-in disabled in motion. Register the command runtime
with the real request monitor so explicitly started actions can renew their
existing watchdog. This script never accesses or deploys to hardware.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def prepare(baseline, output):
    shutil.copytree(baseline, output,
                    ignore=shutil.ignore_patterns('__pycache__', '.pio'))
    path = output / 'policy/motion.py'
    source = path.read_text()
    old = "payload = self._request(SOURCE_STATE, b'\\x01')"
    assert source.count(old) == 1
    source = source.replace(old, 'payload = self._request(SOURCE_STATE)')
    old = ("result['servo_voltage_polling']=dict(enabled=bool(result.get('voltage_supported')),\n"
           "            method='grouped_registers_56_62', additional_servo_requests=0, age_resolution_ms=100)")
    assert source.count(old) == 1
    source = source.replace(old,
        "result['servo_voltage_polling']=dict(enabled=False,\n"
        "            reason='运控恢复原始寄存器56..61同步读取；电压仅在独立维护时采集')")
    compile(source, str(path), 'exec')
    path.write_text(source)
    path = output / 'service/native_gamepad.py'
    source = path.read_text()
    old = "for name in ('browser', 'x5_local')}"
    assert source.count(old) == 1
    source = source.replace(old, "for name in ('browser', 'x5_local', 'x5_runtime')}")
    compile(source, str(path), 'exec')
    path.write_text(source)
    names = ('policy/motion.py', 'service/native_gamepad.py')
    return {name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in names}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.baseline, args.output), indent=2))
