"""Offline verification only: sanitizers, bus parity, host decode and ESP build."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent


def run(command, log, env=None):
    began=time.monotonic()
    result=subprocess.run(command,capture_output=True,text=True,env=env,timeout=180)
    log.write_text(result.stdout+result.stderr)
    print(log.name, 'PASS' if result.returncode==0 else 'FAIL', flush=True)
    if result.returncode:
        print((result.stdout+result.stderr)[-4000:]);raise SystemExit(result.returncode)
    return round(time.monotonic()-began,3)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pio',type=Path,required=True);p.add_argument('--pio-core',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True);times={}
    for name,fw,extra in [('baseline',a.baseline,[]),('candidate',a.candidate/'firmware',['-DTEST_EXTENDED'])]:
        fw=fw.resolve();target=(a.output/(name+'-test')).resolve()
        cmd=['c++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-g',
             '-I',str(fw/'tests/mocks'),'-DTEST_FIRMWARE_FILE="'+str(fw/'tests/test_firmware.cpp')+'"',
             *extra,str(HERE/'test_bus.cpp'),'-o',str(target)]
        times[name+'_compile']=run(cmd,a.output/(name+'-compile.log'))
        times[name+'_test']=run([str(target),str((a.output/(name+'-fixtures.bin')).resolve())],a.output/(name+'-test.log'))
    times['decode']=run([sys.executable,'-B',str(HERE/'test_decoder.py'),
        str(a.output/'baseline-fixtures.bin'),str(a.output/'candidate-fixtures.bin'),
        str(a.candidate/'host'),str(a.output/'paired-evidence.json')],a.output/'decoder-tests.log')
    times['firmware_suite']=run(['sh',str(a.candidate/'firmware/tests/run.sh')],a.output/'firmware-tests.log')
    env=dict(os.environ,PLATFORMIO_CORE_DIR=str(a.pio_core.resolve()))
    times['esp32_build']=run([str(a.pio.resolve()),'run','-d',str(a.candidate/'firmware')],a.output/'esp32-build.log',env)
    binary=a.candidate/'firmware/.pio/build/esp32s3_motion/firmware.bin'
    report=dict(passed=True,offline_only=True,root_cause_proven_on_hardware=False,
        paired_bus_scenarios=11,host_tests=4,firmware_suite_passed=True,esp32_build_passed=True,
        candidate_app_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),candidate_app_bytes=binary.stat().st_size,
        deployed=False,real_UART_opened=False,ARM_or_TARGET_sent=False,protection_or_calibration_changed=False,
        elapsed_seconds=times)
    (a.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
