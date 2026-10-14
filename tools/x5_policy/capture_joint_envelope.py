"""Read-only manual joint-range observation. No connect, ARM, TARGET or save API.

Requires an already connected workbench in OFF with all torque confirmed off.
Observed extrema are evidence, never new safe limits or calibration updates.
"""
import argparse
import hashlib
import http.cookiejar
import json
import math
from pathlib import Path
import time
import urllib.request

# This is manual OFF observation through the page's ~250 ms display cache,
# not the controller's 100 ms actuation freshness boundary.
DISPLAY_MAX_AGE_MS = 500


def sample(hardware, joint_id):
    if not hardware or hardware.get('mode') != 'off' or hardware.get('connected') is not True:
        raise ValueError('Read-only range capture requires connected OFF feedback')
    if (hardware.get('armed') is not False or hardware.get('torque_ids') != []
            or hardware.get('release_confirmed') is not True):
        raise ValueError('All torque must be confirmed off; capture will not command STOP')
    if hardware.get('fault_ids') or joint_id not in hardware.get('online_ids', []):
        raise ValueError('Selected joint feedback is invalid')
    age = hardware.get('age_ms')
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age < DISPLAY_MAX_AGE_MS:
        return None
    row = next((r for r in hardware.get('rawdata', []) if r.get('id') == joint_id), None)
    if row is None or type(row.get('position')) is not int or not 0 <= row['position'] <= 4095:
        raise ValueError('Encoder position invalid')
    return dict(sample_us=hardware['sample_us'], position=row['position'], feedback_age_ms=age)


def display_sample(motion, joint_id):
    if motion.get('owns_connection') is not True or motion.get('error'):
        raise ValueError('Read-only connection ended or reported an error')
    # Two independent display caches can temporarily hide aged data. Skip the
    # absent packet, never reuse last_hardware; the advancing-sample watchdog
    # below still makes a sustained gap fail the capture.
    hardware = motion.get('hardware')
    return None if hardware is None else sample(hardware, joint_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--id', type=int, default=14)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.id not in [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34] or not 1 <= args.seconds <= 60:
        parser.error('Known servo ID and 1..60 seconds required')
    if args.output.exists():
        parser.error('Output must be new')
    base = args.url.rstrip('/')
    client = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    client.open(base+'/', timeout=3).read()
    def read():
        return json.load(client.open(base+'/api/live-status', timeout=3))
    first = read()
    calibration = first['calibration']
    rows, failure, last = [], None, None
    started = time.monotonic()
    last_advance = started
    try:
        while time.monotonic()-started < args.seconds:
            status = read()
            if status['calibration'] != calibration:
                raise ValueError('Calibration changed during capture')
            motion = status['motion']
            # Never substitute last_hardware: it is historical, not live proof.
            row = display_sample(motion, args.id)
            if row is not None and row['sample_us'] != last:
                rows.append(dict(row, elapsed_s=time.monotonic()-started))
                last = row['sample_us']
                last_advance = time.monotonic()
            if time.monotonic()-last_advance > 1:
                raise ValueError('No advancing live encoder sample for one second')
            time.sleep(.1)
    except (OSError, KeyError, ValueError) as error:
        failure = str(error)
    joint = next(j for j in calibration['joints'] if j['id'] == args.id)
    positions = [r['position'] for r in rows]
    result = dict(joint_id=args.id, samples=rows, error=failure,
        observed_raw_min=min(positions, default=None), observed_raw_max=max(positions, default=None),
        saved_joint=joint, calibration_sha256=hashlib.sha256(json.dumps(calibration,
            sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
        complete=failure is None and len(rows)>=5, motor_commands=0, calibration_writes=0,
        display_max_age_ms=DISPLAY_MAX_AGE_MS,
        warning='Manual observed extrema are not independently verified safe mechanical bounds.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','saved_joint')}))
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
