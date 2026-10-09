#!/usr/bin/env python3
"""Check the installed motion link without enabling torque or sending targets.

STOP is the only mutating operation: it releases torque at entry and exit.
Support the robot before running this probe. HELLO/STATE then check the exact
motion protocol, feedback and IMU. No ARM, CONFIG, TARGET or servo-register
write command is available through this program.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import time

from motion import (ALL_MASK, HELLO, MOTOR_IDS, STATE, STOP, SerialTransport,
                    decode_state)

EXPECTED_HELLO = b'XGODUCK_MOTION_1'
MAX_AGE_SECONDS = .05  # Match firmware's initial ARM freshness requirement.
GRAVITY_MS2 = 9.80665


def signed_us_delta(newer, older):
    difference = (newer - older) & 0xffffffff
    return difference - 0x100000000 if difference >= 0x80000000 else difference


def ids_for_mask(mask):
    return [sid for i, sid in enumerate(MOTOR_IDS) if mask & (1 << i)]


def distribution(values):
    if not values:
        return None
    return dict(min=min(values), mean=statistics.fmean(values), max=max(values))


def run_probe(transport, *, seconds=10., settle=2., healthy_for=3., poll_period=.01,
              clock=time.monotonic, sleep=time.sleep):
    """Return raw records and a pass/fail summary; opening hardware is explicit.

    ``transport`` is SerialTransport in production and an injected fake in
    tests. All commands are visibly restricted to STOP, HELLO and STATE.
    """
    result = dict(test='xgoduck_motion_postflash_disarmed_link_acceptance',
                  protocol='XGODUCK_MOTION_1', expected_ids=list(MOTOR_IDS),
                  port=getattr(transport, 'port', None),
                  parameters=dict(seconds=seconds, settle_seconds=settle,
                                  required_final_healthy_seconds=healthy_for,
                                  poll_period_seconds=poll_period,
                                  max_age_ms=MAX_AGE_SECONDS * 1000,
                                  acceleration_norm_range_ms2=[GRAVITY_MS2 * .8, GRAVITY_MS2 * 1.2]),
                  commands_allowed=['STOP', 'HELLO', 'STATE'],
                  torque_release=True, arm_sent=False, config_sent=False, targets_sent=False,
                  records=[], errors=[])
    opened = False
    started = clock()
    hello_verified = False
    initial_stop_confirmed = False
    final_stop_confirmed = False
    last_sample_us = last_imu_us = None
    sample_changed_at = imu_changed_at = None
    sample_advance_times = []
    imu_advance_times = []
    sample_intervals_us = []
    imu_intervals_us = []
    continuous_healthy_since = None
    online_union = fault_union = torque_union = 0
    count = 0
    try:
        transport.open()
        opened = True
        transport.request(STOP)
        initial_stop_confirmed = True
        reply = transport.request(HELLO)
        if reply != EXPECTED_HELLO:
            raise ValueError('HELLO 不是 XGODUCK_MOTION_1，无法验收')
        hello_verified = True
        started = clock()
        while clock() - started < seconds:
            tick_started = clock()
            elapsed = tick_started - started
            record = dict(elapsed_ms=round(elapsed * 1000, 3), settled=elapsed >= settle)
            try:
                payload = transport.request(STATE)
                state = decode_state(payload)
                received = clock()
                record['response_ms'] = round((received - tick_started) * 1000, 3)
                record['state'] = state
                record['motors'] = [dict(id=sid,
                                          online=bool(state['online_mask'] & (1 << i)),
                                          fault=bool(state['fault_mask'] & (1 << i)),
                                          torque=bool(state['torque_mask'] & (1 << i)),
                                          position=state['positions'][i], velocity_raw=state['velocities'][i])
                                     for i, sid in enumerate(MOTOR_IDS)]
                online_union |= state['online_mask']
                fault_union |= state['fault_mask']
                torque_union |= state['torque_mask']
                reasons = []
                for name, value in (('sample', state['sample_us']), ('imu', state['imu_us'])):
                    previous = last_sample_us if name == 'sample' else last_imu_us
                    if previous is None:
                        if name == 'sample': sample_changed_at = received
                        else: imu_changed_at = received
                    elif value != previous:
                        difference = signed_us_delta(value, previous)
                        if difference <= 0:
                            reasons.append(name + '_timestamp_backwards')
                        else:
                            if name == 'sample':
                                sample_advance_times.append(received)
                                sample_intervals_us.append(difference)
                            else:
                                imu_advance_times.append(received)
                                imu_intervals_us.append(difference)
                        if name == 'sample': sample_changed_at = received
                        else: imu_changed_at = received
                last_sample_us, last_imu_us = state['sample_us'], state['imu_us']
                sample_age = max(0., received - sample_changed_at)
                imu_age = max(max(0., received - imu_changed_at),
                              sample_age + max(0, signed_us_delta(state['sample_us'], state['imu_us'])) / 1e6)
                record['sample_age_ms'] = round(sample_age * 1000, 3)
                record['imu_age_ms'] = round(imu_age * 1000, 3)
                norm = math.sqrt(sum(value * value for value in state['acc']))
                record['acceleration_norm_ms2'] = norm
                if state['online_mask'] != ALL_MASK: reasons.append('servos_missing_or_stale')
                if state['fault_mask']: reasons.append('servo_fault')
                if state['torque_mask']: reasons.append('observed_torque_enabled')
                if state['armed']: reasons.append('firmware_armed')
                if state['configured']: reasons.append('unexpected_ram_configuration')
                if state['cookie']: reasons.append('unexpected_arm_cookie')
                if not state['imu_ok']: reasons.append('imu_not_ready')
                if not state['sample_us'] or sample_age > MAX_AGE_SECONDS:
                    reasons.append('servo_timestamp_not_fresh')
                if not state['imu_us'] or imu_age > MAX_AGE_SECONDS:
                    reasons.append('imu_timestamp_not_fresh')
                if not GRAVITY_MS2 * .8 <= norm <= GRAVITY_MS2 * 1.2:
                    reasons.append('acceleration_norm_not_near_gravity')
                record['healthy'] = not reasons
                record['reasons'] = reasons
                count += 1
            except Exception as exc:
                record.update(healthy=False, reasons=['invalid_or_missing_state'], error=str(exc))
                result['errors'].append(dict(elapsed_ms=record['elapsed_ms'], phase='STATE', error=str(exc)))
            if record['settled'] and record['healthy']:
                if continuous_healthy_since is None:
                    continuous_healthy_since = clock()
            else:
                continuous_healthy_since = None
            result['records'].append(record)
            sleep(max(.0001, poll_period - (clock() - tick_started)))
    except Exception as exc:
        result['errors'].append(dict(elapsed_ms=round((clock() - started) * 1000, 3),
                                     phase='connect_or_hello', error=str(exc)))
        continuous_healthy_since = None
    finally:
        ended = clock()
        if opened:
            try:
                transport.request(STOP)
                final_stop_confirmed = True
            except Exception as exc:
                result['errors'].append(dict(phase='final_STOP', error=str(exc)))
            try:
                transport.close()
            except Exception as exc:
                result['errors'].append(dict(phase='close', error=str(exc)))
    final_healthy = max(0., ended - continuous_healthy_since) if continuous_healthy_since is not None else 0.
    records = result['records']
    settled = [record for record in records if record['settled']]
    bad_settled = [record for record in settled if not record['healthy']]
    valid = [record for record in records if 'state' in record]
    def observed_rate(times):
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.
    passed = (hello_verified and initial_stop_confirmed and final_stop_confirmed and
              count >= 2 and final_healthy >= healthy_for and not
              any(error['phase'] in ('final_STOP', 'close') for error in result['errors']))
    reasons = []
    if not hello_verified: reasons.append('exact_motion_protocol_not_verified')
    if not initial_stop_confirmed: reasons.append('initial_STOP_not_acknowledged')
    if not final_stop_confirmed: reasons.append('final_STOP_not_acknowledged')
    if count < 2: reasons.append('insufficient_valid_feedback')
    if final_healthy < healthy_for: reasons.append('final_continuous_healthy_window_too_short')
    if any(error['phase'] == 'close' for error in result['errors']): reasons.append('transport_close_failed')
    result['summary'] = dict(passed=passed, duration_seconds=round(ended - started, 3),
                             hello_verified=hello_verified,
                             initial_stop_acknowledged=initial_stop_confirmed,
                             final_stop_acknowledged=final_stop_confirmed,
                             valid_state_count=count, record_count=len(records),
                             settled_record_count=len(settled), unsettled_record_count=len(records) - len(settled),
                             unhealthy_after_settle_count=len(bad_settled),
                             final_continuous_healthy_seconds=round(final_healthy, 3),
                             failure_reasons=reasons,
                             last_state=valid[-1]['state'] if valid else None,
                             last_record_reasons=records[-1].get('reasons', []) if records else [],
                             observed_online_ids=ids_for_mask(online_union),
                             observed_fault_ids=ids_for_mask(fault_union),
                             observed_torque_ids=ids_for_mask(torque_union),
                             observed_unique_servo_state_hz=observed_rate(sample_advance_times),
                             observed_unique_imu_timestamp_hz=observed_rate(imu_advance_times),
                             servo_timestamp_step_us=distribution(sample_intervals_us),
                             imu_timestamp_step_us=distribution(imu_intervals_us),
                             sample_age_ms=distribution([r['sample_age_ms'] for r in valid]),
                             imu_age_ms=distribution([r['imu_age_ms'] for r in valid]),
                             response_ms=distribution([r['response_ms'] for r in valid]),
                             acceleration_norm_ms2=distribution([r['acceleration_norm_ms2'] for r in valid]),
                             notes=['STOP releases torque; ACK is not physical proof for unreachable servos.',
                                    'Online masks and feedback order are firmware reports, not a duplicate-ID scan.',
                                    'Observed rates depend on polling and do not certify the 100 Hz scheduling target.',
                                    'Passing this probe does not calibrate axes, joints or enable motion.'])
    return result


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.motion-probe-', suffix='.json', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--settle', type=float, default=2.)
    parser.add_argument('--healthy-for', type=float, default=3.)
    parser.add_argument('--poll-period', type=float, default=.01)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    numbers = (args.seconds, args.settle, args.healthy_for, args.poll_period)
    if (not all(math.isfinite(v) for v in numbers) or not 0 < args.seconds <= 60 or
            args.settle < 0 or args.healthy_for <= 0 or args.seconds < args.settle + args.healthy_for or
            not .005 <= args.poll_period <= .05):
        parser.error('需要 0<seconds≤60、settle≥0、healthy-for>0、seconds≥settle+healthy-for、poll-period=.005..05')
    report = run_probe(SerialTransport(args.port), seconds=args.seconds, settle=args.settle,
                       healthy_for=args.healthy_for, poll_period=args.poll_period)
    write_report(args.output, report)
    print(json.dumps(dict(output=str(args.output), **report['summary']), ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report['summary']['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
