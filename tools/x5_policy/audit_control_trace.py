"""Summarize saved host/MCU motion evidence without opening any device.

The MCU goal is a successful-transmit cache, NOT a motor register readback.
Samples are a complete servo scan, NOT simultaneous per-joint measurements.
These records cannot identify actuator torque constants or the body's COM.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    if not values:
        return dict(count=0, mean=None, p95=None, maximum=None)
    values = sorted(values)
    p = .95*(len(values)-1)
    lo = int(p)
    return dict(count=len(values), mean=statistics.mean(values),
                p95=values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*(p-lo),
                maximum=values[-1])


def analyze(trace):
    ids = trace['motor_ids']
    joints = trace['calibration']['joints']
    if len(ids) != 15 or len(set(ids)) != 15 or [j['id'] for j in joints] != ids:
        raise ValueError('Trace must carry its own ordered 15-joint calibration')
    rows, skipped, modes = [], {}, {}
    goal_history = deque(maxlen=32)
    for frame in trace['frames']:
        if frame['event'] != 'target-attempt' or frame.get('request_error') is not None:
            continue
        mode = frame['mode']
        record = modes.setdefault(mode, dict(target_times=[], feedback_age_ms=[]))
        record['target_times'].append(frame['request_ended_monotonic'])
        if frame.get('age_s') is not None:
            record['feedback_age_ms'].append(frame['age_s']*1000)
        goal_us = frame.get('MCU_written_goal_us')
        actual, goals = frame.get('actual_raw_ticks'), frame.get('MCU_written_goal_ticks')
        if not goal_us or len(actual or []) != 15 or len(goals or []) != 15:
            skipped['missing_goal_cache'] = skipped.get('missing_goal_cache', 0)+1
            continue
        goal_history.append((goal_us, goals))
        # The STATE reply can contain a TX cache written AFTER this servo scan.
        # Use the latest recorded prior TX, never compare older feedback to a
        # future goal. Intermediate MCU writes between host polls are unknown.
        earlier = [(stamp, values) for stamp, values in goal_history
                   if ((frame['sample_us']-stamp)&0xffffffff) < 100000]
        if not earlier:
            skipped['no_prior_goal_within_100ms'] = skipped.get('no_prior_goal_within_100ms', 0)+1
            continue
        goal_us, goals = min(earlier, key=lambda row:(frame['sample_us']-row[0])&0xffffffff)
        lag = (frame['sample_us']-goal_us) & 0xffffffff
        if any(not 0 <= value <= 4095 for value in actual+goals):
            raise ValueError('Invalid encoder or cached goal tick')
        for i, joint in enumerate(joints):
            convert = lambda tick: (tick-joint['zero'])*360/4095*joint['sign']
            a, g = convert(actual[i]), convert(goals[i])
            rows.append(dict(mode=mode, host_at=frame['monotonic'],
                relative_s=frame['monotonic']-record['target_times'][0],
                sample_us=frame['sample_us'], goal_write_us=goal_us,
                goal_cache_age_at_scan_end_ms=lag/1000, id=ids[i],
                actual_angle_deg=a, cached_goal_deg=g, error_deg=a-g,
                forward_command_m_s=frame['command'][0]))
    summaries = {}
    for mode, record in modes.items():
        times = record['target_times']
        gaps = [(b-a)*1000 for a,b in zip(times,times[1:]) if b>a]
        by_joint = {}
        for sid in ids:
            # Exclude the three-second HOLD interpolation plus one second.
            # This is a diagnostic window, never proof of load/COM identity.
            usable = [r for r in rows if r['id']==sid and r['mode']==mode and r['relative_s']>=4]
            unique = {r['sample_us']:r for r in usable}.values()
            errors = [r['error_deg'] for r in unique]
            by_joint[str(sid)] = dict(mean_signed_error_deg=statistics.mean(errors) if errors else None,
                absolute_cached_goal_error_deg=distribution([abs(e) for e in errors]))
        summaries[mode] = dict(successful_targets=len(times),
            target_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
            target_gap_ms=distribution(gaps), feedback_age_ms=distribution(record['feedback_age_ms']),
            joint_after_first_4s=by_joint)
    return dict(kind='offline_motion_evidence_audit', modes=summaries,
        skipped_frames=skipped, stop_reason=trace.get('reason'),
        calibration_sha256=hashlib.sha256(json.dumps(trace['calibration'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        physical_walking_verified=False, suitable_for_actuator_identification=False,
        limits=['Goal cache confirms successful UART transmission, not motor acceptance.',
            'Uses the latest recorded prior MCU goal; unobserved intermediate writes are possible.',
            'Scan-end timestamp cannot resolve each servo sample time within the scan.',
            'Operator support and contact loads are unknown; errors do not identify the cause.',
            'BAM fitting requires an identified testbench load, geometry, gains and voltage.']), rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trace',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();raw=args.trace.read_bytes();report,rows=analyze(json.loads(raw))
    report['source_sha256']=hashlib.sha256(raw).hexdigest()
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if rows:
        with (args.output/'goal-feedback.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(dict(rows=len(rows),modes=list(report['modes']),device_access=False)))


if __name__=='__main__': main()
