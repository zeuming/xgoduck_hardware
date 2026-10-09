"""Separate HOLD and model episodes in retained traces, without hardware I/O."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

WINDOW_END_REASON = '零速试验30秒窗口结束，已自动停止，请保持支撑'


def summarize(trace):
    frames = trace['frames']
    if not isinstance(frames, list) or not frames:
        raise ValueError('Missing trial frames')
    complete = (trace.get('truncated') is False
        and trace.get('total_frames') == trace.get('retained_frames') == len(frames))
    groups = {}
    for frame in frames:
        key = (frame['mode'], frame['generation'])
        groups.setdefault(key, []).append(frame)
    terminal = trace.get('terminal') or {}
    for frame in frames + ([terminal] if terminal else []):
        gravity = frame.get('gravity')
        if (not isinstance(gravity, list) or len(gravity) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in gravity)
                or not math.isclose(math.sqrt(sum(v*v for v in gravity)), 1., abs_tol=1e-4)):
            raise ValueError('Trial gravity must be a finite unit vector')
        if any(type(frame.get(key)) not in (int, float) or not math.isfinite(frame[key])
               for key in ('monotonic', 'age_s')):
            raise ValueError('Trial timestamps/ages must be finite')
    rows = []
    for (mode, generation), values in groups.items():
        starts = [f for f in values if f['event'] == 'start']
        targets = [f for f in values if f['event'] == 'target-attempt']
        successes = [f for f in targets if f.get('request_error') is None
            and f.get('attempted_target_ticks') is not None
            and f.get('request_ended_monotonic') is not None]
        same_terminal = terminal.get('mode') == mode and terminal.get('generation') == generation
        began = starts[0]['monotonic'] if len(starts) == 1 else values[0]['monotonic']
        ended = terminal['monotonic'] if same_terminal else values[-1]['monotonic']
        observed = values + ([terminal] if same_terminal else [])
        tilt = [math.degrees(math.acos(max(-1., min(1., -f['gravity'][2]))))
            for f in observed]
        errors = [f['request_error'] for f in targets if f.get('request_error') is not None]
        zero = all(f.get('command') == [0., 0., 0.] for f in observed)
        gaps = [(b['monotonic']-a['monotonic'])*1000 for a,b in zip(successes,successes[1:])]
        silence = ([max(0., successes[0]['monotonic']-began)*1000,
            max(0., ended-successes[-1]['request_ended_monotonic'])*1000] + gaps
            if successes else [math.inf])
        failures = []
        if mode != 'walk': failures.append('not_model_walk_mode')
        if not complete: failures.append('trace_incomplete')
        if len(starts) != 1: failures.append('missing_or_duplicate_model_start')
        if not same_terminal: failures.append('no_matching_terminal')
        if ended-began < 29.9: failures.append('model_episode_shorter_than_30s')
        if not successes: failures.append('no_successful_TARGET')
        # Evidence coverage only; this does not modify a runtime gate. Use
        # the existing 400ms lease window to reject unobserved long intervals.
        if max(silence) >= 400.: failures.append('TARGET_coverage_gap_reached_400ms')
        if errors: failures.append('TARGET_request_error')
        if not zero: failures.append('nonzero_command_present')
        scheduled_end = (same_terminal and mode == 'walk' and ended-began >= 29.9
            and terminal.get('stop_reason') == WINDOW_END_REASON)
        if same_terminal and terminal.get('stop_reason') is not None and not scheduled_end:
            failures.append('protective_stop')
        if max(tilt) >= 15.: failures.append('tilt_reached_15deg')
        ages = [f['age_s']*1000 for f in observed]
        if any(not math.isfinite(age) or age < 0 or age >= 100 for age in ages):
            failures.append('feedback_not_fresh')
        rows.append(dict(mode=mode, generation=generation, duration_s=ended-began,
            start_monotonic=began, end_monotonic=ended,
            successful_TARGETs=len(successes), TARGET_errors=errors,
            max_tilt_deg=max(tilt), max_feedback_age_ms=max(ages),
            max_TARGET_gap_ms=max(gaps) if gaps else None,
            max_TARGET_silence_ms=max(silence) if successes else None,
            zero_command_only=zero, matching_terminal=same_terminal,
            stop_reason=terminal.get('stop_reason') if same_terminal else None,
            scheduled_window_end=scheduled_end,
            zero_model_30s_software_screen_passed=not failures, failures=failures))
    return dict(trace_sequence=trace.get('sequence'), trace_complete=complete,
        mode_segments=rows, physical_independent_support_inferred=False,
        walking_qualified=False,
        scope='Retained software evidence only. A separate operator report must confirm independent support; HOLD duration never qualifies model balance.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = [dict(path=str(path.resolve()), **summarize(json.loads(path.read_text())))
        for path in args.traces]
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
