"""Extra offline checks for the existing fixed-head motion contract.

This module performs no device I/O and grants no runtime authorization. The
base simulation screen does not check the native fixed-head deviation gate;
this separate assessment can also rescore frozen, hash-pinned result files.
"""
from __future__ import annotations

import math
from audit_policy_sim import assess_result


def assess_fixed_head_runtime_contract(result):
    base = assess_result(result)
    failures = list(base['failures'])
    if result.get('executor') != 'fixed-head':
        failures.append('fixed_head_executor_required')
    deviations = result.get('head_max_deviation_deg')
    valid = (type(deviations) is list and len(deviations) == 4
        and all(type(value) in (int, float) and math.isfinite(value) and value >= 0
                for value in deviations))
    if not valid:
        failures.append('missing_or_invalid_actual_head_deviation')
    # Captured head references originate from integer encoder ticks. Native
    # feedback is quantized too: its 2deg gate trips at 23 ticks, so a true
    # displacement near 1.978deg can already round to a rejected sample.
    # At exact half-ticks choose the rejecting side conservatively.
    tick_deg = 360. / 4095
    quantized = ([math.floor(value / tick_deg + .5 + 1e-12) * tick_deg
                  for value in deviations] if valid else None)
    if valid and max(quantized) >= 2.:
        failures.append('encoder_head_deviation_reached_2_deg')
    return dict(base_assessment=base, offline_contract_screen_passed=not failures,
        failures=failures, head_ids=[30, 31, 32, 33], head_limit_deg=2.,
        actual_head_max_deviation_deg=deviations if valid else None,
        simulated_encoder_head_max_deviation_deg=quantized,
        encoder_tick_deg=tick_deg, head_gate_trip_ticks=math.ceil(2. / tick_deg),
        quantization_assumption='Integer captured reference; nearest tick, rejecting half-tick ties.',
        physical_standing_verified=False, physical_walking_verified=False,
        deployment_authorized=False,
        interpretation='Offline range, tilt, foot-motion and fixed-head checks only; '
            'current calibration, real freshness/ownership, torque and physical acceptance remain required.')
