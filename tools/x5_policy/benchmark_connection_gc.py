"""Hardware-denied comparison of long-lived heap scans and new cyclic garbage.

This is a host microbenchmark, not an X5 control-frequency or walking test.
The retained synthetic heap makes the scan reduction measurable on a laptop.
"""
import argparse
from collections import deque
import gc
import importlib.util
import json
import os
from pathlib import Path
import statistics
import sys
import time
import tracemalloc
import weakref


def deny_hardware(event, args):
    if event == 'open' and args and isinstance(args[0], (str, bytes)):
        path = os.path.realpath(os.fsdecode(args[0]))
        if path.startswith('/dev/') and path != '/dev/null':
            raise PermissionError('GC benchmark cannot open a device')
    if event in ('socket.connect', 'socket.bind'):
        raise PermissionError('GC benchmark cannot access the network')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path(__file__).with_name('native_runtime.py'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.addaudithook(deny_hardware)
    spec = importlib.util.spec_from_file_location('gc_benchmark_runtime', args.runtime)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    enabled, thresholds = gc.isenabled(), gc.get_threshold()
    if not enabled or gc.get_freeze_count():
        raise ValueError('Run in a fresh interpreter with automatic GC enabled')
    # Nested containers represent retained metadata. Their absolute size and
    # timings are deliberately not attributed to the captured X5 heap.
    retained = [{'index': i, 'items': [i, {'value': i}]} for i in range(100000)]
    gc.collect()
    loaded_count = len(gc.get_objects())
    baseline = []
    for _ in range(7):
        began = time.perf_counter()
        gc.collect(2)
        baseline.append((time.perf_counter() - began) * 1000)
    window = runtime.ConnectionGCWindow()
    window.prepare()
    try:
        frozen = []
        for _ in range(7):
            began = time.perf_counter()
            gc.collect(2)
            frozen.append((time.perf_counter() - began) * 1000)
        class Cell:
            pass
        survivors = []
        history = deque(maxlen=150)
        tracemalloc.start()
        blocks = []
        for block in range(10):
            references = []
            for index in range(5000):
                cell = Cell()
                cell.cycle = cell
                references.append(weakref.ref(cell))
                history.append({'frame': index, 'rows': [{'id': i, 'value': index} for i in range(15)]})
                del cell
            reclaimed = gc.collect(2)
            alive = sum(reference() is not None for reference in references)
            assert alive == 0, 'New cycles were retained during the connection'
            del references
            current, peak = tracemalloc.get_traced_memory()
            blocks.append(dict(block=block, traced_current_bytes=current,
                               traced_peak_bytes=peak, final_collection_reclaimed=reclaimed))
            survivors.append(alive)
        tracemalloc.stop()
        assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
        # The bounded history should settle after the first full block.
        growth = blocks[-1]['traced_current_bytes'] - blocks[1]['traced_current_bytes']
        assert growth < 1024 * 1024, ('Unbounded traced memory growth', growth)
        result = dict(scope='laptop_synthetic_heap_microbenchmark_not_X5_acceptance',
            python=sys.version, hardware_access=False, loaded_tracked_objects=loaded_count,
            retained_records=len(retained), window=window.snapshot(),
            baseline_full_gc_ms=baseline, frozen_full_gc_ms=frozen,
            baseline_median_ms=statistics.median(baseline),
            frozen_median_ms=statistics.median(frozen),
            new_cycles_created=50000, new_cycles_left=survivors,
            memory_blocks=blocks, traced_growth_after_warmup_bytes=growth,
            automatic_gc_enabled=gc.isenabled(), thresholds_unchanged=gc.get_threshold() == thresholds)
    finally:
        window.finish(serial_closed=True)
    assert gc.get_freeze_count() == 0
    result.update(unfrozen_after_close=True, passed=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('passed', 'loaded_tracked_objects',
        'baseline_median_ms', 'frozen_median_ms', 'new_cycles_created',
        'traced_growth_after_warmup_bytes', 'unfrozen_after_close')}))


if __name__ == '__main__':
    main()
