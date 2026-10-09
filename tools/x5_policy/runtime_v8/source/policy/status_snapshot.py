"""Detach bounded display inputs; formatting must not hold the control lock.

This view is private to status rendering. Control requests always use the live
controller and its existing gates. It cannot access the hardware transport.
"""
import copy
import threading


def clone_display_value(value):
    """Copy JSON-shaped display data without deepcopy's per-scalar memo work."""
    kind = type(value)
    if kind is dict:
        return {key: clone_display_value(item) for key, item in value.items()}
    if kind is list:
        return [clone_display_value(item) for item in value]
    if kind is tuple:
        return tuple(clone_display_value(item) for item in value)
    if value is None or kind in (bool, int, float, str, bytes):
        return value
    # Preserve the original behavior for any future non-JSON diagnostic type.
    return copy.deepcopy(value)


def capture_display_view(controller):
    """Caller holds controller._lock; returned view is never published itself."""
    view = object.__new__(type(controller))
    view.__dict__ = controller.__dict__.copy()
    view._lock = threading.RLock()
    view._transport = None
    if hasattr(controller, "_tracking"): view._tracking = copy.deepcopy(controller._tracking)
    if hasattr(controller, "_forward"): view._forward = copy.deepcopy(controller._forward)
    # State/config contain only bounded JSON-shaped values. The sensor filter
    # produces a new State each sample, never mutates a published old State.
    view._state = clone_display_value(controller._state)
    cfg = controller._config
    view._config = dict(cfg, joints=[dict(row) for row in cfg['joints']],
                        imu=clone_display_value(cfg['imu']))
    for name in ('_command', '_previous_action', '_smooth_action'):
        value = getattr(controller, name, None)
        if value is not None:
            setattr(view, name, value.copy())
    for name in ('_state_receive_times', '_state_times', '_target_times'):
        setattr(view, name, tuple(getattr(controller, name)))
    view._timing_records = {name: tuple(rows)
                            for name, rows in controller._timing_records.items()}
    view._last_action = copy.deepcopy(controller._last_action)
    view._inference_attempts = dict(getattr(controller, '_inference_attempts', {}))
    collection = controller._imu_collection
    if collection is not None:
        view._imu_collection = dict(collection, samples=tuple(collection['samples']))
    # Other diagnostic roots, profile arrays, head references and filtered
    # State objects are replace-only in the pinned controller sources.
    return view
