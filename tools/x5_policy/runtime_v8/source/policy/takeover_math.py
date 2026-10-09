"""Bounded, time-based interpolation from a held stance to zero-policy targets."""
import math

TAKEOVER_SECONDS = 2.0


def takeover_weight(elapsed):
    if isinstance(elapsed, bool) or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed):
        raise ValueError('零速接管计时无效')
    fraction = max(0., min(1., elapsed / TAKEOVER_SECONDS))
    return fraction * fraction * (3. - 2. * fraction)


def blend_stance(anchor, policy, elapsed):
    if len(anchor) != 15 or len(policy) != 15:
        raise ValueError('零速接管需要完整15关节目标')
    if not all(math.isfinite(value) for value in (*anchor, *policy)):
        raise ValueError('零速接管关节目标无效')
    weight = takeover_weight(elapsed)
    if weight == 0.:
        return list(anchor)
    if weight == 1.:
        return list(policy)
    return [held if held == target else (1. - weight) * held + weight * target
            for held, target in zip(anchor, policy)]
