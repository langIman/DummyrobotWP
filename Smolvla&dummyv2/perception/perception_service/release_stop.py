"""One-shot stop destination from verified feedback; no I/O or speed changes.

Deceleration is an initial host-side estimate, NOT a measured driver setting.
Only normal operator release uses this prediction; fault stops bypass it.
"""
import math

DECELERATION_DEG_S2 = 100.0
MAX_EXTENSION_DEG = 5.0
MIN_SPEED_DEG_S = 1.0
MAX_ESTIMATED_SPEED_DEG_S = 90.0


def release_destination(state, history, limits, now_ns):
    positions = list(state['positions'])
    result = {
        'positions': positions, 'offsets_deg': [0.0] * 6,
        'velocities_deg_s': [0.0] * 6, 'applied': False,
        'reason': 'insufficient_feedback', 'estimated': True,
        'deceleration_deg_s2': DECELERATION_DEG_S2,
        'max_extension_deg': MAX_EXTENSION_DEG,
        'sampled_at_ms': state.get('sampled_at_ms'),
    }
    stamp = state.get('monotonic_ns')
    if type(stamp) is not int or state.get('verification_retries', 0):
        return result
    age = (now_ns - stamp) / 1e9
    if not 0 <= age <= .15:
        result['reason'] = 'feedback_too_old_for_prediction'
        return result
    # Require two independent recent intervals. Cached or rapid rereads do not
    # establish velocity, and observations before a gap cannot bridge that gap.
    samples = [s for s in history if type(s.get('monotonic_ns')) is int
               and s['monotonic_ns'] < stamp and not s.get('verification_retries', 0)]
    previous = next((s for s in reversed(samples)
                     if .06 <= (stamp - s['monotonic_ns']) / 1e9 <= .35), None)
    if previous is None:
        return result
    older = next((s for s in reversed(samples)
                  if .06 <= (previous['monotonic_ns'] - s['monotonic_ns']) / 1e9 <= .35), None)
    if older is None:
        return result
    dt = (stamp - previous['monotonic_ns']) / 1e9
    old_dt = (previous['monotonic_ns'] - older['monotonic_ns']) / 1e9
    velocities = []
    for p, last, old in zip(positions, previous['positions'], older['positions']):
        recent_v, old_v = (p - last) / dt, (last - old) / old_dt
        stable = (math.isfinite(recent_v) and math.isfinite(old_v)
                  and recent_v * old_v > 0
                  and max(abs(recent_v), abs(old_v)) <= MAX_ESTIMATED_SPEED_DEG_S
                  and min(abs(recent_v), abs(old_v)) >= MIN_SPEED_DEG_S)
        # Do not extrapolate reversal/noise; during acceleration use the lower
        # observed speed to avoid an excessive stop extension from one sample.
        velocities.append(math.copysign(min(abs(recent_v), abs(old_v)), recent_v) if stable else 0.0)
    offsets = [v * age + v * abs(v) / (2 * DECELERATION_DEG_S2) for v in velocities]
    scale = min(1.0, MAX_EXTENSION_DEG / max(max(map(abs, offsets)), 1e-9))
    targets = [min(max(p + d * scale, min(low, p)), max(high, p))
               for p, d, (low, high) in zip(positions, offsets, limits)]
    actual_offsets = [q - p for q, p in zip(targets, positions)]
    applied = max(map(abs, actual_offsets)) >= .02
    result.update(positions=targets if applied else positions,
                  offsets_deg=actual_offsets if applied else [0.0] * 6,
                  velocities_deg_s=velocities, applied=applied,
                  reason='predicted_stop' if applied else 'stationary_unstable_or_at_limit')
    return result
