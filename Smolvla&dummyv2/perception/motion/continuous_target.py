"""Integrate Servo velocities with joint limits and an optional feedback lead cap."""
import math


try:
    from .joint_contract import HARDWARE_LIMITS
except ImportError:  # Executed by the standalone ROS gateway.
    from joint_contract import HARDWARE_LIMITS


class ContinuousTarget:
    PERIOD = .025
    MAX_LEAD_DEG = None

    def __init__(self):
        self.reset()

    def reset(self):
        self.target = None
        self.at = None
        self.limited = False
        self.lead_limited = False
        self.joint_limited = False
        self.joint_limit_axes = []

    def advance(self, velocities, actual, now):
        if (len(velocities) != 6 or len(actual) != 6
                or not all(math.isfinite(v) for v in [*velocities, *actual, now])):
            raise ValueError('six_finite_axes_required')
        if self.at is not None and (now <= self.at or now - self.at > .1):
            self.reset()  # Never integrate a scheduling outage as catch-up motion.
            return None
        dt = self.PERIOD if self.at is None else min(now - self.at, self.PERIOD * 2)
        base = ([min(max(a, low), high) for a, (low, high) in zip(actual, HARDWARE_LIMITS)]
                if self.target is None else self.target)
        delta = [v * dt for v in velocities]
        joint_scale = lead_scale = 1.0
        constraints = []
        for axis, (q, a, d, (low, high)) in enumerate(zip(base, actual, delta, HARDWARE_LIMITS), 1):
            # Encoder noise may be just outside a bound; emitted goals may not.
            lower, upper = low, high
            if d > 0:
                ratio = max(0., (upper-q)/d)
                joint_scale = min(joint_scale, ratio)
                constraints.append((ratio, {'joint': f'J{axis}', 'bound': 'upper', 'limit_deg': high}))
                if self.MAX_LEAD_DEG is not None:
                    lead_scale = min(lead_scale, max(0., (a+self.MAX_LEAD_DEG-q)/d))
            elif d < 0:
                ratio = max(0., (lower-q)/d)
                joint_scale = min(joint_scale, ratio)
                constraints.append((ratio, {'joint': f'J{axis}', 'bound': 'lower', 'limit_deg': low}))
                if self.MAX_LEAD_DEG is not None:
                    lead_scale = min(lead_scale, max(0., (a-self.MAX_LEAD_DEG-q)/d))
        # A changed feedback sample can put an existing target outside the
        # envelope. Allow recovery but never snap the target or extend that gap.
        scale = min(joint_scale, lead_scale)
        # One common scale preserves the direction across axes. No wind-up:
        # rejected increments are discarded rather than stored for later.
        self.target = [min(max(q + scale*d, low), high)
                       for q, d, (low, high) in zip(base, delta, HARDWARE_LIMITS)]
        self.at = now
        self.limited = scale < .999999
        self.lead_limited = lead_scale < .999999
        self.joint_limited = joint_scale < .999999
        self.joint_limit_axes = [info for ratio, info in constraints
                                 if self.joint_limited and abs(ratio-joint_scale) < 1e-8]
        return list(self.target)
