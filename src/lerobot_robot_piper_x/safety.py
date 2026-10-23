"""Limits apply to targets, before conversion to the SDK's integer units."""

import math

from .protocol import GRIPPER_FACTOR, JOINT_FACTOR, KEYS

GRIPPER_FEEDBACK_TOLERANCE_M = 0.01
JOINT_FEEDBACK_TOLERANCE_RAD = math.radians(0.5)


def validate_initial_pose(present, config, *, feedback=True):
    """Report every out-of-range axis before changing modes or enabling motors."""
    problems = []
    for i, key in enumerate(KEYS):
        value = float(present[key])
        if not math.isfinite(value):
            problems.append(f"{key}: non-finite feedback {value}")
            continue
        lo, hi = config.joint_limits_rad[i] if i < 6 else (0.0, config.gripper_max_m)
        tolerance = (JOINT_FEEDBACK_TOLERANCE_RAD if i < 6 else GRIPPER_FEEDBACK_TOLERANCE_M) if feedback else 0.0
        if not lo - tolerance <= value <= hi + tolerance:
            scale = 180 / math.pi if i < 6 else 1000
            unit = "deg" if i < 6 else "mm"
            problems.append(
                f"{key}: measured {value * scale:.3f} {unit}, "
                f"allowed [{lo * scale:.3f}, {hi * scale:.3f}] {unit}"
            )
    if problems:
        raise ValueError(
            "Cannot enable Piper-X: initial pose is outside the configured limits:\n  "
            + "\n  ".join(problems)
            + "\nVerify the physical pose and hardware zero calibration against the arm's stored limits. "
            "Do not widen limits or reset zeros merely to bypass this check."
        )


def limit_action(action, present, config):
    if set(action) != set(KEYS):
        raise ValueError(f"Action must contain exactly {KEYS}")
    result = {}
    for i, key in enumerate(KEYS):
        value, current = float(action[key]), float(present[key])
        if not math.isfinite(value) or not math.isfinite(current):
            raise ValueError(f"Non-finite position for {key}")
        if i < 6:
            lo, hi = config.joint_limits_rad[i]
            delta, factor = config.max_joint_step_rad, JOINT_FACTOR
            tolerance = JOINT_FEEDBACK_TOLERANCE_RAD
        else:
            lo, hi = 0, config.gripper_max_m
            delta, factor = config.max_gripper_step_m, GRIPPER_FACTOR
            tolerance = GRIPPER_FEEDBACK_TOLERANCE_M
        if not lo - tolerance <= current <= hi + tolerance:
            raise ValueError(f"{key} feedback is outside the configured limits: {current}")
        # For endpoint feedback offsets, permit recovery to the nearest
        # representable in-range position even when beyond the normal step.
        if current < lo:
            delta = max(delta, math.ceil(lo * factor) / factor - current + 1e-12)
        elif current > hi:
            delta = max(delta, current - math.floor(hi * factor) / factor + 1e-12)
        # Intersect absolute and relative limits; targets stay inside the envelope.
        low, high = max(lo, current - delta), min(hi, current + delta)
        raw_low, raw_high = math.ceil(low * factor), math.floor(high * factor)
        if raw_low > raw_high:
            raise ValueError(f"{key} feedback is outside the configured limits: {current}")
        raw = min(max(round(value * factor), raw_low), raw_high)
        result[key] = raw / factor
    return result
