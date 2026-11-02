"""Calibrated motor/ROS conversion and bounded absolute joint targets."""

import math

from .record import ROS_ARM_JOINTS


def motor_degrees_to_ros(values, calibrations):
    if len(values) != len(calibrations):
        raise ValueError("Motor target and calibration dimensions differ")
    result = []
    for value, calibration in zip(values, calibrations):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("Non-finite motor target")
        motor = math.radians(value)
        result.append(
            (motor - calibration["encoder_zero"])
            * calibration["direction"]
            * calibration["gear_ratio"]
            + calibration["ros_zero"]
        )
    return result


def guard_action(current, proposed, limits, max_step_deg, *, gripper_lower_tolerance_deg=0.0,
                 gripper_max_step_deg=None):
    if len(current) != 7 or len(proposed) != 7 or len(limits) != 7:
        raise ValueError("Action guard requires seven joints")
    if not math.isfinite(max_step_deg) or max_step_deg <= 0:
        raise ValueError("max_step_deg must be positive and finite")
    if not math.isfinite(gripper_lower_tolerance_deg) or gripper_lower_tolerance_deg < 0:
        raise ValueError("gripper_lower_tolerance_deg must be nonnegative and finite")
    if gripper_max_step_deg is not None and (not math.isfinite(gripper_max_step_deg) or gripper_max_step_deg <= 0):
        raise ValueError("gripper_max_step_deg must be positive and finite")
    guarded = []
    for index, (state, target, bounds) in enumerate(zip(current, proposed, limits)):
        state, target = float(state), float(target)
        low, high = map(float, bounds)
        if not all(math.isfinite(value) for value in (state, target, low, high)):
            raise ValueError("Action guard received a non-finite value")
        name = ROS_ARM_JOINTS[index] if index < 6 else "gripper"
        state_low = low - gripper_lower_tolerance_deg if index == 6 else low
        if low >= high or not state_low <= state <= high:
            raise RuntimeError(
                f"Current joint {index} ({name}) is outside guarded limits: "
                f"{state:.3f} deg not in [{state_low:.3f}, {high:.3f}] deg"
            )
        if index == 6 and state_low <= target < low:
            target = low
        if not low <= target <= high:
            raise RuntimeError(
                f"Policy joint {index} ({name}) target is outside guarded limits: "
                f"{target:.3f} deg not in [{low:.3f}, {high:.3f}] deg"
            )
        # Tolerated closed-stop feedback is a control reference at the lower bound, never a negative command.
        state = max(low, state)
        step = gripper_max_step_deg if index == 6 and gripper_max_step_deg is not None else max_step_deg
        delta = min(max(target - state, -step), step)
        guarded.append(state + delta)
    return guarded
