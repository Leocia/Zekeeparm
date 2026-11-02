"""Bound continuous learned gripper angles before the execution guards."""

import math


def gripper_target(value, settings, limits):
    value = float(value)
    low, high = limits
    margin = settings["prediction_margin_deg"]
    if not math.isfinite(value) or not low - margin <= value <= high + margin:
        raise RuntimeError(f"Gripper angle prediction outside bounded range: {value:.3f} deg")
    return min(high, max(low, value))
