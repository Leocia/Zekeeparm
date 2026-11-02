"""Calibrated hue checks inside object masks; independent of YOLO class scores."""
from __future__ import annotations

import cv2
import numpy as np


COLOR_NAMES = ("red", "purple", "blue", "green", "yellow")
BLOCK_NAMES = {"block", "gray block"} | {f"{color} block" for color in COLOR_NAMES}
MIN_PIXELS = 32


def hue_distance(hue, center):
    # OpenCV uint8 hue is circular, including red across 179/0.
    return np.abs((np.asarray(hue, dtype=float) - center + 90) % 180 - 90)


def colored_pixels(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv, (hsv[:, :, 1] >= 60) & (hsv[:, :, 2] >= 40)


def validate_color_calibration(calibration):
    if calibration.get("version") != 1 or not calibration.get("colors"):
        raise ValueError("Invalid block color calibration version or empty colors")
    profiles = calibration["colors"]
    for name, profile in profiles.items():
        center, radius = float(profile["hue_center"]), float(profile["hue_radius"])
        if name not in COLOR_NAMES or not (0 <= center < 180 and 0 < radius <= 15):
            raise ValueError(f"Invalid block color profile: {name}")
        profile.update(hue_center=center, hue_radius=radius)
    names = list(profiles)
    for index, first in enumerate(names):
        for second in names[index + 1:]:
            a, b = profiles[first], profiles[second]
            if hue_distance(a["hue_center"], b["hue_center"]) <= a["hue_radius"] + b["hue_radius"]:
                raise ValueError(f"Overlapping color samples: {first}, {second}; resample under stable lighting")
    return calibration


def fit_color_calibration(samples):
    """Fit human-labeled BGR patches, rejecting mixed or overlapping hues."""
    profiles = {}
    for name, patches in samples.items():
        if name not in COLOR_NAMES or not patches:
            raise ValueError(f"Unsupported or empty color sample: {name}")
        hues = []
        for patch in patches:
            hsv, valid = colored_pixels(patch)
            if valid.sum() < MIN_PIXELS or valid.mean() < .5:
                raise ValueError(f"Insufficient saturated, lit pixels for {name}")
            hues.append(hsv[:, :, 0][valid])
        hues = np.concatenate(hues).astype(float)
        mode = int(np.bincount(hues.astype(int), minlength=180).argmax())
        offsets = (hues - mode + 90) % 180 - 90
        center = float((mode + np.median(offsets)) % 180)
        radius = max(4.0, float(np.percentile(hue_distance(hues, center), 99)) + 2.0)
        profiles[name] = {"hue_center": center, "hue_radius": radius}
    return validate_color_calibration({"version": 1, "colors": profiles})


def classify_block_color(image, mask, calibration):
    """Return a calibrated color, or None for dark, neutral or ambiguous masks."""
    if mask.shape != image.shape[:2]:
        raise ValueError("Block color mask does not match image dimensions")
    mask = cv2.erode((mask > 0).astype(np.uint8), np.ones((3, 3), np.uint8),
                     borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    rows, cols = np.nonzero(mask)
    if len(rows) < MIN_PIXELS:
        return None
    y1, y2, x1, x2 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    hsv, valid = colored_pixels(image[y1:y2, x1:x2])
    interior = mask[y1:y2, x1:x2]
    valid &= interior
    count = int(valid.sum())
    if count < MIN_PIXELS or count < .60 * len(rows):
        return None
    hues = hsv[:, :, 0][valid]
    votes = sorted(((float(np.mean(hue_distance(hues, p["hue_center"]) <= p["hue_radius"])), name)
                    for name, p in calibration["colors"].items()), reverse=True)
    best, name = votes[0]
    runner_up = votes[1][0] if len(votes) > 1 else 0.0
    return name if best >= .75 and best * count >= .60 * len(rows) and best - runner_up >= .20 else None
