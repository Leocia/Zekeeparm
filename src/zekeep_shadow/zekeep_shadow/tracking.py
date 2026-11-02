from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import cv2


@dataclass
class Detection:
    name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    mask: np.ndarray
    position_base: Optional[np.ndarray] = None


def detect_red_blocks(image: np.ndarray, min_area: int = 180) -> list[Detection]:
    """Find red connected regions in an aligned BGR frame."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 140, 65), (6, 255, 255))
    mask |= cv2.inRange(hsv, (174, 140, 65), (179, 255, 255))
    blue, green, red = cv2.split(image)
    mask[(red.astype(np.uint16) < 2 * green) |
         (red.astype(np.uint16) < 2 * blue)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if not 0.4 <= w / h <= 2.5 or area / (w * h) < 0.5 or area / hull_area < 0.8:
            continue
        component = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(component, [contour], -1, 1, cv2.FILLED)
        result.append(Detection("red block", 1.0, (x, y, x + w, y + h), component.astype(bool)))
    return result


def stabilize_camera_error(error: np.ndarray, previous: Optional[np.ndarray],
                           moving: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Smooth RGB-D error and hold each axis through sensor noise."""
    filtered = error.copy() if previous is None else 0.35 * error + 0.65 * previous
    start = np.array([0.01, 0.01, 0.025])
    stop = np.array([0.005, 0.005, 0.012])
    moving = np.where(moving, np.abs(filtered) > stop, np.abs(filtered) > start)
    command = filtered.copy()
    command[~moving] = 0.0
    return command, filtered, moving


def track_base_target(point: np.ndarray, anchor: Optional[np.ndarray], deadband_m: float = 0.015) -> np.ndarray:
    """Keep a stationary object's world target despite view-dependent surface points."""
    point = np.asarray(point, dtype=float)
    if anchor is None:
        return point.copy()
    delta = point - anchor
    distance = np.linalg.norm(delta)
    return anchor + delta * (1.0 - deadband_m / distance) if distance > deadband_m else anchor


class TargetLock:
    """Keep a clicked detection by image overlap; ambiguity invalidates lock."""

    def __init__(self, min_iou: float = 0.10, ambiguity_ratio: float = 0.8):
        self.min_iou = min_iou
        self.ambiguity_ratio = ambiguity_ratio
        self.selected: Optional[Detection] = None
        self.failed = False

    def select(self, detections: list[Detection], x: int, y: int) -> bool:
        matches = [d for d in detections if d.name.lower() == "red block" and
                   0 <= y < d.mask.shape[0] and 0 <= x < d.mask.shape[1] and d.mask[y, x]]
        if len(matches) != 1:
            self.selected = None
            self.failed = False
            return False
        self.selected = matches[0]
        self.failed = False
        return True

    def update(self, detections: list[Detection]) -> Optional[Detection]:
        if self.selected is None or self.failed:
            return None
        selected_area = np.count_nonzero(self.selected.mask)
        candidates = []
        for d in detections:
            if d.name.lower() != self.selected.name.lower():
                continue
            distance = (np.linalg.norm(self.selected.position_base - d.position_base)
                        if self.selected.position_base is not None and d.position_base is not None
                        else float("inf"))
            if self._iou(self.selected.mask, d.mask) < 0.05 and distance >= 0.02:
                continue
            area_ratio = np.count_nonzero(d.mask) / selected_area
            if 0.65 <= area_ratio <= 1.5:
                candidates.append(d)
            elif 0.5 <= area_ratio <= 2.0 and distance < 0.03:
                candidates.append(d)
        if self.selected.position_base is not None:
            scored_3d = sorted(
                ((float(np.linalg.norm(self.selected.position_base - d.position_base)), d)
                 for d in candidates if d.position_base is not None), key=lambda item: item[0])
            if not scored_3d or scored_3d[0][0] > 0.10:
                return None
            if len(scored_3d) > 1 and scored_3d[1][0] - scored_3d[0][0] < 0.02:
                self.failed = True
                return None
            self.selected = scored_3d[0][1]
            return self.selected
        scored = sorted(((self._iou(self.selected.mask, d.mask), d) for d in candidates),
                        key=lambda item: item[0], reverse=True)
        if not scored or scored[0][0] < self.min_iou:
            return None
        if len(scored) > 1 and scored[1][0] >= scored[0][0] * self.ambiguity_ratio:
            self.failed = True
            return None
        self.selected = scored[0][1]
        return self.selected

    @staticmethod
    def _iou(a: np.ndarray, b: np.ndarray) -> float:
        if a.shape != b.shape:
            return 0.0
        union = np.count_nonzero(a | b)
        return float(np.count_nonzero(a & b) / union) if union else 0.0


def estimate_camera_point(detection: Detection, depth_mm: np.ndarray, K: np.ndarray,
                          min_depth_m: float = 0.10, max_depth_m: float = 2.0):
    """Back-project mask center using valid local median depth, in meters."""
    mask = np.asarray(detection.mask, dtype=bool)
    if mask.shape != depth_mm.shape or K.shape != (3, 3):
        return None
    v, u = np.nonzero(mask)
    if not len(u):
        return None
    u0, v0 = float(np.median(u)), float(np.median(v))
    cx, cy = int(round(u0)), int(round(v0))
    y0, y1 = max(0, cy - 5), min(mask.shape[0], cy + 6)
    x0, x1 = max(0, cx - 5), min(mask.shape[1], cx + 6)
    local = depth_mm[y0:y1, x0:x1][mask[y0:y1, x0:x1]]
    values = local.astype(np.float64) / 1000.0
    values = values[np.isfinite(values) & (values >= min_depth_m) & (values <= max_depth_m)]
    if values.size < 20:
        values = depth_mm[mask].astype(np.float64) / 1000.0
        values = values[np.isfinite(values) & (values >= min_depth_m) & (values <= max_depth_m)]
    if values.size < 20:
        return None
    z = float(np.median(values))
    return np.array([(u0 - K[0, 2]) * z / K[0, 0],
                     (v0 - K[1, 2]) * z / K[1, 1], z], dtype=np.float64)


if __name__ == "__main__":
    a = np.zeros((8, 8), dtype=np.uint8); a[1:7, 1:7] = 1
    lock = TargetLock()
    d = Detection("red block", .9, (1, 1, 2, 2), a)
    assert lock.select([d], 1, 1) and lock.update([d]) is d
    depth = np.full((8, 8), 500, dtype=np.uint16)
    point = estimate_camera_point(d, depth, np.array([[100, 0, 3.5], [0, 100, 3.5], [0, 0, 1]]))
    assert point is not None and np.allclose(point, [0, 0, .5])
    image = np.zeros((30, 30, 3), np.uint8)
    image[5:25, 5:25] = (0, 0, 255)
    assert len(detect_red_blocks(image)) == 1
    hsv = np.zeros((60, 60, 3), np.uint8)
    hsv[10:30, 10:30] = (2, 210, 180)
    hsv[15:50, 30:55] = (9, 110, 190)  # Skin-like red touching the block.
    assert len(detect_red_blocks(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))) == 1
    larger = np.zeros((8, 8), bool); larger[0:8, 0:8] = True
    intruder = Detection("red block", 1.0, (0, 0, 8, 8), larger, np.array([0.06, 0, 0]))
    d.position_base = np.zeros(3)
    assert lock.update([intruder]) is None and lock.selected is d
    near = Detection("red block", 1.0, (0, 0, 8, 8), larger, np.array([0.005, 0, 0]))
    assert lock.update([near]) is near
    first = np.zeros((20, 20), bool); first[1:7, 1:7] = True
    jumped = np.zeros((20, 20), bool); jumped[12:18, 12:18] = True
    lock.select([Detection("red block", 1.0, (1, 1, 7, 7), first, np.zeros(3))], 2, 2)
    same_block = Detection("red block", 1.0, (12, 12, 18, 18), jumped, np.array([0.01, 0, 0]))
    assert lock.update([same_block]) is same_block
    filtered, moving = None, np.zeros(3, dtype=bool)
    for z in (0.012, -0.012) * 10:
        command, filtered, moving = stabilize_camera_error(np.array([0.0, 0.0, z]), filtered, moving)
        assert np.all(command == 0)
    for _ in range(8):
        command, filtered, moving = stabilize_camera_error(np.array([0.0, 0.0, 0.05]), filtered, moving)
    assert command[2] > 0
    for _ in range(12):
        command, filtered, moving = stabilize_camera_error(np.zeros(3), filtered, moving)
    assert not np.any(moving) and np.all(command == 0)
    anchor = track_base_target(np.zeros(3), None)
    assert np.allclose(track_base_target(np.array([0.0, 0.014, 0.0]), anchor), anchor)
    assert np.allclose(track_base_target(np.array([0.0, 0.035, 0.0]), anchor), [0.0, 0.020, 0.0])
