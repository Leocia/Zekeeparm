"""YOLO model loading and raw detection parsing helpers."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

try:
    from .block_colors import BLOCK_NAMES, classify_block_color, validate_color_calibration
    from .common_utils import class_name, clip_bbox, detection_count, tensor_to_numpy
except ImportError:
    from block_colors import BLOCK_NAMES, classify_block_color, validate_color_calibration
    from common_utils import class_name, clip_bbox, detection_count, tensor_to_numpy


@dataclass
class YoloDetection:
    result_index: int
    detection_index: int
    class_name: str
    conf: float
    bbox_xyxy: tuple[int, int, int, int]
    mask: np.ndarray
    has_segmentation: bool = False


# Backward-compatible alias for older scripts. New code should use YoloDetection.
DetectionTarget = YoloDetection


def select_target_at_pixel(
    detections: list[YoloDetection], x: int, y: int,
) -> Optional[YoloDetection]:
    """Select the smallest detected mask containing the click, never a neighbor."""
    hits = [
        target for target in detections
        if 0 <= y < target.mask.shape[0] and 0 <= x < target.mask.shape[1]
        and target.mask[y, x] > 0
    ]
    return min(hits, key=lambda target: (np.count_nonzero(target.mask), -target.conf), default=None)


def target_mask_iou(first: YoloDetection, second: YoloDetection) -> float:
    """Compare instance regions while the camera is back at the same ready pose."""
    if first.mask.shape != second.mask.shape:
        return 0.0
    union = np.count_nonzero((first.mask > 0) | (second.mask > 0))
    return float(np.count_nonzero((first.mask > 0) & (second.mask > 0)) / union) if union else 0.0


def resolve_yolo_model_path(model_name: str, project_root: Path) -> Path:
    model_path = Path(str(model_name)).expanduser()
    if model_path.is_absolute():
        return model_path
    if len(model_path.parts) > 1:
        return project_root / model_path
    return project_root / "models" / model_path


def load_yolo(
    cfg: dict[str, Any],
    *,
    project_root: Path,
    no_yolo: bool = False,
    model_override: Optional[str] = None,
    device_override: Optional[str] = None,
    conf_override: Optional[float] = None,
    iou_override: Optional[float] = None,
    infer_every_override: Optional[int] = None,
    extra_classes: Optional[list[str]] = None,
) -> tuple[Optional[Any], dict[str, Any]]:
    gp_cfg = cfg.get("grasp_pipeline", {})
    yolo_opts: dict[str, Any] = {
        "enabled": not no_yolo,
        "infer_every": max(1, int(infer_every_override or gp_cfg.get("infer_every_live", 3))),
    }
    if no_yolo:
        return None, yolo_opts

    color_path = cfg.get("yolo", {}).get("color_calibration")
    if color_path:
        color_path = Path(color_path).expanduser()
        if not color_path.is_absolute():
            color_path = project_root / color_path
        with color_path.open(encoding="utf-8") as stream:
            yolo_opts["color_calibration"] = validate_color_calibration(json.load(stream))

    from ultralytics import YOLO

    yolo_cfg = cfg.get("yolo", {})
    det_cfg = cfg.get("detection", {})
    model_name = str(model_override or yolo_cfg.get("model_name", "yoloe-26s-seg.pt"))
    model_path = resolve_yolo_model_path(model_name, project_root)
    device = device_override or yolo_cfg.get("device", "cpu")
    conf = float(conf_override if conf_override is not None else det_cfg.get("conf_threshold", 0.25))
    iou = float(iou_override if iou_override is not None else det_cfg.get("iou_threshold", 0.45))
    custom_classes = list(yolo_cfg.get("custom_classes", []))
    for extra_class in extra_classes or []:
        if extra_class and extra_class not in custom_classes:
            custom_classes.append(extra_class)
    use_world = bool(yolo_cfg.get("use_world", True))

    print(f"Loading YOLO target detector: {model_path}")
    model = YOLO(str(model_path))
    if use_world and ("world" in model_name.lower() or "yoloe" in model_name.lower()) and custom_classes:
        model.set_classes(custom_classes)
        print(f"YOLO open-vocabulary classes: {custom_classes}")

    yolo_opts.update(
        {
            "model_name": model_name,
            "device": device,
            "imgsz": int(yolo_cfg.get("imgsz", 640)),
            "conf": conf,
            "iou": iou,
            "custom_classes": custom_classes,
        }
    )
    return model, yolo_opts


def _segmentation_mask(
    result: Any,
    index: int,
    image_shape: tuple[int, int],
) -> Optional[np.ndarray]:
    h, w = image_shape
    masks = getattr(result, "masks", None)
    data = getattr(masks, "data", None)
    if data is not None:
        try:
            if len(data) > index:
                mask = tensor_to_numpy(data[index])
                if mask is not None:
                    mask = np.asarray(mask, dtype=np.float32)
                    if mask.shape != (h, w):
                        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                    return (mask > 0.5).astype(np.uint8)
        except Exception:
            pass
    return None


def detection_mask(
    result: Any,
    index: int,
    image_shape: tuple[int, int],
    bbox_xyxy: tuple[int, int, int, int],
) -> np.ndarray:
    segmentation = _segmentation_mask(result, index, image_shape)
    if segmentation is not None:
        return segmentation
    h, w = image_shape
    x1, y1, x2, y2 = bbox_xyxy
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y1 : y2 + 1, x1 : x2 + 1] = 1
    return mask


def obb_points(result: Any, index: int, image_shape: tuple[int, int]) -> Optional[np.ndarray]:
    obb = getattr(result, "obb", None)
    if obb is None:
        return None
    points = None
    for attr in ("xyxyxyxy", "xyxyxyxyn"):
        values = getattr(obb, attr, None)
        if values is None:
            continue
        try:
            points = tensor_to_numpy(values[index])
        except Exception:
            points = None
        if points is not None:
            break
    if points is None:
        return None

    points = np.asarray(points, dtype=np.float32)
    if points.ndim == 3 and points.shape[0] == 1:
        points = points[0]
    if points.ndim == 1 and points.size == 8:
        points = points.reshape(4, 2)
    if points.shape != (4, 2):
        return None
    if float(np.max(np.abs(points))) <= 1.5:
        h, w = image_shape
        points = points * np.array([w, h], dtype=np.float32)
    return points.astype(np.float32)


def obb_detection_meta(result: Any, index: int, image_shape: tuple[int, int]) -> tuple[str, float, tuple[int, int, int, int]]:
    names = getattr(result, "names", {})
    obb = getattr(result, "obb", None)
    if obb is None:
        raise ValueError("YOLO result has no OBB detections")

    cls_row = tensor_to_numpy(getattr(obb, "cls")[index])
    conf_row = tensor_to_numpy(getattr(obb, "conf")[index])
    cls_id = int(np.asarray(cls_row).reshape(-1)[0])
    conf = float(np.asarray(conf_row).reshape(-1)[0])

    xyxy = getattr(obb, "xyxy", None)
    if xyxy is not None:
        bbox = clip_bbox(np.asarray(tensor_to_numpy(xyxy[index])).reshape(-1), image_shape)
    else:
        points = obb_points(result, index, image_shape)
        if points is None:
            raise ValueError("YOLO OBB result has neither xyxy nor polygon points")
        bbox = clip_bbox(np.concatenate([points.min(axis=0), points.max(axis=0)]), image_shape)
    return class_name(names, cls_id), conf, bbox


def box_detection_meta(result: Any, index: int, image_shape: tuple[int, int]) -> tuple[str, float, tuple[int, int, int, int]]:
    names = getattr(result, "names", {})
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        raise ValueError("YOLO result has no box detections")
    box = boxes[index]
    xyxy = tensor_to_numpy(box.xyxy[0]).reshape(-1)
    cls_id = int(tensor_to_numpy(box.cls[0]).reshape(-1)[0])
    conf = float(tensor_to_numpy(box.conf[0]).reshape(-1)[0])
    return class_name(names, cls_id), conf, clip_bbox(xyxy, image_shape)


def detection_meta(result: Any, index: int, image_shape: tuple[int, int]) -> tuple[str, float, tuple[int, int, int, int]]:
    if getattr(result, "obb", None) is not None:
        return obb_detection_meta(result, index, image_shape)
    return box_detection_meta(result, index, image_shape)


def detection_polygon_mask(points: np.ndarray, image_shape: tuple[int, int]) -> np.ndarray:
    mask = np.zeros(image_shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.round(points).astype(np.int32)], 1)
    return mask


def collect_detections(results: list[Any], image_shape: tuple[int, int]) -> list[YoloDetection]:
    detections: list[YoloDetection] = []
    for result_index, result in enumerate(results):
        for detection_index in range(detection_count(result)):
            try:
                name, conf, bbox = detection_meta(result, detection_index, image_shape)
                points = obb_points(result, detection_index, image_shape)
                segmentation = _segmentation_mask(result, detection_index, image_shape) if points is None else None
                mask = segmentation
                if mask is None:
                    mask = detection_polygon_mask(points, image_shape) if points is not None else detection_mask(result, detection_index, image_shape, bbox)
            except Exception:
                continue
            detections.append(YoloDetection(result_index, detection_index, name, conf, bbox, mask,
                                            has_segmentation=segmentation is not None))
    return detections


def detect_objects(
    model: Any,
    color_bgr: np.ndarray,
    yolo_opts: dict[str, Any],
) -> tuple[list[Any], list[YoloDetection]]:
    results = model.predict(
        color_bgr,
        verbose=False,
        device=yolo_opts.get("device", "cpu"),
        imgsz=int(yolo_opts.get("imgsz", 640)),
        conf=float(yolo_opts.get("conf", 0.25)),
        iou=float(yolo_opts.get("iou", 0.45)),
    )
    detections = collect_detections(results, color_bgr.shape[:2])
    calibration = yolo_opts.get("color_calibration")
    if calibration:
        for detection in detections:
            if detection.class_name in BLOCK_NAMES:
                x1, y1, x2, y2 = detection.bbox_xyxy
                # Bounding-box fallbacks include background and cannot establish a block's color.
                color = classify_block_color(color_bgr[y1:y2 + 1, x1:x2 + 1],
                    detection.mask[y1:y2 + 1, x1:x2 + 1], calibration) if detection.has_segmentation else None
                detection.class_name = f"{color} block" if color else "block"
    return results, detections


# Compatibility names kept for existing callers; they do not perform target selection.
target_mask = detection_mask
collect_targets = collect_detections
detect_targets = detect_objects
