"""Offline color calibration checks; no camera or motors."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.block_colors import classify_block_color, fit_color_calibration, validate_color_calibration
from utils.yolo_utils import YoloDetection, collect_detections, detect_objects, load_yolo

ROOT = Path(__file__).resolve().parents[1]


def patch_hue(hue, saturation=220, value=180):
    return cv2.cvtColor(np.full((24, 24, 3), (hue, saturation, value), np.uint8), cv2.COLOR_HSV2BGR)


class BlockColorTest(unittest.TestCase):
    def setUp(self):
        self.hues = {"red": 0, "purple": 145, "blue": 108, "green": 65}
        self.calibration = fit_color_calibration({name: [patch_hue(hue)] for name, hue in self.hues.items()})
        self.mask = np.ones((24, 24), np.uint8)

    def test_four_colors_and_red_wrap_ignore_brightness(self):
        for name, hue in self.hues.items():
            for value in (65, 220):
                with self.subTest(name=name, value=value):
                    self.assertEqual(classify_block_color(patch_hue(hue, value=value), self.mask, self.calibration), name)
        self.assertEqual(classify_block_color(patch_hue(179), self.mask, self.calibration), "red")
        wrap = fit_color_calibration({"red": [patch_hue(179), patch_hue(1)]})
        self.assertLess(min(wrap["colors"]["red"]["hue_center"], 180 - wrap["colors"]["red"]["hue_center"]), 2)

    def test_neutral_dark_mixed_unseen_and_tiny_regions_are_unknown(self):
        mixed = patch_hue(0)
        mixed[:, 12:] = patch_hue(145)[:, 12:]
        images = (patch_hue(0, saturation=10), patch_hue(0, value=20), mixed, patch_hue(85))
        for image in images:
            self.assertIsNone(classify_block_color(image, self.mask, self.calibration))
        neutral = patch_hue(0, saturation=0)
        neutral[:8] = patch_hue(0)[:8]
        self.assertIsNone(classify_block_color(neutral, self.mask, self.calibration))
        tiny = np.zeros_like(self.mask)
        tiny[:4, :4] = 1
        self.assertIsNone(classify_block_color(patch_hue(0), tiny, self.calibration))

    def test_background_and_mask_edge_do_not_set_color(self):
        image = patch_hue(145)
        image[4:20, 4:20] = patch_hue(0)[:16, :16]
        mask = np.zeros_like(self.mask)
        mask[3:21, 3:21] = 1
        self.assertEqual(classify_block_color(image, mask, self.calibration), "red")

    def test_bad_training_and_overlapping_profiles_fail(self):
        with self.assertRaises(ValueError):
            fit_color_calibration({"red": [patch_hue(0, saturation=0)]})
        with self.assertRaises(ValueError):
            fit_color_calibration({"blue": [patch_hue(90)], "green": [patch_hue(94)]})
        bad = copy.deepcopy(self.calibration)
        bad["colors"]["red"]["hue_center"] = float("nan")
        with self.assertRaises(ValueError):
            validate_color_calibration(bad)

    def test_shared_detector_corrects_label_and_preserves_geometry_and_score(self):
        wrong = YoloDetection(0, 0, "purple block", .58, (0, 0, 23, 23), self.mask, has_segmentation=True)
        pen = YoloDetection(0, 1, "pen", .7, (0, 0, 23, 23), self.mask)
        model = Mock()
        with patch("utils.yolo_utils.collect_detections", return_value=[wrong, pen]):
            _, detections = detect_objects(model, patch_hue(0), {"color_calibration": self.calibration})
        self.assertEqual([d.class_name for d in detections], ["red block", "pen"])
        self.assertIs(detections[0].mask, self.mask)
        self.assertEqual((wrong.conf, wrong.bbox_xyxy), (.58, (0, 0, 23, 23)))
        with patch("utils.yolo_utils.collect_detections", return_value=[wrong]):
            detect_objects(model, patch_hue(85), {"color_calibration": self.calibration})
        self.assertEqual(wrong.class_name, "block")

    def test_box_fallback_does_not_use_background_as_block_color(self):
        box = SimpleNamespace(xyxy=np.array([[0, 0, 23, 23]]), cls=np.array([0]), conf=np.array([.8]))
        result = SimpleNamespace(names={0: "purple block"}, boxes=[box], masks=None)
        model = Mock()
        model.predict.return_value = [result]
        self.assertFalse(collect_detections([result], (24, 24))[0].has_segmentation)
        _, detections = detect_objects(model, patch_hue(0), {"color_calibration": self.calibration})
        self.assertEqual(detections[0].class_name, "block")
        result.masks = SimpleNamespace(data=np.ones((1, 24, 24)))
        _, detections = detect_objects(model, patch_hue(0), {"color_calibration": self.calibration})
        self.assertTrue(detections[0].has_segmentation)
        self.assertEqual(detections[0].class_name, "red block")

    def test_default_profile_loads_and_recognizes_recorded_bgr_samples(self):
        import yaml
        cfg = yaml.safe_load((ROOT / "config/default.yaml").read_text())
        model = Mock()
        with patch.dict(sys.modules, {"ultralytics": SimpleNamespace(YOLO=lambda path: model)}):
            _, options = load_yolo(cfg, project_root=ROOT)
        profile = options["color_calibration"]
        self.assertEqual(profile, json.loads((ROOT / cfg["yolo"]["color_calibration"]).read_text()))
        # Medians of human-labeled top/side patches, not labels produced by YOLO.
        samples = {"red": [(21, 31, 173), (28, 31, 147), (54, 53, 140)],
                   "green": [(95, 134, 0), (85, 112, 12), (70, 94, 2)],
                   "blue": [(205, 148, 0)],
                   "purple": [(130, 64, 127), (111, 55, 101), (118, 68, 109)]}
        for name, colors in samples.items():
            for bgr in colors:
                with self.subTest(name=name, bgr=bgr):
                    self.assertEqual(classify_block_color(np.full((24, 24, 3), bgr, np.uint8), self.mask, profile), name)
        cfg["yolo"]["color_calibration"] = "missing-color-calibration.json"
        with self.assertRaises(FileNotFoundError):
            load_yolo(cfg, project_root=ROOT)


if __name__ == "__main__":
    unittest.main()
