"""Color configuration and grasp rejection diagnostics, without hardware access."""
from contextlib import ExitStack
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[2] / 'zekeep_grasp'
sys.path.insert(0, str(PROJECT))

from utils.yolo_utils import YoloDetection, detect_objects, load_yolo


class WebColorDetectionTest(unittest.TestCase):
    def test_default_detector_passes_purple_to_open_vocabulary_model(self):
        cfg = yaml.safe_load((PROJECT / 'config/default.yaml').read_text())
        model = Mock()
        with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda path: model)}):
            loaded, opts = load_yolo(cfg, project_root=PROJECT)
        self.assertIs(loaded, model)
        model.set_classes.assert_called_once_with(cfg['yolo']['custom_classes'])
        self.assertIn('purple block', opts['custom_classes'])
        self.assertIn('blue block', opts['custom_classes'])
        self.assertIn('green block', opts['custom_classes'])
        from drivers.robot.grasp_driver import GRIPPER_MAX_DISTANCE_M
        self.assertEqual(cfg['robot']['gripper_max_width_m'], GRIPPER_MAX_DISTANCE_M)
        self.assertEqual(opts['imgsz'], 1280)
        model.predict.return_value = []
        frame = np.zeros((800, 1280, 3), dtype=np.uint8)
        self.assertEqual(detect_objects(model, frame, opts), ([], []))
        model.predict.assert_called_once_with(frame, verbose=False, device='0',
            imgsz=1280, conf=.3, iou=.45)
        del cfg['yolo']['imgsz']
        with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda path: model)}):
            _, legacy_opts = load_yolo(cfg, project_root=PROJECT)
        self.assertEqual(legacy_opts['imgsz'], 640)

    def test_width_rejection_is_specific_and_keeps_physical_limit(self):
        from utils import graspnet_utils as g
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        target = YoloDetection(0, 0, 'green block', .36, (0, 0, 10, 10),
                               np.ones((10, 10), dtype=bool))
        for widths, rejected in (([.072, .082], True), ([.069, .082], False), ([], False)):
            with self.subTest(widths=widths), ExitStack() as stack:
                data = np.zeros((len(widths), 17))
                data[:, 0] = .8
                data[:, 1] = widths
                candidates = g.GraspGroup(data)
                stack.enter_context(patch.object(g, 'build_end_points', return_value=(None, None, None)))
                stack.enter_context(patch.object(g, 'infer_grasps', return_value=(candidates,
                    {'decoded': 1024, 'collision_removed': 810, 'pre_collision': 1024})))
                for name in ('filter_grasps_by_bbox', 'filter_grasps_by_target_mask',
                             'bias_grasp_scores_by_mask_center'):
                    stack.enter_context(patch.object(g, name, side_effect=lambda grasps, *a, **kw: grasps))
                stack.enter_context(patch.object(g, 'estimate_target_depth', return_value=None))
                result = g.infer_frame(None, frame, np.zeros((10, 10)), np.eye(3),
                    num_point=10, min_depth=.1, max_depth=2, collision_thresh=.01,
                    selected_target_override=target, max_grasp_width_m=.070)
                self.assertEqual('rejected: width filter removed all candidates' in result.status, rejected)
                self.assertEqual(len(result.grasps), sum(width <= .070 for width in widths))
                if rejected:
                    self.assertIsNone(result.best)
                    self.assertIn('width_limit=0.070m', result.status)
                    self.assertIn('width=[0.072,0.082]m', result.status)


if __name__ == '__main__':
    unittest.main()
