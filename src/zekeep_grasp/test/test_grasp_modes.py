"""Mouse/continuous orchestration checks; all camera, GUI and motor I/O is mocked."""

import argparse
from contextlib import ExitStack, redirect_stdout
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import main as route
from utils.yolo_utils import YoloDetection, select_target_at_pixel, target_mask_iou


def target(index, x, name="red block", conf=0.9):
    mask = np.zeros((8, 12), dtype=np.uint8)
    mask[2:5, x:x + 3] = 1
    return YoloDetection(0, index, name, conf, (x, 2, x + 2, 4), mask)


class GraspModesTest(unittest.TestCase):
    def setUp(self):
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(redirect_stdout(io.StringIO()))
        self.cfg = {
            "camera": {"type": "orbbec_gemini2"},
            "place": {"enabled": True},
            "robot": {"end_effector_frame": "end_link"},
            "safety": {"table_collision_enabled": False},
        }
        self.args = argparse.Namespace(config="unused", camera_type=None,
                                       checkpoint=None, target_class=None)
        self.camera = Mock(K=np.eye(3), serial="test")
        self.robot = Mock(control_loop_active=False)
        self.frame_index = 0
        self.callback = None

        def get_frame():
            self.frame_index += 1
            if self.frame_index > 30:
                self.fail("grasp loop did not stop")
            return (np.full((8, 12, 3), self.frame_index, np.uint8),
                    np.full((8, 12), self.frame_index, np.uint16))

        self.camera.get_frame.side_effect = get_frame

        def inference(*args):
            return Mock(best=object(), selected_target=args[-1], status="ok")

        values = {
            "_args": Mock(return_value=self.args),
            "load_config": Mock(return_value=self.cfg),
            "configure_camera": Mock(return_value=self.cfg),
            "make_camera": Mock(return_value=self.camera),
            "OfficialSdkRobot": Mock(return_value=self.robot),
            "load_hand_eye": Mock(return_value=(np.eye(4), "eye_in_hand")),
            "load_yolo": Mock(return_value=(Mock(), {"infer_every": 1})),
            "build_net": Mock(),
            "detect_objects": Mock(return_value=([], [])),
            "infer_configured_frame": Mock(side_effect=inference),
            "camera_to_base_from_hand_eye_reference": Mock(return_value=np.eye(4)),
            "_anchor_grasps_to_measured_center": Mock(return_value=True),
            "_choose_candidate": Mock(return_value=(object(), [np.zeros(6)] * 3, [np.zeros(6)] * 3)),
            "_execute_candidate": Mock(return_value=True),
            "_snapshot_feedback": Mock(return_value=np.zeros((8, 12, 3), np.uint8)),
            "_save_snapshot": Mock(),
        }
        self.mocks = values
        for name, value in values.items():
            stack.enter_context(patch.object(route, name, value))
        stack.enter_context(patch("utils.calibration_identity.validate_calibration_identity"))
        for name in ("namedWindow", "imshow", "destroyAllWindows"):
            stack.enter_context(patch.object(route.cv2, name))
        stack.enter_context(patch.object(route.cv2, "setMouseCallback", side_effect=self.set_callback))
        self.wait_key = stack.enter_context(patch.object(route.cv2, "waitKey", return_value=-1))

    def set_callback(self, window, callback):
        self.callback = callback

    def test_disabled_placement_rejects_cycle_before_opening_devices(self):
        self.cfg["place"]["enabled"] = False
        self.args.target_class = "red block"
        self.assertEqual(route.main(), 1)
        self.mocks["make_camera"].assert_not_called()
        self.mocks["OfficialSdkRobot"].assert_not_called()

    def test_click_selects_exact_instance_and_displayed_rgb_depth(self):
        first, second = target(0, 1), target(1, 6, conf=0.5)
        self.mocks["load_yolo"].return_value = (Mock(), {"infer_every": 3})
        self.mocks["detect_objects"].return_value = ([], [first, second])
        steps = 0

        def events(delay):
            nonlocal steps
            steps += 1
            if steps == 1:
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 11, 7, 0, None)
            if steps == 4:
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 7, 3, 0, None)
            if steps == 5:
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 2, 3, 0, None)
            return ord("q") if steps == 6 else -1

        self.wait_key.side_effect = events
        self.assertEqual(route.main(click_to_grasp=True), 0)
        inference = self.mocks["infer_configured_frame"]
        inference.assert_called_once()
        args = inference.call_args.args
        self.assertIs(args[-1], second)
        self.assertTrue(np.all(args[1] == 3))
        self.assertTrue(np.all(args[2] == 3))
        self.assertEqual(self.frame_index, 6)
        self.assertEqual(self.mocks["load_yolo"].call_args.kwargs["extra_classes"], [])

    def test_continuous_grasps_two_red_cubes_and_ignores_placed_objects(self):
        first, second = target(0, 1), target(1, 6, conf=0.8)
        placed = target(2, 9, conf=0.99)
        self.args.target_class = "redcube"
        self.mocks["detect_objects"].side_effect = [
            ([], [first, second]), ([], [placed, second]), ([], [placed]),
        ]
        self.assertEqual(route.main(continuous=True), 0)
        selected = [call.args[-1] for call in self.mocks["infer_configured_frame"].call_args_list]
        self.assertEqual([item.detection_index for item in selected], [0, 1])
        self.assertEqual(self.mocks["load_yolo"].call_args.kwargs["extra_classes"], ["red block"])
        self.assertEqual(self.mocks["_execute_candidate"].call_count, 2)

    def test_right_click_resumes_for_a_second_selection(self):
        first, second = target(0, 1), target(1, 6)
        self.mocks["detect_objects"].side_effect = [([], [first, second]), ([], [second])]
        steps = 0

        def events(delay):
            nonlocal steps
            steps += 1
            if steps == 1:
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 2, 3, 0, None)
            if steps == 2:
                self.callback(route.cv2.EVENT_RBUTTONDOWN, 0, 0, 0, None)
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 2, 3, 0, None)
            if steps == 3:
                self.callback(route.cv2.EVENT_LBUTTONDOWN, 7, 3, 0, None)
            return ord("q") if steps == 4 else -1

        self.wait_key.side_effect = events
        self.assertEqual(route.main(click_to_grasp=True), 0)
        selected = [call.args[-1].detection_index for call in self.mocks["infer_configured_frame"].call_args_list]
        self.assertEqual(selected, [0, 1])

    def test_cube_grasps_each_color_and_preserves_class_for_placement(self):
        red, blue = target(0, 1), target(1, 6, "blue block")
        pen = target(2, 9, "pen")
        self.args.target_class = "cube"
        self.mocks["detect_objects"].side_effect = [([], [red, blue, pen]), ([], [blue, pen]), ([], [pen])]
        self.assertEqual(route.main(continuous=True), 0)
        self.assertEqual([call.kwargs["class_name"] for call in self.mocks["_execute_candidate"].call_args_list],
                         ["red block", "blue block"])

    def test_missing_remaining_target_stops_with_failure(self):
        self.args.target_class = "red cube"
        self.mocks["detect_objects"].side_effect = [([], [target(0, 1), target(1, 6, conf=0.8)]), ([], [])]
        self.assertEqual(route.main(continuous=True), 1)
        self.mocks["_execute_candidate"].assert_called_once()

    def test_empty_grasp_stops_without_retry(self):
        self.mocks["detect_objects"].return_value = ([], [target(0, 1)])
        self.mocks["_execute_candidate"].return_value = False
        self.assertEqual(route.main(continuous=True), 1)
        self.mocks["_execute_candidate"].assert_called_once()
        self.mocks["detect_objects"].assert_called_once()

    def test_planning_failure_stops_without_execution(self):
        self.mocks["detect_objects"].return_value = ([], [target(0, 1)])
        self.mocks["_choose_candidate"].return_value = None
        self.assertEqual(route.main(continuous=True), 1)
        self.mocks["_execute_candidate"].assert_not_called()

    def test_execution_fault_holds_and_stops_without_retry(self):
        self.mocks["detect_objects"].return_value = ([], [target(0, 1)])
        self.mocks["_execute_candidate"].side_effect = RuntimeError("tracking fault")
        self.assertEqual(route.main(continuous=True), 1)
        self.robot.hold_current.assert_called_once()
        self.mocks["_execute_candidate"].assert_called_once()

    def test_no_matching_targets_does_not_execute(self):
        self.mocks["detect_objects"].return_value = ([], [target(0, 1, "pen")])
        self.assertEqual(route.main(continuous=True), 0)
        self.mocks["_execute_candidate"].assert_not_called()

    def test_original_entry_still_waits_for_g_and_uses_fresh_capture(self):
        self.args.target_class = "red block"
        self.mocks["detect_objects"].return_value = ([], [target(0, 1)])
        self.wait_key.side_effect = [-1, ord("g"), ord("q")]
        self.assertEqual(route.main(), 0)
        self.mocks["_execute_candidate"].assert_called_once()
        args = self.mocks["infer_configured_frame"].call_args.args
        self.assertTrue(np.all(args[1] == 3))


class PixelSelectionTest(unittest.TestCase):
    def test_hit_background_bounds_overlap_and_empty_mask(self):
        first, second = target(0, 1), target(1, 6)
        self.assertIs(select_target_at_pixel([first, second], 7, 3), second)
        for x, y in ((0, 0), (-1, 3), (12, 3), (2, -1), (2, 8)):
            self.assertIsNone(select_target_at_pixel([first, second], x, y))
        first.mask[3, 2] = 0
        self.assertIsNone(select_target_at_pixel([first], 2, 3))
        smaller = target(2, 6, conf=0.1)
        smaller.mask[2, :] = 0
        self.assertIs(select_target_at_pixel([second, smaller], 7, 3), smaller)
        self.assertEqual(target_mask_iou(first, second), 0.0)
        self.assertEqual(target_mask_iou(second, second), 1.0)
        second.mask[:] = 0
        self.assertEqual(target_mask_iou(second, second), 0.0)


if __name__ == "__main__":
    unittest.main()
