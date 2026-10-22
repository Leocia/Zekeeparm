import math
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from zekeep_embodied.pi05_preview import load_preview, make_preview_trajectory, PredictionPreview
from zekeep_embodied.pi05_runtime import Pi05Runtime
from zekeep_embodied.record import _load_ros_calibration


class PredictionPreviewTest(unittest.TestCase):
    def setUp(self):
        self.hardware_config = Path(__file__).resolve().parents[2] / "zekeep_bringup/config/zekeep_hardware.yaml"
        self.calibrations, self.open_ros, self.close_ros = _load_ros_calibration(self.hardware_config)

    def test_full_horizon_calibration_gripper_and_timestamps(self):
        state = [0.0] * 7
        actions = [[-10.0 - index] * 6 + [math.degrees(self.open_ros)] for index in range(10)]
        message = make_preview_trajectory(
            state, actions, self.calibrations, self.open_ros, self.close_ros, 0.070, 9.0,
        )
        trajectory = message.trajectory[0].joint_trajectory
        self.assertEqual(trajectory.joint_names, [f"joint{i}" for i in range(1, 7)] + ["gripper_joint"])
        self.assertEqual(len(trajectory.points), 11)
        self.assertEqual(list(message.trajectory_start.joint_state.position), [0.0] * 7)
        for index, point in enumerate(trajectory.points[1:]):
            self.assertAlmostEqual(point.positions[0], math.radians(10 + index))
            self.assertAlmostEqual(point.positions[6], 0.035)
            duration = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            self.assertAlmostEqual(duration, (index + 1) / 9, places=8)

    def test_raw_out_of_limit_prediction_is_not_silently_clipped(self):
        message = make_preview_trajectory(
            [0.0] * 7, [[400.0] * 7] * 10,
            self.calibrations, self.open_ros, self.close_ros, 0.070, 9.0,
        )
        self.assertAlmostEqual(message.trajectory[0].joint_trajectory.points[1].positions[0], -math.radians(400))
        for actions in ([[0.0] * 7] * 9, [[math.nan] * 7] * 10):
            with self.assertRaises(ValueError):
                make_preview_trajectory(
                    [0.0] * 7, actions, self.calibrations,
                    self.open_ros, self.close_ros, 0.070, 9.0,
                )

    def test_only_display_topics_are_created_and_published(self):
        node = Mock()
        node.calibrations = self.calibrations
        node.gripper_open_ros, node.gripper_close_ros = self.open_ros, self.close_ros
        node.runtime = {"policy_hz": 9.0}
        # Use a real ROS Time message for generated message type checks.
        from builtin_interfaces.msg import Time

        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        preview = PredictionPreview(node, self.hardware_config)
        self.assertEqual([call.args[1] for call in node.create_publisher.call_args_list],
                         ["/pi05_preview/trajectory", "/pi05_preview/state"])
        preview.publish([0.0] * 7, [[0.0] * 7] * 10)
        self.assertEqual(node.create_publisher.return_value.publish.call_count, 2)
        node.create_client.assert_not_called()

    def test_preview_cannot_be_enabled_for_real_execution(self):
        with self.assertRaisesRegex(ValueError, "shadow-only"):
            Pi05Runtime({}, {}, execute=True, preview=True)
        with self.assertRaisesRegex(ValueError, "shadow-only"):
            Pi05Runtime({}, {}, execute=True, preview_once=True)
        with self.assertRaisesRegex(ValueError, "shadow-only"):
            Pi05Runtime({}, {}, execute=True, save_preview="preview.json")

    def test_save_needs_no_viewer_preserves_horizon_and_refuses_overwrite(self):
        from builtin_interfaces.msg import Time

        node = Mock()
        node.calibrations = self.calibrations
        node.gripper_open_ros, node.gripper_close_ros = self.open_ros, self.close_ros
        node.runtime = {"policy_hz": 9.0, "repo_id": "test/fixture", "experiment": "test"}
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        state = [0.0] * 7
        actions = [[-10.0 - index] * 6 + [0.0] for index in range(10)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "prediction.json"
            preview = PredictionPreview(node, self.hardware_config, save_path=path)
            preview.publish(state, actions, report={"action_guard_passed": False})
            self.assertTrue(preview.sent)
            node.create_publisher.assert_not_called()
            message, snapshot = load_preview(path)
            self.assertEqual(snapshot["state_deg"], state)
            self.assertEqual(snapshot["actions_deg"], actions)
            self.assertFalse(snapshot["guard_report"]["action_guard_passed"])
            self.assertEqual(len(message.trajectory[0].joint_trajectory.points), 11)
            original = path.read_bytes()
            preview.publish(state, [[0.0] * 7] * 10)
            self.assertEqual(path.read_bytes(), original)
            with self.assertRaises(FileExistsError):
                PredictionPreview(node, self.hardware_config, save_path=path)
            raced_path = Path(directory) / "raced.json"
            raced_preview = PredictionPreview(node, self.hardware_config, save_path=raced_path)
            raced_path.write_text("preserve existing prediction")
            with self.assertRaises(FileExistsError):
                raced_preview.publish(state, actions)
            self.assertEqual(raced_path.read_text(), "preserve existing prediction")
            self.assertFalse(raced_preview.sent)

    def test_corrupt_saved_geometry_is_rejected(self):
        from rosidl_runtime_py.convert import message_to_ordereddict

        message = make_preview_trajectory(
            [0.0] * 7, [[0.0] * 7] * 10,
            self.calibrations, self.open_ros, self.close_ros, 0.070, 9.0,
        )
        for corruption in ("shape", "nan", "timing", "start", "version"):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                data = {"schema_version": 1, "kind": "zekeep_pi05_preview",
                        "display_trajectory": message_to_ordereddict(message)}
                points = data["display_trajectory"]["trajectory"][0]["joint_trajectory"]["points"]
                if corruption == "shape":
                    points.pop()
                elif corruption == "nan":
                    points[1]["positions"][0] = math.nan
                elif corruption == "timing":
                    points[1]["time_from_start"] = {"sec": 0, "nanosec": 0}
                elif corruption == "start":
                    data["display_trajectory"]["trajectory_start"]["joint_state"]["position"][0] = 1.0
                else:
                    data["schema_version"] = 99
                path = Path(directory) / "bad.json"
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load_preview(path)

    def test_capture_tick_stops_requesting_after_save_or_error(self):
        for sent, error in ((True, None), (False, OSError("disk full"))):
            with self.subTest(sent=sent):
                runtime = SimpleNamespace(
                    save_preview="preview.json", preview=SimpleNamespace(sent=sent, error=error),
                    _finish_inference=Mock(), _capture_request=Mock(),
                )
                Pi05Runtime._tick(runtime)
                runtime._capture_request.assert_not_called()

    def test_save_error_is_retained_for_capture_loop_to_exit(self):
        from concurrent.futures import Future
        from unittest.mock import patch

        future = Future()
        future.set_result({"actions": [[3.0] * 7] * 10, "infer_ms": 400.0})
        runtime = SimpleNamespace(
            inference=future, inference_started=0.0, inference_state=[0.0] * 7,
            latest_serial=0, execute=False, preview=Mock(), save_preview="preview.json",
            runtime={"max_inference_s": 1.0, "max_action_step_deg": 0.5},
            motor_limits=[(0.0, 10.0)] * 7,
            _motor_state=lambda: [0.0] * 7, get_logger=lambda: Mock(),
        )
        failure = OSError("disk full")
        runtime.preview.publish.side_effect = failure
        with patch("builtins.print"):
            Pi05Runtime._finish_inference(runtime, 0.5)
        self.assertIs(runtime.preview.error, failure)

    def test_capture_main_saves_once_and_closes_cameras_without_rviz(self):
        import numpy as np
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from zekeep_msgs.msg import JointMotorState
        from zekeep_embodied import pi05_runtime
        from zekeep_embodied.record import load_config

        config, base = load_config(Path(__file__).resolve().parents[1] / "configs/record.yaml")
        cameras = [Mock(), Mock()]
        sequences = iter(range(1000))
        for camera in cameras:
            camera.snapshot.side_effect = lambda: (np.zeros((16, 16, 3), dtype=np.uint8),
                                                    time.monotonic(), next(sequences))
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"ROS_DOMAIN_ID": "97"}):
            # All feedback below is synthetic and confined to a test-only DDS domain.
            rclpy.init()
            source = rclpy.create_node("preview_capture_test_feedback")
            arm = source.create_publisher(JointState, "/zekeep/joint_states", qos_profile_sensor_data)
            gripper = source.create_publisher(JointMotorState, "/zekeep/gripper/state", qos_profile_sensor_data)
            joints = JointState(name=[f"joint{i}" for i in range(1, 7)], position=[0.0] * 6)
            finger = JointMotorState(position=0.0)
            stop = threading.Event()

            def feedback():
                while not stop.is_set() and rclpy.ok():
                    arm.publish(joints)
                    gripper.publish(finger)
                    stop.wait(0.02)

            worker = threading.Thread(target=feedback)
            worker.start()
            watchdog = threading.Timer(10, rclpy.try_shutdown)
            watchdog.start()
            path = Path(directory) / "prediction.json"
            try:
                with (patch("sys.argv", ["pi05_runtime", "--config", "unused.yaml", "--shadow",
                                        "--save-preview", str(path)]),
                      patch.object(pi05_runtime, "load_config", return_value=(config, base)),
                      patch.object(pi05_runtime, "_check_server", return_value={"test_fixture": True}),
                      patch.object(pi05_runtime, "Camera", side_effect=cameras),
                      patch.object(pi05_runtime.rclpy, "init"),
                      patch.object(pi05_runtime.Pi05Runtime, "_infer",
                                   return_value={"actions": [[-10.0] * 6 + [20.0]] * 10,
                                                 "infer_ms": 1.0}) as inference,
                      patch("builtins.print")):
                    pi05_runtime.main()
                _, snapshot = load_preview(path)
                self.assertEqual(len(snapshot["actions_deg"]), 10)
                inference.assert_called_once()
                for camera in cameras:
                    camera.close.assert_called_once()
                self.assertFalse(rclpy.ok())
            finally:
                watchdog.cancel()
                stop.set()
                worker.join(timeout=2)
                source.destroy_node()
                rclpy.try_shutdown()

    def test_replay_cli_sends_saved_messages_without_loading_torch(self):
        import rclpy
        from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory
        from rosidl_runtime_py.convert import message_to_ordereddict
        from zekeep_embodied.storage import write_json

        message = make_preview_trajectory(
            [0.0] * 7, [[-10.0] * 6 + [20.0]] * 10,
            self.calibrations, self.open_ros, self.close_ros, 0.070, 9.0,
        )
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"ROS_DOMAIN_ID": "98"}):
            path = Path(directory) / "test.json"
            write_json(path, {"schema_version": 1, "kind": "zekeep_pi05_preview",
                              "display_trajectory": message_to_ordereddict(message)})
            rclpy.init()
            viewer = rclpy.create_node("preview_replay_test_viewer")
            received = {}
            viewer.create_subscription(DisplayRobotState, "/pi05_preview/state",
                                       lambda msg: received.update(state=msg), 10)
            viewer.create_subscription(DisplayTrajectory, "/pi05_preview/trajectory",
                                       lambda msg: received.update(trajectory=msg), 10)
            process = subprocess.Popen(
                [sys.executable, "-m", "zekeep_embodied.pi05_preview", str(path)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            try:
                deadline = time.monotonic() + 8
                while len(received) < 2 and process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(viewer, timeout_sec=0.1)
                self.assertEqual(len(received), 2, "Replay CLI did not publish both display messages")
                self.assertEqual(received["trajectory"], message)
                self.assertEqual(received["state"].state, message.trajectory_start)
                self.assertNotIn("libtorch", Path(f"/proc/{process.pid}/maps").read_text())
                status = Path(f"/proc/{process.pid}/status").read_text()
                print("Replay memory:", next(line for line in status.splitlines() if line.startswith("VmRSS:")))
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    output, error = process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    output, error = process.communicate()
                viewer.destroy_node()
                rclpy.try_shutdown()
            self.assertEqual(process.returncode, 0, error)
            self.assertIn("offline_replay", output)

    def test_once_waits_for_viewer_and_sends_only_one_block(self):
        node = Mock()
        node.calibrations = self.calibrations
        node.gripper_open_ros, node.gripper_close_ros = self.open_ros, self.close_ros
        node.runtime = {"policy_hz": 9.0}
        from builtin_interfaces.msg import Time

        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        publisher = node.create_publisher.return_value
        publisher.get_subscription_count.return_value = 0
        preview = PredictionPreview(node, self.hardware_config, once=True)
        preview.publish([0.0] * 7, [[0.0] * 7] * 10)
        publisher.publish.assert_not_called()
        publisher.get_subscription_count.return_value = 1
        preview.publish([0.0] * 7, [[0.0] * 7] * 10)
        preview.publish([0.0] * 7, [[3.0] * 7] * 10)
        self.assertEqual(publisher.publish.call_count, 2)

    def test_preview_is_published_even_if_first_action_guard_rejects(self):
        from concurrent.futures import Future
        from unittest.mock import patch

        state = [0.0] * 6 + [-9.0]
        future = Future()
        future.set_result({"actions": [[3.0] * 7] * 10, "infer_ms": 400.0})
        runtime = SimpleNamespace(
            inference=future, inference_started=0.0, inference_state=state,
            latest_serial=0, execute=False, preview=Mock(),
            runtime={"max_inference_s": 1.0, "max_action_step_deg": 0.5},
            motor_limits=[(0.0, 10.0)] * 7,
            _motor_state=lambda: state, get_logger=lambda: Mock(),
        )
        with patch("builtins.print"):
            Pi05Runtime._finish_inference(runtime, 0.5)
        runtime.preview.publish.assert_called_once()
        self.assertEqual(runtime.preview.publish.call_args.args[0], state)
        self.assertIsNone(runtime.policy_error)


if __name__ == "__main__":
    unittest.main()
