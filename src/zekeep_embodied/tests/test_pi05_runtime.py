from concurrent.futures import Future
import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from zekeep_embodied.pi05_runtime import (
    _runtime_config,
    gripper_transition,
    guard_action,
    motor_degrees_to_ros,
    Pi05Runtime,
)
from zekeep_embodied.record import _ros_to_motor_degrees


class Pi05RuntimeSafetyTest(unittest.TestCase):
    def setUp(self):
        self.calibration = {
            "direction": -1.0,
            "gear_ratio": 2.0,
            "encoder_zero": 0.2,
            "ros_zero": -0.1,
            "hard_lower": -2.0,
            "hard_upper": 2.0,
        }

    def test_motor_ros_round_trip(self):
        motor = [37.5]
        ros = motor_degrees_to_ros(motor, [self.calibration])
        restored = _ros_to_motor_degrees(ros, [self.calibration])
        self.assertAlmostEqual(restored[0], motor[0], places=9)

    def test_guard_limits_each_step_and_rejects_hard_limit_violation(self):
        limits = [(-10.0, 10.0)] * 7
        guarded = guard_action([0.0] * 7, [3.0, -3.0, 0, 0, 0, 0, 0], limits, 0.5)
        self.assertEqual(guarded[:2], [0.5, -0.5])
        with self.assertRaisesRegex(RuntimeError, "outside guarded limits"):
            guard_action([0.0] * 7, [11.0] + [0.0] * 6, limits, 0.5)
        with self.assertRaises(ValueError):
            guard_action([0.0] * 7, [math.nan] + [0.0] * 6, limits, 0.5)

    def test_gripper_hysteresis(self):
        self.assertEqual(gripper_transition(0, 60, 65, 0, 5), "open")
        self.assertEqual(gripper_transition(60, 0, 65, 0, 5), "close")
        self.assertIsNone(gripper_transition(0, 34, 65, 0, 5))

    def test_shadow_prints_predictions_even_when_guard_rejects(self):
        for current, target, passed in (
            ([0.0] * 7, [3.0] * 7, True),
            ([0.0] * 6 + [-9.0], [3.0] * 7, False),
            ([0.0] * 7, [11.0] * 7, False),
        ):
            with self.subTest(current=current, target=target):
                future = Future()
                future.set_result({"actions": [target] * 10, "infer_ms": 400.0})
                logger = Mock()
                runtime = SimpleNamespace(
                    inference=future, inference_started=0.0, latest_serial=0,
                    execute=False, runtime={"max_inference_s": 1.0, "max_action_step_deg": 0.5},
                    motor_limits=[(0.0, 10.0)] * 7,
                    _motor_state=lambda: current, get_logger=lambda: logger,
                )
                with patch("builtins.print") as output:
                    Pi05Runtime._finish_inference(runtime, 0.5)
                report = json.loads(output.call_args.args[0])
                self.assertEqual(report["first_action_deg"], target)
                self.assertEqual(report["current_state_deg"], current)
                self.assertEqual(report["action_guard_passed"], passed)
                self.assertEqual("guarded_delta_deg" in report, passed)
                if not passed:
                    self.assertIn("outside guarded limits", report["guard_error"])
                    if current[6] < 0:
                        self.assertIn("joint 6 (gripper)", report["guard_error"])
                self.assertIsNone(runtime.policy_error)
                logger.error.assert_not_called()

    def test_server_must_be_loopback(self):
        config = {"pi05_runtime": {"server": "http://192.168.1.2:8000"}}
        with self.assertRaisesRegex(ValueError, "local loopback"):
            _runtime_config(config)

    def test_cli_checks_selected_new_model_before_opening_devices(self):
        from zekeep_embodied import pi05_runtime

        config = {
            "ros_joystick": {}, "cameras": {"global": {}, "wrist": {}},
            "pi05_runtime": {"repo_id": "local/old", "experiment": "old", "max_action_step_deg": 0.5},
        }
        config["ros_joystick"]["arm_namespace"] = "zekeep"
        with (patch("sys.argv", ["pi05_runtime", "--config", "unused.yaml", "--shadow",
                                 "--repo-id", "local/red_cube_new", "--exp-name", "new_trial"]),
              patch.object(pi05_runtime, "load_config", return_value=(config, None)),
              patch.object(pi05_runtime, "_check_server", side_effect=RuntimeError("stop before devices")) as check,
              patch.object(pi05_runtime, "Camera") as camera):
            with self.assertRaisesRegex(RuntimeError, "stop before devices"):
                pi05_runtime.main()
        expected = check.call_args.args[2]
        self.assertEqual(expected["repo_id"], "local/red_cube_new")
        self.assertEqual(expected["experiment"], "new_trial")
        self.assertEqual(expected["max_action_step_deg"], 0.5)
        camera.assert_not_called()

    def test_missing_feedback_is_reported_without_spamming(self):
        logger = Mock()
        runtime = SimpleNamespace(
            inference=None, next_request=0, execute=False,
            last_capture_warning=float("-inf"),
            _finish_inference=Mock(),
            _capture_request=Mock(side_effect=RuntimeError("Waiting for arm and gripper feedback")),
            get_logger=lambda: logger,
        )
        Pi05Runtime._tick(runtime)
        Pi05Runtime._tick(runtime)
        self.assertIn("Waiting for arm and gripper feedback", runtime.policy_error)
        logger.warning.assert_called_once()


if __name__ == "__main__":
    unittest.main()
