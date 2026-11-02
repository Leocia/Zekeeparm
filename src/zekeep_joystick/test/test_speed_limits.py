"""Check matching angular speeds and actual IK target increments offline."""

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from zekeep_joystick.cartesian_node import CartesianJoystickController
import test_safe_home


def cartesian_controller():
    node = test_safe_home.make_controller(CartesianJoystickController, "active")
    node.target_joints = [0.0] * 6
    node.target_pose = [0.0] * 6
    node.pending_pose = [0.0, 0.0, 0.0, .1, 0.0, 0.0]
    node.axes = [0.0] * 4 + [1.0, 1.0, 0.0, 0.0]
    node.buttons = [0] * 19
    node.buttons[node.deadman_button] = 1
    node.last_tick = node.last_joy_time = 10.0
    node.publish_rate, node.joy_timeout = 15.0, .3
    node.max_joint_speed, node.max_joint_step = .5, .15
    node.axis_indices = [1, 0, 3, 7, 6, 2]
    node.axis_signs = [1.0] * 6
    node.max_speeds = [.10] * 3 + [.5] * 3
    node.deadzone = .12
    node.joint_names = [f"joint{i}" for i in range(1, 7)]
    node.publisher = Mock()
    node.gripper_jog = Mock()
    node._warn = Mock()
    return node


class SpeedLimitsTest(unittest.TestCase):
    def test_profiles_use_the_same_angular_speed(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        for path in config_dir.glob("*.yaml"):
            settings = next(iter(yaml.safe_load(path.read_text()).values()))["ros__parameters"]
            if "joint_max_speeds" in settings:
                self.assertEqual(settings["max_speeds"][3:], [.5] * 3)
                self.assertEqual(settings["joint_max_speeds"], [.5] * 6)
                self.assertEqual(settings["max_joint_speed"], .5)
                self.assertEqual(settings["max_speeds"][:3], [.10] * 3)
            else:
                self.assertEqual(settings["max_speeds"], [.5] * 6)
            self.assertEqual(settings["gripper_speed"], .575)

    def test_ik_limits_positions_and_velocities_and_finishes_the_exact_solution(self):
        node = cartesian_controller()
        solution = [.1, .05, 0.0, -.025, 0.0, 0.0]
        pose = list(node.pending_pose)
        future = Future()
        future.set_result(SimpleNamespace(success=True, message="", q_solution=solution))
        node._on_ik(future)
        node.publisher.publish.assert_not_called()
        now = 10.0
        for elapsed in (1 / 15, 1 / 15, .8, 1 / 15, 1 / 15, 1 / 15, 1 / 15):
            previous = list(node.target_joints)
            now += elapsed
            node.last_joy_time = now
            with patch("time.monotonic", return_value=now):
                node._tick()
            point = node.publisher.publish.call_args.args[0].points[0]
            allowed_step = .5 * min(elapsed, 2 / 15)
            self.assertLessEqual(max(abs(q - old) for q, old in zip(point.positions, previous)),
                                 allowed_step + 1e-12)
            self.assertLessEqual(max(abs(v) for v in point.velocities), .5 + 1e-12)
            self.assertAlmostEqual(point.positions[1], point.positions[0] / 2)
            if node.ik_solution is None:
                break
            self.assertEqual(node.target_pose, [0.0] * 6)
        self.assertIsNone(node.ik_solution)
        self.assertIsNone(node.pending_pose)
        self.assertEqual(node.target_pose, pose)
        for actual, expected in zip(node.target_joints, solution):
            self.assertAlmostEqual(actual, expected)
        node.gripper_jog.update.assert_called()

    def test_stop_discards_queued_solution_and_invalid_ik_still_fails(self):
        node = cartesian_controller()
        node.ik_solution = [.1] * 6
        node._stop("deadman released")
        self.assertIsNone(node.ik_solution)
        self.assertIsNone(node.pending_pose)
        node.publisher.publish.assert_not_called()
        for response in (
            SimpleNamespace(success=False, message="no solution", q_solution=[]),
            SimpleNamespace(success=True, message="", q_solution=[float("nan")] * 6),
            SimpleNamespace(success=True, message="", q_solution=[.151] * 6),
        ):
            node = cartesian_controller()
            future = Future()
            future.set_result(response)
            node._on_ik(future)
            self.assertIsNone(node.ik_solution)
            self.assertIsNone(node.pending_pose)
            node.publisher.publish.assert_not_called()
            node._warn.assert_called_once()


if __name__ == "__main__":
    unittest.main()
