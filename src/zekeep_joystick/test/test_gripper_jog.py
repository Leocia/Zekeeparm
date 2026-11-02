"""Trigger duration and controller interlocks; no ROS nodes or hardware."""

from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from zekeep_joystick.gripper_jog import GripperJog
import test_safe_home


def make_jog(rate=50.0, **overrides):
    node = Mock()
    parameters = {}
    node.declare_parameter.side_effect = lambda name, default: parameters.setdefault(
        name, overrides.get(name, default))
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=parameters[name])
    jog = GripperJog(node, "/zekeep", rate, 0.3)
    jog.last_tick = jog.feedback_time = 10.0
    jog.feedback = 0.0
    return jog


def axes(lt=1.0, rt=1.0):
    return [0.0, 0.0, 0.0, 0.0, rt, lt, 0.0, 0.0]


class GripperJogTest(unittest.TestCase):
    def test_full_travel_takes_two_seconds_in_both_directions(self):
        for rate in (15, 50):
            for opening in (True, False):
                jog = make_jog(rate)
                jog.feedback = 0.0 if opening else 1.15
                for tick in range(1, rate * 2 + 1):
                    now = 10.0 + tick / rate
                    jog.feedback_time = now
                    jog.update(axes(lt=-1.0) if opening else axes(rt=-1.0), now)
                    jog.feedback = jog.target
                    if tick == rate:
                        self.assertAlmostEqual(jog.target, 0.575)
                self.assertAlmostEqual(jog.target, 1.15 if opening else 0.0)
                jog.update(axes(lt=-1.0) if opening else axes(rt=-1.0), now + 1 / rate)
                self.assertEqual(jog.target, 1.15 if opening else 0.0)

    def test_release_holds_feedback_and_contact_cannot_queue_motion(self):
        jog = make_jog()
        jog.feedback = 0.7
        for tick in range(1, 101):
            now = 10.0 + tick / 50
            jog.feedback_time = now
            jog.update(axes(rt=-1.0), now)
            self.assertGreaterEqual(jog.target, 0.7 - .2 * jog.speed)
        jog.update(axes(), now + .02)
        self.assertEqual(jog.target, .7)
        self.assertEqual(jog.velocity, 0.0)
        jog.feedback = .69
        jog.update(axes(), now + .04)
        self.assertEqual(jog.target, .7)
        jog.update(axes(lt=-1.0, rt=-.5), now + .06)
        self.assertEqual(jog.target, .7)
        jog.update(axes(lt=-1.0), now + .08)
        self.assertGreater(jog.target, .69)
        jog.update(axes(rt=-1.0), now + .10)
        self.assertLess(jog.target, .69)

    def test_tracking_allowance_is_the_same_at_both_publish_rates(self):
        targets = []
        for rate in (15, 50):
            jog = make_jog(rate)
            jog.feedback = .7
            for tick in range(1, rate + 1):
                now = 10.0 + tick / rate
                jog.feedback_time = now
                jog.update(axes(rt=-1.0), now)
                self.assertGreaterEqual(jog.target, .7 - .2 * jog.speed)
            targets.append(jog.target)
            jog.update(axes(), now + 1 / rate)
            self.assertEqual(jog.target, .7)
        self.assertAlmostEqual(targets[0], targets[1])

    def test_release_at_endpoint_stops_at_actual_position(self):
        jog = make_jog()
        jog.feedback = 1.149
        jog.update(axes(lt=-1.0), 10.02)
        self.assertEqual(jog.target, 1.15)
        jog.update(axes(), 10.04)
        self.assertEqual(jog.target, 1.149)

    def test_idle_preserves_grasp_and_partial_trigger_is_slower(self):
        jog = make_jog()
        jog.feedback = None
        jog.update(axes(), 10.02)
        jog.update(axes(lt=-1.0, rt=-1.0), 10.04)
        message, point = SimpleNamespace(joint_names=["joint1"]), SimpleNamespace()
        jog.append(message, point)
        self.assertEqual(message.joint_names, ["joint1"])
        jog.feedback = .5
        jog.update(axes(lt=0.0), 10.06)
        self.assertGreater(jog.target, .5)
        self.assertLess(jog.target, .5 + .575 * .02)
        jog.reset()
        self.assertIsNone(jog.target)

    def test_bad_inputs_and_stale_feedback_fail_closed(self):
        for values in ([], axes(lt=float("nan")), axes(rt=float("inf")), axes(lt=1.1)):
            with self.assertRaises(ValueError):
                make_jog().update(values, 10.02)
        for feedback in (None, .5):
            jog = make_jog()
            jog.feedback = feedback
            with self.assertRaisesRegex(ValueError, "feedback"):
                jog.update(axes(lt=-1.0), 10.4)
        for parameters in (dict(gripper_speed=0), dict(gripper_speed=float("nan")),
                           dict(gripper_open_axis=4), dict(gripper_close_axis=-1),
                           dict(gripper_open_position=0.0)):
            with self.assertRaises(ValueError):
                make_jog(**parameters)

    def test_both_controllers_publish_gripper_even_while_ik_is_pending(self):
        for controller_type, mode in test_safe_home.SafeHomeTest.modes:
            node = test_safe_home.make_controller(controller_type, "active", mode)
            node.gripper_jog = make_jog(15)
            node.axes = axes(lt=-1.0)
            node.buttons = [0] * 19
            node.buttons[node.deadman_button] = 1
            node.last_joy_time = node.last_tick = 10.0
            node.joy_timeout, node.publish_rate = .3, 15.0
            node.axis_indices = node.joint_axis_indices = [0, 1, 3, 2, 6, 7]
            node.axis_signs = node.joint_axis_signs = [1.0] * 6
            node.max_speeds = node.joint_max_speeds = [.5] * 6
            node.lower_limits = node.joint_lower_limits = [-2.0] * 6
            node.upper_limits = node.joint_upper_limits = [2.0] * 6
            node.deadzone = .12
            node.joint_names = [f"joint{i}" for i in range(1, 7)]
            node.publisher = Mock()
            node.target_pose = [0.0] * 6
            node.ik_pending = True
            with patch("time.monotonic", return_value=10.05):
                node._tick()
            message = node.publisher.publish.call_args.args[0]
            self.assertEqual(message.joint_names[-1], "gripper")
            self.assertAlmostEqual(message.points[0].positions[-1], .575 * .05)
            self.assertEqual(len(node.joint_names), 6)
            self.assertEqual(len(node.target_joints), 6)
            self.assertEqual(len(node.targets), 6)
            stop = node._stop if hasattr(node, "_stop") else node._stop_stream
            with patch.object(node, stop.__name__) as stopped:
                node.buttons[node.deadman_button] = 0
                with patch("time.monotonic", return_value=10.06):
                    node._tick()
                stopped.assert_called_once_with("deadman released")
            node.buttons[node.deadman_button] = 1
            node.gripper_jog.feedback_time = 9.0
            with patch.object(node, stop.__name__) as stopped:
                with patch("time.monotonic", return_value=10.07):
                    node._tick()
                stopped.assert_called_once_with("gripper feedback missing or stale")
            node.gripper_jog.feedback_time = 10.0
            with patch.object(node, stop.__name__) as stopped:
                with patch("time.monotonic", return_value=10.4):
                    node._tick()
                stopped.assert_called_once_with("joystick timed out")

    def test_standard_configs_match_two_second_calibrated_travel(self):
        workspace = Path(__file__).resolve().parents[3]
        config_dir = workspace / "src/zekeep_joystick/config"
        hardware_dir = workspace / "src/zekeep_bringup/config"
        for name in ("zekeep_hardware.yaml", "zekeep_hardware_withnogravity.yaml"):
            hardware = yaml.safe_load((hardware_dir / name).read_text())["models"]["sixaxis"]["overrides"]
            for config in config_dir.glob("*.yaml"):
                settings = next(iter(yaml.safe_load(config.read_text()).values()))["ros__parameters"]
                limits = hardware["gripper"]["position_limits"]
                self.assertEqual(settings["gripper_open_position"], limits["open"])
                self.assertEqual(settings["gripper_close_position"], limits["close"])
                self.assertAlmostEqual((limits["open"] - limits["close"]) / settings["gripper_speed"], 2.0)
                self.assertGreaterEqual(hardware["joint_calibration"]["gripper"]["max_velocity"],
                                        settings["gripper_speed"])


if __name__ == "__main__":
    unittest.main()
