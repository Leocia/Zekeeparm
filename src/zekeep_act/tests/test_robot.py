from concurrent.futures import Future
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml
from builtin_interfaces.msg import Time

from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
from zekeep_act.config import load_config
from zekeep_act.robot import RosArm
from zekeep_embodied.record import ROS_ARM_JOINTS, _load_ros_calibration, _ros_to_motor_degrees


def robot():
    settings, capture, _ = load_config(DEFAULT_CONFIG)
    result = RosArm.__new__(RosArm)
    result.control, result.ros = settings["control"], capture["ros_joystick"]
    result.gripper_settings = settings["gripper"]
    result.gripper_speed = .575
    result.fps = capture["fps"]
    result.leader = None
    result.keyboard = None
    result.command_count = 0
    result.lock, result.node = threading.RLock(), Mock()
    result.node.count_publishers.return_value = 1
    result.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    now = time.monotonic()
    result.arm, result.gripper = [0.] * 6, .5
    result.arm_time = result.gripper_time = result.status_time = result.joy_time = now
    result.motor_status = {name: (1, now) for name in ROS_ARM_JOINTS}
    result.gripper_status = 1
    result.status = SimpleNamespace(enabled=True, control_loop_active=True, error_codes=[], state_machine="SERVO_RUNNING")
    result.calibrations = [dict(direction=1., gear_ratio=1., encoder_zero=0., ros_zero=0.)] * 7
    result.motor_limits = [(-90, 90)] * 7
    result.deadman_required = True
    result.released = True
    result.buttons = [0] * 7
    result.buttons[result.ros["deadman_button"]] = 1
    result.active = result.owns_servo = True
    result.starting = False
    result.generation = 1
    result.target = [0.] * 6 + [.5]
    result.target_time = result.source_time = result.started_time = now - .01
    result.stop_future = None
    result.stop_reason = ""
    result.services = {"stop": Mock()}
    result.publisher = Mock()
    result.root = "/test_act_only"
    return result


class RobotSafetyTest(unittest.TestCase):
    def test_keyboard_replaces_joystick_and_stops_on_release_or_monitor_loss(self):
        for held in (False, True):
            arm = robot()
            arm.keyboard = Mock(held=held, ready=held)
            arm.buttons = []
            arm.joy_time = 0
            arm._tick()
            self.assertEqual(arm.active, held)
            if held:
                arm.keyboard.held = False
                arm.publisher.publish.reset_mock()
                arm._tick()
            self.assertFalse(arm.active)
            arm.publisher.publish.assert_not_called()
            arm.keyboard.disarm.assert_called_once()
            arm.services["stop"].call_async.assert_called_once()

    def test_keyboard_start_requires_release_and_rechecks_hold_after_service(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"
        arm.keyboard = Mock(held=True, ready=False)
        arm._call = Mock()
        with self.assertRaisesRegex(RuntimeError, "Release m"):
            arm.start(deadman=True)
        arm._call.assert_not_called()
        arm.keyboard.ready = True

        def call(name, generation=None):
            if name == "start_with_gripper":
                arm.keyboard.held = False

        arm._call = call
        with self.assertRaisesRegex(RuntimeError, "canceled"):
            arm.start(deadman=True)
        self.assertFalse(arm.active)
        arm.keyboard.disarm.assert_called_once()

    def test_follow_updates_at_50_hz_between_camera_samples(self):
        arm = robot()
        arm.fps = 50
        arm.leader = Mock()
        arm.leader.leader.config.max_step_deg = 12
        arm.leader.target.return_value = [10.] * 6 + [40.]
        arm.follow_speeds = [math.radians(60)] * 3 + [1.5] * 3 + [.5]
        start = time.monotonic()
        arm.target_time = arm.source_time = start
        for cycle in range(1, 7):
            now = start + cycle / 50
            arm.arm_time = arm.gripper_time = arm.joy_time = now
            arm.motor_status = {name: (1, now) for name in ROS_ARM_JOINTS}
            with patch("zekeep_act.robot.time.monotonic", return_value=now):
                arm._tick()
        self.assertTrue(arm.active, arm.stop_reason)
        self.assertEqual(arm.leader.target.call_count, 6)
        self.assertEqual(arm.publisher.publish.call_count, 6)
        self.assertAlmostEqual(math.degrees(arm.target[0]), 7.2, places=5)
        self.assertAlmostEqual(arm.target[6] - .5, .5 * .12, places=5)
        velocity = arm.publisher.publish.call_args.args[0].points[0].velocities
        self.assertAlmostEqual(velocity[0], math.radians(60))
        self.assertLessEqual(velocity[6], .5)
        # Fresh leader/feedback must not hide a stalled camera/recording loop.
        arm.source_time = start - 1
        arm.publisher.publish.reset_mock()
        arm._tick()
        self.assertFalse(arm.active)
        arm.publisher.publish.assert_not_called()

    def test_policy_doubles_target_progress_and_preserves_step_and_speed_limits(self):
        for dt in (.1, .02):
            progress = []
            for scale in (1, 2):
                arm = robot()
                arm.control = dict(arm.control, max_step_deg=.5 * scale, max_velocity_rad_s=.1 * scale)
                now = time.monotonic()
                arm.target_time = now - dt
                with patch("zekeep_act.robot.time.monotonic", return_value=now):
                    arm.command([10.] * 6 + [40.], now)
                    arm._tick()
                progress.append(arm.target[0])
                self.assertLessEqual(abs(math.degrees(arm.target[0])), .5 * scale + 1e-6)
                velocities = arm.publisher.publish.call_args.args[0].points[0].velocities
                self.assertTrue(all(abs(value) <= .1 * scale for value in velocities[:6]))
                self.assertLessEqual(abs(velocities[6]), arm.gripper_speed)
            self.assertAlmostEqual(progress[1], 2 * progress[0], places=5)

    def test_follow_error_stops_instead_of_republishing_old_target(self):
        arm = robot()
        arm.leader = Mock()
        arm.leader.target.side_effect = ConnectionError("leader disconnected")
        arm._tick()
        self.assertFalse(arm.active)
        self.assertIn("leader disconnected", arm.stop_reason)
        arm.publisher.publish.assert_not_called()

    def test_joint3_teleop_limits_include_calibrated_zero_margin(self):
        _, capture, _ = load_config(DEFAULT_CONFIG)
        calibrations, _, _ = _load_ros_calibration(capture["ros_joystick"]["hardware_config"])
        calibration = calibrations[2]
        hard = sorted(_ros_to_motor_degrees(
            [calibration["hard_lower"], calibration["hard_upper"]], [calibration] * 2))
        teleop = yaml.safe_load(Path(capture["teleop_config"]).read_text())
        low, high = teleop["robot"]["joint_limits"]["elbow_flex"]
        self.assertLessEqual(low, .011)
        self.assertLessEqual(.011, high)
        self.assertGreaterEqual(low, hard[0])
        self.assertLessEqual(high, hard[1])

    def test_start_rejects_invalid_feedback_before_servo_services(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"
        arm.motor_limits[2] = (-90, 0)
        arm.arm[2] = math.radians(.011)
        arm._call = Mock()
        with self.assertRaisesRegex(RuntimeError, "Current joint 2"):
            arm.start()
        arm._call.assert_not_called()
        self.assertFalse(arm.active)

    def test_gripper_tolerance_starts_and_commands_without_negative_targets(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"
        arm.motor_limits[6] = (0., 65.89)
        arm.gripper = math.radians(-.667)
        arm._call = Mock()
        arm.start()
        self.assertTrue(arm.active)
        self.assertEqual(arm.target[-1], 0.)
        self.assertAlmostEqual(arm.observation()[-1], -.667)
        self.assertEqual(arm.command([0.] * 7, time.monotonic())[-1], 0.)
        self.assertEqual(arm.command([0.] * 6 + [-.043], time.monotonic())[-1], 0.)
        with self.assertRaisesRegex(RuntimeError, "outside bounded range"):
            arm.command([0.] * 6 + [-10.001], time.monotonic())

    def test_gripper_beyond_tolerance_is_rejected_before_servo_services(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"
        arm.motor_limits[6] = (0., 65.89)
        arm.gripper = math.radians(-1.01)
        arm._call = Mock()
        with self.assertRaisesRegex(RuntimeError, "Current joint 6"):
            arm.start()
        arm._call.assert_not_called()

    def test_gripper_feedback_after_start_acknowledgment_is_rechecked(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"
        arm.motor_limits[6] = (0., 65.89)
        arm.gripper = math.radians(-.667)

        def call(name, generation=None):
            if name == "start_with_gripper":
                arm.gripper = math.radians(-1.01)

        arm._call = call
        with self.assertRaisesRegex(RuntimeError, "Current joint 6"):
            arm.start()
        self.assertFalse(arm.active)
        self.assertIsNone(arm.target)

    def test_reported_joint3_feedback_can_command_toward_zero(self):
        arm = robot()
        arm.motor_limits[2] = (-211.99, .57)
        arm.calibrations = [dict(value) for value in arm.calibrations]
        arm.calibrations[2]["direction"] = -1
        arm.arm[2] = arm.target[2] = -math.radians(.011)
        proposed = arm.observation()
        proposed[2] = 0
        result = arm.command(proposed, time.monotonic(), clip=True)
        self.assertGreaterEqual(result[2], 0)
        self.assertLess(result[2], .011)
        arm.publisher.publish.assert_not_called()

    def test_start_initializes_from_feedback_after_service_round_trip(self):
        arm = robot()
        arm.active = arm.owns_servo = False
        arm.status.state_machine = "IDLE"

        def call(name, generation=None):
            if name == "start_with_gripper":
                arm.arm = [.25] * 6
                arm.gripper = .75

        arm._call = call
        arm.start()
        self.assertEqual(arm.target, [.25] * 6 + [.75])

    def test_deadman_release_stops_without_disabling_or_publishing(self):
        arm = robot()
        arm.buttons = [0] * 7
        arm._tick()
        self.assertFalse(arm.active)
        self.assertFalse(arm.released)
        arm.publisher.publish.assert_not_called()
        arm.services["stop"].call_async.assert_called_once()

    def test_fresh_feedback_does_not_mask_stale_policy(self):
        arm = robot()
        arm.source_time = time.monotonic() - 1
        arm._tick()
        self.assertFalse(arm.active)
        self.assertIn("timed out", arm.stop_reason)

    def test_motor_fault_stops_even_with_empty_software_errors(self):
        arm = robot()
        arm.motor_status["joint3"] = (8, time.monotonic())
        arm._tick()
        self.assertFalse(arm.active)
        self.assertIn("joint3", arm.stop_reason)
        arm.publisher.publish.assert_not_called()

    def test_prediction_after_stop_is_never_applied(self):
        arm = robot()
        arm.stop("operator pause during inference")
        with self.assertRaises(RuntimeError):
            arm.command([0.] * 7, time.monotonic())
        arm.publisher.publish.assert_not_called()

    def test_delayed_start_acknowledgment_is_stopped_after_cancellation(self):
        arm = robot()
        arm.control = dict(arm.control, service_timeout_s=.001)
        future = Future()
        start = Mock()
        start.call_async.return_value = future
        arm.services["start_with_gripper"] = start
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            arm._call("start_with_gripper", generation=arm.generation)
        arm.stop("start timed out")
        future.set_result(SimpleNamespace(success=True))
        self.assertEqual(arm.services["stop"].call_async.call_count, 2)

    def test_another_servo_publisher_cancels_active_control(self):
        arm = robot()
        arm.node.count_publishers.return_value = 2
        arm._tick()
        self.assertFalse(arm.active)
        arm.publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
