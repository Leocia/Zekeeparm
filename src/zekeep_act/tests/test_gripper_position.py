import math
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from zekeep_act.gripper import gripper_target
from zekeepcontroller.gripper_grasp import GripperGraspConfig
from zekeepcontroller.servo_stream import ServoStream
from test_robot import robot
from test_servo import hardware


SETTINGS = dict(prediction_margin_deg=10.)
LIMITS = (0., math.degrees(1.15))


def grasp_hardware(position=.9):
    hw = hardware()
    hw._state_machine = "IDLE"
    hw._shutdown_pending = False
    hw.start_endpos_control = Mock()
    hw._gripper_grasp_config = GripperGraspConfig(1.15, 0., 1., .3, 1.5, .05, .1, .3, .05, 5., 1., .5)
    hw.gripper_grasp_timeout = 5.
    hw.get_gripper_state.return_value = (position, .1, 0., 1)
    hw.begin_servo_stream(with_gripper=True, gripper_grasp=True)
    return hw


class GripperPositionTest(unittest.TestCase):
    def test_continuous_angles_are_preserved_and_only_small_endpoint_overshoots_are_bounded(self):
        for value in (26., 27., 28., 29., 33., 35., 40.):
            self.assertEqual(gripper_target(value, SETTINGS, LIMITS), value)
        self.assertEqual(gripper_target(-2.65, SETTINGS, LIMITS), 0.)
        self.assertEqual(gripper_target(72.47, SETTINGS, LIMITS), LIMITS[1])
        for value in (-10.01, 80., float("nan"), float("inf")):
            with self.assertRaisesRegex(RuntimeError, "outside bounded range"):
                gripper_target(value, SETTINGS, LIMITS)

    def test_policy_sends_learned_intermediate_angles_and_keeps_feedback_guards(self):
        arm = robot()
        arm.motor_limits[6] = LIMITS
        for target in (27., 28., 29.):
            now = time.monotonic()
            arm.target_time = now - .1
            arm.command([0.] * 6 + [target], now)
            arm._tick()
            message = arm.publisher.publish.call_args.args[0]
            self.assertAlmostEqual(message.points[0].positions[6], math.radians(target))
            self.assertTrue(all(abs(value) <= .2 for value in message.points[0].velocities[:6]))
            self.assertLessEqual(abs(message.points[0].velocities[6]), arm.gripper_speed)
        arm.gripper = math.radians(-1.01)
        with self.assertRaisesRegex(RuntimeError, "Current joint 6"):
            arm.command([0.] * 6 + [28.], time.monotonic())

    def test_open_gripper_can_follow_a_28_degree_target_without_a_threshold_switch(self):
        arm = robot()
        arm.motor_limits[6] = LIMITS
        arm.gripper = 1.15
        arm.target[6] = arm.gripper
        start = time.monotonic()
        arm.target_time = start
        for cycle in range(1, 14):
            now = start + cycle * .1
            previous = arm.gripper
            arm.arm_time = arm.gripper_time = arm.joy_time = now
            arm.motor_status = {name: (1, now) for name in arm.motor_status}
            with patch("zekeep_act.robot.time.monotonic", return_value=now):
                arm.command([0.] * 6 + [28.], now)
                arm._tick()
            self.assertTrue(arm.active, arm.stop_reason)
            self.assertLessEqual(abs(arm.target[6] - previous), arm.gripper_speed * .1 + 1e-6)
            arm.gripper = arm.target[6]
        self.assertAlmostEqual(math.degrees(arm.gripper), 28., places=5)

    def test_contact_state_is_not_restarted_and_pause_keeps_holding_torque(self):
        hw = grasp_hardware()
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        state = hw._gripper_grasp_state
        self.assertEqual(state.state, "closing")
        np.testing.assert_array_equal(hw._endpos_ctrl._q_target, np.zeros(6))
        np.testing.assert_array_equal(hw._endpos_ctrl._qd_target, np.zeros(6))
        hw.get_gripper_state.return_value = (.5, 0., 1., 1)
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        self.assertIs(hw._gripper_grasp_state, state)
        self.assertEqual(state.state, "holding")
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .1)
        hw.stop_servo_stream()
        self.assertEqual(hw.state_machine, "IDLE")
        self.assertIs(hw._gripper_grasp_state, state)
        self.assertEqual(state.update(position=.5, velocity=0., torque=1.).torque, -.3)
        hw.begin_servo_stream(with_gripper=True, gripper_grasp=True)
        hw.set_servo_target([.1] * 6, [0.] * 6, gripper_position=0.)
        self.assertIs(hw._gripper_grasp_state, state)

    def test_pause_and_watchdog_cancel_unfinished_closing(self):
        for watchdog in (False, True):
            with self.subTest(watchdog=watchdog):
                hw = grasp_hardware()
                hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
                if watchdog:
                    stream = ServoStream.__new__(ServoStream)
                    stream._lock, stream.hardware, stream.timeout = threading.Lock(), hw, .2
                    stream.node, stream.last_command = Mock(), time.monotonic() - 1
                    stream.watchdog()
                else:
                    hw.stop_servo_stream()
                self.assertIsNone(hw._gripper_grasp_state)
                self.assertEqual(hw._endpos_ctrl._gripper_target, .9)
                self.assertEqual(hw.state_machine, "IDLE")

    def test_opening_cancels_force_and_keeps_calibrated_slew_limit(self):
        hw = grasp_hardware()
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        hw.get_gripper_state.return_value = (.5, 0., 1., 1)
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        with patch("zekeepcontroller.hardware_manager.time.monotonic", return_value=10.):
            hw.set_servo_target([.2] * 6, [.1] * 6, gripper_position=1.15)
        self.assertIsNone(hw._gripper_grasp_state)
        self.assertLessEqual(hw._endpos_ctrl._gripper_target, .6)
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .1)
        hw.get_gripper_state.return_value = (1.15, 0., 0., 1)
        with patch("zekeepcontroller.hardware_manager.time.monotonic", return_value=10.1):
            hw.set_servo_target([.2] * 6, [.1] * 6, gripper_position=1.15)
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .2)

    def test_invalid_stop_feedback_keeps_latest_valid_hold_not_initial_open_target(self):
        hw = grasp_hardware()
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        hw.get_gripper_state.return_value = (.6, .1, 0., 1)
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        hw.get_gripper_state.return_value = (float("nan"), 0., 0., 1)
        hw.stop_servo_stream()
        self.assertIsNone(hw._gripper_grasp_state)
        self.assertAlmostEqual(hw._endpos_ctrl._gripper_target, .6)
        self.assertEqual(hw.state_machine, "IDLE")

    def test_closed_initial_pose_is_allowed_but_empty_grasp_and_timeout_stop(self):
        hw = grasp_hardware(0.)
        hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
        self.assertEqual(hw._gripper_grasp_state.state, "empty")
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .1)
        for failure in ("empty", "timeout"):
            with self.subTest(failure=failure):
                hw = grasp_hardware()
                hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=0.)
                if failure == "empty":
                    hw.get_gripper_state.return_value = (0., 0., 1., 1)
                else:
                    hw._servo_gripper_deadline = time.monotonic() - 1
                stream = ServoStream.__new__(ServoStream)
                stream._lock, stream.hardware, stream.node = threading.Lock(), hw, Mock()
                stream.names = tuple(hw.joint_names)
                from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
                message = JointTrajectory()
                message.joint_names = list(hw.joint_names) + ["gripper"]
                message.points = [JointTrajectoryPoint(positions=[.2] * 6 + [0.], velocities=[.1] * 6 + [0.])]
                stream.command(message)
                self.assertEqual(hw.state_machine, "IDLE")
                self.assertIsNone(hw._gripper_grasp_state)
                stream.node.get_logger.return_value.error.assert_called_once()

    def test_grasp_service_keeps_action_locks_and_rejects_nonendpoint_commands(self):
        stream = ServoStream.__new__(ServoStream)
        stream._lock, stream.hardware = threading.Lock(), Mock()
        lock = threading.Lock()
        stream.node = SimpleNamespace(arm_actions=SimpleNamespace(_gripper_execution_lock=lock),
                                      publish_arm_status=Mock())
        lock.acquire()
        result = stream.start_with_gripper_grasp(None, SimpleNamespace(success=False, message=""))
        self.assertFalse(result.success)
        stream.hardware.begin_servo_stream.assert_not_called()
        lock.release()
        result = stream.start_with_gripper_grasp(None, SimpleNamespace(success=False, message=""))
        self.assertTrue(result.success)
        stream.hardware.begin_servo_stream.assert_called_once_with(with_gripper=True, gripper_grasp=True)
        hw = grasp_hardware()
        for value in (-.01, .5, 1.2, float("nan")):
            with self.assertRaises(ValueError):
                hw.set_servo_target([.2] * 6, [.1] * 6, gripper_position=value)
            np.testing.assert_array_equal(hw._endpos_ctrl._q_target, np.zeros(6))


if __name__ == "__main__":
    unittest.main()
