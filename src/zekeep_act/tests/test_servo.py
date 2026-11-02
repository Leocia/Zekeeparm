import threading
from types import SimpleNamespace
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from zekeepcontroller.hardware_manager import HardwareManager
from zekeepcontroller.joint_mapping import JointCalibration, JointMapping
from zekeepcontroller.servo_stream import ServoStream


def hardware():
    result = HardwareManager.__new__(HardwareManager)
    result._cmd_lock = threading.RLock()
    result._robot = SimpleNamespace(has_gripper=True)
    result._arm_group = SimpleNamespace(joint_names=[f"joint{i}" for i in range(1, 7)])
    result._joint_mapping = JointMapping([
        JointCalibration(name, hard_lower=-2, hard_upper=2, max_velocity=.5)
        for name in result.joint_names])
    result._gripper_mapping = JointMapping([
        JointCalibration("gripper", hard_lower=0, hard_upper=1.57, max_velocity=.5)])
    result.gripper_close_position, result.gripper_open_position = 0, 1.15
    result._gripper_grasp_state = None
    result._endpos_ctrl = SimpleNamespace(_q_target=np.zeros(6), _qd_target=np.zeros(6), _gripper_target=.5)
    result._endpos_ctrl.set_gripper_target = lambda value: setattr(result._endpos_ctrl, "_gripper_target", value)
    result._state_machine = "SERVO_RUNNING"
    result._servo_gripper_active = True
    result._servo_gripper_time = 10
    result.get_gripper_state = Mock(return_value=(.5, 0, 0, 1))
    result.get_joint_positions = Mock(return_value=np.full(6, .25))
    result.set_state_machine = lambda value: setattr(result, "_state_machine", value)
    return result


class ServoTest(unittest.TestCase):
    def test_all_modes_are_checked_before_any_vendor_wide_enable(self):
        for hybrid in (False, True):
            for gripper_ok in (False, True):
                with self.subTest(hybrid=hybrid, gripper_ok=gripper_ok):
                    hw = hardware()
                    hw._robot.control_loop_active = False
                    hw._enabled = False
                    hw._arm_control_mode = "posvel"
                    hw._arm_pos_vel_vlim = np.full(6, .5)
                    hw._gravity_feedforward_mask = [False] * 6
                    events = []

                    def group(name, ok=True):
                        def mode(**_):
                            events.append(name + ":mode")
                            return ok

                        return SimpleNamespace(
                            joint_names=hw.joint_names, mode_pos_vel=mode, mode_mit=mode,
                            enable=lambda: events.append(name + ":enable"))

                    names = hw.joint_names
                    hw._arm_group = group("arm")
                    hw._gripper_group = group("gripper", gripper_ok)
                    hw._hybrid_arm_groups = {name: group(name) for name in names} if hybrid else {}
                    if gripper_ok:
                        hw._configure_groups_for_endpos()
                        last_mode = max(i for i, event in enumerate(events) if event.endswith(":mode"))
                        first_enable = min(i for i, event in enumerate(events) if event.endswith(":enable"))
                        self.assertLess(last_mode, first_enable)
                    else:
                        with self.assertRaisesRegex(RuntimeError, "gripper did not enter mit mode"):
                            hw._configure_groups_for_endpos()
                        self.assertFalse(any(event.endswith(":enable") for event in events))

    def test_closed_feedback_tolerance_keeps_start_stop_and_commands_nonnegative(self):
        hw = hardware()
        hw._state_machine = "IDLE"
        hw._shutdown_pending = False
        hw.start_endpos_control = Mock()
        hw.get_gripper_state.return_value = (math.radians(-.667), 0., 0., 1)
        hw.begin_servo_stream(with_gripper=True)
        self.assertEqual(hw._endpos_ctrl._gripper_target, 0.)
        hw.set_servo_target([0.] * 6, [0.] * 6, gripper_position=0.)
        self.assertEqual(hw._endpos_ctrl._gripper_target, 0.)
        hw.stop_servo_stream()
        self.assertEqual(hw._endpos_ctrl._gripper_target, 0.)
        self.assertEqual(hw.state_machine, "IDLE")

    def test_closed_feedback_tolerance_rejects_larger_errors_and_negative_commands(self):
        for position in (math.radians(-1.01), 1.58, float("nan")):
            hw = hardware()
            hw.get_gripper_state.return_value = (position, 0., 0., 1)
            with self.assertRaises(ValueError):
                hw.stop_gripper_grasp()
            self.assertEqual(hw._endpos_ctrl._gripper_target, .5)
        hw = hardware()
        with self.assertRaises(ValueError):
            hw.set_servo_target([0.] * 6, [0.] * 6, gripper_position=math.radians(-.1))

    def test_seven_joint_start_excludes_gripper_action_race(self):
        stream = ServoStream.__new__(ServoStream)
        stream._lock, stream.hardware = threading.Lock(), Mock()
        action_lock = threading.Lock()
        task_lock = threading.Lock()
        stream.node = SimpleNamespace(
            arm_actions=SimpleNamespace(_gripper_execution_lock=action_lock),
            task_actions=SimpleNamespace(_gripper_execution_lock=task_lock),
            publish_arm_status=Mock())

        def begin(**_):
            self.assertFalse(action_lock.acquire(blocking=False))
            self.assertFalse(task_lock.acquire(blocking=False))

        stream.hardware.begin_servo_stream.side_effect = begin
        response = stream.start_with_gripper(None, SimpleNamespace(success=False, message=""))
        self.assertTrue(response.success)
        self.assertFalse(action_lock.locked())
        self.assertFalse(task_lock.locked())

    def test_six_joint_commands_keep_existing_gripper_target(self):
        hw = hardware()
        hw.set_servo_target([.1] * 6, [1.] * 6)
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .1)
        np.testing.assert_allclose(hw._endpos_ctrl._qd_target, .5)
        self.assertEqual(hw._endpos_ctrl._gripper_target, .5)

    def test_invalid_gripper_never_partially_commits_arm(self):
        for position in (1.2, float("nan"), -.1):
            hw = hardware()
            with self.assertRaises(ValueError):
                hw.set_servo_target([.1] * 6, [0.] * 6, gripper_position=position)
            np.testing.assert_array_equal(hw._endpos_ctrl._q_target, np.zeros(6))
            self.assertEqual(hw._endpos_ctrl._gripper_target, .5)

    def test_gripper_position_is_slew_limited_and_watchdog_holds_both(self):
        hw = hardware()
        with patch("zekeepcontroller.hardware_manager.time.monotonic", return_value=10.1):
            hw.set_servo_target([.1] * 6, [.1] * 6, gripper_position=1.)
        self.assertAlmostEqual(hw._endpos_ctrl._gripper_target, .55)
        stream = ServoStream.__new__(ServoStream)
        stream._lock, stream.hardware, stream.timeout = threading.Lock(), hw, .2
        stream.node, stream.last_command = Mock(), 10.1
        with patch("zekeepcontroller.servo_stream.time.monotonic", return_value=10.4):
            stream.watchdog()
        self.assertEqual(hw.state_machine, "IDLE")
        np.testing.assert_allclose(hw._endpos_ctrl._q_target, .25)
        np.testing.assert_array_equal(hw._endpos_ctrl._qd_target, np.zeros(6))
        self.assertEqual(hw._endpos_ctrl._gripper_target, .5)
        self.assertIsNone(stream.last_command)

    def test_stop_keeps_last_valid_targets_if_feedback_fails(self):
        hw = hardware()
        hw.get_gripper_state.side_effect = RuntimeError("feedback lost")
        hw.get_joint_positions.side_effect = RuntimeError("feedback lost")
        hw.stop_servo_stream()
        self.assertEqual(hw.state_machine, "IDLE")
        self.assertEqual(hw._endpos_ctrl._gripper_target, .5)
        np.testing.assert_array_equal(hw._endpos_ctrl._q_target, np.zeros(6))

    def test_seven_joint_start_takes_gripper_before_first_command(self):
        hw = hardware()
        hw._state_machine = "IDLE"
        hw._shutdown_pending = False
        hw.start_endpos_control = Mock()
        hw._gripper_grasp_state = SimpleNamespace(contact_position=.4)
        hw.begin_servo_stream(with_gripper=True)
        self.assertIsNone(hw._gripper_grasp_state)
        self.assertTrue(hw._servo_gripper_active)
        hw.stop_servo_stream()
        self.assertEqual(hw._endpos_ctrl._gripper_target, .5)

    def test_named_seven_joint_command_reorders_without_losing_gripper(self):
        stream = ServoStream.__new__(ServoStream)
        stream._lock, stream.hardware, stream.node = threading.Lock(), Mock(), Mock()
        stream.names = tuple(f"joint{i}" for i in range(1, 7))
        stream.hardware.state_machine = "SERVO_RUNNING"
        message = JointTrajectory()
        message.joint_names = ["gripper"] + list(reversed(stream.names))
        point = JointTrajectoryPoint()
        point.positions, point.velocities = [0.8, 6., 5., 4., 3., 2., 1.], [0.] * 7
        message.points = [point]
        stream.command(message)
        stream.hardware.set_servo_target.assert_called_once_with([1., 2., 3., 4., 5., 6.], [0.] * 6, gripper_position=.8)
        stream.hardware.stop_servo_stream.assert_not_called()


if __name__ == "__main__":
    unittest.main()
