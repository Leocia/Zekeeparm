"""Exercise both joystick callbacks without starting ROS nodes or hardware."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from zekeep_joystick.cartesian_node import CartesianJoystickController
from zekeep_joystick.joystick_node import JoystickController


def make_controller(controller_type, state="idle", mode="cartesian"):
    node = object.__new__(controller_type)
    node.get_logger = Mock(return_value=Mock())
    node.state = state
    node.control_mode = mode
    node.buttons = []
    node.deadman_button, node.close_button, node.open_button = 6, 0, 1
    node.safe_home_button, node.joint_mode_button = 3, 2
    node.return_ready_modifier_button = 7
    node.require_deadman_release = False
    node.pending_after_stop = None
    node.targets = node.target_joints = [0.1] * 6
    node.target_pose = node.pending_pose = None
    node.ik_pending = False
    node.stop_client = Mock()
    node.stop_client.service_is_ready.return_value = True
    node.stop_client.call_async.return_value = Future()
    node.safe_home_client = Mock()
    node.safe_home_client.service_is_ready.return_value = True
    node.safe_home_client.call_async.return_value = Future()
    node.return_ready_client = Mock()
    node.return_ready_client.service_is_ready.return_value = True
    node.return_ready_client.call_async.return_value = Future()
    node.open_client, node.close_client = Mock(), Mock()
    return node


def press(node, *buttons):
    values = [int(index in buttons) for index in range(19)]
    node._on_joy(SimpleNamespace(axes=[0.0] * 8, buttons=values))


class SafeHomeTest(unittest.TestCase):
    modes = ((JoystickController, "joint"),
             (CartesianJoystickController, "cartesian"),
             (CartesianJoystickController, "joint"))

    def test_x_alone_triggers_once_and_requires_lb_release_after_completion(self):
        for controller_type, mode in self.modes:
            with self.subTest(controller=controller_type.__name__, mode=mode):
                node = make_controller(controller_type, mode=mode)
                press(node, 3)
                self.assertEqual(node.state, "safe_home")
                node.safe_home_client.call_async.assert_called_once()
                node.stop_client.call_async.assert_not_called()
                press(node, 3)
                node.safe_home_client.call_async.assert_called_once()
                press(node, 3, 6)
                node.safe_home_client.call_async.return_value.set_result(
                    SimpleNamespace(success=True, message="done")
                )
                self.assertEqual(node.state, "idle")
                self.assertTrue(node.require_deadman_release)
                press(node)
                self.assertFalse(node.require_deadman_release)

    def test_active_motion_stops_successfully_before_home_and_x_has_priority(self):
        for controller_type, mode in self.modes:
            for success in (True, False):
                with self.subTest(controller=controller_type.__name__, mode=mode,
                                  stop_success=success):
                    node = make_controller(controller_type, "active", mode)
                    node._request_gripper = Mock()
                    press(node, 0, 3, 6)
                    self.assertEqual(node.state, "stopping")
                    node.stop_client.call_async.assert_called_once()
                    node.safe_home_client.call_async.assert_not_called()
                    node._request_gripper.assert_not_called()
                    node.stop_client.call_async.return_value.set_result(
                        SimpleNamespace(success=success, message="stop failed")
                    )
                    self.assertEqual(node.safe_home_client.call_async.call_count, int(success))
                    self.assertIsNone(node.pending_after_stop)

    def test_unavailable_services_busy_states_and_home_failures_do_not_resume(self):
        for controller_type, mode in self.modes:
            with self.subTest(controller=controller_type.__name__, mode=mode):
                node = make_controller(controller_type, "active", mode)
                node.stop_client.service_is_ready.return_value = False
                press(node, 3, 6)
                node.safe_home_client.call_async.assert_not_called()
                self.assertIsNone(node.pending_after_stop)
                self.assertTrue(node.require_deadman_release)
                for state in ("starting", "stopping", "gripper", "safe_home"):
                    node = make_controller(controller_type, state, mode)
                    press(node, 3)
                    node.safe_home_client.call_async.assert_not_called()
                    self.assertEqual(node.state, state)
                node = make_controller(controller_type, mode=mode)
                node.safe_home_client.service_is_ready.return_value = False
                press(node, 3)
                node.safe_home_client.call_async.assert_not_called()
                self.assertEqual(node.state, "idle")
                self.assertTrue(node.require_deadman_release)
                for failure in (SimpleNamespace(success=False, message="collision"),
                                RuntimeError("service failed")):
                    node = make_controller(controller_type, mode=mode)
                    press(node, 3)
                    future = node.safe_home_client.call_async.return_value
                    if isinstance(failure, Exception):
                        future.set_exception(failure)
                    else:
                        future.set_result(failure)
                    self.assertEqual(node.state, "idle")
                    self.assertTrue(node.require_deadman_release)
                    node.get_logger().error.assert_called_once()
                node = make_controller(controller_type, mode=mode)
                node.safe_home_client.call_async.side_effect = RuntimeError("request failed")
                press(node, 3)
                self.assertEqual(node.state, "idle")
                self.assertTrue(node.require_deadman_release)
                node.get_logger().error.assert_called_once()

    def test_gripper_still_waits_for_stop_confirmation(self):
        for success in (True, False):
            node = make_controller(JoystickController, "active")
            node._command_gripper = Mock()
            press(node, 1, 6)
            node._command_gripper.assert_not_called()
            node.stop_client.call_async.return_value.set_result(
                SimpleNamespace(success=success, message="stop failed")
            )
            self.assertEqual(node._command_gripper.call_count, int(success))
            if success:
                node._command_gripper.assert_called_once_with(node.open_client, "open")

    def test_rb_x_returns_once_without_gripper_and_requires_lb_release(self):
        for controller_type, mode in self.modes:
            for initial_state in ("idle", "active"):
                with self.subTest(controller=controller_type.__name__, mode=mode,
                                  state=initial_state):
                    node = make_controller(controller_type, initial_state, mode)
                    press(node, 7)
                    node.return_ready_client.call_async.assert_not_called()
                    press(node, 3, 7)
                    if initial_state == "active":
                        node.return_ready_client.call_async.assert_not_called()
                        node.stop_client.call_async.assert_called_once()
                        node.stop_client.call_async.return_value.set_result(
                            SimpleNamespace(success=True, message="stopped")
                        )
                    self.assertEqual(node.state, "return_ready")
                    press(node, 3, 7)
                    node.return_ready_client.call_async.assert_called_once()
                    node.safe_home_client.call_async.assert_not_called()
                    node.open_client.assert_not_called()
                    node.close_client.assert_not_called()
                    press(node, 3, 6, 7)
                    node.return_ready_client.call_async.return_value.set_result(
                        SimpleNamespace(success=True, message="done")
                    )
                    self.assertEqual(node.state, "idle")
                    self.assertTrue(node.require_deadman_release)
                    press(node)
                    self.assertFalse(node.require_deadman_release)

    def test_rb_x_obeys_busy_stop_and_service_failures(self):
        for controller_type, mode in self.modes:
            with self.subTest(controller=controller_type.__name__, mode=mode):
                for state in ("starting", "stopping", "gripper", "safe_home", "return_ready"):
                    node = make_controller(controller_type, state, mode)
                    press(node, 3, 7)
                    node.return_ready_client.call_async.assert_not_called()
                    self.assertEqual(node.state, state)
                for available in (True, False):
                    node = make_controller(controller_type, "active", mode)
                    node.stop_client.service_is_ready.return_value = available
                    press(node, 3, 7)
                    if available:
                        node.stop_client.call_async.return_value.set_result(
                            SimpleNamespace(success=False, message="stop failed")
                        )
                    node.return_ready_client.call_async.assert_not_called()
                    self.assertTrue(node.require_deadman_release)
                node = make_controller(controller_type, mode=mode)
                node.return_ready_client.service_is_ready.return_value = False
                press(node, 3, 7)
                node.return_ready_client.call_async.assert_not_called()
                self.assertEqual(node.state, "idle")
                self.assertTrue(node.require_deadman_release)
                for failure in (SimpleNamespace(success=False, message="collision"),
                                RuntimeError("service failed")):
                    node = make_controller(controller_type, mode=mode)
                    press(node, 3, 7)
                    future = node.return_ready_client.call_async.return_value
                    if isinstance(failure, Exception):
                        future.set_exception(failure)
                    else:
                        future.set_result(failure)
                    self.assertEqual(node.state, "idle")
                    self.assertTrue(node.require_deadman_release)
                    node.get_logger().error.assert_called_once()

    def test_rb_alone_is_idle_and_rb_x_takes_priority_over_gripper(self):
        for controller_type, mode in self.modes:
            with self.subTest(controller=controller_type.__name__, mode=mode):
                node = make_controller(controller_type, mode=mode)
                press(node, 7)
                self.assertEqual(node.state, "idle")
                node.safe_home_client.call_async.assert_not_called()
                node.return_ready_client.call_async.assert_not_called()
                node._request_gripper = Mock()
                press(node, 0, 3, 6, 7)
                node.return_ready_client.call_async.assert_called_once()
                node.safe_home_client.call_async.assert_not_called()
                node._request_gripper.assert_not_called()

    def test_ready_modifier_rejects_conflicting_or_invalid_indices(self):
        for controller_type, mode in self.modes:
            for index in (-1, 0, 1, 3, 6):
                with self.subTest(controller=controller_type.__name__, button=index):
                    node = make_controller(controller_type, mode=mode)
                    node.return_ready_modifier_button = index
                    with self.assertRaisesRegex(ValueError, "return_ready_modifier_button"):
                        node._validate_parameters()


if __name__ == "__main__":
    unittest.main()
