"""Actual DDS/services against an in-memory driver, without SDK/device access."""

import threading
import time
import unittest
import uuid
from unittest.mock import Mock

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from zekeep_msgs.msg import ArmStatus, JointMotorState

from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
from zekeep_act.config import load_config
from zekeep_act.robot import RosArm
from zekeepcontroller.servo_stream import ServoStream
from test_servo import hardware


class FakeDriver(Node):
    def __init__(self, namespace):
        super().__init__("act_fake_driver_" + uuid.uuid4().hex[:8])
        self.hardware = hardware()
        self.hardware._state_machine = "IDLE"
        self.hardware._shutdown_pending = False
        self.hardware._arm_control_mode = "posvel"
        self.hardware.start_endpos_control = lambda: None
        self.hardware._endpos_ctrl._q_target[:] = .25
        self.web_task_gate = type("Gate", (), {"service": staticmethod(lambda callback: callback)})()
        self.reentrant_group = None
        self.root = "/" + namespace
        self.joints = self.create_publisher(JointState, self.root + "/joint_states", qos_profile_sensor_data)
        self.gripper = self.create_publisher(JointMotorState, self.root + "/gripper/state", qos_profile_sensor_data)
        self.motors = {name: self.create_publisher(JointMotorState, self.root + f"/joints/{name}/state", qos_profile_sensor_data)
                       for name in self.hardware.joint_names}
        self.status = self.create_publisher(ArmStatus, self.root + "/arm_status", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.joy = self.create_publisher(Joy, self.root + "/joy", qos_profile_sensor_data)
        self.held = False
        self.stream = ServoStream(self, self.hardware, namespace)
        self.create_timer(.02, self.publish_feedback)
        self.publish_arm_status()

    def publish_arm_status(self, **_):
        message = ArmStatus()
        message.enabled = message.control_loop_active = True
        message.state_machine = self.hardware.state_machine
        message.joint_names = self.hardware.joint_names
        message.per_joint_status_code = [1] * 6
        self.status.publish(message)

    def publish_feedback(self):
        message = JointState()
        message.name = self.hardware.joint_names
        message.position = [.25] * 6
        self.joints.publish(message)
        for name, publisher in self.motors.items():
            motor = JointMotorState()
            motor.joint_name, motor.position, motor.status_code = name, .25, 1
            publisher.publish(motor)
        motor = JointMotorState()
        motor.joint_name, motor.position, motor.status_code = "gripper", self.hardware.get_gripper_state()[0], 1
        self.gripper.publish(motor)
        joy = Joy()
        joy.buttons = [0] * 7
        joy.buttons[6] = int(self.held)
        self.joy.publish(joy)


class RosIntegrationTest(unittest.TestCase):
    def test_learned_intermediate_gripper_angles_reach_driver_and_deadman_holds_position(self):
        settings, capture, _ = load_config(DEFAULT_CONFIG)
        namespace = "act_grasp_test_" + uuid.uuid4().hex[:8]
        capture["ros_joystick"] = dict(capture["ros_joystick"], arm_namespace=namespace,
                                       joy_topic="/" + namespace + "/joy")
        rclpy.init()
        driver = FakeDriver(namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        robot = None
        try:
            robot = RosArm(settings, capture)
            state = robot.wait_ready()
            deadline = time.monotonic() + 3
            while not robot.released and time.monotonic() < deadline:
                time.sleep(.01)
            driver.held = True
            while not robot.deadman() and time.monotonic() < deadline:
                time.sleep(.01)
            robot.start(deadman=True)
            self.assertFalse(driver.hardware._servo_gripper_grasp)
            proposed = state[:6] + [28.]
            for _ in range(3):
                robot.command(proposed, time.monotonic())
                time.sleep(.03)
            self.assertIsNone(driver.hardware._gripper_grasp_state)
            self.assertAlmostEqual(driver.hardware._endpos_ctrl._gripper_target, np.radians(28.), places=4)
            proposed[6] = 29.
            for _ in range(3):
                robot.command(proposed, time.monotonic())
                time.sleep(.03)
            self.assertIsNone(driver.hardware._gripper_grasp_state)
            self.assertAlmostEqual(driver.hardware._endpos_ctrl._gripper_target, np.radians(29.), places=4)
            driver.held = False
            deadline = time.monotonic() + 1
            while (robot.active or driver.hardware.state_machine != "IDLE") and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(robot.active)
            self.assertEqual(driver.hardware.state_machine, "IDLE")
            self.assertIsNone(driver.hardware._gripper_grasp_state)
            self.assertEqual(driver.hardware._endpos_ctrl._gripper_target, .5)
        finally:
            if robot:
                robot.close()
            executor.shutdown()
            thread.join(timeout=1)
            driver.destroy_node()
            rclpy.try_shutdown()

    def test_follow_timer_runs_independently_of_10_hz_capture(self):
        settings, capture, _ = load_config(DEFAULT_CONFIG)
        namespace = "act_follow_test_" + uuid.uuid4().hex[:8]
        capture["ros_joystick"] = dict(capture["ros_joystick"], arm_namespace=namespace)
        leader = Mock()
        leader.leader.config.use_background_read = True
        leader.leader.config.max_step_deg = 12
        rclpy.init()
        driver = FakeDriver(namespace)
        driver.hardware.set_servo_target = Mock(wraps=driver.hardware.set_servo_target)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        robot = None
        try:
            robot = RosArm(settings, capture, leader=leader)
            state = robot.wait_ready()
            leader.target.return_value = (np.asarray(state) + 4).tolist()
            self.assertEqual(robot.fps, 50)
            np.testing.assert_allclose(robot.follow_speeds, [np.pi / 3] * 3 + [1.5] * 3 + [.575])
            robot.start()
            started = time.monotonic()
            for _ in range(10):
                with robot.lock:
                    robot.source_time = time.monotonic()
                time.sleep(.1)
                self.assertTrue(robot.active, robot.stop_reason)
            elapsed = time.monotonic() - started
            received = driver.hardware.set_servo_target.call_count
            print(f"\n模拟 ROS 跟随：{received / elapsed:.1f} Hz，采集心跳 10 Hz", flush=True)
            self.assertGreaterEqual(received, 30)
            self.assertGreaterEqual(leader.target.call_count, 30)
            deadline = time.monotonic() + 1
            while (robot.active or driver.hardware.state_machine != "IDLE") and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(robot.active)
            self.assertEqual(driver.hardware.state_machine, "IDLE")
            self.assertIn("timed out", robot.stop_reason)
        finally:
            if robot:
                robot.close()
            executor.shutdown()
            thread.join(timeout=1)
            driver.destroy_node()
            rclpy.try_shutdown()

    def test_keyboard_controls_seven_dimensional_commands_and_stops_with_joystick_held(self):
        settings, capture, _ = load_config(DEFAULT_CONFIG)
        namespace = "act_test_" + uuid.uuid4().hex[:8]
        capture["ros_joystick"] = dict(capture["ros_joystick"], arm_namespace=namespace, joy_topic="/" + namespace + "/joy")
        rclpy.init()
        driver = FakeDriver(namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        robot = None
        try:
            keyboard = Mock(held=False, ready=False)
            robot = RosArm(settings, capture, keyboard=keyboard)
            state = robot.wait_ready()
            with self.assertRaisesRegex(RuntimeError, "Release m"):
                robot.start(deadman=True)
            keyboard.held = keyboard.ready = True
            robot.start(deadman=True)
            self.assertTrue(robot.active)
            proposed = np.asarray(state) + .1
            for _ in range(3):
                robot.command(proposed, time.monotonic())
                time.sleep(.03)
            self.assertEqual(driver.hardware.state_machine, "SERVO_RUNNING")
            self.assertIsNotNone(driver.stream.last_command)
            driver.held = True  # Joystick state cannot keep keyboard-controlled motion alive.
            keyboard.held = False
            deadline = time.monotonic() + 1
            while (robot.active or driver.hardware.state_machine != "IDLE") and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(robot.active)
            self.assertIn("keyboard", robot.stop_reason)
            self.assertEqual(driver.hardware.state_machine, "IDLE")
            self.assertFalse(driver.hardware._servo_gripper_active)
            self.assertEqual(driver.hardware._endpos_ctrl._gripper_target, .5)
        finally:
            if robot:
                robot.close()
            executor.shutdown()
            thread.join(timeout=1)
            driver.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
