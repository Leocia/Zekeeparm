"""One ROS servo owner with an independent deadman and freshness watchdog."""

import math
import threading
import time

import numpy as np
import rclpy
import yaml
from pathlib import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.msg import ArmStatus, JointMotorState

from zekeep_embodied.record import ROS_ARM_JOINTS, _load_ros_calibration, _ros_to_motor_degrees
from zekeep_embodied.ros_coordinates import guard_action, motor_degrees_to_ros
from zekeep_embodied.storage import JOINTS, vector

from .config import positive
from .gripper import gripper_target


class RosArm:
    def __init__(self, settings, capture, *, readonly=False, leader=None, keyboard=None):
        self.control = settings["control"]
        self.gripper_settings = settings["gripper"]
        self.fps = capture["fps"]
        self.leader = leader
        self.keyboard = keyboard
        self.command_count = 0
        self.ros = capture["ros_joystick"]
        self.calibrations, opened, closed = _load_ros_calibration(self.ros["hardware_config"])
        self.motor_limits = []
        for index, calibration in enumerate(self.calibrations):
            bounds = [calibration["hard_lower"], calibration["hard_upper"]]
            if index == 6:
                bounds = [max(min(opened, closed), bounds[0]), min(max(opened, closed), bounds[1])]
            motor = _ros_to_motor_degrees(bounds, [calibration] * 2)
            self.motor_limits.append([min(motor), max(motor)])
        teleop = yaml.safe_load(Path(capture["teleop_config"]).read_text())
        hardware = yaml.safe_load(Path(self.ros["hardware_config"]).read_text())
        limits = hardware["models"][hardware["default_model"]]["overrides"]["joint_calibration"]
        self.gripper_speed = limits["gripper"]["max_velocity"]
        positive(self.gripper_speed, "hardware.gripper.max_velocity")
        if leader is not None:
            if not leader.leader.config.use_background_read:
                raise ValueError("ROS following requires the leader background reader")
            self.fps = teleop["fps"]
            positive(self.fps, "teleop.fps")
            velocities = vector(teleop["robot"]["pos_vel_velocity"])
            self.follow_speeds = []
            for name, velocity, calibration in zip(ROS_ARM_JOINTS + ("gripper",), velocities, self.calibrations):
                positive(velocity, f"teleop.{name}.velocity")
                maximum = limits[name]["max_velocity"]
                positive(maximum, f"hardware.{name}.max_velocity")
                self.follow_speeds.append(min(math.radians(velocity) * calibration["gear_ratio"], maximum))
        for index, name in enumerate(JOINTS):
            low, high = teleop["robot"]["joint_limits"][name]
            self.motor_limits[index] = [max(low, self.motor_limits[index][0]),
                                        min(high, self.motor_limits[index][1])]
            if self.motor_limits[index][0] >= self.motor_limits[index][1]:
                raise ValueError(f"No common calibrated joint range for {name}")
        self.lock = threading.RLock()
        self.arm = self.gripper = self.status = None
        self.arm_time = self.gripper_time = self.status_time = self.joy_time = 0.0
        self.gripper_status = 0
        self.motor_status = {}
        self.buttons = []
        self.released = False
        self.active = self.owns_servo = self.starting = False
        self.deadman_required = False
        self.generation = 0
        self.target = None
        self.target_time = self.source_time = 0.0
        self.started_time = 0.0
        self.stop_reason = ""
        self.stop_future = None
        self.own_context = not rclpy.ok()
        if self.own_context:
            # Keep DDS alive while KeyboardInterrupt cleanup sends the holding stop.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("zekeep_act")
        self.root = "/" + self.ros["arm_namespace"].strip("/")
        for message, topic, callback in (
            (JointState, "/joint_states", self._on_arm),
            (JointMotorState, "/gripper/state", self._on_gripper),
        ):
            self.node.create_subscription(message, self.root + topic, callback, qos_profile_sensor_data)
        self.node.create_subscription(
            ArmStatus, self.root + "/arm_status", self._on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        if keyboard is None:
            self.node.create_subscription(Joy, self.ros.get("joy_topic", "/joy"), self._on_joy, qos_profile_sensor_data)
        for name in ROS_ARM_JOINTS:
            self.node.create_subscription(JointMotorState, self.root + f"/joints/{name}/state",
                                          self._on_motor, qos_profile_sensor_data)
        self.publisher = (None if readonly else self.node.create_publisher(
            JointTrajectory, self.root + "/servo_joint_trajectory", 10))
        self.services = {name: self.node.create_client(Trigger, self.root + "/servo/" + name)
                         for name in ("start_with_gripper", "stop",
                                      "require_posvel", "require_gripper_stream")}
        self.services["enable"] = self.node.create_client(Trigger, self.root + "/enable")
        self.node.create_timer(1 / self.fps if leader is not None else 0.02, self._tick)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_arm(self, message):
        by_name = dict(zip(message.name, message.position))
        if all(name in by_name for name in ROS_ARM_JOINTS):
            values = [float(by_name[name]) for name in ROS_ARM_JOINTS]
            if all(math.isfinite(value) for value in values):
                with self.lock:
                    self.arm, self.arm_time = values, time.monotonic()

    def _on_gripper(self, message):
        if math.isfinite(message.position):
            with self.lock:
                self.gripper, self.gripper_time = float(message.position), time.monotonic()
                self.gripper_status = int(message.status_code)

    def _on_status(self, message):
        with self.lock:
            self.status, self.status_time = message, time.monotonic()

    def _on_motor(self, message):
        with self.lock:
            self.motor_status[message.joint_name] = (int(message.status_code), time.monotonic())

    def _on_joy(self, message):
        with self.lock:
            self.buttons, self.joy_time = list(message.buttons), time.monotonic()
            if not self.deadman():
                self.released = True

    def deadman(self):
        if self.keyboard is not None:
            return self.keyboard.held
        index = self.ros["deadman_button"]
        return (time.monotonic() - self.joy_time <= self.control["joy_timeout_s"]
                and index < len(self.buttons) and bool(self.buttons[index]))

    def observation(self, *, enabled=True):
        with self.lock:
            now = time.monotonic()
            if (self.arm is None or self.gripper is None or self.status is None
                    or now - min(self.arm_time, self.gripper_time) > self.control["feedback_timeout_s"]):
                raise RuntimeError("ROS arm/gripper/status feedback is missing or stale")
            if self.status.error_codes or self.gripper_status not in (0, 1):
                raise RuntimeError("Robot reports a motor fault")
            for name in ROS_ARM_JOINTS:
                code, received = self.motor_status.get(name, (-1, 0.0))
                if code not in (0, 1) or now - received > self.control["feedback_timeout_s"]:
                    raise RuntimeError(f"Motor feedback missing, stale or faulted: {name}")
            if enabled and (not self.status.enabled or not self.status.control_loop_active):
                raise RuntimeError("Manually enable the ROS driver before motion")
            return _ros_to_motor_degrees(self.arm + [self.gripper], self.calibrations)

    def wait_ready(self):
        deadline = time.monotonic() + self.ros["connect_timeout_s"]
        error = None
        while time.monotonic() < deadline:
            try:
                return self.observation(enabled=False)
            except RuntimeError as exc:
                error = exc
                time.sleep(0.02)
        raise RuntimeError(str(error))

    def _call(self, name, generation=None):
        client = self.services[name]
        timeout = self.control["service_timeout_s"]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Missing {client.srv_name}; rebuild/start the updated driver")
        event = threading.Event()
        future = client.call_async(Trigger.Request())

        def complete(result):
            if name == "start_with_gripper":
                with self.lock:
                    if generation != self.generation:
                        self.services["stop"].call_async(Trigger.Request())
            event.set()

        future.add_done_callback(complete)
        if not event.wait(timeout):
            raise RuntimeError(f"servo/{name} timed out")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)

    def start(self, *, deadman=False):
        with self.lock:
            if self.publisher is None:
                raise RuntimeError("Read-only robot cannot start motion")
            if self.active or self.starting or (self.stop_future is not None and not self.stop_future.done()):
                raise RuntimeError("Wait for the previous servo operation to stop")
            state = self.observation()
            hold = list(state)
            hold[6] = max(self.motor_limits[6][0], hold[6])
            guard_action(state, hold, self.motor_limits, self.control["max_step_deg"],
                         gripper_lower_tolerance_deg=self.control.get("gripper_lower_tolerance_deg", 0.0))
            if self.status.state_machine != "IDLE":
                raise RuntimeError(f"Driver is busy: {self.status.state_machine}")
            if self.node.count_publishers(self.root + "/servo_joint_trajectory") != 1:
                raise RuntimeError("Another servo publisher is active")
            ready = self.keyboard.ready if self.keyboard is not None else self.released and self.deadman()
            if deadman and not ready:
                key = "m" if self.keyboard is not None else "LB"
                raise RuntimeError(f"Release {key} once, then hold {key} before starting")
            self.deadman_required = deadman
            self.starting = True
            self.stop_reason = ""
            self.generation += 1
            generation = self.generation
        try:
            self._call("require_posvel")
            self._call("require_gripper_stream")
            with self.lock:
                if generation != self.generation:
                    raise RuntimeError("Servo start canceled")
                self.owns_servo = True
                self.started_time = time.monotonic()
            self._call("start_with_gripper", generation)
            with self.lock:
                if generation != self.generation or (deadman and not self.deadman()):
                    raise RuntimeError("Servo start canceled")
                state = self.observation()
                hold = list(state)
                hold[6] = max(self.motor_limits[6][0], hold[6])
                hold = guard_action(state, hold, self.motor_limits, self.control["max_step_deg"],
                                    gripper_lower_tolerance_deg=self.control.get("gripper_lower_tolerance_deg", 0.0))
                self.target = motor_degrees_to_ros(hold, self.calibrations)
                self.target_time = self.source_time = time.monotonic()
                self.starting, self.active = False, True
        except BaseException:
            self.stop("start failed")
            raise

    def command(self, proposed, source_time, *, clip=False):
        with self.lock:
            if not self.active:
                raise RuntimeError(self.stop_reason or "Servo is not active")
            current = self.observation()
            proposed = vector(proposed)
            if self.leader is None:
                proposed[6] = gripper_target(proposed[6], self.gripper_settings, self.motor_limits[6])
            if clip:
                proposed = [min(high, max(low, value)) for value, (low, high) in zip(proposed, self.motor_limits)]
            now = time.monotonic()
            dt = min(max(now - self.target_time, 0.001), self.control["inference_timeout_s"])
            gripper_step = None
            if self.leader is None:
                ratio = max(calibration["gear_ratio"] for calibration in self.calibrations)
                step = min(self.control["max_step_deg"], math.degrees(self.control["max_velocity_rad_s"] * dt / ratio))
                gripper_step = math.degrees(self.gripper_speed * dt / self.calibrations[6]["gear_ratio"])
            else:
                step = self.leader.leader.config.max_step_deg or max(high - low for low, high in self.motor_limits)
            bounded = guard_action(current, proposed, self.motor_limits, step,
                                   gripper_lower_tolerance_deg=self.control.get("gripper_lower_tolerance_deg", 0.0),
                                   gripper_max_step_deg=gripper_step)
            previous = _ros_to_motor_degrees(self.target, self.calibrations)
            bounded = guard_action(previous, bounded, self.motor_limits, step, gripper_max_step_deg=gripper_step)
            if self.leader is not None:
                steps = [math.degrees(speed * dt / calibration["gear_ratio"])
                         for speed, calibration in zip(self.follow_speeds, self.calibrations)]
                bounded = (np.asarray(previous) + np.clip(np.asarray(bounded) - previous, -np.asarray(steps), steps)).tolist()
            self.target = motor_degrees_to_ros(bounded, self.calibrations)
            self.target_time, self.source_time = now, source_time
            return bounded

    def stop(self, reason="operator stop"):
        with self.lock:
            self.generation += 1
            self.active = self.starting = False
            self.target = None
            self.released = False
            if self.keyboard is not None:
                self.keyboard.disarm()
            self.stop_reason = reason
            if self.owns_servo:
                self.owns_servo = False
                if self.services["stop"].service_is_ready():
                    self.stop_future = self.services["stop"].call_async(Trigger.Request())
                else:
                    self.node.get_logger().error("servo/stop unavailable; driver watchdog must hold")

    def _tick(self):
        with self.lock:
            if self.starting and self.deadman_required and not self.deadman():
                self.stop("deadman released during start")
            if not self.active:
                return
            try:
                self.observation()
                if self.node.count_publishers(self.root + "/servo_joint_trajectory") != 1:
                    raise RuntimeError("Another servo publisher appeared")
                now = time.monotonic()
                if self.deadman_required and not self.deadman():
                    raise RuntimeError("m released or keyboard unavailable" if self.keyboard is not None
                                       else "LB released or joystick timed out")
                if now - min(self.target_time, self.source_time) > self.control["inference_timeout_s"]:
                    raise RuntimeError("Action or observation timed out")
                if self.status_time >= self.started_time and self.status.state_machine != "SERVO_RUNNING":
                    raise RuntimeError("Driver is no longer in servo mode")
                if self.status_time < self.started_time and now - self.started_time > self.control["status_timeout_s"]:
                    raise RuntimeError("Driver did not acknowledge servo status")
                if self.leader is not None:
                    # Read the cached leader frame here, independently of camera capture/disk I/O.
                    self.command(self.leader.target(), self.source_time, clip=True)
                message = JointTrajectory()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.joint_names = list(ROS_ARM_JOINTS) + ["gripper"]
                point = JointTrajectoryPoint()
                point.positions = list(self.target)
                current = self.arm + [self.gripper]
                speed = ([self.control["max_velocity_rad_s"]] * 6 + [self.gripper_speed]
                         if self.leader is None else self.follow_speeds)
                speed = np.asarray(speed)
                point.velocities = np.clip((np.asarray(self.target) - current) * self.fps, -speed, speed).tolist()
                message.points = [point]
                self.publisher.publish(message)
                self.command_count += 1
            except Exception as exc:
                self.stop(str(exc))
                self.node.get_logger().error(f"ACT stopped: {exc}")

    def close(self):
        try:
            self.stop("shutdown")
            if self.stop_future is not None:
                deadline = time.monotonic() + self.control["service_timeout_s"]
                while not self.stop_future.done() and time.monotonic() < deadline:
                    time.sleep(0.01)
                if not self.stop_future.done() or not self.stop_future.result().success:
                    self.node.get_logger().error("Stop was not acknowledged; check hardware/watchdog before leaving")
        finally:
            self.executor.shutdown()
            self.thread.join(timeout=1)
            self.node.destroy_node()
            if self.own_context:
                rclpy.try_shutdown()
