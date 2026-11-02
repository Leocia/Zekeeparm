from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.srv import GripperCommand

from .follow_control import joint_velocity


class ShadowFollower(Node):
    def __init__(self):
        super().__init__("zekeep_shadow_follower")
        self.declare_parameter("target_loss_timeout_s", 0.35)
        self.declare_parameter("max_observation_age_s", 0.15)
        self.declare_parameter("max_robot_state_age_s", 0.10)
        self.declare_parameter("base_frame", "base_link")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.loss_timeout = float(self.get_parameter("target_loss_timeout_s").value)
        self.observation_max_age = float(self.get_parameter("max_observation_age_s").value)
        self.robot_max_age = float(self.get_parameter("max_robot_state_age_s").value)
        if min(self.loss_timeout, self.observation_max_age, self.robot_max_age) <= 0:
            raise ValueError("all feedback timeouts must be positive")

        urdf = Path(get_package_share_directory("zekeep_bringup")) / "description/urdf/sixaxis.urdf"
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId("grasp_tcp")
        self.joint_names = tuple(f"joint{i}" for i in range(1, 7))
        if (self.model.nq != 8 or self.model.nv != 8 or
                tuple(self.model.names[1:7]) != self.joint_names or
                self.frame_id == len(self.model.frames)):
            raise RuntimeError("Pinocchio model does not match six-joint controller/TCP")
        self.lower = self.model.lowerPositionLimit[:6] + 0.02
        self.upper = self.model.upperPositionLimit[:6] - 0.02

        self.target_error = None
        self.target_received = 0.0
        self.joint_positions = None
        self.joint_velocity_feedback = None
        self.gripper_positions = np.zeros(2)
        self.joint_received = 0.0
        self.g_held = False
        self.g_received = 0.0
        self.active = False
        self.state = "STARTUP_RETURNING_TO_READY"
        self.desired_rotation = None
        self.previous_velocity = np.zeros(6)
        self.command_positions = None
        self.last_tick = 0.0
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.operation = 0

        self.commands = self.create_publisher(JointTrajectory, "/zekeep/servo_joint_trajectory", 10)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "~/status", status_qos)
        self.create_subscription(PointStamped, "/zekeep_shadow_vision/target_error", self.target_cb, 10)
        self.create_subscription(String, "/zekeep_shadow_vision/status", self.vision_status_cb, 10)
        self.create_subscription(Bool, "/zekeep_shadow_vision/g_held", self.hold_cb, 10)
        self.create_subscription(JointState, "/zekeep/joint_states", self.joints_cb, qos_profile_sensor_data)
        self.start_driver = self.create_client(Trigger, "/zekeep/servo/start")
        self.stop_driver = self.create_client(Trigger, "/zekeep/servo/stop")
        self.require_posvel = self.create_client(Trigger, "/zekeep/servo/require_posvel")
        self.ready = self.create_client(Trigger, "/zekeep/return_ready")
        self.open_gripper = self.create_client(GripperCommand, "/zekeep/gripper/open")
        self.create_service(Trigger, "~/return", self.return_cb)
        self.create_timer(0.01, self.control_tick)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()
        self.worker.submit(self._startup_return)

    def target_cb(self, msg):
        age = (self.get_clock().now().nanoseconds -
               (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) * 1e-9
        error = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        if msg.header.frame_id != self.base_frame or age < -0.05 or age > self.observation_max_age:
            return
        if np.all(np.isfinite(error)):
            self.target_error = error
            self.target_received = time.monotonic()

    def joints_cb(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        velocity_by_name = dict(zip(msg.name, msg.velocity))
        if all(name in by_name for name in self.joint_names):
            q = np.array([by_name[name] for name in self.joint_names], dtype=float)
            if np.all(np.isfinite(q)):
                self.joint_positions = q
                self.joint_received = time.monotonic()
                if all(name in velocity_by_name for name in self.joint_names):
                    velocity = np.array([velocity_by_name[name] for name in self.joint_names])
                    if np.all(np.isfinite(velocity)):
                        self.joint_velocity_feedback = velocity
        if all(name in by_name for name in ("gripper_joint", "right_joint")):
            self.gripper_positions = np.array([by_name["gripper_joint"], by_name["right_joint"]])

    def vision_status_cb(self, msg):
        if msg.data == "WAITING_FOR_SELECTION" and (self.active or self.state == "STARTING"):
            self._stop("VISION_WAITING_FOR_SELECTION")

    def hold_cb(self, msg):
        was_held = self.g_held
        self.g_held = bool(msg.data)
        self.g_received = time.monotonic()
        if self.g_held and not was_held:
            self._start_follow()
        elif not self.g_held and was_held and (self.active or self.state == "STARTING"):
            self._stop("PAUSED")

    def _kinematics(self):
        q = np.zeros(self.model.nq)
        q[:6] = self.joint_positions
        q[6:] = self.gripper_positions
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)
        rotation = self.data.oMf[self.frame_id].rotation.copy()
        jacobian = pin.getFrameJacobian(
            self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, :6]
        return rotation, jacobian

    def _start_follow(self):
        if self.active or self.state == "STARTING":
            return
        if self.state not in ("WAITING_FOR_SELECTION", "PAUSED", "WAITING_FOR_CONFIRMATION",
                              "VISION_WAITING_FOR_SELECTION") and not self.state.startswith("START_FAILED"):
            self.get_logger().warning(f"cannot follow from {self.state}")
            return
        if self.target_error is None or time.monotonic() - self.target_received > self.loss_timeout:
            self.get_logger().warning("g held but no fresh selected target")
            return
        if self.joint_positions is None or time.monotonic() - self.joint_received > self.robot_max_age:
            self.state = "START_FAILED: fresh joint feedback unavailable"
            self._publish_status()
            return
        if np.any(self.joint_positions < self.lower) or np.any(self.joint_positions > self.upper):
            self.state = "START_FAILED: joint outside guarded limit"
            self._publish_status()
            return
        try:
            self.desired_rotation = self._kinematics()[0]
        except Exception as exc:
            self.state = "START_FAILED: " + str(exc)
            self._publish_status()
            return
        self.previous_velocity[:] = 0.0
        self.command_positions = self.joint_positions.copy()
        self.last_tick = time.monotonic()
        self.state = "STARTING"
        self.operation += 1
        self.worker.submit(self._start_motion, self.operation)
        self._publish_status()

    def control_tick(self):
        if not self.active:
            return
        now = time.monotonic()
        if not self.g_held or now - self.g_received > 0.25:
            self._stop("PAUSED")
            return
        if now - self.target_received > self.loss_timeout:
            self._stop("TARGET_LOST")
            return
        try:
            if self.joint_positions is None or now - self.joint_received > self.robot_max_age:
                raise RuntimeError("joint feedback is stale")
            q = self.joint_positions.copy()
            if np.any(q < self.lower) or np.any(q > self.upper):
                raise RuntimeError("joint outside guarded limit")
            rotation, jacobian = self._kinematics()
            velocity = joint_velocity(jacobian, self.target_error.copy(), rotation,
                                      self.desired_rotation, self.previous_velocity)
            velocity[(q <= self.lower + 0.02) & (velocity < 0)] = 0
            velocity[(q >= self.upper - 0.02) & (velocity > 0)] = 0
            self.previous_velocity = velocity
            dt = min(0.03, now - self.last_tick)
            self.last_tick = now
            # Integrate velocity while bounding command lead over measured joints.
            if np.any(velocity):
                self.command_positions = np.clip(
                    self.command_positions + dt * velocity, q - 0.04, q + 0.04)
            else:
                self.command_positions = q.copy()
            target = np.clip(self.command_positions, self.lower, self.upper)
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = list(self.joint_names)
            point = JointTrajectoryPoint()
            point.positions = target.tolist()
            point.velocities = velocity.tolist()
            msg.points = [point]
            self.commands.publish(msg)
            actual = (np.max(np.abs(self.joint_velocity_feedback))
                      if self.joint_velocity_feedback is not None else float("nan"))
            self.state = (f"FOLLOWING err={np.linalg.norm(self.target_error):.3f}m "
                          f"ori={np.rad2deg(np.linalg.norm(pin.log3(rotation.T @ self.desired_rotation))):.1f}deg "
                          f"cmd={np.max(np.abs(velocity)):.2f} act={actual:.2f}rad/s")
        except Exception as exc:
            self._stop("STOPPED: " + str(exc))

    def _stop(self, reason):
        self.active = False
        if self.joint_positions is not None:
            hold = JointTrajectory()
            hold.header.stamp = self.get_clock().now().to_msg()
            hold.joint_names = list(self.joint_names)
            point = JointTrajectoryPoint()
            point.positions = self.joint_positions.tolist()
            point.velocities = [0.0] * 6
            hold.points = [point]
            self.commands.publish(hold)
        self.previous_velocity[:] = 0.0
        self.command_positions = None
        self.state = reason
        self.operation += 1
        self._publish_status()
        self.worker.submit(self._stop_motion)

    def _trigger(self, client, label, request=None):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(Trigger.Request() if request is None else request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(35.0) or future.result() is None or not future.result().success:
            detail = future.result().message if future.done() and future.result() else "timeout"
            raise RuntimeError(f"{label} failed: {detail}")

    def _start_motion(self, operation):
        try:
            self._trigger(self.require_posvel, "POS_VEL follow mode")
            self._trigger(self.start_driver, "SDK servo stream")
            if operation != self.operation:
                self._stop_motion()
                return
            self.active, self.state = True, "FOLLOWING"
        except Exception as exc:
            self.active, self.state = False, "START_FAILED: " + str(exc)
        self._publish_status()

    def _stop_motion(self):
        try:
            self._trigger(self.stop_driver, "SDK servo stream")
        except Exception as exc:
            self.get_logger().error(str(exc))

    def _startup_return(self):
        try:
            self._trigger(self.require_posvel, "POS_VEL follow mode")
            self._trigger(self.ready, "startup return_ready")
            self._open_gripper()
            self.state = "WAITING_FOR_SELECTION"
        except Exception as exc:
            self.state = "STARTUP_RETURN_FAILED: " + str(exc)
            self.get_logger().error(self.state)
        self._publish_status()

    def return_cb(self, _request, response):
        if self.state != "TARGET_LOST":
            response.success, response.message = False, "return is allowed only after target loss"
            return response
        self.state = "RETURNING_TO_READY"
        self.worker.submit(self._return_to_ready)
        response.success, response.message = True, "return requested"
        self._publish_status()
        return response

    def _return_to_ready(self):
        self._stop_motion()
        try:
            self._trigger(self.ready, "return_ready")
            self._open_gripper()
            self.state = "WAITING_FOR_CONFIRMATION"
        except Exception as exc:
            self.state = "RETURN_FAILED: " + str(exc)
        self._publish_status()

    def _open_gripper(self):
        request = GripperCommand.Request()
        request.position = 0.0
        request.timeout = 5.0
        self._trigger(self.open_gripper, "open gripper before following", request)

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShadowFollower()
    try:
        rclpy.spin(node)
    finally:
        if node.active:
            node._stop("SHUTDOWN")
        node.worker.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()
