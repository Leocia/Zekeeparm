from __future__ import annotations

from contextlib import ExitStack
import time
import threading
import math

from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


class ServoStream:
    """Guarded latest-command adapter to the SDK loop."""

    def __init__(self, node: Node, hardware, namespace: str):
        self.node = node
        self.hardware = hardware
        self.names = tuple(hardware.joint_names)
        node.declare_parameter("servo_command_timeout_s", 0.2)
        self.timeout = float(node.get_parameter("servo_command_timeout_s").value)
        if not 0.05 <= self.timeout <= 1.0:
            raise ValueError("servo_command_timeout_s must be in [0.05, 1.0]")
        self.last_command = None
        self._lock = threading.Lock()
        root = f"/{namespace}/servo"
        self.start_service = node.create_service(Trigger, root + "/start", node.web_task_gate.service(self.start))
        self.start_gripper_service = node.create_service(
            Trigger, root + "/start_with_gripper", node.web_task_gate.service(self.start_with_gripper))
        self.start_grasp_service = node.create_service(
            Trigger, root + "/start_with_gripper_grasp", node.web_task_gate.service(self.start_with_gripper_grasp))
        self.stop_service = node.create_service(Trigger, root + "/stop", self.stop)
        self.require_mit_service = node.create_service(
            Trigger, root + "/require_mit", node.web_task_gate.service(self.require_mit)
        )
        self.require_posvel_service = node.create_service(
            Trigger, root + "/require_posvel", node.web_task_gate.service(self.require_posvel)
        )
        self.require_gripper_service = node.create_service(
            Trigger, root + "/require_gripper_stream", node.web_task_gate.service(self.require_gripper_stream)
        )
        self.subscription = node.create_subscription(
            JointTrajectory,
            f"/{namespace}/servo_joint_trajectory",
            self.command,
            10,
            callback_group=node.reentrant_group,
        )
        self.timer = node.create_timer(0.02, self.watchdog, callback_group=node.reentrant_group)

    def start(self, _request, response):
        return self._start(response, with_gripper=False)

    def start_with_gripper(self, _request, response):
        return self._start(response, with_gripper=True)

    def start_with_gripper_grasp(self, _request, response):
        return self._start(response, with_gripper=True, gripper_grasp=True)

    def _start(self, response, *, with_gripper, gripper_grasp=False):
        with self._lock:
            try:
                if with_gripper:
                    with ExitStack() as locks:
                        for name in ("arm_actions", "task_actions"):
                            actions = getattr(self.node, name, None)
                            if actions is not None:
                                lock = actions._gripper_execution_lock
                                if not lock.acquire(blocking=False):
                                    raise RuntimeError("another gripper action is active")
                                locks.callback(lock.release)
                        if gripper_grasp:
                            self.hardware.begin_servo_stream(with_gripper=True, gripper_grasp=True)
                        else:
                            self.hardware.begin_servo_stream(with_gripper=True)
                else:
                    self.hardware.begin_servo_stream()
                self.last_command = time.monotonic()
                self.node.publish_arm_status(read_hardware=False)
                response.success, response.message = True, "servo stream started"
            except Exception as exc:
                response.success, response.message = False, str(exc)
        return response

    def require_mit(self, _request, response):
        response.success = self.hardware._arm_control_mode == "mit"
        response.message = (
            "MIT mode configured" if response.success
            else "follow requires MIT; restart bringup with zekeep_hardware_withnogravity.yaml"
        )
        return response

    def require_posvel(self, _request, response):
        response.success = self.hardware._arm_control_mode == "posvel"
        response.message = (
            "POS_VEL mode configured" if response.success
            else "follow requires POS_VEL; restart bringup with zekeep_hardware.yaml"
        )
        return response

    def require_gripper_stream(self, _request, response):
        response.success = self.hardware.has_gripper
        response.message = "seven-joint servo with gripper watchdog" if response.success else "gripper unavailable"
        return response

    def stop(self, _request, response):
        with self._lock:
            self.hardware.stop_servo_stream()
            self.last_command = None
            self.node.publish_arm_status(read_hardware=False)
        response.success, response.message = True, "servo stream stopped; holding position"
        return response

    def command(self, msg: JointTrajectory):
        with self._lock:
            if self.hardware.state_machine != "SERVO_RUNNING":
                return
            try:
                # A single point replaces the prior servo target immediately.
                with_gripper = set(msg.joint_names) == set(self.names) | {"gripper"}
                expected = self.names + (("gripper",) if with_gripper else ())
                if len(msg.points) != 1 or len(msg.joint_names) != len(expected):
                    raise ValueError("servo command requires all six arm joints and optional gripper")
                if len(set(msg.joint_names)) != len(expected) or set(msg.joint_names) != set(expected):
                    raise ValueError("servo command joint names do not match arm")
                point = msg.points[0]
                if len(point.positions) != len(expected) or len(point.velocities) != len(expected):
                    raise ValueError("servo point dimensions do not match joint names")
                by_name = {name: i for i, name in enumerate(msg.joint_names)}
                positions = [point.positions[by_name[name]] for name in self.names]
                velocities = [point.velocities[by_name[name]] for name in self.names]
                if with_gripper:
                    if not math.isfinite(point.velocities[by_name["gripper"]]):
                        raise ValueError("non-finite gripper velocity")
                    self.hardware.set_servo_target(
                        positions, velocities, gripper_position=point.positions[by_name["gripper"]]
                    )
                else:
                    self.hardware.set_servo_target(positions, velocities)
                self.last_command = time.monotonic()
            except RuntimeError as exc:
                if (str(exc) == "servo command received while servo stream is inactive"
                        and self.hardware.state_machine != "SERVO_RUNNING"):
                    self.last_command = None
                    return
                self._reject(exc)
            except Exception as exc:
                self._reject(exc)

    def _reject(self, exc):
        self.node.get_logger().error(f"servo command rejected; holding: {exc}")
        self.hardware.stop_servo_stream()
        self.last_command = None
        self.node.publish_arm_status(read_hardware=False)

    def watchdog(self):
        with self._lock:
            if (self.hardware.state_machine == "SERVO_RUNNING" and self.last_command is not None
                    and time.monotonic() - self.last_command > self.timeout):
                self.node.get_logger().error("servo command timeout; holding arm")
                self.hardware.stop_servo_stream()
                self.last_command = None
                self.node.publish_arm_status(read_hardware=False)
