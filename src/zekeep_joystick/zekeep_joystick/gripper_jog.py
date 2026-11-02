from __future__ import annotations

import math
import time

from rclpy.qos import qos_profile_sensor_data
from zekeep_msgs.msg import JointMotorState

from .mapping import apply_deadzone


class GripperJog:
    """Add bounded, time-integrated trigger targets to the existing servo stream."""

    def __init__(self, node, root: str, publish_rate: float, timeout: float,
                 open_axis: int = 5, close_axis: int = 4):
        defaults = dict(gripper_open_axis=open_axis, gripper_close_axis=close_axis,
                        gripper_speed=0.575, gripper_open_position=1.15,
                        gripper_close_position=0.0)
        for name, default in defaults.items():
            node.declare_parameter(name, default)
        self.open_axis = int(node.get_parameter("gripper_open_axis").value)
        self.close_axis = int(node.get_parameter("gripper_close_axis").value)
        self.speed = float(node.get_parameter("gripper_speed").value)
        self.open_position = float(node.get_parameter("gripper_open_position").value)
        self.close_position = float(node.get_parameter("gripper_close_position").value)
        if (min(self.open_axis, self.close_axis) < 0 or self.open_axis == self.close_axis
                or not all(math.isfinite(v) for v in (
                    self.speed, self.open_position, self.close_position))
                or self.speed <= 0 or self.open_position == self.close_position):
            raise ValueError("invalid gripper jog parameters")
        self.period, self.timeout = 1.0 / publish_rate, timeout
        self.feedback = None
        self.feedback_time = 0.0
        self.reset()
        self.subscription = node.create_subscription(
            JointMotorState, root + "/gripper/state", self._on_state,
            qos_profile_sensor_data,
        )

    def _on_state(self, message):
        if math.isfinite(message.position) and message.status_code in (0, 1):
            self.feedback = float(message.position)
            self.feedback_time = time.monotonic()

    def reset(self):
        self.target = None
        self.velocity = 0.0
        self.moving = False
        self.last_tick = time.monotonic()

    def update(self, axes, now: float):
        dt = max(0.0, min(now - self.last_tick, 2 * self.period))
        self.last_tick = now
        if len(axes) <= max(self.open_axis, self.close_axis):
            raise ValueError("Joy message has too few trigger axes")
        values = [float(axes[i]) for i in (self.open_axis, self.close_axis)]
        if not all(math.isfinite(v) and -1.0 <= v <= 1.0 for v in values):
            raise ValueError("invalid gripper trigger input")
        # joy_node reports released triggers as +1 and fully pressed as -1.
        opened, closed = [apply_deadzone((1.0 - v) / 2.0, 0.12) for v in values]
        direction = 0.0 if opened and closed else opened - closed
        if self.target is None and not direction:
            return  # Preserve A's holding torque until a trigger is used.
        if self.feedback is None or now - self.feedback_time > self.timeout:
            raise ValueError("gripper feedback missing or stale")
        low, high = sorted((self.close_position, self.open_position))
        velocity = direction * self.speed * (1 if self.open_position > self.close_position else -1)
        if velocity:
            start = self.target if self.velocity * velocity > 0 else self.feedback
            # Bound tracking lead in time so higher publish rates do not slow the gripper.
            lead = self.speed * 0.2
            target = max(self.feedback - lead, min(self.feedback + lead, start + velocity * dt))
            self.target = max(low, min(high, target))
        elif self.moving:
            self.target = max(low, min(high, self.feedback))
        self.moving = bool(direction)
        self.velocity = velocity if low < self.target < high else 0.0

    def append(self, message, point):
        if self.target is not None:
            message.joint_names = list(message.joint_names) + ["gripper"]
            point.positions = list(point.positions) + [self.target]
            point.velocities = list(point.velocities) + [self.velocity]
