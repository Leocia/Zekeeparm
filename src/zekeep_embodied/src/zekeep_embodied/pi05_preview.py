"""Visualization-only prediction messages. No controllers, services, or actions."""

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time

from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory, RobotTrajectory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
from zekeepcontroller.ros_publishers import _gripper_motor_to_joint_position

from .pi05_runtime import motor_degrees_to_ros
from .record import ROS_ARM_JOINTS
from .storage import write_json


def make_preview_trajectory(state, actions, calibrations, open_ros, close_ros, width, hz):
    """Show raw predictions, not safety-clipped commands, using training cadence."""
    state = np.asarray(state, dtype=np.float64)
    actions = np.asarray(actions, dtype=np.float64)
    if (state.shape != (7,) or actions.shape != (10, 7)
            or not np.isfinite(state).all() or not np.isfinite(actions).all()):
        raise ValueError("Preview requires seven finite states and ten seven-joint actions")
    if not math.isfinite(hz) or hz <= 0 or not math.isfinite(width) or width <= 0:
        raise ValueError("Preview cadence and gripper width must be positive and finite")
    if not all(math.isfinite(value) for value in (open_ros, close_ros)) or open_ros == close_ros:
        raise ValueError("Preview requires distinct finite gripper endpoints")

    message = DisplayTrajectory()
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(ROS_ARM_JOINTS) + ["gripper_joint"]
    for index, values in enumerate([state, *actions]):
        ros = motor_degrees_to_ros(values, calibrations)
        if not all(math.isfinite(value) for value in ros):
            raise ValueError("Preview calibration produced non-finite positions")
        # Same angle-to-finger mapping as the driver; only visual finger travel is clamped.
        finger = _gripper_motor_to_joint_position(ros[6], open_ros, close_ros, width)
        point = JointTrajectoryPoint()
        point.positions = ros[:6] + [finger]
        nanoseconds = round(index / hz * 1_000_000_000)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
        trajectory.joint_trajectory.points.append(point)
    message.trajectory_start.joint_state.name = list(trajectory.joint_trajectory.joint_names)
    message.trajectory_start.joint_state.position = list(trajectory.joint_trajectory.points[0].positions)
    message.trajectory = [trajectory]
    return message


class PredictionPreview:
    def __init__(self, node, hardware_config, *, once=False, save_path=None):
        self.node = node
        self.once = once
        self.sent = False
        self.error = None
        self.save_path = Path(save_path).expanduser().resolve() if save_path is not None else None
        if self.save_path is not None and self.save_path.exists():
            raise FileExistsError(f"Preview file already exists: {self.save_path}")
        with Path(hardware_config).open() as stream:
            hardware = yaml.safe_load(stream)
        model = hardware["default_model"]
        width = float(hardware["models"][model]["overrides"]["gripper"]["max_width_m"])
        if not math.isfinite(width) or width <= 0:
            raise ValueError("Preview requires calibrated gripper max_width_m")
        self.width = width
        # Official display-only contract: no JointTrajectory controller topic.
        # https://github.com/moveit/moveit_msgs/blob/ros2/msg/DisplayTrajectory.msg
        if self.save_path is None:
            self.trajectory = node.create_publisher(DisplayTrajectory, "/pi05_preview/trajectory", 10)
            self.state = node.create_publisher(DisplayRobotState, "/pi05_preview/state", 10)

    def publish(self, state, actions, *, report=None):
        if self.sent and (self.once or self.save_path is not None):
            return
        if self.save_path is None and self.once and (
            self.trajectory.get_subscription_count() == 0 or self.state.get_subscription_count() == 0
        ):
            return
        message = make_preview_trajectory(
            state, actions, self.node.calibrations,
            self.node.gripper_open_ros, self.node.gripper_close_ros,
            self.width, float(self.node.runtime["policy_hz"]),
        )
        message.trajectory_start.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        if self.save_path is not None:
            snapshot = {
                "schema_version": 1,
                "kind": "zekeep_pi05_preview",
                "saved_at_unix_s": time.time(),
                "repo_id": self.node.runtime["repo_id"],
                "experiment": self.node.runtime["experiment"],
                "state_deg": np.asarray(state).tolist(),
                "actions_deg": np.asarray(actions).tolist(),
                "guard_report": report,
                "display_trajectory": message_to_ordereddict(message),
            }
            # Publish a complete file atomically; a concurrent writer cannot be overwritten.
            with tempfile.TemporaryDirectory(prefix=".pi05-preview-", dir=self.save_path.parent) as directory:
                temporary = Path(directory) / "prediction.json"
                write_json(temporary, snapshot)
                os.link(temporary, self.save_path)
            self.sent = True
            return
        measured = DisplayRobotState()
        measured.state = message.trajectory_start
        self.state.publish(measured)
        self.trajectory.publish(message)
        self.sent = True


def load_preview(path):
    """Read display geometry already calibrated at capture; never load a model."""
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        snapshot = json.load(stream)
    if not isinstance(snapshot, dict):
        raise ValueError("Preview file must contain a JSON object")
    if snapshot.get("schema_version") != 1 or snapshot.get("kind") != "zekeep_pi05_preview":
        raise ValueError("Unsupported pi05 preview file")
    message = DisplayTrajectory()
    set_message_fields(message, snapshot["display_trajectory"])
    names = list(ROS_ARM_JOINTS) + ["gripper_joint"]
    if len(message.trajectory) != 1:
        raise ValueError("Preview must contain one trajectory")
    trajectory = message.trajectory[0].joint_trajectory
    if trajectory.joint_names != names or len(trajectory.points) != 11:
        raise ValueError("Preview must contain measured start plus ten seven-joint waypoints")
    start = message.trajectory_start.joint_state
    if start.name != names or list(start.position) != list(trajectory.points[0].positions):
        raise ValueError("Preview start state does not match first waypoint")
    previous = -1
    for point in trajectory.points:
        duration = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
        if (len(point.positions) != 7 or not all(math.isfinite(value) for value in point.positions)
                or duration <= previous or duration < 0):
            raise ValueError("Preview has invalid positions or waypoint timing")
        previous = duration
    return message, snapshot


def main():
    parser = argparse.ArgumentParser(description="Replay a saved pi05 prediction in RViz; no model or cameras")
    parser.add_argument("file", type=Path)
    args = parser.parse_args()
    try:
        message, snapshot = load_preview(args.file)
    except (OSError, ValueError, KeyError, TypeError, AttributeError, AssertionError) as exc:
        parser.error(str(exc))
    rclpy.init()
    node = rclpy.create_node("pi05_preview_replay")
    try:
        state = node.create_publisher(DisplayRobotState, "/pi05_preview/state", 10)
        trajectory = node.create_publisher(DisplayTrajectory, "/pi05_preview/trajectory", 10)
        print("离线回放：等待RViz；不加载模型、不打开相机、不发送实机指令。", flush=True)
        while rclpy.ok() and (state.get_subscription_count() == 0 or trajectory.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return
        measured = DisplayRobotState()
        measured.state = message.trajectory_start
        state.publish(measured)
        trajectory.publish(message)
        print(json.dumps({"mode": "offline_replay", "file": str(args.file),
                          "repo_id": snapshot.get("repo_id"), "experiment": snapshot.get("experiment"),
                          "guard_report": snapshot.get("guard_report")}, ensure_ascii=False), flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
