"""Guarded shadow and deadman-held ROS runtime for the local pi0.5 policy."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import sys
import time
from xmlrpc.client import Binary, ServerProxy, Transport

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand as GripperCommandAction
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.action import GripperGrasp
from zekeep_msgs.msg import JointMotorState

from .cameras import Camera
from .record import ROS_ARM_JOINTS, _load_ros_calibration, _ros_to_motor_degrees, load_config


class _TimeoutTransport(Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


def motor_degrees_to_ros(values, calibrations):
    if len(values) != len(calibrations):
        raise ValueError("Motor target and calibration dimensions differ")
    result = []
    for value, calibration in zip(values, calibrations):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("Non-finite motor target")
        motor = math.radians(value)
        result.append(
            (motor - calibration["encoder_zero"])
            * calibration["direction"]
            * calibration["gear_ratio"]
            + calibration["ros_zero"]
        )
    return result


def guard_action(current, proposed, limits, max_step_deg):
    if len(current) != 7 or len(proposed) != 7 or len(limits) != 7:
        raise ValueError("Action guard requires seven joints")
    if not math.isfinite(max_step_deg) or max_step_deg <= 0:
        raise ValueError("max_step_deg must be positive and finite")
    guarded = []
    for index, (state, target, bounds) in enumerate(zip(current, proposed, limits)):
        state, target = float(state), float(target)
        low, high = map(float, bounds)
        if not all(math.isfinite(value) for value in (state, target, low, high)):
            raise ValueError("Action guard received a non-finite value")
        name = ROS_ARM_JOINTS[index] if index < 6 else "gripper"
        if low >= high or not low <= state <= high:
            raise RuntimeError(
                f"Current joint {index} ({name}) is outside guarded limits: "
                f"{state:.3f} deg not in [{low:.3f}, {high:.3f}] deg"
            )
        if not low <= target <= high:
            raise RuntimeError(
                f"Policy joint {index} ({name}) target is outside guarded limits: "
                f"{target:.3f} deg not in [{low:.3f}, {high:.3f}] deg"
            )
        delta = min(max(target - state, -max_step_deg), max_step_deg)
        guarded.append(state + delta)
    return guarded


def gripper_transition(current, predicted, open_deg, close_deg, hysteresis_deg):
    span = open_deg - close_deg
    if not all(math.isfinite(value) for value in (
        current, predicted, open_deg, close_deg, hysteresis_deg
    )) or span == 0 or hysteresis_deg < 0 or 2 * hysteresis_deg >= abs(span):
        raise ValueError("Invalid gripper transition parameters")
    current_ratio = (current - close_deg) / span
    predicted_ratio = (predicted - close_deg) / span
    hysteresis = hysteresis_deg / abs(span)
    if current_ratio < 0.5 and predicted_ratio >= 0.5 + hysteresis:
        return "open"
    if current_ratio >= 0.5 and predicted_ratio <= 0.5 - hysteresis:
        return "close"
    return None


def _runtime_config(config):
    values = dict(config.get("pi05_runtime") or {})
    defaults = {
        "server": "http://127.0.0.1:8000",
        "repo_id": "local/zekeep_pick_place_9hz",
        "experiment": "pick_place_9hz_1000",
        "policy_hz": 9.0,
        "max_action_step_deg": 0.5,
        "max_joint_velocity_rad_s": 0.10,
        "max_inference_s": 1.0,
        "joy_timeout_s": 0.25,
        "joint_limit_margin_rad": 0.02,
        "gripper_hysteresis_deg": 5.0,
    }
    for key, value in defaults.items():
        values.setdefault(key, value)
    numeric = [key for key in defaults if key not in {"server", "repo_id", "experiment"}]
    if any(not math.isfinite(float(values[key])) or float(values[key]) <= 0 for key in numeric):
        raise ValueError("pi05_runtime numeric settings must be positive and finite")
    if not str(values["server"]).startswith("http://127.0.0.1:"):
        raise ValueError("pi05_runtime server must use local loopback")
    return values


class Pi05Runtime(Node):
    def __init__(self, config, cameras, *, execute=False, server=None, preview=False,
                 preview_once=False, save_preview=None):
        if execute and (preview or preview_once or save_preview is not None):
            raise ValueError("Trajectory preview is shadow-only")
        if save_preview is not None and (preview or preview_once):
            raise ValueError("Saving a preview does not require a live RViz preview")
        super().__init__("zekeep_pi05_runtime")
        self.config = config
        self.runtime = _runtime_config(config)
        self.server = server or self.runtime["server"]
        self.cameras = cameras
        self.execute = execute
        ros = config["ros_joystick"]
        self.namespace = ros["arm_namespace"].strip("/")
        self.root = f"/{self.namespace}"
        self.calibrations, self.gripper_open_ros, self.gripper_close_ros = (
            _load_ros_calibration(ros["hardware_config"])
        )
        margin = float(self.runtime["joint_limit_margin_rad"])
        self.motor_limits = []
        for index, calibration in enumerate(self.calibrations):
            low = calibration["hard_lower"] + (margin if index < 6 else 0.0)
            high = calibration["hard_upper"] - (margin if index < 6 else 0.0)
            bounds = _ros_to_motor_degrees([low, high], [calibration, calibration])
            self.motor_limits.append((min(bounds), max(bounds)))
        self.gripper_open_deg, self.gripper_close_deg = _ros_to_motor_degrees(
            [self.gripper_open_ros, self.gripper_close_ros], self.calibrations[-1:] * 2
        )
        self.preview = None
        self.save_preview = save_preview
        if preview or preview_once or save_preview is not None:
            from .pi05_preview import PredictionPreview

            self.preview = PredictionPreview(
                self, ros["hardware_config"], once=preview_once, save_path=save_preview,
            )

        self.arm_state = None
        self.gripper_state = None
        self.state_time = self.gripper_time = self.joy_time = 0.0
        self.buttons = []
        self.deadman_button = int(ros["deadman_button"])
        self.max_state_age = float(ros["max_state_age_s"])
        self.last_sequences = {}
        self.next_request = 0.0
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.inference = None
        self.inference_started = 0.0
        self.inference_state = None
        self.latest_action = None
        self.latest_serial = self.applied_serial = 0
        self.policy_error = None
        self.last_capture_warning = float("-inf")
        self.mode = "shadow" if not execute else "idle"
        self.require_deadman_release = execute
        self.arm_target = None
        self.operation = 0
        self.pending_gripper = None
        self.gripper_goal = None
        self.closed = False

        self.create_subscription(
            JointState, self.root + "/joint_states", self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointMotorState, self.root + "/gripper/state", self._on_gripper_state,
            qos_profile_sensor_data,
        )
        if execute:
            self.create_subscription(
                Joy, ros.get("joy_topic", "/joy"), self._on_joy, qos_profile_sensor_data
            )
            self.publisher = self.create_publisher(
                JointTrajectory, self.root + "/servo_joint_trajectory", 10
            )
            self.require_posvel = self.create_client(Trigger, self.root + "/servo/require_posvel")
            self.start_servo = self.create_client(Trigger, self.root + "/servo/start")
            self.stop_servo = self.create_client(Trigger, self.root + "/servo/stop")
            self.open_gripper = ActionClient(
                self, GripperCommandAction, self.root + "/gripper/command"
            )
            self.close_gripper = ActionClient(
                self, GripperGrasp, self.root + "/gripper/grasp"
            )
        self.create_timer(0.02, self._tick)

    def _on_joint_state(self, message):
        by_name = dict(zip(message.name, message.position))
        if all(name in by_name for name in ROS_ARM_JOINTS):
            values = [float(by_name[name]) for name in ROS_ARM_JOINTS]
            if all(math.isfinite(value) for value in values):
                self.arm_state = values
                self.state_time = time.monotonic()

    def _on_gripper_state(self, message):
        value = float(message.position)
        if math.isfinite(value):
            self.gripper_state = value
            self.gripper_time = time.monotonic()

    def _on_joy(self, message):
        self.buttons = list(message.buttons)
        self.joy_time = time.monotonic()
        if not self._deadman():
            self.require_deadman_release = False
            if self.mode == "gripper" and self.gripper_goal is not None:
                self.gripper_goal.cancel_goal_async()

    def _deadman(self):
        return (
            time.monotonic() - self.joy_time <= float(self.runtime["joy_timeout_s"])
            and self.deadman_button < len(self.buttons)
            and bool(self.buttons[self.deadman_button])
        )

    def _motor_state(self):
        if self.arm_state is None or self.gripper_state is None:
            raise RuntimeError("Waiting for arm and gripper feedback")
        now = time.monotonic()
        if now - min(self.state_time, self.gripper_time) > self.max_state_age:
            raise RuntimeError("Arm or gripper feedback is stale")
        return _ros_to_motor_degrees(
            self.arm_state + [self.gripper_state], self.calibrations
        )

    def _capture_request(self):
        state = self._motor_state()
        images, times, sequences = {}, {}, {}
        for name, camera in self.cameras.items():
            images[name], times[name], sequences[name] = camera.snapshot()
        now = time.monotonic()
        timestamps = list(times.values()) + [self.state_time, self.gripper_time, now]
        if max(timestamps) - min(timestamps) > self.config["max_sync_s"]:
            raise RuntimeError("Camera/state synchronization tolerance exceeded")
        if any(sequences[name] <= self.last_sequences.get(name, 0) for name in sequences):
            raise RuntimeError("Duplicate camera frame")
        self.last_sequences = sequences
        return images, state

    @staticmethod
    def _png(image):
        ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise RuntimeError("PNG encoding failed")
        return Binary(encoded.tobytes())

    def _infer(self, images, state):
        proxy = ServerProxy(
            self.server,
            transport=_TimeoutTransport(float(self.runtime["max_inference_s"])),
            allow_none=False,
            use_builtin_types=True,
        )
        return proxy.infer(
            self._png(images["global"]),
            self._png(images["wrist"]),
            state,
            self.config["task"],
        )

    def _tick(self):
        now = time.monotonic()
        self._finish_inference(now)
        if getattr(self, "save_preview", None) is not None and (
            self.preview.sent or self.preview.error is not None
        ):
            return
        if self.inference is None and now >= self.next_request:
            try:
                images, state = self._capture_request()
                self.inference_started = now
                self.inference_state = list(state)
                self.inference = self.pool.submit(self._infer, images, state)
                self.next_request = now + 1.0 / float(self.runtime["policy_hz"])
            except RuntimeError as exc:
                self.policy_error = str(exc)
                if now - self.last_capture_warning >= 2.0:
                    self.get_logger().warning(f"Waiting for policy input: {exc}")
                    self.last_capture_warning = now

        if not self.execute:
            return
        if self.mode in {"active", "starting"} and not self._deadman():
            self._stop("deadman released or timed out")
            return
        if self.mode == "active":
            if self.policy_error:
                self._stop(self.policy_error)
                return
            if self.latest_action is None or now - self.latest_action[1] > float(
                self.runtime["max_inference_s"]
            ):
                self._stop("policy response is stale")
                return
            try:
                self._motor_state()
            except RuntimeError as exc:
                self._stop(str(exc))
                return
            if self.latest_serial != self.applied_serial:
                self._apply_latest_action()
            if self.mode == "active" and self.arm_target is not None:
                self._publish_target()
        elif (
            self.mode == "idle"
            and self._deadman()
            and not self.require_deadman_release
            and self.latest_action is not None
            and now - self.latest_action[1] <= float(self.runtime["max_inference_s"])
        ):
            self._start()

    def _finish_inference(self, now):
        if self.inference is None or not self.inference.done():
            return
        future, elapsed = self.inference, now - self.inference_started
        self.inference = None
        try:
            result = future.result()
            actions = np.asarray(result["actions"], dtype=np.float64)
            if actions.shape != (10, 7) or not np.isfinite(actions).all():
                raise RuntimeError(f"Invalid policy actions: {actions.shape}")
            if elapsed > float(self.runtime["max_inference_s"]):
                raise RuntimeError(f"Policy inference exceeded deadline: {elapsed:.3f}s")
            self.latest_serial += 1
            self.latest_action = (actions[0].tolist(), now)
            self.policy_error = None
            if not self.execute:
                report = {
                    "mode": "shadow",
                    "infer_ms": round(float(result["infer_ms"]), 1),
                    "first_action_deg": [round(value, 3) for value in actions[0]],
                    "action_guard_passed": False,
                }
                try:
                    state = self._motor_state()
                    report["current_state_deg"] = [round(value, 3) for value in state]
                    guarded = guard_action(
                        state, actions[0], self.motor_limits,
                        float(self.runtime["max_action_step_deg"]),
                    )
                    report["guarded_delta_deg"] = [
                        round(a - b, 3) for a, b in zip(guarded, state)
                    ]
                    report["action_guard_passed"] = True
                except (RuntimeError, ValueError) as exc:
                    report["guard_error"] = str(exc)
                print(json.dumps(report, ensure_ascii=False), flush=True)
                if getattr(self, "preview", None) is not None:
                    try:
                        self.preview.publish(self.inference_state, actions, report=report)
                    except Exception as exc:
                        if self.save_preview is not None:
                            self.preview.error = exc
                        self.get_logger().warning(f"Trajectory preview failed: {exc}")
        except Exception as exc:
            self.policy_error = f"policy inference failed: {exc}"
            self.get_logger().error(self.policy_error)

    def _start(self):
        if self.count_publishers(self.root + "/servo_joint_trajectory") != 1:
            self.require_deadman_release = True
            self.get_logger().error("another servo command publisher is active")
            return
        if not self.require_posvel.service_is_ready() or not self.start_servo.service_is_ready():
            self.require_deadman_release = True
            self.get_logger().error("servo services unavailable")
            return
        self.mode = "starting"
        self.operation += 1
        operation = self.operation
        self.require_posvel.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._on_posvel(future, operation)
        )

    def _on_posvel(self, future, operation):
        try:
            response = future.result()
            if operation != self.operation or not self._deadman():
                raise RuntimeError("start canceled")
            if not response.success:
                raise RuntimeError(response.message)
            self.start_servo.call_async(Trigger.Request()).add_done_callback(
                lambda result: self._on_started(result, operation)
            )
        except Exception as exc:
            self.mode = "idle"
            self.require_deadman_release = True
            self.get_logger().error(f"cannot start policy control: {exc}")

    def _on_started(self, future, operation):
        try:
            response = future.result()
            if response.success and (operation != self.operation or not self._deadman()):
                if self.stop_servo.service_is_ready():
                    self.stop_servo.call_async(Trigger.Request())
                raise RuntimeError("start canceled")
            if not response.success:
                raise RuntimeError(response.message)
            self.arm_target = list(self.arm_state)
            self.mode = "active"
            self.get_logger().warning("policy control active while deadman is held")
        except Exception as exc:
            self.mode = "idle"
            self.require_deadman_release = True
            self.get_logger().error(f"cannot start policy control: {exc}")

    def _apply_latest_action(self):
        try:
            state = self._motor_state()
            proposed = self.latest_action[0]
            guarded = guard_action(
                state, proposed, self.motor_limits,
                float(self.runtime["max_action_step_deg"]),
            )
            transition = gripper_transition(
                state[6], proposed[6], self.gripper_open_deg, self.gripper_close_deg,
                float(self.runtime["gripper_hysteresis_deg"]),
            )
            self.applied_serial = self.latest_serial
            if transition:
                self._stop(f"before gripper {transition}", pending_gripper=transition)
                return
            self.arm_target = motor_degrees_to_ros(guarded[:6], self.calibrations[:6])
        except Exception as exc:
            self._stop(f"action rejected: {exc}")

    def _publish_target(self):
        delta = np.asarray(self.arm_target) - np.asarray(self.arm_state)
        speed = float(self.runtime["max_joint_velocity_rad_s"])
        velocity = np.clip(delta * float(self.runtime["policy_hz"]), -speed, speed)
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(ROS_ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(self.arm_target)
        point.velocities = velocity.tolist()
        message.points = [point]
        self.publisher.publish(message)

    def _stop(self, reason, pending_gripper=None):
        if self.mode in {"stopping", "gripper"}:
            return
        self.operation += 1
        self.pending_gripper = pending_gripper
        self.require_deadman_release = True
        if self.arm_state is not None and hasattr(self, "publisher"):
            self.arm_target = list(self.arm_state)
            self._publish_target()
        self.mode = "stopping"
        if self.stop_servo.service_is_ready():
            self.stop_servo.call_async(Trigger.Request()).add_done_callback(
                lambda future: self._on_stopped(future, reason)
            )
        else:
            self.mode = "idle"
            self.pending_gripper = None
            self.get_logger().error(f"servo/stop unavailable after {reason}; watchdog must hold")

    def _on_stopped(self, future, reason):
        command = self.pending_gripper
        self.pending_gripper = None
        self.arm_target = None
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().warning(f"policy control stopped: {reason}")
        except Exception as exc:
            self.mode = "idle"
            self.get_logger().error(f"servo stop failed; watchdog must hold: {exc}")
            return
        if command is None:
            self.mode = "idle"
        else:
            self._command_gripper(command)

    def _command_gripper(self, command):
        client = self.open_gripper if command == "open" else self.close_gripper
        if not client.server_is_ready() or not self._deadman():
            self.mode = "idle"
            self.get_logger().error(f"gripper {command} canceled: action unavailable or deadman released")
            return
        self.mode = "gripper"
        if command == "open":
            goal = GripperCommandAction.Goal()
            goal.command.position = self.gripper_open_ros
            goal.command.max_effort = 1.0
        else:
            goal = GripperGrasp.Goal()
            goal.closing_torque = goal.hold_torque = goal.timeout = 0.0
        client.send_goal_async(goal).add_done_callback(
            lambda future: self._on_gripper_accepted(future, command)
        )

    def _on_gripper_accepted(self, future, command):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("goal rejected")
            self.gripper_goal = handle
            if not self._deadman():
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(
                lambda result: self._on_gripper_done(result, command)
            )
        except Exception as exc:
            self.gripper_goal = None
            self.mode = "idle"
            self.get_logger().error(f"gripper {command} failed: {exc}")

    def _on_gripper_done(self, future, command):
        self.gripper_goal = None
        self.mode = "idle"
        self.require_deadman_release = True
        try:
            wrapped = future.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"status={wrapped.status}")
            result = wrapped.result
            success = result.reached_goal if command == "open" else result.success
            if not success:
                raise RuntimeError(getattr(result, "message", "target not reached"))
            self.get_logger().info(f"gripper {command} complete; release deadman before continuing")
        except Exception as exc:
            self.get_logger().error(f"gripper {command} failed: {exc}")

    def request_stop(self):
        if self.mode == "gripper" and self.gripper_goal is not None:
            self.gripper_goal.cancel_goal_async()
        elif self.execute and self.mode not in {"idle", "shadow"}:
            self._stop("shutdown")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.pool.shutdown(wait=False, cancel_futures=True)
        for camera in self.cameras.values():
            camera.close()


def _check_server(url, timeout, expected):
    proxy = ServerProxy(
        url, transport=_TimeoutTransport(timeout), allow_none=False, use_builtin_types=True
    )
    metadata = proxy.health()
    if metadata.get("action_dim") != 7 or metadata.get("units") != "degrees":
        raise RuntimeError(f"Unexpected policy metadata: {metadata}")
    if metadata.get("repo_id") != expected["repo_id"] or metadata.get("experiment") != expected["experiment"]:
        raise RuntimeError(f"Wrong policy loaded: {metadata}")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--shadow", action="store_true", help="infer and log; never publish commands")
    mode.add_argument("--execute", action="store_true", help="move only while deadman is held")
    parser.add_argument("--server")
    parser.add_argument("--repo-id", help="expected dataset; overrides pi05_runtime.repo_id")
    parser.add_argument("--exp-name", help="expected experiment; overrides pi05_runtime.experiment")
    parser.add_argument("--preview", action="store_true", help="publish RViz prediction animation; shadow-only")
    parser.add_argument("--preview-once", action="store_true", help="preview one prediction block, then hold it")
    parser.add_argument("--save-preview", type=Path, help="save one prediction to JSON and exit; no RViz needed")
    args = parser.parse_args()
    if (args.preview or args.preview_once or args.save_preview is not None) and not args.shadow:
        parser.error("--preview/--preview-once/--save-preview require --shadow")
    if args.save_preview is not None:
        args.save_preview = args.save_preview.expanduser().resolve()
        if args.preview or args.preview_once:
            parser.error("--save-preview cannot be combined with --preview/--preview-once")
        if args.save_preview.exists():
            parser.error(f"Preview file already exists; choose a new path: {args.save_preview}")
        if not args.save_preview.parent.is_dir():
            parser.error(f"Preview directory does not exist: {args.save_preview.parent}")
    config, base = load_config(args.config)
    runtime_overrides = dict(config.get("pi05_runtime") or {})
    config["pi05_runtime"] = runtime_overrides
    if args.repo_id is not None:
        runtime_overrides["repo_id"] = args.repo_id
    if args.exp_name is not None:
        runtime_overrides["experiment"] = args.exp_name
    if not config.get("ros_joystick") or set(config["cameras"]) != {"wrist", "global"}:
        parser.error("runtime requires ros_joystick plus wrist and global cameras")
    runtime = _runtime_config(config)
    server = args.server or runtime["server"]
    metadata = _check_server(server, float(runtime["max_inference_s"]), runtime)
    if args.execute:
        if not sys.stdin.isatty():
            parser.error("--execute requires an interactive terminal")
        confirmation = input("将允许模型在按住 deadman 时移动机械臂。输入 MOVE 确认：")
        if confirmation != "MOVE":
            parser.error("confirmation failed")

    print(json.dumps({"policy_server": metadata, "mode": "execute" if args.execute else "shadow"},
                     ensure_ascii=False, indent=2), flush=True)
    cameras = {}
    node = executor = None
    try:
        for name, camera_config in config["cameras"].items():
            cameras[name] = Camera(camera_config, base)
        time.sleep(1)
        for camera in cameras.values():
            camera.snapshot()
        rclpy.init()
        node = Pi05Runtime(config, cameras, execute=args.execute, server=server,
                           preview=args.preview, preview_once=args.preview_once,
                           save_preview=args.save_preview)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        print(
            "影子模式：只记录预测，不会移动机械臂。" if args.shadow else
            "执行模式：先松开 deadman，再按住以移动；松开立即停止。",
            flush=True,
        )
        if args.save_preview is not None:
            while rclpy.ok() and not node.preview.sent and node.preview.error is None:
                executor.spin_once(timeout_sec=0.1)
            if node.preview.error is not None:
                raise RuntimeError("Could not save prediction preview") from node.preview.error
            if not node.preview.sent:
                raise RuntimeError("Runtime stopped before saving a prediction")
            print(f"预测已保存：{args.save_preview}。相机端退出；关闭策略服务后再启动RViz。", flush=True)
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.request_stop()
            if executor is not None:
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline and node.mode in {"stopping", "gripper"}:
                    executor.spin_once(timeout_sec=0.05)
            node.close()
            node.destroy_node()
        else:
            for camera in cameras.values():
                camera.close()
        if executor is not None:
            executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
