"""Interactive demonstration recorder. Hardware starts only with --execute."""

import argparse
import dataclasses
import json
import math
from pathlib import Path
import select
import sys
import threading
import time
import uuid

import numpy as np
import yaml

from .cameras import Camera
from .storage import Episode, JOINTS, vector, write_json


ROS_ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
COMMAND_ALIASES = {"s": "start", "p": "save", "w": "fail", "d": "discard", "e": "q"}


def load_config(path):
    path = Path(path).expanduser().resolve()
    with path.open() as stream:
        config = yaml.safe_load(stream)
    for key in ("fps", "max_sync_s", "max_cycle_s", "queue_size", "max_target_step_deg"):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if int(config["fps"]) != config["fps"] or int(config["queue_size"]) != config["queue_size"]:
        raise ValueError("fps and queue_size must be integers")
    if not config.get("task", "").strip() or not config.get("installation_id", "").strip():
        raise ValueError("task and installation_id are required")
    if not config.get("cameras"):
        raise ValueError("At least one camera is required")
    for name in config["cameras"]:
        if name not in {"wrist", "global"}:
            raise ValueError("Camera names must be wrist or global")
    for key in ("teleop_config", "output"):
        config[key] = str((path.parent / config[key]).resolve())
    for camera in config["cameras"].values():
        if camera["type"] == "orbbec":
            interpreter = path.parent / Path(camera["python"]).expanduser()
            # Preserve the final venv symlink so Python keeps its isolated environment.
            camera["python"] = str(interpreter.parent.resolve() / interpreter.name)
    ros = config.get("ros_joystick")
    if ros:
        ros["hardware_config"] = str((path.parent / ros["hardware_config"]).resolve())
        if not ros.get("arm_namespace", "").strip("/"):
            raise ValueError("ros_joystick.arm_namespace is required")
        for key in ("deadman_button", "close_button", "open_button"):
            if not isinstance(ros.get(key), int) or ros[key] < 0:
                raise ValueError(f"ros_joystick.{key} must be a nonnegative integer")
        for key in ("max_state_age_s", "connect_timeout_s"):
            if not math.isfinite(ros[key]) or ros[key] <= 0:
                raise ValueError(f"ros_joystick.{key} must be positive and finite")
    return config, path.parent


def check_sample(state, images, camera_times, camera_sequences, previous_sequences,
                 observation_time, action_time, max_sync_s, max_cycle_s):
    vector(state)
    if not images or set(images) != set(camera_times) or set(images) != set(camera_sequences):
        raise ValueError("Incomplete camera sample")
    times = list(camera_times.values()) + [observation_time, action_time]
    if not all(math.isfinite(value) for value in times):
        raise ValueError("Invalid timestamps")
    if action_time < observation_time or action_time - observation_time > max_cycle_s:
        raise RuntimeError("Observation-to-action interval exceeded")
    if max(times) - min(times) > max_sync_s:
        raise RuntimeError("Camera/state synchronization tolerance exceeded")
    for name in images:
        if camera_sequences[name] <= previous_sequences.get(name, 0):
            raise RuntimeError(f"Duplicate camera frame: {name}")


class Hardware:
    def __init__(self, path, max_target_step_deg=2.0):
        from lerobot_teleoperator_zhongling import ZhonglingLeader, ZhonglingLeaderConfig
        from lerobot_robot_seeed_b601 import SeeedB601DMFollower, SeeedB601DMFollowerConfig
        with Path(path).open() as stream:
            self.source = yaml.safe_load(stream)
        leader_config = dict(self.source["teleop"])
        follower_config = dict(self.source["robot"])
        if leader_config.pop("type") != "zhongling_leader":
            raise ValueError("Only the calibrated Zhongling leader is supported")
        if follower_config.pop("type") != "seeed_b601_dm_follower":
            raise ValueError("Only the calibrated DM follower is supported")
        if follower_config.get("cameras"):
            raise ValueError("Configure cameras in record.yaml, not inside the follower")
        self.leader = ZhonglingLeader(ZhonglingLeaderConfig(**leader_config))
        self.robot = SeeedB601DMFollower(SeeedB601DMFollowerConfig(**follower_config))
        self.max_target_step_deg = max_target_step_deg
        if tuple(self.robot.motor_names) != JOINTS:
            raise ValueError("Unexpected motor order")
        config = self.robot.config
        velocities = config.pos_vel_velocity
        if not isinstance(velocities, list):
            velocities = [velocities] * len(JOINTS)
        if len(velocities) != len(JOINTS) or any(not math.isfinite(v) or v <= 0 for v in velocities):
            raise ValueError("Expected seven positive finite motor velocities")
        for name in JOINTS:
            low, high = config.joint_limits[name]
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError(f"Invalid motor limits: {name}")
            if config.joint_directions[name] not in (-1, 1):
                raise ValueError(f"Unverified direction/scale: {name}")
        if not 0 < config.force_pos_torque_ration <= 1:
            raise ValueError("Invalid gripper torque ratio")
        self.source["effective_robot"] = dataclasses.asdict(self.robot.config)
        self.source["effective_leader"] = dataclasses.asdict(self.leader.config)

    def connect(self):
        # Never invoke the plugin's calibration path (it writes hardware zero).
        self.leader.connect()
        self.robot.connect(calibrate=False)

    def observation(self):
        for motor in self.robot.motors.values():
            motor.request_feedback()
        self.robot.bus.poll_feedback_once()  # Do not swallow bus errors or fill zero.
        values = []
        for name in JOINTS:
            state = self.robot.motors[name].get_state()
            if state is None:
                raise RuntimeError(f"Missing feedback: {name}")
            # DM feedback low nibble is enabled=1 / disabled=0; other codes are faults.
            if state.status_code not in (0, 1):
                raise RuntimeError(f"Motor fault {name}: {state.status_code}")
            values.append(math.degrees(state.pos))
        return vector(values)

    def action(self, state):
        from lerobot.robots.utils import ensure_safe_goal_position
        raw = self.leader.get_action()
        targets = {}
        config = self.robot.config
        for name in JOINTS:
            value = float(raw[f"{name}.pos"]) * config.joint_directions[name]
            if not math.isfinite(value):
                raise ValueError(f"Non-finite leader target: {name}")
            low, high = config.joint_limits[name]
            targets[name] = min(high, max(low, value))
        if config.max_relative_target is not None:
            targets = ensure_safe_goal_position(
                {name: (targets[name], state[i]) for i, name in enumerate(JOINTS)},
                config.max_relative_target,
            )
        targets = ensure_safe_goal_position(
            {name: (targets[name], state[i]) for i, name in enumerate(JOINTS)},
            self.max_target_step_deg,
        )
        # Plugin 1.0.0 hardcodes arm velocity to 32 rad/s. Honor the existing
        # calibrated velocity configuration here instead of propagating that bug.
        for i, name in enumerate(JOINTS):
            velocity = config.pos_vel_velocity[i] if isinstance(config.pos_vel_velocity, list) else config.pos_vel_velocity
            if not math.isfinite(velocity) or velocity <= 0:
                raise ValueError("Invalid motor velocity")
            motor = self.robot.motors[name]
            if name == "gripper":
                motor.send_force_pos(math.radians(targets[name]), math.radians(velocity),
                                     config.force_pos_torque_ration)
            else:
                motor.send_pos_vel(math.radians(targets[name]), math.radians(velocity))
        return vector([targets[name] for name in JOINTS])

    def close(self):
        # Preserve the established supervised teleoperation shutdown behavior.
        try:
            if self.robot.bus is not None:
                self.robot.disconnect()
        finally:
            if self.leader.is_connected:
                self.leader.disconnect()


def _load_ros_calibration(path):
    path = Path(path)
    with path.open() as stream:
        source = yaml.safe_load(stream) or {}
    model = source.get("default_model")
    overrides = ((source.get("models") or {}).get(model) or {}).get("overrides") or {}
    configured = overrides.get("joint_calibration") or {}
    names = ROS_ARM_JOINTS + ("gripper",)
    calibrations = []
    for name in names:
        values = configured.get(name) or {}
        required = ("direction", "gear_ratio", "encoder_zero", "ros_zero",
                    "hard_lower", "hard_upper")
        if any(key not in values for key in required):
            raise ValueError(f"Missing ROS hardware calibration: {name}")
        calibration = {key: float(values[key]) for key in required}
        if (calibration["direction"] not in (-1.0, 1.0)
                or calibration["gear_ratio"] <= 0.0
                or calibration["hard_lower"] >= calibration["hard_upper"]
                or not all(math.isfinite(value) for value in calibration.values())):
            raise ValueError(f"Invalid ROS hardware calibration: {name}")
        calibrations.append(calibration)
    gripper = overrides.get("gripper") or {}
    limits = gripper.get("position_limits") or {}
    if not all(key in limits and math.isfinite(float(limits[key])) for key in ("open", "close")):
        raise ValueError("Missing gripper position limits in ROS hardware config")
    return calibrations, float(limits["open"]), float(limits["close"])


def _ros_to_motor_degrees(values, calibrations):
    if len(values) != len(calibrations):
        raise ValueError("ROS target and calibration dimensions differ")
    result = []
    for value, calibration in zip(values, calibrations):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("Non-finite ROS target")
        motor = ((value - calibration["ros_zero"])
                 / (calibration["direction"] * calibration["gear_ratio"])
                 + calibration["encoder_zero"])
        result.append(math.degrees(motor))
    return result


class RosJoystickHardware:
    """Observe the ROS joystick path without owning or enabling the motors."""

    def __init__(self, config):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState, Joy
        from trajectory_msgs.msg import JointTrajectory
        from zekeep_msgs.msg import JointMotorState

        self.rclpy = rclpy
        self.config = config
        self.calibrations, open_position, close_position = _load_ros_calibration(
            config["hardware_config"]
        )
        self.open_target = _ros_to_motor_degrees(
            [open_position], self.calibrations[-1:]
        )[0]
        self.close_target = _ros_to_motor_degrees(
            [close_position], self.calibrations[-1:]
        )[0]
        namespace = config["arm_namespace"].strip("/")
        root = f"/{namespace}"
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("zekeep_joystick_recorder")
        self.node.create_subscription(
            JointState, root + "/joint_states", self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            JointMotorState, root + "/gripper/state", self._on_gripper_state,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            JointTrajectory, root + "/servo_joint_trajectory", self._on_trajectory, 10
        )
        self.node.create_subscription(
            Joy, config.get("joy_topic", "/joy"), self._on_joy,
            qos_profile_sensor_data,
        )
        self.arm_state = self.arm_target = None
        self.gripper_state = self.gripper_target = None
        self.state_time = self.gripper_time = 0.0
        self.buttons = []
        joint_limits = {}
        for name, calibration in zip(JOINTS, self.calibrations):
            bounds = _ros_to_motor_degrees(
                [calibration["hard_lower"], calibration["hard_upper"]],
                [calibration, calibration],
            )
            joint_limits[name] = [min(bounds), max(bounds)]
        self.source = {
            "control_source": "ros_joystick",
            "hardware_config": config["hardware_config"],
            "topics": {
                "state": root + "/joint_states",
                "gripper_state": root + "/gripper/state",
                "action": root + "/servo_joint_trajectory",
                "joy": config.get("joy_topic", "/joy"),
            },
            "effective_robot": {"joint_limits": joint_limits},
            "effective_leader": {"type": "ros_joystick"},
        }
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def connect(self):
        deadline = time.monotonic() + self.config["connect_timeout_s"]
        while time.monotonic() < deadline:
            if self.arm_state is not None and self.gripper_state is not None and self.buttons:
                if self.arm_target is None:
                    self.arm_target = list(self.arm_state)
                if self.gripper_target is None:
                    self.gripper_target = self.gripper_state
                return
            time.sleep(0.01)
        missing = []
        if self.arm_state is None:
            missing.append("joint_states")
        if self.gripper_state is None:
            missing.append("gripper/state")
        if not self.buttons:
            missing.append("joy")
        raise RuntimeError("ROS joystick recorder missing topics: " + ", ".join(missing))

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
            self.gripper_state = math.degrees(value)
            self.gripper_time = time.monotonic()

    def _on_trajectory(self, message):
        if len(message.points) != 1:
            return
        by_name = dict(zip(message.joint_names, message.points[0].positions))
        if all(name in by_name for name in ROS_ARM_JOINTS):
            values = [float(by_name[name]) for name in ROS_ARM_JOINTS]
            if all(math.isfinite(value) for value in values):
                self.arm_target = values

    def _on_joy(self, message):
        previous = self.buttons
        self.buttons = list(message.buttons)
        deadman = self._button(int(self.config["deadman_button"]))
        close = int(self.config["close_button"])
        open_ = int(self.config["open_button"])
        if deadman and not (self._button(close) and self._button(open_)):
            if self._rising(previous, close):
                self.gripper_target = self.close_target
            elif self._rising(previous, open_):
                self.gripper_target = self.open_target

    def _button(self, index):
        return 0 <= index < len(self.buttons) and bool(self.buttons[index])

    def _rising(self, previous, index):
        return self._button(index) and (index >= len(previous) or not previous[index])

    def observation(self):
        now = time.monotonic()
        if (self.arm_state is None or self.gripper_state is None
                or now - min(self.state_time, self.gripper_time)
                > self.config["max_state_age_s"]):
            raise RuntimeError("ROS arm or gripper feedback is stale")
        return vector(
            _ros_to_motor_degrees(self.arm_state, self.calibrations[:6])
            + [self.gripper_state]
        )

    def action(self, state):
        arm = (_ros_to_motor_degrees(self.arm_target, self.calibrations[:6])
               if self.arm_target is not None else state[:6])
        return vector(arm + [self.gripper_target if self.gripper_target is not None else state[6]])

    def close(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=1.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def simulate(config, frames):
    episode = Episode(config["output"], dict(config, simulated=True,
                      timestamp_basis="synthetic", feedback_freshness="synthetic"), config["queue_size"])
    try:
        for i in range(frames):
            stamp = i / config["fps"]
            images = {}
            for offset, name in enumerate(config["cameras"]):
                image = np.zeros((120, 160, 3), np.uint8)
                image[:, :, offset % 3] = (i * 7) % 255
                images[name] = image
            episode.append(dict(state=[0.] * 7, action=[float(i % 5)] * 7,
                                observation_time=stamp, action_time=stamp + .002,
                                camera_times={name: stamp for name in images},
                                camera_sequences={name: i + 1 for name in images}), images)
            # Simulation should exercise the writer, without outrunning real time.
            time.sleep(1 / config["fps"])
        return episode.finish("success")
    except BaseException:
        episode.finish("aborted", "simulation interrupted")
        raise


def run(config, base, preview=False, seconds=None, joystick=False):
    cameras = {}
    hardware = None
    episode = None
    previous_sequences = {}
    started = time.monotonic()
    metadata = dict(config, simulated=False, timestamp_basis="host_monotonic_receive_seconds",
                    feedback_freshness="SDK has no per-motor receive timestamp; host poll only")
    check_first, check_last = {}, {}
    check_samples, max_skew, duplicate_samples = 0, 0., 0
    try:
        for name, camera_config in config["cameras"].items():
            cameras[name] = Camera(camera_config, base)
        metadata["camera_devices"] = {name: getattr(camera, "device_info", camera.config)
                                      for name, camera in cameras.items()}
        time.sleep(1)
        for camera in cameras.values():
            camera.snapshot()
        if preview:
            import cv2
            snapshot_dir = Path(config["output"]) / ("camera_check_" + uuid.uuid4().hex[:8])
            snapshot_dir.mkdir(parents=True, exist_ok=False)
            for name, camera in cameras.items():
                if not cv2.imwrite(str(snapshot_dir / f"{name}.png"), camera.snapshot()[0]):
                    raise OSError("Cannot save camera check image")
            print(f"相机检查图片：{snapshot_dir}", flush=True)
        if not preview:
            hardware = (RosJoystickHardware(config["ros_joystick"]) if joystick else
                        Hardware(config["teleop_config"], config["max_target_step_deg"]))
            # Resolve JSON serialization and config failures before enabling motors.
            metadata["hardware"] = json.loads(json.dumps(hardware.source, default=str))
            print(("只监听 ROS 手柄控制；采集器不会使能或控制机械臂。" if joystick else
                   "即将使能从臂。退出会释放力矩，请扶稳机械臂；急停须就绪。"), flush=True)
            hardware.connect()
        print("相机预检中，输入 e 回车退出" if preview else
              "输入命令并回车：s 开始 | p 保存 | w 错误 | d 放弃 | e 退出", flush=True)
        started = time.monotonic()  # Requested duration excludes camera initialization.
        last_status = started
        interactive = sys.stdin.isatty()
        while seconds is None or time.monotonic() - started < seconds:
            tick = time.monotonic()
            command = None
            if interactive and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                command = line.strip() if line else "e"
                command = COMMAND_ALIASES.get(command, command)
            if command == "q":
                break
            if command == "start" and episode is None and not preview:
                episode = Episode(config["output"], metadata, config["queue_size"])
                previous_sequences = {}
                print(f"录制：{episode.path}", flush=True)
            if command in {"save", "fail", "discard"} and episode is not None:
                status = {"save": "success", "fail": "failure", "discard": "discarded"}[command]
                print(f"已结束：{episode.finish(status)}", flush=True)
                episode = None
            images, camera_times, sequences = {}, {}, {}
            for name, camera in cameras.items():
                images[name], camera_times[name], sequences[name] = camera.snapshot()
            state = hardware.observation() if hardware else [0.] * 7
            observed = time.monotonic()
            # Reject missing or stale images before any movement command.
            check_sample(state, images, camera_times, sequences,
                         previous_sequences if episode else {}, observed, observed,
                         config["max_sync_s"], config["max_cycle_s"])
            if preview:
                check_samples += 1
                max_skew = max(max_skew, max(camera_times.values()) - min(camera_times.values()))
                if any(sequences[name] <= check_last.get(name, (0, 0))[0] for name in sequences):
                    duplicate_samples += 1
                for name in sequences:
                    point = (sequences[name], camera_times[name])
                    check_first.setdefault(name, point)
                    check_last[name] = point
            if observed - tick > config["max_cycle_s"]:
                raise RuntimeError("Control cycle exceeded time budget before action")
            action = hardware.action(state) if hardware else state
            sent = time.monotonic()
            if episode:
                check_sample(state, images, camera_times, sequences, previous_sequences,
                             observed, sent, config["max_sync_s"], config["max_cycle_s"])
                episode.append(dict(state=state, action=action, observation_time=observed,
                                    action_time=sent, camera_times=camera_times,
                                    camera_sequences=sequences), images)
                previous_sequences = sequences
            if not interactive and sent - last_status >= 1:
                print(f"{'相机预检' if preview else '遥操作'} | cameras={list(images)} | "
                      f"work={(sent-tick)*1000:.1f} ms | frames={episode.count if episode else 0}", flush=True)
                last_status = sent
            time.sleep(max(0, 1 / config["fps"] - (time.monotonic() - tick)))
        if preview:
            measured_fps = {name: ((check_last[name][0] - first[0]) / (check_last[name][1] - first[1])
                                   if check_last[name][1] > first[1] else 0.)
                            for name, first in check_first.items()}
            report = dict(camera_devices=metadata["camera_devices"], samples=check_samples,
                          measured_fps=measured_fps, max_receive_skew_ms=max_skew * 1000,
                          duplicate_samples=duplicate_samples,
                          fresh_samples_ok=check_samples >= 2 and duplicate_samples == 0,
                          timestamp_basis="host receive time; not exposure synchronization")
            write_json(snapshot_dir / "camera_check.json", report)
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
            if not report["fresh_samples_ok"]:
                raise RuntimeError("Camera check has too few fresh samples or duplicate frames; check capture FPS")
    finally:
        # Close motor ownership first, even if disk finalization fails.
        try:
            if hardware:
                hardware.close()
        finally:
            try:
                if episode and not episode.closed:
                    print(f"未完成记录：{episode.finish('aborted', 'exit or capture error')}")
            finally:
                for camera in cameras.values():
                    try:
                        camera.close()
                    except Exception as exc:
                        print(f"Camera cleanup failed: {exc}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--camera-check", action="store_true")
    mode.add_argument("--execute", action="store_true", help="connect and enable real follower")
    mode.add_argument("--joystick", action="store_true", help="record existing ROS joystick control")
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config, base = load_config(args.config)
    if args.output:
        config["output"] = str(args.output.expanduser().resolve())
    if args.frames <= 0 or (args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0)):
        parser.error("frames and seconds must be positive")
    if args.simulate:
        print(simulate(config, args.frames))
    else:
        if (args.execute or args.joystick) and not sys.stdin.isatty():
            parser.error("--execute and --joystick require an interactive terminal")
        if args.joystick and not config.get("ros_joystick"):
            parser.error("--joystick requires ros_joystick configuration")
        run(config, base, preview=args.camera_check, seconds=args.seconds,
            joystick=args.joystick)


if __name__ == "__main__":
    main()
