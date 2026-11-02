"""Shared model-independent demonstrations using the existing guarded ROS collector."""

import argparse
from contextlib import ExitStack
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import select
import sys
import time

import numpy as np
import yaml

from .cameras import Camera
from .record import check_sample, load_config, _ros_to_motor_degrees
from .storage import Episode, JOINTS, vector, successful_episodes


def check_workspace(workspace):
    if workspace.get("confirmed") is not True or not str(workspace.get("description", "")).strip():
        raise ValueError("Confirm workspace bounds and description in the capture configuration before live operation")
    for key in ("x_m", "y_m"):
        bounds = workspace.get(key)
        if (not isinstance(bounds, list) or len(bounds) != 2
                or not all(isinstance(x, (float, int)) and math.isfinite(x) for x in bounds)
                or bounds[0] >= bounds[1]):
            raise ValueError(f"workspace.{key} requires [minimum, maximum] in base-frame meters")
    z = workspace.get("table_z_m")
    if not isinstance(z, (int, float)) or not math.isfinite(z):
        raise ValueError("workspace.table_z_m must be finite")


def fingerprint(capture):
    """Identify the physical mapping, camera layout and supervised area."""
    teleop = yaml.safe_load(Path(capture["teleop_config"]).read_text())
    hardware = yaml.safe_load(Path(capture["ros_joystick"]["hardware_config"]).read_text())
    camera_sources = {}
    for name, camera in capture["cameras"].items():
        if camera["type"] == "orbbec":
            source = Path(capture["_camera_base"]) / camera["config"]
            camera_sources[name] = yaml.safe_load(source.read_text())["camera"]
    values = {"teleop": teleop, "hardware": hardware, "cameras": capture["cameras"],
              "installation_id": capture["installation_id"], "workspace": capture["workspace"],
              "fps": capture["fps"], "camera_sources": camera_sources, "task": capture["task"]}
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def recording_metadata(capture):
    identity = fingerprint(capture)
    # Keep the old key for existing consumers of raw ACT recordings.
    return dict(capture, capture_fingerprint=identity, act_fingerprint=identity)


class Leader:
    def __init__(self, path):
        from lerobot_teleoperator_zhongling import ZhonglingLeader, ZhonglingLeaderConfig

        self.source = yaml.safe_load(Path(path).read_text())
        config = dict(self.source["teleop"])
        if config.pop("type") != "zhongling_leader":
            raise ValueError("Only the calibrated Zhongling leader is supported")
        self.leader = ZhonglingLeader(ZhonglingLeaderConfig(**config))
        self.directions = [self.source["robot"]["joint_directions"][name] for name in JOINTS]
        if tuple(self.leader.config.joint_names) != JOINTS or any(x not in (-1, 1) for x in self.directions):
            raise ValueError("Unexpected leader ordering/direction")

    def target(self):
        raw = self.leader.get_action()
        return vector([float(raw[f"{name}.pos"]) * direction for name, direction in zip(JOINTS, self.directions)])

    def close(self):
        if self.leader.is_connected:
            self.leader.disconnect()


def terminal_command():
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        return line.strip().lower() if line else "e"
    return ""


def open_cameras(stack, capture, base):
    cameras = {}
    for name, config in capture["cameras"].items():
        cameras[name] = Camera(config, base)
        stack.callback(cameras[name].close)
    deadline = time.monotonic() + 5
    while True:
        try:
            for camera in cameras.values():
                camera.snapshot()
            return cameras
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.05)


def snapshot(robot, cameras, capture, previous):
    with robot.lock:
        state = robot.observation(enabled=robot.active)
        state_times = [robot.arm_time, robot.gripper_time]
    images, times, sequences = {}, {}, {}
    for name, camera in cameras.items():
        images[name], times[name], sequences[name] = camera.snapshot()
    observed = time.monotonic()
    check_sample(state, images, times, sequences, previous, observed, observed,
                 capture["max_sync_s"], capture["max_cycle_s"])
    if max(list(times.values()) + state_times) - min(list(times.values()) + state_times) > capture["max_sync_s"]:
        raise RuntimeError("Camera and ROS feedback receive times are not synchronized")
    return state, images, times, sequences, observed, min(list(times.values()) + state_times)


def record(settings, capture, base):
    from zekeep_embodied.record import run

    check_workspace(capture["workspace"])
    if not sys.stdin.isatty():
        raise RuntimeError("Joystick recording requires an interactive terminal")
    count = len(successful_episodes(capture["output"]))
    if count >= settings["episodes"]:
        raise ValueError("Collection already has the requested successful episodes; choose a new output")
    print(f"手柄示教：已有成功 {count}/{settings['episodes']}；采集器只监听，不控制机械臂。", flush=True)
    run(recording_metadata(capture), base, joystick=True,
        max_episodes=settings["episodes"] - count)


def record_leader(settings, capture, base):
    from zekeep_act.robot import RosArm

    check_workspace(capture["workspace"])
    if not sys.stdin.isatty():
        raise RuntimeError("Leader recording requires an interactive terminal")
    count = len(successful_episodes(capture["output"]))
    if count >= settings["episodes"]:
        raise ValueError("Collection already has the requested successful episodes; choose a new output")
    episode = None
    with ExitStack() as stack:
        cameras = open_cameras(stack, capture, base)
        leader = Leader(capture["teleop_config"])
        stack.callback(leader.close)
        print("主臂须在已标定启动零位；连接主臂不会使能从臂。", flush=True)
        leader.leader.connect()
        robot = RosArm(settings, capture, leader=leader)
        stack.callback(robot.close)
        robot.wait_ready()
        metadata = dict(recording_metadata(capture), simulated=False,
                        timestamp_basis="host_monotonic_receive_seconds",
                        feedback_freshness="ROS receive time; not per-motor hardware timestamps",
                        camera_devices={name: camera.device_info for name, camera in cameras.items()},
                        hardware={"control_source": "ros_leader",
                                  "effective_robot": {"joint_limits": dict(zip(JOINTS, robot.motor_limits)),
                                                      "calibration": robot.calibrations},
                                  "effective_leader": dataclasses.asdict(leader.leader.config),
                                  "follow": {"fps": robot.fps, "velocity_rad_s": robot.follow_speeds}},
                        leader_session_zero_pwm=leader.leader._startup_pwm_centers)
        print("输入后回车：f 核对姿态并跟随 | s 录制 | p 成功保存并暂停 | w 失败保存 | h 暂停 | d 放弃 | e 退出", flush=True)
        previous = {}
        following = False
        status_time, status_count = time.monotonic(), 0
        try:
            while count < settings["episodes"]:
                tick = time.monotonic()
                command = terminal_command()
                if command == "e":
                    break
                if following and not robot.active:
                    following = False
                    if episode is not None:
                        print(f"异常记录保留：{episode.finish('aborted', robot.stop_reason)}", flush=True)
                        episode = None
                    print(f"已停止：{robot.stop_reason}；检查后重新输入 f。", flush=True)
                if command in {"p", "w", "d", "h"}:
                    robot.stop("operator pause")
                    following = False
                    if episode is not None:
                        status = {"p": "success", "w": "failure", "d": "discarded", "h": "aborted"}[command]
                        path = episode.finish(status, "operator pause" if command == "h" else "")
                        if episode.metadata["status"] == "success":
                            count += 1
                        print(f"{path} | 成功 {count}/{settings['episodes']}", flush=True)
                        episode = None
                    previous = {}
                if not following:
                    target = leader.target()
                if command == "f" and not following:
                    state = robot.observation()
                    bounded = [min(high, max(low, value)) for value, (low, high) in zip(target, robot.motor_limits)]
                    differences = np.abs(np.asarray(bounded) - state)
                    index = int(np.argmax(differences))
                    difference = float(differences[index])
                    if difference > settings["control"]["alignment_tolerance_deg"]:
                        joint = f"J{index + 1}" if index < 6 else "夹爪"
                        print(f"未启动跟随：{joint} 相差 {difference:.3f}°，"
                              f"超过 {settings['control']['alignment_tolerance_deg']:.1f}°；"
                              f"从臂电机角={state[index]:+.3f}°，主臂目标={bounded[index]:+.3f}°。"
                              "请调整主臂，使目标角接近从臂反馈，再输入 f。", flush=True)
                    else:
                        robot.start()
                        following = True
                        previous = {}
                        status_time, status_count = time.monotonic(), robot.command_count
                        print(f"已开始跟随：控制 {robot.fps} Hz，图像采集 {capture['fps']} Hz。", flush=True)
                if command == "s" and episode is None:
                    if not following:
                        print("先输入 f，完成姿态核对并开始跟随。", flush=True)
                    else:
                        episode = Episode(capture["output"], metadata, capture["queue_size"])
                        print(f"录制：{episode.path}", flush=True)
                if following:
                    try:
                        state, images, times, sequences, observed, source_time = snapshot(robot, cameras, capture, previous)
                        with robot.lock:
                            if not robot.active:
                                raise RuntimeError(robot.stop_reason or "Servo is not active")
                            action = _ros_to_motor_degrees(robot.target, robot.calibrations)
                            robot.source_time = source_time
                            sent = time.monotonic()
                        check_sample(state, images, times, sequences, previous, observed, sent,
                                     capture["max_sync_s"], capture["max_cycle_s"])
                        if episode is not None:
                            episode.append(dict(state=state, action=action, observation_time=observed,
                                                action_time=sent, camera_times=times, camera_sequences=sequences), images)
                        previous = sequences
                        if sent - status_time >= 1:
                            cycles = robot.command_count - status_count
                            print(f"跟随：{cycles / (sent - status_time):.1f} Hz | 目标 {robot.fps} Hz", flush=True)
                            status_time, status_count = sent, robot.command_count
                    except Exception as exc:
                        robot.stop(str(exc))
                time.sleep(max(0, 1 / capture["fps"] - (time.monotonic() - tick)))
        finally:
            robot.stop("recording shutdown")
            if episode is not None and not episode.closed:
                episode.finish("aborted", "recording interrupted")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("record", "record-leader"))
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--episodes", type=int, default=50, help="本目录的成功示范总数，默认 50")
    args = parser.parse_args(argv)
    if args.episodes <= 0:
        parser.error("episodes must be positive")
    try:
        capture, base = load_config(args.config)
        if set(capture["cameras"]) != {"wrist", "global"} or not capture.get("ros_joystick"):
            raise ValueError("Shared capture requires wrist/global cameras and ROS calibration")
        check_workspace(capture.get("workspace", {}))
        capture["output"] = str(args.output.expanduser().resolve())
        capture["_camera_base"] = str(base)
        settings = dict(episodes=args.episodes, control=capture.get("leader_control", {}), gripper={})
        if args.mode == "record-leader":
            for key in ("max_step_deg", "gripper_lower_tolerance_deg", "max_velocity_rad_s",
                        "alignment_tolerance_deg", "feedback_timeout_s", "status_timeout_s",
                        "joy_timeout_s", "inference_timeout_s", "service_timeout_s"):
                value = settings["control"].get(key)
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value <= 0):
                    raise ValueError(f"leader_control.{key} must be positive and finite")
        collector = record if args.mode == "record" else record_leader
        collector(settings, capture, base)
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"Capture: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
