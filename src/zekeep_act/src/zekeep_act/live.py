"""ACT execution; acquisition is shared with the other policies."""

from contextlib import ExitStack
import json
import sys
import time

from zekeep_embodied.capture import (Leader, check_workspace, open_cameras, record,
                                     record_leader, snapshot, terminal_command)
from zekeep_embodied.keyboard_deadman import KeyboardDeadman

from .gripper import gripper_target


def run_policy(settings, capture, base, checkpoint, *, execute=False, seconds=None):
    from .policy import Policy
    from .robot import RosArm

    check_workspace(capture["workspace"])
    policy = Policy(checkpoint, settings["training"]["device"], **settings["inference"])
    policy.check_capture(settings, capture)
    if execute:
        if not sys.stdin.isatty() or input("将允许 ACT 控制机械臂；确认现场安全后输入 MOVE：") != "MOVE":
            raise RuntimeError("Motion confirmation required")
    with ExitStack() as stack:
        keyboard = stack.enter_context(KeyboardDeadman(settings["control"]["joy_timeout_s"])) if execute else None
        cameras = open_cameras(stack, capture, base)
        devices = {name: camera.device_info for name, camera in cameras.items()}
        if devices != policy.contract["camera_devices"]:
            raise RuntimeError("Live camera identity/settings differ from the training capture")
        robot = RosArm(settings, capture, readonly=not execute, keyboard=keyboard)
        stack.callback(robot.close)
        robot.wait_ready()
        state, images, *_ = snapshot(robot, cameras, capture, {})
        policy.infer(images, state)  # Warm CUDA before permitting any movement.
        policy.reset()
        print("全局按住 m 启动，松开暂停；h 回车暂停 | e 回车退出；停止后松开 m 再按。" if execute else
              "shadow：只预测，不下发动作。", flush=True)
        started = time.monotonic()
        previous = {}
        was_active = False
        try:
            while seconds is None or time.monotonic() - started < seconds:
                tick = time.monotonic()
                command = terminal_command()
                if execute:
                    command = command.replace("m", "").strip()
                if command == "e":
                    break
                if command == "h":
                    robot.stop("operator pause")
                if was_active and not robot.active:
                    policy.reset()
                    previous = {}
                    print(f"已停止：{robot.stop_reason}；松开 m 后重新按住才能继续。", flush=True)
                was_active = robot.active
                if execute and keyboard.ready and not robot.active:
                    try:
                        policy.reset()
                        robot.start(deadman=True)
                        previous = {}
                    except RuntimeError as exc:
                        keyboard.disarm()
                        print(f"未启动：{exc}", flush=True)
                if not execute or robot.active:
                    try:
                        state, images, _, sequences, _, source_time = snapshot(robot, cameras, capture, previous)
                        action, duration = policy.infer(images, state)
                        raw_action = list(policy.raw_action)
                        policy_action = list(action)
                        if duration > settings["control"]["inference_timeout_s"]:
                            raise RuntimeError(f"ACT inference exceeded deadline: {duration:.3f}s")
                        if execute:
                            action = robot.command(action, source_time)
                        else:
                            action[6] = gripper_target(action[6], settings["gripper"], robot.motor_limits[6])
                        print(json.dumps({"mode": "execute" if execute else "shadow",
                                          "inference_ms": round(duration * 1000, 2),
                                          "action_deg": action, "raw_action_deg": raw_action,
                                          "policy_action_deg": policy_action,
                                          "state_deg": state,
                                          "gripper_mode": "position"}, ensure_ascii=False), flush=True)
                        previous = sequences
                    except Exception as exc:
                        robot.stop(str(exc))
                        policy.reset()
                        if not execute:
                            raise
                        print(f"已停止：{exc}", flush=True)
                was_active = robot.active
                time.sleep(max(0, 1 / capture["fps"] - (time.monotonic() - tick)))
        finally:
            robot.stop("policy shutdown")
            policy.reset()
