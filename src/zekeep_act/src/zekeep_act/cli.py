"""ACT collection, local training and guarded ROS execution."""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import sys

from .config import check_workspace, load_config


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "configs/act.yaml"


def doctor(settings, capture, base):
    versions = {name: importlib.metadata.version(name)
                for name in ("lerobot", "torch", "torchvision", "numpy", "av")}
    if versions["lerobot"] != "0.4.4":
        raise RuntimeError("This package requires LeRobot 0.4.4")
    from lerobot.policies.act.modeling_act import ACTPolicy  # noqa: F401
    from zekeep_msgs.msg import ArmStatus, JointMotorState  # noqa: F401
    for path in (capture["teleop_config"], capture["ros_joystick"]["hardware_config"]):
        if not Path(path).is_file():
            raise FileNotFoundError(path)
    for camera in capture["cameras"].values():
        if camera["type"] == "orbbec":
            for path in (Path(os.environ.get("ZKEEP_VISION_PYTHON") or camera["python"]).expanduser(), base / camera["config"]):
                if not path.is_file():
                    raise FileNotFoundError(path)
    import torch
    result = {"versions": versions, "cuda_available": torch.cuda.is_available(),
              "output": settings["output"], "fps": capture["fps"], "image_size": settings["image_size"],
              "episodes": settings["episodes"], "workspace_confirmed": settings["workspace"]["confirmed"],
              "robot_or_cameras_opened": False}
    if settings["training"]["device"] == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA training selected but CUDA is unavailable")
    if torch.cuda.is_available():
        device = torch.cuda.get_device_properties(0)
        # is_available() alone also returns True for wheels lacking this GPU architecture.
        test = (torch.ones(1, device="cuda") * 2).item()
        if test != 2:
            raise RuntimeError("CUDA arithmetic self-check failed")
        result.update(gpu=device.name, gpu_memory_mib=device.total_memory // 1024 ** 2)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    subparsers = {}
    for name, help_text in (
        ("doctor", "检查环境与 CUDA，不连接机械臂或相机"),
        ("camera-check", "仅检查双相机"),
        ("enable", "现场人工确认后使能，不开始运动"),
        ("record", "兼容入口；统一数采见 tools/collect.sh record"),
        ("record-leader", "兼容入口；统一数采见 tools/collect.sh record-leader"),
        ("check", "检查原始示范"),
        ("preview", "导出单条示范视频"),
        ("prepare", "按完整 episode 划分并导出 45/5 数据集"),
        ("train", "本地训练 ACT"),
        ("validate", "计算验证集预测误差，不移动机械臂"),
        ("shadow", "读取现场图像和状态，仅预测"),
        ("execute", "人工确认后全局按住 m 执行，松开暂停"),
    ):
        child = commands.add_parser(name, help=help_text)
        child.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
        subparsers[name] = child
    subparsers["record"].add_argument("--output", type=Path, help="原始示范目录")
    subparsers["record-leader"].add_argument("--output", type=Path, help="原始示范目录；勿与手柄示范混合")
    subparsers["check"].add_argument("root", type=Path, nargs="?")
    subparsers["preview"].add_argument("episode", type=Path)
    subparsers["preview"].add_argument("--output", type=Path, required=True)
    subparsers["prepare"].add_argument("--raw", type=Path)
    subparsers["prepare"].add_argument("--output", type=Path)
    subparsers["train"].add_argument("--dataset", type=Path)
    subparsers["train"].add_argument("--output", type=Path, required=True)
    subparsers["train"].add_argument("--steps", type=int)
    subparsers["validate"].add_argument("checkpoint", type=Path)
    subparsers["validate"].add_argument("--dataset", type=Path)
    subparsers["validate"].add_argument("--max-frames", type=int, default=100)
    for name in ("shadow", "execute"):
        subparsers[name].add_argument("checkpoint", type=Path)
        subparsers[name].add_argument("--seconds", type=float)
    subparsers["camera-check"].add_argument("--seconds", type=float, default=10)
    args = parser.parse_args(argv)
    for key in ("seconds", "steps", "max_frames"):
        value = getattr(args, key, None)
        if value is not None:
            from .config import positive
            try:
                positive(value, key)
            except ValueError as exc:
                parser.error(str(exc))
    try:
        settings, capture, base = load_config(args.config)
        dataset = getattr(args, "dataset", None) or Path(settings["output"]) / "dataset"
        result = None
        if args.command == "doctor":
            result = doctor(settings, capture, base)
        elif args.command == "camera-check":
            from zekeep_embodied.record import run
            run(capture, base, preview=True, seconds=args.seconds)
        elif args.command == "enable":
            from .robot import RosArm
            check_workspace(capture["workspace"])
            if not sys.stdin.isatty() or input("确认姿态、区域及急停后输入 ENABLE：") != "ENABLE":
                raise ValueError("Manual enable confirmation required")
            robot = RosArm(settings, capture, readonly=True)
            try:
                robot._call("enable")
            finally:
                robot.close()
        elif args.command in {"record", "record-leader"}:
            from .live import record, record_leader
            if args.output:
                capture["output"] = str(args.output.expanduser().resolve())
            collector = record if args.command == "record" else record_leader
            collector(settings, capture, base)
        elif args.command == "check":
            from .data import audit
            result = audit(args.root or capture["output"])
            if result["invalid"]:
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 1
        elif args.command == "preview":
            from zekeep_embodied.dataset import preview
            result = preview(args.episode, args.output)
        elif args.command == "prepare":
            from .data import prepare
            result = prepare(settings, capture, raw=args.raw, output=args.output)
        elif args.command == "train":
            from .policy import train
            result = train(settings, capture, dataset, args.output, steps=args.steps)
        elif args.command == "validate":
            from .policy import validate
            result = validate(args.checkpoint, dataset, device=settings["training"]["device"],
                              max_frames=args.max_frames)
        else:
            from .live import run_policy
            run_policy(settings, capture, base, args.checkpoint, execute=args.command == "execute",
                       seconds=args.seconds)
        if result is not None:
            print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"ACT: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
