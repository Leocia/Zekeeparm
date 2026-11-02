"""Episode-level splits and the existing LeRobot exporter."""

from pathlib import Path
import random
import sys

from zekeep_embodied.dataset import export, inspect_episode, read_episode, resample_indices
from zekeep_embodied.storage import JOINTS, write_json, successful_episodes

from .config import fingerprint


def audit(root):
    report = {"success": [], "excluded": [], "invalid": []}
    paths = sorted(Path(root).glob("episode_*/episode.json"))
    for index, path in enumerate(paths, 1):
        print(f"检查 {index}/{len(paths)}: {path.parent.name}", file=sys.stderr, flush=True)
        try:
            metadata, rows = read_episode(path.parent)
            if metadata["status"] == "success" and not metadata.get("simulated"):
                resample_indices(rows, metadata["fps"])
            metadata, rows, _ = inspect_episode(path.parent)
            if (path.parent.name.endswith(".partial") or metadata["status"] != "success"
                    or metadata.get("simulated")):
                report["excluded"].append(str(path.parent))
            else:
                report["success"].append({"path": str(path.parent), "frames": len(rows)})
        except (ValueError, RuntimeError, OSError, KeyError) as exc:
            report["invalid"].append({"path": str(path.parent), "error": str(exc)})
    return report


def split_episodes(paths, validation_count, seed):
    if not 0 < validation_count < len(paths) or len(set(paths)) != len(paths):
        raise ValueError("Split requires distinct episodes and a nonempty train/validation set")
    paths = sorted(paths)
    random.Random(seed).shuffle(paths)
    return paths[validation_count:], paths[:validation_count]


def prepare(settings, capture, *, raw=None, output=None, allow_simulated=False):
    raw = Path(raw or capture["output"]).expanduser().resolve()
    output = Path(output or Path(settings["output"]) / "dataset").expanduser().resolve()
    paths = successful_episodes(raw)
    if len(paths) != settings["episodes"]:
        raise ValueError(f"Expected {settings['episodes']} successful episodes, found {len(paths)}")
    expected = fingerprint(capture)
    contract = None
    simulated = False
    print(f"预检 {len(paths)} 条记录的状态、配置和时间戳（不解码图片）", file=sys.stderr, flush=True)
    for path in paths:
        metadata, rows = read_episode(path)
        try:
            resample_indices(rows, capture["fps"])
        except ValueError as exc:
            raise ValueError(f"{path}: {exc}") from exc
        if metadata.get("simulated") and not allow_simulated:
            raise ValueError(f"Simulated data cannot enter real training: {path}")
        simulated |= bool(metadata.get("simulated"))
        identities = [metadata[key] for key in ("capture_fingerprint", "act_fingerprint") if key in metadata]
        if (not identities or any(value != expected for value in identities) or metadata.get("fps") != capture["fps"]
                or metadata.get("action_semantics") != "absolute_motor_target_after_mapping_and_clipping"
                or set(metadata["cameras"]) != {"wrist", "global"}):
            raise ValueError(f"Episode does not match the current ACT capture contract: {path}")
        current = [metadata.get(key) for key in (
            "hardware", "camera_devices", "cameras", "installation_id", "workspace", "simulated")]
        if contract is not None and current != contract:
            raise ValueError("Cannot mix hardware calibration, camera layouts, or workspaces")
        contract = current
    train, validation = split_episodes(paths, settings["validation_episodes"], settings["seed"])
    staging = output.with_name(output.name + ".partial")
    if output.exists() or staging.exists():
        raise FileExistsError(output)
    staging.mkdir(parents=True)
    for name, selected in (("train", train), ("validation", validation)):
        export(raw, staging / name, settings["repo_id"] + "_" + name, capture["fps"],
               allow_simulated=allow_simulated, episode_paths=selected,
               image_size=settings["image_size"])
    manifest = {"fingerprint": expected, "fps": capture["fps"], "image_size": settings["image_size"],
                "repo_id": settings["repo_id"], "seed": settings["seed"], "simulated": simulated,
                "units": "degrees", "joint_names": list(JOINTS),
                "action_semantics": "absolute_motor_target_after_mapping_and_clipping",
                "train": [str(path) for path in train], "validation": [str(path) for path in validation],
                "workspace": capture["workspace"], "task": capture["task"],
                "camera_devices": metadata.get("camera_devices")}
    write_json(staging / "split.json", manifest)
    staging.rename(output)
    return {"dataset": str(output), "train_episodes": len(train), "validation_episodes": len(validation)}
