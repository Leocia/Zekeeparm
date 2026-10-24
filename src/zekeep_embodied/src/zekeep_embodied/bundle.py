"""Audit raw demonstrations and deliver portable, checksummed train/validation archives."""

import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import tarfile


def digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        result.update(block)
    return result.hexdigest()


def eligible(path):
    from .dataset import inspect_episode

    if path.name.endswith(".partial") or path.resolve().name.endswith(".partial"):
        raise ValueError("Incomplete episode (.partial)")
    metadata = json.loads((path / "episode.json").read_text())
    if metadata.get("status") != "success" or metadata.get("simulated", False):
        raise ValueError("Requires a successful, non-simulated episode")
    metadata, rows, shapes = inspect_episode(path)
    if not isinstance(metadata.get("task"), str) or not metadata["task"].strip():
        raise ValueError("Missing task instruction")
    if metadata.get("action_semantics") != "absolute_motor_target_after_mapping_and_clipping":
        raise ValueError("Expected absolute motor targets in degrees")
    return metadata, len(rows), shapes


def episodes(root):
    if not root.is_dir():
        raise NotADirectoryError(root)
    paths = sorted(root.glob("episode_*"))
    if not paths:
        raise ValueError(f"No episodes in {root}")
    return paths


def audit(root, output):
    paths = episodes(root)
    report = {"root": str(root.resolve()), "eligible": [], "excluded": []}
    for path in paths:
        try:
            metadata, frames, _ = eligible(path)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            report["excluded"].append({"episode": path.name, "reason": str(exc)})
        else:
            report["eligible"].append({"episode": path.name, "frames": frames,
                                       "task": metadata["task"]})
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    return {"report": str(output), "eligible": len(report["eligible"]),
            "excluded": len(report["excluded"])}


def bundle(train, validation, output):
    partial = output.with_name(output.name + ".partial")
    if output.exists() or output.is_symlink() or partial.exists() or partial.is_symlink():
        raise FileExistsError(output)
    selected = []
    seen_paths = set()
    contract = None
    for split, root in (("train", train), ("validation", validation)):
        for path in episodes(root):
            resolved = path.resolve(strict=True)
            if resolved in seen_paths:
                raise ValueError(f"Duplicate episode: {path}")
            seen_paths.add(resolved)
            metadata, frames, shapes = eligible(path)
            current = (metadata["cameras"], metadata.get("camera_devices"),
                       metadata["installation_id"], metadata.get("hardware"), shapes)
            if contract is not None and current != contract:
                raise ValueError("Cannot mix camera layouts or calibration")
            contract = current
            files = [Path("episode.json"), Path("frames.jsonl")]
            files.extend(Path(camera) / f"{i:06d}.png"
                         for camera in sorted(shapes) for i in range(frames))
            selected.append((split, path, frames, files))
    manifest = {"format": "zekeep-raw-bundle-v1", "episodes": [], "sha256": {}}
    seen_content = set()
    # PNGs are already compressed; an uncompressed tar avoids extra CPU and dependencies.
    with partial.open("xb") as target, tarfile.open(fileobj=target, mode="w") as archive:
        for split, path, frames, files in selected:
            content = hashlib.sha256()
            for relative in files:
                name = f"{split}/{path.name}/{relative.as_posix()}"
                with (path / relative).open("rb") as source:
                    checksum = digest(source)
                    source.seek(0)
                    info = tarfile.TarInfo(name)
                    info.size = os.fstat(source.fileno()).st_size
                    info.mode = 0o644
                    archive.addfile(info, source)
                manifest["sha256"][name] = checksum
                content.update(f"{relative.as_posix()}\0{checksum}\n".encode())
            identity = content.hexdigest()
            if identity in seen_content:
                raise ValueError(f"Duplicate episode content: {path}")
            seen_content.add(identity)
            manifest["episodes"].append({"path": f"{split}/{path.name}", "frames": frames})
        payload = json.dumps(manifest, ensure_ascii=False, indent=2).encode()
        info = tarfile.TarInfo("bundle.json")
        info.size, info.mode = len(payload), 0o644
        archive.addfile(info, io.BytesIO(payload))
    result = verify(partial)
    # Publish without overwriting a concurrent writer; failures retain the .partial archive.
    os.link(partial, output)
    partial.unlink()
    return dict(result, archive=str(output))


def verify(path):
    actual = {}
    manifest = None
    with tarfile.open(path, mode="r|") as archive:
        for member in archive:
            name = member.name
            parts = PurePosixPath(name).parts
            if (not member.isfile() or not parts or name.startswith("/")
                    or ".." in parts or PurePosixPath(name).as_posix() != name):
                raise ValueError(f"Unsafe archive member: {name}")
            if name in actual or (name == "bundle.json" and manifest is not None):
                raise ValueError(f"Duplicate archive member: {name}")
            with archive.extractfile(member) as stream:
                if name == "bundle.json":
                    manifest = json.load(stream)
                else:
                    actual[name] = digest(stream)
    if (not isinstance(manifest, dict) or manifest.get("format") != "zekeep-raw-bundle-v1"
            or not actual or actual != manifest.get("sha256")):
        raise ValueError("Missing manifest or SHA-256/file-list mismatch")
    return {"archive": str(path), "files": len(actual), "verified": True}


def doctor(config_path):
    from .record import load_config

    config, base = load_config(config_path)
    modules = ("numpy", "cv2", "yaml", "av")
    missing = [name for name in modules if importlib.util.find_spec(name) is None]
    paths = [Path(config_path), Path(config["teleop_config"])]
    for camera in config["cameras"].values():
        if camera["type"] == "orbbec":
            paths += [Path(os.environ.get("ZKEEP_VISION_PYTHON", camera["python"])).expanduser(), (base / camera["config"]).resolve()]
    if config.get("ros_joystick"):
        paths.append(Path(config["ros_joystick"]["hardware_config"]))
    missing += [str(path) for path in paths if not path.is_file()]
    if missing:
        raise RuntimeError("Missing data dependencies/configuration: " + ", ".join(missing))
    return {"data_environment": "ok", "config": str(config_path),
            "note": "No devices opened; use camera-check and supervised capture to verify hardware. "
                    "ROS/teleoperation runtime, model weights and GPU are not checked."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("doctor")
    check.add_argument("--config", required=True, type=Path)
    review = commands.add_parser("audit")
    review.add_argument("root", type=Path)
    review.add_argument("--output", required=True, type=Path)
    pack = commands.add_parser("bundle")
    pack.add_argument("--train", required=True, type=Path)
    pack.add_argument("--validation", required=True, type=Path)
    pack.add_argument("--output", required=True, type=Path)
    validate = commands.add_parser("verify")
    validate.add_argument("archive", type=Path)
    args = parser.parse_args()
    if args.command == "doctor":
        result = doctor(args.config)
    elif args.command == "audit":
        result = audit(args.root, args.output)
    elif args.command == "bundle":
        result = bundle(args.train, args.validation, args.output)
    else:
        result = verify(args.archive)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
