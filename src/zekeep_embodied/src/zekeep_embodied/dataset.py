"""Validate raw episodes, create video previews, and export LeRobot v3 datasets."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .record import check_sample
from .storage import JOINTS, vector, write_json


def inspect_episode(path):
    path = Path(path)
    metadata = json.loads((path / "episode.json").read_text())
    if metadata.get("schema_version") != 1 or metadata["joint_names"] != list(JOINTS):
        raise ValueError("Unknown schema or joint ordering")
    if metadata["units"] != "degrees":
        raise ValueError("Unexpected units")
    cameras = metadata["cameras"]
    if not cameras or not set(cameras) <= {"wrist", "global"}:
        raise ValueError("Invalid camera keys")
    rows = [json.loads(line) for line in (path / "frames.jsonl").read_text().splitlines()]
    if not rows or len(rows) != metadata["frames"]:
        raise ValueError("Empty episode or frame count mismatch")
    shapes = {}
    previous = {}
    previous_time = None
    for index, row in enumerate(rows):
        if row["index"] != index:
            raise ValueError("Non-contiguous frame index")
        vector(row["action"])
        images = {}
        for name in cameras:
            image = cv2.imread(str(path / name / f"{index:06d}.png"))
            if image is None:
                raise ValueError(f"Missing/corrupt image: {name}/{index}")
            if image.shape != shapes.setdefault(name, image.shape):
                raise ValueError("Camera resolution changed")
            images[name] = image
        check_sample(row["state"], images, row["camera_times"], row["camera_sequences"],
                     previous, row["observation_time"], row["action_time"],
                     metadata["max_sync_s"], metadata["max_cycle_s"])
        if previous_time is not None and row["observation_time"] <= previous_time:
            raise ValueError("Observation timestamps are not increasing")
        if "hardware" in metadata:
            limits = metadata["hardware"]["effective_robot"]["joint_limits"]
            for name, target in zip(JOINTS, row["action"]):
                if not limits[name][0] <= target <= limits[name][1]:
                    raise ValueError(f"Action outside configured limit: {name}")
        previous = row["camera_sequences"]
        previous_time = row["observation_time"]
    return metadata, rows, shapes


def preview(path, output):
    metadata, rows, shapes = inspect_episode(path)
    import av
    names = list(shapes)
    height = 360
    widths = [round(shapes[name][1] / shapes[name][0] * height / 2) * 2 for name in names]
    output = Path(output)
    # Never replace an existing review artifact implicitly.
    with output.open("xb") as file:
        with av.open(file, mode="w", format="mp4") as container:
            stream = container.add_stream("libx264", rate=int(metadata["fps"]))
            stream.width, stream.height = sum(widths), height
            stream.pix_fmt = "yuv420p"
            for row in rows:
                tiles = []
                for name, width in zip(names, widths):
                    frame = cv2.imread(str(Path(path) / name / f"{row['index']:06d}.png"))
                    frame = cv2.resize(frame, (width, height))
                    cv2.putText(frame, f"{name} frame {row['index']}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 1)
                    tiles.append(frame)
                image = av.VideoFrame.from_ndarray(np.concatenate(tiles, axis=1), format="bgr24")
                for packet in stream.encode(image):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
    # Retain measured times and state/action values for plotting or inspection.
    return {"video": str(output), "frames": len(rows), "timing": "nominal fps; raw JSONL preserves measured time"}


def resample_indices(rows, fps):
    times = np.array([row["observation_time"] for row in rows])
    grid = np.arange(int(np.floor((times[-1] - times[0]) * fps + 1e-6)) + 1) / fps + times[0]
    right = np.minimum(np.searchsorted(times, grid), len(times) - 1)
    left = np.maximum(right - 1, 0)
    indices = np.where(abs(times[left] - grid) <= abs(times[right] - grid), left, right)
    if np.max(abs(times[indices] - grid)) > .5 / fps + 1e-6:
        raise ValueError("Capture gap exceeds half an output frame; reject episode or use a lower export fps")
    if len(set(indices.tolist())) != len(indices):
        raise ValueError("Resampling would duplicate observations; use a lower export fps")
    return indices


def export(root, output, repo_id, fps=None, allow_simulated=False, *, openpi_format=False):
    if openpi_format:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    else:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    candidates = sorted(path.parent for path in Path(root).glob("episode_*/episode.json")
                        if not path.parent.name.endswith(".partial"))
    selected = []
    contract = None
    for path in candidates:
        metadata = json.loads((path / "episode.json").read_text())
        if metadata["status"] != "success":
            continue
        if metadata.get("simulated") and not allow_simulated:
            continue
        metadata, rows, shapes = inspect_episode(path)
        if openpi_format:
            if set(shapes) != {"wrist", "global"}:
                raise ValueError("pi05 Zekeep requires wrist and global cameras")
            if metadata["action_semantics"] != "absolute_motor_target_after_mapping_and_clipping":
                raise ValueError("pi05 Zekeep requires absolute motor targets in degrees")
            if not isinstance(metadata.get("task"), str) or not metadata["task"].strip():
                raise ValueError("pi05 Zekeep requires a non-empty task instruction")
        current = (metadata["cameras"], metadata.get("camera_devices"), metadata["installation_id"],
                   metadata.get("hardware"), metadata["action_semantics"], shapes,
                   metadata.get("simulated", False))
        if contract is not None and contract != current:
            raise ValueError("Cannot mix camera layouts, calibration, simulation, or action conventions")
        contract = current
        selected.append((path, metadata, rows, shapes))
    if not selected:
        raise ValueError("No successful eligible episodes; simulation requires --allow-simulated")
    fps = fps or selected[0][1]["fps"]
    if not isinstance(fps, int) or fps <= 0:
        raise ValueError("Export fps must be a positive integer")
    grids = [resample_indices(rows, fps) for _, _, rows, _ in selected]
    features = {
        "observation.state": {"dtype": "float32", "shape": (7,), "names": list(JOINTS)},
        "action": {"dtype": "float32", "shape": (7,), "names": list(JOINTS)},
    }
    for name, shape in selected[0][3].items():
        features[f"observation.images.{name}"] = {
            "dtype": "image" if openpi_format else "video", "shape": shape,
            "names": ["height", "width", "channels"]}
    output = Path(output)
    if output.exists() or output.with_name(output.name + ".partial").exists():
        raise FileExistsError(output)
    staging = output.with_name(output.name + ".partial")
    options = {"use_videos": False} if openpi_format else {"vcodec": "h264"}
    dataset = LeRobotDataset.create(repo_id=repo_id, fps=fps, features=features,
                                   root=staging, robot_type="zekeep", video_backend="pyav",
                                   image_writer_threads=0, **options)
    try:
        for (path, metadata, rows, shapes), indices in zip(selected, grids):
            for index in indices:
                row = rows[index]
                frame = {"observation.state": np.asarray(row["state"], dtype=np.float32),
                         "action": np.asarray(row["action"], dtype=np.float32), "task": metadata["task"]}
                for name in shapes:
                    image = cv2.imread(str(path / name / f"{row['index']:06d}.png"))
                    frame[f"observation.images.{name}"] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                dataset.add_frame(frame)
            if openpi_format:
                dataset.save_episode()
            else:
                dataset.save_episode(parallel_encoding=False)
    finally:
        if openpi_format:
            dataset.stop_image_writer()
        else:
            dataset.finalize()
    write_json(staging / "capture_provenance.json", {
        "source_episodes": [str(path) for path, _, _, _ in selected],
        "source_frame_indices": [indices.tolist() for indices in grids],
        "resampling": "nearest complete sample; no interpolation or duplicate observations",
        "units": "degrees", "joint_names": JOINTS, "capture_metadata": selected[0][1],
        "format": "openpi-lerobot-v2.1" if openpi_format else "lerobot-v3",
        "note": "Compute normalization using only the training split; never use validation episodes."})
    # Actually decode a sample, not just confirm that metadata exists.
    loaded = LeRobotDataset(repo_id=repo_id, root=staging, video_backend="pyav")
    sample = loaded[0]
    staging.rename(output)
    return {"root": str(output), "episodes": loaded.num_episodes, "frames": len(loaded),
            "sample_state_shape": list(sample["observation.state"].shape)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("check")
    check.add_argument("episode", type=Path)
    review = commands.add_parser("preview")
    review.add_argument("episode", type=Path)
    review.add_argument("--output", required=True, type=Path)
    convert = commands.add_parser("export")
    convert.add_argument("root", type=Path)
    convert.add_argument("--output", required=True, type=Path)
    convert.add_argument("--repo-id", default="local/zekeep_pick_place")
    convert.add_argument("--fps", type=int)
    convert.add_argument("--allow-simulated", action="store_true")
    args = parser.parse_args()
    if args.command == "check":
        metadata, rows, shapes = inspect_episode(args.episode)
        result = {"status": metadata["status"], "frames": len(rows), "cameras": shapes,
                  "simulated": metadata.get("simulated", False),
                  "feedback_freshness": metadata["feedback_freshness"]}
    elif args.command == "preview":
        result = preview(args.episode, args.output)
    else:
        result = export(args.root, args.output, args.repo_id, args.fps, args.allow_simulated)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
