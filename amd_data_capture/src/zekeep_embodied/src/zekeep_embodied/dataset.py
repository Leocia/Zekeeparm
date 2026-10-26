"""Validate raw episodes and create CPU video previews."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .record import check_sample
from .storage import JOINTS, vector


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("check")
    check.add_argument("episode", type=Path)
    review = commands.add_parser("preview")
    review.add_argument("episode", type=Path)
    review.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "check":
        metadata, rows, shapes = inspect_episode(args.episode)
        result = {"status": metadata["status"], "frames": len(rows), "cameras": shapes,
                  "simulated": metadata.get("simulated", False),
                  "feedback_freshness": metadata["feedback_freshness"]}
    else:
        result = preview(args.episode, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
