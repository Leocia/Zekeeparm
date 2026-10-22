"""Bounded asynchronous raw recording; publish only completed episodes."""

import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import cv2
import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
          "wrist_yaw", "wrist_roll", "gripper")


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def vector(values):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError("Expected seven finite joint values")
    return result.tolist()


class Episode:
    def __init__(self, root, metadata, queue_size=16):
        self.name = time.strftime("episode_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        self.path = Path(root) / (self.name + ".partial")
        self.path.mkdir(parents=True, exist_ok=False)
        self.metadata = dict(metadata, schema_version=1, status="recording",
                             joint_names=list(JOINTS), units="degrees",
                             action_semantics="absolute_motor_target_after_mapping_and_clipping")
        write_json(self.path / "episode.json", self.metadata)
        self.queue = queue.Queue(maxsize=queue_size)
        self.error = None
        self.count = 0
        self.closed = False
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def _write(self):
        stream = None
        try:
            stream = (self.path / "frames.jsonl").open("w", encoding="utf-8")
        except Exception as exc:
            self.error = exc
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    if self.error is not None:
                        continue
                    record, images = item
                    for name, image in images.items():
                        directory = self.path / name
                        directory.mkdir(exist_ok=True)
                        target = directory / f"{record['index']:06d}.png"
                        if not cv2.imwrite(str(target), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                            raise OSError(f"Failed to write {target}")
                    stream.write(json.dumps(record, allow_nan=False) + "\n")
                    stream.flush()
                except Exception as exc:
                    self.error = exc
                finally:
                    self.queue.task_done()
        finally:
            if stream is not None:
                try:
                    stream.flush()
                    os.fsync(stream.fileno())
                    stream.close()
                except Exception as exc:
                    self.error = exc

    def append(self, record, images):
        if self.closed:
            raise RuntimeError("Episode is closed")
        if self.error:
            raise RuntimeError("Episode writer failed") from self.error
        if set(images) != set(self.metadata["cameras"]):
            raise ValueError("Camera set changed within episode")
        record = dict(record, index=self.count,
                      state=vector(record["state"]), action=vector(record["action"]))
        copies = {}
        for name, image in images.items():
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError("Expected uint8 BGR images")
            copies[name] = image.copy()
        try:
            self.queue.put_nowait((record, copies))
        except queue.Full as exc:
            raise RuntimeError("Writer queue full; episode must be marked aborted") from exc
        self.count += 1

    def finish(self, status, reason=""):
        if self.closed:
            raise RuntimeError("Episode already closed")
        if status not in {"success", "failure", "discarded", "aborted"}:
            raise ValueError("Invalid episode status")
        self.closed = True
        self.queue.put(None)
        self.worker.join()
        self.metadata.update(status=status, reason=reason, frames=self.count)
        if not self.count:
            self.metadata.update(status="aborted", reason="empty episode")
        if self.error:
            self.metadata.update(status="aborted", reason=str(self.error))
        write_json(self.path / "episode.json", self.metadata)
        if self.count and status in {"success", "failure"} and self.error is None:
            self.path = self.path.rename(self.path.with_name(self.name))
        return self.path
