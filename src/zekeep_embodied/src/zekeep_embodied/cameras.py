"""Fresh host-receive timestamped frames; camera threads never own motors."""

from pathlib import Path
import fcntl
import io
import json
from multiprocessing.connection import Connection
import os
import subprocess
import struct
import threading
import time

import cv2
import numpy as np


def resize_with_padding(image, size):
    """Keep the complete camera view when making square policy inputs."""
    if not isinstance(size, int) or size <= 0:
        raise ValueError("Image size must be a positive integer")
    height, width = image.shape[:2]
    scale = size / max(height, width)
    resized = cv2.resize(image, (max(1, round(width * scale)), max(1, round(height * scale))))
    result = np.zeros((size, size, 3), dtype=image.dtype)
    y, x = (size - resized.shape[0]) // 2, (size - resized.shape[1]) // 2
    result[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return result


class Camera:
    def __init__(self, config, base):
        self.config = config
        self.frame = None
        self.sequence = 0
        self.error = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.kind = config["type"]
        if self.kind == "orbbec":
            source = (base / config["config"]).resolve()
            reader, writer = os.pipe()
            self.connection = Connection(reader, writable=False)
            environment = dict(os.environ)
            environment.pop("PYTHONPATH", None)
            environment["PYTHONNOUSERSITE"] = "1"
            try:
                self.process = subprocess.Popen(
                    [str(Path(os.environ.get("ZKEEP_VISION_PYTHON") or config["python"]).expanduser()),
                     str(Path(__file__).with_name("orbbec_worker.py")), str(source), str(writer)],
                    pass_fds=(writer,), env=environment, stdout=subprocess.DEVNULL)
            except BaseException:
                self.connection.close()
                raise
            finally:
                os.close(writer)
            try:
                if not self.connection.poll(15):
                    raise RuntimeError("Orbbec worker startup timed out")
                try:
                    self.device_info = json.loads(self.connection.recv_bytes(1024 * 1024))
                except EOFError as exc:
                    raise RuntimeError("Orbbec worker exited before startup; check USB permissions and SDK environment") from exc
                if "error" in self.device_info:
                    raise RuntimeError(self.device_info["error"])
            except BaseException:
                self.connection.close()
                self.process.terminate()
                self.process.wait(timeout=5)
                raise
        elif self.kind == "opencv":
            fourcc = config.get("fourcc")
            if fourcc is not None and (not isinstance(fourcc, str) or len(fourcc) != 4):
                raise ValueError("fourcc must contain four characters, e.g. MJPG")
            self.device = cv2.VideoCapture(config["device"], cv2.CAP_V4L2)
            if not self.device.isOpened():
                self.device.release()
                raise RuntimeError(f"Cannot open camera {config['device']}")
            try:
                # Negotiate compressed transport first: C100 uses USB 2.0.
                if fourcc is not None:
                    self.device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                self.device.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
                self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
                self.device.set(cv2.CAP_PROP_FPS, config["fps"])
                self.device.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if "exposure" in config and config.get("auto_exposure") is not False:
                    raise ValueError("Manual exposure requires auto_exposure: false")
                controls = {}
                for key, prop in (("auto_exposure", cv2.CAP_PROP_AUTO_EXPOSURE),
                                  ("exposure", cv2.CAP_PROP_EXPOSURE), ("gain", cv2.CAP_PROP_GAIN),
                                  ("auto_white_balance", cv2.CAP_PROP_AUTO_WB),
                                  ("brightness", cv2.CAP_PROP_BRIGHTNESS), ("contrast", cv2.CAP_PROP_CONTRAST),
                                  ("saturation", cv2.CAP_PROP_SATURATION), ("hue", cv2.CAP_PROP_HUE),
                                  ("gamma", cv2.CAP_PROP_GAMMA), ("sharpness", cv2.CAP_PROP_SHARPNESS),
                                  ("backlight_compensation", cv2.CAP_PROP_BACKLIGHT)):
                    if key not in config:
                        continue
                    value = config[key]
                    if key in ("auto_exposure", "auto_white_balance"):
                        if not isinstance(value, bool):
                            raise ValueError(f"{key} must be a boolean")
                        # V4L2 raw modes: manual=1, aperture-priority auto=3.
                        value = (3 if value else 1) if key == "auto_exposure" else int(value)
                    elif isinstance(value, bool) or not isinstance(value, int):
                        raise ValueError(f"{key} must be an integer")
                    elif key in ("exposure", "gain") and value < (1 if key == "exposure" else 0):
                        raise ValueError(f"Invalid {key}={value}")
                    if not self.device.set(prop, value):
                        raise RuntimeError(f"Camera rejected {key}={value}")
                    controls[key] = self.device.get(prop)
                    if not np.isclose(controls[key], value, rtol=0, atol=.1):
                        raise RuntimeError(f"Camera rejected {key}={value}: actual={controls[key]}")
                # These two V4L2 controls have no OpenCV property equivalents.
                for key, control_id in (("power_line_frequency", 0x00980918),
                                        ("exposure_auto_priority", 0x009A0903)):
                    if key not in config:
                        continue
                    value = config[key]
                    if key == "exposure_auto_priority":
                        if not isinstance(value, bool):
                            raise ValueError(f"{key} must be a boolean")
                        value = int(value)
                    elif isinstance(value, bool) or not isinstance(value, int):
                        raise ValueError(f"{key} must be an integer")
                    data = bytearray(struct.pack("Ii", control_id, value))
                    try:
                        with open(config["device"], "rb+", buffering=0) as control_device:
                            fcntl.ioctl(control_device, 0xC008561C, data)  # VIDIOC_S_CTRL
                            fcntl.ioctl(control_device, 0xC008561B, data)  # VIDIOC_G_CTRL
                    except OSError as exc:
                        raise RuntimeError(f"Camera rejected {key}={value}") from exc
                    controls[key] = struct.unpack("Ii", data)[1]
                    if controls[key] != value:
                        raise RuntimeError(f"Camera rejected {key}={value}: actual={controls[key]}")
                actual = {key: self.device.get(prop) for key, prop in (
                    ("width", cv2.CAP_PROP_FRAME_WIDTH), ("height", cv2.CAP_PROP_FRAME_HEIGHT),
                    ("fps", cv2.CAP_PROP_FPS), ("fourcc_code", cv2.CAP_PROP_FOURCC))}
                code = int(actual.pop("fourcc_code"))
                actual["fourcc"] = "".join(chr((code >> (8 * i)) & 255) for i in range(4))
                for key in ("width", "height", "fps"):
                    if abs(actual[key] - config[key]) > .1:
                        raise RuntimeError(f"Camera rejected {key}={config[key]}: actual={actual[key]}")
                if fourcc is not None and actual["fourcc"] != fourcc:
                    raise RuntimeError(f"Camera rejected fourcc={fourcc}: actual={actual['fourcc']}")
                actual.update(controls)
                self.device_info = dict(actual, device=config["device"], model=config.get("model", "UVC"),
                                        fps_source="driver_reported; measure with camera-check")
            except BaseException:
                self.device.release()
                raise
        else:
            raise ValueError(f"Unknown camera type: {self.kind}")
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                if self.kind == "orbbec":
                    if not self.connection.poll(.5):
                        if self.process.poll() is not None:
                            raise RuntimeError("Orbbec worker exited")
                        continue
                    packet = self.connection.recv_bytes(64 * 1024 * 1024)
                    received = np.frombuffer(packet[:8], dtype="<f8")[0].item()
                    frame = np.load(io.BytesIO(packet[8:]), allow_pickle=False)
                else:
                    ok, frame = self.device.read()
                    if not ok:
                        raise RuntimeError("Camera read failed")
                    if frame.shape[:2] != (self.config["height"], self.config["width"]):
                        raise RuntimeError(f"Camera frame resolution changed: {frame.shape}")
                    received = time.monotonic()
                if frame is None:
                    continue
                with self.lock:
                    self.sequence += 1
                    self.frame = (frame, received, self.sequence)
        except Exception as exc:
            self.error = exc

    def snapshot(self):
        if self.error:
            raise RuntimeError("Camera capture stopped") from self.error
        with self.lock:
            if self.frame is None:
                raise RuntimeError("Camera has not produced a frame")
            image, received, sequence = self.frame
            return image.copy(), received, sequence

    def close(self):
        self.stop.set()
        self.worker.join(timeout=2)
        if self.kind == "orbbec":
            self.connection.close()
            if self.process.poll() is None:
                self.process.terminate()
            self.process.wait(timeout=5)
        else:
            self.device.release()
        self.worker.join(timeout=2)
