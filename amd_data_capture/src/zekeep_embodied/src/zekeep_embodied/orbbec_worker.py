"""Run the existing Orbbec driver in its compatible SDK environment."""

import io
import json
from multiprocessing.connection import Connection
from pathlib import Path
import signal
import sys
import time

import numpy as np
import yaml


def main():
    config_path = Path(sys.argv[1])
    connection = Connection(int(sys.argv[2]), readable=False)
    camera = None
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        sys.path.insert(0, str(config_path.parent.parent))
        from drivers.camera import make_camera
        with config_path.open() as stream:
            config = yaml.safe_load(stream)
        camera = make_camera(config)
        camera.open()
        connection.send_bytes(json.dumps({"serial": camera.serial,
                                         "config": config["camera"]}).encode())
        while True:
            frame, _ = camera.get_frame()
            received = time.monotonic()
            if frame is not None:
                buffer = io.BytesIO()
                np.save(buffer, frame, allow_pickle=False)
                connection.send_bytes(np.array([received], dtype="<f8").tobytes() + buffer.getvalue())
    except Exception as exc:
        try:
            connection.send_bytes(json.dumps({"error": str(exc)}).encode())
        except (BrokenPipeError, OSError):
            pass
        raise
    finally:
        if camera is not None:
            camera.close()
        connection.close()


if __name__ == "__main__":
    main()
