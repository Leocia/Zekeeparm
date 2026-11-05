"""Verify installed YOLOE weights, text prompts and CPU inference without devices."""
import hashlib
import os
from pathlib import Path
import sys

os.environ["YOLO_AUTOINSTALL"] = "false"

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_ROOT = ROOT / "src/zekeep_grasp"
sys.path.insert(0, str(PERCEPTION_ROOT))
from utils.native_runtime import preload_environment_libstdcpp

preload_environment_libstdcpp()

import numpy as np
import yaml
from ultralytics import YOLO, YOLOE
from utils.yolo_utils import resolve_yolo_model_path


def main():
    from importlib.metadata import version

    assert version("ultralytics") == "8.4.35", "Install ultralytics==8.4.35 in the vision environment"
    assert np.__version__ == "1.26.4", "Use the vision constraints; do not upgrade NumPy"
    config = yaml.safe_load((PERCEPTION_ROOT / "config/default.yaml").read_text())["yolo"]
    model_path = resolve_yolo_model_path(config["model_name"], PERCEPTION_ROOT).resolve()
    required = {model_path, (ROOT / "mobileclip2_b.ts").resolve()}
    verified = set()
    for line in (ROOT / "docs/MODEL_SHA256SUMS").read_text().splitlines():
        expected, relative = line.split(maxsplit=1)
        path = (ROOT / relative).resolve()
        if path not in required:
            continue
        if not path.is_file():
            raise FileNotFoundError(f"Missing model: {path}; run bash tools/download_models.sh base")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        assert digest.hexdigest() == expected, f"Model checksum mismatch: {path}"
        verified.add(path)
    assert verified == required, "Configured model must have an entry in docs/MODEL_SHA256SUMS"
    os.chdir(ROOT)
    model = YOLO(str(model_path)).to("cpu")
    assert isinstance(model, YOLOE), "The delivered configuration requires a YOLOE model"
    model.set_classes(config["custom_classes"])
    results = model.predict(np.zeros((320, 320, 3), dtype=np.uint8), device="cpu", imgsz=320, save=False, verbose=False)
    assert len(results) == 1
    print("YOLOE-26L weights, MobileCLIP prompts and CPU inference: OK")


if __name__ == "__main__":
    main()
