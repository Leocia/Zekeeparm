"""Optionally update camera serial and save a calibration identity snapshot.

Opens only the camera. Never connects to the robot.
Moving computers does not require this command or a saved identity record.
Changing cameras or mounting requires calibration for the current installation.
"""
from datetime import datetime
from pathlib import Path
import argparse
import shutil
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from drivers.camera import make_camera
from utils.camera_utils import load_config
from scripts.read_camera_serial import write_camera_serial
from utils.calibration_identity import (
    calibration_identity,
    confirm_calibration_identity,
    validate_calibration_identity,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/default.yaml"))
    parser.add_argument("--confirm-same-installation", action="store_true", required=True,
                        help="Confirm these files were calibrated for this exact camera, robot, mounting and scene")
    args = parser.parse_args(argv)
    config_path = Path(args.config).expanduser().resolve()
    cfg = load_config(config_path)
    camera = make_camera(cfg)
    try:
        camera.open()
        serial = str(getattr(camera, "serial", "") or "").strip()
    finally:
        camera.close()
    previous_serial = cfg["camera"].get("serial")
    cfg["camera"]["serial"] = serial
    # Check required identities and files before changing either stored record.
    calibration_identity(PROJECT_ROOT, cfg, serial)
    identity_path = (PROJECT_ROOT / "config" / "calibration" /
                     str(cfg["camera"]["type"]).lower() / "identity.local.json")
    backup = (PROJECT_ROOT / "config" / "calibration" / "backups" /
              datetime.now().strftime("device_update_%Y%m%d_%H%M%S_%f"))
    backup.mkdir(parents=True)
    config_backup = backup / "config.yaml"
    identity_backup = backup / "identity.local.json"
    shutil.copy2(config_path, config_backup)
    if identity_path.exists():
        shutil.copy2(identity_path, identity_backup)
    try:
        write_camera_serial(serial, config_path)
        confirm_calibration_identity(PROJECT_ROOT, cfg, serial)
        validate_calibration_identity(PROJECT_ROOT, load_config(config_path), serial)
    except Exception:
        shutil.copy2(config_backup, config_path)
        if identity_backup.exists():
            shutil.copy2(identity_backup, identity_path)
        else:
            identity_path.unlink(missing_ok=True)
        raise
    print(f"Camera serial: {previous_serial} -> {serial}")
    print(f"Config: {config_path}")
    print(f"Calibration identity snapshot saved (optional): {identity_path}")
    print(f"Backup: {backup}")


if __name__ == "__main__":
    main()
