"""Check local capture environments without opening any hardware."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from zekeep_embodied.record import load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config, base = load_config(args.config)
    for value in (config["teleop_config"], config["ros_joystick"]["hardware_config"]):
        if not Path(value).is_file():
            raise FileNotFoundError(value)
    subprocess.run([sys.executable, "-c",
                    "import numpy, cv2, yaml, av, torch; "
                    "from lerobot_teleoperator_zhongling import ZhonglingLeader; "
                    "from lerobot_robot_seeed_b601 import SeeedB601DMFollower; "
                    "assert torch.version.cuda is None, 'Expected CPU PyTorch'"],
                   check=True, timeout=60)
    for camera in config["cameras"].values():
        if camera["type"] == "orbbec":
            camera_config = (base / camera["config"]).resolve()
            if not camera_config.is_file():
                raise FileNotFoundError(camera_config)
            subprocess.run([camera["python"], "-c", "import numpy, cv2, yaml, pyorbbecsdk"],
                           check=True, timeout=30)
    print(json.dumps({"capture_environment": "ok", "gpu_required": False,
                      "note": "No devices opened. ROS build and hardware operation are not verified."},
                     indent=2))


if __name__ == "__main__":
    main()
