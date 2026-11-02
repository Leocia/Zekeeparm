"""Build on the target OS; the Windows bundle contains its own Python runtime."""
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    if not (HERE / "public/vendor/blockly/blockly_compressed.js").is_file():
        raise SystemExit("Run npm ci in apps/studio first.")
    assets = {
        HERE / "public": "public",
        ROOT / "zekeeparm_SDK": "zekeeparm_SDK",
        ROOT / "src/zekeepcontroller/zekeepcontroller": "src/zekeepcontroller/zekeepcontroller",
        ROOT / "src/zekeep_bringup/description": "src/zekeep_bringup/description",
        ROOT / "src/zekeep_bringup/config/zekeep_hardware.yaml": "src/zekeep_bringup/config",
        ROOT / "src/zekeep_grasp/config/sixaxis.urdf": "src/zekeep_grasp/config",
        ROOT / "src/zekeep_moveit_config/config/zekeep.srdf": "src/zekeep_moveit_config/config",
        ROOT / "src/zekeep_bringup/web/public/lib": "src/zekeep_bringup/web/public/lib",
    }
    command = [sys.executable, "-m", "PyInstaller", "--onedir", "--name", "ZekeeparmStudio",
               "--additional-hooks-dir", str(HERE / "hooks"),
               "--paths", str(ROOT / "zekeeparm_SDK"),
               "--paths", str(ROOT / "src/zekeepcontroller"),
               "--hidden-import", "zekeepcontroller.hardware_manager",
               "--collect-all", "pinocchio", "--collect-all", "coal",
               "--collect-all", "motorbridge",
               "--exclude-module", "rclpy", "--exclude-module", "ament_index_python"]
    for source, destination in assets.items():
        command.extend(["--add-data", f"{source}:{destination}"])
    command.append(str(HERE / "studio.py"))
    subprocess.run(command, cwd=HERE, check=True)


if __name__ == "__main__":
    main()
