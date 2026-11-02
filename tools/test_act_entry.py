"""Check ACT ROS launch routing using a fake ros2 executable."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


class ActEntryTest(unittest.TestCase):
    def test_launches_use_existing_nodes_and_never_enable(self):
        with tempfile.TemporaryDirectory(prefix="act entry ") as directory:
            root = Path(directory)
            (root / "tools").mkdir()
            entry = root / "tools/act.sh"
            shutil.copy2(Path(__file__).with_name("act.sh"), entry)
            (root / "tools/activate_ros.sh").write_text('export PATH="$ACT_TEST_BIN:$PATH"\n')
            ros2 = root / "ros2"
            ros2.write_text(
                '#!/usr/bin/env bash\nexec "$ACT_TEST_PYTHON" -c '
                "'import json,os,sys; print(json.dumps([sys.argv[1:],os.environ[\"ROS_LOCALHOST_ONLY\"]]))' "
                '"$@"\n'
            )
            ros2.chmod(0o755)
            environment = dict(os.environ, ACT_TEST_BIN=directory, ACT_TEST_PYTHON=sys.executable)
            environment.pop("ROS_LOCALHOST_ONLY", None)
            for command, expected in (
                ("driver", ["launch", "zekeep_bringup", "driver.launch.py", "auto_enable:=false"]),
                ("joystick", ["launch", "zekeep_joystick", "cartesian_joystick.launch.py"]),
                ("moveit", ["launch", "zekeep_moveit_config", "hardware.launch.py",
                            "arm_namespace:=zekeep", "use_rviz:=false"]),
                ("joy", ["run", "joy", "joy_node", "--ros-args", "-p", "autorepeat_rate:=20.0"]),
            ):
                with self.subTest(command=command):
                    result = subprocess.run(["bash", str(entry), command, "custom:=value with spaces"],
                                            env=environment, capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    args, localhost = json.loads(result.stdout)
                    self.assertEqual(args, expected + ["custom:=value with spaces"])
                    self.assertEqual(localhost, "1")
                    self.assertNotIn("/zekeep/enable", args)


if __name__ == "__main__":
    unittest.main()
