"""Exercise the real entry script with fake runners; never open robot devices."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


class Pi05EntryTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name) / "workspace with spaces"
        (self.root / "tools").mkdir(parents=True)
        self.entry = self.root / "tools/pi05.sh"
        shutil.copy2(Path(__file__).with_name("pi05.sh"), self.entry)
        shutil.copy2(Path(__file__).with_name("collect.sh"), self.entry.with_name("collect.sh"))
        shutil.copy2(Path(__file__).with_name("pi05-model.sh"), self.entry.with_name("pi05-model.sh"))
        runners = self.root / "src/zekeep_embodied/tools"
        runners.mkdir(parents=True)
        self.stub = self.root / "runner.py"
        self.stub.write_text(
            "import json, os, sys\n"
            "print(json.dumps({'args':sys.argv[1:], 'cwd':os.getcwd(), "
            "'cache':os.environ['HF_DATASETS_CACHE'], "
            "'jax':os.environ['JAX_PLATFORMS'], "
            "'local':os.environ['ROS_LOCALHOST_ONLY']}))\n"
        )
        for name in ("run.sh", "run_ros.sh", "run_pi05.sh"):
            (runners / name).write_text(
                '#!/usr/bin/env bash\nexec "$PI05_TEST_PYTHON" "$PI05_TEST_STUB" '
                '"${BASH_SOURCE[0]##*/}" "$@"\n'
            )
        fake_bin = self.root / "fake-bin"
        fake_bin.mkdir()
        ros2 = fake_bin / "ros2"
        ros2.write_text('#!/usr/bin/env bash\nexec "$PI05_TEST_PYTHON" "$PI05_TEST_STUB" ros2 "$@"\n')
        ros2.chmod(0o755)
        (self.root / "tools/activate_ros.sh").write_text('export PATH="$PI05_TEST_BIN:$PATH"\n')
        self.environment = dict(os.environ, PI05_TEST_PYTHON=sys.executable,
                                PI05_TEST_STUB=str(self.stub), PI05_TEST_BIN=str(fake_bin))
        for key in ("HF_DATASETS_CACHE", "JAX_PLATFORMS", "ROS_LOCALHOST_ONLY"):
            self.environment.pop(key, None)

    def run_entry(self, *args):
        return subprocess.run(
            ["bash", str(self.entry), *args], cwd=self.directory.name,
            env=self.environment, text=True, capture_output=True, timeout=10,
        )

    def test_model_routes_forward_explicit_identity_and_keep_callers_directory(self):
        identity = ["--repo-id", "local/red_cube_new", "--exp-name", "new_trial"]
        for command, runner, module, action in (
            ("train", "run_pi05.sh", "pi05", ["train-lora"]),
            ("predict", "run_pi05.sh", "pi05", ["predict"]),
            ("serve", "run_pi05.sh", "pi05_policy_server", []),
            ("shadow", "run_ros.sh", "pi05_runtime", None),
            ("execute", "run_ros.sh", "pi05_runtime", None),
        ):
            with self.subTest(command=command):
                extra = ["--num-train-steps", "2"] if command == "train" else []
                result = self.run_entry(command, *identity, *extra)
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                expected = [runner, "-m", f"zekeep_embodied.{module}"]
                if action is None:
                    expected += ["--config", str(self.root / "src/zekeep_embodied/configs/record.yaml"),
                                 f"--{command}"]
                else:
                    expected += action
                self.assertEqual(report["args"], expected + identity + extra)
                self.assertEqual(report["cwd"], self.directory.name)
                self.assertEqual(report["cache"], str(self.root / "data/openpi/cache/huggingface_datasets"))
                self.assertEqual(report["jax"], "cpu")
                self.assertEqual(report["local"], "1")

    def test_incomplete_commands_never_start_a_runner(self):
        for args in (
            ["record"], ["record-leader"], ["check"], ["export", "raw"], ["stats"],
            ["train", "--repo-id", "local/new", "--exp-name", "new"],
            ["predict", "--repo-id", "local/new"],
            ["serve", "--repo-id", "local/new"],
            ["shadow", "--exp-name", "new"], ["execute"], ["unknown"],
        ):
            with self.subTest(args=args):
                result = self.run_entry(*args)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn('"args"', result.stdout)

    def test_capture_export_and_display_routes(self):
        for command, args, runner, module, action in (
            ("audit", ["raw", "--output", "report.json"], "run.sh", "bundle", "audit"),
            ("bundle", ["--train", "train", "--validation", "val", "--output", "data.tar"],
             "run.sh", "bundle", "bundle"),
            ("verify", ["data.tar"], "run.sh", "bundle", "verify"),
            ("record", ["--output", "new recordings"], "run_ros.sh", "capture", "record"),
            ("record-leader", ["--output", "leader recordings"], "run_ros.sh", "capture", "record-leader"),
            ("camera-check", [], "run.sh", "record", "--camera-check"),
            ("check-raw", ["new episode"], "run.sh", "dataset", "check"),
            ("preview-raw", ["new episode", "--output", "review.mp4"], "run.sh", "dataset", "preview"),
            ("export", ["new recordings", "--repo-id=local/new"], "run_pi05.sh", "pi05", "export"),
            ("check", ["--repo-id=local/new"], "run_pi05.sh", "pi05", "check"),
            ("stats", ["--repo-id=local/new"], "run_pi05.sh", "pi05", "stats"),
            ("replay", ["saved prediction.json"], "run_ros.sh", "pi05_preview", None),
        ):
            with self.subTest(command=command):
                result = self.run_entry(command, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                actual = json.loads(result.stdout)["args"]
                expected = [runner, "-m", f"zekeep_embodied.{module}"]
                if action is not None:
                    expected.append(action)
                if module in {"record", "capture"}:
                    expected += ["--config", str(self.root / "src/zekeep_embodied/configs/record.yaml")]
                    if command == "camera-check":
                        expected += ["--seconds", "10"]
                self.assertEqual(actual, expected + args)

    def test_local_help_and_doctor_do_not_require_model_environment(self):
        result = self.run_entry("help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("bundle --train", result.stdout)
        self.assertNotIn("train --repo-id", result.stdout)
        result = self.run_entry("doctor", "--config", "custom.yaml")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["args"], [
            "run.sh", "-m", "zekeep_embodied.bundle", "doctor", "--config",
            str(self.root / "src/zekeep_embodied/configs/record.yaml"),
            "--config", "custom.yaml",
        ])

    def test_independent_model_entry(self):
        result = subprocess.run([
            "bash", str(self.entry.with_name("pi05-model.sh")), "train",
            "--repo-id", "local/new", "--exp-name", "trial", "--num-train-steps", "1",
        ], env=self.environment, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["args"][:4],
                         ["run_pi05.sh", "-m", "zekeep_embodied.pi05", "train-lora"])

    def test_shared_model_runner_uses_selected_lerobot_without_openpi(self):
        source = Path(__file__).resolve().parents[1] / "src/zekeep_embodied/tools/run_pi05.sh"
        runner = self.root / "src/zekeep_embodied/tools/run_pi05.sh"
        shutil.copy2(source, runner)
        environment = dict(self.environment, LEROBOT_PYTHON=sys.executable)
        for key in ("HF_LEROBOT_HOME", "ZKEEPI_CHECKPOINTS"):
            environment.pop(key, None)
        result = subprocess.run(["bash", str(runner), "-c",
            "import json,os;print(json.dumps({key:os.environ[key] for key in "
            "('PYTHONPATH','PYTHONNOUSERSITE','HF_LEROBOT_HOME','ZKEEPI_CHECKPOINTS')}))"],
            env=environment, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        actual = json.loads(result.stdout)
        self.assertEqual(actual["HF_LEROBOT_HOME"], str(self.root / "data/lerobot/datasets"))
        self.assertEqual(actual["ZKEEPI_CHECKPOINTS"], str(self.root / "data/lerobot/checkpoints"))
        self.assertEqual(actual["PYTHONNOUSERSITE"], "1")
        self.assertNotIn("openpi", actual["PYTHONPATH"])
        self.assertIn(str(self.root / "src/zekeep_act/src"), actual["PYTHONPATH"])

    def test_shared_entry_routes_both_modes_without_auto_enable(self):
        self.entry = self.entry.with_name("collect.sh")
        for command in ("record", "record-leader"):
            with self.subTest(command=command):
                arguments = ["--output", "shared raw", "--config", "custom capture.yaml", "--episodes", "8"]
                result = self.run_entry(command, *arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(result.stdout)["args"], [
                    "run_ros.sh", "-m", "zekeep_embodied.capture", command, "--config",
                    str(self.root / "src/zekeep_embodied/configs/record.yaml"), *arguments,
                ])
                self.assertNotIn("--execute", result.stdout)
                result = self.run_entry(command)
                self.assertEqual(result.returncode, 2)
        result = self.run_entry("help")
        self.assertIn("act.sh prepare", result.stdout)
        self.assertIn("pi05-model.sh export", result.stdout)

    def test_driver_defaults_to_disabled_and_other_launches_do_not_enable(self):
        for command in ("driver", "joystick", "rviz"):
            with self.subTest(command=command):
                result = self.run_entry(command)
                self.assertEqual(result.returncode, 0, result.stderr)
                actual = json.loads(result.stdout)["args"]
                self.assertEqual(actual[:2], ["ros2", "launch"])
                self.assertNotIn("auto_enable:=true", actual)
                self.assertNotIn("/zekeep/enable", actual)
                if command == "driver":
                    self.assertIn("auto_enable:=false", actual)
                elif command == "joystick":
                    self.assertEqual(actual[2:4],
                                     ["zekeep_joystick", "cartesian_joystick.launch.py"])

        result = self.run_entry("joystick", "config:=custom mapping.yaml")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["args"][-1], "config:=custom mapping.yaml")

        result = self.run_entry("joy")
        self.assertEqual(result.returncode, 0, result.stderr)
        actual = json.loads(result.stdout)["args"]
        self.assertEqual(actual[:4], ["ros2", "run", "joy", "joy_node"])
        self.assertIn("autorepeat_rate:=20.0", actual)
        self.assertIn("deadzone:=0.05", actual)
        self.assertNotIn("joystick_control", actual)


if __name__ == "__main__":
    unittest.main()
