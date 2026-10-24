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
        runners = self.root / "src/zekeep_embodied/tools"
        runners.mkdir(parents=True)
        self.stub = self.root / "runner.py"
        self.stub.write_text(
            "import json, os, sys\n"
            "print(json.dumps({'args':sys.argv[1:], 'cwd':os.getcwd(), "
            "'cache':os.environ['HF_DATASETS_CACHE'], "
            "'profile':os.environ.get('PI05_TEST_PROFILE'), "
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
        (self.root / "tools/env.sh").write_text(
            '#!/usr/bin/env bash\n[[ "$1" == ros ]] || exit 97\nshift\n'
            'export PI05_TEST_PROFILE=ros\n'
            'source "$(dirname -- "${BASH_SOURCE[0]}")/activate_ros.sh"\nexec "$@"\n'
        )
        self.environment = dict(os.environ, PI05_TEST_PYTHON=sys.executable,
                                PI05_TEST_STUB=str(self.stub), PI05_TEST_BIN=str(fake_bin))
        for key in ("HF_DATASETS_CACHE", "JAX_PLATFORMS", "ROS_LOCALHOST_ONLY"):
            self.environment.pop(key, None)

    def run_entry(self, *args):
        return subprocess.run(
            ["bash", str(self.entry), *args], cwd=self.directory.name,
            env=self.environment, text=True, capture_output=True, timeout=10,
        )

    def test_capture_runner_uses_miniforge_and_its_own_source(self):
        runner = self.root / "capture-run.sh"
        shutil.copy2(Path(__file__).resolve().parents[1] / "src/zekeep_embodied/tools/run.sh", runner)
        forge = self.root / "miniforge with spaces"
        interpreter = forge / "envs/embodied/bin/python"
        interpreter.parent.mkdir(parents=True)
        interpreter.write_text(
            '#!/usr/bin/env bash\nexec "$PI05_TEST_PYTHON" -c '
            "'import json, os; print(json.dumps(dict(os.environ)))'\n"
        )
        interpreter.chmod(0o755)
        environment = dict(self.environment, ZKEEP_MINIFORGE_DIR=str(forge), PYTHONPATH="wrong overlay")
        environment.pop("EMBODIED_PYTHON", None)
        environment.pop("ZKEEP_VISION_PYTHON", None)
        result = subprocess.run(["bash", str(runner)], env=environment,
                                text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["PYTHONPATH"], str(self.root.parent / "src"))
        self.assertEqual(report["ZKEEP_VISION_PYTHON"], str(forge / "envs/rebotarm/bin/python"))
        self.assertEqual(report["PYTHONNOUSERSITE"], "1")

    def test_model_routes_forward_explicit_identity_and_keep_callers_directory(self):
        identity = ["--repo-id", "local/red_cube_new", "--exp-name", "new_trial"]
        for command, runner, module, action in (
            ("train", "run_pi05.sh", "pi05", ["train-qlora"]),
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

    def test_data_commands_use_capture_runner_without_model_environment(self):
        for command, args in (
            ("doctor", []),
            ("audit", ["raw recordings", "--output", "audit.json"]),
            ("bundle", ["--train", "train set", "--validation", "validation set",
                        "--output", "delivery.tar"]),
            ("verify", ["delivery.tar"]),
        ):
            with self.subTest(command=command):
                result = self.run_entry(command, *args)
                self.assertEqual(result.returncode, 0, result.stderr)
                expected = ["run.sh", "-m", "zekeep_embodied.bundle", command]
                if command == "doctor":
                    expected += ["--config", str(self.root / "src/zekeep_embodied/configs/record.yaml")]
                self.assertEqual(json.loads(result.stdout)["args"], expected + args)

    def test_capture_export_and_display_routes(self):
        for command, args, runner, module, action in (
            ("record", ["--output", "new recordings"], "run_ros.sh", "record", "--joystick"),
            ("record-leader", ["--output", "leader recordings"], "run.sh", "record", "--execute"),
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
                if module == "record":
                    expected += ["--config", str(self.root / "src/zekeep_embodied/configs/record.yaml")]
                    if command == "camera-check":
                        expected += ["--seconds", "10"]
                self.assertEqual(actual, expected + args)

    def test_driver_defaults_to_disabled_and_other_launches_do_not_enable(self):
        for command in ("driver", "joystick", "rviz"):
            with self.subTest(command=command):
                result = self.run_entry(command)
                self.assertEqual(result.returncode, 0, result.stderr)
                actual = json.loads(result.stdout)["args"]
                self.assertEqual(json.loads(result.stdout)["profile"], "ros")
                self.assertEqual(actual[:2], ["ros2", "launch"])
                self.assertNotIn("auto_enable:=true", actual)
                self.assertNotIn("/zekeep/enable", actual)
                if command == "driver":
                    self.assertIn("auto_enable:=false", actual)

        result = self.run_entry("joy")
        self.assertEqual(result.returncode, 0, result.stderr)
        actual = json.loads(result.stdout)["args"]
        self.assertEqual(actual[:4], ["ros2", "run", "joy", "joy_node"])
        self.assertIn("autorepeat_rate:=20.0", actual)
        self.assertNotIn("joystick_control", actual)


if __name__ == "__main__":
    unittest.main()
