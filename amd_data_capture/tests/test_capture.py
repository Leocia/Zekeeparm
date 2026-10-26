"""No-device checks of raw recording, portability, and data rejection."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import av
import numpy as np

from zekeep_embodied import record
from zekeep_embodied.dataset import inspect_episode, preview
from zekeep_embodied.storage import Episode

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("capture_audit", ROOT / "tools/audit.py")
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


class CaptureTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config, self.base = record.load_config(ROOT / "src/zekeep_embodied/configs/record.yaml")
        self.config["output"] = str(self.root / "raw")

    def sample(self, state=None):
        return dict(state=state or [0.] * 7, action=[0.] * 7,
                    observation_time=1., action_time=1.001,
                    camera_times={"wrist": 1., "global": 1.},
                    camera_sequences={"wrist": 1, "global": 1})

    def images(self):
        return {name: np.zeros((12, 16, 3), np.uint8) for name in ("wrist", "global")}

    def test_raw_roundtrip_and_cpu_video_decode(self):
        path = record.simulate(self.config, 3)
        metadata, rows, shapes = inspect_episode(path)
        self.assertTrue(metadata["simulated"])
        self.assertEqual(metadata["units"], "degrees")
        self.assertEqual(len(rows), 3)
        self.assertEqual(set(shapes), {"wrist", "global"})
        video = self.root / "review.mp4"
        preview(path, video)
        with av.open(str(video)) as stream:
            self.assertEqual(len(list(stream.decode(video=0))), 3)
        with self.assertRaises(FileExistsError):
            preview(path, video)

    def test_rejects_stale_duplicate_and_nonfinite_samples(self):
        sample = self.sample()
        def validate(previous=None):
            record.check_sample(sample["state"], self.images(), sample["camera_times"],
                                sample["camera_sequences"], previous or {},
                                sample["observation_time"], sample["action_time"], .1, .2)
        validate()
        with self.assertRaisesRegex(RuntimeError, "Duplicate"):
            validate({"wrist": 1})
        sample["camera_times"]["wrist"] = .8
        with self.assertRaisesRegex(RuntimeError, "synchronization"):
            validate()
        sample["state"][0] = float("nan")
        with self.assertRaises(ValueError):
            validate()

    def test_failed_disk_write_stays_partial(self):
        metadata = dict(self.config, simulated=True, feedback_freshness="synthetic")
        episode = Episode(self.config["output"], metadata)
        with patch("zekeep_embodied.storage.cv2.imwrite", return_value=False):
            episode.append(self.sample(), self.images())
            path = episode.finish("success")
        self.assertTrue(path.name.endswith(".partial"))
        self.assertEqual(json.loads((path / "episode.json").read_text())["status"], "aborted")

    def test_audit_excludes_synthetic_and_corrupt_data_without_overwrite(self):
        path = record.simulate(self.config, 2)
        report = self.root / "audit.json"
        result = audit_module.audit(path.parent, report)
        self.assertEqual(len(result["excluded"]), 1)
        self.assertEqual(result["valid"], [])
        before = report.read_bytes()
        with self.assertRaises(FileExistsError):
            audit_module.audit(path.parent, report)
        self.assertEqual(report.read_bytes(), before)
        (path / "frames.jsonl").write_text("invalid json")
        result = audit_module.audit(path.parent, self.root / "corrupt.json")
        self.assertEqual(len(result["excluded"]), 1)

    def test_paths_resolve_inside_this_package(self):
        self.assertEqual(Path(self.config["cameras"]["wrist"]["python"]),
                         ROOT / ".venv-camera/bin/python")
        for path in [self.config["teleop_config"], self.config["ros_joystick"]["hardware_config"],
                     self.base / self.config["cameras"]["wrist"]["config"]]:
            self.assertTrue(Path(path).is_file())
            self.assertTrue(Path(path).resolve().is_relative_to(ROOT))

    def test_audit_rejects_partial_directory_alias(self):
        path = record.simulate(self.config, 2)
        metadata = json.loads((path / "episode.json").read_text())
        metadata["simulated"] = False
        (path / "episode.json").write_text(json.dumps(metadata))
        partial = path.rename(path.with_name(path.name + ".partial"))
        (path.parent / "episode_alias").symlink_to(partial, target_is_directory=True)
        result = audit_module.audit(path.parent, self.root / "audit.json")
        self.assertEqual(result["valid"], [])
        self.assertEqual(len(result["excluded"]), 2)

    def test_motor_calibration_and_safety_sources_unchanged(self):
        snapshot = json.loads((ROOT / "SOURCE_SNAPSHOT.json").read_text())["files_before_packaging_edits"]
        selected = ["src/zekeep_teleop/configs/zhongling_b601_teleop.yaml",
                    "src/zekeep_bringup/config/zekeep_hardware.yaml"]
        selected += [name for name in snapshot if name.startswith((
            "src/zekeepcontroller/zekeepcontroller/",
            "third_party/reBotArm_control_py/reBotArm_control_py/")) and (ROOT / name).is_file()]
        for name in selected:
            self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(),
                             snapshot[name]["sha256"], name)

    def test_joystick_calibration_roundtrip(self):
        calibrations, _, _ = record._load_ros_calibration(self.config["ros_joystick"]["hardware_config"])
        expected = [.1] * 7
        ros_positions = [(x - c["encoder_zero"]) * c["direction"] * c["gear_ratio"] + c["ros_zero"]
                         for x, c in zip(expected, calibrations)]
        np.testing.assert_allclose(record._ros_to_motor_degrees(ros_positions, calibrations),
                                   np.degrees(expected))

    def test_no_model_commands_or_imports(self):
        result = subprocess.run(["bash", str(ROOT / "tools/capture.sh"), "train"],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 2)
        code = "from zekeep_embodied import record, dataset; import sys; assert 'torch' not in sys.modules"
        subprocess.run([sys.executable, "-c", code], check=True)


if __name__ == "__main__":
    unittest.main()
