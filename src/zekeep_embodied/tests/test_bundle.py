"""Exercise portable raw-data delivery with tiny images; no hardware or models."""

import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from zekeep_embodied import bundle
from zekeep_embodied.storage import JOINTS


class BundleTest(unittest.TestCase):
    def test_doctor_uses_camera_override_without_model_or_gpu(self):
        with tempfile.TemporaryDirectory() as temporary:
            config_path = Path(temporary) / "record.yaml"
            config_path.touch()
            config = {"teleop_config": str(config_path),
                      "cameras": {"wrist": {"type": "orbbec", "python": "/missing/python",
                                            "config": "record.yaml"}},
                      "ros_joystick": {"hardware_config": str(config_path)}}
            with (patch("zekeep_embodied.record.load_config", return_value=(config, config_path.parent)),
                  patch.dict(os.environ, ZKEEP_VISION_PYTHON=sys.executable),
                  patch("importlib.util.find_spec", return_value=object()) as find):
                self.assertEqual(bundle.doctor(config_path)["data_environment"], "ok")
                self.assertEqual([call.args[0] for call in find.call_args_list],
                                 ["numpy", "cv2", "yaml", "av"])
            with (patch("zekeep_embodied.record.load_config", return_value=(config, config_path.parent)),
                  patch.dict(os.environ, ZKEEP_VISION_PYTHON="/missing/camera-python"),
                  patch("importlib.util.find_spec", return_value=object())):
                with self.assertRaisesRegex(RuntimeError, "/missing/camera-python"):
                    bundle.doctor(config_path)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw = self.root / "raw"
        self.raw.mkdir()
        self.train, self.validation = self.root / "train", self.root / "validation"
        self.train.mkdir()
        self.validation.mkdir()
        self.first = self.episode("episode_a", 1)
        self.second = self.episode("episode_b", 2)
        (self.train / self.first.name).symlink_to(self.first, target_is_directory=True)
        (self.validation / self.second.name).symlink_to(self.second, target_is_directory=True)
        self.output = self.root / "delivery.tar"

    def episode(self, name, value):
        path = self.raw / name
        path.mkdir()
        metadata = {
            "schema_version": 1, "joint_names": list(JOINTS), "units": "degrees",
            "action_semantics": "absolute_motor_target_after_mapping_and_clipping",
            "cameras": {"wrist": {}, "global": {}}, "frames": 2,
            "max_sync_s": 0.1, "max_cycle_s": 0.2, "status": "success",
            "simulated": False, "installation_id": "test", "task": "place cube", "fps": 10,
        }
        (path / "episode.json").write_text(json.dumps(metadata))
        rows = []
        for i in range(2):
            rows.append({"index": i, "state": [value] * 7, "action": [value] * 7,
                         "observation_time": i / 10, "action_time": i / 10 + 0.001,
                         "camera_times": {c: i / 10 for c in metadata["cameras"]},
                         "camera_sequences": {c: i + 1 for c in metadata["cameras"]}})
            for camera in metadata["cameras"]:
                (path / camera).mkdir(exist_ok=True)
                cv2.imwrite(str(path / camera / f"{i:06d}.png"), np.full((4, 6, 3), value, np.uint8))
        (path / "frames.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
        return path

    def test_portable_archive_preserves_raw_files_and_verifies_with_stdlib(self):
        result = bundle.bundle(self.train, self.validation, self.output)
        self.assertTrue(result["verified"])
        self.assertFalse(self.output.with_suffix(".tar.partial").exists())
        with tarfile.open(self.output) as archive:
            self.assertTrue(all(member.isfile() for member in archive))
            self.assertEqual(archive.extractfile("train/episode_a/frames.jsonl").read(),
                             (self.first / "frames.jsonl").read_bytes())
            manifest = json.load(archive.extractfile("bundle.json"))
            self.assertEqual(len(manifest["episodes"]), 2)
        # -S removes site packages: verification must work without NumPy, ROS or OpenPI.
        check = subprocess.run([sys.executable, "-S", bundle.__file__, "verify", str(self.output)],
                               capture_output=True, text=True, timeout=10)
        self.assertEqual(check.returncode, 0, check.stderr)
        before = self.output.read_bytes()
        with self.assertRaises(FileExistsError):
            bundle.bundle(self.train, self.validation, self.output)
        self.assertEqual(before, self.output.read_bytes())

    def test_rejects_duplicate_symlink_and_copied_episode(self):
        (self.validation / "episode_alias").symlink_to(self.first, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "Duplicate episode"):
            bundle.bundle(self.train, self.validation, self.output)
        (self.validation / "episode_alias").unlink()
        shutil.copytree(self.first, self.validation / "episode_copy")
        with self.assertRaisesRegex(ValueError, "Duplicate episode content"):
            bundle.bundle(self.train, self.validation, self.output)
        self.assertFalse(self.output.exists())

    def test_audit_reports_failed_simulated_partial_and_corrupt_without_editing(self):
        for name, update in (("episode_failed", {"status": "failed"}),
                             ("episode_sim", {"simulated": True}),
                             ("episode_incomplete.partial", {})):
            path = self.episode(name, 3)
            metadata = json.loads((path / "episode.json").read_text())
            (path / "episode.json").write_text(json.dumps(dict(metadata, **update)))
        broken = self.episode("episode_corrupt", 4)
        (broken / "frames.jsonl").write_text("bad json")
        report = self.root / "audit.json"
        result = bundle.audit(self.raw, report)
        self.assertEqual((result["eligible"], result["excluded"]), (2, 4))
        self.assertEqual((broken / "frames.jsonl").read_text(), "bad json")
        with self.assertRaises(FileExistsError):
            bundle.audit(self.raw, report)
        (self.train / broken.name).symlink_to(broken, target_is_directory=True)
        with self.assertRaises(ValueError):
            bundle.bundle(self.train, self.validation, self.output)
        self.assertFalse(self.output.exists())

    def test_rejects_empty_split_and_mixed_calibration(self):
        empty = self.root / "empty"
        empty.mkdir()
        with self.assertRaisesRegex(ValueError, "No episodes"):
            bundle.bundle(self.train, empty, self.output)
        metadata = json.loads((self.second / "episode.json").read_text())
        metadata["installation_id"] = "different"
        (self.second / "episode.json").write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "Cannot mix"):
            bundle.bundle(self.train, self.validation, self.output)

    def test_rejects_simulation_and_disguised_partial(self):
        partial = self.episode("episode_unfinished.partial", 3)
        alias = self.train / "episode_alias"
        alias.symlink_to(partial, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            bundle.bundle(self.train, self.validation, self.output)
        alias.unlink()
        metadata = json.loads((self.first / "episode.json").read_text())
        metadata["simulated"] = True
        (self.first / "episode.json").write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "non-simulated"):
            bundle.bundle(self.train, self.validation, self.output)

    def test_detects_tampering_and_unsafe_members(self):
        bundle.bundle(self.train, self.validation, self.output)
        damaged = self.root / "damaged.tar"
        with tarfile.open(self.output) as source, tarfile.open(damaged, "w") as target:
            for member in source:
                data = source.extractfile(member).read()
                if member.name.endswith("frames.jsonl"):
                    data = data.replace(b'"index": 0', b'"index": 9')
                target.addfile(member, io.BytesIO(data))
        with self.assertRaisesRegex(ValueError, "mismatch"):
            bundle.verify(damaged)
        for name in ("../escape", "/absolute", "train/link"):
            unsafe = self.root / "unsafe.tar"
            with tarfile.open(unsafe, "w") as archive:
                info = tarfile.TarInfo(name)
                if name == "train/link":
                    info.type, info.linkname = tarfile.SYMTYPE, "/etc/passwd"
                archive.addfile(info)
            with self.assertRaisesRegex(ValueError, "Unsafe"):
                bundle.verify(unsafe)


if __name__ == "__main__":
    unittest.main()
