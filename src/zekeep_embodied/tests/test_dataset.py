"""Keep full image validation while rejecting timing errors before decoding."""

import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from zekeep_embodied import dataset
from zekeep_embodied.storage import JOINTS


class DatasetInspectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "episode_a"
        self.path.mkdir()
        self.metadata = {
            "schema_version": 1, "joint_names": list(JOINTS), "units": "degrees",
            "action_semantics": "absolute_motor_target_after_mapping_and_clipping",
            "cameras": {"wrist": {}, "global": {}}, "frames": 3,
            "max_sync_s": .1, "max_cycle_s": .2, "status": "success",
            "simulated": False, "installation_id": "test", "task": "place cube", "fps": 10,
            "hardware": {"effective_robot": {"joint_limits": {name: [-10, 10] for name in JOINTS}}},
        }
        (self.path / "episode.json").write_text(json.dumps(self.metadata))
        self.rows = []
        for index in range(3):
            stamp = index / 10
            self.rows.append({"index": index, "state": [0.] * 7, "action": [1.] * 7,
                              "observation_time": stamp, "action_time": stamp + .001,
                              "camera_times": {name: stamp for name in self.metadata["cameras"]},
                              "camera_sequences": {name: index + 1 for name in self.metadata["cameras"]}})
            for name in self.metadata["cameras"]:
                (self.path / name).mkdir(exist_ok=True)
                cv2.imwrite(str(self.path / name / f"{index:06d}.png"), np.zeros((4, 6, 3), np.uint8))
        self.write_rows(self.rows)

    def write_rows(self, rows):
        (self.path / "frames.jsonl").write_text("\n".join(json.dumps(row) for row in rows))

    def test_metadata_validation_never_decodes_images(self):
        with patch.object(dataset.cv2, "imread") as imread:
            self.assertEqual(dataset.read_episode(self.path), (self.metadata, self.rows))
        imread.assert_not_called()

    def test_metadata_validation_keeps_state_action_and_timing_guards(self):
        for key, value, error in (
            ("index", 5, "Non-contiguous"),
            ("state", [float("nan")] * 7, "finite"),
            ("action", [11.] * 7, "outside configured limit"),
            ("camera_sequences", {"wrist": 1, "global": 1}, "Duplicate camera frame"),
            ("camera_times", {"wrist": -1., "global": .2}, "synchronization"),
        ):
            with self.subTest(key=key):
                rows = copy.deepcopy(self.rows)
                rows[-1][key] = value
                self.write_rows(rows)
                with self.assertRaisesRegex((ValueError, RuntimeError), error):
                    dataset.read_episode(self.path)

    def test_full_inspection_decodes_every_image_and_returns_shapes(self):
        with patch.object(dataset.cv2, "imread", wraps=cv2.imread) as imread:
            metadata, rows, shapes = dataset.inspect_episode(self.path)
        self.assertEqual((metadata, rows), (self.metadata, self.rows))
        self.assertEqual(shapes, {"wrist": (4, 6, 3), "global": (4, 6, 3)})
        self.assertEqual(imread.call_count, 6)

    def test_full_inspection_rejects_late_corruption_and_resolution_change(self):
        image = self.path / "global/000002.png"
        image.write_bytes(b"corrupt")
        with self.assertRaisesRegex(ValueError, "Missing/corrupt image.*000002"):
            dataset.inspect_episode(self.path)
        cv2.imwrite(str(image), np.zeros((5, 6, 3), np.uint8))
        with self.assertRaisesRegex(ValueError, "Camera resolution changed.*000002"):
            dataset.inspect_episode(self.path)
        image.unlink()
        with self.assertRaisesRegex(ValueError, "Missing/corrupt image.*000002"):
            dataset.inspect_episode(self.path)

    def test_export_checks_all_timestamps_before_any_image_or_output(self):
        shutil.copytree(self.path, self.root / "episode_b")
        self.path = self.root / "episode_b"
        for row, stamp in zip(self.rows, (0., .16, .32)):
            row.update(observation_time=stamp, action_time=stamp + .001,
                       camera_times={name: stamp for name in self.metadata["cameras"]})
        self.write_rows(self.rows)
        with patch.object(dataset, "inspect_episode") as inspect, \
                patch.object(dataset.cv2, "imread") as imread:
            with self.assertRaisesRegex(ValueError, "episode_b.*Capture gap"):
                dataset.export(self.root, self.root / "output", "local/test", fps=10)
        inspect.assert_not_called()
        imread.assert_not_called()
        self.assertFalse((self.root / "output.partial").exists())

    def test_existing_output_is_rejected_before_reading(self):
        output = self.root / "output"
        output.mkdir()
        with patch.object(dataset, "read_episode") as read:
            with self.assertRaises(FileExistsError):
                dataset.export(self.root, output, "local/test")
        read.assert_not_called()


if __name__ == "__main__":
    unittest.main()
