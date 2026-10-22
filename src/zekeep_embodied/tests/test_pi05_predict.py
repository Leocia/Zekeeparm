"""Recorded-frame prediction checks with a fake policy, without old data or hardware."""

import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from zekeep_embodied import pi05


@unittest.skipUnless(importlib.util.find_spec("openpi"), "run in the openpi environment")
class Pi05PredictTest(unittest.TestCase):
    def setUp(self):
        from openpi import transforms

        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.sample = {
            "observation.images.global": np.zeros((3, 8, 12), dtype=np.float32),
            "observation.images.wrist": np.ones((3, 8, 12), dtype=np.float32),
            "observation.state": np.arange(7, dtype=np.float32),
            "action": np.zeros((10, 7), dtype=np.float32),
            "prompt": "把红色方块放入托盘",
        }
        data = SimpleNamespace(repack_transforms=transforms.Group(inputs=[transforms.RepackTransform({
            "observation/image": "observation.images.global",
            "observation/wrist_image": "observation.images.wrist",
            "observation/state": "observation.state", "actions": "action", "prompt": "prompt",
        })]))
        config = SimpleNamespace(data=Mock(), assets_dirs=(), model=SimpleNamespace(action_horizon=10))
        config.data.create.return_value = data
        self.policy = Mock()
        self.policy.infer.return_value = {"actions": np.full((10, 7), 1.5)}
        for target, value in (
            ("zekeep_embodied.pi05.validate_dataset", Path(self.directory.name)),
            ("zekeep_embodied.pi05.make_config", config),
            ("openpi.training.data_loader.create_torch_dataset", [self.sample]),
            ("zekeep_embodied.pi05_policy_server.load_policy", (self.policy, {
                "repo_id": "local/new", "experiment": "new_trial", "units": "degrees",
            })),
        ):
            patcher = patch(target, return_value=value)
            mocked = patcher.start()
            self.addCleanup(patcher.stop)
            if target.endswith("load_policy"):
                self.load_policy = mocked

    def test_uses_recorded_prompt_state_and_images_and_saves_finite_actions(self):
        output = Path(self.directory.name) / "prediction.json"
        result = pi05.predict_frame("local/new", "new_trial", "base", 0, output)
        self.assertEqual(json.loads(output.read_text()), result)
        self.assertEqual(result["state_deg"], list(range(7)))
        self.assertEqual(result["actions_deg"], [[1.5] * 7] * 10)
        observation = self.policy.infer.call_args.args[0]
        self.assertNotIn("actions", observation)
        self.assertEqual(observation["prompt"], self.sample["prompt"])
        np.testing.assert_array_equal(observation["observation/image"], self.sample["observation.images.global"])
        np.testing.assert_array_equal(observation["observation/wrist_image"], self.sample["observation.images.wrist"])
        self.load_policy.assert_called_once_with("local/new", "new_trial", "base")

    def test_bad_frame_or_existing_output_never_loads_model(self):
        for index in (-1, 1):
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, "frame-index"):
                pi05.predict_frame("local/new", "new_trial", "base", index)
        output = Path(self.directory.name) / "keep.json"
        output.write_text("keep existing output")
        with self.assertRaises(FileExistsError):
            pi05.predict_frame("local/new", "new_trial", "base", 0, output)
        self.assertEqual(output.read_text(), "keep existing output")
        self.load_policy.assert_not_called()

    def test_invalid_predictions_and_concurrent_output_are_rejected(self):
        for actions in (np.zeros((1, 7)), np.full((10, 7), np.nan)):
            with self.subTest(shape=actions.shape), self.assertRaisesRegex(RuntimeError, "Invalid policy actions"):
                self.policy.infer.return_value = {"actions": actions}
                pi05.predict_frame("local/new", "new_trial", "base", 0)
        output = Path(self.directory.name) / "raced.json"

        def concurrent_write(observation):
            output.write_text("concurrent output")
            return {"actions": np.zeros((10, 7))}

        self.policy.infer.side_effect = concurrent_write
        with self.assertRaises(FileExistsError):
            pi05.predict_frame("local/new", "new_trial", "base", 0, output)
        self.assertEqual(output.read_text(), "concurrent output")


if __name__ == "__main__":
    unittest.main()
