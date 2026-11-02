import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np

from zekeep_act.policy import Policy
from zekeep_embodied.storage import JOINTS, write_json


class SavedPolicyTest(unittest.TestCase):
    def test_temporal_ensemble_aligns_arm_forecasts_keeps_latest_gripper_and_resets(self):
        import math
        import torch
        from lerobot.policies.act.modeling_act import ACTTemporalEnsembler

        restored = Policy.__new__(Policy)
        restored.torch = torch
        restored.contract = {"image_size": 32}
        restored.ensemble = ACTTemporalEnsembler(.01, 3)
        first = torch.zeros(1, 3, 7)
        first[:, 0, :6] = 10.
        first[:, 1:, :6] = 20.
        second = torch.zeros(1, 3, 7)
        second[:, :, 6] = 65.89
        restored.policy = Mock()
        restored.policy.predict_action_chunk.side_effect = [first, second, second]
        restored.pre = Mock(side_effect=lambda value: value)
        restored.post = Mock(side_effect=lambda value: value)
        restored.reset()
        images = {name: np.zeros((24, 40, 3), np.uint8) for name in ("wrist", "global")}
        initial, _ = restored.infer(images, [0.] * 7)
        blended, _ = restored.infer(images, [0.] * 7)
        # At the second tick, align the prior forecast's second step with the latest first step.
        expected = 20. / (1. + math.exp(-.01))
        np.testing.assert_allclose(blended[:6], expected, atol=1e-5)
        self.assertLess(abs(blended[0] - initial[0]), abs(restored.raw_action[0] - initial[0]))
        self.assertEqual(blended[6], restored.raw_action[6])
        self.assertAlmostEqual(blended[6], 65.89, places=4)
        restored.reset()
        self.assertIsNone(restored.raw_action)
        restarted, _ = restored.infer(images, [0.] * 7)
        np.testing.assert_allclose(restarted[:6], 0.)
        invalid = second.clone()
        invalid[:, 1, 0] = float("nan")
        restored.policy.predict_action_chunk.side_effect = [invalid]
        with self.assertRaisesRegex(RuntimeError, "non-finite arm action chunks"):
            restored.infer(images, [0.] * 7)

    def test_saved_config_processors_and_strict_weights_round_trip(self):
        import torch
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.act.processor_act import make_act_pre_post_processors

        previous_threads = torch.get_num_threads()
        torch.set_num_threads(1)
        try:
            with tempfile.TemporaryDirectory(prefix="act_policy_test_") as directory:
                path = Path(directory)
                inputs = {"observation.state": PolicyFeature(FeatureType.STATE, (7,))}
                inputs.update({f"observation.images.{name}": PolicyFeature(FeatureType.VISUAL, (3, 32, 32))
                               for name in ("wrist", "global")})
                config = ACTConfig(device="cpu", input_features=inputs,
                                   output_features={"action": PolicyFeature(FeatureType.ACTION, (7,))},
                                   pretrained_backbone_weights=None, chunk_size=3, n_action_steps=1,
                                   dim_model=32, n_heads=4, dim_feedforward=64, n_encoder_layers=1,
                                   n_decoder_layers=1, n_vae_encoder_layers=1, latent_dim=4)
                model = ACTPolicy(config)
                model.save_pretrained(path)
                stats = {key: {"mean": torch.zeros((3, 1, 1) if "images" in key else (7,)),
                               "std": torch.ones((3, 1, 1) if "images" in key else (7,))}
                         for key in list(inputs) + ["action"]}
                pre, post = make_act_pre_post_processors(config, stats)
                pre.save_pretrained(path)
                post.save_pretrained(path)
                write_json(path / "act_contract.json", dict(
                    units="degrees", joint_names=list(JOINTS), image_size=32, simulated=True,
                    action_semantics="absolute_motor_target_after_mapping_and_clipping"))
                # Saved configs contain a polymorphic `type`, requiring the base config loader.
                self.assertEqual(json.loads((path / "config.json").read_text())["type"], "act")
                restored = Policy(path, "cpu")
                images = {name: np.full((24, 40, 3), 127, np.uint8) for name in ("wrist", "global")}
                action, duration = restored.infer(images, [0.] * 7)
                self.assertEqual(len(action), 7)
                self.assertTrue(np.isfinite(action).all())
                self.assertGreater(duration, 0)
                np.testing.assert_array_equal(action, restored.raw_action)
                smoothed = Policy(path, "cpu", temporal_ensemble_steps=2)
                smoothed_action, _ = smoothed.infer(images, [0.] * 7)
                self.assertEqual(len(smoothed_action), 7)
                self.assertTrue(np.isfinite(smoothed_action).all())
                self.assertEqual(smoothed_action[6], smoothed.raw_action[6])
                with self.assertRaisesRegex(ValueError, "exceeds checkpoint chunk_size"):
                    Policy(path, "cpu", temporal_ensemble_steps=4)
                with self.assertRaises(ValueError):
                    restored.check_capture({}, {})
        finally:
            torch.set_num_threads(previous_threads)


if __name__ == "__main__":
    unittest.main()
