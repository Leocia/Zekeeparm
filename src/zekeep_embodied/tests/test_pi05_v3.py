"""Shared v3 data, native PI05 processors and a tiny CPU model; no hardware or downloads."""

from contextlib import ExitStack, redirect_stdout
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import torch

from zekeep_embodied import pi05
from zekeep_embodied.capture import recording_metadata
from zekeep_embodied.storage import Episode, JOINTS


class SharedV3Test(unittest.TestCase):
    def test_act_export_is_read_directly_and_stats_do_not_decode_images(self):
        from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
        from zekeep_act.config import load_config
        from zekeep_act.data import prepare
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        settings, capture, _ = load_config(DEFAULT_CONFIG)
        settings = dict(settings, episodes=2, validation_episodes=1)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture["output"] = str(root / "raw")
            metadata = dict(recording_metadata(capture), simulated=True,
                            hardware={"effective_robot": {"joint_limits": {name: [-300, 300] for name in JOINTS}}})
            for number in range(2):
                episode = Episode(capture["output"], metadata)
                for index in range(4):
                    stamp = index / 10
                    state = [float(index)] * 7
                    episode.append(dict(state=state, action=[float(index + 2)] * 7,
                                        observation_time=stamp, action_time=stamp + .001,
                                        camera_times={name: stamp for name in capture["cameras"]},
                                        camera_sequences={name: index + 1 for name in capture["cameras"]}),
                                   {name: np.full((24, 32, 3), 50 + number, np.uint8) for name in capture["cameras"]})
                episode.finish("success")
            raw_files = list((root / "raw").rglob("*"))
            before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in raw_files if p.is_file()}
            prepare(settings, capture, output=root / "shared", allow_simulated=True)
            train = root / "shared/train"
            with self.assertRaisesRegex(ValueError, "Simulated"):
                pi05.open_dataset("local/shared", root=train)
            dataset = pi05.open_dataset("local/shared", root=train, allow_simulated=True)
            self.assertEqual(dataset[0]["action"].shape, (10, 7))
            with patch.object(LeRobotDataset, "_query_videos", side_effect=AssertionError("No video decoding in stats")):
                pi05.compute_statistics("local/shared", root=train, allow_simulated=True)
            stats = pi05.load_statistics(train)
            np.testing.assert_allclose(stats["stats"]["action"]["min"][:6], [2.] * 6)
            self.assertEqual(stats["stats"]["action"]["min"][6], 2.)
            with self.assertRaises(FileExistsError):
                pi05.compute_statistics("local/shared", root=train, allow_simulated=True)
            report = pi05.check_dataset("local/shared", True, root=train)
            self.assertEqual(report["format"], "lerobot-v3")
            self.assertTrue(report["normalized"])
            info = train / "meta/info.json"
            original = json.loads(info.read_text())
            info.write_text(json.dumps(dict(original, codebase_version="v2.1")))
            with self.assertRaisesRegex(ValueError, "v3 is required"):
                pi05.validate_dataset("local/shared", True, train)
            info.write_text(json.dumps(dict(original, fps=original["fps"] + 1)))
            with self.assertRaisesRegex(ValueError, "different backend or dataset"):
                pi05.load_statistics(train)
            self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before})

    def test_native_processors_training_loss_and_absolute_action_roundtrip(self):
        from tokenizers import Tokenizer
        from tokenizers.models import WordLevel
        from tokenizers.pre_tokenizers import Whitespace
        from transformers import PreTrainedTokenizerFast
        from lerobot.policies.pi05 import modeling_pi05 as native
        from lerobot.policies.factory import make_pre_post_processors
        from zekeep_embodied.finetune import prepare_finetune, load_base_policy
        from zekeep_embodied.pi05_policy_server import LeRobotPolicy
        import safetensors.torch

        torch.manual_seed(42)
        backend = Tokenizer(WordLevel({"[UNK]": 0, "[PAD]": 1, "Task": 2}, unk_token="[UNK]"))
        backend.pre_tokenizer = Whitespace()
        tokenizer = PreTrainedTokenizerFast(tokenizer_object=backend, unk_token="[UNK]", pad_token="[PAD]")
        config = pi05.make_config(device="cpu")
        config.dtype = "float32"
        config.gradient_checkpointing = False
        config.image_resolution = (28, 28)
        config.tokenizer_max_length = 64
        config.num_inference_steps = 2
        statistics = {"stats": {key: {"q01": [-10.] * 7, "q99": [10.] * 7}
                                 for key in ("observation.state", "action")}}
        original_vlm, original_expert = native.PaliGemmaForConditionalGeneration, native.GemmaForCausalLM

        def tiny_vlm(config):
            source = config
            source.text_config.vocab_size = 256
            source.vision_config.hidden_size = 32
            source.vision_config.intermediate_size = 64
            source.vision_config.num_attention_heads = 4
            source.vision_config.num_hidden_layers = 1
            source.vision_config.projection_dim = 32
            return original_vlm(source)

        def tiny_expert(config):
            source = config
            source.vocab_size = 256
            return original_expert(source)

        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            root = Path(temporary)
            # Native PI05 assumes eight attention heads; shrink width/depth while retaining that contract.
            stack.enter_context(patch.object(native, "get_gemma_config", return_value=native.GemmaConfig(32, 1, 64, 8, 1, 4)))
            stack.enter_context(patch.object(native, "PaliGemmaForConditionalGeneration", side_effect=tiny_vlm))
            stack.enter_context(patch.object(native, "GemmaForCausalLM", side_effect=tiny_expert))
            with patch("lerobot.processor.tokenizer_processor.AutoTokenizer.from_pretrained", return_value=tokenizer):
                pre, post = pi05.make_processors(config, statistics)
            policy = native.PI05Policy(config)
            # Prove complete native base tensors round-trip and an incomplete checkpoint fails closed.
            policy.model.paligemma_with_expert.paligemma.lm_head = torch.nn.Identity()
            policy.model.paligemma_with_expert.gemma_expert.lm_head = torch.nn.Identity()
            safetensors.torch.save_file(policy.state_dict(), root / "model.safetensors")
            restored = load_base_policy(config, root)
            for name, value in policy.state_dict().items():
                torch.testing.assert_close(restored.state_dict()[name], value)
            prepare_finetune(policy.model)
            batch = {"observation.state": torch.zeros(1, 7), "action": torch.ones(1, 10, 7),
                     "observation.images.global": torch.zeros(1, 3, 28, 28),
                     "observation.images.wrist": torch.ones(1, 3, 28, 28), "task": ["place cube"]}
            batch["action"] = pi05.delta_actions(batch["action"], batch["observation.state"])
            processed = pre(batch)
            self.assertEqual(processed["observation.language.tokens"].shape, (1, 64))
            optimizer = torch.optim.AdamW([p for p in policy.parameters() if p.requires_grad], lr=.001)
            loss, _ = policy(processed)
            self.assertTrue(torch.isfinite(loss))
            loss.backward()
            self.assertTrue(any(p.grad is not None and p.grad.abs().sum() > 0 for p in policy.parameters() if p.requires_grad))
            optimizer.step()
            actions = policy.predict_action_chunk(processed)
            self.assertEqual(actions.shape, (1, 10, 7))
            self.assertTrue(torch.isfinite(post(actions)).all())
            config.save_pretrained(root)
            pre.save_pretrained(root)
            post.save_pretrained(root)
            tokenizer.save_pretrained(root / "tokenizer")
            loaded_pre, loaded_post = make_pre_post_processors(config, pretrained_path=str(root),
                preprocessor_overrides={"tokenizer_processor": {"tokenizer_name": str(root / "tokenizer")}})
            self.assertEqual(loaded_pre(batch)["observation.language.tokens"].shape, (1, 64))
            # Native inference returns deltas; the unchanged RPC boundary returns absolute motor degrees.
            prediction = Mock()
            prediction.predict_action_chunk.return_value = torch.zeros(1, 10, 7)
            wrapper = LeRobotPolicy(prediction, loaded_pre, loaded_post)
            state = np.arange(7, dtype=np.float32)
            result = wrapper.infer({"observation/state": state, "prompt": "place cube",
                                    "observation/image": np.zeros((28, 28, 3), np.uint8),
                                    "observation/wrist_image": np.zeros((28, 28, 3), np.uint8)})
            np.testing.assert_allclose(result["actions"][:, :6], np.tile(state[:6], (10, 1)))
            np.testing.assert_allclose(result["actions"][:, 6], 0.)
            np.testing.assert_array_equal(state, np.arange(7, dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
