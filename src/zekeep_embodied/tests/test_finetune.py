"""Check hybrid fine-tuning gradients and complete trained-weight round trips on CPU."""

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import safetensors.torch
import torch
from torch import nn

from zekeep_embodied.finetune import (
    ACTION_HEADS, RECIPE, LoRALinear, LoRASettings, load_adaptation, prepare_finetune,
)


def tiny_model():
    model = nn.Module()
    core = model.paligemma_with_expert = nn.Module()
    vlm = core.paligemma = nn.Module()
    vlm.vision_tower = nn.Linear(4, 4)
    vlm.multi_modal_projector = nn.Linear(4, 4)
    vlm.language_model = nn.Module()
    vlm.language_model.q_proj = nn.Linear(4, 4)
    vlm.language_model.down_proj = nn.Linear(4, 4)
    vlm.language_model.norm = nn.LayerNorm(4)
    vlm.lm_head = nn.Linear(4, 8)
    expert = core.gemma_expert = nn.Module()
    expert.model = nn.Sequential(nn.Linear(4, 4), nn.LayerNorm(4))
    expert.lm_head = nn.Linear(4, 8)
    for name in ACTION_HEADS:
        setattr(model, name, nn.Linear(4, 4))
    return model


def forward(model, inputs):
    vlm = model.paligemma_with_expert.paligemma
    hidden = vlm.multi_modal_projector(vlm.vision_tower(inputs))
    hidden = vlm.language_model.norm(vlm.language_model.down_proj(vlm.language_model.q_proj(hidden)))
    hidden = model.paligemma_with_expert.gemma_expert.model(hidden)
    for name in ACTION_HEADS:
        hidden = getattr(model, name)(hidden)
    return hidden


class FineTuneTest(unittest.TestCase):
    def test_only_vlm_lora_and_full_expert_and_heads_train(self):
        torch.manual_seed(42)
        model = tiny_model()
        reference = copy.deepcopy(model)
        inputs = torch.randn(3, 4)
        expected = forward(reference, inputs)
        info = prepare_finetune(model, LoRASettings(rank=2, alpha=2))
        torch.testing.assert_close(forward(model, inputs), expected)
        vlm = model.paligemma_with_expert.paligemma
        self.assertIsInstance(vlm.language_model.q_proj, LoRALinear)
        self.assertIsInstance(model.paligemma_with_expert.gemma_expert.model[0], nn.Linear)
        self.assertTrue(all(p.requires_grad for p in model.paligemma_with_expert.gemma_expert.model.parameters()))
        self.assertTrue(all(not p.requires_grad for p in vlm.vision_tower.parameters()))
        self.assertTrue(all(not p.requires_grad for p in vlm.multi_modal_projector.parameters()))
        self.assertEqual(info["quantization"], "none")
        self.assertEqual(info["lora_layers"], 2)
        frozen = {name: p.detach().clone() for name, p in model.named_parameters() if not p.requires_grad}
        optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=0.01)
        original_expert = model.paligemma_with_expert.gemma_expert.model[0].weight.detach().clone()
        for _ in range(2):
            optimizer.zero_grad()
            forward(model, inputs).square().mean().backward()
            for name, parameter in model.named_parameters():
                if parameter.requires_grad:
                    self.assertIsNotNone(parameter.grad, name)
                else:
                    self.assertIsNone(parameter.grad, name)
            optimizer.step()
        self.assertTrue(torch.count_nonzero(vlm.language_model.q_proj.lora_a.weight.grad))
        self.assertFalse(torch.equal(original_expert, model.paligemma_with_expert.gemma_expert.model[0].weight))
        for name, parameter in model.named_parameters():
            if name in frozen:
                torch.testing.assert_close(parameter, frozen[name], rtol=0, atol=0)

    def test_checkpoint_restores_all_trained_tensors_and_rejects_missing_expert(self):
        torch.manual_seed(1)
        source = tiny_model()
        base = copy.deepcopy(source)
        prepare_finetune(source, LoRASettings(rank=2, alpha=4))
        with torch.no_grad():
            for parameter in source.parameters():
                if parameter.requires_grad:
                    parameter.add_(0.1)
        state = {name: p.detach().contiguous() for name, p in source.named_parameters() if p.requires_grad}
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "finetune.json").write_text(json.dumps({
                "recipe": RECIPE, "lora_rank": 2, "lora_alpha": 4,
            }))
            safetensors.torch.save_file(state, directory / "adapter.safetensors")
            restored = copy.deepcopy(base)
            info = load_adaptation(restored, directory)
            self.assertEqual(info["recipe"], RECIPE)
            inputs = torch.randn(3, 4)
            torch.testing.assert_close(forward(restored, inputs), forward(source, inputs))
            expert_key = next(name for name in state if ".gemma_expert.model." in name)
            del state[expert_key]
            safetensors.torch.save_file(state, directory / "adapter.safetensors")
            with self.assertRaisesRegex(RuntimeError, "Incomplete or incompatible"):
                load_adaptation(copy.deepcopy(base), directory)

    def test_missing_or_unsupported_recipe_is_rejected_before_adapting_model(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            model = tiny_model()
            with self.assertRaises(FileNotFoundError):
                load_adaptation(model, directory)
            (directory / "finetune.json").write_text(json.dumps({"recipe": "unsupported"}))
            with self.assertRaisesRegex(ValueError, "Unsupported fine-tuning recipe"):
                load_adaptation(model, directory)
            self.assertIsInstance(model.paligemma_with_expert.paligemma.language_model.q_proj, nn.Linear)

    def test_invalid_settings_or_missing_vlm_are_rejected(self):
        for settings in (LoRASettings(rank=0), LoRASettings(alpha=float("nan"))):
            with self.assertRaises(ValueError):
                prepare_finetune(tiny_model(), settings)
        model = tiny_model()
        model.paligemma_with_expert.paligemma.language_model = nn.Identity()
        with self.assertRaisesRegex(RuntimeError, "No VLM"):
            prepare_finetune(model)

    def test_strict_base_loading_rejects_missing_weights_and_incomplete_tensors(self):
        from zekeep_embodied.finetune import load_base_policy
        from zekeep_embodied.pi05 import make_config

        class TinyPolicy(nn.Module):
            def __init__(self, config):
                super().__init__()
                self.model = tiny_model()
                self.config = config

            def _fix_pytorch_state_dict_keys(self, state, config):
                return state

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = make_config(device="cpu")
            with patch("lerobot.policies.pi05.modeling_pi05.PI05Policy", TinyPolicy):
                with self.assertRaises(FileNotFoundError):
                    load_base_policy(config, root)
                weights = TinyPolicy(config).state_dict()
                safetensors.torch.save_file(weights, root / "model.safetensors")
                model = load_base_policy(config, root)
                self.assertIsInstance(model, TinyPolicy)
                del weights["model.paligemma_with_expert.gemma_expert.model.0.weight"]
                safetensors.torch.save_file(weights, root / "model.safetensors")
                with self.assertRaisesRegex(RuntimeError, "Missing key"):
                    load_base_policy(config, root)

    def test_training_entry_saves_native_recipe_and_all_expert_tensors(self):
        from zekeep_embodied.finetune import train
        from zekeep_embodied.pi05 import BACKEND, make_config

        class TinyPolicy(nn.Module):
            def __init__(self):
                super().__init__()
                self.model = tiny_model()

            def forward(self, batch):
                output = forward(self.model, batch["observation.state"][:, :4])
                return (output - batch["action"][:, 0, :4]).square().mean(), {}

        model = TinyPolicy()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "model.safetensors").touch()
            (root / "capture_provenance.json").write_text("{}")
            (root / "meta").mkdir()
            (root / "meta/info.json").write_text("{}")
            class Dataset(list):
                pass
            dataset = Dataset([{"observation.state": torch.zeros(7), "action": torch.ones(10, 7)}])
            dataset.root = root
            pre, post = Mock(side_effect=lambda batch: batch), Mock()
            pre.steps = []
            with (patch.dict("os.environ", {"ZKEEPI_CHECKPOINTS": str(root / "checkpoints")}),
                  patch("zekeep_embodied.finetune.load_base_policy", return_value=model),
                  patch("zekeep_embodied.pi05.open_dataset", return_value=dataset),
                  patch("zekeep_embodied.pi05.load_statistics", return_value={"stats": {}}),
                  patch("zekeep_embodied.pi05.make_processors", return_value=(pre, post)),
                  patch("torch.cuda.is_available", return_value=True),
                  patch("torch.cuda.reset_peak_memory_stats"), patch("torch.cuda.synchronize"),
                  patch("torch.cuda.max_memory_allocated", return_value=0)):
                result = train(make_config(device="cpu"), repo_id="local/new", exp_name="new",
                               num_train_steps=2, pytorch_weight_path=str(root))
                with self.assertRaises(FileExistsError):
                    train(make_config(device="cpu"), repo_id="local/new", exp_name="new",
                          num_train_steps=1, pytorch_weight_path=str(root))
            output = Path(result["checkpoint"])
            self.assertEqual(result["backend"], BACKEND)
            self.assertEqual(result["recipe"], RECIPE)
            self.assertEqual(json.loads((output / "finetune.json").read_text())["quantization"], "none")
            state = safetensors.torch.load_file(output / "adapter.safetensors")
            for name, parameter in model.model.paligemma_with_expert.gemma_expert.model.named_parameters():
                torch.testing.assert_close(state[f"paligemma_with_expert.gemma_expert.model.{name}"], parameter)
            self.assertTrue((output / "config.json").is_file())
            self.assertTrue((output / "pi05_stats.json").is_file())
            self.assertEqual(pre.call_count, 2)  # The loader starts another epoch instead of stopping early.


if __name__ == "__main__":
    unittest.main()
