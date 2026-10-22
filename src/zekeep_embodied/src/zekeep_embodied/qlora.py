"""Single-GPU NF4 QLoRA training for the pinned openpi PyTorch model."""

import dataclasses
import json
import math
from pathlib import Path
from types import SimpleNamespace

import jax
import numpy as np
import safetensors.torch
import torch
from torch import nn


@dataclasses.dataclass(frozen=True)
class QLoRASettings:
    rank: int = 16
    alpha: float = 16.0
    dropout: float = 0.0
    quantize_vision: bool = True
    max_trainable_params: int = 60_000_000


class QLoRALinear(nn.Module):
    """Frozen NF4 linear layer with a trainable low-rank update."""

    def __init__(self, linear: nn.Linear, settings: QLoRASettings, *, with_lora: bool):
        super().__init__()
        try:
            import bitsandbytes as bnb
        except ImportError as exc:
            raise RuntimeError("Install requirements-pi05-qlora.txt in the openpi environment") from exc

        self.base = bnb.nn.Linear4bit(
            linear.in_features,
            linear.out_features,
            bias=linear.bias is not None,
            compute_dtype=torch.bfloat16,
            compress_statistics=True,
            quant_type="nf4",
        )
        self.base.weight = bnb.nn.Params4bit(
            linear.weight.detach().cpu(),
            requires_grad=False,
            compress_statistics=True,
            quant_type="nf4",
            quant_storage=torch.uint8,
        )
        if linear.bias is not None:
            self.base.bias = nn.Parameter(linear.bias.detach().cpu(), requires_grad=False)
        self._weight_view = SimpleNamespace(dtype=torch.bfloat16)

        self.scale = settings.alpha / settings.rank
        self.dropout = nn.Dropout(settings.dropout)
        if with_lora:
            self.lora_a = nn.Linear(linear.in_features, settings.rank, bias=False, dtype=torch.bfloat16)
            self.lora_b = nn.Linear(settings.rank, linear.out_features, bias=False, dtype=torch.bfloat16)
            nn.init.kaiming_uniform_(self.lora_a.weight, a=math.sqrt(5))
            nn.init.zeros_(self.lora_b.weight)
        else:
            self.lora_a = self.lora_b = None

    @property
    def weight(self):
        # openpi inspects projection.weight.dtype to select the activation dtype.
        # NF4 storage is uint8, while bitsandbytes computes these projections in BF16.
        return self._weight_view

    @property
    def bias(self):
        return self.base.bias

    def forward(self, inputs):
        result = self.base(inputs)
        if self.lora_a is None:
            return result
        update = self.lora_b(self.lora_a(self.dropout(inputs).to(self.lora_a.weight.dtype)))
        return result + update.to(result.dtype) * self.scale


_LORA_TARGETS = {"q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"}
_TRAINABLE_HEADS = ("action_in_proj", "action_out_proj", "time_mlp_in", "time_mlp_out")


def prepare_qlora(model: nn.Module, settings=QLoRASettings()):
    """Freeze the base model, replace its linear layers with NF4, and inject LoRA."""
    for parameter in model.parameters():
        parameter.requires_grad_(False)

    # Training consumes hidden states directly, so neither vocabulary projection is used.
    model.paligemma_with_expert.paligemma.lm_head = nn.Identity()
    model.paligemma_with_expert.gemma_expert.lm_head = nn.Identity()

    replaced = lora_layers = 0

    def replace(parent, prefix):
        nonlocal replaced, lora_layers
        for name, child in list(parent.named_children()):
            full_name = f"{prefix}.{name}" if prefix else name
            if isinstance(child, nn.Linear):
                is_vision = ".vision_tower." in full_name
                if is_vision and not settings.quantize_vision:
                    continue
                with_lora = (
                    name in _LORA_TARGETS
                    and (".language_model." in full_name or ".gemma_expert.model." in full_name)
                )
                setattr(parent, name, QLoRALinear(child, settings, with_lora=with_lora))
                replaced += 1
                lora_layers += int(with_lora)
            else:
                replace(child, full_name)

    replace(model.paligemma_with_expert, "paligemma_with_expert")
    for name in _TRAINABLE_HEADS:
        for parameter in getattr(model, name).parameters():
            parameter.requires_grad_(True)

    trainable = sum(parameter.numel() for parameter in model.parameters() if parameter.requires_grad)
    if not lora_layers:
        raise RuntimeError("No LoRA target layers found")
    if trainable > settings.max_trainable_params:
        raise RuntimeError(f"QLoRA freeze guard failed: {trainable:,} trainable parameters")
    return {"quantized_linear_layers": replaced, "lora_layers": lora_layers, "trainable_params": trainable}


def _learning_rate(config, step):
    schedule = config.lr_schedule
    if step < schedule.warmup_steps:
        initial = schedule.peak_lr / (schedule.warmup_steps + 1)
        return initial + (schedule.peak_lr - initial) * step / schedule.warmup_steps
    progress = min(1.0, (step - schedule.warmup_steps) / max(1, schedule.decay_steps - schedule.warmup_steps))
    return schedule.decay_lr + (schedule.peak_lr - schedule.decay_lr) * (1 + np.cos(np.pi * progress)) / 2


def _move_batch_to_cuda(observation, actions, device):
    observation = jax.tree.map(lambda value: value.to(device), observation)
    images = {
        name: image.permute(0, 3, 1, 2).contiguous() if image.ndim == 4 and image.shape[-1] == 3 else image
        for name, image in observation.images.items()
    }
    observation = observation.replace(images=images)
    return observation, actions.to(device=device, dtype=torch.float32)


def train(config, *, exp_name, num_train_steps, pytorch_weight_path, overwrite=False):
    """Run local single-GPU QLoRA training and save trainable adapter tensors."""
    try:
        import bitsandbytes as bnb
    except ImportError as exc:
        raise RuntimeError("Install requirements-pi05-qlora.txt in the openpi environment") from exc
    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch
    from openpi.training import data_loader

    if not torch.cuda.is_available():
        raise RuntimeError("QLoRA training requires a CUDA GPU")
    weights = Path(pytorch_weight_path).expanduser() / "model.safetensors"
    if not weights.is_file():
        raise FileNotFoundError(f"Converted PyTorch checkpoint not found: {weights}")
    output = Path(config.checkpoint_base_dir) / config.name / exp_name
    if output.exists() and not overwrite:
        raise FileExistsError(f"Checkpoint directory exists: {output}; pass --overwrite to replace adapter files")
    model_config = dataclasses.replace(config.model, dtype="bfloat16", pytorch_compile_mode=None)
    model = PI0Pytorch(model_config)
    safetensors.torch.load_model(model, str(weights))
    settings = QLoRASettings()
    qlora_info = prepare_qlora(model, settings)
    model.gradient_checkpointing_enable()

    device = torch.device("cuda:0")
    torch.cuda.reset_peak_memory_stats(device)
    model.to(device).train()
    trainable = [parameter for parameter in model.parameters() if parameter.requires_grad]
    optimizer = bnb.optim.PagedAdamW8bit(
        trainable,
        lr=config.lr_schedule.peak_lr,
        betas=(config.optimizer.b1, config.optimizer.b2),
        eps=config.optimizer.eps,
        weight_decay=config.optimizer.weight_decay,
    )
    loader = data_loader.create_data_loader(config, framework="pytorch", shuffle=True)
    iterator = iter(loader)

    for step in range(num_train_steps):
        observation, actions = next(iterator)
        observation, actions = _move_batch_to_cuda(observation, actions, device)
        for group in optimizer.param_groups:
            group["lr"] = _learning_rate(config, step)
        loss = model(observation, actions).mean()
        loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(trainable, config.optimizer.clip_gradient_norm)
        optimizer.step()
        optimizer.zero_grad(set_to_none=True)
        torch.cuda.synchronize(device)
        print(json.dumps({
            "step": step,
            "loss": float(loss.detach()),
            "grad_norm": float(grad_norm),
            "peak_vram_mib": round(torch.cuda.max_memory_allocated(device) / 2**20, 1),
        }))

    adapter = {
        name: parameter.detach().cpu().contiguous()
        for name, parameter in model.named_parameters()
        if parameter.requires_grad
    }
    output.mkdir(parents=True, exist_ok=True)
    safetensors.torch.save_file(adapter, output / "adapter.safetensors")
    summary = {
        **qlora_info,
        "quantization": "nf4",
        "lora_rank": settings.rank,
        "lora_alpha": settings.alpha,
        "steps": num_train_steps,
        "peak_vram_mib": round(torch.cuda.max_memory_allocated(device) / 2**20, 1),
        "base_checkpoint": str(weights.parent),
    }
    (output / "qlora.json").write_text(json.dumps(summary, indent=2) + "\n")
    return {"checkpoint": str(output), **summary}
