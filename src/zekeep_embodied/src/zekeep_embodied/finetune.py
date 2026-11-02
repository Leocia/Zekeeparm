"""VLM language LoRA with a fully trainable, unquantized pi0.5 action expert."""

import dataclasses
import json
import math
from pathlib import Path

import safetensors.torch
import torch
from torch import nn


RECIPE = "vlm_lora_expert_full"
LORA_TARGETS = {"q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"}
ACTION_HEADS = ("action_in_proj", "action_out_proj", "time_mlp_in", "time_mlp_out")


@dataclasses.dataclass(frozen=True)
class LoRASettings:
    rank: int = 16
    alpha: float = 16.0


class LoRALinear(nn.Module):
    def __init__(self, base, settings):
        super().__init__()
        self.base = base
        self.scale = settings.alpha / settings.rank
        options = {"bias": False, "device": base.weight.device, "dtype": base.weight.dtype}
        self.lora_a = nn.Linear(base.in_features, settings.rank, **options)
        self.lora_b = nn.Linear(settings.rank, base.out_features, **options)
        nn.init.kaiming_uniform_(self.lora_a.weight, a=math.sqrt(5))
        nn.init.zeros_(self.lora_b.weight)

    @property
    def weight(self):
        return self.base.weight

    @property
    def bias(self):
        return self.base.bias

    def forward(self, inputs):
        return self.base(inputs) + self.lora_b(self.lora_a(inputs)) * self.scale


def prepare_finetune(model, settings=LoRASettings()):
    if (not isinstance(settings.rank, int) or settings.rank <= 0
            or not math.isfinite(settings.alpha) or settings.alpha <= 0):
        raise ValueError("LoRA rank and alpha must be positive; rank must be an integer")
    model.requires_grad_(False)
    core = model.paligemma_with_expert
    # These vocabulary heads are unused by the action policy.
    core.paligemma.lm_head = nn.Identity()
    core.gemma_expert.lm_head = nn.Identity()
    targets = [(parent, name, child) for parent in core.paligemma.language_model.modules()
               for name, child in parent.named_children()
               if name in LORA_TARGETS and isinstance(child, nn.Linear)]
    if not targets:
        raise RuntimeError("No VLM language LoRA targets found")
    for parent, name, child in targets:
        setattr(parent, name, LoRALinear(child, settings))
    core.gemma_expert.model.requires_grad_(True)
    for name in ACTION_HEADS:
        getattr(model, name).requires_grad_(True)
    expert_params = sum(p.numel() for p in core.gemma_expert.model.parameters())
    if not expert_params:
        raise RuntimeError("Action expert has no parameters")
    lora_params = sum(p.numel() for p in core.paligemma.parameters() if p.requires_grad)
    head_params = sum(p.numel() for name in ACTION_HEADS for p in getattr(model, name).parameters())
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    if trainable != lora_params + expert_params + head_params:
        raise RuntimeError("Unexpected trainable parameters outside the selected recipe")
    return {"recipe": RECIPE, "quantization": "none", "lora_layers": len(targets),
            "vlm_lora_params": lora_params, "action_expert_params": expert_params,
            "action_head_params": head_params, "trainable_params": trainable}


def load_adaptation(model, directory):
    """Recreate the saved recipe and require every trained tensor before inference."""
    metadata_path = directory / "finetune.json"
    metadata = json.loads(metadata_path.read_text())
    if metadata.get("recipe") != RECIPE:
        raise ValueError("Unsupported fine-tuning recipe")
    info = prepare_finetune(model, LoRASettings(metadata["lora_rank"], metadata["lora_alpha"]))
    state = safetensors.torch.load_file(str(directory / "adapter.safetensors"))
    expected = {name for name, parameter in model.named_parameters() if parameter.requires_grad}
    if set(state) != expected:
        missing, extra = expected - state.keys(), state.keys() - expected
        raise RuntimeError(f"Incomplete or incompatible trained weights: missing={sorted(missing)}, extra={sorted(extra)}")
    model.load_state_dict(state, strict=False)
    return info


def load_base_policy(config, directory):
    """Load every required base tensor strictly; never fall back to random weights."""
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    weights = Path(directory).expanduser() / "model.safetensors"
    if not weights.is_file():
        raise FileNotFoundError(f"Download the LeRobot pi05_base checkpoint first: {weights}")
    # Construct on CPU to avoid keeping two complete copies on the GPU while loading.
    model = PI05Policy(dataclasses.replace(config, device="cpu"))
    core = model.model
    # These heads are unused by action prediction and can be absent in tied-weight checkpoints.
    core.paligemma_with_expert.paligemma.lm_head = nn.Identity()
    core.paligemma_with_expert.gemma_expert.lm_head = nn.Identity()
    state = safetensors.torch.load_file(str(weights))
    state = {key.removeprefix("model."): value for key, value in state.items()}
    state = model._fix_pytorch_state_dict_keys(state, config)
    state = {"model." + key: value for key, value in state.items() if ".lm_head." not in key}
    model.load_state_dict(state, strict=True)
    model.config.device = config.device
    return model


def train(config, *, repo_id, exp_name, num_train_steps, pytorch_weight_path, root=None):
    """Train the native LeRobot PI05 model from v3 data and save the full adaptation."""
    from torch.utils.data import DataLoader
    from .pi05 import (BACKEND, checkpoint_dir, dataset_identity, delta_actions,
                       load_statistics, make_processors, open_dataset)

    if num_train_steps <= 0:
        raise ValueError("num-train-steps must be positive")
    output = checkpoint_dir(exp_name)
    if output.exists():
        raise FileExistsError(f"Checkpoint directory exists: {output}; choose a new experiment")
    dataset = open_dataset(repo_id, root=root)
    statistics = load_statistics(dataset.root)
    if not torch.cuda.is_available():
        raise RuntimeError("Training requires a CUDA GPU")
    weights = Path(pytorch_weight_path).expanduser() / "model.safetensors"
    if not weights.is_file():
        raise FileNotFoundError(weights)
    # Resolve tokenizer and statistics before allocating the large model.
    pre, post = make_processors(config, statistics)
    model = load_base_policy(config, weights.parent)
    settings = LoRASettings()
    adaptation_info = prepare_finetune(model.model, settings)
    print(json.dumps(adaptation_info), flush=True)
    model.to(config.device).train()
    trainable = [parameter for parameter in model.parameters() if parameter.requires_grad]
    optimizer = torch.optim.AdamW(trainable, lr=config.optimizer_lr,
                                 betas=config.optimizer_betas, eps=config.optimizer_eps,
                                 weight_decay=config.optimizer_weight_decay)
    scheduler = config.get_scheduler_preset().build(optimizer, num_training_steps=num_train_steps)
    loader = DataLoader(dataset, batch_size=1, shuffle=True, num_workers=0)
    iterator = iter(loader)
    torch.cuda.reset_peak_memory_stats()
    for step in range(num_train_steps):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            batch = next(iterator)
        batch["action"] = delta_actions(batch["action"], batch["observation.state"])
        loss, _ = model(pre(batch))
        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite pi05 training loss")
        loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(trainable, config.optimizer_grad_clip_norm,
                                                  error_if_nonfinite=True)
        optimizer.step()
        scheduler.step()
        optimizer.zero_grad(set_to_none=True)
        torch.cuda.synchronize()
        print(json.dumps({"step": step, "loss": float(loss.detach()), "grad_norm": float(grad_norm),
                          "peak_vram_mib": round(torch.cuda.max_memory_allocated() / 2**20, 1)}))

    adapter = {name: parameter.detach().cpu().contiguous()
               for name, parameter in model.model.named_parameters() if parameter.requires_grad}
    output.mkdir(parents=True, exist_ok=False)
    safetensors.torch.save_file(adapter, output / "adapter.safetensors")
    config.save_pretrained(output)
    pre.save_pretrained(output)
    post.save_pretrained(output)
    for processor in pre.steps:
        if getattr(processor, "input_tokenizer", None) is not None:
            processor.input_tokenizer.save_pretrained(output / "tokenizer")
    (output / "pi05_stats.json").write_text(json.dumps(statistics, indent=2) + "\n")
    provenance = (Path(dataset.root) / "capture_provenance.json").read_text()
    (output / "capture_provenance.json").write_text(provenance)
    summary = dict(adaptation_info, backend=BACKEND, repo_id=repo_id,
                   dataset_identity=dataset_identity(dataset.root),
                   lora_rank=settings.rank, lora_alpha=settings.alpha, steps=num_train_steps,
                   base_checkpoint=str(weights.parent.resolve()),
                   peak_vram_mib=round(torch.cuda.max_memory_allocated() / 2**20, 1))
    # Written last: a partial artifact directory can never be loaded as a trained policy.
    (output / "finetune.json").write_text(json.dumps(summary, indent=2) + "\n")
    return {"checkpoint": str(output), **summary}
