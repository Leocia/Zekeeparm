"""Local-only QLoRA policy server for the guarded ROS runtime."""

import argparse
import dataclasses
import json
from pathlib import Path
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer


def _decode_png(value):
    import cv2
    import numpy as np

    payload = value.data if hasattr(value, "data") else value
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Invalid PNG image")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def load_policy(repo_id, exp_name, base_checkpoint):
    import numpy as np
    import safetensors.torch
    import torch
    from openpi import transforms
    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch
    from openpi.policies.policy import Policy

    from .pi05 import make_config, validate_dataset
    from .qlora import prepare_qlora

    validate_dataset(repo_id)
    config = make_config(repo_id)
    data = config.data.create(config.assets_dirs, config.model)
    if data.norm_stats is None:
        raise FileNotFoundError(f"Normalization statistics not found for {repo_id}")

    weights = Path(base_checkpoint).expanduser() / "model.safetensors"
    adapter = Path(config.checkpoint_base_dir) / config.name / exp_name / "adapter.safetensors"
    if not weights.is_file() or not adapter.is_file():
        raise FileNotFoundError(f"Missing base or adapter checkpoint: {weights}, {adapter}")
    if not torch.cuda.is_available():
        raise RuntimeError("Policy server requires CUDA")

    model_config = dataclasses.replace(
        config.model, dtype="bfloat16", pytorch_compile_mode=None
    )
    model = PI0Pytorch(model_config)
    safetensors.torch.load_model(model, str(weights))
    qlora = prepare_qlora(model)
    adapter_state = safetensors.torch.load_file(str(adapter))
    incompatible = model.load_state_dict(adapter_state, strict=False)
    if incompatible.unexpected_keys:
        raise RuntimeError(f"Unexpected adapter keys: {incompatible.unexpected_keys}")

    policy = Policy(
        model,
        transforms=[
            *data.data_transforms.inputs,
            transforms.Normalize(data.norm_stats, use_quantiles=data.use_quantile_norm),
            *data.model_transforms.inputs,
        ],
        output_transforms=[
            *data.model_transforms.outputs,
            transforms.Unnormalize(data.norm_stats, use_quantiles=data.use_quantile_norm),
            *data.data_transforms.outputs,
        ],
        metadata=config.policy_metadata,
        is_pytorch=True,
        pytorch_device="cuda:0",
    )
    warmup_started = time.monotonic()
    policy.infer({
        "observation/image": np.zeros((224, 224, 3), dtype=np.uint8),
        "observation/wrist_image": np.zeros((224, 224, 3), dtype=np.uint8),
        "observation/state": np.zeros(7, dtype=np.float32),
        "prompt": "warm up policy",
    })
    return policy, {
        "repo_id": repo_id,
        "experiment": exp_name,
        "action_horizon": config.model.action_horizon,
        "action_dim": 7,
        "units": "degrees",
        "warmup_ms": round((time.monotonic() - warmup_started) * 1000, 1),
        **qlora,
    }


class RuntimePolicy:
    def __init__(self, policy, metadata):
        self.policy = policy
        self.metadata = metadata
        self.lock = threading.Lock()

    def health(self):
        return self.metadata

    def infer(self, global_png, wrist_png, state, prompt):
        import numpy as np

        state = np.asarray(state, dtype=np.float32)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError("State must contain seven finite values")
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("Prompt must not be empty")
        observation = {
            "observation/image": _decode_png(global_png),
            "observation/wrist_image": _decode_png(wrist_png),
            "observation/state": state,
            "prompt": prompt.strip(),
        }
        started = time.monotonic()
        with self.lock:
            result = self.policy.infer(observation)
        actions = np.asarray(result["actions"], dtype=np.float64)
        if actions.shape != (10, 7) or not np.isfinite(actions).all():
            raise RuntimeError(f"Unexpected policy action shape: {actions.shape}")
        return {
            "actions": actions.tolist(),
            "infer_ms": (time.monotonic() - started) * 1000,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default="local/zekeep_pick_place_9hz")
    parser.add_argument("--exp-name", default="pick_place_9hz_1000")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--base-checkpoint",
        default="~/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch",
    )
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be in [1024, 65535]")

    policy, metadata = load_policy(args.repo_id, args.exp_name, args.base_checkpoint)
    runtime = RuntimePolicy(policy, metadata)
    with SimpleXMLRPCServer(
        ("127.0.0.1", args.port), allow_none=False, logRequests=False, use_builtin_types=True
    ) as server:
        server.register_introspection_functions()
        server.register_instance(runtime)
        print(json.dumps({"server": f"http://127.0.0.1:{args.port}", **metadata}, indent=2), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
