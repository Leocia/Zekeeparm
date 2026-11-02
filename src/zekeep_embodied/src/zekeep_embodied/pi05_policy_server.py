"""Local-only policy server for the guarded ROS runtime and saved fine-tuning recipe."""

import argparse
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


class LeRobotPolicy:
    """Keep the existing RPC observation contract while using native LeRobot processors."""

    def __init__(self, model, pre, post):
        self.model, self.pre, self.post = model, pre, post

    def infer(self, observation):
        import numpy as np
        import torch

        state = torch.as_tensor(observation["observation/state"], dtype=torch.float32).clone()
        if state.shape != (7,) or not torch.isfinite(state).all():
            raise ValueError("State must contain seven finite values")
        if not isinstance(observation.get("prompt"), str) or not observation["prompt"].strip():
            raise ValueError("Prompt must not be empty")
        batch = {"observation.state": state, "task": observation["prompt"]}
        for name, source in (("global", "observation/image"), ("wrist", "observation/wrist_image")):
            image = torch.as_tensor(observation[source])
            if image.ndim != 3:
                raise ValueError("Expected a three-dimensional RGB image")
            if image.shape[0] != 3 and image.shape[-1] == 3:
                image = image.permute(2, 0, 1)
            if image.shape[0] != 3:
                raise ValueError("Expected three RGB channels")
            image = image.float() / 255 if image.dtype == torch.uint8 else image.float()
            if not torch.isfinite(image).all() or image.min() < 0 or image.max() > 1:
                raise ValueError("Float RGB images must be finite and in [0, 1]")
            batch[f"observation.images.{name}"] = image
        with torch.inference_mode():
            actions = self.post(self.model.predict_action_chunk(self.pre(batch)))[0].cpu().numpy().copy()
        if actions.shape != (10, 7) or not np.isfinite(actions).all():
            raise RuntimeError("Invalid native LeRobot action chunk")
        actions[:, :6] += state.numpy()[:6]
        return {"actions": actions}


def load_policy(repo_id, exp_name, base_checkpoint, *, root=None):
    import numpy as np
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    from lerobot.policies.factory import make_pre_post_processors
    from .pi05 import BACKEND, checkpoint_dir, dataset_identity, load_statistics, validate_dataset
    from .finetune import load_adaptation, load_base_policy

    root = validate_dataset(repo_id, root=root)
    output = checkpoint_dir(exp_name)
    summary = json.loads((output / "finetune.json").read_text())
    if (summary.get("backend") != BACKEND or summary.get("repo_id") != repo_id
            or summary.get("dataset_identity") != dataset_identity(root)):
        raise ValueError("Checkpoint backend or dataset differs; legacy OpenPI adapters require retraining")
    statistics = load_statistics(root)
    if json.loads((output / "pi05_stats.json").read_text()) != statistics:
        raise ValueError("Checkpoint statistics do not match the selected training data")
    weights = Path(base_checkpoint).expanduser() / "model.safetensors"
    if not weights.is_file():
        raise FileNotFoundError(weights)
    if not torch.cuda.is_available():
        raise RuntimeError("Policy server requires CUDA")
    config = PreTrainedConfig.from_pretrained(output, local_files_only=True)
    if (not isinstance(config, PI05Config) or config.chunk_size != 10
            or config.output_features["action"].shape != (7,)):
        raise ValueError("Unsupported pi05 policy configuration")
    config.device = "cuda"
    config.compile_model = False
    pre, post = make_pre_post_processors(
        config, pretrained_path=str(output),
        preprocessor_overrides={
            "device_processor": {"device": "cuda"},
            "normalizer_processor": {"device": "cuda"},
            "tokenizer_processor": {"tokenizer_name": str(output / "tokenizer")},
        },
    )
    model = load_base_policy(config, weights.parent)
    adaptation = load_adaptation(model.model, output)
    model.to("cuda").eval()
    policy = LeRobotPolicy(model, pre, post)
    warmup_started = time.monotonic()
    policy.infer({
        "observation/image": np.zeros((224, 224, 3), dtype=np.uint8),
        "observation/wrist_image": np.zeros((224, 224, 3), dtype=np.uint8),
        "observation/state": np.zeros(7, dtype=np.float32), "prompt": "warm up policy",
    })
    return policy, {"repo_id": repo_id, "experiment": exp_name, "backend": BACKEND,
                    "action_horizon": 10, "action_dim": 7, "units": "degrees",
                    "warmup_ms": round((time.monotonic() - warmup_started) * 1000, 1), **adaptation}


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
        default="~/.cache/zekeep/pi05_base",
    )
    parser.add_argument("--dataset-root", type=Path)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be in [1024, 65535]")

    policy, metadata = load_policy(args.repo_id, args.exp_name, args.base_checkpoint, root=args.dataset_root)
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
