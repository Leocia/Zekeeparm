"""LeRobot 0.4.4 ACT training, saved processors and offline validation.

Source: https://github.com/huggingface/lerobot/tree/v0.4.4/examples/tutorial/act
"""

import json
from pathlib import Path
import time

import numpy as np

from zekeep_embodied.cameras import resize_with_padding
from zekeep_embodied.storage import JOINTS, write_json

from .config import fingerprint, positive


def train(settings, capture, dataset, output, *, steps=None, allow_simulated=False):
    from lerobot.configs.default import DatasetConfig
    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.scripts.lerobot_train import train as lerobot_train

    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    manifest = json.loads((dataset / "split.json").read_text())
    if ((manifest["simulated"] and not allow_simulated) or manifest["fingerprint"] != fingerprint(capture)
            or manifest["fps"] != capture["fps"] or manifest["image_size"] != settings["image_size"]):
        raise ValueError("Training data must match this real capture configuration")
    values = dict(settings["training"])
    if steps is not None:
        values["steps"] = steps
    config = TrainPipelineConfig(
        dataset=DatasetConfig(repo_id=settings["repo_id"] + "_train", root=str(dataset / "train"),
                              video_backend="pyav"),
        policy=ACTConfig(device=values["device"], push_to_hub=False,
                         chunk_size=values["chunk_size"], n_action_steps=1,
                         pretrained_backbone_weights=("ResNet18_Weights.IMAGENET1K_V1"
                                                      if values["pretrained_backbone"] else None),
                         optimizer_lr=values["learning_rate"],
                         optimizer_lr_backbone=values["learning_rate"]),
        output_dir=output, seed=settings["seed"], batch_size=values["batch_size"],
        num_workers=values["num_workers"], steps=values["steps"],
        save_freq=values["save_freq"], log_freq=50, eval_freq=0,
    )
    try:
        lerobot_train(config)
    finally:
        if output.is_dir():
            contract = dict(manifest, training=values, control=settings["control"])
            write_json(output / "act_contract.json", contract)
            for checkpoint in (output / "checkpoints").glob("*/pretrained_model"):
                write_json(checkpoint / "act_contract.json", contract)
    return {"run": str(output), "policy": str(output / "checkpoints/last/pretrained_model")}


class Policy:
    def __init__(self, checkpoint, device="cuda", *, temporal_ensemble_steps=1, temporal_ensemble_coeff=0.01):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy, ACTTemporalEnsembler
        from lerobot.policies.factory import make_pre_post_processors

        self.torch = torch
        self.path = Path(checkpoint).expanduser().resolve()
        self.contract = json.loads((self.path / "act_contract.json").read_text())
        if (self.contract.get("units") != "degrees" or self.contract.get("joint_names") != list(JOINTS)
                or self.contract.get("action_semantics") != "absolute_motor_target_after_mapping_and_clipping"):
            raise ValueError("Unsupported checkpoint action units, ordering or semantics")
        config = PreTrainedConfig.from_pretrained(self.path, local_files_only=True)
        if not isinstance(config, ACTConfig):
            raise ValueError("Checkpoint must contain an ACT policy")
        positive(temporal_ensemble_steps, "temporal_ensemble_steps", integer=True)
        positive(temporal_ensemble_coeff, "temporal_ensemble_coeff")
        if temporal_ensemble_steps > config.chunk_size:
            raise ValueError("temporal_ensemble_steps exceeds checkpoint chunk_size")
        config.device = device
        config.n_action_steps = 1
        config.temporal_ensemble_coeff = None
        self.ensemble = (ACTTemporalEnsembler(temporal_ensemble_coeff, temporal_ensemble_steps)
                         if temporal_ensemble_steps > 1 else None)
        # Backbone weights are already in the local checkpoint; never fetch them on deployment.
        config.pretrained_backbone_weights = None
        self.policy = ACTPolicy.from_pretrained(self.path, config=config, local_files_only=True, strict=True)
        self.policy.to(device).eval()
        self.pre, self.post = make_pre_post_processors(
            config, pretrained_path=str(self.path),
            preprocessor_overrides={"device_processor": {"device": device},
                                    "normalizer_processor": {"device": device}})
        self.reset()

    def reset(self):
        self.policy.reset()
        self.pre.reset()
        self.post.reset()
        if self.ensemble is not None:
            self.ensemble.reset()
        self.raw_action = None

    def check_capture(self, settings, capture):
        if (self.contract["simulated"] or self.contract["fingerprint"] != fingerprint(capture)
                or self.contract["fps"] != capture["fps"]
                or self.contract["image_size"] != settings["image_size"]):
            raise ValueError("Checkpoint does not match the physical capture/mapping/camera contract")

    def infer(self, images, state):
        import cv2
        torch = self.torch
        batch = {"observation.state": torch.tensor(state, dtype=torch.float32)}
        for name in ("wrist", "global"):
            image = resize_with_padding(images[name], self.contract["image_size"])
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            batch[f"observation.images.{name}"] = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float() / 255
        started = time.monotonic()
        with torch.inference_mode():
            chunks = self.policy.predict_action_chunk(self.pre(batch))
            # Keep the newest gripper decision; only blend overlapping six-axis predictions.
            raw = chunks[:, 0].clone()
            action = raw.clone()
            if self.ensemble is not None:
                arms = chunks[:, :self.ensemble.chunk_size, :6]
                if not torch.isfinite(arms).all():
                    raise RuntimeError("ACT returned non-finite arm action chunks")
                action[:, :6] = self.ensemble.update(arms)
            decoded = self.post(torch.cat((raw, action), dim=0)).cpu().numpy()
            raw, action = decoded[0].reshape(-1), decoded[1].reshape(-1)
        if (raw.shape != (7,) or action.shape != (7,)
                or not np.isfinite(raw).all() or not np.isfinite(action).all()):
            raise RuntimeError("ACT returned invalid seven-dimensional motor targets")
        self.raw_action = raw.tolist()
        return action.tolist(), time.monotonic() - started


def validate(checkpoint, dataset, *, device="cuda", max_frames=100):
    """Report held-out motor-angle prediction error; this is not task success."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    policy = Policy(checkpoint, device)
    dataset = Path(dataset).resolve()
    split = json.loads((dataset / "split.json").read_text())
    if policy.contract["fingerprint"] != split["fingerprint"] or policy.contract["validation"] != split["validation"]:
        raise ValueError("Validation split does not belong to this checkpoint")
    frames = LeRobotDataset(split["repo_id"] + "_validation", root=dataset / "validation", video_backend="pyav")
    errors, durations = [], []
    count = min(len(frames), max_frames)
    if count <= 0:
        raise ValueError("Validation needs at least one frame")
    previous_episode = None
    torch = policy.torch
    for index in np.linspace(0, len(frames) - 1, count, dtype=int):
        sample = frames[int(index)]
        episode = int(sample["episode_index"])
        if episode != previous_episode:
            policy.reset()
        previous_episode = episode
        batch = {key: value for key, value in sample.items() if key.startswith("observation.")}
        start = time.monotonic()
        with torch.inference_mode():
            prediction = policy.post(policy.policy.select_action(policy.pre(batch))).cpu().numpy().reshape(7)
        durations.append(time.monotonic() - start)
        errors.append(np.abs(prediction - sample["action"].numpy()))
    return {"frames": count, "mae_deg_per_joint": np.mean(errors, axis=0).tolist(),
            "mean_inference_ms": float(np.mean(durations) * 1000),
            "p95_inference_ms": float(np.percentile(durations, 95) * 1000),
            "note": "offline prediction error, not physical pick/place success"}
