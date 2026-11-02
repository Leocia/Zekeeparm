"""LeRobot 0.4.4/v3 data, configuration and offline pi0.5 commands; no hardware."""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from .storage import JOINTS

CONFIG_NAME = "pi05_zekeep_lora"
REPO_ID = "local/zekeep_pick_place_train"
BACKEND = "lerobot-0.4.4-v3"
BASE_CHECKPOINT = "~/.cache/zekeep/pi05_base"
ACTION_SEMANTICS = "absolute_motor_target_after_mapping_and_clipping"


def dataset_root(repo_id, root=None):
    if len(repo_id.split("/")) != 2 or any(part in ("", ".", "..") for part in repo_id.split("/")):
        raise ValueError("repo-id must be namespace/name")
    if root is not None:
        return Path(root).expanduser().resolve()
    from lerobot.utils.constants import HF_LEROBOT_HOME
    return Path(HF_LEROBOT_HOME) / repo_id


def validate_dataset(repo_id, allow_simulated=False, root=None):
    root = dataset_root(repo_id, root)
    info = json.loads((root / "meta/info.json").read_text())
    if info.get("codebase_version") != "v3.0":
        raise ValueError("LeRobot v3 is required; re-export the original episodes into a new directory")
    provenance = json.loads((root / "capture_provenance.json").read_text())
    metadata = provenance["capture_metadata"]
    if (provenance.get("format") != "lerobot-v3" or provenance.get("units") != "degrees"
            or provenance.get("joint_names") != list(JOINTS)
            or metadata.get("action_semantics") != ACTION_SEMANTICS
            or set(metadata.get("cameras", {})) != {"global", "wrist"}
            or not str(metadata.get("task", "")).strip()):
        raise ValueError("Dataset does not match the Zekeep dual-camera/absolute-degree contract")
    if metadata.get("simulated") and not allow_simulated:
        raise ValueError("Simulated data is allowed only for explicit offline checks")
    for key in ("action", "observation.state"):
        if info["features"][key]["shape"] != [7]:
            raise ValueError(f"{key} must have seven dimensions")
    return root


def dataset_identity(root):
    return {name: json.loads((Path(root) / name).read_text())
            for name in ("capture_provenance.json", "meta/info.json")}


def make_config(repo_id=REPO_ID, *, device="cuda"):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    dataset_root(repo_id)  # Validate the public dataset identity even with an explicit root.
    return PI05Config(
        device=device, dtype="bfloat16", chunk_size=10, n_action_steps=1,
        gradient_checkpointing=True, compile_model=False, freeze_vision_encoder=True,
        input_features={
            "observation.images.global": PolicyFeature(FeatureType.VISUAL, (3, 224, 224)),
            "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, 224, 224)),
            "observation.state": PolicyFeature(FeatureType.STATE, (7,)),
        },
        output_features={"action": PolicyFeature(FeatureType.ACTION, (7,))},
        empty_cameras=1,
    )


def open_dataset(repo_id, *, root=None, allow_simulated=False):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = validate_dataset(repo_id, allow_simulated, root)
    fps = json.loads((root / "meta/info.json").read_text())["fps"]
    return LeRobotDataset(repo_id, root=root, video_backend="pyav",
                          delta_timestamps={"action": [index / fps for index in range(10)]})


def delta_actions(actions, state):
    """Use the current state for the whole chunk; gripper remains absolute."""
    result = actions.clone()
    result[..., :6] -= state[..., None, :6]
    return result


def statistics_path(root):
    return Path(root) / "pi05_stats.json"


def load_statistics(root):
    path = statistics_path(root)
    value = json.loads(path.read_text())
    if value.get("backend") != BACKEND or value.get("dataset_identity") != dataset_identity(root):
        raise ValueError("Statistics belong to a different backend or dataset; recompute with stats")
    if value.get("action_representation") != "arm_delta_gripper_absolute" or value.get("chunk_size") != 10:
        raise ValueError("Statistics action representation does not match the model")
    return value


def compute_statistics(repo_id, *, root=None, allow_simulated=False):
    """Compute train-only delta statistics using numeric columns, without decoding videos."""
    from lerobot.datasets.compute_stats import RunningQuantileStats
    dataset = open_dataset(repo_id, root=root, allow_simulated=allow_simulated)
    path = statistics_path(dataset.root)
    if path.exists():
        raise FileExistsError(f"Statistics already exist: {path}; choose a new dataset version")
    accumulators = {key: RunningQuantileStats() for key in ("observation.state", "action")}
    columns = dataset.hf_dataset.select_columns(["index", "episode_index", "observation.state"])
    for row in columns:
        query, padding = dataset._get_query_indices(int(row["index"]), int(row["episode_index"]))
        actions = dataset._query_hf_dataset(query)["action"]
        state = row["observation.state"]
        valid = ~padding["action_is_pad"]
        accumulators["observation.state"].update(state.numpy()[None].astype(np.float64))
        accumulators["action"].update(delta_actions(actions, state)[valid].numpy().astype(np.float64))
    value = dict(backend=BACKEND, dataset_identity=dataset_identity(dataset.root), chunk_size=10,
                 action_representation="arm_delta_gripper_absolute",
                 stats={key: {name: array.tolist() for name, array in accumulator.get_statistics().items()}
                        for key, accumulator in accumulators.items()})
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(payload + "\n")
    return {"statistics": str(path), "frames": len(dataset), "backend": BACKEND}


def make_processors(config, statistics):
    import torch
    from lerobot.policies.pi05.processor_pi05 import make_pi05_pre_post_processors
    stats = {key: {name: torch.tensor(value) for name, value in values.items()}
             for key, values in statistics["stats"].items()}
    return make_pi05_pre_post_processors(config, stats)


def check_dataset(repo_id, allow_simulated=False, *, root=None):
    dataset = open_dataset(repo_id, root=root, allow_simulated=allow_simulated)
    sample = dataset[0]
    if (sample["observation.state"].shape != (7,) or sample["action"].shape != (10, 7)
            or not np.isfinite(sample["action"].numpy()).all() or not sample["task"].strip()):
        raise ValueError("Invalid state, action chunk or language instruction")
    for name in ("global", "wrist"):
        image = sample[f"observation.images.{name}"]
        if image.ndim != 3 or image.shape[0] != 3 or not np.isfinite(image.numpy()).all():
            raise ValueError(f"Invalid {name} image")
    normalized = statistics_path(dataset.root).is_file()
    if normalized:
        load_statistics(dataset.root)
    return {"root": str(dataset.root), "format": "lerobot-v3", "frames": len(dataset),
            "action_shape": [10, 7], "model_action_dim": 32, "normalized": normalized,
            "note": "Data and timing only; no model weights, tokenizer or hardware loaded"}


def checkpoint_dir(exp_name):
    if not exp_name or Path(exp_name).name != exp_name or exp_name in (".", ".."):
        raise ValueError("exp-name must be a single new directory name")
    return Path(os.environ["ZKEEPI_CHECKPOINTS"]) / CONFIG_NAME / exp_name


def doctor(base_checkpoint=BASE_CHECKPOINT):
    import torch
    from transformers.models.siglip import check
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy  # noqa: F401
    if importlib.metadata.version("lerobot") != "0.4.4":
        raise RuntimeError("Shared model environment requires LeRobot 0.4.4")
    if not check.check_whether_transformers_replace_is_installed_correctly():
        raise RuntimeError("Install the pinned official Transformers fork from requirements-models.txt")
    return {"backend": BACKEND, "python": sys.version.split()[0],
            "cuda_available": torch.cuda.is_available(),
            "base_weights_present": (Path(base_checkpoint).expanduser() / "model.safetensors").is_file(),
            "note": "Import/dependency check only; no weights or devices opened"}


def predict_frame(repo_id, exp_name, pytorch_weight_path, frame_index, output=None, *, root=None):
    if output is not None:
        output = Path(output).expanduser()
        if output.exists():
            raise FileExistsError(output)
        if not output.parent.is_dir():
            raise FileNotFoundError(f"Prediction output directory does not exist: {output.parent}")
    dataset = open_dataset(repo_id, root=root)
    if not 0 <= frame_index < len(dataset):
        raise ValueError(f"frame-index must be in [0, {len(dataset) - 1}]")
    sample = dataset[frame_index]
    observation = {"observation/image": sample["observation.images.global"],
                   "observation/wrist_image": sample["observation.images.wrist"],
                   "observation/state": sample["observation.state"], "prompt": sample["task"]}
    state = np.asarray(observation["observation/state"], dtype=np.float64)
    if state.shape != (7,) or not np.isfinite(state).all():
        raise ValueError("Recorded state must contain seven finite values")
    from .pi05_policy_server import load_policy
    policy, metadata = load_policy(repo_id, exp_name, pytorch_weight_path, root=dataset.root)
    started = time.monotonic()
    actions = np.asarray(policy.infer(observation)["actions"], dtype=np.float64)
    if actions.shape != (10, 7) or not np.isfinite(actions).all():
        raise RuntimeError(f"Invalid policy actions: {actions.shape}")
    result = dict(schema_version=1, kind="zekeep_pi05_offline_prediction", **metadata,
                  frame_index=frame_index, prompt=observation["prompt"], state_deg=state.tolist(),
                  actions_deg=actions.tolist(), infer_ms=round((time.monotonic() - started) * 1000, 1),
                  note="Recorded observation only; no hardware commands or grasp-success measurement")
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if output is not None:
        with output.open("x", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("doctor", "export", "check", "stats", "train", "train-lora", "config", "predict"):
        child = commands.add_parser(name)
        if name == "doctor":
            child.add_argument("--base-checkpoint", default=BASE_CHECKPOINT)
            continue
        child.add_argument("--repo-id", default=REPO_ID)
        child.add_argument("--dataset-root", type=Path, help="Use an existing shared v3 directory, e.g. ACT dataset/train")
        if name == "export":
            child.add_argument("raw", type=Path)
            child.add_argument("--fps", type=int)
        if name in {"export", "check"}:
            child.add_argument("--allow-simulated", action="store_true")
        if name in {"train", "train-lora", "predict"}:
            child.add_argument("--exp-name", required=True)
            child.add_argument("--pytorch-weight-path", default=BASE_CHECKPOINT)
        if name in {"train", "train-lora"}:
            child.add_argument("--num-train-steps", type=int, required=True)
        if name == "predict":
            child.add_argument("--frame-index", type=int, default=0)
            child.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "doctor":
            result = doctor(args.base_checkpoint)
        elif args.command == "export":
            from .dataset import export
            result = export(args.raw, dataset_root(args.repo_id, args.dataset_root), args.repo_id,
                            fps=args.fps, allow_simulated=args.allow_simulated)
        elif args.command == "check":
            result = check_dataset(args.repo_id, args.allow_simulated, root=args.dataset_root)
        elif args.command == "stats":
            result = compute_statistics(args.repo_id, root=args.dataset_root)
        elif args.command == "config":
            result = vars(make_config(args.repo_id))
        elif args.command == "predict":
            result = predict_frame(args.repo_id, args.exp_name, args.pytorch_weight_path,
                                   args.frame_index, args.output, root=args.dataset_root)
        else:
            if args.num_train_steps <= 0:
                raise ValueError("num-train-steps must be positive")
            from .finetune import train
            result = train(make_config(args.repo_id), repo_id=args.repo_id, exp_name=args.exp_name,
                           num_train_steps=args.num_train_steps, pytorch_weight_path=args.pytorch_weight_path,
                           root=args.dataset_root)
        print(json.dumps(result, ensure_ascii=False, indent=2, default=str, allow_nan=False))
        return 0
    except (ValueError, RuntimeError, OSError, ImportError, KeyError) as exc:
        print(f"Pi05: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
