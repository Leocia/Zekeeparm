"""Offline Zekeep adapter for pinned openpi; this module never connects hardware.

Source: https://github.com/Physical-Intelligence/openpi/tree/215abfb217dbac7d5f1273282331b9b1866c0479
Reuse upstream RGB parsing, missing-view masks, delta actions and inverse transforms.
"""

import argparse
import dataclasses
import json
import os
from pathlib import Path
import runpy
import sys
import time

import numpy as np

OPENPI_REVISION = "215abfb217dbac7d5f1273282331b9b1866c0479"
CONFIG_NAME = "pi05_zekeep_lora"
REPO_ID = "local/zekeep_pick_place_train"


def make_config(repo_id=REPO_ID):
    from openpi import transforms
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import config, weight_loaders

    @dataclasses.dataclass(frozen=True)
    class ZekeepDataConfig(config.LeRobotLiberoDataConfig):
        def create(self, assets_dirs, model_config):
            result = super().create(assets_dirs, model_config)
            return dataclasses.replace(result, repack_transforms=transforms.Group(inputs=[
                transforms.RepackTransform({
                    "observation/image": "observation.images.global",
                    "observation/wrist_image": "observation.images.wrist",
                    "observation/state": "observation.state",
                    "actions": "action",
                    "prompt": "prompt",
                }),
            ]))

    model = Pi0Config(pi05=True, action_horizon=10, action_dim=32,
                      paligemma_variant="gemma_2b_lora", action_expert_variant="gemma_300m_lora")
    return config.TrainConfig(
        name=CONFIG_NAME,
        model=model,
        data=ZekeepDataConfig(repo_id=repo_id, extra_delta_transform=True,
                             base_config=config.DataConfig(prompt_from_task=True,
                                                           action_sequence_keys=("action",))),
        weight_loader=weight_loaders.CheckpointWeightLoader(
            "gs://openpi-assets/checkpoints/pi05_base/params"),
        freeze_filter=model.get_freeze_filter(),
        ema_decay=None,
        batch_size=1,
        num_workers=0,
        wandb_enabled=False,
        assets_base_dir=os.environ["ZKEEPI_ASSETS"],
        checkpoint_base_dir=os.environ["ZKEEPI_CHECKPOINTS"],
        policy_metadata={"robot": "zekeep", "units": "degrees", "action_dim": 7,
                         "action_semantics": "absolute_motor_target_after_mapping_and_clipping",
                         "hardware_execution_validated": False},
    )


def validate_dataset(repo_id, allow_simulated=False):
    from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME

    root = HF_LEROBOT_HOME / repo_id
    if not (root / "capture_provenance.json").is_file():
        raise FileNotFoundError(f"Dataset not prepared: {root}; export new recordings first")
    provenance = json.loads((root / "capture_provenance.json").read_text())
    metadata = provenance["capture_metadata"]
    if provenance.get("format") != "openpi-lerobot-v2.1" or provenance["units"] != "degrees":
        raise ValueError("Export this dataset using the pi05 export command first")
    if metadata.get("simulated") and not allow_simulated:
        raise ValueError("Simulated data is allowed only for explicit offline checks")
    return root


def check_dataset(repo_id, allow_simulated=False):
    from openpi.training import data_loader

    root = validate_dataset(repo_id, allow_simulated)
    config = make_config(repo_id)
    data = config.data.create(config.assets_dirs, config.model)
    dataset = data_loader.create_torch_dataset(data, config.model.action_horizon, config.model)
    transformed = data_loader.transform_dataset(dataset, data, skip_norm_stats=data.norm_stats is None)
    sample = transformed[0]
    assert sample["state"].shape == (32,)
    assert sample["actions"].shape == (10, 32)
    assert np.isfinite(sample["actions"]).all()
    assert all(im.shape == (224, 224, 3) for im in sample["image"].values())
    assert sample["image_mask"]["base_0_rgb"] and sample["image_mask"]["left_wrist_0_rgb"]
    assert not sample["image_mask"]["right_wrist_0_rgb"]
    assert sample["tokenized_prompt"].shape == (config.model.max_token_len,)
    return {"root": str(root), "frames": len(dataset), "action_shape": [10, 32],
            "robot_action_dim": 7, "normalized": data.norm_stats is not None,
            "note": "Data pipeline only; no model forward pass or hardware execution"}


def predict_frame(repo_id, exp_name, pytorch_weight_path, frame_index, output=None):
    """Predict from a recorded observation without starting ROS or opening devices."""
    if output is not None:
        output = Path(output).expanduser()
        if output.exists():
            raise FileExistsError(output)
        if not output.parent.is_dir():
            raise FileNotFoundError(f"Prediction output directory does not exist: {output.parent}")
    validate_dataset(repo_id)
    from openpi import transforms
    from openpi.training import data_loader
    from .pi05_policy_server import load_policy

    config = make_config(repo_id)
    data = config.data.create(config.assets_dirs, config.model)
    dataset = data_loader.create_torch_dataset(data, config.model.action_horizon, config.model)
    if not 0 <= frame_index < len(dataset):
        raise ValueError(f"frame-index must be in [0, {len(dataset) - 1}]")
    observation = transforms.compose(data.repack_transforms.inputs)(dataset[frame_index])
    observation.pop("actions", None)
    state = np.asarray(observation["observation/state"], dtype=np.float64)
    if state.shape != (7,) or not np.isfinite(state).all():
        raise ValueError("Recorded state must contain seven finite values")

    policy, metadata = load_policy(repo_id, exp_name, pytorch_weight_path)
    started = time.monotonic()
    prediction = policy.infer(observation)
    elapsed_ms = (time.monotonic() - started) * 1000
    actions = np.asarray(prediction["actions"], dtype=np.float64)
    if actions.shape != (10, 7) or not np.isfinite(actions).all():
        raise RuntimeError(f"Invalid policy actions: {actions.shape}")
    result = {
        "schema_version": 1,
        "kind": "zekeep_pi05_offline_prediction",
        **metadata,
        "frame_index": frame_index,
        "prompt": observation["prompt"],
        "state_deg": state.tolist(),
        "actions_deg": actions.tolist(),
        "infer_ms": round(elapsed_ms, 1),
        "note": "Recorded observation only; no hardware commands or grasp-success measurement",
    }
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if output is not None:
        with output.open("x", encoding="utf-8") as stream:
            stream.write(payload + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    export_parser = sub.add_parser("export", help="Export selected raw episodes to openpi LeRobot v2.1")
    export_parser.add_argument("raw", type=Path)
    export_parser.add_argument("--repo-id", default=REPO_ID)
    export_parser.add_argument("--fps", type=int)
    export_parser.add_argument("--allow-simulated", action="store_true")
    check_parser = sub.add_parser("check", help="Read images, action chunks and tokenize language offline")
    check_parser.add_argument("--repo-id", default=REPO_ID)
    check_parser.add_argument("--allow-simulated", action="store_true")
    for name in ("stats", "train", "config"):
        command = sub.add_parser(name)
        command.add_argument("--repo-id", default=REPO_ID)
    qlora_parser = sub.add_parser("train-qlora", help="Single-GPU NF4 QLoRA training")
    qlora_parser.add_argument("--repo-id", default=REPO_ID)
    qlora_parser.add_argument("--exp-name", required=True)
    qlora_parser.add_argument("--num-train-steps", type=int, default=1)
    qlora_parser.add_argument(
        "--pytorch-weight-path",
        default="~/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch",
    )
    qlora_parser.add_argument("--overwrite", action="store_true")
    predict_parser = sub.add_parser("predict", help="Predict one recorded frame; no ROS or hardware")
    predict_parser.add_argument("--repo-id", required=True)
    predict_parser.add_argument("--exp-name", required=True)
    predict_parser.add_argument("--frame-index", type=int, default=0)
    predict_parser.add_argument("--output", type=Path)
    predict_parser.add_argument(
        "--pytorch-weight-path",
        default="~/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch",
    )
    args, extra = parser.parse_known_args()
    if args.command != "train" and extra:
        parser.error(f"Unrecognized arguments: {extra}")
    if len(args.repo_id.split("/")) != 2 or any(p in ("", ".", "..") for p in args.repo_id.split("/")):
        parser.error("repo-id must be namespace/name")
    if args.command == "export":
        from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME
        from .dataset import export

        result = export(args.raw, HF_LEROBOT_HOME / args.repo_id, args.repo_id,
                        fps=args.fps, allow_simulated=args.allow_simulated, openpi_format=True)
        print(json.dumps(result, indent=2))
    elif args.command == "check":
        print(json.dumps(check_dataset(args.repo_id, args.allow_simulated), indent=2))
    elif args.command == "config":
        print(make_config(args.repo_id))
    elif args.command == "predict":
        print(json.dumps(predict_frame(
            args.repo_id, args.exp_name, args.pytorch_weight_path,
            args.frame_index, args.output,
        ), ensure_ascii=False, indent=2))
    elif args.command == "train-qlora":
        if args.num_train_steps <= 0:
            parser.error("num-train-steps must be positive")
        validate_dataset(args.repo_id)
        from .qlora import train

        print(json.dumps(train(
            make_config(args.repo_id),
            exp_name=args.exp_name,
            num_train_steps=args.num_train_steps,
            pytorch_weight_path=args.pytorch_weight_path,
            overwrite=args.overwrite,
        ), indent=2))
    else:
        # Pinning upstream makes this small registry adapter reproducible, without editing its source.
        from openpi.training import config
        import openpi

        validate_dataset(args.repo_id)
        config._CONFIGS_DICT[CONFIG_NAME] = make_config(args.repo_id)
        scripts = Path(openpi.__file__).resolve().parents[2] / "scripts"
        if args.command == "stats":
            sys.argv = [str(scripts / "compute_norm_stats.py"), "--config-name", CONFIG_NAME]
        else:
            sys.argv = [str(scripts / "train.py"), CONFIG_NAME, *extra]
        runpy.run_path(sys.argv[0], run_name="__main__")


if __name__ == "__main__":
    main()
