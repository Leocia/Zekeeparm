"""Resolve ACT settings without opening devices."""

import math
from pathlib import Path

import yaml

from zekeep_embodied.record import load_config as load_capture_config
from zekeep_embodied.capture import check_workspace, fingerprint


def positive(value, name, *, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    if integer and (not isinstance(value, int)):
        raise ValueError(f"{name} must be an integer")


def load_config(path):
    path = Path(path).expanduser().resolve()
    settings = yaml.safe_load(path.read_text())
    capture, base = load_capture_config(path.parent / settings["capture_config"])
    if set(capture["cameras"]) != {"wrist", "global"} or not capture.get("ros_joystick"):
        raise ValueError("ACT requires wrist/global cameras and ROS hardware calibration")
    settings["output"] = str((path.parent / settings["output"]).resolve())
    if not isinstance(settings.get("repo_id"), str) or settings["repo_id"].count("/") != 1:
        raise ValueError("repo_id must be namespace/dataset")
    for key in ("image_size", "episodes", "validation_episodes"):
        positive(settings[key], key, integer=True)
    if not 0 < settings["validation_episodes"] < settings["episodes"]:
        raise ValueError("validation_episodes must be smaller than episodes")
    if not isinstance(settings["seed"], int) or isinstance(settings["seed"], bool) or settings["seed"] < 0:
        raise ValueError("seed must be a nonnegative integer")
    training = settings["training"]
    for key in ("batch_size", "steps", "save_freq", "chunk_size"):
        positive(training[key], f"training.{key}", integer=True)
    if not isinstance(training["num_workers"], int) or isinstance(training["num_workers"], bool) or training["num_workers"] < 0:
        raise ValueError("num_workers must be a nonnegative integer")
    positive(training["learning_rate"], "training.learning_rate")
    if training["device"] not in {"cpu", "cuda"} or not isinstance(training["pretrained_backbone"], bool):
        raise ValueError("training requires device cpu/cuda and boolean pretrained_backbone")
    inference = settings.setdefault("inference", dict(temporal_ensemble_steps=2, temporal_ensemble_coeff=0.01))
    positive(inference["temporal_ensemble_steps"], "inference.temporal_ensemble_steps", integer=True)
    positive(inference["temporal_ensemble_coeff"], "inference.temporal_ensemble_coeff")
    if inference["temporal_ensemble_steps"] > training["chunk_size"]:
        raise ValueError("temporal_ensemble_steps must not exceed training.chunk_size")
    for key, value in settings["control"].items():
        positive(value, f"control.{key}")
    if settings["control"]["max_velocity_rad_s"] > 0.20:
        raise ValueError("max_velocity_rad_s must not exceed the authorized 0.20 rad/s execution ceiling")
    gripper = settings.setdefault("gripper", dict(prediction_margin_deg=10.0))
    positive(gripper["prediction_margin_deg"], "gripper.prediction_margin_deg")
    capture["output"] = str(Path(settings["output"]) / "raw")
    settings["workspace"] = settings.get("workspace", capture.get("workspace", {}))
    capture["workspace"] = settings["workspace"]
    capture["_camera_base"] = str(base)
    return settings, capture, base
