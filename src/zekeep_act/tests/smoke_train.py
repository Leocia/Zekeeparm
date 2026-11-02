"""Disposable synthetic 50-episode export/train/load/validate check; never opens hardware."""

import json
from pathlib import Path
import tempfile

import numpy as np

from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
from zekeep_act.config import fingerprint, load_config
from zekeep_act.data import prepare
from zekeep_act.policy import Policy, train, validate
from zekeep_embodied.storage import Episode, JOINTS


def main():
    import torch

    settings, capture, _ = load_config(DEFAULT_CONFIG)
    settings["training"]["num_workers"] = 0
    settings["training"]["save_freq"] = 1
    with tempfile.TemporaryDirectory(prefix="zekeep_act_smoke_") as directory:
        root = Path(directory)
        capture["output"] = str(root / "raw")
        settings["output"] = str(root)
        metadata = dict(capture, simulated=True, act_fingerprint=fingerprint(capture),
                        feedback_freshness="synthetic", timestamp_basis="synthetic",
                        hardware={"effective_robot": {"joint_limits": {name: [-220, 220] for name in JOINTS}}},
                        camera_devices={"wrist": "synthetic", "global": "synthetic"})
        for episode_index in range(50):
            episode = Episode(root / "raw", metadata)
            for frame in range(12):
                stamp = episode_index * 10 + frame / capture["fps"]
                state = np.array([-10, -30, -40, 0, 1, 2, 30.]) + frame * .02 + episode_index * .01
                images = {"wrist": np.full((24, 32, 3), 20 + frame, np.uint8),
                          "global": np.full((30, 40, 3), 40 + frame, np.uint8)}
                episode.append(dict(state=state, action=state + .03, observation_time=stamp,
                                    action_time=stamp + .001,
                                    camera_times={name: stamp for name in images},
                                    camera_sequences={name: frame + 1 for name in images}), images)
            episode.finish("success")
        prepared = prepare(settings, capture, allow_simulated=True)
        assert prepared["train_episodes"] == 45 and prepared["validation_episodes"] == 5
        trained = train(settings, capture, root / "dataset", root / "run", steps=1, allow_simulated=True)
        training_peak = torch.cuda.max_memory_allocated() // 1024 ** 2
        policy = Policy(trained["policy"], settings["training"]["device"])
        assert policy.contract["training"]["steps"] == 1
        try:
            policy.check_capture(settings, capture)
        except ValueError:
            pass
        else:
            raise AssertionError("Synthetic policy must never pass physical execution checks")
        images = {"wrist": np.full((24, 32, 3), 25, np.uint8), "global": np.full((30, 40, 3), 45, np.uint8)}
        torch.cuda.reset_peak_memory_stats()
        durations = []
        for _ in range(5):
            action, duration = policy.infer(images, [-10, -30, -40, 0, 1, 2, 30])
            assert len(action) == 7 and np.isfinite(action).all()
            durations.append(duration * 1000)
        del policy
        result = validate(trained["policy"], root / "dataset", max_frames=5)
        print(json.dumps({"synthetic_only": True, "prepared": prepared,
                          "inference_ms": durations, "validation": result,
                          "training_peak_memory_mib": training_peak,
                          "inference_peak_memory_mib": torch.cuda.max_memory_allocated() // 1024 ** 2}, indent=2))


if __name__ == "__main__":
    main()
