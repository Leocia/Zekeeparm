import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
from zekeep_act.config import check_workspace, fingerprint, load_config
from zekeep_act import data
from zekeep_act.data import split_episodes
from zekeep_embodied.cameras import resize_with_padding
from zekeep_embodied.record import _ros_to_motor_degrees
from zekeep_embodied.ros_coordinates import guard_action, motor_degrees_to_ros
from zekeep_embodied.storage import Episode
from zekeep_embodied.capture import recording_metadata
from zekeep_embodied.record import load_config as load_capture_config


class DataContractTest(unittest.TestCase):
    def test_shared_and_act_configs_have_the_same_capture_identity(self):
        _, actual, _ = load_config(DEFAULT_CONFIG)
        source = yaml.safe_load(DEFAULT_CONFIG.read_text())
        common, base = load_capture_config(DEFAULT_CONFIG.parent / source["capture_config"])
        common["_camera_base"] = str(base)
        self.assertEqual(fingerprint(actual), fingerprint(common))

    def test_prepare_accepts_shared_and_legacy_identity_but_rejects_conflicts(self):
        settings, capture, _ = load_config(DEFAULT_CONFIG)
        settings = dict(settings, episodes=2, validation_episodes=1)
        with tempfile.TemporaryDirectory() as directory:
            capture["output"] = str(Path(directory) / "raw")
            metadata = dict(recording_metadata(capture), simulated=False)
            paths = []
            for _ in range(2):
                episode = Episode(capture["output"], metadata)
                for index in range(3):
                    stamp = index / 10
                    images = {name: np.zeros((4, 6, 3), np.uint8) for name in capture["cameras"]}
                    episode.append(dict(state=[0.] * 7, action=[0.] * 7,
                                        observation_time=stamp, action_time=stamp + .001,
                                        camera_times={name: stamp for name in images},
                                        camera_sequences={name: index + 1 for name in images}), images)
                paths.append(episode.finish("success") / "episode.json")
            originals = [json.loads(path.read_text()) for path in paths]
            for name, identities, valid in (
                ("shared", {"capture_fingerprint": fingerprint(capture)}, True),
                ("legacy", {"act_fingerprint": fingerprint(capture)}, True),
                ("both", {"capture_fingerprint": fingerprint(capture), "act_fingerprint": fingerprint(capture)}, True),
                ("conflict", {"capture_fingerprint": fingerprint(capture), "act_fingerprint": "wrong"}, False),
                ("missing", {}, False),
            ):
                for path, original in zip(paths, originals):
                    values = {key: value for key, value in original.items()
                              if key not in {"capture_fingerprint", "act_fingerprint"}}
                    path.write_text(json.dumps(dict(values, **identities)))
                output = Path(directory) / name
                with self.subTest(name=name), patch.object(data, "export") as exporter:
                    if valid:
                        result = data.prepare(settings, capture, output=output)
                        self.assertEqual((result["train_episodes"], result["validation_episodes"]), (1, 1))
                        self.assertEqual(exporter.call_count, 2)
                    else:
                        with self.assertRaisesRegex(ValueError, "capture contract"):
                            data.prepare(settings, capture, output=output)
                        exporter.assert_not_called()
                        self.assertFalse(output.with_name(output.name + ".partial").exists())

    def test_temporal_ensemble_window_and_coefficient_are_validated(self):
        source = yaml.safe_load(DEFAULT_CONFIG.read_text())
        source["capture_config"] = str((DEFAULT_CONFIG.parent / source["capture_config"]).resolve())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "act.yaml"
            for steps, coeff, valid in ((1, .01, True), (2, .01, True), (3, .01, True), (0, .01, False),
                                         (True, .01, False), (11, .01, False), (3, float("nan"), False)):
                source["inference"] = dict(temporal_ensemble_steps=steps, temporal_ensemble_coeff=coeff)
                path.write_text(yaml.safe_dump(source))
                if valid:
                    self.assertEqual(load_config(path)[0]["inference"], source["inference"])
                else:
                    with self.assertRaises(ValueError):
                        load_config(path)

    def test_double_speed_defaults_keep_a_bounded_velocity_ceiling(self):
        settings, _, _ = load_config(DEFAULT_CONFIG)
        self.assertEqual(settings["control"]["max_step_deg"], 1.)
        self.assertEqual(settings["control"]["max_velocity_rad_s"], .2)
        source = yaml.safe_load(DEFAULT_CONFIG.read_text())
        source["capture_config"] = str((DEFAULT_CONFIG.parent / source["capture_config"]).resolve())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "act.yaml"
            for speed in (.1, .2, .200001):
                source["control"]["max_velocity_rad_s"] = speed
                path.write_text(yaml.safe_dump(source))
                if speed <= .2:
                    self.assertEqual(load_config(path)[0]["control"]["max_velocity_rad_s"], speed)
                else:
                    with self.assertRaisesRegex(ValueError, "0.20 rad/s execution ceiling"):
                        load_config(path)

    def test_split_keeps_complete_episodes_disjoint_and_reproducible(self):
        paths = [f"episode_{index:03d}" for index in range(50)]
        train, validation = split_episodes(paths, 5, 42)
        self.assertEqual((len(train), len(validation)), (45, 5))
        self.assertFalse(set(train) & set(validation))
        self.assertEqual(set(train + validation), set(paths))
        self.assertEqual((train, validation), split_episodes(paths, 5, 42))
        self.assertNotEqual(validation, split_episodes(paths, 5, 43)[1])

    def test_padding_preserves_full_camera_view(self):
        image = np.full((20, 40, 3), 127, np.uint8)
        result = resize_with_padding(image, 40)
        self.assertEqual(result.shape, (40, 40, 3))
        np.testing.assert_array_equal(result[10:30], image)
        self.assertEqual(int(result[:10].sum() + result[30:].sum()), 0)

    def test_guard_and_nontrivial_calibration_round_trip(self):
        calibration = dict(direction=-1, gear_ratio=2, encoder_zero=.3, ros_zero=-.1)
        motor = [37.5]
        ros = motor_degrees_to_ros(motor, [calibration])
        np.testing.assert_allclose(_ros_to_motor_degrees(ros, [calibration]), motor)
        self.assertEqual(guard_action([0] * 7, [4] * 7, [(-10, 10)] * 7, .5), [.5] * 7)
        with self.assertRaises(RuntimeError):
            guard_action([0] * 7, [11] * 7, [(-10, 10)] * 7, .5)

    def test_independent_gripper_step_preserves_arm_and_hard_limit_checks(self):
        limits = [(-90., 90.)] * 7
        result = guard_action([0.] * 7, [10.] * 7, limits, 1., gripper_max_step_deg=3.)
        self.assertEqual(result, [1.] * 6 + [3.])
        with self.assertRaisesRegex(RuntimeError, "Policy joint 6"):
            guard_action([0.] * 7, [0.] * 6 + [91.], limits, 1., gripper_max_step_deg=3.)
        for step in (0., -1., float("nan"), float("inf")):
            with self.assertRaisesRegex(ValueError, "gripper_max_step_deg"):
                guard_action([0.] * 7, [10.] * 7, limits, 1., gripper_max_step_deg=step)

    def test_gripper_lower_tolerance_clamps_small_targets_and_keeps_other_limits(self):
        limits = [(0., 90.)] * 7
        state = [0.] * 6 + [-.667]
        with self.assertRaisesRegex(RuntimeError, "Current joint 6"):
            guard_action(state, [0.] * 7, limits, .5)
        result = guard_action(state, [0.] * 7, limits, .5, gripper_lower_tolerance_deg=1.)
        self.assertEqual(result, [0.] * 7)
        self.assertEqual(state[-1], -.667)
        for target in (-.043, -1.):
            result = guard_action(state, [0.] * 6 + [target], limits, .5,
                                  gripper_lower_tolerance_deg=1.)
            self.assertEqual(result, [0.] * 7)
        with self.assertRaisesRegex(RuntimeError, "Policy joint 6"):
            guard_action([0.] * 7, [0.] * 6 + [-.043], limits, .5)
        result = guard_action([0.] * 6 + [30.], [0.] * 6 + [-.043], limits, .5,
                              gripper_lower_tolerance_deg=1.)
        self.assertEqual(result[-1], 29.5)
        for current, proposed, error in (
            ([0.] * 6 + [-1.001], [0.] * 7, "Current joint 6"),
            ([0.] * 6 + [90.001], [0.] * 7, "Current joint 6"),
            ([-.001] + [0.] * 6, [0.] * 7, "Current joint 0"),
            (state, [0.] * 6 + [-1.001], "Policy joint 6"),
            (state, [0.] * 6 + [90.001], "Policy joint 6"),
            ([0.] * 7, [-.001] + [0.] * 6, "Policy joint 0"),
            (state, [0.] * 6 + [float("nan")], "non-finite"),
        ):
            with self.subTest(current=current, proposed=proposed):
                with self.assertRaisesRegex((RuntimeError, ValueError), error):
                    guard_action(current, proposed, limits, .5, gripper_lower_tolerance_deg=1.)
        result = guard_action(state, [0.] * 6 + [20.], limits, .5, gripper_lower_tolerance_deg=1.)
        self.assertEqual(result[-1], .5)
        for tolerance in (-1., float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                guard_action(state, [0.] * 7, limits, .5, gripper_lower_tolerance_deg=tolerance)

    def test_workspace_requires_explicit_physical_setup(self):
        settings, _, _ = load_config(DEFAULT_CONFIG)
        with self.assertRaises(ValueError):
            check_workspace(dict(settings["workspace"], confirmed=False))
        check_workspace(dict(confirmed=True, x_m=[.1, .3], y_m=[-.1, .1], table_z_m=.02,
                             description="supervised table"))

    def test_external_camera_config_change_invalidates_fingerprint(self):
        _, capture, _ = load_config(DEFAULT_CONFIG)
        capture = copy.deepcopy(capture)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "camera.yaml"
            source.write_text(yaml.safe_dump({"camera": {"serial": "first", "width": 640}}))
            capture["_camera_base"] = directory
            capture["cameras"]["wrist"]["config"] = "camera.yaml"
            before = fingerprint(capture)
            source.write_text(yaml.safe_dump({"camera": {"serial": "second", "width": 1280}}))
            self.assertNotEqual(before, fingerprint(capture))

    def test_prepare_rejects_timing_before_images_or_output(self):
        settings, capture, _ = load_config(DEFAULT_CONFIG)
        settings = dict(settings, episodes=2, validation_episodes=1)
        with tempfile.TemporaryDirectory() as directory:
            settings["output"] = directory
            capture["output"] = str(Path(directory) / "raw")
            metadata = dict(capture, simulated=False, act_fingerprint=fingerprint(capture))
            for stamps in ((0., .1, .2), (0., .16, .32)):
                episode = Episode(capture["output"], metadata)
                for index, stamp in enumerate(stamps):
                    images = {name: np.zeros((4, 6, 3), np.uint8) for name in capture["cameras"]}
                    episode.append(dict(state=[0.] * 7, action=[0.] * 7,
                                        observation_time=stamp, action_time=stamp + .001,
                                        camera_times={name: stamp for name in images},
                                        camera_sequences={name: index + 1 for name in images}), images)
                episode.finish("success")
            bad_path = Path(capture["output"]) / episode.path.name
            with patch.object(data, "export") as exporter, \
                    patch("zekeep_embodied.dataset.cv2.imread") as imread:
                with self.assertRaisesRegex(ValueError, bad_path.name + ".*Capture gap"):
                    data.prepare(settings, capture)
            exporter.assert_not_called()
            imread.assert_not_called()
            self.assertFalse((Path(directory) / "dataset.partial").exists())
            report = data.audit(bad_path.parent)
            self.assertEqual(len(report["success"]), 1)
            self.assertEqual(len(report["invalid"]), 1)
            self.assertEqual(report["invalid"][0]["path"], str(bad_path))


if __name__ == "__main__":
    unittest.main()
