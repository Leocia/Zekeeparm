from tests.delivery_config import RECORD_CONFIG
"""Shared capture CLI and identity checks; no devices or model imports required."""

import copy
from contextlib import redirect_stderr
import io
from pathlib import Path
import unittest
from unittest.mock import patch

from zekeep_embodied import capture, record


CONFIG = RECORD_CONFIG


class SharedCaptureTest(unittest.TestCase):
    def test_modes_receive_the_same_raw_contract_and_explicit_output(self):
        expected, base = record.load_config(CONFIG)
        expected["_camera_base"] = str(base)
        for mode, function in (("record", "record"), ("record-leader", "record_leader")):
            with self.subTest(mode=mode), patch.object(capture, function) as collector:
                self.assertEqual(capture.main([mode, "--config", str(CONFIG),
                                               "--output", "data/shared raw", "--episodes", "8"]), 0)
                settings, actual, actual_base = collector.call_args.args
                self.assertEqual(actual_base, base)
                self.assertEqual(settings["episodes"], 8)
                self.assertEqual(actual["output"], str(Path("data/shared raw").resolve()))
                self.assertEqual(capture.fingerprint(actual), capture.fingerprint(expected))
                metadata = capture.recording_metadata(actual)
                self.assertEqual(metadata["capture_fingerprint"], metadata["act_fingerprint"])

    def test_invalid_configuration_is_rejected_before_a_collector_starts(self):
        source, base = record.load_config(CONFIG)
        invalid = []
        for workspace in ({}, dict(source["workspace"], confirmed=False)):
            invalid.append(dict(source, workspace=workspace))
        invalid.append(dict(source, cameras={"wrist": source["cameras"]["wrist"]}))
        for value in (0, -1, float("nan"), float("inf"), True):
            item = copy.deepcopy(source)
            item["leader_control"]["feedback_timeout_s"] = value
            invalid.append(item)
        for item in invalid:
            with self.subTest(config=item), patch.object(capture, "load_config", return_value=(item, base)), \
                    patch.object(capture, "record_leader") as collector, redirect_stderr(io.StringIO()):
                self.assertEqual(capture.main(["record-leader", "--config", str(CONFIG),
                                               "--output", "unused"]), 2)
                collector.assert_not_called()

    def test_invalid_count_never_loads_config_or_devices(self):
        for arguments in (["record", "--config", str(CONFIG)],
                          ["record", "--config", str(CONFIG), "--output", "unused", "--episodes", "0"]):
            with self.subTest(arguments=arguments), patch.object(capture, "load_config") as loader, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                capture.main(arguments)
            self.assertEqual(error.exception.code, 2)
            loader.assert_not_called()


if __name__ == "__main__":
    unittest.main()
