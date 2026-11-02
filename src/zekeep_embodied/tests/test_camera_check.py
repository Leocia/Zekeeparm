"""Camera-check screenshots must represent the end of the requested preview."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from zekeep_embodied import record


class CameraCheckTest(unittest.TestCase):
    def test_saves_settled_frame_at_end_without_opening_robot(self):
        clock = [0.]
        sequence = [0]
        camera = Mock(device_info={"device": "fake"})

        def sleep(seconds):
            clock[0] += seconds

        def snapshot():
            sequence[0] += 1
            color = (0, 255, 0) if clock[0] < 3 else (200, 200, 200)
            return np.full((2, 2, 3), color, np.uint8), clock[0], sequence[0]

        camera.snapshot.side_effect = snapshot
        with tempfile.TemporaryDirectory() as directory:
            config = dict(output=directory, cameras={"global": {}}, fps=10, max_sync_s=.1, max_cycle_s=.2)
            with patch.object(record, "Camera", return_value=camera), \
                    patch.object(record, "Hardware") as hardware, \
                    patch.object(record, "RosJoystickHardware") as ros_hardware, \
                    patch.object(record.time, "monotonic", side_effect=lambda: clock[0]), \
                    patch.object(record.time, "sleep", side_effect=sleep), \
                    patch.object(record.sys.stdin, "isatty", return_value=False), redirect_stdout(io.StringIO()):
                record.run(config, Path(directory), preview=True, seconds=4)
            output = next(Path(directory).glob("camera_check_*"))
            np.testing.assert_array_equal(cv2.imread(str(output / "global.png")), np.full((2, 2, 3), 200, np.uint8))
            self.assertTrue(json.loads((output / "camera_check.json").read_text())["fresh_samples_ok"])
            hardware.assert_not_called()
            ros_hardware.assert_not_called()
            camera.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
