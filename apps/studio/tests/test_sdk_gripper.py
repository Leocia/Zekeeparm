"""Studio uses the hardware profile for gripper conversion; no device connection."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import SDKDevice


class SDKGripperTest(unittest.TestCase):
    def test_feedback_and_commands_follow_profile_in_both_motor_directions(self):
        for closed, opened in ((0.0, 1.15), (0.2, -0.95)):
            with self.subTest(closed=closed, opened=opened):
                device = SDKDevice.__new__(SDKDevice)
                manager = device.manager = SimpleNamespace(
                    enabled=False, gripper_open_position=opened, gripper_close_position=closed,
                    gripper_max_width=.070, set_gripper_target=Mock(),
                    get_joint_state=lambda: (np.zeros(6), np.zeros(6), np.zeros(6)),
                    get_motor_feedback_errors=lambda: [])
                for width in (0, 35, 65.17241379310346, 70):
                    expected = closed + width / 70 * (opened - closed)
                    device.gripper(width)
                    self.assertAlmostEqual(manager.set_gripper_target.call_args.args[0], expected)
                    manager.get_gripper_state = lambda: (expected, 0.0, 0.0, 0)
                    self.assertAlmostEqual(device.state()['opening_mm'], width)
                manager.set_gripper_target.reset_mock()
                for invalid in (-.01, 70.01, float('nan'), float('inf')):
                    with self.assertRaises(ValueError):
                        device.gripper(invalid)
                manager.set_gripper_target.assert_not_called()


if __name__ == '__main__':
    unittest.main()
