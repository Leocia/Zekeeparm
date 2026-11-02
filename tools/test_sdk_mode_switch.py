"""Check startup failure handling using mock motors; never connect hardware."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import threading
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "zekeeparm_SDK"))
sys.path.insert(0, str(ROOT / "src/zekeep_grasp"))

sys.path.insert(0, str(ROOT / "src/zekeepcontroller"))
from zekeepcontroller.hardware_manager import HardwareManager
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
from drivers.robot.grasp_driver import GraspDriver


class ModeSwitchTest(unittest.TestCase):
    def test_gripper_position_command_restores_gains_after_free_mode(self):
        manager = HardwareManager.__new__(HardwareManager)
        manager._cmd_lock = threading.RLock()
        manager._begin_gripper_command = Mock()
        manager._gripper_mapping = Mock()
        manager._endpos_ctrl = Mock()
        manager._gripper_group = SimpleNamespace(_mit_kp=np.zeros(1), _mit_kd=np.zeros(1), send_mit=Mock())
        manager._gripper_mit_kp, manager._gripper_mit_kd = np.array([5.]), np.array([1.])
        manager.set_gripper_target(.5)
        np.testing.assert_array_equal(manager._gripper_group.send_mit.call_args.kwargs['kp'], [5.])
        np.testing.assert_array_equal(manager._gripper_group.send_mit.call_args.kwargs['kd'], [1.])
        manager._endpos_ctrl.set_gripper_target.assert_called_once_with(.5)

    def test_gain_write_failure_rejects_posvel_mode(self):
        arm = RebotArm()
        for name in arm.joint_names:
            arm._motor_map[name] = Mock()
        motor = arm._motor_map["joint4"]
        motor.write_register_f32.side_effect = RuntimeError("register write failed")
        with patch("zekeeparm_SDK.actuator.rebotarm.time.sleep"):
            self.assertFalse(arm.groups["arm"].mode_pos_vel())
        motor.ensure_mode.assert_not_called()
        self.assertFalse(arm._connected)

    def test_both_startup_paths_check_all_modes_before_enable(self):
        cfg = yaml.safe_load((ROOT / "src/zekeep_grasp/config/default.yaml").read_text())
        for use_driver in (False, True):
            for failed_group in ("arm", "gripper", None):
                with self.subTest(grasp_driver=use_driver, failed_group=failed_group):
                    arm = RebotArm()
                    arm.connect = Mock()
                    arm.start_control_loop = Mock()
                    for name in arm.joint_names:
                        arm._motor_map[name] = Mock()
                    controller = RebotArmEndPose(arm, arm_control_mode="posvel")
                    starter = controller
                    if use_driver:
                        starter = GraspDriver(arm, controller,
                            joint_mapping_config=cfg["robot"]["joint_mapping"])
                        starter.read_stable_arm_positions = Mock(return_value=np.zeros(6))
                        starter._prime_arm_target = Mock()
                        starter._prime_gripper_state = Mock()
                    arm.groups["arm"].mode_pos_vel = Mock(return_value=failed_group != "arm")
                    arm.groups["gripper"].mode_mit = Mock(return_value=failed_group != "gripper")
                    for group in arm.groups.values():
                        group.enable = Mock()
                    if failed_group is not None:
                        with self.assertRaisesRegex(RuntimeError, f"{failed_group} mode switch failed"):
                            starter.start()
                        for group in arm.groups.values():
                            group.enable.assert_not_called()
                        arm.start_control_loop.assert_not_called()
                    else:
                        starter.start()
                        for group in arm.groups.values():
                            group.enable.assert_called_once()
                        arm.start_control_loop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
