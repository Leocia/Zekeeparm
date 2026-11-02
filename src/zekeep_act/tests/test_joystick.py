from tests.delivery_config import ACT_CONFIG as DEFAULT_CONFIG
"""Joystick recording uses the existing read-only ROS path, without devices."""

from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import uuid
from unittest.mock import Mock, patch

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from zekeep_act import cli, data, live
from zekeep_act.config import fingerprint, load_config
from zekeep_embodied import record
from zekeep_embodied import capture as shared_capture
from zekeep_embodied.storage import JOINTS
from test_ros_integration import FakeDriver


class JoystickRecordTest(unittest.TestCase):
    def test_records_and_prepares_joystick_episodes_without_leader(self):
        settings, capture, base = load_config(DEFAULT_CONFIG)
        settings = dict(settings, episodes=2, validation_episodes=1)
        clock = [1.]
        sequence = [0]
        hardware = Mock(source={
            "control_source": "ros_joystick", "effective_leader": {"type": "ros_joystick"},
            "effective_robot": {"joint_limits": {name: [-300, 300] for name in JOINTS}},
        })
        hardware.observation.return_value = [-10., -20., -30., 0., 1., 2., 30.]
        hardware.action.return_value = [-11., -21., -31., 0., 1., 2., 31.]

        def camera(config, _base):
            result = Mock(config=config, device_info={"device": "fake"})

            def snapshot():
                sequence[0] += 1
                return np.full((2, 2, 3), 100, np.uint8), clock[0], sequence[0]

            result.snapshot.side_effect = snapshot
            return result

        def sleep(seconds):
            clock[0] += seconds

        with tempfile.TemporaryDirectory() as directory:
            capture["output"] = str(Path(directory) / "raw")
            settings["output"] = directory
            with patch.object(record, "Camera", side_effect=camera), \
                    patch.object(record, "RosJoystickHardware", return_value=hardware), \
                    patch.object(record, "Hardware") as direct_hardware, \
                    patch.object(live, "Leader") as leader, \
                    patch.object(record.time, "monotonic", side_effect=lambda: clock[0]), \
                    patch.object(record.time, "sleep", side_effect=sleep), \
                    patch.object(record.sys.stdin, "isatty", return_value=True), \
                    patch.object(record.select, "select", return_value=([record.sys.stdin], [], [])), \
                    patch.object(record.sys.stdin, "readline", side_effect=["s\n", "p\n", "s\n", "w\n", "s\n", "p\n"]), \
                    redirect_stdout(io.StringIO()):
                live.record(settings, capture, base)
            direct_hardware.assert_not_called()
            leader.assert_not_called()
            hardware.close.assert_called_once()
            report = data.audit(capture["output"])
            self.assertEqual(len(report["success"]), 2)
            self.assertEqual(len(report["excluded"]), 1)
            self.assertEqual(report["invalid"], [])
            path = Path(report["success"][0]["path"])
            metadata = json.loads((path / "episode.json").read_text())
            self.assertEqual(metadata["act_fingerprint"], fingerprint(capture))
            self.assertEqual(metadata["capture_fingerprint"], fingerprint(capture))
            self.assertEqual(metadata["hardware"]["control_source"], "ros_joystick")
            row = json.loads((path / "frames.jsonl").read_text().splitlines()[0])
            self.assertEqual(row["action"], hardware.action.return_value)
            self.assertNotEqual(row["action"], row["state"])
            with patch.object(data, "export") as exporter:
                prepared = data.prepare(settings, capture)
            self.assertEqual((prepared["train_episodes"], prepared["validation_episodes"]), (1, 1))
            self.assertEqual(exporter.call_count, 2)

    def test_cli_keeps_both_collectors_and_output_override(self):
        with tempfile.TemporaryDirectory() as directory:
            for command, name in (("record", "record"), ("record-leader", "record_leader")):
                with self.subTest(command=command), patch.object(live, name) as collector:
                    self.assertEqual(cli.main([command, "--config", str(DEFAULT_CONFIG), "--output", directory]), 0)
                    self.assertEqual(collector.call_args.args[1]["output"], directory)

    def test_complete_collection_never_opens_devices(self):
        settings, capture, base = load_config(DEFAULT_CONFIG)
        with patch.object(shared_capture, "successful_episodes", return_value=[None] * settings["episodes"]), \
                patch.object(live.sys.stdin, "isatty", return_value=True), \
                patch.object(record, "run") as run:
            with self.assertRaisesRegex(ValueError, "already has"):
                live.record(settings, capture, base)
        run.assert_not_called()

    def test_gripper_feedback_uses_full_calibration(self):
        observer = record.RosJoystickHardware.__new__(record.RosJoystickHardware)
        observer.calibrations = [dict(direction=-1, gear_ratio=2, encoder_zero=.3, ros_zero=-.1)]
        observer._on_gripper_state(SimpleNamespace(position=.5, status_code=1))
        self.assertAlmostEqual(observer.gripper_state, math.degrees(0))
        self.assertEqual(observer.gripper_status, 1)

    def test_trigger_targets_are_recorded_with_gripper_calibration(self):
        observer = record.RosJoystickHardware.__new__(record.RosJoystickHardware)
        observer.calibrations = [dict(direction=-1, gear_ratio=2, encoder_zero=.3, ros_zero=-.1)]
        message = JointTrajectory()
        message.joint_names = list(record.ROS_ARM_JOINTS) + ["gripper"]
        point = JointTrajectoryPoint()
        message.points = [point]
        for position in (.5, .7, .6):
            point.positions = [.1] * 6 + [position]
            message.points = [point]
            observer._on_trajectory(message)
            expected = record._ros_to_motor_degrees([position], observer.calibrations)[0]
            self.assertAlmostEqual(observer.gripper_target, expected)
            self.assertEqual(observer.arm_target, [.1] * 6)
        for positions in ([.2] * 6, [.2] * 6 + [float("nan")]):
            point.positions = positions
            message.points = [point]
            observer._on_trajectory(message)
            self.assertAlmostEqual(observer.gripper_target, expected)

    def test_joystick_jitter_has_its_own_timeout_and_feedback_stays_strict(self):
        _, capture, _ = load_config(DEFAULT_CONFIG)
        observer = record.RosJoystickHardware.__new__(record.RosJoystickHardware)
        observer.config = capture["ros_joystick"]
        observer.arm_state = [0.] * 6
        observer.gripper_state = 0.
        observer.calibrations = [dict(direction=1, gear_ratio=1, encoder_zero=0, ros_zero=0)] * 6
        observer.status = SimpleNamespace(enabled=True, control_loop_active=True,
                                          error_codes=[], per_joint_status_code=[0] * 6)
        observer.gripper_status = 0
        observer.state_time = observer.gripper_time = 10.
        self.assertEqual(observer.config["max_state_age_s"], .1)
        self.assertEqual(observer.config["max_joy_age_s"], .3)
        with patch.object(record.time, "monotonic", return_value=10.):
            for age in (.10136, .299):
                with self.subTest(joy_age=age):
                    observer.joy_time = 10. - age
                    self.assertEqual(observer.observation(), [0.] * 7)
            observer.joy_time = 9.699
            with self.assertRaisesRegex(RuntimeError, "ROS joystick feedback is stale"):
                observer.observation()
            observer.joy_time = 10.
            for name in ("state_time", "gripper_time"):
                with self.subTest(stale_feedback=name):
                    setattr(observer, name, 9.899)
                    with self.assertRaisesRegex(RuntimeError, "ROS arm or gripper feedback is stale"):
                        observer.observation()
                    setattr(observer, name, 10.)

    def test_joystick_timeout_config_validation_and_legacy_default(self):
        source = DEFAULT_CONFIG.parent / "../../zekeep_embodied/configs/record.yaml"
        config = yaml.safe_load(source.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "record.yaml"
            for value in (0, -1, float("nan"), float("inf"), True, "0.3"):
                with self.subTest(value=value):
                    config["ros_joystick"]["max_joy_age_s"] = value
                    path.write_text(yaml.safe_dump(config))
                    with self.assertRaisesRegex(ValueError, "max_joy_age_s"):
                        record.load_config(path)
            del config["ros_joystick"]["max_joy_age_s"]
            path.write_text(yaml.safe_dump(config))
            loaded, _ = record.load_config(path)
            self.assertEqual(loaded["ros_joystick"]["max_joy_age_s"],
                             loaded["ros_joystick"]["max_state_age_s"])

    def test_idle_feedback_records_bounded_actions_without_changing_observation(self):
        observer = record.RosJoystickHardware.__new__(record.RosJoystickHardware)
        observer.status = None
        observer.source = {"effective_robot": {"joint_limits": {
            **{name: [-10., 10.] for name in JOINTS[:-1]}, "gripper": [0., 90.],
        }}}
        for feedback, target, expected in ((-.557, -.557, 0.), (-.557, None, 0.),
                                           (90.2, None, 90.), (30., 65., 65.)):
            with self.subTest(feedback=feedback, target=target):
                state = [0.] * 6 + [feedback]
                observer.gripper_target = target
                self.assertEqual(observer.action(state), [0.] * 6 + [expected])
                self.assertEqual(state[-1], feedback)
        observer.gripper_target = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            observer.action([0.] * 7)

    def test_ros_observer_records_targets_and_release_without_owning_servo(self):
        _, capture, _ = load_config(DEFAULT_CONFIG)
        namespace = "act_joystick_test_" + uuid.uuid4().hex[:8]
        config = dict(capture["ros_joystick"], arm_namespace=namespace, joy_topic="/" + namespace + "/joy")
        rclpy.init()
        driver = FakeDriver(namespace)
        driver.held = True
        driver.hardware._state_machine = "SERVO_RUNNING"
        driver.publish_arm_status()
        publisher = driver.create_publisher(JointTrajectory, driver.root + "/servo_joint_trajectory", 10)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        observer = None
        try:
            observer = record.RosJoystickHardware(config)
            observer.connect()
            message = JointTrajectory()
            message.joint_names = list(record.ROS_ARM_JOINTS)
            point = JointTrajectoryPoint()
            point.positions, point.velocities = [.4] * 6, [0.] * 6
            message.points = [point]
            deadline = time.monotonic() + 2
            while observer.target_time == 0 and time.monotonic() < deadline:
                publisher.publish(message)
                time.sleep(.01)
            self.assertGreater(observer.target_time, 0)
            state = observer.observation()
            action = observer.action(state)
            np.testing.assert_allclose(action[:6], record._ros_to_motor_degrees([.4] * 6, observer.calibrations[:6]))
            self.assertNotEqual(action[:6], state[:6])
            self.assertEqual(observer.node.count_publishers(driver.root + "/servo_joint_trajectory"), 1)
            driver.held = False
            while observer._button(config["deadman_button"]) and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(observer.action(state)[:6], state[:6])
            with patch.object(record.time, "monotonic", return_value=observer.joy_time + 1):
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    observer.observation()
            observer.status.error_codes = ["test fault"]
            with self.assertRaisesRegex(RuntimeError, "motor fault"):
                observer.observation()
        finally:
            executor.shutdown()
            thread.join(timeout=1)
            if observer:
                observer.close()
            driver.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
