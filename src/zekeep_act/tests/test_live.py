import unittest
from unittest.mock import Mock, patch

from zekeep_act.live import run_policy


class LiveKeyboardTest(unittest.TestCase):
    def test_hold_starts_without_s_start_error_requires_release_and_m_does_not_hide_exit(self):
        settings = {"training": {"device": "cpu"}, "inference": {},
                    "control": {"inference_timeout_s": 0.2, "joy_timeout_s": 0.25}}
        capture = {"workspace": {}, "fps": 10}
        keyboard = Mock(ready=True)
        robot = Mock(active=False, stop_reason="m released")
        policy = Mock(contract={"camera_devices": {}})
        policy.infer.return_value = ([0.] * 7, 0.01)
        policy.raw_action = [0.] * 7
        robot.command.return_value = [0.] * 7

        def start(**_):
            if robot.start.call_count == 1:
                raise RuntimeError("driver busy")
            robot.active = True

        def stop(*_):
            robot.active = False
            keyboard.ready = False

        robot.start.side_effect = start
        robot.stop.side_effect = stop
        keyboard.disarm.side_effect = lambda: setattr(keyboard, "ready", False)
        commands = iter(("", "", "release", "hold", "mmmmme"))

        def command():
            value = next(commands)
            if value == "release":
                robot.stop("m released")
                return ""
            if value == "hold":
                keyboard.ready = True
                return ""
            return value

        with (patch("zekeep_act.policy.Policy", return_value=policy),
              patch("zekeep_act.robot.RosArm", return_value=robot) as create_robot,
              patch("zekeep_act.live.KeyboardDeadman") as monitor,
              patch("zekeep_act.live.check_workspace"),
              patch("zekeep_act.live.open_cameras", return_value={}),
              patch("zekeep_act.live.snapshot", return_value=([0.] * 7, {}, None, {}, None, 1.0)),
              patch("zekeep_act.live.terminal_command", side_effect=command),
              patch("zekeep_act.live.sys.stdin.isatty", return_value=True),
              patch("builtins.input", return_value="MOVE"),
              patch("builtins.print"), patch("zekeep_act.live.time.sleep")):
            monitor.return_value.__enter__.return_value = keyboard
            run_policy(settings, capture, None, "checkpoint", execute=True)
        self.assertEqual(robot.start.call_count, 2)
        keyboard.disarm.assert_called_once()
        self.assertEqual(robot.command.call_count, 1)
        self.assertEqual(policy.infer.call_count, 2)  # Warmup and one fresh action after rearming.
        self.assertIs(create_robot.call_args.kwargs["keyboard"], keyboard)
        robot.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
