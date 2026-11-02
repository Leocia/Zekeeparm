"""Check shutdown races without connecting ROS nodes, SDKs, or motors."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/zekeepcontroller'))
from zekeepcontroller.hardware_manager import HardwareManager
from zekeepcontroller.ros_publishers import JointStatePublisher
from zekeepcontroller.zekeep_controller import ZekeepController


def publisher():
    feedback = JointStatePublisher.__new__(JointStatePublisher)
    feedback._feedback_lock = threading.Lock()
    feedback._paused = False
    feedback._timer = Mock()
    feedback._publish = Mock()
    feedback._publish_status = Mock()
    return feedback


class FeedbackShutdownTest(unittest.TestCase):
    def test_disconnect_waits_for_active_feedback_and_blocks_queued_reads(self):
        feedback = publisher()
        entered, release, disconnected = (threading.Event() for _ in range(3))

        def read():
            self.assertFalse(disconnected.is_set())
            entered.set()
            self.assertTrue(release.wait(3))
            self.assertFalse(disconnected.is_set())

        feedback._publish.side_effect = read
        reader = threading.Thread(target=feedback.publish)
        reader.start()
        self.assertTrue(entered.wait(3))

        def close():
            feedback.pause()
            disconnected.set()

        closer = threading.Thread(target=close)
        try:
            closer.start()
            self.assertFalse(disconnected.wait(0.05))
        finally:
            release.set()
            reader.join(3)
            closer.join(3)
        self.assertFalse(reader.is_alive() or closer.is_alive())
        self.assertTrue(disconnected.is_set())
        feedback.publish()
        feedback.publish_status()
        feedback._publish.assert_called_once()
        feedback._publish_status.assert_not_called()
        feedback._timer.cancel.assert_called_once()

    def test_shutdown_order_and_failed_exit_recovery(self):
        for enabled, failure in ((True, None), (False, None),
                                 (True, 'home'), (True, 'disconnect')):
            with self.subTest(enabled=enabled, failure=failure):
                calls = []
                feedback = publisher()
                feedback._timer.cancel.side_effect = lambda: calls.append('pause')
                feedback._timer.reset.side_effect = lambda: calls.append('resume')
                hardware = HardwareManager.__new__(HardwareManager)
                hardware._cmd_lock = threading.RLock()
                hardware._connected = True
                hardware._enabled = enabled
                hardware._motion_generation = 0

                def home():
                    self.assertFalse(feedback._paused)
                    calls.append('home')
                    if failure == 'home':
                        raise RuntimeError('home rejected')

                def disable():
                    calls.append('disable')
                    hardware._enabled = False

                def disconnect():
                    self.assertTrue(feedback._paused)
                    calls.append('disconnect')
                    feedback.publish()
                    feedback.publish_status()
                    if failure == 'disconnect':
                        raise RuntimeError('disconnect failed')

                hardware.safe_home = home
                hardware.disable = disable
                hardware._robot = SimpleNamespace(disconnect=disconnect)
                for name in ('stop_motion', 'stop_gravity_compensation',
                             'hold_current_position', 'set_state_machine'):
                    setattr(hardware, name, Mock())
                node = SimpleNamespace(hardware=hardware, disable_after_safe_home=True,
                                       joint_state_publisher=feedback)
                if failure:
                    with self.assertRaisesRegex(RuntimeError, failure):
                        ZekeepController.shutdown(node)
                    self.assertTrue(hardware._connected)
                    self.assertFalse(feedback._paused)
                    self.assertEqual(calls[-1], 'resume')
                    if failure == 'home':
                        self.assertTrue(hardware._enabled)
                        self.assertNotIn('disable', calls)
                        self.assertNotIn('disconnect', calls)
                        hardware.hold_current_position.assert_called_once()
                    feedback.publish()
                    feedback._publish.assert_called_once()
                else:
                    ZekeepController.shutdown(node)
                    self.assertEqual(calls, (['home', 'disable'] if enabled else [])
                                     + ['pause', 'disconnect'])
                    self.assertFalse(hardware._connected or hardware._enabled)
                    feedback._publish.assert_not_called()
                feedback._publish_status.assert_not_called()


if __name__ == '__main__':
    unittest.main()
