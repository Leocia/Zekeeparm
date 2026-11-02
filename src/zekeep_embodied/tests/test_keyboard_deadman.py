import os
import unittest
from unittest.mock import Mock, patch

from zekeep_embodied.keyboard_deadman import KeyboardDeadman


class KeyboardDeadmanTest(unittest.TestCase):
    def test_startup_release_hold_rearm_timeout_and_close(self):
        keyboard = KeyboardDeadman()
        connection = Mock()

        def sample(pressed, now):
            keys = [0] * 32
            keys[58 // 8] = (1 << (58 % 8)) if pressed else 0
            connection.query_keymap.return_value = keys
            with patch("zekeep_embodied.keyboard_deadman.time.monotonic", return_value=now):
                keyboard._sample(connection, 58)
                return keyboard.held, keyboard.ready

        self.assertEqual(sample(True, 1.0), (True, False))
        self.assertEqual(sample(False, 1.02), (False, False))
        self.assertEqual(sample(True, 1.04), (True, True))
        keyboard.disarm()
        self.assertEqual(sample(True, 1.06), (True, False))
        self.assertEqual(sample(False, 1.08), (False, False))
        self.assertEqual(sample(True, 1.10), (True, True))
        with patch("zekeep_embodied.keyboard_deadman.time.monotonic", return_value=1.36):
            self.assertFalse(keyboard.held)
            self.assertFalse(keyboard.ready)
        keyboard.close()
        self.assertEqual(sample(True, 1.38), (False, False))

    def test_connection_loss_fails_closed(self):
        keyboard = KeyboardDeadman()
        keyboard._pressed = keyboard._released = True
        connection = Mock()
        connection.query_keymap.side_effect = OSError("desktop disconnected")
        with patch.object(keyboard._stop, "wait", return_value=False):
            keyboard._run(connection, 58)
        self.assertFalse(keyboard.held)
        self.assertFalse(keyboard.ready)
        self.assertIn("desktop disconnected", keyboard.error)
        connection.close.assert_called_once()

    def test_no_desktop_fails_before_motion(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "X11 desktop"):
                with KeyboardDeadman():
                    self.fail("monitor must not start")


if __name__ == "__main__":
    unittest.main()
