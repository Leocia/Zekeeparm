"""UVC controls must take effect before capture, or fail without leaking devices."""

from pathlib import Path
import struct
import unittest
from unittest.mock import Mock, patch

import cv2

from zekeep_embodied.cameras import Camera


class CameraControlsTest(unittest.TestCase):
    def setUp(self):
        self.config = dict(type="opencv", device="/dev/fake", width=1280, height=720, fps=30)
        self.values = {cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*"MJPG")}
        self.device = Mock()
        self.device.set.side_effect = lambda prop, value: self.values.update({prop: value}) or True
        self.device.get.side_effect = lambda prop: self.values.get(prop, 0)
        capture = patch("zekeep_embodied.cameras.cv2.VideoCapture", return_value=self.device)
        thread = patch("zekeep_embodied.cameras.threading.Thread")
        capture.start()
        thread.start()
        self.addCleanup(capture.stop)
        self.addCleanup(thread.stop)

    def test_manual_controls_are_ordered_and_recorded(self):
        camera = Camera(dict(self.config, auto_exposure=False, exposure=100, gain=25), Path("."))
        self.addCleanup(camera.close)
        calls = self.device.set.call_args_list
        modes = [(call.args[0], call.args[1]) for call in calls if call.args[0] in (
            cv2.CAP_PROP_AUTO_EXPOSURE, cv2.CAP_PROP_EXPOSURE, cv2.CAP_PROP_GAIN)]
        self.assertEqual(modes, [(cv2.CAP_PROP_AUTO_EXPOSURE, 1),
                                 (cv2.CAP_PROP_EXPOSURE, 100), (cv2.CAP_PROP_GAIN, 25)])
        self.assertEqual({key: camera.device_info[key] for key in ("auto_exposure", "exposure", "gain")},
                         dict(auto_exposure=1, exposure=100, gain=25))

    def test_omitted_controls_preserve_device_settings(self):
        camera = Camera(self.config, Path("."))
        self.addCleanup(camera.close)
        self.assertNotIn(cv2.CAP_PROP_AUTO_EXPOSURE, self.values)
        self.assertNotIn(cv2.CAP_PROP_EXPOSURE, self.values)
        self.assertNotIn(cv2.CAP_PROP_GAIN, self.values)

    def test_auto_mode_uses_v4l2_aperture_priority(self):
        camera = Camera(dict(self.config, auto_exposure=True), Path("."))
        self.addCleanup(camera.close)
        self.assertEqual(camera.device_info["auto_exposure"], 3)

    def test_invalid_controls_release_camera(self):
        for controls in (dict(auto_exposure=True, exposure=100),
                         dict(auto_exposure=1), dict(auto_exposure=False, gain=-1),
                         dict(auto_exposure=False, exposure=0), dict(brightness=True),
                         dict(auto_white_balance=1), dict(exposure_auto_priority=1),
                         dict(power_line_frequency=True)):
            with self.subTest(controls=controls), self.assertRaises(ValueError):
                Camera(dict(self.config, **controls), Path("."))
            self.device.release.assert_called()
            self.device.release.reset_mock()

    def test_saved_c100_settings_are_applied_with_auto_modes(self):
        import yaml

        source = Path(__file__).resolve().parents[1] / "configs/record.yaml"
        config = yaml.safe_load(source.read_text())["cameras"]["global"]
        native = {}

        def ioctl(device, request, data):
            control_id, value = struct.unpack("Ii", data)
            if request == 0xC008561C:
                native[control_id] = value
            else:
                self.assertEqual(request, 0xC008561B)
                data[:] = struct.pack("Ii", control_id, native[control_id])

        with patch("zekeep_embodied.cameras.open"), patch("zekeep_embodied.cameras.fcntl.ioctl", side_effect=ioctl):
            camera = Camera(config, Path("."))
        self.addCleanup(camera.close)
        expected = {key: value for key, value in config.items() if key in (
            "auto_exposure", "exposure_auto_priority", "gain", "auto_white_balance", "brightness",
            "contrast", "saturation", "hue", "gamma", "sharpness", "backlight_compensation", "power_line_frequency")}
        expected["auto_exposure"] = 3
        self.assertEqual({key: camera.device_info[key] for key in expected}, expected)
        self.assertNotIn(cv2.CAP_PROP_EXPOSURE, self.values)
        self.assertEqual(native, {0x00980918: 1, 0x009A0903: 1})

    def test_rejected_or_ignored_control_releases_camera(self):
        for accepted in (False, True):
            self.device.set.side_effect = lambda prop, value: (
                accepted if prop == cv2.CAP_PROP_GAIN else self.values.update({prop: value}) or True)
            with self.subTest(accepted=accepted), self.assertRaisesRegex(RuntimeError, "rejected gain=25"):
                Camera(dict(self.config, auto_exposure=False, gain=25), Path("."))
            self.device.release.assert_called()
            self.device.release.reset_mock()

    def test_rejected_or_ignored_native_control_releases_camera(self):
        def ioctl(device, request, data):
            if request == 0xC008561B:
                control_id, _ = struct.unpack("Ii", data)
                data[:] = struct.pack("Ii", control_id, 0)

        for effect in (OSError("unsupported control"), ioctl):
            with self.subTest(effect=effect), patch("zekeep_embodied.cameras.open"), \
                    patch("zekeep_embodied.cameras.fcntl.ioctl", side_effect=effect), \
                    self.assertRaisesRegex(RuntimeError, "rejected power_line_frequency=1"):
                Camera(dict(self.config, power_line_frequency=1), Path("."))
            self.device.release.assert_called()
            self.device.release.reset_mock()


if __name__ == "__main__":
    unittest.main()
