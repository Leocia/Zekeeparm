"""Moving computers does not require refreshing a saved calibration identity."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from utils.calibration_identity import calibration_identity, validate_calibration_identity


class CalibrationPortabilityTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.directory = self.root / "config/calibration/orbbec_gemini2"
        self.directory.mkdir(parents=True)
        for name in ("hand_eye.npz", "intrinsics.npz"):
            (self.directory / name).write_bytes(b"calibration")
        self.cfg = {
            "camera": {"type": "orbbec_gemini2", "serial": "CAM1"},
            "robot": {"id": "arm1", "end_effector_frame": "end_link"},
            "calibration": {"installation_id": "site1"},
        }

    def test_missing_stale_or_malformed_record_does_not_block_or_get_rewritten(self):
        record = self.directory / "identity.local.json"
        for contents in (None, '{"previous": true}\n', "invalid json"):
            with self.subTest(contents=contents):
                if contents is not None:
                    record.write_text(contents)
                self.assertIsNone(validate_calibration_identity(self.root, self.cfg, "CAM1"))
                self.assertEqual(record.read_text() if record.exists() else None, contents)

    def test_configuration_and_file_changes_need_no_confirmation(self):
        before = calibration_identity(self.root, self.cfg, "CAM1")
        self.cfg["place"] = {"enabled": False}
        (self.directory / "hand_eye.npz").write_bytes(b"updated calibration")
        validate_calibration_identity(self.root, self.cfg, "CAM1")
        after = calibration_identity(self.root, self.cfg, "CAM1")
        self.assertNotEqual(before["configuration_sha256"], after["configuration_sha256"])
        self.assertNotEqual(before["files"], after["files"])

    def test_missing_or_wrong_camera_serial_still_rejected(self):
        for serial in (None, "", "OTHER"):
            with self.subTest(serial=serial), self.assertRaisesRegex(ValueError, "camera serial"):
                validate_calibration_identity(self.root, self.cfg, serial)

    def test_missing_calibration_file_still_rejected(self):
        for name in ("hand_eye.npz", "intrinsics.npz"):
            with self.subTest(name=name):
                path = self.directory / name
                path.unlink()
                with self.assertRaises(FileNotFoundError):
                    validate_calibration_identity(self.root, self.cfg, "CAM1")
                path.write_bytes(b"calibration")


if __name__ == "__main__":
    unittest.main()
