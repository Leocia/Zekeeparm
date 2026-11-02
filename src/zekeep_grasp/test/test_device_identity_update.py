"""Device updates preserve configuration and recover from failed writes."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import confirm_calibration as update


class DeviceIdentityUpdateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        directory = self.root / "config/calibration/orbbec_gemini2"
        directory.mkdir(parents=True)
        for name in ("hand_eye.npz", "intrinsics.npz"):
            (directory / name).write_bytes(b"unchanged calibration")
        self.identity = directory / "identity.local.json"
        self.identity.write_text('{"previous": true}\n')
        self.config = self.root / "custom.yaml"
        self.original = ('# Preserve this comment\ncamera:\n  type: orbbec_gemini2\n'
                         '  serial: OLD\nrobot:\n  id: arm1\n'
                         'calibration:\n  installation_id: site1\n'
                         'safety:\n  motion_position_tolerance_rad: 0.005\n')
        self.config.write_text(self.original)
        self.camera = Mock(serial="NEW")
        self.addCleanup(patch.stopall)
        patch.object(update, "PROJECT_ROOT", self.root).start()
        patch.object(update, "make_camera", return_value=self.camera).start()

    def run_update(self):
        with redirect_stdout(io.StringIO()):
            update.main(["--config", str(self.config), "--confirm-same-installation"])

    def test_updates_custom_config_and_identity_with_backups(self):
        self.run_update()
        self.assertEqual(self.config.read_text(), self.original.replace("serial: OLD", 'serial: "NEW"'))
        cfg = yaml.safe_load(self.config.read_text())
        update.validate_calibration_identity(self.root, cfg, "NEW")
        self.assertEqual(json.loads(self.identity.read_text())["camera_serial"], "NEW")
        backups = list((self.root / "config/calibration/backups").iterdir())
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / "config.yaml").read_text(), self.original)
        self.assertEqual((backups[0] / "identity.local.json").read_text(), '{"previous": true}\n')
        self.camera.open.assert_called_once()
        self.camera.close.assert_called_once()

    def test_failed_identity_write_restores_both_files(self):
        with patch.object(update, "validate_calibration_identity", side_effect=OSError("validation failed")):
            with self.assertRaisesRegex(OSError, "validation failed"):
                self.run_update()
        self.assertEqual(self.config.read_text(), self.original)
        self.assertEqual(self.identity.read_text(), '{"previous": true}\n')

    def test_missing_identity_is_removed_after_failed_update(self):
        self.identity.unlink()
        with patch.object(update, "validate_calibration_identity", side_effect=OSError("validation failed")):
            with self.assertRaises(OSError):
                self.run_update()
        self.assertEqual(self.config.read_text(), self.original)
        self.assertFalse(self.identity.exists())

    def test_missing_serial_or_calibration_leaves_files_untouched(self):
        for failure in ("serial", "calibration"):
            with self.subTest(failure=failure):
                if failure == "serial":
                    self.camera.serial = " "
                else:
                    self.camera.serial = "NEW"
                    (self.identity.parent / "intrinsics.npz").unlink()
                with self.assertRaises((ValueError, FileNotFoundError)):
                    self.run_update()
                self.assertEqual(self.config.read_text(), self.original)
                self.assertEqual(self.identity.read_text(), '{"previous": true}\n')

    def test_requires_explicit_calibration_confirmation(self):
        with redirect_stdout(io.StringIO()), patch("sys.stderr", io.StringIO()):
            with self.assertRaises(SystemExit):
                update.main(["--config", str(self.config)])
        self.camera.open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
