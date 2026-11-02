"""Protect source-only delivery and atomic, checked model downloads; no network."""
import hashlib
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import zipfile

from check_delivery import check

ROOT = Path(__file__).resolve().parents[1]


class DeliveryTest(unittest.TestCase):
    def test_private_files_weights_links_and_old_identity_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'src/zekeep_grasp/config/default.yaml'
            config.parent.mkdir(parents=True)
            config.write_text('camera:\n  serial: ""\nrobot:\n  id: ""\ncalibration:\n  installation_id: ""\n')
            calibration = config.parent / 'calibration/orbbec_gemini2/hand_eye.npz'
            calibration.parent.mkdir(parents=True)
            with zipfile.ZipFile(calibration, 'w') as archive:
                archive.writestr('T_result.npy', b'fixture')
            for package in ('zekeep_msgs', 'zekeepcontroller', 'zekeep_bringup', 'zekeep_moveit_config',
                            'zekeep_teach', 'zekeep_joystick', 'zekeep_shadow', 'zekeep_llm', 'zekeep_sim'):
                path = root / 'src' / package / 'package.xml'
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f'<package><name>{package}</name></package>')
            self.assertGreater(check(root)['files'], 0)
            (root / '.git').mkdir()
            (root / '.git/config').write_text('repository metadata')
            self.assertGreater(check(root)['files'], 0)
            for name in ('model.pt', 'jev.local.json', 'intrinsics.npz', 'identity.local.json'):
                path = root / name
                path.touch()
                with self.subTest(name=name), self.assertRaises(ValueError):
                    check(root)
                path.unlink()
            link = root / 'outside'
            link.symlink_to('/tmp')
            with self.assertRaises(ValueError):
                check(root)
            link.unlink()
            config.write_text(config.read_text().replace('serial: ""', 'serial: "OLD"'))
            with self.assertRaisesRegex(ValueError, 'serial'):
                check(root)

    def test_download_checks_hash_and_preserves_existing_file_on_failure(self):
        script = (ROOT / 'tools/download_models.sh').read_text()
        helper = script[script.index('download_checked() {'):script.index('\ncase "${1:-help}"')]
        program = 'set -euo pipefail\ndie() { exit 1; }\n' + helper + '\ndownload_checked "$TEST_URL" "$TEST_DEST" "$TEST_SHA"'
        with tempfile.TemporaryDirectory(prefix='model download ') as temporary:
            root = Path(temporary)
            source = root / 'payload'
            source.write_bytes(b'checked model fixture')
            destination = root / 'models/weights'
            env = dict(os.environ, TEST_URL=source.as_uri(), TEST_DEST=str(destination),
                       TEST_SHA=hashlib.sha256(source.read_bytes()).hexdigest())
            result = subprocess.run(['bash', '-c', program], env=env, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(destination.read_bytes(), source.read_bytes())
            # Matching files must work even after the source disappears.
            source.unlink()
            self.assertEqual(subprocess.run(['bash', '-c', program], env=env, capture_output=True).returncode, 0)
            source.write_bytes(b'corrupt replacement')
            env['TEST_SHA'] = '0' * 64
            self.assertNotEqual(subprocess.run(['bash', '-c', program], env=env, capture_output=True).returncode, 0)
            self.assertEqual(destination.read_bytes(), b'checked model fixture')
            self.assertFalse(list(destination.parent.glob('*.part.*')))


if __name__ == '__main__':
    unittest.main()
