"""New-device templates cannot silently authorize old calibration or collection."""
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]


class NewDeviceTest(unittest.TestCase):
    def test_serials_identity_and_motion_authorization_start_empty_or_disabled(self):
        vision = yaml.safe_load((ROOT / 'src/zekeep_grasp/config/default.yaml').read_text())
        self.assertEqual(vision['camera']['serial'], '')
        self.assertEqual(vision['robot']['id'], '')
        self.assertEqual(vision['calibration']['installation_id'], '')
        self.assertFalse(vision['place']['enabled'])
        poses = yaml.safe_load((ROOT / 'src/zekeep_grasp/config/calibration/orbbec_gemini2/auto_poses.yaml').read_text())
        self.assertEqual(poses['poses'], [])
        self.assertEqual(poses['validated_start_joint_positions'], [])
        for path in (ROOT / 'src/zekeep_bringup/config').glob('zekeep_hardware*.yaml'):
            overrides = yaml.safe_load(path.read_text())['models']['sixaxis']['overrides']
            self.assertFalse(overrides['control']['motion_authorized'], path)
            self.assertFalse(overrides['gravity_compensation']['supervised_trial_authorized'], path)
            for joint in overrides['joint_calibration'].values():
                self.assertEqual(joint['encoder_zero'], 0.0)
                self.assertEqual(joint['ros_zero'], 0.0)

    def test_collection_requires_a_new_installation_identity(self):
        from zekeep_embodied.record import load_config
        path = ROOT / 'src/zekeep_embodied/configs/record.yaml'
        with self.assertRaisesRegex(ValueError, 'installation_id'):
            load_config(path)
        self.assertFalse(yaml.safe_load(path.read_text())['workspace']['confirmed'])


if __name__ == '__main__':
    unittest.main()
