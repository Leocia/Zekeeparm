"""Use explicitly configured temporary fixtures; never authorize delivery defaults."""
from pathlib import Path
from tempfile import TemporaryDirectory
import yaml

ROOT = Path(__file__).resolve().parents[1]
_directory = TemporaryDirectory(prefix='zekeeparm-test-config-')
BASE = Path(_directory.name)
ACT_CONFIG = BASE / 'src/zekeep_act/configs/act.yaml'
RECORD_CONFIG = BASE / 'src/zekeep_embodied/configs/record.yaml'
ACT_CONFIG.parent.mkdir(parents=True)
RECORD_CONFIG.parent.mkdir(parents=True)

record_source = ROOT / 'src/zekeep_embodied/configs/record.yaml'
record = yaml.safe_load(record_source.read_text())
record['installation_id'] = 'offline-test-installation'
record['workspace']['confirmed'] = True
for key in ('teleop_config', 'output'):
    record[key] = str((record_source.parent / record[key]).resolve())
record['ros_joystick']['hardware_config'] = str((record_source.parent / record['ros_joystick']['hardware_config']).resolve())
for camera in record['cameras'].values():
    if 'config' in camera:
        camera['config'] = str((record_source.parent / camera['config']).resolve())
RECORD_CONFIG.write_text(yaml.safe_dump(record, allow_unicode=True))
settings = yaml.safe_load((ROOT / 'src/zekeep_act/configs/act.yaml').read_text())
settings['output'] = str(BASE / 'data/act')
ACT_CONFIG.write_text(yaml.safe_dump(settings, allow_unicode=True))
