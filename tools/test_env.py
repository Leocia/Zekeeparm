"""Check environment routing without Conda installs, ROS nodes or hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


REPORT = '''import json, os, sys
print(json.dumps({"args": sys.argv[1:], "cwd": os.getcwd(),
                  "env": {key: os.getenv(key) for key in (
                      "ZKEEP_WS", "ZKEEP_MINIFORGE_DIR", "EMBODIED_PYTHON",
                      "ZKEEP_VISION_PYTHON", "PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH",
                      "VIRTUAL_ENV", "CONDA_PREFIX", "JAX_PLATFORMS", "HF_LEROBOT_HOME",
                      "ZKEEPI_ASSETS", "ZKEEPI_CHECKPOINTS", "PYTHONNOUSERSITE")}}))
'''


class EnvironmentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'project with spaces'
        (self.root / 'tools').mkdir(parents=True)
        self.entry = self.root / 'tools/env.sh'
        shutil.copy2(Path(__file__).with_name('env.sh'), self.entry)
        self.report = self.root / 'report.py'
        self.report.write_text(REPORT)
        self.forge = self.root / 'custom miniforge'
        (self.forge / 'bin').mkdir(parents=True)
        conda = self.forge / 'bin/conda'
        conda.write_text('''#!/usr/bin/python3
import os, sys
assert sys.argv[1:3] == ["run", "--no-capture-output"]
assert sys.argv[3] == "-p"
prefix = sys.argv[4]
os.environ["CONDA_PREFIX"] = prefix
os.environ["PATH"] = prefix + "/bin:" + os.environ["PATH"]
os.execvpe(sys.argv[5], sys.argv[5:], os.environ)
''')
        conda.chmod(0o755)
        for name in ('rebotarm', 'lerobot', 'embodied', 'zekeep-studio'):
            self.make_python(self.forge / 'envs' / name)
        self.make_python(self.root / 'third_party/openpi/.venv')
        self.make_python(self.root / '.venv-ros')
        (self.root / 'tools/activate_ros.sh').write_text(
            'export PATH="$ZKEEP_WS/.venv-ros/bin:$PATH"\n'
            'export PYTHONPATH=selected-ros-overlay\n'
        )
        self.env = dict(os.environ, ZKEEP_MINIFORGE_DIR=str(self.forge),
                        PYTHONPATH='/wrong/overlay', PYTHONHOME='/wrong/python',
                        LD_LIBRARY_PATH='/wrong/libraries', CONDA_PREFIX='/wrong/conda',
                        VIRTUAL_ENV='/wrong/venv')
        for key in ('EMBODIED_PYTHON', 'ZKEEP_VISION_PYTHON', 'JAX_PLATFORMS',
                    'HF_LEROBOT_HOME', 'ZKEEPI_ASSETS', 'ZKEEPI_CHECKPOINTS'):
            self.env.pop(key, None)

    def make_python(self, prefix):
        (prefix / 'bin').mkdir(parents=True)
        (prefix / 'bin/python').symlink_to(sys.executable)

    def run_env(self, *args):
        return subprocess.run(['/bin/bash', str(self.entry), *args], cwd=self.temp.name,
                              env=self.env, text=True, capture_output=True, timeout=10)

    def test_all_profiles_choose_paths_and_keep_arguments(self):
        for profile, selected in (
            ('base', None), ('ros', None), ('vision', 'rebotarm'),
            ('teleop', 'lerobot'), ('vla', 'embodied'), ('studio', 'zekeep-studio'),
            ('model', None),
        ):
            with self.subTest(profile=profile):
                command = '/usr/bin/python3' if profile == 'base' else 'python'
                result = self.run_env(profile, command, str(self.report), 'two words', '--literal=$value')
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                self.assertEqual(report['args'], ['two words', '--literal=$value'])
                self.assertEqual(report['cwd'], str(self.root))
                env = report['env']
                self.assertEqual(env['ZKEEP_WS'], str(self.root))
                self.assertEqual(env['PYTHONNOUSERSITE'], '1')
                self.assertIsNone(env['PYTHONHOME'])
                self.assertIsNone(env['LD_LIBRARY_PATH'])
                if selected:
                    self.assertEqual(env['CONDA_PREFIX'], str(self.forge / 'envs' / selected))
                    self.assertIsNone(env['PYTHONPATH'])
                if profile == 'ros':
                    self.assertEqual(env['PYTHONPATH'], 'selected-ros-overlay')
                if profile == 'model':
                    self.assertEqual(env['VIRTUAL_ENV'], str(self.root / 'third_party/openpi/.venv'))
                    self.assertEqual(env['HF_LEROBOT_HOME'], str(self.root / 'data/openpi/datasets'))
                    self.assertEqual(env['PYTHONPATH'], str(self.root / 'src/zekeep_embodied/src'))

    def test_explicit_interpreters_and_dataset_override_are_preserved(self):
        alternate = self.root / 'alternate collection'
        self.make_python(alternate)
        self.env['EMBODIED_PYTHON'] = str(alternate / 'bin/python')
        self.env['HF_LEROBOT_HOME'] = '/chosen/datasets'
        result = self.run_env('vla', 'python', str(self.report))
        self.assertEqual(json.loads(result.stdout)['env']['CONDA_PREFIX'], str(alternate))
        result = self.run_env('model', 'python', str(self.report))
        self.assertEqual(json.loads(result.stdout)['env']['HF_LEROBOT_HOME'], '/chosen/datasets')

    def test_missing_environment_and_bad_profile_fail_before_running_command(self):
        (self.forge / 'envs/embodied/bin/python').unlink()
        result = self.run_env('vla', '/bin/echo', 'must not run')
        self.assertEqual(result.returncode, 1)
        self.assertIn('环境未安装', result.stderr)
        self.assertNotIn('must not run', result.stdout)
        self.assertEqual(self.run_env('unknown').returncode, 2)
        self.assertEqual(self.run_env('--help').returncode, 0)

    def test_command_failure_status_is_returned(self):
        self.assertEqual(self.run_env('base', '/bin/bash', '-c', 'exit 23').returncode, 23)


if __name__ == '__main__':
    unittest.main()
