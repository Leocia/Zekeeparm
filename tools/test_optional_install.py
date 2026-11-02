"""Exercise optional installer routes with disposable tools; no network or devices."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class OptionalInstallTest(unittest.TestCase):
    def test_every_module_installs_only_its_own_dependencies(self):
        with tempfile.TemporaryDirectory(prefix='optional install ') as temporary:
            root = Path(temporary)
            (root / 'tools').mkdir()
            entry = root / 'tools/install_optional.sh'
            shutil.copy2(Path(__file__).with_name('install_optional.sh'), entry)
            (root / 'tools/activate_ros.sh').write_text('export PATH="$TEST_BIN:$PATH"\n')
            for folder in ('apps/studio', 'web_mujoco', '.venv-ros/bin', 'forge/bin', 'bin'):
                (root / folder).mkdir(parents=True)
            (root / 'apps/studio/environment.yml').write_text('name: zekeep-studio\n')
            stub = root / 'stub.py'
            stub.write_text('''#!/usr/bin/python3
import json, os, sys
from pathlib import Path
name, args = Path(sys.argv[0]).name, sys.argv[1:]
with open(os.environ['TEST_LOG'], 'a') as out:
    out.write(json.dumps([name, args]) + '\\n')
if name == 'python3' and args[:2] == ['-m', 'venv']:
    path = Path(args[-1]) / 'bin/python'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.symlink_to(os.environ['TEST_STUB'])
''')
            stub.chmod(0o755)
            for name in ('python3', 'python', 'npm', 'node'):
                (root / 'bin' / name).symlink_to(stub)
            (root / '.venv-ros/bin/python').symlink_to(stub)
            (root / 'forge/bin/conda').symlink_to(stub)
            log = root / 'commands.jsonl'
            env = dict(os.environ, TEST_LOG=str(log), TEST_BIN=str(root / 'bin'), TEST_STUB=str(stub),
                       ZKEEP_MINIFORGE_DIR=str(root / 'forge'), PATH=str(root / 'bin') + ':' + os.environ['PATH'])
            for module in ('collect', 'models', 'sim', 'web-sim', 'studio'):
                with self.subTest(module=module):
                    log.write_text('')
                    result = subprocess.run(['bash', str(entry), module], env=env, capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    calls = [json.loads(line) for line in log.read_text().splitlines()]
                    text = json.dumps(calls)
                    self.assertTrue(calls)
                    self.assertNotIn('download_models', text)
                    self.assertNotIn('sudo', text)
                    expected = {'collect': 'requirements-embodied.txt', 'models': 'requirements-models.txt',
                                'sim': 'zekeep_sim', 'web-sim': 'build', 'studio': 'environment.yml'}[module]
                    self.assertIn(expected, text)
            (root / 'apps/studio/environment.yml').unlink()
            log.write_text('')
            result = subprocess.run(['bash', str(entry), 'studio'], env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn('未随 GitHub', result.stderr)
            self.assertEqual(log.read_text(), '')
            self.assertEqual(subprocess.run(['bash', str(entry), '--help'], capture_output=True).returncode, 0)
            self.assertEqual(subprocess.run(['bash', str(entry), 'unknown'], capture_output=True).returncode, 2)


if __name__ == '__main__':
    unittest.main()
