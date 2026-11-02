"""Exercise the batch launcher without starting Python, a browser, or hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]


@unittest.skipUnless(sys.platform == 'win32' or os.environ.get('STUDIO_TEST_WINE'),
                     'Windows cmd is required; STUDIO_TEST_WINE enables an explicit Wine check')
class WindowsLauncherTests(unittest.TestCase):
    def test_conda_discovery_and_explicit_path_with_spaces_and_chinese(self):
        wine = os.environ.get('STUDIO_TEST_WINE') if sys.platform != 'win32' else None
        with tempfile.TemporaryDirectory(prefix='studio-launcher-') as directory:
            root = Path(directory)
            studio = root / 'Studio 学生 demo'
            studio.mkdir()
            (studio / 'studio-working-directory.marker').touch()
            for name in ('start_windows.cmd', 'find_conda.cmd'):
                (studio / name).write_bytes((HERE / name).read_bytes().replace(b'\r\n', b'\n').replace(b'\n', b'\r\n'))
            conda = root / 'Miniforge custom' / 'condabin' / 'conda.bat'
            conda.parent.mkdir(parents=True)
            conda.write_bytes(b'@echo off\r\nif not exist "studio-working-directory.marker" exit /b 9\r\n> "%STUDIO_TEST_OUTPUT%" echo %*\r\nexit /b 0\r\n')
            result = root / 'result.txt'
            path = lambda p: 'Z:' + str(p).replace('/', '\\') if wine else str(p)
            for source in ('argument', 'STUDIO_CONDA', 'CONDA_EXE'):
                with self.subTest(source=source):
                    env = dict(os.environ)
                    for name in ('STUDIO_CONDA', 'CONDA_EXE'):
                        env.pop(name, None)
                    env['STUDIO_TEST_OUTPUT'] = path(result)
                    env['STUDIO_TEST_LAUNCHER'] = path(studio / 'start_windows.cmd')
                    env['STUDIO_TEST_CONDA'] = path(conda)
                    argument = ''
                    if source == 'argument':
                        env['STUDIO_CONDA'] = 'missing-conda.exe'
                        argument = ' "%STUDIO_TEST_CONDA%"'
                    else:
                        env[source] = path(conda)
                    wrapper = '@echo off\r\nchcp 65001 >nul\r\ncall "%STUDIO_TEST_LAUNCHER%"' + argument + '\r\nexit /b %errorlevel%\r\n'
                    (root / 'run-test.cmd').write_bytes(wrapper.encode('utf-8'))
                    if result.exists():
                        result.unlink()
                    command = [wine, 'cmd'] if wine else [env.get('COMSPEC', 'cmd.exe')]
                    completed = subprocess.run(command + ['/d', '/c', 'run-test.cmd'], cwd=root, env=env,
                                               capture_output=True, timeout=60)
                    self.assertEqual(completed.returncode, 0, (completed.stdout + completed.stderr).decode(errors='replace'))
                    lines = result.read_text(encoding='utf-8').splitlines()
                    self.assertEqual(lines[0].strip(), 'run --no-capture-output -n zekeep-studio python studio.py')


if __name__ == '__main__':
    unittest.main()
