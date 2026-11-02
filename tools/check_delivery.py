"""Check the pristine source delivery without dependencies, downloads or hardware."""
from pathlib import Path
import ast
import json
import re
import subprocess
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.git', '.venv', '.venv-ros', '__pycache__', '.pytest_cache',
            'node_modules', 'build', 'install', 'dist', 'log', 'logs', 'Log'}
WEIGHTS = {'.pt', '.pth', '.safetensors', '.onnx', '.engine', '.ckpt'}


def check(root=ROOT):
    root = Path(root)
    counts = dict(files=0, python=0, shell=0, json=0, xml=0)
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if relative.parts[0] == '.git':
            continue
        if path.is_symlink():
            raise ValueError(f'External or generated link in delivery: {relative}')
        if path.is_dir():
            if path.name in EXCLUDED or path.name.endswith('.egg-info'):
                raise ValueError(f'Generated directory in delivery: {relative}')
            continue
        counts['files'] += 1
        if (path.suffix in WEIGHTS | {'.so', '.pyc', '.key', '.pem'}
                or path.name in {'mobileclip2_b.ts', 'checkpoint-rs.tar', 'intrinsics.npz', 'identity.local.json'}
                or path.name.endswith(('.local.json', '.local.yaml', '.local.yml'))
                or path.name in {'.env', '.env.local'}):
            raise ValueError(f'Private, generated or device-specific file: {relative}')
        if path.suffix == '.py':
            ast.parse(path.read_text(), filename=str(relative))
            counts['python'] += 1
        elif path.suffix == '.sh':
            subprocess.run(['bash', '-n', str(path)], check=True, capture_output=True)
            counts['shell'] += 1
        elif path.suffix in {'.json', '.webmanifest'}:
            contents = json.loads(path.read_text())
            counts['json'] += 1
            if path.name == 'block_colors.json' and contents.get('camera_serial'):
                raise ValueError('Old color-calibration camera serial retained')
        elif path.name == 'package.xml' or path.suffix in {'.urdf', '.srdf'}:
            ET.parse(path)
            counts['xml'] += 1
        if path.suffix in {'.py', '.js', '.sh', '.json', '.yaml', '.yml'}:
            text = path.read_text()
            if re.search(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----', text):
                raise ValueError(f'Private key material in {relative}')
            if re.search(r'"api_key"\s*:\s*"(?!"|YOUR_|\$|example)[^"]{12,}"', text):
                raise ValueError(f'Embedded credential in {relative}')
    config = (root / 'src/zekeep_grasp/config/default.yaml').read_text()
    for name in ('serial', 'id', 'installation_id'):
        if not re.search(rf'^  {name}: ""(?:\s|$)', config, re.M):
            raise ValueError(f'New-device field must be empty: {name}')
    hand_eye = root / 'src/zekeep_grasp/config/calibration/orbbec_gemini2/hand_eye.npz'
    with zipfile.ZipFile(hand_eye) as archive:
        if 'T_result.npy' not in archive.namelist():
            raise ValueError('Missing retained hand-eye transform')
    expected = {'zekeep_msgs', 'zekeepcontroller', 'zekeep_bringup', 'zekeep_moveit_config',
                'zekeep_teach', 'zekeep_joystick', 'zekeep_shadow', 'zekeep_llm', 'zekeep_sim'}
    packages = {ET.parse(path).getroot().findtext('name') for path in (root / 'src').glob('*/package.xml')}
    if packages != expected:
        raise ValueError(f'ROS package set differs: {packages ^ expected}')
    if any((root / 'src' / name).exists() for name in ('rebot_grasp', 'zekeep_vlm')):
        raise ValueError('Duplicate source and portable package names')
    return counts


if __name__ == '__main__':
    print(json.dumps(check(), ensure_ascii=False))
    print('Source delivery checks: OK; no device, GPU or clean-machine installation tested.')
