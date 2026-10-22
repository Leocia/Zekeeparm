"""Check environment selection and GraspNet setup without installs or hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

script = (Path(__file__).resolve().parents[1] / "setup.sh").read_text()
block = script.split("graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}", 1)[1]
block = "graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}" + block.split('step "创建 LeRobot 遥操作环境"', 1)[0]
stubs = r"""
set -euo pipefail
conda_exe=conda_stub
vision_env_dir="$workspace_dir/envs/rebotarm"
git() { printf 'CLONE %s\n' "$*" >> "$CHECK_LOG"; mkdir -p "${@: -1}"; }
conda_stub() { printf 'INSTALL %s\n' "$*" >> "$CHECK_LOG"; }
download_checked() { printf 'DOWNLOAD %s\n' "$2" >> "$CHECK_LOG"; }
"""

with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    for name, accepted, existing, clone_count in (
        ("declined", "NO", False, 0),
        ("fresh", "YES", False, 2),
        ("existing", "YES", True, 0),
    ):
        workspace = root / name
        workspace.mkdir()
        if existing:
            (workspace / "third_party/graspnet-baseline/graspnetAPI").mkdir(parents=True)
        log = workspace / "commands"
        log.touch()
        environment = dict(os.environ, workspace_dir=str(workspace), CHECK_LOG=str(log),
                           ACCEPT_GRASPNET_LICENSE=accepted)
        subprocess.run(["bash", "-c", stubs + block], env=environment,
                       input="NO\n", text=True, check=True, capture_output=True)
        commands = log.read_text().splitlines()
        assert sum(line.startswith("CLONE ") for line in commands) == clone_count, commands
        if accepted == "YES":
            assert sum(line.startswith("INSTALL ") for line in commands) == 1, commands
            assert commands[-1].endswith("checkpoints/checkpoint-rs.tar"), commands
            if not existing:
                assert "graspnet/graspnet-baseline.git" in commands[0], commands
                assert "graspnet/graspnetAPI.git" in commands[1], commands
        else:
            assert not commands, commands
print("GraspNet declined, fresh checkout, and existing checkout: OK")

environment_block = script.split('step "准备 Miniforge"', 1)[1].split('step "下载视觉模型"', 1)[0]
teleop_block = script.split('step "创建 LeRobot 遥操作环境"', 1)[1].split('step "配置 Orbbec USB 权限"', 1)[0]
with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    other_bin = root / 'other-conda/bin'
    other_bin.mkdir(parents=True)
    other = other_bin / 'conda'
    other.write_text('#!/bin/bash\nexit 99\n')
    other.chmod(0o700)
    for existing in (False, True):
        miniforge = root / ('existing forge' if existing else 'fresh forge')
        conda = miniforge / 'bin/conda'
        conda.parent.mkdir(parents=True)
        conda.write_text(
            '#!/bin/bash\n'
            '[[ -z "${PYTHONPATH+x}${PYTHONHOME+x}${LD_LIBRARY_PATH+x}" ]] || exit 92\n'
            '[[ "$1" != env || "$PIP_CONSTRAINT" == "$workspace_dir/tools/constraints-vision.txt" ]] || exit 93\n'
            'printf "%s\\n" "$*" >> "$CHECK_LOG"\n'
        )
        conda.chmod(0o700)
        vision, teleop = miniforge / 'envs/rebotarm', miniforge / 'envs/lerobot'
        if existing:
            for prefix in (vision, teleop):
                (prefix / 'conda-meta').mkdir(parents=True)
                (prefix / 'conda-meta/history').touch()
        log = miniforge / 'commands'
        env = dict(os.environ, workspace_dir=str(root), miniforge_dir=str(miniforge),
                   vision_env_dir=str(vision), teleop_env_dir=str(teleop), CHECK_LOG=str(log),
                   PATH=str(other_bin) + os.pathsep + os.environ['PATH'],
                   PYTHONPATH='/old/ros/overlay', PYTHONHOME='/old/python', LD_LIBRARY_PATH='/old/ros/lib')
        subprocess.run(['bash', '-c', 'set -euo pipefail\nstep() { :; }\n'
                        + environment_block + teleop_block], env=env, check=True, capture_output=True)
        commands = log.read_text().splitlines()
        assert commands[0].startswith(f'env {"update" if existing else "create"} -p {vision} -f '), commands
        assert sum(line.startswith(f'create -p {teleop} ') for line in commands) == int(not existing), commands
        assert all('-n ' not in line for line in commands), commands
        assert all(str(vision) in line or str(teleop) in line for line in commands), commands
print('Miniforge prefixes, fresh/repeated setup, and competing PATH Conda: OK')

# Run the whole installer with disposable command substitutes; never install or open devices.
import json

with tempfile.TemporaryDirectory(prefix='zekeeparm-install-check-') as temporary:
    root = Path(temporary)
    tool = root / 'command_stub.py'
    tool.write_text(r'''#!/usr/bin/python3
import json, os, sys
from pathlib import Path
name, args = Path(sys.argv[0]).name, sys.argv[1:]
with open(os.environ['CHECK_LOG'], 'a') as log:
    log.write(json.dumps({'tool': name, 'args': args,
                         'pythonpath': os.getenv('PYTHONPATH'),
                         'pythonhome': os.getenv('PYTHONHOME'),
                         'libraries': os.getenv('LD_LIBRARY_PATH')}) + '\n')
if name == 'conda':
    if args[0] == 'env':
        history = Path(args[args.index('-p') + 1]) / 'conda-meta/history'
        history.parent.mkdir(parents=True, exist_ok=True)
        history.touch()
    elif args[0] == 'run':
        args = args[3:]
        if args[0] != 'python':
            sys.exit(0)
        name, args = 'python', args[1:]
if name in ('python', 'python3'):
    if args[:2] == ['-m', 'venv']:
        target = Path(args[-1]) / 'bin'
        target.mkdir(parents=True)
        (target / 'python').symlink_to(os.environ['CHECK_STUB_TOOL'])
        (target / 'activate').write_text('deactivate() { :; }\n')
    code = args[1] if args and args[0] == '-c' else ''
    if 'find_spec("pyorbbecsdk")' in code:
        print(os.environ['CHECK_ORBBEC_RULES'])
    elif 'get_device_capability' in code:
        print('8.6')
    elif os.getenv('CHECK_FAIL') == 'vision' and 'import cv2' in code:
        sys.exit(41)
    elif os.getenv('CHECK_FAIL') == 'backend' and 'import utils.graspnet_utils' in code:
        sys.exit(42)
elif name == 'git' and args[0] == 'clone':
    Path(args[-1]).mkdir(parents=True, exist_ok=True)
elif name == 'curl':
    if '-o' not in args:
        print('{"tag_name": "1.2.3"}')
    else:
        destination = Path(args[args.index('-o') + 1])
        if destination.name == 'Miniforge3-Linux-x86_64.sh':
            destination.write_text('#!/bin/bash\nmkdir -p "$3/bin"\nln -s "$CHECK_STUB_TOOL" "$3/bin/conda"\n')
        else:
            destination.write_bytes(b'disposable download')
elif name == 'sudo' and args[:2] == ['dpkg', '-i']:
    Path(os.environ['CHECK_ROS_SETUP']).write_text('test -z "$AMENT_TRACE_SETUP_FILES"\nexport PYTHONPATH=/fake/ros\nexport LD_LIBRARY_PATH=/fake/ros/lib\n')
elif name == 'sha256sum':
    sys.stdin.read()
elif name == 'nvidia-smi':
    sys.exit(0 if os.getenv('CHECK_GPU') == 'yes' else 1)
''')
    tool.chmod(0o755)
    fake_bin = root / 'bin'
    fake_bin.mkdir()
    for name in ('sudo', 'rosdep', 'python', 'python3', 'curl', 'sha256sum', 'git', 'nvidia-smi'):
        (fake_bin / name).symlink_to(tool)
    for label, accepted, gpu, fail in (
        ('basic', 'NO', 'no', ''),
        ('fresh-machine', 'NO', 'no', ''),
        ('graspnet', 'YES', 'yes', ''),
        ('no-cuda', 'YES', 'no', ''),
        ('bad-vision', 'NO', 'no', 'vision'),
        ('bad-backend', 'YES', 'yes', 'backend'),
    ):
        workspace = root / (label + ' workspace')
        workspace.mkdir()
        entry = workspace / 'setup.sh'
        # Platform/root guards are tested by the real entry point, outside this stub run.
        ros_setup = workspace / 'ros-setup.bash'
        entry.write_text(script.replace('[[ ${EUID} -ne 0 ]] || die', 'true || die')
                         .replace('/opt/ros/humble/setup.bash', f'"{ros_setup}"')
                         .replace('/usr/bin/python3', str(fake_bin / 'python3')))
        forge = workspace / 'miniforge'
        if label != 'fresh-machine':
            ros_setup.write_text('test -z "$AMENT_TRACE_SETUP_FILES"\nexport PYTHONPATH=/fake/ros\nexport LD_LIBRARY_PATH=/fake/ros/lib\n')
            (forge / 'bin').mkdir(parents=True)
            (forge / 'bin/conda').symlink_to(tool)
            (workspace / '.venv-ros/bin').mkdir(parents=True)
            (workspace / '.venv-ros/bin/python').symlink_to(tool)
            (workspace / '.venv-ros/bin/activate').write_text('deactivate() { :; }\n')
        tools = workspace / 'tools'
        tools.mkdir()
        (tools / 'activate_ros.sh').write_text('export PYTHONPATH=ros-check-only\n')
        rules = workspace / 'Orbbec SDK/shared/install_udev_rules.sh'
        rules.parent.mkdir(parents=True)
        rules.write_text('#!/bin/sh\nexit 0\n')
        cuda = workspace / 'cuda'
        (cuda / 'bin').mkdir(parents=True)
        (cuda / 'bin/nvcc').write_text('#!/bin/sh\nexit 0\n')
        (cuda / 'bin/nvcc').chmod(0o755)
        commands = workspace / 'commands.jsonl'
        environment = dict(os.environ, PATH=str(fake_bin) + os.pathsep + os.environ['PATH'],
                           ZKEEP_MINIFORGE_DIR=str(forge), CUDA_HOME=str(cuda),
                           ACCEPT_GRASPNET_LICENSE=accepted, CHECK_GPU=gpu, CHECK_FAIL=fail,
                           CHECK_LOG=str(commands), CHECK_ORBBEC_RULES=str(rules),
                           CHECK_ROS_SETUP=str(ros_setup), CHECK_STUB_TOOL=str(tool),
                           PYTHONPATH='/old/ros', PYTHONHOME='/old/python', LD_LIBRARY_PATH='/old/native-libraries')
        result = subprocess.run(['bash', str(entry)], env=environment,
                                text=True, capture_output=True, timeout=20)
        assert result.returncode == (41 if fail == 'vision' else 42 if fail else 0), (label, result.stdout, result.stderr)
        records = [json.loads(line) for line in commands.read_text().splitlines()]
        assert all(not r['pythonpath'] and not r['pythonhome'] and not r['libraries']
                   for r in records if r['tool'] == 'conda'), label
        assert any(r['tool'] == 'sudo' and r['args'] == ['sh', str(rules)] for r in records), label
        assert any(r['tool'] == 'sudo' and 'libgl1' in r['args'] and 'libc++1' in r['args'] for r in records), label
        backend = [r for r in records if 'import utils.graspnet_utils' in ' '.join(r['args'])]
        assert bool(backend) == (accepted == 'YES' and gpu == 'yes'), label
        assert not any('openpi' in ' '.join(r['args']) or 'embodied' in ' '.join(r['args']) for r in records), label
        logs = list((workspace / 'log').glob('setup-*.log'))
        assert len(logs) == 1, label
        if fail:
            assert '安装失败' in logs[0].read_text(), label
            assert '基础安装和无硬件导入检查完成' not in result.stdout, label
        else:
            assert '基础安装和无硬件导入检查完成' in result.stdout, label
            assert f'GraspNet CUDA 扩展已安装：{str(accepted == "YES" and gpu == "yes").lower()}' in result.stdout, label
print('Whole installer: clean Conda paths, USB rules, native libs, CUDA branches, import failures and retained logs: OK')
