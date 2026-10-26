#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"
if (($#)); then echo '用法：bash setup.sh' >&2; exit 2; fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '默认使用 ROS 2 Humble。请先按 README 安装 ROS，再运行 bash setup.sh。' >&2
  exit 1
fi
/usr/bin/python3 -c 'import sys, platform; assert sys.version_info[:2] == (3, 10) and platform.machine() == "x86_64", "Requires Ubuntu 22.04 x86_64 / Python 3.10"'
export PYTHONNOUSERSITE=1
unset PYTHONPATH
for environment in .venv-capture .venv-camera; do
  /usr/bin/python3 -m venv "$environment"
  "$environment/bin/python" -m pip install --upgrade pip 'setuptools>=71,<81' wheel
done
.venv-camera/bin/python -m pip install -r tools/requirements-camera.txt
# Exact CPU builds prevent later dependency resolution from selecting CUDA wheels.
.venv-capture/bin/python -m pip install \
  torch==2.6.0+cpu torchvision==0.21.0+cpu \
  --index-url https://download.pytorch.org/whl/cpu
.venv-capture/bin/python -m pip install \
  -c tools/requirements-leader.txt -r tools/requirements-leader.txt \
  -e src/zekeep_teleop -e src/zekeep_embodied
.venv-camera/bin/python -m pip check
.venv-capture/bin/python -m pip check
.venv-camera/bin/python -c 'import numpy, cv2, pyorbbecsdk; print("Camera imports OK; no device opened")'
.venv-capture/bin/python -c 'import torch; from zekeep_embodied.record import Hardware; from lerobot_teleoperator_zhongling import ZhonglingLeader; from lerobot_robot_seeed_b601 import SeeedB601DMFollower; assert torch.version.cuda is None; print("Leader imports / CPU PyTorch OK; no device opened")'
bash tools/install_ros.sh
./tools/capture.sh doctor
./tools/capture.sh self-check
echo '安装完成。先核对 README 中的相机权限、设备路径和标定，再做 camera-check。'
