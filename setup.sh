#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace_dir/tools/environment.sh"
miniforge_dir="$ZKEEP_MINIFORGE_DIR"
vision_env_dir="$miniforge_dir/envs/rebotarm"
teleop_env_dir="$miniforge_dir/envs/lerobot"
ros_venv_dir="${ZKEEP_ROS_VENV:-$workspace_dir/.venv-ros}"
ros_packages=(
  zekeep_msgs
  zekeepcontroller
  zekeep_bringup
  zekeep_moveit_config
  zekeep_teach
  zekeep_joystick
  zekeep_shadow
  zekeep_llm
)

die() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

step() {
  printf '\n== %s ==\n' "$*"
}


[[ "$(uname -m)" == "x86_64" ]] || die "只支持 x86_64"
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || die "只支持 Ubuntu 22.04"
[[ ${EUID} -ne 0 ]] || die "请用普通用户运行，脚本会在需要时调用 sudo"
conda_exe="$miniforge_dir/bin/conda"
[[ -x "$conda_exe" ]] || die "请先手动安装 Miniforge，并在 tools/environment.sh 配置 ZKEEP_MINIFORGE_DIR：$conda_exe"
case "${ZKEEP_DOWNLOAD_MODELS:-1}" in
  0|1) ;;
  *) die "ZKEEP_DOWNLOAD_MODELS 只接受 0（跳过基础模型）或 1（下载并验证）" ;;
esac
if [[ -e "$ros_venv_dir" && ! -f "$ros_venv_dir/conda-meta/history" ]]; then
  die "ROS 环境目录不是 Conda 环境：$ros_venv_dir；请按 docs/SDK_INSTALL.md 迁移旧 venv，脚本不会覆盖。"
fi
mkdir -p "$workspace_dir/log" "$workspace_dir/third_party"
setup_log="$workspace_dir/log/setup-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$setup_log") 2>&1
printf '安装日志：%s\n' "$setup_log"
printf 'NVIDIA 驱动和 CUDA Toolkit 须另行手动安装；版本与步骤见 docs/GPU_SETUP.md。\n'
trap 'printf "安装失败：第 %s 行；日志：%s\n" "$LINENO" "$setup_log" >&2' ERR
# A sourced workspace can make pip mistake foreign packages for local installs.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
"$conda_exe" --version
setup_temp_dir="$(mktemp -d)"
trap 'rm -rf -- "$setup_temp_dir"' EXIT

step "安装 ROS 2 Humble 和系统依赖"
sudo apt-get update
sudo apt-get install -y ca-certificates curl wget locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository universe -y

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  # Pin the bootstrap package so fresh installs do not depend on GitHub API limits.
  ros_apt_version=1.3.0
  ros_apt_deb="$setup_temp_dir/ros2-apt-source.deb"
  curl -fL --retry 3 -o "$ros_apt_deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.jammy_all.deb"
  sudo dpkg -i "$ros_apt_deb"
fi

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-rosbridge-server \
  ros-humble-joy ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rosidl-default-generators \
  ros-humble-tf-transformations python3-colcon-common-extensions \
  python3-rosdep python3-venv python3-pip python3-numpy python3-opencv python3-yaml \
  build-essential gcc-11 g++-11 \
  cmake git git-lfs lsof nodejs ffmpeg \
  libgl1 libglib2.0-0 libgomp1 libudev1 libusb-1.0-0 libc++1 libc++abi1

# ROS dependencies are installed by apt above; no rosdep network/cache prerequisite.
# rosdep update
sudo usermod -aG dialout "$(id -un)"

step "创建 ROS Python 环境并构建工作区"
if [[ -f "$ros_venv_dir/conda-meta/history" ]]; then
  "$conda_exe" env update -p "$ros_venv_dir" -f "$workspace_dir/tools/environment-ros.yml"
else
  "$conda_exe" env create -p "$ros_venv_dir" -f "$workspace_dir/tools/environment-ros.yml"
fi
set +u
source "$workspace_dir/tools/activate_ros.sh"
set -u
export PYTHONNOUSERSITE=1
python -m pip install --upgrade pip
python -m pip install -r "$workspace_dir/tools/requirements-ros-sdk.txt"
python -m pip install --no-deps -e "$workspace_dir/zekeeparm_SDK"
cd "$workspace_dir"
ros_build_dir="$workspace_dir/build"
python -m colcon --log-base "$workspace_dir/log" build \
  --build-base "$ros_build_dir" --install-base "$workspace_dir/install" \
  --base-paths src --packages-select "${ros_packages[@]}" --symlink-install
conda deactivate
unset VIRTUAL_ENV

step "使用已手动安装的 Conda"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

step "创建视觉环境"
if [[ -f "$vision_env_dir/conda-meta/history" ]]; then
  PIP_CONSTRAINT="$workspace_dir/tools/constraints-vision.txt" \
    "$conda_exe" env update -p "$vision_env_dir" -f "$workspace_dir/src/zekeep_grasp/environment.yml"
else
  PIP_CONSTRAINT="$workspace_dir/tools/constraints-vision.txt" \
    "$conda_exe" env create -p "$vision_env_dir" -f "$workspace_dir/src/zekeep_grasp/environment.yml"
fi
step "安装视觉 PyTorch CUDA 12.8 wheel（显示下载与错误输出）"
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install \
  -c "$workspace_dir/tools/constraints-vision.txt" \
  torch==2.7.1+cu128 torchvision==0.22.1+cu128 torchaudio==2.7.1+cu128 \
  --index-url https://download.pytorch.org/whl/cu128 --extra-index-url https://pypi.org/simple
step "安装相机、SDK 和视觉基础依赖"
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install \
  -c "$workspace_dir/tools/constraints-vision.txt" -r "$workspace_dir/tools/requirements-vision.txt"
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install \
  -c "$workspace_dir/tools/constraints-vision.txt" \
  ftfy regex tqdm pynput "git+https://github.com/ultralytics/CLIP.git@c4b6ea0932a2c0f39a0fa528af5ec4982ff15cab"
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install --no-deps -e "$workspace_dir/zekeeparm_SDK"
step "单独安装并验证 Ultralytics（YOLO26/YOLOE）"
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install \
  -c "$workspace_dir/tools/constraints-vision.txt" ultralytics==8.4.35
"$conda_exe" run --no-capture-output -p "$vision_env_dir" python -m pip install --force-reinstall --no-deps opencv-contrib-python==4.7.0.72
"$conda_exe" run -p "$vision_env_dir" python -c \
  'import ultralytics, numpy; from ultralytics import YOLO, YOLOE; assert ultralytics.__version__ == "8.4.35"; assert numpy.__version__ == "1.26.4"; print("Ultralytics / YOLO / YOLOE 导入和版本：OK")'

step "下载视觉模型"
if [[ "${ZKEEP_DOWNLOAD_MODELS:-1}" == 1 ]]; then
  bash "$workspace_dir/tools/download_models.sh" base
  "$conda_exe" run -p "$vision_env_dir" python -B "$workspace_dir/tools/check_yolo.py"
else
  printf '%s\n' '已显式跳过基础模型下载：视觉模型尚未完成安装；请按 README 下载并运行 tools/check_yolo.py。'
fi

step "创建 LeRobot 遥操作环境"
if [[ ! -f "$teleop_env_dir/conda-meta/history" ]]; then
  "$conda_exe" create --override-channels -c conda-forge -p "$teleop_env_dir" python=3.12 -y
fi
"$conda_exe" run -p "$teleop_env_dir" python -m pip install --upgrade pip
"$conda_exe" run -p "$teleop_env_dir" python -m pip install -e "$workspace_dir/src/zekeep_teleop"

step "配置 Orbbec USB 权限"
orbbec_rules_installer="$("$conda_exe" run -p "$vision_env_dir" python -c \
  'from importlib.util import find_spec; from pathlib import Path; print(Path(find_spec("pyorbbecsdk").origin).parent / "shared/install_udev_rules.sh")')"
[[ -f "$orbbec_rules_installer" ]] || die "缺少 Orbbec SDK udev 安装器：$orbbec_rules_installer"
sudo sh -e "$orbbec_rules_installer"

step "验证安装（不连接设备）"
(
  set +u
  source "$workspace_dir/tools/activate_ros.sh"
  set -u
  python -m pip check
  python -B "$workspace_dir/tools/check_sdk.py"
  python -c 'import rclpy, tf_transformations; from zekeep_msgs.srv import WebTeachCommand'
)
"$conda_exe" run -p "$vision_env_dir" python -m pip check
"$conda_exe" run -p "$vision_env_dir" python -c \
  'import cv2, numpy, torch, open3d, pyorbbecsdk, pyrealsense2, clip, motorbridge, coal; assert hasattr(cv2, "aruco"); assert numpy.__version__ == "1.26.4"; assert torch.__version__ == "2.7.1+cu128"; print("视觉依赖导入和版本：OK")'
if [[ "${ZKEEP_DOWNLOAD_MODELS:-1}" == 1 ]]; then
  "$conda_exe" run -p "$vision_env_dir" python -B "$workspace_dir/tools/check_vision_model.py"
fi
"$conda_exe" run -p "$teleop_env_dir" python -m pip check
"$conda_exe" run -p "$teleop_env_dir" python -c \
  'import lerobot_teleoperator_zhongling.teleoperate, servo_gui.server; from lerobot_robot_seeed_b601 import SeeedB601DMFollower; print("主从遥操作依赖导入：OK")'
printf '\n基础安装和无硬件导入检查完成。重新登录使 dialout 权限生效；相机请拔插一次。\n'
if [[ "${ZKEEP_DOWNLOAD_MODELS:-1}" == 0 ]]; then
  printf '%s\n' '注意：本次跳过了模型下载，视觉推理尚未就绪。'
fi
printf '%s\n' 'GraspNet 须按 third_party/README.md 独立下载、安装和验证；其余选装模块按 README 安装。'
printf '安装日志：%s\n' "$setup_log"
printf 'ROS：cd %q && source tools/activate_ros.sh\n' "$workspace_dir"
printf '视觉：%q run -p %q bash\n' "$conda_exe" "$vision_env_dir"
printf '遥操作：%q run -p %q zhongling-teleoperate --config_path=%q\n' "$conda_exe" "$teleop_env_dir" "$workspace_dir/src/zekeep_teleop/configs/zhongling_b601_teleop.yaml"
