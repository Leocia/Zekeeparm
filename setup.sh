#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
miniforge_dir="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
vision_env_dir="$miniforge_dir/envs/rebotarm"
teleop_env_dir="$miniforge_dir/envs/lerobot"
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

download_checked() {
  local url="$1" destination="$2" expected="$3" temporary
  mkdir -p "$(dirname -- "$destination")"
  if [[ -f "$destination" ]] && printf '%s  %s\n' "$expected" "$destination" | sha256sum -c - >/dev/null 2>&1; then
    printf '已存在：%s\n' "$destination"
    return
  fi
  temporary="$(mktemp "${destination}.part.XXXXXX")"
  if ! curl -fL --retry 3 --retry-delay 2 -o "$temporary" "$url"; then
    rm -f -- "$temporary"
    return 1
  fi
  if ! printf '%s  %s\n' "$expected" "$temporary" | sha256sum -c -; then
    rm -f -- "$temporary"
    die "下载文件校验失败：$destination"
  fi
  mv -f -- "$temporary" "$destination"
}

[[ "$(uname -m)" == "x86_64" ]] || die "只支持 x86_64"
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || die "只支持 Ubuntu 22.04"
[[ ${EUID} -ne 0 ]] || die "请用普通用户运行，脚本会在需要时调用 sudo"
mkdir -p "$workspace_dir/log"
setup_log="$workspace_dir/log/setup-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$setup_log") 2>&1
printf '安装日志：%s\n' "$setup_log"
trap 'printf "安装失败：第 %s 行；日志：%s\n" "$LINENO" "$setup_log" >&2' ERR
# A sourced workspace can make pip mistake foreign packages for local installs.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONNOUSERSITE=1
setup_temp_dir="$(mktemp -d)"
trap 'rm -rf -- "$setup_temp_dir"' EXIT

step "安装 ROS 2 Humble 和系统依赖"
sudo apt-get update
sudo apt-get install -y ca-certificates curl wget locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository universe -y

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' | head -n1)"
  [[ -n "$ros_apt_version" ]] || die "无法取得 ros-apt-source 版本"
  ros_apt_deb="$setup_temp_dir/ros2-apt-source.deb"
  curl -fL -o "$ros_apt_deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.jammy_all.deb"
  sudo dpkg -i "$ros_apt_deb"
fi

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-rosbridge-server \
  ros-humble-joy ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-tf-transformations python3-colcon-common-extensions \
  python3-rosdep python3-venv python3-pip build-essential gcc-11 g++-11 \
  cmake git git-lfs lsof nodejs ffmpeg \
  libgl1 libglib2.0-0 libgomp1 libudev1 libusb-1.0-0 libc++1 libc++abi1

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install --from-paths "$workspace_dir/src" --ignore-src --rosdistro humble -r -y
sudo usermod -aG dialout "$(id -un)"

step "创建 ROS Python 环境并构建工作区"
if [[ ! -x "$workspace_dir/.venv-ros/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$workspace_dir/.venv-ros"
fi
set +u
source "$workspace_dir/.venv-ros/bin/activate"
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
deactivate

step "准备 Miniforge"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
conda_exe="$miniforge_dir/bin/conda"
if [[ ! -x "$conda_exe" ]]; then
  miniforge_installer="$setup_temp_dir/Miniforge3-Linux-x86_64.sh"
  curl -fL --retry 3 -o "$miniforge_installer" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  curl -fL --retry 3 -o "${miniforge_installer}.sha256" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh.sha256
  miniforge_sha="$(awk 'NR == 1 {print $1}' "${miniforge_installer}.sha256")"
  printf '%s  %s\n' "$miniforge_sha" "$miniforge_installer" | sha256sum -c -
  bash "$miniforge_installer" -b -p "$miniforge_dir"
fi

step "创建视觉环境"
if [[ -f "$vision_env_dir/conda-meta/history" ]]; then
  PIP_CONSTRAINT="$workspace_dir/tools/constraints-vision.txt" \
    "$conda_exe" env update -p "$vision_env_dir" -f "$workspace_dir/src/zekeep_grasp/environment.yml"
else
  PIP_CONSTRAINT="$workspace_dir/tools/constraints-vision.txt" \
    "$conda_exe" env create -p "$vision_env_dir" -f "$workspace_dir/src/zekeep_grasp/environment.yml"
fi
"$conda_exe" run -p "$vision_env_dir" python -m pip install --force-reinstall \
  -c "$workspace_dir/tools/constraints-vision.txt" \
  torch==2.7.1+cu128 torchvision==0.22.1+cu128 torchaudio==2.7.1+cu128 \
  --index-url https://download.pytorch.org/whl/cu128 \
  --extra-index-url https://pypi.org/simple
"$conda_exe" run -p "$vision_env_dir" python -m pip install \
  -c "$workspace_dir/tools/constraints-vision.txt" \
  ftfy regex tqdm "git+https://github.com/ultralytics/CLIP.git@c4b6ea0932a2c0f39a0fa528af5ec4982ff15cab"
"$conda_exe" run -p "$vision_env_dir" python -m pip install --no-deps -e "$workspace_dir/zekeeparm_SDK"
"$conda_exe" run -p "$vision_env_dir" python -m pip install --force-reinstall --no-deps opencv-contrib-python==4.7.0.72

step "下载视觉模型"
download_checked \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26l-seg.pt \
  "$workspace_dir/src/zekeep_grasp/models/yoloe-26l-seg.pt" \
  a612d2d505f24e14d87ec82d688b823b6cb600646664f16125ce6c84ce360da9
download_checked \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts \
  "$workspace_dir/src/zekeep_grasp/mobileclip2_b.ts" \
  35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f

graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}
if [[ "$graspnet_accepted" != YES ]]; then
  printf '%s\n' 'GraspNet 仅限本人或同一机构单站点的非商业内部研究，禁止转让或向第三方分发。'
  if ! read -r -p '接受上述限制并获取 GraspNet 源码和权重？输入 YES：' graspnet_accepted; then
    graspnet_accepted=NO
  fi
fi
if [[ "$graspnet_accepted" == YES ]]; then
  mkdir -p "$workspace_dir/third_party"
  if [[ ! -d "$workspace_dir/third_party/graspnet-baseline" ]]; then
    git clone --depth 1 https://github.com/graspnet/graspnet-baseline.git \
      "$workspace_dir/third_party/graspnet-baseline"
  fi
  if [[ ! -d "$workspace_dir/third_party/graspnet-baseline/graspnetAPI" ]]; then
    git clone --depth 1 https://github.com/graspnet/graspnetAPI.git \
      "$workspace_dir/third_party/graspnet-baseline/graspnetAPI"
  fi
  "$conda_exe" run -p "$vision_env_dir" python -m pip install \
    -c "$workspace_dir/tools/constraints-vision.txt" --no-build-isolation \
    "$workspace_dir/third_party/graspnet-baseline/graspnetAPI"
  download_checked \
    'https://drive.usercontent.google.com/download?id=1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk&export=download&confirm=t' \
    "$workspace_dir/third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar" \
    60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
else
  printf '%s\n' '已跳过 GraspNet 源码、权重和原生扩展。'
fi

step "创建 LeRobot 遥操作环境"
if [[ ! -f "$teleop_env_dir/conda-meta/history" ]]; then
  "$conda_exe" create -p "$teleop_env_dir" python=3.12 -y
fi
"$conda_exe" run -p "$teleop_env_dir" python -m pip install --upgrade pip
"$conda_exe" run -p "$teleop_env_dir" python -m pip install -e "$workspace_dir/src/zekeep_teleop"

step "配置 Orbbec USB 权限"
orbbec_rules_installer="$("$conda_exe" run -p "$vision_env_dir" python -c \
  'from importlib.util import find_spec; from pathlib import Path; print(Path(find_spec("pyorbbecsdk").origin).parent / "shared/install_udev_rules.sh")')"
[[ -f "$orbbec_rules_installer" ]] || die "缺少 Orbbec SDK udev 安装器：$orbbec_rules_installer"
sudo sh "$orbbec_rules_installer"

graspnet_extensions_ready=false
if [[ "$graspnet_accepted" == YES ]]; then
  cuda_home=${CUDA_HOME:-/usr/local/cuda-12.8}
  if [[ -x "$cuda_home/bin/nvcc" ]] && nvidia-smi >/dev/null 2>&1; then
    step "构建 GraspNet CUDA 扩展"
    gpu_arch="$("$conda_exe" run -p "$vision_env_dir" python -c 'import torch; print(".".join(map(str, torch.cuda.get_device_capability())))')"
    CUDA_HOME="$cuda_home" TORCH_CUDA_ARCH_LIST="$gpu_arch" \
      "$conda_exe" run -p "$vision_env_dir" bash "$workspace_dir/tools/build_vision_extensions.sh"
    graspnet_extensions_ready=true
  else
    printf '%s\n' '未找到 CUDA 12.8 Toolkit 或可见 NVIDIA GPU；已跳过 GraspNet 原生扩展。'
    printf '%s\n' '按 README 安装驱动和 CUDA 12.8 后重跑本脚本。'
  fi
fi

step "验证安装（不连接设备）"
(
  set +u
  source "$workspace_dir/tools/activate_ros.sh"
  set -u
  python -B "$workspace_dir/tools/check_sdk.py"
  python -c 'import rclpy, tf_transformations; from zekeep_msgs.srv import WebTeachCommand'
)
"$conda_exe" run -p "$vision_env_dir" python -m pip check
"$conda_exe" run -p "$vision_env_dir" python -c \
  'import cv2, numpy, torch, open3d, pyorbbecsdk, pyrealsense2, clip, motorbridge, coal; assert hasattr(cv2, "aruco"); assert numpy.__version__ == "1.26.4"; assert torch.__version__ == "2.7.1+cu128"; print("视觉依赖导入和版本：OK")'
"$conda_exe" run -p "$vision_env_dir" python -B "$workspace_dir/tools/check_vision_model.py"
"$conda_exe" run -p "$teleop_env_dir" python -m pip check
"$conda_exe" run -p "$teleop_env_dir" python -c \
  'import lerobot_teleoperator_zhongling.teleoperate, servo_gui.server; from lerobot_robot_seeed_b601 import SeeedB601DMFollower; print("主从遥操作依赖导入：OK")'
if [[ "$graspnet_extensions_ready" == true ]]; then
  "$conda_exe" run -p "$vision_env_dir" python -c \
    'import sys; sys.path.insert(0, sys.argv[1]); from utils.native_runtime import preload_environment_libstdcpp; preload_environment_libstdcpp(); import utils.graspnet_utils; print("GraspNet 后端导入：OK")' \
    "$workspace_dir/src/zekeep_grasp"
fi

printf '\n基础安装和无硬件导入检查完成。重新登录使 dialout 权限生效；相机请拔插一次。\n'
printf 'GraspNet CUDA 扩展已安装：%s；VLA、Studio 仍需按 README 独立选装。\n' "$graspnet_extensions_ready"
printf '安装日志：%s\n' "$setup_log"
printf 'ROS：cd %q && source tools/activate_ros.sh\n' "$workspace_dir"
printf '视觉：%q run -p %q bash\n' "$conda_exe" "$vision_env_dir"
printf '遥操作：%q run -p %q zhongling-teleoperate --config_path=%q\n' "$conda_exe" "$teleop_env_dir" "$workspace_dir/src/zekeep_teleop/configs/zhongling_b601_teleop.yaml"
