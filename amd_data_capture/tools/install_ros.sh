#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '请先按 README 安装 ROS 2 Humble，再运行此脚本。' >&2
  exit 1
fi
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
set -u
sudo apt-get update
sudo apt-get install -y ros-humble-moveit ros-humble-joy \
  ros-humble-tf-transformations ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-xacro \
  python3-colcon-common-extensions python3-rosdep python3-venv \
  build-essential cmake lsof
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
rosdep install --from-paths src/zekeep_bringup src/zekeepcontroller src/zekeep_msgs \
  src/zekeep_joystick src/zekeep_moveit_config --ignore-src --rosdistro humble -r -y
/usr/bin/python3 -m venv --system-site-packages .venv-ros
export PYTHONNOUSERSITE=1
.venv-ros/bin/python -m pip install --upgrade pip 'setuptools>=71,<80' wheel
.venv-ros/bin/python -m pip install -r tools/requirements-ros-sdk.txt
.venv-ros/bin/python -m pip install --no-deps --no-build-isolation -e third_party/reBotArm_control_py
set +u
source tools/activate_ros.sh
set -u
python -c 'import rclpy, pinocchio, motorbridge; from reBotArm_control_py.actuator import RebotArm; print("ROS / SDK imports OK; no device opened")'
python -m colcon build --base-paths src/zekeep_msgs src/zekeepcontroller \
  src/zekeep_bringup src/zekeep_joystick src/zekeep_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
echo 'ROS 数采依赖构建完成；没有启动驱动或使能电机。'
