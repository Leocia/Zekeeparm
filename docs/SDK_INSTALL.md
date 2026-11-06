# Zekeep reBotArm Python SDK 安装

工作区随附 SDK 源码：`zekeeparm_SDK`。控制接口由 Python 提供；Pinocchio 等依赖包含 C++ 原生库。

## 功能特性

- 电机连接、状态读取和控制循环。
- 关节正逆运动学。
- 基于 Pinocchio 的动力学计算与重力补偿。
- 末端控制和轨迹规划工具。
- YAML 硬件配置。

## 环境安装

本包的 ROS 驱动和视觉抓取使用独立 Python 环境；完整部署时，SDK 分别关联到两者中。
这是为了隔离 ROS 与相机、OpenCV、PyTorch 依赖，SDK 本身不要求两套环境。只使用一类功能时，准备并安装对应环境即可。
先设置工作区路径：

```bash
export ZKEEP_WS="/实际工作区路径"
cd "$ZKEEP_WS"
```

SDK 环境统一使用 **Python 3.10、NumPy 1.26.4、Pinocchio 3.9.0、MotorBridge 0.5.1**。
先由 Conda 安装 NumPy、Pinocchio 和配套原生库，再安装 pip 依赖清单，最后以 `--no-deps` 安装本地 SDK。
不要把 PyPI 的 Pinocchio 3.9 wheel 与 NumPy 1.26.4 强行拼装；该 wheel 的原生依赖会要求 NumPy 2.x。本包使用 Conda 的配套构建。
`--no-deps` 会跳过全部依赖安装，不能仅运行该条命令就认为 SDK 依赖齐全。
SDK 源码始终位于项目的 `zekeeparm_SDK/`；editable 安装只把源码关联到所选 Python 环境。
MotorBridge、NumPy、Pinocchio 等依赖分别装在 ROS 和视觉环境中；两套环境虽使用相同 NumPy 版本，依赖仍独立管理。

### MotorBridge 分环境安装

SDK 的 ROS 与视觉环境均使用 **`motorbridge==0.5.1`**。`bash setup.sh ros` 与 `bash setup.sh vision` 分别包含各自环境的此依赖；分步安装或补装时，在环境创建完成后分别执行：

```bash
# ROS SDK 环境
bash tools/env.sh ros python -m pip install motorbridge==0.5.1
bash tools/env.sh ros python -c 'from importlib.metadata import version; from motorbridge import Controller, Mode, CallError; print("MotorBridge:", version("motorbridge"))'

# 视觉 Conda 环境
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt motorbridge==0.5.1
bash tools/env.sh vision python -c 'from importlib.metadata import version; from motorbridge import Controller, Mode, CallError; print("MotorBridge:", version("motorbridge"))'
```

这些检查只导入模块，不创建 Controller、不连接串口。安装命令会把依赖放入所选环境，不使用 `sudo pip` 或用户全局目录。

### ROS 驱动环境

目标为 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。先完成 ROS 系统依赖安装和手动 Conda 准备。
ROS 环境现在由 Conda 管理，目录名沿用 `.venv-ros`，由 `tools/activate_ros.sh` 同时加载 ROS 和 Conda。
colcon、EmPy、Lark 等构建工具也装入该环境；不把系统 `/usr/lib/python3/dist-packages` 整体加入，避免混入另一套 Python 原生包。
首次创建时执行：

```bash
cd "$ZKEEP_WS"
source tools/environment.sh
ros_env_dir="${ZKEEP_ROS_VENV:-$ZKEEP_WS/.venv-ros}"
"$ZKEEP_MINIFORGE_DIR/bin/conda" env create -p "$ros_env_dir" -f tools/environment-ros.yml
source tools/activate_ros.sh
python -m pip install -r tools/requirements-ros-sdk.txt
# 上面的清单已包含 motorbridge==0.5.1。
python -m pip install --no-deps -e zekeeparm_SDK
source tools/activate_ros.sh
```

环境已由本包创建时，`ros` 模块安装器会使用同一清单更新；无需重复执行 `env create`。
后续 ROS 构建和运行都使用 `source tools/activate_ros.sh` 或 `bash tools/env.sh ros ...`。

### 视觉抓取环境

先按[可迁移安装说明](INSTALL_PORTABLE.md)创建 `rebotarm` 环境并完成视觉依赖安装，再执行：

```bash
cd "$ZKEEP_WS"
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt motorbridge==0.5.1
bash tools/env.sh vision python -m pip install --no-deps -e ./zekeeparm_SDK
```

ROS SDK 和视觉 SDK 都固定 NumPy 1.26.4。视觉环境由 `src/zekeep_grasp/environment.yml` 创建，并由 `tools/constraints-vision.txt` 约束后续 pip 安装。

### 旧包环境迁移

旧包的 `.venv-ros` 由 Python venv 创建，安装清单包含 NumPy 2.2.6 和 PyPI Pinocchio。
不要在该环境中只执行 `pip install numpy==1.26.4`；这样会留下原生库或依赖约束不一致的组合。
关闭使用该环境的进程，在项目根目录将旧环境改名备份，再运行新版 ROS 模块安装：

```bash
ros_env_dir="${ZKEEP_ROS_VENV:-$PWD/.venv-ros}"
# 仅在确认这是需要迁移的旧 venv 时执行；自定义目录同样使用上面的变量。
mv -- "$ros_env_dir" "${ros_env_dir}.backup-$(date +%Y%m%d-%H%M%S)"
bash tools/env.sh base bash setup.sh ros
```

安装器会检查目录类型，遇到旧 venv 时直接停止，不删除或覆盖旧环境。
新环境安装完成后，分别验证 ROS 与视觉中的 NumPy、Pinocchio、MotorBridge 版本，再运行无设备 SDK 检查。

### 独立遥操作环境

`zekeep_teleop` 和 `zekeep_embodied` 不安装 reBotArm SDK。它们使用独立的 `motorbridge==0.4.9`；ROS 与视觉环境使用 `motorbridge==0.5.1`。
遥操作和数采的安装入口已分别声明 0.4.9，不要用 SDK 的 0.5.1 覆盖这两个环境。

## 安装验证

在目标环境激活后运行对应导入检查。命令只导入 SDK，不连接硬件：

ROS 环境：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
python -c '
import rclpy, pinocchio, motorbridge, numpy
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
from zekeeparm_SDK.dynamics import compute_generalized_gravity
from zekeeparm_SDK.kinematics import compute_fk
'
```

视觉环境：

```bash
conda activate rebotarm
python -c '
import pinocchio, motorbridge, numpy
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
from zekeeparm_SDK.dynamics import compute_generalized_gravity
from zekeeparm_SDK.kinematics import compute_fk
'
```

确认 Python 路径和安装版本：

```bash
python -c 'import sys; print(sys.executable)'
python -m pip show zekeeparm-sdk motorbridge pin
```

## 项目使用

- ROS 真机驱动由 `zekeepcontroller` 调用 SDK；安装和启动步骤见[控制器说明](../src/zekeepcontroller/README.md)。
- 视觉抓取直接调用 SDK 并独占机械臂串口；用法见[视觉抓取说明](../src/zekeep_grasp/README.md)。
- ROS 驱动和直接 SDK 抓取不可同时连接同一机械臂。
- 串口权限使用 `dialout` 组，不要用 `chmod 777` 放宽设备权限。

## SDK 目录

```text
zekeeparm_SDK/
├── config/                  # SDK 硬件配置
└── zekeeparm_SDK/
    ├── actuator/            # 电机连接与控制循环
    ├── controllers/         # 末端控制器
    ├── dynamics/            # 动力学计算
    ├── kinematics/          # 正逆运动学
    └── trajectory/          # 轨迹工具
```
