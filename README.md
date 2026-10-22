# Zekeeparm

六轴机械臂运行包，支持 ROS 真机控制、MoveIt、视觉抓取、拖动示教和主从遥操作，另提供可选的 Studio 积木编程与 π0.5 VLA。

**目标平台：** Ubuntu 22.04 x86_64 · ROS 2 Humble

[项目组成](#项目组成) · [一键安装](#一键安装) · [GPU 配置](#nvidia-驱动和-cuda-128) · [运行入口](#运行入口) · [标定边界](#标定边界) · [许可与安全](#许可与安全)

## 项目组成

| 模块 | 功能 |
| --- | --- |
| `zekeep_bringup` | 模型、真机、RViz 和网页启动 |
| `zekeepcontroller` / `zekeep_msgs` | 硬件控制、状态机与 ROS 接口 |
| `zekeep_moveit_config` | MoveIt 运动规划 |
| `zekeep_teach` | 重力补偿拖动示教与轨迹复现 |
| `zekeep_shadow` / `zekeep_llm` | 网页视觉抓取、LLM/MCP 计划与确认执行 |
| `zekeep_joystick` | 关节与末端手柄控制 |
| `zekeep_grasp` | 相机、标定、检测与 SDK 抓取 |
| `zekeep_teleop` | LeRobot 主从遥操作 |
| `zekeeparm_SDK` | 底层 SDK，DM 硬件配置 |
| `zekeep_embodied`（选装） | π0.5 数据采集、训练与监督执行 |
| `apps/studio`（选装） | 独立积木编程工作台 |

## 一键安装

### 1. 获取源码

```bash
git clone --branch 1022 https://github.com/Leocia/Zekeeparm.git "$HOME/Desktop/Zekeeparm"
cd "$HOME/Desktop/Zekeeparm"
```

已有源码时只需进入项目目录，跳过克隆。GitHub `1022` 分支不含 `apps/`，Studio 需另行取得。
迁移时复制源码，在新电脑重建环境；不要沿用旧电脑的 `build/`、`install/`、`.venv*` 或 CUDA 编译产物。
数据、模型、密钥和硬件标定须单独核对迁移。

### 2. 配置 Conda 路径

已有 Miniconda、Miniforge 或 Anaconda 时，在当前终端指定根目录：

```bash
export ZKEEP_MINIFORGE_DIR="$(conda info --base)"
```

未安装 Conda 可直接进入下一步，脚本会自动安装 Miniforge 到 `~/miniforge3`。
自定义目录时设置 `ZKEEP_MINIFORGE_DIR=/实际路径`；后续新终端也须保留该设置。

### 3. 执行安装

使用未激活其他 Conda/ROS 工作区的新终端；需要 GraspNet 时，先完成[GPU 配置](#nvidia-驱动和-cuda-128)。

```bash
bash tools/env.sh base bash setup.sh
```

脚本需联网并调用 `sudo apt`，自动安装 ROS/SDK、视觉与遥操作环境，构建工作区并下载基础视觉权重。
**不会安装 NVIDIA 驱动、CUDA、VLA 或 Studio，也不会连接硬件或使能电机。**

GraspNet 需阅读并接受其非商业研究许可，在提示时输入 `YES` 才会下载源码和权重。
具备 CUDA 12.8 与可用 GPU 时编译扩展；未接受许可或缺少 GPU/Toolkit 时，GraspNet 抓取不可用，其他基础功能仍可安装。
补齐 GPU 环境后可重跑安装，不能直接复制旧电脑的扩展。

安装结束会做无硬件导入检查，日志保存在 `log/setup-*.log`。重新登录使串口权限生效，并拔插一次相机。

### 各环境的边界

| 用途 | 环境 | Python | 进入命令 |
| --- | --- | --- | --- |
| ROS / SDK | `.venv-ros` | 3.10 | `bash tools/env.sh ros` |
| 视觉抓取 | Conda `rebotarm` | 3.10 | `bash tools/env.sh vision` |
| 主从遥操作 | Conda `lerobot` | 3.12 | `bash tools/env.sh teleop` |
| VLA 采集（选装） | Conda `embodied` | 3.10 | `bash tools/env.sh vla` |
| VLA 模型（选装） | OpenPI `.venv` | 3.11 | `bash tools/env.sh model` |

从项目根目录执行；命令会进入独立子终端，输入 `exit` 返回。不要 `source tools/env.sh`。
环境依赖保持隔离，不要混装；也可在环境名后直接附加命令：

```bash
bash tools/env.sh ros python -B tools/check_sdk.py
bash tools/env.sh vision python -m pip check
```

安装检查通过不等于相机、标定或真机已验收。更多用法见 `bash tools/env.sh --help`。

## NVIDIA 驱动和 CUDA 12.8

使用 GPU 视觉、GraspNet 或 VLA 模型计算时配置；已有可用驱动可跳过驱动安装。

### 1. 安装驱动

以下最后一条命令会重启电脑：

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

重启后执行 `nvidia-smi` 确认驱动正常。

### 2. 安装 Toolkit

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
nvcc --version
```

返回项目目录执行基础安装；新终端使用 CUDA 时也需设置 `CUDA_HOME` 和 `PATH`。

## 运行入口

### ROS、RViz 与网页

每个 ROS 终端先进入环境：

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh ros
```

按需选择一个入口：

```bash
# 无硬件 MoveIt + RViz
ros2 launch zekeep_moveit_config demo.launch.py

# 网页、rosbridge 与只读 AI 后端，不启动实机驱动或相机
ros2 launch zekeep_bringup web.launch.py

# 网页统一启动器：只读预览
./tools/start_web_sim.sh --preview
```

网页地址：`http://127.0.0.1:3001`。
统一启动器去掉 `--preview` 会启动真机与视觉组件，不自动使能或抓取；AI 运动还需 `--ai-motion` 和网页人工确认。
启动与安全退出见[网页说明](src/zekeep_bringup/web/README.md)。

### 视觉抓取、示教与遥操作

| 功能 | 使用说明 |
| --- | --- |
| 独立视觉抓取 | [相机、检测与 SDK 抓取](src/zekeep_grasp/README.md) |
| 拖动示教 | [记录与轨迹复现](src/zekeep_teach/README.md) |
| 手柄控制 | [关节与末端控制](src/zekeep_joystick/README.md) |
| 主从遥操作 | [串口、零位检查与启动](src/zekeep_teleop/README.md) |
| 真机调试 | [关节小幅运动测试](docs/JOINT_SMALL_MOTION_TEST.md) |

独立 SDK 抓取、遥操作与 ROS 驱动不能同时占用同一机械臂串口。
SDK 抓取会连接机械臂并执行准备位运动；主从遥操作和 `record-leader` 会使能从臂，退出会释放力矩，退出前须扶稳机械臂。

### 可选功能

**VLA 独立选装，不随 `setup.sh` 安装。**

- **π0.5 VLA：** [依赖与权重安装](docs/PI05_INSTALL.md) · [采集、训练与执行](src/zekeep_embodied/docs/pi05-entry.md)。需自行准备任务数据；8 GB 显存 NF4 QLoRA 为实验实现，不保证任务效果。执行器保留 `MOVE` 确认与 deadman，但没有环境碰撞或自碰撞检查。
## 标定边界

随包标定来自原设备，不能直接视为新设备已验证。设备、安装位置或场景变化后必须重新标定。
设备与安装完全未变时，先如实核对 `src/zekeep_grasp/config/default.yaml` 中的
`camera.serial`、`robot.id` 和 `calibration.installation_id`，再执行身份确认：

```bash
bash tools/env.sh vision python src/zekeep_grasp/scripts/confirm_calibration.py \
  --config src/zekeep_grasp/config/default.yaml --confirm-same-installation
```

该命令只打开相机，不连接或使能机械臂。完整步骤见[标定说明](src/zekeep_grasp/README.md#手眼标定文件与参数修改位置)。
夹爪映射基准为 `1.45 rad = 70 mm`，目标限制 `0–1.35 rad`，最大开口约 `65.17 mm`。
ROS 与 SDK 模型保持一致，保留 `official_tcp` 兼容坐标系；修改模型或硬件后须重新核验标定。

## 许可与安全

本工作区仅供本人或同一机构内部非商业研究，不得用于商业用途或向第三方转让、分发。
GraspNet 源码与权重另受其许可限制；SDK、OpenPI、模型及其他依赖按各自适用条款使用。

真机运动前检查急停、串口独占、工作空间、标定与速度，按对应模块说明操作。
安装与模拟检查不能代替实机验收。更新记录见 [CHANGELOG.md](CHANGELOG.md)。
