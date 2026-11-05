# Ubuntu 22.04 新机安装

本包目标为 Ubuntu 22.04 x86_64、ROS 2 Humble。仅交付源码；首次安装、模型获取需要网络。
安装完成并缓存所需模型后，本机网页、ROS 演示和本地视觉可以离线运行；远程 LLM 依赖用户配置的服务。

## 环境隔离

| 环境 | Python | 用途 |
| --- | --- | --- |
| `.venv-ros`（Conda 管理，沿用原目录名） | 3.10，NumPy 1.26.4 | ROS、SDK、Pinocchio 3.9.0 |
| Conda `rebotarm` | 3.10 | Orbbec、YOLOE、可选 GraspNet |
| Conda `lerobot` | 3.12 | 独立主从遥操作 |
| Conda `embodied`（选装） | 3.10 | 共用数采、导出、ROS 策略执行端 |
| Conda `act`（选装） | 3.10 | ACT 与 π0.5 模型 |
| `web_mujoco/.venv`（选装） | 系统 Python | 独立网页仿真的模型导出 |

不要复制旧电脑的环境、build/install/log、CUDA `.so` 或 node_modules。可复制项目到任意路径再重建。
先按根目录 [README 的首次安装步骤](../README.md#首次安装)手动下载安装 Miniforge；已有 Conda 可直接复用。
在 `tools/environment.sh` 配置 `ZKEEP_MINIFORGE_DIR`，安装器和项目启动脚本统一读取该文件。
默认 `~/miniforge3`；已有 Miniconda 等安装时，改为 `conda info --base` 显示的实际根目录。
终端已导出的同名变量优先；修改 Conda 根目录后建议打开新终端。
若在当前终端切换，先执行 `unset ZKEEP_MINIFORGE_DIR ZKEEP_VISION_PYTHON EMBODIED_PYTHON`，再运行项目入口，避免保留旧的派生路径。
无需 `conda init`；安装器不负责 Miniforge 下载或安装，缺少可执行的 `bin/conda` 时提前退出。
`ZKEEP_ROS_VENV`、`ZKEEP_VISION_PYTHON`、`EMBODIED_PYTHON`、`LEROBOT_PYTHON` 可覆盖对应解释器。
`ZKEEP_ROS_VENV` 同时指定 ROS 环境的安装和运行目录；基础安装与 `sim` 选装均使用该路径。
ROS SDK 先按 `tools/environment-ros.yml` 安装 Conda 的 NumPy 1.26.4 和 Pinocchio 3.9.0，再安装 pip 清单及本地 SDK。
该顺序避免 PyPI Pinocchio 的原生依赖把 NumPy 拉到 2.x；旧包的 Python venv 须先按 [SDK 迁移说明](SDK_INSTALL.md#旧包环境迁移)备份，不能直接在原目录更新。
视觉 Conda 清单只准备 Python、NumPy、Pinocchio 等基础库；随后单独安装固定的 cu128 PyTorch wheel，再按 `tools/requirements-vision.txt` 安装视觉依赖。
pip 步骤直接显示下载和错误输出，不再隐藏在 Conda 的批量 pip 阶段中；后续 pip 安装均使用 `tools/constraints-vision.txt` 保护关键版本。
Ultralytics 从批量环境清单中拆出，待视觉基础依赖就绪后单独安装固定版 8.4.35，并立即检查 `YOLO`、`YOLOE` 导入。

## NVIDIA 驱动与 CUDA

这两项从项目安装中独立出来，按 [GPU 环境单独安装说明](GPU_SETUP.md)手动准备。
顺序为 NVIDIA 驱动、重启验证、CUDA Toolkit 12.8.1、配置 `tools/environment.sh` 的 `CUDA_HOME`，再运行基础安装器。
驱动建议基线为 570.124.06 或更新且支持实际 GPU 的版本；Toolkit 默认路径为 `/usr/local/cuda-12.8`。
安装器不下载、安装或升级驱动和 Toolkit。PyTorch cu128 wheel 仍作为 Python 依赖安装。
GraspNet 本地扩展需要 Toolkit 12.8、GCC/G++ 11 和可见 GPU；不使用该扩展时可先安装基础功能。

## 基础安装

回到项目根目录，在未激活其他 ROS/Conda 工作区的新终端运行：

```bash
bash tools/env.sh base bash setup.sh
```

安装器准备 ROS、串口权限、SDK、八个 ROS 包、视觉及遥操作依赖。
`rosdep update` 已注释；不自动执行 `rosdep init` 或 `rosdep install`，避免新机因网络或空缓存中断。
基础 ROS 依赖由脚本中的 apt 列表安装，SDK 的 Pinocchio 在隔离 Python 环境内安装。
视觉环境使用 `conda-forge` 和 `nodefaults`；遥操作与数采/模型环境创建时也显式指定 `conda-forge`，不修改用户全局源配置。
`nodefaults` 的含义见 [Conda 环境文件说明](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html#creating-an-environment-file-manually)。
ROS apt 源引导包固定为 [ros-apt-source 1.3.0](https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0)，
直接下载 Jammy 安装包，不依赖 GitHub API 查询最新版本。
GraspNet 已移出基础安装器；源码、API、CUDA 扩展、权重及后端验证均按 [独立安装步骤](../third_party/README.md)完成。
第三方目录为 `third_party/`；原始包只保留该目录的 README，使用者从官方 GitHub 原址手动下载源码并放入指定目录，地址和路径见该说明。
独立安装时先安装 `Cython==3.0.12`，再安装 GraspNet API 并构建目标 GPU 的扩展，不必重跑基础安装器。
YOLOE-26L 与 MobileCLIP 基础权重默认由安装器下载并校验，再运行 `tools/check_yolo.py` 做模型加载和 CPU 推理检查。
若显式设置 `ZKEEP_DOWNLOAD_MODELS=0`，模型仍未就绪；补装步骤、文件位置及检查命令见 [README](../README.md#ultralytics-与-yolo26-模型)。
GraspNet 权重从 baseline 官方 README 的原始链接单独下载，保存为 `third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar`，不解压。
SDK 配置保留 DM 型号和交付接口，未导入源包的旧全局配置文件。

安装日志保存在 `log/setup-*.log`；重新登录使 `dialout` 生效，相机按提示拔插。
脚本只做无硬件导入检查。安装成功不代表新设备零位、手眼、桌面或运动已验收。
ROS SDK 和视觉 SDK 均固定 NumPy 1.26.4，但仍分别安装各自依赖；数采、模型和遥操作环境不安装本 SDK，不要混装。

## 新设备初始化

按根目录 [README](../README.md#新设备配置与标定)填写设备身份、读取新相机内参并核验安装。
`camera.serial`、设备 ID、安装标识均为空模板；确认工具不能代替新安装的实际标定。
手眼、颜色文件来自原设备，按交付要求保留。旧设备内参、身份快照和个人数据不随新包分发。
运动前核验 `zekeep_hardware.yaml` 的零位、方向、控制增益和现场几何，再显式打开所需授权。
保持启动不自动使能，不绕过序列号、内参、反馈新鲜度或设备占用检查。

选装数采、ACT、π0.5 和仿真见 [OPTIONAL_MODULES.md](OPTIONAL_MODULES.md)。
