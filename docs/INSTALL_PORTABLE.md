# Ubuntu 22.04 新机安装

本包目标为 Ubuntu 22.04 x86_64、ROS 2 Humble。仅交付源码；首次安装、模型获取需要网络。
安装完成并缓存所需模型后，本机网页、ROS 演示和本地视觉可以离线运行；远程 LLM 依赖用户配置的服务。

## 环境隔离

| 环境 | Python | 用途 |
| --- | --- | --- |
| `.venv-ros` | 3.10，system-site-packages | ROS、SDK、Pinocchio |
| Conda `rebotarm` | 3.10 | Orbbec、YOLOE、可选 GraspNet |
| Conda `lerobot` | 3.12 | 独立主从遥操作 |
| Conda `embodied`（选装） | 3.10 | 共用数采、导出、ROS 策略执行端 |
| Conda `act`（选装） | 3.10 | ACT 与 π0.5 模型 |
| `web_mujoco/.venv`（选装） | 系统 Python | 独立网页仿真的模型导出 |

不要复制旧电脑的环境、build/install/log、CUDA `.so` 或 node_modules。可复制项目到任意路径再重建。
已有 Conda 时每个新终端先设置 `ZKEEP_MINIFORGE_DIR="$(conda info --base)"`。
`ZKEEP_ROS_VENV`、`ZKEEP_VISION_PYTHON`、`EMBODIED_PYTHON`、`LEROBOT_PYTHON` 可覆盖对应解释器。

## NVIDIA 驱动与 CUDA

安装受系统支持的 NVIDIA 驱动，重启后确认 `nvidia-smi` 正常。
PyTorch 使用 CUDA 12.8 wheel；GraspNet 本地扩展还需要 CUDA Toolkit 12.8、GCC/G++ 11。
不需要 GraspNet 时可先安装基础功能。

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-8
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
nvcc --version
```

驱动、Toolkit 和 GPU 必须兼容；不同 GPU 架构需重新编译原生扩展。不要复用旧电脑编译结果。
以上命令来源：[NVIDIA CUDA 安装说明](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)。

## 基础安装

回到项目根目录，在未激活其他 ROS/Conda 工作区的新终端运行：

```bash
bash tools/env.sh base bash setup.sh
```

安装器准备 ROS、串口权限、SDK、八个 ROS 包、视觉及遥操作依赖。
ROS apt 源引导包固定为 [ros-apt-source 1.3.0](https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0)，
直接下载 Jammy 安装包，不依赖 GitHub API 查询最新版本。
GraspNet 安装需自行阅读并接受上游许可；输入 YES 安装该可选依赖，不会自动接受许可。
接受许可后，安装器先安装 `Cython==3.0.12`，再构建 GraspNet API 的依赖。
基础权重随后执行 `bash tools/download_models.sh base` 按需获取。
GraspNet 权重另执行 `ACCEPT_GRASPNET_LICENSE=YES bash tools/download_models.sh graspnet`。
SDK 配置保留 DM 型号和交付接口，未导入源包的旧全局配置文件。

安装日志保存在 `log/setup-*.log`；重新登录使 `dialout` 生效，相机按提示拔插。
脚本只做无硬件导入检查。安装成功不代表新设备零位、手眼、桌面或运动已验收。
ROS venv 与视觉 Conda 使用不同 NumPy ABI，不要混装依赖。

## 新设备初始化

按根目录 [README](../README.md#新设备配置与标定)填写设备身份、读取新相机内参并核验安装。
`camera.serial`、设备 ID、安装标识均为空模板；确认工具不能代替新安装的实际标定。
手眼、颜色文件来自原设备，按交付要求保留。旧设备内参、身份快照和个人数据不随新包分发。
运动前核验 `zekeep_hardware.yaml` 的零位、方向、控制增益和现场几何，再显式打开所需授权。
保持启动不自动使能，不绕过序列号、内参、反馈新鲜度或设备占用检查。

选装数采、ACT、π0.5 和仿真见 [OPTIONAL_MODULES.md](OPTIONAL_MODULES.md)。
