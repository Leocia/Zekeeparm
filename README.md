# Zekeeparm

六轴机械臂运行包，包含 ROS 真机控制、MoveIt、视觉抓取、示教、主从遥操作、
独立积木编程 Studio，以及可选的 π0.5 数据采集、训练和监督执行。

**ROS 目标平台：Ubuntu 22.04 x86_64、ROS 2 Humble。** ROS/视觉/采集使用 Python 3.10，
遥操作使用 Python 3.12，OpenPI 使用独立 Python 3.11。Studio 的 Windows 入口与限制另见其说明。

[快速路线](#场景化快速路线) · [获取源码](#获取源码与迁移范围) · [GPU 准备](#nvidia-驱动和-cuda-128) ·
[基础安装](#一键安装) · [下载清单](#下载资源清单) ·
[VLA 可选安装](#π05-vla-入口) · [Studio 可选安装](#studio-可选安装) ·
[安装验收](#安装验收) · [运行](#运行入口) · [标定](#标定边界)

`setup.sh` 负责基础功能安装；
**VLA 和 Studio 独立选装，不随 `bash setup.sh` 自动安装或下载。**

## 场景化快速路线

先按用途选路线，再执行对应章节。表中命令用于**安装完成后**进入或检查环境，不会代替安装。
当前 `setup.sh` 统一安装基础 ROS、视觉和遥操作依赖；VLA 与 Studio 独立选装。

| 你要做什么 | 安装路线 | 安装后的入口 |
| --- | --- | --- |
| 只用积木编程和模拟设备（需另行取得 `apps/studio`） | [Studio 选装](#studio-可选安装)，无需整套 ROS 或模型权重 | `bash tools/env.sh studio python apps/studio/studio.py` |
| 先看机械臂模型，不连接硬件 | [基础安装](#一键安装)，再看 [ROS 运行](#运行入口) | `bash tools/env.sh ros ros2 launch zekeep_moveit_config demo.launch.py` |
| 真机控制、手柄或拖动示教 | [基础安装](#一键安装)、[标定边界](#标定边界)、[手柄](src/zekeep_joystick/README.md)或[示教](src/zekeep_teach/README.md) | `bash tools/env.sh ros` |
| RGB-D 视觉与 GraspNet 抓取 | [GPU 准备](#nvidia-驱动和-cuda-128)、基础安装时选择 GraspNet、[标定](#标定边界) | `bash tools/env.sh vision` |


从项目根目录执行上表命令；省略命令会进入子终端，输入 `exit` 返回。
真机控制和主从遥操作须按对应说明检查现场；选环境本身不连接设备或使能电机。

## 项目组成

| 模块 | 功能 |
| --- | --- |
| `zekeep_bringup` | 模型、硬件参数、真机、RViz 和网页启动 |
| `zekeepcontroller` | 硬件抽象、状态机、ROS 服务和轨迹 Action |
| `zekeep_msgs` | 自定义消息、服务和 Action |
| `zekeep_moveit_config` | MoveIt 配置 |
| `zekeep_teach` | 重力补偿拖动示教、网页任务和轨迹复现 |
| `zekeep_shadow` | 网页 RGB-D 检测及 YOLO/GraspNet 抓取后端 |
| `zekeep_llm` | 网页 LLM/MCP 计划、人工确认执行和取消 |
| `zekeep_joystick` | 六轴关节及末端手柄控制 |
| `zekeep_grasp` | 相机、当前标定、检测和抓取 |
| `zekeep_teleop` | 独立 LeRobot 主从遥操作 |
| `zekeep_embodied` | π0.5 采集、导出、训练、预测与受监督运行 |
| `zekeeparm_SDK` | 机械臂 SDK，仅保留 DM 硬件配置 |

## 获取源码与迁移范围

可复制提供的源码包，或在首次获取时克隆仓库：

```bash
sudo apt update
sudo apt install -y git curl wget ca-certificates
mkdir -p "$HOME/Desktop"
git clone --branch 1022 https://github.com/Leocia/Zekeeparm.git "$HOME/Desktop/Zekeeparm"
```

已复制或已克隆项目时跳过 `git clone`，不覆盖原目录。
本说明对应 GitHub 的 `1022` 分支，包含 `src/zekeep_embodied`、`tools/pi05.sh` 和转换补丁。
本地完整包另有独立 `apps/` 组件，GitHub 分支按本次发布范围排除该目录。

后文统一从实际包目录进入准备子终端，脚本自动设置工作区和各解释器路径。路径不同时只改 `cd`：

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh base
```

准备子终端无需已安装 Miniforge；后续安装命令在其中执行，完成后输入 `exit` 返回。
自定义 Miniforge 目录时，可一次指定 `ZKEEP_MINIFORGE_DIR=/实际路径 bash tools/env.sh base`。

保留 `src/`、`zekeeparm_SDK/`、`tools/`、`docs/`、`setup.sh` 和需要的 `apps/studio/` 目录结构。
迁移源码时排除旧电脑的 `build/`、`install/`、`log/`、`Log/`、`.venv*`、
`node_modules/`、Python 缓存和已编译 CUDA 扩展。旧桌面工作区的 `.venv-ros` 是指向源电脑的符号链接，
不能作为新电脑环境使用；应只复制源码，在新电脑重新创建和构建。

模型、OpenPI、虚拟环境、数据、checkpoint 和密钥不随本次 VLA 同步提供。
已有示范、用户项目和硬件标定需单独核对和迁移，不能用重新下载模型代替。

## NVIDIA 驱动和 CUDA 12.8

需要 GPU 视觉、GraspNet 或 VLA 模型计算时，先在新电脑安装驱动。以下命令会重启电脑：

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

重启后确认驱动：

```bash
nvidia-smi
```

安装 CUDA Toolkit 12.8，不使用 `cuda` 元包，避免由该命令主动安装驱动：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
```

重启后在当前终端设置 Toolkit 路径，再执行下一节的基础安装：

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
nvcc --version
```

## 一键安装

先在**未激活其他 Conda/ROS 工作区的新终端**设置上面的工作区路径。
安装需联网并使用 `sudo apt`；不会安装 NVIDIA 驱动或 CUDA Toolkit，也不会连接设备或使能电机。
需要 GraspNet 时先完成上面的 GPU 准备，避免扩展编译被跳过。

```bash
bash tools/env.sh base bash setup.sh
```

该脚本安装基础功能，并在结束前检查关键导入；VLA、Studio 不在安装范围内：

| 内容 | 安装行为 |
| --- | --- |
| ROS 与构建工具 | 配置 ROS apt 源，安装 Humble、MoveIt、Joy、控制器、rosbridge 和编译依赖 |
| ROS/SDK Python | 创建 `.venv-ros`，安装 `tools/requirements-ros-sdk.txt` 与本地 SDK，构建 ROS 包 |
| Miniforge | 缺少时下载安装器并校验 SHA-256；使用明确的安装路径 |
| 视觉环境 | 创建/更新 `rebotarm`，安装 Orbbec SDK、OpenCV、Pinocchio、Torch cu128、CLIP 和本地 SDK |
| 视觉模型 | 下载并校验 YOLOE、MobileCLIP2 |
| GraspNet | 用户接受许可后才下载源码、API、权重；具备 Toolkit/GPU 时编译扩展 |
| 主从遥操作 | 创建独立 Python 3.12 `lerobot` 环境，安装本地遥操作插件 |
| 设备权限 | 当前用户加入 `dialout`，安装 Orbbec SDK 的 udev 规则；重新登录并拔插相机后生效 |
| 安装检查与日志 | 检查 SDK、视觉和遥操作导入；记录 `log/setup-*.log`，失败时保留日志并退出 |

基础依赖清单分别是 [ROS/SDK](tools/requirements-ros-sdk.txt)、
[视觉 Conda](src/zekeep_grasp/environment.yml)、[视觉约束](tools/constraints-vision.txt) 和
[遥操作](src/zekeep_teleop/pyproject.toml)。SDK 已随包提供，不需要另找旧版 SDK。
安装中断后可重跑；下载文件会校验并复用，但 pip/Conda 安装和构建仍会执行。
每次安装自动保留 `log/setup-*.log`；报错会显示失败行号和日志位置。
脚本在 ROS 与 Conda 阶段之间清理外部 Python/原生库路径，视觉安装全程沿用同一版本约束。
**无硬件导入检查通过不表示相机身份或真机已验收，须继续执行安装验收。**
GraspNet CUDA 扩展是否安装会在末尾单独显示，缺少 Toolkit/GPU 时明确报告跳过。

### GraspNet 下载和编译

脚本提示时，接受其本人或同机构内部非商业研究限制后输入 `YES`。
接受后才下载 `graspnet-baseline`、`graspnetAPI`、`checkpoint-rs.tar` 并尝试编译 `pointnet2`/`knn`。
未接受时跳过这些步骤，ROS、示教和独立遥操作仍可安装；依赖 GraspNet 的抓取后端不可用。
已阅读并接受限制后，也可通过 `ACCEPT_GRASPNET_LICENSE=YES bash setup.sh` 明确选择安装。

安装时若报告跳过 CUDA 扩展，完成 GPU/Toolkit 准备后，从新终端重跑 `bash setup.sh`，
再次按实际意愿选择 GraspNet。不能复制旧电脑的 `.so` 代替在目标 GPU 上编译。

### 各环境的边界

| 用途 | 默认位置 | Python | 关键约束 |
| --- | --- | --- | --- |
| ROS / SDK | `$ZKEEP_WS/.venv-ros` | 3.10 | NumPy 2.2.6、motorbridge 0.5.1 |
| 视觉 / Orbbec | `$ZKEEP_MINIFORGE_DIR/envs/rebotarm` | 3.10 | NumPy 1.26.4、Torch 2.7.1 cu128、motorbridge 0.5.1 |
| 主从遥操作 | `$ZKEEP_MINIFORGE_DIR/envs/lerobot` | 3.12 | LeRobot 0.4.4、motorbridge 0.4.9 |
| VLA 采集（选装） | `$ZKEEP_MINIFORGE_DIR/envs/embodied` | 3.10 | NumPy 2.2.6、Torch 2.6.0、motorbridge 0.4.9 |
| VLA 模型（选装） | `$ZKEEP_WS/third_party/openpi/.venv` | 3.11 | NumPy 1、Torch 2.7.1 cu128、NF4 |
| Studio（选装） | `$ZKEEP_MINIFORGE_DIR/envs/zekeep-studio` | 3.10 | 独立 Pinocchio、motorbridge 0.5.1 |

统一使用 [tools/env.sh](tools/env.sh)，不再手动组合 `export`、`conda activate` 和 ROS overlay。
脚本自动定位工作区、设置解释器路径、清理旧 Python/ROS/原生库环境，并使用目标环境的可执行文件。
所有命令以工作区根目录为当前目录；父终端的变量、PATH 和工作目录不变。

```bash
# 一条命令执行完即返回原终端
bash tools/env.sh ros python -B tools/check_sdk.py
bash tools/env.sh vision python -m pip check
bash tools/env.sh vla ./tools/pi05.sh doctor

# 多步操作：进入对应子终端，完成后输入 exit
bash tools/env.sh teleop
```

| 包装环境 | 用途 |
| --- | --- |
| `base` | 首次安装/环境恢复；无需目标环境已安装，自动提供工作区和解释器变量 |
| `ros` | 复用 `tools/activate_ros.sh`，加载 ROS、工作区和项目 SDK venv |
| `vision` / `teleop` / `studio` | 在相应的 Miniforge 环境执行命令 |
| `vla` | 在 `embodied` 采集环境执行命令；`tools/pi05.sh` 继续自动选择各步骤的运行环境 |
| `model` | 使用 OpenPI venv，加载本地适配源码并设置数据、统计和 checkpoint 路径 |

用 `bash tools/env.sh --help` 查看用法。环境未安装时会报错并提示安装，不会自动下载。
包装脚本应通过 `bash` 执行，不能 `source`；子终端跳过用户 Bash 启动文件，避免自动加载其他工作区。
自定义 `ZKEEP_MINIFORGE_DIR`、`ZKEEP_VISION_PYTHON`、`EMBODIED_PYTHON` 或 `ZKEEP_ROS_VENV` 时，
在调用命令前设置对应变量即可；只用一次时可写成 `变量=值 bash tools/env.sh 环境 命令`。
`vision`、`vla` 包装使用 Conda，覆盖解释器时须指向 Conda 环境的 `bin/python`；普通 venv 不适用这两种包装。
`base` 使用系统工具 PATH；其他环境由对应的 Conda/venv 提供工具，`CUDA_HOME/bin` 会在配置后保留。

各环境依赖仍保持隔离；SDK 的 `--no-deps` 只用于先装好锁定清单后的特定安装步骤。

### Orbbec USB 权限（手动恢复）

`dialout` 只处理机械臂串口权限，不等于相机 USB 权限。更新后的基础脚本已安装 SDK 自带的 udev 规则。
安装后拔插相机；如果规则被改动，或使用了旧版安装脚本，可在视觉环境定位并手动恢复：

```bash
bash tools/env.sh vision
orbbec_rules_installer="$(python -c 'from importlib.util import find_spec; from pathlib import Path; print(Path(find_spec("pyorbbecsdk").origin).parent / "shared/install_udev_rules.sh")')"
test -f "$orbbec_rules_installer"
cat "$orbbec_rules_installer"
```

确认该脚本来自当前安装的 Orbbec SDK 后，手动执行：

```bash
sudo sh "$orbbec_rules_installer"
```

脚本会写入 `/etc/udev/rules.d/` 并重载规则；执行后拔插相机。
如果 SDK 已能正常访问相机，无需重复配置。此步骤不连接或控制机械臂。

## 下载资源清单

| 资源 | 获取方式 | 安装后位置 |
| --- | --- | --- |
| ROS、系统依赖 | `setup.sh` 使用 Ubuntu/ROS apt 源 | `/opt/ros/humble` 与系统目录 |
| Miniforge、Python 依赖 | `setup.sh` 从 Miniforge、Conda/PyPI/PyTorch 源获取 | 上述环境目录 |
| YOLOE | `setup.sh` 下载 `yoloe-26l-seg.pt` | `src/zekeep_grasp/models/` |
| MobileCLIP2 | `setup.sh` 下载 `mobileclip2_b.ts` | `src/zekeep_grasp/` |
| GraspNet 源码、API、权重 | 接受许可后由 `setup.sh` 下载 | `third_party/graspnet-baseline/` |
| OpenPI 及模型依赖 | 下方 VLA 手动选装步骤 | `third_party/openpi/` |
| π0.5 官方 JAX 基座 | VLA 官方下载器 | `~/.cache/openpi/openpi-assets/checkpoints/pi05_base/` |
| π0.5 tokenizer | VLA 官方下载器 | `~/.cache/openpi/big_vision/paligemma_tokenizer.model` |
| π0.5 PyTorch 基座 | 在新电脑转换 JAX 基座 | `~/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch/` |
| Studio Blockly 资源 | Studio 中执行 `npm ci` | `apps/studio` 的静态资源目录 |
| 本机任务适配器、统计和示范 | 自行采集、导出和训练，不由基础下载生成 | `data/` |

YOLOE、MobileCLIP2 和 GraspNet 的来源 URL/校验值以 [setup.sh](setup.sh) 为准，
校验清单见 [MODEL_SHA256SUMS](docs/MODEL_SHA256SUMS)。接受并安装全部视觉模型后，可运行：

```bash
cd "$ZKEEP_WS"
sha256sum -c docs/MODEL_SHA256SUMS
```

跳过 GraspNet 时，完整清单会因缺少该权重而失败，应只核对前两项：

```bash
cd "$ZKEEP_WS"
head -n 2 docs/MODEL_SHA256SUMS | sha256sum -c -
```

原始 π0.5 JAX 基座约 11.6 GiB，源电脑 OpenPI 环境约 9.2 GiB；
转换后的 PyTorch 基座、下载缓存、PNG 示范和训练产物另占空间。
源码同步很小，不代表完整安装也小。RAM/显存与磁盘占用不同，跨电脑的转换/训练峰值尚未验证。

## π0.5 VLA 入口

**可选安装，不修改或扩展 `setup.sh`。** 只做 ROS、示教或视觉抓取可跳过本节。
只采集 VLA 数据可安装采集环境而跳过模型；训练和模型预测需要完成全部步骤。
完成基础安装后，进入准备子终端安装可选依赖：

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh base
```

若工作区不在桌面，修改第一行。下面的首次安装步骤在该子终端内执行。
安装后，日常流程使用 `bash tools/env.sh vla`；直接操作模型环境使用 `bash tools/env.sh model`。
源电脑 8 GB 显存的 NF4 QLoRA 是实验实现，不保证其他电脑容量足够，也不保证任务效果。

### 1. 采集与数据处理依赖

只采集数据可完成本节后继续阅读下方的“新数据与训练”，不必下载模型。
首次创建环境；若该路径已存在环境，跳过 `conda create`：

```bash
"$ZKEEP_MINIFORGE_DIR/bin/conda" create \
  -p "$ZKEEP_MINIFORGE_DIR/envs/embodied" python=3.10 -y
"$EMBODIED_PYTHON" -m pip install --upgrade pip
"$EMBODIED_PYTHON" -m pip install \
  -r "$ZKEEP_WS/src/zekeep_embodied/requirements-embodied.txt" \
  -e "$ZKEEP_WS/src/zekeep_teleop" -e "$ZKEEP_WS/src/zekeep_embodied"
"$EMBODIED_PYTHON" -m pip check
```

固定版本包括 LeRobot 0.4.4、从臂插件 1.0.0、motorbridge 0.4.9、Torch 2.6.0、
TorchVision 0.21.0、TorchCodec 0.2.1，完整清单见上述 requirements 文件。
此 Torch 用于采集/数据导出，不承担 Blackwell 模型计算。Orbbec SDK 继续由
`setup.sh` 创建的独立 `rebotarm` 进程加载，不要装入采集或模型环境。

### 2. OpenPI 源码和模型依赖

固定上游 OpenPI 为 `215abfb217dbac7d5f1273282331b9b1866c0479`，
LeRobot v2.1 数据支持由它的锁文件固定。首次执行以下块；若已存在 OpenPI 目录，
先核对其版本和本地改动，不要强制覆盖。

```bash
"$EMBODIED_PYTHON" -m pip install uv
export UV="$ZKEEP_MINIFORGE_DIR/envs/embodied/bin/uv"
export UV_LINK_MODE=copy
mkdir -p "$ZKEEP_WS/third_party"
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Physical-Intelligence/openpi.git "$ZKEEP_WS/third_party/openpi"
cd "$ZKEEP_WS/third_party/openpi"
git checkout 215abfb217dbac7d5f1273282331b9b1866c0479
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
GIT_LFS_SKIP_SMUDGE=1 "$UV" sync --frozen --python 3.11
```

本包保留了源电脑用于减少权重转换内存占用的本地补丁。只在尚未应用时执行：

```bash
git apply --check "$ZKEEP_WS/tools/patches/openpi-conversion-memory.patch"
git apply "$ZKEEP_WS/tools/patches/openpi-conversion-memory.patch"
```

安装 CUDA 12.8 版模型计算依赖，保持上游 Torch 版本号，并安装 NF4 依赖：

```bash
"$UV" pip install --python .venv/bin/python \
  'torch==2.7.1+cu128' 'torchvision==0.22.1+cu128' \
  --index-url https://download.pytorch.org/whl/cu128
"$UV" pip install --python .venv/bin/python \
  -r "$ZKEEP_WS/src/zekeep_embodied/requirements-pi05-qlora.txt"
"$UV" pip check --python .venv/bin/python
```

上游 PyTorch 模型要求 Transformers 4.53.2 的专用替换文件。仅替换这个独立环境：

```bash
.venv/bin/python -c 'import transformers; assert transformers.__version__ == "4.53.2"'
cp -r src/openpi/models_pytorch/transformers_replace/. \
  .venv/lib/python3.11/site-packages/transformers/
```

`UV_LINK_MODE=copy` 避免修改 uv 缓存中的硬链接，影响其他项目。
后续重新 `uv sync` 或重装 Transformers 后，须重新安装 cu128、NF4 依赖并应用替换文件。
日常使用本包 `run_pi05.sh`，不会自动同步环境。

### 3. 下载官方基座与 tokenizer

使用固定上游的下载器从官方公共 GCS 获取，不需要复制旧电脑缓存。
默认缓存为 `~/.cache/openpi`。下列路径和 `doctor` 保持一致；准备时不要另设
`OPENPI_DATA_HOME`，否则需自行统一权重路径。

```bash
cd "$ZKEEP_WS"
bash src/zekeep_embodied/tools/run_pi05.sh - <<'PY'
from openpi.shared.download import maybe_download
for uri in (
    "gs://openpi-assets/checkpoints/pi05_base",
    "gs://big_vision/paligemma_tokenizer.model",
):
    print(maybe_download(uri, gs={"token": "anon"}))
PY
```

下载器会复用已有完整缓存。网络中断时先重跑相同命令，不要把不完整文件当作基座。
官方权重是通用基础模型，不包含本机任务的适配器或归一化统计。

### 4. 转换 PyTorch 基座并检查

转换只需一次。若输出目录已有文件，先确认它来自完整成功的同版本转换；
不要覆盖已有模型。使用一个不存在的输出目录执行：

```bash
cd "$ZKEEP_WS"
test ! -e "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch" && \
JAX_PLATFORMS=cpu third_party/openpi/.venv/bin/python \
  third_party/openpi/examples/convert_jax_model_to_pytorch.py \
  --checkpoint_dir "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_base" \
  --config_name pi05_libero \
  --output_path "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch" \
  --precision bfloat16
```

`pi05_libero` 在此只用于相同 π0.5 基座结构的格式转换，不代表使用 LIBERO 任务权重。
模型运行默认读取该目录的 `model.safetensors`。
转换失败会留下部分输出，需人工核对后换新输出目录重试；不要直接进入训练。

```bash
cd "$ZKEEP_WS"
./tools/pi05.sh help
./tools/pi05.sh doctor
bash src/zekeep_embodied/tools/run_pi05.sh - <<'PY'
import torch
import bitsandbytes
import transformers
from pathlib import Path
from safetensors import safe_open
assert torch.__version__ == "2.7.1+cu128"
assert transformers.__version__ == "4.53.2"
assert torch.cuda.is_available(), "CUDA unavailable: check NVIDIA driver"
with safe_open(str(Path.home() / ".cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch/model.safetensors"), framework="pt", device="cpu") as weights:
    assert len(list(weights.keys())) > 0
print(torch.cuda.get_device_name(0), bitsandbytes.__version__)
PY
```

这些检查不加载完整策略、不连接设备，也不证明模型已适配任务。

### 5. 新数据与训练

便携包不带已适配的任务模型。先核对 `src/zekeep_embodied/configs/record.yaml` 的相机设备、
`installation_id`、视觉解释器，以及其引用的主从臂串口与硬件标定。
按 [完整流程](src/zekeep_embodied/docs/pi05-entry.md) 进行相机预检、采集、
训练/验证划分、导出和统计，再用独立实验执行一步训练检查。

```bash
cd "$ZKEEP_WS"
./tools/pi05.sh help
./tools/pi05.sh train --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_smoke_v1 --num-train-steps 1
```

上例须先按完整流程准备同名真实训练集及统计。没有数据时不能直接运行训练或抓放。
`execute` 保留 `MOVE` 确认和 deadman，不自动使能或回零；现有执行器没有环境碰撞或自碰撞检查。
分开的安装教程及官方来源见 [PI05_INSTALL.md](docs/PI05_INSTALL.md)。

## Studio 可选安装

`apps/` 为独立的本地可选组件，本次 GitHub 分支不包含该目录。
以下步骤适用于另行取得 `apps/studio/` 的完整本地包；仅从本分支下载时可跳过本节。

Studio 不需要 ROS 或模型权重。需要积木编程时，在独立新终端准备它自己的环境。
基础安装已提供 Miniforge；若只安装 Studio，先按 [Miniforge 官方说明](https://github.com/conda-forge/miniforge)
单独安装 Miniforge，无须为了 Studio 执行整套 ROS 安装。

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh base
"$ZKEEP_MINIFORGE_DIR/bin/conda" env create -f apps/studio/environment.yml \
  -p "$ZKEEP_MINIFORGE_DIR/envs/zekeep-studio"
"$ZKEEP_MINIFORGE_DIR/bin/conda" install \
  -p "$ZKEEP_MINIFORGE_DIR/envs/zekeep-studio" -c conda-forge 'nodejs>=24.15,<25' -y
bash tools/env.sh studio
cd "$ZKEEP_WS/apps/studio"
node --version
npm --version
npm ci
python -m pip check
python -c 'import motorbridge, pinocchio, yaml'
python studio.py --self-test
python studio.py
```

已有 Studio 环境时跳过 `env create`，按需用 `conda env update` 更新。
Node.js/npm 用于首次准备静态资源，运行时不需要；当前 本地 `apps/studio/package-lock.json`的依赖要求较新的 Node.js，
不能假定基础安装中的 Ubuntu `nodejs` 足够。`npm ci` 会执行本包 `prepare` 复制 Blockly 资源。
Node 24.15.0 的官方发行文件见 [Node.js 发布目录](https://nodejs.org/download/release/v24.15.0/)。
打开 `http://127.0.0.1:8765`；启动不连接硬件。Windows 打包和真机限制见 本地 `apps/studio/README.md`。

`pip check` 只检查已安装包声明的依赖，不能发现 `environment.yml` 中漏装的整个包。
若导入检查提示缺少 `motorbridge`，优先按环境文件恢复完整环境：

```bash
cd "$ZKEEP_WS"
"$ZKEEP_MINIFORGE_DIR/bin/conda" env update \
  -f apps/studio/environment.yml -p "$ZKEEP_MINIFORGE_DIR/envs/zekeep-studio"
```

恢复后重新执行导入检查和 `python apps/studio/studio.py --self-test`，不以 `pip check` 单项通过作为完成标准。

## 安装验收

以下检查使用新电脑自己的环境，不连接机械臂。按实际选装范围执行，不能把旧电脑结果当作新电脑验收。

### ROS 与 SDK

```bash
bash tools/env.sh ros ros2 pkg prefix zekeep_bringup
bash tools/env.sh ros python -B tools/check_sdk.py
```

`ros` 包装复用 `tools/activate_ros.sh`，同时加载 ROS、项目 venv 和 Pinocchio 原生库路径。
不依赖系统 Python 恰好已经安装 SDK。更多导入检查见 [SDK 安装说明](docs/SDK_INSTALL.md)。

### 视觉与 CUDA 扩展

从项目根目录进入视觉子终端后执行：

```bash
bash tools/env.sh vision
python -m pip check
python -B tools/check_vision_model.py
python -c 'import torch; assert torch.cuda.is_available(); print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))'
python -c 'import cv2, pyorbbecsdk; assert hasattr(cv2, "aruco"); print(cv2.__version__)'
```

安装 GraspNet 后额外检查：

```bash
python -c 'import graspnetAPI, pointnet2._ext, knn_pytorch.knn_pytorch'
```

上述检查不打开相机。相机设备权限、实际 FPS 和标定身份还需现场按各模块说明验收。

### VLA 与安装入口

安装 VLA 后，从项目根目录进入子终端检查：

```bash
bash tools/env.sh vla
bash src/zekeep_embodied/tools/run.sh -m pip check
./tools/pi05.sh doctor
bash src/zekeep_embodied/tools/run.sh -m zekeep_embodied.record \
  --config src/zekeep_embodied/configs/record.yaml --simulate --frames 40 \
  --output /tmp/zekeep_capture_demo
python3 -m unittest discover -s tools -p test_pi05_entry.py -v
python3 tools/check_setup.py
sha256sum -c docs/VLA_SOURCE_SHA256SUMS
```

`doctor` 只检查路径、权重文件和 GPU，不加载完整模型。模拟只检查文件流程，不代表真实采集或训练成功。
模拟输出用独立目录，不与正式训练数据混用。根目录 `MANIFEST.sha256` 是历史快照；VLA 同步使用独立清单。
`check_setup.py` 用命令替身执行完整安装流程，覆盖环境隔离、系统库/USB 规则、CUDA 分支、
导入失败和日志保留；另检查首次/重复安装与不同 Conda 路径。它不执行真实 apt/pip 安装或 CUDA 编译。

## 运行入口

### ROS、RViz、网页、手柄和示教

每个 ROS 终端执行：

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh ros
```

无硬件 RViz：

```bash
ros2 launch zekeep_moveit_config demo.launch.py
```

仅网页（默认只读 AI 后端，不启动实机驱动或相机）：

```bash
ros2 launch zekeep_bringup web.launch.py
```

网页地址为 `http://127.0.0.1:3001`。视觉抓取统一入口只读预览：

```bash
"$ZKEEP_WS/tools/start_web_sim.sh" --preview
```

运行不带 `--preview` 的启动器会启动更多真机/视觉组件，须先按
[网页 README](src/zekeep_bringup/web/README.md) 完成现场检查；不自动使能或抓取，
AI 运动仍需 `--ai-motion` 和网页人工确认。
解释器可用 `ZKEEP_VISION_PYTHON` 覆盖，ROS venv 可用 `ZKEEP_ROS_VENV` 覆盖。
手柄和示教分别见 [手柄 README](src/zekeep_joystick/README.md)、
[示教 README](src/zekeep_teach/README.md)；真机关节测试见 [关节测试教程](docs/JOINT_SMALL_MOTION_TEST.md)。

### 视觉抓取与主从遥操作

两者使用独立 Conda 环境，且不能与 ROS 驱动同时占用同一机械臂串口。
先激活所需环境并查看对应说明，不把启动真机程序当作安装检查：

```bash
bash tools/env.sh vision
# 或在另一个终端从项目根目录进入主从遥操作环境：
# bash tools/env.sh teleop
```

视觉抓取见 [zekeep_grasp](src/zekeep_grasp/README.md)，主从臂串口和零位检查见
[zekeep_teleop](src/zekeep_teleop/README.md)。主从遥操及 `record-leader` 会连接并使能从臂，
退出会释放力矩；操作前确认急停，退出前扶稳机械臂。

Studio 后续启动：

```bash
bash tools/env.sh studio python apps/studio/studio.py
```

## 标定边界

包内可能包含已有 `intrinsics.npz`、`hand_eye.npz` 及当前机械臂配置；这些数据不能证明新电脑或新设备已验证。
设备、安装位置或场景变化后必须重新标定。完全未变化时也须先核对相机序列号、机械臂身份与安装标识，
再在视觉环境执行相机身份确认：

```bash
cd "$ZKEEP_WS"
python src/zekeep_grasp/scripts/confirm_calibration.py \
  --config src/zekeep_grasp/config/default.yaml --confirm-same-installation
```

该命令只打开相机，不连接或使能机械臂。不要把示例或源电脑设备身份直接当作新设备身份。
本次同步保留桌面包已有方向、零位、夹爪限位及固定夹爪零点保护。
当前夹爪线性映射基准为 `1.45 rad = 70 mm`，目标限制为 `0–1.35 rad`，最大开口约 `65.17 mm`；
ROS 与 SDK URDF 内容一致，SDK 保留 `official_tcp` 兼容坐标系。
修改模型或更换硬件后须重新核对现场并确认标定身份。

## 常见遗漏与恢复

| 现象 | 检查与处理 |
| --- | --- |
| `ros2` 或 SDK 导入失败 | 在工作区执行 `bash tools/env.sh ros`，确认新电脑已构建；不加载旧电脑 install |
| Conda 里读到错误 NumPy / PyAV / MotorBridge | 使用对应的 `tools/env.sh` 包装，确认 Python 路径后再 `pip check` |
| GraspNet 扩展缺失 | 确认接受许可、`nvidia-smi` 可用、`nvcc --version` 为 12.8，再在目标机重跑基础安装 |
| 视觉模型校验失败 | 重跑 `setup.sh` 获取对应文件；下载失败不能用空文件或 LFS 指针代替 |
| 新用户无法打开串口 | 完成安装后重新登录，运行 `id -nG` 确认包含 `dialout`，不使用 `chmod 777` |
| Orbbec 普通用户找不到设备 | 核对 USB 连接和实际相机序列号，按上文检查并安装 SDK udev 规则，再拔插相机 |
| VLA `doctor` 缺少 OpenPI 或权重 | 基础安装不会安装 VLA，手动完成 VLA 选装及转换 |
| VLA 数据集/实验不存在 | 重新采集并导出统计，显式选择 `--repo-id`、`--exp-name`；基础权重不是任务适配器 |
| Studio 找不到 `npm` 或 Node 版本过低 | 在 Studio 独立环境安装上文 Node.js，运行 `npm ci` 准备资源 |
| Studio 的 `pip check` 通过但缺少 `motorbridge` | 按 `environment.yml` 更新完整环境，再做导入检查和 `--self-test` |
| 离线新电脑缺依赖或权重 | 本包不是完整离线安装介质；先联网完成相应模块准备，或另行准备完整包缓存 |

## 许可与安全

本工作区仅供本人或同一机构内部非商业研究，不得用于商业用途或向第三方转让、分发。
GraspNet 源码与权重另受其许可限制；SDK、OpenPI、模型及其他依赖按各自适用许可条款使用。

安装和离线检查不会授权实机运动。真机启动前检查串口独占、急停、工作空间、标定、速度与碰撞边界。
监督执行、遥操作、SDK 抓取各有不同设备行为，须按模块说明操作；模拟通过不等于实机验收。

更新记录与历史验证结果见 [CHANGELOG.md](CHANGELOG.md)。
