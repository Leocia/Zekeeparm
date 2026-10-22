# 新电脑 π0.5 依赖与权重安装

本说明对应随包提供的 `src/zekeep_embodied` 和 `tools/pi05.sh`。
便携包只带源码、测试和补丁，不带虚拟环境、OpenPI、模型权重、示范数据或训练结果。
首次准备需要联网；安装和权重转换不会连接相机或机械臂。

## 1. 基础环境

目标平台沿用本包：Ubuntu 22.04 x86_64、ROS 2 Humble。先按根目录
[README](../README.md) 安装 NVIDIA 驱动并执行 `bash setup.sh`，准备 ROS、SDK、
视觉 `rebotarm` 环境。VLA 额外安装不自动加入原安装脚本，避免默认下载大模型。

在新终端进入实际包目录，以下示例使用桌面路径；路径不同只改这一处：

```bash
cd "$HOME/Desktop/Zekeeparm"
bash tools/env.sh base
```

包装脚本自动设置 `ZKEEP_WS`、`ZKEEP_MINIFORGE_DIR`、`EMBODIED_PYTHON` 和 `ZKEEP_VISION_PYTHON`，
并清理旧 ROS/Python 路径。以下首次安装命令在这个准备子终端内执行；`exit` 返回原终端。
自定义 Miniforge 目录时用 `ZKEEP_MINIFORGE_DIR=/实际路径 bash tools/env.sh base`。
日常采集/训练流程使用 `bash tools/env.sh vla`，直接操作 OpenPI 使用 `bash tools/env.sh model`。
详细边界见[总 README](../README.md#各环境的边界)。

不要把旧电脑的 `.venv-ros` 符号链接、`build/`、`install/`、`log/` 或 Conda 环境搬过去，
应在新电脑本地创建和构建。

采集环境使用 Python 3.10 / NumPy 2，视觉环境使用 Python 3.10 / NumPy 1，
模型环境使用 Python 3.11 / NumPy 1，三者保持隔离。
原始 JAX 基座约 11.6 GiB，转换后的 PyTorch 基座另占空间；模型环境在源电脑约 9.2 GiB。
下载缓存、转换临时文件、示范 PNG 和训练产物还会增加占用，应另留充足磁盘空间。
转换和训练的 RAM/VRAM 峰值没有跨电脑保证；8 GB 显卡只适用本地 QLoRA 实验路径，
不能据此保证新电脑的训练效果或容量足够。

## 2. 采集与数据处理依赖

只采集数据可完成本节后跳到第 6 节，不必下载模型。
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

## 3. OpenPI 源码和模型依赖

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

## 4. 下载官方基座与 tokenizer

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

## 5. 转换 PyTorch 基座并检查

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

## 6. 新电脑现场配置与开始训练

核对 `src/zekeep_embodied/configs/record.yaml`：全局相机设备名、双相机安装标识、
视觉解释器，以及它引用的 `zekeep_teleop` 串口和 `zekeep_bringup` 硬件标定。
包内现有夹爪标定和固定零点保护已保留；换机械臂后须重新核对该机械臂的标定。

先做无硬件模拟，再按 [完整流程](../src/zekeep_embodied/docs/pi05-entry.md)
进行相机预检、采集、按 episode 划分训练/验证、导出、统计和一步训练检查。
默认无示范、无适配器，`train` 会要求先准备新的真实数据，不能直接执行抓放。

```bash
cd "$ZKEEP_WS"
bash src/zekeep_embodied/tools/run.sh -m zekeep_embodied.record \
  --config src/zekeep_embodied/configs/record.yaml --simulate --frames 40 \
  --output /tmp/zekeep_capture_demo
```

监督执行继续保留 `MOVE` 确认和 deadman，不自动使能或回零；
现有执行器没有环境碰撞或自碰撞检查，首次实机运行须现场监督。

## 来源

- [固定 OpenPI 版本、官方权重下载及 Transformers 替换说明](https://github.com/Physical-Intelligence/openpi/tree/215abfb217dbac7d5f1273282331b9b1866c0479)
- [PyTorch 2.7.1 官方 CUDA 12.8 安装命令](https://pytorch.org/get-started/previous-versions/#v271)
- [bitsandbytes 0.50.2 发布](https://pypi.org/project/bitsandbytes/0.50.2/)

同步时只验证本地源码、入口、补丁和无硬件测试；未在另一台干净电脑重新下载全部依赖和权重。
