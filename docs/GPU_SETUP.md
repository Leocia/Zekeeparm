# NVIDIA 驱动与 CUDA：单独手动安装

适用平台：Ubuntu 22.04 x86_64。先准备 GPU 环境，再运行项目的 `setup.sh`。
驱动与 CUDA Toolkit 均由使用者在项目安装器外下载、安装和维护；`setup.sh` 不安装或升级这两项。
Miniforge 也在脚本外手动安装，见 [README](../README.md#首次安装)。

## 版本与安装顺序

| 项目 | 本包使用的版本 / 要求 | 安装位置或验证方式 |
| --- | --- | --- |
| NVIDIA 驱动 | 建议部署基线为 Linux 570.124.06 或更新的、支持实际 GPU 的驱动 | 重启后 `nvidia-smi` 能列出 GPU |
| CUDA Toolkit | 12.8.1（CUDA 12.8 Update 1） | 默认 `/usr/local/cuda-12.8`；`nvcc --version` 显示 `release 12.8` |
| CUDA 扩展编译器 | GCC/G++ 11 | `gcc-11 --version`、`g++-11 --version` |
| 视觉 `rebotarm` | PyTorch 2.7.1+cu128、torchvision 0.22.1+cu128、torchaudio 2.7.1+cu128；Python 3.10 | 基础安装器负责 Python wheel |
| 模型 `act`（选装） | PyTorch 2.7.1/cu128、torchvision 0.22.1；Python 3.10 | `models` 选装入口负责 Python wheel |
| 数采 `embodied`（选装） | PyTorch 2.6.0、torchvision 0.21.0；Python 3.10 | 保留独立版本约束，不套用视觉环境的版本 |

顺序：**驱动安装、重启并检查 → CUDA Toolkit 单独下载安装 → 配置路径 → 项目安装 → PyTorch 检查**。
上表驱动基线取自 [NVIDIA CUDA 12.8.1 对应驱动表](https://docs.nvidia.com/cuda/archive/12.8.1/cuda-toolkit-release-notes/index.html#cuda-driver)。
它是本包的部署建议，不是 CUDA 12.x 所有兼容模式的最低版本；新 GPU 可能需要更新驱动，以实际 GPU 支持列表为准。
PyTorch 配套版本依据 [PyTorch 官方历史版本表](https://pytorch.org/get-started/previous-versions/)。

## 1. 单独安装 NVIDIA 驱动

已有驱动且满足实际 GPU 和上述部署要求时，先验证，正常即可保留。
新机建议在 Ubuntu「软件和更新 → 附加驱动」中选择支持当前 GPU 的受支持驱动，应用后重启。
也可在项目安装器外执行以下命令，由 Ubuntu 选择适配硬件的驱动：

```bash
sudo apt-get update
sudo apt-get install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

重启后检查：

```bash
nvidia-smi
nvidia-smi --query-gpu=name,driver_version --format=csv
```

应能看到实际 GPU 与驱动版本；如版本不满足部署要求，先解决驱动兼容问题，再继续 CUDA 安装。
启用 Secure Boot 时按驱动安装提示完成模块签名或密钥注册。安装方法见 [Ubuntu 官方驱动说明](https://ubuntu.com/desktop/docs/en/latest/how-to/graphics/install-nvidia-drivers/)。

## 2. 单独下载并安装 CUDA Toolkit 12.8.1

从 [NVIDIA CUDA 12.8.1 下载页](https://developer.nvidia.com/cuda-12-8-1-download-archive)
选择 Linux / x86_64 / Ubuntu / 22.04 / runfile (local)，可用浏览器下载。
安装文件约 5.4 GB，保存在项目目录外；命令行下载可断点续传：

```bash
mkdir -p "$HOME/Downloads/zekeeparm-installers"
cd "$HOME/Downloads/zekeeparm-installers"
wget -c https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run
```

确认下载完整并阅读 NVIDIA 许可后，单独安装 Toolkit：

```bash
sudo apt-get install -y build-essential gcc-11 g++-11
sudo sh cuda_12.8.1_570.124.06_linux.run --toolkit --toolkitpath=/usr/local/cuda-12.8
```

如果出现组件选择界面，仅选 CUDA Toolkit，取消 Driver；驱动已经在上一步单独安装。
文件名里的 `570.124.06` 是该安装包附带的驱动版本，不要求覆盖现有较新驱动。
已有 apt 安装的 CUDA 12.8 时直接验证并复用，不要再用 runfile 覆盖同一套 Toolkit。
安装选项依据 [NVIDIA CUDA 12.8.1 Linux 安装说明](https://docs.nvidia.com/cuda/archive/12.8.1/cuda-installation-guide-linux/index.html#runfile)。

## 3. 写入项目环境配置

回到项目根目录，在 [tools/environment.sh](../tools/environment.sh) 中设置：

```bash
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
```

安装在其他位置时修改这一行的默认路径。已导出的 `CUDA_HOME` 优先；若终端仍保留旧路径，先执行 `unset CUDA_HOME`。
项目的环境选择器和安装器会读取该配置，使用实际安装目录；无需改系统全局 CUDA 软链接。

```bash
source tools/environment.sh
"$CUDA_HOME/bin/nvcc" --version
gcc-11 --version
g++-11 --version
bash tools/env.sh base bash setup.sh
```

不要把系统 CUDA 的 `lib64` 全局置于所有 Python 环境的 `LD_LIBRARY_PATH` 前面；项目保留各环境自己的原生库隔离。
若在项目外编译或运行独立 CUDA 程序，按 NVIDIA 安装说明为该程序的终端单独配置库路径。

## 4. 项目安装后的检查

```bash
bash tools/env.sh vision python -c 'import torch; print("PyTorch:", torch.__version__); print("wheel CUDA:", torch.version.cuda); print("GPU:", torch.cuda.is_available()); assert torch.__version__ == "2.7.1+cu128"; assert torch.version.cuda == "12.8"; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))'
```

三个版本字段含义不同：`nvidia-smi` 的 CUDA Version 是驱动支持的 CUDA 版本上限，
`nvcc --version` 是本机 Toolkit 编译器版本，`torch.version.cuda` 是 PyTorch wheel 的 CUDA 构建版本。
因此较新的驱动显示 CUDA 13.x、而 `nvcc` 和 PyTorch 显示 12.8，可以是正常组合。
字段含义见 [NVIDIA nvidia-smi 说明](https://docs.nvidia.com/deploy/nvidia-smi/index.html#cuda-version)。

仅运行 PyTorch 预编译推理时，wheel 提供所需 CUDA 用户态依赖，仍需可用驱动；
GraspNet 本地扩展编译还需要本机 Toolkit 12.8、GCC/G++ 11 和可见 GPU。
GraspNet 不在基础安装器中安装；准备好 Toolkit 和 GPU 后，按 [GraspNet 独立安装步骤](../third_party/README.md)下载源码、安装 API 并编译扩展。
不同 GPU 架构应在目标机重新编译，不复制旧电脑的扩展 `.so`。
