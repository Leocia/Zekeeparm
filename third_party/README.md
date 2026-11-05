# 第三方源码目录

本目录通过此说明文件随源码包和 GitHub 仓库保留。目录名为 `third_party`。
GraspNet 源码、API、CUDA 扩展和权重全部独立准备，基础 `setup.sh` 不执行这些步骤，也不验证 GraspNet 后端。
须阅读 [第三方说明](../docs/THIRD_PARTY_NOTICES.md)及其链接的上游许可后按需获取。

## 独立下载源码和权重

阅读并接受上游许可后，直接从以下官方 GitHub 原址手动下载。表中的路径均相对于项目根目录。

| 下载内容 | 官方原址 | 下载后放置位置 |
| --- | --- | --- |
| GraspNet baseline 源码 | [graspnet/graspnet-baseline](https://github.com/graspnet/graspnet-baseline) | `third_party/graspnet-baseline/` |
| GraspNet API 源码 | [graspnet/graspnetAPI](https://github.com/graspnet/graspnetAPI) | `third_party/graspnet-baseline/graspnetAPI/` |
| `checkpoint-rs.tar` 权重 | [baseline 官方 README 的 Training and Testing 部分](https://github.com/graspnet/graspnet-baseline#training-and-testing)中 `checkpoint-rs.tar` 的下载链接 | `third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar` |

源码可在各仓库选择 **Code → Download ZIP**；分别解压后，将 `graspnet-baseline-main` 改名为 `graspnet-baseline`，
将 `graspnetAPI-master` 改名为 `graspnetAPI`，放入上表指定位置。API 目录应直接包含 `setup.py` 和其内部的 `graspnetAPI/` Python 包目录。
权重由官方 README 链接的网盘提供，不包含在 GitHub 源码 ZIP 中；保留 `checkpoint-rs.tar` 文件名，不解压。
不要命名为 `third_part`，也不要额外嵌套一层解压目录。最终结构应为：

```text
third_party/
├── README.md
└── graspnet-baseline/
    ├── models/graspnet.py
    ├── pointnet2/setup.py
    ├── knn/setup.py
    ├── graspnetAPI/setup.py
    └── checkpoints/checkpoint-rs.tar
```

下载的依赖不纳入本仓库提交，也不属于原始交付清单。

## 基础安装后独立安装 GraspNet

先完成基础视觉环境，以及 [驱动和 CUDA 单独安装](../docs/GPU_SETUP.md)。获取源码后，先安装 Cython，供 GraspNet API 的 `grasp_nms` 在关闭构建隔离时使用，再安装 API 和扩展：

```bash
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt Cython==3.0.12
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt --no-build-isolation ./third_party/graspnet-baseline/graspnetAPI
source tools/environment.sh
export TORCH_CUDA_ARCH_LIST="$(bash tools/env.sh vision python -c 'import torch; print(".".join(map(str, torch.cuda.get_device_capability())))')"
bash tools/env.sh vision bash tools/build_vision_extensions.sh
```

最后做无设备导入检查和权重校验，不打开相机或机械臂：

```bash
bash tools/env.sh vision python -m pip check
bash tools/env.sh vision python -c 'import sys; sys.path.insert(0, "src/zekeep_grasp"); from utils.native_runtime import preload_environment_libstdcpp; preload_environment_libstdcpp(); import utils.graspnet_utils; print("GraspNet 后端导入：OK")'
grep '  third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar$' docs/MODEL_SHA256SUMS | sha256sum -c -
```

检查输出应包含 `third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar: OK`；文件缺失或校验不符时该检查会失败。
仅有本目录或下载源码不代表 GraspNet 已可用，还需要 API 安装、目标 GPU 上的扩展编译和 checkpoint。
跳过 GraspNet 不影响 ROS/SDK 基础功能；使用 GraspNet 后端的抓取入口需补齐上述依赖。
