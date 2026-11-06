# Zekeeparm 源码交付包

目标平台：Ubuntu 22.04 x86_64；ROS 模块使用 Humble，视觉/模型的 GPU 运行使用 NVIDIA GPU。路径与用户名可变。
仅含源码、机器人 URDF/STL 等结构资源，以及指定保留的手眼和颜色标定；不含模型权重、训练数据、密钥、虚拟环境或本机编译产物。

## 首次安装

在实际项目根目录，先校验收到的原始交付包：

```bash
export ZKEEP_WS="$PWD"
if [[ -f MANIFEST.sha256 ]]; then sha256sum -c MANIFEST.sha256; fi
python3 -B tools/check_delivery.py
```

桌面交付包附带 `MANIFEST.sha256`；GitHub 源码不含此快照清单，运行 `check_delivery.py` 即可。
清单只针对未安装、未填写设备配置的交付快照；修改或安装后产生差异是正常现象。

先在安装脚本外手动准备 Conda。没有 Conda 时，从 [Miniforge 官方发布页](https://github.com/conda-forge/miniforge/releases/latest)
下载 `Miniforge3-Linux-x86_64.sh` 及其同版本 `.sha256` 文件，在下载目录执行：

```bash
sha256sum -c Miniforge3-Linux-x86_64.sh.sha256
bash Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
```

已有 Conda 时跳过下载安装，不要覆盖现有目录。统一在 [tools/environment.sh](tools/environment.sh) 设置安装根目录；默认行如下：

```bash
export ZKEEP_MINIFORGE_DIR="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
```

若安装在 `~/miniconda3`，把该行的 `$HOME/miniforge3` 改为 `$HOME/miniconda3`；其他路径同理。
已有终端中的 `ZKEEP_MINIFORGE_DIR` 优先于文件默认值，可用 `conda info --base` 查询实际根目录。
安装器、环境选择器和项目启动脚本都会读取此文件，无需 `conda init`，无需每个终端重复设置路径。

## 按功能选择安装模块

安装以 `Zekeeparm_ws` 已实现的功能为基础。**每次只安装选中的模块；不带参数只显示帮助。**
完整的系统依赖、环境版本、步骤、模型位置与验证命令统一见 [分模块安装说明](docs/INSTALL_PORTABLE.md)。
已有 Conda 环境会被对应模块更新；安装前确认 `tools/environment.sh` 指向本次部署使用的 Conda。

| 要使用的功能 | 需要安装 | 安装顺序 |
| --- | --- | --- |
| ROS 驱动、MoveIt、网页、固定点搬运、拖动示教、手柄、终端 LLM | `ros` | `bash setup.sh ros` |
| 相机、标定、YOLOE、直接 SDK 视觉功能 | `vision` | `bash setup.sh vision`；GraspNet 抓取另按下一行补齐 |
| 原 YOLO＋GraspNet 抓取后端 | `vision` + GraspNet | 先装 `vision`，再按 [GraspNet 步骤](third_party/README.md)安装源码、API、扩展和 checkpoint |
| 网页相机抓取、视觉跟随、LLM 视觉任务 | `ros` + `vision`；选择 GraspNet 后端时还需 GraspNet | 分别执行两个模块，再补齐后端；见 [功能组合](docs/INSTALL_PORTABLE.md#按实际功能组合模块) |
| 独立主从遥操作、舵机配置工具 | `teleop` | `bash setup.sh teleop`；无需 ROS 或视觉环境 |
| 手柄/主臂示范采集 | `ros` + `vision` + `collect` | 先装前两个，再执行 `bash tools/install_optional.sh collect` |
| ACT / π0.5 离线导出、训练、模型推理服务 | `models` | `bash tools/install_optional.sh models`；执行真机策略再补 `ros`、`vision`、`collect` |
| ROS MuJoCo 仿真 | `ros` + `sim` | 先装 `ros`，再执行 `bash tools/install_optional.sh sim` |
| 独立网页 MuJoCo 仿真 | `web-sim` | 准备 Node.js 22+/npm、系统 Python venv，再执行 `bash tools/install_optional.sh web-sim`；无需 ROS、Conda 或机械臂 |

`ros`、`teleop` 和独立网页仿真不要求 CUDA Toolkit。视觉 GPU 推理、模型 GPU 训练需可用 NVIDIA 驱动；
**GraspNet 扩展编译还需单独安装 CUDA Toolkit 12.8.1、GCC/G++ 11。**
驱动、Toolkit 和 Miniforge 均手动安装，项目脚本不替代这些安装，步骤见 [GPU 说明](docs/GPU_SETUP.md)。
`vision` 安装 cu128 PyTorch、相机库、SDK、Ultralytics 8.4.35，默认下载并校验 YOLOE-26L 与 MobileCLIP，
不安装 GraspNet。模型、训练数据和设备配置仍按后文分别准备。

所有命令从实际项目根目录运行。`setup.sh` 会执行 `sudo apt`、加入 `dialout` 组；`vision` 还安装 Orbbec USB 规则。
选装入口只安装软件依赖，不执行 sudo；其系统前置包须先按安装说明准备。安装不连接设备、不使能电机。
安装日志为 `log/setup-<模块>-*.log`。可单独重跑失败模块；不要重跑未使用的模块。
原先的 `bash tools/env.sh base bash setup.sh` 现在仅显示帮助，须在末尾加 `ros`、`vision` 或 `teleop`。
环境选择器 `tools/env.sh` 的 `base` 表示干净终端环境，不是“基础安装”模块。

环境分别为 `.venv-ros`（Conda / Python 3.10）、`rebotarm`（3.10）、`lerobot`（3.12）、
`embodied`（3.10）、`act`（3.10）、`web_mujoco/.venv`（系统 Python），不能把各环境依赖混装。
`apps/` 是另行交付的应用目录，不上传 GitHub，也不属于以上安装流程。

先做安装自检，再按本文完成新设备配置和现场验收。此前 SDK 隔离环境已完成八个 ROS 包构建、SDK 自检、
FK/IK、ROS 接口导入和 `pip check`；视觉已完成库导入、YOLOE/MobileCLIP 加载及 CPU 推理。
这些历史结果不代表本次已完成新 Ubuntu 全量安装、GPU 或新设备验收。当前安装入口基准检查见 [安装说明](docs/INSTALL_PORTABLE.md#最小基准检查)。

## SDK 依赖与安装顺序

SDK 源码保留在 `zekeeparm_SDK/`；ROS 和视觉分别安装依赖，并以 editable 方式关联同一份 SDK 源码。
两套环境是按 ROS 驱动和视觉功能隔离，并非 SDK 自身要求；只部署其中一类功能时，只需准备对应环境。分别用 `bash setup.sh ros` 和 `bash setup.sh vision` 安装，按需选择。
两个 SDK 环境统一使用 **Python 3.10、NumPy 1.26.4、Pinocchio 3.9.0、MotorBridge 0.5.1**。
Pinocchio 及其原生库先由 Conda 安装，再安装 pip 依赖，最后安装 SDK；不要用裸 `pip install pin` 或先装 SDK 再让 pip 自动选择依赖。
Pinocchio 3.9 的 PyPI wheel 与 Conda 包使用不同的 NumPy/原生库依赖组合，旧脚本的 ROS 清单会把 NumPy 升到 2.2.6；本包已移除该安装组合。

`ros` 和 `vision` 安装入口分别按此顺序安装对应环境；分步操作和 MotorBridge 补装命令见 [SDK 安装说明](docs/SDK_INSTALL.md)。
SDK 的 `--no-deps` 命令不会补装 MotorBridge，必须先完成对应环境的依赖安装。安装后分别检查：

```bash
bash tools/env.sh ros python -c 'import numpy, pinocchio; from importlib.metadata import version; from motorbridge import Controller, Mode, CallError; assert numpy.__version__ == "1.26.4"; assert pinocchio.__version__ == "3.9.0"; assert version("motorbridge") == "0.5.1"; print("ROS SDK 依赖：OK")'
bash tools/env.sh vision python -c 'import numpy, pinocchio; from importlib.metadata import version; from motorbridge import Controller, Mode, CallError; assert numpy.__version__ == "1.26.4"; assert pinocchio.__version__ == "3.9.0"; assert version("motorbridge") == "0.5.1"; print("视觉 SDK 依赖：OK")'
```

**已用旧包创建 `.venv-ros` 的接收方：** 先按 [旧环境迁移步骤](docs/SDK_INSTALL.md#旧包环境迁移)备份旧环境，再按新清单创建。
安装器遇到旧 venv 会停止并提示，不会覆盖。不要只在旧环境内强行降级 NumPy。
遥操作、数采和模型环境独立管理依赖，不安装本 SDK；其中需要 NumPy 2.x 的环境不与 SDK 混用。

## 新设备配置与标定

本次部署使用同型号的新机械臂和新相机，**原设备身份不沿用**。

三个字段都位于 `src/zekeep_grasp/config/default.yaml`，用途和获取方式如下：

| 字段 | 作用 | 如何填写、何时更新 |
| --- | --- | --- |
| `camera.serial` | 标识实际使用的相机；运行时与设备读出的序列号核对 | 由下面的脚本读取并自动写入，无需手动抄写；更换相机后重新读取 |
| `robot.id` | 为这台实体机械臂提供稳定的标定身份标签 | 人工填写，可用铭牌编号或自定编号，如 `zekeep_arm_01`；同一台机械臂保持不变，不同机械臂使用不同编号 |
| `calibration.installation_id` | 标识相机、机械臂和工作区的本次安装布局，供标定配置核对与记录使用 | 人工命名，如 `desk_A_v1`；重新安装相机或机械臂、改变工作区布局后更新，如 `desk_A_v2`；仅换电脑且设备与布局未变时可保持不变 |

`robot.id` 和 `calibration.installation_id` 都无法从设备自动读取。当前视觉标定校验要求二者非空，
建议使用字母、数字、下划线或连字符，并加双引号。`robot.id` 不负责指令寻址；多机械臂还须分别配置串口和 ROS 命名空间。
**填写编号不等于完成标定，也不会授权运动。** 按以下顺序准备新设备，所有命令均在项目根目录执行。

1. **读取并自动填写相机序列号。** 先完成 `vision` 安装，重新登录使串口组权限生效，相机按安装提示拔插一次。
   只连接本次要配置的一台相机，关闭其他占用相机的软件，再执行：

   ```bash
   bash tools/env.sh vision python src/zekeep_grasp/scripts/read_camera_serial.py
   ```

   脚本会打开相机、打印真实序列号，并自动写入 `default.yaml` 的 `camera.serial`；不会连接机械臂。

2. **手动填写机械臂编号和安装布局编号。** 在同一份 `default.yaml` 的现有 `robot` 和 `calibration` 节点下修改 `id`、`installation_id`。
   以下仅展示这两个字段，不要用它覆盖整个配置文件，也不要重复添加同名节点：

   ```yaml
   robot:
     id: "zekeep_arm_01"
   calibration:
     installation_id: "desk_A_v1"
   ```

   示例名称可自行替换；新设备应使用自己的编号。两台机械臂可分别命名为 `zekeep_left`、`zekeep_right`。

3. **保存当前相机内参。** 确认 `default.yaml` 中的图像尺寸和相机预设符合后续使用方式，再执行：

   ```bash
   bash tools/env.sh vision python src/zekeep_grasp/scripts/save_orbbec_intrinsics.py
   ```

   脚本只打开相机，将当前内参保存到 `src/zekeep_grasp/config/calibration/orbbec_gemini2/intrinsics.npz`；
   已有文件会先备份。更换相机或改变图像配置后，重新保存并核验内参。该步骤不会计算手眼标定。

4. **核验标定和机械臂参数。** 核对串口、方向、零位、夹爪行程、TCP 和桌面几何。
   手眼与颜色标定按要求保留，**保留文件不代表新安装已经验证**；安装位置、曝光或光照变化时先核验，必要时重新标定。
   硬件 YAML 的运动授权与监督重力试验授权默认关闭，经验力矩修正为 1；完成现场核验后再明确开启所需授权。

5. **按所用功能补齐配置。** 遥操作还需在对应配置中填写主从设备 ID 并完成设备零位标定，这些字段与上述视觉配置的 `robot.id` 分开管理。
   ACT/VLA 还需选择全局相机路径，填写 `src/zekeep_embodied/configs/record.yaml` 的 `installation_id` 和工作区域，
   现场核验后将 `workspace.confirmed` 设为 `true`。采集配置不会自动复制视觉配置中的安装标识；同一安装布局使用一致的标识。

自动标定姿态也提供空模板，须按新现场生成并验证后使用。独立 SDK 抓取入口执行完整抓取放置循环，未现场确认 `place.enabled` 时拒绝启动。固定放置功能默认关闭；源代码的放置点与工作台尺寸仅为待现场核对的参考值。

## Ultralytics 与 YOLO26 模型

视觉环境为 Conda `rebotarm`。Ultralytics 是运行库；本包默认模型文件是 `yoloe-26l-seg.pt`，文本提示还需要 `mobileclip2_b.ts`。
安装完成应同时具备这三项。只有 Python 包或只有权重文件都不算视觉安装完成。
`bash setup.sh vision` 会依次执行 Ultralytics 独立安装、导入检查、模型下载和自检；任何一步报错会停止，并将失败步骤记录在 `log/setup-*.log`。
若另一台电脑曾安装中断，在项目根目录按以下顺序补齐，不必重装整套 ROS：

若视觉环境尚未创建，先执行以下一次性步骤；该 Conda 清单只安装基础库，之后仍须执行下面的 pip 步骤：

```bash
source tools/environment.sh
"$ZKEEP_MINIFORGE_DIR/bin/conda" env create -p "$ZKEEP_MINIFORGE_DIR/envs/rebotarm" -f src/zekeep_grasp/environment.yml
```

已有视觉环境时，用 `env update` 替代上述命令中的 `env create`，再执行以下步骤：

```bash
# 先按 src/zekeep_grasp/environment.yml 创建视觉 Conda 环境；以下命令使用本包约束。
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt torch==2.7.1+cu128 torchvision==0.22.1+cu128 torchaudio==2.7.1+cu128 --index-url https://download.pytorch.org/whl/cu128 --extra-index-url https://pypi.org/simple
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt -r tools/requirements-vision.txt
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt ftfy regex tqdm pynput 'git+https://github.com/ultralytics/CLIP.git@c4b6ea0932a2c0f39a0fa528af5ec4982ff15cab'
bash tools/env.sh vision python -m pip install --no-deps -e ./zekeeparm_SDK
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt ultralytics==8.4.35
bash tools/env.sh vision python -m pip install --force-reinstall --no-deps opencv-contrib-python==4.7.0.72
bash tools/env.sh vision python -c 'import ultralytics, numpy; from ultralytics import YOLO, YOLOE; assert ultralytics.__version__ == "8.4.35"; assert numpy.__version__ == "1.26.4"; print("Ultralytics / YOLO / YOLOE：OK")'
bash tools/download_models.sh base
bash tools/env.sh vision python -B tools/check_yolo.py
bash tools/env.sh vision python -m pip check
```

`check_yolo.py` 校验两份权重的 SHA-256，加载模型、设置配置中的文本类别，并用空白图像做 CPU 推理；不打开相机或机械臂。
权重位置分别为 `src/zekeep_grasp/models/yoloe-26l-seg.pt` 和项目根目录的 `mobileclip2_b.ts`；下载脚本同时建立视觉模块目录中的文本权重链接。
CLIP 等视觉依赖须先按 `vision` 模块步骤准备；不要对 Ultralytics 使用 `--no-deps` 来掩盖缺失依赖。
`bash setup.sh vision` 默认下载 YOLOE 与 MobileCLIP 权重。仅准备源码依赖时可执行 `ZKEEP_DOWNLOAD_MODELS=0 bash setup.sh vision` 跳过，但安装器会提示视觉模型尚未就绪；之后必须执行上面的下载和自检命令。
该开关不安装或下载 GraspNet。

## 其他模型下载

GraspNet 直接从官方 GitHub 原址手动下载，路径相对于项目根目录：

| 内容 | 官方原址 | 放置位置 |
| --- | --- | --- |
| baseline 源码 | [graspnet/graspnet-baseline](https://github.com/graspnet/graspnet-baseline) | `third_party/graspnet-baseline/` |
| API 源码 | [graspnet/graspnetAPI](https://github.com/graspnet/graspnetAPI) | `third_party/graspnet-baseline/graspnetAPI/` |
| `checkpoint-rs.tar` | [baseline 官方 README 的权重链接](https://github.com/graspnet/graspnet-baseline#training-and-testing) | `third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar` |

ZIP 解压后须去掉目录名的 `-main` / `-master` 后缀；权重不解压。完整目录结构和后续安装见 [第三方目录说明](third_party/README.md)。

π0.5 基础权重、tokenizer、ACT 用户模型及训练结果按[选装说明](docs/OPTIONAL_MODULES.md)单独准备。
下载脚本不连接相机或机械臂。

## 功能与入口

| 功能 | 模块 / 入口 |
| --- | --- |
| 驱动、MoveIt、网页 | `zekeepcontroller`、`zekeep_bringup`、`zekeep_moveit_config` |
| 视觉抓取、跟随、LLM | `zekeep_grasp`、`zekeep_shadow`、`zekeep_llm` |
| 示教、手柄、遥操作 | `zekeep_teach`、`zekeep_joystick`、`zekeep_teleop` |
| 共用数采 | `tools/collect.sh` |
| ACT、π0.5（选装） | `tools/act.sh`、`tools/pi05-model.sh`；兼容 `tools/pi05.sh` |
| ROS MuJoCo（选装） | `zekeep_sim`，独立命名空间 `/zekeep_sim` |
| 独立网页仿真（选装） | `web_mujoco/`，不依赖真机或 ROS |
| 底层 SDK | `zekeeparm_SDK/` |

```bash
# ROS 环境；以下 demo 使用模拟关节，不连接真机。
bash tools/env.sh ros ros2 launch zekeep_moveit_config demo.launch.py
# 网页与只读 AI 后端，不启动机械臂驱动或相机。
bash tools/env.sh ros ros2 launch zekeep_bringup web.launch.py
# 保留原交付入口；--preview 不启动真机驱动，仍打开相机和视觉。
./tools/start_web_sim.sh --preview
```

网页默认 `http://127.0.0.1:3001`。`start_web_sim.sh` 是原网页控制台的历史入口名；独立 MuJoCo 仿真见选装说明。
真机 SDK、ROS、遥操作不能同时占用同一机械臂串口。所有运动都需要新设备现场配置、标定及人工授权。

环境入口：`bash tools/env.sh ros|vision|teleop|vla|model|act [命令]`。
`vla` 用于数采，`model` 与 `act` 均选择共用 LeRobot `act` 环境。完整安装与操作见各模块 README。

接口见 [ROS API](API_zh.md)，第三方来源见 [许可说明](docs/THIRD_PARTY_NOTICES.md)。
