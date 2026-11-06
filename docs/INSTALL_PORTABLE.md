# Ubuntu 22.04 分模块安装

目标平台为 Ubuntu 22.04 x86_64、ROS 2 Humble。以下命令均从实际项目根目录执行。
安装需要网络；本包不附环境、权重、训练数据、密钥或编译产物。准备完成并缓存模型后，
本机网页、ROS 演示和本地视觉可以离线运行；远程 LLM 仍依赖用户配置的网络服务。

## 按实际功能组合模块

本说明以 `Zekeeparm_ws` 的现有实现为功能依据，不要求一次安装所有模块。
原 `src/rebot_grasp` 在交付包中为 `src/zekeep_grasp`；原 `src/zekeep_vlm` 为 `src/zekeep_llm`；
SDK 由 `third_party/reBotArm_control_py` 整理到 `zekeeparm_SDK`。功能启动入口仍见各模块 README。
原工作区的 `kinematics.yaml` 使用 KDL，交付包沿用 KDL，无需额外编译 `trac_ik/`。
不复制原电脑的 Conda 环境、build/install/log、CUDA `.so`、node_modules、设备身份或密钥。

| 场景 | 必需模块 | 另外需要准备 |
| --- | --- | --- |
| 无硬件 MoveIt 演示、ROS 驱动、固定点搬运、拖动示教、手柄、普通网页 | `ros` | 真机使用前填写硬件配置；LLM 文本任务另配自己的 API Key |
| 相机预览、内参/手眼标定、YOLOE 检测、直接 SDK 视觉功能 | `vision` | 相机和新安装标定；实际抓取选用 GraspNet 时补装下一节的后端 |
| 原 YOLO＋GraspNet 直接 SDK 抓取 | `vision` + GraspNet | CUDA Toolkit、API、CUDA 扩展、checkpoint；无需 ROS 驱动同时运行 |
| 网页相机抓取、分拣、堆叠，或 LLM 调用这些任务 | `ros` + `vision` + GraspNet | 当前原抓取流程使用 `yolo-graspnet`，不能只装 YOLOE；还需模型、相机、标定、任务点位 |
| 视觉跟随 | `ros` + `vision` | 相机与该功能的配置，不因安装 ROS 包而自动安装视觉环境 |
| 独立主从遥操作、舵机网页工具 | `teleop` | 主从串口、设备 ID、主臂零位；不要求 `ros`、`vision`、`collect` |
| 手柄采集或主臂经 ROS 示教采集 | `ros` + `vision` + `collect` | 双相机、现场范围、任务和安装标识；`collect` 自带主臂 Python 插件，不要求另建 `lerobot` 环境 |
| ACT / π0.5 离线导出、训练、模型服务 | `models` | 自备 LeRobot v3 数据、模型权重；GPU 运行另需驱动 |
| ACT / π0.5 真机策略执行 | `models` + `ros` + `vision` + `collect` | 匹配的模型、双相机、坐标/设备配置、现场授权；无需同时运行独立遥操作 |
| ROS MuJoCo 后端 | `ros` + `sim` | 离屏相机需要可用 EGL；与独立网页仿真分别安装 |
| 浏览器内独立 MuJoCo 仿真 | `web-sim` | 系统 Python venv、Node.js 22+/npm；不要求 ROS、Conda、CUDA Toolkit 或真机 |

`apps/` 为独立应用包，保留其自身安装说明，不纳入本安装流程或 GitHub 源码发布。

## 公共准备：只准备所选模块需要的工具

1. 在项目根目录执行 `python3 -B tools/check_delivery.py`。桌面原始交付包另执行 `sha256sum -c MANIFEST.sha256`；GitHub 不含该快照清单。
2. 安装 `ros`、`vision`、`teleop`、`collect`、`models` 时，先按 [README](../README.md#首次安装)手动安装或复用 Miniforge/Miniconda。
3. 在 `tools/environment.sh` 配置 `ZKEEP_MINIFORGE_DIR`。默认 `~/miniforge3`，已有安装可用 `conda info --base` 查路径。
4. 在新终端运行安装，不同时加载其他 ROS 工作区。安装入口不执行 `conda init`，不修改全局 Conda 源。
5. 只有使用 GPU 推理/训练时才需 NVIDIA 驱动；GraspNet 扩展编译额外需要 CUDA Toolkit 12.8.1 和 GCC/G++ 11，详见 [GPU_SETUP.md](GPU_SETUP.md)。纯 ROS、遥操作、网页仿真无需先装 Toolkit。

环境根目录由上述公共配置统一读取，已导出的同名变量优先。改变 Conda 根目录后打开新终端；
当前终端切换时先 `unset ZKEEP_MINIFORGE_DIR ZKEEP_VISION_PYTHON EMBODIED_PYTHON`，再运行入口。
`ZKEEP_ROS_VENV` 可指定 ROS 环境安装目录；`ZKEEP_VISION_PYTHON` 可指定视觉环境解释器，安装器按其父目录安装。
`EMBODIED_PYTHON`、`LEROBOT_PYTHON` 是运行时解释器覆盖；`collect`、`models` 仍分别安装到配置的 Conda 根目录下 `envs/embodied`、`envs/act`。
不要用这些覆盖值指向尚未安装或版本不匹配的环境。

### 环境与版本

| 模块 | 安装位置 | Python / 关键依赖 | 依赖清单 |
| --- | --- | --- | --- |
| `ros` | 项目 `.venv-ros`（Conda 管理，保留旧目录名） | Python 3.10、NumPy 1.26.4、Pinocchio 3.9.0、MotorBridge 0.5.1 | `tools/environment-ros.yml`、`tools/requirements-ros-sdk.txt` |
| `vision` | Conda `envs/rebotarm` | Python 3.10、NumPy 1.26.4、Pinocchio 3.9.0、MotorBridge 0.5.1、torch 2.7.1+cu128、Ultralytics 8.4.35 | `src/zekeep_grasp/environment.yml`、`tools/requirements-vision.txt`、`tools/constraints-vision.txt` |
| `teleop` | Conda `envs/lerobot` | Python 3.12、LeRobot 0.4.4、MotorBridge 0.4.9 | `src/zekeep_teleop/pyproject.toml` |
| `collect` | Conda `envs/embodied` | Python 3.10、NumPy 2.2.6、torch 2.6.0、MotorBridge 0.4.9 | `src/zekeep_embodied/requirements-embodied.txt` 与本地主臂/数采包 |
| `models` | Conda `envs/act` | Python 3.10、LeRobot 0.4.4、NumPy 2.2.6、torch 2.7.1/cu128 | `src/zekeep_embodied/requirements-models.txt` 与本地数采/ACT 包 |
| `sim` | 加入已有 ROS 环境 | MuJoCo 3.10.0 | `tools/install_optional.sh sim` |
| `web-sim` | `web_mujoco/.venv` 和该目录的 `node_modules` | 系统 Python、Node.js 22+、浏览器 MuJoCo WASM 3.12.0 | `web_mujoco/requirements.txt`、`web_mujoco/package-lock.json` |

ROS/视觉先装 Conda 的 Pinocchio 原生库，再装 pip 清单，最后以 editable 方式关联同一 SDK 源码。
不使用 `sudo pip`。不要把 SDK 的 MotorBridge 0.5.1 覆盖到遥操作/数采环境，也不要将 SDK 的 NumPy 1.x 约束套给模型环境。
旧 Python venv 类型的 `.venv-ros` 须先按 [SDK 迁移说明](SDK_INSTALL.md#旧包环境迁移)备份，脚本不会覆盖。
已有同名 Conda 环境会被模块安装更新；不要指向其他项目共享且不能变更的环境。

## 1. ROS、SDK、MoveIt 与控制网页

```bash
bash setup.sh ros
```

此入口安装系统 ROS 2 Humble Desktop、MoveIt、rosbridge、joy、ros2_control/controllers、xacro、
robot_state_publisher、接口生成工具、tf_transformations、Node.js、colcon 和所需 Python 系统包。
ROS apt 源引导包固定为 1.3.0；依赖由 apt 清单安装，不执行 rosdep init/update/install。
创建或更新 `.venv-ros`，安装 SDK 依赖和 SDK，再构建以下八个 ROS 包：
`zekeep_msgs`、`zekeepcontroller`、`zekeep_bringup`、`zekeep_moveit_config`、`zekeep_teach`、
`zekeep_joystick`、`zekeep_shadow`、`zekeep_llm`。

网页服务器只使用 Node.js 内置模块，不需要在 `src/zekeep_bringup/web` 中运行 npm install。
`zekeep_shadow` 包编译完成不代表已安装相机环境；视觉跟随仍需 `vision`。
LLM 使用用户自己的 `ZKEEP_LLM_API_KEY` 和服务配置，不随包提供密钥；基础文本控制不要求视觉权重。

安装器执行 `pip check`、SDK 导入/模型检查及 ROS 服务类型导入，不连接设备。
手动验证只启动无硬件演示：

```bash
bash tools/env.sh ros python -B tools/check_sdk.py
bash tools/env.sh ros ros2 launch zekeep_moveit_config demo.launch.py
# 只启动网页及只读 AI 后端，不启动驱动或相机。
bash tools/env.sh ros ros2 launch zekeep_bringup web.launch.py
```

网页地址 `http://127.0.0.1:3001`。真机启动与现场配置见 [驱动](../src/zekeepcontroller/README.md)、
[MoveIt](../src/zekeep_moveit_config/README.md)、[网页](../src/zekeep_bringup/web/README.md)、[LLM](../src/zekeep_llm/README.md)。

## 2. 视觉、相机、SDK 与 YOLOE

GPU 推理先按 [GPU 说明](GPU_SETUP.md)准备驱动。仅导入和 CPU 自检不构成 GPU 验收。

```bash
bash setup.sh vision
# 仅准备依赖、暂不下载 YOLOE/MobileCLIP 时，改用：
# ZKEEP_DOWNLOAD_MODELS=0 bash setup.sh vision
```

此入口独立创建/更新 `rebotarm`，不安装 ROS 或 `lerobot`。
依次安装固定 cu128 PyTorch wheel、相机/Open3D/MotorBridge 等依赖、固定提交的 CLIP、SDK、Ultralytics，
最后重装 contrib OpenCV 并验证 ChArUco。现有视觉依赖清单也保留 FunASR、ModelScope 和 soundfile；
安装这些库不表示提供了语音控制入口，当前终端 LLM 只接受文本。
系统侧安装图形/USB 运行库和 GCC/G++ 11；通过 SDK 附带安装器配置 Orbbec udev 规则。

默认下载和 SHA-256 校验：`src/zekeep_grasp/models/yoloe-26l-seg.pt`、根目录 `mobileclip2_b.ts`，
并建立视觉目录所需的文本权重链接。安装器做 YOLOE CPU 推理、SDK、FK/IK 和依赖检查，不打开相机。
**GraspNet 源码、API、CUDA 扩展和 checkpoint 不在这个入口中安装。**

模型跳过下载或中途失败时，补齐后检查：

```bash
bash tools/download_models.sh base
bash tools/env.sh vision python -B tools/check_yolo.py
bash tools/env.sh vision python -B tools/check_vision_model.py
bash tools/env.sh vision python -m pip check
```

仅 YOLOE 检测通过不能证明抓取后端齐全。相机序列号、内参和手眼另按 [新设备步骤](../README.md#新设备配置与标定)准备。
相机预览会打开相机，属于人工设备验收，不列入安装基准检查。

## 3. GraspNet：按需独立准备

前置：`vision`、NVIDIA 驱动、CUDA Toolkit 12.8.1、GCC/G++ 11、目标 GPU。
阅读并接受上游许可后，按 [third_party/README.md](../third_party/README.md)从官方原址下载：

- baseline 源码放在 `third_party/graspnet-baseline/`。
- API 源码放在 `third_party/graspnet-baseline/graspnetAPI/`。
- `checkpoint-rs.tar` 放在 `third_party/graspnet-baseline/checkpoints/`，不解压。

随后安装 Cython 3.0.12、GraspNet API、编译 pointnet2/knn 扩展，再做后端导入和 checkpoint 哈希检查。
完整命令以第三方说明为准；不能把旧电脑编译的 `.so` 复制来代替编译。
原网页抓取的 `yolo-graspnet` 后端和直接 GraspNet 抓取均需要这一步。

## 4. 独立主从遥操作与舵机工具

```bash
bash setup.sh teleop
bash tools/env.sh teleop python -m pip check
bash tools/env.sh teleop python -c 'from lerobot_teleoperator_zhongling import ZhonglingLeader, ZhonglingLeaderConfig; import servo_gui.server; print("遥操作导入：OK")'
```

此入口只准备 Python 3.12 的 `lerobot` 和本地主臂插件，依赖由 `src/zekeep_teleop/pyproject.toml` 安装。
不要求 ROS、MoveIt、相机、SDK 环境或模型权重。系统需要 ffmpeg、构建工具及串口权限，入口会安装并加入 dialout。
按照 [遥操作 README](../src/zekeep_teleop/README.md)填写主从端口、设备 ID、方向与启动零位；
上面的导入检查不连接设备。实际 `zhongling-teleoperate` 会连接并控制从臂，不能用作安装自检。

## 5. 数采、ACT/π0.5 与仿真

选装入口不执行 sudo；独立安装时先准备该模块的系统依赖。

- `collect`、`models`：`sudo apt-get install -y git ffmpeg build-essential`；部分依赖需联网下载。
- `web-sim`：`sudo apt-get install -y python3-venv python3-pip`，另在当前终端安装并选择 Node.js 22+/npm。
  执行 `node --version`、`npm --version` 确认可用；Ubuntu 22.04 自带 Node.js 可能版本过旧。
- `sim`：先完成 `bash setup.sh ros`；需要离屏渲染时由目标机准备可用 EGL。

按需选择下面的单条命令；它们互不代表已安装其他模块：

```bash
bash tools/install_optional.sh collect  # 数采；实际采集还需 ros + vision
bash tools/install_optional.sh models   # ACT/π0.5；离线训练可独立安装
bash tools/install_optional.sh sim      # 依赖已安装的 ros
bash tools/install_optional.sh web-sim  # 独立网页仿真，无 ROS/Conda 前置
```

`collect` 同时 editable 安装本地主臂插件和数采包，支持源工作区的主臂采集功能；
不要求另外安装整个 `teleop` 环境。相机进程使用 `rebotarm`，ROS 采集/执行端保留 ROS 环境。
`models` 使用 `act` 环境，安装 ACT/π0.5 共用模型依赖，不自动下载用户权重或数据。
ROS MuJoCo 安装 Python `mujoco==3.10.0` 并构建 `zekeep_sim`；
独立网页仿真创建系统 Python venv、`npm ci`、导出模型并执行 `npm run build`。

详细的无设备检查、模型下载、许可与运行命令见 [选装模块说明](OPTIONAL_MODULES.md)。

## 最小基准检查

当前任务只运行源码交付和安装入口的基础检查，不做全量回归、训练或硬件测试。
这里的“基准检查”检查安装入口与包结构，不是 GPU 吞吐量或运动性能测量。

```bash
python3 -B tools/check_delivery.py
# 仅原始桌面交付包有此清单；安装或修改后不再适用。
if [[ -f MANIFEST.sha256 ]]; then sha256sum -c MANIFEST.sha256; fi
```

`check_delivery.py` 检查 Python/Shell/JSON/XML 语法、ROS 包集合、空设备模板、禁带资源，
以及安装入口帮助和非法模块退出行为；不下载依赖、不使用 GPU、不连接设备。
各模块实际安装时仍执行已有的 `pip check` 和导入/模型自检，失败即停止，不跳过验证来报告成功。
这不能代替目标新机的完整安装、GPU 推理、相机出帧或人工监督真机验收。

## 重跑、日志与权限

`setup.sh` 不带参数只显示说明，必须显式选择 `ros`、`vision` 或 `teleop`，不接受多个模块参数。
`bash tools/env.sh base bash setup.sh ros` 与 `bash setup.sh ros` 选择同一模块；`base` 只是干净环境配置。
安装日志为 `log/setup-<模块>-<时间>.log`。失败时先查看日志，修复网络或依赖原因后只重跑失败模块。
选装失败直接显示终端错误；不会生成同名 setup 日志。已有 Conda 环境按清单更新，不自动清理环境或用户数据。

`setup.sh` 会用 sudo 安装系统包、把当前用户加入 dialout；`vision` 另安装 Orbbec USB 规则。
重新登录使串口组权限生效，相机按安装提示拔插。安装不会连接机械臂、使能电机或验证实际运动。
ROS、独立 SDK、遥操作不得同时占用同一机械臂串口。保持交付包现场授权默认关闭；
填写设备标识和安装成功均不代表已完成新现场标定。

## Qt 插件冲突与相机取帧超时

2026-11-05 收到另一台电脑的反馈：Conda 环境包含 `lib/qt6/plugins`，而 pip 安装的 OpenCV 4.7.0.72 使用 Qt5；预览在 `cv2.namedWindow` 卡住，同时出现 `[Camera] 10 consecutive frame failures: wait_for_frames timeout`。
Qt 插件必须匹配运行库的主版本，不能混用 Qt5/Qt6，参见 [Qt 插件部署说明](https://doc.qt.io/qt-6.8/deployment-plugins.html)。仅凭上述相机日志不能证明 Qt 死锁的具体调用链。

交付包的相机预览、手眼采集、直接视觉抓取和视觉跟随入口，现会在创建 OpenCV 窗口前，依据当前 `cv2.__file__` 将本进程的 `QT_PLUGIN_PATH` 和 `QT_QPA_PLATFORM_PLUGIN_PATH` 指向 `cv2/qt/plugins`，覆盖继承的 Conda 插件路径；不修改全局环境或删除 Conda Qt。此处理仅针对带有该插件目录的 Linux OpenCV wheel。

相机超时是独立问题：Qt 路径修正不会恢复相机出帧。预热阶段现检查是否收到完整 RGB-D 帧；全部失败时报告底层原因并退出，不再继续弹窗。直接视觉抓取会在连接及使能机械臂前完成这项检查。
出现超时先用以下相机预览入口排查，退出其他相机客户端，检查 USB 连接/带宽、udev 权限与实际支持的分辨率/帧率。该入口不连接机械臂。

```bash
bash tools/env.sh vision python src/zekeep_grasp/scripts/camera_preview.py
```

本地独立环境对照测试：OpenCV 4.7.0（Qt 5.15.0）在插件变量被指向 Conda Qt6 后无法加载平台插件并中止；应用上述路径修正后，真实窗口创建、显示与关闭通过。无帧测试确认预热失败会退出并释放相机，不进入 `namedWindow`。该测试未连接相机或机械臂，也未复现目标电脑的死锁调用链。

目标电脑仍需验证弹窗和实际相机出帧；模型导入及 CPU 推理通过不能代替这两项验证。若仍卡住，保留触发命令、完整错误输出，并临时添加 `QT_DEBUG_PLUGINS=1` 收集实际插件加载路径；不要将调试变量写入全局环境。

## 新设备初始化

按根目录 [README](../README.md#新设备配置与标定)填写设备身份、读取新相机内参并核验安装。
`camera.serial`、设备 ID、安装标识均为空模板；确认工具不能代替新安装的实际标定。
手眼、颜色文件来自原设备，按交付要求保留。旧设备内参、身份快照和个人数据不随新包分发。
运动前核验 `zekeep_hardware.yaml` 的零位、方向、控制增益和现场几何，再显式打开所需授权。
保持启动不自动使能，不绕过序列号、内参、反馈新鲜度或设备占用检查。

选装数采、ACT、π0.5 和仿真见 [OPTIONAL_MODULES.md](OPTIONAL_MODULES.md)。
