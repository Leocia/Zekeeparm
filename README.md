# Zekeeparm 源码交付包

目标平台：Ubuntu 22.04 x86_64、ROS 2 Humble、NVIDIA GPU。路径与用户名可变。
仅含源码、机器人 URDF/STL 等结构资源，以及指定保留的手眼和颜色标定；不含模型权重、训练数据、密钥、虚拟环境或本机编译产物。

## 首次安装

在实际项目根目录，先校验收到的原始交付包：

```bash
export ZKEEP_WS="$PWD"
sha256sum -c MANIFEST.sha256
python3 -B tools/check_delivery.py
```

清单只针对未安装、未填写设备配置的交付快照；修改或安装后产生差异是正常现象。

若已有 Conda，先指定安装根目录：

```bash
export ZKEEP_MINIFORGE_DIR="$(conda info --base)"
bash tools/env.sh base bash setup.sh
```

没有 Conda 时省略第一行，安装器会安装 Miniforge 到 `~/miniforge3`。
安装需要网络、`sudo apt` 和足够磁盘空间；不会连接设备、使能电机，也不会安装 NVIDIA 驱动或 CUDA。
基础安装不下载模型；GraspNet 源码及原生扩展按安装提示选择，未接受许可时跳过。
GPU 驱动、CUDA 12.8 和具体安装边界见 [安装说明](docs/INSTALL_PORTABLE.md)。

## 整套环境的安装顺序

先完成基础 `setup.sh`，它已安装 ROS/SDK、视觉和独立遥操作；模块内的手动安装示例用于分步部署，不必重复执行。
各终端从实际项目根目录运行命令，已有 Conda 时设置 `ZKEEP_MINIFORGE_DIR`，通过 `tools/env.sh` 选择环境。

| 环境 | Python | 安装方式与用途 |
| --- | --- | --- |
| `.venv-ros` | 3.10 | 基础安装；ROS、SDK、MoveIt、网页后端 |
| Conda `rebotarm` | 3.10 | 基础安装；相机、视觉推理、直接 SDK 抓取 |
| Conda `lerobot` | 3.12 | 基础安装；独立主从遥操作 |
| Conda `embodied` | 3.10 | `collect` 选装；统一数采 |
| Conda `act` | 3.10 | `models` 选装；ACT 与 π0.5 共用模型环境 |
| `web_mujoco/.venv` | 系统 Python | `web-sim` 选装；独立网页仿真的模型导出 |

需要完整功能时，基础安装后执行以下选装依赖入口；独立网页仿真须先在当前终端准备 Node.js 22+ 和 npm：

```bash
bash tools/install_optional.sh collect
bash tools/install_optional.sh models
bash tools/install_optional.sh sim      # MuJoCo 加入现有 ROS venv
bash tools/install_optional.sh web-sim
```

随后按本文下载视觉模型、配置新设备并核验标定；ACT/π0.5 权重和训练数据另行准备。
先使用无硬件 MoveIt 演示验证 ROS，再做现场设备验证。源码交付包不附开发回归测试，安装自检和运行时安全检查保留。
现有环境中的构建、自检和 CPU 基准已验证；全新 Ubuntu 安装、目标 GPU/CUDA 和新设备标定仍需在目标机验收。

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

1. **读取并自动填写相机序列号。** 先完成基础安装，重新登录使串口组权限生效，相机按安装提示拔插一次。
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

## 模型按需下载

```bash
# YOLOE 与 MobileCLIP：带 SHA-256 校验，可重复执行。
bash tools/download_models.sh base
# 先阅读许可，并在 setup.sh 中选择安装 GraspNet 依赖后：
ACCEPT_GRASPNET_LICENSE=YES bash tools/download_models.sh graspnet
```

π0.5 基础权重、tokenizer、ACT 用户模型及训练结果按[选装说明](docs/OPTIONAL_MODULES.md)单独准备。
也可在首次安装时显式设置 `ZKEEP_DOWNLOAD_MODELS=1` 下载所选视觉模块权重。
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
