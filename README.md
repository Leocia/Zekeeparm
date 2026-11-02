# Zekeeparm 源码交付包

目标平台：Ubuntu 22.04 x86_64、ROS 2 Humble、NVIDIA GPU。路径与用户名可变。
仅含源码、机器人 URDF/STL 等结构资源，以及指定保留的手眼和颜色标定；不含模型权重、训练数据、密钥、虚拟环境或本机编译产物。

## 首次安装

在实际项目根目录运行。若已有 Conda，先指定安装根目录：

```bash
export ZKEEP_MINIFORGE_DIR="$(conda info --base)"
bash tools/env.sh base bash setup.sh
```

没有 Conda 时省略第一行，安装器会安装 Miniforge 到 `~/miniforge3`。
安装需要网络、`sudo apt` 和足够磁盘空间；不会连接设备、使能电机，也不会安装 NVIDIA 驱动或 CUDA。
基础安装不下载模型；GraspNet 源码及原生扩展按安装提示选择，未接受许可时跳过。
GPU 驱动、CUDA 12.8 和具体安装边界见 [安装说明](docs/INSTALL_PORTABLE.md)。

校验收到的原始交付包：

```bash
sha256sum -c MANIFEST.sha256
python3 -B tools/check_delivery.py
```

清单只针对未安装、未填写设备配置的交付快照；修改或安装后产生差异是正常现象。

## 新设备配置与标定

本次部署使用同型号的新机械臂和新相机，**原设备身份不沿用**。

1. 在 `src/zekeep_grasp/config/default.yaml` 填写新的 `camera.serial`、`robot.id`、`calibration.installation_id`。
2. 核对串口、方向、零位、夹爪行程、TCP 和桌面几何。硬件 YAML 的运动授权与监督重力试验授权默认关闭，配置中的经验力矩修正恢复为 1；现场核验后再明确开启。
3. 只接相机，通过 SDK 保存新设备当前图像配置的内参：

   ```bash
   bash tools/env.sh vision python src/zekeep_grasp/scripts/read_camera_serial.py
   bash tools/env.sh vision python src/zekeep_grasp/scripts/save_orbbec_intrinsics.py
   ```

4. 手眼与颜色标定按要求保留，**保留文件不代表新安装已经验证**。新机安装位置、曝光或光照变化时先核验，必要时重新标定。
5. 若使用遥操作，重新填写主从设备 ID 并完成设备零位标定；若使用 ACT/VLA，选择新的全局相机路径并填写 `record.yaml` 的安装标识和工作区域，核验后将 `workspace.confirmed` 设为 `true`。

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
| 积木编程 Studio（独立提供） | `apps/studio/`，不随 GitHub 仓库发布 |
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

环境入口：`bash tools/env.sh ros|vision|teleop|vla|model|act|studio [命令]`。
`vla` 用于数采，`model` 与 `act` 均选择共用 LeRobot `act` 环境。完整安装与操作见各模块 README。

接口见 [ROS API](API_zh.md)，第三方来源见 [许可说明](docs/THIRD_PARTY_NOTICES.md)。
