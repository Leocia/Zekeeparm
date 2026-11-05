# zekeeparm_SDK

Zekeeparm 机械臂 Python 控制 SDK，包含电机分组控制、正逆运动学、动力学、末端控制和轨迹规划。

先按 [SDK 安装说明](../docs/SDK_INSTALL.md)准备所选环境的 NumPy、Pinocchio 等依赖。
SDK 必须安装 MotorBridge；`--no-deps` 不会自动补装依赖。以下命令均从工作区根目录执行。

两套 SDK 环境统一使用 Python 3.10、NumPy 1.26.4、Pinocchio 3.9.0、MotorBridge 0.5.1。
先从 Conda 安装 Pinocchio 与 NumPy，再安装 pip 依赖和本 SDK。SDK 的依赖声明固定 NumPy 1.26.4 和 MotorBridge 0.5.1，避免普通 pip 安装静默升级到未验证的组合。

ROS 驱动环境（先按 `tools/environment-ros.yml` 创建 `.venv-ros`，该目录现在由 Conda 管理）：

```bash
bash tools/env.sh ros python -m pip install motorbridge==0.5.1
bash tools/env.sh ros python -m pip install --no-deps -e ./zekeeparm_SDK
```

视觉抓取环境（先完成 Conda `rebotarm` 环境的创建）：

```bash
bash tools/env.sh vision python -m pip install -c tools/constraints-vision.txt motorbridge==0.5.1
bash tools/env.sh vision python -m pip install --no-deps -e ./zekeeparm_SDK
```

两套环境分别安装依赖。SDK 源码保留在项目的 `zekeeparm_SDK/` 中，`pip install -e` 将这份源码关联到所选 Python 环境，不复制或重新下载一份 SDK 源码。
基础 `setup.sh` 已通过 ROS 依赖清单和视觉环境清单安装 MotorBridge 0.5.1；上面的显式命令也可用于补装遗漏的依赖。
旧包的 ROS venv 不要直接强降 NumPy，先按 [旧环境迁移步骤](../docs/SDK_INSTALL.md#旧包环境迁移)备份并重建。
独立遥操作和数采环境使用 MotorBridge 0.4.9，不安装本 SDK，见 [安装说明](../docs/SDK_INSTALL.md#独立遥操作环境)。

唯一默认硬件配置为 `config/rebotarm_dm.yaml`（达妙电机）。
SDK 模型 `../src/zekeep_grasp/config/sixaxis.urdf` 与 ROS 主模型的六轴关节和 TCP 保持一致；
SDK 模型将两根夹指关节固定，ROS 模型保留其平移自由度，两份 URDF 并非逐字相同。
保留 `official_tcp` 坐标系兼容现有位姿命令；夹爪目标限制为 `0–1.35 rad`。
线性映射基准为 `1.45 rad = 70 mm`，最大允许开口约 `65.17 mm`。
ROS 控制器仍可传入已合并的运行时硬件配置。

```python
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
```

包名由 `reBotArm_control_py` 改为 `zekeeparm_SDK`，控制类名保持不变。
完整安装说明见 [SDK 安装说明](../docs/SDK_INSTALL.md)。
