# π0.5 红色方块抓放入口

统一入口是工作区根目录的 `tools/pi05.sh`。目标是同一个红色方块在小范围内改变起点，
抓起后放入固定托盘。当前先交付可运行的采集、训练和运行入口，数据之后准备，不设置成功率门槛。

便携包不带示范、导出、统计、缓存、官方基础权重或旧适配器；先按安装说明准备。
下面使用新的数据集 `local/zekeep_red_cube_v1` 和实验 `red_cube_v1`。更换数据或相机安装时换一个数据集名；
每轮训练换一个实验名，避免覆盖已有结果。

## 先检查环境

所有示例从工作区根目录运行：

```bash
cd "$ZKEEP_WS"
./tools/pi05.sh help
./tools/pi05.sh doctor
```

`help` 不依赖模型环境。`doctor` 检查解释器、ROS 安装、转换后的基础权重和 GPU，
不加载模型、不打开相机、不连接机械臂。环境缺失时按[安装说明](../../../docs/PI05_INSTALL.md)
和 [π0.5 环境说明](pi05.md)准备，不向 ROS 或采集环境混装模型依赖。

脚本分别复用采集环境、OpenPI Python 3.11 环境和 ROS 环境。模型路径默认是
`~/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch/model.safetensors`。
首次准备需要完成官方权重下载、PyTorch 转换和 QLoRA 依赖安装，具体命令见环境说明。
数据集缓存默认写入 `data/openpi/cache/huggingface_datasets`；新电脑需自行规划 `data` 的磁盘空间。

## 采集新的示范

先核对 `src/zekeep_embodied/configs/record.yaml` 的相机设备和 `installation_id`。
当前任务指令为“把红色方块放入托盘”。相机移动或设备布局变化后更新安装标识，重新预检。
机械臂标定仍来自现有硬件配置，不在入口中另设零位或修改方向。

只检查相机：

```bash
./tools/pi05.sh camera-check --seconds 10
```

### 主臂遥操采集

使用已有 Zhongling 主臂带动从臂，直接录制双相机、实际状态和下发目标：

```bash
./tools/pi05.sh record-leader --output "$PWD/data/pi05_red_cube_v1/raw"
```

该命令直接连接主从臂串口并使能从臂，不需要游戏手柄、蓝牙或 ROS 驱动。
启动前停止占用同一从臂的 ROS 驱动、手柄控制、模型执行及其他 SDK 程序；
核对 `src/zekeep_teleop/configs/zhongling_b601_teleop.yaml` 中的两个串口，
并按[主臂遥操说明](../../zekeep_teleop/README.md)检查启动零位和主臂通信。
原有方向、比例、限位和启动零位检查均复用已有实现，不会写硬件零位。

连接后从臂就会跟随主臂；`s` 只控制是否开始录制，不是运动许可开关。
退出、Ctrl+C 或异常会释放从臂力矩，退出前先扶稳机械臂。此模式不使用游戏手柄的 LB/deadman。

### 游戏手柄采集

此方式与主臂遥操二选一。分开三个终端，每个终端先进入工作区：

```bash
# 终端 1：真机驱动，默认不使能电机。
./tools/pi05.sh driver

# 终端 2：手柄控制。现场检查机械臂和急停后，手动使能，再启动手柄。
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
./tools/pi05.sh joystick

# 终端 3：只读采集，不使能电机、不发布运动命令。
./tools/pi05.sh record --output "$PWD/data/pi05_red_cube_v1/raw"
```

录制命令输入后按回车：`s` 开始，`p` 成功保存，`w` 失败保存，`d` 放弃，`e` 退出。
每条示范包含接近、夹紧、抬起、搬运、放下、松爪、撤离。先录少量并回看，
确认状态、动作和双相机完整，再补充不同起点的示范。失败记录保留，但不会默认导出训练。

将实际保存的 episode 路径填入以下命令：

```bash
./tools/pi05.sh check-raw data/pi05_red_cube_v1/raw/episode_实际名称
./tools/pi05.sh preview-raw data/pi05_red_cube_v1/raw/episode_实际名称 \
  --output data/pi05_red_cube_v1/review.mp4
```

## 导出并训练

正式训练前按完整 episode 分出训练集和验证集。可在 `data/pi05_red_cube_v1/train` 和
`data/pi05_red_cube_v1/validation` 下分别放指向原始 episode 的符号链接；使用绝对目标路径。
同一 episode 不能同时进入两个集合。下面假定这两个目录已由你按新的示范准备好。

```bash
./tools/pi05.sh export data/pi05_red_cube_v1/train \
  --repo-id local/zekeep_red_cube_v1 --fps 9
./tools/pi05.sh check --repo-id local/zekeep_red_cube_v1
./tools/pi05.sh stats --repo-id local/zekeep_red_cube_v1
./tools/pi05.sh check --repo-id local/zekeep_red_cube_v1

# 验证集单独导出，不对它计算训练归一化统计。
./tools/pi05.sh export data/pi05_red_cube_v1/validation \
  --repo-id local/zekeep_red_cube_val_v1 --fps 9

# 先验证一步训练能运行，再开始独立实验。
./tools/pi05.sh train --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_smoke_v1 --num-train-steps 1
./tools/pi05.sh train --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_v1 --num-train-steps 1000
```

`--fps 9` 用于当前双相机采集配置；如果新相机实测更慢，应降低导出频率，不能插值或重复观测。
首次 `check` 可以报告 `normalized: false`，`stats` 后应为 `true`。没有新数据时，命令会明确报数据集未准备。
统计只用训练集；模拟数据不能用于正式统计和训练。示例中的 1,000 步用于开始实验，不保证任务已经学会。

当前入口使用本机 NF4 QLoRA 实验实现，冻结基座、训练低秩参数和动作投影。
结果位于 `data/openpi/checkpoints/pi05_zekeep_lora/实验名/`。数据导出和训练结果默认拒绝覆盖。

## 离线检查模型预测

```bash
./tools/pi05.sh predict --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_v1 --frame-index 0 \
  --output data/pi05_red_cube_v1/offline_prediction.json
```

此命令使用所选训练数据中的双相机、状态和任务指令，复用策略服务的加载和归一化逻辑，
保存 10 步、每步 7 维的绝对电机角度（度）。它不启动 ROS，不连接相机或电机。
输出必须是新文件，父目录必须存在。该输出用于检查动作数值，不能直接作为实机轨迹执行，
也不是抓放成功率。首次读取数据可能生成较大的缓存。

## 影子模式与预测回放

分开终端启动策略服务和读取端。保留一个驱动进程；如果驱动已运行，不要重复启动。

```bash
# 终端 1：模型服务，仅监听 127.0.0.1:8000。
./tools/pi05.sh serve --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1

# 终端 2：已有驱动时跳过；默认不使能电机。
./tools/pi05.sh driver

# 终端 3：双相机和关节反馈进入模型，只打印预测，不发布运动指令。
./tools/pi05.sh shadow --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1
```

读取端会核对服务中的数据集名和实验名，错误模型不会进入运行。数值限位通过不代表碰撞检查通过。
采集和影子模式共用相机，不能同时占用。

本机 16 GB 内存建议先保存预测，停止模型和相机，再打开 RViz：

```bash
./tools/pi05.sh shadow --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1 \
  --save-preview data/pi05_red_cube_v1/live_preview.json
# 保存端退出后，在策略服务终端按 Ctrl+C 释放模型。

# 两个终端分别启动，只显示已保存预测，不连接实机：
./tools/pi05.sh rviz
./tools/pi05.sh replay data/pi05_red_cube_v1/live_preview.json
```

`replay` 使用 `shadow --save-preview` 保存的显示轨迹，不接受 `predict` 的数值 JSON。
回放不会模拟物体接触或验证抓取成功。

## 现场监督执行

完成影子检查后，现场确认物体、托盘、线缆和急停。停止采集程序和 `joystick` 控制终端，
避免手柄控制节点与策略节点争用伺服。随后仅启动 `joy`，继续提供 LB/deadman 按钮状态：

```bash
# 终端 1：保留或重新启动 serve；终端 2：保留唯一的 driver。
./tools/pi05.sh joy

# 另一个终端：现场手动使能电机后，再运行策略。
./tools/pi05.sh execute --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1
```

`execute` 必须在交互终端输入 `MOVE`，先松开再按住 LB。松开 LB、按钮超时、反馈过期、
推理超时、图像不同步或动作越界时停止伺服。夹爪操作后要松开再按 LB。
入口不自动使能、不自动回零、不自动复位物体；现有受监督执行器不提供环境碰撞或自碰撞检查。
详细行为、速度限制和标定语义见 [π0.5 接入说明](pi05.md)。
