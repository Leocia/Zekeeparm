# Zekeep 示范数据采集

本地采集双相机图像、机械臂状态和动作，检查后打包上传云端训练。本地采集不需要 π0.5 权重。
支持主从臂遥操作和 ROS 手柄；默认 Orbbec 腕部相机 + C100 全局相机。

## 首次安装

先按[安装说明](../../docs/INSTALL_PORTABLE.md)准备独立的 `rebotarm` 相机环境，不要把 Orbbec SDK 装进 `embodied`。
在工作区根目录执行：

```bash
conda create -n embodied python=3.10 -y
conda activate embodied
export EMBODIED_PYTHON="$CONDA_PREFIX/bin/python"
bash src/zekeep_embodied/tools/run.sh -m pip install \
  -r src/zekeep_embodied/requirements-embodied.txt -e src/zekeep_teleop -e src/zekeep_embodied
bash src/zekeep_embodied/tools/run.sh -m pip check
```

默认解释器为 `~/miniforge3/envs/embodied/bin/python`；其他位置需设置 `EMBODIED_PYTHON`。
检查 [record.yaml](configs/record.yaml) 中的任务、相机设备和相机 Python 路径。
相机位置或设备变化后，更新 `installation_id`，重新检查画面；不同布局数据不要直接混用。

## ACT、π0.5 共用采集入口

以下命令均在工作区根目录执行。采集配置统一放在 [record.yaml](configs/record.yaml)：
任务描述 `task`、安装布局 `installation_id`、双相机、采样频率、ROS 标定、现场范围 `workspace`、
主臂保护 `leader_control`。现场重新核对范围后再设置 `workspace.confirmed: true`；迁移的配置值不代表新现场已确认。
采集不加载 ACT 或 π0.5 模型。先检查环境和相机：

```bash
./tools/collect.sh doctor
./tools/collect.sh camera-check
```

相机检查不连接机械臂。关闭其他占用相机的程序，查看输出图片的清晰度、视野和遮挡。
选择一种示教方式；同一批原始示范可分别用于两个模型，不需要为 ACT 和 π0.5 重复采集。
不同示教方式、相机布局或标定使用不同原始目录。

### 游戏手柄

按[手柄采集步骤](docs/pi05-entry.md#游戏手柄采集)启动唯一 ROS 驱动、MoveIt 和手柄，现场手动使能后运行：

```bash
./tools/collect.sh record --output "$PWD/data/demonstrations_v1/raw" --episodes 50
```

`record` 只读取现有 ROS 反馈和动作目标，不连接主臂，不使能或控制从臂。
输入后回车：`s` 开始、`p` 成功保存、`w` 失败保存、`d` 放弃、`e` 退出。
保存和退出只停止记录；先松开 LB，再保存或退出。
`--episodes` 是该目录累计成功示范数，失败数据不计数；默认达到 50 条成功示范后退出。

### 主从示教

采用同一 ROS 驱动和原有主臂标定。先停止手柄、SDK 遥操和模型控制，保留唯一 ROS 驱动，
核对串口、标定、主臂启动零位和从臂初始姿态。详见[主臂步骤](docs/pi05-entry.md#主臂遥操采集)。

```bash
./tools/collect.sh record-leader --output "$PWD/data/leader_demonstrations_v1/raw" --episodes 50
```

采集器不自动使能、不自动对齐。现场手动使能后，输入 `f`＋回车核对姿态；各轴差异不超过 5° 才开始跟随。
输入 `s` 开始录制，`p` 成功保存并暂停，`w` 失败保存并暂停，`h` 暂停，`d` 放弃并暂停，`e` 退出。
下条示范需再次输入 `f`。退出或异常会停止跟随，由 ROS 驱动尝试持位；不要关闭负责持力的驱动。
本入口不提供环境碰撞或自碰撞检查；首次现场监督、小幅验证。

旧 `act.sh record/record-leader` 通过 ACT 配置调用同一采集实现，保留原参数和默认目录；
旧 `pi05.sh` 的采集命令转发至 `collect.sh`。**π0.5 旧主从入口也改走 ROS，不再直连 SDK 自动使能。**
推荐新流程显式指定原始目录。可通过 `--config FILE.yaml` 选择另一份采集配置。

## 共用检查与原始数据

先录几条检查，再批量采集。输出文件须使用新名称。

```bash
./tools/collect.sh check-raw data/demonstrations_v1/raw/episode_实际目录
./tools/collect.sh preview-raw data/demonstrations_v1/raw/episode_实际目录 --output /tmp/review.mp4
./tools/collect.sh audit data/demonstrations_v1/raw --output data/demonstrations_v1/audit.json
```

原始目录保持 `episode_.../episode.json`、`frames.jsonl`、`wrist/` 和 `global/`。
每帧保存双 RGB、7 维状态与 7 维动作目标、时间戳；任务文字保存在 `episode.json` 的 `task`，
两个导出器均转为 LeRobot 任务字段。π0.5 用该文字作为语言条件，ACT 不依赖语言编码。
动作是完成标定映射和限位后的从臂电机绝对角度，单位度；不要再应用一次主从方向映射。

新示范记录通用 `capture_fingerprint`，同时保留同值 `act_fingerprint` 兼容旧消费者；
ACT 处理仍接受只有旧指纹的已有数据，两个指纹同时存在时必须一致且匹配当前配置。
原有数据不会自动补写指纹；早期不带指纹的数据仍须核对来源，不能绕过 ACT 配置校验。
失败、放弃、中止和模拟数据默认不进入真实训练集。`data/` 不随 Git 迁移，需单独备份。
图像按主机接收时间对齐，不是硬件同步；回看视频不能证明真实采集时序或反馈新鲜度。

## 分别处理：ACT 与 π0.5

保持原始目录不变，分别生成新的模型数据目录。模型处理配置、图像预处理和数据格式各自保留：

| 阶段 | ACT | π0.5（LeRobot 原生实现） |
|---|---|---|
| 原始输入 | 同一批成功的 `episode_...` | 同一批成功的 `episode_...` |
| 划分 | 默认 50 条成功示范按完整 episode 分为 45/5，写入 `split.json` | 先按完整 episode 划分 train/validation，见云端流程 |
| 导出 | LeRobot v3；按 ACT 配置重采样与缩放补边 | 同一 LeRobot v3；可直接读取 ACT 的 train 目录 |
| 统计 | ACT 训练仅使用训练集统计 | 对 π0.5 训练集单独计算统计；验证集不参与 |

ACT 示例（默认要求 50 条成功示范；其他数量修改 `act.yaml` 的 `episodes/validation_episodes`）：

```bash
./tools/act.sh prepare --raw data/demonstrations_v1/raw --output data/demonstrations_v1/act_dataset
./tools/act.sh train --dataset data/demonstrations_v1/act_dataset --output data/demonstrations_v1/act_run_001
```

π0.5 可以直接使用上面已经导出的同一训练集，只分别准备模型需要的统计与配置：

```bash
./tools/pi05-model.sh stats --repo-id local/shared_v1_train --dataset-root data/demonstrations_v1/act_dataset/train
./tools/pi05-model.sh train --repo-id local/shared_v1_train --dataset-root data/demonstrations_v1/act_dataset/train   --exp-name shared_v1_pi05 --num-train-steps 1000
```

ACT 与 π0.5 共用这一次完整 episode 划分。π0.5 的六轴相对动作和分位数归一化在读取时处理，
原始及 v3 文件保留绝对电机目标。模型环境见[共用安装与迁移说明](docs/pi05.md)。
需要从其他原始子集单独导出时仍使用 `pi05-model.sh export`，格式同为 v3；不要覆盖已有模型数据。

## 云端训练

入口：`./tools/pi05-model.sh`。默认 **VLM LoRA + Action Expert 全量微调**。
当前仅在训练结束时保存，不支持断点恢复或加载上次微调结果继续训练。

- [数据上传、转换与训练](docs/cloud-data.md)
- [模型环境安装](docs/pi05.md)
- [离线预测、shadow 与实机验证](docs/pi05-entry.md)
- [相机配置与预检](docs/pi05-entry.md#相机检查)

ACT 和 π0.5 的模型执行统一使用本机 X11 桌面全局 `m` 键：输入 `MOVE` 确认后，按住即启动、松开即暂停，
停止后需松开再按。无需输入 `s` 或启动 `joy_node`；采集仍使用原有手柄按键。
切换到浏览器或编辑器后 `m` 仍生效，打字也可能触发运动。无桌面 SSH 和 Wayland 不支持；
键盘采样中断或超时会停止运动，仍须现场监督。完整命令见[ACT](../zekeep_act/README.md)和[π0.5 执行说明](docs/pi05-entry.md#现场监督执行)。

所有导出器统一输出 LeRobot v3；ACT 和 π0.5 共用 LeRobot 0.4.4 模型环境。
动作是从臂电机坐标的 7 维绝对目标，单位度；模型执行时不要再次应用主从臂方向映射。
