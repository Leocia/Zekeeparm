# Zekeep 示范数据采集器

在 `embodied`（Python 3.10）环境运行，支持 Zhongling 主从臂或 ROS 手柄示范采集。
当前配置启用 Orbbec 腕部相机与 WHEELTEC C100 全局相机。
采集器不绑定 ACT；导出的 LeRobot 数据也可适配 π0.5。

π0.5 独立环境、兼容数据导出与训练入口见 [π0.5 接入说明](docs/pi05.md)。
新数据的完整流程统一使用工作区根目录 `./tools/pi05.sh`，从 `help`、`doctor` 开始；
逐步命令见 [π0.5 红色方块抓放入口](docs/pi05-entry.md)。模型步骤必须显式选择数据集和实验。
原 `dataset export` 仍输出 LeRobot v3；openpi 的 v2.1 兼容导出使用单独入口。

## 已实现

- 手动开始、成功保存、失败保存、放弃和退出。
- 记录从臂实际六关节与夹爪状态，以及映射、限幅后实际下发的目标。
- 一路或双路 RGB；保存各路主机接收时间、序号、观测与动作时间。
- 有界后台写盘；异常与放弃数据保留在 `.partial` 目录，不覆盖旧记录。
- 图像、数值、时序、动作限位检查；生成并排回看视频。
- 离线导出 LeRobot v3 视频数据集，并实际加载、解码首个样本。
- 无硬件模拟模式，不连接相机、主臂、从臂，也不使能电机。

## 1. 环境

新电脑需独立创建采集环境，并先按[安装说明](../../docs/PI05_INSTALL.md)准备
`rebotarm` 相机环境。Orbbec 驱动使用独立 `rebotarm` 进程：
`pyorbbecsdk2==2.1.1` 依赖 NumPy 1 / PyAV 12，与 LeRobot 0.4.4 的依赖冲突，
所以通过独立相机进程传帧，不向 `embodied` 混装该 SDK。

设置 `ZKEEP_WS` 为实际工作区绝对路径。首次安装：

```bash
conda create -n embodied python=3.10 -y
conda activate embodied
export EMBODIED_PYTHON="$CONDA_PREFIX/bin/python"
cd "$ZKEEP_WS/src/zekeep_embodied"
bash tools/run.sh -m pip install -r requirements-embodied.txt -e ../zekeep_teleop -e .
bash tools/run.sh -m pip check
```

后续终端激活 `embodied` 后同样设置 `EMBODIED_PYTHON="$CONDA_PREFIX/bin/python"`，
进入本包目录。命令统一通过 `tools/run.sh` 清除 ROS `PYTHONPATH` 和用户包污染。
默认采集解释器为 `${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/embodied/bin/python`，
也可用 `EMBODIED_PYTHON` 显式覆盖。
腕部相机解释器可用 `ZKEEP_VISION_PYTHON` 覆盖，否则使用 Miniforge 的 `rebotarm` 环境；
全局相机的 `/dev/v4l/by-id/...` 应按新电脑连接结果核对。
本包通过 `COLCON_IGNORE` 排除在 ROS 构建之外。

不需要下载 π0.5 权重，也不需要 GPU 推理。现有 Torch/CUDA 组合仅用于当前数据处理，
不代表已验证 RTX 5060 上的模型训练或推理兼容性。

## 2. 先运行无硬件模拟

```bash
bash tools/run.sh -m zekeep_embodied.record \
  --config configs/record.yaml --simulate --frames 40 \
  --output /tmp/zekeep_capture_demo
```

终端会输出新建 episode 的实际路径。模拟数据标记为 `simulated: true`，
默认禁止导入正式训练集。模拟只验证采集文件流程，不能验证真实相机和电机。

## 3. 检查相机，不连接机械臂

先关闭其他占用 Orbbec 的程序：

```bash
bash tools/run.sh -m zekeep_embodied.record \
  --config configs/record.yaml --camera-check --seconds 10
```

命令只打开配置中的相机，报告实测帧率、最大接收时间差和重复帧次数，并将检查图片与
`camera_check.json` 存入独立的 `camera_check_*` 文件夹。打开图片检查视野、清晰度与遮挡。
腕部相机复用 `zekeep_grasp/config/default.yaml` 的分辨率与 Viewer preset。

全局相机采用 C100 的稳定 `/dev/v4l/by-id/...-video-index0` 路径，配置为
1280×720、MJPEG、请求 30 FPS。驱动报告的 30 FPS 不代表实测帧率；本机当前实测约 15 FPS，
因此采集频率设为 10 Hz，避免把相同图像重复记录。模式不被设备接受时会明确报错。
两路帧以主机接收时间进行软件对齐，默认容差 100 ms，不是曝光级硬件同步。
更换安装位置、分辨率或设备后，修改 `installation_id` 并重新进行预检。
详细接入信息见 [C100 接入说明](docs/c100-global-camera.md)。
不要把腕部图片复制成全局图片；不同布局数据不能直接混合导出。

## 4. 使用 ROS 手柄录制真机示范

先分别启动真机驱动和 `zekeep_joystick`，手动使能并确认手柄控制正常。第三个终端启动
只读采集器；它监听 `/zekeep/joint_states`、`/zekeep/gripper/state`、
`/zekeep/servo_joint_trajectory` 和 `/joy`，不会使能或控制机械臂：

```bash
conda activate embodied
export EMBODIED_PYTHON="$CONDA_PREFIX/bin/python"
cd "$ZKEEP_WS/src/zekeep_embodied"
bash tools/run_ros.sh -m zekeep_embodied.record \
  --config configs/record.yaml --joystick
```

手柄控制可以使用关节或末端笛卡尔入口，但只能启动其中一个。采集命令使用
`s`、`p`、`w`、`d`、`e`，输入后按回车。状态和动作会按当前硬件标定转换为现有数据契约：
七维电机坐标绝对目标，单位度；A 记录闭合目标，B 记录打开目标。不要同时运行
`--execute` 主从臂采集器，它会尝试直接占用同一从臂。

`tools/run_ros.sh` 只为采集进程组合 ROS 消息包与 `embodied` Python 环境；普通离线处理仍用
`tools/run.sh`。修改消息包后先在工作区重新构建。

## 5. 人工监督下录制主从臂示范

**该命令会连接并使能从臂。** 先检查原遥操作配置中的两个串口、主臂启动零位、
从臂初始姿态、线缆和急停。ROS 驱动、SDK 抓取程序与其他遥操作程序不能同时占用从臂串口。

统一入口（在工作区根目录运行）：

```bash
./tools/pi05.sh record-leader --output "$PWD/data/pi05_red_cube_v1/raw"
```

采集入口不会执行写硬件零位的自动校准。原有方向、零位和限位来自：
`../zekeep_teleop/configs/zhongling_b601_teleop.yaml`。

```bash
bash tools/run.sh -m zekeep_embodied.record \
  --config configs/record.yaml --execute
```

连接后进入遥操作，未输入 `s` 时仍会跟随主臂，但不录制。
每个命令输入后按回车：

| 命令 | 行为 |
|---|---|
| `s` | 开始一条示范 |
| `p` | 标记成功并保存，继续遥操作 |
| `w` | 标记错误并保存，默认不导出训练 |
| `d` | 放弃当前示范，保留原始文件用于排错 |
| `e` | 退出，未结束示范标记 aborted |

旧命令 `start`、`save`、`fail`、`discard`、`q` 仍可使用。

退出、Ctrl+C 或异常时沿用原遥操作的断开方式，**会释放从臂力矩，请先扶稳机械臂**。
该监督采集入口不等于具备受控保持功能的自主策略执行器。

建议一条示范覆盖：接近、夹紧、抬起、搬运、放下、松爪和撤离。
初期先录几条并检查，不直接录上百条。

### 控制参数说明

- 默认采集控制频率 10 Hz，适配当前 C100 实测约 15 FPS；这是目标值，实际频率需测量。
- `max_target_step_deg: 2.0` 限制每次目标相对反馈的变化，首轮低速验证后再按需求调整。
- 复用原配置 `pos_vel_velocity`，正确从度/秒转为弧度/秒。
- 第三方从臂插件 1.0.0 的原 `send_action` 把手臂速度写死为 32 rad/s，
  本采集入口使用经校验的本地动作发送逻辑绕开该问题；原插件和遥操作入口没有改动。
- 状态缺失、数值异常、总线异常、旧图像、重复帧或写盘队列满都会中断本次采集。
- 保存时会等待后台写盘，此期间暂停下发新目标；完成后恢复遥操作，单步限制仍生效。

## 6. 检查与回看

把下面路径替换为命令实际输出的 episode 路径：

```bash
bash tools/run.sh -m zekeep_embodied.dataset check /实际路径/episode_...
bash tools/run.sh -m zekeep_embodied.dataset preview /实际路径/episode_... \
  --output /实际路径/review.mp4
```

`preview` 不覆盖已有视频。双相机时并排显示，单相机只显示现有一路。
视频按标称 FPS 播放；真实时间戳、状态和动作保留在 `frames.jsonl`，不以回看视频推断实际时序。

原始目录：

```text
data/embodied/episode_时间_唯一编号/
  episode.json       # 成功状态、任务、相机与硬件配置、单位、安装版本
  frames.jsonl       # state/action、时间戳与相机序号
  wrist/000000.png   # 无损 BGR 经 OpenCV 保存为正常 PNG
  global/000000.png  # 仅配置全局相机时存在
```

保存为独立 episode，不自动上传 Hugging Face。
中断或放弃目录以 `.partial` 结尾，不进入正常导出。
PNG 原始文件占用较大，应监控磁盘空间；先验证流程再考虑压缩优化。

## 7. 导出 LeRobot

```bash
bash tools/run.sh -m zekeep_embodied.dataset export \
  "$ZKEEP_WS/data/embodied" \
  --output "$ZKEEP_WS/data/lerobot_pick_place_v1" \
  --repo-id local/zekeep_pick_place
```

默认仅导出成功、非模拟、完整的 episode。失败与放弃的数据仍保留在原目录。
输出路径必须不存在；导出中间产物留在 `.partial`，成功验证后发布到最终路径。

导出使用最邻近的完整样本重采样到固定 FPS，不插值、不重复观测。
采集间隙过大或会重复样本时拒绝导出，应检查性能，或明确选择更低的 `--fps`。
`capture_provenance.json` 保留源 episode、重采样索引和数据语义。

只验证模拟链路时可使用 `--allow-simulated`，并使用独立输出目录：

```bash
bash tools/run.sh -m zekeep_embodied.dataset export /tmp/zekeep_capture_demo \
  --output /tmp/zekeep_lerobot_demo --allow-simulated
```

导出字段：

- `observation.images.wrist`，可选 `observation.images.global`：RGB。
- `observation.state`：从臂电机坐标中的 7 维真实位置，单位度。
- `action`：从臂电机坐标中的 7 维绝对目标，单位度，已执行方向映射与限幅。
- LeRobot 自动生成的 episode、frame、timestamp、task 等元数据。

后续执行模型动作时，**不能再次应用主臂到从臂的方向映射**。
π0.5 的字段映射和动作维度处理由独立适配入口提供。
正式归一化统计仍需在真实训练集到达后计算；不要使用验证集或模拟数据代替。

## 8. 已知限制与验收边界

- 时间戳是主机接收/查询时间，不是相机曝光时间，也不是硬件同步。
- motorbridge 0.4.9 的 `MotorState` 不提供每个电机的反馈接收时间或序号。
  本采集器显式请求反馈并拒绝总线错误、缺失状态和故障状态，但不能证明缓存反馈一定是本轮新报文。
  该限制记录在每条 episode 中；正式批量采集前需要断连与时序实测，严格新鲜度保证需扩展 SDK。
- 不自动提供桌面、自碰撞检查或策略执行安全控制；首次必须人工监督、小幅低速验证。
- 腕部视野与全局相机安装、串口状态、实际采集频率和真实抓取成功率需要现场验收。
- C100 已接入；正式双相机数据应在两路相机布局固定后采集。

## 9. 依赖检查与数据迁移

```bash
bash tools/run.sh -m pip check
```

数据默认写入工作区 `data/`，被 Git 忽略，不会随代码克隆到新电脑。
需迁移已有示范时单独复制数据目录，保留 episode 元数据、图像和时间戳。
