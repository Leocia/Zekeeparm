# 采集与实机操作

所有命令从工作区根目录运行。首次安装见 [README](../README.md)，数据上传与训练见[云端流程](cloud-data.md)。
本地采集不需要模型权重；模型预测和执行使用 ACT、π0.5 共用的 LeRobot 环境及匹配权重。
使用 ACT 已导出的 v3 训练集时，`predict`、`serve` 同样加 `--dataset-root 数据目录/train`；`shadow/execute` 仍按训练时的 repo-id 与实验名核对服务。

## 相机检查

```bash
./tools/collect.sh doctor
./tools/collect.sh camera-check --seconds 10
```

`doctor` 检查采集依赖和配置路径，不打开设备，也不验证 ROS、相机 SDK 或电机运行。
`camera-check` 只打开相机，保存两路图片和 `camera_check.json`；检查视野、清晰度、帧率和时间差。

配置见 [record.yaml](../configs/record.yaml)：

- 腕部 Orbbec 使用独立 `rebotarm` Python 环境，图像配置来自 `zekeep_grasp/config/default.yaml`。
- 全局 C100 使用 `新相机的 `/dev/v4l/by-id/` 路径`，避免使用会变化的 `/dev/videoN`。
- C100 默认 1280×720、MJPEG、请求 30 FPS；此前实测约 15 FPS，因此采集设为 10 Hz。以当前预检结果为准。
- 相同序列号的多台相机改用各自的 `/dev/v4l/by-path`。移动相机或更换设备后更新 `installation_id` 并重测。
- 双相机按主机接收时间软件对齐，容差 100 ms；不是曝光级硬件同步。训练和部署需保持相同图像处理。

全局视野应覆盖目标、托盘和夹爪活动范围。不要复制腕部图像充当全局图像，也不要直接混合不同安装布局的数据。

## 主臂遥操采集

ACT 与 π0.5 共用此入口。核对[遥操作配置](../../zekeep_teleop/configs/zhongling_b601_teleop.yaml)中的串口、标定和启动零位。
停止手柄、SDK 遥操和模型执行；保留唯一 ROS 真机驱动。采集器不写硬件零位，不自动使能或对齐。

```bash
# 终端 1：已有驱动时跳过，不得重复启动
./tools/collect.sh driver
# 终端 2：现场检查并支撑机械臂后，人工使能
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
# 返回 success: true 后启动采集；与手柄原始目录分开
./tools/collect.sh record-leader --output "$PWD/data/leader_demonstrations_v1/raw" --episodes 50
```

输入后回车：`f` 核对两臂姿态并跟随，`s` 开始，`p` 成功保存并暂停，`w` 失败保存并暂停，
`h` 暂停，`d` 放弃并暂停，`e` 退出。启动要求每轴差异不超过 `leader_control.alignment_tolerance_deg`（默认 5°）；
不满足时调整主臂后再输入 `f`，程序不会自动拉齐。从下一条示范开始前也需重新输入 `f`。
退出、Ctrl+C 或异常会停止跟随，由 ROS 驱动尝试持位；不要关闭负责持力的驱动。
本模式不使用手柄 deadman，也不提供环境碰撞或自碰撞检查，首次现场监督、小幅验证。
旧 `pi05.sh record-leader` 现转发到此 ROS 流程，不再启动直连 SDK 的自动使能采集。

## 游戏手柄采集

与主臂遥操二选一。按以下顺序操作，四个终端分别运行；已启动的驱动或 MoveIt 不要重复启动。下列工作区路径按本机填写，换机时替换。

默认使用末端笛卡尔控制，必须同时保持驱动、MoveIt 和手柄节点运行。`driver` 只启动驱动，不发布 TF；`joystick` 也不会自动启动 MoveIt。终端 2 的 `hardware.launch.py` 提供 `robot_state_publisher`（发布 `base_link` 到 `link6` 的 TF）和 MoveIt，不能跳过。

**第 1 步，终端 1：启动唯一的真机驱动，默认不使能。保持该终端运行。**

```bash
cd $HOME/Desktop/Zekeeparm
./tools/collect.sh driver
```

**第 2 步，终端 2：启动 TF 和 MoveIt。保持该终端运行，不能跳过。**

```bash
cd $HOME/Desktop/Zekeeparm
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false
```

**第 3 步，终端 3：先验证 TF，再手动使能并启动手柄。**

```bash
cd $HOME/Desktop/Zekeeparm
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 node list
ros2 run tf2_ros tf2_echo base_link link6
```

节点列表应包含 `/robot_state_publisher` 和 `/move_group`。等待 `tf2_echo` 持续输出 `Translation` 和 `Rotation`；若持续报 frame 不存在，先修复终端 2 的启动错误，不要继续使能。

TF 正常后，按 Ctrl+C 退出 `tf2_echo`，在同一终端完成现场检查，再运行：

```bash
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
./tools/collect.sh joystick
```

手动使能须返回 `success: true`。手柄显示 `ready` 后，按住 LB 做小幅运动，松开 LB 确认停止；已有手柄节点时，确认 TF 正常后松开 LB 再重新按住。

**第 4 步，终端 4：确认手柄控制正常后，启动只读录制。**

```bash
cd $HOME/Desktop/Zekeeparm
./tools/collect.sh record --output "$PWD/data/demonstrations_v1/raw"
```

默认末端笛卡尔控制；`LB+Y` 切换关节模式，松开再按 LB 恢复运动。单按 X 回安全零位，会关闭夹爪，只在 episode 结束后使用，保持 MoveIt 运行。
`RB+X` 回驱动配置的准备位（当前关节角 `[0°, 40°, 40°, -35°, 0°, 0°]`），不改变夹爪开合目标；先按住 RB，再按 X，先按 X 会触发回零。先停止伺服，再进行 MoveIt 全路径碰撞校验并返回，完成后松开再按 LB。返回运动不要求持续按住 RB、X 或 LB。若希望模型学习回准备位，在录制中撤离物体后按 RB+X，等机械臂回位停稳，再按 `p`＋回车保存；仅为下一条复位时，在保存后按 RB+X。
只启动一个控制入口。录制按键为 `s/p/w/d/e`，不需要主臂模式的 `f`；保存只结束记录，不停止手柄控制，先松开 LB 再保存或退出。先录几条，按 [README](../README.md#共用检查与原始数据)检查并回看。

若手柄显示 `ready` 后仍报 `TF unavailable: "base_link" passed to lookupTransform argument target_frame does not exist.`，表示手柄尚未收到所需 TF；`ready` 只表示手柄节点启动完成。回到第 2、3 步检查，各终端须加载同一工作区、使用相同的 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`。不要另启 `zekeep_bringup bringup.launch.py` 补 TF，该入口会重复启动真机驱动。

## 离线预测

先按[结果交付说明](cloud-data.md#训练结果)准备基础权重、实验目录、数据和统计。

```bash
./tools/pi05-model.sh predict --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_v1 --frame-index 0 \
  --output data/pi05_red_cube_v1/offline_prediction.json
```

输出目录必须存在，文件须用新名称。预测为 10×7 的绝对电机角度（度），不连接硬件；数值预测不能直接当作实机轨迹，也不是任务成功率评估。

## 影子模式与回放

三个终端分别运行；已有驱动时不重复启动，停止其他占用相机的程序：

```bash
./tools/pi05-model.sh serve --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1
./tools/collect.sh driver
./tools/pi05-model.sh shadow --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1
```

服务仅监听本机 `127.0.0.1:8000`。`shadow` 读取相机和反馈，只预测、不控制机械臂。
检查模型标识、预测方向、限位和延迟；`action_guard_passed` 不代表经过碰撞检查。

内存紧张时，用下面的保存端替代普通 `shadow`，保存成功后停止模型服务，再回放：

```bash
./tools/pi05-model.sh shadow --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1 \
  --save-preview data/pi05_red_cube_v1/live_preview.json

# 另外两个终端分别运行，不需要模型、相机或实机驱动。
./tools/pi05-model.sh rviz
./tools/pi05-model.sh replay data/pi05_red_cube_v1/live_preview.json
```

`replay` 只接受 `shadow --save-preview` 的显示轨迹，不接受 `predict` JSON。
实时预览可给 `shadow` 加 `--preview` 或 `--preview-once`，并单独启动 `rviz`。
回放不模拟物体接触，不验证抓取成功；越界预测可显示，但不代表可执行。不要停止负责实机保持或重力补偿的节点。

## 现场监督执行

先完成影子检查，清空工作区并准备硬急停。停止采集、shadow 和 joystick 控制节点；保留唯一的 `serve` 和驱动。
现场手动使能后，在本机桌面的终端运行：

```bash
./tools/pi05-model.sh execute --repo-id local/zekeep_red_cube_v1 --exp-name red_cube_v1
```

执行要求交互输入 `MOVE`，随后全局按住 `m` 即启动，松开即暂停，不需要手柄、`joy_node` 或额外的 `s` 命令。
停止后需松开再按；若启动程序时已按着 `m`，先松开一次。切换到其他窗口后 `m` 仍生效，打字也可能触发运动。
支持本机 X11 桌面，使用 `python-xlib` 读取按键状态；无桌面 SSH 和 Wayland 不支持。
键盘状态由独立线程每 20 ms 读取；`pi05_runtime.joy_timeout_s` 沿用为键盘采样新鲜度上限，默认 0.25 s。
松开 `m`、键盘连接中断或采样超时、反馈/推理超时、图像不同步或动作越界会停止伺服。
每次仅执行动作块第一步；默认单步不超过 0.5°、关节速度不超过 0.10 rad/s。夹爪操作后需松开再按 `m`。
入口不自动使能、回零或复位物体，也不提供环境碰撞或自碰撞检查。
