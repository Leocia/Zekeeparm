# ACT 训练与执行

双 RGB＋六关节和夹爪状态，学习红色方块抓取放入固定容器。采集、执行统一走 ROS；不自动回零、复位或判断成功。

采集与原始数据检查统一使用 [`tools/collect.sh`](../../tools/collect.sh)，见[共用数采说明](../zekeep_embodied/README.md)。
同一份原始数据可分别导出 ACT 和 π0.5 格式，本节只列 ACT 的处理和执行步骤。

## 参数

修改 [configs/act.yaml](configs/act.yaml)：

| 参数 | 默认 / 说明 |
|---|---|
| `capture_config` | 复用双相机和 ROS 硬件标定；采集 10 Hz |
| 采集 `workspace` | 统一在 `record.yaml` 中现场确认 `x_m/y_m`、`table_z_m` 和描述；ACT 默认继承；旧 ACT 配置显式覆盖仍有效 |
| `output` | `data/act_joystick_pick_place` |
| `episodes / validation_episodes` | 成功 50 条，45 训练＋5 验证 |
| `image_size` | 224，等比缩放并补边 |
| `training` | CUDA，batch 4，学习率 1e-5，20000 步，动作块 10 步，每次执行首步 |
| `inference` | 六轴时间集成窗口 2 步，权重系数 0.01；夹爪使用最新预测；窗口设为 1 可关闭集成 |
| `control` | ACT 六轴自主执行单步最多 1.0°、速度 0.20 rad/s；超时 0.20 s |
| `gripper` | 跟随模型第 7 维连续角度；预测越界最多 10° 时收敛至标定端点，越界更大则停止 |

默认 `record` 只监听现有手柄控制，记录双相机、关节和夹爪状态及动作目标。
不连接主臂、不占用电机串口、不发送运动命令。手柄控制频率与 10 Hz 采样相互独立。
推荐原始目录 `data/demonstrations_v1/raw`，ACT 处理与训练继续写入 `data/act_joystick_pick_place`。旧数据保留；不要混合两种来源的示范。

六轴执行上限按人工授权提高至原来的 2 倍：在 10 Hz 动作更新下，目标推进上限约由 5°/s 提高至
10°/s。实际速度仍取决于预测目标和跟随情况，不保证整项任务耗时减半。配置拒绝超过 0.20 rad/s；
夹爪力矩、开合速度、硬件限位和所有超时保持不变。无需重训或重新导出，重新启动 ACT 进程后生效。
本次提速不能解决目标反复修正或失败恢复问题，仍需先做短时监督验证。

`shadow` 和 `execute` 使用 LeRobot 原生 ACT 时间集成，融合最近最多 2 次预测中对应当前时刻的
六轴目标，不是连续执行旧动作块。10 Hz 下最旧预测来自约 0.1 s 前，每次仍读取最新图像和反馈。
夹爪始终使用本次预测的首步，不经过时间集成，保持原开闭时机。暂停后重新启动、异常停止及 CUDA
预热结束都会清空历史，避免把上次预测带入新执行。时间集成旨在降低帧间摆动，不能修正系统性定位
偏差或增加失败恢复能力；速度、步长、关节限位和超时保护仍由原执行路径检查。
1095 帧原始验证图像的离线回放中，2 步集成使六轴预测目标的平均绝对二阶差分降低约 46.7%，
六轴平均 MAE 从约 1.76° 增至 1.82°；选择该窗口以减少 3 步集成引入的额外偏差。该指标只描述
预测目标的平滑程度，不代表实机摆幅或抓取成功率。`validate` 继续报告原始首步预测误差，便于比较训练结果。

`shadow` 和 `execute` 使用模型第 7 维作为连续夹爪目标角度，不再使用开闭阈值、保持上一意图或
自动切换堵转夹持。例如模型预测 28° 就跟随 28°，不会把它转换为 0°、65.89° 或保持张开。
非有限值，以及低于闭合端点 10° 或高于张开端点 10° 的预测仍会停止；容许范围内的小幅越界先
收敛至合法端点，再限制步长。原始预测保留用于核对，不修改训练数据或 checkpoint。
日志保留原始首步预测 `raw_action_deg`、集成后的 `policy_action_deg`、实际反馈 `state_deg` 和
`gripper_mode: "position"`；shadow 的 `action_deg` 是边界处理后的策略输出，execute 的 `action_deg`
经过执行保护，末项是实际采用的连续夹爪目标。无需重新导出或重训，退出并重启 ACT 后生效。

ACT 使用 `/zekeep/servo/start_with_gripper`，经七维位置 servo 下发六轴和夹爪目标，与示教时的
连续夹爪角度通路一致。六轴保持 1°、0.20 rad/s 上限；夹爪单独使用硬件配置的速度上限，当前为
0.575 rad/s，与 LT/RT 连续角度示教一致，并根据实际更新间隔限制目标步长，不会把夹爪误限到六轴速度。

松开 `m`、暂停、退出或看门狗超时后，由驱动尝试保持实际位置；不会由 ACT 自动重抓或释放。
不要关闭负责持位的驱动。外部 gripper action 在 servo 运行时仍被拒绝，主从采集通路保持不变。
位置控制的限位、限速不等于夹持力限幅，实际夹持力仍须单独验收。

`control.gripper_lower_tolerance_deg: 1.0` 继续用于真实反馈校验，允许闭合下限以下最多 1° 的反馈，
原始观测不变，保持目标限制在硬限位内；预测角度的 10° 边界余量不会放宽实际反馈或硬件限位。
先重新运行 shadow 核对连续角度，再做短时实机验证；角度预测的准确性和夹持时机仍由当前模型决定。

C100 图像参数在 [record.yaml](../zekeep_embodied/configs/record.yaml) 的
`cameras.global` 下设置，每次打开相机都会应用并读回核对。当前保存了 2026-10-27
在 qv4l2 调好的参数，保留自动曝光、自动白平衡及动态帧率；实际曝光和白平衡仍随光照变化。
需要固定曝光时间时设 `auto_exposure: false` 并添加 `exposure`；
`exposure: 100` 表示 10 ms，数值越小越暗。增益范围 0–100，越大越亮。
Orbbec Viewer 预设只作用于腕部相机。
修改后重启采集并运行 `camera-check`；新旧相机配置的数据不能混合用于同一训练。
预检图片在检查结束时保存，让自动曝光、白平衡有时间稳定；`--seconds 10` 的截图取自预检结束时。

## 步骤与命令

按[分模块安装](../../docs/INSTALL_PORTABLE.md)准备所需环境。离线训练只需 `models`；
实际采集与真机执行另需 `ros`、`vision`、`collect`，在工作区根目录按需安装。
使用 ACT、π0.5 共用的 `act` 环境；其他解释器优先用 `LEROBOT_PYTHON`，兼容 `ACT_PYTHON`。

```bash
bash tools/install_optional.sh collect
bash tools/install_optional.sh models
source tools/activate_ros.sh
colcon build --symlink-install --packages-select zekeepcontroller zekeep_joystick
```

1. 先填写 `src/zekeep_embodied/configs/record.yaml` 的任务、非空 `installation_id`、相机设备和解释器路径，
   确认采集区域、物体和固定容器位置；现场核验后再将 `workspace.confirmed` 设为 `true`。
   相机移动后更新 `installation_id` 并重新采集。关闭其他机械臂控制程序，再运行检查：

```bash
./tools/collect.sh doctor
./tools/act.sh doctor
# 以下命令会打开相机，不连接机械臂。
./tools/collect.sh camera-check
```

2. 分别开终端启动唯一的驱动、MoveIt、手柄控制。默认是已有的末端笛卡尔控制，需要 MoveIt：

```bash
# 终端 1：
./tools/collect.sh driver
# 终端 2：
./tools/collect.sh moveit
# 终端 3：同时启动 joy_node，不要另开 act.sh joy：
./tools/collect.sh joystick
# 终端 4：现场确认后输入 ENABLE，再启动采集：
./tools/act.sh enable
./tools/collect.sh record --output "$PWD/data/demonstrations_v1/raw" --episodes 50
```

采集器允许 `/joy` 消息间隔最多 300 ms，与手柄控制器的超时一致；静止时由 `joy_node` 自动重复发布，无需持续拨动手柄。
关节及夹爪反馈仍要求在 100 ms 内更新。超过手柄超时时，错误会显示实际消息延迟；检查手柄连接和 `joy_node` 是否正常运行。
这些阈值位于 `record.yaml` 的 `ros_joystick.max_joy_age_s` 和 `max_state_age_s`，修改后重启采集器生效。

按住 LB 允许运动，松开 LB 保持。左摇杆控制末端 X/Y，右摇杆上下控制 Z、左右控制 yaw，
十字键控制 roll/pitch；LB+A 夹持，LB+B 打开夹爪。夹爪动作完成后松开并重新按住 LB 恢复臂运动。
LB+Y 可切换到关节控制；恢复末端模式需重启手柄节点。
按钮及方向沿用 [zekeep_joystick](../zekeep_joystick/README.md) 的现有映射。

采集终端输入后回车：`s` 开始一条，`p` 成功保存，`w` 失败保存，`d` 放弃，`e` 退出。
不再需要输入 `f` 或对齐主从臂。保存只结束本条记录，采集器不会停止手柄控制；先松开 LB，再保存或退出。
录制中松开 LB 可以停顿；仅在结束 episode 后使用 X 回零或 RB+X 回准备位，避免混入未记录的回位动作。
相机、ROS 状态或手柄反馈异常会中止记录，未完成 episode 不进入训练；采集报错时也须松开 LB。
成功达到 50 条后自动结束采集。放入容器、松爪、撤离停稳后保存；下一条人工复位。

3. 采满 50 条成功示范后检查、导出并训练；输出目录须为新路径：

```bash
./tools/collect.sh audit data/demonstrations_v1/raw --output data/demonstrations_v1/audit.json
./tools/collect.sh preview-raw data/demonstrations_v1/raw/episode_实际目录 --output /tmp/review.mp4
./tools/act.sh prepare --raw data/demonstrations_v1/raw
./tools/act.sh train --output data/act_joystick_pick_place/runs/act_001
./tools/act.sh validate data/act_joystick_pick_place/runs/act_001/checkpoints/last/pretrained_model
```

π0.5 可以用 `--dataset-root data/act_joystick_pick_place/dataset/train` 直接读取同一 v3 训练集，见[模型环境与迁移说明](../zekeep_embodied/docs/pi05.md)。

训练首次可能下载 ResNet18 预训练权重。显存不足先减小 `training.batch_size`；记录参数后重新使用新训练目录。

`prepare` 先读取小体积 JSON/JSONL，检查全部示范的状态、动作、配置和 10 Hz 重采样时间戳；
时间间隔异常会直接报告具体 episode，无需先解码全部图片。随后使用 4 个线程逐张解码检查图片，
保留图片损坏、分辨率变化及原有安全检查；导出前的配置预检不再重复解码图片。
`check` 同样并行检查全部图片，并检查重采样时间戳。检查和导出会显示当前 episode 及总数。
检查与编码仍需读取原始图片；导出成功后重新训练不需要再执行 `prepare`。

4. 保持驱动运行，停止手柄控制节点（终端 3 按 Ctrl+C），再预测检查和执行。
手柄控制节点不能与 ACT execute 同时运行。首次执行前另行人工使能：

```bash
./tools/act.sh shadow data/act_joystick_pick_place/runs/act_001/checkpoints/last/pretrained_model --seconds 10
./tools/act.sh execute data/act_joystick_pick_place/runs/act_001/checkpoints/last/pretrained_model
```

执行输入 `MOVE` 确认；在本机 X11 桌面全局按住 `m` 即启动，松开即暂停，不再输入 `s`，不需要手柄或 `joy_node`。
停止后松开并重新按住 `m` 才能继续；若启动程序时已经按着 `m`，先松开一次。
`h`＋回车暂停，`e`＋回车退出。切到浏览器或编辑器后，按住 `m` 仍会使能运动，其他窗口打字也可能触发。
监听使用已安装的 `python-xlib`；无桌面 SSH 和 Wayland 不支持。程序每 20 ms 独立读取键盘状态，
沿用 `control.joy_timeout_s`（默认 0.25 s）检查键盘采样新鲜度；连接中断、键盘采样超时、反馈超时或推理异常都会停止。
暂停及重新启动会清空 ACT 预测历史。采集仍使用原有手柄按键。
停止后驱动保持使能及持力，勿关闭负责持力的驱动。首版没有在线碰撞检测，须现场监督并准备硬急停。真实抓放效果由操作者验收。

## 可选：主从采集

统一入口为 `./tools/collect.sh record-leader --output 新的主从原始数据目录`。
旧 `act.sh record/record-leader` 保留原参数，调用同一实现；新流程使用采集配置的 `leader_control`，旧 ACT 入口继续使用 ACT 配置的 `control`。
先停止手柄控制节点；主臂须在原标定启动零位连接。
输入后回车：`f` 核对两臂姿态并跟随，`s` 录制，`p` 成功保存并暂停，`w` 失败保存，
`d` 放弃，`h` 暂停，`e` 退出。启动要求每轴目标与反馈相差不超过 5°，程序不会自动对齐。
主从和手柄数据使用不同目录，导出时通过 `prepare --raw 原始目录` 选择一种来源。

主从跟随独立于图像采集：复用 `zekeep_teleop` 配置的 `fps: 50`，
每 20 ms 读取主臂后台缓存并通过 ROS 下发最新目标；双相机和数据保存仍为 10 Hz。
跟随速度复用 `robot.pos_vel_velocity`，同时受 ROS 硬件 `max_velocity` 限制：
当前 J1–J3 为 60°/s，J4–J6 为 1.5 rad/s，夹爪为 0.575 rad/s。
保留主臂滤波、标定、关节限位、启动对齐、反馈故障及超时停机；
采集循环超过 0.20 s 未提供有效样本也会停止跟随。
终端每秒显示实际跟随发布频率。录制动作保存采样时正在下发的有界目标。
ACT 自主执行继续使用 `control` 的慢速限制。
