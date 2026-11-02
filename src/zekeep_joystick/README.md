# zekeep_joystick

六轴关节和末端笛卡尔遥操作包，默认使用实测的 Zikway HID gamepad 映射。节点不会调用 `/zekeep/enable`；电机使能必须手动完成。

## 控制映射

- 按住 `LB`：允许运动；松开后保持当前位置。
- 左摇杆横/纵：J1/J2。
- 右摇杆纵/横：J3/J4。
- 十字键横/纵：J5/J6。
- 按住 `LB` 时，`A` 力矩夹持（接触后转保持力矩），`B` 打开夹爪。
- 按住 `LB+LT` 逐步打开夹爪，`LB+RT` 逐步闭合；按得越久变化越大，松开扳机保持当前开度。两扳机同时按下保持，松开 LB 或手柄超时则停止。适用于关节、末端及 LB+Y 切换后的关节模式。
- 两个控制入口均支持单按 `X`：运动中先停止伺服并确认成功，再调用 `/zekeep/safe_home` 回安全零位。需要 MoveIt 全路径碰撞校验，回零也会关闭夹爪；完成后松开并重新按住 LB 恢复控制。
- 两个控制入口均支持 `RB+X` 回准备位：先按住 RB，再按 X；先停止伺服并确认成功，再调用 `/zekeep/return_ready`，需要 MoveIt 全路径碰撞校验。准备位来自驱动的 `ready_pose.positions`，当前六轴关节角为 `[0°, 40°, 40°, -35°, 0°, 0°]`；此操作不改变夹爪开合目标，完成后松开并重新按住 LB 恢复控制。返回运动不要求持续按住 RB、X 或 LB，使用前确认返回路径安全。

按 A/B 后，节点先停止关节伺服并确认保持成功，再执行夹爪动作。夹爪动作期间暂停关节遥操作，完成后松开并重新按住 LB 恢复。停止失败时不会发送夹爪命令；动作失败原因会显示在终端。

A 调用驱动的 `/zekeep/gripper/grasp` Action。检测到物体后，驱动持续提供保持力矩，恢复关节控制后仍保持；B 打开并退出夹持。动作执行期间 A/B 不重复触发，等待结果后再按。
LT/RT 使用位置伺服，可与机械臂遥操同时进行；首次使用扳机会退出 A 的力矩保持。位置闭合不执行 A 的接触检测，抓住物体后需要力矩保持时用 A。扳机按到底的速度由 `gripper_speed` 控制，默认 0.575 rad/s（0–1.15 rad 全行程约 2 秒），轻按更慢。开闭端点参数须与驱动夹爪标定一致，驱动仍校验限位并限速；目标受反馈约束，避免接触后累积闭合行程。采集器同步记录连续夹爪目标。修改驱动夹爪限速后须重启驱动，手柄参数修改后须重启手柄节点。
目标最多超前反馈 0.2 秒的设定行程（当前为 0.115 rad），不随 15 Hz/50 Hz 发布频率变化；松开扳机时改为保持反馈位置。约 2 秒为理想目标时间，实际开合时间受负载、摩擦和电机跟踪影响。
`grasp_closing_torque`、`grasp_hold_torque`（单位 N·m）和 `grasp_timeout`（秒）为 0 时继承驱动配置。当前标准配置为闭合 1.0 N·m、保持 0.30 N·m、超时 5 秒；驱动限制力矩上限为 1.5 N·m。
接触判据为速度绝对值不超过 0.05 rad/s、反馈力矩绝对值至少 0.10 N·m，并且已闭合运动至少 0.30 rad；闭合硬限位附近判为空抓。先用 B 打开再夹持，较宽物体若无法达到最小运动量可能超时。这些阈值来自驱动硬件配置，需按物体和反馈调试；电机力矩不等于指尖夹持力。

## 末端笛卡尔模式

关节控制入口保持不变。末端控制使用独立入口，两者不能同时运行：

```bash
ros2 launch zekeep_joystick cartesian_joystick.launch.py
```

手动使能后按住 LB：左摇杆上下/左右控制基坐标系 X/Y，右摇杆上下控制 Z，十字键左右控制 pitch、上下控制 roll，右摇杆左右控制 yaw；LB+A 力矩夹持，LB+B 打开夹爪。单按 `X` 先停止伺服，再调用 `/zekeep/safe_home` 自动回安全零位；该驱动动作也会关闭夹爪，且必须同时运行 MoveIt 做全路径碰撞校验。`LB+Y` 从末端模式切换到关节位置模式；松开 LB 后重新按住，映射变为左摇杆 J1/J2、右摇杆 J3/J4、十字键 J5/J6。此切换单向生效，恢复末端模式需重启手柄节点。

A/B 夹爪动作、回零或模式切换前都会先停止末端伺服。动作完成后松开再按 LB。松开 LB 保持。目标是 `base_link` 下的 `link6`；速度、方向、按钮和 IK 连续性阈值见 `config/zikway_cartesian.yaml`。录制示范时只在一段 episode 结束后使用 X，避免把未记录的回零动作混入状态序列。

关节入口参数在 `config/zikway.yaml` 修改，末端入口参数在 `config/zikway_cartesian.yaml` 修改。
手柄角速度统一为 0.5 rad/s（约 28.6°/s）：六轴关节、末端 roll/pitch/yaw 及末端逆解关节上限均一致。末端逆解按周期限制实际关节目标增量，各关节等比例推进；保持原有 IK 跳变检查，等待逆解期间持续发送保持目标。平移速度为 0.10 m/s，夹爪仍为约 2 秒全行程；逆解所需关节运动较大时，末端实际速度会降低以满足关节上限。
`LB` 只是按住运行开关，不会使能或失能电机。

## 构建与启动

设置 `ZKEEP_WS` 为工作区实际路径。每个 ROS 终端加载同一工作区：

首次使用或修改代码后构建：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
colcon build --packages-select zekeep_joystick zekeep_bringup --symlink-install
source install/setup.bash
```

终端 1 启动唯一的真机驱动：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 launch zekeep_bringup driver.launch.py channel:=/dev/ttyACM0
```

普通关节控制不需要 MoveIt；使用 X 回零或 RB+X 回准备位时必须另外运行下述 MoveIt。终端 2 手动使能并启动：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
ros2 launch zekeep_joystick joystick.launch.py
```

末端笛卡尔控制及两个入口的 X 回安全零位、RB+X 回准备位都需要 MoveIt。在独立终端启动 MoveIt：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false
```

终端 3 手动使能并启动末端手柄：

```bash
cd $HOME/Desktop/Zekeeparm
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
ros2 launch zekeep_joystick cartesian_joystick.launch.py
```

关节控制与末端控制只能二选一，不能同时启动。两个启动文件都会启动 `joy_node`。末端控制不要另启 `zekeep_bringup bringup.launch.py`，`hardware.launch.py` 已提供 TF。

π0.5 采集入口 `./tools/pi05.sh joystick` 默认启动末端笛卡尔控制，需要保持驱动和 MoveIt 运行；`LB+Y` 可切换到关节控制，X 回零仍有效。不要与其他手柄入口同时启动。

ACT 使用相同手柄控制：`./tools/act.sh joystick` 启动末端控制，`./tools/act.sh record`
只读采集。先运行唯一的驱动和 MoveIt，并现场人工使能；不需要连接主臂。
不要同时运行 π0.5 和 ACT 的手柄入口。采集停止或退出不会停止手柄控制，须松开 LB 保持。

需要录制 π0.5 示范时，保持驱动和一种手柄入口运行，在新终端启动只读采集器：

```bash
conda activate embodied
export ROS_LOCALHOST_ONLY=1
export EMBODIED_PYTHON="$CONDA_PREFIX/bin/python"
cd "$ZKEEP_WS/src/zekeep_embodied"
bash tools/run_ros.sh -m zekeep_embodied.record --config configs/record.yaml --joystick
```

采集器不会使能或控制机械臂；详细命令和数据格式见 `src/zekeep_embodied/README.md`。

## 手柄配置与排错

Zikway 的实体 X/Y 分别为按钮 3/2，LB 为按钮 6；右摇杆横/纵为轴 2/3，RT/LT 为轴 4/5（松开 +1、按到底 -1）。
RB 使用按钮 7（Linux `BTN_TR`），由 `return_ready_modifier_button` 配置；先按住 RB 再按 X 回准备位，先按 X 会触发原有回零。M 未检测到 `/joy` 按钮事件，不用于回位映射。
其他手柄先核对 `/joy`，可通过 `config:=/绝对路径/映射.yaml` 选择配置；`config/xbox.yaml` 保留给旧布局。

以上命令假设所有 ROS 节点运行在本机，因此每个终端都设置 `ROS_LOCALHOST_ONLY=1`。
如果需要连接其他电脑上的 ROS 节点，所有相关终端应使用一致的网络设置。

如果终端反复显示 `cartesian joystick control active` 和 `deadman released`，检查 `/joy` 是否有重复发布者：

```bash
ros2 topic info /joy --verbose
```

正常只应有一个 `/joy` 发布者。多个发布者可能交替发送不同的 LB 状态。先停止多余的手柄或仿真节点；如果其他电脑的 ROS 节点混入本机图，停止现有 ROS 进程，在每个终端按上述方式设置 `ROS_LOCALHOST_ONLY=1` 后重启。必要时再运行 `ros2 daemon stop` 和 `ros2 daemon start`，并复查发布者数量。

首次连接不同型号手柄时先检查轴号：

```bash
ros2 topic echo /joy
```

真机前先悬空或降低 `max_speeds`，逐轴确认方向。手柄断连或 `/joy` 超过 0.3 秒无更新时，节点请求保持；驱动自身还有 0.2 秒伺服看门狗。

## 离线验证

在工作区根目录加载 `tools/activate_ros.sh` 后运行，不连接硬件、不启动 ROS 节点：

```bash
python -m unittest discover -s src/zekeep_joystick/test -v
python -m unittest discover -s tools -p test_pi05_entry.py -v
```

覆盖两个控制入口及末端切换后的关节模式：LT/RT 按时长开合、两秒全行程、松手保持、接触时限制目标超前、反馈超时、X 单次触发、停止确认、失败保持、LB 重新许可和 A/B 夹爪动作，以及采集入口默认启动末端控制。
