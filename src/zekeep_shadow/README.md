# zekeep_shadow：红色方块视觉跟随

使用奥比中光 RGB-D 相机和现有眼在手上标定。OpenCV HSV 检测高饱和红色区域并过滤不规则轮廓；点击选定目标后，目标大小、图像位置和深度位置必须连续，避免跟踪跳到手上。

## 控制行为

控制节点以 100 Hz 使用 Pinocchio 计算 TCP 雅可比矩阵。平移跟随图像中心与 0.15 m 深度；跟随启动时记录的 TCP 姿态在平移零空间内保持。关节速度积分成 POS_VEL 位置目标，由控制器伺服流下发。

按住 `g` 时，目标在基座坐标系中保持锚点；视觉测得的表面点移动超过 15 mm 后，锚点才跟随，避免视角改变导致静止方块被反复追逐。检测框短时跳动导致图像区域不重叠时，若三维位置在 20 mm 内仍保持目标锁定。相机误差使用指数平滑和启停双阈值：横向误差超过 10 mm、深度误差超过 25 mm 才开始修正；回到横向 5 mm、深度 12 mm 内停止修正。停止时立即发零关节速度并保持位置目标。

## 启动与按键

设置 `ZKEEP_WS` 为工作区实际路径。启动相机驱动终端：

跟随时按住 `g`，松开保持；`c` 清除目标；目标丢失后按 `h` 请求返回 `ready_pose`；`q` 退出。启动时自动请求 `return_ready` 并打开夹爪，失败则禁止跟随。控制器必须使用 `zekeep_hardware.yaml` 的 POS_VEL 模式并已使能。

相机暂时没有 RGB-D 帧时，预览窗口会显示 `NO RGB-D FRAME` 和底层原因，并继续等待恢复；按 `q` 可退出。若设备被其他程序占用，先关闭那个相机程序再重启跟随。

```bash
cd "$ZKEEP_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
conda activate rebotarm
ros2 launch zekeep_bringup bringup.launch.py hardware_config:="$ZKEEP_WS/src/zekeep_bringup/config/zekeep_hardware.yaml"
```

另一个终端，在机械臂已获得可靠支撑、运动区域安全时运行：

```bash
cd "$ZKEEP_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
conda activate rebotarm
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
ros2 launch zekeep_shadow follow.launch.py
```

## 安全边界与验证状态

此版本不启动 MoveIt，也不执行环境碰撞或自碰撞检查。控制器仍保留硬限位；节点保留 0.02 rad 软件限位余量、0.455 rad/s 关节速度限幅、视觉与关节反馈超时、按键心跳和伺服命令看门狗。

平移误差增益为 2.4/s，单轴期望速度先限于 0.12 m/s，再经 1.3 倍关节速度缩放和 0.455 rad/s 限幅；底层 POS_VEL 限速仍为 0.50 rad/s。状态中的 `cmd` 和 `act` 分别显示最大指令与反馈关节速度。真机跟随、姿态偏差 2 度及端到端延迟 100 ms 指标尚未验证。

## 网页只读预览

新增 `web_vision` 节点复用 RGB-D 驱动与红色检测，发布 `/zekeep/web/camera/image_raw`
和 `/zekeep/web/vision/detections`，不启动跟随器，不打开电机连接。
无有效标定或机械臂移动时只显示预览，抓取目标需要静止反馈、TF 与视觉/执行端标定配置一致。
无需本机身份指纹或 `identity.local.json`。
该节点独占相机，运行前关闭其他占用同一相机的程序。

可通过 `zekeep_bringup web_tasks.launch.py start_vision:=true` 显式启动。
抓取任务由 `zekeep_teach` 的 ROS 后端协调，使用 MoveIt 规划和控制器任务接口。
启动说明见[网页 README](../zekeep_bringup/web/README.md)。
