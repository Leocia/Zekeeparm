# Zekeeparm 独立数采包（ROS 2 / CPU，适用 AMD 电脑）

本包只用于人工操作机械臂时采集示范数据，支持主从臂遥操作、游戏手柄两种方式。
默认平台：**Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10**。
默认使用 ROS 手柄数采流程；主从臂保留独立采集入口。
不要求 NVIDIA 显卡、CUDA 或 ROCm。AMD 显卡不参与数采计算。

保留：双相机图像、机械臂状态和动作、录制与保存、数据检查、视频回看。
不包含：YOLO、GraspNet、自动抓取、π0.5/OpenPI、模型权重、训练、推理、网页或仿真应用。
手柄模式所需的驱动、MoveIt、机器人模型、碰撞几何和原安全保护属于数采依赖。
主从臂复用 LeRobot 设备接口，因此安装其依赖和 **CPU 版** PyTorch；不下载模型或启动训练服务。

## 1. 安装

将整个压缩包解压到任意有写权限的目录，进入解压后的根目录。
本包为源码包，首次安装需要联网下载依赖，不是离线安装镜像。

先按 [ROS 2 Humble 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
配置 ROS 软件源并安装 `ros-humble-desktop`。已有 Humble 时跳过这一步。
随后在本包根目录运行：

```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip build-essential ffmpeg libusb-1.0-0 libgl1 lsof
bash setup.sh
```

脚本默认安装并构建 ROS 数采依赖，在包内创建三个隔离环境：

- `.venv-capture`：采集、主从设备接口、CPU PyTorch、数据检查和视频回看。
- `.venv-camera`：Orbbec SDK、NumPy 1.x、相机取帧。
- `.venv-ros`：ROS 驱动、手柄、MoveIt 所需 SDK。

不会安装显卡驱动，不会连接机械臂、打开相机或使能电机。

`setup.sh` 自动调用 `tools/install_ros.sh`，安装 MoveIt、手柄、ROS 消息依赖，
并构建五个数采依赖包。安装过程会调用 `sudo apt-get` 和 `rosdep`。
只需重新构建 ROS 环境时可单独运行 `bash tools/install_ros.sh`。
ROS 驱动使用 `motorbridge==0.5.1`，主从接口使用 `motorbridge==0.4.9`，两个环境不能混装。
ROS 命令终端使用 `source tools/activate_ros.sh`，它会加载 Humble、本包环境和本包的 `install/setup.bash`。
`capture.sh` 的手柄相关入口自动完成这一步。主从采集使用隔离环境直接访问 SDK，不能同时启动 ROS 驱动。

## 2. 设备与配置

先检查以下文件，再连接设备进行操作：

| 文件 | 需要核对 |
| --- | --- |
| `src/zekeep_embodied/configs/record.yaml` | `task`、`installation_id`、全局相机设备路径、采集频率 |
| `src/zekeep_camera/config/camera.yaml` | 腕部相机分辨率、帧率、预设文件；`serial` 为原设备记录，不是设备选择器 |
| `src/zekeep_teleop/configs/zhongling_b601_teleop.yaml` | 主臂及从臂串口、方向、限位、启动零位标定 |
| `src/zekeep_bringup/config/zekeep_hardware.yaml` | 手柄模式的从臂串口、关节映射、速度和退出保护 |

默认腕部 Orbbec + 全局 C100；只连接一台要使用的 Orbbec，原驱动打开默认设备。

串口和 UVC 相机权限：

```bash
sudo usermod -aG dialout,video "$USER"
```
重新登录后生效。优先使用 `/dev/serial/by-id/` 和 `/dev/v4l/by-id/` 的稳定设备路径。
Orbbec SDK 自带 udev 规则，可在安装完成后运行：

```bash
ORBBEC_RULES="$(.venv-camera/bin/python -c 'import pathlib, pyorbbecsdk; print(pathlib.Path(pyorbbecsdk.__file__).parent / "shared/install_udev_rules.sh")')"
sudo bash "$ORBBEC_RULES"
```

然后拔插相机。SDK 资料：[Orbbec Python SDK](https://github.com/orbbec/pyorbbecsdk)。


## 3. 默认 ROS 游戏手柄采集

以下四个终端都先 `cd` 到本包根目录

终端 1，启动唯一驱动，默认不使能：

```bash
./tools/capture.sh driver
```

终端 2，启动 TF 与 MoveIt，必须保持运行：

```bash
./tools/capture.sh moveit
```

终端 3，先验证 TF：

```bash
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 run tf2_ros tf2_echo base_link link6
```

确认持续输出 Translation / Rotation，按 Ctrl+C 退出检查。
如果 frame 不存在，先修复 MoveIt 启动错误，不要继续使能。
现场确认姿态、空间和急停后，在同一终端手动使能并启动手柄：

```bash
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
./tools/capture.sh joystick
```

使能必须返回 `success: true`。按住 LB 做小幅运动，松开 LB 确认停止。
终端 4，控制正常后启动只读录制；录制时按键后回车：s 开始、p 成功保存、w 失败保存、d 放弃、e 退出：

```bash
./tools/capture.sh record --output "$PWD/data/my_task/raw"
```

录制器不会使能或发送运动命令，观察现有手柄控制。退出录制器不会停止其他终端中的驱动。
`LB+Y` 切换关节模式，松开再按 LB 恢复运动。
单按 X 回安全零位并关闭夹爪，只在 episode 结束后使用；MoveIt 必须保持运行。
`RB+X` 回原配置准备位，保持夹爪目标；先按住 RB 再按 X，避免触发单独回零。
返回会先检查 MoveIt 路径，返回过程中不要求持续按住组合键。
回准备位动作只在希望记录这一动作时放在录制内，否则先保存示范再复位。

正常退出 ROS 驱动沿用原有的安全回零与失能保护；退出时保持 MoveIt 和现场监督。
回零失败时保护可能保持力矩并等待处理，不要强杀负责保持机械臂的进程。
MoveIt 只检查已建模的场景，不能感知所有现场障碍物。

## 4. 主从臂采集

与手柄模式二选一，不能同时运行。停止 ROS 驱动及其他占用从臂串口的程序。
先核对主从臂串口、原设备标定和启动姿态，准备急停及足够运动空间。

**运行下列命令会使能从臂，从臂立即跟随主臂；未开始录制时也会跟随。**

```bash
./tools/capture.sh record-leader --output "$PWD/data/my_task/raw"
```

输入按键后按回车：

| 按键 | 操作 |
| --- | --- |
| `s` | 开始一条示范 |
| `p` | 成功保存 |
| `w` | 失败保存 |
| `d` | 放弃，保留 `.partial` 文件 |
| `e` | 退出，未完成示范标记为 aborted |

退出、Ctrl+C 或异常会释放从臂力矩，**退出前先扶稳机械臂**。
主从模式没有环境碰撞或自碰撞检查，需要现场人工监督，首次小幅验证。
采集入口不会写硬件零位。

## 5. 数据保存与回看

每条示范为独立目录：

```text
data/my_task/raw/episode_.../
  episode.json      任务、相机、标定元信息、状态、单位
  frames.jsonl      每帧状态、动作、时间戳
  wrist/*.png      腕部 RGB 图像
  global/*.png     全局 RGB 图像
```
与原项目原始数据格式保持兼容。状态和动作各 **7 维（六关节 + 夹爪）**，单位为度，
动作是映射和限幅后的绝对电机目标。相机保存彩色图像，**当前格式不保存深度图**。
未完成或写入失败的数据保留为 `.partial`，不要当作成功示范使用。

将下面的 `episode_...` 换成实际示范目录，回看和报告须使用新文件名：

```bash
./tools/capture.sh check-raw data/my_task/raw/episode_...
./tools/capture.sh preview-raw data/my_task/raw/episode_... --output data/my_task/review.mp4
./tools/capture.sh audit data/my_task/raw --output data/my_task/audit.json
```

回看视频使用名义帧率；真实时间在 JSONL 中，不能仅凭视频判断同步和反馈新鲜度。
`audit` 列出通过检查的成功真实示范及排除原因，不删除或修改原始数据。
单独备份整个 `data/` 目录。PNG 数据量较大，采集前检查磁盘空间。
