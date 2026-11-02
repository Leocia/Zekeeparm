# Six-axis 物理仿真

独立 MuJoCo 进程，不连接串口、不导入实机驱动。HTTP 仅监听 127.0.0.1:8083；可选 ROS 发布只使用 `/zekeep_sim/*`。

## 独立启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zekeep_bringup web_sim.launch.py
```

主控制网页已移除 MuJoCo 入口和仿真轮询。本后端仍可独立启动，通过 `http://127.0.0.1:8083/state` 查询；不自动占用网页模型。

- 虚拟相机来自 MuJoCo 离屏渲染；红色像素与渲染深度反投影生成目标。物体真值只用于显示和结果确认，不作为检测输入。
- “抓放检测到的红块”执行预抓、下探、双指接触、抬升、搬运、开爪放置、退回。成功需双指正接触力、物体实际抬升至少 70mm，并在放置位置落稳。没有 attach/weld。
- “只读镜像实机关节到 MuJoCo”只读取 `/zekeep/joint_states`。六关节 rad 原样映射，单指行程乘二得到夹爪总开口；模式内不推进物理、不接受抓放或普通运动。点击“重置仿真场景”恢复独立物理模式。此镜像不会发送实机命令。
- “仿真关节数组”只调用本进程的 HTTP 接口。任务执行中互斥；停止/网页心跳超过 3 秒时取消后续步骤并保持。
- 可选 `publish_ros:=true` 发布 `/zekeep_sim/joint_states`、`/zekeep_sim/tcp_pose`、`/zekeep_sim/mujoco/object_states`、`/zekeep_sim/camera/image_raw`、`/zekeep_sim/vision/detections`。没有 `/zekeep/*` 输出，也不提供实机同名服务。

## 模型与边界

模型运行时从现有 URDF 读取关节原点、轴、限位、TCP、质量和惯性，保留实机文件。夹爪每指行程 0–35mm，总开口 0–70mm。URDF 的 STL 作为外观，机械臂碰撞使用 MuJoCo 网格凸包；夹指接触使用 STL 末端范围的盒体近似。当前机器人自身碰撞未启用；此模型不能证明实机路径安全。

首版场景是一块 50mm 红色方块，抓取方向为竖直，放置位置沿 X 增加 60mm。物体默认质量 0.04kg，摩擦系数 1.2，桌面 Z=0。启动参数 `object_mass`、`friction`、`table_z` 可调整；参数尚未做实物接触标定。仅支持本场景的单次抓放，IK/接触/抬升/放置失败均明确返回失败。

无桌面浏览器也可使用：

```bash
MUJOCO_GL=egl PYTHONPATH=src/zekeep_sim python3 -m zekeep_sim.server --bringup src/zekeep_bringup
```

MuJoCo 和 NumPy 是运行依赖，离屏相机需要可用 EGL。图像失败时显示具体错误，不伪造检测。模型和渲染依据 [MuJoCo XML 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)与 [Python 文档](https://mujoco.readthedocs.io/en/stable/python.html)。
