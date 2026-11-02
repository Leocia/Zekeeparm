# zekeep_teach

通过 ROS 进行人工监督拖动示教、离散点记录和低速轨迹复现。
电机连接由 `zekeepcontroller` 独占，示教程序不另开串口。

## 启动

先完成[工作区构建](../../README.md)，在独立终端启动真机驱动及 MoveIt。
设置实际 `ZKEEP_WS` 并运行：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
ros2 run zekeep_teach teach_replay
```

默认参数位于 `config/default.yaml`。用 `ros2 run zekeep_teach teach_replay --help`
查看命令行选项。命名空间须与驱动一致，默认为 `zekeep`。

## 按键

| 按键 | 行为 |
| --- | --- |
| `T` | 进入重力补偿拖动 |
| 空格 | 记录离散点 |
| `U` | 撤销最后一个离散点 |
| `C` | 开始/停止连续记录；覆盖已有轨迹前询问 |
| `1` | 复现离散点 |
| `2` | 复现连续轨迹 |
| `S` | 保存 |
| `Q` | 请求安全退出、回零和失能 |

复现进行时只接受 `Q`。进入重力补偿可能使能电机，应先支撑机械臂。
标准配置允许人工监督补偿，但动力学尚未标记为验证完成。
退出期间保持驱动和 MoveIt 运行；回零失败应处理故障。

位置采用 ROS 弧度制，遵循当前驱动限位和方向，与独立 LeRobot 角度制配置不同。
详细流程见[拖动示教说明](docs/ros-hand-guided-teaching.md)。

## 网页实机示教与抓取任务

`web_tasks` 提供网页会话服务，复用现有示教状态、反馈采样、低速回放和安全退出。
默认 `motion_authorized=false`；由现场人员确认后显式授权。任务通过控制器专用 ROS
入口执行，控制器租约阻止普通运动抢占。会话保存拒绝覆盖同名文件，网页失联请求保持。

完成一次实机回放后，点击“清除本次记录”并确认，当前会话的静止点和连续路径归零。
清除不发送电机指令，不退出会话，也不删除已保存文件；录制或回放中禁止清除。
回放结束处于 HOLD 时，可先清除，再“进入拖动示教”，记录新的点或连续路径后回放。
GUIDING 状态下停止连续录制后也可清除。开始录制不会静默覆盖旧路径。

```bash
ros2 launch zekeep_bringup web_tasks.launch.py perception_root:="$ZKEEP_WS/src/zekeep_grasp"
```

`start_vision=false` 默认不开相机。启动参数与任务入口见
[网页说明](../zekeep_bringup/web/README.md)。抓取只接受新鲜、标定有效且静止采集的目标。
