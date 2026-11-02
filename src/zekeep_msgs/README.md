# zekeep_msgs

驱动、示教客户端和网页桥接共用的 ROS 2 接口定义。

| 目录 | 接口 |
| --- | --- |
| `msg/` | `ArmStatus`、`JointMitCmd`、`JointMotorState`、`JointPosVelCmd` |
| `srv/` | `GripperCommand`、`MoveToPoseIK`、`SetGripper`、`SetZero` |
| `action/` | `GripperGrasp`、`MoveToPose`、`WebGrasp`、`WebManipulate` |

字段和单位以 `.msg`、`.srv`、`.action` 文件为准。
轨迹执行使用标准 `control_msgs/action/FollowJointTrajectory`；使能、失能、停止、回零
使用 `std_srvs/srv/Trigger`，这些接口不在本包重复定义。

## 构建与查看

先完成[环境安装](../../docs/INSTALL_PORTABLE.md)，设置实际 `ZKEEP_WS`：

```bash
cd "$ZKEEP_WS"
source "$ZKEEP_WS/tools/activate_ros.sh"
python -m colcon build --base-paths src --packages-select zekeep_msgs
source "$ZKEEP_WS/tools/activate_ros.sh"
ros2 interface show zekeep_msgs/msg/ArmStatus
ros2 interface show zekeep_msgs/srv/SetGripper
ros2 interface show zekeep_msgs/action/MoveToPose
```

默认接口实例位于 `/zekeep`，服务表见[控制器说明](../zekeepcontroller/README.md)。
修改定义后需同步修改生产者与消费者，重新构建依赖包并重启相关节点。

`WebManipulate` 位于 `/zekeep/web/manipulate`，用于已确认的单目标分拣和两色块堆叠。
`detection_id` 指定抓取对象，留空则只放置当前后端已抬升的物体；目的地必须且只能
选择一个 `target_name`（已示教命名点）或 `support_detection_id`（新鲜、实测的参照色块）。
接口不接受任意位姿，复用网页任务授权、心跳、取消和 `grasp` 控制器租约。
原 `WebGrasp` 字段及 `/web/grasp`、`/web/pick`、`/web/place` 保持兼容。
