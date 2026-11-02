# Zekeep 终端自然语言控制

使用 DeepSeek 生成工具计划，人工确认后执行。无需网页，不支持语音或图片输入。

## 1. 构建（首次或更新后）

先正常停止相关任务进程，再执行：

```bash
export ZKEEP_WS="$HOME/Zekeeparm_ws"
cd "$ZKEEP_WS"
source tools/activate_ros.sh
colcon build --packages-select zekeep_msgs zekeep_shadow zekeep_teach zekeep_llm zekeep_bringup \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
```

## 2. 启动终端助手

```bash
export ZKEEP_WS="$HOME/Zekeeparm_ws"
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ZKEEP_LLM_API_KEY="你的 DeepSeek API Key"

# 只生成计划，不调用 ROS；仍需联网请求模型。
ros2 run zekeep_llm terminal --dry-run "查询机械臂状态"

# 交互模式
ros2 run zekeep_llm terminal
```

在 `任务>` 后输入自然语言，例如“查询机械臂状态”“使能机械臂”“打开夹爪”。
写操作会显示计划，输入精确的 `yes` 才执行；其他输入取消。
驱动和 MoveIt 按需启动，已有服务直接复用，不自动使能。

单次指令执行后退出：

```bash
ros2 run zekeep_llm terminal "打开夹爪，然后回准备位"
```

## 3. 视觉抓取、分拣与堆叠

这些操作还需任务后端。另开终端执行以下命令，不会启动网页。
`motion_authorized:=true` 表示允许任务运动，仅在现场监督时使用；启动仍不自动使能。

```bash
export ZKEEP_WS="$HOME/Zekeeparm_ws"
cd "$ZKEEP_WS"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH PYTHONNOUSERSITE
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zekeep_bringup web_tasks.launch.py \
  grasp_backend:=yolo-graspnet start_vision:=true motion_authorized:=true \
  python_executable:="$HOME/miniconda3/envs/rebotarm/bin/python" \
  perception_root:="$ZKEEP_WS/src/zekeep_grasp"
```

保持后端运行，回到助手终端逐条输入所需指令：

```text
使能机械臂
有哪些放置点和识别类别？
当前桌上有什么？
仅夹取红色物块，不放置
把已夹住的物体放到 red_place
把红色物块放到 blue_place
把红色物块堆到蓝色物块上面
```

运动前检查计划并输入 `yes`。目标必须在当前配置类别中且唯一；同类多个目标会拒绝任意选择。
堆叠只支持两块不同颜色、实测边长 20–60 mm 的立方体，不支持多层塔。

## 参数

### 终端命令

| 参数 | 默认值 / 说明 |
| --- | --- |
| `"任务文本"` | 可选；省略进入交互模式 |
| `--dry-run` | 只生成计划，不启动驱动、MoveIt 或相机 |
| `--timeout` | 模型请求超时，默认 `30` 秒，范围 `1–120` |
| `--namespace` | 默认 `zekeep`，须与控制器和任务后端一致 |

### 视觉后端

| 参数 | 说明 |
| --- | --- |
| `grasp_backend:=yolo-graspnet` | 使用现有 YOLO/GraspNet 抓取流程 |
| `start_vision:=true` | 启动相机；已有视觉进程时设为 `false`，避免重复占用 |
| `motion_authorized` | 默认 `false`；视觉抓放需显式设为 `true` |
| `python_executable` | 已安装视觉依赖的 Python 路径 |
| `perception_root` / `perception_config` | 视觉项目根目录 / 配置路径；配置默认 `config/default.yaml` |

### 工具参数（由模型填入计划，执行前核对）

| 工具 | 参数 |
| --- | --- |
| `pick_color` | `color`：red/blue/green/yellow/purple；`place_after=false` 仅夹取并抬升，默认 `true` |
| `pick_and_place` | `object_name`：视觉类别原名，如 `red block`；`target_name`：已示教命名点 |
| `place_object` | 可选 `target_name`；省略则按已抓物体类别使用原放置点 |
| `stack_blocks` | `source_color`：抓取物；`target_color`：底部参照物；颜色须不同，另支持 gray |
| `move_joints` | `positions_rad`：六关节角；可选 `relative`、`oscillate`、`repeat=1–20`、`duration_s=0.2–60` |
| `move_to_pose` / `ik_check` | `position_m=[x,y,z]`、`rpy_rad=[r,p,y]`；坐标系 `base_link`，单位 m/rad；后者只求解 |
| `set_gripper_opening_mm` | `opening_mm=0–70`，同时受实际夹爪标定限制 |

`list_placement_targets`、`detect_blocks`、状态查询、使能/失能、回零/预备位均无参数。
命名放置点在 `src/zekeep_grasp/config/default.yaml` 的 `place.named_joint_targets_rad` 中配置，
当前已有 `red_place`、`blue_place`、`green_place`；不能凭料仓名称猜测坐标。

### 环境变量

| 变量 | 说明 |
| --- | --- |
| `ZKEEP_LLM_API_KEY` | 必填，DeepSeek 密钥；不要写入代码或提交到 Git |
| `ZKEEP_WS` | 工作区绝对路径 |
| `ZKEEP_LLM_PROXY_URL` | 可选，已有 HTTP/HTTPS 代理地址；设置地址不会自动启动代理 |
| `ZKEEP_VISION_PYTHON` | 可选，助手自动启动相机时使用的 Python，默认 `~/miniforge3/envs/rebotarm/bin/python` |

## 退出

输入 `exit` / `quit` 或按 `Ctrl+C` 退出助手。自动启动的驱动和 MoveIt 会继续运行，
不会因退出助手自动回零或失能；仅助手自己启动的相机随助手退出。
抓取后端的持物状态保存在内存中，持物时不要直接重启后端。
驱动与 MoveIt 启动日志：`~/.local/share/zekeep/llm_driver.log`、`llm_moveit.log`。
