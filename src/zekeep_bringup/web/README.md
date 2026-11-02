# Six-axis 网页仿真器

网页加载当前 bringup 包的 URDF 和网格，不依赖作者用户名或固定工作区目录。

## 启动

完成[工作区安装](../../../docs/INSTALL_PORTABLE.md)，设置实际 `ZKEEP_WS` 后执行：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
ros2 launch zekeep_bringup web.launch.py
```

网页地址 `http://127.0.0.1:3001`，ROS 连接 `ws://127.0.0.1:9090`。
Node 服务与 rosbridge 默认只监听本机；不能直接用另一台电脑访问这些端口。
该启动文件不启动真机驱动，实机控制需先启动驱动与所需的 MoveIt。

服务器使用 Node 内置模块，无需 `npm install`。仅查看模型时可独立运行：

```bash
node "$ZKEEP_WS/src/zekeep_bringup/web/server.js"
```

默认从服务器相邻的 bringup 目录读取模型；`ZKEEP_BRINGUP_SHARE` 可显式覆盖，
`ZKEEP_WS` 则指定源码工作区。若 source 安装包但仍设置 `ZKEEP_WS`，模型会读取该源码目录。
可选 text-agent/MCP 服务需单独部署，基础模型和 ROS 控制不依赖这些服务。

## 网页相机抓取统一启动

本机只需一个终端、一条命令；无需手动 source、export 或另开驱动终端：

```bash
$HOME/Desktop/Zekeeparm/tools/start_web_grasp.sh
```

脚本按自身路径找到工作区，自动隔离 ROS/SDK 与原视觉 Python 环境，启动网页、
rosbridge、MoveIt、YOLO＋GraspNet、任务后端和真机驱动。默认使用硬件配置中的
串口；可用 `--channel /dev/ttyACM0` 覆盖。视觉 Python 沿用本机
`~/miniforge3/envs/rebotarm/bin/python`，其他电脑可用 `ZKEEP_VISION_PYTHON` 指定。
SDK 环境沿用 `tools/activate_ros.sh`（默认 `.venv-ros`）。

AI 面板默认只能生成计划。要允许人工确认后执行 AI 计划，使用同一入口：

```bash
$HOME/Desktop/Zekeeparm/tools/start_web_grasp.sh --ai-motion
```

已有进程时，先结束任务、放下物体并按 Ctrl+C 等待安全退出，再用上述命令启动。
打开网页实机控制锁、使能机械臂，在 AI 面板生成新计划，勾选现场监督后点击
“确认执行计划”。授权不自动使能或执行，计划确认、互斥、取消与碰撞检查保留。
`--ai-motion` 和只读 `--preview` 不能同时使用；底层 launch 对应参数为
`ai_motion_authorized:=true`，与视觉任务授权相互独立。

普通网页控制默认关节速度上限为 `0.30 rad/s`、轨迹请求时长为 `2 s`，
实际时长仍按位移和三次插值的峰值速度自动延长。AI 关节运动峰值上限为
`0.30 rad/s`；视觉抓放的预抓搬运、放置和返回 Ready 上限为 `0.30 rad/s`，
下降、抬升仍限制在 `0.10 rad/s`。闭爪接触确认、释放停留、碰撞及到位检查保留。

启动允许网页中的人工监督控制，但不自动使能、不自动回 Ready、不自动抓取。
打开 `http://127.0.0.1:3001`，连接 `ws://127.0.0.1:9090`：

1. 核对现场、急停、关节反馈、工作台高度和搬运目标。
2. 打开网页控制锁并点击“使能”；必要时执行已验证的 Ready 姿态。
3. 保持机械臂静止，在“LLM / MCP 文本控制”查询“当前桌上有什么”，检查检测结果。
4. 使用 `--ai-motion` 启动后，可输入“抓取红色物块”，生成并核对计划，勾选现场监督后确认执行。

拆分夹取和放置：输入“仅夹取红块，不放置”，确认计划为
`pick_color`、`color=red`、`place_after=false`。成功后机械臂抬升并保持夹持，
不自动放置或回 Ready。之后输入“放置已夹住的物体”，确认 `place_object`，
使用既有放置配置移动、释放，再按配置返回 Ready。“夹取并放置”仍支持原完整流程。
单独放置不需要新物块检测，但必须已成功完成夹取抬升，且驱动、MoveIt、使能、
标定与运动授权有效。持物状态不跨后端重启恢复。

持物回预备位：输入“夹取红块后回我的预备位，不放置”，核对计划按顺序为
`pick_color`（`place_after=false`）和 `return_ready`。夹取、抬升成功后回现有配置的
预备位，保留夹持和附着物碰撞模型，不自动释放。目标沿用
`zekeep_grasp/config/default.yaml` 的 `official_sdk.ready_joints` 对应预备位；
当前与驱动的 `ready_pose.positions`、MoveIt 的 `ready_pose` 一致。
其他位置需明确六个关节角，或完整的 base_link 位置和 RPY；不自动猜测目标。


复用 `zekeep_grasp/scripts/main.py` 的 YOLO 分割、GraspNet 参数、中心测量、姿态、
IK 和夹爪碰撞筛选，运动通过 ROS/MoveIt 执行。不需要另外填写固定 RPY 和物块
尺寸。相机内参、手眼标定、设备身份、工作空间和 `safety.table_z_m` 沿用已有
配置；没有可靠结果时拒绝运动。目标类别沿用 `yolo.custom_classes`。

颜色抓取支持 `red`、`blue`、`green`、`yellow`、`purple`；实际能否检测到还取决于
YOLO 类别配置和画面。默认类别已加入 `purple block`，可输入“仅夹取紫色物块”
或“夹起紫色物块后回预备位，不放置”。紫色没有单独示教放置点，完整搬运使用
原默认固定点。增加开放词汇类别不保证颜色准确率，误识别仍需实物画面验证。
默认 `yolo.imgsz=1280`，让小物块使用更高分辨率推理，检测阈值仍为 0.3。
这会增加显存和推理开销；未配置 `imgsz` 的旧配置仍使用 640。网页预览的
640×400 缩放发生在检测后，不决定 YOLO 输入尺寸。

GraspNet 的 `grasps=0/27/214` 表示宽度筛选后剩 0 个、目标筛选后 27 个、
碰撞筛选后 214 个。`width=[0.072,0.082]m` 是候选夹爪开口范围；当前
`robot.gripper_max_width_m=0.070`，超限候选全部拒绝，状态同时显示宽度限值。
不要为绕过拒绝而扩大实机夹爪限值。

YOLO/GraspNet 抓取在任务开始时选取一次新鲜、已标定且机械臂静止的 RGB-D
快照，并固定该快照的目标、点云和抓取位姿。GraspNet 与 MoveIt 规划后、开爪后
不再要求实时 YOLO 重新看见目标，也不再等待连续三个新观测；后续检测抖动、
漏检或遮挡不会取消已规划的抓取。任务开始时仍拒绝过期快照、无有效深度、
无标定或不唯一的目标，规划和执行前仍检查机械臂位置与静止反馈。
此流程适用于静止桌面物块：选定快照后不要移动或取走物块；物块移动时应取消
任务并重新采集。旧固定几何后端仍保留运动前的多帧确认。
YOLO 后端的位置校验复用原视觉流程的掩膜点云、上表面高度和物体中心测量，
使用 `position_reference=measured_object_center`；旧固定后端保留局部深度点。
GraspNet 使用所选 RGB-D 快照独立规划，不占用实时 YOLO 检测锁。
TF 按采集时间查询，最多等待 0.2 秒；等待后仍检查 0.25 秒采集时限和静止关节反馈。
AI Action 失败时显示后端具体原因，保留原有停止、取消及限位检查。

统一启动同时加载已有工作台场景节点：夹爪三条链接沿用 3 mm 碰撞余量，
其他机械臂链接保留 20 mm 默认余量。下降路径必须完整通过碰撞检查，拒绝部分路径。

只有接触确认才抬升。完整流程且 `place.enabled=true` 时按实际识别类别搬运：
`red block` 使用 `red_place`，`blue block` 使用 `blue_place`，`green block`
使用 `green_place`；其他类别仍使用原 `place.joint_target_rad` 固定点。
三个点来自 `ready_3poses_20261009_095828.json`，存于视觉配置的
`place.named_joint_targets_rad`，并在 SRDF 中提供同名位姿。仅夹取后再单独放置
仍使用夹取成功时的类别。释放后按配置返回 Ready；关闭搬运时保持夹持，
可确认承接面后手动释放。
其他搬运模式在运动前拒绝。任务互斥、心跳、取消和低速上限保留，抓取授权不
传给 AI 后端。视觉进程不连接机械臂串口，原 CLI 的直接 SDK 流程仍保留。

驱动缺失、SDK 环境错误或连接失败时，网页与视觉后端继续运行，实机抓取不可用；
MoveIt 退出也不会关闭网页。加载视觉权重需要等待；后端画面话题仍为
`/zekeep/web/camera/image_raw`，网页不再显示画面。只读模式使用同一个入口加 `--preview`，
不启动驱动，也不授权运动；检测查询仍要求有效标定与新鲜的静止关节反馈。

结束任务并放下物体后，按一次 Ctrl+C。脚本先让驱动安全退出，保留 MoveIt 到
驱动退出成功，再关闭网页和视觉。回零失败时保留驱动、保持控制和 MoveIt；
处理故障后再次按 Ctrl+C 重试，不强杀驱动。

首次从旧启动方式切换时，先按原顺序关闭旧流程：任务结束、驱动安全退出，再
关闭旧网页、MoveIt 和视觉。新入口会检查重复启动和旧端口/节点占用，避免两个
进程同时控制机械臂或占用相机；不会自动结束其他进程。

更新后只需构建一次（本机本次修改已构建）：

```bash
cd $HOME/Desktop/Zekeeparm
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m colcon build --base-paths src \
  --packages-select zekeep_bringup zekeep_shadow zekeep_teach zekeep_moveit_config
```

新电脑先按 [SDK 安装说明](../../../docs/SDK_INSTALL.md)和视觉安装说明配置环境。
新增执行接口仍需现场低速验收，自动检查不等于实机验收。

高级用法仍保留 `ros2 launch zekeep_bringup web_grasp.launch.py`（不包含驱动，
默认不授权运动）；原固定姿态方案可选择 `grasp_backend:=fixed`，其人工确认
参数要求不变。

## 网页功能

- 精确显示 Six-axis URDF 外形和材质。
- 主模型跟随 `/zekeep/joint_states` 实际反馈。
- 半透明目标模型显示待发送姿态。
- 支持零位、Ready、关节滑块、夹爪宽度控制。
- 姿态预设先选择并预览，再通过“执行到所选姿态”按钮执行；控制锁打开时发送实机。
- Pose 输入支持 X/Y/Z（米）和时长。XYZ 输入提交及自动填入统一保留两位小数，步长为 0.01 m，预览和 Pose 执行使用显示的坐标。
- XYZ 范围沿用网页工作空间估算：按关节限位采样当前 URDF 的 `link6` TCP，X/Y 使用正负水平半径，Z 上限使用高度，向上取整至厘米（沿用现有 0.65 m 最小估算值）；Z 下限为 0 m，沿用网页地面约束，并非机械臂理论最低可达高度。当前范围为 X/Y [−0.65, 0.65] m、Z [0, 0.74] m；模型加载后自动设置并显示，超限会阻止 IK 求解和执行，但点击预览仍显示红色目标小球。这是粗略输入边界，不保证范围内目标可达或无碰撞。
- 点击“获取当前 Pose”将网页当前主模型的 TCP 位置（ROS `base_link`，米）填入 XYZ，保留所填时长；不会自动预览或执行运动。
- 编辑 XYZ 或时长后，点击“预览目标”才显示目标小球与连线；连接 ROS / MoveIt 后，只读 IK 生成半透明机械臂预览，按所填时长播放，速度限制需要延长时显示预览时长。修改参数会清除旧预览，需再次点击按钮。主模型保留实际反馈。
- 目标 IK / 碰撞 / 关节限位校验未通过时，小球保持显示并标红，不播放虚影动作；离线或服务异常保留橙色小球，并提示尚不能确认可达性。
- 预览不会发送实机运动命令。点击“IK 运动”后才重新求 IK、核对限位并执行；预览不代替最终求解或整条路径的碰撞验收。
- 未连接 ROS 或控制锁关闭时，Pose 在网页本地仿真。
- Pose 的 IK 姿态默认保持当前 TCP 姿态，避免单位四元数导致不可达。
- 连接 ROS 并打开控制锁后，Pose、规划轨迹和示教轨迹按对应按钮直接发送实机。
- 开始新示教录制时，如已有路径会明确询问是否覆盖；取消后旧路径保持不变。
- 关节和夹爪指令仍受控制锁保护。
- 保留可折叠的 TCP 目标预览、网页示教和实机示教面板；相机/单次抓取面板已移除，视觉功能由文本控制调用。
- 碰撞几何会加载，但不在网页中显示。

## 坐标和夹爪

Pose 输入使用 ROS `base_link` 坐标系，单位米。网页模型转换为 Three.js 坐标：

```text
Three.js = (ROS X, ROS Z, -ROS Y)
```

J1 `±2.58 rad`、J2 `0–3.7 rad`、J3 `-0.01–3.7 rad`、J4–J6 `±1.57 rad`，与 ROS 模型一致。
夹爪界面开口 0–70 mm，实机换算读取控制器的只读参数；当前配置为电机
0–1.15 rad / 0–70 mm。没有取得有效配置时拒绝实机夹爪命令。URDF `gripper_joint` 表示单指平移 0–0.035 m。

## 实机安全

默认只读，前端控制锁是操作保护，不是服务端认证。控制实机前确认：

1. ROS 控制器、关节反馈和 rosbridge 正常；
2. 急停可用、机械臂周围无人；
3. 网页反馈姿态与实机一致；
4. 勾选“允许网页向真实机械臂发控制”。打开后运动按钮会直接下发实机。

## 控制与任务入口

- TCP 拖拽仅更新目标预览，点击“执行 TCP 目标”才提交。
- 网页示教是本地编辑轨迹，JSON 为 `zekeep_ros_waypoints_v1`；实机示教记录真实关节反馈，使用 `zekeep_teach` 会话格式。两者分别标记来源。
- 实机示教提供“清除本次记录”：人工确认后清空当前点和连续路径、重置录制时间；已保存文件保留。录制或回放中禁止清除。每次开始新一轮示教前先清除，再进入拖动示教并记录。
- 网页示教实机回放逐段等待 Action，再等待该端点夹爪结果，失败或取消不继续。
- “停止并保持”接入 `/zekeep/stop` 与任务取消。只有后端确认后显示停止；失能是独立操作。
- 控制器任务租约保护实机示教/抓取与普通控制互斥；网页或任务进程失联时请求保持。
- 统一视觉入口复用现有 YOLO/GraspNet，只接受静止机械臂采集的标定有效目标。
  单次任务执行预抓、下探、接触确认、抬升，以及配置中启用的搬运。
  独立 `web_tasks.launch.py` 的默认 `fixed` 后端保留旧红色物块方案。

可选任务服务启动：

```bash
ros2 launch zekeep_bringup web_tasks.launch.py   perception_root:="$ZKEEP_WS/src/zekeep_grasp"
```

默认不授权运动，不打开相机。现场授权后再设置 `motion_authorized:=true`、`start_vision:=true`。
抓取还要求确认抓取姿态、量测物体尺寸与工作台，显式设置 `grasp_geometry_confirmed:=true`。
## 文本控制

侧栏保留“LLM / MCP 文本控制”“MCP 工具调试”。MuJoCo 入口及其网页轮询已移除；独立仿真后端仍可单独使用。

```bash
ros2 launch zekeep_bringup web_ai.launch.py  # 默认禁止运动；密钥只在后端环境
```

AI 服务默认地址为本机 8082，可在网页启动前用 `ZKEEP_TEXT_AGENT_URL` 指定实际本机地址；网页不能自行改变代理目标。Pose 预览仅开放只读 `/compute_ik` 服务。更新本次修改后需重启 rosbridge 以加载服务白名单，并强制刷新网页。

具体契约和配置：[AI README](../../zekeep_llm/README.md)、[独立仿真 README](../../zekeep_sim/README.md)。

`web.launch.py` 现在默认同时启动文本 LLM/MCP 后端（8082），运动授权仍为 false。已有独立 AI 服务时，使用 `start_ai:=false` 避免端口冲突。相机、实机控制器和物理仿真仍分别按需启动。
