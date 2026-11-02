# zekeep_grasp

Zekeep 直接 SDK 视觉抓取、RGB-D 相机与标定工具。

## 环境与启动

先按[安装说明](../../docs/INSTALL_PORTABLE.md)准备视觉环境、Orbbec SDK、模型权重和 CUDA 扩展，
再按[SDK 安装说明](../../docs/SDK_INSTALL.md)安装 reBotArm SDK。设置 `ZKEEP_WS` 为实际工作区绝对路径，
使用独立 `rebotarm` 环境，不把视觉 NumPy 约束安装进 ROS venv。

主抓取入口：

```bash
conda activate rebotarm
cd "$ZKEEP_WS/src/zekeep_grasp"
python scripts/main.py --help
python scripts/main.py --config config/default.yaml \
  --target-class "red block"
```

`main.py` 直接占用官方 SDK 串口，不得与 `zekeepcontroller`、Gateway 或其他
SDK 进程同时运行。启动后会连接机械臂并执行准备位运动；先完成相机内参、手眼标定和现场检查，再运行上述命令。

## 鼠标选取与连续抓取

新增 [鼠标选取入口](scripts/click_grasp.py) 和
[连续抓取入口](scripts/continuous_grasp.py)，原 `scripts/main.py` 用法保持不变。
这两个入口是直接运行的 Python 脚本，无需 `colcon build`；先完成上述环境与模型安装。

```bash
conda activate rebotarm
cd "$ZKEEP_WS/src/zekeep_grasp"

# 无需参数；左键点击识别到的物体，立即抓取并放置。
python scripts/click_grasp.py

# 连续抓取所有红色方块，包括同类的多个实例。
python scripts/continuous_grasp.py --target-class "red cube"

# 连续抓取所有识别到的方块。
python scripts/continuous_grasp.py --target-class cube
```

例如桌面上有两个 `red cube`，第一条连续抓取命令会依次抓取两个红色方块，
每次放置后返回准备位，再识别并抓取下一个。`cube` 则依次处理首轮识别到的所有颜色方块。

鼠标模式按物体分割区域选择具体实例，不会按类别换成另一个物体。
每轮结果停留显示，右键或 `R` 恢复画面，再左键选择下一个；点击背景不触发抓取。
连续模式将 `cube` / `red cube` / `redcube` 对应到现有 `block` / `red block`
识别类别，每轮完成后重新识别；未传类别时使用配置类别，配置为空则抓所有方块。
任务范围为首轮识别到的目标区域，成功后移除已处理区域，避免反复抓取放置区物体。
后续只处理与未完成区域的分割重叠度至少 50% 的检测，不追加后来放入的物体。
无匹配目标时结束；无可执行候选、抓空或执行故障时停止，不自动重复失败目标。
识别漏检或剩余物体移动也会结束本轮，并打印未完成区域数，可检查画面后重新启动。

两种模式均复用原有标定、抓取筛选和按识别类别放置逻辑。
红、蓝、绿方块使用对应放置点，其他类别使用原默认点；`cube` 不包含杯子、笔等类别。
启动会连接 SDK 并运动到准备位，连续模式随后自动抓取；`Q` / `Esc` 在画面响应时退出，
沿用原流程回关节零位、松爪并断使能。运动执行期间界面阻塞，键盘不能作为急停。
勿与 ROS 驱动或其他 SDK 进程同时运行。采集后目标必须保持静止。
同类物体沿用同一固定放置点，不自动调整堆叠高度；多物体放置空间需现场确认。

离线检查在上述 `rebotarm` 环境、本目录执行，不打开相机或连接机械臂：

```bash
python -m unittest discover -s test -v
python ../zekeep_bringup/test/test_color_placement.py -v
python scripts/click_grasp.py --help
python scripts/continuous_grasp.py --help
```

本次验证通过 20 项离线测试、Python 语法检查和两个入口的 `--help` 检查；
尚未进行实机抓取验证。

## 色块颜色校准

默认加载 `yolo.color_calibration` 指定的
`config/calibration/orbbec_gemini2/block_colors.json`。当前已采样红、紫、蓝、绿四色。
`hue_center` / `hue_radius` 使用 OpenCV 的 H 范围 `0–179`，不是角度；红色跨越首尾边界。
YOLO 仍负责定位和分割；颜色由真实分割区域内的 HSV 像素校验。
颜色不确定、未采样颜色或只有检测框时返回普通 `block`，不会被按颜色抓取选中。
YOLO 检测分数仍是检测分数，不代表颜色可信度。内参、手眼和相机预设不变。

换相机、灯光或色块后，重新采图并用色块内部矩形取样：

```bash
python scripts/capture_single_frame.py --out-dir data/color_calibration/new --warm-up-frames 60
python scripts/calibrate_block_colors.py --help
# 每个 --sample 格式为 red:X1,Y1,X2,Y2，可重复取不同面的样本。
# 必填：--image 图片路径 --sample 颜色:矩形坐标 --out 校准JSON路径。
```

矩形右下坐标不包含在取样范围内；避开背景、边缘和反光。
按实际颜色人工标注，不用模型标签；换位置和朝向复验后再启用新文件。
直接 SDK、鼠标/连续抓取和网页/LLM 的 YOLO 流程共用此校验。
修改后重启视觉进程生效；没有修改 ROS 消息，无需重新编译。

## 当前配置与标定

主模型由 `robot.urdf_path` 指向 bringup URDF，SDK 路径由 `robot.repo_root` 解析。
这些相对路径从视觉项目目录解析，不要求固定用户名或工作区目录。
J1 `±2.58 rad`、J2 `0–3.7 rad`、J3 `-0.01–3.7 rad`、J4–J6 `±1.57 rad`，六轴方向均为 `-1`，与 ROS 一致。
准备位使用 `official_sdk.ready_joints`，应与 SRDF 的 `ready_pose` 一致。
joint6 零位法兰已按当前安装绕自身轴增加 `+90°`；准备位 TCP 姿态已按新 URDF FK 重算，
关节值和方向保持不变。

现有设备手眼标定已完成，以当前参数为准。仅换电脑且相机、机械臂、安装和场景不变时，
直接沿用当前内参与手眼文件，无需生成或刷新身份指纹。
CLI、鼠标/连续抓取和网页入口均不读取 `identity.local.json`，缺失、过期或损坏不会阻止启动。
相机序列号仍须与 `camera.serial` 一致，标定文件、参考帧和运行相机参数仍需通过检查。

需要更新配置中的相机序列号时，可选用原工具：

```bash
python scripts/confirm_calibration.py --config config/default.yaml --confirm-same-installation
```

该脚本只打开相机，不连接机械臂。它自动读取当前相机序列号，更新
配置中的 `camera.serial`，并保存可选的 `identity.local.json` 快照。
每次更新前备份到 `config/calibration/backups/device_update_<时间>/`；更新失败时恢复原文件。
可通过 `--config /实际路径/config.yaml` 更新其他配置，只替换其中的 `camera.serial` 行。
工作区路径由脚本位置确定，现有相对路径随工作区迁移；机械臂 ID 不随电脑名改变。

`--confirm-same-installation` 表示确认当前内参、手眼文件属于当前相机、机械臂、安装和场景。
更换相机或安装后，先完成相应标定，再更新配置中的序列号及安装信息。
脚本不重新计算标定，也不调整控制模式、增益、放置点或安全容差。
更新记录不能代替重新标定。软件不再通过持久指纹判断配置或标定是否属于当前安装，
操作者需核对设备、安装和场景；网页仍检查运行中视觉与执行端使用相同的标定配置。

## 抓取参数

下降高度由 `config/default.yaml` 中的参数直接控制：

- `grasp_pipeline.grasp.pregrasp_offset_m` 是预抓点到抓取点的下降量。
- `grasp_pipeline.grasp.grasp_center_z_offset_m` 是最终 TCP 相对物体中心的 Z 修正。
- `grasp_pipeline.grasp.allow_shorter_descent_fallback` 默认为 `false`，因此不会
  在 IK 失败时静默换成更短的下降量。

低于安全线或发生碰撞的请求会明确失败，不会被静默改成更高的目标。

ROS 标准开爪目标为 `1.15 rad`，本包 `robot.gripper` 当前 `angle_open=1.30 rad`；
这是仍存在的配置差异，不代表两条执行路径已统一。

主抓取入口没有 `--dry-run` 参数，不能用它做无运动检查。
相机预览使用 `python scripts/camera_preview.py`，仅用于查看图像。

## 按识别类别放置

直接 SDK 和网页 YOLO/GraspNet 流程共用 `place.class_targets`：识别为
`red block`、`blue block`、`green block` 时，分别使用 `red_place`、
`blue_place`、`green_place`。其他类别仍使用原 `place.joint_target_rad` 固定点。
按实际识别类别选择，不按用户请求文本或物品名称中的颜色单词猜测。

三个放置点来自工作区 `ready_3poses_20261009_095828.json` 的第 1、2、3 个记录，
原始记录保留；关节值存于 `place.named_joint_targets_rad`，并同步为 SRDF 的
同名 `arm` 位姿。顺序为 joint1–joint6，单位 rad，不能再次取反或转成角度写入。
分开执行夹取、放置时，网页后端保留夹取成功时的类别，后续放置无需重新识别。

更改放置配置无需刷新身份记录；更改前仍需核对设备、安装、模型及标定文件。
配置或标定变化后重启网页视觉与执行进程，确保两端使用相同配置。
网页搬运保留 MoveIt 规划、
持物碰撞检查、低速及取消机制；直接 SDK 的关节搬运仍不做环境路径碰撞规划。
首次使用新点需现场低速验证放置高度、夹爪朝向和搬运路径。

## 网页 ROS 执行入口

推荐用工作区的 `tools/start_web_grasp.sh` 一次启动网页、原视觉、MoveIt 和驱动，
不自动使能或抓取，退出时保持驱动先于 MoveIt 安全退出。底层
`ros2 launch zekeep_bringup web_grasp.launch.py` 复用本包 `main.py` 的 YOLO、
GraspNet 配置、中心测量和候选筛选，通过现有 ROS 驱动与 MoveIt 执行。
视觉进程不调用 `OfficialSdkRobot.connect()`，不占用电机串口；原 CLI 保持
直接 SDK 执行。网页和视觉预览可先于驱动启动，默认不授权运动。
默认后端为 `grasp_backend:=yolo-graspnet`，尺寸和姿态由现有视觉流程估计。
网页抓取选取一次新鲜、已标定且静止采集的 RGB-D 快照后，固定目标进行
GraspNet 和 MoveIt 规划；不在规划后或开爪后反复要求实时检测到目标。
初始快照、标定、机械臂反馈、限位、碰撞、取消及夹爪接触检查仍保留。
该流程要求物块在采集后保持不动，移动或取走物块时应取消并重新采集。
完整启动、使能、抓取及退出流程见 [网页说明](../zekeep_bringup/web/README.md)。

## 目录与诊断

标定工具位于 `calibration/`，相机驱动位于 `drivers/camera/`，坐标变换、抓取
候选筛选和执行安全检查位于 `utils/`。
位姿误差诊断入口为 `scripts/diagnose_pose_error.py`（直接 SDK）和
`scripts/diagnose_ros_pose_error.py`（ROS/MoveIt）；默认只规划，`--execute` 才执行运动。
