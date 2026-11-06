# 选装模块

以下模块按需独立安装，不要求先装整套环境。所有操作从项目根目录运行，需要网络。
公共前置、模块组合及完整依赖清单见 [分模块安装说明](INSTALL_PORTABLE.md)。
选装脚本不执行 sudo；所需系统包、Conda 和 Node.js 须按该说明先准备。
所有安装入口只安装依赖，不连接设备或使能电机；本包不附模型、训练结果或数据。

## 共用数采

前置：Conda；系统 `git`、`ffmpeg`、`build-essential`。实际双相机采集还需先安装 `ros` 与 `vision`。
只做离线数据检查/打包时，可仅安装 `collect`。

```bash
bash tools/install_optional.sh collect
```

`embodied` 环境使用独立的采集依赖，并安装本地主臂插件 `src/zekeep_teleop`；无需另建 `lerobot` 环境。相机进程仍使用 `rebotarm`，ROS 策略端保留 ROS 环境。
安装后先填写 `src/zekeep_embodied/configs/record.yaml` 的任务、设备路径、非空 `installation_id` 和工作区域；
新设备默认未确认，现场核验后再将 `workspace.confirmed` 设为 `true`。
相机解释器优先使用 `ZKEEP_VISION_PYTHON`，未设置或为空时使用 YAML 中的 `cameras.wrist.python`。
完成配置后再运行无硬件检查：

```bash
./tools/collect.sh doctor
```

`camera-check` 会打开相机；本次交付验证不执行该命令。

## ACT 与 π0.5 模型

前置：Conda；系统 `git`、`ffmpeg`、`build-essential`，GPU 训练另需可用 NVIDIA 驱动。
离线导出、训练和模型服务不要求 ROS、视觉或独立遥操作环境；真机策略执行须补齐 `ros`、`vision`、`collect`。

```bash
bash tools/install_optional.sh models
bash tools/env.sh model python -m pip check
./tools/pi05-model.sh doctor
```

共用环境为 Conda `act`：Python 3.10、LeRobot 0.4.4、PyTorch 2.7.1/cu128、torchvision 0.22.1、
TorchCodec 0.3.0、PyAV 15.1.0、NumPy 2.2.6。π0.5 的 Transformers 固定为源码中的官方分支提交。
这里的 TorchCodec 配对依据[官方兼容表](https://github.com/pytorch/torchcodec#compatibility-with-torch-versions)，
π0.5 依赖依据 [LeRobot v0.4.4](https://github.com/huggingface/lerobot/blob/v0.4.4/pyproject.toml)。
安装后 `pip check` 和模型导入检查必须通过；新电脑完整安装与显存需求仍须现场验证。

π0.5 权重和 tokenizer 按需获取：

```bash
bash tools/env.sh model hf download lerobot/pi05_base --local-dir "$HOME/.cache/zekeep/pi05_base"
# 先在模型网站接受许可，并用 hf auth login 登录自己的账户。
bash tools/env.sh model python -c 'from transformers import AutoTokenizer; AutoTokenizer.from_pretrained("google/paligemma-3b-pt-224")'
```

ACT 的 ResNet 预训练骨干在首次使用时联网获取；用户训练模型需自行训练或指定完整结果目录。
π0.5 基础模型并不保证能执行本机械臂任务，需匹配的数据和微调结果。

现有 `pi05.sh` 入口保留，当前模型路径转发到 `pi05-model.sh`，数采转发到 `collect.sh`。
旧 OpenPI 的 adapter 不自动转换，原实现及数据仍在旧包备份中。

详细流程：[ACT](../src/zekeep_act/README.md)、[数采](../src/zekeep_embodied/README.md)、
[π0.5](../src/zekeep_embodied/docs/pi05.md)。执行端仍保留人工确认、deadman、限位、限速和超时保护。

## ROS MuJoCo

前置：先执行 `bash setup.sh ros`。本模块只向现有 ROS 环境加入 MuJoCo 3.10.0，并构建 `zekeep_sim`。
不依赖 `vision`、`teleop` 或模型权重。

```bash
bash tools/install_optional.sh sim
bash tools/env.sh ros ros2 launch zekeep_bringup web_sim.launch.py python_executable:="${ZKEEP_ROS_VENV:-$PWD/.venv-ros}/bin/python"
```

只启动独立仿真，默认不发布 ROS；启用发布时只使用 `/zekeep_sim/*`。
离屏图像需要有效 EGL。该仿真不能证明真机路径安全。见 [模块说明](../src/zekeep_sim/README.md)。

## 独立网页仿真

前置：`sudo apt-get install -y python3-venv python3-pip`；当前终端的 Node.js 22+ 和 npm。
先执行 `node --version`、`npm --version` 确认。Ubuntu Node.js 可能版本较旧。
此模块不要求 `setup.sh`、Conda、ROS、CUDA Toolkit 或真机。
脚本创建 `web_mujoco/.venv`，安装 Python 导出依赖，执行 `npm ci`、模型导出和网页构建。

```bash
bash tools/install_optional.sh web-sim
cd web_mujoco
export PATH="$PWD/.venv/bin:$PATH"
npm run dev
```

访问 `http://127.0.0.1:5174`。模型来自 `web_mujoco/robot`，导出资源在安装/构建时生成。
不连接机械臂，不需要 ROS。见 [网页仿真说明](../web_mujoco/README.md)。
