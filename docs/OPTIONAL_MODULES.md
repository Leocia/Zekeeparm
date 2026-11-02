# 选装模块

基础安装不安装模型训练、Studio 或 MuJoCo。以下操作从项目根目录运行，需要网络。
所有安装入口只安装依赖，不连接设备或使能电机；本包不附模型、训练结果或数据。

## 共用数采

```bash
bash tools/install_optional.sh collect
./tools/collect.sh doctor
```

`embodied` 环境使用独立的采集依赖；相机进程仍使用 `rebotarm`，ROS 策略端保留 ROS 环境。
先填写 `src/zekeep_embodied/configs/record.yaml` 的设备路径、安装标识、工作区域；新设备默认未确认。
`camera-check` 会打开相机；本次交付验证不执行该命令。

## ACT 与 π0.5 模型

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

```bash
bash tools/install_optional.sh sim
bash tools/env.sh ros ros2 launch zekeep_bringup web_sim.launch.py python_executable:="$PWD/.venv-ros/bin/python"
```

只启动独立仿真，默认不发布 ROS；启用发布时只使用 `/zekeep_sim/*`。
离屏图像需要有效 EGL。该仿真不能证明真机路径安全。见 [模块说明](../src/zekeep_sim/README.md)。

## 独立网页仿真

需在当前终端先安装并选择 Node.js 22+；基础安装的 Ubuntu Node.js 可能版本较旧。

```bash
bash tools/install_optional.sh web-sim
cd web_mujoco
export PATH="$PWD/.venv/bin:$PATH"
npm run dev
```

访问 `http://127.0.0.1:5174`。模型来自 `web_mujoco/robot`，导出资源在安装/构建时生成。
不连接机械臂，不需要 ROS。见 [网页仿真说明](../web_mujoco/README.md)。

## Studio（独立提供）

Studio 源码不随 GitHub 仓库发布。另行取得后，先将目录放到 `apps/studio/`，再执行以下安装步骤。

当前 Blockly 依赖要求较新的 Node.js 与 npm：可用 Node.js 22.22.2+（22.x）、24.15+（24.x）或 26+。
安装器用 `npm ci --engine-strict` 核对锁定依赖的实际要求；先在当前终端准备这两个工具。

```bash
bash tools/install_optional.sh studio
bash tools/env.sh studio python apps/studio/studio.py
```

默认启动行为与模拟/真机选择见另行提供的 `apps/studio/README.md`。
浏览器依赖通过 npm 安装；原 SDK 和硬件接口保留。Windows 打包仍属未验证范围。
