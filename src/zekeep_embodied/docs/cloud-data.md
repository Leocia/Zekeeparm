# 数据打包与云端训练

ACT 与 π0.5 共用 `tools/collect.sh` 采集和质检；本节说明 π0.5 的划分、上传、导出和训练。
示例原始目录可替换为共用的 `data/demonstrations_v1/raw`。命令均从工作区根目录运行，示例路径替换为实际路径。
采集见[操作说明](pi05-entry.md)，模型依赖与权重见[环境说明](pi05.md)。

## 筛选与划分

```bash
./tools/collect.sh audit data/pi05_red_cube_v1/raw --output data/pi05_red_cube_v1/audit.json
./tools/collect.sh preview-raw data/pi05_red_cube_v1/raw/episode_实际名称 \
  --output data/pi05_red_cube_v1/review.mp4
```

`audit` 不修改原始记录，排除失败、模拟、未完成或损坏数据。`eligible` 只表示格式和数值合格，仍需人工回看。
报告和视频不覆盖已有文件。按完整 episode 划分，两个集合必须非空，同一示范不得拆帧分到两边；相似场景按采集批次分组。

```bash
mkdir -p data/pi05_red_cube_v1/train data/pi05_red_cube_v1/validation
ln -s "$PWD/data/pi05_red_cube_v1/raw/episode_训练示范" data/pi05_red_cube_v1/train/
ln -s "$PWD/data/pi05_red_cube_v1/raw/episode_验证示范" data/pi05_red_cube_v1/validation/
```

入口不自动划分、裁剪或平滑动作，原始数据保留。

## 打包与上传

```bash
./tools/collect.sh bundle \
  --train data/pi05_red_cube_v1/train \
  --validation data/pi05_red_cube_v1/validation \
  --output data/pi05_red_cube_v1/red_cube_v1.tar
./tools/collect.sh verify data/pi05_red_cube_v1/red_cube_v1.tar
```

打包拒绝无效示范、混合布局/标定及完全重复的示范；近似重复需人工筛选。符号链接展开成真实文件，`bundle.json` 记录划分和 SHA-256。
上传生成的 tar，不要仅上传本机符号链接。打包期间不要修改源数据；未压缩 tar 需额外预留接近所选数据总量的空间。
输出拒绝覆盖，失败产物可能保留为 `.tar.partial`。`verify` 校验文件和路径，不解压，也不认证发送者身份。

## 云端接收与训练

云端准备同版本源码及[模型环境](pi05.md)，无需 ROS 或硬件。先校验，再解压自己的数据包到新目录：

```bash
python3 src/zekeep_embodied/src/zekeep_embodied/bundle.py verify /数据路径/red_cube_v1.tar
# 仅在上一步成功后执行；目录应为新建空目录。
mkdir -p data/cloud
mkdir data/cloud/red_cube_v1
tar -xf /数据路径/red_cube_v1.tar -C data/cloud/red_cube_v1
```

导出统一使用 LeRobot v3。若已有 ACT 的 v3 train 目录，直接使用 `--dataset-root`，见[共用训练流程](pi05.md#同一份-v3-数据分别配置模型)，可跳过本节导出步骤。

```bash
./tools/pi05-model.sh doctor
./tools/pi05-model.sh export data/cloud/red_cube_v1/train \
  --repo-id local/zekeep_red_cube_v1 --fps 10
./tools/pi05-model.sh check --repo-id local/zekeep_red_cube_v1
./tools/pi05-model.sh stats --repo-id local/zekeep_red_cube_v1
./tools/pi05-model.sh check --repo-id local/zekeep_red_cube_v1
./tools/pi05-model.sh export data/cloud/red_cube_v1/validation \
  --repo-id local/zekeep_red_cube_val_v1 --fps 10

# 先检查一步训练，再以新实验名开始正式实验。
./tools/pi05-model.sh train --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_smoke_v1 --num-train-steps 1
./tools/pi05-model.sh train --repo-id local/zekeep_red_cube_v1 \
  --exp-name red_cube_v1 --num-train-steps 1000
```

- `--fps 10` 是当前示例导出频率，须符合实际采集时序；不插值或重复观测。图像统一为 v3 视频存储。
- 只用训练集计算统计；`check` 在统计前可为 `normalized: false`，之后应为 `true`。模拟数据不能用于正式统计或训练；统计绑定六轴相对、夹爪绝对的模型表示，ACT 的绝对动作统计不能直接代用。
- 默认 VLM LoRA + Action Expert 全量微调，参数范围见[模型说明](pi05.md#训练方式)。一步训练用于实测显存；1,000 步仅为实验示例，不保证学会任务。
- 数据集和实验使用新名称，默认拒绝覆盖。当前仅在训练结束时保存，不支持周期检查点、断点恢复或加载上次微调结果继续训练；中断会丢失本轮进度。
- 验证集单独保存，当前训练器未实现自动验证集评估。

## 训练结果

结果位于 `data/lerobot/checkpoints/pi05_zekeep_lora/<实验名>/`。部署需要一起保存：

- 匹配的基础权重、`adapter.safetensors` 和 `finetune.json`；adapter 包含 LoRA、完整动作专家及动作头。
- 同目录的原生 `config.json`、前后处理器、tokenizer、`pi05_stats.json` 和固定版本代码。
- 对应数据集目录中的 `capture_provenance.json`；离线 `predict` 还需要导出的数据集内容。

`pi05.py` 和 `finetune.py` 是共享代码，不因改用云端而删除。
按[实机操作](pi05-entry.md)依次验证离线预测、shadow 和监督执行。当前服务绑定本机回环地址，尚未接入云端远程推理；本地推理显存和速度需实测。
