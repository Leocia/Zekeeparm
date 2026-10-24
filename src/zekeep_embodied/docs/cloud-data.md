# 数据打包与云端交付

本地用 `tools/pi05.sh` 准备数据，打包后上传云端训练。命令均从工作区根目录运行，示例路径替换为实际路径。
采集见[总 README](../../../README.md#示范数据采集与云端交付)和[操作说明](pi05-entry.md)。本地采集、检查和打包无需模型权重或 OpenPI。

## 筛选与划分

```bash
./tools/pi05.sh audit data/pi05_red_cube_v1/raw --output data/pi05_red_cube_v1/audit.json
./tools/pi05.sh preview-raw data/pi05_red_cube_v1/raw/episode_实际名称 \
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
./tools/pi05.sh bundle \
  --train data/pi05_red_cube_v1/train \
  --validation data/pi05_red_cube_v1/validation \
  --output data/pi05_red_cube_v1/red_cube_v1.tar
./tools/pi05.sh verify data/pi05_red_cube_v1/red_cube_v1.tar
```

打包拒绝无效示范、混合布局/标定及完全重复的示范；近似重复需人工筛选。符号链接展开成真实文件，`bundle.json` 记录划分和 SHA-256。
上传生成的 tar，不要仅上传本机符号链接。打包期间不要修改源数据；未压缩 tar 需额外预留接近所选数据总量的空间。
输出拒绝覆盖，失败产物可能保留为 `.tar.partial`。`verify` 校验文件和路径，不解压，也不认证发送者身份。

## 云端接收

通过自己的文件传输工具上传 tar，并在云端准备同版本源码。以下校验仅使用 Python 标准库，不需要 ROS、相机、机械臂、模型权重或 GPU：

```bash
python3 src/zekeep_embodied/src/zekeep_embodied/bundle.py verify /数据路径/red_cube_v1.tar
# 仅在校验成功后执行，使用新建空目录。
mkdir -p data/cloud
mkdir data/cloud/red_cube_v1
tar -xf /数据路径/red_cube_v1.tar -C data/cloud/red_cube_v1
```

保留 `bundle.json`、原始图像、`episode.json` 和 `frames.jsonl`。训练/验证集合及标定、相机布局应随数据交付。
`verify` 只校验文件完整性，不认证发送者；只解压自己生成并成功校验的数据包。

云端训练所需模型环境、权重和数据转换按所选训练方案单独准备。本次采集流程不运行训练或模型部署。
普通 `dataset export` 输出 LeRobot v3；已有 π0.5 模型入口使用 v2.1，二者不能直接混用。
