> 历史记录：以下结果来自源电脑，便携包不携带其环境和数据，不代表新电脑验收结果。

# π0.5 准备工作验收记录

日期：2026-09-16。

## 已完成

- 固定官方 openpi 提交 `215abfb217dbac7d5f1273282331b9b1866c0479`，包含子模块。
- 独立 `.venv`：Python 3.11.15、JAX 0.5.3、PyTorch 2.7.1+cu128、
  torchvision 0.22.1+cu128、NumPy 1.26.4。
- `uv pip check`：240 个包依赖兼容。
- 基础权重 `pi05_base`：官方 29 个对象，12,441,749,581 字节，约 11.59 GiB。
  所有对象大小与 CRC32C 校验通过；提供 MD5 的对象另通过 MD5 校验。
  完整官方对象清单保存在权重目录的 `download_manifest.json`。
- 官方 PaliGemma tokenizer 下载完成，实际中文编码测试通过。
- Orbax 成功读取本地 checkpoint 元数据：51 个参数叶节点。
- RTX 5060（sm_120）上 JAX JIT bfloat16 矩阵乘法与 PyTorch CUDA bfloat16
  矩阵乘法均返回预期结果 32768。
- 原采集/遥操作测试：20 passed，新增 openpi 专用测试在该环境中按设计跳过。
- openpi 专用集成测试：2 passed，覆盖真实 v2.1 导出读取、动作 chunk、中文、
  颜色顺序、输入形状、关节增量与绝对动作往返、临时统计的归一化和反归一化。
- 命令行模拟：1 条 episode、40 帧；兼容导出和 openpi 数据变换检查通过。
- `stats` 和 `train` 命令均拒绝模拟数据，未产生正式训练统计或微调产物。
- Python 语法/未定义变量检查、Shell 语法检查、本次 `.gitignore` 差异空白检查通过。

## 解决的环境问题

上游锁文件默认安装 PyTorch CUDA 12.6 构建，不包含 RTX 5060 所需的 sm_120 内核，
实际 GPU 运算报 `no kernel image is available for execution on the device`。
依照官方安装矩阵换用同版本 CUDA 12.8 构建后复测通过。
该覆盖步骤已记录在 [接入说明](pi05.md)，再次 `uv sync` 后需重做。

NVIDIA GPU 探测和 Orbax checkpoint 元数据读取在 Codex 沙箱内受到限制；
在获得工具权限后于沙箱外复测通过。未修改显卡驱动。

工作区已有其他未提交更改及空白问题；本轮没有整理或覆盖这些无关更改。

## 验收边界

- 没有加载完整 π0.5 模型执行前向推理，也没有训练或测量模型吞吐。
- 本机约 8 GB 显存不足以满足上游列出的常规 LoRA 显存要求（>22.5 GB）。
- 正式示范、独立验证集、正式归一化统计及微调 checkpoint 尚不存在。
- 没有连接相机、主臂、从臂，没有使能电机或执行机器人动作。
- 此配置使用 JAX 训练路线。PyTorch GPU 测试只验证 CUDA 运算，
  未执行上游 PyTorch 模型专用 Transformers 补丁与权重转换。

后续操作与文件位置见 [π0.5 接入说明](pi05.md)。
