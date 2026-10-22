# 更新记录

## 2026-10-22

### 使用入口与文档

- 总 README 在项目组成前新增场景化快速路线。
- 新增 `tools/env.sh`，统一准备、ROS、视觉、遥操作、VLA 采集、模型和 Studio 环境入口。
- 环境包装在子进程中执行，不改变调用终端；未安装环境会报出路径和安装提示。
- README 的环境切换示例改用包装脚本，安装器与 VLA 保持原有边界。
- 安装修复与验证记录从总 README 移至本文件。
- VLA 的纯 ROS 子命令复用干净的 `ros` 包装；采集和模型执行器清除外部 Python/原生库路径。
- GitHub 发布排除独立 `apps/` 目录、本地环境、下载资源和设备身份记录。

### 基础安装修复与核查

2026-10-22 基础安装修复：ROS 初始化时临时关闭 `nounset`，避免初始化脚本因未设置变量而退出；
ROS/Conda 阶段隔离 `PYTHONPATH`、`PYTHONHOME` 和原生库路径；视觉首次环境安装和后续 pip 安装统一版本约束；
明确安装 OpenCV/Open3D 所需原生库，选择 cu128 轮子，安装 Orbbec udev 规则。
安装末尾新增 SDK/视觉/遥操作导入及 GraspNet 后端检查，失败保留日志。VLA、Studio 仍为独立选装。
Studio 的环境、Conda shell 初始化、Node/npm/Blockly 和恢复方法也已写入本 README。
ROS/SDK、视觉与 VLA 的 Python 3.10 依赖解析通过（仅解析包元数据，未执行安装）。
安装器替身测试覆盖无 ROS/Miniforge/venv 的首次安装、重复安装、CUDA 可用/缺失和两种导入失败。
旧安装器会在新增的环境污染回归检查中失败，修复版通过。
本机 SDK、视觉 FK/IK 与三项视觉权重校验通过；现有 Studio 环境缺少 `motorbridge`，
`--self-test` 未通过，恢复步骤已列在总 README，未替用户改动现有环境。
本次没有在另一台干净电脑重新执行全部安装、完整权重下载、转换、训练或硬件验证。
