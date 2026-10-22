# WHEELTEC C100 全局相机接入

接入日期：2026-09-14。C100 为全局 RGB 视角，复用采集器 OpenCV/V4L2 后端，
不需要安装厂商 Python 例程或刷新固件。腕部 Orbbec 配置保留。

## 设备与资料

- USB VID:PID：`0c45:636b`。
- 系统名称：`HJ USB 2.0 Camera`。
- 当前视频采集节点：`/dev/video10`，此编号可能变化；`/dev/video11` 不作为图像输入。
- 配置绑定：`/dev/v4l/by-id/usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0`。
- 同时连接多个相同且序列号相同的设备时，应改用各自唯一的 `/dev/v4l/by-path` 路径。

厂商资料位于：
厂商提供的 WHEELTEC C100 C70 产品资料目录（需另行获取，不随包分发）。

本次依据 `1.产品技术规格书/WHEELTEC C100产品技术规格书_20240826.pdf` 和
`5.Python例程/camera.py`：C100 支持 USB 2.0、免驱视频输入、YUY/MJPEG、最高 1920×1080。
规格书没有给出所有分辨率下的实测帧率，不能把宣传参数当作当前采集性能。

未应用资料中的示例内参或畸变参数；如需去畸变，应按这台实际相机及选定分辨率重新标定，
并保持训练与推理的图像处理一致。

## 默认采集配置

`configs/record.yaml` 已启用 `cameras.global`：1280×720，MJPEG，请求 30 FPS。
通过 MJPEG 避免使用大带宽的未压缩 USB 2.0 视频传输。
采集器读取并校验设备协商的尺寸、像素格式与 FPS，保存协商结果到 episode 元数据。
设备报告值和实测值分别记录，不能用协商 FPS 代替测量。

本机在 640×480 和 1280×720 模式下都实测约 15 FPS，即使驱动报告 30 FPS。
因此默认采集控制频率调整为 10 Hz。当前没有修改自动曝光、白平衡或固件，
也未把该现象归因为某个未经验证的原因。

`installation_id` 为 `wrist_c100_global_trial_v1`；这是试采布局。
正式安装支架后应修改为新的安装版本，避免与旧单相机或不同视角数据混用。

## 双相机预检

```bash
conda activate embodied
cd "$ZKEEP_WS/src/zekeep_embodied"
bash tools/run.sh -m zekeep_embodied.record \
  --config configs/record.yaml --camera-check --seconds 10
```

命令不连接电机，保存 `wrist.png`、`global.png` 与 `camera_check.json`。
报告包括两路实测 FPS、采样数、重复帧次数以及两路主机接收时间最大偏差。
出现重复采样或样本不足时预检明确失败，应调整采样频率或设备设置后重测。

本方案是软件对齐：取两路最新帧，检查主机时间戳跨度不超过 `max_sync_s=0.10` 秒。
未实现曝光级硬件同步，USB 缓冲与曝光延迟也不由主机接收时间直接表示。

正式录制时两路图像自动写入同一 episode，导出为
`observation.images.wrist` 与 `observation.images.global`，无需另建 C100 专用采集程序。

### 本次验证结果

- 19 项离线测试通过，包括 C100 MJPEG 协商与模式不匹配时的资源释放。
- 10 秒双相机预检：100 组样本，重复采样 0 次。
- 腕部实测 30.01 FPS，全局实测 15.00 FPS。
- 最大主机接收时间差 53.87 ms，小于配置的 100 ms 容差。
- 检查图片与完整报告：`$ZKEEP_WS/data/embodied/camera_check_3aa8632c/`。
- 只验证相机软件对齐与数据链路；尚未在运动中的机械臂上录制真实示范。

## 安装提醒

首次取到的 C100 画面朝向显示器和天花板，尚未覆盖桌面抓取与放置区。
请固定支架、调整全局视角后重新预检，再进行批量示范采集。
本轮只进行相机读取、配置和离线测试，没有使能机械臂。
