# 第三方来源与安装边界

本文件记录依赖与来源，不替代上游完整许可，也不为未标注许可的文件补授许可。

- ROS 2、MoveIt、Pinocchio、MuJoCo、LeRobot、PyTorch 等按各自上游许可使用。
- Ultralytics YOLOE 与 MobileCLIP 模型由用户按需从官方发布下载；代码及模型许可分别以原发布者为准。
- GraspNet 源码、checkpoint 不随包分发。安装器保留许可确认，下载前须阅读 [GraspNet License](https://github.com/graspnet/graspnet-baseline/blob/main/LICENSE)；不接受时跳过。
- `web_mujoco` 记录的网页实现来源为 [Rebot_Arm_AGV](https://github.com/Yang-Ci/Rebot_Arm_AGV/tree/445b48d5fd9fef8050c8a9ef5fd6ab32e3ace5cf/web_mujoco)，保留目录 README 的来源信息。
- π0.5 与 tokenizer 可能需要登录和接受模型许可；凭据由部署者自行配置，不随源码保存。

本仓库不包含模型权重、训练数据或用户凭据；部署时由使用者自行准备。
