# Qwen 工具纠正与里程计修复

2026-09-16 14:29 HKT。针对[上一轮 p05 失败](NAVIGATION_QWEN_SENSOR_TEST_20260916.md)，已修复无效工具历史导致的二次 HTTP400，并替换弱纹理下漂移严重的里程计。完成了实际 Pi 连接复现、同记录回放及 GPU0 新传感器序列验证；本次没有新增完整导航回合。LightNav 分工研究见 [自主权讨论](LIGHTNAV_AUTONOMY_BOUNDARY.md)。

## Qwen：已确认并修复纠正请求的错误

原记录 seq165 的 `accept_measurements.arguments` 是字符串 `"selections"`；seq166 被本地 schema 拒绝，未执行。旧纠正路径把这条 assistant tool call 原样放回历史。Pi 0.85.1 将其序列化为 JSON 字符串，而非参数对象。用实际实验室入口复放这一历史，稳定得到原来的 HTTP400：`Can only get item pairs from a mapping.`，推理 token 为零。

[Qwen 官方模板](https://huggingface.co/Qwen/Qwen3.6-35B-A3B/raw/main/chat_template.jinja)对工具参数使用 `items` 遍历，与该错误相符。服务端实际部署版本未公开，不能断言已经检查到服务内部的同一行代码；错误历史本身的因果关系已由配对请求验证。

`pi/src/decision.ts` 现在把被拒绝的非对象参数及实际校验错误作为文字反馈，请模型重新提出工具调用；不再把非法参数放回 assistant tool-call 历史。对象形状正确但内容违反 schema 的错误仍使用原工具错误反馈。保留原来最多两次尝试、30 秒决策约束、2048 输出上限；不替换参数、不执行错误提议。

三次实际 Pi/实验室调用的用途与结果：

| 调用 | 结果 |
| --- | --- |
| 原非法历史复放 | 复现相同 HTTP400，零推理 token |
| 同一错误改为文字反馈 | HTTP200，返回 schema 合法的新提议 |
| 生产 PiDecision 的第一次结果回放原错误、第二次真实请求 | HTTP200，纠正成功；仍未执行工具 |

另用确定性错误回放确认第二次仍无效时停止，没有增加网络请求或无限重试。入口保持实验室 `Qwen3.6-35B-A3B-FP8/`、thinking off、服务器直连。证据：[请求、响应与生产集成记录](../.local/outputs/qwen_tool_fix_20260916/README.md)。

**未解决的部分：**原回合没有保留原始入站 SSE，Pi 最终结果已丢弃 partialArgs，因而还不能区分首次字符串参数由模型生成、服务解析还是 Pi 流式解析产生。此次修复解决的是纠正链路再次 HTTP400，不保证模型永远生成合法参数；简短探针也不证明测量 ID 或语义正确。

## 里程计：用深度约束实际可观测方向

旧 Open3D RGB-D hybrid 方法在近乎空白的墙面上把图像变化解释成平移。原有 `information_rank=6` 不能证明可靠：Open3D 0.19 的 `CreateInformationMatrix` 从单位矩阵开始累加几何对应项，并不是优化残差的可观测性 Hessian。[对应官方源码](https://github.com/isl-org/Open3D/blob/v0.19.0/cpp/open3d/pipelines/odometry/Odometry.cpp)。

局部诊断还发现，水平边缘的栅格阶梯可产生通过光流一致性检查的伪角点；问题帧对的特征真实重投影误差中位数为 10.67 像素。尝试过的特征 PnP 与固定旋转特征平移方案均失败，未进入生产。

当前 `robot/odometry.py` 使用单一的 `depth_imu_proprioception` 估计方法：

1. 每个 5ms IMU 样本积分旋转；旧 A2 策略已有的速度预测提供平移先验。该输出来自 IMU、关节和策略历史，不是运动命令积分。
2. 按原 checkpoint 的训练配置 `observations.pred.base_lin_vel.scale=2.0`，将 `aux[:3] / 2` 解码为 m/s。模型权重和后拉力估计不变。
3. 深度点云做固定陀螺旋转的点到平面配准，只在表面法向实际约束的方向修正平移。近空白墙面约束不了的切向运动保留本体速度预测，并在 quality 中公开这些方向。
4. 重叠不足直接失效并锁存 lost。没有真值回填、指令积分、旧估计器 fallback 或假协方差。

几何、MPC、Monitor 继续共用同一 `RobotState`。速度质量字段改为 `velocity_estimated`，明确它是融合估计，而非所有方向均被视觉直接观测；首帧和失效状态不能作为停稳证据。停止门槛和通路/候梯条件未放宽。详见 [状态合同](ROBOT_STATE.md)。

仿真 adapter 保存完整 200Hz 原始 IMU/关节包、解码后的速度预测及初始化记录；估计器不读 MuJoCo 基座位姿或真实速度。真值只在估计结束后离线评分，使用首帧一次刚体对齐，不逐帧对齐。

## 配对结果

### 同一份已归档的 63 帧

原失败回合只归档了 63 张不同 RGB-D 图像，并非在线估计器处理过的全部 178 帧。新旧方法使用同一份可得的 RGB-D/IMU 序列；新方法另使用当时已经保存的本体速度预测。

| 指标 | 旧方法 | 当前方法 |
| --- | ---: | ---: |
| 三维位置 RMSE | 8.15cm | 4.02cm |
| 最大位置误差 | 11.27cm | 6.97cm |
| 平面速度 RMSE | 5.45cm/s | 3.10cm/s |
| 计算时间中位数 | 370.94ms | 14.65ms |

[完整回放数据](../.local/outputs/odometry_fix_20260916/production_replay.json)。原 178 帧在线 RMSE 25.06cm 属于另一采样序列，不能拿来直接与当前 63 帧的 4.02cm 组成配对改善。

### 新的 GPU0 传感器序列

同一 A2 上运行静止、左转、静止、右转、前进、静止，共 19 秒、191 帧 RGB-D 和 3801 个 IMU 样本。当前方法在线估计，旧方法回放完全相同的原始传感器序列。原生 GPU0/EGL2，无 Qwen、LightNav 或完整导航参与。

| 指标 | 旧方法回放 | 当前方法在线 |
| --- | ---: | ---: |
| 三维位置 RMSE | 26.32cm | 4.77cm |
| 最大位置误差 | 41.04cm | 8.93cm |
| 末端位置误差 | 13.21cm | 6.49cm |
| 平面速度 RMSE | 10.87cm/s | 2.92cm/s |
| 静止阶段估计速度中位数 | 4.54cm/s | 1.28cm/s |

[新鲜序列及对照结果](../.local/outputs/odometry_fix_20260916/fresh_sensor_01/comparison.json)，同目录保存原始帧、传感器、初始化、估计输出和离线真值。包含旧方法回放的总墙钟为 88.30 秒。证据脚本、失败候选和旧方法快照都在 [证据目录](../.local/outputs/odometry_fix_20260916/)。

## 验证范围与结论

Python 相关模块编译检查及 Pi TypeScript 构建通过，实际生产估计器完成上述 191 帧。任务 tmux 已退出，GPU0 已释放；既有未提交工作保留，未 commit/push。

这些结果支持“已修复纠正请求的二次错误，短时里程计误差明显降低”。它们不证明完整通门、候梯、Monitor 自动完成、后拉暂停恢复或实机精度。深度弱约束方向仍依赖学习速度，存在累积漂移；当前没有闭环重定位，陀螺偏置、打滑、长期运行和真实 D435i 噪声尚待验证。完整导航记录的原 NEEDS_USER 和原成功分母保持不变。
