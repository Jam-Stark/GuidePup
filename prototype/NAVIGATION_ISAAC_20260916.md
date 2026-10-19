# Isaac Hospital：一次多房间导航

2026-09-16 17:08 HKT。按 Owner 要求，将新实验切换到原生 Isaac Sim/Lab，使用官方 Hospital，GPU0、实验室 Qwen，无后拉、人工暂停、问答或人工恢复。

## 结果

**完整多房间任务真实成功 0/1，NEEDS_USER，运行错误 0。**

机器人已完整穿过第一道门，实际行走 6.427 m，最终停在走廊对侧一扇关闭的门前，没有进入指定的右侧开放房间。Monitor 未取得第一道门的完成证据，第一阶段始终 ACTIVE；三次恢复后求助。没有阶段误完成。

| 项目 | 新回合证据 |
|---|---|
| 任务墙钟时间 | 142.128 s；预算 360 s，决策 30 s，恢复最多 3 次 |
| 物理记录窗口 | 36.02–66.72 s，共 6141 个 5 ms 样本 |
| Qwen | 7 次实际 Pi 请求，无工具格式/HTTP 错误 |
| LightNav | 69 响应，65 接受、4 丢弃；3 次接受的 STOP；visible 始终 false |
| MPC | 240 次；236 次成功并生效，全部采用 LightNav 轨迹；4 次 blocked |
| 第一道门 | 44.775 s 完整穿越；穿越中三圆截面对开口最小横向净距 0.0761 m |
| 第二道门 | 未进入；完整任务未完成 |
| Monitor | 0 阶段完成，0 误完成，恢复上限后 NEEDS_USER |
| 干预 | 外力注入未提供；HUMAN_PAUSED 0，人工恢复 0 |
| 最终停止 | 最后 0.3 s 实际平移速度最大 0.00263 m/s、偏航速度最大 0.000765 rad/s |
| 估计误差 | 308 个时刻，一次首帧 SE3 对齐后，水平位置 RMSE 0.1352 m、最大 0.2097 m、末端 0.1905 m；偏航 RMSE 0.101° |

## 本轮暴露的问题

### 1. 实际通过第一道门，在线阶段没有推进

Qwen 将任务规划成：

1. `pass_doorway`：走出当前房间、穿过走廊；LightNav 指令为 `Exit the current room and walk through the corridor`。
2. `pass_doorway`：进入对面右侧开放房间。
3. `approach_target`：走向房间内部中央区域并停下。

第一阶段实际跨门后继续横穿走廊，但本轮没有 `measurement_requested`、门洞认证或 `stage_completed`。Monitor 一直缺少相向侧面、全机身自由通路和远端扩展证据。

当前 `task-monitor.ts` 的近目标取证触发依赖 `lightnav_hint.visible_claim` 和两米内的目标指向深度。本轮 LightNav 所有 `visible=false`，这一触发条件未成立。已有 portal 的跨越检测又需要此前先完成 portal 认证。**这条链路漏掉了持续行进时的过门证据。** 这是本轮运行记录与当前源代码共同支持的缺口，不是“机器人实际上没有出门”。

恢复时，Qwen正确看到眼前的关闭门，却继续把目标表述为寻找当前房间出口。它没有依据行进历史调整已经过门的任务状态。最后还把“进门后停下”拆为“靠近房间中央”的独立阶段；该阶段没有执行，其完成语义与当前基于目标表面的 approach 判据是否匹配仍待处理。

### 2. MPC 报告地图与机身重叠，不能等同实际撞门

首次 blocked 为 55.960 s：`Current/delayed A2 footprint violates hard clearance: -0.326 m < 0.018 m.`

保存的 12463 个二维占用点中，有 254 个落在当前三圆包络内。最近点距离后部圆心仅 2.77 mm。这说明控制器确实收到了与自身包络重叠的障碍输入；后续重新继续导航也无法解除这个约束。

本轮非足部净法向接触力所有记录均为 0；足部支撑力有真实记录。当前记录只有每个部件的合力，没有接触对象、接触点、摩擦力，且相反法向可能抵消，所以不能声称已证明零碰撞。二维占用日志也没有保留对应像素、体素高度及观测来源，暂时不能确定重叠来自地面分类、累积地图、位姿误差或具体几何。保留该缺口，没有修改净距条件或用真值清理地图。

### 3. 这次 STOP 发生在已经出门之后

首个接受的 STOP 为 seq62，曝光 58.700 s、距第一输入 20.600 个传感器秒；真实位置已在走廊对侧，早于它的第一次 MPC blocked 已发生。随后重复继续原语义指令，又得到两次 STOP。Qwen之后改变语义策略，仍被 MPC 的包络重叠检查挡住。

因此，本轮与此前 MuJoCo“尚未出门就 STOP”的现象不同。此次场景、物理引擎、原生隐式 PD、相机实际安装高度及指令也有差异，不能把行为变化单独归因于渲染器，更不能据此确认旧问题的原因。

## 实现与实验条件

- Isaac Sim 5.1 / Isaac Lab 0.54.4，使用本机现有 Isaac 环境；导航依赖留在既有 sim 环境，通过本地 Unix socket 连接原生 plant。
- 原生 PhysX A2 articulation、原始 URDF 和已有 policy；采用训练来源的隐式 PD、增益、关节限制与 armature。运行时没有基座位姿写入或传送。
- 原生 RTX 640×360 RGB 和 `distance_to_image_plane`，固定相机外参与实际内参；原始 IMU、关节编码器。首帧深度有效率 95.88%，初始静止高度约 0.5123 m，tracking 正常。
- 同一传感器估计连接几何、Monitor 和 MPC。根位姿、实际速度、接触记录只写到独立离线文件；场景坐标只用于放置初始状态和评估，不进入模型或在线控制。
- Hospital 617 文件、1,070,394,700 字节，USD/MDL/纹理/HDR 依赖完整，下载收尾 0 缺失、0 错误。下载和调用均使用服务器直连，清除大小写代理变量并禁止客户端读取代理；Codex 专用代理未改动。
- 保留官方两道开放门的原始门扇变换、碰撞体及地面；没有新增地板、renderer fallback 或驱动重置。YOLO 词表位于任务 scenario，未增加目标外观模板。
- GPU0 / RTX A6000，CUDA 按本机 UUID 绑定，RTX single-GPU 0。Qwen 使用实验室 endpoint，精确模型 ID `Qwen3.6-35B-A3B-FP8/`、thinking off、输出上限 2048；未启用 Gemini。
- 编译检查一次；先完成有界原生 plant/sensor-policy 接通，再检查正式场景图像并提交这一回合。此前平地接通不计导航成功。正式回合之后没有重跑求成功或扩批。

入口及复现方式见 [Isaac README](isaac/README.md)。官方资产说明见 [Isaac Sim 5.1 环境目录](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_environments.html)。

## 保存的证据

实验目录：`.local/outputs/isaac_hospital_qwen_20260916_01/`。

- `results.json`、`physical_evaluation.json`、`navigation_evidence.json`：两项独立评估和汇总。
- `preview.jpg`、`episode/initial.jpg`、`episode/final.jpg`、`episode/camera.mp4`：原始 RTX 画面和视频。
- `trajectory.png`：官方几何切片与离线实际轨迹；没有提供给导航。
- `episode/agent.jsonl`、`episode/source_snapshot/`：实际 Pi 请求/工具返回及执行源码。
- `failure_detail.json`：首次 blocked 的占用重叠数值。
- `physics/ground_truth.jsonl`、`physics/contacts.jsonl`：200 Hz 原生 PhysX 记录。
- `download_receipt.json`、`direct_network_receipt.json`、`route_candidate.json`：资产和任务布置来源。
- 原始在线传感器、LightNav 输入帧与 MPC 日志：`.local/outputs/isaac_sim_20260916_165411/`。

所有本任务服务已停止，端口释放、GPU0 为 1 MiB / 0%。未训练、操作硬件、扩展轮次或提交 Git。

后续优先事项是持续行进中的门洞取证与阶段切换，以及重叠占用点的像素/高度来源诊断；本次保持原框架判据并交付这一个回合的实际结果。
