# 从原始房间到当前全开门房间：一次新导航

## 任务和条件

Owner在上一STOP反馈回合结束后，明确要求把robot放回原来的初始房间，走到当前右侧门完全打开的房间。本记录属于新授权的一轮完整任务；旧回合与离线预测不计入本轮成功。

- 新scenario：`prototype/isaac/scenarios/hospital_room_to_open_room.json`。
- 原始起点：从`hospital_rooms.json`恢复`[12.8, 3.35, 0.55, yaw=0°]`，位于原房间内。
- 当前终点：与`hospital_open_room.json`的`room_b_entry`及目标房间几何完全相同。离线依次核验`room_a_exit`、`room_b_entry`，再核验整机位于目标房间内并停稳。
- 实际提交中文任务：**走出当前房间，穿过走廊，进入对面右侧那扇门完全打开的房间，直到整个机身进入房间后停下。**
- 保留当前STOP证据分级反馈和同目标会话历史策略；本轮没有修改生产导航代码或完成条件。
- 保留完整原生深度、默认相机前0.30米/高0.10米/俯视5°、独立10厘米期望净距footprint、原身体/硬门槛/停车模型，无居中前缀。
- Qwen实验室`Qwen3.6-35B-A3B-FP8/`、thinking off、2048、无需鉴权；直连。GPU0启动前无计算进程；其他项目进程、Codex专用代理未动。
- 360秒任务、30秒决策、最多3次恢复；无后拉、人工暂停或恢复。独立tmux `guidepup-room-to-open-20260917`，一次新任务，无自动重跑。

证据根：`.local/outputs/room_to_open_room_20260917_01/`。登记范围和启动命令见`registration.json`与`run_receipt.json`。先渲染检查实际初始RGB-D画面，再提交任务；坐标、门框几何和独立真值只供初始化和离线评分，不输入模型/在线控制。

## 评估范围

既有evaluator适用于这里两道门的+X方向穿越；独立物理评分按三圆包络依次经过两门并停在目标房间内。它不是完整场景连续碰撞证明。Monitor阶段完成事件按pass_doorway完成顺序映射到两门，因此结果分析还需要核对实际Qwen阶段计划，不能在阶段语义不符时盲信该映射。

### 本轮实际Qwen计划

| 阶段 | 类型 | 目标与实际LightNav instruction |
|---|---|---|
| 0 | pass_doorway | 当前房门；`Move through the door out of the current room.` |
| 1 | pass_doorway | 走廊尽头的门；`Move across the hallway.` |
| 2 | approach_target | 对面右侧完全打开的门；`Enter the completely open room on the right opposite and stop when the whole body is inside.` |

这是Qwen实际输出，运行中未人为改写。阶段1的走廊意图没有明确绑定目标房间开口，阶段2又使用approach_target，因此不能把第二个pass_doorway完成事件自动等同目标房间通门完成。实际记录若出现这类事件，需要另列其语义映射限制；物理两门顺序评分仍独立有效。

## 结果

**真实完整任务成功0/1，NEEDS_USER：已完整走出原房间，未进入目标房间。** 实际行走5.93837米，最终在走廊对面关闭门前停下。Monitor阶段0仍ACTIVE，阶段1、2未开始；没有误报阶段完成。两项evaluator和results已完成。

| 项目 | 新回合实际证据 |
|---|---|
| 任务墙钟 | 211.318秒；含启动/预览/收尾的runner403.119秒 |
| 独立物理窗口 | 14.000–39.500仿真秒 |
| 原房间出口 | 23.875秒整机完整通过，即任务物理窗口开始后9.875秒；最小门框截面横向净距0.11635米 |
| 目标房间入口 | 未通过，末态机身前缘沿门法向仍差1.56327米 |
| 实际末态 | 世界x=18.602993米、y=3.687188米、yaw=0.370507弧度；目标房间内界净距−2.92665米 |
| 实际停稳 | true；末0.3秒最大平移速度0.00609米/秒、yaw角速度0.00465弧度/秒 |
| Monitor | NEEDS_USER，0阶段完成、0误完成；恢复计数0 |
| Qwen | 5次实际请求，无工具格式/HTTP/运行错误 |
| LightNav | 70次响应全部接受，18次visible=true，唯一STOP在seq69 |
| MPC | 207/207成功且实际下发，全部采用LightNav轨迹，0次blocked |
| 干预 | 无外力，HUMAN_PAUSED采样0，无人工恢复 |

![实际两房间路线与独立速度](../.local/outputs/room_to_open_room_20260917_01/trajectory.png)

### 已出门，为什么仍停在阶段0

本轮首次取证发生在STOP之后。在此之前，Monitor始终没有认证第一门的portal。LightNav虽然有18次`visible=true`，但是其opos像素结合深度得到的提示点，距离当前估计机身6.536–8.256米，均超过现有`target_evidence_range_m=2.0`。因此`task-monitor.ts`的靠近目标取证条件没有触发。提示点是模型指向与对应深度表面，不是已认证的出口身份；不能仅凭可见声明把远处深度点当成本房门框。

证据是`hint_range_evidence.json`中的完整在线提示、RobotState及距离采样，计算未使用真值。本轮一次首帧对齐后的独立位置RMSE为1.554厘米、最大3.991厘米；这是估计误差评分，不会自动补足缺失的门洞几何证据，也不证明通用或实机定位可靠性。

23.875秒物理上已经出门，但在线没有portal或完成事件，后续LightNav指令始终保持：

```text
Move through the door out of the current room.
```

目标房间阶段的instruction没有实际下发。本轮因此暴露的是已通过出口后的证据与阶段衔接问题，不能描述为A2无法出门，或MPC拒绝执行进门轨迹。

### STOP与后续审核

seq69曝光36.520秒，收到响应对应状态36.700秒；它发生在真实完整出门后约12.8个仿真秒。`visible=false`，之后控制进入ARRIVAL_CANDIDATE并清零命令；日志里保留的MPC出处是上一seq68，不能计为STOP后的新导航。

该次实际模型图像可见对面的关闭门。Qwen选择S2/S1的墙体/门板区域，随后三次审核都包含`not_target`，第三次将两者都判为not_target并给出`sameOpening=false`。Monitor依原规则拒绝目标身份，三次同一观察未取得可核验证据后停止：

> 同一观察的目标选区三次未取得可核验证据。需要帮助：帮助确认目标位置或提供更清晰视角。

Qwen没有调用continue_navigation或change_view；`stop_continuation_feedback`事件0。本轮没有纠正后的实际模型输入、恢复轨迹或重复STOP，因此上轮接入的语义反馈仍未在真实闭环触发。没有忽略STOP、强行推进或新增回合。

70次输入保持同一session `3f1d43d9-bdd9-4686-a48e-b7d71d7714af`，仅输入0重置；seq69会话内序号69、实际前帧间隔0.300秒。原4fps时间前缀未改变，实际传感器时间和请求墙钟完整保存。

### 阶段规划和评分限制

前述“走廊=pass_doorway、最终房间=approach_target”的阶段类型不匹配是本轮实际计划中的另一个问题，但阶段1、2没有运行，不能把本次停止归因于它们的执行效果。由于本轮没有任何`stage_completed`事件，evaluator也没有生成实际的错误阶段映射结果；物理两门顺序评分和Monitor停在阶段0均有独立原始证据。后续如果推进到这些阶段，需要先处理语义映射，而非盲信位置顺序。

### 净距、接触和错误

MPC最小记录`minimum_clearance_m=0.08607`米，最小`stop_reserve_m=0.000567`米，全部求解成功。10厘米是期望偏好，不是硬门槛；这些值是估计地图上的优化/停车余量，不能当作真实场景全局最近净距。第一出口的0.11635米是独立物理姿态与门框截面的评分，范围不同。

非足部最大净法向力0牛，超过1牛采样0。observer不提供接触对象、点或摩擦，可能有自接触/合力抵消，不能据此保证无接触。门框截面与三圆包络评分也不是全关节、全场景连续碰撞证明。

recorder_error为空，runtime_error_events和control_error_reasons均为空。recording退出码2对应NEEDS_USER；navigation退出码−15为runner正常清理。没有fallback、隐藏运行错误或驱动操作。

## 新证据与收尾

- `.local/outputs/room_to_open_room_20260917_01/navigation/results.json`、`physical_evaluation.json`、`navigation_evidence.json`：两项独立评估与汇总。
- `navigation/episode/camera.mp4`、initial/final图、`agent_observations/`和`agent.jsonl`：视频、RGB/裁图、Qwen请求/返回、Monitor记录。
- `.local/outputs/isaac_sim_20260917_012131/`：70对LightNav实际JPEG/NPZ、模型输入输出、MPC/telemetry/sensors及独立ground_truth/contacts。
- `analysis/evidence.json`、`analysis/REPORT.md`、`analysis/reduce_room_to_open_room.py`：按实际STOP和两道门归约；`hint_range_evidence.json`：在线取证距离证据。
- `trajectory.png`和`plot_route.py`：本次独立实际路线图；房间阴影仅表示评分内界，不表示无障碍地图。
- `preview_review.json`、registration、run_receipt与source_snapshot保留实际起点、参数和执行来源。
- `final_resources.json`确认本任务四个服务PID和独立tmux全部退出，GPU0无计算进程（4MiB/0%）；其他项目与Codex代理未操作。

本任务仅新建场景配置及记录文档，没有修改生产导航代码、默认配置、完成门槛或相机。新证据/memory已更新；没有第二次尝试、扩批、训练、硬件操作或Git提交/推送。
