# Qwen 决策、只读观测与 Nav2 MPPI：实施及唯一完整回合

## 结论

架构修改和原生 Nav2 MPPI 替换已完成，随后只提交了一次完整导航任务。**真实成功 0/1；142.372 秒后 LIMIT_REACHED。** 实际移动 0.969 m，未完整离开初始房间，未进入目标房间，末态停稳。Qwen 没有声明完成，无误报成功。

语义 Monitor 本轮没有阻止阶段推进或强制取证。失败发生在保留的身体/停车余量检查：113 次原生 MPPI 求解成功并执行，之后 4 次控制尝试在原生求解前被拒绝，3 次恢复预算耗尽。替换没有解决这次停车后无法继续的问题。

## 实施

- Qwen 决定完整目标、子目标、策略、实际英文 instruction 和结束。`complete_task(reason,evidence)` 记录模型声明，独立物理 evaluator 判定实际成功。
- LightNav 连续局部导航；同目标文本修改保留会话和视觉历史。实际指令原文传输，reason 不替代 instruction。
- 语义观测提供已测、未测、冲突条件；移除距离触发暂停、自动阶段推进、全阶段证书完成门槛、重复策略自动否决和测量失败后的自动求助。
- STOP 和真实执行失败/无进展返回 Qwen。运行协调器保留通信、租约、人工权限、时间预算及全任务累计最多 3 次恢复。
- 局部控制使用官方 `nav2_mppi_controller::Optimizer` 1.3.13。CasADi/IPOPT 与旧五原语恢复路径从生产代码移除，原文件保存在证据根 `before/` 和历史回合快照。

原生 MPPI 使用 24×0.1 s 预测、512 个候选、一次迭代、零失败重试。路径/目标代价来自 Nav2；A2 critic 保留三圆包络、4 cm 体素膨胀、原硬净距、0.15 s 命令响应及原经验停车模型。候选命令按原速度权限有界；滤波后重新施加原生指令约束，并核验最终实际命令序列。没有备用控制器或强制前进。

原生路径代价使用有界 DiffDrive 加速度模型，身体/停车检查使用经验 A2 响应，二者预测模型有差异。碰撞检查仅覆盖观测到的深度占用；unknown 保留为 unknown，不能视为已证实自由空间。

## 环境与验证

官方 ROS/Jazzy 软件包走服务器直连，在 `.local/nav2_sysroot` 无 root 安装；未修改共享 Isaac 环境和 Codex 专用代理。234 个包的版本清单、安装路径修正、原生构建记录在证据根 `setup_*`。

TypeScript 构建、Python 集成源码解析和一次变更空白检查通过。`cpu_native/` 对一条旧回合真实传感器状态/参考路径/45 个占用点执行了原生库，求解 97.03 ms，单次优化。没有执行该离线替代轨迹，不能计为新导航成功。

## 正式条件及准备阶段事件

- 场景 `prototype/isaac/scenarios/hospital_room_to_open_room.json`；原位置 `[12.8,3.35,0.55,0°]`。
- 默认相机前方 0.30 m、上方 0.10 m、俯视 5°；完整原生深度和 RGB 标定保持。
- 独立 `footprint_10cm.json` 保留 10 cm 期望净距；机身尺寸/硬门槛不变，不加居中前缀，不扩展横移。
- Qwen 实验室 `Qwen3.6-35B-A3B-FP8/`，thinking off，输出 2048；无 Gemini。
- 360 秒任务上限、30 秒决策、全任务最多 3 次恢复；无后拉、人工暂停或人工恢复。
- 实际使用物理 GPU1，UUID `GPU-2c5d7dc8-847b-6d4a-e3cd-e1139ef56787`。CUDA UUID、Kit GPU 表及实际 renderer_gpu=1 已核对。GPU0 被其他项目占用，其进程未被结束。

准备阶段有两个事件，均在 Qwen/LightNav 启动和任务提交前：`navigation/` 因相对 footprint 路径失败，已改为启动器传绝对路径；`navigation_01/` 在发现 GPU0 被其他任务占用后主动取消本任务启动。两份记录保留，不能计作完整导航。正式任务仅 `navigation_02/` 一次。

## 实际决策与执行

Qwen 把完整路线设为一个阶段，首次实际下传：

> Go out of the current room, walk through the hallway, and enter the fully open room on the right opposite side, then stop once the entire body is inside.

共 6 次 Qwen 调用：一次规划、五次运动提议，四次下发；第五次运动提议因全任务恢复预算耗尽未下发。首次理由将位置称为走廊，这与原房间起点不符，是模型判断，不能当已出门证据。

| 动作 | 实际执行情况 |
|---|---|
| 完整目标的 LightNav 连续导航 | 439/495 个控制 tick 非零，113 条 MPPI 指令全部采用 LightNav 参考 |
| 受阻后同阶段新 instruction | LightNav seq31 接受；42 个 tick 全零 |
| Qwen 请求左转 30° | 29 个 tick 全零，未真正执行转向 |
| Qwen 请求右转 30° | 32 个 tick 全零，未真正执行转向 |
| 再请求右转 20° | 第四次恢复被预算拒绝，没有下发 |

四个实际动作按 control_task_id 关联，Qwen 提议、动作 instruction、semanticInstruction、控制状态和 LightNav 请求原文一致。35 个模型返回中 31 接受、4 因执行暂停而丢弃；一个会话，仅 seq0 重置。visible 始终 false。

每次受阻决策都获得当前/动作起点 RGB、估计运动、实际控制摘要及控制器原因。只读语义证据虽未证实，也没有禁止继续或换视角；没有测量请求或语义几何暂停。

**没有 LightNav STOP。** 因此本轮无法评价首次 STOP 的位置或 STOP 后语义纠正效果；`complete_task`、不同阶段切换和测量审核出口也未获实跑覆盖，不能据此宣称全部分支已经验证。

## 停车约束拒绝

117 次 Python 控制尝试包含 113 次实际原生优化和 4 次前置拒绝。`navigation_evidence.local_controller.solves` 统计的是这个调用次数；原生请求/响应数量见 `analysis/controller_flow.json`。

| 传感器时间 s | 当前/延迟净距 cm | 停车包络净距 cm | 硬门槛 cm | 请求 |
|---:|---:|---:|---:|---|
| 25.20 | 8.23 | 1.19 | 1.50 | LightNav |
| 26.64 | 5.57 | −1.47 | 1.50 | LightNav |
| 27.80 | 6.27 | −0.76 | 1.50 | 左转 |
| 29.06 | 6.03 | −1.00 | 1.50 | 右转 |

这里是估计地图和经验模型的数值，不是真实物理净距。后三次已接近静止仍被保留的停车包络拒绝，不能说转向已执行。没有原生优化不收敛、替换命令或清空障碍。

113 次实际求解中位 101.1 ms、最大 227.9 ms；报告的最小预测净距 9.826 cm、最小停车余量 1.356 cm。首次前置拒绝的独立实际位姿为 `[13.5662,3.0849,0.06824 rad]`；前缘已越过源门近面约 14.90 cm，后缘仍差 1.215 m 清空源门，整机未通过。

这些记录定位了阻断环节，但未证明最近占用对应哪一个真实实体，也不能仅凭整体定位 RMSE 判断局部地图正确。后续应定位该占用和停车包络在低速状态的作用，不直接放宽门槛。本任务不继续改控制器或再跑。

## 独立物理结果与限制

实际路径 0.969239 m，最终位姿 `[13.56988,3.08423,0.03040 rad]`；源门未整机通过，目标房间未进入。末尾 0.3 s 窗口实测最大线速度 0.00186 m/s、角速度 0.00378 rad/s，停稳成立。

首次配对帧单次 SE3 对齐后，位置 RMSE 1.412 cm、最大 3.204 cm；仅用于离线评分，未回灌控制。

3397 个原生 PhysX 接触采样的非足部净法向力最大值为 0；脚部有正常力测量。这种 observer 不提供对方 collider、接触点或摩擦，可能有力抵消，因此不能据此证明全局无碰撞。门口截面包络净距最小约 18.94 cm，也不是全场景最近障碍净距。

正式回合 recorder_error=null，运行错误事件为空；终态 **LIMIT_REACHED**，不是 NEEDS_USER 或 ERROR。进程退出 2 明确表示受控任务未完成，supervisor 的 PROCESS_FAILED 不能被解释成模型/原生库崩溃。Qwen 无完成声明，物理成功为 false，无误完成。

## 证据与收尾

证据根：`.local/outputs/mppi_integration_20260917_01/`。

- 正式：`navigation_02/results.json`、`physical_evaluation.json`、`navigation_evidence.json`、`episode/camera.mp4`、Pi 全请求/响应、source_snapshot。
- 原始：`.local/outputs/isaac_sim_20260917_161247/`，RGB-D、LightNav 完整输入/输出和时间/会话、`local_controller.jsonl`、原生 MPPI 全输入/输出、独立物理轨迹及接触。
- 分析：`analysis/QWEN_FLOW.md`、`qwen_flow.json`、`controller_flow.json`；`trajectory.png` 仅绘制离线真实轨迹、评分房间边界和机身包络，不是障碍真值地图。
- 初始预览、两次未提交任务的启动记录、setup/native CPU 证据、`final_resources.json`、`completion.json` 全部保留。

本任务 physics/navigation/LightNav/recorder/原生桥均结束，相关端口和 GPU1 释放，独立 tmux 结束。未启动第二次任务、未训练或操作硬件。Memory 已更新。
