# 取消全任务时间预算后的单次导航

**唯一新回合已结束，真实成功0/1；Qwen声明完成属于误报。** 任务实际持续575.443秒，已越过旧360秒上限。机器人走9.420米、完整出源门并停稳，但仍在目标门外。没有控制器受阻或运行错误。

## 范围与实现

Owner授权取消全任务时间上限，并在同一Hospital原房间→对面右侧全开门房间任务上重新跑一次。

- 删除Pi `MonitorOptions.timeoutS`、任务定时器及开始/恢复时的deadline注册；CLI移除`--timeout-s`。
- 删除记录器的整回合墙钟超时分支；同步移除调用入口的参数转发。结果和manifest的`timeout_s=null`表示无全任务时间预算。
- 保留单次决策30秒、全任务最多3次恢复、局部无进展/搜索事件、模型完成/求助/执行错误/人工取消终止。
- MPPI的2.4秒局部停车余量恢复期限保持，它约束候选运动可行性，与全任务时间上限无关。
- 传感器、标定、相机、身体/体素、速度权限、停车模型、MPPI配置、场景与初始位置保持；使用独立期望净距10cm配置，不加居中指令、外力或人工暂停恢复。
- 当前Python入口语法检查及TypeScript编译通过；没有添加测试套件、兼容参数或备用路径。

证据根`.local/outputs/mppi_no_task_budget_20260918_01/`；基线为`mppi_stop_planning_20260917_01/navigation`，原始新记录`isaac_sim_20260918_113358`。使用物理GPU0，CUDA UUID与Kit Vulkan GPU0实际对应。独立tmux的1800秒ETA仅控制等待复查时刻，不终止导航。

## 越过旧时限的实际证据

任务`goal_changed`发生在2026-09-18 03:35:24.475 UTC（11:35:24.475 HKT）。到03:41:30.037 UTC，任务已运行365.562秒，控制状态仍为`NAVIGATING`，没有`task_timeout`或任务终止事件。见`budget_milestone.json`。该证据说明本轮确实越过旧360秒上限，不代表已经进房间。

## 完整实验结果

### 配置与时间

独立输出为`navigation/`；Qwen实验室模型`Qwen3.6-35B-A3B-FP8/`、thinking off、输出2048保持；未使用Gemini，网络入口沿用服务器直连，Codex代理未动。只有这一轮导航，没有额外模型探测、离线调参或第二轮任务。

- Task.startedAt 03:35:22.758 UTC → task_completed 03:44:58.201 UTC：**575.443秒**。
- 从goal_changed事件到完成：573.726秒；两者相差约1.72秒的开始动作失效化处理。
- 记录器总时长584.714秒，包含启动及终止后取证；不要混为任务时间。
- 全程`task_timeout`事件0、ERROR状态0；`timeout_s=null`已写入记录器summary，实际命令不带旧参数。

### 物理结果

| 项目 | 新回合结果 |
|---|---|
| 真实累计路径 | 9.42013m |
| 整机出源门 | 是，33.095仿真秒；门洞截面最小横向余量17.489cm |
| 目标门完整穿越／整机入房 | 均否 |
| 最终世界XY/yaw，仅离线 | [19.88642, −0.64543]m，0.00857rad |
| 三圆前缘距目标门框近侧 | 仍差25.950cm，尚未开始穿越 |
| 三圆后缘距门框远侧 | 仍差1.62495m |
| 最终停稳 | 是；末0.3秒最大平移0.000988m/s、yaw0.001024rad/s |
| Qwen声明／独立物理成功 | COMPLETED／false，false_completion=true |
| 请求用户帮助／恢复耗尽 | 均没有 |
| 运行错误 | 没有 |

物理记录11953样本，窗口10.96–70.72仿真秒。实际轨迹向右来到目标门前，画面能看见房间内部；这不代表机身已经进入。见`trajectory.png`、`analysis/stop_geometry.json`、`navigation/physical_evaluation.json`、`navigation/episode/final.jpg`和视频。

### 实际STOP→Qwen完成链路

仅3次Qwen调用：plan_task → select_stage → complete_task；只下发1个连续LightNav导航动作，恢复次数0/3。Qwen实际给LightNav的指令为：

> Go out of the current room, walk through the corridor, and enter the fully open room on the right side across the corridor. Stop only after the entire robot has fully entered the room.

184次LightNav输入/返回全部接受，只有seq183输出STOP；一个会话，仅首帧重置。STOP输入曝光68.00仿真秒时，三圆前缘尚差31.892cm到门框；响应到达68.30秒时尚差27.258cm。最后停稳仍差25.950cm。因此本轮不是“已进房后停下”。

Qwen收到的只读参考仍是`unverified`，目标身份`not_reviewed`、通门`not_observed`、`stableStopVerified=false`。它仍用房内家具画面、约8米累计位移、LightNav STOP及约0.16米预测停车净距作为已经进房的理由，调用complete_task。累计位移、可见室内及停车余量都不能证明整机穿越门框；0.16m也不是房间边界距离。模型原文与结构化在线反馈详见`analysis/QWEN_FLOW.md`/`qwen_flow.json`。

最终独立物理停稳为真，并不改变完成声明当时在线证据不足，也不补成通门成功。只读参考没有新增语义否决；框架按Qwen声明结束，离线评估明确标记误报。本次没有顺带修改完成决策策略或追加纠正实验。

### MPPI与实际命令

415次原生MPPI成功，414个结果实际下发，全部采用LightNav参考。另1次在STOP导致的控制代际切换期间未下发。414条命令与接受检查的最终序列首命令全部一致。

共212480个候选，197966通过包络评价，14514因正常模式的停车包络约束被拒；所有求解均有合格最终结果。没有Python前置拒绝、原生候选无解、最终结果拒绝或接口/优化错误；全415次为normal，restoration未触发。

已下发预测轨迹最小几何净距10.127cm，扣原固定误差后的停车净距3.120cm；这些是估计地图/经验动力学预测，不能当成全场景真实网格距离。求解中位667ms、最大1739ms；本轮没有重调性能或控制参数。

### 边界与收尾

- 非足部净法向力峰值0N，足部峰值744.24N。observer只记录各刚体净法向力，缺对端对象、接触点和摩擦，包含自接触且可能相消；不能据此宣称严格无碰撞。
- 离线里程计位置RMSE3.967cm、末端3.978cm、偏航RMSE0.114°；真值没有进入在线模型、地图或控制。
- 虽只修改时间预算配置/执行路径，两轮Qwen实际指令及模型轨迹不同：前轮初始先要求出源房间，本轮直接下达完整进房任务。不是冻结输入的配对对照，不能把全部路线变化归因取消时间上限。
- 本轮直接证明任务持续超过360秒；没有验证停车余量恢复、受阻后恢复、决策超时、三次恢复耗尽或实机分支。
- 两项evaluator和`navigation/results.json`已完成；视频、RGB-D、模型完整输入输出、原生候选/最终命令、独立物理/接触和源码快照保留。所有本任务服务、原生bridge、8050/8766/8770端口及独立tmux均关闭，GPU0释放，见`final_resources.json`。没有训练、硬件操作、Git提交或追加任务。

