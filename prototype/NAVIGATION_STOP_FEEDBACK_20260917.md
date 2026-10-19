# STOP 后的语义纠正与一次完整导航

## 范围与证据

本任务只修正 `LightNav STOP → Monitor 完成核验 → Qwen continue_navigation → LightNav instruction` 的语义反馈接入，并执行一次新的完整导航。新证据根为 `.local/outputs/stop_feedback_20260917_01/`，正式回合为 `navigation/`。原有未提交工作保留。

实施前读取项目 memory、`NAVIGATION_NATIVE_DEPTH_STOP_20260916.md`，以及侧任务 `.local/outputs/lightnav_stop_input_ab_20260917_01/` 的 REPORT、summary、results、run.py 和 isolated_server.py。该侧任务没有修改生产代码或默认配置；其结果在本报告单独归档，不计为新导航成功。

## 已有模型输入对照支持什么

四个独立会话各使用相同的49张原始JPEG，共196次实际LightNav推理。所有组seq0–45输出相同；原样组和重复原样组均与旧记录49/49完全一致，STOP在seq45、48。

仅从seq46替换为明确纠正指令后，seq46–48均不再STOP，预测前向终点约0.40、0.40、0.66米。仅补暂停丢失的1.15秒时间表达，seq46、48仍STOP。支持“原框架没有真正下传纠正语义”是可修复接入缺口；不支持把时间编码当作重复STOP的唯一原因。

离线纠正指令包含“尚未完整通过”的肯定表述。生产代码必须依据在线证据调整其确定程度：缺少完成证据时表述“尚未核验”，不能借用离线真值断言未通过。因此本轮采用相同的正向整机通门要求与保留历史策略，但不声称文本与离线组逐字相同。

这些输入对照没有执行替代轨迹，没有定位首次STOP的全部内部原因，也没有分离各句文字的作用。后续输入仍是原来停车后的画面，不能计为实际进门或轨迹可行性证明。

## 实际链路与最小实现

1. `prototype/sim/server.py`保留模型STOP为`ARRIVAL_CANDIDATE`，由控制层停止运动；没有删除STOP或强制改为前进。
2. `prototype/pi/src/task-monitor.ts`记录这一到达候选的在线控制状态。收到`continue_navigation`时先用原有`completeIfReady`和完成条件重新核验。
3. 对未完成的继续请求，记录当前未满足条件、RobotState、可用portal/空间证据及语义审核，区分`completion_unverified`、`measured_not_through`、`measured_exit_margin_unmet`、`target_identity_uncertain`和`stopping_unverified`。最后一种要求保持停止；身份未确认时要求先找回指定目标。
4. 同一开口且仅缺完成证明时，将以下内容追加到原任务instruction并通过实际`navigate`请求发给LightNav：

   > The Monitor has not yet verified that the whole robot has passed through the doorway. Continue through the same doorway until the whole robot has passed it. Do not stop at the entrance.

5. Qwen的reason保留为预期效果/日志；新增纠正语义由在线核验生成，不依赖reason是否足够清晰。重复继续不会无限叠加纠正文本；每次消耗既有恢复预算，已有recover请求不重复计数。
6. 同一目标的完成纠正不递增`strategyRevision`，避免仅改变文本时意外重置视觉历史。另行明确替换目标身份仍保留原策略版本变更规则。
7. `stop_continuation_feedback`记录完整指令及在线依据；每次实际模型请求新增会话重置标志、会话内输入序号和发送/接收墙钟时间。原有capture/receive传感器时间与真实帧间隔继续保存。

Monitor原完成条件、360秒任务/30秒决策/最多3次恢复、LightNav时间前缀及历史选帧、MPC自由度、身体硬门槛、停车模型和相机布局均未修改。

### 实施验证

TypeScript构建及Python源码编译通过。另用旧首STOP、旧观察和旧reason-only继续请求调用编译后的实际`continueLocal`，只截获发出的动作：策略版本与scope不变，恢复次数0→1，实际instruction包含“尚未核验”和整机通门要求。该CPU回放不调用模型或物理，不计导航成功。证据：`analysis/compiled_continue_local_replay.json`。

## 唯一新回合配置

- 场景`prototype/isaac/scenarios/hospital_open_room.json`；目标“穿过右侧那扇门完全打开的门口，进入房间。”
- 起点、相机实际profile与旧`clearance_10cm_01`一致：前方0.30米、上方0.10米、俯视5度；保留RGB标定对应及约87°×58°原生深度。
- 独立`footprint_10cm.json`从旧实际resolved配置复制；期望净距0.10米，身体和硬门槛不变。不加居中语言前缀。
- 无后拉力、人工暂停或人工恢复。GPU0启动前空闲，其他项目进程未操作。
- 实验室Qwen `Qwen3.6-35B-A3B-FP8/`，thinking off，输出2048，无鉴权；调用直连，Codex专用代理不变。Gemini暂缓。
- 独立tmux `guidepup-stop-feedback-20260917`；运行命令、启动时间、预计480秒及只准一轮的范围见`run_receipt.json`。

## 新回合结果

**新真实成功0/1，Monitor为NEEDS_USER，未完整进门；停止后语义纠正分支在这次实跑中未触发。** 唯一回合已完成两项评估并关闭服务，没有重跑。导航任务墙钟145.139秒，包含启动、收尾和评估的runner为282.675秒。

| 指标 | 本轮实际证据 |
|---|---|
| 独立物理路程 | 2.98525米 |
| LightNav输入/响应 | 38/38，全部接受；唯一STOP为seq37 |
| MPC | 111/111成功且实际下发，全部采用LightNav轨迹 |
| Qwen | 5次实际调用；没有continue_navigation、change_view或request_help提议 |
| 纠正事件 | `stop_continuation_feedback`为0；恢复计数为0 |
| 实际完整通门 | false；没有整机穿越时刻 |
| 实际停稳 | true；末0.3秒最大平移0.00107米/秒，最大yaw角速度0.00142弧度/秒 |
| Monitor自动完成 | false；没有阶段完成或误完成 |
| 运行错误 | recorder_error为空、runtime_error_events为空、control_error_reasons为空 |
| 干预 | 无施力，HUMAN_PAUSED采样0，无人工恢复 |

### 1. 首次STOP的位置与真实通过情况

seq37图像曝光在仿真21.000秒，收到响应时对应状态21.200秒（任务物理采样窗口8.120–25.220秒）。

| 对齐时点 | 独立世界位置x、y、yaw | 机身包络最前端距入口平面 | 包络最后端相对远端平面 |
|---|---|---:|---:|
| STOP图像曝光 | 19.605404米、1.209056米、−1.240467弧度 | 尚差0.7432米 | −1.7033米 |
| STOP响应 | 19.620205米、1.144274米、−1.236577弧度 | 尚差0.7273米 | −1.6896米 |

使用原三圆身体包络和目标门框原生几何离线计算；整个包络尚未与该门框厚度区间相交，门洞横向净距此时不适用。这里的距离是沿门洞法向的投影，不能当作到入口的最短可行路径长度。机器人仍在走廊中，没有完整通过。真值只用于本节离线核对，没有输入在线导航。

![本轮独立物理轨迹与速度](../.local/outputs/stop_feedback_20260917_01/actual_navigation.png)

### 2. Monitor的依据与模型实际收到的指令

STOP时LightNav `visible=false`。Monitor的未满足条件是：尚无传感器观测到的相向侧边界、完整机身自由走廊及远侧扩展证据。Qwen先选两个区域，随后在同一观察上三次接受D13/D5。第一次及第三次几何拒绝为“两片表面不相向”；第二次缺少`sameOpening=true`的本次明确确认。这些拒绝不能证明目标不存在，也不是“已经确定未通过”的在线证明。

继续导航和换视角工具实际可用，但Qwen没有选择它们。三次审核无新证据后，由Monitor原有审核上限转为NEEDS_USER：

> 同一观察的目标选区三次未取得可核验证据。需要帮助：帮助确认目标位置或提供更清晰视角。

**本轮实际发给LightNav的完整instruction始终为：**

```text
Go through the open doorway on the right
```

没有新的纠正instruction实际发出。新增语义链路只在前述编译后CPU回放中验证，不能把待发文字写成这次LightNav听到的内容。

### 3. 反馈后是重复STOP还是继续执行

本轮没有语义反馈请求，也没有STOP之后的新LightNav输入；因此既没有重复STOP证据，也没有反馈后持续产生/执行入门轨迹的证据。111次成功且实际下发的MPC都在首STOP前，对应LightNav非STOP轨迹。STOP后保留的MPC状态引用前一个seq36，不能计作seq37的前进执行。

### 4. 会话与历史是否保持

38次实际输入全部处于同一session `0292d75f-2ecd-48e0-9a48-29d8e96b84a9`，scope `90d0af95-bef1-43a4-91d3-dd1ee80ee2dc:1:0:0`。仅seq0记录reset=true，后续false；seq37会话内输入序号37，真实前帧间隔0.300秒。

seq37请求发送墙钟为1789576618.5700061，接收1789576619.383274。完整传感器时刻/墙钟在`lightnav.jsonl`。没有改4fps文本时间前缀。本轮没有跨纠正边界，所以只能确认运行中的单会话连续性，不能声称在真实纠正后复现了离线组的保持历史效果；该条件仅由CPU动作回放验证。

### 5. 受阻类型

首先是LightNav主动STOP，随后是目标/开口几何证据不足且同帧重复审核，最终NEEDS_USER。**没有MPC拒绝或运行错误，也没有证据表明被门框卡住。** 尚未形成有效portal，此时不能把未完成进一步归为某一处unknown空间或已经证明不可通过。

recording退出码2代表本次NEEDS_USER；navigation退出码−15是runner收尾发送SIGTERM。原始日志及两项评估未记录运行错误，未使用fallback掩盖失败。

### 6. 真实进入、停稳与Monitor完成分别如何

实际完整进入false，独立末态机身相对房间内界净距−1.69753米；实际停稳true。Monitor没有自动完成，也没有误报。因机器人实际上未进门，本轮无法检验“成功进入后Monitor是否能正确自动完成”，更不能把走廊停车算作目标完成。

### 7. 接触与净距

本轮MPC记录的最小`minimum_clearance_m`为0.099218米、最小`stop_reserve_m`为0.014398米，所有求解成功；这是估计地图/控制计算值，不能当成真实物体全局最近净距。期望10厘米是代价偏好，不是新硬门槛，略低于10厘米不表示违反原硬约束。

接触observer非足部最大净法向力0牛，超过1牛采样0。它只有机体合成法向力，不含接触对、点和摩擦，可能包括自接触及力抵消，因此只能说未观测到非足部净法向力，不能证明全过程绝无接触。身体包络和几何评分也不是全部关节配置/完整场景的连续碰撞证明。

## 可比性与结论边界

同一场景、设定起点、相机、控制参数和用户目标已保持。Qwen本轮自行生成的初始语义为`Go through the open doorway on the right`，旧10厘米轮为`Pass through the doorway on the right that is fully open.`；两者在干预前已经不同，真实轨迹和图像也不相同。故本轮不是严格冻结初始模型文本的单变量因果比较，不能用2.985米对3.835米推断新增纠正有益或有害。

接入修正已经实现并通过编译后实际函数回放；**闭环有效性仍未验证，因为本轮Qwen没有触发继续请求。** 剩余明确缺口是STOP后重复无效审核没有转入已有的继续导航/换视角动作。此任务不再更改该策略或自动增加回合；不放宽Monitor完成条件或把真值引入在线模块。

## 证据与收尾

- 新回合：`.local/outputs/stop_feedback_20260917_01/navigation/`，含`results.json`、`physical_evaluation.json`、`navigation_evidence.json`和scenario/source_snapshot。
- 视频：`navigation/episode/camera.mp4`；Qwen输入图、裁图与完整请求/响应在`episode/agent_observations/`和`episode/agent.jsonl`。
- 实际模型/RGB-D/控制：`.local/outputs/isaac_sim_20260917_003420/`；38对JPEG/NPZ，`lightnav.jsonl`、`mpc.jsonl`、`sensors.jsonl`、`telemetry.jsonl`及独立`ground_truth.jsonl`、`contacts.jsonl`。
- STOP时刻/会话/应用轨迹分析：`analysis/new_run_evidence.json`和`analysis/reduce_new_stop_episode.py`。旧case helper有旧序号，仅用于旧证据；新分析自动枚举STOP及真实portal状态。
- 图：`actual_navigation.png`；绘图脚本`analysis/plot_actual_navigation.py`。
- `final_resources.json`确认四个本任务服务PID均已退出、独立tmux结束，GPU0无计算进程（4MiB、0%）。其他项目进程和Codex专用代理未动。

2026-09-17 00:43 HKT：本任务实现、唯一回合、两项评估和证据收尾完成。没有第二轮、扩批、训练、硬件操作或Git提交/推送。
