# 常驻任务 Monitor：实现与实测

2026-09-14 HKT。已实现并实际运行“出门走到电梯门口”的两阶段流程，也完成一次同任务内的后拉制动、暂停看图问答和人工恢复。事件与执行依赖问题得到改善；**尚不能宣称 Qwen 已能稳定、准确地自主完成这句话所表达的目标**。同配置出现过出口识别失败、门扇误当门框、踢脚线误当电梯门；“门口”仍有尺度歧义。

设计见 [MONITOR_DESIGN.md](pi/MONITOR_DESIGN.md)，使用方法见 [Pi README](pi/README.md)。所有回合，包括失败和录制程序中止，都在 [task_monitor](../.local/outputs/task_monitor/) 保留。下文严格分开执行记录、模型判断与离线核验；`COMPLETED` 只表示 Monitor 接受了有条件的阶段证据。

## 实现边界

- Pi 任务进程中只有一个 Monitor、一条事件队列、最多一个有效的 Qwen 提议。复用 SSE；普通遥测只更新状态。规划、阶段观察、候选到达、受阻、用户提问和人工恢复触发决策。
- 原始需求、阶段顺序、未满足条件、测量事实、像素语义主张和未知项写入 SQLite；完整事件镜像为 JSONL。重启恢复进度并等待人工，仿真实例变化会使旧几何证据失效。
- 每次观察由 Monitor 取得状态和 RGB-D。Qwen 只选归一化像素区域；Monitor 绑定实际 scene 帧，测量后单独发起裁图/距离层审核，再进入行动决策。测量失败时没有行动或完成工具；概览不是目标 ROI 证据。
- 阶段推进与局部导航结束分开。穿门检查测量门平面与完整三圆包络；接近阶段必须执行并结束自己的导航，再用新目标 ROI、停止状态和模型判断申请完成。未从用户原话获得数值时，不添加 1 米或其他距离阈值。
- Python 保留运动权。人工暂停使旧提议失效，只能人工恢复；强制动不等待模型。控制版本比较防止迟到动作执行，独立 3 秒心跳超时停止 Monitor 所有的导航。
- 恢复看到实际动作、位移、转角和终止原因。前进、左转、右转是当前能力；没有倒退。更换前进措辞不算新策略。受阻预算用尽后保持暂停并说明需要的帮助。

新增 `room_elevator` 为原生 MuJoCo 静态场景：4×4 米房间连接 10×8 米大厅，门洞净宽 2.2 米，起点电梯被遮挡。不是 Open-RMF 场景转换；没有电梯开门、乘梯或人体跟踪。

## 配置和证据

| 项目 | 本次实际使用 |
|---|---|
| 原话 | `出门走到电梯门口`，未补充距离或目标坐标 |
| 场景/起点 | `room_elevator`，spawn `[2.3,-6.6,0.55]`，yaw 73°；实际起步有站立收敛偏差 |
| 相机 | `d435i_rgb_nominal`，640×360，69°×42.2726°，base 前伸 .30 米、上移 .10 米、下俯 5°；理想同帧 RGB-D |
| 控制 | LightNav 原始 RGB → 同帧位姿变换 → 身体约束 MPC → 旧 A2 policy 50 Hz |
| policy | 旧 `model_59999.pt` 已有导出，6D 辅助输出；没有换 checkpoint 或重新训练 |
| 模型 | 实验室 `Qwen3.6-35B-A3B-FP8/`，thinking off；Pi 0.85.1，required 单工具提议 |
| 预算 | 每任务 360 秒、每决策 30 秒、每阶段最多 3 次控制失败恢复 |
| 运动包络 | 三圆，base x=−.30/0/.30 米，各 r=.30 米；原 MPC 净空参数保留 |
| 记录 | 5 FPS RGB 视频、5 Hz 录制状态、原服务 20 ms 轨迹与 5 ms 接触、SSE、模型请求与回复、工具测量、SQLite、源码快照 |

独立核验读取场景真实门孔、门板几何和实际轨迹，不把这些真值送给 Qwen 或 LightNav。穿门登记平面为 `y=-4.1`，孔口 `x=[2,4.2]`；跨过中心平面不等于清空门框所有厚度。距离为 base 到有限电梯门面的最近水平距离，和某个 ROI 表面距离中位数定义不同。接触统计只排除四足球体与地面的正常支撑；身体/腿部碰地仍算非支撑接触。

## 先验证已发生的具体问题

[12 个确定性契约用例全部通过](../.local/outputs/task_monitor/monitor-contract-final.tap)。它们运行生产 Monitor 和 SQLite，使用受控提议与模拟控制接口，验证执行关系，**不证明模型视觉能力**：

| 问题 | 现在的行为与证据 |
|---|---|
| SSE 与 scene 帧号混用 | 模型工具不含 frame_id；测量使用刚取得的 scene。用例故意让二者不同；实际所有本次回合没有帧错配 409 |
| 未收到测量结果便行动 | 选区、真实测量、审核、行动各有顺序；失败阶段没有行动/完成工具 |
| 无效 ROI / 重复距离层混充证据 | 真实第二轮暴露像素越界后改为 0–1000 坐标转换；每 ROI 恰好一个实际层，重复索引不能被接受 |
| 暂停问答沿用旧图 | 每次问题重新取状态和图像；确定性用例与第三次真实交互都确认新图已进入请求 |
| 人工暂停/换目标后迟到决策执行 | 取消队列和当前提议、任务修订与控制 generation 校验；即使模型驱动忽略 abort，旧提议也不能执行 |
| 恢复重置阶段、前阶段完成冒充后阶段 | SQLite 恢复保留阶段；下一阶段清空旧局部完成状态，接近阶段须有自己的实际导航 |
| 受阻反复换词前进 | 根据实际尝试和位姿识别同策略，无效重复不能重新下达；实际恢复能力另看下面的正负结果 |

[真实 Python 控制服务检查通过](../.local/outputs/task_monitor/control_contract.json)：旧/失效版本及暂停越权请求共 7 次返回 409；旧人工恢复请求被拒；人工恢复清空旧指令；活动导航在 3 秒租约过期后由控制服务停止，旧心跳不能复活；真实 −80 N 后拉 0.70 个仿真秒后锁存。最初在房间测试时，LightNav 提前结束局部导航，未覆盖活动导航超时，因此保留 `control_contract_initial.json` 的未覆盖结果，再用 flat 红箱任务执行上述控制检查。这不是视觉导航测试。

## 多阶段实际任务

场景、目标、相机、policy 和 MPC 固定，但 primary 各轮之间根据实际失败修改 Monitor/指令，因此下表是开发迭代记录，不能计算为固定实现成功率。`primary_04` 与三次暂停交互使用相同任务实现；第一次暂停交互后仅修了 recorder 的同 tick 判读。最后另有制动原因解释的小改动，只做了契约检查和真实 Qwen 固定帧回放，未把它称为新的完整路线实测。

| 回合 | 执行结果 | 模型/Monitor 判断 | 独立核验与限制 |
|---|---|---|---|
| [primary_01](../.local/outputs/task_monitor/primary_01/independent_review.md) | 68.45 秒，10.029 米，零非支撑接触 | 错漏“出门”阶段，添加 4.5 米；最终 NEEDS_USER | 实际过门、距电梯 2.143 米；不能将实际运动补成正确阶段计划。6 次 ROI 都成功，模型却声称帧对齐失败 |
| [primary_02](../.local/outputs/task_monitor/primary_02/independent_review.md) | 40.58 秒，3.153 米，零非支撑接触 | 两阶段计划正确；过早结束第一阶段；第二阶段无自身导航仍提完成，未被接受 | 阶段切换时尾缘仍在真实门内 24.68 厘米；3 次 ROI 越界，另有重复层覆盖及墙面误选；最终距电梯 7.946 米 |
| [primary_03](../.local/outputs/task_monitor/primary_03/independent_review.md) | 37.83 秒，1.705 米，零非支撑接触 | 转向后前进，MPC 受阻；NEEDS_USER | 没有过门。一次前进 1.061 米后受阻，再试仅 7.25 毫米；记录证明没有形成有效脱困 |
| [primary_04](../.local/outputs/task_monitor/primary_04/independent_review.md) | 63.05 秒，9.293 米，实际执行两段导航，零非支撑接触 | 两阶段均被接受，COMPLETED；Qwen 报目标 ROI 2.188 米 | 全包络 sim 22.58 秒过真实门平面，25.60 秒才完成阶段；最终 ROI 确为电梯门板，最终最近门面 1.900 米，仍有明显空间 |

针对实际失败已做的修正：纯文本规划并检查原文阶段覆盖；门口检查由持续满足改为穿越边沿触发；后阶段必须有独立导航；归一化 ROI 和唯一层选择；模型获得实测门中心相对方向；穿门前进统一为 LightNav 实跑有效的 `Move straight forward to pass through the doorway.`。模型仍决定何时前进、转向和核验，代码不使用真实目标坐标。

`primary_04` 的 2.188 米是实际选中门板 ROI 的距离中位数，不能改写为 1.900 米的最近门面距离。该回合左侧“门框”ROI 实际还是敞开的木门扇；真正过门成立，并不证明在线门框语义全部正确。也不能仅因电梯门在图中很大就判定已经到“门口”。

## 常驻、制动、问答与恢复

固定干预规则：第一阶段导航位移达到 .5 米且前进指令 ≥.2 m/s 后，施加 base 局部 −80 N、持续 2 个仿真秒；等待真实 force brake 及放力；问“我们已经出门了吗？现在看到什么，为什么停了？”；再明确人工恢复。没有用普通 pause 伪造 force brake。

| 回合 | 执行和模型结果 | 独立解释 |
|---|---|---|
| [human_pause_question_resume_01](../.local/outputs/task_monitor/human_pause_question_resume_01/independent_review.md) | 实际制动锁存，录制器中止，未问答或恢复 | −80 N 至锁存 .72 秒；下一 20 ms tick 有效命令为零。录制器错把锁存同一 tick 的刚执行命令当成新命令，已修。外力仍使身体位移，不能声称静止 |
| [human_pause_question_resume_02](../.local/outputs/task_monitor/human_pause_question_resume_02/independent_review.md) | Qwen 在初始图说看不到门，NEEDS_USER；没有运动和干预 | 门洞与两侧门扇实际可见，属于识别/决策失败。终态后进程仍常驻 3.04 秒，模型调用 2→2；保留失败，没有算成制动通过 |
| [human_pause_question_resume_03](../.local/outputs/task_monitor/human_pause_question_resume_03/independent_review.md) | 89.67 秒，10.856 米，干预全部执行，后续两阶段 COMPLETED；零非支撑接触 | 同阶段恢复；真正完整过门发生在阶段完成前；最终最近电梯门面 1.692 米。中间把踢脚线当电梯证据，最终层代表点在门板，但大 ROI 仍混有门框和地面 |

第三轮的原始控制记录：sim 14.30 秒注力，15.02 秒锁存，15.04 秒命令归零。15.02 秒至人工恢复前，后续 165 个 20 ms 样本有效命令均为零；放力不解锁，sim 18.32 秒人工恢复，仍执行第一阶段。后拉与身体收敛导致这段时间最大位移约 .307 米，符合旧顺应 policy 会在零命令下受力移动的限制，不能称为位置锁定。

原问答确实收到新 RGB，正确说“尚未完全出门”，但把 force_brake 错解为障碍检测。现在上下文增加来自控制服务的明确原因：后向外力估计触发的人工锁存，撤力不解锁，事件没有报告障碍或碰撞。[一次真实 Qwen 固定帧回放](../.local/outputs/task_monitor/pause_dialogue_cause_replay/result.json)正确解释后拉与人工恢复；使用原暂停图、同一问题，约 1.03 秒。它不是另一段实时制动实验，也不能证明今后所有语言解释都正确。

恢复的实际结果同时有正例和负例。第三轮右转在移动 .112 米、转动 18.2° 后 MPC 受阻，随后改为左转，实际移动 .442 米、转动约 64.4°，之后完成出门并接近电梯；真实全包络 sim 42.48 秒过门，46.22 秒记录阶段完成，尾缘裕度 +.237 米。与 `primary_03` 的失败并列，说明记录和策略变化已生效，但不能证明一般脱困可靠。45° 是遥测触发阈值，非精准转角控制。

语义核验仍有实质缺口：第三轮两个初始“门框”点均在敞开的门扇上，第一次过门 checkpoint 比真实全包络过门早；Qwen 随后选择继续动作，最终阶段推进时间才符合真实几何。阶段完成理由仍声称看见棕色门框，但当时新图主要是白墙、踢脚线和地面。接近阶段的首个 D14 ROI 实为北墙踢脚线；最后 D3 的代表点位于电梯门板，距离中位 1.734 米，但该距离层还混有非门板像素，67.96% 是距离带占比而非目标像素纯度。工具执行正确不能修正这些语义判断。

## 上下文、常驻与复现

生产决策每次只有一个重新构造的用户消息，保留当前阶段摘要、有限失败记录、本次观测/ROI及明确标注的阶段起始图；不累积完整聊天或历史图片。primary_04 共 13 次模型调用，单次文本最多 24,662 字符、最多 6 张图；交互第三轮共 19 次，单次最多 25,628 字符、6 张图。字符统计不含工具 JSON 和图片编码，[明细](../.local/outputs/task_monitor/context_metrics.json)。这只覆盖当前两阶段、约一分钟至一分钟半的任务，不是多小时压力测试。

交互第三轮任务完成后程序仍常驻 3.03 秒，模型调用 19→19；第二轮 NEEDS_USER 后同样没有继续调用。正常 SSE 遥测和心跳不触发模型。原始请求日志保留目的、图像标签、实际响应与用量，SQLite 不保存 base64 图像，磁盘路径指向实际存档。

本轮结束时已重置到房间起点，启动 `forcecontrol-task-monitor` tmux 常驻待命，地址 `127.0.0.1:8770`。运行状态/事件在 `.local/monitor/resident/`，与实验产物分开；[启动核验](../.local/outputs/task_monitor/resident_ready.json)记录无任务、无有效决策、队列为空、零模型调用，控制为 PAUSED/monitor_claim。可以提交原话：

```bash
curl --noproxy '*' -H 'Content-Type: application/json' \
  -d '{"goal":"出门走到电梯门口"}' http://127.0.0.1:8770/task
```

需要跑下面的独立录制实验时，先 `tmux send-keys -t forcecontrol-task-monitor C-c` 结束待命 Monitor，再让 recorder 占用该仿真。之后重新启动同一常驻入口即可；不触碰其他训练 session。

```bash
cd /home/baoquanc/workspace/ForceControl-lightnav
bash prototype/run.sh build
.local/envs/node-v22.23.2-linux-x64/bin/node --test prototype/pi/dist/monitor-contract.test.js

# 完整不干预任务；输出目录必须使用新名字以保留每次结果
.local/envs/sim/bin/python prototype/sim/record_task.py \
  --scene room_elevator --goal '出门走到电梯门口' \
  --output .local/outputs/task_monitor/new_primary --timeout-s 360 --max-recoveries 3

# 常驻 Monitor + 真实后拉 + 新图问答 + 人工恢复
.local/envs/sim/bin/python prototype/sim/record_monitor.py \
  --output .local/outputs/task_monitor/new_interaction \
  --timeout-s 360 --decision-timeout-s 30 --max-recoveries 3

# 离线读取实际轨迹和接触，不向在线任务输入真值
.local/envs/sim/bin/python prototype/sim/evaluate_monitor.py \
  --episode .local/outputs/task_monitor/new_interaction
```

录制实验前关闭同一仿真的其他 Monitor；每个 recorder 会重置场景和启动自己的任务进程。普通使用时启动 `bash prototype/run.sh monitor --port 8770`，通过其 `/task` 提交一句原话。Python 和 LightNav 服务保持独立；所有网络入口沿用直连、不走代理。

各回合目录包含 `camera.mp4`、`summary.json`、`agent.jsonl`、`agent_observations/`、`physical_evaluation.json` 和独立说明；交互回合另有 SQLite、`interactionlog.json`。原服务原始目录记录在 summary 中。primary_02 源码快照为事后补存，随附说明明确不能当作该回合完全一致的源码；primary_03/04 和交互回合启动前保存源码。历史失败没有被新成功覆盖。

尚未解决：固定门框与门扇/背景的稳定语义关联；模糊“门口”的可靠接近尺度；部分 MPC 受阻位姿缺少可执行脱困动作；新起点/长时间/更多阶段泛化。这里没有实机、D435i 真实噪声、人体跟随、乘梯或跨场景成功率结论。
