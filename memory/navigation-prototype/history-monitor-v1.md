# Navigation prototype：当前入口

2026-09-14 HKT。已在 Pi 中实现常驻 TaskMonitor，并实际运行“出门走到电梯门口”的两阶段仿真。基础执行协议已改善，Qwen 语义和模糊到达尺度仍不可靠。最新入口 `prototype/MONITOR_TEST.md`、`prototype/pi/MONITOR_DESIGN.md`、`prototype/pi/README.md`；原始证据 `.local/outputs/task_monitor/`。历史叙述已归档到 `history-before-monitor.md`，其中旧六工具 Agent、TaskSession 和 complete_navigation 不再是生产实现。

## 固定基础与范围

- 写入工作树 `/home/baoquanc/workspace/ForceControl-lightnav`，分支 `codex/a2-lightnav-prototype`，base `4605004`；原 `ForceControl` 有新版未提交工作，不是写入目标。未 commit/push。
- `.local` 指向 `/mnt/ssd1/baoquanc/forcecontrol-lightnav`；代码在工作树，环境/模型/输出在数据盘。所有网络和下载直连，清除大小写 HTTP/HTTPS/ALL_PROXY，NO_PROXY/no_proxy=`*`；tmux 也遵守。
- 旧 run `2026-05-23_14-25-38_A2_base_only_v2/model_59999.pt`，已有 `mujoco_exported/policy.pt` 与 JSON。46×32 term-major 历史、12 关节动作、旧 6D 辅助输出（velocity/base force），不是新版 9D 扶手 wrench。没有训练/重导出/换 checkpoint。
- 实验室 Qwen `http://vic.precognition.team:8888/v1`，精确 ID `Qwen3.6-35B-A3B-FP8/`，末尾斜杠保留；不下载本地模型。Pi0.85.1、Node22.23.2，Qwen thinking off。凭证仅本地未跟踪文件，不写入 memory。
- 仿真 prototype：MuJoCo、静态平地、局部语义导航与 force 交互。不含人体跟踪、人体包络、乘梯、Isaac Lab 接通或实机部署。
- D435i `d435i_rgb_nominal`：640×360、69°×42.2726°、base [.30,0,.10]m、下俯5°、目标10Hz，有效光轴深度.2–8m。是理想对齐 RGB-D，不是实机标定、原生更宽深度FOV或真实噪声证明。
- LightNav 只收原始 RGB；局部路径用拍摄时刻 base 位姿转 world，MPC 读实际位姿/速度。没有给 Qwen/LightNav 目标真值坐标或真实路线剩余距离。
- Depth 翻译器给原图、6×3编号图、区域分层统计，ROI 给真实裁图/选框/层像素图。距离统一 base 水平 hypot(x,y)，不是光轴Z、机身前缘净空或路线剩余距离；unknown 不代表自由空间。层颜色/占比不是物体语义或识别置信度。层代表像素给完整同帧变换后的 base/world 点，供测量门平面。
- MPC 仍是三圆包络（base x=−.30/0/.30、r=.30），4cm占用体素及膨胀，15×.15s预测，上限.3m/s/.6rad/s，预测实际响应/求解耗时，完整点集核验，只支持前进和转向。地图只记录4m内已观测占用。MSAA offsamples=0 修复深度边缘假点的既有结论不变。
- force brake 由 Python 独立处理：旧 auxFx≤−.6 连续2tick且 requested vx≥.2；NAV 前进转弯不受旧人工速度模式 abs(wz)≤.1 限制。HUMAN_PAUSED 锁存、仅人工恢复。检测 tick 的 effective_cmd 是刚执行的旧指令，下一50Hz tick归零；仍执行旧 policy/PD，零命令不等于位置锁定。

## 常驻 Monitor 当前实现

- `prototype/pi/src/task-monitor.ts` 是 run/chat/monitor 共用入口。FIFO 事件队列、同时最多一个有效 Qwen 决策；使用 Pi `models.complete` 生成单个提议，无自动批量工具执行。SQLite 保存任务/阶段/有限事实摘要与完整事件，JSONL 镜像，图片单独存档。
- 纯文本规划保留原文和有序 clause，检查拼接覆盖完整原始需求。当前阶段类型 pass_doorway / approach_target；英文局部导航指令。明确数字距离必须来自原话；无默认1米。
- 每次视觉决策新取状态和 RGB-D；Qwen 选0–1000坐标，Monitor转实际半开像素ROI并绑定 scene.frame_id，绝不让模型抄 SSE 帧号。顺序为观察→选区→实测→独立裁图/层审核→行动。失败无行动或完成工具；每 ROI 恰好一个真实层，不允许重复索引混充。概览不能替代目标ROI。
- 当前事实、模型像素语义主张、未知项分开。模型只看到当前ROI与明确保留的门平面，不累积全聊天/旧图片；当前图、阶段入口对比图和本次ROI图最多6张。
- 门平面由两侧选中点和里程计得到，完整三圆越过后触发检查。是条件几何：模型把门扇认成固定门框时平面仍会错。穿越事件是边沿触发，避免已在门外后持续暂停。穿门前进统一 LightNav 指令 `Move straight forward to pass through the doorway.`。
- 接近阶段必须真实执行并结束自己的局部导航，再取新目标ROI、停止状态和模型判断；不能继承上一阶段 LOCAL_DONE。阶段推进与任务终态原子持久化。所有阶段通过才记 COMPLETED，但不是独立语义成功证明。
- 用户提问先新观察，人工暂停保留阶段；只有人工resume解锁并重新观察。任务变更/暂停取消旧提议；控制版本和 monitor ID 在 Python 原子检查，迟到提议不能借新版本执行。重启等待人工，仿真实例改变则废弃旧世界几何。
- Python3秒运动租约独立于 Qwen；Monitor1Hz心跳。模型调用期间 Python 照常 force brake；租约过期停止其导航。SSE普通遥测不调用模型，候选/受阻/提问等才决策。
- 恢复记录实际指令、maneuver、起止位姿、位移/转角、控制终止原因；原地无效前进换措辞不算新策略。每阶段最多3次控制失败恢复；预算耗尽暂停并求助。转向45°为遥测触发阈值，有实际超调，并非精准转角控制。
- 实跑暂停回答曾把 force_brake 错解为障碍。新增 controlEventFact 明确后向外力估计、锁存和人工恢复权限；一次原暂停帧真实Qwen回放已正确解释，但未验证所有问法。

## 最新证据与限制

- 12个受控提议/模拟服务的 Monitor 契约用例通过，覆盖帧号分离、测量失败与依赖、层重复、暂停新图、迟到提议、换目标、阶段持久化与无效恢复。不是视觉能力证据。真实 Python 控制契约 PASS：7次越权/失效请求409、3秒运动租约停下、过期心跳不能复活、−80N后拉制动0.70sim秒。
- `primary_01` 错漏出门阶段/添加4.5米，实际10.029m且过门，最终NEEDS_USER。`primary_02` 两阶段但早判出门（真实尾缘尚差24.68cm）、3个ROI越界、无阶段2动作仍提完成；最终NEEDS_USER。`primary_03` 门前MPC受阻、1.705m未过门。所有失败保留，不用后续成功覆盖。
- `primary_04` 两段真实导航，63.05秒/9.293m、零非支撑接触；三圆22.58sim秒过真实门平面，25.60秒才推进阶段。最终门板ROI中位2.188m确有该表面支持，base到最近门面1.900m。COMPLETED不等于“门口”无歧义；左门框ROI仍是敞开的门扇。
- 暂停交互01在制动锁存同tick被recorder误判中止；原trace证明下一tick零命令，未覆盖问答/恢复。修recorder后02 Qwen起点否认看见门，NEEDS_USER，无导航/受力。保留负结果。
- 暂停交互03完整注力→锁存→放力仍保持→新图问答→人工恢复原阶段→右转受阻后左转→出门→电梯接近。89.67秒/10.856m、零非支撑接触；真正过门42.48s，阶段完成46.22s，最终最近门面1.692m。−80N API到锁存.72s，下一.02s零指令；至恢复前165个后续tick有效命令均零，但外力下最大位移.307m，不能称机械静止。
- 03原问答正确说尚未出门，但制动原因错；阶段0完成理由仍含不符当前图像的门框叙述。阶段1首ROI D14实为北墙踢脚线，最后D3代表点在电梯门板但大ROI/距离层混有门框地面，1.734m不能解释成纯门板统计；67.96%是距离带占比。语义错误仍存在。
- primary04与三次交互的任务实现相同，仅01后修录制同tick判断；之后仅新增制动原因事实并做12契约+1真实固定帧回放。不是多次独立同版本成功率实验。所有源码快照/视频/模型工具/轨迹/接触保留；primary02事后源码快照有局限说明。
- primary04共13次Qwen调用，最多24662文本字符/6图；交互03共19次，最多25628字符/6图（不含工具JSON和base64）。终态仍常驻3.03s，调用19→19；交互02也是2→2。没有多小时压力或跨起点泛化结论。

## 运行路线

- 统一入口 `prototype/run.sh build/up/reset/status/run/chat/monitor/down`。无scene的reset仍默认ProcTHOR；新任务场景用 `reset --scene room_elevator --record-contacts`。先关闭正在持有控制权的Monitor再重置。
- 常驻 `bash prototype/run.sh monitor --port 8770`，默认 `.local/monitor/tasks.sqlite` 和 events.jsonl；HTTP GET /state、POST /task{goal}、/question{text}、/pause、/resume。只绑定127.0.0.1。
- 完整录像：`.local/envs/sim/bin/python prototype/sim/record_task.py --scene room_elevator --goal '出门走到电梯门口' --output <new> --timeout-s 360 --max-recoveries 3`。
- 完整制动/问答/恢复录像：同环境 `prototype/sim/record_monitor.py --output <new> --timeout-s 360 --decision-timeout-s 30 --max-recoveries 3`；离线 `prototype/sim/evaluate_monitor.py --episode <dir>`。结果按执行、模型和独立核验分开。
- 本地持久服务名 LightNav=`forcecontrol-lightnav-serve`、sim=`forcecontrol-lightnav-sim`；Monitor 可放 `forcecontrol-task-monitor`，使用 `.local/monitor/resident/` 独立运行日志，避免改写实验产物。不要干扰其他训练tmux或GPU。

历史单阶段电梯、RGB-D MPC、三模型同帧实验见 `history-before-monitor.md` 的报告路由；那些结果不可直接升级成新 Monitor 的能力结论。
