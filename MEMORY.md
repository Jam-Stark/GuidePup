# ForceControl Memory

- Astra medium标准档单轮已结束（2026-10-16 01:03 HKT）：28次实际请求确认gpt-6-astra/medium/default并禁用fast；自然complete、真实13.370m且停稳、无导航运行错。终点仍是G标牌双门，真实建筑出口身份未证实，physical_success=null；动态估计外力STOP及planner恢复已取证。两评估/视频/报告齐全，GPU4释放、GPU3模型保留，不追加。详见prototype/NAVIGATION_ASTRA_MEDIUM_STANDARD_20261016.md及memory/navigation-prototype/description.md。

- 持续至GPT complete已结束（2026-10-16 00:27 HKT）：默认goal保留闭门、Find the building exit and stop near it.；04由GPT自然complete、实际35.615m并停稳、无导航运行错；停在带EXIT牌的G双门，非A（末距A15.483m），真实建筑出口身份未证实，physical_success=null，不计已验证成功。两评估/完整19:35.7视频/英文10x回放/轨迹及4过程账本齐全；仿真和planner关闭/GPU4释放，GPU3/PID2454462/8050保留。不自动追加。入口：prototype/NAVIGATION_PLANNER_COMPLETE_20261015.md；细节memory/navigation-prototype/description.md。

- 站立与目标交接单轮已结束（2026-10-15 22:12 UTC+8）：显式站立、同原JPEG目标框→模型首指向→GPT确认后新鲜连续预测已实现，Monitor只读。旧图2次真实GPT拒绝错误左侧对象；协议/WBC检查及第4次短物理站立±30°通过，前3失败保留并修复观察低速步态deadband与近角停步，速度/3°容差/原动作时间不改。唯一新完整任务NEEDS_USER/error=null/无误完成/真实0of1：走1.257m仍E2附近室内、末尾停稳；2次实转−61.395/−90.321°，14GPT仅3接受，导航输入/目标探测/MPPI均0。新增0.10m XY新鲜度否决因未校正WBC积分漂移误丢10条（首估25cm实际1.25cm），另1条因force authority变化；不能称本轮验证了目标交接或模型改善。事后已移除不可靠XY硬否决，XY明确未验证参考，保留IMU3°/epoch/tracking/generation；10记录窄检查和yaw/lost/epoch拒绝通过，无额外模型/仿真，修正后尚无完整闭环。125.28s学习力60.016N触发，峰66.613N，非足净法向全0且无注入力；原因/静止标定未定，不能称碰撞。GPT见持续估计求助，0恢复；停后全零且无步态，已站立故非动态紧急停车验证。两评估/root results/140.467s视频/英文图/报告齐全，5尾141.92s相等；报告prototype/NAVIGATION_TARGET_HANDOFF_20261015.md，证据.local/outputs/campus_target_handoff_20261015_01。任务服务关闭/GPU4释放，GPU3/PID2454462/8050保留。不得计离线/短验证为新成功；无追加完整/训练/硬件/Git。

- 目标交接只读复盘（2026-10-15 21:26 UTC+8）：同一已完成出口任务未新增运行/模型调用或生产改动。d10当前图134.04/rgb_001988已见右侧大厅/玻璃外绿植；不要误用d11当前图或据此声称GPT混淆新旧图。seq43在141.30同图目标opos[280,35.26]却落在左侧近玻璃/rot_left，说明该步目标交接没有指向同一对象，不能把全部失败归为GPT识别。d10等待363零tick仍真实右转12.421°、净移.55490m/行程.79055m，318tick force_walking；12次图龄6.12–9.48s。engine只generation校验；client零命令仍可20N进入/5N维持受力步态，区别于60N STOP；未隔离全部运动因果。建议先明确站立等待/观察settle权限与图像新鲜度，再做frame_id+目标区域及首帧导航指向给GPT核对；框不是现有导航API输入，Monitor仍只读。建议未实施。详见现报告Follow-up、analysis/planner_handoff_audit.json与对照图。GPU3保留，无新增完整任务。

- 自然建筑出口单轮已结束（2026-10-15 21:03 HKT）：按Owner撤去树篱/地垫等细节，默认goal为“Find the building exit and go outside.”；仅task.goal变化，相机/起点/代码/控制参数不变，未向在线给图/地图/目标坐标。此前细节指令回合因Owner改任务SIGINT取消，保留88.467s部分视频，不计完成或运行故障。新完整任务NEEDS_USER/error=null/GPT未误完成，真实0/1：15.241m仍室内、末尾未停稳。末右90°请求实际−6.80°/估计−6.28°，不是仅里程计误报；hold352零tick仍移.481m。171MPPI全实际执行、60导航输入3STOP/单会话、12GPT/8观察/3纠错cycle（仅2次导航继续）；最大GPT21.717s。估计峰47.756N未触发外力STOP，base_link非足净法向峰2901.78N对端未知，不等于学习施加力。两评估/results/176.8s完整视频/英文带底图轨迹/报告已保存，五类尾178.28s完整。报告 `prototype/NAVIGATION_BUILDING_EXIT_20261015.md`，证据 `.local/outputs/campus_building_exit_20261015_01`。GPU4释放，GPU3/PID2454462/8050保留；任务已结束，无自动追加/训练/硬件/Git。新目标为室外，不以旧A室内距离验收；描述变更未解决执行失败，也不作独立因果结论。

- 无限次数单轮已结束（2026-10-15 20:43 HKT）：自动外力STOP允许planner恢复、人工brake仍human-only；skill/GPT思考时长/纠错上限已取消，单动作物理范围保留。唯一完整回合STOP_NOT_CONFIRMED，GPT误认走廊侧玻璃而误完成，真实0/1，走12.819m、末距原A17.242m且未停稳，无运行异常；8观察/同cycle最多5、12GPT、2纠错真实运动、143MPPI全执行、51导航输入3STOP/1会话。外力估计峰48.598N未触发，真实自动STOP恢复仍未覆盖。两评估/results/完整视频/英文图已保存，见 `prototype/NAVIGATION_UNLIMITED_RECOVERY_20261015.md`；源报告 `prototype/NAVIGATION_STOP_AUTHORITY_20261015.md` 与只读GPT复盘证据保留。旧任务服务已退出，GPU3模型保留。

- 外力制动接入与单次任务（2026-10-15 20:11 HKT）：现有WBC aux[3:6]/.01为yaw-base施加外力估计；新增水平60N/连续2个不同50Hz样本锁存，覆盖导航/观察/等待，零命令+关闭force_walking/步态，planner不能解除。权限/陈旧决策拒绝检查通过，但真实制动未覆盖：新1轮NEEDS_USER/无运行异常/无误完成，走11.713m未到A/未停稳；估计峰42.409N未触发，虽有非足/机身接触observer信号。原失败精确WBC复放峰50.586N也不触发；不能称碰撞停止问题已解决。报告 `prototype/NAVIGATION_FORCE_BRAKE_20261015.md`，证据 `.local/outputs/campus_force_stop_20261015_01/`，两评估/results/视频/英文图保存。GPU4释放、GPU3模型保留，不追加。

- 观察动作skills已接入（2026-10-15 19:44 HKT）：GPT结构化动作与Pi/Qwen具名工具共用turn/advance/hold接口，均匀采图回传。两次真实task尝试均未成功：01在2次实转后、首张导航输入前断连；持续reader修复及45s同连接验证通过。02为NEEDS_USER/无运行异常/无误完成，真实12.218m、未到A且未停稳；4次观察、150MPPI实际执行。详见 `prototype/NAVIGATION_OBSERVATION_SKILLS_20261015.md` 与 navigation-prototype entry。GPU3模型保留、GPU4释放，不追加。

- 官方指令规则后的新完整回合（2026-10-15 18:54 HKT）：真实0/1，`STOP_NOT_CONFIRMED`，GPT把走廊西端另一扇玻璃门误判为A。走48.137m，末距A34.762m；第二次恢复真实执行442控制/27.002m。832 MPPI均下发、无命令不匹配/导航运行异常；最后零命令后仍移2.754m且物理未停稳。两评估/results/完整视频/对照轨迹已保存。报告 `prototype/NAVIGATION_CAMPUS_PROMPT_RULES_20261015.md`；GPU4释放、GPU3模型保留，不追加回合。

- 顶层指令规则与STOP证据已接入（2026-10-15 18:40 HKT）：GPT与Pi显式共享 `prototype/pi/prompts/lightnav-instruction-rules.txt`；STOP同时提供原始模型输入RGB/指向/边界标记、当前RGB、逐动作下发命令与估计进展、历史尝试。GPT原文决策，Monitor只读；保留Codex GPT-6.1 Sol medium/fast连接、会话、30秒/3恢复及现有控制。4旧STOP输入复放核对通过，2次实际GPT均选择继续并指出底边指向/剩余大厅，未执行新轨迹/未新增导航成功。Python/Pi构建及加载通过；GPU3模型保留。报告 `prototype/NAVIGATION_PLANNER_INSTRUCTIONS_20261015.md`，证据 `.local/outputs/planner_instruction_feedback_20261015_01/`。

- 当前RGB简化与新任务：2026-10-15 18:19 HKT：HEAD RGB简化/GPT-6.1 Sol medium fast/CUDA MPPI轨迹跟踪已落地，默认prototype/run.sh接新入口。Owner选新任务E2电梯门前→A玻璃入口（室内附近停下），spawn[12.51,-2.45,.55,180]仅初始化；无深度/占用图/在线真值，IMU+WBC辅助速度短时推算，语义参考只读。独立新完整导航1轮NEEDS_USER/真实未到达：走13.389m，末物理位置距A13.290m，已停稳；GPT无误完成/无抛出运行错误。112导航模型输入、1会话1初reset、4次STOP，3次恢复仅首7条控制/6.1cm，后两次0非零执行。298GPU MPPI成功且实际执行；中位GPU3.05ms/总4.58ms，速度上限.50m/s、真实峰值.575m/s（仿真）。FR_thigh非足部净接触峰1266.9N，对端未知；不可称无碰撞。原回合末GT缺.205s/接触缺.035s，保留未补造，已修flush-before-close/等正常退出，另静止检查3.52s在线/GT/接触完整相等，非第二轮导航。模型带图决策验证通过；HEAD参数768×272/132°/15Hz是已有仿真参数非实机已标定。英文标志物图/任务图/原生参照照片、新完整视频/物理轨迹/两项新评价/root results及源快照齐全。输出.local/outputs/campus_rgb_e2_to_entrance_20261015_01；配置证据.local/outputs/campus_reframe_20261015_01；入口prototype/navigation/README.md。所有本任务仿真/检查服务结束，GPU7释放；GPU3/PID2454462/8050保留，不关闭模型，GPU4他项目未动。不追加完整试验/训练/硬件/Git。

- 前序导航交付/模型保留：2026-10-15 17:22 HKT：校园默认任务真实成功0；初始规划失败后修复原文覆盖，02/03实际8.277/9.328m均Qwen误完成，03仍横向差4.861m。04走0.597m后Owner取消，不再重试。50.1秒英文未完成进度视频已渲染，所有失败/两评估保留。GPU4仿真及8768/8770关闭；Owner要求GPU3 LightNav保留，已在tmux guidepup-lightnav-gpu3、8050/PID2454462常驻约8980MiB。未来显式--reuse-lightnav复用且不关闭外部模型；未新增导航验证。报告prototype/NAVIGATION_CAMPUS_PROGRESS_20261015.md。

本文件是项目根目录的 file-based memory system 顶层入口。开始实现、调试、review 或文档更新前，先读取本文件，再按任务类型读取最小必要 memory entry。

## 路由

- 当前学校MF-R1场景接入：`prototype/NAVIGATION_CAMPUS_MF_R1_20261015.md`。Owner附件原样解压，独立引用层保留米制/Z-up两层模型及1602碰撞体，仅停用导入PhysicsScene。默认Isaac场景改为`campus_mf_r1_elevator.json`，任务为一层起点到中间2号电梯门前；端口8768和SIM_URL贯通，不占原8766。GPU7实际48.12秒加载验证通过，原生RGB-D有效92.11%、里程计tracking、局部地图就绪；完整导航任务0/模型调用0。电梯保持原闭门状态，未接跨层控制。预览/证据保存，服务和GPU7释放，未操作其他项目。

- 最新GRScenes任务：2026-09-20 18:14 HKT：GRScenes全236文件76.616GB直连镜像下载完成，医院ABY8原生资产按厘米缩放/原碰撞接入，GPU7唯一实际创建任务175.265秒、1.212m未出源门/未入目标、最终停稳；Qwen完成误报，真实0/1。54LightNav返回52接受，唯一有效STOPseq53距源门前缘仍63.77cm；85原生成功1候选受阻、84下发匹配，左25°恢复真实25.507°。无任务内ERROR；初始化材质缺陷和03任务创建前地面估计错误单列。预览/模型启动等待已改为暂停仿真时钟，04的hold/release均1.74s。两评估/results/视频与原始证据完整，服务/端口/tmux关闭，GPU7释放，不追加。报告`prototype/NAVIGATION_GRSCENES_20260920.md`。

- 当前取消全任务时间预算与唯一新回合：`prototype/NAVIGATION_NO_TASK_BUDGET_20260918.md`。移除Pi任务timer/CLI参数和记录器整回合超时，保留决策30秒/全任务3恢复/原MPPI及传感器配置。新GPU0任务实际575.443秒，越过旧360秒点仍导航；9.420m完整出源门、目标门外前缘仍差25.95cm，停稳但未入房，Qwen完成误报，真实0/1。3Qwen调用/1连续LightNav动作/0恢复，184输入唯一STOP；415原生成功414下发0blocked，无运行错误。两评估/证据/memory完成，服务关闭，不追加。

- 前序停车模型/MPPI职责修正（本轮沿用）：`prototype/NAVIGATION_STOP_PLANNING_20260917.md`。Python只保留当前/延迟前缀基本几何检查；原生候选和最终命令统一评价，原7.033cm固定经验误差保留，初始仅包络不足可按零指令比较上限在固定2.4秒内恢复。旧4失败输入均获512候选规划并通过最终检查（非导航成功）。唯一新GPU0任务真实0/1：7.507m完整出源门、未入目标、终止未停稳；259原生成功/255下发/0blocked，全正常模式，恢复分支未闭环触发。LIMIT先发生，随后地面缺失ERROR；两评估/results与证据已保存，服务结束，不追加。

- 前序Qwen/LightNav只读观测与原生Nav2 MPPI替换：`prototype/NAVIGATION_MPPI_20260917.md`。架构和官方1.3.13替换已完成；唯一GPU1完整任务真实0/1 LIMIT_REACHED，0.969m未整机出源门，113原生求解执行/4停车包络求解前拒绝。4动作原文实传、单会话、0STOP；后三次恢复未实际转动。无正式运行错/完成误报，两评估/证据/memory已保存，服务结束，不追加任务。此前能力讨论“未实施”状态已被本轮接续。


- 最新LightNav-0能力与架构讨论：原文`prototype/discussions/LIGHTNAV_0_CAPABILITY_CHAT_20260917.md`，核对/讨论`prototype/discussions/LIGHTNAV_0_ARCHITECTURE_DISCUSSION_20260917.md`。已确认当前实际加载官方LightOriginsHQ/LightNav-0，与附件同一模型。Owner倾向取消阶段监督，改Qwen→LightNav→STOP→Qwen及只读完成参考；讨论建议另保留执行受阻事件（上轮0STOP/4blocked），评估独立局部避障/MPPI。仅资料归档、核对与讨论，未修改运行实现或新增实验。

- 当前Qwen决策权修复与完整测试：`prototype/NAVIGATION_QWEN_AUTHORITY_20260917.md`。Qwen显式选择阶段/策略并生成原样下传instruction；Monitor仅反馈证据和认证，不再补写语义或自动推进。未核验阶段可暂留DEFERRED，原完成条件不变。CPU和2次实际Pi验证通过；新完整回合真实0/1 LIMIT_REACHED，2.364米、4次MPC受阻（含2次迭代上限），整机尚未完整出第一门。5个动作文本已逐动作ID核对，单会话保持，无LightNav STOP；不同阶段切换/STOP纠正未在本轮覆盖。两评估、证据/memory完成，服务结束，无追加回合。

- 前序原房间→全开门房间新回合：`prototype/NAVIGATION_ROOM_TO_ROOM_20260917.md`。按Owner新授权恢复原起点，一次GPU0/Qwen真实0/1 NEEDS_USER：完整出第一门、走5.938米，未进目标房间。207MPC全成功；Monitor仍卡阶段0。18次可见提示深度距离6.54–8.26米，未触发2米取证，未建立出口portal；STOP后同帧三次身份拒绝，无continue/反馈触发。两评估、视频与证据完整，服务结束；没有改生产导航代码或自动再跑。

- 前序STOP语义反馈接入与唯一新回合：`prototype/NAVIGATION_STOP_FEEDBACK_20260917.md`。同目标continue把在线未满足条件真正写入LightNav instruction并保留history/原恢复预算；编译和CPU原case回放通过。新回合真实0/1、NEEDS_USER，2.985米、111次MPC全成功，唯一seq37 STOP后同帧三次审核失败；Qwen未调用continue，故新增反馈未在闭环触发、有效性尚未验证。实际停稳但未进门，无误完成或运行错。离线196推理的语义/时间对照单列，不计成功；两评估/证据/memory完成，服务结束，无第二轮。

- 前序完整深度／停车／恢复与独立对照：`prototype/NAVIGATION_NATIVE_DEPTH_STOP_20260916.md`。完整87×58深度已接通，sensor-only停步包络与有界恢复已实现；新深度首轮0/1，四有效比较0/4/NEEDS_USER，后移30cm另为自机遮挡地面缺失ERROR。均值预测非全面改善、无成功脱离，旧贴trash末态未被新回合复现。两评估/新证据/memory完整，服务结束；不扩批。

- 当前 workflow v1.4.0 与同步证据：`memory/workflow/description.md`；角色按 `.codex/TEAM.md`，历史 roster 不代表存活 agent。

- 跨机器继续当前导航测试：`prototype/MIGRATION.md`，发布分支 `codex/a2-lightnav-prototype`，迁移标记 `20260915_215130_HKT`。新机器已恢复，GPU0/Qwen p05新回合真实成功0/1、NEEDS_USER，Gemini按Owner要求暂缓；最新结果`prototype/NAVIGATION_MIGRATION_20260916.md`。

- 当前传感器状态重建：`prototype/ROBOT_STATE.md`、`prototype/NAVIGATION_SENSOR_REBUILD_20260916.md`；仅RGB-D/IMU/关节输入，共享odom给几何/Monitor/MPC，完美位姿下的同侧开口误判已关闭。有界功能运行通过；随后一次正式Qwen p05于89.475s后因工具纠正HTTP400终止，NEEDS_USER、后拉未触发，并发现定位漂移；见`prototype/NAVIGATION_QWEN_SENSOR_TEST_20260916.md`。尚无实机验证。

- 前序修复与分工研究：`prototype/NAVIGATION_REPAIRS_20260916.md`；非法Qwen参数历史导致的纠正HTTP400已复现并修复，首次参数异常来源仍未确定。里程计改为深度/IMU/本体速度融合，新19秒191帧配对位置RMSE由26.32cm降至4.77cm。研究后Owner批准局部自主权实施，当前状态见下一条及`prototype/LIGHTNAV_AUTONOMY_BOUNDARY.md`。

- 当前已实现的局部自主权与最新回合：`prototype/NAVIGATION_LOCAL_AUTONOMY_20260916.md`。Owner随后批准实施并重跑一次GPU0/Qwen p05：106/106生效MPC采用LightNav轨迹，真实成功0/1、NEEDS_USER、未完整出门。正确相向门框之后，通路/扩展42unknown+1occupied；Qwen同帧重复审核，无误完成。−80N/暂停/问答/人工恢复执行，但问答错误否认真实受力。两项evaluator/results已完成，服务结束；无扩批。

- 历史全开门归因（实施已由当前有序任务接续）：`prototype/NAVIGATION_ISAAC_OPEN_ROOM_20260916.md`后续更新；侧任务证据`.local/outputs/stop_clearance_diagnosis_20260916/`。垃圾桶历史RGB-D占用已关联，首次局部净距误差约5mm；停车模型即用真实初始状态仍低估余量消耗，实际停步后约不足1cm导致恢复被拒。LightNav左转恢复预测已接受但未执行。先修停车预测/余量，再几何恢复；隔离净距/语言对照不计闭环成功，生产修复未实施。

- 最新全开门直接任务：`prototype/NAVIGATION_ISAAC_OPEN_ROOM_20260916.md`。Owner要求改到旁边门完全敞开的房间；从旧走廊终点新初始化，一次GPU0/Qwen103.254s、0/1 NEEDS_USER，移动3.278m未入目标门。146生效MPC全LightNav、0STOP；净距blocked后换视角也未执行，三次not_target同帧审核后求助。首净距.014m<.015m且裸三圆占用0，不能套用旧254点解释。两项评估/视频已保存，服务结束。

- 最新原生Isaac多房间：`prototype/NAVIGATION_ISAAC_20260916.md`，运行入口`prototype/isaac/README.md`。Hospital617文件1.07GB全直连，sensor-only原生PhysX/RTX已接通。一次GPU0/Qwen142.128s，完整0/1/NEEDS_USER；实际已出第一门、走6.427m，Monitor未建portal仍卡阶段0。236生效MPC全LightNav，4次地图包络重叠blocked，3次恢复耗尽；无误完成/运行错。两项评估/视频完成，服务结束。新场景/物理等同时变化，不能归因旧MuJoCo渲染。

- 最新无干预对照：`prototype/NAVIGATION_NO_INTERACTION_20260916.md`。Owner要求去掉后拉/人工暂停/问答/恢复；新GPU0 p05实际外力0、HUMAN_PAUSED为0，仍在连续0.30–0.40s输入下提前STOP，首STOP时前缘距入口1.083m。真实0/1、NEEDS_USER，81次MPC采用LightNav，Qwen继续后再次STOP。40导航文件同原快照，但指令Move/Pass差异且施力前已分歧；只能证明无干预也可复现，不能确定原STOP因果。两项评估完成，服务结束。

- 旧 A2 + LightNav + Pi navigation：见 `memory/navigation-prototype/description.md`，入口 `prototype/README.md`；最新通用框架修正/上下文回放/运行限制 `prototype/NAVIGATION_FRAMEWORK.md`，此前YOLO与两模型72轮 `prototype/NAVIGATION_YOLO.md`，压力及80帧异常 `prototype/NAVIGATION_STRESS.md`，v2 `prototype/NAVIGATION_V2.md`。

- MuJoCo sim2sim experiment 配置核查/解释（alan）：见 `memory/mujoco-experiment/description.md`
- MuJoCo 官方 API / XML / stepping 规范核查：见 `memory/mujoco_official_api/description.md`
- Manager-based uniFP Aliengo cfg 核查/解释/修改方案（john）：见 `memory/manager_based_cfg/description.md`
- 领域角色历史与任务分配路由：见 `memory/agent_roster/description.md`

## 使用约束

- Owner 2026-09-14：后续所有下载使用服务器自身网络直连，不走代理。下载命令须显式清除大小写 HTTP/HTTPS/ALL_PROXY，设置 NO_PROXY/no_proxy=`*`；tmux 可能保留旧代理环境，不能只看当前 shell。安装工具的显式代理配置也需覆盖。
- Owner 2026-09-15：迁移所需 Git commit/push 和 checkpoint/log 的 Google Drive 上传已授权；本次 Drive 上传必须走代理。下载与模型服务继续按直连入口执行。
- Qwen 不在本机下载：使用 Owner 提供的实验室服务 `http://vic.precognition.team:8888/v1`，模型 `Qwen3.6-35B-A3B-FP8/`，无需鉴权。

- memory 只记录可复用的项目事实、施工状态、设计决策、当前 blocker 和下一步 TODO，不作为聊天历史。
- 每个 entry 先读 `description.md`；需要施工或判断当前状态时，再读 `TODO.md` 和 `DONE.md`。
- 完成 TODO 时，同步更新对应 `TODO.md`、`DONE.md`、`description.md` summary；如改变路由，同步更新本文件和 `memory/MEMORY.md`。
- Timestamp 格式使用 `YYYY-MM-DD HH:MM HKT`。
