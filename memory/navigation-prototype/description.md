# Navigation prototype：HEAD RGB简化与导航测试

- Astra medium标准档单轮已结束（2026-10-16 01:03 HKT）：source_of_truth=prototype/navigation/codex_planner.py及独立astra-config.json；仅支持显式Sol/Astra和default/fast，实际CLI按档位启用/禁用fast，不覆写原Sol profile。本轮28/28实际请求Astra/medium/default、--disable fast_mode/--ignore-user-config通过，CLI无服务端计费档位回显；1次真实带图连接20.732s不执行运动，另保留1次本地未知ultrafast_mode flag拒绝（无推理），均不算导航成功。沿用闭门/E2/自然Find the building exit and stop near it.、HEAD RGB/Fusion/完整历史/控制权限，无任务/GPT/次数上限。唯一新完整任务953.611s墙钟，自然COMPLETED/error=null、实际13.370m且末尾停稳；28GPT/13skills(9turn+4advance)/81模型输入(12probe)/0连续STOP+1probe STOP，1会话仅初reset。216GPU MPPI求解，215实际执行匹配/7.997m连续导航；216被外力STOP抢占，非无解。60.752N/168.859°估计载荷在125.22s锁存，467tick全零且无步态；实际0.149m停步路径，0.790s后开始合格.6s停稳窗。Astra134.56s自行选左60°恢复，真实552非零tick/56.797°/0.396m后停稳（诚实报告ANGLE_NOT_REACHED）；动态阈值停步与planner恢复分支本轮覆盖，不证明接触估计已标定。非足净法向峰2358.716N(FR_thigh)早于锁存、对端未知，不能称无碰撞。后段4个.15m/s×3s观察前移分别实际.492/.497/.494/.490m，不能冒称LightNav恢复导航成功。终点[4.233,-1.952,-1.871]在同一G带EXIT牌双门前，距门平面2.298m/距A16.555m均仅离线描述；G真实室外连接仍未证实，physical_success/false_completion均null。两评估/五类尾355.64s完整/354.2s原视频全解码/35.433s英文10x回放/轨迹/root results/报告/CLOSURE齐全。只共同更换模型与处理档位，不作单变量或泛化优劣结论。全部任务服务结束/GPU4释放，GPU3/PID2454462/8050保留；不追加/训练/硬件/Git。证据.local/outputs/campus_astra_medium_standard_20261016_01；报告prototype/NAVIGATION_ASTRA_MEDIUM_STANDARD_20261016.md。

- 持续至GPT complete已结束（2026-10-16 00:27 HKT）：Owner改goal为保留闭门、Find the building exit and stop near it.。Fusion1.3.3连续gyro+accel姿态共享给WBC/状态，旧28,085固定packet实际生产类回放tilt RMSE6.466→.311°；不修正平移/heading、不等于接触力标定。独立GPT先前缺少被否决交接/决策历史，现每次补完整只读历史；04实际87请求与既往online事件逐项一致，18交接同原JPEG/单会话/仅初reset/不执行probe。新04自然COMPLETED/error=null、35.615m、末0.6s真实停稳，87GPT/65skills(31turn+34advance)/68模型输入/2连续STOP+7probeSTOP/146MPPI全实际下发匹配；后段主要由GPT观察skills移动，恢复导航片段0命令不能算运动。终点[3.843,-.947,-1.796]面对OfficeGlazing/Foyer_DoorLeafGlass0–1即G，距门平面3.303m；并非A，末距A15.483m。G确有Exit.png标牌/把手，但资产visual_estimate且室外连接未证实，physical_success/false_completion均null。非足net normal峰511.985N(RR_thigh)，对端未知；学习力峰26.654N、无60N stop/恢复，接触检测未验证。4过程分列：01旧室外目标NEEDS_USER，02Owner改目标取消，03因反馈缺口修复中断，04自然完成声明/身份未证实；两部分不算成功，离线/旧任务不算新成功。两评估/五类尾1177.14s完整/19:35.7原视频解码通过/1:57.57英文10x回放/轨迹/root results/源快照/报告/CLOSURE齐全。任务全部结束，GPU4释放；GPU3/PID2454462/8050常驻保留，无训练/硬件/Git，不自动追加。source_of_truth=prototype/robot/proprio_odometry.py、prototype/navigation/engine.py/decision_evidence.py/planner-system.txt；证据.local/outputs/campus_planner_complete_20261015_04及planner_complete_20261015；报告prototype/NAVIGATION_PLANNER_COMPLETE_20261015.md。

- 站立与目标交接单轮已结束（2026-10-15 22:12 UTC+8）：显式站立、同原JPEG目标框→模型首指向→GPT确认后新鲜连续预测已实现，Monitor只读。旧图2次真实GPT拒绝错误左侧对象；协议/WBC检查及第4次短物理站立±30°通过，前3失败保留并修复观察低速步态deadband与近角停步，速度/3°容差/原动作时间不改。唯一新完整任务NEEDS_USER/error=null/无误完成/真实0of1：走1.257m仍E2附近室内、末尾停稳；2次实转−61.395/−90.321°，14GPT仅3接受，导航输入/目标探测/MPPI均0。新增0.10m XY新鲜度否决因未校正WBC积分漂移误丢10条（首估25cm实际1.25cm），另1条因force authority变化；不能称本轮验证了目标交接或模型改善。事后已移除不可靠XY硬否决，XY明确未验证参考，保留IMU3°/epoch/tracking/generation；10记录窄检查和yaw/lost/epoch拒绝通过，无额外模型/仿真，修正后尚无完整闭环。125.28s学习力60.016N触发，峰66.613N，非足净法向全0且无注入力；原因/静止标定未定，不能称碰撞。GPT见持续估计求助，0恢复；停后全零且无步态，已站立故非动态紧急停车验证。两评估/root results/140.467s视频/英文图/报告齐全，5尾141.92s相等；报告prototype/NAVIGATION_TARGET_HANDOFF_20261015.md，证据.local/outputs/campus_target_handoff_20261015_01。任务服务关闭/GPU4释放，GPU3/PID2454462/8050保留。不得计离线/短验证为新成功；无追加完整/训练/硬件/Git。

- 目标交接只读复盘（2026-10-15 21:26 UTC+8）：同一已完成出口任务未新增运行/模型调用或生产改动。d10当前图134.04/rgb_001988已见右侧大厅/玻璃外绿植；不要误用d11当前图或据此声称GPT混淆新旧图。seq43在141.30同图目标opos[280,35.26]却落在左侧近玻璃/rot_left，说明该步目标交接没有指向同一对象，不能把全部失败归为GPT识别。d10等待363零tick仍真实右转12.421°、净移.55490m/行程.79055m，318tick force_walking；12次图龄6.12–9.48s。engine只generation校验；client零命令仍可20N进入/5N维持受力步态，区别于60N STOP；未隔离全部运动因果。建议先明确站立等待/观察settle权限与图像新鲜度，再做frame_id+目标区域及首帧导航指向给GPT核对；框不是现有导航API输入，Monitor仍只读。建议未实施。详见现报告Follow-up、analysis/planner_handoff_audit.json与对照图。GPU3保留，无新增完整任务。

## 当前自然出口指令任务已结束（2026-10-15 21:03 HKT）

- 自然建筑出口单轮已结束（2026-10-15 21:03 HKT）：按Owner撤去树篱/地垫等细节，默认goal为“Find the building exit and go outside.”；仅task.goal变化，相机/起点/代码/控制参数不变，未向在线给图/地图/目标坐标。此前细节指令回合因Owner改任务SIGINT取消，保留88.467s部分视频，不计完成或运行故障。新完整任务NEEDS_USER/error=null/GPT未误完成，真实0/1：15.241m仍室内、末尾未停稳。末右90°请求实际−6.80°/估计−6.28°，不是仅里程计误报；hold352零tick仍移.481m。171MPPI全实际执行、60导航输入3STOP/单会话、12GPT/8观察/3纠错cycle（仅2次导航继续）；最大GPT21.717s。估计峰47.756N未触发外力STOP，base_link非足净法向峰2901.78N对端未知，不等于学习施加力。两评估/results/176.8s完整视频/英文带底图轨迹/报告已保存，五类尾178.28s完整。报告 `prototype/NAVIGATION_BUILDING_EXIT_20261015.md`，证据 `.local/outputs/campus_building_exit_20261015_01`。GPU4释放，GPU3/PID2454462/8050保留；任务已结束，无自动追加/训练/硬件/Git。新目标为室外，不以旧A室内距离验收；描述变更未解决执行失败，也不作独立因果结论。

- 无限次数单轮已结束（2026-10-15 20:43 HKT）：自动外力STOP允许planner恢复、人工brake仍human-only；skill/GPT思考时长/纠错上限已取消，单动作物理范围保留。唯一完整回合STOP_NOT_CONFIRMED，GPT误认走廊侧玻璃而误完成，真实0/1，走12.819m、末距原A17.242m且未停稳，无运行异常；8观察/同cycle最多5、12GPT、2纠错真实运动、143MPPI全执行、51导航输入3STOP/1会话。外力估计峰48.598N未触发，真实自动STOP恢复仍未覆盖。两评估/results/完整视频/英文图已保存，见 `prototype/NAVIGATION_UNLIMITED_RECOVERY_20261015.md`；源报告 `prototype/NAVIGATION_STOP_AUTHORITY_20261015.md` 与只读GPT复盘证据保留。旧任务服务已退出，GPU3模型保留。

## 当前外力制动与单次任务（2026-10-15 20:11 HKT）

- source_of_truth：`prototype/robot/force_brake.py`、`prototype/isaac/client.py`、`prototype/navigation/{engine,observation_skills}.py`、共享`planner-system.txt`；协议`prototype/navigation/FORCE_BRAKE.md`，报告`prototype/NAVIGATION_FORCE_BRAKE_20261015.md`。
- force aux[3:6]/exported .01换算名义N，训练标签为yaw-base施加在base的外力命令，不是接触合力/部位/物体。x前/y左/z上；向量是机身受力方向，不能直接当接触侧。保留旧brake 60N/2个不同50Hz样本，扩为水平范数、取消仅前进资格，覆盖纯转/零命令等待；竖直/力矩不触发。plant优先锁存零命令、force_walking=false、步态[0,1]，不冻结物理。人工权限，不提供planner解除/自动恢复。撤销path/generation，中断技能保留部分图，丢弃旧planner/model结果，新FORCE_BRAKE请求提供力/方向/中断活动；语义Monitor仍只读。
- 最小验证：旧02记录5065tick/20261packet用原WBC精确复放，aux速度零误差，峰50.586N低于60故无假设制动。窄合成阈值/方向/双采样/撤力锁存及真实WBC固定传感器输入权限检查通过，旧决策丢弃和planner越权请求拒绝通过；不算物理制动或导航成功。
- 新唯一任务`.local/outputs/campus_force_stop_20261015_01/`：284.598wall秒、93.88sensor秒、NEEDS_USER/error=null/无误完成，真实11.713m，末距A30.086m，未到达/未停稳；127MPPI求解全实际执行、45模型输入/1STOPseq44、1会话1初reset、6GPT调用/4观察/恢复1of3。GPT先两次左90，再去亮板观察子目标，STOP后hold1s与左90后因目标不明/停止未确认求助。hold200零tick实际仍移.365m；末左90实际86.82°，不能归为本轮同样卡转。
- 关键限制：本轮估计峰42.409N（[-41.973,+6.070,+.100]N，171.77°后稍左），没有制动触发/没有真实FORCE_BRAKE planner调用。离线净法向峰FR_thigh3535.78N/base_link1283.65N，对端未知；不与学习施加力等同，也未供在线使用。新机制不能被称为可靠碰撞检测或问题已解决；阈值未降低。940零命令tick仍因原受力规则walking，不等于已证明全部残余运动原因。
- task/config/相机标定与旧02逐项相同；完整RGB/IMU/关节/force+gait/模型输入输出/MPPI/独立物理与接触、两主评估+observer/force覆盖、根results、英文轨迹与力时序图保存。视频1409帧/93.933s完整解码；五类尾95.40s相等。独立tmux/仿真/视频进程退出，GPU4释放；GPU3/PID2454462/8050保留。无新增任务/训练/硬件/Git。实际制动后停稳/技能中断、人恢复、竖直/扭矩与障碍接触标定未覆盖，后续须新任务范围。

## 当前观察动作skills与新任务（2026-10-15 19:44 HKT）

- source_of_truth：`prototype/navigation/{engine,observation_skills,planner_contract,pi_planner,codex_planner}.py`、`prototype/navigation/{planner-contract.json,planner-system.txt,OBSERVATION_SKILLS.md}`、`prototype/pi/src/rgb-planner.ts`。Canonical report：`prototype/NAVIGATION_OBSERVATION_SKILLS_20261015.md`。
- GPT默认不变；Pi/Qwen显式backend=pi_tools，使用独立qwen-config.json，未改lab profile。两真实模型各+30°提议接口通过（不执行）。技能turn：正左负右10–180°/最大.30rad/s；advance：.10–.20m/s/.5–4s；hold：零指令。2Hz采样/每张最多12幅联系图，非标定全景。技能独占WBC命令、暂挂MPPI/导航推理；保留单会话及原控制自由度；无几何避障。每cycle最多2观察、全任务6，观察后继续导航同算1/3恢复，无整任务timer。
- 01真实执行2次右90°（实际−89.39/−88.35°、转身位移.296/.436m），在首张导航图片和MPPI前ERROR，记录保留。server默认20+20s心跳而aiohttp仅receive时自动PONG；改成持续reader+TaskGroup，不重连/不重置，实际同连接45.007s通过。
- 02真实任务NEEDS_USER，GPT未误完成，无运行异常：304.78wall秒/101.28sensor秒、实际12.218m至走廊东端，末距A30.079m，未停稳。53模型输入/1STOP，150MPPI均实际发送无不匹配，1会话/1初reset，恢复1/3。4次观察实转−89.51/−177.99/+83.11/+4.98°；末次请求+90°估计+15.27°但实际+4.98°，ANGLE_NOT_REACHED/STOP_NOT_CONFIRMED，GPT据在线结果求助，未见offline真值。末段RL_thigh净接触峰1386.58N，对端未知；零指令等planner仍有物理运动，WBC停步及原地漂移未修。
- 证据`.local/outputs/campus_observation_skills_20261015_{01,02}/`：两尝试各两项评估/results/视频/独立物理与接触；02另observer评分、wait-motion、英文轨迹与联系图。视频30.87/101.33s均完整解码、尾记录完整。advance/hold未在物理任务触发，STOP后观察再重开导航未覆盖。不把观察链路可用或旧/离线输出计成功。task/场景/相机/MPPI/WBC未改；未训练/硬件/Git/额外试验。GPU3 PID2454462/8050保留、GPU4释放。

- 官方指令规则后的新完整回合（2026-10-15 18:54 HKT）：真实0/1，`STOP_NOT_CONFIRMED`，GPT把走廊西端另一扇玻璃门误判为A。走48.137m，末距A34.762m；第二次恢复真实执行442控制/27.002m。832 MPPI均下发、无命令不匹配/导航运行异常；最后零命令后仍移2.754m且物理未停稳。两评估/results/完整视频/对照轨迹已保存。报告 `prototype/NAVIGATION_CAMPUS_PROMPT_RULES_20261015.md`；GPU4释放、GPU3模型保留，不追加回合。

- 顶层指令规则与STOP证据已接入（2026-10-15 18:40 HKT）：GPT与Pi显式共享 `prototype/pi/prompts/lightnav-instruction-rules.txt`；STOP同时提供原始模型输入RGB/指向/边界标记、当前RGB、逐动作下发命令与估计进展、历史尝试。GPT原文决策，Monitor只读；保留Codex GPT-6.1 Sol medium/fast连接、会话、30秒/3恢复及现有控制。4旧STOP输入复放核对通过，2次实际GPT均选择继续并指出底边指向/剩余大厅，未执行新轨迹/未新增导航成功。Python/Pi构建及加载通过；GPU3模型保留。报告 `prototype/NAVIGATION_PLANNER_INSTRUCTIONS_20261015.md`，证据 `.local/outputs/planner_instruction_feedback_20261015_01/`。

- current_reframe：2026-10-15 18:19 HKT：HEAD RGB简化/GPT-6.1 Sol medium fast/CUDA MPPI轨迹跟踪已落地，默认prototype/run.sh接新入口。Owner选新任务E2电梯门前→A玻璃入口（室内附近停下），spawn[12.51,-2.45,.55,180]仅初始化；无深度/占用图/在线真值，IMU+WBC辅助速度短时推算，语义参考只读。独立新完整导航1轮NEEDS_USER/真实未到达：走13.389m，末物理位置距A13.290m，已停稳；GPT无误完成/无抛出运行错误。112导航模型输入、1会话1初reset、4次STOP，3次恢复仅首7条控制/6.1cm，后两次0非零执行。298GPU MPPI成功且实际执行；中位GPU3.05ms/总4.58ms，速度上限.50m/s、真实峰值.575m/s（仿真）。FR_thigh非足部净接触峰1266.9N，对端未知；不可称无碰撞。原回合末GT缺.205s/接触缺.035s，保留未补造，已修flush-before-close/等正常退出，另静止检查3.52s在线/GT/接触完整相等，非第二轮导航。模型带图决策验证通过；HEAD参数768×272/132°/15Hz是已有仿真参数非实机已标定。英文标志物图/任务图/原生参照照片、新完整视频/物理轨迹/两项新评价/root results及源快照齐全。输出.local/outputs/campus_rgb_e2_to_entrance_20261015_01；配置证据.local/outputs/campus_reframe_20261015_01；入口prototype/navigation/README.md。所有本任务仿真/检查服务结束，GPU7释放；GPU3/PID2454462/8050保留，不关闭模型，GPU4他项目未动。不追加完整试验/训练/硬件/Git。
- source_current_rgb：`prototype/navigation/{run,engine,codex_planner}.py`、`prototype/navigation/{config,task}.json`、`prototype/control/{gpu_tracking.py,rgb_tracking.json}`、`prototype/robot/proprio_odometry.py`、`prototype/isaac/{worker,client}.py`。
- previous_last_verified：2026-10-15 17:22 HKT：校园默认任务真实成功0；初始规划失败后修复原文覆盖，02/03实际8.277/9.328m均Qwen误完成，03仍横向差4.861m。04走0.597m后Owner取消，不再重试。50.1秒英文未完成进度视频已渲染，所有失败/两评估保留。GPU4仿真及8768/8770关闭；Owner要求GPU3 LightNav保留，已在tmux guidepup-lightnav-gpu3、8050/PID2454462常驻约8980MiB。未来显式--reuse-lightnav复用且不关闭外部模型；未新增导航验证。报告prototype/NAVIGATION_CAMPUS_PROGRESS_20261015.md。
- source_of_truth：`prototype/robot/{state,odometry,sensors}.py`、`prototype/sim/{a2,server,depth,depth_map,truth}.py`、`prototype/control/{mppi,stop_model}.py`、`prototype/control/nav2/`、`prototype/pi/src/{task-monitor,monitor-evidence,monitor-tools,decision,sim-client}.ts`、`prototype/pi/navigation-contract.json`。
- current_docs：`prototype/NAVIGATION_CAMPUS_PROGRESS_20261015.md`、`prototype/NAVIGATION_CAMPUS_MF_R1_20261015.md`、`prototype/NAVIGATION_GRSCENES_20260920.md`、`prototype/NAVIGATION_NO_TASK_BUDGET_20260918.md`、`prototype/NAVIGATION_STOP_PLANNING_20260917.md`、`prototype/NAVIGATION_MPPI_20260917.md`、`prototype/NAVIGATION_QWEN_AUTHORITY_20260917.md`、`prototype/NAVIGATION_ROOM_TO_ROOM_20260917.md`、`prototype/NAVIGATION_STOP_FEEDBACK_20260917.md`、`prototype/NAVIGATION_NATIVE_DEPTH_STOP_20260916.md`、`prototype/camera/NATIVE_DEPTH.md`、`prototype/NAVIGATION_ISAAC_OPEN_ROOM_20260916.md`、`prototype/NAVIGATION_ISAAC_20260916.md`、`prototype/isaac/README.md`、`prototype/ROBOT_STATE.md`、`prototype/NAVIGATION_FRAMEWORK.md`、`prototype/NAVIGATION_NO_INTERACTION_20260916.md`、`prototype/NAVIGATION_LOCAL_AUTONOMY_20260916.md`、`prototype/LIGHTNAV_AUTONOMY_BOUNDARY.md`；此前修复`prototype/NAVIGATION_REPAIRS_20260916.md`、重建`prototype/NAVIGATION_SENSOR_REBUILD_20260916.md`保留历史。
- read_when：继续A2/LightNav/Pi导航、状态估计、通路几何、Monitor完成或暂停恢复。

## 当前停止状态与模型保留

2026-10-15 17:22 HKT：校园默认任务真实成功0；初始规划失败后修复原文覆盖，02/03实际8.277/9.328m均Qwen误完成，03仍横向差4.861m。04走0.597m后Owner取消，不再重试。50.1秒英文未完成进度视频已渲染，所有失败/两评估保留。GPU4仿真及8768/8770关闭；Owner要求GPU3 LightNav保留，已在tmux guidepup-lightnav-gpu3、8050/PID2454462常驻约8980MiB。未来显式--reuse-lightnav复用且不关闭外部模型；未新增导航验证。报告prototype/NAVIGATION_CAMPUS_PROGRESS_20261015.md。

规划覆盖、Qwen实例/距离/odom和动作工具语义修正已保存。最新动作接口修正仅探针+04部分回合，未闭环成功。不要用02/03的COMPLETED标签计成功；不要自动恢复导航。只保留模型权重，其他任务服务退出，旧8766是他项目。

## 当前学校MF-R1场景

- Canonical report：`prototype/NAVIGATION_CAMPUS_MF_R1_20261015.md`；证据根`.local/outputs/campus_mf_r1_integration_20261015_01/`；实际raw `isaac_sim_20261015_163025`。这是场景替换/加载验证，不是完整导航回合。
- 原包`.local/assets/campus_mf_r1/多楼层场景包_MF-R1/`，89文件/130.55MB，源USD为米制Z-up、16USD层/39资产依赖全部解析，1602碰撞API。`prepare_campus.py`生成独立引用层`integration/environment.usda`，只停用导入PhysicsScene，由现有A2物理循环管理。几何/材质/碰撞未改。
- Owner选择默认任务“一层起点走到中间2号电梯门前”，配置`prototype/isaac/scenarios/campus_mf_r1_elevator.json`，原Start1F xy=[0,2]，A2 z=.55/yaw−20°。实际电梯门前表面y=−3.7225m，只用于离线评分；在线不提供场景坐标、拓扑或航点。
- `run_experiment.py`默认校园场景，新增--preview-only和--navigation-port(default8768)；记录器/Pi通过SIM_URL连接同一服务。原8766为其他项目team-scope服务(PID292116)，禁止当本任务残留进程终止。
- Owner指定GPU7；实际CUDA UUID7c8cb1d2-4ebf-e2e3-35ad-fa0f6f72924d与Kit Vulkan active7对应。48.12秒PREVIEW_COMPLETE，1.82sim秒时depth有效92.110%、848×496，sensor-only tracking、depth rank3、36地图占用体素/捕获1.62秒。无资源/材质/runtime Error，正常headless/缓存提示单列。当前验证原默认footprint，无运动；10cm独立副本在证据根可供正式任务显式选择，不覆盖默认。
- 新目标离线评分沿用原waiting convention：机身到门平面.6–.9m、朝向25°、横向.65m、原停稳条件。仅做已知.75m的解析计算检查，未实际到目标。未改Qwen/只读Monitor/导航模型/Mppi权限或阈值。
- 一层/三层摆放Z=0/11.10m；高差非实测，源轿厢近似。附带中间电梯控制器保留，但本任务不执行升降，保持原始闭门状态。A2过20mm门槛、进出梯、搭乘、跨层定位/调度均未验证；原包8kg方块结果不能当A2证据。
- 完整任务0、Qwen/导航模型调用0。原生预览/配置、原始物理/接触/传感器与资源收尾保存。ownphysics/navigation/bridge/tmux退出，8768关闭，GPU7约3MiB/0%；GPU0/6和原8766未操作，无训练/硬件/Git。后续若执行完整导航，用新目录及当前场景入口，不能把本次预览计导航成功。

## 前序GRScenes导航

2026-09-20 18:14 HKT：GRScenes全236文件76.616GB直连镜像下载完成，医院ABY8原生资产按厘米缩放/原碰撞接入，GPU7唯一实际创建任务175.265秒、1.212m未出源门/未入目标、最终停稳；Qwen完成误报，真实0/1。54LightNav返回52接受，唯一有效STOPseq53距源门前缘仍63.77cm；85原生成功1候选受阻、84下发匹配，左25°恢复真实25.507°。无任务内ERROR；初始化材质缺陷和03任务创建前地面估计错误单列。预览/模型启动等待已改为暂停仿真时钟，04的hold/release均1.74s。两评估/results/视频与原始证据完整，服务/端口/tmux关闭，GPU7释放，不追加。报告`prototype/NAVIGATION_GRSCENES_20260920.md`。

- 全量资产`.local/assets/GRScenes`；商业场景已解压，家庭分卷保留。公开HF原端点直连失败，下载直连hf-mirror，显式移除代理，Codex代理未改。
- 场景适配`prototype/isaac/prepare_grscenes.py`、`asset_python.sh`、`scenarios/grscenes_aby8_room_to_room.json`，官方UJITSO关闭；精确房间多边形只做离线评价。
- 现有最大问题仍为模型对出房/到达的判断，不授权增加语义监督者或继续试验。

## 前序取消全任务时间上限与新回合

- Canonical report：`prototype/NAVIGATION_NO_TASK_BUDGET_20260918.md`；证据`.local/outputs/mppi_no_task_budget_20260918_01/`、raw `isaac_sim_20260918_113358`。Owner只授权本次重跑，已经完成。
- 删除Pi `timeoutS`/taskTimer/armDeadline及开始、人工恢复时注册，CLI移除`--timeout-s`；记录器移除整回合超时分支，所有当前调用入口同步，manifest/summary timeout_s=null。不是设置更大截止。保留单次决策30秒、全任务最多3恢复、局部无进展/错误/人工取消；MPPI2.4秒余量恢复期限保持。
- 类型构建及Python语法通过；实际从Task.startedAt到Qwen完成575.443秒（goal_changed起573.726，记录器584.714）。旧360秒两侧均NAVIGATING，task_timeout0，证明已越过旧时间上限。
- 唯一GPU0新任务真实0/1：9.42013m、33.095sim秒整机出源门，目标未穿越；末态三圆前缘距目标门框近側25.950cm、后缘距远侧1.62495m，独立停稳为真。Qwen却声明COMPLETED，false_completion=true；不把进程exit0当导航成功。
- Qwen3调用：plan_task→select_stage→complete_task；只有1个实际完整语义导航动作、0/3恢复。184LightNav输入/返回全接受，唯一seq183STOP；一会话仅首帧reset。STOP曝光时三圆前缘尚差31.892cm，响应时27.258cm。只读反馈unverified/targetIdentity not_reviewed/passage not_observed/stableStopVerified=false，Qwen仍以房内家具、8.18m估计净位移、STOP及约.1615m预测停车余量作为入房证据。该推断不证明整机越门，未新增监督/模板纠正。
- 415原生优化成功、414命令实际下发且全部匹配最终检查，均LightNav参考，0前拒绝/0候选无解/0最终拒绝；全normal，未覆盖restoration。1结果在STOP切换时未执行。无运行ERROR/请求帮助/恢复耗尽。估计预测最小几何10.127cm/停车3.120cm；非足净法向0不能证明无接触。独立里程计位置RMSE3.967cm、末端3.978cm，不用于在线控制。
- 只改时间预算实现，但新Qwen初始指令直接完整任务，旧轮先出房指令；非冻结前缀，不能将所有路线变化归因为取消时间上限。场景/标定/相机/速度/足印/停车和MPPI配置实际逐项相同。
- 两评估/results、视频/全输入/原生候选/物理与接触/源码保存。本任务服务、原生bridge、端口/tmux关闭，GPU0释放。无训练、硬件、Git、追加实验。未来修正完成决策需要新Owner任务范围。

## 停车预测与原生MPPI职责（沿用，前序回合）

- Canonical report：`prototype/NAVIGATION_STOP_PLANNING_20260917.md`，证据根`.local/outputs/mppi_stop_planning_20260917_01/`，唯一正式任务`navigation`，raw `isaac_sim_20260917_191631`。
- Python取消经验停车包络统一前拒绝，只检查当前/已承诺延迟前缀基本几何；区分估计地图重叠、基本间隔与期望净距，均不冒充真实接触。原生候选及滤波后实际序列共享同一evaluator；实测twist/延迟/响应/nominal停步几何保留。
- 原六例仅moving→paused停车，没有静止重启/低速缩放标定；原7.03299cm残差和所有系数不变。正常状态全时域满足原停车余量；初始仅经验包络不足时，缺额不得超过零NAV命令预测基线，固定2.4s截止恢复完整余量，截止不随重算/Qwen动作重置、上限只收紧。属于工程可行性恢复，不是形式化安全证明。
- 唯一窄离线复放旧114–117行，各一次真实原生512候选；可行112/37/71/71，最终通过，均未物理执行。原生编译/必要语法检查通过，没有扩成测试套件/调参实验。
- 唯一新GPU0任务：7.50724m，30.085sim秒整机出源门，未进目标房间，末0.3s yaw0.14948rad/s故未停稳。259原生成功、255实际下发(221LightNav/34metricturn)，4切换期间未下发；255命令均对应接受的最终轨迹。3328采样候选停车包络拒绝不等于整次无解；0前拒绝/0nativeblocked，全正常模式，恢复分支未闭环触发。计算中位653ms/max1414ms，360秒墙钟预算不变。
- Qwen11调用/5动作逐字实传；117LightNav回复115接受/2丢弃，导航3STOP、观察转向7STOP提示，单会话仅首帧reset。三恢复均ARRIVAL_CANDIDATE：左15实际15.41°；第二次完整任务继续0非零tick；右25实际18.12°后超时。没有complete_task或模型误完成。
- 60.96sim秒task_timeout→LIMIT_REACHED；61.30sim秒ERROR缺少重力对齐地面，发生在超时后，不能写无运行错误，也不能倒推其造成先前STOP。results同时保留LIMIT和control_error_reasons。不得自动扩范围修地面/终态或追加运行。
- 两评估/results、视频、完整RGB-D/模型/原生候选与最终命令/源码快照保留；非足部净法向力0不能证明无接触。里程计离线RMSE12.05cm、末端26.38cm，未用于控制。所有本任务服务/端口/原生bridge/tmux关闭，GPU0释放，未动他项目/硬件/训练。

## 前序只读观测与原生MPPI替换

- Canonical report：`prototype/NAVIGATION_MPPI_20260917.md`；证据根`.local/outputs/mppi_integration_20260917_01/`，唯一正式任务`navigation_02`，raw `isaac_sim_20260917_161247`。原附件和讨论继续保留，讨论结论已由本轮实施接续。
- Qwen自行决定目标、策略与完成；语义观测不改指令、不暂停或监督阶段、不要求证书。STOP/实际执行受阻返回Qwen；模型完成声明与独立物理成功分开。保留人工权限、360/30秒及全任务3次恢复。
- 实际官方Nav2 MPPI1.3.13替换CasADi；原身体/体素/硬间隔/停车包络保留，原生路径代价与经验A2响应模型差异显式记录。记录实际命令摘要与原生请求，不把预测当运动。依赖rootless官方ROS直连；共享Isaac环境和Codex代理未修改。
- 唯一完整回合GPU1（GPU0外部占用），0/1 LIMIT_REACHED：142.372s、0.969239m、源门后缘仍差1.212m、未进目标、停稳。113实际原生求解均执行LightNav参考，4次在原生优化前被当前/延迟停车包络拒绝；后三个恢复动作非零命令为0。
- 6次Qwen调用，4动作原文实传，35返回/31接受/0STOP；仅seq0重置一个会话。没有语义门控、测量请求或误完成。Qwen首次“已在走廊”判断与起点不符。STOP/complete_task/跨阶段/测量分支本轮未覆盖。
- 两评估与results、视频和物理/接触证据完成；非足部净法向力0不能证明无接触，里程计RMSE1.412cm仅离线。全部本任务服务/端口/GPU1/tmux结束；不额外运行。之前的两个启动事件都在任务提交前，分别为配置路径错误和GPU0竞争取消，单独保留。

## 前序Qwen决策权修复与讨论（历史）

- 2026-09-17 15:14 HKT资料核对/讨论（非新运行）：Owner附件原样归档`prototype/discussions/LIGHTNAV_0_CAPABILITY_CHAT_20260917.md`，讨论结论`prototype/discussions/LIGHTNAV_0_ARCHITECTURE_DISCUSSION_20260917.md`。download_model→environment/serve→本地模型卡/config→最近实际load log与官方来源共同确认当前就是LightOriginsHQ/LightNav-0（Qwen3-VL-4B基底、RVQ/10航点），与文档同一模型。
- Owner讨论倾向Qwen驱动LightNav，Monitor不参与阶段推进监督、可只供完成参考。建议语义阶段门控及全证书结束条件退出运行，仅保留事件协调/只读证据，仿真严格完成放离线评分。只等STOP不够：最新真实回合0STOP/4MPCblocked，暂停后无新模型请求；需执行层受阻/失败事件回Qwen。该更进一步的精简尚未实施，当前代码仍是02:08记录版本。
- 当前MPC已经有深度体素/身体/延迟/停车余量约束，不能按官方纯跟踪demo理解。外部避障建议让参考路径允许局部绕行；MPPI是候选替换局部规划/跟踪角色，不是已验证解决方案；不会修复错误地图。只读语义证据与Nav2 Collision Monitor速度限停是不同职责，后者不监督阶段，也不保证零碰撞。无安装、服务、模型推理或新导航。

- Canonical source与证据：`prototype/NAVIGATION_QWEN_AUTHORITY_20260917.md`、`.local/outputs/qwen_decision_authority_20260917_01/`。移除Monitor自动选择/启动下一阶段、补写STOP语义、自动测量后继续、精确控制或脱离策略；由Qwen显式工具决定。监控与预算仍可停止运动，最终完成仍由原严格证据核验。
- `monitorFeedback`结构化给出verified/unverified/contradicted、未满足条件、测量身份/空间/RobotState/STOP来源，不生成动作文字。`continue_navigation.instruction`必填且原样下传，同目标历史保持。`select_stage`允许跨未证实阶段，旧stage=DEFERRED不授予完成，新目标scope明确重置。三次同帧审核失败交Qwen progress_reassessment而非自动help；精确执行和脱离通过显式工具请求原控制能力。
- CPU旧真实STOP输入回放验证原文传递/恢复计数/同scope、以及后续阶段选择保留DEFERRED和RUNNING。2次实际Pi：新plan2个pass_doorway；旧失败在线图像与运动上Qwen主动change_view右30°，指令原文派发被截获。这些不计实际导航成功；原conditions方法逐字相同。
- 随后唯一正式回合实际plan为1个pass_doorway涵盖全路线；226.159秒、2.36385米、0/1 LIMIT_REACHED，后三圆边缘尚差21.710厘米清空第一门，目标门未进，末态停稳，无误完成。Qwen9调用、6运动提议/5实际动作，最后提议因3恢复耗尽未下发。57模型输入/53接受/4丢弃，接受STOP0，一个session仅seq0重置。
- 5实际动作按control task ID关联到各自seq0–10/11–15/16–19/20–24/25–56，action/semantic/模型文本逐字一致。Qwen真实选择了15°转向和新走廊/右门语义。141MPC/137实际采用（125LightNav、12metric_turn）/4blocked：13.52及18.12秒净距/停车余量拒绝；21.74及32.58秒Maximum_Iterations_Exceeded。进程/HTTP/recorder异常为空不等于优化全部成功；体素来源与不收敛尚未归因。
- 本轮没有STOP、跨阶段切换或三次审核耗尽事件；这些新分支不能冒充实跑覆盖。单阶段涵盖两门时离线证书对应不一一映射，evaluator在阶段数不符时记未映射；同记录离线重算不改本轮结果。独立物理两门仍按序评分。非足净法向力0不是无接触证明。两评估、视频/RGB-D/原文、memory和收尾齐全，服务/tmux结束、GPU0释放。

## 前序原房间到全开门房间

- 来源：`prototype/NAVIGATION_ROOM_TO_ROOM_20260917.md`，`.local/outputs/room_to_open_room_20260917_01/`，raw `isaac_sim_20260917_012131/`。新scenario `hospital_room_to_open_room.json`恢复原[12.8,3.35,.55,0°]，目标portal/房间与hospital_open_room相同，先实际预览再提交一轮；新配置不改生产代码/默认相机/原生深度/独立10cm/当前STOP接入。
- 211.318秒任务，独立路径5.93837米；23.875sim秒完整出第一门（最小门框截面横向净距11.635厘米），目标门未进，末态停稳。70LightNav全接受，唯一seq69 STOP曝光36.52/响应36.70秒；此前207MPC全成功生效，无blocked。实际模型instruction始终是`Move through the door out of the current room.`，目标房间instruction未下发。
- Monitor从未认证源门portal，阶段0仍ACTIVE、阶段1/2 PENDING；18次visible=true对应的在线深度指向点距机身6.536–8.256米，未满足2米取证触发；不能套用旧回合“visible始终false”的解释。完整在线距离来源保存在hint_range_evidence.json。此次一次首帧对齐定位RMSE1.554厘米仍不能补缺失门洞证据。
- STOP后Qwen选墙体/关闭门，三次测量审核都有not_target，最后两者not_target且sameOpening=false；无continue/change_view，恢复0，反馈事件0，Monitor审核上限NEEDS_USER。真实0/1、无误完成/运行错；非足净法向力0仅为受限observer证据，不保证无接触。
- 实际plan把穿走廊设pass_doorway、最终进房设approach_target；这两个阶段未执行，不能归因其执行效果。evaluator按pass_doorway顺序映射portal对该plan存在语义限制，但本轮无完成事件所以未生成错误映射结果；独立physical ordered_portals有效。服务/tmux结束、GPU0释放，不自动再跑。

## 前序STOP反馈接入与证据边界

- 来源为`prototype/NAVIGATION_STOP_FEEDBACK_20260917.md`及`.local/outputs/stop_feedback_20260917_01/`。侧对话`.local/outputs/lightnav_stop_input_ab_20260917_01/`已纳入memory：同49JPEG/四会话/196实际推理，前46输出全同、原样两次49/49复现；只纠正语义使seq46–48不STOP，仅补暂停1.15秒仍seq46/48 STOP。这是固定图像离线预测，不是新导航成功或首STOP全部成因证明。
- 新continueLocal在已记录STOP且完成未通过时生成在线证据分级反馈，区分未核验、实测未通过/余量不足、目标身份未确认及仅停车未验证；通门正向要求为继续穿过同一开口直到整机通过、不停入口。同目标纠正不增加strategyRevision，不无意重置会话；另行身份替换仍是明确策略变更。保留原完成条件和3恢复/360秒预算，记录完整下传文本、证据、会话reset/index及真实时间；4fps时间前缀不变。
- 编译后旧STOP/reason-only回放实际输出纠正instruction，scope/strategyRevision保持、恢复0→1。唯一新GPU0/Qwen正式回合38响应、111MPC全成功、2.985米，seq37曝光21.0秒/响应21.2秒时机身前缘沿门法向尚差0.743/0.727米；未进门，末态停稳。Qwen三次接受同一D13/D5，两次相向性失败、一次缺sameOpening；Monitor审核上限NEEDS_USER。continue/反馈事件均0，因此纠正闭环尚未验证。
- 新轮初始英文为`Go through the open doorway on the right`，旧轮为`Pass through the doorway on the right that is fully open.`，并非严格冻结文本单变量对照。单session38帧只首帧reset；无跨纠正边界证据。非足部净法向力0不证明无碰撞；估计MPC最小净距9.922厘米、stop reserve1.440厘米不是真实全局障碍净距。无误完成/运行错/人工干预。视频、RGB-D、模型原文、两评估/results保留，本任务服务/tmux结束，GPU0释放。

## 前序有序实施与新证据

- 完整原生RTX深度848×496、87°×58.07°；RGB640×360/69°×42.27°不变。848×496为仿真栅格，非硬件流模式；两光心理想共点，真实外参未验证。原生深度供估计/地图，按K/T投影的RGB-grid深度供语义；同一RobotState/epoch和完整地图快照用于控制。
- 深度首轮115.38s/2.105m/NEEDS_USER；新增视场历史地面被当前floor漂移重新判障。现按当帧原始点保存SUPPORT/OCCUPIED，保持6cm阈值、射线清空/重观测和unknown；32帧同输入回放不丢真实高点，未修复全部里程计漂移。
- 六组停步标定只使用sensor状态，残差加raw IMU/本体速度计算的采样间覆盖项。独立实际记录覆盖六标定、旧trash记录及新六停车有效≤2s窗口；时间常数并非每组更准。MPC当前及预测节点增加停车约束，保留原初始/延迟/身体硬门槛；一般部分制动及未来状态误差未得到保证。
- 新基线3.712m/133次、10cm3.835m/139次、居中2.996m/122次、相机后移5cm1.003m/52次生效MPC均为LightNav；四轮全NEEDS_USER/无真实进门/无误完成。前三者仅居中出现一次stopreserve不足；五恢复候选也被拒。没有成功执行脱离，不将CPU回放记成功。
- 后移30cm使相机被自身机壳挡住、地面不可测，0MPC且运行ERROR。已修复Monitor claim/pause覆盖ERROR及启动只看tracking不看地图的问题。后移5cm实测仍有自机遮挡，未隐藏机身/改变高度/renderer。
- 新实际停步最大位移约4.73/5.32/5.03cm；对应原trash最小净距约18.5/11.8/172.8cm，不能称重现并解决旧不足1cm狭窄末态。零非足净法向力仅限该observer，非全局无碰撞证明。
- 新证据根`.local/outputs/native_depth_sequence_20260916/`，每轮保留两evaluator/results/源快照。GPU0/Qwen直连、360/30/3、无人工干预；服务结束、其他项目GPU任务未动。

## 当前输入与设计

- `RobotState`只有传感器估计：stamp_s、epoch、odom/base_link、SE3、机身六维twist、投影平面pose/twist、tracking和quality。局部位置/偏航原点任意，重力方向从初始静止IMU测量获得。首帧initializing且velocity_estimated=false不能证明停稳。
- 当前source=depth_imu_proprioception：每5ms原始IMU积分旋转，已有A2 velocity aux按训练scale2解码为aux[:3]/2 m/s提供平移先验，深度点到平面配准只修正实际法向可观测方向。弱约束方向保留本体预测并在quality公开，不假装完整视觉可观测。Open3D用于点云/法向，旧hybrid路径移除；不是实机验证的VIO/SLAM，无闭环。失效锁存lost，无指令积分/真值回填/自动跳帧，不伪造协方差。
- MuJoCo只在adapter/plant生成理想RGB-D、IMU和编码器。旧A2 policy仍为46×32 term-major/12动作/6D velocity+base-force aux；低层观测改用编码器、陀螺仪和估计重力，checkpoint不变。仿真在default-joint PD稳定后用最后0.5s IMU初始化重力。
- 相机固定标定外参+曝光时同一RobotState用于几何；MPC/Monitor/Qwen使用同一epoch。地图从RGB-D分割可见平面选择重力对齐的支撑面，不使用世界z=0；4cm体素区分占用/射线观测自由/未知。
- 通门必须有相向侧面、纵向支持重叠、完整机身自由走廊和远端两侧完整包络扩展；实际估计轨迹经过入口并清空远端。旧“两点相距0.7m即建portal”不可再用。候梯最终几何来自当前语义复核RGB-D，旧anchor只供接近。
- 完成停车需多个不同采样时间的平移/偏航低速估计。−80N后拉、auxFx≤−.6连续2tick、requested vx≥.2、HUMAN_PAUSED锁存/仅人工恢复保留；历史有干预回合已实际触发并执行恢复，本次对照没有施力/人工暂停，暂停不等于身体瞬间静止。
- Qwen管进展/阶段选择/消歧/策略，只有实际工具提议能启动导航。continue_navigation完整指令原样传递，同目标保留session；select_stage切换执行目标，旧未证实阶段DEFERRED不授予完成。2m内可见同帧深度提示仍触发证据检查；已认证入口/停点1m内可由Qwen显式follow_measured_target，具体视角和recover_clearance同样由Qwen选择。Monitor只核验原完成条件，物理控制约束不变。输入不同曝光至少.25传感器秒，保留真实间隔/MPC采用frame与sequence。
- search_repeated报告共享odom空间回访（.3m/30°内，走2m且离开.8m）或旋转一圈且≥8s；不证明目标不存在。最多3次恢复；任务360秒预算人工恢复不重置。控制ERROR直接终止；有完整记录的失败仍执行两个evaluator并保留错误退出码。
- 在线API只给robot_state/odom；独立`ground_truth.jsonl`供两项evaluator按时刻离线关联，不能将odom直接与场景XML当真值比。旧真值API/fixture已删除，旧证据使用其源码快照。
- YOLO词表仍为任务配置，S为帧内mask，B为局部边界证据，不等于实体ID；不添加目标外观模板，不放宽机身/碰撞条件。

## 最新证据和限制

- 19:52 HKT接收侧任务证据`.local/outputs/stop_clearance_diagnosis_20260916/{REPORT.md,clearances.json,brake_dynamics.json,paired_mpc.json,paired_language.json,summary.json}`，仅更新文档/memory。它针对全开门回合，不适用于更早多房间回合254点自占用。RGB-D黑色垃圾桶→历史占用格关联已确认；在线仍只用传感器局部odom地图，原生几何/位姿仅离线核对。
- 同一三圆包络首次blocked：实际位姿+原生垃圾桶XY凸投影净距3.994cm，估计+未膨胀地图4.487cm，体素膨胀后1.659cm，延迟前缀1.407cm，对要求1.5cm。局部相对差约5mm，不能以整段20cm定位RMSE直接归因，也不是精确关节网格距离/全程地图准确性证明。
- 47.80s首次零指令，用真实初始位姿+速度隔离现有.15s停车模型：1s预测3.279cm/实际.864cm，2s预测3.277cm/实际.635cm。支持实际停步消耗余量超预测→初始不可行拒绝恢复；低层响应、步态转换、回摆贡献未分离。51.6s LightNav seq40左转约79°预测已接受但MPC未执行，不是实际转角。
- 隔离对照：48相同MPC输入，仅期望净距3/6/10cm，各47/48可解，10cm使部分输入提前减速/转向；不可行末态仍不可解。实际LightNav两会话相同38JPEG，居中语言让38帧预测全变但未稳定增大空间。均无替代轨迹闭环执行，不计新成功。相机后移约半秒可见性增益仅用户转交的无机身遮挡投影估算；原生深度FOV分流尚未实施。先停车模型/余量，再几何恢复；其余变量分别比较。

- 最新全开门直接任务：`.local/outputs/isaac_hospital_open_room_20260916_01/`，原始`isaac_sim_20260916_171620`。上一门SM_Door_02b_51只开约10°，旁边SM_Door_02b2开约90°；保持原始资产。从上一终点xy19.1361/3.9046/yaw10.1156°新初始化，地图/估计/模型会话全新，不能称连续恢复或旧地图已修复。只新增scenario，控制/导航逻辑不变。
- 新任务103.254s，0/1 NEEDS_USER；实际路径3.278m、未过目标门；LightNav42响应39接受/0STOP/visible全false，146生效MPC全LightNav+3blocked。首blocked净距.014m<.015m，裸三圆内占用点0（区别于旧254点）；停止后−.017m，右转换视角0位移/转角。当时二维MPC日志不足以关联trashcan_6；19:52后续RGB-D离线诊断已确认垃圾桶来源，仍不能把膨胀净距为负等同实际碰撞。Qwen最后明确判断另处双开门not_target，三次同帧提交后求助。9请求含一次clause覆盖计划拒绝/纠正，无HTTP/工具格式/运行错、无误完成/人工干预。228帧位置RMSE20.42cm/最大22.86cm/末端22.01cm，偏航RMSE.017°。两项评估/results/视频齐全，服务结束。

- 原生Isaac实验：`.local/outputs/isaac_hospital_qwen_20260916_01/`，原始`isaac_sim_20260916_165411`。现有IsaacSim5.1/IsaacLab0.54.4在独立plant进程运行原生PhysX/RTX，通过Unix socket只传RGB-D、IMU、关节状态；sim环境复用估计器/LightNav/MPC/Monitor。Hospital617文件/1,070,394,700B直连下载完整，原生门/碰撞/地面保留；GPU0实际UUID。入口`prototype/isaac/run_experiment.py`，文档`prototype/isaac/README.md`。
- 一次正式142.128s测试完整成功0/1、NEEDS_USER。6141个物理样本、实际路径6.427m；44.775s完整过第一门，最小截面横向净距7.61cm，未进入目标房间。Monitor0阶段完成、0误完成；7Qwen请求无工具/HTTP错，69LightNav/65接受/3STOP/visible全false，236/236生效MPC采用LightNav、另4blocked；无后拉/HUMAN_PAUSED/人工恢复。真实终点停稳，非足部净法向力0但contact仅合力，无对象/点/摩擦，不能证明零碰撞。
- 当前缺口：近目标取证依赖LightNav visible+指向，本轮无measurement_requested/portal，实际出门后第一阶段仍ACTIVE。Qwen把出门+横穿走廊合为pass_doorway，且将进门后停下另拆approach房间中央（该阶段未执行）。首STOP在首输入20.6sim秒后、已过门，并晚于首次mpc_blocked。不能称复现原“未出门STOP”，也不能隔离渲染因果，因场景/物理/PD/高度/指令同时变化。
- 首MPC blocked55.96s报净距−.326m；12463占用点中254在三圆内，最近距后圆心2.77mm。日志只有XY，未保存原像素/体素高度来源，不能等同实际撞门或确定是地面/地图/里程计哪项。308配对状态一次首帧对齐位置RMSE13.52cm/最大20.97cm/末端19.05cm，偏航RMSE.101°，无tracking失效。无新完成判据放宽、模板、在线真值或扩批。

- 无干预对照：`.local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01/`，原始`sim_20260916_152935`。record_monitor/runner新增`--interaction none`，默认force_resume保留，40导航/控制/传感器文件逐字节同原快照。实际外力0、HUMAN_PAUSED/问答/人工恢复0，36.472s后NEEDS_USER，真实0/1、无误完成/非支撑接触。模型24响应全接受、2STOP、1session，81MPC均采用LightNav；实际路径1.53836m。
- 新组首STOP seq22曝光10.90/响应11.30s，距首输入7.48s；此前23张不同帧均连续.30–.40s，没有人工暂停/长输入间隔。前缘距入口仍1.083m。seq20末点1.450m→21近原地转向末点.007m→22全零STOP，visible全false。Qwen随后continue_navigation，下一次seq23又STOP；两次STOP之间4s为核验造成，不能解释第一次STOP。
- 原组首STOP距首输入13.18s、前缘距入口1.409m，恢复后仍有14个非STOP预测才停止。两组均可提前STOP，但Qwen指令Move through/Pass through不同，记录初始xy差4mm，seq4在4.82s已分歧，早于原组7.06s实际受力；非冻结前缀、各1例，不能排除暂停对原组有影响，更不能推断内部虚构里程。对照证据`.local/outputs/no_interaction_control_20260916/comparison.json`。
- 新组最终选门框与灰白墙面，三次不满足相向侧面，未生成portal；不同于原组正确相向门框但体积空间不足。新组无运行错误/工具错误，两项评估和results完整。不要混淆相同NEEDS_USER标签下的失败原因。
- 当前局部自主权回合：`.local/outputs/navigation_framework/local_autonomy_qwen_p05_20260916_02/`，原始`sim_20260916_145718`。41.541s/6Qwen调用，LightNav29响应28接受1丢弃、106/106生效MPC采用26帧模型轨迹；1session跨人工暂停。实际路径1.90065m、未完整出门、最终仍差远门框2.90180m、正非支撑接触0。visible全false，stop一次提前触发核验，无portal/阶段完成。
- 本次S1/D5与S2/D4确为东西门框内侧（离线x4.2/2.0），拟合宽2.2086m对真实2.2m；不是旧同侧误配。41通路+2扩展查询42unknown/1occupied；具体触发体素未保存，不能断言物理堵塞或occupied归属。Qwen两次接受同一17.10s观察，第三次重选同mask，三审核预算后NEEDS_USER；未实际选择继续导航/新视角。
- 实际−80N持续100个50Hz采样，施力请求至制动观测.70sim秒；撤力保持锁存，问答后人工恢复原阶段/session。暂停最大估计位移.23492m。Qwen错误声称噪声误判/非真实受力，问答语义失败；没有工具校验/HTTP错误。正式回合记录器exit2、runnerexit0，评估完成。
- 最新263帧/26.2sim秒估计位置RMSE9.57cm、峰值21.22cm、末端3.51cm，平面速度RMSE8.31cm/s；含后拉且轨迹不同，不与旧19s试验直接配对。近处精确控制、重复搜索事件及阶段自动完成本轮未触发；不能记为动态验证通过。
- Qwen纠正：真实Pi复放非对象arguments历史重现mapping HTTP400/零token；改为文字反馈和生产纠正路径分别返回200/schema合法。仅3次短调用、不执行提议，仍最多2次尝试。首次原始SSE未保存，不能定位最初scalar来源；部署模板版本不可见。证据`.local/outputs/qwen_tool_fix_20260916/README.md`。
- 里程计：63归档RGBD帧配对位置RMSE8.15→4.02cm；新GPU0 19秒191帧/3801IMU配对26.32→4.77cm，静止估计速度中位4.54→1.28cm/s。新方法在线、旧方法同原始序列回放，真值只用于离线一次首帧对齐后评分。非完整导航/非硬件，弱几何方向、陀螺偏置和长期漂移未解决。证据`.local/outputs/odometry_fix_20260916/`，报告`prototype/NAVIGATION_REPAIRS_20260916.md`。
- LightNav此前调查：上一完整回合56次推理、197次MPC均metric_turn，0次采用LightNav轨迹，语义输入确为Go through the doorway。不能把此失败当作连续局部导航能力证据；未统计旧72轮全部分支。SysNav室内是经典覆盖/TSP且用360RGB+LiDAR/房间图，当前只有局部RGBD地图，不能宣称复现其房间级能力。四类传感器输入对照仍仅为建议，未自动扩批。
- 上一次完整范围导航尝试`.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/`：10请求、4转向、无前进/portal/阶段完成；89.475s后NEEDS_USER。seq165字符串arguments→本地拒绝→非法历史纠正HTTP400；力交互未触发，恢复0次。两项evaluator及results.json在同一记录上补齐，原runner失败保留。
- 该旧方法完整记录178帧/17.68s：估计位置RMSE0.25056m，峰值0.34880m，末端0.34844m，偏航末端−0.953°。只有63帧归档可供配对回放，不能把原178帧误差与新63帧误差直接配对。原始`sim_20260916_131404`；真实路径0.48188m、未完整出门、正非支撑接触0。
- 此次执行前修正：observe普通规划/取图不再要求先停稳，完成仍须连续低速；completeIfReady先取当前状态；导航图像归档去掉已删除变量；施力记录器按已发时程加一控制tick等撤力，独立真值另作核验。

- `.local/outputs/sensor_state_rebuild_20260916/`：原同侧墙/门扇完美位姿反例被明确拒绝；只属旧数据路径验证，不是新任务成功。
- 有界GPU0功能运行：真实sensor→geometry/map→生产MPC，局部0.25m请求，26/26求解成功，PAUSED/advance_checkpoint；16.61s墙钟。离线25帧约2.4s的估计位置RMSE5.025cm、末端0.889cm，真实路径0.3183m、末端速度0.0111m/s、正非支撑接触0。原始`.local/outputs/sim_20260916_114439/`。
- 两项evaluator在该有界记录上完成；未调用LightNav/Qwen、不计完整导航成绩。该有界记录不替代后续失败的正式尝试，也不证明完整通门/候梯、力交互或实机能力。
- 当前通路模型支持近似平行的相向侧表面，延续侧墙/遮挡/未知地面可能无法认证。地图局部平地、底部地面上6cm至基座上25cm体积假设；不是任意地形/门洞拓扑保证。

## 固定环境与权限

- 仓库`/home/baoquanc/workspace/GuidePup`，分支`codex/a2-lightnav-prototype`；迁移标记`20260915_215130_HKT`。既有未提交迁移工作保留，未commit/push。
- Owner指定物理GPU0；LIGHTNAV_GPU=0，经UUID匹配CUDA/EGL。本机UUID`GPU-f593e489-014b-eed5-4331-b01447615b6e`，EGL2，RTX A6000，driver580.173.02。不可复制旧机器UUID、重置驱动或用renderer fallback。
- Qwen实验室`http://vic.precognition.team:8888/v1`，精确`Qwen3.6-35B-A3B-FP8/`末尾斜杠，无鉴权，thinking off/2048；Pi0.85.1、Node22.23.2。Gemini按Owner要求暂缓，不覆盖Qwen profile。
- `.local`为本机目录。安装/公开模型/服务用服务器直连，清除双写proxy变量并NO_PROXY=*。服务器全局模式已改split，预配Codex专用代理未动；不要重复修改。
- Python3.11.7、Torch2.10+cu128、MuJoCo3.6、Open3D CPU0.19；D435i nominal640×360、69°×42.2726°、base[.30,0,.10]、俯5°、目标10Hz、有效深度.2–8m。传感器理想仿真，实机尚未接入。
- MPC三圆centers x=−.3/0/.3、r=.3，4cm占用体素，15×.15s，上限.3m/s/.6rad/s；只前进和转向。任务360s/决策30s/恢复最多3次。无训练、硬件或扩批授权。

## 历史结果路由

- 新机器恢复和上一完整Qwen p05：`prototype/NAVIGATION_MIGRATION_20260916.md`，`.local/outputs/navigation_framework/migration_qwen_20260915_server_01/`。完整成功0/1、NEEDS_USER；旧Monitor同侧配对导致阶段0误完成，真实机身距远门框仍差0.893m。−80N/问答/人工恢复执行，正非支撑接触0；不能被此次离线反例或有界运行改写。
- 2026-09-15迁移初期直连受阻记录：`prototype/NAVIGATION_MIGRATION_20260915.md`；后续环境恢复成功，不重复报告旧阻塞。
- 旧72轮：`prototype/NAVIGATION_YOLO.md`；压力/80帧问题`prototype/NAVIGATION_STRESS.md`，v2 `prototype/NAVIGATION_V2.md`；此前Monitor历史`history-monitor-v1.md`、`history-before-monitor.md`。旧GPU初始化阻塞仍是历史，未宣称解释驱动根因。

## Presentation assets

2026-09-22: Prepared English-only presentation assets for the explicitly selected 2026-09-18 Hospital video. Display name: Navigation Model. Three 16:9 figures (measured trajectory, current architecture, actual experiment flow), each 3840x2160 PNG/SVG/PDF; combined PDF, English speaker notes, physical trace CSV, unchanged selected video and renderer packaged at `.local/outputs/mppi_no_task_budget_20260918_01/presentation_en_20260922.zip`. Source directory `presentation_en_20260922/` under that experiment root. Physical trace retains9.420m/source exit/target not crossed/26cm before entry; Qwen false completion explicitly distinguished. WBC learned policy/PD and nominal50Hz/200Hz verified against current source and episode snapshot. No code changes, new simulation, model calls, GPU services or new success claims.
