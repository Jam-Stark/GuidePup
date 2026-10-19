# Done

- Astra medium标准档单轮已结束（2026-10-16 01:03 HKT）：28次实际请求确认gpt-6-astra/medium/default并禁用fast；自然complete、真实13.370m且停稳、无导航运行错。终点仍是G标牌双门，真实建筑出口身份未证实，physical_success=null；动态估计外力STOP及planner恢复已取证。两评估/视频/报告齐全，GPU4释放、GPU3模型保留，不追加。详见prototype/NAVIGATION_ASTRA_MEDIUM_STANDARD_20261016.md及memory/navigation-prototype/description.md。

- 持续至GPT complete已结束（2026-10-16 00:27 HKT）：默认goal保留闭门、Find the building exit and stop near it.；04由GPT自然complete、实际35.615m并停稳、无导航运行错；停在带EXIT牌的G双门，非A（末距A15.483m），真实建筑出口身份未证实，physical_success=null，不计已验证成功。两评估/完整19:35.7视频/英文10x回放/轨迹及4过程账本齐全；仿真和planner关闭/GPU4释放，GPU3/PID2454462/8050保留。不自动追加。入口：prototype/NAVIGATION_PLANNER_COMPLETE_20261015.md；细节memory/navigation-prototype/description.md。

- 站立与目标交接单轮已结束（2026-10-15 22:12 UTC+8）：显式站立、同原JPEG目标框→模型首指向→GPT确认后新鲜连续预测已实现，Monitor只读。旧图2次真实GPT拒绝错误左侧对象；协议/WBC检查及第4次短物理站立±30°通过，前3失败保留并修复观察低速步态deadband与近角停步，速度/3°容差/原动作时间不改。唯一新完整任务NEEDS_USER/error=null/无误完成/真实0of1：走1.257m仍E2附近室内、末尾停稳；2次实转−61.395/−90.321°，14GPT仅3接受，导航输入/目标探测/MPPI均0。新增0.10m XY新鲜度否决因未校正WBC积分漂移误丢10条（首估25cm实际1.25cm），另1条因force authority变化；不能称本轮验证了目标交接或模型改善。事后已移除不可靠XY硬否决，XY明确未验证参考，保留IMU3°/epoch/tracking/generation；10记录窄检查和yaw/lost/epoch拒绝通过，无额外模型/仿真，修正后尚无完整闭环。125.28s学习力60.016N触发，峰66.613N，非足净法向全0且无注入力；原因/静止标定未定，不能称碰撞。GPT见持续估计求助，0恢复；停后全零且无步态，已站立故非动态紧急停车验证。两评估/root results/140.467s视频/英文图/报告齐全，5尾141.92s相等；报告prototype/NAVIGATION_TARGET_HANDOFF_20261015.md，证据.local/outputs/campus_target_handoff_20261015_01。任务服务关闭/GPU4释放，GPU3/PID2454462/8050保留。不得计离线/短验证为新成功；无追加完整/训练/硬件/Git。

- 自然建筑出口单轮已结束（2026-10-15 21:03 HKT）：按Owner撤去树篱/地垫等细节，默认goal为“Find the building exit and go outside.”；仅task.goal变化，相机/起点/代码/控制参数不变，未向在线给图/地图/目标坐标。此前细节指令回合因Owner改任务SIGINT取消，保留88.467s部分视频，不计完成或运行故障。新完整任务NEEDS_USER/error=null/GPT未误完成，真实0/1：15.241m仍室内、末尾未停稳。末右90°请求实际−6.80°/估计−6.28°，不是仅里程计误报；hold352零tick仍移.481m。171MPPI全实际执行、60导航输入3STOP/单会话、12GPT/8观察/3纠错cycle（仅2次导航继续）；最大GPT21.717s。估计峰47.756N未触发外力STOP，base_link非足净法向峰2901.78N对端未知，不等于学习施加力。两评估/results/176.8s完整视频/英文带底图轨迹/报告已保存，五类尾178.28s完整。报告 `prototype/NAVIGATION_BUILDING_EXIT_20261015.md`，证据 `.local/outputs/campus_building_exit_20261015_01`。GPU4释放，GPU3/PID2454462/8050保留；任务已结束，无自动追加/训练/硬件/Git。新目标为室外，不以旧A室内距离验收；描述变更未解决执行失败，也不作独立因果结论。

- 无限次数单轮已结束（2026-10-15 20:43 HKT）：自动外力STOP允许planner恢复、人工brake仍human-only；skill/GPT思考时长/纠错上限已取消，单动作物理范围保留。唯一完整回合STOP_NOT_CONFIRMED，GPT误认走廊侧玻璃而误完成，真实0/1，走12.819m、末距原A17.242m且未停稳，无运行异常；8观察/同cycle最多5、12GPT、2纠错真实运动、143MPPI全执行、51导航输入3STOP/1会话。外力估计峰48.598N未触发，真实自动STOP恢复仍未覆盖。两评估/results/完整视频/英文图已保存，见 `prototype/NAVIGATION_UNLIMITED_RECOVERY_20261015.md`；源报告 `prototype/NAVIGATION_STOP_AUTHORITY_20261015.md` 与只读GPT复盘证据保留。旧任务服务已退出，GPU3模型保留。

- 2026-10-15 20:11 HKT：外力制动职责接入完成：原aux/scale换算、水平60N/2独立样本、plant锁存并关闭force_walking/步态，覆盖导航/观察/等待；陈旧结果丢弃、技能部分证据和planner报告、人权限。旧记录精确5065tick回放与合成权限验证通过。唯一新任务NEEDS_USER/无异常/无误完成，真实11.713m未到A/未停稳；127MPPI实际执行、45输入1STOP、4观察/6GPT，估计峰42.409N未触发。接触observer有强信号但不等于学习施加力；不能报碰撞STOP已验证/已解决。两评估/results/视频93.933s/英文轨迹与力图保存，尾95.4s完整；GPU4释放、GPU3模型保留，无追加/训练/硬件/Git。报告 `prototype/NAVIGATION_FORCE_BRAKE_20261015.md`。

- 2026-10-15 19:44 HKT：观察skills端到端落地，GPT/Pi-Qwen共用schema和执行器；两次真实模型提议通过。01在2次实转后首张导航输入前ERROR，持续WS reader修复及45s空闲验证通过；02为NEEDS_USER/无运行异常/无误完成，实际12.218m、未到A/未停稳，4实转（末次90°请求仅实转4.98°、估计15.27°），53模型输入/1STOP/150MPPI实际执行。1/3恢复，1386.58N非足净接触峰/对端未知；零命令仍物理移动未修。全部证据/两评估/results/英文轨迹和联系图、完整视频及memory保存；GPU3模型保留、GPU4释放，不追加/训练/硬件/Git。报告 `prototype/NAVIGATION_OBSERVATION_SKILLS_20261015.md`。

- 2026-10-15 18:54 HKT：按Owner新授权仅跑1轮官方指令规则/STOP证据新版，3 STOP/263输入/1会话/1初reset，2恢复第二真实27.002m；832/832 MPPI执行。真实0/1：错过大厅转入，西端另门误完成；在线STOP_NOT_CONFIRMED且物理未停稳，无导航异常。最后842零命令tick后仍移2.754m，base_link接触峰755.67N，对端未知；五类记录尾120.56s完整。两评估、results、视频/轨迹/源快照/memory完成；GPU4释放，GPU3保留。报告 `prototype/NAVIGATION_CAMPUS_PROMPT_RULES_20261015.md`，不追加实验。

- 2026-10-15 18:40 HKT：完成官方LightNav规则共享加载（实际GPT与Pi）、STOP来源帧指向及逐动作执行/估计进展接入。4归档STOP核对与2次实际GPT决策通过，生成简短接近目标指令，无误完成；未执行提议，完整导航0轮、LightNav调用0。验证证据与限制见 `prototype/NAVIGATION_PLANNER_INSTRUCTIONS_20261015.md`。

- 2026-10-15 18:19 HKT：HEAD RGB简化/GPT-6.1 Sol medium fast/CUDA MPPI轨迹跟踪已落地，默认prototype/run.sh接新入口。Owner选新任务E2电梯门前→A玻璃入口（室内附近停下），spawn[12.51,-2.45,.55,180]仅初始化；无深度/占用图/在线真值，IMU+WBC辅助速度短时推算，语义参考只读。独立新完整导航1轮NEEDS_USER/真实未到达：走13.389m，末物理位置距A13.290m，已停稳；GPT无误完成/无抛出运行错误。112导航模型输入、1会话1初reset、4次STOP，3次恢复仅首7条控制/6.1cm，后两次0非零执行。298GPU MPPI成功且实际执行；中位GPU3.05ms/总4.58ms，速度上限.50m/s、真实峰值.575m/s（仿真）。FR_thigh非足部净接触峰1266.9N，对端未知；不可称无碰撞。原回合末GT缺.205s/接触缺.035s，保留未补造，已修flush-before-close/等正常退出，另静止检查3.52s在线/GT/接触完整相等，非第二轮导航。模型带图决策验证通过；HEAD参数768×272/132°/15Hz是已有仿真参数非实机已标定。英文标志物图/任务图/原生参照照片、新完整视频/物理轨迹/两项新评价/root results及源快照齐全。输出.local/outputs/campus_rgb_e2_to_entrance_20261015_01；配置证据.local/outputs/campus_reframe_20261015_01；入口prototype/navigation/README.md。所有本任务仿真/检查服务结束，GPU7释放；GPU3/PID2454462/8050保留，不关闭模型，GPU4他项目未动。不追加完整试验/训练/硬件/Git。

- 2026-10-15 18:09 HKT：Owner批准HEAD RGB无深度＋导航模型RGB避障/GPU MPPI仅轨迹跟踪；Codex GPT-6.1 Sol/medium/fast真实带图决策15.35秒通过，独立CUDA graph MPPI512×24中位GPU2.97ms/总3.53ms，速度上限.30→.50m/s（非实走验证）。新默认任务Owner选E2电梯门前→A玻璃入口室内附近；spawn[12.51,-2.45,.55,180]只用于初始化，在线无地图/目标坐标。默认prototype/run.sh转navigation/run.py，camera默认HEAD768×272/132°/15Hz；新proprio/IMU短时推算无depth或真值修正，语义参考只读。原stop model仅.30标定，不用于无深度模式的净距保证。两个HEAD静止预览通过，最新E2起点31帧/101WBC tick/零非零命令/0完整导航。地图标志物及英文原生参考图已生成。当前唯一新完整任务刚提交独立tmux guidepup-rgb-e2-a-20261015，输出.local/outputs/campus_rgb_e2_to_entrance_20261015_01；结果PENDING，不计成功。GPU4他项目占用，用7；GPU3/PID2454462/8050保留不关闭。

- 2026-10-15 17:22 HKT：校园默认任务真实成功0；初始规划失败后修复原文覆盖，02/03实际8.277/9.328m均Qwen误完成，03仍横向差4.861m。04走0.597m后Owner取消，不再重试。50.1秒英文未完成进度视频已渲染，所有失败/两评估保留。GPU4仿真及8768/8770关闭；Owner要求GPU3 LightNav保留，已在tmux guidepup-lightnav-gpu3、8050/PID2454462常驻约8980MiB。未来显式--reuse-lightnav复用且不关闭外部模型；未新增导航验证。报告prototype/NAVIGATION_CAMPUS_PROGRESS_20261015.md。

- 2026-10-15 16:37 HKT：按Owner用学校MF-R1替换默认Isaac场景，选择一层到中间2号电梯门前，指定GPU7。保留89文件原包/全部两层USD与材质碰撞，准备独立引用层停用嵌入PhysicsScene，默认scenario/preview-only/8768与SIM_URL接通；新增原等待约定的离线电梯目标评分。GPU7原生48.12秒预览完成，RGB-D92.11%、tracking/地图就绪，实际完整任务0/模型0。未接电梯升降或跨层任务；几何只放置与离线。预览/报告/证据/memory保存，自建服务/tmux/8768结束GPU7释放，其他项目8766/GPU0/6未动，无训练硬件Git。详见prototype/NAVIGATION_CAMPUS_MF_R1_20261015.md。

- 2026-09-22: Prepared English-only presentation assets for the explicitly selected 2026-09-18 Hospital video. Display name: Navigation Model. Three 16:9 figures (measured trajectory, current architecture, actual experiment flow), each 3840x2160 PNG/SVG/PDF; combined PDF, English speaker notes, physical trace CSV, unchanged selected video and renderer packaged at `.local/outputs/mppi_no_task_budget_20260918_01/presentation_en_20260922.zip`. Source directory `presentation_en_20260922/` under that experiment root. Physical trace retains9.420m/source exit/target not crossed/26cm before entry; Qwen false completion explicitly distinguished. WBC learned policy/PD and nominal50Hz/200Hz verified against current source and episode snapshot. No code changes, new simulation, model calls, GPU services or new success claims.

- 2026-09-20 18:14 HKT：GRScenes全236文件76.616GB直连镜像下载完成，医院ABY8原生资产按厘米缩放/原碰撞接入，GPU7唯一实际创建任务175.265秒、1.212m未出源门/未入目标、最终停稳；Qwen完成误报，真实0/1。54LightNav返回52接受，唯一有效STOPseq53距源门前缘仍63.77cm；85原生成功1候选受阻、84下发匹配，左25°恢复真实25.507°。无任务内ERROR；初始化材质缺陷和03任务创建前地面估计错误单列。预览/模型启动等待已改为暂停仿真时钟，04的hold/release均1.74s。两评估/results/视频与原始证据完整，服务/端口/tmux关闭，GPU7释放，不追加。报告`prototype/NAVIGATION_GRSCENES_20260920.md`。

- 2026-09-18 11:51 HKT：按Owner取消全任务时间预算并只重跑一次。移除Pi timer/恢复重注册/CLI与记录器全回合超时，入口同步；保留决策30秒/3恢复及全部传感器/MPPI约束。新任务575.443秒跨过旧360点；真实9.420m出源门但仍在目标门外，前缘差25.95cm、停稳；Qwen在只读证据未证实下声明完成，false_completion=true、真实0/1。3Qwen调用/1动作/0恢复、184LightNav唯一STOP；415原生成功414下发0blocked，无运行ERROR。两评估/results、报告/视频/完整证据/memory保存，本任务服务/端口/tmux关闭GPU0释放，无追加实验/训练/硬件/Git。见prototype/NAVIGATION_NO_TASK_BUDGET_20260918.md。

- 2026-09-17 19:32 HKT：按Owner完成停车模型服务MPPI规划的边界修正，追踪Python/原生critic/滤波后检查。保留原几何/自由度/系数，原始六例复算后不缩小7.033cm；共享候选规则允许固定2.4s、零命令基线缺额上限的恢复。旧4失败记录各512候选真实离线求解通过（非导航成功）。仅一次GPU0完整任务：0/1，7.50724m、完整出源门未入目标、终止未停稳；259原生成功255下发0blocked，指令与最终检查一致，restoration未触发。Qwen11调用5动作实传，左15/右25分别实际15.41°/18.12°，一次继续0非零；先LIMIT后地面缺失ERROR均公开。两评估/results、视频/全输入/原生候选/物理与接触记录/memory保存；服务、端口、GPU0与tmux关闭，无追加实验/训练/硬件/Git。报告prototype/NAVIGATION_STOP_PLANNING_20260917.md。

- 2026-09-17 16:27 HKT：Owner批准全部架构结论并替换MPPI、跑一次完整任务。移除语义几何暂停/阶段证书完成门槛/重复策略否决；Qwen原文指令、显式完成声明、全任务3恢复，观测只读。官方Nav2 Jazzy1.3.13隔离直连安装/原生编译及旧记录CPU实际执行通过，保留三圆/硬净距/经验停车模型；移除旧CasADi和几何恢复执行路径。启动相对配置路径错误已修复；随后GPU0冲突只取消本任务启动，均未提交任务。唯一GPU1正式回合142.372s、0.969239m、真实0/1 LIMIT_REACHED，未整机出源门/未进目标，停稳；113原生求解成功执行全LightNav、4停车包络求解前拒绝，后三次恢复无非零命令。35模型返回/31接受/0STOP、单会话；4动作原文按ID实传。无正式运行错误、无Qwen完成声明或误完成；非足净法向力0仅为受限observer，定位RMSE1.412cm仅离线。两评估/results/视频/原始RGB-D/模型/控制/物理/接触齐全；服务、端口、GPU1、tmux结束。详见prototype/NAVIGATION_MPPI_20260917.md；不追加任务、无训练硬件。

- 2026-09-17 15:14 HKT：按Owner要求原样保存LightNav-0能力边界导出MD至prototype/discussions，并完成模型身份门控核对。确认实际官方HF权重LightOriginsHQ/LightNav-0、Qwen3-VL-4B/RVQ/10步与附件同一模型，证据含下载/启动路径及真实加载log。官方论文/协议/部署/MPPI/Collision Monitor资料核对后，讨论语义Monitor退出监督、只读证据与STOP/执行受阻事件，以及局部避障替换角色和地图误差限制。独立讨论文档已保存；未实施架构、下载资产、安装、启动服务、模型推理或导航，旧实验成绩不变。
- 2026-09-17 02:08 HKT：按Owner修复Qwen/Monitor决策边界并追加一次完整测试。Qwen显式阶段/策略/实际instruction，Monitor结构化证据、DEFERRED独立于完成、不再自动推进或补写语义；精确/脱离也显式选择，三审核失败交Qwen重判。原完成条件逐字未变，构建+CPU旧STOP派发回放+2实际Pi验证通过。唯一正式原房间→全开门任务226.159s/2.364m，0/1 LIMIT_REACHED；5动作原文与模型输入按动作ID吻合，单session保持，0STOP。141MPC/137生效（125LightNav/12turn）、2净距/停车拒绝+2迭代上限；后缘仍差.2171m出第一门，无误完成。最后Qwen提议因3恢复预算未执行。单stage跨两门离线映射记未映射，同记录重算不改结果。两评估/视频/RGB-D/原文/物理/报告/memory完整，本任务服务/tmux结束、GPU0释放；无追加导航/训练/硬件/Git。`prototype/NAVIGATION_QWEN_AUTHORITY_20260917.md`。

- 2026-09-17 01:32 HKT：按Owner新授权将A2恢复原房间[12.8,3.35,.55,0°]，当前全开门房间终点不变，新scenario/实际预览后仅跑一次GPU0/Qwen。211.318s、5.938米，23.875sim秒完整出第一门/横向截面净距11.635cm，但未进目标房间、0/1 NEEDS_USER；207MPC全成功，70LightNav/唯一seq69STOP，Monitor阶段0未推进。18次visible=true深度提示距离6.54–8.26m未满足2m取证；STOP后3次身份审核拒绝，无continue/反馈触发。plan走廊pass_doorway/进房approach潜在语义问题已记录，未修改运行框架/完成条件。两评估/results/视频/RGB-D/原始模型/物理证据和报告齐全，服务/tmux结束、GPU0释放。无运行错/误完成/人工干预/新训练/硬件/Git/扩批。`prototype/NAVIGATION_ROOM_TO_ROOM_20260917.md`。

- 2026-09-17 00:44 HKT：读取并纳入侧任务49JPEG×4/196实际推理AB证据；原样两次49/49、纠正语义消除后3帧STOP、仅补时间未消除。实现STOP→continue在线证据分级指令反馈及同目标history保持，保留预算和完成条件，新增实际会话/时间日志；构建和旧case编译后CPU回放通过。唯一新GPU0/Qwen全开门回合145.139s、2.985米、38LightNav、111MPC全成功，seq37主动STOP，Qwen同帧3次无效审核而无continue；真实0/1、NEEDS_USER，实际停稳、无误完成/运行错。新反馈闭环未触发，不计验证成功，初始英文与旧轮不同。两评估/results/视频/模型和RGB-D证据齐全，服务及tmux结束、GPU0释放；无第二轮/扩批/训练/硬件/Git。报告`prototype/NAVIGATION_STOP_FEEDBACK_20260917.md`。

- 2026-09-16 21:48 HKT：按Owner新顺序完成完整深度视场、共享地图捕获状态、地面历史分类修复、sensor-only停步标定/余量约束及有界可核验恢复。新深度首轮0/1；修正基线/10cm/居中/后移5cm四轮真实0/4、NEEDS_USER，另后移30cm运行ERROR（自机遮挡无地面）。四有效组生效MPC133/139/122/52全部LightNav；居中一次stopreserve不足，五恢复候选拒绝，无成功脱离、无误完成。六个实际停车有效窗口均在经验包络内，均值并非普遍改善，新轨迹非旧不足1cm末态复现。修复ERROR被Monitor覆盖及重复数值速度误拒；GPU0/Qwen直连，保留360/30/3，无施力/人工暂停/训练/硬件/Git/扩批。两评估/results/报告、原失败与快照保存；本任务服务及tmux已结束。`prototype/NAVIGATION_NATIVE_DEPTH_STOP_20260916.md`。

- 2026-09-16 19:52 HKT：接收Owner转交的stop_clearance_diagnosis侧任务证据，读取报告/数值并核对原始LightNav恢复预测。更新全开门归因：垃圾桶历史RGB-D占用关联确认，首次局部净距差约5mm；真实初始位姿/速度隔离仍见停车模型低估（1s3.28cm预测/.864cm实际）。恢复左转79°已接受但MPC拒绝执行。48冻结MPC与38固定图像语言对照均非闭环成功，相机方案仍未实施。报告/memory已更新，原导航0/1不变；本次无生产代码/默认配置改动、无服务/模型/导航运行。

- 2026-09-16 17:23 HKT：按Owner换到旁边完全敞开的房间，核实原门约10°/目标约90°且保持原资产。从原走廊终点新初始化地图/估计/模型会话，一次GPU0/Qwen直接任务103.254s、0/1 NEEDS_USER，3.278m未进目标门。39接受LightNav无STOP，146生效MPC全LightNav、3次blocked；首净距.014<.015m，后−.017m，裸三圆占用0，区别旧254点。Qwen对另处闭门三次not_target后审核耗尽；无误完成/HTTP/工具格式/运行错或人工干预。两项评估/results/视频/轨迹齐全，报告prototype/NAVIGATION_ISAAC_OPEN_ROOM_20260916.md；服务结束/GPU0释放，无自动重跑或训练/硬件/Git。

- 2026-09-16 17:08 HKT：按Owner切换原生IsaacSim5.1/IsaacLab0.54.4与官方Hospital，617文件1.07GB全部直连。新增sensor-only PhysX/RTX worker+Unix IPC、共享server显式backend、任务scenario与两项离线评估。实际GPU0/Qwen一次多房间142.128s：完整0/1/NEEDS_USER，真实完整出第一门并走6.427m，但无portal认证/阶段完成，236生效MPC全LightNav，4次地图包络重叠blocked，恢复3次用尽。7Qwen请求无工具/HTTP或运行错误；无外力/人工暂停/恢复、无误完成。非足部净法向力0但缺接触对象/点，不能证明无碰撞；位置RMSE13.52cm。两项评估/results/视频/实际轨迹图已保存，报告prototype/NAVIGATION_ISAAC_20260916.md。服务结束GPU0释放，无扩批/训练/硬件/Git；不把新场景变化归因为旧渲染根因。

- 2026-09-16 15:34 HKT：按Owner去掉后拉/人工暂停/问答/恢复，新增记录器--interaction none并运行一次同起点GPU0/Qwen对照。40导航文件同原快照；实际外力0、HUMAN_PAUSED0，首次STOP前连续.30–.40s输入，仍在前缘距入口1.083m时STOP；Qwen继续后再次STOP。36.472s/7调用、81MPC均LightNav、真实0/1/NEEDS_USER，无误完成/非支撑接触/运行错误。两项evaluator/results和对照timeline完成。措辞Move/Pass、seq4干预前分歧限制因果结论；只证明无干预亦可复现。报告prototype/NAVIGATION_NO_INTERACTION_20260916.md，服务结束/GPU0释放，无扩批/训练/硬件/Git。

- 2026-09-16 15:04 HKT：按Owner批准实施Qwen阶段/策略、LightNav连续局部轨迹、Monitor近处几何接管及独立完成；同scope保留历史，记录真实输入节奏/MPC采用，增加重复搜索事件。一次GPU0/Qwen p05用106/106 MPC实际执行模型轨迹，29响应、6Qwen调用，41.541s后NEEDS_USER，真实0/1、无误完成/非支撑接触。−80N/暂停问答/人工恢复执行，但问答误称噪声。正确东西门框后空间查询42unknown+1occupied，未完整出门；同帧重复审核后终止。两项evaluator/results齐全；另保留0任务ffmpeg PATH启动失败并修复入口环境。报告prototype/NAVIGATION_LOCAL_AUTONOMY_20260916.md，服务结束/GPU0释放，未扩批/硬件/训练/Git。

- 2026-09-16 14:29 HKT：完成Qwen工具错误调查/修复、传感器里程计改善和LightNav边界研究。三次实际Pi短调用分别复现非法历史HTTP400、验证文字反馈与生产纠正成功；最初scalar来源未定。里程计替换为深度/IMU/本体速度融合，正确解码aux速度尺度并消费200Hz IMU；新19秒191帧配对位置RMSE26.32→4.77cm，63归档帧8.15→4.02cm。构建及生产估计器运行通过。LightNav实际采用0/197 MPC，建议连续局部自主权，尚未改权限/重跑完整导航。报告prototype/NAVIGATION_REPAIRS_20260916.md和LIGHTNAV_AUTONOMY_BOUNDARY.md；服务结束/GPU0释放，未训练/硬件/Git。

- 2026-09-16 13:22 HKT：按Owner执行一次正式GPU0/Qwen p05传感器框架导航尝试，89.475s、10请求、4转向，NEEDS_USER。无效工具arguments的纠正请求HTTP400；无前进/portal/完成，后拉未触发，正非支撑接触0。两项evaluator/汇总对同一记录完成；发现17.68s里程计位置RMSE0.25056m。另保留0模型请求启动失败，修正规划观察误套停稳门槛及归档旧变量后才进入正式尝试。证据prototype/NAVIGATION_QWEN_SENSOR_TEST_20260916.md，服务已结束，无额外回合/硬件/训练/Git。

- 2026-09-16 11:47 HKT：按Owner确认的RGB-D/IMU/关节输入重建共享RobotState，移除在线基座位姿/速度真值，独立ground_truth供evaluator；重写相向边界/自由走廊/远端扩展及当前目标相对完成证据。旧同侧配对完美位姿反例被拒绝；GPU0有界0.25m sensor→MPC运行26/26求解成功、PAUSED，正非支撑接触0，两项evaluator完成。不是新完整导航或实机验证；完整报告prototype/NAVIGATION_SENSOR_REBUILD_20260916.md。服务/安装已结束，未训练/硬件/Git。


- 2026-09-16 00:28 HKT：完成新机器全部环境/权重及真实Pi Qwen连接（250ms），GPU0原生NVIDIA EGL成功运行指定p05。新真实完整成功0/1、NEEDS_USER：同侧西墙/西门扇误配portal，Monitor提前完成阶段0，真实机身未完整出门；电梯净距6.853m，三次恢复后净空不足。−80N后0.72sim秒制动，问答/人工恢复保持任务，正非支撑接触0。修复runner误拒退出码2和evaluator过时modelReason字段，以同一物理记录完成两项评估和results；没有重跑、放宽条件或新增测试。Gemini暂缓；服务结束、GPU0为1MiB/0%，服务器全局代理关闭、Codex专用配置未动。详见prototype/NAVIGATION_MIGRATION_20260916.md。

- 2026-09-15 23:16 HKT：新机器克隆指定分支并恢复全部迁移附件（3 ZIP/447文件）和公开YOLOE/MobileCLIP权重；建立Python3.11.7独立venv，写入默认Qwen2048/off/精确FP8末尾斜杠配置，raw实验室models可达。实际GPU0 UUID匹配EGL2，但未启动renderer。官方Node/PyTorch/PyPI/npm/HF直连受阻，安装停止，实际Pi和Qwen p05回合均NOT_RUN；Gemini按Owner最新要求暂缓。证据`prototype/NAVIGATION_MIGRATION_20260915.md`，未改框架/条件、未新增测试/训练/硬件操作，任务安装/下载会话已结束。

- 2026-09-15 HKT：按Owner从GPU3切至物理GPU0，CUDA/EGL共同UUID绑定（GPU0/EGL2），移除GPU3专名入口。实际GPU0 LightNav加载成功，但EGL仿真仍在nvkms_open_common阻塞，两个新任务回合均未开始；失败及进程设备证据在navigation_framework/startup_failure_gpu0.json。已停本任务服务释放GPU0。另将问答/审核用途提示与行动分离，4份记录状态上下文检查通过。

- 2026-09-15 HKT：针对Owner指出的框架问题，还原完整上下文并修正未锚定时的工具/提示矛盾；改用Pi StringEnum、自动用途/关联、通用空间边界证据，移除模型阶段完成工具，改为Monitor基于当前语义证据和最新状态自动推进。Owner明确YOLO词表保留为任务配置，通用框架不加特定目标识别模板。原图回放中Qwen两次求助改为有效转向、Gemini返回有效掩码参数；重复掩码及两次到达记录的代码路径通过。新鲜仿真被NVIDIA初始化阻塞，未有新完整回合，服务已关闭。详情prototype/NAVIGATION_FRAMEWORK.md，旧冻结成绩保留。

- 2026-09-15 HKT：实现GPU3物理UUID/EGL对应绑定、YOLOE26l同帧实例掩码深度测量、主动机身观察和跨视角jamb证据。完成固定Qwen36与条件Gemini36；完整通过0/0、真实候梯4/0、真实出门7/2、误报及正非支撑接触均0。Gemini33次enum/const参数失败限制模型比较；Qwen两次真实候梯因测量姿态有效性检查终止。8个实际后拉暴露均锁存/问答/恢复；其余未暴露单列。GPU3历史重放161/161未复现旧80帧异常，不等于根因已修复。全部失败、冻结源码、GPU配置、掩码/视频和判据见prototype/NAVIGATION_YOLO.md。初次EGL编号错误导致开发回合渲染GPU1，已披露并排除；正式72轮全部GPU3。收尾服务关闭，未commit/push。

- 2026-09-15 HKT：完成36轮压力导航（24整圈随机、12面向门洞随机）及限定复放；独立完整验收0/36，1例南墙假完成。11个后拉真实暴露均锁存/问答/恢复并保持task；全部无非支撑接触。确立门叶/墙误作jamb→错误portal→MPC停止链，复现LightNav80帧视觉编码异常及50→52采样边界；没有模型/历史规避改动。显式Mesa渲染用于避开默认NVIDIA EGL启动故障，区别于旧GPU渲染结果。报告prototype/NAVIGATION_STRESS.md保留全部失败和中断。

- 2026-09-15 HKT：当前最终实现primary_08完整完成同一句“出门走到电梯门口”。78.487s、实际11.1166m、真实门前包络净距0.8090m、朝向3.516°、零非支撑接触；首次/末次ROI对应真实门板。14次Qwen调用，电梯连续目标31.18sim秒内无调用；10个带buffer上下文均保持唯一上一动作，路程与原始50Hz一致。同版本交互02仍在目标搜索失败，故仅记本回合成功；原始报告和全部负结果在prototype/NAVIGATION_V2.md。

- 2026-09-15 HKT：完成workflow v1.4.0定向同步、场景外观、唯一上一动作buffer、50Hz实测/LightNav预测分开、同阶段导航session、目标平面及显式候梯约定、连续follow_target。全部开发回合保留在`.local/outputs/navigation_v2/`；详细实现和结果见`prototype/NAVIGATION_V2.md`，不把版本迭代作为固定版本成功率。
- 真实交互01发现问答误用change_goal改写原任务；移除问答的目标写入工具后交互02保留任务/阶段，−80N请求0.66sim秒后锁存，后续160个控制采样至恢复均零有效命令，人工恢复后真正出门。暂停仍可移动0.319m；问答“力矩刹车”术语不准，电梯搜索仍失败。无非支撑接触；终态3.04秒无新Qwen调用。源码构建及受该错误影响的现有暂停问答用例通过。

- 2026-09-14 HKT：实现Pi常驻TaskMonitor（事件队列、单个有效模型提议、SQLite/JSONL、原需求和阶段、scene绑定/ROI分步审核、权限版本、真实结果驱动恢复）。Python保留force brake与独立3秒运动租约；移除旧生产Agent/TaskSession路径。12契约用例与真实Python控制契约通过。
- 新增room_elevator房间→大厅→闭合电梯场景。4轮主任务和3轮交互全保留：primary04真实两阶段9.293m、零接触、距门1.900m；交互03真实制动/放力锁存/新图问答/人工恢复原阶段/受阻转向/两阶段10.856m、零接触、距门1.692m。不是可靠语义成功或成功率；门扇/踢脚线误认和混合ROI仍在，见prototype/MONITOR_TEST.md。
- force_brake问答错因已补明确控制事实，一次真实Qwen同暂停帧回放纠正原因；原错答保留。常驻终态3秒窗口无多余模型调用。长期结论与未覆盖项见TODO。
- 对旧memory作语义归档：history-before-monitor.md保留此前实验和旧实现说明，description.md只路由当前Monitor与固定基础；不删失败证据。

- 2026-09-14 HKT，新增原生MuJoCo elevator_lobby（10×8m、闭合双门/楼层显示/按钮、普通门干扰），明确不是RMF Hotel World转换。新增真实任务录制入口。一次预登记电梯任务40.12s/6.574m、无非支撑接触、MPC256次成功，closer追加.675m，终点最近可见门板约1.229m；未加1m条件。Qwen两次ROI帧错配/最终错引351保留，1.269m来源实际为350概览。报告 `prototype/ELEVATOR_TEST.md`，完整证据 `.local/outputs/elevator_scene/`；收尾服务为电梯大厅IDLE。

- 2026-09-14 HKT，按Owner要求确认默认实验室Qwen并跑完3个预登记完整回合，同一生产实现未改动/未挑选重跑。普通38.83s/5.961m、受力39.87s/6.932m均过门且无非支撑接触；同Pi会话强后拉0.60sim秒锁存、下一tick归零、放力/问答保持、navigate409、人工恢复后继续路线全部执行。但暂停问答无新图并错误声称穿门，最终ROI语义仍混背景。
- 数值回合40.27s/5.476m，无非支撑接触但身体包络未清门，4次净空阻塞/3次恢复后NEEDS_USER。3个measure409已定位SSE帧号与同批scene返回值依赖错误；终点附近最后帧独立目标mask最近2.165m，未达到约1m。全部失败保留，源码快照/三段视频/物理与语义分析在 `.local/outputs/qwen_full_test/`，报告 `prototype/QWEN_FULL_TEST.md`。收尾reset默认ProcTHOR/D435i为IDLE。

- 2026-09-14 HKT，增强Depth翻译器完成：原RGB+编号图，全图选框+裁图+距离层像素图；Pi多图接口与生产MuJoCo/CLI实际通过。完成三模型×旧/增强表达×三冻结帧18回合，同prompt未重跑；45次成功ROI分层计数与独立参考一致，1次Gemini越界后纠正。Gemini增强最终ROI均100%目标；DeepSeek增强有目标距离但仍混背景；Qwen两条缺测并声称已测量、增强两条错ROI/错层。全为1m负例，不代表动态停车成功。见 `prototype/pi/DEPTH_COMPARISON.md`。

- 2026-09-14 HKT，本轮改回实验室Qwen、D435i RGB-D，实现编号区域深度概览/同帧ROI裁图与测量、新A2身体/摆腿包络MPC。实际修复地图GIL排序、Pi重复读图、求解延迟/减速模型、转向制动，以及仿真MSAA深度边缘假点。
- `integrated_05`实跑32.92s、5.618m，基座过门且5ms接触无非地面碰撞；保留4轮集成/1轮独立控制的早停负结果。Qwen仍错误用墙/地板作为目标证据，COMPLETED不是独立任务验收。详见 `prototype/RGBD_MPC_RESULTS.md`。
- 最终版本 `force_01` 的−40 N后拉使actual vx均值0.0744 m/s（command 0.300），随后MPC暂停，未过门或执行强制动。独立 `force_authority_flat` 无Pi权限实验通过：−80 N请求至锁存0.60 sim秒、下一20 ms tick指令归零；放力仍锁存、navigate409、human resume200，随后actual vx=0.148 m/s。两轮均无非地面接触；已reset回默认ProcTHOR/D435i初始场景。

- 2026-09-14，ZenMux两模型已接入Pi，RGB/工具回传通过；数值1米任务均暂停，DeepSeek解释原因、Gemini仅工具暂停。各一次语义导航41.35s/46.31s，base均过门、模型报告COMPLETED；不代表无碰撞/精确距离/错误恢复验证。DeepSeek首次thinking+required返回400，改auto后通过，失败保留。默认Qwen和旧checkpoint不变，密钥仅在本地未跟踪文件。详见 `prototype/pi/ZENMUX_RESULTS.md`。
- 2026-09-14 17:00 HKT前后，按Owner视频反馈复核：默认迁移到头部相机768×272/132°×77°/15Hz；位置与Unitree官方camera_joint完全一致，具体实机分辨率等未核实。
- 移除Pi仿真深度/目标距离输入与默认1米检查。`head_rgb_only_01`真实穿门但撞沙发和门框，5ms接触记录明确到机器人部位/场景geom；不再宣称无碰撞或已可靠完成。
- 数值任务 `head_rgb_metric_request` 未通过：模型明知无测距仍两次试图确认约1米，随后3次有效导航，30s超时。保留原始失败，未继续改提示求成功。到达判定、机器人几何避障均仍是设计缺口。

以下为此前版本的历史进展；当时的数值到达硬检查后来被移除。当前已重新接入模块化深度翻译器，但没有恢复历史默认1米格点检查。

- 2026-09-14 HKT：建立旧 A2 worktree，隔离数据盘依赖。
- 配置相机 profile、安装 Pi，真实读取仿真 status/RGB。
- 实现 MuJoCo A2 物理/相机、旧 policy PD、LightNav WS/MPC 接口与任务状态。
- 完成一次旧 policy 站立/前进和真实外力功能演示，观察到 force brake 锁存与人工恢复路径。
- 官方 MPC 与旧 A2 的固定 2 m 直线路径已运行；0 / −40 N 对比中实际位姿驱动的路径进度容纳了受力减速，详情 `prototype/RUN_STATUS.md`，不代表 LightNav 完整闭环。
- LightNav 官方模型已下载并启动，A2 首帧10点轨迹实际返回，约354ms server / 409ms client，驻留8972MiB。
- LightNav/MPC/A2 已运行一次真实转向前进闭环，8.5s后进入候选到达；最终画面不能确认已穿门，未声明语义成功。运行后已人工暂停。
- 下载代理问题确认来自tmux环境；所有后续下载入口持久化直连。本地 Qwen 下载随后按 Owner 新指令取消，详见下一项。
- 2026-09-14 13:04 HKT：按 Owner 最新指令改接实验室 Qwen `vic.precognition.team:8888`，精确模型ID保留末尾斜杠；本地下载和待启动服务已停止，废弃数据清理约8.07GiB，本地部署脚本移除。
- Pi 已通过实验室 Qwen 实际调用status/scene并读取RGB，首文字约1.01s、完整回合3.65s，模型未确认已穿门；没有发送运动请求。运行证据 `lab_qwen_pi.json`。
- 2026-09-14 16:13 HKT：完整 `build_force_08` 在57.407s内完成穿门、接近家具约1m、受力制动/人工恢复及一次局部纠正；退出0，COMPLETED。base实际过门，目标表面格点1.149m，详见运行记录；不以Qwen误写的0.81m代替实际测量。
- 实现 SSE 事件驱动 Pi TaskSession、run/chat、原目标保留、自动候选核验和局部恢复。实际故障后修正工具只写计划问题及同回合重复导航；一个成功动作后等待下个事件。
- 导航受力实跑发现旧 RC 的角速度资格会漏制动；NAV前进转弯移除该限制，旧checkpoint与力阈值不变。run08强后拉0.42s锁存，下一0.02s有效命令归零，松手不解锁。
- 根据真实完成误判，加入同帧理想RGB-D与base水平距离转换；首次navigate冻结距离，complete由代码取格点数值核验。没有提供场景全局几何给Qwen/LightNav。
- 统一 `prototype/run.sh build/up/down/reset/status/run/chat`；仅本任务两个tmux服务及GPU0。暂停问答第二轮真实status/scene调用、无运动工具、保持HUMAN_PAUSED；补齐了policy/PD仍维持站立的对话事实。

- 2026-09-15：迁移实现已准备：仓库相对数据路径、显式传入tmux的GPU选择、独立Python环境依赖、两个固定起点的便携runner及`prototype/MIGRATION.md`。本次未重启已受阻的仿真；Git发布/Drive上传状态以迁移回执为准。
