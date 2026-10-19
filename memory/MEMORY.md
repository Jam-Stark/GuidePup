# Memory Index

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

本目录保存 ForceControl 的 file-based memory entries。读取时按根目录 `MEMORY.md` 路由进入，避免一次性读取无关 subsystem。

## Entries

- `navigation-prototype/` 最新：2026-09-20 18:14 HKT：GRScenes全236文件76.616GB直连镜像下载完成，医院ABY8原生资产按厘米缩放/原碰撞接入，GPU7唯一实际创建任务175.265秒、1.212m未出源门/未入目标、最终停稳；Qwen完成误报，真实0/1。54LightNav返回52接受，唯一有效STOPseq53距源门前缘仍63.77cm；85原生成功1候选受阻、84下发匹配，左25°恢复真实25.507°。无任务内ERROR；初始化材质缺陷和03任务创建前地面估计错误单列。预览/模型启动等待已改为暂停仿真时钟，04的hold/release均1.74s。两评估/results/视频与原始证据完整，服务/端口/tmux关闭，GPU7释放，不追加。报告`prototype/NAVIGATION_GRSCENES_20260920.md`。

- 当前 workflow v1.4.0 与同步证据：`memory/workflow/description.md`；角色按 `.codex/TEAM.md`，历史 roster 不代表存活 agent。

- `navigation-prototype/`：当前默认学校MF-R1场景，一层起点到中间2号电梯门前。详见`prototype/NAVIGATION_CAMPUS_MF_R1_20261015.md`。GPU7完成A2/RTX/RGB-D/里程计/地图加载验证，未执行完整导航；8768隔离原8766，服务已关闭。两层场景保留，跨层乘梯未接入；此前导航结果按description历史路由。

- `mujoco-experiment/`：alan 长期职责；当前 MuJoCo sim2sim experiment 配置核查/解释；维护 failing command、runtime 参数、policy/model/export、delay/friction/contact/first divergence TODO。
- `mujoco_official_api/`：bob 长期职责；MuJoCo 官方 docs-first 约束；IsaacLab 到 MuJoCo sim2sim 的 XML/API/solver/contact/control stepping 规范证据。
- `manager_based_cfg/`：john 长期职责；当前 manager-based 版本 cfg 的核查、解释和修改方案；重点维护 `unifpaliengo_env_cfg.py` 的 observation/action/command/reward/termination/noise/delay/domain randomization 契约与 sim2sim 对齐 TODO。
- `agent_roster/`：长期子agent roster；记录 alan/bob/john 的 agent id、职责 memory 入口、回答前证据约束与后续任务分配规则。

## Build History

- 2026-06-03 22:51 HKT：初始化 `mujoco-experiment` entry，记录 alan 职责、回答 memory 证据约束、failing path 和初始 sim2sim experiment TODO。
- 2026-06-03 22:52 HKT：初始化 memory system 最小路由，并新增 `mujoco_official_api` entry。
- 2026-06-03 22:52 HKT：新增 `manager_based_cfg` entry，记录 john 职责、回答 code 引用约束、当前 cfg 摘要和 sim2sim 对齐 TODO。
- 2026-06-03 22:55 HKT：主控复核并补齐 `mujoco-experiment` 路由；新增 `agent_roster` entry，记录三个长期子agent 的运行时 id 与分工。
