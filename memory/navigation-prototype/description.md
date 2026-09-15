# Navigation prototype：通用框架与RGB-D当前入口

- last_verified：2026-09-15 HKT；status：通用框架修正已构建并完成3次真实模型上下文回放及3项记录数据路径验证；Owner改用GPU0后，GPU0与此前GPU3均在NVIDIA初始化阻塞，未取得新完整回合。此前冻结72轮仍各0/36完整通过。
- source_of_truth：`prototype/pi/src/task-monitor.ts`、`prototype/pi/src/monitor-tools.ts`、`prototype/sim/server.py`、`prototype/sim/semantic_depth.py`、`prototype/pi/navigation-contract.json`；最新证据`prototype/NAVIGATION_FRAMEWORK.md`及`.local/outputs/navigation_framework/`；此前冻结结果见`prototype/NAVIGATION_YOLO.md`、`prototype/NAVIGATION_STRESS.md`、`prototype/NAVIGATION_V2.md`。
- read_when：继续Pi/Qwen/LightNav/MPC导航、单段记忆、目标/停点语义或中断恢复。
- supersedes：旧Monitor实现和实验摘要保留于`history-monitor-v1.md`，更早实现见`history-before-monitor.md`；历史成功不自动代表当前版本。

## 固定基础与范围

- Owner后续明确将本任务改用物理GPU0，覆盖此前GPU3限制。当前`prototype/gpu.sh`配置LIGHTNAV_GPU=0并用GPU0 UUID绑定CUDA；`gpu.py.configure_gpu()`在同一server进程、MuJoCo导入前匹配EGL物理UUID，GPU0当前对应EGL索引2。LightNav/YOLO进程内cuda:0对应物理GPU0。此前正式72轮为GPU3/EGL1，是历史实验配置；EGL3实际对应GPU1，不能把EGL编号等同物理GPU编号。远程模型沿用既有API。GPU1句柄异常与GPU3渲染阻塞同时出现，因果尚未证明。

- 写入工作树 `/home/baoquanc/workspace/ForceControl-lightnav`，分支 `codex/a2-lightnav-prototype`；原 `ForceControl` 有新版未提交工作，不是写入目标。未 commit/push。
- `.local` 指向 `/mnt/ssd1/baoquanc/forcecontrol-lightnav`；代码在工作树，环境/模型/输出在数据盘。软件/模型下载及模型服务直连，清除大小写 HTTP/HTTPS/ALL_PROXY，NO_PROXY/no_proxy=`*`；tmux 也遵守。Owner 2026-09-15 指定迁移附件 Drive 上传走代理。
- 旧 run `2026-05-23_14-25-38_A2_base_only_v2/model_59999.pt`，已有 `mujoco_exported/policy.pt` 与 JSON。46×32 term-major 历史、12 关节动作、旧 6D 辅助输出（velocity/base force），不是新版 9D 扶手 wrench。没有训练/重导出/换 checkpoint。
- 实验室 Qwen `http://vic.precognition.team:8888/v1`，精确 ID `Qwen3.6-35B-A3B-FP8/`，末尾斜杠保留；不下载本地模型。Pi0.85.1、Node22.23.2，Qwen thinking off。凭证仅本地未跟踪文件，不写入 memory。
- 仿真 prototype：MuJoCo、静态平地、局部语义导航与 force 交互。不含人体跟踪、人体包络、乘梯、Isaac Lab 接通或实机部署。
- D435i `d435i_rgb_nominal`：640×360、69°×42.2726°、base [.30,0,.10]m、下俯5°、目标10Hz，有效光轴深度.2–8m。是理想对齐 RGB-D，不是实机标定、原生更宽深度FOV或真实噪声证明。
- LightNav 只收原始 RGB；局部路径用拍摄时刻 base 位姿转 world，MPC 读实际位姿/速度。没有给 Qwen/LightNav 目标真值坐标或真实路线剩余距离。
- Depth 翻译器给原图、6×3编号图、区域分层统计，ROI 给真实裁图/选框/层像素图。距离统一 base 水平 hypot(x,y)，不是光轴Z、机身前缘净空或路线剩余距离；unknown 不代表自由空间。层颜色/占比不是物体语义或识别置信度。层代表像素给完整同帧变换后的 base/world 点，供测量门平面。
- MPC 仍是三圆包络（base x=−.30/0/.30、r=.30），4cm占用体素及膨胀，15×.15s预测，上限.3m/s/.6rad/s，预测实际响应/求解耗时，完整点集核验，只支持前进和转向。地图只记录4m内已观测占用。MSAA offsamples=0 修复深度边缘假点的既有结论不变。
- force brake 由 Python 独立处理：旧 auxFx≤−.6 连续2tick且 requested vx≥.2；NAV 前进转弯不受旧人工速度模式 abs(wz)≤.1 限制。HUMAN_PAUSED 锁存、仅人工恢复。检测 tick 的 effective_cmd 是刚执行的旧指令，下一50Hz tick归零；仍执行旧 policy/PD，零命令不等于位置锁定。

## 当前实现

- workflow v1.4.0已从`feature/a2-migration`源工作树的未提交文件定向同步；未merge/commit/push，宿主hooks发现/信任未验证。证据`.ai/WORKFLOW_SYNC.md`。
- Pi `models.complete`每次重建上下文：短系统协议、当前任务/观测、唯一上一轮被采纳的运动决策及其真实运动。普通决策带当前RGB、上一决策RGB及当前YOLO覆盖图；不携带阶段入口图或聊天历史。选区/审核/问答不替换buffer；新运动归档上一段，按需`recall_memory`最多取两段本次使用。
- YOLOE26l先做RGB开放词汇实例分割，四类door/door frame/elevator door/wall，再以同帧对齐depth和内外参计算掩码成员深度层、代表点、距离与平面。实例ID只在帧内有效，目标ID另有稳定身份；原始掩码/标签/分数与RGB均可追溯。只在请求观察时推理并按帧缓存；LightNav仍收原始RGB，不接收标签。没有注入场景真值。
- 三张预选仿真帧默认0.25阈值全漏检；自然照片正常且低阈值仿真候选仍误标。正式固定NMS分支、候选阈值0.03；视觉接受使用confirmed_target/uncertain/not_target，后两者不能锚定。用途role由动作类型赋值，模型不再填写。Owner确认YOLO词表仍是任务配置；通用提示不加目标外观模板或类别接受规则。
- Python `/motion`从50Hz真实位姿累计路程/转角，最多32个展示点；首个及最新两段LightNav预测分别记录捕获位姿/时间，不能拼接当路程。完整原始日志另存。同阶段LightNav会话持续；换scope显式reset，失效响应收完后丢弃。
- 目标保存稳定ID、语义主张、同帧ROI引用及实测平面。测距层不等于目标分割。Qwen仍需核验当前RGB和距离层像素归属；几何一致不证明对象身份。
- 无明确数字距离时采用显式原型约定：三圆包络到已测目标平面0.6–0.9m、朝向±25°、相对目标patch横向≤0.65m。用户明确数字距离优先。出门要求完整包络越过观测入口平面并留0.35m；离线另查真实门框远端。
- `search_target`为指定左右/15–90°/待找证据的实测机身转向，停车后用新图规划；`change_view`为5–45°局部观察。A2没有独立头轴。观察不需要先识别目标，currentStep按真实工具/证据状态生成。`inspect_target_hint`保留LightNav当前指向的小块测量，仍需身份审核。
- 掩码/深度层输出实际世界支持范围，mask_relations给出重叠和空间间隔。通行阶段使用B编号的空间边界证据组，不按两个mask直接分配左右。现有0.7m可用跨度用来判定能否作为开口两侧，不证明同一物体身份；分开的两组仍需sameOpening视觉确认。模型只在旧识别错误时设replacePreviousIdentification；内部association状态由框架维护。
- 工具枚举使用Pi现有StringEnum。参数错误不执行，以原始toolResult/isError回传，在同一30秒决策期限内可纠正一次；再次失败显式停止。不是自动把无效字符串转换为合法动作。当前三次新模型回放均首轮有效，尚未在本轮真实模型调用中触发纠正分支。
- `follow_target`用已测退出点/停点及最终朝向，整段由Python连续执行。一个controlTaskId和buffer贯穿内部阶段；`goal_phase_revision`丢弃旧MPC结果，不改变外部generation或唤醒模型。0.08m位置锁存、最终朝向8°内触发target_checkpoint。当前ROI语义确认后Monitor依据本阶段执行记录、最新位姿/速度自动完成，不再使用propose_stage_complete。条件变化进入新观察；正常yaw漂移不独立使世界目标失效。generation及人工暂停权限保留，控制API成功后才推进。
- reference_source明确区分lightnav/metric_turn/metric_advance/metric_goal；显式运动用MPC参考，LightNav仍后台阶段语义推理/记录/指向。既有身体、障碍、响应和求解延迟约束保留。原地转向命令vx=0，实际机身可漂移。
- 问答入口只允许answer_user和recall_memory；换目标由既有操作方`/task`或`/run`明确提交。真实交互01发现Qwen把答案错当change_goal替换原任务，已删除该模型工具和执行路径。
- Python force brake、3秒运动租约、人工恢复和控制版本仍独立于模型。新图问答含准确controlEventFact；解释是否正确由实际答案核对。

## 证据入口与已知限制

`prototype/NAVIGATION_V2.md`记录每次开发回合、具体失败及相应改动，不能把不同版本挑选成成功率。初次整合构建/13可控提议用例为协议证据；后续仅检查受实际问题影响的路径。

- primary_01–05暴露宽ROI地面/门扇/墙面误认、LightNav短语动作方向/停止尺度、Qwen左右换算错误；全部负结果保留。
- primary_06已选中真实电梯门板，距已测候梯点剩0.428m时因中间小动作后目标离开画面求助，推动完整follow_target实现。
- primary_07一次连续出门、真实整个包络sim19.94s清空门框远端；buffer路程与50Hz原始累计一致，同阶段单LightNav会话。电梯小块测量本可用，但Qwen扩大ROI后选混合层，NEEDS_USER；无非支撑接触。该轮没有执行电梯连续接近/最终朝向。
- human_pause_question_resume_01实际制动/撤力仍暂停/问答/人工恢复均发生，但问答误换任务；修复后的02保留原任务，0.66sim秒锁存，160个后续控制采样至人工恢复均零有效命令，真正出门，后续电梯搜索失败。
- primary_08与交互02同实现，78.487s/11.1166m完整到电梯；真实三圆前缘净距0.8090m、朝向误差3.516°、无非支撑接触，首次和末次ROI代表点及图像均支持真实门板。本次14个Qwen调用，电梯连续目标31.18sim秒中无模型调用；buffer和50Hz路程一致。只是单例完整结果，不是稳定成功率。
- 外观与多项协议同时改变，无隔离因果归因；无跨场景成功率、长时间压力或硬件结论。Qwen仍会错误关联实体，候梯横向距离是相对所选patch而非完整电梯中心。

## 运行路线

- 工作目录固定`/home/baoquanc/workspace/ForceControl-lightnav`；`bash prototype/run.sh build/reset/status/run/chat/monitor/down`。
- 无scene的reset仍默认ProcTHOR；当前场景`reset --scene room_elevator --record-contacts`。先关闭持有控制权的Monitor再重置。
- `monitor --port 8770`：GET /state，POST /task{goal}、/question{text}、/pause、/resume。只绑定127.0.0.1；问答不修改任务。
- 主任务录制：`.local/envs/sim/bin/python prototype/sim/record_task.py --scene room_elevator --goal '出门走到电梯门口' --output <new> --timeout-s 360 --max-recoveries 3`。
- 交互录制：同环境`prototype/sim/record_monitor.py --output <new> --timeout-s 360 --decision-timeout-s 30 --max-recoveries 3`。离线先evaluate_monitor.py、后evaluate_navigation_v2.py，均用`--episode <dir>`。
- 任务服务名LightNav=`forcecontrol-lightnav-serve`、sim=`forcecontrol-lightnav-sim`；不要干扰其他训练tmux或GPU。输出/服务当下状态在任务报告记录，不以历史服务状态替代实际查询。

## 2026-09-15 旧Mesa压力测试事实

- 详见`prototype/NAVIGATION_STRESS.md`及`.local/outputs/navigation_stress/`：整圈随机12位姿×两条件24轮均未出门；随后面向门洞6随机位置×两条件12轮，10轮真正出门、11轮进入阶段2。合计独立完整验收0/36，Monitor1轮误报完成。两起点分布分开；还有一次启动失败0样本及一次实验者中断尝试，均单列保留。
- 11次实际−80N后拉均锁存、问答、人工恢复，task ID保持。7个后拉条件样本未触发力即结束，不算恢复失败。36轮均无正非支撑接触。
- 明确感知失败：打开门叶/东墙被当成左右jamb，退出点被放到东墙后；MPC停止不等于身体撞墙。指定jamb中点的首图投影0/12同时在视锥（只是指定点覆盖，不证明整框可见/遮挡）。回忆能正常取出片段，但旧错误portal可被当成真实目标；另有一次recall空参数拒绝。
- t06_force_resume假完成：最终包络前缘距真实电梯平面8.267m；target实际是y=−4的大厅南墙，semanticClaim还明确写“非门板”。几何一致不等于实体正确。
- 9个导航回合在本scope第80帧发生CUDA错误；其中面向门洞组7轮已有真实电梯anchor。限定复放：原80帧和把第80张换成第79张都在80失败；失败图作为首帧正常。位置/人工暂停不是该故障必要条件。源码/CPU计算显示SlowFast采样50→52帧、PatchEmbed Conv3d首维11200→11648；num_history_frames64是该分支初始分配容量，不是历史cap。合法shape为何触发CUDA仍未定位到内部算法；未通过重置/丢帧规避。
- 默认NVIDIA EGL启动在nvkms卡住，GPU1句柄UnknownError；显式Mesa vendor软件EGL索引4+llvmpipe已真实运行，GL配置被记录，LightNav仍是GPU0既有服务。无GPU/驱动重置。Mesa输入/墙钟速率不能假定等同旧NVIDIA回合。
- 运行器新增`--spawn-pose X Y YAW_DEG`，在qpos/history初始化前应用；`--record-navigation-inputs`保存每次next精确JPEG和同帧RGB-D/外参到navigation_frames。server配置保存实际spawn和GL实现；stack显式向tmux传渲染env，避免tmux丢失选择。
- 旧压力入口`prototype/sim/run_navigation_stress.py prepare/run`，离线`evaluate_navigation_stress.py --batch DIR`；旧manifest保存实际Mesa环境。当前GPU3启动由物理UUID映射EGL的入口控制，不沿用旧EGL索引。

## 2026-09-15 GPU3与YOLO对照事实

- 新入口按物理GPU3 UUID绑定CUDA，并在server同一进程、MuJoCo导入前选择相同UUID的EGL设备；实际GPU3对应EGL1。准备阶段development_01曾误把EGL3当GPU3，实际渲染GPU1；已停止、单列不合规开发样本并修正，全部正式72轮配置均为GPU3/EGL1。
- GPU3环境重放原80输入80/80、以79替80的序列80/80、旧失败图单独1/1，共161/161。没有改LightNav模型/历史处理；只证明此环境未复现，不证明已解释或修复旧CUDA内部故障。
- 同一冻结源码、相同18个初始位姿×普通/后拉两条件，Qwen36不足29完整通过后自动运行Gemini36。全部NEEDS_USER；Qwen真实整个机身出门7、几何候梯并停稳4，Gemini出门2、候梯0；完整通过0/0，无误报完成和正非支撑接触。整圈24与门向12分开，详情只链接报告，不从混合便利样本估计一般成功概率。
- Qwen实际使用掩码9轮/18次，Gemini5轮/9次；手工矩形52/39次，实测观察转向46/59次。Qwen p12跨视角门框配对后真实出门；也有同一门框两个实例被当成两侧的错误。标签和sameOpening主张不能证明身份/关联。
- Qwen p05_force_resume、t04_primary实际候梯合格却在接受当前ROI后被“测量后姿态改变”抛错终止：观测到终态平移仅约1.3cm，转角超过旧0.1rad条件。另两次实际候梯p12_primary、t02_force_resume卡在终点证据/提议确认，未报完整成功。
- Gemini33轮工具校验失败全为enum/const不匹配：association23、选区role9、recall reason1；另有一次决策超时。两个出门回合已看见真实电梯，S1代表点也在真实门板上，却因role输出target_surface/elevator door（要求target）在接近前中止。Pi实发anyOf/const非strict schema；未捕获网关原生payload，不能唯一归因模型或网关。这是实际配置比较，不能宣称Gemini视觉更差。
- Qwen实际后拉6轮、Gemini2轮，均锁存/问答/人工恢复；另外12/16个后拉条件未暴露，仍保留分母。当前对照未在正式批次中改协议或重试失败，所有源码快照、原始RGB-D/掩码、agent/interaction/status日志、视频和逐轮判据均留存。
- `prototype/sim/run_navigation_comparison.py prepare/run --output <new>`为固定套件入口；单回合record_task.py/record_monitor.py支持`--model qwen|gemini`。Qwen lab FP8/、2048/off；Gemini既有ZenMux google/gemini-3.8-flash、4096/low，每次请求记录实际配置，秘密不进入memory。
- 本次正式运行17:19:30–18:10:44 HKT；收尾仿真/LightNav服务已关闭，GPU3回到18MiB桌面基线，没有活跃长跑。未来服务状态以当前查询为准，不以此历史收尾事实替代。

## 2026-09-15 通用框架修正证据

- 完整上下文确认p03_primary、p08_force_resume只有一份边界且无portal时仍收到follow_target/退出点规则，工具却未提供这些动作；搜索描述混入识别接受限制。不存在先有portal才能search的硬代码循环。上述提示冲突已修正，不能由旧失败直接推断模型导航能力差。
- 原图字节及顺序一致的真实回放：Qwen这两例均返回search_target（左45°/右45°），Gemini t01返回有效S1掩码选择，不填写role。各一次新提议，不是完整任务或成功率；多个框架因素同时改变，不能隔离单项文本因果。
- p02真实S1/S4的D4点距0.11648m、支持间隔0、掩码交集1698px，新路径只形成一份B证据组。原例在非法参数处终止，并未放行错误portal。
- p05/t04的原始当前ROI与失败前最后一条同generation状态，在新completeIfReady中均到达complete API调用边界；没有伪造控制成功或模拟新物理。一次限定源码审阅未发现当前证据/人工暂停权限绕过。
- 新鲜Qwen p05启动在NVIDIA nvkms_open_common阻塞，未进入任务；随后Gemini计划未启动。LightNav在GPU3启动，GPU1同时有句柄Unknown Error，因果未确定。未改渲染后端或重置驱动；所有本任务服务已停止，GPU3回到15MiB基线。继续完整回合需要先有可用渲染环境。
- 审查图像上下文时，observation.imagePaths是全部传感器归档（RGB/网格/语义图），不等于request附件顺序；历史图须按原buffer图像源追踪。初次离线提取误配已在模型调用前纠正，最终回放逐项比对实际字节/格式/顺序，不使用哈希。
- Owner随后改用GPU0，代码统一为gpu.sh的LIGHTNAV_GPU=0及gpu.py.configure_gpu()的UUID匹配，GPU0/EGL2。smoke_02_gpu0中LightNav521803确实在GPU0加载8972MiB，仿真524100的CUDA环境也为GPU0但仍卡nvkms_open_common并超时；没有新回合，不能说换卡解除了驱动阻塞。服务已停，GPU0回到15MiB基线；未来继续用run_smoke.py --output <新目录>，不覆盖失败。
- currentStep按purpose区分问答、测量审核和行动；问答/审核不带当次不可用follow_target指令。以两份真实记录状态构建4个相应用途上下文检查通过。自动完成条件/控制权限没有在此修正中改动。

## 跨机器接续

2026-09-15：Owner已授权当前prototype/workflow代码commit+push及必要checkpoint/log上传Drive。迁移入口`prototype/MIGRATION.md`，版本标记`20260915_215130_HKT`；实际上传以回执为准。Pi数据路径统一为仓库`.local`，GPU通过`LIGHTNAV_GPU`选择并显式传入tmux；新入口`prototype/sim/run_framework_reproductions.py`固定两个未运行case，不读旧机器绝对路径。Python环境从明确依赖重建，原A2资产和选定历史证据走Drive，LightNav公开权重重新下载。新机器仍需实际验证CUDA/EGL及完整导航；迁移改动不增加runtime成功声明。
