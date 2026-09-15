# Done

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
