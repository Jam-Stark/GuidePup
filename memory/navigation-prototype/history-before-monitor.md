# Navigation prototype

历史快照：以下为常驻 Monitor 实现前的记录。旧 Agent / TaskSession / complete_navigation 和当时的服务状态已被替代；当前实现请读 description.md 与 prototype/MONITOR_TEST.md。保留此文件用于查找原实验和失败证据。

2026-09-14 HKT，当前是实验室Qwen + D435i RGB-D + 增强Depth翻译器 + A2尺寸约束MPC。最新已接入电梯大厅并完成一次接近/停车运行，Qwen测量引用仍有错误。入口 `prototype/README.md`，最新 `prototype/ELEVATOR_TEST.md`，此前3回合 `prototype/QWEN_FULL_TEST.md`，同帧三模型 `prototype/pi/DEPTH_COMPARISON.md`。先读实际源与最新日志，不把旧memory状态当运行状态。

- 最新场景 `elevator_lobby`：原生MJCF自建10×8m大厅，仅参考RMF双门宽1.7m，不是Hotel World转换；电梯闭门/显示/按钮、普通木门和边侧长凳/盆栽。`prototype/sim/scenes/elevator_lobby.xml`由A2装配，spawn[2.8,-1.6,.55]/yaw10°，D435i起点同时看见两种门。没有乘梯/开门/隐藏目标搜索。当前服务已reset此大厅IDLE；CLI默认仍procthor，重置电梯必须 `prototype/run.sh reset --scene elevator_lobby`。
- `prototype/sim/record_task.py --scene elevator_lobby --goal '找到电梯，走到电梯门口并停下。' --output <new directory> --timeout-s 180 --max-recoveries 3` 一次重置/真实Pi任务/5FPS视频/5ms接触/20mstelemetry；终态额外GET scene只供离线评价，不能当成模型读过。首轮plan/source/结果在 `.local/outputs/elevator_scene/`，未改旧policy、MPC或Pi提示。
- `elevator_01` 40.12墙钟秒/6.574m，全部非支撑contact0，MPC256次成功；正确走向电梯，1次closer实际追加.675m，终点[8.640,1.072,yaw.162]，最后1s平均平面速度.00612m/s。独立terminal帧371精确qpos渲染RGB/Depth一致，门板最近base水平距离1.229m/中位1.264m；不暗加1m验收。
- 本次Qwen两次ROI409：SSE267→scene283，随后正确283重测并继续；SSE336→scene350后没有补测就complete。完成理由1.269m真实来自350门板网格cell3/9/15，但写成status拍摄帧351的“测量”；无350/351成功ROI。目标识别/实际接近有证据，最终帧与测量类型引用错误。仍需解决调用依赖，不能把该数值写成完全编造，也不能用离线终帧补模型证据。

- 最新Owner要求恢复Qwen完整test：默认本就为实验室Qwen，3回合实际provider均核实；同配置预先登记，没有重跑挑选，源码副本/plan/model/raw/analysis/semantic在 `.local/outputs/qwen_full_test/`。生产源未在这轮改动。
- `navigation_01` 38.83墙钟秒/5.961m，`force_route_01` 39.87秒/6.932m，均无非支撑contact，基座和当前三圆包络均过门；Qwen报告COMPLETED不等于目标语义验收。前者有5次实测，但把混合背景距离概括为目标；后者最终单ROI混立柱/层板/墙，2.248m不能归为单一目标表面。
- 最新受力完整路线已经执行强后拉与恢复，取代“只在平地完成权限流程”的当前限制：−80N请求sim19.52，20.12锁存，20.14零命令，21.56放力仍锁存；问答22.54–23.50保持暂停，navigate409，human resume200，23.56解除，24.54 actual vx=.1127/effective .240m/s，随后过门。受力且仍NAV的29样本command .300、actual均值−.078m/s。零命令非位置锁定。
- 同一受力会话Qwen暂停问答没有任何新工具调用，却声称已穿门；此时base仍在起点附近，唯一已读图为门外。权限正确、视觉进度错误必须分别报告；现有提示不能保证先取新图。
- `metric_route_01` 40.27秒/5.476m，无非支撑contact，基座中心过门而包络未清门；4次current/delayed footprint净空.011/.006/.001/.005m < .015m，3次基本重复前进的恢复后NEEDS_USER/PAUSED。最后scene185.32s独立mask最近目标2.165m，距终态.58s/2.8mm，0个目标像素在登记的.9–1.1m；RGB/Depth精确姿态重建逐像素一致，仅离线，不送模型。
- 本次数值回合3个measure409原因是工具依赖：模型先生成status+scene+measure整批，测量参数用了SSE中的1754/1775/1795，scene实际返回1769/1789/1811（公共前缀2188200116-）；不是缓存淘汰。只有GET scene缓存帧；扩大8帧缓存不能纠正未读取scene结果就预填参数。最后真实pause无虚报complete，但没有具体向人解释失败。收尾reset默认ProcTHOR/D435i为IDLE，服务保留。

- 工作树 `/home/baoquanc/workspace/ForceControl-lightnav`，分支 `codex/a2-lightnav-prototype`，base `4605004`。原 `ForceControl` 工作树有新版未提交工作，不是本任务写入目标。未commit/push。
- 旧run `2026-05-23_14-25-38_A2_base_only_v2/model_59999.pt`，复用 `mujoco_exported/policy.pt` + JSON。46×32 term-major历史，12关节动作，6D旧辅助输出（velocity/base force），不是新版9D扶手wrench。未训练/重导出新policy。
- Scope仅局部语义导航、force交互和交流；不引入人体位置/人体包络。没有Isaac Lab原型或实机部署。
- `.local` → `/mnt/ssd1/baoquanc/forcecontrol-lightnav`。只用GPU0；1–3属于其他任务。LightNav8050、仿真8766，专用tmux `forcecontrol-lightnav-serve` / `forcecontrol-lightnav-sim`。
- 所有下载直连：清除大小写HTTP/HTTPS/ALL proxy，并设NO_PROXY=*，包括tmux继承环境。不得下载本地Qwen；旧本地权重/专用环境/缓存约8.07GiB已按Owner要求清理。
- 默认实验室URL `http://vic.precognition.team:8888/v1`，模型ID `Qwen3.6-35B-A3B-FP8/`（保留末尾斜杠），无鉴权、262144上下文。Pi0.85.1/Node22.23.2。备选ZenMux仍可用，凭证仅在未跟踪 `.local/secrets/zenmux.env`，不得复制到memory；旧对照见 `prototype/pi/ZENMUX_RESULTS.md`。
- `prototype/run.sh build/up/reset/status/run/chat/down` 为统一入口。场景现有flat/procthor/elevator_lobby；无scene参数的reset恢复ProcTHOR/D435i，不会保留电梯选择。up/reset的其余参数传给sim，run/chat其余参数传Pi。
- 当前相机 `d435i_rgb_nominal`：640×360、69°×42.2726°，相对base[.30,0,.10]m、下俯5°、目标10Hz、有效光轴深度.2–8m。同视角理想RGB-D，不是原生更宽D435i深度FOV或实机标定/误差证明。旧A2头部profile保留历史资料，当前RGB-D服务只接受有depth配置的profile。
- LightNav只收原始RGB，返回局部路径按同帧base位姿转world。实机需要里程计。Pi不接收目标世界坐标或路线剩余距离。
- Depth翻译器给原始RGB+6×3编号图、各格分离距离带/有效比例；`measure_region(frame_id,bbox_px)`只测缓存同帧半开像素ROI，返回全图青色选框、真实裁图、RGB/距离层像素并排图。HTTP与Pi统一images[{kind,label,image}]，移除旧单image；Pi按label+image发送，统计不重复含base64。各距离带有D编号、颜色、像素bbox与真实代表像素，未知灰色；同色不等于同物体。距离统一base水平hypot(x,y)，不是光轴Z/前缘净空。p10/p90是中央80%像素分布，不是置信区间。
- 新MPC源 `prototype/control/`：3个r=.30m圆，base x=-.30/0/.30，覆盖259个记录行走姿态的身体/摆腿，不代表所有关节极限。4cm障碍体素加半对角膨胀；15×.15s预测，全局速度上限.3m/s、.6rad/s。完整局部占用点集核验；只支持前进/转向。
- MPC使用实际位姿/平面twist、上一求解实际sim耗时的旧指令前缀、tau=.15s速度响应。向0减速可立即下达，继续限制加速/反向起步；实际响应仍由模型预测。目标10Hz，实际耗时写入mpc.jsonl，不宣称严格实时或所有外力可预测。
- 深度地图保留4m内已观测占用，不代表已验证自由空间。已移除误删近身沙发的.6m“自身方盒”；当前前装/有效近距视锥的最小base x约.492m，超出已记录机器人前端.394m。
- MuJoCo RGB-D必须关闭quality.offsamples：精确姿态重建证实多重采样门框边缘可生成19–21cm假点；offsamples=0后触发体素消失，对应射线误差约1–4mm。离线mj_ray仅作诊断，没有用场景标签在线过滤。
- Pi六工具：status/scene/measure_region/navigate/pause/complete_navigation。每个任务事件或人工问题只成功读一次status/scene，Pi原生prepareNextTurnWithContext移除已读工具；同批重复读不执行I/O；一个成功动作后等待SSE。源于真实177次scene的上下文溢出，不是无限prompt重试。
- 人的force brake高于导航：旧auxFx≤−.6连续2tick且requested vx≥.2；NAV不受旧遥控abs(wz)≤.1限制，人工速度模式仍保留它。HUMAN_PAUSED锁存，只有人工resume，暂停仍有policy/PD站立力矩；目前不是全方向制动或机械静止保证。
- `integrated_05`：32.92墙钟秒，5.618m路程，基座在sim23.485s/y11.2493m过门；5ms接触记录无非地面接触。基线5.801m有1.365s沙发接触。这是单episode描述，不是成功率。MPC求解p50/p90/p99为55.8/77.2/101.7ms。
- 历史受力路线 `force_01` 在−40 N后拉时command vx均值.300 m/s、actual .0744，auxFx最低−.4519未锁存；随后MPC几何约束暂停，路程1.054m未过门，−80 N/人工恢复未执行。历史独立平地 `force_authority_flat` 完成权限流程。保留旧失败，不以最新一次完整受力路线覆盖它；详情 `prototype/RGBD_MPC_RESULTS.md`。
- Qwen的COMPLETED不可信作独立验收：05把最右列墙/地板ROI称作置物架。初始探针也有地板/墙误选；端点只读测量选到包含家具的区域并否定约1m，但混有多表面。服务没有目标语义或数值距离硬门槛，没有重引入历史默认1m格点gate。
- Owner追问Depth翻译器归因后已实现像素可视化。18回合同帧对照：far/doorway/near各3模型×旧/增强表达，相同prompt、auto工具、最多4ROI/150s/4096回复tokens，Qwen thinking off、ZenMux low。45个成功ROI逐层像素数均与独立mask/距离参考一致，另1Gemini越界400后纠正。没有重跑模型，目标mask仅离线评价。
- 对照结论：Gemini增强版最终ROI三帧均100%目标，距离7.406/2.337/1.973m；旧版最终距离也有目标像素支持。DeepSeek增强版三帧有目标距离支持，但far/doorway目标比例87.83%/96.37%，不能采信“完全无背景”；旧doorway4ROI后仅DSML调用文本，无最终判断。Qwen旧doorway/增强near未测量却声称实测；增强far4ROI目标皆0%，增强doorway采用仅0.98%目标像素的D6层。新增图示没有解决Qwen的缺测与语义绑定。
- 对照是单场景三帧、全为1m负例，17条no+1条无最终判断，不证明动态到达/1m正例/实机准确。增强回合中位墙钟Qwen4.540s、Gemini20.612s、DeepSeek36.897s；Qwen含缺测不能当速度优势。当前默认Qwen未自动切换；Gemini是下一轮动态核验候选，不是已验证可靠裁判。
- `prototype/sim/depth_replay.py --manifest .../reference/case_manifest.json`为只读8767重放；`prototype/pi/experiments/compare.sh --cases far,doorway,near --prompt-file .../prompt.txt --output <new>`复用三个实际provider。原始输出/plan/输入prompt/源码副本/独立evaluation在 `.local/outputs/depth_translator_compare/`。收尾停止专用对照tmux，现有LightNav/仿真保持运行。
- 上一轮MPC集成失败在 `.local/outputs/rgbd_body_mpc/` 保留。`qwen_readonly_crop`原结构化日志曾被端点probe误覆写；端点数据已移至独立目录，原console/ROI/传感帧仍在，图片从原帧重建，归档限制在其RESULT.md明确记录。
