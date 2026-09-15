# 本机运行记录

2026-09-14。以下是功能接通的实际记录，不是跨场景成功率评测。

## 最新：开阔电梯大厅

新增原生MuJoCo `elevator_lobby`，10×8m自定布局，金属闭合双门、楼层显示和呼梯按钮，旁置普通木门干扰。只参考RMF的1.7m门宽，不是Hotel World原始转换。旧A2、D435i、LightNav、MPC、Qwen/Pi提示和控制权限未改；增加 `prototype/sim/record_task.py` 实际录像入口。

预登记一次 `elevator_01` 目标“找到电梯，走到电梯门口并停下。”40.12墙钟秒，实际路程6.574m，全部非支撑contact为0，256次MPC求解成功。一次closer指令实际追加约.675m，终点在电梯双门前、独立可见门板最近base水平距离1.229m；末1秒平均平面速度.00612m/s。没有暗加1m要求，起点电梯可见，不代表场景外搜索或实机能力。

两次measure帧错配409保留：第一次267/283错配后重测283成功；第二次336/350错配后未补测。completion中的1.269m确来自scene350门板网格，但模型写成status帧351的测量，最终ROI引用链仍不成立。模型识别/实际接近与测量引用分别评价。见[完整报告、视频与轨迹](ELEVATOR_TEST.md)。收尾已reset电梯大厅为IDLE，当前服务保留此场景。

## 此前：Qwen完整任务测试

保持默认实验室Qwen，本轮精确调用 ID 为 `Qwen3.6-35B-A3B-FP8/`。固定现有实现完成预先登记的3个回合，没有重跑挑选结果。普通导航38.83墙钟秒、5.961m，受力完整路线39.87秒、6.932m，两者基座及当前三圆身体包络均过门；3个回合5ms接触记录中全部非支撑contact为0。

受力回合在同一个Pi任务中实际执行−80N后拉、0.60sim秒锁存、下一20ms tick命令归零、放力保持、暂停交流、navigate409、人工resume200和恢复后继续过门。但Qwen问答没有读新status/scene，错误声称当时已经穿门；最终测量仍混目标和背景，模型COMPLETED不代表可靠语义验收。

约1米回合40.27秒、5.476m，基座中心过门但身体包络未清门，4次MPC净空阻塞、3次恢复后NEEDS_USER/PAUSED。Qwen将SSE帧号预填进与scene同批的measure，3次409；不是scene缓存被淘汰。最后可见目标的离线最近base水平距离2.165m，未满足登记的0.9–1.1m条件。详见[完整报告与三段视频](QWEN_FULL_TEST.md)。生产代码未在本轮改动，收尾已reset默认ProcTHOR/D435i为IDLE。

## 此前：增强Depth翻译器与三模型对照

生产Pi/MuJoCo已接入原始RGB/编号图、全图ROI选框、裁图与距离层像素图，TypeScript编译和实际HTTP/CLI接口通过。同一组三张RGB-D帧，Qwen/Gemini/DeepSeek分别使用旧/增强表达，共18回合；没有重跑挑选结果。46次ROI请求中45次成功、1次越界后纠正，逐层像素计数与独立参考完全一致。

Gemini增强版三帧最终ROI目标比例均100%，所报距离分别7.406/2.337/1.973m。DeepSeek增强版三帧取得目标距离，但far/doorway仍混入背景；旧表达doorway在4次测量后没有最终回答。Qwen旧doorway/增强near两条没有ROI测量却声称已测量，增强far全部测量未含目标，增强doorway采用了主要为背景的距离层。增强图示未解决Qwen的可靠性问题。

17条最终距离判断均no，另1条没有最终判断；三张图全为1m负例，没有动态停车、1m正例或实机验证。增强版每回合墙钟中位数为Qwen4.540s、Gemini20.612s、DeepSeek36.897s，Qwen包含缺测回合，不能据此认为完成测量更快。详见[正式对照报告](pi/DEPTH_COMPARISON.md)。

## 此前：Qwen + D435i RGB-D + A2尺寸约束MPC

按Owner新选择恢复实验室Qwen和D435i；旧checkpoint不变。已接入6×3编号RGB与分层深度、同帧ROI测量及裁图，LightNav仍只接收原始RGB。MPC增加A2身体/摆腿包络、深度局部障碍、真实平面速度、求解延迟与速度响应；制动权限保持人工优先。

`integrated_05` 在32.92墙钟秒内行驶5.618m，基座于sim23.485s从门洞内y=11.2493m穿门，5ms接触记录中非地面接触为0。旧无障碍MPC基线行驶5.801m，有累计1.365s沙发正力接触。两者都是单次描述性运行，不能推断成功率；基座过门也不等于整个机器人已清门。

Qwen报告COMPLETED，但把右侧墙面和地板ROI当成目标表面，完成理由仍不可靠。服务没有目标语义或数值距离硬验收，不把这次标记当作可靠任务完成。初版早停、读图风暴、时延/转向制动问题和仿真MSAA深度假点均保留，见[完整本轮记录](RGBD_MPC_RESULTS.md)、[视频](../.local/outputs/rgbd_body_mpc/integrated_05/camera.mp4)、[接触统计](../.local/outputs/rgbd_body_mpc/integrated_05/contact_summary.md)。

受力完整场景回合在−40 N后拉后被MPC暂停，未完成路线，也未执行后续强制动环节；期间指令vx均值0.300 m/s、实际0.0744 m/s。独立平地回合使用同一LightNav/MPC/旧policy（无Pi），−80 N请求至制动锁存0.60 sim秒，下一20 ms tick命令归零；放力仍锁存，navigate返回409，人工恢复后实际vx达到0.148 m/s。两次均无非地面接触。见[受力路线](../.local/outputs/rgbd_body_mpc/force_01/contact_summary.md)、[平地权限证据](../.local/outputs/rgbd_body_mpc/force_authority_flat/contact_summary.md)。当前服务已重置为默认ProcTHOR/D435i初始场景。

以下是此前版本的历史记录；其中“当前/本轮”均指各自当时的版本。

## 历史：ZenMux Gemini / DeepSeek

Owner 提供接口后，已通过同一 Pi 接入 `google/gemini-3.8-flash` 与
`deepseek/deepseek-v4.1-flash`。RGB/工具回合均通过，数值约1米任务均暂停；各一次
语义导航分别41.35秒与46.31秒，报告COMPLETED，离线base轨迹均证实过门。
DeepSeek thinking + required 首次400，改为auto后通过；失败原样保留。完成理由
仍含不充分推断，不能宣称已可靠核验或无碰撞。见 [ZenMux实跑记录](pi/ZENMUX_RESULTS.md)。

## 历史：头部相机、纯 RGB 与能力复核

Owner 查看视频后指出碰撞及仿真距离依赖。本轮以旧 checkpoint 和已有模型服务为基础，
将相机改为 `a2_head_isaaclab`：768×272、132°×77.0025°、15 Hz 目标采样、水平朝前、
位置 `[0.3381,0.0336,0.0525]`，裁剪0.1–20m。位置/方向与 Unitree 官方 camera_joint 一致；
视野、频率来自 Owner 的仿真参数，提升后的分辨率不是官方实机规格。详见 [相机资料](camera/README.md)。

LightNav 始终仅收 RGB；本轮也移除了 Pi 的深度网格、目标采样距离和默认1米数值检查。
两个模型均没有获得目标世界坐标或真实剩余距离；机器人位姿仍来自仿真，实机需要里程计。
新增 `--record-contacts` 在每个5ms物理步记录接触，仅供离线分析，不进入模型或控制。

**`head_rgb_only_01`：30.732秒墙钟时间，模型报告 COMPLETED，但存在物理碰撞。**
目标为“穿过右前方的敞开门，走到门内白色置物架前，然后停下”，没有自行加入距离条件，
没有注入人工外力。base 确实穿过 x=11.033m 门平面，交点 y=11.184m 在开口内；终点base
`[11.691,10.456,yaw=-1.015]`，最低高度0.509m。Qwen认为已靠近家具；该结论没有数值距离核验。
本轮同时改变了相机、Pi观测和施力条件，不能把时长差异当作相机改善的因果证据。

| 接触对象 | 仿真时间窗口 | 实际接触部位与单点法向峰值 |
| --- | --- | --- |
| 沙发 Sofa_207_2 | 17.725–18.350s、20.400–20.505s，窗口内为间歇接触 | 右前脚/大腿、右后大腿；最高186.7N |
| 门框 Doorframe_4 | 33.635–33.840s，窗口内为间歇接触 | 右前大腿/脚、左后大腿；最高658.6N |

这些来自 `mj_contactForce` 的正法向力，排除了正常足底-地面支撑，不是仅凭画面或ncon判断；
数值是 MuJoCo 接触求解器输出，不等于实机冲击力。
运行时间窗是sim11.20–39.42s，之前是场景站立等待；上述接触时间不能直接当作视频秒数。

[新相机视频](../.local/outputs/head_rgb_only_01/camera.mp4)、
[终点图](../.local/outputs/head_rgb_only_01/final.jpg)、
[任务结果](../.local/outputs/head_rgb_only_01/summary.json)、
[接触分类](../.local/outputs/collision_diagnosis/head_rgb_only_01_contacts.json)。
[接触诊断报告](../.local/outputs/collision_diagnosis/head_rgb_only_01_REPORT.md)进一步定位了门框两侧的接触与LightNav轨迹上下文。
原始仿真记录在 `.local/outputs/sim_20260914_164607/`，包含 contacts、geometry、telemetry和LightNav回复。

旧 `build_force_08` 的sim32.5–40.5s也有强受阻迹象：请求vx=0.3，实际均值0.00268m/s，
base卡在门前约0.4m，人工外力为0。旧日志没有接触对/关节状态，不能回溯确认具体碰撞部位。
其间LightNav的中心路径仍穿过门洞。已确认的集成缺口是官方MPC和当前调用没有障碍物、
A2完整尺寸或摆腿空间约束；中心路径可通不等于四足全身可通。当前证据不能将全部原因
归给LightNav，也没有证明仅换相机就能解决。旧窗口证据见
[碰撞诊断](../.local/outputs/collision_diagnosis/REPORT.md)。

**`head_rgb_metric_request`：数值条件失败，30秒后 LIMIT_REACHED / PAUSED。**
在上一目标结束后提交“在距离眼前白色置物架前表面约1米处停下”。尽管系统上下文明确说明
没有测距、不能核验数值要求，Qwen仍两次尝试用“细节清晰／视觉接近”确认约1米，实际工具
因非ARRIVAL_CANDIDATE状态返回409；之后发生3次有效navigate，最终超时。两次错误完成
没有被接受，但原因是状态约束，并不是系统拥有距离核验能力。原始
[模型工具记录](../.local/outputs/head_rgb_metric_request_summary.json)保留了这项失败。

因此，当前 Qwen 可用作需求转换、图像说明和局部纠正的助手，不能独立担任可靠的最终到达裁判；
LightNav stop 也只是候选。旧1.149m结果依赖理想深度，不能证明RGB完成能力，更不是路线剩余距离。
实机若需要可核验距离和碰撞约束，应接入实际雷达/深度等观测；若保留纯RGB，则完成只能作为
视觉候选并交给人确认。本轮没有实现新的几何感知、避障或人工完成确认流程，也没有操作实机。

## 历史：深度辅助版本的完整软件链路

统一入口 `prototype/run.sh build/up/run/chat/reset/status/down` 已实现，Pi 类型编译通过。
当前使用旧 A2 6D 导出、官方 LightNav HF/SDPA、Pi 0.85.1 和实验室
`Qwen3.6-35B-A3B-FP8/`。没有更换 checkpoint、启动本地 Qwen 或运行实机。

最终完整功能运行 `build_force_08` 的原目标为：
“穿过右前方的敞开门，走到门内白色置物架前，在距离置物架前表面约1米处停下”。
从初始场景到 `COMPLETED` 用时 **57.407 秒墙钟时间**，包含受力、人工暂停和恢复。
施力与恢复由演示脚本通过人工接口执行；恢复不是 Qwen 发出的。

| 已发生的行为 | 实际证据 |
| --- | --- |
| Qwen 理解中文目标并调用工具 | 同一 Pi Agent 保留原目标及起点图，首次导航冻结 1.0 ± 0.2 m 距离条件 |
| LightNav → MPC → 旧 A2 | 真实 RGB 推理、按拍摄位姿变换路径、使用实际 base pose 跟踪；没有使用固定路径代替 LightNav |
| −40 N 后拉期间出现速度偏差 | 施力后段平均请求 vx=0.300 m/s，实际 vx=0.028 m/s；这是单次运行中的观测，不是受控因果比较 |
| −80 N 后拉锁存制动 | 施力到锁存 0.42 s 仿真时间；锁存后下一 0.02 s 控制步的有效命令归零 |
| 前进转弯仍可制动 | 锁存前请求 wz=0.241 rad/s；旧遥控的 0.10 rad/s 资格限制会漏掉这次后拉，NAV 已移除该角速度限制 |
| 人的权限高于导航 | 释放外力后仍为 HUMAN_PAUSED，实际 vx≈−0.008 m/s；人工 resume 后才继续原目标 |
| 真实穿门 | base 轨迹穿过 x=11.033 m 门平面，交点 y=11.351 m 在场景墙洞范围内；几何仅用于离线核对，没有提供给模型 |
| 到达核验与自动恢复 | 发生 1 次实际局部导航纠正；距离检查先拒绝 1.841 m、1.757 m，最终接受目标表面采样点的 1.149 m |
| 结束状态 | COMPLETED；base≈[12.096,10.869] m，最终实际 vx≈−0.001 m/s；最低 base 高度 0.502 m，未观察到倒地 |

原始证据位于 `.local/outputs/build_force_08/`：
[完整相机视频](../.local/outputs/build_force_08/camera.mp4)、
[运行结果](../.local/outputs/build_force_08/summary.json)、
[受力和穿门测量](../.local/outputs/build_force_08/measurements.json)、
[Pi 原始事件](../.local/outputs/build_force_08/agent.jsonl)、
[终点图像](../.local/outputs/build_force_08/final.jpg)。
该次仿真原始控制与 LightNav 轨迹在 `.local/outputs/sim_20260914_161257/`。

上述 1.149 m 是代码读取同帧 `base_planar_range_m[0][6]` 的值。
Qwen 的完成理由把光轴深度约 0.81 m 错写成了 base 水平距离；该文字没有参与数值通过判定。
终点图中选中的格点落在目标家具前侧立柱上。稀疏格点靠近细边缘时会随姿态变化切换到背景，
所以前两次拒绝值到最后通过值不能解释成连续平滑的接近距离。当前实现验证采样点的数值条件，
目标语义和像素选择仍由模型负责；不能据此宣称通用到达判定已经可靠。

暂停问答也已使用实际模型运行：人工暂停后询问“现在为什么不动？你看到什么？保持暂停，不要恢复运动。”
Pi 实际调用 `status` 和 `scene`，读取当前图像并回答，没有调用运动或完成工具，结束仍是 HUMAN_PAUSED。
原始记录为 `.local/outputs/paused_chat_02.jsonl`，具体结果见
[暂停问答记录](../.local/outputs/paused_chat_02_summary.json)。

### 本轮实际故障与修正

- `build_doorway_01/02`：模型将看见家具误当接近，RGB 也不能可靠估计距离；保留完整原目标并加入同帧理想深度。
- `build_force_04/05`：MPC 转向增大导致旧 RC 条件漏制动，以及 Qwen 自行放宽用户距离；调整 NAV 制动资格，任务层冻结并计算距离条件。
- `build_force_06`：模型只写工具计划而未调用，系统如实停在 NEEDS_USER；自动任务回合改用 Pi 原生 `toolChoice: required`。
- `build_force_07`：一个回合连续导航替换了尚未执行的路径；每回合一个成功动作，之后等待仿真事件。
- 暂停问答首轮没有取图并把停车说成电机断力；上下文补充实际 policy/PD 站立机制，并要求先读取 status/scene，第二轮实际读图且保留暂停。

这些是逐项修复期间的功能运行，不是固定配置下的成功率统计。当前验证使用单一 ProcTHOR 场景、
D435i 名义 RGB 视角和 MuJoCo 理想深度。真实相机取流/标定、Isaac Lab 对照以及实机接入尚未运行。

## 初始连接记录（完整任务实现之前）

| 项目 | 实际状态 | 证据 |
| --- | --- | --- |
| 隔离工作树 | 已创建，后续读写在新目录 | `codex/a2-lightnav-prototype`，base `4605004` |
| 旧 checkpoint | 已复制原 6D 导出与 metadata | `.local/assets/a2-old/provenance.json` |
| MuJoCo 环境 | 已安装并运行 | torch 2.10.0、MuJoCo 3.6.0，新增依赖独立 venv |
| 旧 policy 前进 | 10 s 演示完成，未观察到倒地 | `.local/outputs/a2_baseline_initial.json` |
| 外力顺应 | 0.3 m/s 请求下，−80 N 后出现减速/后退 | `.local/outputs/a2_force_initial.json` |
| force brake 权限 | 已真实施力触发，导航不能解锁，人工可恢复 | `.local/outputs/force_brake_http.json` |
| A2 相机 | ProcTHOR 场景真实渲染成功 | `.local/outputs/a2_camera.jpg` |
| Pi 安装 | Node 22.23.2、Pi 0.85.1、TypeScript 编译/CLI 已运行 | `prototype/pi/` |
| Pi → 仿真 | 实际读取 status 与 JPEG 成功 | `.local/outputs/pi_scene.jpg` |
| 官方 MPC → 旧 A2 | 固定 2 m 直线路径已运行，含一组 0 / −40 N 对比 | `.local/outputs/mpc_straight_summary.json` |
| LightNav | 官方权重已完整下载，真实 A2 首帧推理通过，服务监听 8050 | `.local/cache/lightnav/a2_image_prediction.json` |
| 实验室 Qwen | `/v1/models` 及真实 Pi 图像/工具回合均已成功 | `http://vic.precognition.team:8888/v1`，`Qwen3.6-35B-A3B-FP8/` |
| LightNav → MPC → A2 | 已实际闭环转向、前进并进入候选到达；穿门语义成功未确认 | `.local/outputs/lightnav_closed_loop{,_summary}.json`、同名 JPEG |
| Qwen → Pi 工具调用 | status、scene 各一次成功，未发送运动操作 | `.local/outputs/lab_qwen_pi.json`、同名 log |

在一次 ProcTHOR 前进施力演示中，外力施加于仿真 `t=66.56 s`，制动锁存于 `t=67.16 s`，下一控制步有效指令归零。0.60 s 包含估计响应时间，不能将“连续 2 tick”直接宣称为 40 ms 总制动延迟。释放外力后，`HUMAN_PAUSED` 仍保持；新 navigate 返回 409，人工 resume 后进入 IDLE。

运行日志记录 `requested_cmd`、`effective_cmd`、真实 base 位姿/速度、辅助估计原始值、实际注入外力与任务状态。没有将 ground-truth force 冒充为 policy 力估计。

官方 MPC 的独立连接演示使用固定 2 m 直线路径，目标速度上限 0.3 m/s，两个 12 s 仿真 episode；第二个在 t=3–5 s 施加 −40 N。在 t=4–5 s，MPC 平均请求均为 0.3 m/s，实际前向均速分别为 0.266 和 0.078 m/s；t=5 s 的 x 进度分别为 1.209 和 0.827 m，t=12 s 分别达到 2.033 和 2.031 m。这支持“按真实位姿跟踪可以容纳这次受力造成的速度偏差”，尚未覆盖 LightNav 的视觉历史、弯道或持续牵拉。这两个 episode 只连接 MPC 和 policy，不包含任务服务的制动状态机；制动链路的证据来自前述 HTTP 施力演示。

LightNav 官方 HF/SDPA 服务首帧推理返回 10 个路径点，`stop=false`、`apos_state=rot_right`；服务端 353.84 ms，客户端 408.63 ms。GPU0 驻留 8972 MiB，启动与首帧期间每 200 ms 采样的观测峰值同为 8972 MiB；不是精确 allocator 瞬时峰值，长期历史的峰值尚需另测。

真实闭环指令是 `Turn right and go through the doorway.`。A2 从 `[6.933, 13.721, yaw=0.656]` 运动至候选到达时的 `[7.645, 13.393, yaw=-0.466]`，用时 8.20 s 仿真时间 / 8.50 s 墙钟时间，导航期间最低 base 高度 0.507 m。后续观察至 20 s，保存相机图后人工暂停。图像仍显示前方门口，当前证据不能确认已完成穿门；没有调用 `/complete`。这次结果证明软件/动态链路已通，同时保留完成判定问题给 Pi 的图像核验。

实验室 Qwen 的真实 Pi 回合随后读取了 status 和 scene，各一次。首模型流式增量 342 ms，首文字 1012 ms，总回合 3648 ms（包含两次工具执行及模型的后续请求）。模型结论是“当前图像不能确认已经完全穿门”，没有把 `arrival_candidate=true` 直接当作完成，也没有发送运动请求。该次输出仍包含对门框位置和机器人本体位置的推断，不能据此宣称完成判定已经可靠；本轮证明模型、图片和工具连接可用，自动恢复与完成判定质量留待后续任务运行。

Owner 指出代理后，确认旧下载继承了 tmux 的 `127.0.0.1:18889` 代理。LightNav 恰在检查时完成了原下载，没有重新下载；其后所有下载入口均已改为显式直连。Owner 随后提供实验室 Qwen 服务，本地 Qwen 下载和待启动任务已全部停止，废弃权重、缓存和专用环境按要求清理，记录在 `.local/outputs/qwen_local_cleanup.json`。

清理的本地 Qwen 数据共约 8.07 GiB：专用 vLLM 环境、未完成模型文件和下载/安装缓存。废弃本地部署脚本同时移除。当前工作依赖 LightNav、Pi、MuJoCo 和旧 A2 资源；没有本机 Qwen 下载或服务进程。

保留的本机长进程是 `forcecontrol-lightnav-sim` 和 `forcecontrol-lightnav-serve`。Qwen 使用实验室服务；本机只在 GPU0 运行 LightNav 和相机渲染，没有占用或终止 GPU1–3 的任务。
