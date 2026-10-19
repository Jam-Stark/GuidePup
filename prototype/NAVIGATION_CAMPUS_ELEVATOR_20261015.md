# 学校 MF-R1 默认任务：GPU3/GPU4 首次实际测试

2026-10-15。按Owner要求，使用GPU3加载LightNav，GPU4运行Isaac/本地GPU感知，提交一次默认任务“一层当前起点走到中间2号电梯门前，朝向电梯门并停下”。**任务在Qwen规划阶段进入NEEDS_USER，没有进入导航执行。** 新任务记录1，实际导航动作0，真实完成0；不能称为完整导航成功，也不能用本次评价LightNav、MPPI或到达性能。

## 执行与最小代码改动

证据根：`.local/outputs/campus_elevator_task_20261015_01/`；实际任务目录`navigation/`；原始传感器/物理记录`.local/outputs/isaac_sim_20261015_164436/`。

启动器`prototype/isaac/run_experiment.py`新增独立`--sim-gpu`和`--lightnav-gpu`选择。物理/导航服务继承sim GPU，LightNav子进程单独覆盖LIGHTNAV_GPU；仍通过现有gpu.sh/gpu.py按本机UUID绑定CUDA，Isaac显式使用对应物理renderer序号。状态记录各进程的分配。保留已有未提交工作，原启动器副本保存在证据根`before/`；没有改规划、语义Monitor、LightNav、MPPI、停车模型或场景。

```bash
.local/envs/sim/bin/python prototype/isaac/run_experiment.py \
  --sim-gpu 4 --lightnav-gpu 3 \
  --controller-footprint .local/outputs/campus_elevator_task_20261015_01/footprint_10cm.json \
  --output .local/outputs/campus_elevator_task_20261015_01/navigation \
  --wait-for-start
```

独立tmux：`guidepup-campus-task-20261015`。预览核对后只写入一次start_task，实际只有一个goal_changed/task UUID。未追加任务、自动重跑或修改任务文本。

- 默认校园场景/起点/几何、D435i前0.30m/高0.10m/俯5°、完整原生深度与RGB标定保持。
- 使用独立10cm期望净距footprint，不覆盖默认；机身/硬约束/速度/自由度不变。
- 沿用无全任务时间上限，每次决策30秒，最多3次恢复，无后拉/人工暂停/人工恢复。本轮0恢复。
- Qwen直连实验室endpoint，精确`Qwen3.6-35B-A3B-FP8/`，thinking off，输出2048，无鉴权；真实2次调用均返回工具调用，无HTTP/接口错误。Gemini未启用。
- 在线仍为RGB-D/IMU/关节及估计结果；目标坐标和物理真值仅初始化/离线评分。不执行电梯控制器或跨层移动。

## GPU与启动

GPU3/4启动前空闲；没有终止其他项目。预览实际显示校园一层绿墙、电梯门和玻璃外立面，RGB-D、tracking、局部地图正常。

| 组件 | 分配与证据 |
|---|---|
| Isaac/本地GPU感知 | GPU4，运行时CUDA UUID `20093912-98d6-3c89-9517-3ac344e38fc3`，renderer=4，多GPU关闭；原始configuration.json记录 |
| LightNav | 子进程LIGHTNAV_GPU=3，由gpu.sh转换为本机GPU3 UUID `ffc02ac2-e15e-00e3-f842-6f501cb0b6e5`；CUDA模型加载、约1.226秒预热和WS登录成功 |
| 原生MPPI/当前WBC | 现有CPU实现，不宣称运行于GPU4 |
| Qwen | 远端实验室模型，不占本机GPU |

本轮提前结束，未保留成功的LightNav存活期NVML快照；GPU3证据是启动器实际子进程选择、UUID绑定实现和CUDA加载日志，不能写成独立NVML确认。GPU4有运行时UUID和renderer记录。详见`gpu_bindings_runtime.json`。

## 为什么没有开始走

原任务：

> 从一层当前起点走到中间的2号电梯门前，朝向电梯门并停下。

两次plan_task均只返回一个clause：

> 从一层当前起点走到中间的2号电梯门前

遗漏的是整个“，朝向电梯门并停下。”，而非标点或空白。现有plan校验在去掉标点/空白后比较所有clause与完整原文；两个结果均不匹配。第二次请求已包含第一次的rejectedPlanReasons，但模型仍重复遗漏。固定两次规划后，框架进入NEEDS_USER；这不是用户没有说明目标，原请求已经明确。

两次实际英文instruction分别是：

1. `Go through the doorway into the main corridor and head towards the central elevator.`
2. `Navigate from current position towards the central elevator area.`

**这些只是被拒绝的Qwen提议，均未传入LightNav。** 两次kind均为pass_doorway，而请求终态是在电梯门外接近并停下；该语义不匹配值得后续修正，但不是日志记录的拒绝条件。现有规划接口两次imageCount均为0，不能声称模型通过画面认出了目标电梯。

记录定位：agent.jsonl seq10–13为首次请求/回应/提议/拒绝；seq15–18为第二次；task-monitor.ts现有plan分支在两次失败后调用help。没有task_planned、select_stage或action_executed。

## 独立结果

| 项目 | 本轮结果 |
|---|---:|
| 启动至评估/收尾 | 123.066秒 |
| 记录器任务窗口 | 16.410秒墙钟；物理1.9999999999999793–4.0199999999999365秒 |
| 实际导航动作/非零有效命令样本 | 0 / 0 |
| LightNav任务输入/输出/STOP | 0 / 0 / 0 |
| 原生MPPI求解/下发 | 0 / 0；原生日志2行仅初始化握手 |
| 物理轨迹累计 | 0.022641m，静止站立/收敛运动，不是导航进度 |
| 最终停稳 | 是，原0.3秒窗口；最大速度0.003966m/s、yaw率0.005433rad/s |
| 电梯目标完成 | 否；离目标平面前缘5.327m，横向偏差12.532m，朝向误差70.494° |
| Qwen完成声明/误完成 | 否 / 否 |
| 运行异常/HTTP错误 | 无；控制性NEEDS_USER，非仿真或优化错误 |

本轮没有几何碰撞拒绝、停车包络拒绝、候选无解、LightNav停止或恢复命令执行；这些分支都没有被测试。监督器PROCESS_FAILED是记录器受控非成功退出码2，不是Python崩溃；原始两项evaluator和results保留其区别。

任务窗口405个接触样本，非足部净法向力峰值0N，足部峰值114.605N。observer只给各机体净法向力，没有接触对象、接触点、摩擦力或逐接触法向，也包含自接触和可能的相消；0N不能证明全过程无碰撞，本轮更不能支持导航接触结论。离线短静止窗里程计RMSE3.71mm不代表运动定位精度。

## 证据与收尾

- `navigation/results.json`、`physical_evaluation.json`、`navigation_evidence.json`：两项独立评估和汇总。
- `navigation/episode/camera.mp4`、initial/final.jpg、agent.jsonl、tasks.sqlite、monitor_state.json、samples.jsonl：视频与完整Qwen输入/输出/状态。
- 原始raw目录：RGB-D、IMU/关节、ground_truth.jsonl、contacts.jsonl、telemetry.jsonl、配置及原生MPPI初始化。空LightNav/local_controller日志如实保存，不伪造执行记录。
- `analysis/planning_failure.json`、`analysis/QWEN_FLOW.md`、`analysis/analyze.py`：本次失败链路，未调用模型或重跑物理。
- `scope.json`、`preview_acceptance.json`、`gpu_bindings_runtime.json`、`resources_final.json`及独立supervisor receipt保留分配和清理证据。

本任务physics/navigation/LightNav/recorder及tmux均已退出，8050/8768/8770无监听；GPU3约15MiB、GPU4约3MiB，均0%利用率。GPU0他项目与8766 PID292116持续运行，未操作。无新训练、硬件、驱动重置、fallback或Git提交。

未修正Qwen规划接入或再次运行。后续应针对已保存的两次规划输出处理“完整任务保留与终态类型”问题；不能通过删除朝向/停下要求、代替Qwen编写导航指令或放宽物理完成条件制造成功。
