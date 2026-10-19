# 完整深度视场、停车余量与独立闭环对照

2026-09-16。顺序遵循 Owner 要求：完整原生深度视场 → 停车动态和提前余量 → 可核验恢复 → 三项分别闭环对照。

## 结果

|运行|真实目标达成|Monitor|实际路径 m|生效 MPC／其中 LightNav|运行错误|
|---|---|---|---:|---:|---|
|仅完整深度视场|False|NEEDS_USER|2.105|116／116|False|
|停车／地图／恢复修正基线|False|NEEDS_USER|3.712|133／133|False|
|仅期望净距 10 cm|False|NEEDS_USER|3.835|139／139|False|
|仅居中语言|False|NEEDS_USER|2.996|122／122|False|
|后移 30 cm：运行错误|False|ERROR|0.016|0／0|True|
|仅相机后移 5 cm|False|NEEDS_USER|1.003|52／52|False|

每组均为一次实际新运行。六组标定、冻结输入求解、旧记录回放均不计导航成功。四组修正后运行各自使用新地图、估计器和模型会话；单例并非统计显著性或严格冻结模型前缀的因果试验。

相机后移 30 cm 的首试因机身遮挡、没有可测地面而发生运行错误，没有 MPC 控制；保留原始 Monitor NEEDS_USER 和控制 ERROR 的不同记录，综合分类为 ERROR。随后只改为后移 5 cm，实际预览确认仍能建立传感器地图，再执行一轮。

## 分组原因与证据

### 仅完整深度视场

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/depth_only_01`。
- 原 Monitor 原因：同一观察的目标选区三次未取得可核验证据。 需要帮助：帮助确认目标位置或提供更清晰视角。
- 首次控制受阻：{'sim_time_s': 18.5200000000006, 'reason': 'Current/delayed A2 footprint violates hard clearance: -0.114 m < 0.016 m.'}
- 几何恢复候选选择 0 次，可证实路径 0 次；终态：`None`。
- 独立位姿评分 RMSE：0.1230 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 5 次；LightNav 30 次，接受 29 次、STOP 0 次。
- 实际配置：`{'desired_clearance_m': 0.03, 'camera_position_base_m': [0.3, 0.0, 0.1], 'lightnav_instruction_prefix': '', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

### 停车／地图／恢复修正基线

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/integrated_baseline_01`。
- 原 Monitor 原因：同一观察的目标选区三次未取得可核验证据。 需要帮助：帮助确认目标位置或提供更清晰视角。
- 首次控制受阻：无
- 几何恢复候选选择 0 次，可证实路径 0 次；终态：`None`。
- 独立位姿评分 RMSE：0.1428 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 6 次；LightNav 47 次，接受 47 次、STOP 2 次。
- 实际配置：`{'desired_clearance_m': 0.03, 'camera_position_base_m': [0.3, 0.0, 0.1], 'lightnav_instruction_prefix': '', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

### 仅期望净距 10 cm

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/clearance_10cm_01`。
- 原 Monitor 原因：当前观测到的两个边界区域（左侧B2和右侧B1）在物理上属于同一墙体结构，它们之间存在约1.2米的间隙，但RGB证据显示这是一面完整的墙体（有灰色平整墙面和深色纹理墙面），并未显示出中间存在一个‘完全打开的门口’。由于缺少中间自由通道的RGB视觉证据，且LightNav的stop信号未与明确的开口边界证据对齐，系统无法确认此处为可用开口。 需要帮助：需要人工确认：1. 当前看到的墙体间隙（B1与B2之间）是否就是任务所指的‘右侧完全打开的门口’？2. 或者是否存在另一个被遮挡的门口，需要改变搜索策略？
- 首次控制受阻：无
- 几何恢复候选选择 0 次，可证实路径 0 次；终态：`None`。
- 独立位姿评分 RMSE：0.1174 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 6 次；LightNav 49 次，接受 49 次、STOP 2 次。
- 实际配置：`{'desired_clearance_m': 0.1, 'camera_position_base_m': [0.3, 0.0, 0.1], 'lightnav_instruction_prefix': '', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

### 仅居中语言

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/language_center_01`。
- 原 Monitor 原因：当前传感器地图无法证实局部脱离路径同时满足完整机身空间、停车余量和净距增益。 需要帮助：需要新的有效空间观测或人工调整位置，不能放宽门槛强行转动。
- 首次控制受阻：{'sim_time_s': 22.79999999999975, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.003 m (model native_hospital_sensor_stop_20260916).'}
- 几何恢复候选选择 1 次，可证实路径 0 次；终态：`{'state': 'unavailable', 'candidates': [{'name': 'forward', 'reference_end_odom': [0.32952424445482925, -2.685785809292888, -1.6036526188126978], 'start_clearance_m': 0.07846771017930772, 'accepted': False, 'minimum_swept_clearance_m': None, 'stop_reserve_m': None, 'end_clearance_m': None, 'predicted_gain_m': None, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.026 m (model native_hospital_sensor_stop_20260916).'}, {'name': 'arc_left', 'reference_end_odom': [0.4356405118791757, -2.656846958451549, -0.7036526188126978], 'start_clearance_m': 0.07846771017930772, 'accepted': False, 'minimum_swept_clearance_m': None, 'stop_reserve_m': None, 'end_clearance_m': None, 'predicted_gain_m': None, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.026 m (model native_hospital_sensor_stop_20260916).'}, {'name': 'arc_right', 'reference_end_odom': [0.2255372856090966, -2.649941260333408, -2.5036526188126977], 'start_clearance_m': 0.07846771017930772, 'accepted': False, 'minimum_swept_clearance_m': None, 'stop_reserve_m': None, 'end_clearance_m': None, 'predicted_gain_m': None, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.026 m (model native_hospital_sensor_stop_20260916).'}, {'name': 'turn_left', 'reference_end_odom': [0.337736839641233, -2.435920739144409, -0.7036526188126978], 'start_clearance_m': 0.07846771017930772, 'accepted': False, 'minimum_swept_clearance_m': None, 'stop_reserve_m': None, 'end_clearance_m': None, 'predicted_gain_m': None, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.026 m (model native_hospital_sensor_stop_20260916).'}, {'name': 'turn_right', 'reference_end_odom': [0.337736839641233, -2.435920739144409, -2.5036526188126977], 'start_clearance_m': 0.07846771017930772, 'accepted': False, 'minimum_swept_clearance_m': None, 'stop_reserve_m': None, 'end_clearance_m': None, 'predicted_gain_m': None, 'reason': 'Current/delayed state has insufficient calibrated stopping reserve: -0.026 m (model native_hospital_sensor_stop_20260916).'}], 'reason': 'No bounded escape has observed free swept space, stopping reserve and clearance gain.'}`。
- 独立位姿评分 RMSE：0.0909 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 1 次；LightNav 44 次，接受 42 次、STOP 0 次。
- 实际配置：`{'desired_clearance_m': 0.03, 'camera_position_base_m': [0.3, 0.0, 0.1], 'lightnav_instruction_prefix': 'Walk along the middle of the corridor, leaving ample space from walls and furniture.', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

### 后移 30 cm：运行错误

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/camera_back_01`。
- 原 Monitor 原因：同一观察的目标选区三次未取得可核验证据。 需要帮助：帮助确认目标位置或提供更清晰视角。
- 首次控制受阻：无
- 几何恢复候选选择 0 次，可证实路径 0 次；终态：`None`。
- 独立位姿评分 RMSE：0.0066 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 5 次；LightNav 28 次，接受 27 次、STOP 0 次。
- 实际配置：`{'desired_clearance_m': 0.03, 'camera_position_base_m': [0.0, 0.0, 0.1], 'lightnav_instruction_prefix': '', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

### 仅相机后移 5 cm

- 目录：`/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/camera_back_02`。
- 原 Monitor 原因：同一观察的目标选区三次未取得可核验证据。 需要帮助：帮助确认目标位置或提供更清晰视角。
- 首次控制受阻：无
- 几何恢复候选选择 0 次，可证实路径 0 次；终态：`None`。
- 独立位姿评分 RMSE：0.1392 m；记录的非足部净法向力峰值：0.000 N。
- Qwen 5 次；LightNav 18 次，接受 18 次、STOP 1 次。
- 实际配置：`{'desired_clearance_m': 0.03, 'camera_position_base_m': [0.25, 0.0, 0.1], 'lightnav_instruction_prefix': '', 'model_profile': 'qwen', 'task_budget_s': 360, 'decision_budget_s': 30, 'max_recoveries': 3}`。

## 完整深度与地图

- RGB 保持 640×360、69°×42.27°；独立深度为原生 RTX 848×496、87°×58.07°，不缩放原始深度，也不扩大 RGB。848×496 是仿真渲染栅格，不宣称为 D435i 硬件流模式；两光心暂按理想共点，实机仍需实际 K/T 标定。
- 同一 PhysX 曝光；完整深度供里程计和几何地图，按标定重投影的 RGB 对齐深度供掩码测量。原始两路 K/T、深度和估计状态均归档。
- 实际标定验证：208433 个有效深度点位于 RGB 之外；内部区域对齐误差 p95 为 3.42 mm，地面误差 p95 为 9.91 mm。失败的元数据投影尝试保留在 calibration_01–05；calibration_06 为实际通过记录。
- 第一轮深度基线的阻挡来自新增视场测得的真实地面，在地面估计变化后被历史体素重新分类成障碍。现按原始点及当帧地面存储 SUPPORT/OCCUPIED，后续射线和观测正常覆盖；未放宽 6 cm 阈值、填充 unknown 或清空历史障碍。
- 地图快照同时保存 RobotState、三维网格和障碍集合；每次求解固定一份快照，并计入观测年龄。

## 停车动态与提前余量

- 六组原生物理标定：两种步态相位直行、左右弯行、左右转向；每组保留 RGB-D、200 Hz IMU/关节、指令、估计状态以及独立离线物理记录。无策略训练。
- 传感器拟合时间常数 `[vx,vy,wz]` 为 `[0.07951805983710569, 0.10305072156273873, 0.1476872136611893]` 秒。均值并未在每组改善，完整逐组结果见 `stopping_cases/CANDIDATE_REPORT.md`。
- 预测同时包含观测残差和由原始传感器速度／角速率计算的采样间运动量。初版在 dense physics 中漏掉最多 2.49 mm；修订包络覆盖六组及未参与拟合的旧垃圾桶停步记录。真值只做评分，不回填系数。
- 当前及未来 MPC 节点均检查零指令停步的完整三圆包络与余量；仍保留初始状态、执行延迟和原硬净距约束。停车预测不再把带速度的机身直接画成静止点。
- 冻结旧输入中 46.02 s 已因停车余量不足拒绝原来的前进动作；47.78 s 原始硬净距拒绝仍保留。冻结输入不等同新闭环。
- 限制：当前前进动力学中的 0.15 s 响应仍用于一般指令变化；六组全零停步数据不覆盖任意部分制动、所有未来状态或实机。经验包络在低速时也适用，可能保守阻止恢复。

## 恢复与模型权限

- LightNav 仍负责持续局部导航，Qwen 仍负责任务意图、目标歧义和策略调整。
- 首次 MPC 受阻后，Monitor 在原最多三次恢复预算内尝试五个有界几何原语：前进、左右短弧、左右转向。没有添加倒退或强制转动。
- 候选由同一 MPC 求解，完整预测扫掠和停车轨迹查询已测三维空间；未知不作 free。需要至少一个地图格的预测净距增益。
- 实际执行逐次重新求解并验证，最多 5 秒；净距增益对固定起始障碍集合计算，避免因障碍从地图消失而伪造进展。选定路径或预测不是已执行成功。
- 未取得可证实路径或未取得实测增益时明确 NEEDS_USER。该证书覆盖所述地图与模型条件，不是全局真实无碰撞证明。设计参考 [Nav2 有界行为与碰撞检查](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/)。

## 保持的条件与局限

- GPU0、Qwen 实验室直连、精确模型 `Qwen3.6-35B-A3B-FP8/`、thinking off/2048；360 秒任务、30 秒决策、最多三次恢复。无 Gemini、训练、硬件、外部施力或人工暂停恢复。
- 三个对照每次只改一个指定参数；相机后移时 RGB/depth 整体移动且同步更新外参。默认相机、期望净距和 Qwen 配置未被比较运行覆盖。
- Monitor 成功和独立真实几何成功分别记录；STOP 或选区身份不等同完成。接触记录仅有机身净法向力，无对象、接触点和摩擦，零值不单独证明无碰撞。
- 入口：`prototype/isaac/run_experiment.py`。所有运行各自有 `results.json`、两项 evaluator、源代码快照和真实运行日志。汇总：`.local/outputs/native_depth_sequence_20260916/comparison_summary.json`。

## 新闭环停车的独立核对

详见 `.local/outputs/native_depth_sequence_20260916/closed_loop_stop_evaluation/REPORT.md`。四个有效比较回合共六个自动停车事件；两次在1.64/1.54秒重启，只核验对应零指令窗口，其余四次在2秒后满足停稳阈值。六个事件有效前2秒内，sensor-init与actual-init的圆心运动均被经验包络覆盖。真值不用于拟合或在线控制。

平均轨迹没有普遍变准。居中语言组的位置RMSE从旧模型0.75cm增至2.42cm；后移5cm组从1.23cm增至1.72cm。当前改善是显式保留停步摆动和采样间运动余量，不能写成均值预测全面改善或任意制动已解决。

基线首次停车实际继续移动4.73cm，居中语言组5.32cm，后移5cm组5.03cm。原垃圾桶的实际最小净距分别约18.5cm、11.8cm、1.73m，均不同于旧轮不足1cm的贴近末态；不构成同一狭窄状态的成功脱离复现。

## 恢复功能与本次修正边界

居中语言组实际进入恢复分支，五候选因当前校准停车余量不足而拒绝，没有降低门槛或强行运动。没有任何新回合成功执行几何脱离。五种有限原语无候选，不等于物理上全局无路。

另用基线47个真实归档帧做了一次CPU功能回放：五种真实MPC求解均成功，每候选979个扫掠／停车体积查询全为free，但预测末态净距增益均为负。该地图来自稀疏归档，不能代替原在线10Hz地图，也不计物理执行成功。见 `recovery_functional_replay/`。

回放暴露一个重复检查错误：已受MPC约束的数值零速度−2.28e−29被当成倒退拒绝。已移除后置速度检查，命令仍由原MPC的非负前进、恢复速度0.15m/s和原转速0.6rad/s上限约束；身体净距、停车包络、未知空间及有界轨迹检查保留。五候选增益仍为负，故不能宣称脱离成功。

相机后移30cm还暴露ERROR状态被Monitor接管／pause覆盖的问题。现已保留ERROR锁存、拒绝在错误状态接管，并要求启动具备可用地图；未对地面缺失加入替代数据。后移5cm复测完成时没有运行错误，但实际图像仍有明显自机遮挡。

以上错误处理和重复速度检查修正发生在初始四组之后。前三组没有触及错误覆盖；居中组在原初始停车检查就被拒绝，未到后置速度检查。后移5cm使用修正后的版本。各组原始源码快照和失败记录均保留。

## 相机位置的实际预览

默认位置：

![默认相机](/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/integrated_baseline_01/preview.jpg)

后移30cm，机身遮挡且地面不可测：

![后移30厘米](/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/camera_back_01/preview.jpg)

后移5cm，可以建图但仍有自机遮挡：

![后移5厘米](/home/baoquanc/workspace/GuidePup/.local/outputs/native_depth_sequence_20260916/camera_back_02/preview.jpg)

安装位置应按真实机器人外形、视线和设备标定验证；本次没有提高相机、隐藏机身或更换renderer来取得对照结果。

## 收尾

全部本任务plant、导航、LightNav和评分进程以及独立tmux已结束；GPU0上剩余的`gr00t.rl.eval_agent_trl`属于其他项目，未操作。团队协调已归档停用。没有commit/push、训练、硬件动作或扩展成36/72轮。
