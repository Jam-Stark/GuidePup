# 传感器状态与 Monitor 重建结果

后续完整范围导航尝试已执行并提前失败，暴露工具纠正HTTP400和较长时间里程计漂移，见[重跑报告](NAVIGATION_QWEN_SENSOR_TEST_20260916.md)。下文保留重建交付时的有界验证范围。

2026-09-16 11:47 HKT。Owner确认实机只有 D435i RGB-D、IMU 和关节状态；本次按这组输入重建在线状态与完成判据。GPU0，Qwen默认配置保留；未新增完整导航回合、训练或硬件操作。

## 已实现

1. **真实可获得的输入**：RGB-D hybrid odometry配合IMU重力/旋转初始化产生局部 SE3 和速度。低层旧A2策略的历史观测也改为关节编码器、陀螺仪与估计重力，保留原checkpoint/动作/力估计输出。没有用真值加噪声冒充估计。
2. **同一状态连接全部模块**：`RobotState`携带时间、epoch、变换、速度、跟踪状态；曝光时几何、LightNav路径对齐、MPC初态、Monitor和Qwen共享它。删除在线世界真值位姿/速度字段及旧的仿真真值fixture，不增加旧接口兼容。
3. **完成证据重建**：通路须有相向侧表面、实际重叠范围、完整机身体积自由走廊与远端两侧自由扩展；估计轨迹实际穿越并清空远端后，再核验当前停止空间。候梯完成使用当前已审核RGB-D表面的机身相对几何。停车要求多个不同采样时间的低速估计，不能用零指令或初始化占位速度证明。
4. **离线真值隔离**：`ground_truth.jsonl`独立保存物理状态和施力；两项evaluator在离线阶段按时间连接它。在线服务、模型上下文和`telemetry.jsonl`不含这些真值。

架构和硬件输入合同：[ROBOT_STATE.md](ROBOT_STATE.md)；完成规则：[NAVIGATION_FRAMEWORK.md](NAVIGATION_FRAMEWORK.md)。

## 实际证据

证据目录：[sensor_state_rebuild_20260916](../.local/outputs/sensor_state_rebuild_20260916/)。

| 核验 | 结果 | 不能据此声称 |
| --- | --- | --- |
| 原同侧墙面/门扇测量，保留原完美位姿 | 新`makePortal`明确拒绝：表面不相向 | 不是新导航或新的模型成功 |
| GPU0原生渲染，真实编码器/IMU/RGB-D → 几何/地图 → 生产MPC跟踪 | 0.25m局部前进请求执行，26/26次MPC求解成功，终态`PAUSED/advance_checkpoint`；16.61s墙钟 | 未调用LightNav/Qwen，不是完整任务完成 |
| 同次运行独立真值对照 | 25个跟踪帧、约2.4s；位置RMSE 0.05025m，末端误差0.00889m；一次坐标对齐仅用于离线比较 | 不构成实机定位精度、长期漂移或弱纹理保证 |
| 同次物理记录 | 路径0.31832m，末端真实平移速度0.01109m/s，正非支撑接触0 | 未重新覆盖−80N/问答/恢复整轮协议 |
| 两项evaluator | 在此次有界运行的独立真值记录上均完成；输出physical_evaluation.json与v2_evidence.json | 输出不是新完整回合成绩，未写入旧72轮或迁移回合分母 |
| 构建 | TypeScript生产构建、Python编译通过 | 不替代完整导航或实机验证 |

具体文件：

- [完美位姿反例](../.local/outputs/sensor_state_rebuild_20260916/perfect_pose_counterexample.json)
- [实际功能运行](../.local/outputs/sensor_state_rebuild_20260916/live_sensor_path.json)
- [估计与独立真值比较](../.local/outputs/sensor_state_rebuild_20260916/live_comparison.json)
- [两项evaluator与源码快照](../.local/outputs/sensor_state_rebuild_20260916/functional_evaluation/)
- 原始传感器、在线状态、MPC及离线真值：[sim_20260916_114439](../.local/outputs/sim_20260916_114439/)

## 联调中暴露并修复的问题

MuJoCo named sensor的名称和地址类型与初版adapter假设不同，导致两次初始化错误；现按选定A2资产的实际encoder名及整数地址绑定。

低高度图像点中，墙面像素可能比地面更多。直接把最大平面当成地面候选会拒绝仍可看见地面的帧；现在分割实际平面，再选择与测得重力对齐的支撑面。失败原帧重算得到法向约`[0.00227,0.00050,0.999997]`。没有放宽原15°方向条件/2cm拟合阈值，也未读取场景地面高度。[失败帧诊断](../.local/outputs/sensor_state_rebuild_20260916/floor_plane_diagnosis.json)。联调错误日志01–04均保留，最后成功记录为05。

## 当前边界

- 通路证据目前支持相向、近似平行的侧表面。延续侧墙、遮挡远端或未被观测的地面会使认证未满足，需换视角/有界移动或求助；不会推测门厚或把unknown当free。
- 地图使用局部平地假设和当前A2的机身高度包络。当前传感器是理想仿真RGB-D/IMU，未标定真实D435i外参、时间同步、噪声和IMU偏置。
- Open3D里程计没有经过长期闭环或实机验证；其information matrix未被当作协方差。跟踪失败锁存，旧epoch几何不能继续使用。
- 既有360s/30s/3次恢复、−80N制动、人工暂停/恢复权限及机身/碰撞条件保留。此次只做针对已报告问题的构建与有界功能核验，没有重开完整Qwen p05或Gemini t01。

本次运行已退出，GPU0回到1MiB、0%使用；安装与功能运行tmux已结束，导航服务未常驻。未commit/push。
