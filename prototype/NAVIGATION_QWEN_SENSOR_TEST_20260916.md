# Qwen 完整导航重跑：提前异常终止

2026-09-16 HKT。按Owner要求，用当前传感器状态框架、物理GPU0重跑固定`p05_force_resume`。正式导航尝试已执行，**未成功**：89.475秒后进入`NEEDS_USER`，原因是无效工具参数的纠正请求收到HTTP400。

后续已完成工具纠正和里程计修复，见[独立修复报告](NAVIGATION_REPAIRS_20260916.md)。本页保留该次失败原始结果；后续探针和有界传感器验证不改写本次成绩。

## 配置与范围

- 任务：出门走到电梯门口；初始参数`(2.7662063165748876, -7.173436056562866, 164.29460352541886°)`。
- Qwen：`http://vic.precognition.team:8888/v1`，`Qwen3.6-35B-A3B-FP8/`，thinking off，2048 tokens，服务器直连。实际请求配置保存在`results.json`。
- 360秒任务、30秒决策、最多3次恢复及固定−80N/暂停问答/人工恢复协议保留。本轮提前终止，不能称为所有协议环节均已验证。
- GPU0 UUID与EGL2匹配，原生NVIDIA渲染；未运行Gemini、训练、硬件或扩批。

## 结果

| 项目 | 实际记录 |
| --- | --- |
| Monitor终态 | NEEDS_USER |
| 真实任务成功 | 否；未完整出门、未到电梯 |
| 模型请求 | 10次，含最后一次被HTTP400拒绝的纠正请求 |
| 动作 | 左转60°、右转30°、右转45°、右转45°，均到turn_checkpoint；无前进/follow_target |
| 物理路径 | 0.48188m，包含转向伴随的平移 |
| 机身距真实远侧门框 | 最终仍差3.58210m |
| 电梯真实前缘净距 | 6.79510m；朝向误差89.34°，不满足候梯条件 |
| 阶段完成/误完成 | 无stage_completed；未报告完成 |
| 接触 | 正非支撑接触0 |
| 最终真实速度 | 0.01433m/s |
| −80N/暂停问答/人工恢复 | 未触发：始终未达到固定前进施力条件；不能记为通过 |
| 恢复 | 无recovery_started事件 |

## 运行错误与 NEEDS_USER 原因

终止链路有明确日志，不能只归因于视觉能力或笼统服务故障：

1. seq165：`accept_measurements.arguments`是字符串`"selections"`，而非对象。
2. seq166：本地schema校验`root: must be object`，`executed=false`，没有执行非法提议。
3. seq167：按既有一次纠正规则发送第二次请求，携带该工具错误。
4. seq169：远端返回HTTP400 / `BadRequestError` / `Can only get item pairs from a mapping.`，23.89ms、token计数0。
5. seq170→171：决策失败，Monitor暂停并进入NEEDS_USER。

该次记录没有完整下游服务实现证据，当时尚不能确定是服务端模板、SDK序列化还是请求历史格式的具体根因。该次报告未另发模型probe或第二个正式导航回合来掩盖失败；随后单独开展的调查见上方修复报告。

## 搜索、mask和完成证据覆盖

搜索动作的提议→执行→结束序号分别为21→23→45、55→58→72、81→84→102；最后一个普通转向为111→113→129。

seq138选择S2/S3，原图与mask复核显示两者都在可见开口左侧，右侧另有S1；检测器将S2标成了`elevator door`。seq149先把S2/D5称为电梯门、S3/D3标为not_target，seq150被框架拒绝。seq155重新选择相同两份mask并改称开口两侧，随后在接受测量时发生上述参数错误。

**没有B合并、portal创建、空间认证或阶段完成事件。**因此本轮不能证明新几何认证接受或拒绝了这组边界，也未实际覆盖到达后的自动完成。S2/S3像素交集为0，未触发重叠mask合并。

## 状态估计暴露的问题

独立离线真值对照178个跟踪帧、17.68秒传感器时间：位置RMSE **0.25056m**，最大误差0.34880m，末端0.34844m；末端偏航误差−0.953°。末帧估计平移速度0.07703m/s，真实速度0.01543m/s。

比较只在离线将第一个匹配帧做一次SE3坐标对齐，没有将真值回填在线。该结果表明先前短时有界运行的末端小误差不能代表较长时间稳定性；静止/转向条件下的视觉里程计漂移需要后续处理。

## 启动问题与证据完整性

另保留一次**首个Qwen请求前**的启动失败：`sensor_qwen_p05_20260916_130924`，0模型请求、0运动提交。旧`observe()`要求每次规划/取图之前先满足停稳，5秒超时；该时段真实速度均值约0.0058m/s，而估计速度明显偏大。

本次修正了规划/取图的全局停稳前置条件、完成检查先刷新状态，以及导航图像归档残留的`capture_transform`引用。连续停稳、几何完成阈值、时间预算和人工权限未放宽。修正后在新目录执行了上述一次正式导航尝试。

正式尝试的记录器因固定交互未完成而退出1，runner保持PROCESS_FAILED并在自动评估之前退出。随后在**同一份物理记录**上单独完成两项evaluator和原`episode_result`汇总，未新增物理或模型调用。`evaluation_completion.json`保留这一过程，不把进程失败改成成功。

## 交付

- [汇总results.json](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/results.json)
- [本轮完整目录](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/qwen_p05_force_resume/)
- [录像](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/qwen_p05_force_resume/camera.mp4)
- [物理评估](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/qwen_p05_force_resume/physical_evaluation.json)
- [第二项evaluator](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/qwen_p05_force_resume/v2_evidence.json)
- [里程计独立对照](../.local/outputs/navigation_framework/sensor_qwen_p05_20260916_131320/qwen_p05_force_resume/odometry_evaluation.json)
- [原始传感器、在线状态、独立真值和接触](../.local/outputs/sim_20260916_131404/)

新证据、源码快照、模型上下文及两项服务日志均已保存。任务服务和独立tmux已结束，GPU0回到1MiB/0%。未commit/push。旧72轮、旧迁移回合和离线probe的分母不变。
