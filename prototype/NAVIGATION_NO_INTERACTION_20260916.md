# 去除后拉与人工暂停的单次对照

2026-09-16 15:34 HKT。**没有后拉、人工暂停、暂停问答或人工恢复，也复现了LightNav提前STOP。**新回合真实任务成功0/1，终态NEEDS_USER，未完整出门，无误完成或正非支撑接触。这个结果说明人工制动环节不是复现提前停止的必要条件；它不能证明该环节对原回合完全没有影响，也不能证明模型内部累计了虚构位移。

## 对照方法

- 原组：[局部自主权回合](NAVIGATION_LOCAL_AUTONOMY_20260916.md)，固定p05起点，包含−80N后拉/暂停/问答/人工恢复。
- 新组：相同场景`room_elevator`、起点`(2.7662063165748876,-7.173436056562866,164.29460352541886°)`、用户目标“出门走到电梯门口”，GPU0、实验室Qwen精确模型`Qwen3.6-35B-A3B-FP8/`、thinking off、2048 tokens、服务器直连。
- 保留360秒任务、30秒决策、最多3次恢复、同一传感器估计/LightNav/MPC/Monitor和完成条件。
- 40个导航、控制、传感器、场景、模型入口与配置文件与原回合源码快照逐字节相同。只给记录器/runner加入`--interaction none`，跳过实验注入环节；模型STOP之后的正常到达核验仍运行。
- 新组只有一次新导航。两组各1次，独立初始化，未恢复完全相同的机器人状态、模型历史和执行前缀。实际Qwen指令差异见下文。

执行入口：

```bash
LIGHTNAV_GPU=0 .local/envs/sim/bin/python prototype/sim/run_framework_reproductions.py \
  --model qwen --interaction none \
  --output .local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01
```

独立tmux执行，登记/结束回执位于`.local/outputs/no_interaction_control_20260916/`。两项evaluator及results由runner自动生成。

## 结果比较

下表的STOP时刻按LightNav输入图像的曝光时间计算；“从首帧到STOP”使用传感器时间，不是任务墙钟时间。

| 指标 | 原组：有后拉/人工暂停 | 新组：无干预 |
| --- | ---: | ---: |
| 实际外力峰值 | 80N | **0N** |
| HUMAN_PAUSED记录 | 172个控制记录 | **0** |
| 暂停问答 / 人工恢复 | 1 / 1 | **0 / 0** |
| 首次STOP前最大输入间隔 | 3.78s | **0.40s** |
| 首帧至首次STOP | 13.18s | **7.48s** |
| 首次STOP时前缘距入口门框 | 1.409m | **1.083m** |
| LightNav响应 / STOP | 29 / 1 | 24 / 2 |
| 实际采用LightNav的MPC求解 | 106 | 81 |
| 实际总路径 | 1.901m | 1.538m |
| Qwen请求 | 6 | 7 |
| 任务墙钟 | 41.541s | 36.472s |
| 完整出门 / 到电梯 | 否 / 否 | 否 / 否 |
| 终态 | NEEDS_USER | NEEDS_USER |
| 正非支撑接触 / 误完成 | 0 / 0 | 0 / 0 |

新组原始真值日志非零外力采样数为0，在线日志HUMAN_PAUSED数为0，交互日志仅有提交任务、终态和终态驻留记录。不是只关闭了“计划施力”的配置而实际仍有后拉。

## 第一次STOP发生在连续导航期间

新组从3.46至11.34传感器秒持续NAVIGATING。首个STOP为seq22：曝光10.90秒，响应11.30秒。此前23张输入均为不同真实帧，一个session，全部接受，间隔0.30–0.40秒，没有跨人工暂停的长间隔。STOP图像对应实际机身中心仍在门平面之前1.833m；按身体包络计算，前缘仍在入口门框之前1.083m。

预测变化如下：

| 新组序号 | 曝光时间 | 最后航点平移长度 | 最后航点偏航 |
| --- | ---: | ---: | ---: |
| 20 | 10.10s | 1.450m | +0.503rad |
| 21 | 10.50s | 0.007m | +0.270rad |
| 22 | 10.90s | 0m | 0rad，STOP |

即先从前进轨迹变为近乎原地转向，再变为全零轨迹。原组seq26/27/28也呈现近乎原地转向、小位移、全零STOP的末段变化，但转向方向和此前轨迹不同。两组`visible`始终为false；不能将STOP解释成已证实的内部“出门完成”判断。

首次STOP后，Qwen做了一次测量审核，明确调用`continue_navigation`，保持原阶段指令和同一LightNav session。下一次模型响应seq23（曝光14.90秒、响应15.30秒）再次STOP。因此新组后续确实出现4秒输入间隔，但这是**首次STOP之后核验造成的间隔**，不能反过来作为首次STOP的原因。

## 新组为何最终NEEDS_USER

第二次STOP后，Qwen把门框窄条和灰白墙面选作同一开口两侧。Monitor三次拒绝：`Need opposing passage side surfaces; these patches do not face each other.` 本轮未建立portal、未查询已认证通路、未完成阶段。

这与原组“正确相向门框已找到，但完整体积查询42unknown+1occupied”的后续失败不同。两组都没有完成导航，不能把共同的NEEDS_USER标签当成完全相同的失败原因。

## 因果限制

本次最直接的证据是：**在没有实际后拉、人工暂停和长输入间隔的条件下，首次提前STOP仍然发生。**所以不能把所有提前停止都归结为人工brake造成的历史断层。

但本次不是严格冻结前缀的反事实实验：

- 原组阶段指令为`Move through the doorway to exit the room`；新组Qwen生成为`Pass through the doorway to exit the room`，同一意图但文本不同。
- 请求出生参数相同，记录开始时实际位置相差约4mm，第一张模型图像的拍摄时间也不同。
- seq0–3航点数组完全相同；seq4在4.82传感器秒已经不同，而原组约7.06秒才开始实际受力。两组在干预前已经发生轨迹分歧。
- 各条件只有一次，后拉、暂停、问答和恢复被作为整体去除，不能分别估计各环节的影响。

因此，原回合是否受到暂停影响、LightNav为何转向后输出零轨迹、是否存在内部进度假设，仍未被本次对照定位。后续调查应围绕实际图像/指令到转向及STOP的预测变化，保留这些不确定性。

## 证据与收尾

- [对照数据与逐次预测](../.local/outputs/no_interaction_control_20260916/comparison.json)
- [原组暂停前后时序](../.local/outputs/no_interaction_control_20260916/baseline_analysis.json)
- [源码一致性记录](../.local/outputs/no_interaction_control_20260916/source_comparison.json)
- [新组results.json](../.local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01/results.json)
- [物理评估](../.local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01/qwen_p05_no_interaction/physical_evaluation.json)、[上下文与几何评估](../.local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01/qwen_p05_no_interaction/v2_evidence.json)
- [新组视频](../.local/outputs/navigation_framework/local_autonomy_qwen_p05_no_interaction_20260916_01/qwen_p05_no_interaction/camera.mp4)
- 原始仿真：`.local/outputs/sim_20260916_152935/`；对照脚本`no_interaction_control_20260916/compare.py`。

新组无运行错误，记录器exit2代表非成功终态，runner/evaluator流程正常完成。任务服务和tmux已退出，GPU0释放为1MiB/0%；未扩批、训练、操作硬件、修改Codex代理或commit/push。
