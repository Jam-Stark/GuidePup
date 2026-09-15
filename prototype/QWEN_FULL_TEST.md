# Qwen 完整任务测试

2026-09-14 HKT。默认模型确认并保持实验室 `Qwen3.6-35B-A3B-FP8/`，本轮三回合均真实调用 `http://vic.precognition.team:8888/v1`。导航、旧 A2 policy、增强 Depth 翻译器与人工制动链路已联动运行；**不能判定完整任务能力全部通过**，数值停车和自主恢复失败，暂停问答也有错误的视觉进度声明。

本轮固定现有生产实现，没有在回合之间修改提示、控制参数或 checkpoint。三个回合提前登记，失败原样保留，没有重跑挑选模型答案。只新增实验记录、离线分析和本文；[计划](../.local/outputs/qwen_full_test/plan.json)、[实际模型](../.local/outputs/qwen_full_test/model.json)和[运行源码副本](../.local/outputs/qwen_full_test/source_snapshot/)均已保存。

## 三个实际结果

| 回合 | 墙钟时间 / 实际路程 | 物理与控制结果 | 任务与模型结果 |
|---|---|---|---|
| 普通导航 `navigation_01` | 38.83 s / 5.961 m | 无非支撑接触；基座及当前三圆身体包络均通过门平面；235 次 MPC 求解成功 | Qwen 声明 COMPLETED，0 次恢复；实际测了 5 个 ROI，但目标与背景的距离归属仍有混淆 |
| 受力完整路线 `force_route_01` | 39.87 s / 6.932 m | 无非支撑接触；后拉制动、保持暂停、人工恢复后继续过门全部执行；256 次 MPC 求解成功 | Qwen 声明 COMPLETED；暂停问答未取新图却错误声称已穿门；最终 ROI 混有目标和墙面 |
| 约 1 米停止 `metric_route_01` | 40.27 s / 5.476 m | 无非支撑接触；基座中心过门、身体包络未清门；220 次成功、4 次 MPC 阻塞 | 3 次恢复后 NEEDS_USER / PAUSED；3 次 ROI 请求因帧引用错误失败；最后可见目标最近约 2.165 m，没有完成约 1 米要求 |

前两回合目标原文是“穿过右前方的敞开门，走到门内白色置物架前，然后停下”，没有自行增加 1 米门槛。第三回合明确要求从机器人 base 原点到置物架朝向机器人的可见实体表面约 1 米；预先登记的离线解释为 0.9–1.1 m。这一解释、目标分割和场景门平面仅供离线评价，没有送给 Qwen、LightNav 或 MPC。

“无非支撑接触”来自每 5 ms 的 MuJoCo 机器人接触对记录，排除正常足底支撑；三回合非支撑 contact 记录与非地面正法向力接触均为 0。实际路程与过门使用每 20 ms 的基座位姿。身体清门按当前 MPC 三圆包络计算，不能扩大成所有关节姿态或实机的保证。三次描述性运行不构成跨场景成功率，也不能把数值任务的阻塞归因于措辞本身。

## 人工制动与速度不匹配

受力回合在同一个 Pi TaskSession 中保留原始目标，执行以下完整流程。外力和恢复由实验脚本扮演操作者，Qwen 没有人工 resume 工具。

| 事件 | 实际证据 |
|---|---|
| 施加后拉 | sim 19.52 s 请求局部 x 方向 −80 N，持续 2 sim s |
| 制动锁存 | sim 20.12 s 首次 HUMAN_PAUSED；相对请求 0.60 s，包含旧辅助估计与执行响应 |
| 指令归零 | sim 20.14 s 下一 policy tick effective command 全零 |
| 放力后保持 | sim 21.56 s 首次记录外力释放，仍保持锁存 |
| 暂停交流 | 问答前后 sim 22.54–23.50 s 均为 HUMAN_PAUSED、零运动命令 |
| 导航不能解锁 | 操作者通过导航接口试探得到 HTTP 409；这不是 Qwen 越权尝试 |
| 人工恢复 | `source=human` 的 resume 返回 200；sim 23.56 s 首次解除锁存，24.02 s 首个非零有效指令 |
| 恢复实际运动 | sim 24.54 s，effective vx=0.240 m/s，actual vx=0.1127 m/s；随后基座和身体包络均过门 |

本回合有外力且仍处于 NAVIGATING 的 29 个控制样本中，前进命令为 0.300 m/s，实际前向速度均值为 **−0.078 m/s**；保留了受力减速/后退行为。整个有运动命令的导航期间，命令/实际 vx 均值为 0.2847/0.2467 m/s，平均绝对差 0.0423 m/s。MPC 使用实际位姿推进路径，这次在人工恢复后完成了路线行进；它没有消除顺应行为导致的速度差。零命令期间仍有 policy/PD 力矩，也不意味着位置锁定。

详见[权限事件](../.local/outputs/qwen_full_test/analysis/force_route_01/force_authority.json)和[物理报告](../.local/outputs/qwen_full_test/analysis/force_route_01/PHYSICAL.md)。

## Qwen 与 Depth 工具的实际问题

普通导航中，Qwen 对同一帧实际发出 5 个成功测量。前侧立柱 ROI 的 D3 层中位距离为 1.668 m，确有目标像素支持；其它区域混有架子和背景。Qwen 的完成理由却把目标主体概括成 2.1–2.3 m。该目标没有数值停车条件，因此不按 1 米判它失败；但其距离解释不能作为可靠的目标实体测距结论。

受力回合的暂停回答更明确暴露了交流问题：提示要求先读 status/scene，Qwen 没有调用任何工具，直接声称“已通过门口，现在位于门内房间”。当时 base 仍在起点附近，人工尚未恢复，它此前唯一读到的图也是门外视角。**控制权限保持成立，但这句视觉进度错误。** 最终测量 ROI `[533,120,640,240]` 含右立柱、少量层板及大块背景墙，落在同一距离带；混合中位值 2.248 m 不能直接当成目标表面距离。距离带颜色不等于物体分割。

数值回合的 3 个测量错误属于调用依赖问题：Qwen 在收到 scene 返回值之前，已经一次生成了 `status + scene + measure_region`，把 SSE 中的拍摄帧号填入测量参数。工具顺序执行不会自动改写预先生成的参数。

| 恢复回合 | SSE 中的帧尾号 | scene 实际返回 | measure 使用 | 结果 |
|---|---|---|---|---|
| 1 | 1754 | 1769 | 1754 | 409 |
| 2 | 1775 | 1789 | 1775 | 409 |
| 3 | 1795 | 1811 | 1795 | 409 |

共同前缀为 `2188200116-`。只有经 `/scene` 读取的帧会进入测量缓存；这里不是缓存过早淘汰，也不是深度反投影算错。扩大缓存并不能纠正“未看 scene 返回值就测量”的依赖错误。前两批还预先包含 navigate，因此测量失败后未经过新一轮模型判断就开始恢复。三次恢复基本重复向前指令，未解决当前几何约束；最后真实调用 pause，没有虚报完成，却也没有向人解释具体原因。

这些结果把两类问题分开：**目标与距离层的语义绑定仍由 Qwen 出错；帧引用错误则暴露了模型与工具调用顺序的集成缺口。** 增强 Depth 翻译器提供像素与数值对应，但不会自行保证模型先观察、再选实体、再解释测量。上一轮同帧计算核对见[三模型测距对照](pi/DEPTH_COMPARISON.md)。本轮没有静默改变工具协议来掩盖这些失败。

## 数值停车为何没完成

首次受阻在 sim 175.34 s，base 约 `[11.1359,11.3655,yaw=-0.0852]`。四次 MPC 报告的当前/延迟身体包络净空分别为 **0.011、0.006、0.001、0.005 m**，低于已配置的 0.015 m；后三次几乎原地。它们是硬净空检查暂停，不是记录到了物理碰撞。当前/延迟包络已经违反约束时，重复下发同方向目标不能产生可行解；现有前进/转向控制也没有提供一条实际验证过的脱困路径。

最后一次模型读取的帧为 `2188200116-1854`，sim 185.32 s，距终态记录 0.58 s，期间基座仅移动 2.8 mm。使用同一 qpos/qvel 离线重建，RGB 与光轴深度均逐像素一致；按实际可见目标几何分割并独立反投影，20,233 个有效目标像素的 base 水平距离为：最小 **2.165 m**，p10 **2.341 m**，中位 **2.971 m**，p90 **3.707 m**。没有目标像素落入登记的 0.9–1.1 m，因此本次数值要求明确未达成。

该 mask 只包含置物架结构，排除碗、纸箱、墙、地板和结构空隙；没有把场景目标标签提供给模型。距离只描述最后可见表面，不推断未见表面。离线脚本初次把资产名当作 MuJoCo body 名而报错，按已有参考的真实 body 名修正后完成；没有改动或重跑导航回合。

[终点参考与重建误差](../.local/outputs/qwen_full_test/reference/metric_route_01/reference.json)、[目标像素示意](../.local/outputs/qwen_full_test/reference/metric_route_01/target_overlay.png)、[四次阻塞位置](../.local/outputs/qwen_full_test/analysis/metric_route_01/mpc_blocked_positions.json)。

## 视频、记录与后续实现位置

| 回合 | 视频和原始任务记录 | 独立核验 |
|---|---|---|
| 普通导航 | [视频](../.local/outputs/qwen_full_test/navigation_01/camera.mp4)、[summary](../.local/outputs/qwen_full_test/navigation_01/summary.json)、[Pi 事件](../.local/outputs/qwen_full_test/navigation_01/agent.jsonl) | [物理](../.local/outputs/qwen_full_test/analysis/navigation_01/PHYSICAL.md)、[语义](../.local/outputs/qwen_full_test/semantic/navigation_01/REVIEW.md) |
| 受力完整路线 | [视频](../.local/outputs/qwen_full_test/force_route_01/camera.mp4)、[summary](../.local/outputs/qwen_full_test/force_route_01/summary.json)、[施力/对话/恢复](../.local/outputs/qwen_full_test/force_route_01/force_events.json) | [物理](../.local/outputs/qwen_full_test/analysis/force_route_01/PHYSICAL.md)、[语义](../.local/outputs/qwen_full_test/semantic/force_route_01/REVIEW.md) |
| 约 1 米停止 | [视频](../.local/outputs/qwen_full_test/metric_route_01/camera.mp4)、[summary](../.local/outputs/qwen_full_test/metric_route_01/summary.json)、[Pi 事件](../.local/outputs/qwen_full_test/metric_route_01/agent.jsonl) | [物理](../.local/outputs/qwen_full_test/analysis/metric_route_01/PHYSICAL.md)、[语义](../.local/outputs/qwen_full_test/semantic/metric_route_01/REVIEW.md) |

各 summary 的 `before/final.output` 指向原始仿真目录，包含 contacts、20 ms telemetry、MPC、LightNav 回复和实际读过的 RGB-D 缓存。录像按目标 5 FPS 采集，精确时序应以日志为准。

后续应围绕本次已暴露的具体问题改实现：先取得 scene 返回值再允许依赖它的测量；暂停问答取得当前状态和画面后再回答；给 MPC 当前包络净空不足的状态定义可执行的恢复方式；目标实体测量与到达判定继续分别核验。这些尚未修复，不能写成现有能力。

本轮使用同一个 ProcTHOR 场景、D435i 名义 640×360 理想对齐 RGB-D、当前 A2 三圆尺寸约束 MPC、原 `model_59999.pt` 的 6D 导出。LightNav 仍只收 RGB，Pi 没有真实剩余路线距离。未训练、未下载本地 Qwen、未运行 Isaac Lab 或实机；网络直连、仅使用 GPU0。收尾将默认仿真重置到 IDLE，保留 Qwen 配置与两个本机服务。
