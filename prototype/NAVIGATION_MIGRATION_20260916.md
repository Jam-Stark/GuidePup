# 新机器 Qwen p05_force_resume 结果

2026-09-16 00:28 HKT。迁移版本 `20260915_215130_HKT`，分支 `codex/a2-lightnav-prototype`，工作目录 `/home/baoquanc/workspace/GuidePup`。

**环境恢复完成；本次新执行 1 个 Qwen 回合，真实完整成功 0/1，终态 NEEDS_USER。** Owner 后续要求先只用 Qwen，Gemini `t01_force_resume` 暂缓。没有把旧 72 轮、离线 probe 或连接请求加入本次导航分母。

## 环境与执行

- 六个迁移附件、三个 ZIP 共 447 个文件已恢复，保留仓库 `.local/` 层级、旧 A2 policy/资产和历史证据，没有覆盖原有未提交工作。
- 服务器对部分原始 CDN 的直接连接失败，改由服务器直连可达分发源恢复同一锁定版本：Tsinghua PyPI、npm/Node 分发镜像、HF mirror。Qwen 继续直连原实验室 endpoint。没有使用客户端转发代理下载。
- Python 3.11.7；两个环境实际 `torch.__version__=2.10.0+cu128`、CUDA 12.8；sim 为 MuJoCo 3.6.0、Ultralytics 8.4.152，LightNav 为 Transformers 5.8.0。Node 22.23.2、Pi 0.85.1，原 package-lock 未修改。详细依赖见环境记录。
- LightNav、YOLOE/MobileCLIP 全部就绪，任务词表仍为 `door / door frame / elevator door / wall`。22 个公开文件的大小与迁移清单相同，所附 LightNav JSON 配置内容相同；来源和比较范围见 [模型恢复记录](../.local/outputs/navigation_framework/new_machine_restore_20260915/resume_01/public_model_comparison.json)。
- Qwen 精确 ID `Qwen3.6-35B-A3B-FP8/`，`http://vic.precognition.team:8888/v1`，无鉴权、thinking off、2048。实际 Pi `configuredModel()/models.complete()` 短连接请求 250 ms 成功；导航中的 23 次请求均使用同一 profile。
- 物理 GPU0：`GPU-f593e489-014b-eed5-4331-b01447615b6e`，CUDA/EGL 同 UUID，EGL2。实际 renderer 为 `NVIDIA RTX A6000/PCIe/SSE2`，驱动 580.173.02，MSAA offsamples=0。本次原生 NVIDIA 初始化成功；没有重置驱动或使用 renderer fallback。

执行入口：

```bash
source .local/migration/20260915_215130_HKT/new_machine_environment.sh
.local/envs/sim/bin/python prototype/sim/run_framework_reproductions.py \
  --model qwen \
  --output .local/outputs/navigation_framework/migration_qwen_20260915_server_01
```

起点 `(2.7662063165748876, -7.173436056562866, 164.29460352541886°)`；`room_elevator`，任务“出门走到电梯门口”。保留 360 秒任务上限、30 秒决策、最多 3 次恢复、−80 N 后拉/暂停问答/人工恢复。安装与回合均在独立 tmux 中执行，按已登记 ETA 等待。

## 新回合的真实结果

运行约 00:14:29–00:16:18 HKT；录制墙钟 108.566 秒，物理记录窗口 0.780–97.840 sim 秒，真实累计路程 4.614 m。

| 项目 | 结果 |
| --- | --- |
| Monitor 最终状态 | **NEEDS_USER**，没有最终任务假 COMPLETED |
| 真实完整出门 | **未观测到**；50 Hz 全部记录均未使三圆包络完全越过真实门框远端 |
| 出门阶段自动完成 | sim43.10s、seq136；当时包络到真实门框远端的有符号净距 **−0.893 m**，阶段完成错误 |
| 终态真实门框远端净距 | −0.750 m；负值表示尚未完整越过 |
| 终态真实电梯平面前缘净距 | **6.853 m**；原候梯约定为 0.6–0.9 m |
| 终态朝向误差 / 横向偏移 | 49.336° / 4.923 m；均不满足原约定 |
| 目标 ROI 代表点 | 后续已落到真实电梯门扇，几何平面残差约 0.00001 m；不等于已经到达 |
| 停止 | requested/effective command 均为零；实际平面速度 0.001857 m/s |
| 正非支撑接触 | **0**；不能将 MPC 的净空阻塞说成身体已发生接触 |
| 新真实完整成功 | **0/1** |

直接求助原因是阶段恢复次数用尽：末次原地观察动作的当前/延迟机身包络净空为 `0.010 m < 0.015 m`，控制器拒绝继续。记录出现四次 `recovery_started`，前三次进入恢复，第四次因已有初次失败和三次恢复后失败而直接求助，没有再执行第四次恢复。

## 框架核验

### 搜索上下文与动作已实际通过

seq16 在目标缺失、YOLO 空实例时明确允许搜索；seq20→22→28 实际执行右转30°并到达 `turn_checkpoint`。后续目标丢失时也执行了 `search_target` 和 `change_view`；一项右转45°完成，另一些转向被实际净空限制阻止。因此观察入口可用，但不能把它等同于搜索方向和目标关联都正确。

### 同侧表面被错误配成门洞两侧

obs3 / frame `4406718663-70` 中，模型在 seq62 设置 `sameOpening=true`：

| 边界 | 测量 | 实际物理位置 |
| --- | --- | --- |
| B1 | S4/D6，bbox `[399,1,461,247]`；代表点 XY `[1.22659,-4.2]` | 门洞西侧墙面 |
| B2 | S2/D4，bbox `[470,0,565,317]`；代表点 XY `[1.965,-5.09108]` | 西侧打开的门扇 |

两组选区在图像中呈左墙/右门扇，但**都属于真实门洞西侧**；真实开口 x=[2,4.2]。两点构成的斜门洞平面 normal=[0.769985,0.638062] 不是实际门洞横截面。Monitor 随后在该错误平面上自动完成出门阶段，机器人仍在真实门框附近，后续动作受到物理净空限制。

所选 S4/S2 的掩码交集为0。同帧 S1/S2 有2640像素重叠，但S1没有被选。seq63虽然名为 `boundary_evidence_merged`，B1/B2各只有一份测量；本次**没有实际检验多个重叠mask合入同一B的路径**。距离分开和不同B编号没有证明“开口两侧”的语义关系。

### 自动完成触发范围

seq135→136→137 真实触发 Monitor 检查、自动完成阶段0并推进。独立物理结果证明这次阶段完成不成立。阶段1没有满足候梯条件，也没有终点完成检查事件；**本轮没有检验到“已合格候梯后自动完成”的路径**。通用提示、YOLO词表、导航框架及几何/接触条件没有在本轮修改。

## 制动、暂停问答和恢复

- −80 N 请求 sim22.16s，50 Hz 首次锁存 sim22.88s，延迟 **0.72 sim秒**。
- 检测 tick 的 effective command 仍代表刚执行的旧指令；随后206个暂停采样的 effective command 全为零，207个暂停采样的 requested command 全为零。
- 放力后继续保持 HUMAN_PAUSED；问答和明确人工恢复均发生，Monitor task ID 保留，之后继续原阶段。
- 50 Hz 从首次锁存位置计算的最大位移为 **0.238 m**；录像器从稍后的观察位置计算为0.234 m。零命令没有被当成位置锁定。
- 答案明确表示尚未完全出门，引用 `force_brake` 和必须人工恢复的控制事实；没有修改目标。答案没有进一步解释后向力估计的触发细节。
- 最终终态后3.077秒观察窗没有新增模型请求。

## 实际发现并修复的汇总错误

| 错误 | 修复及验证 |
| --- | --- |
| runner 对 recorder 使用 `check=True`，将正常 NEEDS_USER 退出码2当成崩溃，跳过 evaluator | 接受已记录结果码0/2，其余退出码继续显式失败 |
| `evaluate_monitor.py` 仍读取已删除的 `stage_completed.modelReason` | 改读当前事件的 `completionAuthority` 和 `reason`；没有改几何、接触计算或阈值 |

同一已完成物理回合随后补执行两项 evaluator，均退出0，并使用原 `episode_result()` 生成 `results.json`。没有重跑导航、追加模型请求或新建测试。原 runner 失败日志、旧文件和补评估源码均保存；原 supervisor 的 PROCESS_FAILED 历史没有改写。[补评估记录](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/evaluation_completion.json)说明了这一差别。Recorder 自身 `recorder_error=null`，没有把这些汇总程序错误当作导航失败原因。

## 证据与收尾

- [results.json](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/results.json)
- [重点核验与暂停指标](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/focused_findings.json)
- [独立物理评估](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/qwen_p05_force_resume/physical_evaluation.json)、[v2证据](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/qwen_p05_force_resume/v2_evidence.json)
- [完整录像](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/qwen_p05_force_resume/camera.mp4)、[终态图像](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/qwen_p05_force_resume/final.jpg)
- [实际 GPU/renderer](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/runtime_device.json)、[Pi 实际连接](../.local/outputs/navigation_framework/new_machine_pi_20260915/resume_01/qwen_complete_result.json)
- 原始物理/RGB-D/mask/LightNav记录：`.local/outputs/sim_20260916_001428/`；本次agent/interaction/source_snapshot在新回合目录。

本任务仿真、LightNav、安装及下载会话均已停止；GPU0回到1 MiB/0%。按Owner要求仅将已有全局网络模式由global切至split，并清理tmux全局代理环境；Codex专用代理配置及其进程没有改动。[收尾状态](../.local/outputs/navigation_framework/migration_qwen_20260915_server_01/shutdown.json)

未扩展36/72轮、未训练、未操作硬件、未commit/push。后续待Owner确认；当前明确问题是同侧空间证据被错误关联为开口两侧，以及在真实门框附近受限的观察/接近行为。不能通过外观模板、放宽条件或输入场景真值把本轮改记为成功。
