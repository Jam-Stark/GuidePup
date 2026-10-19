# 校园默认导航：当前交付与模型保留

2026-10-15。Owner先要求持续排障并取得完成视频，随后明确要求“不再重试，交付当前情况并render视频”，并要求保留LightNav加载状态。**已停止导航；当前真实成功0次。** 交付是未完成任务的真实进度录像，不是成功视频。GPU3的模型独立常驻，仿真与任务服务已关闭。

## 本次实际结果

默认任务保持“一层起点→中间2号电梯门前→朝向门并停下”。场景、初始位置、相机、完整原生深度、机身包络、10cm期望净距和物理完成条件均未改。Qwen使用实验室精确FP8末尾斜杠模型/off/2048，模型调用直连；原生MPPI与WBC保留。在线没有使用场景真值。

| 记录 | 实际路程 | 结束与真实结果 |
|---|---:|---|
| 初始task_01 | 0.023m | 规划漏掉朝向/停下要求，NEEDS_USER；0导航动作，路程为静止收敛 |
| navigation_02 | 8.277m | Qwen误报COMPLETED；真实未完成；横向偏差5.663m、朝向58.98°、前缘距目标平面0.955m；停稳 |
| navigation_03 | 9.328m | Qwen误报COMPLETED；真实未完成；横向偏差4.861m、朝向23.53°、前缘距目标平面0.435m；停稳 |
| navigation_04 | 0.597m | Owner要求停止后取消的部分回合，不是自然终止或成功；未覆盖新修正的STOP后分支 |

02的169次原生MPPI求解全部成功并下发，无控制受阻。03有192次成功下发（163导航模型/29显式转向），1次受阻；说明不能把02的失败归因于MPPI。03在首次STOP后实际选择并执行了观察转向，但最终仍未辨认/抵达指定实例。接触observer非足部净法向力峰值0N，不能据此证明无碰撞；缺接触对象/点/摩擦，且可能包含自接触/力相消。

独立条件仍为目标门前缘净距0.6–0.9m、横向≤0.65m、朝向误差≤25°，以及原停稳窗口。没有放宽条件、替换为近处其他电梯或通过场景坐标控制。

## 当前修正及其验证边界

1. 分离GPU：启动器支持`--sim-gpu 4 --lightnav-gpu 3`。实际NVML记录确认模型GPU3、Isaac/本地感知GPU4，CUDA与renderer UUID匹配。原生MPPI/当前WBC在CPU，Qwen远端。
2. 规划接入：完整保留用户原文和末尾要求；明确类型取决于终态；被拒绝计划连同精确原文/已提交文本返回Qwen。真实Pi探针和02/03完整任务已覆盖完整原文/approach_target。
3. 完成判断：明确具体实例身份区别于类别外观；像素深度区别于机身净距；odom坐标区别于目标相对位置。02的STOP回放中Qwen改为继续接近；03确实未在第一次STOP直接结束，但仍在后续错误结束。
4. 动作接口：明确complete_task只结束，不执行reason中描述的新动作；继续/换视角工具排在结束工具之前；不增加用户未说明的“必须有可见编号标牌”假设。03受阻记录的真实Pi探针返回continue_navigation。04仅运行63.885秒/0.597m后按Owner要求中止，**这些最新修正未取得闭环成功验证**。不能将工具顺序单独宣称为已定位的唯一根因。

Qwen仍负责阶段、策略、指令及完成声明；语义Monitor仅提供参考，没有新增证书门槛或自动语义决策。没有删除STOP、强制前进、备用控制器或新增自由度。源代码类型构建通过；传感器、控制器和成功阈值未改。

04的resident没有HTTP取消端点。按Owner要求先调用真实/pause停止运动，再SIGINT关闭resident以封装录像和结束launcher。原始results保留连接关闭造成的recorder_error，未改成假成功；operator_cancelled.json记录实际取消。该错误是人工收尾造成，不列为自主导航新故障。独立评分最后0.3秒未满足停稳，不宣称04完成停车。

## 视频交付

证据根：`.local/outputs/campus_elevator_completion_20261015/`。

- `delivery/campus_progress_attempt03_2x.mp4`：最新自然结束的03回合，50.1秒、1120×360；原有500张有序录像帧以2倍录制回放，配英文事实面板，明确TASK INCOMPLETE/Physical success NO。没有合成运动、替换场景或隐藏失败。
- `navigation_03/episode/camera.mp4`：原始录像100秒。录像器按5fps封装实际收到的RGB，回合墙钟401.13秒；录制回放不是物理实时。新视频的“2x”相对这段已录制视频。
- `navigation_04/episode/camera.mp4`：最新取消回合的原始部分录像，另存、不与03拼成成功。
- 每个回合的results.json、physical_evaluation.json、navigation_evidence.json、episode/agent.jsonl、传感器/接触/真值/原生控制记录均保留。attempts.json分类区分误完成和人工取消；delivery/video.json记录渲染方法。

## 保留的模型与关闭的服务

取消04时旧启动器先自动关闭了模型，随后按Owner最新要求恢复一次。现在`guidepup-lightnav-gpu3`独立tmux中常驻LightNav，PID2454462，端口8050，物理GPU3 UUID `ffc02ac2-e15e-00e3-f842-6f501cb0b6e5`，约8980MiB。实际CUDA加载/预热和WS登录确认READY；此保留服务没有提交新导航任务。

`run_experiment.py`新增显式`--reuse-lightnav`：要求现有8050服务就绪，不启动替代模型，实验退出不关闭外部服务。下一次导航若由Owner重新要求，可用`--sim-gpu 4 --lightnav-gpu 3 --reuse-lightnav`。语法检查和实际常驻服务登录通过；为遵守不再重试，未新增导航来验证该选项的完整闭环。保留的是模型权重，每个新任务仍使用独立视觉会话。

GPU4仿真、8768/8770任务服务和本次任务tmux均退出；只保留Owner要求的GPU3/8050模型服务。GPU0和8766其他项目未操作。resources_final.json与model_resident/status.json记录收尾。无训练、硬件、驱动重置、默认Qwen覆盖、Git提交或追加试验。
