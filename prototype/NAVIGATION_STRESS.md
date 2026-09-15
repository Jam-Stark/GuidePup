# Navigation v2：随机起点与恢复压力实验

2026-09-15 HKT。已完成36个正式导航回合及限定模型复放。36轮中没有一轮通过独立完整任务验收；Monitor有1轮错误报完成。核心发现分为随机视角下的门框/目标误认，以及可复现的LightNav第80帧视觉编码错误。11次实际后拉均完成制动锁存、问答、人工恢复并保留原任务，不能把后续失败笼统归为恢复权限丢失。

任务仍是“出门走到电梯门口”。两组起点分布分开报告，所有失败、未暴露后拉的回合及一次实验者中断均保留。

## 实验范围与环境

- 第一组：12个随机位姿×普通/后拉恢复两种条件，共24个正式回合。位置采样x[1.65,4.35]、y[−7.55,−5.05]m，yaw在整圈12个扇区各取一个随机值；seed20260915。仅在运行前排除与静态碰撞几何重叠的初始包络，3个被排除候选也存档，没有按后续是否能完成筛起点。
- 第二组：6个随机位置×两种条件，共12个回合。x[2.4,3.4]、y[−7.1,−6.5]m，初始朝向对准两门框的角平分方向并加±3°随机量，seed20260916。用于让第一步观察容易一些，以实际覆盖出门后的搜索。两组人群分开报告。
- 两组均保持旧A2 policy、物理场景、相机profile、MPC和Pi/Qwen工具/提示词。每任务360秒墙钟上限、每阶段最多3次控制失败恢复；后拉条件是阶段0实际位移≥0.5m、前进指令≥0.2m/s时施加局部−80N/2sim秒，然后撤力、新图问答、人工恢复。
- 配对只保证相同请求初始位姿；Qwen决策/英文导航指令、控制前缀和实际运动独立生成，未固定Qwen随机种子。统计差异不自动等于后拉的因果效应。未触发后拉就失败的回合是UNEXPOSED。
- 默认NVIDIA EGL在首轮任务开始前卡住；GPU1设备句柄报Unknown Error。本轮明确选用已有Mesa EGL软件设备4、llvmpipe、4个CPU渲染线程，LightNav仍由既有GPU0服务推理。实际GL实现写入每个sim的configuration.json。分辨率仍640×360，但像素、数值与墙钟速率不可假定和旧NVIDIA渲染相同。
- 第一批启动失败没有导航样本。Mesa组的一轮CUDA错误触发Main暂停检查，随后日志证明同一服务新会话恢复推理，没有重启服务/驱动。已结束10轮保留，中途被Main停止的p06_force_resume尝试单列EXPERIMENTER_CANCELLED，剩余14个原定组合另存继续运行，合并为24个正式槽位。

[整圈起点布置](../.local/outputs/navigation_stress/cohort_01/initial_pose_coverage.png) · [第一组逐回合CSV](../.local/outputs/navigation_stress/cohort_01/episodes.csv) · [初始化失败证据](../.local/outputs/navigation_stress/baseline_01/infrastructure_failure.json)

## 第一组：整圈朝向

| 条件 | 完整任务 | 实际整个包络出门 | 进入电梯阶段 | 实际后拉暴露 | 非支撑接触 |
|---|---:|---:|---:|---:|---:|
| 普通 | 0/12 | 0/12 | 0/12 | 不适用 | 0/12 |
| 后拉恢复 | 0/12 | 0/12 | 0/12 | 5/12 | 0/12 |

5个实际后拉回合均锁存制动，完成问答和人工恢复，原task ID保留；另7个回合在后拉前已经结束。不能把这7个样本算作制动/恢复失败。

第一阶段存在可核对的感知→目标→控制失败链：

| 回合 | 被接受的“门框”测量 | 生成的错误退出点 | 后果 |
|---|---|---|---|
| p01_primary | 左点[4.235,−5.059]落在打开的东门叶；右点[4.984,−5.936]落在东墙附近 | [5.454,−4.777] | 目标落在东墙后，MPC停止，多次恢复耗尽 |
| p02_primary | 左点[4.200,−4.096]是真东侧jamb；右点[4.984,−5.169]是东墙，并非西侧jamb | [5.488,−3.978] | MPC首先在sim11.44s停止，最终求助 |
| p02_force_resume | 首次两ROI靠同一东jamb；重选后主要是8m外无效/混合区域 | 未建立 | 未移动、未触发后拉就求助 |

上述回合均没有正非支撑接触，`mpc_blocked`不能解释成已经撞墙。MPC阻止了执行错误视觉关联得到的目标；它没有对象身份识别能力。

对12个普通回合的首张阶段决策图，固定门框内侧中点[2,−4.1,1.2]、[4.2,−4.1,1.2]的同帧视锥覆盖为0/12；旧固定起点primary08的两个指定中点都在视锥内。这是指定点的投影结果，不等于整个门框都不可见，也不证明没有遮挡。原图与错误ROI另行支持“单侧门框/门叶/墙混淆”。当前流程要求同一观察里取得两侧jamb，没有完成跨视角门洞关联，随机朝向下的主动找门能力不足。

p08_primary声称黑屏，但所保存的当前输入图清楚显示门叶和大厅，这条停止理由不受图像支持。p06_force_resume实际调用两次recall_memory并取回归档，但随后把旧的错误退出点当成正确目标继续推断；记忆取回成功不等于旧目标身份正确。p09_force_resume一次空参数recall_memory调用被工具校验拒绝。

[门框投影原始结果](../.local/outputs/navigation_stress/cohort_01/initial_jamb_projection.json) · [逐回合汇总](../.local/outputs/navigation_stress/cohort_01/batch_summary.json) · [明确异常](../.local/outputs/navigation_stress/cohort_01/explicit_error_outcomes.json)

## 出门后的搜索：受控线索

旧的失败交互02和成功primary08，在阶段2首个输入的位置约差2.3cm、朝向约差3.2°；英文分别为Approach the elevator和Walk to the elevator door，首个转向相反。两者都新建scope/session，首输出step=1；可观察协议没有证明reset失效。

用两个已存档的阶段入口画面，分别输入上述两句英文，每格3次独立reset，共12次实际LightNav调用。每格输出可重复，指令和画面会改变转角/轨迹细节，但所有首步都是右转，未复现旧回合的左右反转。旧的真正首LightNav输入帧未归档，所以此对照不能替代原帧重放，也不能据此断言单一英文短语造成失败。

本轮已补足每次推理的精确JPEG、同帧RGB-D/位姿/内外参存档，放在独立navigation_frames目录；不向模型加入目标真值。

[固定画面对照](../.local/outputs/navigation_stress/baseline_01/lightnav_probe/summary.json)

## 长会话异常

p03_force_resume与p10_force_resume均在同一阶段第80次观察（seq79）得到LightNav CUDA错误，前79次有正常响应；错误在视觉PatchEmbed的Conv3d处报告。随后新会话可以继续推理，所以不是持续不可用的证据。

本地实际SlowFast采样配置在79→80次观察间增加一个mid层帧对，实际送视觉编码的帧数50→52；PatchEmbed的Conv3d输入首维11200→11648，完整shape为[11648,3,2,16,16]。post-ViT pooling发生在该卷积之后。此配置下num_history_frames=64只是初始缓存分配容量，并非SlowFast的历史长度上限。

实际限定复放结果：

| 输入与历史 | 结果 |
|---|---|
| 原始80帧，独立新会话 | 前79次正常，第80次CUDA错误，43.41s |
| 相同前79帧，第80张替换成第79张JPEG | 仍在第80次CUDA错误，43.93s |
| 原失败JPEG单独作为新会话首帧 | 正常，0.286s |

这证明失败可随历史长度边界复现，既不需要人工后拉，也不需要那一张特定的新图；问题定位到历史增长后的视觉编码执行路径。尚未证明CUDA内部具体哪个算法/驱动缺陷造成该合法shape失败，不能把源码shape变化本身叫作无效输入。没有通过重置会话、丢帧或降低历史来掩盖异常。

[完整复放结果](../.local/outputs/navigation_stress/history_probe/summary.json) · [精确输入与脚本](../.local/outputs/navigation_stress/history_probe/replay.py)

[源码/CPU形状证据](../.local/outputs/navigation_stress/history_probe/shape_context.json)

## 第二组与复放结果

第二组12轮全部结束，10轮实际整个包络清空真实门框远端，11轮进入第二阶段；有1轮提前推进了阶段，另1轮在首次导航前错误要求确认goal_changed。

| 条件 | Monitor报完成 | 独立完整验收 | 实际整个包络出门 | 进入第二阶段 | 实际后拉 |
|---|---:|---:|---:|---:|---:|
| 普通 | 0/6 | 0/6 | 5/6 | 5/6 | 不适用 |
| 后拉恢复 | 1/6 | 0/6 | 5/6 | 6/6 | 6/6 |

第二阶段的具体分解：

- 7轮（t01_primary、t03_primary、t03_force_resume、t04两组、t05_force_resume、t06_primary）曾关联到真实电梯门板，随后在本阶段80帧边界报CUDA错。普通4轮、后拉3轮；这些是模型执行错误，不是无法恢复原任务。
- t01_force_resume没有建立电梯目标，转向搜索后求助。t02_primary把错误表面作为目标，又在不可见时收到stop候选，最终求助。
- t02_force_resume的“出门完成”过早：阶段完成时整个包络尚差0.502m才清空真实门框远端，整轮最终也未真正清空。这是错误门洞关联引起的阶段验收问题，不能称真正出门后的搜索失败。
- t05_primary将初始控制原因goal_changed误解为需要重新征求许可，未导航即求助。它不是用户真的修改了任务。
- t06_force_resume错误报COMPLETED。最终位姿[1.269,−2.703,−1.684rad]；包络前缘至真实电梯平面8.267m、朝向误差96.46°。两次target_associated在seq362/419，选点落在y=−4的大厅南墙，且模型自己的semanticClaim写着“地面与墙角区域，非门板”。几何站位满足的是错误墙面，不能满足电梯任务。

全36轮无正非支撑接触。11个实际后拉暴露全部锁存、完成问答/人工恢复且任务ID保持；余7个后拉条件回合未触发干预便结束。36个异质回合的0个独立成功是本次记录的描述，不是跨场景成功概率估计。

[第二组逐回合CSV](../.local/outputs/navigation_stress/door_facing_01/episodes.csv) · [搜索失败证据](../.local/outputs/navigation_stress/door_facing_01/search_failure_evidence.json) · [错误完成回合视频](../.local/outputs/navigation_stress/door_facing_01/t06_force_resume/camera.mp4) · [36轮总数核对](../.local/outputs/navigation_stress/overall_results.json)

## 结论与下一步

当前最明确的实现问题是LightNav长期阶段在80帧边界的可复现视觉编码错误；应先针对该合法输入shape定位并修复模型后端。感知方面，需要主动调整观察位置/朝向及跨视角门框关联，并保留目标身份与几何测量的区别，防止非门板表面进入电梯完成判据。正确取回旧记忆不能纠正原始对象误认。

原交互02的左右搜索分歧尚未被单独归因：新会话reset可观察地发生，固定入口图交换英文未重现方向反转。新的完整输入存档为后续精确搜索复放提供依据，不能把本次发现的80帧故障强加为旧短回合的原因。

所有录制器、批次和复放已结束，模型/控制代码未为追求成功改动；本轮仅新增起点参数、精确推理输入记录、显式渲染环境传递及实验工具。既有LightNav与Mesa仿真服务保留、导航指令为零。未commit/push，未重置驱动或其他GPU任务。
