# Navigation v2：实现与实验

2026-09-15 HKT。Owner确认：先从`feature/a2-migration`工作树同步新workflow，再实现视觉场景改进、单段buffer、目标/站位语义，并进行新版实验。

当前结果：`primary_08`完整完成“出门走到电梯门口”，独立几何、终态门板ROI及接触记录支持本回合结果。78.49秒/11.117m，最终三圆前缘至真实门面0.809m、朝向误差3.52°、无非支撑接触。同版本交互02通过后拉锁存/问答/原任务恢复并真正出门，但电梯搜索失败；不宣称稳定成功率。

[完整任务视频](../.local/outputs/navigation_v2/primary_08/camera.mp4) · [终态画面](../.local/outputs/navigation_v2/primary_08/final.jpg) · [物理结果](../.local/outputs/navigation_v2/primary_08/physical_evaluation.json) · [buffer与目标执行证据](../.local/outputs/navigation_v2/primary_08/goal_execution_evidence.json)


## 工作流与实现

workflow v1.4.0从源工作树未提交的文件定向同步；保留原型项目配置、memory、旧policy和运行环境，未合并整个分支或commit/push。一次离线preflight和配置解析通过；宿主hook发现/信任未验证。详情见[WORKFLOW_SYNC](../.ai/WORKFLOW_SYNC.md)。

- 普通Qwen请求：当前任务/观测＋唯一上一轮动作决策和该决策至今轨迹。选区/测量/审核/问答不滚动buffer；被采纳的新动作归档并替换。按需recall只影响本次上下文。
- `/motion`从50Hz实际位姿累计运动，最多32个展示点；LightNav每次预测单独记录拍摄位姿和时间，首个/最新两个轨迹供模型理解，完整日志保留。
- 同一阶段保持LightNav会话，改变scope显式reset；失效响应收完并丢弃，不与下一次请求错配。
- 已观测目标通过`follow_target`一次交给导航。Python连续执行对齐、接近、最终朝向；内部阶段不唤醒Qwen、不替换buffer。MPC沿用身体/障碍/延迟约束，LightNav继续阶段语义推理与指向记录；状态明确标记`reference_source=metric_goal`。
- 目标保存稳定标识、语义主张、观测引用与实测平面。候梯范围：三圆包络至目标平面0.6–0.9m，朝向±25°，相对所选目标patch横向≤0.65m。这是显式原型约定，不冒充用户说了这些数字。
- 穿门要求整个包络越过观测门平面并留0.35m出口余量；该余量不等于实测门框厚度，离线另查真实门框远端。
- 场景使用内置材质和视觉细节增强；原碰撞布局/门洞/起点、电梯门面、D435i和MPC保持，新增视觉几何无碰撞。没有第三方下载。

[设计](pi/MONITOR_DESIGN.md) · [使用](pi/README.md) · [实验预先说明](../.local/outputs/navigation_v2/EXPERIMENT.md)

## 验证范围

首次整合：TypeScript构建、Main修改Python语法、13个可控提议用例通过。首轮实际失败后增加主动换视角路径和可读几何分类、移除不合理的门框正面限制；重跑受影响的三个用例通过。它们只证明实现关系，不证明视觉识别。

### primary_01

实际任务在第1阶段停止，未导航。Qwen选择过大的左右ROI，并把两个D3水平地面层描述成竖直门框；几何结果拒绝，之后请求人工提供新视角。保留[视频](../.local/outputs/navigation_v2/primary_01/camera.mp4)、[离线结果](../.local/outputs/navigation_v2/primary_01/v2_evidence.md)和全部模型/ROI记录。

这一失败揭示三件具体事：模型需要直接可读的水平/竖直/混合几何；审核阶段需要主动换视角路径；固定门框内侧面也是合法可见表面，不应强制只用正面。后续改动分别对应这些问题，未放宽MPC或注入目标真值。

### primary_02

95.45秒后NEEDS_USER。实际整个三圆包络清空真实门框远端，首次记录sim 30.34秒；后续接近电梯，最终三圆前缘距真实门面2.035米，朝向误差31.5°，未达到显式站位要求。Qwen共29次调用。中途曾把同一面墙的不同区域当成电梯，之后两次ROI确实落在电梯门板；最后反复使用包含站位术语的Approach指令仍被LightNav提前停止。一条Turn right还实际平移1.775米。

针对这些实际问题，下一轮：把可达站位点及其bearing单独提供；引入LightNav同帧指向的重投影提示；让转向通过MPC严格限制前向命令为零；由50Hz实际运动检查转角和短步距离。未向模型提供场景真值坐标，也未放宽身体约束。

[第二轮视频](../.local/outputs/navigation_v2/primary_02/camera.mp4) · [第二轮离线结果](../.local/outputs/navigation_v2/primary_02/v2_evidence.md)

### primary_03

83.80秒后NEEDS_USER，真正的完整出门仍未完成。前三次0.5m请求实际位移约0.502–0.504m；后续LightNav对Go straight输出大幅转向，部分动作只平移0.074m却转了107°。Qwen最后错误描述已出门和需要人工推进状态；实测包络仍未达到门外条件。

下一轮将显式定距/定角请求交给MPC运动参考，LightNav持续按阶段语义目标观察/规划，其stop不覆盖明确的局部原语。同时为出门提供由已测门平面计算的退出点，区别于已经落到身后的门洞中心。状态与日志明确区分reference_source；没有自动fallback或假轨迹。

### primary_04

90.38秒后NEEDS_USER。实际出门完成，首次电梯ROI也选到了真实门板，但Qwen先把负向误差解释成左转，随后多次右转到面朝墙。四次原地转向均正常达到其真实角度检查点，只有最后向墙前进被MPC拒绝；模型“无法执行任何前进或转向”的解释不符合记录。

因此将已识别目标的方向/步长换算交给`follow_target`：Qwen确认目标与阶段意图，导航程序由实测点与实际位姿派发正确左右方向和距离。MPC原语仍记录真实运动，不把模型的文字解释当执行结果。

### primary_05–primary_07

| 回合 | 墙钟时长 / Qwen调用 | 实际结果 | 后续对应改动 |
|---|---|---|---|
| primary_05 | 79.40s / 21 | 已出门；Qwen先把盆栽、后把背景墙作为电梯，最终NEEDS_USER。真实门前三圆净距5.731m。 | 提供语义`search_target`及同帧`inspect_target_hint`，让LightNav指向作为待核验候选。 |
| primary_06 | 98.14s / 21 | 首次和末次选点均落真实电梯门板；到在线候梯点剩0.428m时因转向后目标离开视野而求助。最终三圆门前净距1.232m，朝向误差35.0°。 | `follow_target`改成同一控制任务连续执行局部目标，减少内部小动作之间的模型判断。 |
| primary_07 | 36.79s / 10 | 一个连续目标动作完成出门，真实整个包络sim19.94s清空门框远端。阶段2的LightNav小ROI两层都支持真实门板，但Qwen扩大ROI并选择混合表面，三次审核后NEEDS_USER。 | 保留完整负结果；继续预定制动/问答/恢复交互回合。 |

第7轮已证明连续出门路径及其buffer，但未执行电梯目标的连续接近与最终朝向，不能据此宣称完整目标跟随通过。Qwen对可用小ROI的重新扩选是实际语义/工具使用失败；没有为了该回合放宽平面阈值。

开发回合间代码变化明确记录，不按固定版本成功率统计。视频、原图/裁图、模型提议、50Hz轨迹、接触记录和源码快照均在`.local/outputs/navigation_v2/<回合>/`保留。

### human_pause_question_resume_01

19.43秒，7次Qwen调用。真实−80N后拉触发制动；撤力仍暂停，新图问答和人工恢复接口全部执行。暂停最大实际位移0.379m，零命令不代表机械锁止。

问答出现具体失败：Qwen为“已经出门了吗、现在看到什么、为什么停了”选择`change_goal`，把自己的说明作为新任务，恢复后原两阶段目标被替换，最终NEEDS_USER。因此不把“RESUMED”或接口成功记为原任务恢复成功。

修复：问答只暴露`answer_user`和`recall_memory`，移除模型`change_goal`路径；操作方仍可通过既有`/task`或`/run`明确换目标。同步移除连续目标导航已不再需要的逐小步提示，说明门板测距只需可辨认的小块表面。TypeScript构建及与此问题对应的现有暂停问答用例通过。

### human_pause_question_resume_02

68.51秒，12次Qwen调用。−80N请求至50Hz首个制动锁存0.66sim秒；随后至人工恢复的160个控制采样有效指令均为零，撤力仍保持暂停。暂停最大实际位移0.319m；这说明旧policy仍是柔顺站立，并非位置锁死。

暂停问答使用新图和原运动buffer，正确说尚未完全出门、外力暂停且不是前方障碍，但“力矩刹车”术语不准确。原任务ID及两个阶段保留，人工恢复后继续同一门洞目标，整个包络sim30.20s清空真实门框远端。跨暂停前后同阶段LightNav仍是同一session。

电梯搜索随后偏转并给出不可测候选，Qwen最终求助，未取得电梯目标的可靠ROI。全回合无非支撑接触。终态仍常驻3.04秒，Qwen调用12→12。原始交互及精确制动采样摘要见[interaction_evidence.json](../.local/outputs/navigation_v2/human_pause_question_resume_02/interaction_evidence.json)。

### primary_08

与交互02相同实现。完整路径为一个出门`follow_target`、一个LightNav语义搜索、一个电梯`follow_target`；14次Qwen调用后COMPLETED。

| 独立指标 | 实测 |
|---|---|
| 墙钟时长 / 实际50Hz累计路程 | 78.487s / 11.1166m |
| 整个包络清空真实门框远端 | sim20.92s；阶段完成sim24.54s时已超出远端0.175m |
| 最终base至真实门面 | 1.4084m |
| 最终三圆包络前缘至真实门面 | 0.8090m，落在显式0.6–0.9m范围 |
| 朝真实门法向误差 | 3.516°，落在±25°范围 |
| 在线最终审核的横向误差 | 0.0541m，相对于已测目标patch；不是电梯整体中心误差 |
| 最终实际平面速度 / 有效指令 | 0.00131m/s / 全零 |
| 全回合非支撑接触 | 未出现（物理步长5ms） |

电梯目标执行sim33.94–65.12s贯穿接近、最终朝向和reached；同一个controlTaskId，中途无Qwen请求。位置到达锁存至最终朝向检查点仍平移0.0448m，真实终态距原指令目标点0.0566m；记录这些漂移，不将位置锁存等同物理位置锁定。

首次target_associated为seq116（审核seq117），代表点`[9.87,1.74882,1.39167]`；最终为seq180（审核seq181），代表点`[9.87,2.08559,0.74447]`。均在真实左叶门面`x=9.87, y∈[1.604,2.45], z∈[0,2.3]`，对应全景/裁图可见银灰门板与中缝。最终画面只见近距离门板，身份还依赖本段出发图及保留的目标关联，不能单凭金属表面就识别任意场景的电梯。

10个带buffer的上下文均指向最近一次已采纳动作，无嵌套buffer；普通视觉请求为当前RGB+上一决策RGB，ROI审核追加当前crop/layers，最多4图。三段motion摘要与原始50Hz累计路程误差分别约−4.4e−16、0、0米（浮点舍入量级），未把LightNav预测拼接当实际轨迹。目标接近段保留55次有效LightNav预测的完整原日志，模型上下文仅首个/最新两段。

每阶段各一个LightNav session，切阶段明确换session；stage2搜索与后续metric_goal共享同一session。Qwen总usage输入91321、输出1169、合计92490 tokens；最大请求13628文本字符、2809工具JSON字符、4图。这不分离图片token开销，也不证明buffer单独导致了成功。

[离线结果](../.local/outputs/navigation_v2/primary_08/v2_evidence.json) · [目标执行证据](../.local/outputs/navigation_v2/primary_08/goal_execution_evidence.json) · [完整模型日志](../.local/outputs/navigation_v2/primary_08/agent.jsonl)

## 当前证据边界与运行状态

- 当前版本有一个无干预完整成功回合和一个中断后继续出门、但目标搜索失败的回合。之前7个primary和交互01负结果保留，代码变化均有源码快照；不能计算成固定版本成功率。
- 场景视觉细节、buffer、目标语义和控制执行同时变化，没有单独验证哪一项导致改善。LightNav负责语义搜索/指向；已测目标通过MPC参考连续执行，不能描述为纯LightNav轨迹闭环。
- Qwen仍会误选ROI、误认背景，或者给出不精确的解释。回忆工具尚无跨阶段长任务效果证据。没有额外训练、更多场景或硬件实验。
- 记录器和临时Monitor已退出；仿真保持电梯前LOCAL_DONE、导航命令为零。既有LightNav及仿真tmux服务继续保留。未commit/push。
