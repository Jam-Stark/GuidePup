# YOLO RGB-D、主动观察与模型对照

后续通用框架修正及其验证范围见 [NAVIGATION_FRAMEWORK.md](NAVIGATION_FRAMEWORK.md)。下列72轮为此前冻结版本的结果，保留原样。

2026-09-15 HKT。YOLO实例分割与同帧深度测量、主动转向观察和跨视角门框关联已经实现。正式Qwen36轮和Gemini3.8Flash36轮均已完成，完整流程通过数均为0/36。Qwen实际到达正确候梯位置并停稳4轮，Gemini0轮；Gemini有33轮在工具参数枚举校验失败后停止，因此本次结果不能单独解释为视觉模型能力差异。

正式运行17:19:30–18:10:44，共72轮，全部保留；所有正式仿真、YOLO和LightNav本地GPU工作使用物理GPU3。前次36轮结果见 [NAVIGATION_STRESS.md](NAVIGATION_STRESS.md)。本轮 [实验合同](../.local/outputs/navigation_yolo/EXPERIMENT.md)、[汇总表](../.local/outputs/navigation_yolo/comparison_01/result_table.md)和[逐轮原始输出](../.local/outputs/navigation_yolo/comparison_01)可追溯结果。收尾已停止本任务仿真和LightNav服务；没有提交或推送Git。

## 正式结果

| 指标 | Qwen3.6-35B-A3B-FP8/ | Gemini3.8Flash |
|---|---:|---:|
| 预登记回合 | 36 | 36 |
| 完整流程通过 | 0 | 0 |
| 实际到达正确候梯位置并停稳 | 4 | 0 |
| 真实整个机身出门 | 7 | 2 |
| Monitor报告完成 / 误报完成 | 0 / 0 | 0 / 0 |
| 正非支撑接触回合 | 0 | 0 |
| 出现工具参数校验失败的回合 | 4 | 33 |
| 实际使用YOLO掩码的回合 | 9 | 5 |
| 成功掩码测量 / 手工矩形测量 | 18 / 52 | 9 / 39 |
| 实际完成观察转向 | 46 | 59 |
| 后拉实际施力 / 锁存 / 问答 / 人工恢复 | 6 / 6 / 6 / 6 | 2 / 2 / 2 / 2 |
| 模型请求次数 | 281 | 180 |
| 各回合墙钟时间合计（秒） | 1252.334 | 1261.427 |

“实际到达并停稳”要求真实完整出门、真实电梯前缘净距0.6–0.9m、朝向及横向位置满足约定、实测速度低于0.05m/s、命令归零和无正非支撑接触；不要求Monitor宣告完成。完整通过还要求Monitor完成。两模型全部终态均为NEEDS_USER。墙钟时间不含服务重置及离线分析，较少请求或较短时间不能作为成功效率指标。

两种起点分布和后拉条件分别如下；整圈组每条件12轮，门向组每条件6轮。后拉只有达到预定施力时机才发生，Qwen12轮、Gemini16轮后拉条件未触发力，不能把它们算成制动或恢复失败。

| 起点 / 条件 | Qwen完整通过 | Qwen真实候梯 / 出门 | Gemini完整通过 | Gemini真实候梯 / 出门 |
|---|---:|---:|---:|---:|
| 整圈朝向 / 普通 | 0/12 | 1 / 1 | 0/12 | 0 / 0 |
| 整圈朝向 / 后拉 | 0/12 | 1 / 1 | 0/12 | 0 / 0 |
| 面向门洞 / 普通 | 0/6 | 1 / 2 | 0/6 | 0 / 0 |
| 面向门洞 / 后拉 | 0/6 | 1 / 3 | 0/6 | 0 / 2 |

这里只比较同一组起点上的两套实际配置，不估计一般场景成功概率，也不以普通/后拉两条件的差值推断暂停的因果影响。

## 运行路径

`prototype/gpu.sh`固定本任务只使用物理GPU3：CUDA按GPU3硬件UUID绑定；`gpu.py`在服务进程内用EGL物理UUID选择渲染设备，当前为EGL索引1。进程内cuda:0对应物理GPU3。LightNav、YOLO和仿真渲染共用这张卡。Qwen和Gemini使用既有远程服务，旧A2控制策略仍使用原有CPU路径。

深度翻译器新增 [semantic_depth.py](sim/semantic_depth.py)。YOLOE-26l先对当前RGB预测door、door frame、elevator door、wall四类实例掩码，再使用该帧RGB严格对齐的depth、内外参测量掩码成员。输出帧内实例ID、类别、分数、像素框、有效深度比例、base水平距离、实测代表点与平面RMS，并附带当前RGB标签覆盖图及掩码成员图。只在Monitor请求观察时推理，同一帧缓存测量；LightNav继续接收原始RGB。

YOLO标签仍是预测。它可能漏检、错标，门框掩码也可能包含多个表面；标签与平面拟合不能证明实体身份。Qwen需要核对实际RGB和所选掩码，明确接受、表示未知或拒绝。无法检测到的局部表面仍可按真实RGB选区测量。没有向感知或决策器提供场景geom名称或真实目标坐标。

## 可执行观察技能

A2相机固定，没有独立头部转轴。`search_target`指定左右、15–90度、需要寻找的证据和理由，执行一次机身原地转向，在实测角度检查点停车；下一次决策根据新RGB-D和实际运动继续规划。`change_view`用于5–45度的局部视角调整。控制器请求零前向速度，实际足式机身位移仍通过50Hz姿态记录。

门洞可以分视角测量：每侧只保留一份门框实测点及来源裁图。组合前由Qwen明确确认sameOpening；历史裁图只在配对审核时按需进入上下文。已确认的门洞/电梯仍交给`follow_target`连续执行。普通记忆保持上一段已接受决策，不展开完整图片历史。人工后拉暂停仍由控制线程锁存，只有明确人工恢复才能解除。

## 比较方案

复用上次18个已接受起点：整圈朝向12个位置姿态×普通/后拉两条件共24轮，面向门洞6个姿态×两条件共12轮；两组分别报告。相同初始配置不代表恢复了完全相同的模型调用或动力学前缀。

先执行Qwen36轮。完整通过要求系统报告完成，并且离线真值核查整个机身真实出门、终点处于真实电梯候梯范围、实测低速停止、命令归零且无正非支撑接触。Qwen少于29轮完整通过、存在误报完成或非支撑接触时，自动执行Gemini3.8Flash的同36轮。不会为提高成绩删除失败或在同一套正式数据内重试。

Qwen为实验室Qwen3.6-35B-A3B-FP8/，thinking关闭、输出上限2048；Gemini为现有ZenMux google/gemini-3.8-flash，reasoning_effort=low、输出上限4096。每次决策记录实际model、provider、endpoint和请求选项。两者共用360秒任务、30秒决策和最多3次恢复限制；模型/服务与推理预算不同，结果是这两套实际配置的比较。

本轮使用NVIDIA GPU3渲染，前次压力测试使用Mesa软件渲染；同时改变了感知与观察策略，所以前后结果不能单独归因为YOLO。

## 当前证据

GPU3 LightNav重放：原80张输入80/80成功；把第80张改为第79张仍80/80成功；旧失败图独立输入1/1成功。共161次，未改变模型/历史处理代码。它证明此GPU3运行环境没有复现旧80帧错误，不证明旧CUDA内部原因已被修复。详见 [重放结果](../.local/outputs/navigation_yolo/probes/history_gpu3/summary.json)。

YOLO默认0.25阈值在三张选定仿真图中均无检出。自然照片可高分检出，这三张仿真图候选分数约0.04–0.10，且普通门框被错标电梯门；更换官方E2E检测头未解决。正式配置固定使用NMS分支和0.03候选输出阈值，保留原始低分及待核验性质，不改变任务成功标准。正式回合并非所有候选都低于0.1，例如Gemini两个出门回合的真实电梯候选约0.215–0.218。

三个归档帧的真实实例掩码、RGB覆盖图、mask-only层测量均已保存：[普通门洞](../.local/outputs/navigation_yolo/probes/door_candidates/semantic_depth.png)、[真实电梯](../.local/outputs/navigation_yolo/probes/elevator_candidates/semantic_depth.png)、[前次误认南墙](../.local/outputs/navigation_yolo/probes/false_wall_candidates/semantic_depth.png)及[深度测量证据](../.local/outputs/navigation_yolo/probes/mask_depth_evidence.json)。这是功能与选定帧证据，不是检测准确率。实际环境为Ultralytics8.4.152、Torch2.10.0、Torchvision0.25.0及官方YOLOE-26l-seg/MobileCLIP2权重。

TypeScript构建通过；真实仿真端点已执行同帧观察→按实例ID选取→仅掩码成员深度测量。正式72轮均保存实际GPU绑定及检测配置，见[GPU配置证据](../.local/outputs/navigation_yolo/comparison_01/gpu_configuration_evidence.json)。Qwen的46个观察转向检查点相对请求角度平均绝对误差0.247°、最大0.489°；只描述检查点角度，不代表零平移或后续站姿不漂移。Qwen p12_primary在seq60合并不同观测的门框证据，并真实出门；跨视角路径已执行，成功关联仍不稳定。

## 失败机制与比较限制

Qwen的4个真实候梯回合如下。不能把完整流程0/36全部归因为没有找到电梯或没有走到位置。

| 回合 | 真实前缘净距 | 朝向误差 | 未完整通过的原因 |
|---|---:|---:|---|
| p05_force_resume | 0.7990m | 6.167° | 接受当前终点测量后，完成提议触发“测量后姿态改变”错误 |
| p12_primary | 0.7514m | 2.233° | 终点同一观察的选区三次没有取得可核验证据 |
| t02_force_resume | 0.7866m | 4.569° | 终点决策多次未形成满足证据/恢复条件的提议 |
| t04_primary | 0.7787m | 3.052° | 接受当前终点测量后，完成提议触发“测量后姿态改变”错误 |

前后两次姿态拒绝的观测到终态平移分别约1.32cm、1.39cm，转角约5.87°、6.66°，超过现有测量姿态有效性条件的0.1rad。当前分支抛错并转NEEDS_USER，没有重新观察完成确认；其几何候梯条件实际上仍满足。证据见[姿态与事件记录](../.local/outputs/navigation_yolo/comparison_01/qwen/arrival_verification_pose_evidence.json)。这些是观测/终态采样，不冒充未单独记录的完成请求瞬间姿态。

Qwen还有实际目标关联与动作选择错误：p02_primary把同一根西侧门框的S1、S4两个实例分别当成左右jamb；p01_force_resume曾选桌腿，p04_primary曾选灰墙；部分回合在理由中说应该转向寻找另一侧，实际却调用request_help。标签、实测平面与sameOpening声明均不能保证正确实体关系。

Gemini的33个工具校验失败全部是枚举/常量值不匹配：23个`accept_measurements.association`、9个选区`role`（7个掩码、2个矩形）和1个`recall_memory.reason`。没有把额外字段、null或根对象错误混入此统计；另有p11_primary一次决策超时。

仅有的两个真实出门回合t01_force_resume和t03_force_resume，已经在RGB看到真实双开电梯门，S1代表点也落在真实门板x≈9.87m上。它们随后分别在seq165和seq161调用`select_semantic_instances`，把只允许`target`的role写成`target_surface`和`elevator door`，在接近电梯前终止。这两例支持“目标已看见但请求格式失败”，不支持“视觉未认出电梯”。

Pi0.85.1实际请求中的工具约束使用`anyOf`+`const`；非strict的openai-completions路径没有改写这些枚举。ZenMux官方文档描述向Gemini传递`parametersJsonSchema`，Google原生Schema列出enum/anyOf等子集；没有本次原始网关到Gemini的请求记录，无法确认中间是否保留或改写了const。因此已证明的是输出违反传入Pi的Schema，尚不能把原因唯一归给模型或网关。正式数据内未改变工具协议、修复后重试或替换失败；下一轮有效的模型能力比较需要先解决此枚举协议问题和已有终点确认失败。

## 证据与复现入口

- [Qwen逐轮结果](../.local/outputs/navigation_yolo/comparison_01/qwen/comparison_summary.json)、[Gemini逐轮结果](../.local/outputs/navigation_yolo/comparison_01/gemini/comparison_summary.json)保存每项真值判据、终态原因、模型实际配置及回合目录。
- [工具与交互计数](../.local/outputs/navigation_yolo/comparison_01/feature_usage.json)保留每轮参数错误、工具使用、观察、转向、后拉与模型用量。供应商报告的总token为Qwen2,241,372、Gemini1,699,268；reasoning明细并非均有返回，未按零补齐。
- 真实候梯但流程失败的[Qwen p05后拉回合视频](../.local/outputs/navigation_yolo/comparison_01/qwen/p05_force_resume/camera.mp4)、[p12跨视角回合视频](../.local/outputs/navigation_yolo/comparison_01/qwen/p12_primary/camera.mp4)；正确看到电梯但参数失败的[Gemini t01后拉回合视频](../.local/outputs/navigation_yolo/comparison_01/gemini/t01_force_resume/camera.mp4)。这些不是完整成功视频。
- 两模型各有`manifest.json`、冻结`source_snapshot`和全部36个回合；每回合保存agent/status/interaction日志、视频、物理与v2真值分析，并链接原始仿真RGB-D及YOLO掩码。准备尝试和两个开发回合单列，不混入72轮。

运行入口为`prototype/sim/run_navigation_comparison.py prepare/run --output <new-directory>`，prepare固定读取本次复用的两个历史起点manifest，run先Qwen后按预登记条件执行Gemini。各回合也可由`record_task.py`/`record_monitor.py --model qwen|gemini`驱动。所需YOLO权重和环境位于`.local`，Gemini使用既有本地凭证文件；不把凭证复制到报告或源码快照。

## 官方来源

[Ultralytics YOLOE](https://docs.ultralytics.com/models/yoloe/)描述开放词汇分割与文本类别配置；[Gemini3.8Flash](https://ai.google.dev/gemini-api/docs/models/gemini-3.8-flash)列明图像输入及工具调用能力。实际依赖版本与本地运行结果以实验材料为准。

[ZenMux协议转换](https://zenmux.ai/docs/api/protocol-conversion.html#chat-completions--gemini)、[ZenMux工具调用](https://zenmux.ai/docs/guide/advanced/tool-calls.html)、[Google函数调用](https://ai.google.dev/gemini-api/docs/generate-content/function-calling)及[Google原生Schema类型](https://googleapis.github.io/js-genai/release_docs/interfaces/types.Schema.html)用于界定上述API比较限制。

开发回合development_01发现初始EGL绑定错误：EGL索引3实际对应GPU1，CUDA推理仍在GPU3。该回合已停止并单列，不作为满足GPU限制的测试；随后按硬件UUID校正绑定并重跑。
