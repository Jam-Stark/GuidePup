# 通用任务框架修正与验证

2026-09-15 HKT。已修正当前状态提示、工具参数、空间证据关系和自动阶段完成流程。YOLO词表保留为任务配置；通用框架不写入特定目标的外观模板或类别接受规则。目标语义来自用户需求及任务状态，模型结合实际RGB判断身份；框架组织测量、执行动作并核验几何条件。

原来72轮的冻结结果仍见 [NAVIGATION_YOLO.md](NAVIGATION_YOLO.md)，不改写其分母或失败。本轮证据在 [navigation_framework](../.local/outputs/navigation_framework/EXPERIMENT.md)。三次真实模型上下文回放和三项真实数据路径验证已完成。Owner随后将分配从GPU3改为GPU0；两次新仿真启动都在NVIDIA初始化时阻塞，没有新完整回合结果。

## 已查明的框架问题

| 路径 | 原始证据 | 可以得出的结论 |
|---|---|---|
| 搜索条件 | Qwen p03_primary context54/request55→response59；p08_force_resume context29/request30→response33。两者只有一份保留边界、没有portal，工具包含search_target，却仍收到要求使用不存在的follow_target/退出点的规则。搜索描述同时混入实体接受限制。 | 执行代码没有“先确认才能搜索”的硬循环，但提示与当前可执行状态矛盾，可能诱发误读。不能单凭模型回复把原因归给导航能力。 |
| 实例与实体关系 | p02_primary把S1/S4选为左右侧；原始mask重叠1698px，IoU0.284708，较小mask重叠48.33%；所选D4代表点XY距离0.11648m。 | 原框架未把已有空间关系组织给模型。本例没有形成错误portal：先因工具参数失败停止；原0.7m宽度检查本来也会拒绝此配对。 |
| 完成流程 | p05_force_resume、t04_primary已有当前ROI确认，实测候梯条件满足，却在追加的模型完成提议之后因超过0.1rad姿态变化抛错求助。 | 世界坐标目标证据与站立姿态漂移被混为同一种失效；阶段完成还被多余的模型调用阻隔。 |
| 工具接口 | Gemini33轮枚举错误：association23、role9、recall reason1。Pi依赖实际提供了面向Google等接口的StringEnum，原定义使用anyOf/const。 | 应先修参数表达和框架负担。先前数据不能用来断言Gemini视觉更差；网关是否改写约束仍没有原始下游请求证据。 |

完整回放输入保留对应system/context/tools、响应、图像源和事件序号，见[输入索引](../.local/outputs/navigation_framework/replay_inputs/index.json)。图像不是按两份数组的下标拼接：当前传感器归档包含RGB、网格图和语义图，实际请求只取指定图像，并另从buffer追踪历史图。初次离线提取映射错误已在任何模型调用前纠正；实际回放逐项匹配原图字节、格式和顺序。

## 当前职责划分

`TaskMonitor.context()`根据当前证据、调用用途和实际工具集合生成`currentStep`。没有已测几何目标时只描述搜索/测量；有目标且当前允许行动时才给出连续执行要求。问答只描述回答任务，测量审核只描述核对证据/继续观察；它们不会收到当前不可用的follow_target指令。阶段完成条件明确属于Monitor的结束判据，不作为观察前提。通用提示不列出目标外观或类别接受模板。

工具只保留需要模型判断的参数：测量用途由动作类型自动赋值，移除模型填写的`role`；观察是否首次、是否延续由框架维护，移除`association`枚举。确有前次识别错误时，用明确的`replacePreviousIdentification`布尔值更换证据。固定枚举使用已有Pi `StringEnum`；掩码ID直接枚举当前帧真实ID。回忆理由使用自然语言，筛选失败动作另用布尔参数。

参数校验错误不执行动作，原始错误以Pi标准`toolResult/isError`回传，允许模型在同一30秒决策期限内纠正一次；再次无效仍失败。没有把无效字符串自动转换成可执行动作，也没有吞掉控制或仿真错误。`decision_request`现保存完整上下文和校验反馈，`tool_validation_error`独立记录未执行的原始提议。当前三次新模型回放均首轮有效，尚未在本轮真实模型调用中触发纠正分支。

## 通用空间证据

RGB-D层与掩码新增实际成员像素的世界XYZ支持范围；`mask_relations`提供掩码交集、IoU、小掩码覆盖比例、代表点XY距离及实测支持范围间隔。没有使用场景geom或目标真值，没有按物体类别作配对规则。

S编号仍是帧内检测。通行阶段的B编号表示空间边界证据组，可合并同一位置的多个mask/视角，保留当前来源及测量次数。现有最小可用开口跨度0.7m用于判断两份支持是否能作为可通行开口的两侧；它不是“同一实体”的识别阈值。空间过近的测量不会被计算为两侧；空间分开后仍需模型结合实际图像确认其间是任务所指的同一开口。左右角色在几何目标构造后派生，模型不再给检测ID直接分配左右槽位。

这些是当前局部通行假设下的证据关系，不是通用物体跟踪已解决的证明。支持范围是已观测像素范围，不是完整物体边界；共面、重叠或距离近均不能独立证明实体身份。模型仍可能作出错误视觉判断。

## 执行与完成

删除模型工具`propose_stage_complete`及其执行分支。每次阶段处理、语义审核完成后，Monitor自动检查阶段条件。接近阶段仍要求本阶段已执行并结束的导航、当前观察中已经审核的目标ROI及关联表面；不能只凭历史anchor完成。

完成前再读取最新控制状态，核对generation，并重新计算世界坐标目标上的净距、朝向、横向位置与停止条件。条件变化时回到观察/调整流程；正常站立中的转角变化不再独立导致求助。明确数值ROI距离仍在明显平移后要求重新测量。控制服务接受complete后，Monitor才推进或结束阶段。人工暂停仍取消当前工作，Python拒绝HUMAN_PAUSED下的complete，人工恢复权限不变。

## 本轮验证结果

| 实际记录/模型回放 | 新结果 | 证据范围 |
|---|---|---|
| Qwen p03_primary原始图片与状态 | 有效search_target：左转45° | 一次真实模型提议，未执行物理转向 |
| Qwen p08_force_resume原始图片与状态 | 有效search_target：右转45° | 一次真实模型提议，未执行物理转向 |
| Gemini t01_force_resume原始图片与状态 | 有效select_semantic_instances：S1，无role参数 | 一次真实模型提议，未执行接近 |
| p02原始S1/S4的D4测量 | 一份B证据组，无法作为开口两侧 | CPU重建真实RGB-D/mask后的几何路径 |
| p05原始当前ROI与失败前同generation控制记录 | 进入complete调用；平移0.01290m、转角0.10912rad | 在API调用边界中止，未伪造成功响应 |
| t04同类记录 | 进入complete调用；平移0.01255m、转角0.11553rad | 同上 |

模型回放保持每次原始图像及顺序一致，重建当前框架的提示/工具/关系元数据；所用模型与先前对应配置相同。每个案例只有一次新提议，这支持具体路径改善，不隔离单项提示的因果效果，也不代表全流程成功率。[Qwen回放](../.local/outputs/navigation_framework/probes/qwen_context_01/summary.json)、[Gemini回放](../.local/outputs/navigation_framework/probes/gemini_context_01/summary.json)、[记录数据路径验证](../.local/outputs/navigation_framework/probes/structural_01/summary.json)均保存。

构建通过。另外使用两份真实记录状态检查了问答/审核共4份上下文，均没有不可用的follow_target指令，见[用途上下文检查](../.local/outputs/navigation_framework/probes/phase_context_check.json)。一次限定源码审阅未发现自动完成路径新增的证据或人工权限绕过；这是源码证据，不是新仿真成功。

## 新仿真限制与收尾

预定的Qwen p05_force_resume新回合在120秒启动期限内未就绪；仿真进程停在NVIDIA `nvkms_open_common`，尚未开始任务。随后Gemini t01_force_resume计划未启动。LightNav启动进程实际位于物理GPU3；同期nvidia-smi报告GPU1句柄Unknown Error，尚不能断言两者因果关系。[启动失败记录](../.local/outputs/navigation_framework/startup_failure.json)与setup日志已保留。

Owner随后要求改用GPU0。当前启动配置已改为LIGHTNAV_GPU=0、CUDA按GPU0 UUID绑定、EGL按同UUID匹配索引2；没有保留configure_gpu3别名。GPU0上的LightNav实际加载成功，使用8972MiB；仿真也记录了相同GPU0 CUDA环境，但仍在nvkms_open_common阻塞并超时。Qwen未进入任务，Gemini计划未启动。[GPU0启动证据](../.local/outputs/navigation_framework/startup_failure_gpu0.json)保留进程与设备记录。

换GPU没有解除NVIDIA渲染初始化阻塞；GPU1掉卡是线索，尚未单独验证因果。没有改渲染后端或重置驱动。本任务服务已停止，配置保留GPU0。渲染恢复后，可用run_smoke.py的必选--output参数为两个声明回合指定新目录继续，不覆盖启动失败。当前没有新增完整闭环结果。未commit/push。
