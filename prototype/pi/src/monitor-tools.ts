import { Type } from "typebox";
import { StringEnum, type Tool } from "@earendil-works/pi-ai";
const text = (description: string) => Type.String({ description, minLength: 1, maxLength: 700 });
export function planningContext(originalGoal: string, extra: unknown) {
  return { originalGoal, extra, rules: [
    "默认用一个阶段保留完整请求，让LightNav连续执行途中路径；只有需要独立决策的终态才拆分。单阶段时，clause直接完整复制originalGoal，包括朝向、停止和其他末尾要求。",
    "多阶段时clause必须是用户原文的连续片段，按顺序拼接后覆盖完整原文；不能删掉后续目标或用当前观察替换用户要求。",
    "kind依据最终要达成的内外关系选择，不依据途中是否经过走廊或门口：approach_target=接近实体并停在外侧；pass_doorway=整机穿过开口或进入空间。",
    "例如走到柜子前朝向它并停下属于approach_target；进入另一个房间并停下属于pass_doorway。无需把途中路径另建成穿门阶段。",
    "instruction使用完整的简短英文，保留该阶段的目标身份、终态位置关系、朝向及停止要求。用户未写数值距离时省略distanceM，不得猜测。",
    "英文指令不新增用户未提出、实际RGB也未观察到的外观或标牌假设。用户给对象的编号不自动意味着物体上贴有可读编号；同时保留用户给出的中间、左侧等位置关系。",
    "若extra提供被拒绝计划，对照完整originalGoal修正具体遗漏，重新提交完整plan_task；所有被拒绝的提议都没有执行。",
  ] };
}
export const help: Tool = { name: "request_help", description: "仅当现有观测工具和动作能力确实无法继续时暂停求助，用中文说明具体困难。正常移动、换视角、阶段PENDING和阶段推进都由你自主处理，无需人工批准；不能要求用户代替你调用阶段完成工具。",
  parameters: Type.Object({ reason: text("Actual limitation or failure"), needed: text("Specific help needed") }) };
export const plan: Tool = { name: "plan_task", description: "可将整个用户请求作为一个目标直接交给LightNav，也可自主拆分子目标；无需阶段认证。目标类型：pass_doorway=整个机身出门/穿门/进入房间，approach_target=在指定实体外接近并停下。穿过走廊等途中路径并入其所通向的终态阶段，不要虚构走廊尽头的门；进房后停下属于同一pass_doorway，不是靠近门板。即使后续目标当前不可见也保留。clause按顺序逐字覆盖用户原文；实际阶段选择和指令由你在执行时决定。",
  parameters: Type.Object({ stages: Type.Array(Type.Object({
    kind: StringEnum(["pass_doorway", "approach_target"]),
    clause: text("连续、逐字摘录的用户原文片段；各阶段片段按顺序拼成整个请求，不能用观察分析代替"),
    target: text("Visually identifiable entity"), instruction: text("Short English instruction for this stage"),
    distanceM: Type.Optional(Type.Number({ minimum: 0.1, description: "Only an EXPLICIT numeric stopping distance in the user's request; otherwise OMIT" })),
  }), { minItems: 1, maxItems: 6 }) }) };
type SelectionStage = "pass_doorway" | "approach_target";
export function selectFor(stage: SelectionStage): Tool { return { name: "select_regions", description: stage === "pass_doorway"
  ? "测量当前任务所指通行开口的可见边界区域，可先测一侧或同时测两侧。只选需要测量的表面小块。Monitor按里程计坐标组织边界证据；多个mask不等于多个物体，也不自动构成开口两侧。缺少目标时可continue_navigation让LightNav继续寻找，无需填写role。"
  : "测量当前任务所指实体的一块可见表面。矩形只包含要测量的区域；是否属于任务目标由你依据用户需求和RGB判断。Monitor自动绑定当前帧和测量用途，无需填写role。",
  parameters: Type.Object({ regions: Type.Array(Type.Object({
    bbox_1000: Type.Array(Type.Integer({ minimum: 0, maximum: 1000 }), { minItems: 4, maxItems: 4,
      description: "归一化矩形[x0,y0,x1,y1]，左上[0,0]右下[1000,1000]；不是原图像素。monitor自动转换成同帧像素。只包住窄实体，勿圈整格。" }),
    description: text("Visible surface in this rectangle, before measuring"),
  }), { minItems: 1, maxItems: stage === "pass_doorway" ? 2 : 1 }) }) }; }
export function reviewFor(stage: SelectionStage, regionCount: number): Tool {
  return { name: "accept_measurements", description: "核对真实RGB裁图与深度层成员，按measurements的相同顺序为每个区域选择一层并确认像素身份。只有实体接受需要confirmed_target；未知时仍可搜索观察。Monitor只计算几何并报告完成证据，不替你选择下一阶段或导航指令。",
    parameters: Type.Object({ selections: Type.Array(Type.Object({
    layerId: text("Select ONE actual layer for this region, not multiple bands"),
    semanticClaim: text("这些真实像素表示什么实体/表面，以及支持其与当前任务关联的视觉证据；不要用几何形状或检测标签代替身份判断"),
    visualIdentity: StringEnum(["confirmed_target", "uncertain", "not_target"],
      { description: "confirmed_target=RGB支持它是当前任务所指的实体/边界；uncertain=尚不能确定；not_target=当前证据表明不是。几何和检测标签不独立证明身份。" }),
  }), { minItems: regionCount, maxItems: regionCount }),
    replacePreviousIdentification: Type.Optional(Type.Boolean({ description: "仅当先前保留的实体识别错误时设true，明确放弃旧识别；同一实体的新视角省略或false。" })),
    ...(stage === "pass_doorway" ? { sameOpening: Type.Optional(Type.Boolean({ description: "空间关系支持分开的边界后，再确认它们在语义上属于当前任务所指的开口。此声明不能认证可通行性；传感器几何必须另行证明两侧相向、通路自由和已测出口。" })) } : {}),
  }) };
}
export function originalPixels(regions: { bbox_1000: number[]; description: string }[], width: number, height: number, stage: SelectionStage) {
  return regions.map(({ bbox_1000: [x0, y0, x1, y1], ...region }) => ({ ...region,
    role: stage === "pass_doorway" ? "boundary" as const : "target" as const,
    bbox_px: [Math.floor(x0 * width / 1000), Math.floor(y0 * height / 1000),
      Math.ceil(x1 * width / 1000), Math.ceil(y1 * height / 1000)] }));
}
export const navigate: Tool = { name: "navigate", description: "仅在控制受阻或缺少完成几何的具体视角时，提出一次有界恢复动作。正常搜索和接近交给continue_navigation，不能逐次替LightNav安排转角或步长。turn在局部控制器身体约束下转向；advance必须设置advanceMeters。无倒退能力。",
  parameters: Type.Object({ maneuver: StringEnum(["advance", "turn_left", "turn_right"]),
    instruction: text("Short English LightNav instruction; do not request backward motion"),
    turnDegrees: Type.Optional(Type.Number({ minimum: 5, maximum: 90, description: "Requested measured turn checkpoint for turn maneuvers, in degrees; default 45 for scanning. It is not exact angle control." })),
    advanceMeters: Type.Optional(Type.Number({ minimum: 0.1, maximum: 2, description: "Optional bounded straight move measured by estimated displacement. Use short moves toward the standing point after aligning; this is a local action length, not an invented user stopping distance." })),
    expectedEffect: text("Observable result expected; why this differs from past failures") }) };
export const answer: Tool = { name: "answer_user", description: "Answer using the current images/status supplied now. Distinguish sensor facts, visual judgments and unknowns. Do not invent previous observations or claim unverified task completion.",
  parameters: Type.Object({ text: text("Chinese answer based on the actual current observation and task history") }) };
export const recall: Tool = { name: "recall_memory", description: "Read at most two archived decision segments of this task when the one-segment buffer is insufficient. Does not execute motion or replace the current buffer.",
  parameters: Type.Object({ reason: text("当前需要查回什么历史证据，以及为什么上一段buffer不足"),
    failedAttemptsOnly: Type.Optional(Type.Boolean({ description: "只查找受阻/无进展的历史动作时设true；否则查最近任务片段" })),
    stageIndex: Type.Integer({ minimum: 0, description: "Zero-based stage whose earlier evidence is needed" }) }) };

export const changeView: Tool = { name: "change_view", description: "A2相机固定，无独立头部关节。当前选区无法确认或测量失败时，主动小角度转动整个机身以取得新的观察；实测角度检查点停车，再规划下一步。它只请求观察用转向，不接受当前测量，也不前进或完成阶段。",
  parameters: Type.Object({ direction: StringEnum(["left", "right"]),
    degrees: Type.Number({ minimum: 5, maximum: 45 }),
    instruction: text("English observation instruction sent unchanged to LightNav"),
    reason: text("新的视角应当解决什么具体观察问题") }) };

export const continueNavigation: Tool = { name: "continue_navigation", description: "由你生成当前阶段的完整英文语义指令，原样交给LightNav持续搜索、转向和接近；同一目标保留视觉历史。STOP后结合monitorFeedback生成真实纠正：未核验不等于确定未通过；目标身份错误时不要要求穿过错误目标。Monitor不会替你补写reason。若你判断应执行另一个阶段，使用select_stage；缺少完成证明不禁止改变当前执行目标。",
  parameters: Type.Object({ reason: text("Your progress assessment and reason for this navigation decision"),
    instruction: text("Complete English instruction to send unchanged to LightNav; include any semantic correction yourself") }) };

export function selectStage(stageCount: number): Tool { return { name: "select_stage",
  description: "由你根据原始任务、当前RGB、历史和传感器运动选择计划中的执行阶段，并生成该阶段完整英文指令。前序完成尚未证实也可换到后续阶段或返回补证；被离开的未核验阶段仍待核验，是否完成由你结合全部证据判断。阶段切换会开始新的目标会话；同阶段改指令请用continue_navigation保留历史。",
  parameters: Type.Object({ stageIndex: Type.Integer({ minimum: 0, maximum: stageCount-1, description: "Zero-based stage index from plan" }),
    reason: text("Explain your progress judgment, including any uncertainty about prior stages"),
    instruction: text("Complete English instruction for the chosen stage, sent unchanged to LightNav") }) }; }

export const precisionNavigation: Tool = { name: "follow_measured_target",
  description: "由你决定使用当前已认证开口/目标的实测几何进行精确通过或停靠。只使用已审核的同一目标，几何控制器计算米制轨迹并保持原身体/净距约束；此提议不证明任务完成。",
  parameters: Type.Object({ instruction: text("English instruction for the current measured goal, sent unchanged to LightNav"),
    reason: text("Why to execute the measured local goal now") }) };

export function selectSemanticFor(stage: SelectionStage, instanceIds: string[]): Tool { return { name: "select_semantic_instances", description: "选择当前semanticDepth的掩码ID进行同帧深度测量。mask_relations给出重叠和空间关系；多个ID可能观测同一实体，不能按ID数量推断实体数量或关系。Monitor根据动作类型设置测量用途，无需填写role。随后核对真实RGB与层成员的任务关联；没有合适掩码时可用select_regions。",
  parameters: Type.Object({ regions: Type.Array(Type.Object({
    instanceId: StringEnum(instanceIds, { description: "当前帧的真实掩码ID，只是检测引用，不是永久实体ID" }),
    description: text("What the CURRENT RGB suggests this instance depicts, not just its detector label"),
  }), { minItems: 1, maxItems: stage === "pass_doorway" ? 2 : 1 }) }) }; }

export const inspectHint: Tool = { name: "inspect_target_hint", description: "对当前图中LightNav目标指向附近的一小块自动同帧测量，然后给你RGB裁图与深度层核验身份。指向只是模型候选，不是目标真值。这个工具避免你重新手写像素坐标。",
  parameters: Type.Object({ reason: text("需要核验的当前目标") }) };

export const completeTask: Tool = { name: "complete_task",
  description: "由你依据完整原始任务、当前与动作起点RGB、估计运动和执行结果判断任务完成并结束。逐项说明指定实例身份、终态位置关系、用户朝向/停止要求的真实依据，区分类别与具体实例、预测与执行、已知与未知。STOP、检测标签和累计路程都不单独证明到达。只读观测不要求阶段证书，不替你决定；物理结果由离线评估独立判定。",
  parameters: Type.Object({ reason: text("Why the complete original task is now fulfilled"),
    evidence: text("Specific images, executed movement and stopping evidence supporting your judgment; include uncertainty") }) };
