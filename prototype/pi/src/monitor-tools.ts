import { Type } from "typebox";
import { StringEnum, type Tool } from "@earendil-works/pi-ai";
const text = (description: string) => Type.String({ description, minLength: 1, maxLength: 700 });
export const help: Tool = { name: "request_help", description: "仅当现有观测工具和动作能力确实无法继续时暂停求助，用中文说明具体困难。正常移动、换视角、阶段PENDING和阶段推进都由你自主处理，无需人工批准；不能要求用户代替你调用阶段完成工具。",
  parameters: Type.Object({ reason: text("Actual limitation or failure"), needed: text("Specific help needed") }) };
export const plan: Tool = { name: "plan_task", description: "只根据用户原文分解完整任务。每阶段对应一个动作：pass_doorway=整个机身出门/穿门，approach_target=接近指定终点并停下。即使后续目标当前不可见，也必须列入计划。clause 按顺序逐字摘录用户原文，拼起来必须覆盖全部原文；不要写观察、解释或新条件。",
  parameters: Type.Object({ stages: Type.Array(Type.Object({
    kind: StringEnum(["pass_doorway", "approach_target"]),
    clause: text("连续、逐字摘录的用户原文片段；各阶段片段按顺序拼成整个请求，不能用观察分析代替"),
    target: text("Visually identifiable entity"), instruction: text("Short English instruction for this stage"),
    distanceM: Type.Optional(Type.Number({ minimum: 0.1, description: "Only an EXPLICIT numeric stopping distance in the user's request; otherwise OMIT" })),
  }), { minItems: 1, maxItems: 6 }) }) };
type SelectionStage = "pass_doorway" | "approach_target";
export function selectFor(stage: SelectionStage): Tool { return { name: "select_regions", description: stage === "pass_doorway"
  ? "测量当前任务所指通行开口的可见边界区域，可先测一侧或同时测两侧。只选需要测量的表面小块。Monitor按世界坐标组织边界证据；多个mask不等于多个物体，也不自动构成开口两侧。缺少视角时可直接search_target，无需填写role。"
  : "测量当前任务所指实体的一块可见表面。矩形只包含要测量的区域；是否属于任务目标由你依据用户需求和RGB判断。Monitor自动绑定当前帧和测量用途，无需填写role。",
  parameters: Type.Object({ regions: Type.Array(Type.Object({
    bbox_1000: Type.Array(Type.Integer({ minimum: 0, maximum: 1000 }), { minItems: 4, maxItems: 4,
      description: "归一化矩形[x0,y0,x1,y1]，左上[0,0]右下[1000,1000]；不是原图像素。monitor自动转换成同帧像素。只包住窄实体，勿圈整格。" }),
    description: text("Visible surface in this rectangle, before measuring"),
  }), { minItems: 1, maxItems: stage === "pass_doorway" ? 2 : 1 }) }) }; }
export function reviewFor(stage: SelectionStage, regionCount: number): Tool {
  return { name: "accept_measurements", description: "核对真实RGB裁图与深度层成员，按measurements的相同顺序为每个区域选择一层并确认像素身份。只有实体接受需要confirmed_target；未知时仍可搜索观察。几何量由Monitor计算，阶段满足条件后由Monitor自动推进。",
    parameters: Type.Object({ selections: Type.Array(Type.Object({
    layerId: text("Select ONE actual layer for this region, not multiple bands"),
    semanticClaim: text("这些真实像素表示什么实体/表面，以及支持其与当前任务关联的视觉证据；不要用几何形状或检测标签代替身份判断"),
    visualIdentity: StringEnum(["confirmed_target", "uncertain", "not_target"],
      { description: "confirmed_target=RGB支持它是当前任务所指的实体/边界；uncertain=尚不能确定；not_target=当前证据表明不是。几何和检测标签不独立证明身份。" }),
  }), { minItems: regionCount, maxItems: regionCount }),
    replacePreviousIdentification: Type.Optional(Type.Boolean({ description: "仅当先前保留的实体识别错误时设true，明确放弃旧识别；同一实体的新视角省略或false。" })),
    ...(stage === "pass_doorway" ? { sameOpening: Type.Optional(Type.Boolean({ description: "空间关系支持分开的边界后，再确认它们之间确实是当前任务所指的同一个可见通行开口。几何分离不证明语义关系。" })) } : {}),
  }) };
}
export function originalPixels(regions: { bbox_1000: number[]; description: string }[], width: number, height: number, stage: SelectionStage) {
  return regions.map(({ bbox_1000: [x0, y0, x1, y1], ...region }) => ({ ...region,
    role: stage === "pass_doorway" ? "boundary" as const : "target" as const,
    bbox_px: [Math.floor(x0 * width / 1000), Math.floor(y0 * height / 1000),
      Math.ceil(x1 * width / 1000), Math.ceil(y1 * height / 1000)] }));
}
export const navigate: Tool = { name: "navigate", description: "提出一次有界局部动作，用于探索换位或受阻恢复。turn在MPC身体约束下转向；advance必须设置advanceMeters。已有确认目标时通常由follow_target连续执行，只有需要改变受阻位置才使用此局部动作。无倒退能力。",
  parameters: Type.Object({ maneuver: StringEnum(["advance", "turn_left", "turn_right"]),
    instruction: text("Short English LightNav instruction; do not request backward motion"),
    turnDegrees: Type.Optional(Type.Number({ minimum: 5, maximum: 90, description: "Requested measured turn checkpoint for turn maneuvers, in degrees; default 45 for scanning. It is not exact angle control." })),
    advanceMeters: Type.Optional(Type.Number({ minimum: 0.1, maximum: 2, description: "Optional bounded straight move measured by actual displacement. Use short moves toward the standing point after aligning; this is a local action length, not an invented user stopping distance." })),
    expectedEffect: text("Observable result expected; why this differs from past failures") }) };
export const answer: Tool = { name: "answer_user", description: "Answer using the current images/status supplied now. Distinguish sensor facts, visual judgments and unknowns. Do not invent previous observations or claim unverified task completion.",
  parameters: Type.Object({ text: text("Chinese answer based on the actual current observation and task history") }) };
export const recall: Tool = { name: "recall_memory", description: "Read at most two archived decision segments of this task when the one-segment buffer is insufficient. Does not execute motion or replace the current buffer.",
  parameters: Type.Object({ reason: text("当前需要查回什么历史证据，以及为什么上一段buffer不足"),
    failedAttemptsOnly: Type.Optional(Type.Boolean({ description: "只查找受阻/无进展的历史动作时设true；否则查最近任务片段" })),
    stageIndex: Type.Integer({ minimum: 0, description: "Zero-based stage whose earlier evidence is needed" }) }) };

export const changeView: Tool = { name: "change_view", description: "A2相机固定，无独立头部关节。当前选区无法确认或测量失败时，主动小角度转动整个机身以取得新的观察；实测角度检查点停车，再规划下一步。它只请求观察用转向，不接受当前测量，也不前进或完成阶段。",
  parameters: Type.Object({ direction: StringEnum(["left", "right"]),
    degrees: Type.Number({ minimum: 5, maximum: 45 }), reason: text("新的视角应当解决什么具体观察问题") }) };

export const followTarget: Tool = { name: "follow_target", description: "连续执行当前已确认几何目标：穿越阶段去开口另一侧的退出点；接近阶段去实测表面对应的约定停点并面向它。导航程序完成内部转向和接近；到达、受阻或人工事件才返回。无需手写中间角度、步长或批准小动作。",
  parameters: Type.Object({ reason: text("根据当前目标和证据，为什么应继续前往该目标") }) };

export const searchTarget: Tool = { name: "search_target", description: "目标不可见、当前实体尚未识别、或缺少空间关系证据时，都可直接使用此观察动作，不需要先识别目标或完成测量。选择左右和角度，机身转向后停车拍新RGB-D，再规划下一步；它不接受实体身份、不前进、不完成阶段。",
  parameters: Type.Object({ direction: StringEnum(["left", "right"]),
    degrees: Type.Number({ minimum: 15, maximum: 90 }),
    soughtEvidence: text("这次新视角希望取得哪项当前缺失的任务相关证据"),
    reason: text("Why this direction adds information relative to the current view and previous measured motion") }) };

export function selectSemanticFor(stage: SelectionStage, instanceIds: string[]): Tool { return { name: "select_semantic_instances", description: "选择当前semanticDepth的掩码ID进行同帧深度测量。mask_relations给出重叠和空间关系；多个ID可能观测同一实体，不能按ID数量推断实体数量或关系。Monitor根据动作类型设置测量用途，无需填写role。随后核对真实RGB与层成员的任务关联；没有合适掩码时可用select_regions。",
  parameters: Type.Object({ regions: Type.Array(Type.Object({
    instanceId: StringEnum(instanceIds, { description: "当前帧的真实掩码ID，只是检测引用，不是永久实体ID" }),
    description: text("What the CURRENT RGB suggests this instance depicts, not just its detector label"),
  }), { minItems: 1, maxItems: stage === "pass_doorway" ? 2 : 1 }) }) }; }

export const inspectHint: Tool = { name: "inspect_target_hint", description: "对当前图中LightNav目标指向附近的一小块自动同帧测量，然后给你RGB裁图与深度层核验身份。指向只是模型候选，不是目标真值。这个工具避免你重新手写像素坐标。",
  parameters: Type.Object({ reason: text("需要核验的当前目标") }) };
