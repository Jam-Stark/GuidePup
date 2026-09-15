import {
  hasApi, validateToolCall,
  type ImageContent, type TextContent, type Tool, type Message,
} from "@earendil-works/pi-ai";
import { configuredModel } from "./model.js";
import type { ObservationImage } from "./sim-client.js";

export type DecisionRecorder = (kind: string, data: unknown) => void | Promise<void>;

export interface DecisionInput {
  purpose: string;
  context: unknown;
  images: ObservationImage[];
  tools: Tool[];
  signal: AbortSignal;
}

export interface DecisionResult {
  name: string;
  args: unknown;
  text: string;
  usage: unknown;
  elapsedMs: number;
}

const SYSTEM_PROMPT = `你是 A2 的任务决策器，用中文表达必要说明。
本次purpose指定要完成的一步。结合当前画面、任务状态和唯一上一轮buffer判断进展；图片标签区分当前与历史。
你已被授权自主提出当前工具允许的观察和导航；后续阶段PENDING只是顺序等待，不是人工批准点。currentStep说明现在要解决的事情，可用工具列表就是当前动作入口。
完成条件只决定阶段何时结束，不是搜索或观察的前提。目标不可见或缺少关系证据时，可以直接转向观察。目标语义来自用户需求和当前任务，不预设目标的外观模板。Monitor根据语义确认与最新实测状态自动推进阶段和确认完成。
LightNav轨迹是预测，actualMotion才是实际运动。像素语义是模型判断，深度只证明测到了对应表面；无法确认就说明未知或选择观察动作。
可用recall_memory按需读取更早的任务片段。人工暂停只能由人解除，控制事件给出的停下原因优先于猜测。
必须且只能提交一个当前可用工具的真实 toolCall 提议，不能用文本 JSON 或工具标记代替。
你的有效工具提议会由Monitor执行并返回实际结果，无需再请求用户批准正常动作。
不要提交多个工具调用，也不要声称提议已经执行或任务已经完成。`;

export class PiDecision {
  private readonly configured = configuredModel();

  constructor(private readonly record?: DecisionRecorder) {}

  async decide({ purpose, context, images, tools, signal }: DecisionInput): Promise<DecisionResult> {
    signal.throwIfAborted();
    const { models, model } = this.configured;
    if (!hasApi(model, "openai-completions")) {
      throw new Error("PiDecision requires the configured OpenAI-compatible model");
    }
    if (tools.length === 0) throw new Error("PiDecision requires a nonempty phase tool set");
    const started = performance.now();
    const requestText = `purpose: ${purpose}\ncontext: ${JSON.stringify(context)}`;
    const content: (TextContent | ImageContent)[] = [{ type: "text", text: requestText }];
    for (const { kind, label, image } of images) {
      content.push({ type: "text", text: `${kind}: ${label}` }, image);
    }
    const maxTokens = Number(process.env.PI_MAX_TOKENS ?? 2048);
    const reasoningEffort = this.configured.thinkingLevel === "off" ? undefined : this.configured.thinkingLevel;
    const messages: Message[] = [{ role: "user", content, timestamp: Date.now() }];
    for (let attempt = 0; attempt < 2; attempt++) {
      const requestStarted = performance.now();
      await this.record?.("decision_request", {
        purpose, attempt, model: model.id, provider: model.provider, baseUrl: model.baseUrl,
        reasoningEffort: reasoningEffort ?? "off",
        systemPrompt: SYSTEM_PROMPT, context, tools,
        validationFeedback: attempt ? messages[messages.length-1] : undefined,
        textChars: SYSTEM_PROMPT.length + content.reduce((sum, item) =>
          sum + (item.type === "text" ? item.text.length : 0), 0),
        toolsJsonChars: JSON.stringify(tools).length,
        imageCount: images.length,
        images: images.map(({ kind, label, image }) => ({
          kind, label, mimeType: image.mimeType, base64Chars: image.data.length,
        })),
        maxTokens, toolChoice: "required", parallelToolCalls: false,
      });
      const response = await models.complete(model, {
        systemPrompt: SYSTEM_PROMPT,
        messages,
        tools,
      }, {
        signal, maxTokens, maxRetries: 0, reasoningEffort,
        toolChoice: "required",
        samplingParams: { parallel_tool_calls: false },
      });
      const elapsedMs = performance.now() - requestStarted;
      await this.record?.("decision_response", { purpose, attempt, elapsedMs, response });
      signal.throwIfAborted();
      if (response.stopReason === "error" || response.stopReason === "aborted"
        || response.stopReason === "length") {
        throw new Error(`Pi decision ${response.stopReason}: ${response.errorMessage ?? "no complete proposal"}`);
      }
      const calls = response.content.filter(item => item.type === "toolCall");
      if (calls.length !== 1) {
        throw new Error(`Pi decision requires exactly one tool proposal; received ${calls.length}`);
      }
      const call = calls[0];
      let args: unknown;
      try {
        args = validateToolCall(tools, call);
      } catch (error) {
        await this.record?.("tool_validation_error", { purpose, attempt, tool: call.name,
          arguments: call.arguments, error: String(error), executed: false });
        if (attempt === 1) throw error;
        // Match Pi's tool-result protocol: expose the actual error for one correction.
        messages.push(response, { role: "toolResult", toolCallId: call.id, toolName: call.name,
          content: [{ type: "text", text: `提议未执行。请根据以下参数错误和当前工具定义，重新提交一个有效工具调用：\n${String(error)}` }],
          isError: true, timestamp: Date.now() });
        continue;
      }
      return {
        name: call.name, args,
        text: response.content.filter(item => item.type === "text").map(item => item.text).join("\n"),
        usage: response.usage, elapsedMs: performance.now() - started,
      };
    }
    throw new Error("No valid tool proposal after parameter correction");
  }
}
