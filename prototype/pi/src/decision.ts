import {
  hasApi, validateToolCall,
  type ImageContent, type TextContent, type Tool, type Message,
} from "@earendil-works/pi-ai";
import { readFileSync } from "node:fs";
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

const INSTRUCTION_RULES = readFileSync(new URL("../prompts/lightnav-instruction-rules.txt", import.meta.url), "utf8");

const SYSTEM_PROMPT = `你是 A2 的任务决策器，用中文表达必要说明。
结合原始任务、当前RGB、上一动作起点RGB、估计运动和实际执行命令判断进展。图片标签区分当前与历史。
你自主选择完整目标或子目标、导航指令、继续策略和任务完成；可以把整个任务直接交给LightNav，无需逐阶段认证。
Monitor是只读语义证据提供器和执行协调器，不监督阶段、不改写指令、不用证书禁止推进或结束。未证实不等于未发生；明确区分观测、模型判断和未知。
LightNav持续负责搜索、转向、接近和局部轨迹。continue_navigation的完整英文instruction原样发送，同目标保留历史；reason不替代instruction。正常动作无需逐次指定角度或步长。
LightNav STOP是正常返回事件，不等于任务完成。执行受阻、无实际进展和错误也会返回；由你决定继续、修改指令、换目标、换视角、请求帮助或结束。
用户指定编号、序号或相对位置时，辨认具体实例：读取可见标识，或比较候选集合的位置关系。共有外观、类别标签和附近设施只能支持类别，不能证明是指定实例；看不清时保持未知并选择继续搜索或换视角。不要默认最近的候选就是用户目标。
接近目标并停下的任务，同时判断目标身份、是否正对、横向是否对齐及停靠距离。context.stage.completionConvention给出当前停靠约定；像素深度median、机身到平面净距和行走路程是不同量，不能互相替代，也不能把数米外说成符合亚米级停靠要求。
odom的XY/yaw是相对启动帧的估计坐标，不是目标坐标或目标朝向误差。没有已辨认目标的相对几何时，不能根据odom数值断言已经对齐、正对或到达。
完成理由必须忠于在线证据。STOP且visible=false没有提供目标可见证据；检测置信度不证明身份，失败的平面测量不证明已对准或距离正确。缺少证据时由你选择下一步，不要把未知写成已经完成。
判断整个原始任务完成时调用complete_task，给出具体理由和证据；未满足的观测参考和未认证阶段不构成结束门槛。该结果是Qwen声明，仿真物理成功由离线评估独立判定。
LightNav轨迹是预测，estimatedMotion是传感器估计，不是真值或已执行计划。结合effectiveCommand、localController和前后图像判断动作有没有执行。
测量工具只供参考，无需为完成反复接受同一失败测量。精确几何执行和有界动作由你显式选择，局部控制器决定物理可行性。
可用recall_memory读取更早片段。人工暂停只能由人解除；实际控制事件优先于猜测。
必须且只能提交一个当前可用工具的真实toolCall；不能用文本JSON代替，不要声称尚未执行的提议已经执行。
工具名就是实际动作。continue_navigation才会继续移动，change_view才会转身观察，request_help才会报告无法继续；complete_task只结束整个任务，不会执行reason中描述的新计划。若理由仍是受阻、身份不明、需要重新接近或打算尝试，就选择对应的下一动作，不能用complete_task代替。
正常动作无需用户再次批准。\n\n${INSTRUCTION_RULES}`;


export class PiDecision {
  private readonly configured = configuredModel();

  constructor(private readonly record?: DecisionRecorder, private readonly systemPrompt = SYSTEM_PROMPT) {}

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
        systemPrompt: this.systemPrompt, context, tools,
        validationFeedback: attempt ? messages[messages.length-1] : undefined,
        textChars: this.systemPrompt.length + content.reduce((sum, item) =>
          sum + (item.type === "text" ? item.text.length : 0), 0),
        toolsJsonChars: JSON.stringify(tools).length,
        imageCount: images.length,
        images: images.map(({ kind, label, image }) => ({
          kind, label, mimeType: image.mimeType, base64Chars: image.data.length,
        })),
        maxTokens, toolChoice: "required", parallelToolCalls: false,
      });
      const response = await models.complete(model, {
        systemPrompt: this.systemPrompt,
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
        const feedback = `提议未执行。请根据以下参数错误和当前工具定义，重新提交一个有效工具调用：\n${String(error)}`;
        if (call.arguments === null || typeof call.arguments !== "object" || Array.isArray(call.arguments)) {
          // Qwen's history template requires a mapping for tool-call arguments.
          // Preserve the rejected proposal as text instead of replaying an invalid call.
          messages.push({ role: "user", timestamp: Date.now(),
            content: `上一次未执行的无效工具提议：${JSON.stringify(call)}\n${feedback}` });
        } else {
          messages.push(response, { role: "toolResult", toolCallId: call.id, toolName: call.name,
            content: [{ type: "text", text: feedback }],
            isError: true, timestamp: Date.now() });
        }
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
