import { writeFile } from "node:fs/promises";
import { createNavigationAgent } from "../pi/dist/agent.js";
import { SimClient } from "../pi/dist/sim-client.js";

const agent = createNavigationAgent(new SimClient());
agent.state.tools = agent.state.tools.filter(({ name }) => ["status", "scene"].includes(name));
const started = performance.now();
const calls = [];
let firstTextMs;
let firstModelDeltaMs;
agent.subscribe((event) => {
  if (event.type === "tool_execution_start") {
    calls.push({ name: event.toolName, startedMs: performance.now() - started });
  }
  if (event.type === "message_update" && event.assistantMessageEvent.type === "text_delta") {
    firstTextMs ??= performance.now() - started;
  }
  if (event.type === "message_update" && event.assistantMessageEvent.type.endsWith("_delta")) {
    firstModelDeltaMs ??= performance.now() - started;
  }
});

await agent.prompt("请先调用 status 和 scene 获取真实状态和图像。原目标是 Turn right and go through the doorway，当前图像能否确认已经穿门？请用中文说明依据，不移动机器人。");
const report = {
  model: agent.state.model.id,
  firstModelDeltaMs,
  firstTextMs,
  elapsedMs: performance.now() - started,
  calls,
  error: agent.state.errorMessage,
  messages: agent.state.messages.filter(({ role }) => role === "assistant").map((message) => ({
    content: message.content.filter(({ type }) => type === "text" || type === "toolCall"),
    stopReason: message.stopReason,
    usage: message.usage,
  })),
};
await writeFile(new URL("../../.local/outputs/qwen_pi_readonly.json", import.meta.url), JSON.stringify(report, null, 2));
console.log(`\n${JSON.stringify({ firstModelDeltaMs, firstTextMs, elapsedMs: report.elapsedMs, calls })}`);
if (agent.state.errorMessage) throw new Error(agent.state.errorMessage);
