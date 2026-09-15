import { mkdir, writeFile } from "node:fs/promises";
import { dirname } from "node:path";
import { createReadonlyAgent } from "./experiments/readonly-agent.mjs";
import { SimClient } from "./dist/sim-client.js";

const output = process.argv[2];
if (!output) throw new Error("probe.mjs requires an output JSON path");
const client = new SimClient();
const before = await client.status();
const agent = createReadonlyAgent(client);
agent.state.tools = agent.state.tools.filter(({ name }) => ["status", "scene"].includes(name));
const prompt = "请先调用status和scene获取真实状态和图像。用中文简洁描述门和家具的位置，判断当前是否已经完成‘穿过右前方的敞开门，走到门内白色置物架前’。当前只有RGB，另说明能否确认距离目标前表面约1米；不能测量时明确说明。不要移动或确认完成。";
const start = performance.now();
const calls = [];
let firstTextMs;
let firstModelDeltaMs;
agent.subscribe((event) => {
  if (event.type === "tool_execution_start") {
    calls.push({ name: event.toolName, startedMs: performance.now() - start });
  }
  if (event.type === "message_update") {
    const delta = event.assistantMessageEvent.type;
    if (delta.endsWith("_delta")) firstModelDeltaMs ??= performance.now() - start;
    if (delta === "text_delta") firstTextMs ??= performance.now() - start;
  }
});
const timeout = setTimeout(() => agent.abort(), 60000);
try { await agent.prompt(prompt); }
finally { clearTimeout(timeout); }
const report = {
  model: agent.state.model.id, baseUrl: agent.state.model.baseUrl,
  reasoningEffort: "low", prompt, before,
  firstModelDeltaMs, firstTextMs, elapsedMs: performance.now() - start, calls,
  error: agent.state.errorMessage,
  messages: agent.state.messages.filter(({ role }) => role === "assistant").map((message) => ({
    content: message.content.filter(({ type }) => type === "text" || type === "toolCall"),
    stopReason: message.stopReason, usage: message.usage,
  })),
};
await mkdir(dirname(output), { recursive: true });
await writeFile(output, JSON.stringify(report, null, 2));
console.log(`\n${JSON.stringify({ model: report.model, firstTextMs, elapsedMs: report.elapsedMs, calls, error: report.error, output })}`);
if (agent.state.errorMessage) process.exitCode = 1;
