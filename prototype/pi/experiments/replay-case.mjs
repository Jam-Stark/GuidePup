import { mkdir, readFile, writeFile, appendFile } from "node:fs/promises";
import { existsSync } from "node:fs";
import { resolve, join } from "node:path";
import { fileURLToPath } from "node:url";
import { parseArgs } from "node:util";
import { createReadonlyAgent } from "./readonly-agent.mjs";
import { SimClient } from "../dist/sim-client.js";

const { values } = parseArgs({ options: {
  "model-label": { type: "string" },
  cases: { type: "string" },
  "prompt-file": { type: "string" },
  output: { type: "string" },
  "replay-url": { type: "string", default: "http://127.0.0.1:8767" },
  help: { type: "boolean", short: "h" },
} });
if (values.help) {
  console.log(`Static RGB-D comparison; no navigation tools

bash prototype/pi/experiments/compare.sh \\
  --cases initial,doorway,arrival --prompt-file prompt.txt --output /absolute/new-directory

Optional: --replay-url http://127.0.0.1:8767
Runs Qwen, Gemini and DeepSeek sequentially; baseline and enhanced for each case.
Each case: fresh Agent, tool_choice=auto, at most 4 ROI requests, 150 s,
4096 output-token limit per request. No automatic reruns or model fallback.
The worker accepts --model-label qwen|gemini|deepseek when launched separately.`);
  process.exit(0);
}
for (const name of ["model-label", "cases", "prompt-file", "output"]) {
  if (!values[name]) throw new Error(`Missing --${name}`);
}
const expectedIds = {
  qwen: "Qwen3.6-35B-A3B-FP8/",
  gemini: "google/gemini-3.8-flash",
  deepseek: "deepseek/deepseek-v4.1-flash",
};
const modelLabel = values["model-label"];
if (!(modelLabel in expectedIds)) throw new Error("Unknown --model-label");
const caseIds = values.cases.split(",").map(id => id.trim());
if (caseIds.some(id => !id)) throw new Error("--cases requires comma-separated case IDs");
const prompt = await readFile(resolve(values["prompt-file"]), "utf8");
if (!prompt.trim()) throw new Error("Prompt file is empty");
const envFile = fileURLToPath(new URL("../.env", import.meta.url));
if (existsSync(envFile)) process.loadEnvFile(envFile);
// Experiment-only output budget; the normal Qwen configuration stays unchanged.
process.env.PI_MAX_TOKENS = "4096";
process.env.PI_MODEL_MAX_TOKENS = "4096";

const MAX_ROIS = 4;
const TIMEOUT_MS = 150_000;
const outputRoot = resolve(values.output);
await mkdir(outputRoot, { recursive: true });
const modelRoot = join(outputRoot, modelLabel);
await mkdir(modelRoot); // A repeated run must use a new output directory.

class ReplayClient extends SimClient {
  constructor(caseId, presentation) {
    super(values["replay-url"]);
    this.caseId = caseId;
    this.presentation = presentation;
  }

  async replay(path, body, signal) {
    const url = new URL(`/cases/${encodeURIComponent(this.caseId)}/${path}`, this.baseUrl);
    if (path !== "status") url.searchParams.set("presentation", this.presentation);
    const timeout = AbortSignal.timeout(15_000);
    const response = await fetch(url, {
      method: body === undefined ? "GET" : "POST",
      headers: body === undefined ? undefined : { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: signal ? AbortSignal.any([signal, timeout]) : timeout,
    });
    if (!response.ok) throw new Error(`${path}: HTTP ${response.status}: ${await response.text()}`);
    return response.json();
  }

  status(signal) { return this.replay("status", undefined, signal); }
  async captureScene(signal) {
    this.lastScene = await this.replay("scene", undefined, signal);
    return this.lastScene;
  }
  measureRegion(frameId, bboxPx, signal) {
    return this.replay("measure_region", { frame_id: frameId, bbox_px: bboxPx }, signal);
  }
}

async function runCase(caseId, presentation) {
  const caseDir = join(modelRoot, presentation, encodeURIComponent(caseId));
  await mkdir(caseDir, { recursive: true });
  const client = new ReplayClient(caseId, presentation);
  const agent = createReadonlyAgent(client);
  if (agent.state.model.id !== expectedIds[modelLabel]) {
    throw new Error(`Expected ${expectedIds[modelLabel]}, configured ${agent.state.model.id}`);
  }
  const observed = new Set();
  let roiRequests = 0;
  agent.beforeToolCall = async ({ toolCall }) => {
    if (observed.has(toolCall.name)) {
      return { block: true, reason: `${toolCall.name} already succeeded; use the fixed-frame result.` };
    }
    if (toolCall.name === "measure_region") {
      if (roiRequests === MAX_ROIS) {
        return { block: true, reason: "The four ROI requests are used. Explain the existing evidence and uncertainty." };
      }
      roiRequests++;
    }
  };
  agent.prepareNextTurnWithContext = ({ context }) => ({ context: {
    ...context,
    tools: context.tools?.filter(tool => !observed.has(tool.name)
      && !(tool.name === "measure_region" && roiRequests === MAX_ROIS)),
  } });

  const startedAt = new Date().toISOString();
  const started = performance.now();
  const elapsed = () => Math.round((performance.now() - started) * 1000) / 1000;
  const logPath = join(caseDir, "events.jsonl");
  const record = async (type, data) => appendFile(logPath,
    JSON.stringify({ elapsed_ms: elapsed(), type, ...data }) + "\n");
  const calls = new Map();
  const messages = [];
  const errors = [];
  let turnStarted = started;
  let firstTextMs = null;
  let timedOut = false;
  await writeFile(join(caseDir, "input.json"), JSON.stringify({
    case_id: caseId, presentation, started_at: startedAt,
    model: agent.state.model.id, provider: agent.state.model.provider,
    base_url: agent.state.model.baseUrl, replay_url: client.baseUrl,
    thinking_level: agent.state.thinkingLevel, tool_choice: "auto",
    max_tokens_per_request: 4096, max_roi_requests: MAX_ROIS, timeout_ms: TIMEOUT_MS,
    prompt, system_prompt: agent.state.systemPrompt,
    tools: agent.state.tools.map(({ name, description, parameters }) => ({ name, description, parameters })),
  }, null, 2));

  agent.subscribe(async event => {
    if (event.type === "turn_start") turnStarted = performance.now();
    if (event.type === "message_update" && event.assistantMessageEvent.type === "text_delta"
      && firstTextMs === null) firstTextMs = elapsed();
    if (event.type === "message_end" && event.message.role === "assistant") {
      const message = event.message;
      // Deliberately omit hidden thinking and message snapshots containing image data.
      const visible = {
        text: message.content.filter(item => item.type === "text").map(item => item.text).join(""),
        tool_calls: message.content.filter(item => item.type === "toolCall")
          .map(({ id, name, arguments: args }) => ({ id, name, arguments: args })),
        latency_ms: performance.now() - turnStarted,
        usage: Object.fromEntries(Object.entries(message.usage).filter(([key]) => key !== "cost")),
        stop_reason: message.stopReason, error: message.errorMessage,
      };
      messages.push(visible);
      if (visible.error) errors.push({ source: "model", message: visible.error });
      await record("assistant", visible);
    }
    if (event.type === "tool_execution_start") {
      const call = { index: calls.size + 1, tool: event.toolName, args: event.args, started_ms: elapsed() };
      calls.set(event.toolCallId, call);
      await record("tool_start", { tool_call_id: event.toolCallId, ...call });
    }
    if (event.type === "tool_execution_end") {
      const call = calls.get(event.toolCallId);
      const images = [];
      let label = "";
      for (const item of event.result.content) {
        if (item.type === "text") label = item.text;
        if (item.type === "image") {
          const extension = item.mimeType === "image/png" ? "png" : "jpg";
          const filename = `${String(call.index).padStart(2, "0")}_${event.toolName}_${images.length + 1}.${extension}`;
          await writeFile(join(caseDir, filename), Buffer.from(item.data, "base64"));
          images.push({ label, path: filename, mime_type: item.mimeType });
        }
      }
      Object.assign(call, {
        latency_ms: elapsed() - call.started_ms, is_error: event.isError,
        details: event.result.details,
        text: event.result.content.filter(item => item.type === "text").map(item => item.text),
        images,
      });
      if (!event.isError && ["status", "scene"].includes(event.toolName)) observed.add(event.toolName);
      if (event.isError) errors.push({ source: event.toolName, message: call.text.join("\n") });
      await record("tool_end", { tool_call_id: event.toolCallId, ...call });
    }
  });
  const timer = setTimeout(() => { timedOut = true; agent.abort(); }, TIMEOUT_MS);
  let runError;
  try {
    await agent.prompt(prompt);
    runError = agent.state.errorMessage;
  } catch (error) {
    runError = error instanceof Error ? error.message : String(error);
    errors.push({ source: "runtime", message: runError });
  } finally {
    clearTimeout(timer);
  }
  const finalMessage = messages.at(-1);
  const finalText = finalMessage?.text ?? "";
  const usage = { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, totalTokens: 0 };
  for (const message of messages) {
    for (const key of Object.keys(usage)) usage[key] += message.usage[key];
  }
  const result = {
    case_id: caseId, presentation, model: agent.state.model.id,
    frame_id: client.lastScene?.frame_id,
    outcome: timedOut ? "TIMEOUT" : runError ? "ERROR"
      : finalMessage?.stop_reason === "stop" && finalText ? "FINISHED" : "INCOMPLETE",
    wall_ms: elapsed(), first_text_ms: firstTextMs, request_count: messages.length,
    roi_requests: roiRequests, observed_tools: [...observed], usage, errors,
    final_text: finalText, assistant_messages: messages, tool_calls: [...calls.values()],
  };
  await writeFile(join(caseDir, "result.json"), JSON.stringify(result, null, 2));
  await writeFile(join(caseDir, "answer.txt"), finalText + "\n");
  console.log(`\n[case ${modelLabel}/${presentation}/${caseId}] ${result.outcome}; ${result.wall_ms} ms; ROI ${roiRequests}/${MAX_ROIS}`);
  return result;
}

const results = [];
for (const caseId of caseIds) {
  for (const presentation of ["baseline", "enhanced"]) {
    results.push(await runCase(caseId, presentation));
  }
}
await writeFile(join(modelRoot, "summary.json"), JSON.stringify(results.map(result => ({
  case_id: result.case_id, presentation: result.presentation, frame_id: result.frame_id,
  outcome: result.outcome, wall_ms: result.wall_ms, first_text_ms: result.first_text_ms,
  request_count: result.request_count, roi_requests: result.roi_requests,
  usage: result.usage, errors: result.errors,
})), null, 2));
