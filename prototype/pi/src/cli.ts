import { existsSync } from "node:fs";
import { writeFile } from "node:fs/promises";
import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { join, parse } from "node:path";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";
import { configuredModel } from "./model.js";
import { SimClient } from "./sim-client.js";
import { TaskMonitor, defaultMonitorOptions, type MonitorOptions } from "./task-monitor.js";

const envFile = fileURLToPath(new URL("../.env", import.meta.url));
if (existsSync(envFile)) process.loadEnvFile(envFile);

const HELP = `A2 navigation task monitor (Pi 0.85.1)

./run.sh run "中文目标" [options]  Run one goal through the task monitor
./run.sh chat [options]           Interactive goals, questions, and human controls
./run.sh monitor [options]        Resident HTTP monitor on 127.0.0.1:8770
./run.sh model                    Print selected model and credential presence
./run.sh status                   GET /status; no LLM key needed
./run.sh scene [output.jpg]       Save the first RGB view from GET /scene
./run.sh navigate "instruction"   Direct operator navigation request
./run.sh pause                    Direct operator pause
./run.sh resume                   Explicit human resume; never an LLM tool

Run/chat/monitor options:
  --db path.sqlite --log path.jsonl
  --timeout-s 360 --decision-timeout-s 30 --max-recoveries 3
Monitor option: --port 8770
An explicit --log also sets the matching .sqlite path unless --db is supplied.
The resident monitor defaults to .local/monitor/task.sqlite.
Chat commands: /run 新目标 /status /scene [output.jpg] /pause /resume /abort /quit
First plain text starts a goal; during an unfinished task it asks a question.
HTTP: GET /state; POST /task {goal}, /question {text}, /pause, /resume.
The monitor owns task decisions; HUMAN_PAUSED requires an explicit human resume.
SIM_URL defaults to http://127.0.0.1:8766; see .env.example for models.`;

function print(value: unknown): void { console.log(JSON.stringify(value, null, 2)); }
function errorText(error: unknown): string { return error instanceof Error ? error.message : String(error); }

async function operator(client: SimClient, command: string, args: string[]): Promise<void> {
  switch (command) {
    case "status": print(await client.status()); break;
    case "navigate": {
      const instruction = args.join(" ").trim();
      if (!instruction) throw new Error("navigate requires an instruction");
      print(await client.navigate(instruction));
      break;
    }
    case "pause": print(await client.pause("human")); break;
    case "resume": print(await client.resume()); break;
    case "scene": {
      const path = args[0] ?? "scene.jpg";
      const frame = await client.scene();
      await writeFile(path, Buffer.from(frame.data, "base64"));
      console.log(`Saved ${path}`);
      break;
    }
    default: throw new Error(`Unknown command '${command}'; run ./run.sh --help`);
  }
}

function monitorArgs(command: string, args: string[]): { options: MonitorOptions; goal: string; port: number } {
  const options = defaultMonitorOptions();
  if (command === "monitor") {
    options.dbPath = fileURLToPath(new URL("../../../.local/monitor/task.sqlite", import.meta.url));
  }
  const goal: string[] = [];
  let port = 8770;
  let explicitLog = false;
  let explicitDb = false;
  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg === "--log" || arg === "--db") {
      const path = args[++i];
      if (!path) throw new Error(`${arg} requires a path`);
      if (arg === "--log") { options.logPath = path; explicitLog = true; }
      else { options.dbPath = path; explicitDb = true; }
    } else if (["--max-recoveries", "--timeout-s", "--decision-timeout-s"].includes(arg)) {
      const value = Number(args[++i]);
      const minimum = arg === "--max-recoveries" ? 0 : 1;
      if (!Number.isInteger(value) || value < minimum) {
        throw new Error(`${arg} requires ${minimum === 0 ? "a nonnegative" : "a positive"} integer`);
      }
      if (arg === "--max-recoveries") options.maxRecoveries = value;
      else if (arg === "--timeout-s") options.timeoutS = value;
      else options.decisionTimeoutS = value;
    } else if (arg === "--port" && command === "monitor") {
      port = Number(args[++i]);
      if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error("--port requires an integer from 1 to 65535");
    } else if (arg.startsWith("--")) {
      throw new Error(`Unknown option '${arg}'`);
    } else goal.push(arg);
  }
  if (explicitLog && !explicitDb) {
    const path = parse(options.logPath);
    options.dbPath = join(path.dir, `${path.name}.sqlite`);
  }
  return { options, goal: goal.join(" ").trim(), port };
}

async function withMonitor(
  client: SimClient,
  options: MonitorOptions,
  body: (monitor: TaskMonitor, stopping: Promise<void>) => Promise<void>,
  cancelOnSignal = false,
): Promise<void> {
  const monitor = new TaskMonitor(client, options);
  let requestStop!: () => void;
  const stopping = new Promise<void>((resolve) => { requestStop = resolve; });
  let signalReceived = false;
  const onSignal = () => {
    if (signalReceived) return;
    signalReceived = true;
    if (cancelOnSignal) {
      void monitor.stopCurrent().catch((error) => {
        console.error(errorText(error)); process.exitCode = 1;
      }).finally(requestStop);
    } else requestStop();
  };
  monitor.on("message", (message: string) => console.log(message));
  let operation: Promise<void> | undefined;
  try {
    await monitor.connect();
    process.on("SIGINT", onSignal);
    process.on("SIGTERM", onSignal);
    operation = body(monitor, stopping);
    await Promise.race([operation, stopping]);
  } finally {
    requestStop();
    try {
      await monitor.close();
      await operation;
    } finally {
      process.off("SIGINT", onSignal);
      process.off("SIGTERM", onSignal);
    }
  }
}

async function run(client: SimClient, goal: string, options: MonitorOptions): Promise<void> {
  if (!goal) throw new Error("run requires a goal; run ./run.sh run --help");
  await withMonitor(client, options, async (monitor) => {
    const outcome = await monitor.run(goal);
    print({ ...outcome, log: options.logPath, db: options.dbPath });
    process.exitCode = outcome.status === "COMPLETED" ? 0 : outcome.status === "ERROR" ? 1 : 2;
  }, true);
}

async function chat(client: SimClient, options: MonitorOptions): Promise<void> {
  await withMonitor(client, options, async (monitor, stopping) => {
    const input = createInterface({ input: process.stdin, output: process.stdout, prompt: "you> " });
    let closed = false;
    const inputClosed = new Promise<void>((resolve) => input.on("close", () => { closed = true; resolve(); }));
    console.log("输入目标或提问。/run 替换目标，/pause、/resume 人工操作，/abort 取消任务，/quit 退出。\n");
    input.prompt();
    input.on("line", async (line) => {
      const text = line.trim();
      if (!text) { input.prompt(); return; }
      if (text === "/quit") { input.close(); return; }
      try {
        if (text.startsWith("/")) {
          const [command, ...args] = text.slice(1).split(/\s+/);
          switch (command) {
            case "run": {
              const goal = args.join(" ").trim();
              if (!goal) throw new Error("/run requires a goal");
              await monitor.stopCurrent();
              print({ taskId: await monitor.startGoal(goal) });
              break;
            }
            case "pause": print(await monitor.humanPause()); break;
            case "resume": print(await monitor.humanResume()); break;
            case "abort": await monitor.stopCurrent(); print(monitor.snapshot()); break;
            case "status": print(monitor.snapshot()); break;
            case "scene": await operator(client, "scene", args); break;
            default: throw new Error(`Unknown chat command '/${command}'`);
          }
        } else {
          const { task } = monitor.snapshot() as { task?: { status: string } | null };
          if (!task || ["COMPLETED", "CANCELLED"].includes(task.status)) {
            print({ taskId: await monitor.startGoal(text) });
          } else await monitor.question(text);
        }
      } catch (error) { console.error(errorText(error)); }
      if (!closed) input.prompt();
    });
    try { await Promise.race([inputClosed, stopping]); }
    finally { input.close(); }
  });
}

function respond(response: ServerResponse, status: number, value: unknown): void {
  response.writeHead(status, { "content-type": "application/json; charset=utf-8" });
  response.end(JSON.stringify(value));
}

async function requestBody(request: IncomingMessage): Promise<Record<string, unknown>> {
  const chunks: Buffer[] = [];
  for await (const chunk of request) chunks.push(Buffer.from(chunk));
  return JSON.parse(Buffer.concat(chunks).toString("utf8"));
}

async function resident(client: SimClient, options: MonitorOptions, port: number): Promise<void> {
  await withMonitor(client, options, async (monitor, stopping) => {
    const server = createServer(async (request, response) => {
      const path = new URL(request.url!, "http://127.0.0.1").pathname;
      try {
        if (request.method === "GET" && path === "/state") respond(response, 200, monitor.snapshot());
        else if (request.method === "POST" && path === "/task") {
          const { goal } = await requestBody(request);
          if (typeof goal !== "string" || !goal.trim()) { respond(response, 400, { error: "goal must be a nonempty string" }); return; }
          respond(response, 202, { taskId: await monitor.startGoal(goal.trim()) });
        } else if (request.method === "POST" && path === "/question") {
          const { text } = await requestBody(request);
          if (typeof text !== "string" || !text.trim()) { respond(response, 400, { error: "text must be a nonempty string" }); return; }
          respond(response, 200, { answer: await monitor.question(text.trim()) });
        } else if (request.method === "POST" && path === "/pause") respond(response, 200, await monitor.humanPause());
        else if (request.method === "POST" && path === "/resume") respond(response, 200, await monitor.humanResume());
        else respond(response, 404, { error: "Unknown route" });
      } catch (error) { respond(response, error instanceof SyntaxError ? 400 : 500, { error: errorText(error) }); }
    });
    await new Promise<void>((resolve, reject) => {
      server.once("error", reject);
      server.listen(port, "127.0.0.1", () => { server.off("error", reject); resolve(); });
    });
    console.log(`Task monitor: http://127.0.0.1:${port}; db=${options.dbPath}; log=${options.logPath}`);
    try { await stopping; }
    finally { await new Promise<void>((resolve, reject) => server.close((error) => error ? reject(error) : resolve())); }
  });
}

async function main(): Promise<void> {
  const [command = "--help", ...args] = process.argv.slice(2);
  if (["--help", "-h", "help"].includes(command)) { console.log(HELP); return; }
  if (command === "model") {
    const { model, apiKey, thinkingLevel } = configuredModel();
    print({ provider: model.provider, model: model.id, api: model.api, baseUrl: model.baseUrl,
      input: model.input, thinkingLevel, credentialPresent: Boolean(apiKey) });
    return;
  }
  const client = new SimClient();
  if (["run", "chat", "monitor"].includes(command)) {
    if (args.includes("--help")) { console.log(HELP); return; }
    const { options, goal, port } = monitorArgs(command, args);
    if (command === "run") await run(client, goal, options);
    else {
      if (goal) throw new Error(`${command} takes options only`);
      if (command === "chat") await chat(client, options);
      else await resident(client, options, port);
    }
    return;
  }
  await operator(client, command, args);
}

main().catch((error) => {
  console.error(errorText(error));
  process.exitCode = 1;
});
