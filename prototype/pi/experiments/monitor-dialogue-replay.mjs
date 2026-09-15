// One real Qwen call on an archived resident-monitor dialogue; no simulator writes.
import { readFileSync, mkdirSync, writeFileSync, appendFileSync } from "node:fs";
import { resolve, join } from "node:path";
import { fileURLToPath } from "node:url";
import { parseArgs } from "node:util";
import { PiDecision } from "../dist/decision.js";
import { controlEventFact } from "../dist/monitor-evidence.js";
import { answer, changeGoal } from "../dist/monitor-tools.js";

const { values } = parseArgs({ options: { episode: { type: "string" }, output: { type: "string" } } });
if (!values.episode || !values.output) throw new Error("Use --episode <recorded episode> --output <new directory>");
process.loadEnvFile(fileURLToPath(new URL("../.env", import.meta.url)));
const episode = resolve(values.episode), output = resolve(values.output);
mkdirSync(output);
const journal = readFileSync(join(episode, "agent.jsonl"), "utf8").trim().split("\n").map(JSON.parse);
const record = journal.find(e => e.kind === "decision_context" && e.data.purpose === "user_dialogue");
if (!record) throw new Error("No recorded user dialogue in this episode");
const request = journal.find(e => e.seq > record.seq && e.kind === "decision_request");
if (request.data.purpose !== "user_dialogue") throw new Error("Dialogue request metadata is missing");
const context = structuredClone(record.data.context);
context.controlEventFact = controlEventFact(context.control);
if (!context.controlEventFact) throw new Error("Expected the observed force-brake dialogue failure");
const snapshot = JSON.parse(readFileSync(join(episode, "monitor_state.json"), "utf8"));
let currentImage = 0;
const imagePaths = request.data.images.map(meta => meta.kind === "stage_entry"
  ? snapshot.task.stages[context.currentStage].entryImage : record.data.imagePaths[currentImage++]);
const images = request.data.images.map((meta, i) => ({ kind: meta.kind, label: meta.label,
  image: { type: "image", mimeType: meta.mimeType, data: readFileSync(imagePaths[i]).toString("base64") } }));
const save = (name, data) => writeFileSync(join(output, name), JSON.stringify(data, null, 2) + "\n");
save("input.json", { scope: "Fixed recorded paused observation, same images and question, added production controlEventFact; no live motion or fresh capture.",
  sourceEpisode: episode, sourceSeq: record.seq, context, imagePaths });
const decision = new PiDecision((kind, data) => appendFileSync(join(output, "events.jsonl"), JSON.stringify({ kind, data }) + "\n"));
const result = await decision.decide({ purpose: "user_dialogue", context, images,
  tools: [answer, changeGoal], signal: AbortSignal.timeout(30_000) });
save("result.json", result);
console.log(JSON.stringify(result, null, 2));
