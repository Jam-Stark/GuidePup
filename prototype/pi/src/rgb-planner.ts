/** Pi tool-call adapter for the same robot skills used by the Codex planner.
 * This process proposes one action. Only the Python robot runtime executes it.
 */
import { existsSync, readFileSync, appendFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import type { Tool } from "@earendil-works/pi-ai";
import { PiDecision } from "./decision.js";
import type { ObservationImage } from "./sim-client.js";

const inputPath = process.argv[2];
if (!inputPath) throw new Error("rgb-planner requires an archived request path");
const request = JSON.parse(readFileSync(inputPath, "utf8"));
const envFile = fileURLToPath(new URL("../.env", import.meta.url));
if (existsSync(envFile)) process.loadEnvFile(envFile);
const contract = JSON.parse(readFileSync(new URL("../../navigation/planner-contract.json", import.meta.url), "utf8"));
const system = readFileSync(new URL("../../navigation/planner-system.txt", import.meta.url), "utf8")
  + "\n" + readFileSync(new URL("../prompts/lightnav-instruction-rules.txt", import.meta.url), "utf8")
  + "\nSubmit exactly one available robot action as a tool call. Tool names are actions; omit the action field in arguments.";
const { action: _action, ...properties } = contract.schema.properties;
const tools: Tool[] = Object.entries(contract.actions).map(([name, description]) => ({
  name, description: String(description), parameters: {
    type: "object", properties,
    required: contract.schema.required.filter((key: string) => key !== "action"), additionalProperties: false,
  } as Tool["parameters"],
}));
const images: ObservationImage[] = request.images.map(({ label, path }: { label: string; path: string }) => ({
  kind: "observation", label, image: {
    type: "image", mimeType: "image/jpeg", data: readFileSync(path).toString("base64"),
  },
}));
const planner = new PiDecision((kind, data) => {
  appendFileSync(request.events_path, JSON.stringify({ kind, data }) + "\n");
}, system);
const result = await planner.decide({
  purpose: "Choose one task/navigation/observation action for the ORIGINAL RGB-only robot task",
  context: request.context, images, tools,
  signal: AbortSignal.timeout(request.timeout_s * 1000),
});
const response = { action: result.name, ...(result.args as object) };
writeFileSync(request.response_path, JSON.stringify(response, null, 2) + "\n");
writeFileSync(request.receipt_path, JSON.stringify({ ...result, args: response }, null, 2) + "\n");
