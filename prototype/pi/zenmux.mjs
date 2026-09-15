import dns from "node:dns";
import { syncBuiltinESMExports } from "node:module";
import { fileURLToPath } from "node:url";

const profiles = {
  gemini: { id: "google/gemini-3.8-flash", context: 1048576 },
  deepseek: { id: "deepseek/deepseek-v4.1-flash", context: 1000000 },
};
const [choice, ...args] = process.argv.slice(2);
const profile = profiles[choice];
if (!profile) throw new Error("Usage: zenmux.sh gemini|deepseek <Pi command and arguments>");
if (!process.env.ZENMUX_API_KEY) {
  process.loadEnvFile(fileURLToPath(new URL("../../.local/secrets/zenmux.env", import.meta.url)));
}
Object.assign(process.env, {
  PI_PROVIDER: "openai-compatible",
  PI_MODEL: profile.id,
  PI_BASE_URL: "https://zenmux.ai/api/v1",
  PI_CONTEXT_WINDOW: String(profile.context),
  PI_MODEL_MAX_TOKENS: "4096",
  PI_MAX_TOKENS: "4096",
  PI_VISION: "1",
});

// The server's system resolver returned unreachable addresses for ZenMux.
// Resolve only this hostname via HTTPS DNS; TLS still verifies zenmux.ai.
// No proxy, hosts-file change, cached IP, or change to local simulator DNS.
const response = await fetch("https://1.1.1.1/dns-query?name=zenmux.ai&type=A", {
  headers: { Accept: "application/dns-json" }, signal: AbortSignal.timeout(15000),
});
if (!response.ok) throw new Error(`ZenMux DNS query: HTTP ${response.status}`);
const result = await response.json();
const addresses = result.Answer.filter((item) => item.type === 1)
  .map((item) => ({ address: item.data, family: 4 }));
if (!addresses.length) throw new Error("ZenMux DNS query returned no IPv4 address");
const systemLookup = dns.lookup;
dns.lookup = function (hostname, options, callback) {
  if (hostname !== "zenmux.ai") return systemLookup.apply(this, arguments);
  const done = typeof options === "function" ? options : callback;
  queueMicrotask(() => {
    if (options?.all) done(null, addresses);
    else done(null, addresses[0].address, 4);
  });
};
syncBuiltinESMExports();
console.error(`ZenMux ${profile.id}; direct HTTPS; reasoning_effort=low`);

if (args[0] === "--script") {
  const script = args[1];
  process.argv = [process.argv[0], script, ...args.slice(2)];
  const { pathToFileURL } = await import("node:url");
  await import(pathToFileURL(script).href);
} else {
  const cli = new URL("./dist/cli.js", import.meta.url);
  process.argv = [process.argv[0], fileURLToPath(cli), ...args];
  await import(cli.href);
}
