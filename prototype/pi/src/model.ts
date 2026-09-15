import { createModels, createProvider, type Model } from "@earendil-works/pi-ai";
import { openAICompletionsApi } from "@earendil-works/pi-ai/api/openai-completions.lazy";
import { googleProvider } from "@earendil-works/pi-ai/providers/google";

function required(name: string): string {
  const value = process.env[name];
  if (!value) throw new Error(`Missing ${name}; see prototype/pi/.env.example`);
  return value;
}

function positiveInteger(name: string): number {
  const value = Number(required(name));
  if (!Number.isInteger(value) || value <= 0) throw new Error(`${name} must be a positive integer`);
  return value;
}

export function configuredModel() {
  const models = createModels();
  const provider = process.env.PI_PROVIDER ?? "openai-compatible";
  if (provider === "google") {
    models.setProvider(googleProvider());
    const id = process.env.PI_MODEL ?? "gemini-3.8-flash";
    const model = models.getModel(provider, id);
    if (!model) throw new Error(`Pi 0.85.1 has no Google model '${id}'. Set an exact catalog ID.`);
    return {
      models,
      model,
      apiKey: process.env.PI_API_KEY || process.env.GEMINI_API_KEY || process.env.GOOGLE_API_KEY,
      keyHint: "GEMINI_API_KEY (or PI_API_KEY / GOOGLE_API_KEY)",
      thinkingLevel: "low" as const,
    };
  }
  if (provider === "openai-compatible") {
    const id = required("PI_MODEL");
    const qwen = id.toLowerCase().includes("qwen");
    const baseUrl = required("PI_BASE_URL");
    const zenmux = new URL(baseUrl).hostname === "zenmux.ai";
    const model: Model<"openai-completions"> = {
      id,
      name: id,
      provider,
      api: "openai-completions",
      baseUrl,
      reasoning: qwen || zenmux,
      input: process.env.PI_VISION === "1" ? ["text", "image"] : ["text"],
      contextWindow: positiveInteger("PI_CONTEXT_WINDOW"),
      maxTokens: positiveInteger("PI_MODEL_MAX_TOKENS"),
      // Custom endpoint pricing is not known; the CLI does not report billing estimates.
      cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0 },
      compat: {
        supportsDeveloperRole: false,
        supportsReasoningEffort: zenmux,
        ...(zenmux ? { maxTokensField: "max_completion_tokens" as const } : {}),
        ...(qwen ? { thinkingFormat: "chat-template" as const, chatTemplateKwargs: { enable_thinking: false } } : {}),
      },
    };
    // The lab endpoint does not require auth; the OpenAI SDK requires a nonempty key.
    const apiKey = zenmux ? required("ZENMUX_API_KEY") : process.env.PI_API_KEY || "no-auth";
    models.setProvider(createProvider({
      id: provider,
      models: [model],
      api: openAICompletionsApi(),
      auth: { apiKey: {
        name: zenmux ? "ZENMUX_API_KEY" : "PI_API_KEY",
        resolve: async () => ({ auth: { apiKey } }),
      } },
    }));
    return { models, model, apiKey, keyHint: zenmux ? "ZENMUX_API_KEY" : "PI_API_KEY",
      thinkingLevel: zenmux ? "low" as const : "off" as const };
  }
  throw new Error(`Unsupported PI_PROVIDER '${provider}'; use google or openai-compatible`);
}
